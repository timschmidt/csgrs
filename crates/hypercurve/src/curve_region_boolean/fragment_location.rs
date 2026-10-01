//! Fragment locations and off-boundary classification.

use super::*;

impl<'a> CurveRegionBooleanContext<'a> {
    /// Classifies an already split open-curve piece against the other operand.
    /// The local range belongs to its original prepared source span.
    pub(crate) fn trim_piece_location(
        &self,
        carrier_index: usize,
        curve: &Curve2,
        range: &CurveParameterRange2,
    ) -> ExactCurveResult<RegionPointLocation> {
        if let Some(fragment) = curve.retained_fragment() {
            return self.fragment_location(carrier_index, fragment);
        }
        if let Some(spans) =
            curve.restricted_source_spans(&self.data.policy, CurveOperation2::Subdivision)?
        {
            let [span] = spans else {
                return Err(self.invalid(
                    carrier_index,
                    CurveError::Topology(
                        "a prepared curve trim piece must occupy one source span".into(),
                    ),
                ));
            };
            return self.fragment_location(carrier_index, &span.fragment);
        }
        let spans = curve.native_bezier_fragments_for_operation(
            &self.data.policy,
            CurveOperation2::Subdivision,
        )?;
        let [span] = spans else {
            return Err(self.invalid(
                carrier_index,
                CurveError::Topology(
                    "a prepared curve trim piece must occupy one source span".into(),
                ),
            ));
        };
        let Some((start, end)) = range.as_bezier_parameters() else {
            return Err(self.blocked(carrier_index, UncertaintyReason::Unsupported));
        };
        self.fragment_location(
            carrier_index,
            &BezierSplitFragment2::Materialized {
                start: start.clone(),
                end: end.clone(),
                curve: span.native_curve().clone(),
            },
        )
    }

    pub(super) fn fragment_location(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
    ) -> ExactCurveResult<RegionPointLocation> {
        let carrier = &self.data.carriers[carrier_index];
        let (other, other_operand) = match carrier.operand {
            CurveRegionBooleanOperand2::First => {
                (&self.data.second, CurveRegionBooleanOperand2::Second)
            }
            CurveRegionBooleanOperand2::Second => {
                (&self.data.first, CurveRegionBooleanOperand2::First)
            }
        };
        if self.carrier_bounds_are_outside_other_region(carrier_index) {
            return Ok(RegionPointLocation::Outside);
        }
        let classification = if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
            {
                // Complete pair replay guarantees that an open split fragment
                // cannot change faces. Its interior support point is the
                // authoritative face witness; endpoints can coincide with
                // contacts or retained overlaps and are only a fallback when
                // that interior representation is unavailable.
                let classify_interior = || {
                    let representative = chord
                        .representative_point(&self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    let classification = match representative {
                        Classification::Decided(point) => self
                            .classify_point_evidence_off_boundary(
                                carrier_index,
                                point,
                                other,
                                other_operand,
                            )?,
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    };
                    Ok(classification)
                };
                let interior_classification = classify_interior()?;
                if let Classification::Decided(
                    location @ (RegionPointLocation::Inside | RegionPointLocation::Outside),
                ) = interior_classification
                {
                    return Ok(location);
                }
                let endpoint_classification =
                    self.classify_chord_endpoint_off_other_boundary(carrier_index, chord, other)?;
                if let Some(
                    classification @ Classification::Decided(
                        RegionPointLocation::Inside | RegionPointLocation::Outside,
                    ),
                ) = endpoint_classification
                {
                    classification
                } else {
                    let endpoint_reason = match endpoint_classification {
                        Some(Classification::Uncertain(reason)) => Some(reason),
                        Some(Classification::Decided(RegionPointLocation::Boundary))
                        | Some(Classification::Decided(
                            RegionPointLocation::Inside | RegionPointLocation::Outside,
                        ))
                        | None => None,
                    };
                    match interior_classification {
                        Classification::Decided(
                            location @ (RegionPointLocation::Inside | RegionPointLocation::Outside),
                        ) => Classification::Decided(location),
                        Classification::Decided(RegionPointLocation::Boundary) => {
                            Classification::Uncertain(UncertaintyReason::Boundary)
                        }
                        Classification::Uncertain(UncertaintyReason::Unsupported) => {
                            Classification::Uncertain(
                                endpoint_reason.unwrap_or(UncertaintyReason::Unsupported),
                            )
                        }
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                }
            }
        } else if let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) = fragment {
            let parameter = match fragment
                .representative_parameter()
                .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Err(self.blocked(carrier_index, reason));
                }
            };
            let point = match fragment
                .semicircle()
                .point_evidence_at(&parameter, &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Err(self.blocked(carrier_index, reason));
                }
            };
            self.classify_point_evidence_off_boundary(carrier_index, point, other, other_operand)?
        } else if let BezierSplitFragment2::SelectedFiber(fragment) = fragment {
            let representative = match fragment
                .representative_point(&self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Err(self.blocked(carrier_index, reason));
                }
            };
            other
                .classify_point_raw(&representative, &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?
        } else {
            let (parameter, representative) =
                self.fragment_representative(carrier_index, fragment)?;
            let classification = other
                .classify_point_raw(&representative, &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            if matches!(
                classification,
                Classification::Decided(RegionPointLocation::Inside | RegionPointLocation::Outside)
            ) {
                classification
            } else {
                // A symmetric interior witness can land on a tangent or
                // shared-boundary event even though the open arrangement
                // fragment lies in one face. Complete pair replay guarantees
                // that its face cannot change between split events, so probe
                // exact scalar witnesses on both sides before propagating a
                // boundary ambiguity.
                let Some((start, end)) = fragment_range(fragment) else {
                    return Err(self.blocked(carrier_index, UncertaintyReason::Unsupported));
                };
                let middle = BezierParameter2::Exact(parameter);
                let mut decided = None;
                for (left, right) in [(start, &middle), (&middle, end)] {
                    let witness = match left
                        .strict_scalar_between_ordered(right, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(witness) => witness,
                        Classification::Uncertain(_) => continue,
                    };
                    let point = match carrier
                        .geometry
                        .point_at(&witness, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(_) => continue,
                    };
                    match other
                        .classify_point_raw(&point, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(
                            location @ (RegionPointLocation::Inside | RegionPointLocation::Outside),
                        ) => match decided {
                            Some(previous) if previous != location => {
                                return Err(self.invalid(
                                    carrier_index,
                                    CurveError::Topology(
                                        "one split carrier fragment crossed two Boolean faces"
                                            .into(),
                                    ),
                                ));
                            }
                            Some(_) => {}
                            None => decided = Some(location),
                        },
                        Classification::Decided(RegionPointLocation::Boundary)
                        | Classification::Uncertain(_) => {}
                    }
                }
                decided.map_or(classification, Classification::Decided)
            }
        };
        match classification {
            Classification::Decided(location) => Ok(location),
            Classification::Uncertain(reason) => Err(self.blocked(carrier_index, reason)),
        }
    }

    pub(super) fn classify_chord_endpoint_off_other_boundary(
        &self,
        carrier_index: usize,
        chord: &crate::BezierAlgebraicChord2,
        other_region: &CurveRegion2,
    ) -> ExactCurveResult<Option<Classification<RegionPointLocation>>> {
        let mut last_reason = None;
        for endpoint in [chord.start(), chord.end()] {
            let direct = match endpoint {
                CurvePoint2(CurvePointData2::Exact(point)) => Some(
                    other_region
                        .classify_point_raw(point, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?,
                ),
                CurvePoint2(CurvePointData2::Algebraic(point)) => Some(
                    other_region
                        .classify_algebraic_point_raw(point, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?,
                ),
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    None
                }
            };
            if let Some(classification) = direct {
                match classification {
                    Classification::Decided(
                        location @ (RegionPointLocation::Inside | RegionPointLocation::Outside),
                    ) => return Ok(Some(Classification::Decided(location))),
                    Classification::Decided(RegionPointLocation::Boundary) => {
                        last_reason = Some(UncertaintyReason::Boundary);
                    }
                    Classification::Uncertain(reason) => last_reason = Some(reason),
                }
                continue;
            }
            for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
                let bounds = match endpoint {
                    CurvePoint2(CurvePointData2::Endpoint(point)) => {
                        point.bounds(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::AnalyticParallel(point)) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::Similarity(point)) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::Exact(_))
                    | CurvePoint2(CurvePointData2::Algebraic(_)) => unreachable!(),
                };
                let Classification::Decided(bounds) = bounds else {
                    continue;
                };
                let mut separated_from_boundary = true;
                for boundary in &self.data.carriers {
                    if boundary.operand == self.data.carriers[carrier_index].operand {
                        continue;
                    }
                    let Classification::Decided(boundary_bounds) =
                        boundary.bounds.get_or_init(|| {
                            boundary.geometry.certified_outer_bounds(
                                &boundary.range(),
                                0,
                                &self.data.policy,
                            )
                        })
                    else {
                        separated_from_boundary = false;
                        break;
                    };
                    if bounds.overlaps(boundary_bounds, &self.data.policy)
                        != Classification::Decided(false)
                    {
                        separated_from_boundary = false;
                        break;
                    }
                }
                if !separated_from_boundary {
                    continue;
                }
                let two = Real::from(2_i8);
                let representative = crate::Point2::new(
                    ((bounds.min().x() + bounds.max().x()) / &two)
                        .map_err(|cause| self.invalid(carrier_index, cause.into()))?,
                    ((bounds.min().y() + bounds.max().y()) / &two)
                        .map_err(|cause| self.invalid(carrier_index, cause.into()))?,
                );
                let classification = other_region
                    .classify_point_raw(&representative, &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))?;
                match classification {
                    Classification::Decided(
                        location @ (RegionPointLocation::Inside | RegionPointLocation::Outside),
                    ) => return Ok(Some(Classification::Decided(location))),
                    Classification::Decided(RegionPointLocation::Boundary) => {
                        last_reason = Some(UncertaintyReason::Boundary);
                    }
                    Classification::Uncertain(reason) => last_reason = Some(reason),
                }
                break;
            }
        }
        Ok(last_reason.map(Classification::Uncertain))
    }

    pub(super) fn classify_point_evidence_off_boundary(
        &self,
        owner_carrier_index: usize,
        point: CurvePoint2,
        boundary_region: &CurveRegion2,
        boundary_operand: CurveRegionBooleanOperand2,
    ) -> ExactCurveResult<Classification<RegionPointLocation>> {
        let direct = match &point {
            CurvePoint2(CurvePointData2::Exact(point)) => Some(
                boundary_region
                    .classify_point_raw(point, &self.data.policy)
                    .map_err(|cause| self.invalid(owner_carrier_index, cause))?,
            ),
            CurvePoint2(CurvePointData2::Algebraic(point)) => Some(
                boundary_region
                    .classify_algebraic_point_off_boundary_raw(point, &self.data.policy)
                    .map_err(|cause| self.invalid(owner_carrier_index, cause))?,
            ),
            CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
        };
        match direct {
            Some(decided @ Classification::Decided(_)) => Ok(decided),
            Some(Classification::Uncertain(_)) | None => self
                .classify_retained_point_off_boundary_by_probe(
                    owner_carrier_index,
                    point,
                    boundary_operand,
                    RetainedPointProbeClassification::FilledRegion,
                ),
        }
    }

    /// Classifies retained multi-field point evidence without materializing a
    /// rounded coordinate pair.
    ///
    /// A rational point outside a certified outer box is joined to the target
    /// by one retained algebraic chord. Complete pair replay against the
    /// selected operand then identifies the last transverse boundary crossing.
    /// The boundary's certified filled side determines which face contains the
    /// target. Four exterior corners are the fast candidates. If all are
    /// degenerate, `2n+1` distinct points on one certified exterior line
    /// exclude every direction through the `n` loop vertices and every
    /// direction that can overlap one of the `n` retained line images.
    pub(super) fn classify_retained_point_off_boundary_by_probe(
        &self,
        owner_carrier_index: usize,
        point: CurvePoint2,
        boundary_operand: CurveRegionBooleanOperand2,
        classification_kind: RetainedPointProbeClassification,
    ) -> ExactCurveResult<Classification<RegionPointLocation>> {
        // A terminal equality on one unlucky probe direction must not preempt
        // another direction that has a strict separation proof. Exhaust the
        // complete finite probe set with terminals suppressed, then replay it
        // under APPROXIMATE_512 only when every strict direction is blocked.
        if self.data.policy.permits_approximate_512() {
            match self.data.policy.strict_predicate_pass(|| {
                self.classify_retained_point_off_boundary_by_probe_once(
                    owner_carrier_index,
                    point.clone(),
                    boundary_operand,
                    classification_kind,
                )
            }) {
                Ok(decided @ Classification::Decided(_)) => return Ok(decided),
                Ok(Classification::Uncertain(_)) | Err(ExactCurveError::Blocked(_)) => {}
                Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
            }
        }
        self.classify_retained_point_off_boundary_by_probe_once(
            owner_carrier_index,
            point,
            boundary_operand,
            classification_kind,
        )
    }

    pub(super) fn classify_retained_point_off_boundary_by_probe_once(
        &self,
        owner_carrier_index: usize,
        point: CurvePoint2,
        boundary_operand: CurveRegionBooleanOperand2,
        classification_kind: RetainedPointProbeClassification,
    ) -> ExactCurveResult<Classification<RegionPointLocation>> {
        let boundary_region = match boundary_operand {
            CurveRegionBooleanOperand2::First => self.data.first,
            CurveRegionBooleanOperand2::Second => self.data.second,
        };
        // Every context stores the first operand's carriers before the second.
        // Borrow that existing partition so repeated queries also reuse its
        // retained bounds and injectivity facts.
        let boundary_carriers = match boundary_operand {
            CurveRegionBooleanOperand2::First => {
                &self.data.carriers[..self.data.first_carrier_count]
            }
            CurveRegionBooleanOperand2::Second => {
                &self.data.carriers[self.data.first_carrier_count..]
            }
        };
        if boundary_carriers.is_empty() {
            return Ok(Classification::Decided(RegionPointLocation::Outside));
        }

        let mut last_reason = UncertaintyReason::Unsupported;
        let outer_bounds = match retained_probe_outer_bounds(boundary_carriers, &self.data.policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let fallback_count = boundary_carriers.len().saturating_mul(2).saturating_add(1);
        for candidate_index in 0..fallback_count.saturating_add(4) {
            let Some(outside) = retained_probe_exterior_candidate(&outer_bounds, candidate_index)
            else {
                continue;
            };
            let probe = match crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(outside),
                point.clone(),
                &self.data.policy,
            ) {
                Ok(Classification::Decided(probe)) => probe,
                Ok(Classification::Uncertain(reason)) => {
                    last_reason = reason;
                    continue;
                }
                // This candidate lies strictly outside the certified boundary
                // enclosure. Coincidence with it already locates the target;
                // a zero-length probe needs no intersections or winding replay.
                Err(CurveError::ZeroLengthLine) => {
                    return Ok(Classification::Decided(RegionPointLocation::Outside));
                }
                Err(cause) => return Err(self.invalid(owner_carrier_index, cause)),
            };
            let probe_end = CurveParameter2::from_algebraic_chord(probe.end_parameter());
            let evidence = match self.intersect_algebraic_probe_carriers(
                probe,
                boundary_region,
                boundary_carriers,
                None,
            ) {
                Ok(evidence) => evidence,
                Err(ExactCurveError::Blocked(blocker)) => {
                    last_reason = blocker.reason();
                    continue;
                }
                Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
            };
            if !evidence.overlaps().is_empty() {
                last_reason = UncertaintyReason::Boundary;
                continue;
            }
            if let Some(blocker) = evidence.blockers().first() {
                last_reason = blocker
                    .uncertainty_reason()
                    .unwrap_or(UncertaintyReason::Unsupported);
                continue;
            }

            // Any contact at the probe endpoint proves that the target itself
            // lies on the boundary, including a nontransverse endpoint touch.
            let mut endpoint_is_boundary = false;
            let mut comparison_blocked = false;
            for contact in evidence.contacts() {
                match contact
                    .first_parameter()
                    .cmp_by_refinement(&probe_end, &self.data.policy)
                    .map_err(|cause| self.invalid(owner_carrier_index, cause))?
                {
                    Classification::Decided(Ordering::Equal) => {
                        endpoint_is_boundary = true;
                        break;
                    }
                    Classification::Decided(Ordering::Less) => {}
                    Classification::Decided(Ordering::Greater) => {
                        return Err(self.invalid(
                            owner_carrier_index,
                            CurveError::Topology(
                                "an exterior classification probe retained a contact past its endpoint"
                                    .into(),
                            ),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        comparison_blocked = true;
                        break;
                    }
                }
            }
            if endpoint_is_boundary {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "retained-point-probe",
                    "endpoint-boundary",
                );
                return Ok(Classification::Decided(RegionPointLocation::Boundary));
            }
            if comparison_blocked {
                continue;
            }

            let mut crossings =
                Vec::<(&CurveParameter2, bool, usize)>::with_capacity(evidence.contacts().len());
            let mut ambiguous = false;
            for contact in evidence
                .contacts()
                .iter()
                .filter(|contact| contact.is_certified_transverse())
            {
                let Some(mut cross_is_positive) = contact.evidence.tangent_cross_is_positive()
                else {
                    last_reason = UncertaintyReason::Predicate;
                    ambiguous = true;
                    break;
                };
                let Some(boundary_index) = contact.second().carrier_index().checked_sub(1) else {
                    return Err(self.invalid(
                        owner_carrier_index,
                        CurveError::Topology(
                            "an exterior classification contact resolved to its probe carrier"
                                .into(),
                        ),
                    ));
                };
                let Some(boundary) = boundary_carriers.get(boundary_index) else {
                    return Err(self.invalid(
                        owner_carrier_index,
                        CurveError::Topology(
                            "an exterior classification contact lost its boundary carrier".into(),
                        ),
                    ));
                };
                cross_is_positive ^= boundary.reversed;
                crossings.push((
                    contact.first_parameter(),
                    cross_is_positive,
                    boundary.loop_index,
                ));
            }
            if ambiguous {
                continue;
            }
            match classification_kind {
                RetainedPointProbeClassification::FilledRegion => {
                    // Direct point classification, boundary incidence and an
                    // exterior probe with no crossings need no orientation
                    // theorem. Request cached filled sides only for this
                    // retained-point face decision.
                    let filled_sides = if crossings.is_empty() {
                        &[][..]
                    } else {
                        match boundary_region
                            .filled_side_is_left_raw(&self.data.policy)
                            .map_err(|cause| self.invalid(owner_carrier_index, cause))?
                        {
                            Classification::Decided(sides) => sides,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    };
                    let mut last_contact = None::<(&CurveParameter2, RegionPointLocation)>;
                    for (parameter, cross_is_positive, loop_index) in crossings {
                        let Some(&filled_side_is_left) = filled_sides.get(loop_index) else {
                            return Err(self.invalid(
                                owner_carrier_index,
                                CurveError::Topology(
                                    "an exterior classification contact lost its loop filled side"
                                        .into(),
                                ),
                            ));
                        };
                        let target_is_left_of_boundary = !cross_is_positive;
                        let location = if target_is_left_of_boundary == filled_side_is_left {
                            RegionPointLocation::Inside
                        } else {
                            RegionPointLocation::Outside
                        };
                        let replace = match last_contact {
                            None => true,
                            Some((previous_parameter, previous_location)) => match parameter
                                .cmp_by_refinement(previous_parameter, &self.data.policy)
                                .map_err(|cause| self.invalid(owner_carrier_index, cause))?
                            {
                                Classification::Decided(Ordering::Greater) => true,
                                Classification::Decided(Ordering::Less) => false,
                                Classification::Decided(Ordering::Equal)
                                    if previous_location == location =>
                                {
                                    false
                                }
                                Classification::Decided(Ordering::Equal) => {
                                    last_reason = UncertaintyReason::Boundary;
                                    ambiguous = true;
                                    break;
                                }
                                Classification::Uncertain(reason) => {
                                    last_reason = reason;
                                    ambiguous = true;
                                    break;
                                }
                            },
                        };
                        if replace {
                            last_contact = Some((parameter, location));
                        }
                    }
                    if ambiguous {
                        continue;
                    }
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "retained-point-probe",
                        match last_contact {
                            Some((_, RegionPointLocation::Inside)) => "inside",
                            Some((_, RegionPointLocation::Outside)) | None => "outside",
                            Some((_, RegionPointLocation::Boundary)) => unreachable!(),
                        },
                    );
                    return Ok(Classification::Decided(
                        last_contact.map_or(RegionPointLocation::Outside, |(_, location)| location),
                    ));
                }
                RetainedPointProbeClassification::LoopParity => {
                    for index in 1..crossings.len() {
                        let mut cursor = index;
                        while cursor > 0 {
                            match crossings[cursor]
                                .0
                                .cmp_by_refinement(crossings[cursor - 1].0, &self.data.policy)
                                .map_err(|cause| self.invalid(owner_carrier_index, cause))?
                            {
                                Classification::Decided(Ordering::Less) => {
                                    crossings.swap(cursor, cursor - 1);
                                    cursor -= 1;
                                }
                                Classification::Decided(Ordering::Equal | Ordering::Greater) => {
                                    break;
                                }
                                Classification::Uncertain(reason) => {
                                    last_reason = reason;
                                    ambiguous = true;
                                    break;
                                }
                            }
                        }
                        if ambiguous {
                            break;
                        }
                    }
                    if ambiguous {
                        continue;
                    }

                    let mut inside = false;
                    let mut group_start = 0_usize;
                    while group_start < crossings.len() {
                        let mut group_end = group_start + 1;
                        while group_end < crossings.len() {
                            match crossings[group_end]
                                .0
                                .cmp_by_refinement(crossings[group_start].0, &self.data.policy)
                                .map_err(|cause| self.invalid(owner_carrier_index, cause))?
                            {
                                Classification::Decided(Ordering::Equal) => group_end += 1,
                                Classification::Decided(Ordering::Greater) => break,
                                Classification::Decided(Ordering::Less) => {
                                    return Err(self.invalid(
                                        owner_carrier_index,
                                        CurveError::Topology(
                                            "retained loop probe crossings lost exact order".into(),
                                        ),
                                    ));
                                }
                                Classification::Uncertain(reason) => {
                                    last_reason = reason;
                                    ambiguous = true;
                                    break;
                                }
                            }
                        }
                        if ambiguous {
                            break;
                        }
                        let group = &crossings[group_start..group_end];
                        if group.len() > 2 {
                            last_reason = UncertaintyReason::Boundary;
                            ambiguous = true;
                            break;
                        }
                        let has_positive = group.iter().any(|(_, positive, _)| *positive);
                        let has_negative = group.iter().any(|(_, positive, _)| !*positive);
                        // A single transverse image, or the same oriented
                        // tangent on both sides of a split vertex, crosses the
                        // loop once. Opposite vertex tangents are a touch and
                        // leave parity unchanged.
                        if !(has_positive && has_negative) {
                            inside = !inside;
                        }
                        group_start = group_end;
                    }
                    if ambiguous {
                        continue;
                    }
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "retained-loop-parity-probe",
                        if inside { "inside" } else { "outside" },
                    );
                    return Ok(Classification::Decided(if inside {
                        RegionPointLocation::Inside
                    } else {
                        RegionPointLocation::Outside
                    }));
                }
            }
        }

        Ok(Classification::Uncertain(last_reason))
    }

    pub(super) fn carrier_bounds_are_outside_other_region(&self, carrier_index: usize) -> bool {
        // This entire path is an optional fragment-classification shortcut.
        // If exact bounds cannot separate the operands, the authoritative
        // point/region classifier below must get the decision.
        self.data.policy.strict_predicate_pass(|| {
            let carrier = &self.data.carriers[carrier_index];
            let other_operand = match carrier.operand {
                CurveRegionBooleanOperand2::First => 1,
                CurveRegionBooleanOperand2::Second => 0,
            };
            // Each operand/refinement owns one lazy envelope. Rebuilding this
            // union for every fragment is another Cartesian scan, even after
            // the pair broad phase has discarded all distant components.
            let other_bounds = self.data.operand_bounds[other_operand].get_or_init(Box::default);
            // When both sides' envelopes are exact at level zero, refined
            // levels repeat the same boxes; this optional shortcut stops there.
            let invariant = carrier_bounds_refinement_invariant(carrier)
                && *other_bounds.refinement_invariant.get_or_init(|| {
                    self.data
                        .carriers
                        .iter()
                        .filter(|other| other.operand != carrier.operand)
                        .all(carrier_bounds_refinement_invariant)
                });
            // Set once a decided comparison has overlapped; refined levels of
            // invariant envelopes would repeat it.
            let mut decided_overlap = false;
            CARRIER_BOUND_REFINEMENTS
                .into_iter()
                .enumerate()
                .any(|(level, refinement_steps)| {
                    if invariant && decided_overlap {
                        return false;
                    }
                    let Classification::Decided(carrier_bounds) =
                        carrier_optional_outer_bounds_refined(
                            carrier,
                            refinement_steps,
                            &self.data.policy,
                        )
                    else {
                        return false;
                    };
                    let cell = &other_bounds.refinements[level];
                    let other_bounds = if let Some(bounds) = cell.get() {
                        bounds
                    } else {
                        let mut accumulated = None::<Aabb2>;
                        for other in &self.data.carriers {
                            if other.operand == carrier.operand {
                                continue;
                            }
                            let bounds = match carrier_optional_outer_bounds_refined(
                                other,
                                refinement_steps,
                                &self.data.policy,
                            ) {
                                Classification::Decided(bounds) => bounds,
                                Classification::Uncertain(_) => {
                                    // Unavailable evidence may become decidable
                                    // after another exact kernel refines it.
                                    return false;
                                }
                            };
                            accumulated = Some(match accumulated {
                                None => bounds,
                                Some(previous) => match previous.union(&bounds) {
                                    Classification::Decided(bounds) => bounds,
                                    Classification::Uncertain(_) => {
                                        return false;
                                    }
                                },
                            });
                        }
                        let _ = cell.set(accumulated);
                        cell.get().expect("the exact operand envelope was retained")
                    };
                    let disjoint = other_bounds.as_ref().is_none_or(|other_bounds| {
                        carrier_bounds.overlaps(other_bounds, &self.data.policy)
                            == Classification::Decided(false)
                    });
                    decided_overlap = !disjoint;
                    disjoint
                })
        })
    }

    pub(super) fn fragment_representative(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
    ) -> ExactCurveResult<(crate::Real, crate::Point2)> {
        let carrier = &self.data.carriers[carrier_index];
        let Some((start, end)) = fragment_range(fragment) else {
            return Err(self.blocked(carrier_index, UncertaintyReason::Unsupported));
        };
        let parameter = match start
            .strict_scalar_between_ordered(end, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };
        let representative = match carrier
            .geometry
            .point_at(&parameter, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };
        Ok((parameter, representative))
    }

    pub(super) fn fragment_action(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
        location: RegionPointLocation,
        overlaps: &[CarrierOverlap],
        operation: BooleanOp,
    ) -> ExactCurveResult<RegionFragmentAction> {
        let carrier = &self.data.carriers[carrier_index];
        match location {
            RegionPointLocation::Inside => Ok(action_for_sides(
                operation,
                carrier.operand,
                carrier.filled_side_is_left,
                true,
            )),
            RegionPointLocation::Outside => Ok(action_for_sides(
                operation,
                carrier.operand,
                carrier.filled_side_is_left,
                false,
            )),
            RegionPointLocation::Boundary => {
                self.shared_fragment_action(carrier_index, fragment, overlaps, operation)
            }
        }
    }

    pub(super) fn shared_fragment_action(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
        overlaps: &[CarrierOverlap],
        operation: BooleanOp,
    ) -> ExactCurveResult<RegionFragmentAction> {
        let fragment_range = fragment.curve_region_parameter_range();
        let (start, end) = (fragment_range.start(), fragment_range.end());
        let mut matching_overlap = None;
        for overlap in overlaps {
            let range = if overlap.first_carrier_index == carrier_index {
                Some(&overlap.first_range)
            } else if overlap.second_carrier_index == carrier_index {
                Some(&overlap.second_range)
            } else {
                None
            };
            if let Some(range) = range
                && range_contains_fragment(range, start, end, &self.data.policy)?
            {
                matching_overlap = Some(overlap);
                break;
            }
        }
        let Some(overlap) = matching_overlap else {
            return Err(self.blocked(carrier_index, UncertaintyReason::Boundary));
        };
        if carrier_index >= self.data.first_carrier_count {
            return Ok(RegionFragmentAction::Discard);
        }
        let first = &self.data.carriers[overlap.first_carrier_index];
        let second = &self.data.carriers[overlap.second_carrier_index];
        let same_source_direction = overlap.orientation == CurveOverlapOrientation2::Same;
        let same_traversal = same_source_direction == (first.reversed == second.reversed);
        if let Some(action) =
            self.shared_algebraic_chord_action(overlap, fragment, same_traversal, operation)?
        {
            return Ok(action);
        }
        let second_left_in_first_direction = if same_traversal {
            second.filled_side_is_left
        } else {
            !second.filled_side_is_left
        };
        let left = operation.apply(first.filled_side_is_left, second_left_in_first_direction);
        let right = operation.apply(!first.filled_side_is_left, !second_left_in_first_direction);
        Ok(action_from_result_sides(left, right))
    }

    /// Decides coincident retained chords from the actual global occupancy on
    /// both sides of their shared image. Loop-local `filled_side_is_left`
    /// remains valid for a simple regularized boundary, but it cannot decide
    /// a span made internal by another loop of the same operand. The boundary
    /// side-ray kernel skips each owning source fragment at the common point,
    /// so no finite epsilon or inexact displacement is introduced.
    pub(super) fn shared_algebraic_chord_action(
        &self,
        overlap: &CarrierOverlap,
        first_fragment: &BezierSplitFragment2,
        same_traversal: bool,
        operation: BooleanOp,
    ) -> ExactCurveResult<Option<RegionFragmentAction>> {
        let BezierSplitFragment2::AlgebraicChord(first_chord) = first_fragment else {
            return Ok(None);
        };
        if !matches!(
            self.data.carriers[overlap.second_carrier_index].geometry,
            CurveSupport2::Line(_)
        ) {
            return Ok(None);
        }
        let representative = match first_chord
            .representative_point(&self.data.policy)
            .map_err(|cause| self.invalid(overlap.first_carrier_index, cause))?
        {
            Classification::Decided(representative) => representative,
            Classification::Uncertain(first_reason) => {
                let CurveSupport2::Line(second_chord) =
                    &self.data.carriers[overlap.second_carrier_index].geometry
                else {
                    unreachable!("shared algebraic chord action validated its second carrier")
                };
                let (Some(second_start), Some(second_end)) = (
                    overlap.second_range.start().as_algebraic_chord(),
                    overlap.second_range.end().as_algebraic_chord(),
                ) else {
                    return Err(self.blocked(overlap.first_carrier_index, first_reason));
                };
                let second_fragment =
                    crate::BezierAlgebraicChord2::from_certified_ordered_parameter_range(
                        second_chord,
                        second_start,
                        second_end,
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(overlap.second_carrier_index, cause))?;
                match second_fragment
                    .representative_point(&self.data.policy)
                    .map_err(|cause| self.invalid(overlap.second_carrier_index, cause))?
                {
                    Classification::Decided(representative) => representative,
                    Classification::Uncertain(reason) => {
                        return Err(self.blocked(overlap.second_carrier_index, reason));
                    }
                }
            }
        };
        let location_is_inside = |carrier_index: usize,
                                  classification: (Vec<i32>, RegionPointLocation)|
         -> ExactCurveResult<bool> {
            match classification.1 {
                RegionPointLocation::Inside => Ok(true),
                RegionPointLocation::Outside => Ok(false),
                RegionPointLocation::Boundary => {
                    Err(self.blocked(carrier_index, UncertaintyReason::Boundary))
                }
            }
        };
        let certified_regularized_sides =
            |carrier_index: usize, source_follows_reference_tangent: bool| {
                let region = self.region_for_carrier(carrier_index);
                if !region.has_regularized_filled_left_topology(&self.data.policy) {
                    return None;
                }
                let source_left_is_inside = self.data.carriers[carrier_index].filled_side_is_left;
                let reference_left_is_inside = if source_follows_reference_tangent {
                    source_left_is_inside
                } else {
                    !source_left_is_inside
                };
                Some([reference_left_is_inside, !reference_left_is_inside])
            };
        let (first_sides, second_sides) = match representative {
            CurvePoint2(CurvePointData2::Exact(point)) => {
                let (tangent_x, tangent_y) = first_chord
                    .exact_line()
                    .map(|line| line.delta())
                    .or_else(|| {
                        first_chord
                            .strict_provenance_support_line(&self.data.policy)
                            .map(|line| line.delta())
                    })
                    .or_else(|| first_chord.certified_unit_tangent())
                    .ok_or_else(|| {
                        self.blocked(overlap.first_carrier_index, UncertaintyReason::Unsupported)
                    })?;
                let classify = |carrier_index,
                                source_follows_reference_tangent|
                 -> ExactCurveResult<[bool; 2]> {
                    if let Some(sides) =
                        certified_regularized_sides(carrier_index, source_follows_reference_tangent)
                    {
                        return Ok(sides);
                    }
                    let classify_side = |left| {
                        self.fragment_side_classification_with_reference_tangent(
                            carrier_index,
                            &point,
                            None,
                            &tangent_x,
                            &tangent_y,
                            left,
                            source_follows_reference_tangent,
                        )
                        .and_then(|classification| {
                            location_is_inside(carrier_index, classification)
                        })
                    };
                    Ok([classify_side(true)?, classify_side(false)?])
                };
                (
                    classify(overlap.first_carrier_index, true)?,
                    classify(overlap.second_carrier_index, same_traversal)?,
                )
            }
            CurvePoint2(CurvePointData2::Algebraic(point)) => {
                let tangent = |axis| {
                    first_chord
                        .tangent_axis_sign(axis, &self.data.policy)
                        .map_err(|cause| self.invalid(overlap.first_carrier_index, cause))
                };
                let [tangent_x, tangent_y] = [tangent(Axis2::X)?, tangent(Axis2::Y)?];
                let classify = |carrier_index,
                                source_follows_reference_tangent|
                 -> ExactCurveResult<[bool; 2]> {
                    if let Some(sides) =
                        certified_regularized_sides(carrier_index, source_follows_reference_tangent)
                    {
                        return Ok(sides);
                    }
                    let classify_side = |left| {
                        self.algebraic_fragment_side_classification(
                            carrier_index,
                            &point,
                            tangent_x,
                            tangent_y,
                            left,
                        )
                        .and_then(|classification| {
                            location_is_inside(carrier_index, classification)
                        })
                    };
                    let source_sides = [classify_side(true)?, classify_side(false)?];
                    Ok(if source_follows_reference_tangent {
                        source_sides
                    } else {
                        [source_sides[1], source_sides[0]]
                    })
                };
                (
                    classify(overlap.first_carrier_index, true)?,
                    classify(overlap.second_carrier_index, same_traversal)?,
                )
            }
            CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                return Ok(None);
            }
        };
        Ok(Some(action_from_result_sides(
            operation.apply(first_sides[0], second_sides[0]),
            operation.apply(first_sides[1], second_sides[1]),
        )))
    }
}
