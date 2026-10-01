//! Exact carrier-pair contact, overlap and blocker kernels.

use super::*;

impl<'a> CurveRegionBooleanContext<'a> {
    pub(super) fn authored_parallel_support_contact(
        &self,
        pair: &RegionCarrierPair,
        parallel: &BezierParallel2,
        parallel_index: usize,
        direction_x: &Real,
        direction_y: &Real,
        regular_range: Option<&CurveParameterRange2>,
    ) -> ExactCurveResult<Option<(Real, RealSign)>> {
        let Some((first_at_start, second_at_start)) = self
            .authored_carrier_shared_endpoints(pair.first_carrier_index, pair.second_carrier_index)
        else {
            return Ok(None);
        };
        let parallel_at_start = if parallel_index == pair.first_carrier_index {
            first_at_start
        } else {
            second_at_start
        };
        let Some(parameter) = (if parallel_at_start {
            carrier_traversal_start(&self.data.carriers[parallel_index])
        } else {
            carrier_traversal_end(&self.data.carriers[parallel_index])
        })
        .scalar()
        .cloned() else {
            return Ok(None);
        };
        let tangent_relation = match regular_range {
            Some(range) => parallel.vector_tangent_cross_and_dot_signs_on_regular_range(
                &parameter.clone().into(),
                direction_x,
                direction_y,
                range,
                &self.data.policy,
            ),
            None => parallel.vector_tangent_cross_and_dot_signs(
                &parameter.clone().into(),
                direction_x,
                direction_y,
                &self.data.policy,
            ),
        }
        .map_err(|cause| self.invalid(parallel_index, cause))?;
        let Classification::Decided((cross, dot)) = tangent_relation else {
            return Ok(None);
        };
        Ok((cross != RealSign::Zero || dot != RealSign::Zero).then_some((parameter, cross)))
    }

    pub(super) fn parallel_line_pair_result(
        &self,
        pair: &RegionCarrierPair,
        parallel: &BezierParallel2,
        parallel_index: usize,
        curve: &BezierSubcurve2,
        parallel_is_first: bool,
        regular_range: Option<&CurveParameterRange2>,
    ) -> ExactCurveResult<Classification<Option<RegionPairResult>>> {
        let retained = BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: curve.clone(),
        };
        let line = match crate::bezier_region::retained_line_fragment_segment(
            &retained,
            &self.data.policy,
        )
        .map_err(|cause| self.invalid(0, cause))?
        {
            Classification::Decided(line) => line,
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let certified_tangent_contacts = match curve {
            BezierSubcurve2::Quadratic(curve) => curve
                .retained_parallel_line_tangent_contacts()
                .iter()
                .filter(|contact| contact.parallel() == parallel)
                .collect::<Vec<_>>(),
            BezierSubcurve2::Cubic(_)
            | BezierSubcurve2::RationalQuadratic(_)
            | BezierSubcurve2::Rational(_) => Vec::new(),
        };
        // The loop topology already owns its shared adjacent endpoint. Feed
        // that authored root and its exact first-order kind to the univariate
        // support kernel, which can divide it before isolating every residual
        // contact. This avoids asking independently materialized endpoint
        // expressions to rediscover their equality while preserving complete
        // detection of any later crossing of the finite line segment.
        let (direction_x, direction_y) = line.delta();
        let authored_contact = self.authored_parallel_support_contact(
            pair,
            parallel,
            parallel_index,
            &direction_x,
            &direction_y,
            regular_range,
        )?;
        let certified_crossing = authored_contact.as_ref().and_then(|(parameter, cross)| {
            let direction = match cross {
                RealSign::Positive => BezierLineCrossingDirection::NegativeToPositive,
                RealSign::Negative => BezierLineCrossingDirection::PositiveToNegative,
                RealSign::Zero => return None,
            };
            Some((parameter, direction))
        });
        let mut certified_tangent_parameters = certified_tangent_contacts
            .iter()
            .map(|contact| contact.parameter().clone())
            .collect::<Vec<_>>();
        if let Some((parameter, RealSign::Zero)) = &authored_contact
            && !certified_tangent_parameters.contains(parameter)
        {
            certified_tangent_parameters.push(parameter.clone());
        }
        let relation = match match regular_range {
            Some(range) => parallel
                .relation_to_supporting_line_on_regular_range_with_certified_contacts(
                    &line,
                    range,
                    certified_crossing,
                    &certified_tangent_parameters,
                    false,
                    &self.data.policy,
                ),
            None => parallel.relation_to_supporting_line_with_direction_and_certified_contacts(
                &line,
                &direction_x,
                &direction_y,
                certified_crossing,
                &certified_tangent_parameters,
                false,
                &self.data.policy,
            ),
        }
        .map_err(|cause| self.invalid(0, cause))?
        {
            Classification::Decided(relation) => relation,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let contacts = match relation {
            BezierLineContactRelation::ControlHullDisjoint { .. }
            | BezierLineContactRelation::NoContact => {
                return Ok(Classification::Decided(Some(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                    blockers: Vec::new(),
                })));
            }
            BezierLineContactRelation::OnSupportingLine => {
                return Ok(Classification::Decided(None));
            }
            BezierLineContactRelation::Contacts { contacts } => contacts,
        };
        let reversed_line = LineSeg2::try_new(line.end().clone(), line.start().clone())
            .map_err(|cause| self.invalid(0, cause))?;
        let mut retained_parameters = Vec::with_capacity(contacts.len());
        let mut retained_certified_tangencies = Vec::new();
        for contact in contacts {
            if contact.parameter().scalar().is_some_and(|parameter| {
                authored_contact
                    .as_ref()
                    .is_some_and(|(authored, _)| parameter == authored)
            }) {
                continue;
            }
            if let Some(certified) = contact.parameter().scalar().and_then(|parameter| {
                certified_tangent_contacts
                    .iter()
                    .find(|certified| certified.parameter() == parameter)
            }) {
                retained_certified_tangencies.push(*certified);
                continue;
            }
            let from_start = match match regular_range {
                Some(range) => parallel.supporting_line_parameter_order_on_regular_range(
                    contact.parameter(),
                    &line,
                    range,
                    &self.data.policy,
                ),
                None => parallel.supporting_line_parameter_order(
                    contact.parameter(),
                    &line,
                    &self.data.policy,
                ),
            }
            .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let from_end = match match regular_range {
                Some(range) => parallel.supporting_line_parameter_order_on_regular_range(
                    contact.parameter(),
                    &reversed_line,
                    range,
                    &self.data.policy,
                ),
                None => parallel.supporting_line_parameter_order(
                    contact.parameter(),
                    &reversed_line,
                    &self.data.policy,
                ),
            }
            .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if from_start == Ordering::Less || from_end == Ordering::Less {
                continue;
            }
            retained_parameters.push(contact.parameter().clone());
        }
        let mut result = match self.parallel_exact_parameter_pair_result(
            parallel,
            curve,
            retained_parameters,
            parallel_is_first,
        )? {
            Classification::Decided(Some(result)) => result,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for certified in retained_certified_tangencies {
            let (line_parameter, point) = match certified.line_endpoint() {
                BezierEndpoint::Start => (Real::zero(), line.start().clone()),
                BezierEndpoint::End => (Real::one(), line.end().clone()),
            };
            let parallel_parameter = BezierParameter2::Exact(certified.parameter().clone());
            let line_parameter = BezierParameter2::Exact(line_parameter);
            let (first_parameter, second_parameter) = if parallel_is_first {
                (parallel_parameter, line_parameter)
            } else {
                (line_parameter, parallel_parameter)
            };
            result
                .contacts
                .push(RegionPairContactEvidence::direct_bezier(
                    first_parameter,
                    second_parameter,
                    Some(CurvePoint2::from(point)),
                    false,
                    None,
                ));
        }
        Ok(Classification::Decided(Some(result)))
    }

    pub(super) fn parallel_arc_pair_result(
        &self,
        parallel: &BezierParallel2,
        range: &CurveParameterRange2,
        curve: &BezierSubcurve2,
        parallel_is_first: bool,
    ) -> ExactCurveResult<Classification<Option<RegionPairResult>>> {
        let segment = match crate::bezier_region::materialized_native_subcurve_segment(
            curve,
            &self.data.policy,
        )
        .map_err(|cause| self.invalid(0, cause))?
        {
            Classification::Decided(segment) => segment,
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let Segment2::Arc(arc) = segment else {
            return Ok(Classification::Decided(None));
        };
        let certified_tangent_contacts = match curve {
            BezierSubcurve2::RationalQuadratic(curve) => curve.retained_circular_conic(),
            BezierSubcurve2::Rational(curve) => curve.retained_circular_conic(),
            BezierSubcurve2::Quadratic(_) | BezierSubcurve2::Cubic(_) => None,
        }
        .and_then(|circle| circle.tangent_contacts.as_deref())
        .into_iter()
        .flatten()
        .filter_map(|contact| match contact {
            crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel(contact)
                if contact.parallel == *parallel =>
            {
                Some(contact)
            }
            crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel(_)
            | crate::rational_bezier::RationalQuadraticCircleTangentContact2::Line { .. } => None,
        })
        .collect::<Vec<_>>();
        let certified_tangent_parameters = certified_tangent_contacts
            .iter()
            .map(|contact| {
                (
                    contact.parameter.clone(),
                    contact.eliminant_root_multiplicity,
                )
            })
            .collect::<Vec<_>>();
        let incidence = match parallel
            .circle_incidence(
                arc.center(),
                arc.radius_squared_ref(),
                range,
                &certified_tangent_parameters,
                &self.data.policy,
            )
            .map_err(|cause| self.invalid(0, cause))?
        {
            Classification::Decided(incidence) => incidence,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut parameters = Vec::with_capacity(incidence.len());
        for (parameter, crossing) in incidence {
            if let Some(contact) = parameter.scalar().and_then(|parameter| {
                certified_tangent_contacts
                    .iter()
                    .find(|contact| contact.parameter == *parameter)
            }) && self.data.policy.bounded_exact_predicate_pass(|| {
                arc.contains_sweep_point(&contact.point, &self.data.policy)
            }) == Classification::Decided(false)
            {
                // Circle incidence is already certified. A finite-sweep
                // exclusion needs neither another radius proof nor a conic
                // inverse at its possible affine infinity.
                continue;
            }
            parameters.push((parameter, crossing));
        }
        // The incidence result already owns radial crossing evidence, including
        // tangency. Use the same conic inverse for represented and selected
        // parameters instead of rebuilding scalar point/tangent intersections.
        let rational =
            RationalBezier2::try_from_subcurve(curve).map_err(|cause| self.invalid(0, cause))?;
        if matches!(
            rational
                .quadratic_homogeneous_controls(&self.data.policy)
                .map_err(|cause| self.invalid(0, cause))?,
            Classification::Decided(Some(_))
        ) {
            let mut contacts = Vec::with_capacity(parameters.len());
            for (parallel_parameter, radial_crossing_sign) in &parameters {
                let certified_contact = parallel_parameter.scalar().and_then(|parameter| {
                    certified_tangent_contacts
                        .iter()
                        .find(|contact| contact.parameter == *parameter)
                });
                if certified_contact.is_none()
                    && let Some(exact) = parallel_parameter.scalar()
                {
                    let point = parallel
                        .point_at(exact, &self.data.policy)
                        .map_err(|cause| self.invalid(0, cause))?;
                    // A finite-arc rejection is optional. Unresolved scalar
                    // coordinates retain the selected point's exact field.
                    if let Classification::Decided(point) = point
                        && self.data.policy.bounded_exact_predicate_pass(|| {
                            arc.contains_sweep_point(&point, &self.data.policy)
                        }) == Classification::Decided(false)
                    {
                        continue;
                    }
                }
                let point = certified_contact.map_or_else(
                    || {
                        CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new(
                            parallel.clone(),
                            parallel_parameter.clone(),
                            &self.data.policy,
                        ))
                    },
                    |contact| CurvePoint2::from(contact.point.clone()),
                );
                let other_parameter = if let Some(contact) = certified_contact
                    && contact.point == *rational.start()
                {
                    CurveParameter2::from(Real::zero())
                } else if let Some(contact) = certified_contact
                    && contact.point == *rational.end()
                {
                    CurveParameter2::from(Real::one())
                } else {
                    // Circle incidence already proves that this point lies on
                    // the conic. Its homogeneous inverse stays in the retained
                    // point field and decides the original closed unit chart;
                    // no independent image polynomial is needed for the cut.
                    match crate::bezier_offset::quadratic_conic_parameter_at_incident_point(
                        &point,
                        &rational,
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(0, cause))?
                    {
                        Classification::Decided(Some(parameter)) => parameter,
                        Classification::Decided(None) => continue,
                        Classification::Uncertain(_) => {
                            return Ok(Classification::Decided(None));
                        }
                    }
                };
                let parallel_parameter = CurveParameter2::from(parallel_parameter.clone());
                let (first_parameter, second_parameter) = if parallel_is_first {
                    (parallel_parameter, other_parameter)
                } else {
                    (other_parameter, parallel_parameter)
                };
                let tangent_cross_sign = radial_crossing_sign.map(|sign| {
                    if arc.is_clockwise() ^ !parallel_is_first {
                        match sign {
                            RealSign::Positive => RealSign::Negative,
                            RealSign::Negative => RealSign::Positive,
                            RealSign::Zero => RealSign::Zero,
                        }
                    } else {
                        sign
                    }
                });
                contacts.push(RegionPairContactEvidence::direct(
                    first_parameter,
                    second_parameter,
                    Some(point),
                    matches!(
                        tangent_cross_sign,
                        Some(RealSign::Positive | RealSign::Negative)
                    ),
                    tangent_cross_sign,
                ));
            }
            return Ok(Classification::Decided(Some(RegionPairResult {
                contacts,
                overlaps: Vec::new(),
                blockers: Vec::new(),
            })));
        }
        let mut retained_parameters = Vec::with_capacity(parameters.len());
        for (parameter, _) in parameters {
            if let Some(contact) = parameter.scalar().and_then(|parameter| {
                certified_tangent_contacts
                    .iter()
                    .find(|contact| contact.parameter == *parameter)
            }) {
                match arc.contains_point(&contact.point, &self.data.policy) {
                    Classification::Decided(true) => retained_parameters.push(parameter),
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                continue;
            }
            let Some(exact) = parameter.scalar() else {
                return Ok(Classification::Decided(None));
            };
            let point = match parallel
                .point_at(exact, &self.data.policy)
                .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match arc.contains_point(&point, &self.data.policy) {
                Classification::Decided(true) => retained_parameters.push(parameter),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        self.parallel_exact_parameter_pair_result(
            parallel,
            curve,
            retained_parameters,
            parallel_is_first,
        )
    }

    pub(super) fn parallel_exact_parameter_pair_result(
        &self,
        parallel: &BezierParallel2,
        curve: &BezierSubcurve2,
        parallel_parameters: Vec<BezierParameter2>,
        parallel_is_first: bool,
    ) -> ExactCurveResult<Classification<Option<RegionPairResult>>> {
        let rational =
            RationalBezier2::try_from_subcurve(curve).map_err(|cause| self.invalid(0, cause))?;
        let mut result_contacts = Vec::with_capacity(parallel_parameters.len());
        for parallel_parameter in parallel_parameters {
            let Some(parallel_parameter_exact) = parallel_parameter.scalar() else {
                return Ok(Classification::Decided(None));
            };
            let point = match parallel.point_at(parallel_parameter_exact, &self.data.policy) {
                Ok(Classification::Decided(point)) => point,
                Ok(Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
                Err(cause) => return Err(self.invalid(0, cause)),
            };
            let other_parameters = match rational
                .point_incidence_on_range(
                    &point,
                    &crate::CurveParameterRange2::unit(),
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(RationalBezierPointIncidence2::Parameters(parameters)) => {
                    parameters
                }
                Classification::Decided(RationalBezierPointIncidence2::EntireCurve) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let parallel_derivative = match parallel
                .derivative_at(parallel_parameter_exact, &self.data.policy)
                .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(derivative) => derivative,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            for other_parameter in other_parameters {
                let Some(other_parameter_exact) = other_parameter.scalar() else {
                    return Ok(Classification::Decided(None));
                };
                let other_derivative = match rational
                    .derivative_at_classified(other_parameter_exact, &self.data.policy)
                {
                    Classification::Decided(derivative) => derivative,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let cross = parallel_derivative.dx() * other_derivative.dy()
                    - parallel_derivative.dy() * other_derivative.dx();
                // This is an optional transverse hint. An unresolved exact
                // zero already means no hint; consuming an approximate zero
                // would unnecessarily weaken all later retained contacts.
                let parallel_cross_other = self
                    .data
                    .policy
                    .bounded_exact_predicate_pass(|| real_sign(&cross, &self.data.policy));
                let tangent_cross_sign = parallel_cross_other.and_then(|sign| match sign {
                    RealSign::Positive | RealSign::Negative => Some(if parallel_is_first {
                        sign
                    } else {
                        match sign {
                            RealSign::Positive => RealSign::Negative,
                            RealSign::Negative => RealSign::Positive,
                            RealSign::Zero => unreachable!(),
                        }
                    }),
                    RealSign::Zero => None,
                });
                let (first_parameter, second_parameter) = if parallel_is_first {
                    (parallel_parameter.clone(), other_parameter)
                } else {
                    (other_parameter, parallel_parameter.clone())
                };
                result_contacts.push(RegionPairContactEvidence::direct_bezier(
                    first_parameter,
                    second_parameter,
                    Some(CurvePoint2::from(point.clone())),
                    tangent_cross_sign.is_some(),
                    tangent_cross_sign,
                ));
            }
        }
        Ok(Classification::Decided(Some(RegionPairResult {
            contacts: result_contacts,
            overlaps: Vec::new(),
            blockers: Vec::new(),
        })))
    }

    pub(super) fn authored_carriers_are_adjacent(&self, pair: &RegionCarrierPair) -> bool {
        self.authored_carrier_shared_endpoints(pair.first_carrier_index, pair.second_carrier_index)
            .is_some()
    }

    /// Returns the authored endpoint shared by each carrier. `true` names its
    /// traversal start and `false` its traversal end.
    pub(super) fn authored_carrier_shared_endpoints(
        &self,
        first: usize,
        second: usize,
    ) -> Option<(bool, bool)> {
        let first = &self.data.carriers[first];
        let second = &self.data.carriers[second];
        if first.operand != second.operand || first.loop_index != second.loop_index {
            return None;
        }
        let region = match first.operand {
            CurveRegionBooleanOperand2::First => self.data.first,
            CurveRegionBooleanOperand2::Second => self.data.second,
        };
        let fragment_count = region
            .boundary_loops()
            .get(first.loop_index)
            .map(|boundary| boundary.fragments().len())?;
        let first_precedes_second = first.fragment_index.checked_add(1)
            == Some(second.fragment_index)
            || (second.fragment_index == 0
                && first.fragment_index.checked_add(1) == Some(fragment_count));
        let second_precedes_first = second.fragment_index.checked_add(1)
            == Some(first.fragment_index)
            || (first.fragment_index == 0
                && second.fragment_index.checked_add(1) == Some(fragment_count));
        if first_precedes_second {
            Some((false, true))
        } else if second_precedes_first {
            Some((true, false))
        } else {
            None
        }
    }

    /// Recovers a full-circle endpoint contact from an adjacent sibling
    /// chart. Long selected arcs are stored as consecutive half-circle
    /// fragments; a curve adjacent to one half meets the same complete circle
    /// represented by the other half. Retain the sibling chart and both
    /// endpoint identities, with any additional contact proof: the authored
    /// angular parameter decides half-chart ownership without a new solve.
    pub(super) fn authored_supporting_circle_endpoint(
        &self,
        cusp_index: usize,
        other_index: usize,
        qualifies: impl Fn(&crate::BezierAlgebraicCuspSemicircleFragment2, bool) -> bool,
    ) -> Option<(usize, bool, bool)> {
        let cusp = match &self.data.carriers.get(cusp_index)?.geometry {
            CurveSupport2::Circle(cusp) => cusp,
            _ => return None,
        };
        let other_carrier = self.data.carriers.get(other_index)?;
        let mut certified = None;
        for (candidate_index, candidate) in self.data.carriers.iter().enumerate() {
            let CurveSupport2::Circle(candidate_cusp) = &candidate.geometry else {
                continue;
            };
            if candidate.operand != other_carrier.operand
                || candidate.loop_index != other_carrier.loop_index
                || !cusp
                    .semicircle()
                    .shares_structural_supporting_circle(candidate_cusp.semicircle())
            {
                continue;
            }
            let Some((candidate_at_start, other_at_start)) =
                self.authored_carrier_shared_endpoints(candidate_index, other_index)
            else {
                continue;
            };
            if !qualifies(candidate_cusp, candidate_at_start) {
                continue;
            }
            match certified {
                Some((_, _, previous)) if previous != other_at_start => return None,
                Some(_) => {}
                None => certified = Some((candidate_index, candidate_at_start, other_at_start)),
            }
        }
        certified
    }

    pub(super) fn algebraic_chord_linear_bezier_pair_result(
        &self,
        pair: &RegionCarrierPair,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        curve: &BezierSubcurve2,
        curve_index: usize,
    ) -> ExactCurveResult<Option<RegionPairResult>> {
        let Some(chord_line) = chord.exact_line() else {
            return Ok(None);
        };
        let rational = RationalBezier2::try_from_subcurve(curve)
            .map_err(|cause| self.invalid(curve_index, cause))?;
        let Some(curve_line) = rational.exact_linear_parameterization_line() else {
            return Ok(None);
        };
        let relation = chord_line
            .intersect_line(&curve_line, &self.data.policy)
            .map_err(|cause| self.invalid(chord_index, cause))?;
        let blocker = |reason| RegionPairResult {
            contacts: Vec::new(),
            overlaps: Vec::new(),
            blockers: vec![RegionPairBlocker::Uncertain(reason)],
        };
        let chord_parameter = |point: &crate::Point2| match chord
            .parameter_at_certified_point(CurvePoint2::from(point.clone()), &self.data.policy)
            .map_err(|cause| self.invalid(chord_index, cause))?
        {
            Classification::Decided(Some(parameter)) => Ok(Classification::Decided(
                CurveParameter2::from_algebraic_chord(parameter),
            )),
            Classification::Decided(None) => Err(self.invalid(
                chord_index,
                CurveError::Topology(
                    "an exact chord support contact was outside its finite chord".into(),
                ),
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        };
        let chord_is_first = chord_index == pair.first_carrier_index;
        let result = match relation {
            crate::LineLineIntersection::None => RegionPairResult::empty(),
            crate::LineLineIntersection::Uncertain { reason } => blocker(reason),
            crate::LineLineIntersection::Point { point, b_param, .. } => {
                if self.authored_carriers_are_adjacent(pair) {
                    RegionPairResult::empty()
                } else {
                    let chord_parameter = match chord_parameter(&point)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => return Ok(Some(blocker(reason))),
                    };
                    let curve_parameter = CurveParameter2::from(BezierParameter2::Exact(b_param));
                    let (chord_dx, chord_dy) = chord_line.delta();
                    let (curve_dx, curve_dy) = curve_line.delta();
                    let cross = Real::diff_of_products(&chord_dx, &curve_dy, &chord_dy, &curve_dx);
                    let Some(cross_sign) = real_sign(&cross, &self.data.policy) else {
                        return Ok(Some(blocker(UncertaintyReason::RealSign)));
                    };
                    let cross_sign = orient_tangent_cross_sign(cross_sign, chord_is_first);
                    let (first_parameter, second_parameter) = if chord_is_first {
                        (chord_parameter, curve_parameter)
                    } else {
                        (curve_parameter, chord_parameter)
                    };
                    RegionPairResult {
                        contacts: vec![RegionPairContactEvidence::direct(
                            first_parameter,
                            second_parameter,
                            Some(CurvePoint2::from(point)),
                            cross_sign != RealSign::Zero,
                            Some(cross_sign),
                        )],
                        overlaps: Vec::new(),
                        blockers: Vec::new(),
                    }
                }
            }
            crate::LineLineIntersection::Overlap {
                segment, b_range, ..
            } => {
                let chord_start = match chord_parameter(segment.start())? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => return Ok(Some(blocker(reason))),
                };
                let chord_end = match chord_parameter(segment.end())? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => return Ok(Some(blocker(reason))),
                };
                let chord_range = CurveParameterRange2::new_validated(chord_start, chord_end);
                let (curve_start, curve_end, orientation) =
                    match compare_reals(b_range.start(), b_range.end(), &self.data.policy) {
                        Some(Ordering::Less) => (
                            b_range.start().clone(),
                            b_range.end().clone(),
                            CurveOverlapOrientation2::Same,
                        ),
                        Some(Ordering::Greater) => (
                            b_range.end().clone(),
                            b_range.start().clone(),
                            CurveOverlapOrientation2::Reversed,
                        ),
                        Some(Ordering::Equal) => {
                            return Err(self.invalid(
                                curve_index,
                                CurveError::Topology(
                                    "a positive-length exact line overlap had zero parameter range"
                                        .into(),
                                ),
                            ));
                        }
                        None => return Ok(Some(blocker(UncertaintyReason::Ordering))),
                    };
                let curve_range = CurveParameterRange2::new_validated(
                    CurveParameter2::from(BezierParameter2::Exact(curve_start)),
                    CurveParameter2::from(BezierParameter2::Exact(curve_end)),
                );
                let correspondence = CurveOverlapCorrespondence2::ChordRational {
                    source: Arc::new(BezierAlgebraicChordRationalOverlap2::from_certified_ranges(
                        chord.clone(),
                        rational.clone(),
                        [chord_range.start(), chord_range.end()].map(|p| {
                            p.as_algebraic_chord()
                                .expect("certified chord range")
                                .clone()
                        }),
                        CurveParameterRange2::new_validated(
                            b_range.start().clone().into(),
                            b_range.end().clone().into(),
                        ),
                        orientation,
                    )),
                    chord_first: chord_is_first,
                };
                let (first_range, second_range) = if chord_is_first {
                    (chord_range, curve_range)
                } else {
                    (curve_range, chord_range)
                };
                RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: vec![CurveIntersectionOverlap2 {
                        first_span_index: 0,
                        second_span_index: 0,
                        endpoint_inclusion: [true, true],
                        first_range,
                        second_range,
                        orientation,
                        parameter_correspondence: correspondence,
                    }],
                    blockers: Vec::new(),
                }
            }
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-pair",
            "exact-linear-bezier",
        );
        Ok(Some(result))
    }

    /// Replays finite chord contacts through an authored adjacent Bezier whose
    /// image is shared with the other carrier.
    ///
    /// A retained chord endpoint and its adjacent boundary-carrier endpoint
    /// are already the same exact topology vertex.  Mapping that Bezier
    /// parameter through a certified rational-image overlap therefore gives
    /// an exact parameter for the chord endpoint on the other carrier without
    /// comparing independently adjoined point-coordinate fields.  The
    /// supporting-line kernel remains the completeness authority: this path
    /// succeeds only when its complete finite contact set matches those
    /// transported endpoints one-to-one.
    pub(super) fn algebraic_chord_shared_image_endpoint_pair_result(
        &self,
        pair: &RegionCarrierPair,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        target: &RationalBezier2,
        target_index: usize,
    ) -> ExactCurveResult<Option<RegionPairResult>> {
        let chord_carrier = &self.data.carriers[chord_index];
        let target_carrier = &self.data.carriers[target_index];
        if chord_carrier.operand == target_carrier.operand {
            return Ok(None);
        }
        let region = match chord_carrier.operand {
            CurveRegionBooleanOperand2::First => self.data.first,
            CurveRegionBooleanOperand2::Second => self.data.second,
        };
        let Some(fragment_count) = region
            .boundary_loops()
            .get(chord_carrier.loop_index)
            .map(|boundary| boundary.fragments().len())
        else {
            return Ok(None);
        };
        if fragment_count < 2 {
            return Ok(None);
        }
        let predecessor_fragment = if chord_carrier.fragment_index == 0 {
            fragment_count - 1
        } else {
            chord_carrier.fragment_index - 1
        };
        let successor_fragment = (chord_carrier.fragment_index + 1) % fragment_count;
        let adjacent_carrier = |fragment_index| {
            self.data.carriers.iter().enumerate().find(|(_, carrier)| {
                carrier.operand == chord_carrier.operand
                    && carrier.loop_index == chord_carrier.loop_index
                    && carrier.fragment_index == fragment_index
            })
        };

        let mut mapped_endpoints = Vec::with_capacity(2);
        for (fragment_index, source_parameter, chord_parameter, point) in [
            (
                predecessor_fragment,
                true,
                carrier_traversal_start(chord_carrier),
                chord.start(),
            ),
            (
                successor_fragment,
                false,
                carrier_traversal_end(chord_carrier),
                chord.end(),
            ),
        ] {
            let Some((source_index, source_carrier)) = adjacent_carrier(fragment_index) else {
                continue;
            };
            let CurveSupport2::Bezier(source_curve) = &source_carrier.geometry else {
                continue;
            };
            let source_parameter = if source_parameter {
                carrier_traversal_end(source_carrier)
            } else {
                carrier_traversal_start(source_carrier)
            };
            let Some(source_parameter) = source_parameter.as_bezier_parameter() else {
                continue;
            };
            let source = RationalBezier2::try_from_subcurve(source_curve)
                .map_err(|cause| self.invalid(source_index, cause))?;
            let target_parameter =
                match RationalBezierOverlapParameterCorrespondence2::map_parameter_between_curves(
                    &source,
                    target,
                    source_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(source_index, cause))?
                {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) | Classification::Uncertain(_) => continue,
                };
            let target_region_parameter = CurveParameter2::from(target_parameter.clone());
            match parameter_in_carrier(&target_region_parameter, target_carrier, &self.data.policy)
            {
                Ok(true) => mapped_endpoints.push((
                    target_parameter,
                    chord_parameter.clone(),
                    point.clone(),
                )),
                Ok(false) | Err(ExactCurveError::Blocked(_)) => {}
                Err(error) => return Err(error),
            }
        }
        if mapped_endpoints.is_empty() {
            return Ok(None);
        }

        let Some(support_line) = chord
            .exact_line()
            .or_else(|| chord.strict_provenance_support_line(&self.data.policy))
        else {
            return Ok(None);
        };
        let line_contacts = match target
            .relation_to_line_with_contacts(&support_line, &self.data.policy)
        {
            Classification::Decided(
                BezierLineContactRelation::ControlHullDisjoint { .. }
                | BezierLineContactRelation::NoContact,
            ) => Vec::new(),
            Classification::Decided(BezierLineContactRelation::Contacts { contacts }) => contacts,
            Classification::Decided(BezierLineContactRelation::OnSupportingLine)
            | Classification::Uncertain(_) => return Ok(None),
        };
        let mut finite_contacts = Vec::with_capacity(line_contacts.len());
        for contact in line_contacts {
            let parameter = CurveParameter2::from(contact.parameter().clone());
            match parameter_in_carrier(&parameter, target_carrier, &self.data.policy) {
                Ok(true) => finite_contacts.push(contact),
                Ok(false) => {}
                Err(ExactCurveError::Blocked(_)) => return Ok(None),
                Err(error) => return Err(error),
            }
        }
        if finite_contacts.len() != mapped_endpoints.len() {
            return Ok(None);
        }

        let chord_is_first = chord_index == pair.first_carrier_index;
        let mut matched = vec![false; mapped_endpoints.len()];
        let mut contacts = Vec::with_capacity(finite_contacts.len());
        for contact in finite_contacts {
            let mut match_index = None;
            for (index, (parameter, _, _)) in mapped_endpoints.iter().enumerate() {
                if matched[index] {
                    continue;
                }
                match parameter
                    .same_value(contact.parameter(), &self.data.policy)
                    .map_err(|cause| self.invalid(target_index, cause))?
                {
                    Classification::Decided(true) if match_index.is_none() => {
                        match_index = Some(index);
                    }
                    Classification::Decided(true) | Classification::Uncertain(_) => {
                        return Ok(None);
                    }
                    Classification::Decided(false) => {}
                }
            }
            let Some(match_index) = match_index else {
                return Ok(None);
            };
            matched[match_index] = true;
            let (_, chord_parameter, point) = &mapped_endpoints[match_index];
            let chord_cross_target = match contact.crossing_direction() {
                Some(BezierLineCrossingDirection::NegativeToPositive) => RealSign::Positive,
                Some(BezierLineCrossingDirection::PositiveToNegative) => RealSign::Negative,
                None => RealSign::Zero,
            };
            let tangent_cross_sign = orient_tangent_cross_sign(chord_cross_target, chord_is_first);
            let target_parameter = CurveParameter2::from(contact.parameter().clone());
            let (first_parameter, second_parameter) = if chord_is_first {
                (chord_parameter.clone(), target_parameter)
            } else {
                (target_parameter, chord_parameter.clone())
            };
            contacts.push(RegionPairContactEvidence::direct(
                first_parameter,
                second_parameter,
                Some(point.clone()),
                tangent_cross_sign != RealSign::Zero,
                Some(tangent_cross_sign),
            ));
        }
        if matched.iter().any(|matched| !matched) {
            return Ok(None);
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-pair",
            "shared-image-endpoints",
        );
        Ok(Some(RegionPairResult {
            contacts,
            overlaps: Vec::new(),
            blockers: Vec::new(),
        }))
    }

    pub(super) fn algebraic_chord_rational_pair_result(
        &self,
        pair: &RegionCarrierPair,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        rational: &RationalBezier2,
        regular_component: Option<&BezierParallelRationalComponent2>,
        shared_source_parameter: Option<&CurveParameter2>,
    ) -> ExactCurveResult<Option<RegionPairResult>> {
        let other_index = if chord_index == pair.first_carrier_index {
            pair.second_carrier_index
        } else {
            pair.first_carrier_index
        };
        // Carriers from different operands can still share an exact vertex,
        // for example a boundary rebuilt from its own exported curves. The
        // kernel would rediscover that contact as a selected root and then
        // have to order it against an independently constructed endpoint.
        // A certified equal endpoint is owned here instead.
        let discovered_owned_contact;
        let mut owned_contact = None;
        let shared_source_parameter = match shared_source_parameter {
            Some(parameter) => Some(parameter),
            None if self.data.carriers[chord_index].operand
                != self.data.carriers[other_index].operand =>
            {
                discovered_owned_contact = self.cross_operand_chord_endpoint_contact(
                    pair,
                    chord,
                    chord_index,
                    other_index,
                )?;
                discovered_owned_contact
                    .as_ref()
                    .map(|(parameter, contact)| {
                        owned_contact = Some(contact.clone());
                        parameter
                    })
            }
            None => None,
        };
        let collinear_support = if let Some(line) =
            regular_component.and_then(BezierParallelRationalComponent2::support_line)
        {
            matches!(
                self.data
                    .policy
                    .strict_predicate_pass(|| {
                        chord.has_non_collinear_support_with_exact_line(line, &self.data.policy)
                    })
                    .map_err(|cause| self.invalid(other_index, cause))?,
                Classification::Decided(false),
            )
        } else {
            false
        };
        let mut linear_intersections = None;
        if !collinear_support && let Some(component) = regular_component {
            // Unit-chart projection is complete only when it covers the
            // retained range. Exterior ranges use the common domain replay.
            let unit = CurveParameterRange2::unit();
            let domain = CurveParameterDomain2::new(&unit, None);
            for endpoint in [
                component.regular_range().start(),
                component.regular_range().end(),
            ] {
                if domain
                    .contains_finite_parameter(endpoint, &self.data.policy)
                    .map_err(|cause| self.invalid(other_index, cause))?
                    != Classification::Decided(true)
                {
                    return Ok(None);
                }
            }
            linear_intersections = chord
                .exact_linear_rational_intersections(rational, &self.data.policy)
                .map_err(|cause| self.invalid(other_index, cause))?;
        }
        let intersections = if collinear_support {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair",
                "certified-rational-support-collinear",
            );
            let component =
                regular_component.expect("the regular component certified its line support");
            chord
                .collinear_rational_intersections_on_regular_component(
                    component,
                    shared_source_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(other_index, cause))?
        } else if let Some(mut intersections) = linear_intersections {
            // Two finite straight supports have at most the owned seam as
            // an isolated contact when their authored carriers are adjacent.
            // Positive overlaps retain their full correspondence.
            if self.authored_carriers_are_adjacent(pair)
                && let BezierAlgebraicChordRationalIntersections2::Contacts(contacts) =
                    &mut intersections
            {
                contacts.clear();
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair",
                "exact-linear-parallel-chord-authority",
            );
            Classification::Decided(intersections)
        } else {
            chord
                .rational_intersections(
                    rational,
                    &CurveParameterRange2::new_validated(
                        self.data.carriers[other_index].start.clone(),
                        self.data.carriers[other_index].end.clone(),
                    ),
                    shared_source_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(other_index, cause))?
        };
        let complete = match intersections {
            Classification::Decided(BezierAlgebraicChordRationalIntersections2::Contacts(
                contacts,
            )) => Some((contacts, Vec::new())),
            Classification::Decided(BezierAlgebraicChordRationalIntersections2::Overlaps(
                overlaps,
            )) => Some((Vec::new(), overlaps)),
            Classification::Decided(
                BezierAlgebraicChordRationalIntersections2::ContactsAndOverlaps {
                    contacts,
                    overlaps,
                },
            ) => Some((contacts, overlaps)),
            Classification::Decided(
                BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
            ) => {
                return Ok(Some(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                    blockers: vec![RegionPairBlocker::Uncertain(UncertaintyReason::Boundary)],
                }));
            }
            Classification::Decided(
                BezierAlgebraicChordRationalIntersections2::NotSourceRelated,
            ) => None,
            Classification::Uncertain(reason) => {
                return Ok(Some(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                    blockers: vec![RegionPairBlocker::Uncertain(reason)],
                }));
            }
        };
        let Some((contacts, overlaps)) = complete else {
            return Ok(None);
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-pair",
            if overlaps.is_empty() {
                if self.authored_carriers_are_adjacent(pair) {
                    "adjacent-source-complete"
                } else {
                    "source-complete"
                }
            } else {
                "collinear-overlap-complete"
            },
        );
        let chord_is_first = chord_index == pair.first_carrier_index;
        let contacts = contacts
            .into_iter()
            .map(|contact| {
                let tangent_cross_sign = if chord_is_first {
                    contact.tangent_cross_sign()
                } else {
                    match contact.tangent_cross_sign() {
                        RealSign::Positive => RealSign::Negative,
                        RealSign::Negative => RealSign::Positive,
                        RealSign::Zero => RealSign::Zero,
                    }
                };
                let chord_parameter =
                    CurveParameter2::from_algebraic_chord(contact.chord_parameter().clone());
                let other_parameter = contact.other_parameter().clone();
                let (first_parameter, second_parameter) = if chord_is_first {
                    (chord_parameter, other_parameter)
                } else {
                    (other_parameter, chord_parameter)
                };
                RegionPairContactEvidence::direct(
                    first_parameter,
                    second_parameter,
                    Some(contact.point().clone()),
                    tangent_cross_sign != RealSign::Zero,
                    Some(tangent_cross_sign),
                )
            })
            .collect();
        let overlaps = overlaps
            .into_iter()
            .map(|overlap| {
                let [chord_start, chord_end] = overlap.chord_range();
                let chord_range = CurveParameterRange2::new_validated(
                    CurveParameter2::from_algebraic_chord(chord_start.clone()),
                    CurveParameter2::from_algebraic_chord(chord_end.clone()),
                );
                let source_range = overlap.source_range().clone();
                let orientation = overlap.orientation();
                let (first_range, second_range) = if chord_is_first {
                    (chord_range, source_range)
                } else {
                    (source_range, chord_range)
                };
                CurveIntersectionOverlap2 {
                    first_span_index: 0,
                    second_span_index: 0,
                    endpoint_inclusion: [true, true],
                    parameter_correspondence: CurveOverlapCorrespondence2::ChordRational {
                        source: Arc::new(overlap),
                        chord_first: chord_is_first,
                    },
                    first_range,
                    second_range,
                    orientation,
                }
            })
            .collect();
        let mut contacts: Vec<RegionPairContactEvidence> = contacts;
        contacts.extend(owned_contact);
        Ok(Some(RegionPairResult {
            contacts,
            overlaps,
            blockers: Vec::new(),
        }))
    }

    /// Finds a chord endpoint that exactly equals an endpoint of a carrier from
    /// the other operand, returning that carrier parameter and the contact.
    /// Equality is decided coordinate by coordinate under STRICT; the contact
    /// makes no transversality claim, leaving local topology to the arrangement.
    pub(super) fn cross_operand_chord_endpoint_contact(
        &self,
        pair: &RegionCarrierPair,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        other_index: usize,
    ) -> ExactCurveResult<Option<(CurveParameter2, RegionPairContactEvidence)>> {
        let strict = self.data.policy.strict_counterpart();
        let other = &self.data.carriers[other_index];
        for parameter in [&other.start, &other.end] {
            let Some(point) = exact_carrier_point(other, parameter, &self.data.policy) else {
                continue;
            };
            for chord_end in [chord.start(), chord.end()] {
                let Some(end) = chord_end.coordinates() else {
                    continue;
                };
                let equal = |left: &Real, right: &Real| {
                    strict.strict_predicate_pass(|| {
                        crate::classify::compare_reals(left, right, &strict)
                            == Some(std::cmp::Ordering::Equal)
                    })
                };
                if !equal(point.x(), end.x()) || !equal(point.y(), end.y()) {
                    continue;
                }
                let shared = CurvePoint2::from(end.clone());
                let Classification::Decided(Some(chord_parameter)) = chord
                    .parameter_at_certified_point(shared.clone(), &self.data.policy)
                    .map_err(|cause| self.invalid(chord_index, cause))?
                else {
                    continue;
                };
                let chord_parameter = CurveParameter2::from_algebraic_chord(chord_parameter);
                let (first_parameter, second_parameter) = if chord_index == pair.first_carrier_index
                {
                    (chord_parameter, parameter.clone())
                } else {
                    (parameter.clone(), chord_parameter)
                };
                return Ok(Some((
                    parameter.clone(),
                    RegionPairContactEvidence::direct(
                        first_parameter,
                        second_parameter,
                        Some(shared),
                        false,
                        None,
                    ),
                )));
            }
        }
        Ok(None)
    }

    pub(super) fn algebraic_chord_parallel_pair_result(
        &self,
        pair: &RegionCarrierPair,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        parallel: &BezierParallel2,
        parallel_index: usize,
    ) -> ExactCurveResult<RegionPairResult> {
        let parallel_carrier = &self.data.carriers[parallel_index];
        let retained_range = CurveParameterRange2::new_validated(
            parallel_carrier.start.clone(),
            parallel_carrier.end.clone(),
        );
        let retained_contact_result = |contacts: Vec<
            crate::bezier_offset::BezierAlgebraicChordParallelContact2,
        >| {
            let chord_is_first = chord_index == pair.first_carrier_index;
            let contacts = contacts
                .into_iter()
                .map(|contact| {
                    let tangent_cross_sign =
                        orient_tangent_cross_sign(contact.tangent_cross_sign(), chord_is_first);
                    let chord_parameter =
                        CurveParameter2::from_algebraic_chord(contact.chord_parameter().clone());
                    let parallel_parameter = contact.parallel_parameter().clone();
                    let (first_parameter, second_parameter) = if chord_is_first {
                        (chord_parameter, parallel_parameter)
                    } else {
                        (parallel_parameter, chord_parameter)
                    };
                    RegionPairContactEvidence::direct(
                        first_parameter,
                        second_parameter,
                        Some(contact.point().clone()),
                        tangent_cross_sign != RealSign::Zero,
                        Some(tangent_cross_sign),
                    )
                })
                .collect();
            RegionPairResult {
                contacts,
                overlaps: Vec::new(),
                blockers: Vec::new(),
            }
        };
        let retained_monotone_contact_result =
            |contact: crate::bezier_offset::BezierAlgebraicChordRetainedParallelContact2| {
                let chord_is_first = chord_index == pair.first_carrier_index;
                let tangent_cross_sign =
                    orient_tangent_cross_sign(contact.tangent_cross_sign(), chord_is_first);
                let chord_parameter =
                    CurveParameter2::from_algebraic_chord(contact.chord_parameter().clone());
                let parallel_parameter = contact.parallel_parameter().clone();
                let (first_parameter, second_parameter) = if chord_is_first {
                    (chord_parameter, parallel_parameter)
                } else {
                    (parallel_parameter, chord_parameter)
                };
                RegionPairResult {
                    contacts: vec![RegionPairContactEvidence::direct(
                        first_parameter,
                        second_parameter,
                        Some(contact.point().clone()),
                        true,
                        Some(tangent_cross_sign),
                    )],
                    overlaps: Vec::new(),
                    blockers: Vec::new(),
                }
            };
        // Nonadjacent carriers have no authored endpoint to discharge.  Ask
        // the authoritative retained-support kernel first; monotonicity is a
        // completeness fallback for a support projection that stays blocked,
        // not a reason to spend exponential endpoint-refinement work before a
        // complete support answer that is already available.
        let mut authoritative_support_result = if self.authored_carriers_are_adjacent(pair) {
            None
        } else {
            Some(self.algebraic_chord_parallel_support_pair_result(
                pair,
                chord,
                chord_index,
                parallel,
                parallel_index,
            )?)
        };
        if authoritative_support_result
            .as_ref()
            .is_some_and(|result| result.blockers.is_empty())
        {
            return Ok(authoritative_support_result
                .take()
                .expect("the complete support result was retained above"));
        }
        {
            let monotonic = chord
                .parallel_tangent_cross_sign_on_region_range(
                    parallel,
                    &retained_range,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(parallel_index, cause))?;
            if let Classification::Decided(
                monotonic_sign @ (RealSign::Positive | RealSign::Negative),
            ) = monotonic
            {
                if self.authored_carriers_are_adjacent(pair) {
                    // The authored chain already owns one common endpoint. A
                    // strict support-incidence derivative over the complete
                    // retained span proves that this is its only contact.
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair",
                        "adjacent-parallel-monotone-complete",
                    );
                    return Ok(RegionPairResult::empty());
                }
                let endpoint_side = |parameter: &CurveParameter2| {
                    self.data.policy.strict_predicate_pass(|| {
                        let point = match parallel.point_evidence_on_regular_range(
                            parameter,
                            &retained_range,
                            &self.data.policy,
                        )? {
                            Classification::Decided(point) => point,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        #[cfg(test)]
                        let debug_kind = |point: &CurvePoint2| {
                            match point {
                                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                                CurvePoint2(CurvePointData2::Algebraic(_)) => {
                                    "algebraic"
                                }
                                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => {
                                    "pair"
                                }
                                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => {
                                    "cusp"
                                }
                                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => {
                                    "derived"
                                }
                                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => {
                                    "parallel"
                                }
                                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => {
                                    "analytic"
                                }
                                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                                    "similarity"
                                }
                            }
                        };
                        let retained_side =
                            chord.retained_procedural_point_side(&point, &self.data.policy)?;
                        #[cfg(test)]
                        if std::env::var_os("HYPERCURVE_DEBUG_PARALLEL_ENDPOINT_SIDE").is_some() {
                            eprintln!(
                                "parallel endpoint side point={} chord=({},{}) retained={retained_side:?}",
                                debug_kind(&point),
                                debug_kind(chord.start()),
                                debug_kind(chord.end()),
                            );
                        }
                        if let Some(side) = retained_side {
                            return Ok(Classification::Decided(side));
                        }
                        let interval = chord.strict_oriented_side_by_local_interval_refinement(
                            &point,
                            &self.data.policy,
                        )?;
                        #[cfg(test)]
                        if std::env::var_os("HYPERCURVE_DEBUG_PARALLEL_ENDPOINT_SIDE").is_some() {
                            eprintln!("parallel endpoint local interval={interval:?}");
                        }
                        if matches!(interval, Classification::Decided(_)) {
                            return Ok(interval);
                        }
                        if chord.certified_unit_tangent().is_some() {
                            let certified = chord.certified_tangent_side(&point, &self.data.policy);
                            if matches!(certified, Classification::Decided(_)) {
                                return Ok(certified);
                            }
                        }
                        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
                    })
                };
                let sides = [
                    endpoint_side(retained_range.start()),
                    endpoint_side(retained_range.end()),
                ];
                let mut sides = match sides {
                    [Ok(first), Ok(second)] => [first, second],
                    [Err(cause), _] | [_, Err(cause)] => {
                        return Err(self.invalid(chord_index, cause));
                    }
                };
                // A strict derivative sign orders the two endpoint
                // incidences. If one retained endpoint has a nonzero side and
                // moving to the unknown endpoint changes incidence in that
                // same direction, the unknown endpoint has the same side.
                // This consumes only monotonicity and one compact contact
                // certificate; no endpoint coordinate is materialized.
                for known_index in 0..2 {
                    let unknown_index = 1 - known_index;
                    let Classification::Decided(known_side) = &sides[known_index] else {
                        continue;
                    };
                    let known_side = *known_side;
                    if matches!(sides[unknown_index], Classification::Decided(_)) {
                        continue;
                    }
                    let parameter_order = match retained_range
                        .start()
                        .cmp_by_refinement(retained_range.end(), &self.data.policy)
                        .map_err(|cause| self.invalid(parallel_index, cause))?
                    {
                        Classification::Decided(
                            order @ (std::cmp::Ordering::Less | std::cmp::Ordering::Greater),
                        ) => order,
                        Classification::Decided(std::cmp::Ordering::Equal)
                        | Classification::Uncertain(_) => continue,
                    };
                    let end_minus_start = if parameter_order == std::cmp::Ordering::Less {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    };
                    let unknown_minus_known = if unknown_index == 1 {
                        end_minus_start
                    } else {
                        match end_minus_start {
                            RealSign::Positive => RealSign::Negative,
                            RealSign::Negative => RealSign::Positive,
                            RealSign::Zero => unreachable!("a strict parameter order is nonzero"),
                        }
                    };
                    let difference_sign = if monotonic_sign == unknown_minus_known {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    };
                    let known_sign = match known_side {
                        LineSide::Left => RealSign::Positive,
                        LineSide::On => RealSign::Zero,
                        LineSide::Right => RealSign::Negative,
                    };
                    if known_sign == RealSign::Zero || known_sign == difference_sign {
                        sides[unknown_index] =
                            Classification::Decided(LineSide::from_real_sign(difference_sign));
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-chord-pair",
                            "parallel-monotone-endpoint-order",
                        );
                    }
                }
                if matches!(
                    sides,
                    [
                        Classification::Decided(LineSide::Left),
                        Classification::Decided(LineSide::Left)
                    ] | [
                        Classification::Decided(LineSide::Right),
                        Classification::Decided(LineSide::Right)
                    ]
                ) {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair",
                        "parallel-monotone-one-sided",
                    );
                    return Ok(RegionPairResult::empty());
                }
                // A strictly monotone incidence vanishes at most once. With
                // one endpoint exactly on the support and the other strictly
                // off it, that endpoint is the only possible contact; it is a
                // contact exactly when it also lies on the finite chord.
                let on_endpoint = match sides {
                    [
                        Classification::Decided(LineSide::On),
                        Classification::Decided(LineSide::Left | LineSide::Right),
                    ] => Some(retained_range.start()),
                    [
                        Classification::Decided(LineSide::Left | LineSide::Right),
                        Classification::Decided(LineSide::On),
                    ] => Some(retained_range.end()),
                    _ => None,
                };
                if let Some(parameter) = on_endpoint
                    && let Classification::Decided(point) = parallel
                        .point_evidence_on_regular_range(
                            parameter,
                            &retained_range,
                            &self.data.policy,
                        )
                        .map_err(|cause| self.invalid(parallel_index, cause))?
                    && let Classification::Decided(chord_parameter) = chord
                        .parameter_at_certified_point(point.clone(), &self.data.policy)
                        .map_err(|cause| self.invalid(chord_index, cause))?
                {
                    let Some(chord_parameter) = chord_parameter else {
                        return Ok(RegionPairResult::empty());
                    };
                    let chord_parameter = CurveParameter2::from_algebraic_chord(chord_parameter);
                    let (first_parameter, second_parameter) =
                        if chord_index == pair.first_carrier_index {
                            (chord_parameter, parameter.clone())
                        } else {
                            (parameter.clone(), chord_parameter)
                        };
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair",
                        "parallel-monotone-endpoint-incidence",
                    );
                    return Ok(RegionPairResult {
                        contacts: vec![RegionPairContactEvidence::direct(
                            first_parameter,
                            second_parameter,
                            Some(point),
                            false,
                            None,
                        )],
                        overlaps: Vec::new(),
                        blockers: Vec::new(),
                    });
                }
                if let [
                    Classification::Decided(first @ (LineSide::Left | LineSide::Right)),
                    Classification::Decided(second @ (LineSide::Left | LineSide::Right)),
                ] = sides
                    && first != second
                {
                    match chord
                        .retained_monotone_parallel_contact_on_region_range(
                            parallel,
                            &retained_range,
                            [first, second],
                            monotonic_sign,
                            &self.data.policy,
                        )
                        .map_err(|cause| self.invalid(parallel_index, cause))?
                    {
                        Classification::Decided(Some(contact)) => {
                            return Ok(retained_monotone_contact_result(contact));
                        }
                        Classification::Decided(None) => {
                            return Ok(RegionPairResult::empty());
                        }
                        Classification::Uncertain(_) => {}
                    }
                }
            }
        }
        // Carrier representation is structural. Probe it under STRICT so
        // APPROXIMATE_512 remains terminal evidence rather than dispatch. A
        // source-stationary line can have a different exact rational component
        // on each regular side; that branch component preserves the authored
        // analytic parameter and therefore enters the same rational overlap
        // authority as an ordinary PH carrier.
        if let Classification::Decided(Some(rational)) = parallel
            .exact_rational_parallel_component_on_regular_range(
                &retained_range,
                &CurveContext::STRICT,
            )
            .map_err(|cause| self.invalid(parallel_index, cause))?
        {
            let shared_source_parameter = self
                .authored_carrier_shared_endpoints(
                    pair.first_carrier_index,
                    pair.second_carrier_index,
                )
                .map(|(first_at_start, second_at_start)| {
                    let parallel_at_start = if parallel_index == pair.first_carrier_index {
                        first_at_start
                    } else {
                        second_at_start
                    };
                    if parallel_at_start {
                        carrier_traversal_start(parallel_carrier)
                    } else {
                        carrier_traversal_end(parallel_carrier)
                    }
                });
            if let Some(result) = self.algebraic_chord_rational_pair_result(
                pair,
                chord,
                chord_index,
                rational.curve(),
                Some(&rational),
                shared_source_parameter,
            )? {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "analytic-parallel-strict-rational-component",
                );
                if result.blockers.is_empty() {
                    return Ok(result);
                }
                // A STRICT rational component is a representation fast path,
                // not a completeness boundary. Selected endpoint fields can
                // make its general rational replay inconclusive even though
                // the retained analytic support decides the same carrier.
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "rational-component-fallback",
                );
            }
        }
        let support_result = match authoritative_support_result.take() {
            Some(result) => result,
            None => self.algebraic_chord_parallel_support_pair_result(
                pair,
                chord,
                chord_index,
                parallel,
                parallel_index,
            )?,
        };
        if support_result.blockers.is_empty() {
            return Ok(support_result);
        }
        {
            let blocker = |reason| RegionPairResult {
                contacts: Vec::new(),
                overlaps: Vec::new(),
                blockers: vec![RegionPairBlocker::Uncertain(reason)],
            };
            let retained = chord
                .parallel_intersections_on_regular_range(
                    parallel,
                    &retained_range,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(parallel_index, cause))?;
            let contacts = match retained {
                Classification::Decided(BezierAlgebraicChordParallelIntersections2::Contacts(
                    contacts,
                )) => Some(contacts),
                Classification::Decided(
                    BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent {
                        ..
                    }
                    | BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                ) if chord.exact_line().is_none() => {
                    return Ok(blocker(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) if chord.exact_line().is_none() => {
                    return Ok(blocker(reason));
                }
                Classification::Decided(
                    BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent {
                        ..
                    }
                    | BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                )
                | Classification::Uncertain(_) => None,
            };
            if let Some(contacts) = contacts {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "analytic-parallel-retained-support",
                );
                return Ok(retained_contact_result(contacts));
            }
        }
        let Some(chord_line) = chord.exact_line() else {
            unreachable!("the retained-support path owns non-represented chords");
        };
        let line_curve =
            BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(chord_line.clone()));
        let chord_is_first = chord_index == pair.first_carrier_index;
        let parallel_is_first = parallel_index == pair.first_carrier_index;
        let blocker = |reason| RegionPairResult {
            contacts: Vec::new(),
            overlaps: Vec::new(),
            blockers: vec![RegionPairBlocker::Uncertain(reason)],
        };
        let chord_parameter = |point: CurvePoint2| match chord
            .parameter_at_certified_point(point, &self.data.policy)
            .map_err(|cause| self.invalid(chord_index, cause))?
        {
            Classification::Decided(Some(parameter)) => Ok(Classification::Decided(
                CurveParameter2::from_algebraic_chord(parameter),
            )),
            Classification::Decided(None) => Err(self.invalid(
                chord_index,
                CurveError::Topology(
                    "an analytic-parallel contact was outside its finite chord".into(),
                ),
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        };

        // Preserve the cheaper univariate supporting-line route whenever all
        // retained contacts have directly represented parallel parameters.
        let regular_range = parallel_carrier
            .start
            .as_bezier_parameter()
            .zip(parallel_carrier.end.as_bezier_parameter())
            .map(|_| {
                CurveParameterRange2::new_validated(
                    parallel_carrier.start.clone(),
                    parallel_carrier.end.clone(),
                )
            });
        match self.parallel_line_pair_result(
            pair,
            parallel,
            parallel_index,
            &line_curve,
            parallel_is_first,
            regular_range.as_ref(),
        )? {
            Classification::Decided(Some(mut result)) => {
                for contact in &mut result.contacts {
                    let Some(point) = contact.point.clone() else {
                        return Err(self.invalid(
                            parallel_index,
                            CurveError::Topology(
                                "a direct parallel/line contact lost its exact point evidence"
                                    .into(),
                            ),
                        ));
                    };
                    let parameter = match chord_parameter(point)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => return Ok(blocker(reason)),
                    };
                    if chord_is_first {
                        contact.first_parameter = parameter;
                    } else {
                        contact.second_parameter = parameter;
                    }
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "analytic-parallel-line",
                );
                return Ok(result);
            }
            Classification::Decided(None) | Classification::Uncertain(_) => {}
        }

        let rational_line = RationalBezier2::try_from_subcurve(&line_curve)
            .map_err(|cause| self.invalid(chord_index, cause))?;
        let intersections = match parallel
            .intersections(&rational_line, &self.data.policy)
            .map_err(|cause| self.invalid(parallel_index, cause))?
        {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => return Ok(blocker(reason)),
        };
        let mut contacts = Vec::with_capacity(intersections.contacts().len());
        for contact in intersections.contacts() {
            let chord_parameter = match chord_parameter(contact.point().clone())? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(blocker(reason)),
            };
            let parallel_parameter = CurveParameter2::from(contact.parallel_parameter().clone());
            let tangent_cross_sign = contact
                .tangent_cross_sign()
                .map(|sign| orient_tangent_cross_sign(sign, parallel_is_first));
            let (first_parameter, second_parameter) = if chord_is_first {
                (chord_parameter, parallel_parameter)
            } else {
                (parallel_parameter, chord_parameter)
            };
            contacts.push(RegionPairContactEvidence::direct(
                first_parameter,
                second_parameter,
                Some(contact.point().clone()),
                contact.is_certified_transverse(),
                tangent_cross_sign,
            ));
        }
        let mut overlaps = Vec::with_capacity(intersections.overlaps().len());
        for overlap in intersections.overlaps() {
            let chord_endpoint = |parameter: &BezierParameter2| {
                let point = match exact_contact_point_evidence(
                    &rational_line,
                    parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(chord_index, cause))?
                {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                chord_parameter(point)
            };
            let chord_start = match chord_endpoint(overlap.second_range().start())? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(blocker(reason)),
            };
            let chord_end = match chord_endpoint(overlap.second_range().end())? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(blocker(reason)),
            };
            let chord_range = CurveParameterRange2::new_validated(chord_start, chord_end);
            let parallel_range =
                CurveParameterRange2::from_bezier_range(overlap.first_range().clone());
            let image = self
                .overlap_rational_image(parallel_index)?
                .ok_or_else(|| self.blocked(parallel_index, UncertaintyReason::Unsupported))?;
            let correspondence = CurveOverlapCorrespondence2::ChordRational {
                source: Arc::new(BezierAlgebraicChordRationalOverlap2::from_certified_ranges(
                    chord.clone(),
                    image,
                    [chord_range.start(), chord_range.end()].map(|p| {
                        p.as_algebraic_chord()
                            .expect("certified chord range")
                            .clone()
                    }),
                    parallel_range.clone(),
                    overlap.orientation(),
                )),
                chord_first: chord_is_first,
            };
            let (first_range, second_range) = if chord_is_first {
                (chord_range, parallel_range)
            } else {
                (parallel_range, chord_range)
            };
            overlaps.push(CurveIntersectionOverlap2 {
                first_span_index: 0,
                second_span_index: 0,
                endpoint_inclusion: [true, true],
                first_range,
                second_range,
                orientation: overlap.orientation(),
                parameter_correspondence: correspondence,
            });
        }
        let mut blockers = Vec::with_capacity(2);
        if !intersections.parameter_components().is_empty() {
            blockers.push(RegionPairBlocker::PointImageParameterComponent);
        }
        if !intersections.is_complete() {
            blockers.push(RegionPairBlocker::IncompleteReplay);
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-pair",
            "analytic-parallel-general",
        );
        Ok(RegionPairResult {
            contacts,
            overlaps,
            blockers,
        })
    }

    pub(super) fn algebraic_chord_parallel_support_pair_result(
        &self,
        pair: &RegionCarrierPair,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        parallel: &BezierParallel2,
        parallel_index: usize,
    ) -> ExactCurveResult<RegionPairResult> {
        let blocker = |reason| RegionPairResult {
            contacts: Vec::new(),
            overlaps: Vec::new(),
            blockers: vec![RegionPairBlocker::Uncertain(reason)],
        };
        let parallel_carrier = &self.data.carriers[parallel_index];
        let regular_range = CurveParameterRange2::new_validated(
            parallel_carrier.start.clone(),
            parallel_carrier.end.clone(),
        );
        let Some(support_line) = chord
            .exact_line()
            .or_else(|| chord.strict_provenance_support_line(&self.data.policy))
        else {
            return Ok(blocker(UncertaintyReason::Unsupported));
        };
        let mut authored_direction = None;
        for contact in chord.parallel_tangent_contacts() {
            let tangent = match contact
                .parallel()
                .source_tangent_at(contact.parameter(), &self.data.policy)
                .map_err(|cause| self.invalid(chord_index, cause))?
            {
                Classification::Decided(tangent) => tangent,
                Classification::Uncertain(_) => continue,
            };
            authored_direction = Some(if contact.parallel_fragment_reversed() {
                (-tangent.0, -tangent.1)
            } else {
                tangent
            });
            break;
        }
        let (direction_x, direction_y) = if let Some(direction) =
            authored_direction.or_else(|| chord.certified_unit_tangent())
        {
            direction
        } else {
            // `strict_provenance_support_line` preserves the chord's
            // traversal orientation. Its exact nonzero delta is therefore
            // the direction authority for a canonicalized procedural
            // bevel even when no separately normalized unit tangent was
            // retained on the chord.
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair",
                "provenance-support-direction",
            );
            support_line.delta()
        };
        let directed_line = LineSeg2::try_new(
            support_line.start().clone(),
            support_line
                .start()
                .translated(direction_x.clone(), direction_y.clone()),
        )
        .map_err(|cause| self.invalid(chord_index, cause))?;
        let authored_contact = self.authored_parallel_support_contact(
            pair,
            parallel,
            parallel_index,
            &direction_x,
            &direction_y,
            Some(&regular_range),
        )?;
        let authored_crossing = authored_contact.as_ref().and_then(|(parameter, cross)| {
            let direction = match cross {
                RealSign::Positive => BezierLineCrossingDirection::NegativeToPositive,
                RealSign::Negative => BezierLineCrossingDirection::PositiveToNegative,
                RealSign::Zero => return None,
            };
            Some((parameter, direction))
        });
        let certified_tangent_contacts = chord
            .parallel_tangent_contacts()
            .iter()
            .filter(|contact| contact.parallel() == parallel)
            .collect::<Vec<_>>();
        let mut certified_tangent_parameters = certified_tangent_contacts
            .iter()
            .map(|contact| contact.parameter().clone())
            .collect::<Vec<_>>();
        if let Some((parameter, RealSign::Zero)) = &authored_contact
            && !certified_tangent_parameters.contains(parameter)
        {
            certified_tangent_parameters.push(parameter.clone());
        }
        let relation_on_retained_range = |deep_branch_refinement| {
            parallel.relation_to_supporting_line_on_regular_range_with_certified_contacts(
                &directed_line,
                &regular_range,
                authored_crossing,
                &certified_tangent_parameters,
                deep_branch_refinement,
                &self.data.policy,
            )
        };
        let relation = match relation_on_retained_range(false)
            .map_err(|cause| self.invalid(parallel_index, cause))?
        {
            Classification::Decided(relation) => relation,
            Classification::Uncertain(reason) => {
                if reason != UncertaintyReason::RealSign {
                    return Ok(blocker(reason));
                }
                let unsigned = match parallel
                    .supporting_line_squared_incidence(
                        &directed_line,
                        &regular_range,
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(parallel_index, cause))?
                {
                    Classification::Decided(crate::BezierParallelIncidence2::Parameters(
                        parameters,
                    )) => parameters,
                    Classification::Decided(crate::BezierParallelIncidence2::EntireCurve)
                    | Classification::Uncertain(_) => return Ok(blocker(reason)),
                };
                let mut finite_candidate = false;
                for parameter in unsigned {
                    let mut disjoint = false;
                    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256] {
                        let (
                            Classification::Decided(point_bounds),
                            Classification::Decided(chord_bounds),
                        ) = (
                            parallel.point_bounds_at_parameter(
                                &parameter,
                                refinement_steps,
                                &self.data.policy,
                            ),
                            chord
                                .conservative_bounds_refined(refinement_steps, &self.data.policy)
                                .map_err(|cause| self.invalid(chord_index, cause))?,
                        )
                        else {
                            continue;
                        };
                        if point_bounds.overlaps(&chord_bounds, &self.data.policy)
                            == Classification::Decided(false)
                        {
                            disjoint = true;
                            break;
                        }
                    }
                    if !disjoint {
                        let selected_point =
                            crate::CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new(
                                parallel.clone(),
                                parameter.clone(),
                                &self.data.policy,
                            ));
                        let selected_side = chord
                            .strict_oriented_side_by_fast_refinement(
                                &selected_point,
                                &self.data.policy,
                            )
                            .map_err(|cause| self.invalid(chord_index, cause))?;
                        if matches!(
                            selected_side,
                            Classification::Decided(
                                crate::classify::LineSide::Left | crate::classify::LineSide::Right
                            )
                        ) {
                            disjoint = true;
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-pair",
                                "opposite-parallel-branch-by-retained-side",
                            );
                        }
                    }
                    if !disjoint {
                        finite_candidate = true;
                    }
                }
                if !finite_candidate {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair",
                        "unsigned-support-candidates-outside-chord",
                    );
                    return Ok(RegionPairResult::empty());
                }
                match relation_on_retained_range(true)
                    .map_err(|cause| self.invalid(parallel_index, cause))?
                {
                    Classification::Decided(relation) => relation,
                    Classification::Uncertain(reason) => return Ok(blocker(reason)),
                }
            }
        };
        let line_contacts = match relation {
            BezierLineContactRelation::ControlHullDisjoint { .. }
            | BezierLineContactRelation::NoContact => return Ok(RegionPairResult::empty()),
            BezierLineContactRelation::OnSupportingLine => {
                return Ok(blocker(UncertaintyReason::Boundary));
            }
            BezierLineContactRelation::Contacts { contacts } => contacts,
        };
        let chord_is_first = chord_index == pair.first_carrier_index;
        let mut contacts = Vec::with_capacity(line_contacts.len());
        for contact in line_contacts {
            if contact.parameter().scalar().is_some_and(|parameter| {
                authored_contact
                    .as_ref()
                    .is_some_and(|(authored, _)| parameter == authored)
            }) {
                continue;
            }
            let certified = contact.parameter().scalar().and_then(|parameter| {
                certified_tangent_contacts
                    .iter()
                    .find(|certified| certified.parameter() == parameter)
            });
            let (point, chord_parameter) = if let Some(certified) = certified {
                match certified.line_endpoint() {
                    BezierEndpoint::Start => (chord.start().clone(), chord.start_parameter()),
                    BezierEndpoint::End => (chord.end().clone(), chord.end_parameter()),
                }
            } else {
                let point = match parallel
                    .point_evidence_on_regular_range(
                        &contact.parameter().clone().into(),
                        &regular_range,
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(parallel_index, cause))?
                {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => return Ok(blocker(reason)),
                };
                let chord_parameter = match chord
                    .parameter_at_certified_point(point.clone(), &self.data.policy)
                    .map_err(|cause| self.invalid(chord_index, cause))?
                {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(blocker(reason));
                    }
                };
                (point, chord_parameter)
            };
            let chord_cross_parallel = match contact.crossing_direction() {
                Some(BezierLineCrossingDirection::NegativeToPositive) => RealSign::Positive,
                Some(BezierLineCrossingDirection::PositiveToNegative) => RealSign::Negative,
                None => RealSign::Zero,
            };
            let tangent_cross_sign =
                orient_tangent_cross_sign(chord_cross_parallel, chord_is_first);
            let tangent_topology = if let Some(parallel_side_of_chord) = contact.tangent_side() {
                let tangent_relation = parallel
                    .vector_tangent_cross_and_dot_signs_on_regular_range(
                        &contact.parameter().clone().into(),
                        &direction_x,
                        &direction_y,
                        &regular_range,
                        &self.data.policy,
                    );
                let (cross, dot) =
                    match tangent_relation.map_err(|cause| self.invalid(parallel_index, cause))? {
                        Classification::Decided(signs) => signs,
                        Classification::Uncertain(reason) => return Ok(blocker(reason)),
                    };
                if cross != RealSign::Zero || dot == RealSign::Zero {
                    return Err(self.invalid(
                        parallel_index,
                        CurveError::Topology(
                            "supporting-line tangency disagreed with its tangent relation".into(),
                        ),
                    ));
                }
                let opposite = |side| match side {
                    LineSide::Left => LineSide::Right,
                    LineSide::Right => LineSide::Left,
                    LineSide::On => unreachable!("a tangent neighbor side is strict"),
                };
                let second_side_of_first = if chord_is_first {
                    parallel_side_of_chord
                } else if dot == RealSign::Positive {
                    opposite(parallel_side_of_chord)
                } else {
                    parallel_side_of_chord
                };
                Some((dot, second_side_of_first))
            } else {
                None
            };
            let chord_parameter = CurveParameter2::from_algebraic_chord(chord_parameter);
            let parallel_parameter = CurveParameter2::from(contact.parameter().clone());
            let (first_parameter, second_parameter) = if chord_is_first {
                (chord_parameter, parallel_parameter)
            } else {
                (parallel_parameter, chord_parameter)
            };
            let evidence = RegionPairContactEvidence::direct(
                first_parameter,
                second_parameter,
                Some(point),
                tangent_cross_sign != RealSign::Zero,
                Some(tangent_cross_sign),
            );
            contacts.push(match tangent_topology {
                Some((dot, side)) => evidence.with_tangent_topology(dot, side),
                None => evidence,
            });
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-pair",
            "analytic-parallel-certified-support",
        );
        Ok(RegionPairResult {
            contacts,
            overlaps: Vec::new(),
            blockers: Vec::new(),
        })
    }

    pub(super) fn algebraic_cusp_rational_pair_result(
        &self,
        pair: &RegionCarrierPair,
        cusp: &crate::BezierAlgebraicCuspSemicircleFragment2,
        rational: &RationalBezier2,
        cusp_is_first: bool,
    ) -> ExactCurveResult<RegionPairResult> {
        let other = &self.data.carriers[if cusp_is_first {
            pair.second_carrier_index
        } else {
            pair.first_carrier_index
        }];
        let range = CurveParameterRange2::new_validated(other.start.clone(), other.end.clone());
        let (intersections, parameter_map) = match cusp
            .semicircle()
            .rational_intersections_with_parameter_map(rational, &range, &self.data.policy)
            .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
        {
            Classification::Decided(result) => result,
            Classification::Uncertain(reason) => {
                return Ok(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                    blockers: vec![RegionPairBlocker::Uncertain(reason)],
                });
            }
        };
        match intersections {
            BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps } => {
                let mut retained = Vec::with_capacity(contacts.len());
                for contact in contacts {
                    let cusp_parameter =
                        contact.location.endpoint_parameter().unwrap_or_else(|| {
                            parameter_map
                                .as_ref()
                                .expect(
                                    "an interior cusp/rational contact retains its parameter map",
                                )
                                .contact_parameter(&contact)
                        });
                    let tangent_cross_sign =
                        orient_tangent_cross_sign(contact.tangent_cross_sign, cusp_is_first);
                    let (first_parameter, second_parameter) = if cusp_is_first {
                        (
                            CurveParameter2::from_algebraic_cusp(cusp_parameter),
                            contact.other_parameter,
                        )
                    } else {
                        (
                            contact.other_parameter,
                            CurveParameter2::from_algebraic_cusp(cusp_parameter),
                        )
                    };
                    retained.push(RegionPairContactEvidence::direct(
                        first_parameter,
                        second_parameter,
                        Some(contact.point),
                        tangent_cross_sign != RealSign::Zero,
                        Some(tangent_cross_sign),
                    ));
                }
                Ok(RegionPairResult {
                    contacts: retained,
                    overlaps: overlaps
                        .into_iter()
                        .map(|source| {
                            circle_overlap_evidence(
                                CurveCircleOverlap2::Mapped(source),
                                cusp_is_first,
                            )
                        })
                        .collect(),
                    blockers: Vec::new(),
                })
            }
            BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber {
                contacts,
                overlaps,
            } => Ok(selected_fiber_cusp_result(
                contacts,
                overlaps,
                cusp_is_first,
            )),
            BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection => {
                Ok(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                    blockers: vec![RegionPairBlocker::Uncertain(UncertaintyReason::Unsupported)],
                })
            }
        }
    }

    pub(super) fn retained_cusp_chord_pair_result(
        &self,
        cusp: &crate::BezierAlgebraicCuspSemicircleFragment2,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        cusp_is_first: bool,
        contacts: Vec<crate::bezier_offset::BezierAlgebraicCuspSemicircleRetainedChordContact2>,
    ) -> ExactCurveResult<RegionPairResult> {
        let mut retained = Vec::with_capacity(contacts.len());
        for contact in contacts {
            let tangent_cross_sign =
                orient_tangent_cross_sign(contact.tangent_cross_sign, cusp_is_first);
            let tangent_topology = if contact.tangent_cross_sign == RealSign::Zero {
                match contact
                    .tangent_topology(cusp.semicircle(), chord, &self.data.policy)
                    .map_err(|cause| self.invalid(chord_index, cause))?
                {
                    Classification::Decided(Some((dot, circle_side_of_chord))) => {
                        let opposite = |side| match side {
                            LineSide::Left => LineSide::Right,
                            LineSide::Right => LineSide::Left,
                            LineSide::On => LineSide::On,
                        };
                        let second_side_of_first = if cusp_is_first {
                            if dot == RealSign::Positive {
                                opposite(circle_side_of_chord)
                            } else {
                                circle_side_of_chord
                            }
                        } else {
                            circle_side_of_chord
                        };
                        (second_side_of_first != LineSide::On)
                            .then_some((dot, second_side_of_first))
                    }
                    Classification::Decided(None) | Classification::Uncertain(_) => None,
                }
            } else {
                None
            };
            let chord_parameter = CurveParameter2::from_algebraic_chord(contact.chord_parameter);
            let (first_parameter, second_parameter) = if cusp_is_first {
                (
                    CurveParameter2::from_algebraic_cusp(contact.cusp_parameter),
                    chord_parameter,
                )
            } else {
                (
                    chord_parameter,
                    CurveParameter2::from_algebraic_cusp(contact.cusp_parameter),
                )
            };
            let evidence = RegionPairContactEvidence::direct(
                first_parameter,
                second_parameter,
                Some(contact.point),
                tangent_cross_sign != RealSign::Zero,
                Some(tangent_cross_sign),
            );
            retained.push(match tangent_topology {
                Some((dot, side)) => evidence.with_tangent_topology(dot, side),
                None => evidence,
            });
        }
        Ok(RegionPairResult {
            contacts: retained,
            overlaps: Vec::new(),
            blockers: Vec::new(),
        })
    }

    pub(super) fn overlap_rational_image(
        &self,
        index: usize,
    ) -> ExactCurveResult<Option<RationalBezier2>> {
        let carrier = &self.data.carriers[index];
        let image = match &carrier.geometry {
            CurveSupport2::Parallel(parallel) => parallel
                .exact_rational_parallel_component_on_regular_range(
                    &carrier.range(),
                    &self.data.policy.strict_counterpart(),
                )
                .map(|result| result.map(|image| image.map(|image| image.curve().clone()))),
            _ => carrier.geometry.exact_rational_component(&self.data.policy),
        }
        .map_err(|cause| self.invalid(index, cause))?;
        match image {
            Classification::Decided(image) => Ok(image),
            Classification::Uncertain(reason) => Err(self.blocked(index, reason)),
        }
    }

    pub(super) fn analytic_component_overlaps(
        &self,
        pair: &RegionCarrierPair,
        components: &[BezierParameterComponentOverlap2],
        overlap: &RationalBezierIntersectionOverlap2,
        swapped: bool,
    ) -> ExactCurveResult<Vec<CurveIntersectionOverlap2>> {
        let (first_range, second_range) = if swapped {
            (overlap.second_range(), overlap.first_range())
        } else {
            (overlap.first_range(), overlap.second_range())
        };
        let mut sources = components
            .iter()
            .filter(|source| source.overlap() == overlap)
            .cloned()
            .map(|source| CurveOverlapCorrespondence2::ParameterComponent { source, swapped })
            .collect::<Vec<_>>();
        if sources.is_empty() {
            let first = self.overlap_rational_image(pair.first_carrier_index)?;
            let second = self.overlap_rational_image(pair.second_carrier_index)?;
            let (first, second) = match (first, second) {
                (Some(first), Some(second)) => (first, second),
                _ => {
                    // The analytic pair kernel emits raw overlaps only for
                    // source-image correspondences in unchanged source charts.
                    // Selected nonlinear image components carry their own map.
                    let (CurveSupport2::Parallel(first), CurveSupport2::Parallel(second)) = (
                        &self.data.carriers[pair.first_carrier_index].geometry,
                        &self.data.carriers[pair.second_carrier_index].geometry,
                    ) else {
                        return Err(
                            self.blocked(pair.first_carrier_index, UncertaintyReason::Unsupported)
                        );
                    };
                    (
                        first
                            .source()
                            .to_rational_bezier()
                            .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?,
                        second
                            .source()
                            .to_rational_bezier()
                            .map_err(|cause| self.invalid(pair.second_carrier_index, cause))?,
                    )
                }
            };
            sources.push(CurveOverlapCorrespondence2::for_rational_ranges(
                &first,
                &second,
                first_range,
                second_range,
                overlap.orientation(),
                &self.data.policy,
            ));
        }
        Ok(sources
            .into_iter()
            .map(|source| CurveIntersectionOverlap2 {
                first_span_index: 0,
                second_span_index: 0,
                first_range: CurveParameterRange2::from_bezier_range(first_range.clone()),
                second_range: CurveParameterRange2::from_bezier_range(second_range.clone()),
                orientation: overlap.orientation(),
                endpoint_inclusion: [overlap.includes_start(), overlap.includes_end()],
                parameter_correspondence: source,
            })
            .collect())
    }

    /// Publishes the identity overlap of two carriers on one analytic
    /// parallel whose parameter ranges are nested.
    ///
    /// On an injective range, distinct parameters have distinct points, so a
    /// nested range meets the containing one exactly on itself, with no other
    /// contact. A binary Boolean's carriers are fragments of regularized,
    /// noncrossing operand boundaries and are therefore injective; other
    /// contexts need the containing carrier's certified injective image.
    /// This avoids saturating the identity component out of the parallel's
    /// self-intersection system. Partially overlapping ranges decline.
    pub(super) fn nested_same_parallel_overlap(
        &self,
        pair: &RegionCarrierPair,
    ) -> ExactCurveResult<Option<RationalBezierIntersectionOverlap2>> {
        let first = &self.data.carriers[pair.first_carrier_index];
        let second = &self.data.carriers[pair.second_carrier_index];
        let (Some(first_start), Some(first_end), Some(second_start), Some(second_end)) = (
            first.start.as_bezier_parameter(),
            first.end.as_bezier_parameter(),
            second.start.as_bezier_parameter(),
            second.end.as_bezier_parameter(),
        ) else {
            return Ok(None);
        };
        let strict = self.data.policy.strict_counterpart();
        let not_after = |left: &CurveParameter2, right: &CurveParameter2| match left
            .cmp_by_refinement(right, &strict)
            .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
        {
            Classification::Decided(order) => Ok(Some(order != std::cmp::Ordering::Greater)),
            Classification::Uncertain(_) => Ok(None),
        };
        let contains = |outer: &RegionCarrier, inner: &RegionCarrier| {
            Ok::<_, ExactCurveError>(
                not_after(&outer.start, &inner.start)? == Some(true)
                    && not_after(&inner.end, &outer.end)? == Some(true),
            )
        };
        let (outer, inner_start, inner_end) = if contains(first, second)? {
            (first, second_start, second_end)
        } else if contains(second, first)? {
            (second, first_start, first_end)
        } else {
            return Ok(None);
        };
        let structurally_injective =
            self.data.regularization_fill_rule.is_none() && first.operand != second.operand;
        if !structurally_injective && !carrier_has_certified_injective_image(outer, &strict) {
            return Ok(None);
        }
        Ok(Some(
            RationalBezierIntersectionOverlap2::from_certified_parameters(
                inner_start.clone(),
                inner_end.clone(),
                inner_start.clone(),
                inner_end.clone(),
                CurveOverlapOrientation2::Same,
                [true, true],
            ),
        ))
    }

    pub(super) fn pair_result(
        &self,
        pair: &RegionCarrierPair,
    ) -> ExactCurveResult<RegionPairResult> {
        let first = &self.data.carriers[pair.first_carrier_index];
        let second = &self.data.carriers[pair.second_carrier_index];
        match &pair.context {
            RegionCarrierPairContext::Common(context) => {
                let result = context.result_view()?;
                Ok(RegionPairResult {
                    contacts: result
                        .contacts()
                        .iter()
                        .map(RegionPairContactEvidence::from_intersection)
                        .collect(),
                    overlaps: result.overlaps().to_vec(),
                    blockers: result
                        .blockers()
                        .iter()
                        .cloned()
                        .map(RegionPairBlocker::Common)
                        .chain(
                            (!result.parameter_components().is_empty())
                                .then_some(RegionPairBlocker::PointImageParameterComponent),
                        )
                        .collect(),
                })
            }
            RegionCarrierPairContext::ParallelRational { parallel_is_first } => {
                let (parallel_carrier, parallel, parallel_index, curve) = if *parallel_is_first {
                    (
                        first,
                        first.geometry.parallel(),
                        pair.first_carrier_index,
                        second.geometry.bezier(),
                    )
                } else {
                    (
                        second,
                        second.geometry.parallel(),
                        pair.second_carrier_index,
                        first.geometry.bezier(),
                    )
                };
                let regular_range = CurveParameterRange2::new_validated(
                    parallel_carrier.start.clone(),
                    parallel_carrier.end.clone(),
                );
                // Circle incidence proves source regularity on this range.
                // Retained endpoint storage is not a singularity certificate:
                // ordinary selected cuts use the same conic inverse, while
                // source-cusp limits continue through the regularized kernel.
                match self.parallel_arc_pair_result(
                    parallel,
                    &regular_range,
                    curve,
                    *parallel_is_first,
                )? {
                    Classification::Decided(Some(result)) => return Ok(result),
                    Classification::Decided(None) | Classification::Uncertain(_) => {}
                }
                let line = self.parallel_line_pair_result(
                    pair,
                    parallel,
                    parallel_index,
                    curve,
                    *parallel_is_first,
                    Some(&regular_range),
                )?;
                match line {
                    Classification::Decided(Some(result)) => return Ok(result),
                    Classification::Decided(None) | Classification::Uncertain(_) => {}
                }
                let rational = RationalBezier2::try_from_subcurve(curve)
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?;
                let intersections = parallel.intersections_on_regular_range(
                    &rational,
                    &regular_range,
                    &self.data.policy,
                );
                let result = match intersections
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                {
                    Classification::Decided(result) => result,
                    Classification::Uncertain(reason) => {
                        return Ok(RegionPairResult {
                            contacts: Vec::new(),
                            overlaps: Vec::new(),
                            blockers: vec![RegionPairBlocker::Uncertain(reason)],
                        });
                    }
                };
                let contacts = result
                    .contacts()
                    .iter()
                    .map(|contact| {
                        let (first_parameter, second_parameter, tangent_cross_sign) =
                            if *parallel_is_first {
                                (
                                    contact.parallel_parameter().clone(),
                                    contact.other_parameter().clone(),
                                    contact.tangent_cross_sign(),
                                )
                            } else {
                                (
                                    contact.other_parameter().clone(),
                                    contact.parallel_parameter().clone(),
                                    contact.tangent_cross_sign().map(|sign| match sign {
                                        RealSign::Positive => RealSign::Negative,
                                        RealSign::Negative => RealSign::Positive,
                                        RealSign::Zero => RealSign::Zero,
                                    }),
                                )
                            };
                        RegionPairContactEvidence::direct_bezier(
                            first_parameter,
                            second_parameter,
                            Some(contact.point().clone()),
                            contact.is_certified_transverse(),
                            tangent_cross_sign,
                        )
                    })
                    .collect();
                let mut overlaps = Vec::new();
                for overlap in result.overlaps() {
                    overlaps.extend(self.analytic_component_overlaps(
                        pair,
                        result.component_overlaps(),
                        overlap,
                        !*parallel_is_first,
                    )?);
                }
                let mut blockers = Vec::with_capacity(2);
                if !result.parameter_components().is_empty() {
                    blockers.push(RegionPairBlocker::PointImageParameterComponent);
                }
                if !result.is_complete() {
                    blockers.push(RegionPairBlocker::IncompleteReplay);
                }
                Ok(RegionPairResult {
                    contacts,
                    overlaps,
                    blockers,
                })
            }
            RegionCarrierPairContext::ParallelPair
            | RegionCarrierPairContext::ParallelSameImage => {
                if self.parallel_pair_is_coordinate_disjoint(pair)
                    || self.adjacent_parallel_pair_is_endpoint_only(pair)
                {
                    // A shared strictly monotone coordinate either separates
                    // the complete images or reduces them to one already
                    // seeded adjacent loop vertex.  Neither case needs a
                    // bivariate resultant.
                    return Ok(RegionPairResult {
                        contacts: Vec::new(),
                        overlaps: Vec::new(),
                        blockers: Vec::new(),
                    });
                }
                if matches!(pair.context, RegionCarrierPairContext::ParallelSameImage)
                    && let Some(overlap) = self.nested_same_parallel_overlap(pair)?
                {
                    return Ok(RegionPairResult {
                        contacts: Vec::new(),
                        overlaps: self.analytic_component_overlaps(pair, &[], &overlap, false)?,
                        blockers: Vec::new(),
                    });
                }
                let parallel = first.geometry.parallel();
                // Identity saturation and residual self contacts share the pair kernel.
                let intersection = parallel.parallel_intersections_on_regular_ranges(
                    second.geometry.parallel(),
                    &first.range(),
                    &second.range(),
                    &self.data.policy,
                );
                let result = match intersection
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                {
                    Classification::Decided(result) => result,
                    Classification::Uncertain(reason) => {
                        return Ok(RegionPairResult {
                            contacts: Vec::new(),
                            overlaps: Vec::new(),
                            blockers: vec![RegionPairBlocker::Uncertain(reason)],
                        });
                    }
                };
                let contacts = result
                    .contacts()
                    .iter()
                    .map(|contact| {
                        RegionPairContactEvidence::direct(
                            contact.first_parameter().clone(),
                            contact.second_parameter().clone(),
                            None,
                            contact.is_certified_transverse(),
                            contact.tangent_cross_sign(),
                        )
                    })
                    .collect();
                let mut overlaps = Vec::new();
                for overlap in result.overlaps() {
                    overlaps.extend(self.analytic_component_overlaps(
                        pair,
                        result.component_overlaps(),
                        overlap,
                        false,
                    )?);
                }
                let mut blockers = Vec::with_capacity(2);
                if !result.parameter_components().is_empty() {
                    blockers.push(RegionPairBlocker::PointImageParameterComponent);
                }
                if !result.is_complete() {
                    blockers.push(RegionPairBlocker::IncompleteReplay);
                }
                Ok(RegionPairResult {
                    contacts,
                    overlaps,
                    blockers,
                })
            }
            RegionCarrierPairContext::CuspChord { cusp_is_first } => {
                {
                    let (cusp, cusp_index, chord, chord_index) = if *cusp_is_first {
                        (
                            first.geometry.circle(),
                            pair.first_carrier_index,
                            match &second.geometry {
                                CurveSupport2::Line(chord) => chord,
                                _ => unreachable!("cusp/chord dispatch retained its chord"),
                            },
                            pair.second_carrier_index,
                        )
                    } else {
                        (
                            second.geometry.circle(),
                            pair.second_carrier_index,
                            match &first.geometry {
                                CurveSupport2::Line(chord) => chord,
                                _ => unreachable!("chord/cusp dispatch retained its chord"),
                            },
                            pair.first_carrier_index,
                        )
                    };
                    let mut certified_chord_endpoint_incidence = None;
                    if let Some((first_at_start, second_at_start)) = self
                        .authored_carrier_shared_endpoints(
                            pair.first_carrier_index,
                            pair.second_carrier_index,
                        )
                    {
                        let cusp_at_start = if *cusp_is_first {
                            first_at_start
                        } else {
                            second_at_start
                        };
                        if cusp.certified_tangent_endpoint(cusp_at_start)
                            && !cusp.selected_chord_normal_contact_endpoint(cusp_at_start)
                        {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-circle-chord-pair",
                                "adjacent-authored-tangent",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        let structural_endpoint_only = cusp
                            .authored_adjacent_chord_is_structurally_endpoint_only(
                                chord,
                                cusp_at_start,
                                &self.data.policy,
                            )
                            .map_err(|cause| self.invalid(chord_index, cause))?;
                        let endpoint_only = structural_endpoint_only
                            || self
                                .data
                                .policy
                                .strict_predicate_pass(|| {
                                    cusp.certified_adjacent_chord_is_endpoint_only(
                                        chord,
                                        cusp_at_start,
                                        &self.data.policy,
                                    )
                                })
                                .map_err(|cause| self.invalid(chord_index, cause))?
                                == Classification::Decided(true);
                        if endpoint_only {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-circle-chord-pair",
                                "authored-adjacent-endpoint-only",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        certified_chord_endpoint_incidence = Some(if *cusp_is_first {
                            second_at_start
                        } else {
                            first_at_start
                        });
                    }
                    if let Some((sibling_index, sibling_at_start, chord_at_start)) = self
                        .authored_supporting_circle_endpoint(
                            cusp_index,
                            chord_index,
                            |sibling, at_start| {
                                sibling.certified_tangent_endpoint(at_start)
                                    && !sibling.selected_chord_normal_contact_endpoint(at_start)
                            },
                        )
                        && certified_chord_endpoint_incidence
                            .is_none_or(|incident| incident == chord_at_start)
                    {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-circle-chord-pair",
                            "supporting-circle-sibling-endpoint-tangent",
                        );
                        let sibling = self.data.carriers[sibling_index].geometry.circle();
                        let mapped = self
                            .data
                            .policy
                            .strict_predicate_pass(|| {
                                cusp.parameter_of_shared_circle_endpoint(
                                    sibling,
                                    sibling_at_start,
                                    &self.data.policy,
                                )
                            })
                            .map_err(|cause| self.invalid(cusp_index, cause))?;
                        match mapped {
                            Classification::Decided(None) => return Ok(RegionPairResult::empty()),
                            Classification::Decided(Some(cusp_parameter)) => {
                                let (chord_parameter, point) = if chord_at_start {
                                    (chord.start_parameter(), chord.start().clone())
                                } else {
                                    (chord.end_parameter(), chord.end().clone())
                                };
                                return self.retained_cusp_chord_pair_result(
                                    cusp,
                                    chord,
                                    chord_index,
                                    *cusp_is_first,
                                    vec![BezierAlgebraicCuspSemicircleRetainedChordContact2 {
                                        cusp_parameter,
                                        chord_parameter,
                                        point,
                                        tangent_cross_sign: RealSign::Zero,
                                    }],
                                );
                            }
                            Classification::Uncertain(_) => {}
                        }
                        // A failed chart comparison does not erase the exact
                        // full-circle incidence or prove absence of a contact.
                        certified_chord_endpoint_incidence = Some(chord_at_start);
                    }
                    if certified_chord_endpoint_incidence.is_none()
                        && let Classification::Decided(Some(contact)) = cusp
                            .certified_chord_endpoint_contact(chord, &self.data.policy)
                            .map_err(|cause| self.invalid(chord_index, cause))?
                    {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-circle-chord-pair",
                            "retained-nonadjacent-endpoint-contact",
                        );
                        return self.retained_cusp_chord_pair_result(
                            cusp,
                            chord,
                            chord_index,
                            *cusp_is_first,
                            vec![contact],
                        );
                    }
                    // Refined bounds are only a rejection accelerator. Keep
                    // their proof budget small and fall through to the exact
                    // circle/chord kernel when the boxes continue to overlap;
                    // policy-terminal refinement belongs in predicates that
                    // can decide the result, not in broad phase replay.
                    for refinement_steps in [0, 2] {
                        let circle_bounds = cusp
                            .semicircle()
                            .conservative_bounds_refined(refinement_steps, &self.data.policy)
                            .map_err(|cause| self.invalid(cusp_index, cause))?;
                        let chord_bounds = chord
                            .conservative_bounds_refined(refinement_steps, &self.data.policy)
                            .map_err(|cause| self.invalid(chord_index, cause))?;
                        let (
                            Classification::Decided(circle_bounds),
                            Classification::Decided(chord_bounds),
                        ) = (circle_bounds, chord_bounds)
                        else {
                            continue;
                        };
                        if circle_bounds.overlaps(&chord_bounds, &self.data.policy)
                            == Classification::Decided(false)
                        {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-circle-chord-pair",
                                "refined-bounds-disjoint",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                    }
                    let intersections = match certified_chord_endpoint_incidence {
                        Some(chord_at_start) => cusp
                            .semicircle()
                            .chord_intersections_with_certified_endpoint_incidence(
                                chord,
                                chord_at_start,
                                &self.data.policy,
                            ),
                        None => cusp
                            .semicircle()
                            .chord_intersections(chord, &self.data.policy),
                    }
                    .map_err(|cause| self.invalid(chord_index, cause))?;
                    let intersections = match intersections {
                        Classification::Decided(intersections) => intersections,
                        Classification::Uncertain(reason) => {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-circle-chord-pair",
                                match reason {
                                    UncertaintyReason::Unsupported => "kernel-unsupported",
                                    UncertaintyReason::Predicate => "kernel-predicate",
                                    UncertaintyReason::Ordering => "kernel-ordering",
                                    UncertaintyReason::RealSign => "kernel-real-sign",
                                    UncertaintyReason::Boundary => "kernel-boundary",
                                },
                            );
                            #[cfg(feature = "dispatch-trace")]
                            if reason == UncertaintyReason::Unsupported {
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "algebraic-circle-chord-kernel-blocker",
                                    if chord.exact_line().is_some() {
                                        "exact-line"
                                    } else if chord.certified_unit_tangent().is_some() {
                                        "certified-tangent"
                                    } else {
                                        "general-retained"
                                    },
                                );
                            }
                            return Ok(RegionPairResult {
                                contacts: Vec::new(),
                                overlaps: Vec::new(),
                                blockers: vec![RegionPairBlocker::Uncertain(reason)],
                            });
                        }
                    };
                    let mut contacts = intersections;
                    if contacts.is_empty() {
                        return Ok(RegionPairResult::empty());
                    }
                    if let Some(chord_at_start) = certified_chord_endpoint_incidence {
                        // Boundary-loop seeding already owns this exact
                        // adjacent vertex. The circle/chord solve was still
                        // required because a line through one circle point can
                        // have a second finite contact; discard only the
                        // structurally identified endpoint and retain every
                        // other root.
                        contacts.retain(|contact| {
                            !contact
                                .chord_parameter
                                .is_endpoint_of(chord, chord_at_start)
                        });
                        if contacts.is_empty() {
                            return Ok(RegionPairResult::empty());
                        }
                    }
                    self.retained_cusp_chord_pair_result(
                        cusp,
                        chord,
                        chord_index,
                        *cusp_is_first,
                        contacts,
                    )
                }
            }
            RegionCarrierPairContext::AlgebraicChordPair { endpoint_contact } => {
                {
                    let (chord, chord_index, other, other_index) =
                        match (&first.geometry, &second.geometry) {
                            (CurveSupport2::Line(chord), other) => (
                                chord,
                                pair.first_carrier_index,
                                other,
                                pair.second_carrier_index,
                            ),
                            (other, CurveSupport2::Line(chord)) => (
                                chord,
                                pair.second_carrier_index,
                                other,
                                pair.first_carrier_index,
                            ),
                            _ => unreachable!("an algebraic-chord pair retains one chord"),
                        };
                    if let Some(contact) = endpoint_contact
                        && let CurveSupport2::Bezier(curve) = other
                    {
                        let parameter = if chord_index == pair.first_carrier_index {
                            &contact.second_parameter
                        } else {
                            &contact.first_parameter
                        };
                        let rational = RationalBezier2::try_from_subcurve(curve)
                            .map_err(|cause| self.invalid(other_index, cause))?;
                        if let Some(mut result) = self.algebraic_chord_rational_pair_result(
                            pair,
                            chord,
                            chord_index,
                            &rational,
                            None,
                            Some(parameter),
                        )? {
                            result.contacts.push((**contact).clone());
                            return Ok(result);
                        }
                    }
                    // Every retained-chord pairing below already owns a
                    // complete finite-domain kernel. Refining composite
                    // endpoints into an optional AABB duplicates those exact
                    // predicates and can expand a large shared scalar DAG
                    // before the authoritative carrier relation is consulted.
                    if let CurveSupport2::Line(other_chord) = other {
                        if self.authored_carriers_are_adjacent(pair) {
                            for (support, candidate) in [(chord, other_chord), (other_chord, chord)]
                            {
                                if support.certified_unit_tangent().is_none() {
                                    continue;
                                }
                                for endpoint in [candidate.start(), candidate.end()] {
                                    if matches!(
                                        support
                                            .certified_tangent_side(endpoint, &self.data.policy,),
                                        Classification::Decided(
                                            crate::classify::LineSide::Left
                                                | crate::classify::LineSide::Right
                                        )
                                    ) {
                                        // One endpoint off the retained line
                                        // proves the adjacent supports are
                                        // noncollinear. Their sole support
                                        // intersection is the authored vertex.
                                        #[cfg(feature = "dispatch-trace")]
                                        hyperreal::dispatch_trace::record(
                                            "hypercurve",
                                            "algebraic-chord-pair",
                                            "adjacent-certified-tangent-complete",
                                        );
                                        return Ok(RegionPairResult::empty());
                                    }
                                }
                            }
                        }
                        if self.authored_carriers_are_adjacent(pair)
                            && let (Some(first_tangent), Some(second_tangent)) = (
                                chord.certified_unit_tangent(),
                                other_chord.certified_unit_tangent(),
                            )
                        {
                            let tangent_cross = &first_tangent.0 * &second_tangent.1
                                - &first_tangent.1 * &second_tangent.0;
                            if matches!(
                                real_sign(&tangent_cross, &self.data.policy),
                                Some(RealSign::Positive | RealSign::Negative)
                            ) {
                                // Nonparallel straight supports meet exactly
                                // once. Authored adjacency already owns that
                                // endpoint, so there is no additional contact
                                // or overlap to add to the arrangement.
                                #[cfg(feature = "dispatch-trace")]
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "algebraic-chord-pair",
                                    "adjacent-certified-nonparallel-complete",
                                );
                                return Ok(RegionPairResult::empty());
                            }
                        }
                        if self.authored_carriers_are_adjacent(pair) {
                            for (axis_chord, candidate) in
                                [(chord, other_chord), (other_chord, chord)]
                            {
                                let Some(direction) = axis_chord.certified_axis_direction() else {
                                    continue;
                                };
                                let constant_axis = match direction.axis() {
                                    Axis2::X => Axis2::Y,
                                    Axis2::Y => Axis2::X,
                                };
                                let mut certified_noncollinear = false;
                                for endpoint in [candidate.start(), candidate.end()] {
                                    match self
                                        .data
                                        .policy
                                        .strict_predicate_pass(|| {
                                            crate::BezierAlgebraicChord2::point_axis_order(
                                                axis_chord.start(),
                                                endpoint,
                                                constant_axis,
                                                &self.data.policy,
                                            )
                                        })
                                        .map_err(|cause| self.invalid(chord_index, cause))?
                                    {
                                        Classification::Decided(
                                            std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
                                        ) => {
                                            certified_noncollinear = true;
                                            break;
                                        }
                                        Classification::Decided(std::cmp::Ordering::Equal)
                                        | Classification::Uncertain(_) => {}
                                    }
                                }
                                if certified_noncollinear {
                                    #[cfg(feature = "dispatch-trace")]
                                    hyperreal::dispatch_trace::record(
                                        "hypercurve",
                                        "algebraic-chord-pair",
                                        "adjacent-axis-noncollinear-complete",
                                    );
                                    return Ok(RegionPairResult::empty());
                                }
                            }
                        }
                        let strictly_one_sided = if let Some(line) = other_chord.exact_line() {
                            self.data
                                .policy
                                .strict_predicate_pass(|| {
                                    chord.is_strictly_one_sided_of_exact_line(
                                        &line,
                                        &self.data.policy,
                                    )
                                })
                                .map_err(|cause| self.invalid(chord_index, cause))?
                        } else if let Some(line) = chord.exact_line() {
                            self.data
                                .policy
                                .strict_predicate_pass(|| {
                                    other_chord.is_strictly_one_sided_of_exact_line(
                                        &line,
                                        &self.data.policy,
                                    )
                                })
                                .map_err(|cause| self.invalid(other_index, cause))?
                        } else {
                            Classification::Decided(false)
                        };
                        if strictly_one_sided == Classification::Decided(true) {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-pair",
                                "exact-line-one-sided",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        let intersections = match chord
                            .chord_intersections(other_chord, &self.data.policy)
                            .map_err(|cause| self.invalid(chord_index, cause))?
                        {
                            Classification::Decided(intersections) => intersections,
                            Classification::Uncertain(reason) => {
                                return Ok(RegionPairResult {
                                    contacts: Vec::new(),
                                    overlaps: Vec::new(),
                                    blockers: vec![RegionPairBlocker::Uncertain(reason)],
                                });
                            }
                        };
                        let (mut contacts, overlaps) = match intersections {
                            BezierAlgebraicChordPairIntersections2::Contacts(contacts) => (
                                contacts
                                    .into_iter()
                                    .map(|contact| {
                                        RegionPairContactEvidence::direct(
                                            CurveParameter2::from_algebraic_chord(
                                                contact.first_parameter().clone(),
                                            ),
                                            CurveParameter2::from_algebraic_chord(
                                                contact.second_parameter().clone(),
                                            ),
                                            Some(contact.point().clone()),
                                            contact.tangent_cross_sign() != RealSign::Zero,
                                            Some(contact.tangent_cross_sign()),
                                        )
                                    })
                                    .collect(),
                                Vec::new(),
                            ),
                            BezierAlgebraicChordPairIntersections2::Overlaps(overlaps) => (
                                Vec::new(),
                                overlaps
                                    .into_iter()
                                    .map(|overlap| {
                                        let [first_start, first_end] = overlap.first_range();
                                        let [second_start, second_end] = overlap.second_range();
                                        CurveIntersectionOverlap2 {
                                            first_span_index: 0,
                                            second_span_index: 0,
                                            endpoint_inclusion: [true, true],
                                            parameter_correspondence:
                                                CurveOverlapCorrespondence2::Chords {
                                                    first: chord.clone(),
                                                    second: other_chord.clone(),
                                                    first_range:
                                                        CurveParameterRange2::new_validated(
                                                            CurveParameter2::from_algebraic_chord(
                                                                first_start.clone(),
                                                            ),
                                                            CurveParameter2::from_algebraic_chord(
                                                                first_end.clone(),
                                                            ),
                                                        ),
                                                    second_range:
                                                        CurveParameterRange2::new_validated(
                                                            CurveParameter2::from_algebraic_chord(
                                                                second_start.clone(),
                                                            ),
                                                            CurveParameter2::from_algebraic_chord(
                                                                second_end.clone(),
                                                            ),
                                                        ),
                                                },
                                            first_range: CurveParameterRange2::new_validated(
                                                CurveParameter2::from_algebraic_chord(
                                                    first_start.clone(),
                                                ),
                                                CurveParameter2::from_algebraic_chord(
                                                    first_end.clone(),
                                                ),
                                            ),
                                            second_range: CurveParameterRange2::new_validated(
                                                CurveParameter2::from_algebraic_chord(
                                                    second_start.clone(),
                                                ),
                                                CurveParameter2::from_algebraic_chord(
                                                    second_end.clone(),
                                                ),
                                            ),
                                            orientation: overlap.orientation(),
                                        }
                                    })
                                    .collect(),
                            ),
                        };
                        if self.authored_carriers_are_adjacent(pair) && overlaps.is_empty() {
                            // Adjacent straight chords have only their already
                            // seeded authored endpoint in common unless they
                            // overlap positively, which remains arrangement
                            // evidence.
                            contacts.clear();
                        }
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-chord-pair",
                            if overlaps.is_empty() {
                                "chord-contact-complete"
                            } else {
                                "chord-overlap-complete"
                            },
                        );
                        return Ok(RegionPairResult {
                            contacts,
                            overlaps,
                            blockers: Vec::new(),
                        });
                    }
                    if let CurveSupport2::Bezier(curve) = other {
                        let other_carrier = &self.data.carriers[other_index];
                        let chord_carrier = &self.data.carriers[chord_index];
                        let authored_adjacent = self.authored_carriers_are_adjacent(pair);
                        let chord_precedes_other = authored_adjacent.then(|| {
                            let boundary = match chord_carrier.operand {
                                CurveRegionBooleanOperand2::First => self.data.first,
                                CurveRegionBooleanOperand2::Second => self.data.second,
                            }
                            .boundary_loops()
                            .get(chord_carrier.loop_index)
                            .expect("an admitted carrier retains its authored loop");
                            chord_carrier.fragment_index.checked_add(1)
                                == Some(other_carrier.fragment_index)
                                || (chord_carrier.fragment_index.checked_add(1)
                                    == Some(boundary.fragments().len())
                                    && other_carrier.fragment_index == 0)
                        });
                        if subcurve_is_strict_line_image(curve)
                            && let (Some(start), Some(end)) = (
                                exact_carrier_point(
                                    other_carrier,
                                    &other_carrier.start,
                                    &self.data.policy,
                                ),
                                exact_carrier_point(
                                    other_carrier,
                                    &other_carrier.end,
                                    &self.data.policy,
                                ),
                            )
                            && let Ok(line) = LineSeg2::try_new(start, end)
                            && chord
                                .is_strictly_one_sided_of_exact_line(&line, &self.data.policy)
                                .map_err(|cause| self.invalid(other_index, cause))?
                                == Classification::Decided(true)
                        {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-pair",
                                "exact-line-one-sided",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        if authored_adjacent
                            && carrier_has_certified_injective_image(
                                other_carrier,
                                &self.data.policy,
                            )
                            && subcurve_is_strict_line_image(curve)
                            && let (Some(start), Some(end)) = (
                                exact_carrier_point(
                                    other_carrier,
                                    &other_carrier.start,
                                    &self.data.policy,
                                ),
                                exact_carrier_point(
                                    other_carrier,
                                    &other_carrier.end,
                                    &self.data.policy,
                                ),
                            )
                            && let Ok(line) = LineSeg2::try_new(start, end)
                        {
                            if let (
                                Classification::Decided(Some(chord_direction)),
                                Some(line_direction),
                            ) = (
                                chord
                                    .axis_direction(&self.data.policy)
                                    .map_err(|cause| self.invalid(chord_index, cause))?,
                                exact_axis_aligned_line_direction(&line),
                            ) && chord_direction.axis() != line_direction.axis()
                            {
                                #[cfg(feature = "dispatch-trace")]
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "algebraic-chord-pair",
                                    "adjacent-perpendicular-line-complete",
                                );
                                return Ok(RegionPairResult::empty());
                            }
                            match self
                                .data
                                .policy
                                .strict_predicate_pass(|| {
                                    chord.has_non_collinear_support_with_exact_line(
                                        &line,
                                        &self.data.policy,
                                    )
                                })
                                .map_err(|cause| self.invalid(other_index, cause))?
                            {
                                Classification::Decided(true) => {
                                    #[cfg(feature = "dispatch-trace")]
                                    hyperreal::dispatch_trace::record(
                                        "hypercurve",
                                        "algebraic-chord-pair",
                                        "adjacent-exact-line-complete",
                                    );
                                    return Ok(RegionPairResult::empty());
                                }
                                Classification::Decided(false) | Classification::Uncertain(_) => {}
                            }
                        }
                        if let Some(result) = self.algebraic_chord_linear_bezier_pair_result(
                            pair,
                            chord,
                            chord_index,
                            curve,
                            other_index,
                        )? {
                            return Ok(result);
                        }
                        if let Some((_, circle)) = retained_circular_support(curve)
                            && chord
                                .certifiably_disjoint_from_circle_bounds(
                                    &circle.center,
                                    &circle.radius_squared,
                                    &self.data.policy,
                                )
                                .map_err(|cause| self.invalid(other_index, cause))?
                                == Classification::Decided(true)
                        {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-pair",
                                "retained-circle-bounds-disjoint",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        if let Some(chord_precedes_other) = chord_precedes_other
                            && adjacent_axis_algebraic_chord_circular_curve_is_endpoint_only(
                                chord,
                                chord_carrier,
                                curve,
                                other_carrier,
                                chord_precedes_other,
                                &self.data.policy,
                            )
                            .map_err(|cause| self.invalid(other_index, cause))?
                                == Classification::Decided(true)
                        {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-pair",
                                "adjacent-circular-endpoint-only",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        let rational = RationalBezier2::try_from_subcurve(curve)
                            .map_err(|cause| self.invalid(other_index, cause))?;
                        // This optional adjacency shortcut must not consume
                        // approximation before the common exact pair kernel.
                        if let Some(result) = self.data.policy.strict_predicate_pass(|| {
                            self.algebraic_chord_shared_image_endpoint_pair_result(
                                pair,
                                chord,
                                chord_index,
                                &rational,
                                other_index,
                            )
                        })? {
                            return Ok(result);
                        }
                        let one_sided = chord
                            .rational_control_hull_is_strictly_one_sided(
                                &rational,
                                &other_carrier.range(),
                                &self.data.policy,
                            )
                            .map_err(|cause| self.invalid(other_index, cause))?;
                        if one_sided == Classification::Decided(true) {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-pair",
                                "rational-control-hull-one-sided",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        let shared_source_parameter =
                            if let Some(chord_precedes_other) = chord_precedes_other {
                                let shared_parameter = if chord_precedes_other {
                                    carrier_traversal_start(other_carrier)
                                } else {
                                    carrier_traversal_end(other_carrier)
                                };
                                Some(shared_parameter)
                            } else {
                                None
                            };
                        if let Some(result) = self.algebraic_chord_rational_pair_result(
                            pair,
                            chord,
                            chord_index,
                            &rational,
                            None,
                            shared_source_parameter,
                        )? {
                            return Ok(result);
                        }
                    }
                    if let CurveSupport2::Parallel(parallel) = other {
                        return self.algebraic_chord_parallel_pair_result(
                            pair,
                            chord,
                            chord_index,
                            parallel,
                            other_index,
                        );
                    }
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "unsupported",
                );
                Ok(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                    blockers: vec![RegionPairBlocker::Uncertain(UncertaintyReason::Unsupported)],
                })
            }
            RegionCarrierPairContext::CuspRational { cusp_is_first } => {
                let (cusp, curve, curve_carrier, curve_index) = if *cusp_is_first {
                    (
                        first.geometry.circle(),
                        second.geometry.bezier(),
                        second,
                        pair.second_carrier_index,
                    )
                } else {
                    (
                        second.geometry.circle(),
                        first.geometry.bezier(),
                        first,
                        pair.first_carrier_index,
                    )
                };
                if let Classification::Decided(bounds) = curve_carrier.bounds.get_or_init(|| {
                    curve_carrier.geometry.certified_outer_bounds(
                        &curve_carrier.range(),
                        0,
                        &self.data.policy,
                    )
                }) && cusp
                    .semicircle()
                    .certifiably_disjoint_from_bounds(bounds, &self.data.policy)
                    .map_err(|cause| self.invalid(curve_index, cause))?
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-rational-pair",
                        "bounds-disjoint",
                    );
                    return Ok(RegionPairResult::empty());
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-rational-pair",
                    match (
                        self.authored_carriers_are_adjacent(pair),
                        subcurve_is_strict_line_image(curve),
                        retained_circular_support(curve).is_some(),
                    ) {
                        (true, true, _) => "adjacent-line",
                        (true, false, true) => "adjacent-circle",
                        (true, false, false) => "adjacent-general",
                        (false, true, _) => "nonadjacent-line",
                        (false, false, true) => "nonadjacent-circle",
                        (false, false, false) => "nonadjacent-general",
                    },
                );
                let rational = RationalBezier2::try_from_subcurve(curve)
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?;
                let cusp_index = if *cusp_is_first {
                    pair.first_carrier_index
                } else {
                    pair.second_carrier_index
                };
                if let Some((sibling_index, sibling_at_start, curve_at_start)) =
                    self.authored_supporting_circle_endpoint(cusp_index, curve_index, |_, _| true)
                    && cusp
                        .semicircle()
                        .certifies_unique_rational_circle_contact(
                            &rational,
                            self.data.carriers[sibling_index]
                                .geometry
                                .circle()
                                .certified_tangent_endpoint(sibling_at_start),
                            &self.data.policy,
                        )
                        .map_err(|cause| self.invalid(curve_index, cause))?
                {
                    // Distinct tangent supporting circles share exactly one
                    // point. Boundary connectivity owns its parameter on an
                    // adjacent chart of this complete circle. Reuse that
                    // identity and transport it to the consumed half chart.
                    if sibling_index == cusp_index {
                        return Ok(RegionPairResult::empty());
                    }
                    let sibling = self.data.carriers[sibling_index].geometry.circle();
                    let mapped = self
                        .data
                        .policy
                        .strict_predicate_pass(|| {
                            cusp.parameter_of_shared_circle_endpoint(
                                sibling,
                                sibling_at_start,
                                &self.data.policy,
                            )
                        })
                        .map_err(|cause| self.invalid(cusp_index, cause))?;
                    match mapped {
                        Classification::Decided(None) => return Ok(RegionPairResult::empty()),
                        Classification::Decided(Some(parameter)) => {
                            let point = match sibling
                                .endpoint_point_evidence(sibling_at_start, &self.data.policy)
                                .map_err(|cause| self.invalid(cusp_index, cause))?
                            {
                                Classification::Decided(point) => point,
                                Classification::Uncertain(_) => None,
                            };
                            let circle_parameter = CurveParameter2::from_algebraic_cusp(parameter);
                            let curve_parameter = if curve_at_start {
                                carrier_traversal_start(curve_carrier)
                            } else {
                                carrier_traversal_end(curve_carrier)
                            }
                            .clone();
                            let (first_parameter, second_parameter) = if *cusp_is_first {
                                (circle_parameter, curve_parameter)
                            } else {
                                (curve_parameter, circle_parameter)
                            };
                            return Ok(RegionPairResult {
                                contacts: vec![RegionPairContactEvidence::direct(
                                    first_parameter,
                                    second_parameter,
                                    point,
                                    false,
                                    Some(RealSign::Zero),
                                )],
                                overlaps: Vec::new(),
                                blockers: Vec::new(),
                            });
                        }
                        Classification::Uncertain(_) => {}
                    }
                }
                self.algebraic_cusp_rational_pair_result(pair, cusp, &rational, *cusp_is_first)
            }
            RegionCarrierPairContext::CuspParallel { cusp_is_first } => {
                let (cusp, parallel, parallel_carrier, parallel_index) = if *cusp_is_first {
                    (
                        first.geometry.circle(),
                        second.geometry.parallel(),
                        second,
                        pair.second_carrier_index,
                    )
                } else {
                    (
                        second.geometry.circle(),
                        first.geometry.parallel(),
                        first,
                        pair.first_carrier_index,
                    )
                };
                let parallel_range = CurveParameterRange2::new_validated(
                    parallel_carrier.start.clone(),
                    parallel_carrier.end.clone(),
                );
                if let Classification::Decided(Some(component)) = self.data.policy.bounded_exact_predicate_pass(|| parallel
                    .exact_rational_parallel_component_on_regular_range(&parallel_range, &self.data.policy))
                    .map_err(|cause| self.invalid(parallel_index, cause))?
                    // Exact affine lines are owned by the shared lower
                    // circle/parallel kernel, which delegates to the same
                    // circle/chord authority used by fillets and offsets.
                    && component.curve().exact_linear_parameterization_line().is_none()
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-parallel-pair",
                        "strict-rational-component",
                    );
                    let result = self.algebraic_cusp_rational_pair_result(
                        pair,
                        cusp,
                        component.curve(),
                        *cusp_is_first,
                    )?;
                    if result.blockers.is_empty() {
                        return Ok(result);
                    }
                    // The rationalized component is only a compact fast path.
                    // Recursive selected-circle frames retain a smaller exact
                    // circle/parallel authority that can decide the same
                    // finite range when global rational projection cannot.
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-parallel-pair",
                        "rational-component-fallback",
                    );
                }
                let intersections = match cusp
                    .semicircle()
                    .parallel_intersections(parallel, &parallel_range, None, &self.data.policy)
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                {
                    Classification::Decided(result) => result,
                    Classification::Uncertain(reason) => {
                        return Ok(RegionPairResult {
                            contacts: Vec::new(),
                            overlaps: Vec::new(),
                            blockers: vec![RegionPairBlocker::Uncertain(reason)],
                        });
                    }
                };
                let (contacts, overlaps) = match intersections {
                    BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped { contacts, overlaps } => (contacts, overlaps),
                    BezierAlgebraicCuspSemicircleParallelIntersections2::RetainedContacts(
                        contacts,
                    ) => {
                        return Ok(retained_cusp_parallel_contacts_result(
                            contacts,
                            *cusp_is_first,
                        ));
                    }
                    BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber { contacts, overlaps } => {
                        return Ok(selected_fiber_cusp_result(contacts, overlaps, *cusp_is_first));
                    }
                    BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent
                    | BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection => {
                        return Ok(RegionPairResult {
                            contacts: Vec::new(),
                            overlaps: Vec::new(),
                            blockers: vec![RegionPairBlocker::Uncertain(
                                UncertaintyReason::Unsupported,
                            )],
                        });
                    }
                };
                let parameter_map = if contacts
                    .iter()
                    .any(|contact| contact.retained_cusp_parameter().is_none())
                {
                    match cusp
                        .semicircle()
                        .parallel_parameter_map(parallel, &self.data.policy)
                        .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                    {
                        Classification::Decided(map) => Some(map),
                        Classification::Uncertain(reason) => {
                            return Ok(RegionPairResult {
                                contacts: Vec::new(),
                                overlaps: Vec::new(),
                                blockers: vec![RegionPairBlocker::Uncertain(reason)],
                            });
                        }
                    }
                } else {
                    None
                };
                let mut retained = Vec::with_capacity(contacts.len());
                for contact in contacts {
                    let cusp_parameter = contact.retained_cusp_parameter().unwrap_or_else(|| {
                        parameter_map
                            .as_ref()
                            .expect("an interior cusp/parallel contact retains its parameter map")
                            .contact_parameter(&contact)
                    });
                    let tangent_cross_sign = contact
                        .tangent_cross_sign
                        .map(|sign| orient_tangent_cross_sign(sign, *cusp_is_first));
                    let tangent_topology = if tangent_cross_sign == Some(RealSign::Zero) {
                        match cusp
                            .semicircle()
                            .parallel_contact_endpoint_tangent_topology(
                                parallel,
                                &contact,
                                &self.data.policy,
                            )
                            .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                        {
                            Classification::Decided(Some((dot, circle_side)))
                                if dot != RealSign::Zero =>
                            {
                                let side = if *cusp_is_first {
                                    match parallel
                                        .tangent_side_at(
                                            &contact.parallel_parameter,
                                            &self.data.policy,
                                        )
                                        .map_err(|cause| {
                                            self.invalid(pair.first_carrier_index, cause)
                                        })? {
                                        Classification::Decided(LineSide::Left) => {
                                            Some(if dot == RealSign::Positive {
                                                LineSide::Left
                                            } else {
                                                LineSide::Right
                                            })
                                        }
                                        Classification::Decided(LineSide::Right) => {
                                            Some(if dot == RealSign::Positive {
                                                LineSide::Right
                                            } else {
                                                LineSide::Left
                                            })
                                        }
                                        Classification::Decided(LineSide::On)
                                        | Classification::Uncertain(_) => None,
                                    }
                                } else {
                                    Some(circle_side)
                                };
                                side.map(|side| (dot, side))
                            }
                            Classification::Decided(Some(_))
                            | Classification::Decided(None)
                            | Classification::Uncertain(_) => None,
                        }
                    } else {
                        None
                    };
                    let (first_parameter, second_parameter) = if *cusp_is_first {
                        (
                            CurveParameter2::from_algebraic_cusp(cusp_parameter),
                            CurveParameter2::from(contact.parallel_parameter),
                        )
                    } else {
                        (
                            CurveParameter2::from(contact.parallel_parameter),
                            CurveParameter2::from_algebraic_cusp(cusp_parameter),
                        )
                    };
                    let evidence = RegionPairContactEvidence::direct(
                        first_parameter,
                        second_parameter,
                        None,
                        matches!(
                            tangent_cross_sign,
                            Some(RealSign::Positive | RealSign::Negative)
                        ),
                        tangent_cross_sign,
                    );
                    retained.push(match tangent_topology {
                        Some((dot, side)) => evidence.with_tangent_topology(dot, side),
                        None => evidence,
                    });
                }
                Ok(RegionPairResult {
                    contacts: retained,
                    overlaps: overlaps
                        .into_iter()
                        .map(|source| {
                            circle_overlap_evidence(
                                CurveCircleOverlap2::Mapped(source),
                                *cusp_is_first,
                            )
                        })
                        .collect(),
                    blockers: Vec::new(),
                })
            }
            RegionCarrierPairContext::CuspPair => {
                let first_cusp = first.geometry.circle();
                let second_cusp = second.geometry.circle();
                if let Some((first_at_start, second_at_start)) = self
                    .authored_carrier_shared_endpoints(
                        pair.first_carrier_index,
                        pair.second_carrier_index,
                    )
                    && (first_cusp.certified_tangent_endpoint(first_at_start)
                        || second_cusp.certified_tangent_endpoint(second_at_start))
                {
                    // The round/fillet constructor certifies tangency to its
                    // authored boundary neighbor before the carriers become
                    // independent arrangement entries. Loop seeding already
                    // owns their shared vertex, so there is no extra contact
                    // event to reconstruct.
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-cusp-pair",
                        "adjacent-certified-tangent",
                    );
                    return Ok(RegionPairResult::empty());
                }
                if let Classification::Decided(Some((first_parameter, second_parameter))) =
                    first_cusp
                        .unique_shared_tangent_endpoint_contact(second_cusp, &self.data.policy)
                        .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                {
                    if self.authored_carriers_are_adjacent(pair) {
                        // Loop seeding already owns this exact vertex on both
                        // carrier domains. An authored tangent switch neither
                        // splits either injective carrier nor adds a second
                        // topology event, so replaying its mapped parameter
                        // range would duplicate the retained adjacency proof.
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "curve-region-cusp-pair",
                            "adjacent-retained-endpoint-tangency",
                        );
                        return Ok(RegionPairResult::empty());
                    }
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-cusp-pair",
                        "retained-endpoint-tangency",
                    );
                    return Ok(RegionPairResult {
                        contacts: vec![RegionPairContactEvidence::direct(
                            CurveParameter2::from_algebraic_cusp(first_parameter),
                            CurveParameter2::from_algebraic_cusp(second_parameter),
                            None,
                            false,
                            Some(RealSign::Zero),
                        )],
                        overlaps: Vec::new(),
                        blockers: Vec::new(),
                    });
                }
                let intersections = match first_cusp
                    .semicircle()
                    .pair_intersections(second_cusp.semicircle(), &self.data.policy)
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                {
                    Classification::Decided(result) => result,
                    Classification::Uncertain(reason) => {
                        return Ok(RegionPairResult {
                            contacts: Vec::new(),
                            overlaps: Vec::new(),
                            blockers: vec![RegionPairBlocker::Uncertain(reason)],
                        });
                    }
                };
                let mut retained = Vec::new();
                let mut overlaps = Vec::new();
                match intersections {
                    BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts => {}
                    BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                        contacts,
                        parameter_map,
                    } => {
                        retained.reserve(contacts.len());
                        for contact in contacts {
                            let tangent_cross_sign = contact.tangent_cross_sign;
                            retained.push(RegionPairContactEvidence::direct(
                                CurveParameter2::from_algebraic_cusp(
                                    parameter_map.first_contact_parameter(&contact),
                                ),
                                CurveParameter2::from_algebraic_cusp(
                                    parameter_map.second_contact_parameter(&contact),
                                ),
                                None,
                                tangent_cross_sign != RealSign::Zero,
                                Some(tangent_cross_sign),
                            ));
                        }
                    }
                    BezierAlgebraicCuspSemicirclePairIntersections2::EndpointContacts(contacts) => {
                        retained.reserve(contacts.len());
                        for contact in contacts {
                            let first_parameter = contact
                                .first_location
                                .endpoint_parameter()
                                .expect("an endpoint contact names a first cusp endpoint");
                            let second_parameter = contact
                                .second_location
                                .endpoint_parameter()
                                .expect("an endpoint contact names a second cusp endpoint");
                            retained.push(RegionPairContactEvidence::direct(
                                CurveParameter2::from_algebraic_cusp(first_parameter),
                                CurveParameter2::from_algebraic_cusp(second_parameter),
                                None,
                                false,
                                Some(RealSign::Zero),
                            ));
                        }
                    }
                    BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(overlap) => {
                        overlaps.push(CurveIntersectionOverlap2 {
                            first_span_index: 0,
                            second_span_index: 0,
                            endpoint_inclusion: [true, true],
                            parameter_correspondence: CurveOverlapCorrespondence2::Circle {
                                source: CurveCircleOverlap2::Pair(overlap.clone()),
                                swapped: false,
                            },
                            first_range: CurveParameterRange2::new_validated(
                                CurveParameter2::from_algebraic_cusp(
                                    overlap.first_start_parameter(),
                                ),
                                CurveParameter2::from_algebraic_cusp(overlap.first_end_parameter()),
                            ),
                            second_range: CurveParameterRange2::new_validated(
                                CurveParameter2::from_algebraic_cusp(
                                    overlap.second_start_parameter(),
                                ),
                                CurveParameter2::from_algebraic_cusp(
                                    overlap.second_end_parameter(),
                                ),
                            ),
                            orientation: overlap.orientation(),
                        });
                    }
                }
                Ok(RegionPairResult {
                    contacts: retained,
                    overlaps,
                    blockers: Vec::new(),
                })
            }
        }
    }

    pub(super) fn parallel_pair_is_coordinate_disjoint(&self, pair: &RegionCarrierPair) -> bool {
        let first = &self.data.carriers[pair.first_carrier_index];
        let second = &self.data.carriers[pair.second_carrier_index];
        let (CurveSupport2::Parallel(first_parallel), CurveSupport2::Parallel(second_parallel)) =
            (&first.geometry, &second.geometry)
        else {
            return false;
        };
        let Some(first_start) = exact_carrier_point(
            first,
            carrier_traversal_start(first),
            &self.data.policy,
        ) else {
            return false;
        };
        let Some(first_end) = exact_carrier_point(
            first,
            carrier_traversal_end(first),
            &self.data.policy,
        ) else {
            return false;
        };
        let Some(second_start) = exact_carrier_point(
            second,
            carrier_traversal_start(second),
            &self.data.policy,
        ) else {
            return false;
        };
        let Some(second_end) = exact_carrier_point(
            second,
            carrier_traversal_end(second),
            &self.data.policy,
        ) else {
            return false;
        };

        for axis in [Axis2::X, Axis2::Y] {
            if !first_parallel.range_has_certified_injective_axis_on(
                axis,
                &first.range(),
                &self.data.policy,
            ) || !second_parallel.range_has_certified_injective_axis_on(
                axis,
                &second.range(),
                &self.data.policy,
            ) {
                continue;
            }
            let Some((first_minimum, first_maximum)) =
                ordered_axis_endpoint_points(&first_start, &first_end, axis, &self.data.policy)
            else {
                continue;
            };
            let Some((second_minimum, second_maximum)) =
                ordered_axis_endpoint_points(&second_start, &second_end, axis, &self.data.policy)
            else {
                continue;
            };
            for (lower_maximum, upper_minimum) in [
                (first_maximum, second_minimum),
                (second_maximum, first_minimum),
            ] {
                match compare_reals(
                    point_coordinate(lower_maximum, axis),
                    point_coordinate(upper_minimum, axis),
                    &self.data.policy,
                ) {
                    Some(Ordering::Less) => return true,
                    Some(Ordering::Equal)
                        if points_are_decided_distinct(
                            lower_maximum,
                            upper_minimum,
                            &self.data.policy,
                        ) =>
                    {
                        // Strict coordinate monotonicity makes this boundary
                        // value unique on each carrier.  Distinct endpoint
                        // points therefore exclude even a tangential contact.
                        return true;
                    }
                    Some(Ordering::Equal | Ordering::Greater) | None => {}
                }
            }
        }
        false
    }

    pub(super) fn adjacent_parallel_pair_is_endpoint_only(&self, pair: &RegionCarrierPair) -> bool {
        if pair.first_carrier_index == pair.second_carrier_index {
            return false;
        }
        let first = &self.data.carriers[pair.first_carrier_index];
        let second = &self.data.carriers[pair.second_carrier_index];
        if first.operand != second.operand || first.loop_index != second.loop_index {
            return false;
        }
        let (CurveSupport2::Parallel(first_parallel), CurveSupport2::Parallel(second_parallel)) =
            (&first.geometry, &second.geometry)
        else {
            return false;
        };
        let boundary = match first.operand {
            CurveRegionBooleanOperand2::First => self.data.first.boundary_loops(),
            CurveRegionBooleanOperand2::Second => self.data.second.boundary_loops(),
        }
        .get(first.loop_index);
        let Some(boundary) = boundary else {
            return false;
        };
        let fragment_count = boundary.fragments().len();
        let first_start = carrier_traversal_start(first);
        let first_end = carrier_traversal_end(first);
        let second_start = carrier_traversal_start(second);
        let second_end = carrier_traversal_end(second);
        let (first_other, first_shared, second_shared, second_other) =
            if first.fragment_index.checked_add(1) == Some(second.fragment_index) {
                (first_start, first_end, second_start, second_end)
            } else if first.fragment_index == 0
                && second.fragment_index.checked_add(1) == Some(fragment_count)
            {
                (first_end, first_start, second_end, second_start)
            } else {
                return false;
            };
        let Some(first_other) = exact_carrier_point(first, first_other, &self.data.policy) else {
            return false;
        };
        let Some(first_shared) = exact_carrier_point(first, first_shared, &self.data.policy) else {
            return false;
        };
        let Some(second_shared) = exact_carrier_point(second, second_shared, &self.data.policy)
        else {
            return false;
        };
        let Some(second_other) = exact_carrier_point(second, second_other, &self.data.policy)
        else {
            return false;
        };
        if compare_reals(first_shared.x(), second_shared.x(), &self.data.policy)
            != Some(Ordering::Equal)
            || compare_reals(first_shared.y(), second_shared.y(), &self.data.policy)
                != Some(Ordering::Equal)
        {
            return false;
        }

        for axis in [Axis2::X, Axis2::Y] {
            if !first_parallel.range_has_certified_injective_axis_on(
                axis,
                &first.range(),
                &self.data.policy,
            ) || !second_parallel.range_has_certified_injective_axis_on(
                axis,
                &second.range(),
                &self.data.policy,
            ) {
                continue;
            }
            let first_order = compare_reals(
                point_coordinate(&first_other, axis),
                point_coordinate(&first_shared, axis),
                &self.data.policy,
            );
            let second_order = compare_reals(
                point_coordinate(&second_other, axis),
                point_coordinate(&second_shared, axis),
                &self.data.policy,
            );
            if matches!(
                (first_order, second_order),
                (Some(Ordering::Less), Some(Ordering::Greater))
                    | (Some(Ordering::Greater), Some(Ordering::Less))
            ) {
                return true;
            }
        }
        false
    }

    /// Keeps overlap endpoint incidence independent of the pair kernel's range
    /// ordering. The orientation relates the two underlying source charts.
    pub(super) fn paired_overlap_ranges(
        &self,
        pair: &RegionCarrierPair,
        orientation: CurveOverlapOrientation2,
        (first, second): (CurveParameterRange2, CurveParameterRange2),
    ) -> ExactCurveResult<(CurveParameterRange2, CurveParameterRange2)> {
        let first_direction = decided_parameter_cmp(first.start(), first.end(), &self.data.policy)?;
        let second_direction =
            decided_parameter_cmp(second.start(), second.end(), &self.data.policy)?;
        if first_direction == Ordering::Equal || second_direction == Ordering::Equal {
            return Err(self.invalid(pair.first_carrier_index, CurveError::DegenerateOverlapRange));
        }
        let corresponding = (first_direction == second_direction)
            == (orientation == CurveOverlapOrientation2::Same);
        let second = if corresponding {
            second
        } else {
            CurveParameterRange2::new_validated(second.end().clone(), second.start().clone())
        };
        Ok((first, second))
    }

    pub(super) fn clipped_overlap_ranges(
        &self,
        pair: &RegionCarrierPair,
        overlap: &CurveIntersectionOverlap2,
    ) -> ExactCurveResult<Option<(CurveParameterRange2, CurveParameterRange2)>> {
        let first_carrier = &self.data.carriers[pair.first_carrier_index];
        let second_carrier = &self.data.carriers[pair.second_carrier_index];
        let first_intersects =
            ranges_intersect(&overlap.first_range, first_carrier, &self.data.policy)?;
        let second_intersects =
            ranges_intersect(&overlap.second_range, second_carrier, &self.data.policy)?;
        if !first_intersects || !second_intersects {
            return Ok(None);
        }
        let same_parameter_domain = |first: &CurveParameter2, second: &CurveParameter2| {
            (first.as_bezier_parameter().is_some() && second.as_bezier_parameter().is_some())
                || (first.as_selected_fiber().is_some() && second.as_selected_fiber().is_some())
                || (first.as_recursive_projective().is_some()
                    && second.as_recursive_projective().is_some())
                || (first.is_algebraic_chord() && second.is_algebraic_chord())
                || (first.is_algebraic_cusp() && second.is_algebraic_cusp())
        };
        let range_uses_carrier_domain = |range: &CurveParameterRange2, carrier: &RegionCarrier| {
            same_parameter_domain(range.start(), &carrier.start)
                && same_parameter_domain(range.end(), &carrier.end)
        };
        if range_inside_carrier(&overlap.first_range, first_carrier, &self.data.policy)?
            && range_inside_carrier(&overlap.second_range, second_carrier, &self.data.policy)?
            && range_uses_carrier_domain(&overlap.first_range, first_carrier)
            && range_uses_carrier_domain(&overlap.second_range, second_carrier)
        {
            return Ok(Some((
                overlap.first_range.clone(),
                overlap.second_range.clone(),
            )));
        }
        match overlap
            .restrict_raw(
                &first_carrier.range(),
                &second_carrier.range(),
                &self.data.policy,
            )
            .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
        {
            Classification::Decided(overlap) => {
                Ok(overlap.map(|overlap| (overlap.first_range, overlap.second_range)))
            }
            Classification::Uncertain(reason) => {
                Err(self.blocked(pair.first_carrier_index, reason))
            }
        }
    }
}
