//! Composable exact Booleans over retained curved regions.

use std::cell::OnceCell;
use std::cmp::Ordering;
use std::rc::Rc;

use crate::{
    BezierArrangementFragment2, BezierArrangementGraph2, BezierParameter2, BezierParameterRange2,
    BezierSplitFragment2, BezierSubcurve2, BooleanOp, Classification, Curve2, CurveError,
    CurveFamily2, CurveOperation2, CurvePathBooleanOperand2, CurvePolicy, CurveRegion2,
    CurveRegionFragmentProvenance2, ExactCurveError, ExactCurveResult, PreparedCurveIntersection2,
    RationalBezierIntersectionPointEvidence2, RationalBezierOverlapOrientation2,
    RegionPointLocation, UncertaintyReason,
};

/// Clone-shared retained preparation for repeated curved-region Booleans.
#[derive(Clone, Debug)]
pub struct PreparedCurveRegionBoolean2 {
    data: Rc<PreparedCurveRegionBooleanData>,
}

#[derive(Debug)]
struct PreparedCurveRegionBooleanData {
    first: CurveRegion2,
    second: CurveRegion2,
    policy: CurvePolicy,
    carriers: Rc<[RegionCarrier]>,
    first_carrier_count: usize,
    authored_carrier_pair_count: usize,
    pairs: Vec<PreparedRegionCarrierPair>,
    results: [OnceCell<ExactCurveResult<CurveRegion2>>; 4],
}

#[derive(Clone, Debug)]
struct RegionCarrier {
    operand: CurvePathBooleanOperand2,
    loop_index: usize,
    family: CurveFamily2,
    curve: BezierSubcurve2,
    start: BezierParameter2,
    end: BezierParameter2,
    reversed: bool,
    filled_side_is_left: bool,
    provenance: Option<CurveRegionFragmentProvenance2>,
}

#[derive(Debug)]
struct PreparedRegionCarrierPair {
    first_carrier_index: usize,
    second_carrier_index: usize,
    prepared: PreparedCurveIntersection2,
}

#[derive(Clone, Debug)]
struct CarrierEvent {
    parameter: BezierParameter2,
    topology_vertex: Option<usize>,
}

#[derive(Clone, Debug)]
struct CarrierOverlap {
    first_carrier_index: usize,
    second_carrier_index: usize,
    first_range: BezierParameterRange2,
    second_range: BezierParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
}

#[derive(Clone, Debug)]
struct SplitCarrierFragment {
    fragment: BezierSplitFragment2,
    start_topology_vertex: Option<usize>,
    end_topology_vertex: Option<usize>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RegionFragmentAction {
    Discard,
    Keep,
    KeepReversed,
}

impl CurveRegion2 {
    /// Prepares a region pair once for repeated exact regularized Booleans.
    pub fn try_prepare_boolean(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<PreparedCurveRegionBoolean2> {
        PreparedCurveRegionBoolean2::try_new(self, other, policy)
    }

    /// Computes one exact regularized Boolean against another retained region.
    pub fn boolean_region(
        &self,
        other: &Self,
        operation: BooleanOp,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Self> {
        self.try_prepare_boolean(other, policy)?
            .boolean_region(operation)
    }
}

impl PreparedCurveRegionBoolean2 {
    fn try_new(
        first: &CurveRegion2,
        second: &CurveRegion2,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Self> {
        let first_carriers = build_region_carriers(first, CurvePathBooleanOperand2::First, policy)?;
        let first_carrier_count = first_carriers.len();
        let mut carriers = first_carriers;
        carriers.extend(build_region_carriers(
            second,
            CurvePathBooleanOperand2::Second,
            policy,
        )?);

        let authored_carrier_pair_count =
            first_carrier_count.saturating_mul(carriers.len() - first_carrier_count);
        let curves = carriers
            .iter()
            .map(|carrier| Curve2::from(carrier.curve.clone()))
            .collect::<Vec<_>>();
        let mut pairs = Vec::with_capacity(
            first_carrier_count
                .saturating_add(carriers.len() - first_carrier_count)
                .min(authored_carrier_pair_count),
        );
        for first_carrier_index in 0..first_carrier_count {
            for second_carrier_index in first_carrier_count..carriers.len() {
                let first_curve = &curves[first_carrier_index];
                let second_curve = &curves[second_carrier_index];
                if carrier_bounds_decided_disjoint(first_curve, second_curve, policy) {
                    continue;
                }
                pairs.push(PreparedRegionCarrierPair {
                    first_carrier_index,
                    second_carrier_index,
                    prepared: first_curve.try_prepare_intersection(second_curve, policy)?,
                });
            }
        }

        Ok(Self {
            data: Rc::new(PreparedCurveRegionBooleanData {
                first: first.clone(),
                second: second.clone(),
                policy: policy.clone(),
                carriers: carriers.into(),
                first_carrier_count,
                authored_carrier_pair_count,
                pairs,
                results: std::array::from_fn(|_| OnceCell::new()),
            }),
        })
    }

    /// Returns the retained first region.
    pub fn first(&self) -> &CurveRegion2 {
        &self.data.first
    }

    /// Returns the retained second region.
    pub fn second(&self) -> &CurveRegion2 {
        &self.data.second
    }

    /// Returns the policy captured by preparation.
    pub fn policy(&self) -> &CurvePolicy {
        &self.data.policy
    }

    /// Returns the Cartesian carrier-pair count before certified broad-phase filtering.
    pub fn authored_carrier_pair_count(&self) -> usize {
        self.data.authored_carrier_pair_count
    }

    /// Returns the number of retained cross-region candidate pairs.
    pub fn carrier_pair_count(&self) -> usize {
        self.data.pairs.len()
    }

    /// Returns whether this operation has already been materialized.
    pub fn is_boolean_region_cached(&self, operation: BooleanOp) -> bool {
        self.data.results[boolean_operation_index(operation)]
            .get()
            .is_some()
    }

    /// Returns a clone of one retained exact Boolean result.
    pub fn boolean_region(&self, operation: BooleanOp) -> ExactCurveResult<CurveRegion2> {
        self.boolean_region_view(operation).cloned()
    }

    /// Borrows one retained exact Boolean result.
    pub fn boolean_region_view(&self, operation: BooleanOp) -> ExactCurveResult<&CurveRegion2> {
        match self.data.results[boolean_operation_index(operation)]
            .get_or_init(|| self.build_boolean_region(operation))
        {
            Ok(region) => Ok(region),
            Err(error) => Err(error.clone()),
        }
    }

    fn build_boolean_region(&self, operation: BooleanOp) -> ExactCurveResult<CurveRegion2> {
        if self.data.first.is_empty() || self.data.second.is_empty() {
            return empty_operand_result(&self.data.first, &self.data.second, operation);
        }
        if self.data.first == self.data.second {
            return identical_operand_result(&self.data.first, operation);
        }

        let mut events = vec![Vec::new(); self.data.carriers.len()];
        let mut contact_points = Vec::<(RationalBezierIntersectionPointEvidence2, usize)>::new();
        let mut next_topology_vertex = 0_usize;
        seed_loop_topology_vertices(
            &self.data.carriers,
            &mut events,
            &mut next_topology_vertex,
            &self.data.policy,
        )?;
        let mut overlaps = Vec::new();
        for pair in &self.data.pairs {
            let report = pair.prepared.report_view()?;
            if let Some(blocker) = report.blockers().first() {
                let reason = match blocker.kind() {
                    crate::CurveIntersectionPairBlockerKind2::Uncertain(reason) => *reason,
                    crate::CurveIntersectionPairBlockerKind2::IncompleteReplay { .. } => {
                        UncertaintyReason::Predicate
                    }
                    crate::CurveIntersectionPairBlockerKind2::SharedComponent => {
                        UncertaintyReason::Boundary
                    }
                };
                return Err(self.blocked(pair.first_carrier_index, reason));
            }

            for contact in report.contacts() {
                let first_parameter = contact.first().local_parameter();
                let second_parameter = contact.second().local_parameter();
                if !parameter_in_carrier(
                    first_parameter,
                    &self.data.carriers[pair.first_carrier_index],
                    &self.data.policy,
                )? || !parameter_in_carrier(
                    second_parameter,
                    &self.data.carriers[pair.second_carrier_index],
                    &self.data.policy,
                )? {
                    continue;
                }
                let first_existing = existing_event_vertex(
                    &events[pair.first_carrier_index],
                    first_parameter,
                    &self.data.policy,
                )?;
                let second_existing = existing_event_vertex(
                    &events[pair.second_carrier_index],
                    second_parameter,
                    &self.data.policy,
                )?;
                let topology_vertex = match (first_existing, second_existing) {
                    (Some(first), Some(second)) if first != second => {
                        replace_topology_vertex(&mut events, &mut contact_points, second, first);
                        first
                    }
                    (Some(vertex), _) | (_, Some(vertex)) => vertex,
                    (None, None) => contact_points
                        .iter()
                        .find_map(|(point, vertex)| {
                            same_contact_point(point, contact.point(), &self.data.policy)
                                .then_some(*vertex)
                        })
                        .unwrap_or_else(|| {
                            let vertex = next_topology_vertex;
                            next_topology_vertex += 1;
                            vertex
                        }),
                };
                if !contact_points
                    .iter()
                    .any(|(point, _)| same_contact_point(point, contact.point(), &self.data.policy))
                {
                    contact_points.push((contact.point().clone(), topology_vertex));
                }
                push_carrier_event(
                    &mut events[pair.first_carrier_index],
                    first_parameter.clone(),
                    Some(topology_vertex),
                    &self.data.policy,
                )?;
                push_carrier_event(
                    &mut events[pair.second_carrier_index],
                    second_parameter.clone(),
                    Some(topology_vertex),
                    &self.data.policy,
                )?;
            }

            for overlap in report.overlaps() {
                let first_carrier = &self.data.carriers[pair.first_carrier_index];
                let second_carrier = &self.data.carriers[pair.second_carrier_index];
                let first_intersects =
                    ranges_intersect(overlap.first_range(), first_carrier, &self.data.policy)?;
                let second_intersects =
                    ranges_intersect(overlap.second_range(), second_carrier, &self.data.policy)?;
                if !first_intersects && !second_intersects {
                    continue;
                }
                let (first_range, second_range) = if first_intersects == second_intersects
                    && range_inside_carrier(
                        overlap.first_range(),
                        first_carrier,
                        &self.data.policy,
                    )?
                    && range_inside_carrier(
                        overlap.second_range(),
                        second_carrier,
                        &self.data.policy,
                    )? {
                    (
                        overlap.first_range().clone(),
                        overlap.second_range().clone(),
                    )
                } else {
                    let Some(ranges) = clip_identity_parameter_overlap(
                        overlap.first_range(),
                        overlap.second_range(),
                        overlap.orientation(),
                        first_carrier,
                        second_carrier,
                        &self.data.policy,
                    )?
                    else {
                        if identity_parameter_correspondence(
                            overlap.first_range(),
                            overlap.second_range(),
                            overlap.orientation(),
                            first_carrier,
                            second_carrier,
                            &self.data.policy,
                        )? {
                            continue;
                        }
                        return Err(
                            self.blocked(pair.first_carrier_index, UncertaintyReason::Unsupported)
                        );
                    };
                    ranges
                };
                let first_parameters = [first_range.start(), first_range.end()];
                let second_parameters = [second_range.start(), second_range.end()];
                for (parameter, second_parameter) in
                    first_parameters.into_iter().zip(second_parameters)
                {
                    push_carrier_event(
                        &mut events[pair.first_carrier_index],
                        parameter.clone(),
                        None,
                        &self.data.policy,
                    )?;
                    push_carrier_event(
                        &mut events[pair.second_carrier_index],
                        second_parameter.clone(),
                        None,
                        &self.data.policy,
                    )?;
                }
                overlaps.push(CarrierOverlap {
                    first_carrier_index: pair.first_carrier_index,
                    second_carrier_index: pair.second_carrier_index,
                    first_range,
                    second_range,
                    orientation: overlap.orientation(),
                });
            }
        }

        let mut arrangement_fragments = Vec::new();
        let mut output_provenance = Vec::new();
        let mut provenance_complete = true;
        for (carrier_index, carrier) in self.data.carriers.iter().enumerate() {
            let split_fragments = split_carrier(carrier, &events[carrier_index], &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            for (split_fragment_index, split) in split_fragments.into_iter().enumerate() {
                let action =
                    self.fragment_action(carrier_index, &split.fragment, &overlaps, operation)?;
                if action == RegionFragmentAction::Discard {
                    continue;
                }
                let fragment = match action {
                    RegionFragmentAction::Keep => split.fragment,
                    RegionFragmentAction::KeepReversed => split
                        .fragment
                        .reversed()
                        .map_err(|cause| self.invalid(carrier_index, cause))?,
                    RegionFragmentAction::Discard => unreachable!(),
                };
                let (start_topology_vertex, end_topology_vertex) = match action {
                    RegionFragmentAction::Keep => {
                        (split.start_topology_vertex, split.end_topology_vertex)
                    }
                    RegionFragmentAction::KeepReversed => {
                        (split.end_topology_vertex, split.start_topology_vertex)
                    }
                    RegionFragmentAction::Discard => unreachable!(),
                };
                let arrangement_fragment_index = arrangement_fragments.len();
                arrangement_fragments.push(
                    BezierArrangementFragment2::new(carrier_index, split_fragment_index, fragment)
                        .with_topology_vertices(start_topology_vertex, end_topology_vertex),
                );
                if let Some(provenance) = &carrier.provenance {
                    output_provenance.push(CurveRegionFragmentProvenance2::new(
                        arrangement_fragment_index,
                        carrier_index,
                        Some(carrier.operand),
                        provenance.source_path_index(),
                        provenance.family(),
                        provenance.curve_index(),
                        provenance.promoted_span_index(),
                        split_fragment_index,
                        provenance.span().clone(),
                        provenance.reversed() ^ (action == RegionFragmentAction::KeepReversed),
                    ));
                } else {
                    provenance_complete = false;
                }
            }
        }

        let graph = BezierArrangementGraph2::new(arrangement_fragments)
            .map_err(|cause| self.invalid(0, cause))?;
        let traversal = match graph.traverse_retained_with_tangent_order(&self.data.policy) {
            Classification::Decided(traversal) => traversal,
            Classification::Uncertain(reason) => return Err(self.blocked(0, reason)),
        };
        let mut region = match CurveRegion2::from_retained_arrangement_traversal(&graph, &traversal)
        {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => return Err(self.blocked(0, reason)),
        };
        region = region
            .with_certified_filled_side_is_left(vec![true; traversal.chains().len()])
            .map_err(|cause| self.invalid(0, cause))?;
        if provenance_complete {
            region = region
                .with_fragment_provenance(output_provenance)
                .map_err(|cause| self.invalid(0, cause))?;
        }
        Ok(region)
    }

    fn fragment_action(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
        overlaps: &[CarrierOverlap],
        operation: BooleanOp,
    ) -> ExactCurveResult<RegionFragmentAction> {
        let carrier = &self.data.carriers[carrier_index];
        let representative = match fragment
            .representative_point(&self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Err(self.blocked(carrier_index, reason)),
        };
        let other = match carrier.operand {
            CurvePathBooleanOperand2::First => &self.data.second,
            CurvePathBooleanOperand2::Second => &self.data.first,
        };
        let location = match other
            .classify_point(&representative, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(location) => location,
            Classification::Uncertain(reason) => return Err(self.blocked(carrier_index, reason)),
        };
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

    fn shared_fragment_action(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
        overlaps: &[CarrierOverlap],
        operation: BooleanOp,
    ) -> ExactCurveResult<RegionFragmentAction> {
        let (start, end) = fragment_range(fragment);
        let Some(overlap) = overlaps.iter().find(|overlap| {
            let range = if overlap.first_carrier_index == carrier_index {
                &overlap.first_range
            } else if overlap.second_carrier_index == carrier_index {
                &overlap.second_range
            } else {
                return false;
            };
            range_contains_fragment(range, start, end, &self.data.policy).unwrap_or(false)
        }) else {
            return Err(self.blocked(carrier_index, UncertaintyReason::Boundary));
        };
        if carrier_index >= self.data.first_carrier_count {
            return Ok(RegionFragmentAction::Discard);
        }
        let first = &self.data.carriers[overlap.first_carrier_index];
        let second = &self.data.carriers[overlap.second_carrier_index];
        let same_source_direction = overlap.orientation == RationalBezierOverlapOrientation2::Same;
        let same_traversal = same_source_direction == (first.reversed == second.reversed);
        let second_left_in_first_direction = if same_traversal {
            second.filled_side_is_left
        } else {
            !second.filled_side_is_left
        };
        Ok(action_from_result_sides(
            operation.apply(first.filled_side_is_left, second_left_in_first_direction),
            operation.apply(!first.filled_side_is_left, !second_left_in_first_direction),
        ))
    }

    fn invalid(&self, carrier_index: usize, cause: CurveError) -> ExactCurveError {
        let carrier = &self.data.carriers[carrier_index];
        ExactCurveError::invalid(
            CurveOperation2::Boolean,
            carrier.family,
            source_of(carrier),
            cause,
        )
    }

    fn blocked(&self, carrier_index: usize, reason: UncertaintyReason) -> ExactCurveError {
        let carrier = &self.data.carriers[carrier_index];
        ExactCurveError::blocked(
            CurveOperation2::Boolean,
            carrier.family,
            source_of(carrier),
            reason,
        )
    }
}

fn carrier_bounds_decided_disjoint(first: &Curve2, second: &Curve2, policy: &CurvePolicy) -> bool {
    let (Ok(first_bounds), Ok(second_bounds)) = (first.bounds(), second.bounds()) else {
        return false;
    };
    matches!(
        first_bounds.overlaps(second_bounds, policy),
        Classification::Decided(false)
    )
}

fn build_region_carriers(
    region: &CurveRegion2,
    operand: CurvePathBooleanOperand2,
    policy: &CurvePolicy,
) -> ExactCurveResult<Vec<RegionCarrier>> {
    if region.is_empty() {
        return Ok(Vec::new());
    }
    let filled_sides = match region.filled_side_is_left(policy).map_err(|cause| {
        ExactCurveError::invalid(CurveOperation2::Boolean, CurveFamily2::Line, None, cause)
    })? {
        Classification::Decided(sides) => sides,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Boolean,
                CurveFamily2::Line,
                None,
                reason,
            ));
        }
    };
    let provenance = region.fragment_provenance();
    let mut flat_index = 0_usize;
    let mut carriers = Vec::new();
    for (loop_index, boundary_loop) in region.boundary_loops().iter().enumerate() {
        for fragment in boundary_loop.fragments() {
            let (curve, start, end, reversed) = match fragment {
                BezierSplitFragment2::Materialized { curve, .. } => (
                    curve.clone(),
                    BezierParameter2::Exact(crate::Real::zero()),
                    BezierParameter2::Exact(crate::Real::one()),
                    false,
                ),
                BezierSplitFragment2::AlgebraicEndpointImages {
                    reversed,
                    start,
                    end,
                    source_curve: Some(curve),
                    ..
                } => (curve.clone(), start.clone(), end.clone(), *reversed),
                BezierSplitFragment2::AlgebraicEndpointImages {
                    source_curve: None, ..
                }
                | BezierSplitFragment2::Unresolved { .. } => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Boolean,
                        CurveFamily2::RationalBezier,
                        None,
                        UncertaintyReason::Unsupported,
                    ));
                }
            };
            carriers.push(RegionCarrier {
                operand,
                loop_index,
                family: subcurve_family(&curve),
                curve,
                start,
                end,
                reversed,
                filled_side_is_left: filled_sides[loop_index],
                provenance: provenance
                    .and_then(|records| records.get(flat_index))
                    .cloned(),
            });
            flat_index += 1;
        }
    }
    Ok(carriers)
}

fn split_carrier(
    carrier: &RegionCarrier,
    events: &[CarrierEvent],
    policy: &CurvePolicy,
) -> Result<Vec<SplitCarrierFragment>, CurveError> {
    let mut parameters = events
        .iter()
        .map(|event| event.parameter.clone())
        .collect::<Vec<_>>();
    parameters.push(carrier.start.clone());
    parameters.push(carrier.end.clone());
    let materialization = match carrier
        .curve
        .split_at_parameters_refined(&parameters, policy)?
    {
        Classification::Decided(materialization) => materialization,
        Classification::Uncertain(reason) => {
            return Err(CurveError::Topology(format!(
                "retained curved-region split remained uncertain: {reason:?}"
            )));
        }
    };
    let mut output = Vec::new();
    for fragment in materialization.fragments() {
        let (start, end) = fragment_range(fragment);
        if !parameter_range_inside_carrier(start, end, carrier, policy)? {
            continue;
        }
        output.push(SplitCarrierFragment {
            fragment: if carrier.reversed {
                fragment.reversed()?
            } else {
                fragment.clone()
            },
            start_topology_vertex: event_vertex(events, start, policy)?,
            end_topology_vertex: event_vertex(events, end, policy)?,
        });
    }
    if carrier.reversed {
        output.reverse();
        for fragment in &mut output {
            std::mem::swap(
                &mut fragment.start_topology_vertex,
                &mut fragment.end_topology_vertex,
            );
        }
    }
    Ok(output)
}

fn push_carrier_event(
    events: &mut Vec<CarrierEvent>,
    parameter: BezierParameter2,
    topology_vertex: Option<usize>,
    policy: &CurvePolicy,
) -> ExactCurveResult<()> {
    for event in events.iter_mut() {
        match parameter
            .cmp_by_refinement(&event.parameter, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Boolean,
                    CurveFamily2::RationalBezier,
                    None,
                    cause,
                )
            })? {
            Classification::Decided(Ordering::Equal) => {
                if event.topology_vertex.is_none() {
                    event.topology_vertex = topology_vertex;
                }
                return Ok(());
            }
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Boolean,
                    CurveFamily2::RationalBezier,
                    None,
                    reason,
                ));
            }
        }
    }
    events.push(CarrierEvent {
        parameter,
        topology_vertex,
    });
    Ok(())
}

fn seed_loop_topology_vertices(
    carriers: &[RegionCarrier],
    events: &mut [Vec<CarrierEvent>],
    next_topology_vertex: &mut usize,
    policy: &CurvePolicy,
) -> ExactCurveResult<()> {
    let mut loop_start = 0_usize;
    while loop_start < carriers.len() {
        let operand = carriers[loop_start].operand;
        let loop_index = carriers[loop_start].loop_index;
        let mut loop_end = loop_start + 1;
        while loop_end < carriers.len()
            && carriers[loop_end].operand == operand
            && carriers[loop_end].loop_index == loop_index
        {
            loop_end += 1;
        }
        for current_index in loop_start..loop_end {
            let next_index = if current_index + 1 == loop_end {
                loop_start
            } else {
                current_index + 1
            };
            let vertex = *next_topology_vertex;
            *next_topology_vertex += 1;
            push_carrier_event(
                &mut events[current_index],
                carrier_traversal_end(&carriers[current_index]).clone(),
                Some(vertex),
                policy,
            )?;
            push_carrier_event(
                &mut events[next_index],
                carrier_traversal_start(&carriers[next_index]).clone(),
                Some(vertex),
                policy,
            )?;
        }
        loop_start = loop_end;
    }
    Ok(())
}

fn carrier_traversal_start(carrier: &RegionCarrier) -> &BezierParameter2 {
    if carrier.reversed {
        &carrier.end
    } else {
        &carrier.start
    }
}

fn carrier_traversal_end(carrier: &RegionCarrier) -> &BezierParameter2 {
    if carrier.reversed {
        &carrier.start
    } else {
        &carrier.end
    }
}

fn existing_event_vertex(
    events: &[CarrierEvent],
    parameter: &BezierParameter2,
    policy: &CurvePolicy,
) -> ExactCurveResult<Option<usize>> {
    for event in events {
        match decided_parameter_cmp(parameter, &event.parameter, policy)? {
            Ordering::Equal => return Ok(event.topology_vertex),
            Ordering::Less | Ordering::Greater => {}
        }
    }
    Ok(None)
}

fn replace_topology_vertex(
    events: &mut [Vec<CarrierEvent>],
    contact_points: &mut [(RationalBezierIntersectionPointEvidence2, usize)],
    from: usize,
    to: usize,
) {
    for event in events.iter_mut().flatten() {
        if event.topology_vertex == Some(from) {
            event.topology_vertex = Some(to);
        }
    }
    for (_, vertex) in contact_points {
        if *vertex == from {
            *vertex = to;
        }
    }
}

fn event_vertex(
    events: &[CarrierEvent],
    parameter: &BezierParameter2,
    policy: &CurvePolicy,
) -> Result<Option<usize>, CurveError> {
    for event in events {
        match parameter.cmp_by_refinement(&event.parameter, policy)? {
            Classification::Decided(Ordering::Equal) => return Ok(event.topology_vertex),
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "curved-region event ordering remained uncertain: {reason:?}"
                )));
            }
        }
    }
    Ok(None)
}

fn action_for_sides(
    operation: BooleanOp,
    operand: CurvePathBooleanOperand2,
    own_left: bool,
    other_inside: bool,
) -> RegionFragmentAction {
    let (result_left, result_right) = match operand {
        CurvePathBooleanOperand2::First => (
            operation.apply(own_left, other_inside),
            operation.apply(!own_left, other_inside),
        ),
        CurvePathBooleanOperand2::Second => (
            operation.apply(other_inside, own_left),
            operation.apply(other_inside, !own_left),
        ),
    };
    action_from_result_sides(result_left, result_right)
}

const fn action_from_result_sides(left: bool, right: bool) -> RegionFragmentAction {
    match (left, right) {
        (true, false) => RegionFragmentAction::Keep,
        (false, true) => RegionFragmentAction::KeepReversed,
        (false, false) | (true, true) => RegionFragmentAction::Discard,
    }
}

fn parameter_in_carrier(
    parameter: &BezierParameter2,
    carrier: &RegionCarrier,
    policy: &CurvePolicy,
) -> ExactCurveResult<bool> {
    parameter_between(parameter, &carrier.start, &carrier.end, policy)
}

fn parameter_between(
    parameter: &BezierParameter2,
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurvePolicy,
) -> ExactCurveResult<bool> {
    let lower = decided_parameter_cmp(parameter, start, policy)?;
    let upper = decided_parameter_cmp(parameter, end, policy)?;
    Ok(!lower.is_lt() && !upper.is_gt())
}

fn parameter_range_inside_carrier(
    start: &BezierParameter2,
    end: &BezierParameter2,
    carrier: &RegionCarrier,
    policy: &CurvePolicy,
) -> Result<bool, CurveError> {
    let start_cmp = start.cmp_by_refinement(&carrier.start, policy)?;
    let end_cmp = end.cmp_by_refinement(&carrier.end, policy)?;
    match (start_cmp, end_cmp) {
        (Classification::Decided(start_cmp), Classification::Decided(end_cmp)) => {
            Ok(!start_cmp.is_lt() && !end_cmp.is_gt())
        }
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            Err(CurveError::Topology(format!(
                "curved-region carrier ordering remained uncertain: {reason:?}"
            )))
        }
    }
}

fn ranges_intersect(
    range: &BezierParameterRange2,
    carrier: &RegionCarrier,
    policy: &CurvePolicy,
) -> ExactCurveResult<bool> {
    let (start, end) = ascending_range(range, policy)?;
    Ok(!decided_parameter_cmp(end, &carrier.start, policy)?.is_lt()
        && !decided_parameter_cmp(start, &carrier.end, policy)?.is_gt())
}

fn range_inside_carrier(
    range: &BezierParameterRange2,
    carrier: &RegionCarrier,
    policy: &CurvePolicy,
) -> ExactCurveResult<bool> {
    let (start, end) = ascending_range(range, policy)?;
    Ok(
        !decided_parameter_cmp(start, &carrier.start, policy)?.is_lt()
            && !decided_parameter_cmp(end, &carrier.end, policy)?.is_gt(),
    )
}

fn clip_identity_parameter_overlap(
    first_range: &BezierParameterRange2,
    second_range: &BezierParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
    first_carrier: &RegionCarrier,
    second_carrier: &RegionCarrier,
    policy: &CurvePolicy,
) -> ExactCurveResult<Option<(BezierParameterRange2, BezierParameterRange2)>> {
    if !identity_parameter_correspondence(
        first_range,
        second_range,
        orientation,
        first_carrier,
        second_carrier,
        policy,
    )? {
        return Ok(None);
    }

    let (overlap_start, overlap_end) = ascending_range(first_range, policy)?;
    let start = maximum_parameter(
        [overlap_start, &first_carrier.start, &second_carrier.start],
        policy,
    )?;
    let end = minimum_parameter(
        [overlap_end, &first_carrier.end, &second_carrier.end],
        policy,
    )?;
    match decided_parameter_cmp(&start, &end, policy)? {
        Ordering::Less => {}
        Ordering::Equal | Ordering::Greater => return Ok(None),
    }
    let range = BezierParameterRange2::new_validated(start, end);
    Ok(Some((range.clone(), range)))
}

fn identity_parameter_correspondence(
    first_range: &BezierParameterRange2,
    second_range: &BezierParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
    first_carrier: &RegionCarrier,
    second_carrier: &RegionCarrier,
    policy: &CurvePolicy,
) -> ExactCurveResult<bool> {
    if orientation != RationalBezierOverlapOrientation2::Same
        || first_carrier.curve != second_carrier.curve
    {
        return Ok(false);
    }
    Ok(
        decided_parameter_cmp(first_range.start(), second_range.start(), policy)?
            == Ordering::Equal
            && decided_parameter_cmp(first_range.end(), second_range.end(), policy)?
                == Ordering::Equal,
    )
}

fn maximum_parameter<const N: usize>(
    parameters: [&BezierParameter2; N],
    policy: &CurvePolicy,
) -> ExactCurveResult<BezierParameter2> {
    let mut maximum = parameters[0];
    for parameter in &parameters[1..] {
        if decided_parameter_cmp(parameter, maximum, policy)?.is_gt() {
            maximum = parameter;
        }
    }
    Ok(maximum.clone())
}

fn minimum_parameter<const N: usize>(
    parameters: [&BezierParameter2; N],
    policy: &CurvePolicy,
) -> ExactCurveResult<BezierParameter2> {
    let mut minimum = parameters[0];
    for parameter in &parameters[1..] {
        if decided_parameter_cmp(parameter, minimum, policy)?.is_lt() {
            minimum = parameter;
        }
    }
    Ok(minimum.clone())
}

fn range_contains_fragment(
    range: &BezierParameterRange2,
    fragment_start: &BezierParameter2,
    fragment_end: &BezierParameter2,
    policy: &CurvePolicy,
) -> ExactCurveResult<bool> {
    let (range_start, range_end) = ascending_range(range, policy)?;
    Ok(
        !decided_parameter_cmp(fragment_start, range_start, policy)?.is_lt()
            && !decided_parameter_cmp(fragment_end, range_end, policy)?.is_gt(),
    )
}

fn ascending_range<'a>(
    range: &'a BezierParameterRange2,
    policy: &CurvePolicy,
) -> ExactCurveResult<(&'a BezierParameter2, &'a BezierParameter2)> {
    match decided_parameter_cmp(range.start(), range.end(), policy)? {
        Ordering::Less => Ok((range.start(), range.end())),
        Ordering::Greater => Ok((range.end(), range.start())),
        Ordering::Equal => Err(ExactCurveError::invalid(
            CurveOperation2::Boolean,
            CurveFamily2::RationalBezier,
            None,
            CurveError::DegenerateOverlapRange,
        )),
    }
}

fn decided_parameter_cmp(
    first: &BezierParameter2,
    second: &BezierParameter2,
    policy: &CurvePolicy,
) -> ExactCurveResult<Ordering> {
    match first.cmp_by_refinement(second, policy).map_err(|cause| {
        ExactCurveError::invalid(
            CurveOperation2::Boolean,
            CurveFamily2::RationalBezier,
            None,
            cause,
        )
    })? {
        Classification::Decided(ordering) => Ok(ordering),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Boolean,
            CurveFamily2::RationalBezier,
            None,
            reason,
        )),
    }
}

fn fragment_range(fragment: &BezierSplitFragment2) -> (&BezierParameter2, &BezierParameter2) {
    match fragment {
        BezierSplitFragment2::Materialized { start, end, .. }
        | BezierSplitFragment2::AlgebraicEndpointImages { start, end, .. }
        | BezierSplitFragment2::Unresolved { start, end } => (start, end),
    }
}

const fn subcurve_family(curve: &BezierSubcurve2) -> CurveFamily2 {
    match curve {
        BezierSubcurve2::Quadratic(_) => CurveFamily2::QuadraticBezier,
        BezierSubcurve2::Cubic(_) => CurveFamily2::CubicBezier,
        BezierSubcurve2::RationalQuadratic(_) => CurveFamily2::RationalQuadraticBezier,
        BezierSubcurve2::Rational(_) => CurveFamily2::RationalBezier,
    }
}

fn source_of(carrier: &RegionCarrier) -> Option<crate::CurveSource2> {
    carrier
        .provenance
        .as_ref()
        .and_then(|provenance| provenance.span().source())
}

fn same_contact_point(
    first: &RationalBezierIntersectionPointEvidence2,
    second: &RationalBezierIntersectionPointEvidence2,
    policy: &CurvePolicy,
) -> bool {
    match (first, second) {
        (
            RationalBezierIntersectionPointEvidence2::Exact(first),
            RationalBezierIntersectionPointEvidence2::Exact(second),
        ) => crate::classify::is_zero(&first.distance_squared(second), policy) == Some(true),
        _ => first == second,
    }
}

fn empty_operand_result(
    first: &CurveRegion2,
    second: &CurveRegion2,
    operation: BooleanOp,
) -> ExactCurveResult<CurveRegion2> {
    let result = match operation {
        BooleanOp::Union | BooleanOp::Xor => {
            if first.is_empty() {
                second.clone()
            } else {
                first.clone()
            }
        }
        BooleanOp::Intersection => CurveRegion2::new(Vec::new()).map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Boolean, CurveFamily2::Line, None, cause)
        })?,
        BooleanOp::Difference => first.clone(),
    };
    Ok(result)
}

fn identical_operand_result(
    region: &CurveRegion2,
    operation: BooleanOp,
) -> ExactCurveResult<CurveRegion2> {
    match operation {
        BooleanOp::Union | BooleanOp::Intersection => Ok(region.clone()),
        BooleanOp::Difference | BooleanOp::Xor => CurveRegion2::new(Vec::new()).map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Boolean, CurveFamily2::Line, None, cause)
        }),
    }
}

const fn boolean_operation_index(operation: BooleanOp) -> usize {
    match operation {
        BooleanOp::Union => 0,
        BooleanOp::Intersection => 1,
        BooleanOp::Difference => 2,
        BooleanOp::Xor => 3,
    }
}
