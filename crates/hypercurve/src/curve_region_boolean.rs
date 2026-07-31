//! Immediate exact Booleans over curved regions.

use std::cmp::Ordering;
use std::collections::HashMap;
use std::sync::{Arc, OnceLock};

use crate::bezier_moment::RationalQuadraticAreaIntegralCache;
use crate::bezier_tangent_order::algebraic_endpoint_tangents_are_transverse;
use crate::curve_intersection::{CurveIntersectionBatchCache, CurveIntersectionContext};
use crate::policy::resolve_certified_operation;
use crate::{
    Aabb2, BezierArrangementFragment2, BezierArrangementGraph2, BezierEndpointTangentImage2,
    BezierParameter2, BezierParameterRange2, BezierSplitFragment2, BezierSubcurve2, BooleanOp,
    Classification, Curve2, CurveContext, CurveError, CurveFamily2, CurveIntersectionContact2,
    CurveIntersectionOverlap2, CurveIntersectionPairBlocker2, CurveOperation2, CurveOutcome,
    CurvePathBooleanOperand2, CurveRegion2, ExactCurveError, ExactCurveResult, FillRule,
    RationalBezier2, RationalBezierIntersectionPointEvidence2, RationalBezierOverlapOrientation2,
    RegionPointLocation, UncertaintyReason,
};

/// Stable identity for one retained region-boundary carrier.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionCarrierRef2 {
    carrier_index: usize,
    operand: CurvePathBooleanOperand2,
    loop_index: usize,
    fragment_index: usize,
    family: CurveFamily2,
}

/// One exact contact between retained carriers from two curved regions.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionIntersectionContact2 {
    first: CurveRegionCarrierRef2,
    second: CurveRegionCarrierRef2,
    contact: CurveIntersectionContact2,
}

/// One certified positive-length shared span between two curved regions.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionIntersectionOverlap2 {
    first: CurveRegionCarrierRef2,
    second: CurveRegionCarrierRef2,
    source: CurveIntersectionOverlap2,
    first_range: BezierParameterRange2,
    second_range: BezierParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
}

/// One incomplete retained carrier pair in a curved-region intersection result.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionIntersectionBlocker2 {
    first: CurveRegionCarrierRef2,
    second: CurveRegionCarrierRef2,
    blocker: CurveIntersectionPairBlocker2,
}

/// Clone-shared exact contact, overlap, and blocker result for two curved regions.
#[derive(Clone, Debug)]
pub struct CurveRegionIntersectionResult2 {
    data: Arc<CurveRegionIntersectionResultData>,
}

#[derive(Debug)]
struct CurveRegionIntersectionResultData {
    authored_carrier_pair_count: usize,
    candidate_carrier_pair_count: usize,
    contacts: Arc<[CurveRegionIntersectionContact2]>,
    overlaps: Arc<[CurveRegionIntersectionOverlap2]>,
    blockers: Arc<[CurveRegionIntersectionBlocker2]>,
}

/// The four exact regularized Boolean results for one region pair.
#[derive(Clone, Debug)]
pub struct CurveRegionBooleanResults2 {
    regions: Box<[CurveRegion2; 4]>,
    authored_carrier_pair_count: usize,
    candidate_carrier_pair_count: usize,
    topology_fragment_count: usize,
    topology_point_classification_count: usize,
}

#[derive(Debug)]
struct CurveRegionBooleanContext<'a> {
    data: CurveRegionBooleanContextData<'a>,
}

#[derive(Debug)]
struct CurveRegionBooleanContextData<'a> {
    first: &'a CurveRegion2,
    second: &'a CurveRegion2,
    policy: CurveContext,
    carriers: Vec<RegionCarrier>,
    first_carrier_count: usize,
    authored_carrier_pair_count: usize,
    pairs: Vec<RegionCarrierPair>,
}

#[derive(Clone, Debug)]
struct RegionCarrier {
    operand: CurvePathBooleanOperand2,
    loop_index: usize,
    fragment_index: usize,
    family: CurveFamily2,
    curve: BezierSubcurve2,
    start: BezierParameter2,
    end: BezierParameter2,
    reversed: bool,
    filled_side_is_left: bool,
    image_is_injective: OnceLock<bool>,
    bounds: OnceLock<Classification<Aabb2>>,
}

#[derive(Debug)]
struct RegionCarrierPair {
    first_carrier_index: usize,
    second_carrier_index: usize,
    context: CurveIntersectionContext,
}

#[derive(Clone, Debug)]
struct CarrierEvent {
    parameter: BezierParameter2,
    topology_vertex: Option<usize>,
}

#[derive(Clone, Debug)]
struct ContactVertex {
    point: RationalBezierIntersectionPointEvidence2,
    topology_vertex: usize,
    carrier_indices: [usize; 2],
    parameters: [BezierParameter2; 2],
}

#[derive(Clone, Debug)]
struct CarrierOverlap {
    first_carrier_index: usize,
    second_carrier_index: usize,
    first_range: BezierParameterRange2,
    second_range: BezierParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
}

#[derive(Clone, Copy, Debug)]
struct TransitionContactCandidate {
    first_carrier: usize,
    second_carrier: usize,
    certified_transverse: bool,
}

#[derive(Clone, Debug)]
struct SplitCarrierFragment {
    fragment: BezierSplitFragment2,
    start_topology_vertex: Option<usize>,
    end_topology_vertex: Option<usize>,
}

#[derive(Clone, Debug)]
struct ClassifiedSplitCarrierFragment {
    split: SplitCarrierFragment,
    location: RegionPointLocation,
}

#[derive(Clone, Copy, Debug)]
struct BooleanArrangementFragmentDirection {
    carrier_index: usize,
    follows_carrier: bool,
}

#[derive(Clone, Debug)]
struct CurveRegionBooleanTopology {
    split_fragments: Vec<Vec<ClassifiedSplitCarrierFragment>>,
    overlaps: Vec<CarrierOverlap>,
    transverse_contacts: HashMap<usize, TransitionContactCandidate>,
    point_classification_count: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RegionFragmentAction {
    Discard,
    Keep,
    KeepReversed,
}

impl CurveRegionCarrierRef2 {
    /// Returns the flattened carrier index in the retained pair.
    pub const fn carrier_index(&self) -> usize {
        self.carrier_index
    }

    /// Returns the region operand that owns this carrier.
    pub const fn operand(&self) -> CurvePathBooleanOperand2 {
        self.operand
    }

    /// Returns the retained boundary-loop index in its operand.
    pub const fn loop_index(&self) -> usize {
        self.loop_index
    }

    /// Returns the retained fragment index in its boundary loop.
    pub const fn fragment_index(&self) -> usize {
        self.fragment_index
    }

    /// Returns the exact carrier family used by intersection dispatch.
    pub const fn family(&self) -> CurveFamily2 {
        self.family
    }
}

impl CurveRegionIntersectionContact2 {
    /// Returns the first-region carrier identity.
    pub const fn first(&self) -> &CurveRegionCarrierRef2 {
        &self.first
    }

    /// Returns the second-region carrier identity.
    pub const fn second(&self) -> &CurveRegionCarrierRef2 {
        &self.second
    }

    /// Returns exact local/source parameter and point evidence.
    pub const fn contact(&self) -> &CurveIntersectionContact2 {
        &self.contact
    }
}

impl CurveRegionIntersectionOverlap2 {
    /// Returns the first-region carrier identity.
    pub const fn first(&self) -> &CurveRegionCarrierRef2 {
        &self.first
    }

    /// Returns the second-region carrier identity.
    pub const fn second(&self) -> &CurveRegionCarrierRef2 {
        &self.second
    }

    /// Returns the source-curve overlap before carrier-range clipping.
    pub const fn source(&self) -> &CurveIntersectionOverlap2 {
        &self.source
    }

    /// Returns the exact overlap range clipped to the first retained carrier.
    pub const fn first_range(&self) -> &BezierParameterRange2 {
        &self.first_range
    }

    /// Returns the exact overlap range clipped to the second retained carrier.
    pub const fn second_range(&self) -> &BezierParameterRange2 {
        &self.second_range
    }

    /// Returns relative source-curve traversal orientation.
    pub const fn orientation(&self) -> RationalBezierOverlapOrientation2 {
        self.orientation
    }
}

impl CurveRegionIntersectionBlocker2 {
    /// Returns the first-region carrier identity.
    pub const fn first(&self) -> &CurveRegionCarrierRef2 {
        &self.first
    }

    /// Returns the second-region carrier identity.
    pub const fn second(&self) -> &CurveRegionCarrierRef2 {
        &self.second
    }

    /// Returns retained incomplete intersection evidence.
    pub const fn blocker(&self) -> &CurveIntersectionPairBlocker2 {
        &self.blocker
    }
}

impl CurveRegionIntersectionResult2 {
    /// Returns the full Cartesian carrier-pair count before broad-phase pruning.
    pub fn authored_carrier_pair_count(&self) -> usize {
        self.data.authored_carrier_pair_count
    }

    /// Returns the carrier-pair count retained after certified broad-phase pruning.
    pub fn candidate_carrier_pair_count(&self) -> usize {
        self.data.candidate_carrier_pair_count
    }

    /// Returns exact contacts clipped to both retained carrier ranges.
    pub fn contacts(&self) -> &[CurveRegionIntersectionContact2] {
        &self.data.contacts
    }

    /// Returns exact positive-length overlaps clipped to both carrier ranges.
    pub fn overlaps(&self) -> &[CurveRegionIntersectionOverlap2] {
        &self.data.overlaps
    }

    /// Returns incomplete carrier pairs with retained exact evidence.
    pub fn blockers(&self) -> &[CurveRegionIntersectionBlocker2] {
        &self.data.blockers
    }

    /// Returns true when every retained carrier pair completed exact replay.
    pub fn is_complete(&self) -> bool {
        self.data.blockers.is_empty()
    }

    /// Returns true when complete replay found no contact or overlap.
    pub fn is_disjoint(&self) -> bool {
        self.is_complete() && self.data.contacts.is_empty() && self.data.overlaps.is_empty()
    }
}

impl CurveRegionBooleanResults2 {
    /// Returns the exact result for one Boolean operation.
    pub fn region(&self, operation: BooleanOp) -> &CurveRegion2 {
        &self.regions[boolean_operation_index(operation)]
    }

    /// Returns the exact union.
    pub const fn union(&self) -> &CurveRegion2 {
        &self.regions[0]
    }

    /// Returns the exact intersection.
    pub const fn intersection(&self) -> &CurveRegion2 {
        &self.regions[1]
    }

    /// Returns the exact first-minus-second difference.
    pub const fn difference(&self) -> &CurveRegion2 {
        &self.regions[2]
    }

    /// Returns the exact symmetric difference.
    pub const fn xor(&self) -> &CurveRegion2 {
        &self.regions[3]
    }

    /// Returns the Cartesian carrier-pair count before certified broad-phase filtering.
    pub const fn authored_carrier_pair_count(&self) -> usize {
        self.authored_carrier_pair_count
    }

    /// Returns the number of general cross-region pairs retained by the
    /// certified broad phase, or zero when native topology completed the batch.
    pub const fn candidate_carrier_pair_count(&self) -> usize {
        self.candidate_carrier_pair_count
    }

    /// Returns the number of split fragments shared by all four operations.
    pub const fn topology_fragment_count(&self) -> usize {
        self.topology_fragment_count
    }

    /// Returns the number of exact representative-point classifications shared
    /// by all four operations.
    pub const fn topology_point_classification_count(&self) -> usize {
        self.topology_point_classification_count
    }
}

impl CurveRegion2 {
    /// Computes one exact regularized Boolean immediately.
    pub fn boolean_region(
        &self,
        other: &Self,
        operation: BooleanOp,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            self.boolean_region_raw(other, operation, attempt)
        })
    }

    pub(crate) fn boolean_region_raw(
        &self,
        other: &Self,
        operation: BooleanOp,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if let Some(region) =
            boolean_region_without_general_context(self, other, operation, policy)?
        {
            return Ok(region);
        }
        CurveRegionBooleanContext::try_new(self, other, policy)?
            .build_boolean_region(operation, None)
    }

    /// Computes all four exact regularized Booleans immediately while sharing
    /// intersection and split-topology work within this call.
    pub fn boolean_regions(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveRegionBooleanResults2>> {
        resolve_certified_operation(policy, |attempt| self.boolean_regions_raw(other, attempt))
    }

    pub(crate) fn boolean_regions_raw(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveRegionBooleanResults2> {
        let operations = [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ];
        // Four separate native line or arc Booleans repeat the same support
        // intersections and splits. Degree-elevated affine carriers retain
        // native line dispatch, while retained circular-conic carriers share
        // one exact support relation per circle pair inside the canonical
        // arrangement. These specialized predicates make shared topology the
        // batch authority without discarding the single-operation fast paths.
        let shared_specialized_arrangement = !self.is_empty()
            && !other.is_empty()
            && self != other
            && region_has_only_affine_or_circular_conic_carriers(self)
            && region_has_only_affine_or_circular_conic_carriers(other);
        let immediate = if shared_specialized_arrangement {
            [None, None, None, None]
        } else {
            [
                boolean_region_without_general_context(self, other, operations[0], policy)?,
                boolean_region_without_general_context(self, other, operations[1], policy)?,
                boolean_region_without_general_context(self, other, operations[2], policy)?,
                boolean_region_without_general_context(self, other, operations[3], policy)?,
            ]
        };
        if immediate.iter().all(Option::is_some) {
            return Ok(CurveRegionBooleanResults2 {
                regions: Box::new(
                    immediate
                        .map(|region| region.expect("all immediate Boolean results were checked")),
                ),
                authored_carrier_pair_count: region_carrier_count(self)
                    .saturating_mul(region_carrier_count(other)),
                candidate_carrier_pair_count: 0,
                topology_fragment_count: 0,
                topology_point_classification_count: 0,
            });
        }
        CurveRegionBooleanContext::try_new(self, other, policy)?.build_boolean_regions(immediate)
    }

    /// Collects exact contacts and overlaps against another region immediately.
    pub fn intersect_region(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveRegionIntersectionResult2>> {
        resolve_certified_operation(policy, |attempt| self.intersect_region_raw(other, attempt))
    }

    pub(crate) fn intersect_region_raw(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveRegionIntersectionResult2> {
        CurveRegionBooleanContext::try_new(self, other, policy)?.build_intersection_evidence()
    }
}

impl<'a> CurveRegionBooleanContext<'a> {
    fn try_new(
        first: &'a CurveRegion2,
        second: &'a CurveRegion2,
        policy: &'a CurveContext,
    ) -> ExactCurveResult<Self> {
        let mut rational_quadratic_area_cache = RationalQuadraticAreaIntegralCache::default();
        let first_carriers = build_region_carriers(
            first,
            CurvePathBooleanOperand2::First,
            policy,
            &mut rational_quadratic_area_cache,
        )?;
        let first_carrier_count = first_carriers.len();
        let mut carriers = first_carriers;
        carriers.extend(build_region_carriers(
            second,
            CurvePathBooleanOperand2::Second,
            policy,
            &mut rational_quadratic_area_cache,
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
        let mut intersection_cache = CurveIntersectionBatchCache::default();
        for first_carrier_index in 0..first_carrier_count {
            for second_carrier_index in first_carrier_count..carriers.len() {
                let first_curve = &curves[first_carrier_index];
                let second_curve = &curves[second_carrier_index];
                if carrier_bounds_decided_disjoint(first_curve, second_curve, policy) {
                    continue;
                }
                pairs.push(RegionCarrierPair {
                    first_carrier_index,
                    second_carrier_index,
                    context: CurveIntersectionContext::try_new_with_batch_cache(
                        first_curve,
                        second_curve,
                        policy,
                        &mut intersection_cache,
                    )?,
                });
            }
        }

        Ok(Self {
            data: CurveRegionBooleanContextData {
                first,
                second,
                policy: *policy,
                carriers,
                first_carrier_count,
                authored_carrier_pair_count,
                pairs,
            },
        })
    }

    fn build_intersection_evidence(&self) -> ExactCurveResult<CurveRegionIntersectionResult2> {
        let mut contacts = Vec::new();
        let mut overlaps = Vec::new();
        let mut blockers = Vec::new();
        for pair in &self.data.pairs {
            let result = pair.context.result_view()?;
            let first = self.carrier_ref(pair.first_carrier_index);
            let second = self.carrier_ref(pair.second_carrier_index);
            blockers.extend(result.blockers().iter().cloned().map(|blocker| {
                CurveRegionIntersectionBlocker2 {
                    first: first.clone(),
                    second: second.clone(),
                    blocker,
                }
            }));
            for contact in result.contacts() {
                if parameter_in_carrier(
                    contact.first().local_parameter(),
                    &self.data.carriers[pair.first_carrier_index],
                    &self.data.policy,
                )? && parameter_in_carrier(
                    contact.second().local_parameter(),
                    &self.data.carriers[pair.second_carrier_index],
                    &self.data.policy,
                )? {
                    contacts.push(CurveRegionIntersectionContact2 {
                        first: first.clone(),
                        second: second.clone(),
                        contact: contact.clone(),
                    });
                }
            }
            for overlap in result.overlaps() {
                let Some((first_range, second_range)) =
                    self.clipped_overlap_ranges(pair, overlap)?
                else {
                    continue;
                };
                overlaps.push(CurveRegionIntersectionOverlap2 {
                    first: first.clone(),
                    second: second.clone(),
                    source: overlap.clone(),
                    first_range,
                    second_range,
                    orientation: overlap.orientation(),
                });
            }
        }
        Ok(CurveRegionIntersectionResult2 {
            data: Arc::new(CurveRegionIntersectionResultData {
                authored_carrier_pair_count: self.data.authored_carrier_pair_count,
                candidate_carrier_pair_count: self.data.pairs.len(),
                contacts: contacts.into(),
                overlaps: overlaps.into(),
                blockers: blockers.into(),
            }),
        })
    }

    fn carrier_ref(&self, carrier_index: usize) -> CurveRegionCarrierRef2 {
        let carrier = &self.data.carriers[carrier_index];
        CurveRegionCarrierRef2 {
            carrier_index,
            operand: carrier.operand,
            loop_index: carrier.loop_index,
            fragment_index: carrier.fragment_index,
            family: carrier.family,
        }
    }

    fn clipped_overlap_ranges(
        &self,
        pair: &RegionCarrierPair,
        overlap: &CurveIntersectionOverlap2,
    ) -> ExactCurveResult<Option<(BezierParameterRange2, BezierParameterRange2)>> {
        let first_carrier = &self.data.carriers[pair.first_carrier_index];
        let second_carrier = &self.data.carriers[pair.second_carrier_index];
        let first_intersects =
            ranges_intersect(overlap.first_range(), first_carrier, &self.data.policy)?;
        let second_intersects =
            ranges_intersect(overlap.second_range(), second_carrier, &self.data.policy)?;
        if !first_intersects && !second_intersects {
            return Ok(None);
        }
        if first_intersects == second_intersects
            && range_inside_carrier(overlap.first_range(), first_carrier, &self.data.policy)?
            && range_inside_carrier(overlap.second_range(), second_carrier, &self.data.policy)?
        {
            return Ok(Some((
                overlap.first_range().clone(),
                overlap.second_range().clone(),
            )));
        }
        if let Some(ranges) = clip_identity_parameter_overlap(
            overlap.first_range(),
            overlap.second_range(),
            overlap.orientation(),
            first_carrier,
            second_carrier,
            &self.data.policy,
        )? {
            return Ok(Some(ranges));
        }
        if identity_parameter_correspondence(
            overlap.first_range(),
            overlap.second_range(),
            overlap.orientation(),
            first_carrier,
            second_carrier,
            &self.data.policy,
        )? {
            return Ok(None);
        }
        Err(self.blocked(pair.first_carrier_index, UncertaintyReason::Unsupported))
    }

    fn build_boolean_topology(&self) -> ExactCurveResult<CurveRegionBooleanTopology> {
        let mut events = vec![Vec::new(); self.data.carriers.len()];
        let mut contact_points = Vec::<ContactVertex>::new();
        let mut next_topology_vertex = 0_usize;
        let mut contact_vertex_counts = Vec::<usize>::new();
        let mut transition_candidates = Vec::<Option<TransitionContactCandidate>>::new();
        let mut reclassification_vertices = Vec::<bool>::new();
        seed_loop_topology_vertices(
            &self.data.carriers,
            &mut events,
            &mut next_topology_vertex,
            &self.data.policy,
        )?;
        contact_vertex_counts.resize(next_topology_vertex, 0);
        transition_candidates.resize(next_topology_vertex, None);
        reclassification_vertices.resize(next_topology_vertex, false);
        let mut overlaps = Vec::new();
        for pair in &self.data.pairs {
            let result = pair.context.result_view()?;
            if let Some(blocker) = result.blockers().first() {
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

            for contact in result.contacts() {
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
                let mut matching_contact_vertex = None;
                for existing in &contact_points {
                    if contacts_decided_distinct_from_carriers(
                        existing,
                        [pair.first_carrier_index, pair.second_carrier_index],
                        [first_parameter, second_parameter],
                        &self.data.carriers,
                        &self.data.policy,
                    )? {
                        continue;
                    }
                    match same_contact_point(&existing.point, contact.point(), &self.data.policy) {
                        Classification::Decided(true) => {
                            matching_contact_vertex = Some(existing.topology_vertex);
                            break;
                        }
                        Classification::Decided(false) => {}
                        Classification::Uncertain(reason) => {
                            return Err(self.blocked(pair.first_carrier_index, reason));
                        }
                    }
                }
                let topology_vertex = match (first_existing, second_existing) {
                    (Some(first), Some(second)) if first != second => {
                        replace_topology_vertex(&mut events, &mut contact_points, second, first);
                        contact_vertex_counts[first] += contact_vertex_counts[second];
                        contact_vertex_counts[second] = 0;
                        reclassification_vertices[first] |= reclassification_vertices[second];
                        reclassification_vertices[second] = false;
                        transition_candidates[first] = None;
                        transition_candidates[second] = None;
                        first
                    }
                    (Some(vertex), _) | (_, Some(vertex)) => vertex,
                    (None, None) => matching_contact_vertex.unwrap_or_else(|| {
                        let vertex = next_topology_vertex;
                        next_topology_vertex += 1;
                        vertex
                    }),
                };
                if matching_contact_vertex.is_none() {
                    contact_points.push(ContactVertex {
                        point: contact.point().clone(),
                        topology_vertex,
                        carrier_indices: [pair.first_carrier_index, pair.second_carrier_index],
                        parameters: [first_parameter.clone(), second_parameter.clone()],
                    });
                }
                if contact_vertex_counts.len() <= topology_vertex {
                    contact_vertex_counts.resize(topology_vertex + 1, 0);
                    transition_candidates.resize(topology_vertex + 1, None);
                    reclassification_vertices.resize(topology_vertex + 1, false);
                }
                contact_vertex_counts[topology_vertex] += 1;
                reclassification_vertices[topology_vertex] = true;
                transition_candidates[topology_vertex] = if contact_vertex_counts[topology_vertex]
                    == 1
                    && parameter_strictly_inside_carrier(
                        first_parameter,
                        &self.data.carriers[pair.first_carrier_index],
                        &self.data.policy,
                    )
                    && parameter_strictly_inside_carrier(
                        second_parameter,
                        &self.data.carriers[pair.second_carrier_index],
                        &self.data.policy,
                    ) {
                    Some(TransitionContactCandidate {
                        first_carrier: pair.first_carrier_index,
                        second_carrier: pair.second_carrier_index,
                        certified_transverse: contact.is_certified_transverse(),
                    })
                } else {
                    None
                };
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

            for overlap in result.overlaps() {
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
        for overlap in &overlaps {
            for (carrier_index, range) in [
                (overlap.first_carrier_index, &overlap.first_range),
                (overlap.second_carrier_index, &overlap.second_range),
            ] {
                for parameter in [range.start(), range.end()] {
                    if let Some(vertex) =
                        existing_event_vertex(&events[carrier_index], parameter, &self.data.policy)?
                        && let Some(candidate) = transition_candidates.get_mut(vertex)
                    {
                        *candidate = None;
                        reclassification_vertices[vertex] = true;
                    }
                }
            }
        }

        let split_fragments = self
            .data
            .carriers
            .iter()
            .enumerate()
            .map(|(carrier_index, carrier)| {
                split_carrier(carrier, &events[carrier_index], &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))
            })
            .collect::<ExactCurveResult<Vec<_>>>()?;
        let transverse_vertices = certified_transverse_contact_vertices(
            &split_fragments,
            &transition_candidates,
            &self.data.policy,
        );
        let transverse_contacts = transition_candidates
            .into_iter()
            .zip(&transverse_vertices)
            .enumerate()
            .filter_map(|(vertex, (candidate, transverse))| {
                if *transverse {
                    candidate.map(|candidate| (vertex, candidate))
                } else {
                    None
                }
            })
            .collect();
        let mut classified_split_fragments = Vec::with_capacity(split_fragments.len());
        let mut point_classification_count = 0_usize;
        let mut previous = None::<(
            CurvePathBooleanOperand2,
            usize,
            Option<usize>,
            RegionPointLocation,
        )>;
        for (carrier_index, fragments) in split_fragments.into_iter().enumerate() {
            let carrier = &self.data.carriers[carrier_index];
            let mut classified =
                Vec::<ClassifiedSplitCarrierFragment>::with_capacity(fragments.len());
            for split in fragments {
                let propagated =
                    previous.and_then(|(operand, loop_index, end_topology_vertex, location)| {
                        (operand == carrier.operand
                            && loop_index == carrier.loop_index
                            && end_topology_vertex == split.start_topology_vertex)
                            .then_some(split.start_topology_vertex)
                            .flatten()
                            .and_then(|vertex| {
                                if transverse_vertices.get(vertex).copied().unwrap_or(false) {
                                    toggled_region_location(location)
                                } else if !reclassification_vertices
                                    .get(vertex)
                                    .copied()
                                    .unwrap_or(false)
                                {
                                    Some(location)
                                } else {
                                    None
                                }
                            })
                    });
                let location = match propagated {
                    Some(location) => location,
                    None => {
                        let (start, end) = fragment_range(&split.fragment);
                        let mut shared = false;
                        for overlap in &overlaps {
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
                                shared = true;
                                break;
                            }
                        }
                        if shared {
                            RegionPointLocation::Boundary
                        } else {
                            point_classification_count += 1;
                            self.fragment_location(carrier_index, &split.fragment)?
                        }
                    }
                };
                previous = Some((
                    carrier.operand,
                    carrier.loop_index,
                    split.end_topology_vertex,
                    location,
                ));
                classified.push(ClassifiedSplitCarrierFragment { split, location });
            }
            classified_split_fragments.push(classified);
        }
        Ok(CurveRegionBooleanTopology {
            split_fragments: classified_split_fragments,
            overlaps,
            transverse_contacts,
            point_classification_count,
        })
    }

    fn build_boolean_regions(
        &self,
        immediate: [Option<CurveRegion2>; 4],
    ) -> ExactCurveResult<CurveRegionBooleanResults2> {
        let operations = [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ];
        let topology = self.build_boolean_topology()?;
        let [union, intersection, difference, xor] = immediate;
        let resolve = |region: Option<CurveRegion2>,
                       operation: BooleanOp|
         -> ExactCurveResult<CurveRegion2> {
            match region {
                Some(region) => Ok(region),
                None => self.build_boolean_region_from_topology(operation, &topology),
            }
        };
        let regions = [
            resolve(union, operations[0])?,
            resolve(intersection, operations[1])?,
            resolve(difference, operations[2])?,
            match resolve(xor, operations[3]) {
                Ok(region) => region,
                Err(ExactCurveError::Blocked(_)) => self.build_xor_from_exact_set_identity()?,
                Err(error) => return Err(error),
            },
        ];
        let topology_fragment_count = topology.split_fragments.iter().map(Vec::len).sum();
        let topology_point_classification_count = topology.point_classification_count;
        Ok(CurveRegionBooleanResults2 {
            regions: Box::new(regions),
            authored_carrier_pair_count: self.data.authored_carrier_pair_count,
            candidate_carrier_pair_count: self.data.pairs.len(),
            topology_fragment_count,
            topology_point_classification_count,
        })
    }

    fn build_boolean_region(
        &self,
        operation: BooleanOp,
        topology: Option<&CurveRegionBooleanTopology>,
    ) -> ExactCurveResult<CurveRegion2> {
        let topology_storage;
        let topology = match topology {
            Some(topology) => topology,
            None => {
                topology_storage = self.build_boolean_topology()?;
                &topology_storage
            }
        };
        match self.build_boolean_region_from_topology(operation, topology) {
            Ok(region) => Ok(region),
            Err(ExactCurveError::Blocked(_)) if operation == BooleanOp::Xor => {
                self.build_xor_from_exact_set_identity()
            }
            Err(error) => Err(error),
        }
    }

    fn build_xor_from_exact_set_identity(&self) -> ExactCurveResult<CurveRegion2> {
        let union = self.data.first.boolean_region_raw(
            self.data.second,
            BooleanOp::Union,
            &self.data.policy,
        )?;
        let intersection = self.data.first.boolean_region_raw(
            self.data.second,
            BooleanOp::Intersection,
            &self.data.policy,
        )?;
        if let Ok(xor) =
            union.boolean_region_raw(&intersection, BooleanOp::Difference, &self.data.policy)
        {
            return Ok(xor);
        }
        let mut filled_sides = match union.filled_side_is_left_raw(&self.data.policy) {
            Ok(Classification::Decided(sides)) => sides.to_vec(),
            Ok(Classification::Uncertain(reason)) => return Err(self.blocked(0, reason)),
            Err(cause) => return Err(self.invalid(0, cause)),
        };
        let intersection_filled_sides =
            match intersection.filled_side_is_left_raw(&self.data.policy) {
                Ok(Classification::Decided(sides)) => sides,
                Ok(Classification::Uncertain(reason)) => return Err(self.blocked(0, reason)),
                Err(cause) => return Err(self.invalid(0, cause)),
            };
        filled_sides.extend(intersection_filled_sides.iter().map(|side| !side));
        // XOR is union with the intersection's filled side removed. Retain the
        // two exact boundary sets directly when a second Boolean traversal
        // cannot materialize that difference.
        let mut union_loops = union
            .into_boundary_loops()
            .into_iter()
            // Both derived regions reuse the operands' source records. Strip
            // those records before combining them into one independent region.
            .map(crate::CurveRegionBoundaryLoop2::without_arrangement_sources)
            .collect::<Vec<_>>();
        union_loops.extend(
            intersection
                .into_boundary_loops()
                .into_iter()
                .map(crate::CurveRegionBoundaryLoop2::without_arrangement_sources),
        );
        CurveRegion2::new(union_loops)
            .and_then(|region| region.with_certified_filled_side_is_left(filled_sides))
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Boolean,
                    CurveFamily2::RationalBezier,
                    cause,
                )
            })
    }

    fn build_boolean_region_from_topology(
        &self,
        operation: BooleanOp,
        topology: &CurveRegionBooleanTopology,
    ) -> ExactCurveResult<CurveRegion2> {
        let mut arrangement_fragments = Vec::new();
        let mut arrangement_directions = Vec::new();
        for carrier_index in 0..self.data.carriers.len() {
            for (split_fragment_index, classified) in
                topology.split_fragments[carrier_index].iter().enumerate()
            {
                let split = &classified.split;
                let action = self.fragment_action(
                    carrier_index,
                    &split.fragment,
                    classified.location,
                    &topology.overlaps,
                    operation,
                )?;
                if action == RegionFragmentAction::Discard {
                    continue;
                }
                let fragment = match action {
                    RegionFragmentAction::Keep => split.fragment.clone(),
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
                arrangement_directions.push(BooleanArrangementFragmentDirection {
                    carrier_index,
                    follows_carrier: action == RegionFragmentAction::Keep,
                });
                arrangement_fragments.push(
                    BezierArrangementFragment2::new(carrier_index, split_fragment_index, fragment)
                        .with_topology_vertices(start_topology_vertex, end_topology_vertex),
                );
            }
        }

        let affine_line_output = !arrangement_fragments.is_empty()
            && arrangement_fragments
                .iter()
                .all(|fragment| split_fragment_is_affine_line(fragment.fragment()));
        let graph = BezierArrangementGraph2::from_certified_fragments(arrangement_fragments);
        let certified_successors = certified_boolean_successors(
            &graph,
            &arrangement_directions,
            topology,
            &self.data.carriers,
        );
        let primary = graph
            .traverse_retained_with_certified_successors(&certified_successors, &self.data.policy);
        // Coincident or multi-valent retained boundaries can make the
        // smallest-turn walk ambiguous even when result-side evidence is
        // complete. Retry with the same certified successor set interpreted
        // as filled-left face half-edges for every operation.
        let traversal = match primary {
            Classification::Decided(traversal) => traversal,
            Classification::Uncertain(_) => {
                match graph.traverse_retained_filled_left_faces_with_certified_successors(
                    &certified_successors,
                    &self.data.policy,
                ) {
                    Classification::Decided(traversal) => traversal,
                    Classification::Uncertain(_) => {
                        match graph.traverse_retained_with_tangent_order(&self.data.policy) {
                            Classification::Decided(traversal) => traversal,
                            Classification::Uncertain(reason) => {
                                return Err(self.blocked(0, reason));
                            }
                        }
                    }
                }
            }
        };
        let mut region =
            match CurveRegion2::from_certified_retained_arrangement_traversal(&graph, &traversal) {
                Classification::Decided(region) => region,
                Classification::Uncertain(reason) => return Err(self.blocked(0, reason)),
            };
        region = region
            .with_certified_filled_side_is_left(vec![true; traversal.chains().len()])
            .map_err(|cause| self.invalid(0, cause))?;
        if affine_line_output {
            self.compact_affine_line_result(region)
        } else {
            Ok(region)
        }
    }

    fn compact_affine_line_result(&self, region: CurveRegion2) -> ExactCurveResult<CurveRegion2> {
        let mut material = Vec::new();
        let mut holes = Vec::new();
        let mut reduced_fragment_count = false;
        for boundary in region.boundary_loops() {
            let segments = boundary
                .fragments()
                .iter()
                .map(|fragment| {
                    let BezierSplitFragment2::Materialized {
                        curve: BezierSubcurve2::Quadratic(curve),
                        ..
                    } = fragment
                    else {
                        unreachable!("affine-line result was checked before compaction")
                    };
                    crate::Segment2::Line(
                        curve
                            .retained_exact_line_image()
                            .expect("affine-line result retains its exact line image")
                            .clone(),
                    )
                })
                .collect();
            let contour =
                crate::Contour2::from_validated_closed_segments(segments, FillRule::NonZero);
            let contour = match contour
                .merge_adjacent_collinear_lines(&self.data.policy)
                .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(contour) => contour,
                Classification::Uncertain(reason) => return Err(self.blocked(0, reason)),
            };
            reduced_fragment_count |= contour.len() < boundary.len();
            let area = contour
                .signed_area()
                .map_err(|cause| self.invalid(0, cause))?
                .expect("line contours always have an exact signed area");
            match crate::classify::compare_reals(&area, &crate::Real::zero(), &self.data.policy) {
                Some(Ordering::Greater) => material.push(contour),
                Some(Ordering::Less) => holes.push(contour),
                Some(Ordering::Equal) => {
                    return Err(self.invalid(
                        0,
                        CurveError::Topology(
                            "regularized Boolean emitted a zero-area affine line loop".into(),
                        ),
                    ));
                }
                None => return Err(self.blocked(0, UncertaintyReason::RealSign)),
            }
        }
        if !reduced_fragment_count {
            return Ok(region);
        }
        CurveRegion2::from_certified_oriented_line_contours(material, holes)
            .map_err(|cause| self.invalid(0, cause))
    }

    fn fragment_location(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
    ) -> ExactCurveResult<RegionPointLocation> {
        let carrier = &self.data.carriers[carrier_index];
        let (start, end) = fragment_range(fragment);
        let parameter = match start
            .strict_rational_between_ordered(end, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };
        let representative = match carrier.curve.point_at(&parameter, &self.data.policy) {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };
        let other = match carrier.operand {
            CurvePathBooleanOperand2::First => &self.data.second,
            CurvePathBooleanOperand2::Second => &self.data.first,
        };
        match other
            .classify_point_raw(&representative, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(location) => Ok(location),
            Classification::Uncertain(reason) => Err(self.blocked(carrier_index, reason)),
        }
    }

    fn fragment_action(
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

    fn shared_fragment_action(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
        overlaps: &[CarrierOverlap],
        operation: BooleanOp,
    ) -> ExactCurveResult<RegionFragmentAction> {
        let (start, end) = fragment_range(fragment);
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
        ExactCurveError::invalid(CurveOperation2::Boolean, carrier.family, cause)
    }

    fn blocked(&self, carrier_index: usize, reason: UncertaintyReason) -> ExactCurveError {
        let carrier = &self.data.carriers[carrier_index];
        ExactCurveError::blocked(CurveOperation2::Boolean, carrier.family, reason)
    }
}

fn region_carrier_count(region: &CurveRegion2) -> usize {
    region
        .boundary_loops()
        .iter()
        .map(|boundary| boundary.fragments().len())
        .sum()
}

fn split_fragment_is_affine_line(fragment: &BezierSplitFragment2) -> bool {
    matches!(
        fragment,
        BezierSplitFragment2::Materialized {
            curve: BezierSubcurve2::Quadratic(curve),
            ..
        } if curve.retained_exact_line_image().is_some()
    )
}

fn split_fragment_is_circular_conic(fragment: &BezierSplitFragment2) -> bool {
    matches!(
        fragment,
        BezierSplitFragment2::Materialized {
            curve: BezierSubcurve2::RationalQuadratic(curve),
            ..
        } if curve.retained_circular_conic().is_some()
    )
}

fn region_has_only_affine_or_circular_conic_carriers(region: &CurveRegion2) -> bool {
    region
        .boundary_loops()
        .iter()
        .flat_map(|boundary| boundary.fragments())
        .all(|fragment| {
            split_fragment_is_affine_line(fragment) || split_fragment_is_circular_conic(fragment)
        })
}

fn boolean_region_without_general_context(
    first: &CurveRegion2,
    second: &CurveRegion2,
    operation: BooleanOp,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurveRegion2>> {
    if first.is_empty() || second.is_empty() {
        return empty_operand_result(first, second, operation).map(Some);
    }
    if first == second {
        return identical_operand_result(first, operation).map(Some);
    }

    // Keep the mature line/arc Boolean kernel as an implementation detail of
    // the unified carrier. Promotion retains the exact source
    // `LineArcRegion2`, so immediate operations can bypass general carrier and
    // intersection construction without segmenting curves.
    let invalid =
        |cause| ExactCurveError::invalid(CurveOperation2::Boolean, CurveFamily2::Line, cause);
    if let (Classification::Decided(first), Classification::Decided(second)) = (
        first.native_line_arc_region(policy).map_err(invalid)?,
        second.native_line_arc_region(policy).map_err(invalid)?,
    ) {
        match first
            .boolean_region(second, operation, FillRule::NonZero, policy)
            .map_err(invalid)?
        {
            Classification::Decided(region) => {
                return CurveRegion2::try_from_line_arc_region_raw(&region, policy)
                    .map(Some)
                    .map_err(|error| error.with_operation(CurveOperation2::Boolean));
            }
            Classification::Uncertain(_) => {}
        }
    }
    Ok(None)
}

fn carrier_bounds_decided_disjoint(first: &Curve2, second: &Curve2, policy: &CurveContext) -> bool {
    let (Ok(first_bounds), Ok(second_bounds)) = (first.bounds(), second.bounds()) else {
        return false;
    };
    matches!(
        first_bounds.overlaps(second_bounds, policy),
        Classification::Decided(false)
    )
}

fn subcurve_has_certified_injective_axis(curve: &BezierSubcurve2, policy: &CurveContext) -> bool {
    let rational = match curve {
        BezierSubcurve2::Quadratic(curve) => RationalBezier2::try_new(
            curve.control_points().into_iter().cloned().collect(),
            vec![crate::Real::one(); 3],
        ),
        BezierSubcurve2::Cubic(curve) => RationalBezier2::try_new(
            curve.control_points().into_iter().cloned().collect(),
            vec![crate::Real::one(); 4],
        ),
        BezierSubcurve2::RationalQuadratic(curve) => RationalBezier2::try_new(
            curve.control_points().into_iter().cloned().collect(),
            curve.weights().into_iter().cloned().collect(),
        ),
        BezierSubcurve2::Rational(curve) => return curve.has_certified_injective_axis(policy),
    };
    rational.is_ok_and(|curve| curve.has_certified_injective_axis(policy))
}

fn subcurve_certified_outer_bounds(
    curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => curve.control_hull_box(policy),
        BezierSubcurve2::Cubic(curve) => curve.control_hull_box(policy),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(policy),
    }
}

fn build_region_carriers(
    region: &CurveRegion2,
    operand: CurvePathBooleanOperand2,
    policy: &CurveContext,
    rational_quadratic_area_cache: &mut RationalQuadraticAreaIntegralCache,
) -> ExactCurveResult<Vec<RegionCarrier>> {
    if region.is_empty() {
        return Ok(Vec::new());
    }
    let filled_sides = match region
        .filled_side_is_left_with_area_cache(policy, rational_quadratic_area_cache)
        .map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Boolean, CurveFamily2::Line, cause)
        })? {
        Classification::Decided(sides) => sides,
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Boolean,
                CurveFamily2::Line,
                reason,
            ));
        }
    };
    let mut carriers = Vec::new();
    for (loop_index, boundary_loop) in region.boundary_loops().iter().enumerate() {
        for (fragment_index, fragment) in boundary_loop.fragments().iter().enumerate() {
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
                        UncertaintyReason::Unsupported,
                    ));
                }
            };
            carriers.push(RegionCarrier {
                operand,
                loop_index,
                fragment_index,
                family: subcurve_family(&curve),
                curve,
                start,
                end,
                reversed,
                filled_side_is_left: filled_sides[loop_index],
                image_is_injective: OnceLock::new(),
                bounds: OnceLock::new(),
            });
        }
    }
    Ok(carriers)
}

fn split_carrier(
    carrier: &RegionCarrier,
    events: &[CarrierEvent],
    policy: &CurveContext,
) -> Result<Vec<SplitCarrierFragment>, CurveError> {
    // Most retained events need very little isolator separation. Preserve the
    // former eight-step proof budget for close roots or endpoint images whose
    // complete topology replay needs a narrower interval.
    for max_refinement_steps in [1, 2, 4] {
        if let Ok(fragments) =
            split_carrier_with_refinement(carrier, events, max_refinement_steps, policy)
        {
            return Ok(fragments);
        }
    }
    split_carrier_with_refinement(carrier, events, 8, policy)
}

fn split_carrier_with_refinement(
    carrier: &RegionCarrier,
    events: &[CarrierEvent],
    max_refinement_steps: usize,
    policy: &CurveContext,
) -> Result<Vec<SplitCarrierFragment>, CurveError> {
    let parameters = events
        .iter()
        .map(|event| {
            event
                .parameter
                .clone()
                .refined_isolating_interval(max_refinement_steps, policy)
        })
        .collect::<Vec<_>>();
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

fn certified_boolean_successors(
    graph: &BezierArrangementGraph2,
    directions: &[BooleanArrangementFragmentDirection],
    topology: &CurveRegionBooleanTopology,
    carriers: &[RegionCarrier],
) -> Vec<Option<usize>> {
    // Index starts once so retaining branch certificates stays linear in the
    // emitted graph size even for large curved regions.
    let mut starts_by_vertex = HashMap::<usize, Vec<usize>>::new();
    for (fragment_index, fragment) in graph.fragments().iter().enumerate() {
        if let Some(vertex) = fragment.start_topology_vertex()
            && topology.transverse_contacts.contains_key(&vertex)
        {
            starts_by_vertex
                .entry(vertex)
                .or_default()
                .push(fragment_index);
        }
    }
    graph
        .fragments()
        .iter()
        .enumerate()
        .map(|(current_index, current)| {
            let vertex = current.end_topology_vertex()?;
            let contact = topology.transverse_contacts.get(&vertex)?;
            let crossing_is_positive =
                transverse_carrier_cross_is_positive(topology, contact, vertex, carriers)?;
            let mut candidates = starts_by_vertex
                .get(&vertex)?
                .iter()
                .copied()
                .filter(|candidate_index| *candidate_index != current_index);
            let first_index = candidates.next()?;
            let second_index = candidates.next()?;
            if candidates.next().is_some() {
                return None;
            }
            let current = *directions.get(current_index)?;
            let first = *directions.get(first_index)?;
            let second = *directions.get(second_index)?;
            if [current, first, second].into_iter().any(|direction| {
                direction.carrier_index != contact.first_carrier
                    && direction.carrier_index != contact.second_carrier
            }) {
                return None;
            }
            certified_turn_preference(current, first, second, contact, crossing_is_positive).map(
                |first_before_second| {
                    if first_before_second {
                        first_index
                    } else {
                        second_index
                    }
                },
            )
        })
        .collect()
}

fn transverse_carrier_cross_is_positive(
    topology: &CurveRegionBooleanTopology,
    contact: &TransitionContactCandidate,
    vertex: usize,
    carriers: &[RegionCarrier],
) -> Option<bool> {
    let fragments = topology.split_fragments.get(contact.second_carrier)?;
    let before = fragments
        .iter()
        .find(|fragment| fragment.split.end_topology_vertex == Some(vertex))?
        .location;
    let after = fragments
        .iter()
        .find(|fragment| fragment.split.start_topology_vertex == Some(vertex))?
        .location;
    // For a regular crossing, whether the second oriented carrier enters the
    // first region determines the sign of cross(first tangent, second
    // tangent), after accounting for which side of the first carrier is
    // filled. This reuses the exact region classifications already retained
    // by topology construction.
    transverse_cross_from_locations(
        before,
        after,
        carriers.get(contact.first_carrier)?.filled_side_is_left,
    )
}

const fn transverse_cross_from_locations(
    before: RegionPointLocation,
    after: RegionPointLocation,
    first_filled_side_is_left: bool,
) -> Option<bool> {
    let enters_first_interior = match (before, after) {
        (RegionPointLocation::Outside, RegionPointLocation::Inside) => true,
        (RegionPointLocation::Inside, RegionPointLocation::Outside) => false,
        _ => return None,
    };
    Some(enters_first_interior == first_filled_side_is_left)
}

fn certified_turn_preference(
    base: BooleanArrangementFragmentDirection,
    first: BooleanArrangementFragmentDirection,
    second: BooleanArrangementFragmentDirection,
    contact: &TransitionContactCandidate,
    crossing_is_positive: bool,
) -> Option<bool> {
    let first_half = certified_turn_half(base, first, contact, crossing_is_positive)?;
    let second_half = certified_turn_half(base, second, contact, crossing_is_positive)?;
    if first_half != second_half {
        return Some(first_half < second_half);
    }
    match certified_direction_cross(first, second, contact, crossing_is_positive)? {
        1 => Some(true),
        -1 => Some(false),
        _ => None,
    }
}

fn certified_turn_half(
    base: BooleanArrangementFragmentDirection,
    candidate: BooleanArrangementFragmentDirection,
    contact: &TransitionContactCandidate,
    crossing_is_positive: bool,
) -> Option<u8> {
    if base.carrier_index == candidate.carrier_index {
        return Some(u8::from(base.follows_carrier != candidate.follows_carrier));
    }
    Some(
        if certified_direction_cross(base, candidate, contact, crossing_is_positive)? > 0 {
            0
        } else {
            1
        },
    )
}

fn certified_direction_cross(
    first: BooleanArrangementFragmentDirection,
    second: BooleanArrangementFragmentDirection,
    contact: &TransitionContactCandidate,
    crossing_is_positive: bool,
) -> Option<i8> {
    if first.carrier_index == second.carrier_index {
        return Some(0);
    }
    let source_cross = if first.carrier_index == contact.first_carrier
        && second.carrier_index == contact.second_carrier
    {
        if crossing_is_positive { 1 } else { -1 }
    } else if first.carrier_index == contact.second_carrier
        && second.carrier_index == contact.first_carrier
    {
        if crossing_is_positive { -1 } else { 1 }
    } else {
        return None;
    };
    let first_orientation = if first.follows_carrier { 1 } else { -1 };
    let second_orientation = if second.follows_carrier { 1 } else { -1 };
    Some(source_cross * first_orientation * second_orientation)
}

fn certified_transverse_contact_vertices(
    split_fragments: &[Vec<SplitCarrierFragment>],
    candidates: &[Option<TransitionContactCandidate>],
    policy: &CurveContext,
) -> Vec<bool> {
    candidates
        .iter()
        .enumerate()
        .map(|(vertex, candidate)| {
            let Some(candidate) = candidate else {
                return false;
            };
            if candidate.certified_transverse {
                return true;
            }
            let Some(first) = algebraic_endpoint_tangent_at_vertex(
                &split_fragments[candidate.first_carrier],
                vertex,
            ) else {
                return false;
            };
            let Some(second) = algebraic_endpoint_tangent_at_vertex(
                &split_fragments[candidate.second_carrier],
                vertex,
            ) else {
                return false;
            };
            matches!(
                algebraic_endpoint_tangents_are_transverse(first, second, policy),
                Classification::Decided(true)
            )
        })
        .collect()
}

const fn toggled_region_location(location: RegionPointLocation) -> Option<RegionPointLocation> {
    match location {
        RegionPointLocation::Inside => Some(RegionPointLocation::Outside),
        RegionPointLocation::Outside => Some(RegionPointLocation::Inside),
        RegionPointLocation::Boundary => None,
    }
}

fn algebraic_endpoint_tangent_at_vertex(
    fragments: &[SplitCarrierFragment],
    vertex: usize,
) -> Option<&BezierEndpointTangentImage2> {
    fragments.iter().find_map(|split| {
        let BezierSplitFragment2::AlgebraicEndpointImages {
            reversed,
            start_image,
            end_image,
            ..
        } = &split.fragment
        else {
            return None;
        };
        if split.start_topology_vertex == Some(vertex) {
            return if *reversed { end_image } else { start_image }
                .as_ref()
                .and_then(|image| image.try_tangent().ok());
        }
        if split.end_topology_vertex == Some(vertex) {
            return if *reversed { start_image } else { end_image }
                .as_ref()
                .and_then(|image| image.try_tangent().ok());
        }
        None
    })
}

fn push_carrier_event(
    events: &mut Vec<CarrierEvent>,
    parameter: BezierParameter2,
    topology_vertex: Option<usize>,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    for event in events.iter_mut() {
        match parameter
            .cmp_by_refinement(&event.parameter, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Boolean,
                    CurveFamily2::RationalBezier,
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
    policy: &CurveContext,
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
    policy: &CurveContext,
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
    contact_points: &mut [ContactVertex],
    from: usize,
    to: usize,
) {
    for event in events.iter_mut().flatten() {
        if event.topology_vertex == Some(from) {
            event.topology_vertex = Some(to);
        }
    }
    for contact in contact_points {
        if contact.topology_vertex == from {
            contact.topology_vertex = to;
        }
    }
}

fn contacts_decided_distinct_from_carriers(
    existing: &ContactVertex,
    carrier_indices: [usize; 2],
    parameters: [&BezierParameter2; 2],
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    for existing_carrier in existing.carrier_indices {
        for current_carrier in carrier_indices {
            let existing_bounds = carriers[existing_carrier].bounds.get_or_init(|| {
                subcurve_certified_outer_bounds(&carriers[existing_carrier].curve, policy)
            });
            let current_bounds = carriers[current_carrier].bounds.get_or_init(|| {
                subcurve_certified_outer_bounds(&carriers[current_carrier].curve, policy)
            });
            if let (
                Classification::Decided(existing_bounds),
                Classification::Decided(current_bounds),
            ) = (existing_bounds, current_bounds)
                && existing_bounds.overlaps(current_bounds, policy)
                    == Classification::Decided(false)
            {
                return Ok(true);
            }
        }
    }
    for (existing_slot, existing_carrier) in existing.carrier_indices.iter().copied().enumerate() {
        let carrier = &carriers[existing_carrier];
        let image_is_injective = carrier.image_is_injective.get() == Some(&true)
            || subcurve_has_certified_injective_axis(&carrier.curve, policy);
        if !image_is_injective {
            continue;
        }
        let _ = carrier.image_is_injective.set(true);
        for (current_slot, current_carrier) in carrier_indices.iter().copied().enumerate() {
            if existing_carrier == current_carrier
                && decided_parameter_cmp(
                    &existing.parameters[existing_slot],
                    parameters[current_slot],
                    policy,
                )? != Ordering::Equal
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn event_vertex(
    events: &[CarrierEvent],
    parameter: &BezierParameter2,
    policy: &CurveContext,
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
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    parameter_between(parameter, &carrier.start, &carrier.end, policy)
}

fn parameter_strictly_inside_carrier(
    parameter: &BezierParameter2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> bool {
    matches!(
        (
            decided_parameter_cmp(parameter, &carrier.start, policy),
            decided_parameter_cmp(parameter, &carrier.end, policy),
        ),
        (Ok(Ordering::Greater), Ok(Ordering::Less))
    )
}

fn parameter_between(
    parameter: &BezierParameter2,
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let lower = decided_parameter_cmp(parameter, start, policy)?;
    let upper = decided_parameter_cmp(parameter, end, policy)?;
    Ok(!lower.is_lt() && !upper.is_gt())
}

fn parameter_range_inside_carrier(
    start: &BezierParameter2,
    end: &BezierParameter2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
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
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let (start, end) = ascending_range(range, policy)?;
    Ok(!decided_parameter_cmp(end, &carrier.start, policy)?.is_lt()
        && !decided_parameter_cmp(start, &carrier.end, policy)?.is_gt())
}

fn range_inside_carrier(
    range: &BezierParameterRange2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
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
    policy: &CurveContext,
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
    policy: &CurveContext,
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
    policy: &CurveContext,
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
    policy: &CurveContext,
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
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let (range_start, range_end) = ascending_range(range, policy)?;
    Ok(
        !decided_parameter_cmp(fragment_start, range_start, policy)?.is_lt()
            && !decided_parameter_cmp(fragment_end, range_end, policy)?.is_gt(),
    )
}

fn ascending_range<'a>(
    range: &'a BezierParameterRange2,
    policy: &CurveContext,
) -> ExactCurveResult<(&'a BezierParameter2, &'a BezierParameter2)> {
    match decided_parameter_cmp(range.start(), range.end(), policy)? {
        Ordering::Less => Ok((range.start(), range.end())),
        Ordering::Greater => Ok((range.end(), range.start())),
        Ordering::Equal => Err(ExactCurveError::invalid(
            CurveOperation2::Boolean,
            CurveFamily2::RationalBezier,
            CurveError::DegenerateOverlapRange,
        )),
    }
}

fn decided_parameter_cmp(
    first: &BezierParameter2,
    second: &BezierParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<Ordering> {
    match first.cmp_by_refinement(second, policy).map_err(|cause| {
        ExactCurveError::invalid(
            CurveOperation2::Boolean,
            CurveFamily2::RationalBezier,
            cause,
        )
    })? {
        Classification::Decided(ordering) => Ok(ordering),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Boolean,
            CurveFamily2::RationalBezier,
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

fn same_contact_point(
    first: &RationalBezierIntersectionPointEvidence2,
    second: &RationalBezierIntersectionPointEvidence2,
    policy: &CurveContext,
) -> Classification<bool> {
    match (first, second) {
        (
            RationalBezierIntersectionPointEvidence2::Exact(first),
            RationalBezierIntersectionPointEvidence2::Exact(second),
        ) => match crate::classify::is_zero(&first.distance_squared(second), policy) {
            Some(equal) => Classification::Decided(equal),
            None => Classification::Uncertain(UncertaintyReason::RealSign),
        },
        (
            RationalBezierIntersectionPointEvidence2::Algebraic(first),
            RationalBezierIntersectionPointEvidence2::Algebraic(second),
        ) => {
            if let Some(classification) =
                first.same_injective_parametric_source_point(second, policy)
            {
                return classification;
            }
            // A decided same-sign rational Bezier control hull contains the
            // entire affine curve image, so disjoint source hulls prove that
            // these retained point images cannot represent the same contact.
            if let (
                Some(Classification::Decided(first_bounds)),
                Some(Classification::Decided(second_bounds)),
            ) = (
                first.parametric_source_bounds(policy),
                second.parametric_source_bounds(policy),
            ) && first_bounds.overlaps(&second_bounds, policy) == Classification::Decided(false)
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "contact-point-equality",
                    "source-bounds-disjoint",
                );
                return Classification::Decided(false);
            }
            let (Some(first), Some(second)) = (first.resolved(policy), second.resolved(policy))
            else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let (Some(first_x), Some(first_y), Some(second_x), Some(second_y)) = (
                first.x().and_then(|image| image.representation()),
                first.y().and_then(|image| image.representation()),
                second.x().and_then(|image| image.representation()),
                second.y().and_then(|image| image.representation()),
            ) else {
                return if first == second {
                    Classification::Decided(true)
                } else {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                };
            };
            match (
                crate::bezier_arrangement::represented_roots_equal(first_x, second_x, policy),
                crate::bezier_arrangement::represented_roots_equal(first_y, second_y, policy),
            ) {
                (Some(x_equal), Some(y_equal)) => Classification::Decided(x_equal && y_equal),
                _ => Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        (
            RationalBezierIntersectionPointEvidence2::Exact(exact),
            RationalBezierIntersectionPointEvidence2::Algebraic(algebraic),
        )
        | (
            RationalBezierIntersectionPointEvidence2::Algebraic(algebraic),
            RationalBezierIntersectionPointEvidence2::Exact(exact),
        ) => {
            let Some(algebraic) = algebraic.resolved(policy) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let (Some(x), Some(y)) = (
                algebraic.x().and_then(|image| image.representation()),
                algebraic.y().and_then(|image| image.representation()),
            ) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let exact_x =
                crate::bezier_algebraic_image::exact_real_algebraic_representation(exact.x());
            let exact_y =
                crate::bezier_algebraic_image::exact_real_algebraic_representation(exact.y());
            match (
                crate::bezier_arrangement::represented_roots_equal(x, &exact_x, policy),
                crate::bezier_arrangement::represented_roots_equal(y, &exact_y, policy),
            ) {
                (Some(x_equal), Some(y_equal)) => Classification::Decided(x_equal && y_equal),
                _ => Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
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
            ExactCurveError::invalid(CurveOperation2::Boolean, CurveFamily2::Line, cause)
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
            ExactCurveError::invalid(CurveOperation2::Boolean, CurveFamily2::Line, cause)
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

#[cfg(test)]
mod certified_successor_tests {
    use super::*;
    use crate::{
        BezierAlgebraicParameter2, CurvePath2, LineSeg2, Point2, RationalBezier2,
        RationalBezierAlgebraicPointImage2, Real,
    };

    fn decided<T>(classification: Classification<T>) -> T {
        match classification {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                panic!("classification unexpectedly uncertain: {reason:?}")
            }
        }
    }

    fn sqrt_half_parameter(policy: &CurveContext) -> BezierAlgebraicParameter2 {
        let polynomial = decided(
            crate::BezierParameterPolynomial::try_new_power_basis(
                vec![(-1).into(), 0.into(), 2.into()],
                policy,
            )
            .expect("valid parameter polynomial"),
        );
        let interval = decided(
            crate::BezierParameterInterval::try_new(
                (Real::one() / Real::from(2_i8)).expect("nonzero denominator"),
                Real::one(),
                policy,
            )
            .expect("valid parameter interval"),
        );
        decided(
            BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy)
                .expect("isolated parameter"),
        )
    }

    fn rational_line(start_x: i32, end_x: i32) -> RationalBezier2 {
        RationalBezier2::try_new(
            vec![
                Point2::from_values(start_x, 0),
                Point2::from_values(end_x, 0),
            ],
            vec![Real::one(); 2],
        )
        .expect("valid rational line")
    }

    fn square_region(min_x: i8, min_y: i8, max_x: i8, max_y: i8) -> CurveRegion2 {
        let points = [
            Point2::from_values(min_x, min_y),
            Point2::from_values(max_x, min_y),
            Point2::from_values(max_x, max_y),
            Point2::from_values(min_x, max_y),
        ];
        let curves = (0..points.len())
            .map(|index| {
                Curve2::from(
                    LineSeg2::try_new(
                        points[index].clone(),
                        points[(index + 1) % points.len()].clone(),
                    )
                    .unwrap(),
                )
            })
            .collect();
        CurveRegion2::try_from_boundary_paths(
            &[CurvePath2::try_new(curves).unwrap()],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
    }

    #[test]
    fn native_region_fast_path_matches_forced_general_arrangement() {
        let first = square_region(0, 0, 4, 4);
        let second = square_region(2, 0, 6, 4);
        let policy = CurveContext::STRICT;
        let operations = [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ];
        let fast = operations.map(|operation| {
            first
                .boolean_region_raw(&second, operation, &policy)
                .unwrap()
        });

        let general = CurveRegionBooleanContext::try_new(&first, &second, &policy)
            .unwrap()
            .build_boolean_regions([None, None, None, None])
            .unwrap();
        assert!(general.candidate_carrier_pair_count() > 0);
        assert!(general.topology_fragment_count() > 0);
        for (operation, fast_region) in operations.into_iter().zip(&fast) {
            let general_region = general.region(operation);
            assert_eq!(
                decided(fast_region.signed_area(&policy).unwrap().into_value()),
                decided(general_region.signed_area(&policy).unwrap().into_value())
            );
            for x_numerator in -2_i8..=14 {
                for y_numerator in -2_i8..=10 {
                    let point = Point2::new(
                        (Real::from(x_numerator) / Real::from(2_i8)).unwrap(),
                        (Real::from(y_numerator) / Real::from(2_i8)).unwrap(),
                    );
                    assert_eq!(
                        fast_region
                            .classify_point(&point, &policy)
                            .unwrap()
                            .into_value(),
                        general_region
                            .classify_point(&point, &policy)
                            .unwrap()
                            .into_value(),
                        "forced-general {operation:?} differs at {point:?}"
                    );
                }
            }
        }
    }

    fn direction(
        carrier_index: usize,
        follows_carrier: bool,
    ) -> BooleanArrangementFragmentDirection {
        BooleanArrangementFragmentDirection {
            carrier_index,
            follows_carrier,
        }
    }

    fn vector(
        direction: BooleanArrangementFragmentDirection,
        crossing_is_positive: bool,
    ) -> (i8, i8) {
        let vector = if direction.carrier_index == 3 {
            (1, 0)
        } else if crossing_is_positive {
            (0, 1)
        } else {
            (0, -1)
        };
        if direction.follows_carrier {
            vector
        } else {
            (-vector.0, -vector.1)
        }
    }

    fn numerical_turn_preference(
        base: (i8, i8),
        first: (i8, i8),
        second: (i8, i8),
    ) -> Option<bool> {
        let half = |candidate: (i8, i8)| {
            let cross = base.0 * candidate.1 - base.1 * candidate.0;
            if cross > 0 {
                0
            } else if cross < 0 {
                1
            } else if base.0 * candidate.0 + base.1 * candidate.1 > 0 {
                0
            } else {
                1
            }
        };
        let first_half = half(first);
        let second_half = half(second);
        if first_half != second_half {
            return Some(first_half < second_half);
        }
        match (first.0 * second.1 - first.1 * second.0).cmp(&0) {
            Ordering::Greater => Some(true),
            Ordering::Less => Some(false),
            Ordering::Equal => None,
        }
    }

    #[test]
    fn classified_crossing_side_recovers_oriented_tangent_cross() {
        assert_eq!(
            transverse_cross_from_locations(
                RegionPointLocation::Outside,
                RegionPointLocation::Inside,
                true,
            ),
            Some(true)
        );
        assert_eq!(
            transverse_cross_from_locations(
                RegionPointLocation::Inside,
                RegionPointLocation::Outside,
                true,
            ),
            Some(false)
        );
        assert_eq!(
            transverse_cross_from_locations(
                RegionPointLocation::Outside,
                RegionPointLocation::Inside,
                false,
            ),
            Some(false)
        );
        assert_eq!(
            transverse_cross_from_locations(
                RegionPointLocation::Inside,
                RegionPointLocation::Outside,
                false,
            ),
            Some(true)
        );
    }

    #[test]
    fn contact_point_bounds_reject_disjoint_lazy_sources() {
        let policy = CurveContext::STRICT;
        let parameter = sqrt_half_parameter(&policy);
        let first_curve = rational_line(0, 1);
        let second_curve = rational_line(2, 3);
        let first = RationalBezierIntersectionPointEvidence2::Algebraic(
            RationalBezierAlgebraicPointImage2::from_parametric_source(
                first_curve.clone(),
                parameter.clone(),
                &policy,
            ),
        );
        let second = RationalBezierIntersectionPointEvidence2::Algebraic(
            RationalBezierAlgebraicPointImage2::from_parametric_source(
                second_curve.clone(),
                parameter.clone(),
                &policy,
            ),
        );

        assert!(
            parameter
                .cached_rational_bezier_point_image(&first_curve)
                .is_none()
        );
        assert!(
            parameter
                .cached_rational_bezier_point_image(&second_curve)
                .is_none()
        );
        assert_eq!(
            same_contact_point(&first, &second, &policy),
            Classification::Decided(false)
        );
        assert!(
            parameter
                .cached_rational_bezier_point_image(&first_curve)
                .is_none()
        );
        assert!(
            parameter
                .cached_rational_bezier_point_image(&second_curve)
                .is_none()
        );
    }

    #[test]
    fn identical_injective_source_parameters_compare_without_materialization() {
        let policy = CurveContext::STRICT;
        let parameter = sqrt_half_parameter(&policy);
        let curve = rational_line(0, 1);
        let first = RationalBezierIntersectionPointEvidence2::Algebraic(
            RationalBezierAlgebraicPointImage2::from_parametric_source(
                curve.clone(),
                parameter.clone(),
                &policy,
            ),
        );
        let second = RationalBezierIntersectionPointEvidence2::Algebraic(
            RationalBezierAlgebraicPointImage2::from_parametric_source(
                curve.clone(),
                parameter.clone(),
                &policy,
            ),
        );

        assert_eq!(
            same_contact_point(&first, &second, &policy),
            Classification::Decided(true)
        );
        #[cfg(feature = "predicates")]
        assert!(
            parameter
                .cached_rational_bezier_point_image(&curve)
                .is_none()
        );
    }

    #[test]
    fn certified_branch_order_matches_exact_vector_order() {
        let contact = TransitionContactCandidate {
            first_carrier: 3,
            second_carrier: 7,
            certified_transverse: true,
        };
        for crossing_is_positive in [false, true] {
            for base_carrier in [3, 7] {
                for base_forward in [false, true] {
                    for first_carrier in [3, 7] {
                        for first_forward in [false, true] {
                            for second_carrier in [3, 7] {
                                for second_forward in [false, true] {
                                    let base = direction(base_carrier, base_forward);
                                    let first = direction(first_carrier, first_forward);
                                    let second = direction(second_carrier, second_forward);
                                    assert_eq!(
                                        certified_turn_preference(
                                            base,
                                            first,
                                            second,
                                            &contact,
                                            crossing_is_positive,
                                        ),
                                        numerical_turn_preference(
                                            vector(base, crossing_is_positive),
                                            vector(first, crossing_is_positive),
                                            vector(second, crossing_is_positive),
                                        ),
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
