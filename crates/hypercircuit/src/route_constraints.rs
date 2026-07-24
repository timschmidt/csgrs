//! Exact retained route-constraint realization.
//!
//! Negotiated search consumes region and escape policy directly. This module
//! realizes bounded length-tuning intent after a route proposal has stable
//! semantic identities. The result remains a proposal: ordinary layout
//! validation and HyperDRC release checks still certify the resulting copper.

use std::cmp::Ordering;
use std::collections::BTreeSet;

use hypercurve::{Classification, CurvePolicy, RegionPointLocation};
use hyperlattice::Point2;
use hyperlimit::{
    RingPointLocation, SegmentIntersection, classify_point_ring_even_odd,
    classify_segment_intersection,
};
use hyperpath::LinePathSegment;
use hyperreal::Real;

use crate::autoroute::{
    point_segment_distance_squared, segment_clear_of_placed_pad, segment_segment_distance_squared,
};
use crate::{
    Circuit, CircuitInstanceId, DifferentialPairId, KeepoutId, KeepoutScope, LengthTuningPattern,
    LengthTuningPatternId, LengthTuningSide, NetId, PadId, PcbLayout, PcbRoute, PcbRouteSegment,
    PhaseTuningGroup, PhaseTuningGroupId, RouteId, StackupLayerKind, ViaId, ZoneId,
};
#[cfg(feature = "geometry")]
use crate::{MaterializationOptions, MaterializedCopperIdentity, ZoneMaterializationEvidence};

/// Terminal outcome of one bounded exact tuning request.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LengthTuningStatus {
    /// A serpentine proposal was generated.
    Applied,
    /// Existing orthogonal route length already meets the authored target.
    AlreadySatisfied,
    /// The retained request could not be realized without changing its contract.
    Rejected,
}

/// Audited reason an exact tuning request was rejected.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LengthTuningIssue {
    /// No retained request has the selected stable identity.
    UnknownPattern(LengthTuningPatternId),
    /// At least one selected-net route contains a curve or diagonal segment.
    UnsupportedNetGeometry,
    /// Existing route length is already above the accepted target interval.
    TargetBelowCurrent,
    /// No bounded cycle count can meet the exact target interval.
    UnreachableTarget,
    /// No selected orthogonal route span has enough forward capacity.
    NoEligibleSpan,
    /// Exact region containment could not be decided.
    IndeterminateRegion,
    /// Every geometrically eligible serpentine leaves the authored tuning region.
    OutsideRegion,
}

/// Exact evidence and optional replacement route for one tuning request.
#[derive(Clone, Debug, PartialEq)]
pub struct LengthTuningReport {
    /// Retained request identity.
    pub pattern: LengthTuningPatternId,
    /// Terminal outcome.
    pub status: LengthTuningStatus,
    /// Exact selected-net length before realization, when supported.
    pub original_length: Option<Real>,
    /// Exact selected-net length after realization, when available.
    pub realized_length: Option<Real>,
    /// Exact authored target.
    pub target_length: Option<Real>,
    /// Number of generated serpentine cycles.
    pub cycles: usize,
    /// Stable route selected for replacement.
    pub route: Option<RouteId>,
    /// Index of the replaced source segment.
    pub segment_index: Option<usize>,
    /// Rejected-contract evidence.
    pub issues: Vec<LengthTuningIssue>,
    /// Proposed replacement for exactly one semantic route.
    pub tuned_route: Option<PcbRoute>,
}

impl LengthTuningReport {
    /// Applies an accepted replacement without changing unrelated PCB intent.
    pub fn apply_to(&self, layout: &PcbLayout) -> Option<PcbLayout> {
        let tuned = self.tuned_route.as_ref()?;
        let mut result = layout.clone();
        let route = result
            .routes
            .iter_mut()
            .find(|route| route.id == tuned.id)?;
        route.clone_from(tuned);
        Some(result)
    }
}

/// Terminal outcome of one atomic multi-pattern phase-tuning request.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhaseTuningStatus {
    /// At least one member route changed and the complete assembly was certified.
    Applied,
    /// Every member already met its target and no route changed.
    AlreadySatisfied,
    /// No member change may be applied because the group contract failed.
    Rejected,
}

/// Zone geometry used while certifying an atomic phase-tuning proposal.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhaseTuningZoneCollisionMode {
    /// Conservatively treat every authored foreign-zone boundary as copper.
    AuthoredBoundary,
    /// Repour the candidate layout and inspect the final materialized zone profile.
    RealizedFill,
}

/// Certified outcome for one tuned-route/realized-zone pair.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhaseTuningRealizedZoneStatus {
    /// Materialization proved its configured clearance is at least the required clearance.
    Clear,
    /// An exact point classification proved copper inside the extra clearance band.
    Collision,
    /// Neither the materialization contract nor exact probes could certify the pair.
    Indeterminate,
}

/// Source-addressable realized-fill clearance evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct PhaseTuningRealizedZoneEvidence {
    /// Tuned semantic route.
    pub route: RouteId,
    /// Foreign semantic copper zone.
    pub zone: ZoneId,
    /// Exact clearance required outside the already widened route copper.
    pub required_clearance: Real,
    /// Whether island processing left no copper in the zone.
    pub realized_fill_empty: bool,
    /// Number of islands retained by zone materialization.
    pub retained_islands: usize,
    /// Exact geometry outcome.
    pub status: PhaseTuningRealizedZoneStatus,
}

/// Foreign routed object involved in a phase-tuning clearance finding.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PhaseTuningObstacle {
    /// Another retained route.
    Route(RouteId),
    /// A retained via land spanning the tuned route layer.
    Via(ViaId),
    /// A conservatively enveloped placed pad land.
    Pad {
        /// Placed logical instance.
        instance: CircuitInstanceId,
        /// Land-pattern pad identity.
        pad: PadId,
    },
    /// A retained foreign-net copper-zone source boundary.
    Zone(ZoneId),
    /// An all-copper or layer-scoped copper keepout.
    Keepout(KeepoutId),
}

/// Audited reason an atomic phase-tuning request was rejected.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PhaseTuningIssue {
    /// No retained group has the selected stable identity.
    UnknownGroup(PhaseTuningGroupId),
    /// The group references absent/repeated patterns or fewer than two distinct nets.
    InvalidContract,
    /// One member's ordinary bounded tuning request was rejected.
    MemberRejected(LengthTuningPatternId),
    /// Exact clearance against a curved foreign route is not implemented.
    UnsupportedCollisionGeometry(RouteId),
    /// Generated copper violates the retained edge-clearance contract.
    ClearanceViolation {
        /// Generated/tuned route.
        route: RouteId,
        /// Foreign routed object.
        obstacle: PhaseTuningObstacle,
    },
    /// Exact clearance classification could not be decided.
    IndeterminateClearance {
        /// Generated/tuned route.
        route: RouteId,
        /// Retained obstacle whose relationship was indeterminate.
        obstacle: PhaseTuningObstacle,
    },
    /// Final member centerline lengths exceed the retained group skew.
    SkewExceeded,
    /// A differential-pair group no longer has one constant translated route shape.
    DifferentialCouplingLost,
    /// Candidate-layout zone materialization could not be certified.
    RealizedZoneMaterialization(String),
}

/// Exact all-or-nothing evidence for one retained phase-tuning group.
#[derive(Clone, Debug, PartialEq)]
pub struct PhaseTuningReport {
    /// Retained group identity.
    pub group: PhaseTuningGroupId,
    /// Terminal outcome.
    pub status: PhaseTuningStatus,
    /// Member reports in retained group order.
    pub members: Vec<LengthTuningReport>,
    /// Exact member-net lengths before realization.
    pub original_lengths: Vec<(crate::NetId, Real)>,
    /// Exact member-net lengths after realization, when every member is supported.
    pub realized_lengths: Vec<(crate::NetId, Real)>,
    /// Maximum minus minimum realized member length.
    pub realized_skew: Option<Real>,
    /// Rejected-contract evidence.
    pub issues: Vec<PhaseTuningIssue>,
    /// Conservative authored boundary or candidate-repoured zone geometry.
    pub zone_collision_mode: PhaseTuningZoneCollisionMode,
    /// Source-addressable evidence emitted by realized-fill mode.
    pub realized_zone_evidence: Vec<PhaseTuningRealizedZoneEvidence>,
    /// Complete route replacements, applied only as one atomic set.
    pub tuned_routes: Vec<PcbRoute>,
}

impl PhaseTuningReport {
    /// Applies every accepted replacement atomically.
    pub fn apply_to(&self, layout: &PcbLayout) -> Option<PcbLayout> {
        if self.status == PhaseTuningStatus::Rejected {
            return None;
        }
        let mut result = layout.clone();
        for tuned in &self.tuned_routes {
            let route = result
                .routes
                .iter_mut()
                .find(|route| route.id == tuned.id)?;
            route.clone_from(tuned);
        }
        Some(result)
    }
}

/// Deterministic bounded policy for synthesizing one atomic tuning assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct PhaseTuningSynthesisPolicy {
    /// Stable identity for the proposed phase group and generated member prefix.
    pub group: PhaseTuningGroupId,
    /// Distinct logical nets to length-match in retained order.
    pub nets: Vec<NetId>,
    /// Exact target length; `None` selects the longest current member.
    pub target_length: Option<Real>,
    /// Accepted exact absolute deviation from the target.
    pub tolerance: Real,
    /// Exact perpendicular excursion for each generated cycle.
    pub amplitude: Real,
    /// Exact forward run before each perpendicular excursion.
    pub pitch: Real,
    /// Hard bound on cycles generated for one member.
    pub maximum_cycles: usize,
    /// Exact padding added around each synthesized serpentine region.
    pub region_margin: Real,
    /// Maximum exact centerline-length skew across all members.
    pub maximum_skew: Real,
    /// Minimum exact edge clearance to foreign retained copper and keepouts.
    pub minimum_clearance: Real,
    /// Optional differential pair whose coupling must survive realization.
    pub differential_pair: Option<DifferentialPairId>,
    /// Hard bound on complete candidate assemblies certified.
    pub maximum_candidate_assemblies: usize,
}

impl PhaseTuningSynthesisPolicy {
    /// Creates an exact bounded synthesis request.
    pub fn new(
        group: PhaseTuningGroupId,
        nets: impl IntoIterator<Item = NetId>,
        amplitude: Real,
        pitch: Real,
        maximum_cycles: usize,
    ) -> Self {
        Self {
            group,
            nets: nets.into_iter().collect(),
            target_length: None,
            tolerance: Real::zero(),
            amplitude,
            pitch,
            maximum_cycles,
            region_margin: Real::zero(),
            maximum_skew: Real::zero(),
            minimum_clearance: Real::zero(),
            differential_pair: None,
            maximum_candidate_assemblies: 256,
        }
    }
}

/// Terminal outcome of automatic phase-tuning intent synthesis.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhaseTuningSynthesisStatus {
    /// Generated intent changed at least one route and passed atomic certification.
    Certified,
    /// Generated intent proved that every member already meets the contract.
    AlreadySatisfied,
    /// No candidate assembly could be certified within the retained policy.
    Rejected,
}

/// Audited reason automatic tuning-intent synthesis was rejected.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PhaseTuningSynthesisIssue {
    /// Bounds, exact dimensions, identities, or member selection are invalid.
    InvalidPolicy,
    /// Generated group or member identity already exists.
    DuplicateIdentity,
    /// A selected net is absent from retained routes.
    UnknownNet(NetId),
    /// A selected net contains curved or diagonal route geometry.
    UnsupportedNetGeometry(NetId),
    /// The selected exact target is below one member's current length.
    TargetBelowCurrent(NetId),
    /// No bounded cycle count can meet one member's exact target interval.
    UnreachableTarget(NetId),
    /// No orthogonal span can carry one member's required cycles.
    NoEligibleSpan(NetId),
    /// Exact generated-copper containment against the board could not be decided.
    IndeterminateBoardBoundary(NetId),
    /// Every generated assembly failed atomic phase/clearance certification.
    NoCertifiedAssembly,
    /// Candidate certification stopped at the caller's hard assembly bound.
    CandidateLimitReached,
}

/// Audited generated tuning intent and its exact atomic realization proof.
#[derive(Clone, Debug, PartialEq)]
pub struct PhaseTuningSynthesisReport {
    /// Requested stable group identity.
    pub group_id: PhaseTuningGroupId,
    /// Terminal synthesis outcome.
    pub status: PhaseTuningSynthesisStatus,
    /// Exact selected or inferred common target.
    pub target_length: Option<Real>,
    /// Complete candidate assemblies available before the caller bound.
    pub candidate_assemblies_available: usize,
    /// Complete candidate assemblies passed to atomic certification.
    pub candidate_assemblies_considered: usize,
    /// Certified generated member intent in retained net order.
    pub patterns: Vec<LengthTuningPattern>,
    /// Certified generated atomic group.
    pub group: Option<PhaseTuningGroup>,
    /// Existing exact realization evidence for the certified assembly.
    pub realization: Option<PhaseTuningReport>,
    /// Rejected-policy or bounded-search evidence.
    pub issues: Vec<PhaseTuningSynthesisIssue>,
}

impl PhaseTuningSynthesisReport {
    /// Adds certified generated intent without changing copper.
    pub fn apply_intent_to(&self, circuit: &Circuit, layout: &PcbLayout) -> Option<PcbLayout> {
        if self.status == PhaseTuningSynthesisStatus::Rejected {
            return None;
        }
        let group = self.group.as_ref()?;
        let mut result = layout.clone();
        result
            .rules
            .length_tuning_patterns
            .extend(self.patterns.clone());
        result.rules.phase_tuning_groups.push(group.clone());
        result.validate(circuit).is_valid().then_some(result)
    }

    /// Applies the already-certified route replacements atomically.
    pub fn apply_tuned_to(&self, layout: &PcbLayout) -> Option<PcbLayout> {
        self.realization.as_ref()?.apply_to(layout)
    }
}

impl PcbLayout {
    /// Synthesizes and atomically certifies one bounded multi-net tuning assembly.
    pub fn synthesize_phase_tuning(
        &self,
        circuit: &Circuit,
        policy: PhaseTuningSynthesisPolicy,
    ) -> PhaseTuningSynthesisReport {
        let rejected = |issue| rejected_synthesis(policy.group.clone(), None, 0, 0, issue);
        let unique_nets = policy.nets.iter().collect::<BTreeSet<_>>();
        if policy.nets.len() < 2
            || unique_nets.len() != policy.nets.len()
            || policy.amplitude.partial_cmp(&Real::zero()) != Some(Ordering::Greater)
            || policy.pitch.partial_cmp(&Real::zero()) != Some(Ordering::Greater)
            || policy.maximum_cycles == 0
            || !is_non_negative(&policy.tolerance)
            || !is_non_negative(&policy.region_margin)
            || !is_non_negative(&policy.maximum_skew)
            || !is_non_negative(&policy.minimum_clearance)
            || policy.maximum_candidate_assemblies == 0
            || policy
                .target_length
                .as_ref()
                .is_some_and(|target| target.partial_cmp(&Real::zero()) != Some(Ordering::Greater))
            || !self.validate(circuit).is_valid()
        {
            return rejected(PhaseTuningSynthesisIssue::InvalidPolicy);
        }
        if self
            .rules
            .phase_tuning_groups
            .iter()
            .any(|group| group.id == policy.group)
        {
            return rejected(PhaseTuningSynthesisIssue::DuplicateIdentity);
        }
        if let Some(pair_id) = &policy.differential_pair {
            let Some(pair) = self
                .rules
                .differential_pairs
                .iter()
                .find(|pair| &pair.id == pair_id)
            else {
                return rejected(PhaseTuningSynthesisIssue::InvalidPolicy);
            };
            if policy.nets.len() != 2
                || !unique_nets.contains(&pair.positive)
                || !unique_nets.contains(&pair.negative)
            {
                return rejected(PhaseTuningSynthesisIssue::InvalidPolicy);
            }
        }

        let mut lengths = Vec::with_capacity(policy.nets.len());
        for net in &policy.nets {
            if !self.routes.iter().any(|route| &route.net == net) {
                return rejected(PhaseTuningSynthesisIssue::UnknownNet(net.clone()));
            }
            let Some(length) = net_orthogonal_length_for_net(self, net) else {
                return rejected(PhaseTuningSynthesisIssue::UnsupportedNetGeometry(
                    net.clone(),
                ));
            };
            lengths.push((net.clone(), length));
        }
        let target = match &policy.target_length {
            Some(target) => target.clone(),
            None => lengths
                .iter()
                .map(|(_, length)| length)
                .try_fold(None, |maximum: Option<Real>, length| {
                    Some(Some(match maximum {
                        Some(maximum)
                            if length.partial_cmp(&maximum) != Some(Ordering::Greater) =>
                        {
                            maximum
                        }
                        _ => length.clone(),
                    }))
                })
                .flatten()
                .expect("at least two selected nets have retained lengths"),
        };
        for (net, length) in &lengths {
            if length > &(target.clone() + policy.tolerance.clone()) {
                return rejected_synthesis(
                    policy.group.clone(),
                    Some(target),
                    0,
                    0,
                    PhaseTuningSynthesisIssue::TargetBelowCurrent(net.clone()),
                );
            }
        }

        let mut candidate_sets = Vec::with_capacity(policy.nets.len());
        for (net, length) in &lengths {
            let id =
                LengthTuningPatternId::new(format!("{}-{}", policy.group.as_str(), net.as_str()))
                    .expect("generated synthesis identity is nonempty");
            if self
                .rules
                .length_tuning_patterns
                .iter()
                .any(|pattern| pattern.id == id)
            {
                return rejected_synthesis(
                    policy.group.clone(),
                    Some(target),
                    0,
                    0,
                    PhaseTuningSynthesisIssue::DuplicateIdentity,
                );
            }
            let candidates =
                match tuning_synthesis_candidates(self, net, id, length, &target, &policy) {
                    Ok(candidates) => candidates,
                    Err(issue) => {
                        return rejected_synthesis(policy.group.clone(), Some(target), 0, 0, issue);
                    }
                };
            candidate_sets.push(candidates);
        }
        let available = candidate_sets.iter().fold(1_usize, |product, candidates| {
            product.saturating_mul(candidates.len())
        });
        let group = PhaseTuningGroup {
            id: policy.group.clone(),
            patterns: candidate_sets
                .iter()
                .map(|candidates| candidates[0].id.clone())
                .collect(),
            differential_pair: policy.differential_pair.clone(),
            maximum_skew: policy.maximum_skew.clone(),
            minimum_clearance: policy.minimum_clearance.clone(),
        };
        let mut indices = vec![0_usize; candidate_sets.len()];
        let mut considered = 0_usize;
        loop {
            if considered == policy.maximum_candidate_assemblies {
                return rejected_synthesis(
                    policy.group,
                    Some(target),
                    available,
                    considered,
                    PhaseTuningSynthesisIssue::CandidateLimitReached,
                );
            }
            considered = considered.saturating_add(1);
            let patterns = candidate_sets
                .iter()
                .zip(&indices)
                .map(|(candidates, index)| candidates[*index].clone())
                .collect::<Vec<_>>();
            let mut candidate_layout = self.clone();
            candidate_layout
                .rules
                .length_tuning_patterns
                .extend(patterns.clone());
            candidate_layout
                .rules
                .phase_tuning_groups
                .push(group.clone());
            let realization = candidate_layout.realize_phase_tuning(circuit, &group.id);
            if realization.status != PhaseTuningStatus::Rejected {
                return PhaseTuningSynthesisReport {
                    group_id: group.id.clone(),
                    status: if realization.status == PhaseTuningStatus::Applied {
                        PhaseTuningSynthesisStatus::Certified
                    } else {
                        PhaseTuningSynthesisStatus::AlreadySatisfied
                    },
                    target_length: Some(target),
                    candidate_assemblies_available: available,
                    candidate_assemblies_considered: considered,
                    patterns,
                    group: Some(group),
                    realization: Some(realization),
                    issues: Vec::new(),
                };
            }

            let mut cursor = indices.len();
            while cursor > 0 {
                cursor -= 1;
                indices[cursor] = indices[cursor].saturating_add(1);
                if indices[cursor] < candidate_sets[cursor].len() {
                    break;
                }
                indices[cursor] = 0;
            }
            if cursor == 0 && indices[0] == 0 {
                return rejected_synthesis(
                    policy.group,
                    Some(target),
                    available,
                    considered,
                    PhaseTuningSynthesisIssue::NoCertifiedAssembly,
                );
            }
        }
    }

    /// Realizes one bounded exact serpentine request against retained routes.
    pub fn realize_length_tuning(&self, pattern: &LengthTuningPatternId) -> LengthTuningReport {
        let Some(pattern_value) = self
            .rules
            .length_tuning_patterns
            .iter()
            .find(|candidate| &candidate.id == pattern)
        else {
            return rejected(
                pattern.clone(),
                None,
                None,
                LengthTuningIssue::UnknownPattern(pattern.clone()),
            );
        };
        let target = Some(pattern_value.target_length.clone());
        let Some(original_length) = net_orthogonal_length(self, pattern_value) else {
            return rejected(
                pattern.clone(),
                None,
                target,
                LengthTuningIssue::UnsupportedNetGeometry,
            );
        };
        let deviation = real_abs(&(original_length.clone() - pattern_value.target_length.clone()));
        if deviation <= pattern_value.tolerance {
            return LengthTuningReport {
                pattern: pattern.clone(),
                status: LengthTuningStatus::AlreadySatisfied,
                original_length: Some(original_length.clone()),
                realized_length: Some(original_length),
                target_length: target,
                cycles: 0,
                route: None,
                segment_index: None,
                issues: Vec::new(),
                tuned_route: None,
            };
        }
        if original_length > pattern_value.target_length.clone() + pattern_value.tolerance.clone() {
            return rejected(
                pattern.clone(),
                Some(original_length),
                target,
                LengthTuningIssue::TargetBelowCurrent,
            );
        }
        let Some(cycles) = matching_cycle_count(&original_length, pattern_value) else {
            return rejected(
                pattern.clone(),
                Some(original_length),
                target,
                LengthTuningIssue::UnreachableTarget,
            );
        };

        let mut indeterminate = false;
        let mut had_capacity = false;
        let selected_routes = self
            .routes
            .iter()
            .filter(|route| {
                route.net == pattern_value.net
                    && pattern_value
                        .route
                        .as_ref()
                        .is_none_or(|selected| &route.id == selected)
            })
            .collect::<Vec<_>>();
        for source in selected_routes {
            for (segment_index, segment) in source.segments.iter().enumerate() {
                let PcbRouteSegment::Line(line) = segment else {
                    continue;
                };
                let Some(span) = orthogonal_line_length(line.start(), line.end()) else {
                    continue;
                };
                let required_forward =
                    pattern_value.pitch.clone() * Real::from(cycles.saturating_mul(2) as u128);
                if span < required_forward {
                    continue;
                }
                had_capacity = true;
                let Some(points) = tuning_points(line.start(), line.end(), cycles, pattern_value)
                else {
                    continue;
                };
                match polyline_inside_region(&points, &pattern_value.region) {
                    Some(true) => {
                        let mut tuned = source.clone();
                        let replacements = points
                            .windows(2)
                            .filter(|pair| pair[0] != pair[1])
                            .map(|pair| {
                                PcbRouteSegment::Line(LinePathSegment::new(
                                    pair[0].clone(),
                                    pair[1].clone(),
                                ))
                            })
                            .collect::<Vec<_>>();
                        tuned
                            .segments
                            .splice(segment_index..=segment_index, replacements);
                        let added = pattern_value.amplitude.clone()
                            * Real::from(cycles.saturating_mul(2) as u128);
                        let realized = original_length.clone() + added;
                        return LengthTuningReport {
                            pattern: pattern.clone(),
                            status: LengthTuningStatus::Applied,
                            original_length: Some(original_length),
                            realized_length: Some(realized),
                            target_length: target,
                            cycles,
                            route: Some(source.id.clone()),
                            segment_index: Some(segment_index),
                            issues: Vec::new(),
                            tuned_route: Some(tuned),
                        };
                    }
                    Some(false) => {}
                    None => indeterminate = true,
                }
            }
        }
        rejected(
            pattern.clone(),
            Some(original_length),
            target,
            if indeterminate {
                LengthTuningIssue::IndeterminateRegion
            } else if had_capacity {
                LengthTuningIssue::OutsideRegion
            } else {
                LengthTuningIssue::NoEligibleSpan
            },
        )
    }

    /// Realizes and certifies every member of one phase group atomically.
    ///
    /// Member patterns are evaluated in retained group order against a private
    /// working layout. No partial route set is exposed if one member fails,
    /// final skew exceeds the group contract, or generated line geometry
    /// violates clearance to foreign retained copper or copper keepouts.
    pub fn realize_phase_tuning(
        &self,
        circuit: &Circuit,
        group: &PhaseTuningGroupId,
    ) -> PhaseTuningReport {
        let Some(group_value) = self
            .rules
            .phase_tuning_groups
            .iter()
            .find(|candidate| &candidate.id == group)
        else {
            return rejected_phase(
                group.clone(),
                Vec::new(),
                Vec::new(),
                PhaseTuningIssue::UnknownGroup(group.clone()),
            );
        };
        if !self.validate(circuit).is_valid() {
            return rejected_phase(
                group.clone(),
                Vec::new(),
                Vec::new(),
                PhaseTuningIssue::InvalidContract,
            );
        }
        let unique = group_value.patterns.iter().collect::<BTreeSet<_>>();
        let patterns = group_value
            .patterns
            .iter()
            .filter_map(|id| {
                self.rules
                    .length_tuning_patterns
                    .iter()
                    .find(|pattern| &pattern.id == id)
            })
            .collect::<Vec<_>>();
        let unique_nets = patterns
            .iter()
            .map(|pattern| &pattern.net)
            .collect::<BTreeSet<_>>();
        if group_value.patterns.len() < 2
            || unique.len() != group_value.patterns.len()
            || patterns.len() != group_value.patterns.len()
            || unique_nets.len() != patterns.len()
            || !is_non_negative(&group_value.maximum_skew)
            || !is_non_negative(&group_value.minimum_clearance)
        {
            return rejected_phase(
                group.clone(),
                Vec::new(),
                Vec::new(),
                PhaseTuningIssue::InvalidContract,
            );
        }
        let Some(original_lengths) = phase_lengths(self, &patterns) else {
            return rejected_phase(
                group.clone(),
                Vec::new(),
                Vec::new(),
                PhaseTuningIssue::InvalidContract,
            );
        };

        let mut working = self.clone();
        let mut members = Vec::with_capacity(group_value.patterns.len());
        let mut tuned_ids = BTreeSet::new();
        for pattern in &group_value.patterns {
            let report = working.realize_length_tuning(pattern);
            if report.status == LengthTuningStatus::Rejected {
                return rejected_phase(
                    group.clone(),
                    members,
                    original_lengths,
                    PhaseTuningIssue::MemberRejected(pattern.clone()),
                );
            }
            if let Some(route) = report.tuned_route.as_ref() {
                tuned_ids.insert(route.id.clone());
                let Some(next) = report.apply_to(&working) else {
                    return rejected_phase(
                        group.clone(),
                        members,
                        original_lengths,
                        PhaseTuningIssue::MemberRejected(pattern.clone()),
                    );
                };
                working = next;
            }
            members.push(report);
        }

        if let Some(issue) = phase_tuning_collision(
            circuit,
            &working,
            &tuned_ids,
            &group_value.minimum_clearance,
        ) {
            return rejected_phase(group.clone(), members, original_lengths, issue);
        }
        let Some(realized_lengths) = phase_lengths(&working, &patterns) else {
            return rejected_phase(
                group.clone(),
                members,
                original_lengths,
                PhaseTuningIssue::InvalidContract,
            );
        };
        if let Some(pair_id) = &group_value.differential_pair {
            let Some(pair) = self
                .rules
                .differential_pairs
                .iter()
                .find(|pair| &pair.id == pair_id)
            else {
                return rejected_phase(
                    group.clone(),
                    members,
                    original_lengths,
                    PhaseTuningIssue::InvalidContract,
                );
            };
            if !differential_routes_remain_coupled(&working, &patterns, pair) {
                return rejected_phase(
                    group.clone(),
                    members,
                    original_lengths,
                    PhaseTuningIssue::DifferentialCouplingLost,
                );
            }
        }
        let Some(realized_skew) = length_skew(&realized_lengths) else {
            return rejected_phase(
                group.clone(),
                members,
                original_lengths,
                PhaseTuningIssue::InvalidContract,
            );
        };
        if !matches!(
            realized_skew.partial_cmp(&group_value.maximum_skew),
            Some(Ordering::Less | Ordering::Equal)
        ) {
            return rejected_phase(
                group.clone(),
                members,
                original_lengths,
                PhaseTuningIssue::SkewExceeded,
            );
        }
        let tuned_routes = working
            .routes
            .iter()
            .filter(|route| tuned_ids.contains(&route.id))
            .cloned()
            .collect::<Vec<_>>();
        let status = if tuned_routes.is_empty() {
            PhaseTuningStatus::AlreadySatisfied
        } else {
            PhaseTuningStatus::Applied
        };
        PhaseTuningReport {
            group: group.clone(),
            status,
            members,
            original_lengths,
            realized_lengths,
            realized_skew: Some(realized_skew),
            issues: Vec::new(),
            zone_collision_mode: PhaseTuningZoneCollisionMode::AuthoredBoundary,
            realized_zone_evidence: Vec::new(),
            tuned_routes,
        }
    }

    /// Tunes one atomic group and certifies foreign zones after candidate repour.
    ///
    /// This geometry-enabled path retains all ordinary route/via/pad/keepout
    /// checks and materializes the candidate routes with the original zones.
    /// Successful repour certifies every requirement no greater than the zone's
    /// own clearance. Exact point classification can prove intrusion into any
    /// additional required band; relationships it cannot prove are rejected as
    /// indeterminate after hatch, thermal, keepout, and island processing.
    #[cfg(feature = "geometry")]
    pub fn realize_phase_tuning_with_realized_zones(
        &self,
        circuit: &Circuit,
        group: &PhaseTuningGroupId,
        options: MaterializationOptions,
    ) -> PhaseTuningReport {
        let mut without_zones = self.clone();
        let stitching = without_zones.realize_stitching_vias();
        without_zones.vias.extend(stitching.vias);
        without_zones.zones.clear();
        let mut report = without_zones.realize_phase_tuning(circuit, group);
        report.zone_collision_mode = PhaseTuningZoneCollisionMode::RealizedFill;
        if report.status == PhaseTuningStatus::Rejected || report.tuned_routes.is_empty() {
            return report;
        }
        let Some(group_value) = self
            .rules
            .phase_tuning_groups
            .iter()
            .find(|candidate| &candidate.id == group)
        else {
            return reject_realized_phase(report, PhaseTuningIssue::UnknownGroup(group.clone()));
        };
        let Some(candidate) = report.apply_to(self) else {
            return reject_realized_phase(report, PhaseTuningIssue::InvalidContract);
        };
        let materialized = match candidate.materialize(circuit, options.clone()) {
            Ok(materialized) => materialized,
            Err(error) => {
                return reject_realized_phase(
                    report,
                    PhaseTuningIssue::RealizedZoneMaterialization(error.to_string()),
                );
            }
        };
        let mut evidence = Vec::new();
        if let Some(issue) = realized_zone_collision(
            &candidate,
            &materialized,
            &report.tuned_routes,
            &group_value.minimum_clearance,
            &mut evidence,
        ) {
            report.realized_zone_evidence = evidence;
            return reject_realized_phase(report, issue);
        }
        report.realized_zone_evidence = evidence;
        report
    }
}

#[cfg(feature = "geometry")]
fn reject_realized_phase(
    mut report: PhaseTuningReport,
    issue: PhaseTuningIssue,
) -> PhaseTuningReport {
    report.status = PhaseTuningStatus::Rejected;
    report.realized_lengths.clear();
    report.realized_skew = None;
    report.issues = vec![issue];
    report.tuned_routes.clear();
    report
}

#[cfg(feature = "geometry")]
fn realized_zone_collision(
    layout: &PcbLayout,
    materialized: &crate::PcbMaterializationReport,
    tuned_routes: &[PcbRoute],
    minimum_clearance: &Real,
    evidence: &mut Vec<PhaseTuningRealizedZoneEvidence>,
) -> Option<PhaseTuningIssue> {
    for route in tuned_routes {
        let route_feature_exists = materialized
            .copper_features
            .iter()
            .any(|feature| feature.identity == MaterializedCopperIdentity::Route(route.id.clone()));
        if !route_feature_exists {
            return Some(PhaseTuningIssue::RealizedZoneMaterialization(format!(
                "materialized route {} is missing",
                route.id.as_str()
            )));
        }
        for zone in &layout.zones {
            if zone.net == route.net || zone.layer != route.layer {
                continue;
            }
            let zone_feature = materialized.copper_features.iter().find(|feature| {
                feature.identity == MaterializedCopperIdentity::Zone(zone.id.clone())
            });
            let Some(zone_feature) = zone_feature else {
                return Some(PhaseTuningIssue::RealizedZoneMaterialization(format!(
                    "materialized zone {} is missing",
                    zone.id.as_str()
                )));
            };
            let retained_islands = materialized
                .zone_realizations
                .iter()
                .find(|realization| realization.source == format!("zone:{}", zone.id.as_str()))
                .map(|realization: &ZoneMaterializationEvidence| realization.retained_islands);
            let Some(retained_islands) = retained_islands else {
                return Some(PhaseTuningIssue::RealizedZoneMaterialization(format!(
                    "zone realization evidence {} is missing",
                    zone.id.as_str()
                )));
            };
            let Some(required_clearance) = maximum_real(minimum_clearance, &zone.clearance) else {
                evidence.push(PhaseTuningRealizedZoneEvidence {
                    route: route.id.clone(),
                    zone: zone.id.clone(),
                    required_clearance: minimum_clearance.clone(),
                    realized_fill_empty: zone_feature.profile.is_empty(),
                    retained_islands,
                    status: PhaseTuningRealizedZoneStatus::Indeterminate,
                });
                return Some(PhaseTuningIssue::IndeterminateClearance {
                    route: route.id.clone(),
                    obstacle: PhaseTuningObstacle::Zone(zone.id.clone()),
                });
            };
            let fill_empty = zone_feature.profile.is_empty();
            let clearance_is_sufficient = required_clearance
                .partial_cmp(&zone.clearance)
                .is_some_and(|ordering| ordering != Ordering::Greater);
            let status = if fill_empty || clearance_is_sufficient {
                PhaseTuningRealizedZoneStatus::Clear
            } else {
                match realized_zone_intrusion_probe(
                    route,
                    &zone_feature.profile,
                    &zone.clearance,
                    &required_clearance,
                ) {
                    Some(true) => PhaseTuningRealizedZoneStatus::Collision,
                    Some(false) | None => PhaseTuningRealizedZoneStatus::Indeterminate,
                }
            };
            evidence.push(PhaseTuningRealizedZoneEvidence {
                route: route.id.clone(),
                zone: zone.id.clone(),
                required_clearance,
                realized_fill_empty: fill_empty,
                retained_islands,
                status,
            });
            match status {
                PhaseTuningRealizedZoneStatus::Clear => {}
                PhaseTuningRealizedZoneStatus::Collision => {
                    return Some(PhaseTuningIssue::ClearanceViolation {
                        route: route.id.clone(),
                        obstacle: PhaseTuningObstacle::Zone(zone.id.clone()),
                    });
                }
                PhaseTuningRealizedZoneStatus::Indeterminate => {
                    return Some(PhaseTuningIssue::IndeterminateClearance {
                        route: route.id.clone(),
                        obstacle: PhaseTuningObstacle::Zone(zone.id.clone()),
                    });
                }
            }
        }
    }
    None
}

#[cfg(feature = "geometry")]
fn realized_zone_intrusion_probe(
    route: &PcbRoute,
    zone: &csgrs::sketch::Profile,
    realized_clearance: &Real,
    required_clearance: &Real,
) -> Option<bool> {
    let route_half_width = (route.width.clone() / Real::from(2)).ok()?;
    let clearance_probe =
        ((realized_clearance.clone() + required_clearance.clone()) / Real::from(2)).ok()?;
    let normal_distance = route_half_width + clearance_probe;
    for segment in &route.segments {
        let PcbRouteSegment::Line(line) = segment else {
            return None;
        };
        let horizontal = line.start().y == line.end().y;
        let vertical = line.start().x == line.end().x;
        if !horizontal && !vertical {
            return None;
        }
        for numerator in [Real::one(), Real::from(2), Real::from(3)] {
            let fraction = (numerator / Real::from(4)).ok()?;
            let center = Point2::new(
                line.start().x.clone()
                    + (line.end().x.clone() - line.start().x.clone()) * fraction.clone(),
                line.start().y.clone() + (line.end().y.clone() - line.start().y.clone()) * fraction,
            );
            let probes = if horizontal {
                [
                    Point2::new(center.x.clone(), center.y.clone() - normal_distance.clone()),
                    Point2::new(center.x, center.y + normal_distance.clone()),
                ]
            } else {
                [
                    Point2::new(center.x.clone() - normal_distance.clone(), center.y.clone()),
                    Point2::new(center.x + normal_distance.clone(), center.y),
                ]
            };
            for probe in probes {
                let probe = hypercurve::Point2::new(probe.x, probe.y);
                match zone
                    .as_curve_region()
                    .classify_point(&probe, &CurvePolicy::certified())
                    .ok()?
                {
                    Classification::Decided(
                        RegionPointLocation::Inside | RegionPointLocation::Boundary,
                    ) => return Some(true),
                    Classification::Decided(RegionPointLocation::Outside) => {}
                    Classification::Uncertain(_) => return None,
                }
            }
        }
    }
    Some(false)
}

fn rejected_phase(
    group: PhaseTuningGroupId,
    members: Vec<LengthTuningReport>,
    original_lengths: Vec<(crate::NetId, Real)>,
    issue: PhaseTuningIssue,
) -> PhaseTuningReport {
    PhaseTuningReport {
        group,
        status: PhaseTuningStatus::Rejected,
        members,
        original_lengths,
        realized_lengths: Vec::new(),
        realized_skew: None,
        issues: vec![issue],
        zone_collision_mode: PhaseTuningZoneCollisionMode::AuthoredBoundary,
        realized_zone_evidence: Vec::new(),
        tuned_routes: Vec::new(),
    }
}

fn phase_lengths(
    layout: &PcbLayout,
    patterns: &[&LengthTuningPattern],
) -> Option<Vec<(crate::NetId, Real)>> {
    patterns
        .iter()
        .map(|pattern| Some((pattern.net.clone(), net_orthogonal_length(layout, pattern)?)))
        .collect()
}

fn length_skew(lengths: &[(crate::NetId, Real)]) -> Option<Real> {
    let mut values = lengths.iter().map(|(_, length)| length);
    let first = values.next()?.clone();
    let (minimum, maximum) =
        values.try_fold((first.clone(), first), |(minimum, maximum), value| {
            Some((
                if value.partial_cmp(&minimum)? == Ordering::Less {
                    value.clone()
                } else {
                    minimum
                },
                if value.partial_cmp(&maximum)? == Ordering::Greater {
                    value.clone()
                } else {
                    maximum
                },
            ))
        })?;
    Some(maximum - minimum)
}

fn differential_routes_remain_coupled(
    layout: &PcbLayout,
    patterns: &[&LengthTuningPattern],
    pair: &crate::DifferentialPair,
) -> bool {
    let route_for_net = |net: &crate::NetId| {
        let pattern = patterns.iter().find(|pattern| &pattern.net == net)?;
        let route_id = pattern.route.as_ref()?;
        layout.routes.iter().find(|route| &route.id == route_id)
    };
    let (Some(positive), Some(negative)) =
        (route_for_net(&pair.positive), route_for_net(&pair.negative))
    else {
        return false;
    };
    if positive.layer != negative.layer
        || positive.segments.len() != negative.segments.len()
        || positive.segments.is_empty()
    {
        return false;
    }
    let center_spacing = pair.spacing.clone()
        + match ((positive.width.clone() + negative.width.clone()) / Real::from(2)).ok() {
            Some(half_widths) => half_widths,
            None => return false,
        };
    let first_positive = positive.segments[0].start();
    let first_negative = negative.segments[0].start();
    let translation = Point2::new(
        first_negative.x.clone() - first_positive.x.clone(),
        first_negative.y.clone() - first_positive.y.clone(),
    );
    let translation_squared = translation.x.clone() * translation.x.clone()
        + translation.y.clone() * translation.y.clone();
    if translation_squared != center_spacing.clone() * center_spacing {
        return false;
    }
    positive
        .segments
        .iter()
        .zip(&negative.segments)
        .all(|(positive, negative)| {
            matches!(
                (positive, negative),
                (PcbRouteSegment::Line(_), PcbRouteSegment::Line(_))
            ) && negative.start().x.clone() - positive.start().x.clone() == translation.x
                && negative.start().y.clone() - positive.start().y.clone() == translation.y
                && negative.end().x.clone() - positive.end().x.clone() == translation.x
                && negative.end().y.clone() - positive.end().y.clone() == translation.y
        })
}

fn phase_tuning_collision(
    circuit: &Circuit,
    layout: &PcbLayout,
    tuned_ids: &BTreeSet<RouteId>,
    minimum_clearance: &Real,
) -> Option<PhaseTuningIssue> {
    let stitching = layout.realize_stitching_vias();
    for tuned in layout
        .routes
        .iter()
        .filter(|route| tuned_ids.contains(&route.id))
    {
        for other in &layout.routes {
            if other.id == tuned.id || other.net == tuned.net || other.layer != tuned.layer {
                continue;
            }
            let Some(half_widths) =
                ((tuned.width.clone() + other.width.clone()) / Real::from(2)).ok()
            else {
                return Some(PhaseTuningIssue::UnsupportedCollisionGeometry(
                    tuned.id.clone(),
                ));
            };
            let required = half_widths + minimum_clearance.clone();
            let required_squared = required.clone() * required;
            for tuned_segment in &tuned.segments {
                let PcbRouteSegment::Line(tuned_line) = tuned_segment else {
                    return Some(PhaseTuningIssue::UnsupportedCollisionGeometry(
                        tuned.id.clone(),
                    ));
                };
                for other_segment in &other.segments {
                    let PcbRouteSegment::Line(other_line) = other_segment else {
                        return Some(PhaseTuningIssue::UnsupportedCollisionGeometry(
                            other.id.clone(),
                        ));
                    };
                    let Some(distance) = segment_segment_distance_squared(
                        tuned_line.start(),
                        tuned_line.end(),
                        other_line.start(),
                        other_line.end(),
                    ) else {
                        return Some(PhaseTuningIssue::IndeterminateClearance {
                            route: tuned.id.clone(),
                            obstacle: PhaseTuningObstacle::Route(other.id.clone()),
                        });
                    };
                    let Some(ordering) = distance.partial_cmp(&required_squared) else {
                        return Some(PhaseTuningIssue::IndeterminateClearance {
                            route: tuned.id.clone(),
                            obstacle: PhaseTuningObstacle::Route(other.id.clone()),
                        });
                    };
                    if ordering == Ordering::Less {
                        return Some(PhaseTuningIssue::ClearanceViolation {
                            route: tuned.id.clone(),
                            obstacle: PhaseTuningObstacle::Route(other.id.clone()),
                        });
                    }
                }
            }
        }
        for via in layout.vias.iter().chain(&stitching.vias) {
            if via.net == tuned.net || tuned.layer < via.start_layer || tuned.layer > via.end_layer
            {
                continue;
            }
            let Some(half_widths) =
                ((tuned.width.clone() + via.land_diameter.clone()) / Real::from(2)).ok()
            else {
                return Some(PhaseTuningIssue::UnsupportedCollisionGeometry(
                    tuned.id.clone(),
                ));
            };
            let required = half_widths + minimum_clearance.clone();
            let required_squared = required.clone() * required;
            for segment in &tuned.segments {
                let PcbRouteSegment::Line(line) = segment else {
                    return Some(PhaseTuningIssue::UnsupportedCollisionGeometry(
                        tuned.id.clone(),
                    ));
                };
                let Some(distance) =
                    point_segment_distance_squared(&via.center, line.start(), line.end())
                else {
                    return Some(PhaseTuningIssue::IndeterminateClearance {
                        route: tuned.id.clone(),
                        obstacle: PhaseTuningObstacle::Via(via.id.clone()),
                    });
                };
                let Some(ordering) = distance.partial_cmp(&required_squared) else {
                    return Some(PhaseTuningIssue::IndeterminateClearance {
                        route: tuned.id.clone(),
                        obstacle: PhaseTuningObstacle::Via(via.id.clone()),
                    });
                };
                if ordering == Ordering::Less {
                    return Some(PhaseTuningIssue::ClearanceViolation {
                        route: tuned.id.clone(),
                        obstacle: PhaseTuningObstacle::Via(via.id.clone()),
                    });
                }
            }
        }
        let Some(tuned_half_width) = (tuned.width.clone() / Real::from(2)).ok() else {
            return Some(PhaseTuningIssue::UnsupportedCollisionGeometry(
                tuned.id.clone(),
            ));
        };
        for placement in &layout.placements {
            let Some(instance) = circuit
                .instances
                .iter()
                .find(|instance| instance.id == placement.instance)
            else {
                continue;
            };
            let Some(pattern) = layout
                .land_patterns
                .iter()
                .find(|pattern| pattern.id == placement.land_pattern)
            else {
                continue;
            };
            for pad in &pattern.pads {
                if !pad
                    .copper_layers
                    .iter()
                    .map(|layer| placed_layer(layout, *layer, placement.side))
                    .any(|layer| layer == tuned.layer)
                {
                    continue;
                }
                let pad_net = pattern
                    .pin_map
                    .iter()
                    .filter(|mapping| mapping.pad == pad.id)
                    .find_map(|mapping| {
                        instance
                            .pins
                            .iter()
                            .find(|binding| binding.pin == mapping.pin)
                            .map(|binding| &binding.net)
                    });
                if pad_net.is_some_and(|net| net == &tuned.net) {
                    continue;
                }
                let obstacle = PhaseTuningObstacle::Pad {
                    instance: placement.instance.clone(),
                    pad: pad.id.clone(),
                };
                let required = tuned_half_width.clone() + minimum_clearance.clone();
                if let Some(issue) =
                    route_pad_collision_issue(tuned, placement, pad, &required, obstacle)
                {
                    return Some(issue);
                }
            }
        }
        for zone in &layout.zones {
            if zone.net == tuned.net || zone.layer != tuned.layer {
                continue;
            }
            let Some(clearance) = maximum_real(minimum_clearance, &zone.clearance) else {
                return Some(PhaseTuningIssue::IndeterminateClearance {
                    route: tuned.id.clone(),
                    obstacle: PhaseTuningObstacle::Zone(zone.id.clone()),
                });
            };
            let required = tuned_half_width.clone() + clearance;
            if let Some(issue) = route_polygon_collision_issue(
                tuned,
                &zone.boundary,
                &required,
                PhaseTuningObstacle::Zone(zone.id.clone()),
            ) {
                return Some(issue);
            }
        }
        for keepout in &layout.keepouts {
            let applies = matches!(keepout.scope, KeepoutScope::All)
                || matches!(
                    &keepout.scope,
                    KeepoutScope::Copper(layers) if layers.contains(&tuned.layer)
                );
            if !applies {
                continue;
            }
            let required = tuned_half_width.clone() + minimum_clearance.clone();
            if let Some(issue) = route_polygon_collision_issue(
                tuned,
                &keepout.boundary,
                &required,
                PhaseTuningObstacle::Keepout(keepout.id.clone()),
            ) {
                return Some(issue);
            }
        }
    }
    None
}

fn route_pad_collision_issue(
    route: &PcbRoute,
    placement: &crate::PcbPlacement,
    pad: &crate::LandPatternPad,
    required: &Real,
    obstacle: PhaseTuningObstacle,
) -> Option<PhaseTuningIssue> {
    for segment in &route.segments {
        let PcbRouteSegment::Line(line) = segment else {
            return Some(PhaseTuningIssue::UnsupportedCollisionGeometry(
                route.id.clone(),
            ));
        };
        match segment_clear_of_placed_pad(
            line.start(),
            line.end(),
            placement,
            &pad.center,
            &pad.rotation_degrees,
            &pad.shape,
            required,
        ) {
            Some(true) => {}
            Some(false) => {
                return Some(PhaseTuningIssue::ClearanceViolation {
                    route: route.id.clone(),
                    obstacle: obstacle.clone(),
                });
            }
            None => {
                return Some(PhaseTuningIssue::IndeterminateClearance {
                    route: route.id.clone(),
                    obstacle: obstacle.clone(),
                });
            }
        }
    }
    None
}

fn route_polygon_collision_issue(
    route: &PcbRoute,
    boundary: &[Point2],
    required: &Real,
    obstacle: PhaseTuningObstacle,
) -> Option<PhaseTuningIssue> {
    let required_squared = required.clone() * required.clone();
    for segment in &route.segments {
        let PcbRouteSegment::Line(line) = segment else {
            return Some(PhaseTuningIssue::UnsupportedCollisionGeometry(
                route.id.clone(),
            ));
        };
        for endpoint in [line.start(), line.end()] {
            let Some(location) = classify_point_ring_even_odd(boundary, endpoint).value() else {
                return Some(PhaseTuningIssue::IndeterminateClearance {
                    route: route.id.clone(),
                    obstacle: obstacle.clone(),
                });
            };
            if location != RingPointLocation::Outside {
                return Some(PhaseTuningIssue::ClearanceViolation {
                    route: route.id.clone(),
                    obstacle: obstacle.clone(),
                });
            }
        }
        for index in 0..boundary.len() {
            let Some(distance) = segment_segment_distance_squared(
                line.start(),
                line.end(),
                &boundary[index],
                &boundary[(index + 1) % boundary.len()],
            ) else {
                return Some(PhaseTuningIssue::IndeterminateClearance {
                    route: route.id.clone(),
                    obstacle: obstacle.clone(),
                });
            };
            let Some(ordering) = distance.partial_cmp(&required_squared) else {
                return Some(PhaseTuningIssue::IndeterminateClearance {
                    route: route.id.clone(),
                    obstacle: obstacle.clone(),
                });
            };
            if ordering == Ordering::Less {
                return Some(PhaseTuningIssue::ClearanceViolation {
                    route: route.id.clone(),
                    obstacle: obstacle.clone(),
                });
            }
        }
    }
    None
}

fn placed_layer(
    layout: &PcbLayout,
    layer: hyperpath::TraceLayer,
    side: crate::BoardSide,
) -> hyperpath::TraceLayer {
    if side == crate::BoardSide::Front {
        return layer;
    }
    let last = layout
        .stackup
        .layers
        .iter()
        .filter_map(|candidate| match candidate.kind {
            StackupLayerKind::Conductor(index) => Some(index.0),
            _ => None,
        })
        .max()
        .unwrap_or(layer.0);
    hyperpath::TraceLayer(last.saturating_sub(layer.0))
}

fn maximum_real(first: &Real, second: &Real) -> Option<Real> {
    match first.partial_cmp(second)? {
        Ordering::Less => Some(second.clone()),
        Ordering::Equal | Ordering::Greater => Some(first.clone()),
    }
}

fn rejected_synthesis(
    group_id: PhaseTuningGroupId,
    target_length: Option<Real>,
    candidate_assemblies_available: usize,
    candidate_assemblies_considered: usize,
    issue: PhaseTuningSynthesisIssue,
) -> PhaseTuningSynthesisReport {
    PhaseTuningSynthesisReport {
        group_id,
        status: PhaseTuningSynthesisStatus::Rejected,
        target_length,
        candidate_assemblies_available,
        candidate_assemblies_considered,
        patterns: Vec::new(),
        group: None,
        realization: None,
        issues: vec![issue],
    }
}

fn tuning_synthesis_candidates(
    layout: &PcbLayout,
    net: &NetId,
    id: LengthTuningPatternId,
    original_length: &Real,
    target_length: &Real,
    policy: &PhaseTuningSynthesisPolicy,
) -> Result<Vec<LengthTuningPattern>, PhaseTuningSynthesisIssue> {
    if real_abs(&(original_length.clone() - target_length.clone())) <= policy.tolerance {
        let route = layout
            .routes
            .iter()
            .find(|route| &route.net == net)
            .ok_or_else(|| PhaseTuningSynthesisIssue::UnknownNet(net.clone()))?;
        let points = route
            .segments
            .first()
            .map(|segment| vec![segment.start().clone(), segment.end().clone()])
            .ok_or_else(|| PhaseTuningSynthesisIssue::UnknownNet(net.clone()))?;
        return Ok(vec![LengthTuningPattern {
            id,
            net: net.clone(),
            route: Some(route.id.clone()),
            region: synthesis_region(
                &points,
                &(policy.amplitude.clone() + policy.region_margin.clone()),
            )
            .ok_or_else(|| PhaseTuningSynthesisIssue::NoEligibleSpan(net.clone()))?,
            target_length: target_length.clone(),
            tolerance: policy.tolerance.clone(),
            amplitude: policy.amplitude.clone(),
            pitch: policy.pitch.clone(),
            maximum_cycles: policy.maximum_cycles,
            side: LengthTuningSide::Left,
        }]);
    }
    let Some(cycles) = (1..=policy.maximum_cycles).find(|cycles| {
        let added = policy.amplitude.clone() * Real::from(cycles.saturating_mul(2) as u128);
        real_abs(&(original_length.clone() + added - target_length.clone())) <= policy.tolerance
    }) else {
        return Err(PhaseTuningSynthesisIssue::UnreachableTarget(net.clone()));
    };
    let required_forward = policy.pitch.clone() * Real::from(cycles.saturating_mul(2) as u128);
    let mut candidates = Vec::new();
    let boundary = layout
        .outline
        .boundary_geometry()
        .map_err(|_| PhaseTuningSynthesisIssue::IndeterminateBoardBoundary(net.clone()))?;
    let mut indeterminate_boundary = false;
    for route in layout.routes.iter().filter(|route| &route.net == net) {
        let route_radius = (route.width.clone() / Real::from(2))
            .map_err(|_| PhaseTuningSynthesisIssue::NoEligibleSpan(net.clone()))?;
        for segment in &route.segments {
            let PcbRouteSegment::Line(line) = segment else {
                continue;
            };
            let Some(span) = orthogonal_line_length(line.start(), line.end()) else {
                continue;
            };
            if span < required_forward {
                continue;
            }
            for side in [LengthTuningSide::Left, LengthTuningSide::Right] {
                let draft = LengthTuningPattern {
                    id: id.clone(),
                    net: net.clone(),
                    route: Some(route.id.clone()),
                    region: Vec::new(),
                    target_length: target_length.clone(),
                    tolerance: policy.tolerance.clone(),
                    amplitude: policy.amplitude.clone(),
                    pitch: policy.pitch.clone(),
                    maximum_cycles: policy.maximum_cycles,
                    side,
                };
                let Some(points) = tuning_points(line.start(), line.end(), cycles, &draft) else {
                    continue;
                };
                let inside_board = points.windows(2).try_fold(true, |inside, pair| {
                    if !inside {
                        return Some(false);
                    }
                    match boundary
                        .contains_segment(
                            &pair[0],
                            &pair[1],
                            route_radius.clone(),
                            &CurvePolicy::certified(),
                        )
                        .ok()?
                    {
                        Classification::Decided(decision) => Some(decision),
                        Classification::Uncertain(_) => None,
                    }
                });
                match inside_board {
                    Some(true) => {}
                    Some(false) => continue,
                    None => {
                        indeterminate_boundary = true;
                        continue;
                    }
                }
                let Some(region) = synthesis_region(&points, &policy.region_margin) else {
                    continue;
                };
                candidates.push(LengthTuningPattern { region, ..draft });
            }
        }
    }
    if candidates.is_empty() {
        Err(if indeterminate_boundary {
            PhaseTuningSynthesisIssue::IndeterminateBoardBoundary(net.clone())
        } else {
            PhaseTuningSynthesisIssue::NoEligibleSpan(net.clone())
        })
    } else {
        Ok(candidates)
    }
}

fn synthesis_region(points: &[Point2], margin: &Real) -> Option<Vec<Point2>> {
    let first = points.first()?;
    let mut min_x = first.x.clone();
    let mut max_x = first.x.clone();
    let mut min_y = first.y.clone();
    let mut max_y = first.y.clone();
    for point in &points[1..] {
        if point.x.partial_cmp(&min_x)? == Ordering::Less {
            min_x = point.x.clone();
        }
        if point.x.partial_cmp(&max_x)? == Ordering::Greater {
            max_x = point.x.clone();
        }
        if point.y.partial_cmp(&min_y)? == Ordering::Less {
            min_y = point.y.clone();
        }
        if point.y.partial_cmp(&max_y)? == Ordering::Greater {
            max_y = point.y.clone();
        }
    }
    min_x -= margin.clone();
    max_x += margin.clone();
    min_y -= margin.clone();
    max_y += margin.clone();
    if min_x == max_x || min_y == max_y {
        return None;
    }
    Some(vec![
        Point2::new(min_x.clone(), min_y.clone()),
        Point2::new(max_x.clone(), min_y),
        Point2::new(max_x, max_y.clone()),
        Point2::new(min_x, max_y),
    ])
}

fn rejected(
    pattern: LengthTuningPatternId,
    original_length: Option<Real>,
    target_length: Option<Real>,
    issue: LengthTuningIssue,
) -> LengthTuningReport {
    LengthTuningReport {
        pattern,
        status: LengthTuningStatus::Rejected,
        original_length,
        realized_length: None,
        target_length,
        cycles: 0,
        route: None,
        segment_index: None,
        issues: vec![issue],
        tuned_route: None,
    }
}

fn matching_cycle_count(original_length: &Real, pattern: &LengthTuningPattern) -> Option<usize> {
    (1..=pattern.maximum_cycles).find(|cycles| {
        let added = pattern.amplitude.clone() * Real::from(cycles.saturating_mul(2) as u128);
        real_abs(&(original_length.clone() + added - pattern.target_length.clone()))
            <= pattern.tolerance
    })
}

fn net_orthogonal_length(layout: &PcbLayout, pattern: &LengthTuningPattern) -> Option<Real> {
    net_orthogonal_length_for_net(layout, &pattern.net)
}

fn net_orthogonal_length_for_net(layout: &PcbLayout, net: &NetId) -> Option<Real> {
    layout
        .routes
        .iter()
        .filter(|route| &route.net == net)
        .try_fold(Real::zero(), |total, route| {
            route
                .segments
                .iter()
                .try_fold(total, |route_total, segment| {
                    let PcbRouteSegment::Line(line) = segment else {
                        return None;
                    };
                    Some(route_total + orthogonal_line_length(line.start(), line.end())?)
                })
        })
}

fn orthogonal_line_length(start: &Point2, end: &Point2) -> Option<Real> {
    if start.x == end.x {
        Some(real_abs(&(end.y.clone() - start.y.clone())))
    } else if start.y == end.y {
        Some(real_abs(&(end.x.clone() - start.x.clone())))
    } else {
        None
    }
}

fn tuning_points(
    start: &Point2,
    end: &Point2,
    cycles: usize,
    pattern: &LengthTuningPattern,
) -> Option<Vec<Point2>> {
    let (forward_x, forward_y) = if start.y == end.y {
        (
            direction_sign(&(end.x.clone() - start.x.clone()))?,
            Real::zero(),
        )
    } else if start.x == end.x {
        (
            Real::zero(),
            direction_sign(&(end.y.clone() - start.y.clone()))?,
        )
    } else {
        return None;
    };
    let side = match pattern.side {
        LengthTuningSide::Left => Real::one(),
        LengthTuningSide::Right => -Real::one(),
    };
    let perpendicular_x = -forward_y.clone() * side.clone();
    let perpendicular_y = forward_x.clone() * side;
    let mut cursor = start.clone();
    let mut points = vec![cursor.clone()];
    for _ in 0..cycles {
        cursor = Point2::new(
            cursor.x.clone() + forward_x.clone() * pattern.pitch.clone(),
            cursor.y.clone() + forward_y.clone() * pattern.pitch.clone(),
        );
        points.push(cursor.clone());
        cursor = Point2::new(
            cursor.x.clone() + perpendicular_x.clone() * pattern.amplitude.clone(),
            cursor.y.clone() + perpendicular_y.clone() * pattern.amplitude.clone(),
        );
        points.push(cursor.clone());
        cursor = Point2::new(
            cursor.x.clone() + forward_x.clone() * pattern.pitch.clone(),
            cursor.y.clone() + forward_y.clone() * pattern.pitch.clone(),
        );
        points.push(cursor.clone());
        cursor = Point2::new(
            cursor.x.clone() - perpendicular_x.clone() * pattern.amplitude.clone(),
            cursor.y.clone() - perpendicular_y.clone() * pattern.amplitude.clone(),
        );
        points.push(cursor.clone());
    }
    if &cursor != end {
        points.push(end.clone());
    }
    Some(points)
}

fn direction_sign(value: &Real) -> Option<Real> {
    match value.partial_cmp(&Real::zero())? {
        Ordering::Less => Some(-Real::one()),
        Ordering::Greater => Some(Real::one()),
        Ordering::Equal => None,
    }
}

fn polyline_inside_region(points: &[Point2], region: &[Point2]) -> Option<bool> {
    for point in points {
        if classify_point_ring_even_odd(region, point).value()? == RingPointLocation::Outside {
            return Some(false);
        }
    }
    for segment in points.windows(2) {
        for index in 0..region.len() {
            if classify_segment_intersection(
                &segment[0],
                &segment[1],
                &region[index],
                &region[(index + 1) % region.len()],
            )
            .value()?
                == SegmentIntersection::Proper
            {
                return Some(false);
            }
        }
    }
    Some(true)
}

fn real_abs(value: &Real) -> Real {
    if value.partial_cmp(&Real::zero()) == Some(Ordering::Less) {
        -value.clone()
    } else {
        value.clone()
    }
}

fn is_non_negative(value: &Real) -> bool {
    matches!(
        value.partial_cmp(&Real::zero()),
        Some(Ordering::Equal | Ordering::Greater)
    )
}
