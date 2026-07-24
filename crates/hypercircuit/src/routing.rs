//! Board-level routing handoff to exact `hyperpath` route carriers.
//!
//! Hypercircuit owns terminal/net/layout meaning. Hyperpath owns proposed path
//! geometry and its certification. This module preserves that boundary in both
//! directions without making either crate infer the other's identities.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlattice::Point2;
use hyperpath::{
    MeanderKeepout, NetId as RoutingNetId, PcbTrace, PcbViaStack, SpecctraKeepoutRecord,
    SpecctraLayerAlias, SpecctraNetAlias, SpecctraRoute, SpecctraRouteRuleRecord, TraceLayer,
    ViaDrillIntent,
};

use crate::{
    BoardSide, Circuit, CircuitInstanceId, EscapePolicy, KeepoutId, KeepoutScope,
    LengthTuningPattern, NetClassId, NetId, PadId, PcbLayout, PcbRoute, PcbRouteSegment, PcbVia,
    PhaseTuningGroup, PinRef, Plating, Real, ResolvedNetClass, RouteConstraintRegion, RouteId,
    RouteRuleRegion, RoutingNetAliases, StackupLayerKind, ViaId, ViaStyle,
};

/// One exact placed pad terminal supplied to a board router.
#[derive(Clone, Debug, PartialEq)]
pub struct RoutingTerminal {
    /// Logical instance owning the terminal.
    pub instance: CircuitInstanceId,
    /// Logical pin bound to the terminal net.
    pub pin: PinRef,
    /// Physical land-pattern pad.
    pub pad: PadId,
    /// Circuit-owned logical net.
    pub net: NetId,
    /// Numeric alias consumed by hyperpath.
    pub routing_net: RoutingNetId,
    /// Exact board-space pad center.
    pub center: Point2,
    /// Physical conductor layers carrying this pad after side placement.
    pub layers: Vec<TraceLayer>,
}

/// Retained intent that cannot enter the current hyperpath routing carrier.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RoutingProblemOmission {
    /// Net-class routing policy lacked either a minimum width or clearance.
    IncompleteNetClass(NetClassId),
    /// A polygon was not exactly orthogonal and cannot become a scheduler keepout.
    NonOrthogonalKeepout(KeepoutId),
    /// Via-only keepout needs a typed via-placement carrier.
    ViaOnlyKeepout(KeepoutId),
}

/// Exact board-level route problem assembled from retained circuit/layout intent.
#[derive(Clone, Debug, PartialEq)]
pub struct RoutingProblem {
    /// Stable logical-to-numeric mapping used for every route object.
    pub aliases: RoutingNetAliases,
    /// Human-readable aliases for route interchange and diagnostics.
    pub net_aliases: Vec<SpecctraNetAlias>,
    /// Physical conductor-layer aliases.
    pub layer_aliases: Vec<SpecctraLayerAlias>,
    /// Placed pad terminals requiring connectivity.
    pub terminals: Vec<RoutingTerminal>,
    /// Existing/fixed copper routes supplied as exact obstacles or seeds.
    pub existing: SpecctraRoute,
    /// Circuit-owned via semantics retained beside the geometry-only router carrier.
    pub existing_vias: Vec<PcbVia>,
    /// Exact net-class width/clearance policy that was fully specified.
    pub rules: Vec<SpecctraRouteRuleRecord>,
    /// Complete circuit-owned net classes, including named via-style selection.
    pub net_classes: Vec<ResolvedNetClass>,
    /// Named circuit-owned via constructions available to route adapters.
    pub via_styles: Vec<ViaStyle>,
    /// Exact orthogonal route-search keepouts.
    pub keepouts: Vec<SpecctraKeepoutRecord>,
    /// Circuit-owned polygonal routing constraints retained beside geometry carriers.
    pub route_constraint_regions: Vec<RouteConstraintRegion>,
    /// Circuit-owned regional width/clearance overrides retained for exact search.
    pub route_rule_regions: Vec<RouteRuleRegion>,
    /// Circuit-owned terminal escape policies retained beside geometry carriers.
    pub escape_policies: Vec<EscapePolicy>,
    /// Circuit-owned post-route length-tuning requests retained for proposal replay.
    pub length_tuning_patterns: Vec<LengthTuningPattern>,
    /// Circuit-owned atomic phase groups over retained length-tuning requests.
    pub phase_tuning_groups: Vec<PhaseTuningGroup>,
}

/// Routing problem plus an audit of policy not representable by hyperpath.
#[derive(Clone, Debug, PartialEq)]
pub struct RoutingProblemReport {
    /// Exact routable subset.
    pub problem: RoutingProblem,
    /// Explicit, source-addressable handoff losses.
    pub omissions: Vec<RoutingProblemOmission>,
    /// Deterministic zone-derived vias included in the fixed route inventory.
    pub stitching_realizations: Vec<crate::ZoneStitchingEvidence>,
}

/// Retained candidate geometry not representable by semantic straight routes.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RoutingSolutionOmission {
    /// A newly proposed via has no router-carried mask opening/tenting intent.
    ViaMaskIntentUnavailable(usize),
}

/// Accepted hyperpath candidate mapped back to circuit-owned net identities.
#[derive(Clone, Debug, PartialEq)]
pub struct RoutingSolution {
    /// Semantic straight routes, one source-addressable candidate segment each.
    pub routes: Vec<PcbRoute>,
    /// Semantic vias with retained drill/plating intent.
    pub vias: Vec<PcbVia>,
    /// Candidate features that could not be represented.
    pub omissions: Vec<RoutingSolutionOmission>,
}

/// Terminal status of exact route-quality evaluation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RoutingQualityStatus {
    /// Every multi-terminal problem net has measurable routed geometry.
    Complete,
    /// At least one multi-terminal problem net has no routed geometry.
    Incomplete,
    /// Candidate geometry or exact ordering prevented a complete measurement.
    Unmeasurable,
}

/// Source-addressable reason exact route-quality evidence is incomplete.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RoutingQualityIssue {
    /// A problem net with at least two terminals has no candidate route.
    MissingRoute(NetId),
    /// Circular or Bezier route length needs an explicit approximation policy.
    UnsupportedCurvedRoute(RouteId),
    /// Exact candidate/lower-bound ordering could not be decided.
    IndeterminateMetric(NetId),
}

/// Exact benchmark evidence for one logical routed net.
#[derive(Clone, Debug, PartialEq)]
pub struct RoutingQualityNetEvidence {
    /// Logical net.
    pub net: NetId,
    /// Number of retained placed-pad terminals in the routing problem.
    pub terminals: usize,
    /// Number of semantic route objects in the candidate.
    pub routes: usize,
    /// Exact straight-segment centerline length, absent when unmeasurable or unrouted.
    pub routed_length: Option<Real>,
    /// Exact obstacle-free Euclidean spanning-tree lower bound.
    pub euclidean_mst_lower_bound: Option<Real>,
    /// Routed length minus the lower bound.
    pub excess_length: Option<Real>,
    /// Exact routed/lower-bound ratio when the bound is positive.
    pub stretch: Option<Real>,
    /// Candidate vias assigned to this net.
    pub vias: usize,
}

/// Aggregate exact route-quality evidence suitable for deterministic fixtures.
#[derive(Clone, Debug, PartialEq)]
pub struct RoutingQualityReport {
    /// Terminal outcome.
    pub status: RoutingQualityStatus,
    /// Per-net evidence in stable logical-net order.
    pub nets: Vec<RoutingQualityNetEvidence>,
    /// Exact aggregate candidate centerline length when every net is measurable.
    pub routed_length: Option<Real>,
    /// Exact aggregate obstacle-free Euclidean lower bound.
    pub euclidean_mst_lower_bound: Option<Real>,
    /// Aggregate routed length minus aggregate lower bound.
    pub excess_length: Option<Real>,
    /// Exact aggregate routed/lower-bound ratio when the bound is positive.
    pub stretch: Option<Real>,
    /// Total candidate via count over measured problem nets.
    pub vias: usize,
    /// Typed missing/unsupported/indeterminate evidence.
    pub issues: Vec<RoutingQualityIssue>,
}

/// Structural failure at the circuit/layout/hyperpath boundary.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RoutingAdapterError {
    /// Circuit structure must validate before terminal lowering.
    InvalidCircuit,
    /// PCB structure must validate before terminal lowering.
    InvalidLayout,
    /// A semantic object referenced a circuit net without a numeric alias.
    MissingNetAlias(NetId),
    /// Existing semantic route could not become exact swept geometry.
    InvalidExistingRoute(RouteId),
    /// Existing semantic via could not become an exact hyperpath via.
    InvalidExistingVia(ViaId),
    /// Candidate route used a numeric net not declared by this problem.
    UnknownCandidateNet(RoutingNetId),
    /// Candidate via did not retain a drill diameter.
    MissingCandidateViaDrill(usize),
    /// Generated route/via identity collided while appending a solution.
    IdentityCollision(String),
}

impl std::fmt::Display for RoutingAdapterError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidCircuit => formatter.write_str("circuit is invalid for routing"),
            Self::InvalidLayout => formatter.write_str("layout is invalid for routing"),
            Self::MissingNetAlias(net) => {
                write!(
                    formatter,
                    "routing net alias is missing for {}",
                    net.as_str()
                )
            }
            Self::InvalidExistingRoute(route) => {
                write!(formatter, "existing route {} is invalid", route.as_str())
            }
            Self::InvalidExistingVia(via) => {
                write!(formatter, "existing via {} is invalid", via.as_str())
            }
            Self::UnknownCandidateNet(net) => {
                write!(formatter, "candidate uses unknown routing net {}", net.0)
            }
            Self::MissingCandidateViaDrill(index) => {
                write!(formatter, "candidate via {index} has no drill diameter")
            }
            Self::IdentityCollision(identity) => {
                write!(formatter, "routed feature identity collides: {identity}")
            }
        }
    }
}

impl std::error::Error for RoutingAdapterError {}

impl RoutingProblemReport {
    /// Builds a complete board-level routing handoff from validated source models.
    pub fn from_layout(circuit: &Circuit, layout: &PcbLayout) -> Result<Self, RoutingAdapterError> {
        if !circuit.validate().is_valid() {
            return Err(RoutingAdapterError::InvalidCircuit);
        }
        if !layout.validate(circuit).is_valid() {
            return Err(RoutingAdapterError::InvalidLayout);
        }

        let aliases = RoutingNetAliases::from_circuit(circuit)
            .map_err(|_| RoutingAdapterError::InvalidCircuit)?;
        let net_aliases = aliases
            .iter()
            .map(|(logical, numeric)| SpecctraNetAlias {
                net: numeric,
                name: logical.as_str().to_owned(),
            })
            .collect();
        let layer_aliases = layout
            .stackup
            .layers
            .iter()
            .filter_map(|layer| match layer.kind {
                StackupLayerKind::Conductor(index) => Some(SpecctraLayerAlias {
                    layer: index,
                    name: layer.name.clone(),
                }),
                _ => None,
            })
            .collect();

        let terminals = routing_terminals(circuit, layout, &aliases)?;
        let mut traces = Vec::<PcbTrace>::new();
        let mut arcs = Vec::new();
        let mut beziers = Vec::new();
        for route in &layout.routes {
            let net = aliases
                .get(&route.net)
                .ok_or_else(|| RoutingAdapterError::MissingNetAlias(route.net.clone()))?;
            let lowered = route
                .to_hyperpath(net)
                .map_err(|_| RoutingAdapterError::InvalidExistingRoute(route.id.clone()))?;
            traces.extend(lowered.traces().iter().cloned());
            arcs.extend(lowered.arcs().iter().cloned());
            beziers.extend(lowered.beziers().iter().cloned());
        }
        let stitching = layout.realize_stitching_vias();
        let mut vias = Vec::<PcbViaStack>::new();
        for via in layout.vias.iter().chain(&stitching.vias) {
            let net = aliases
                .get(&via.net)
                .ok_or_else(|| RoutingAdapterError::MissingNetAlias(via.net.clone()))?;
            vias.push(
                via.to_hyperpath(net)
                    .map_err(|_| RoutingAdapterError::InvalidExistingVia(via.id.clone()))?,
            );
        }

        let resolved_net_classes = layout
            .rules
            .resolve_net_classes()
            .expect("validated layout has resolvable net classes");
        let mut omissions = Vec::new();
        let mut rules = Vec::new();
        for class in &resolved_net_classes {
            let width = class
                .preferred_trace_width
                .as_ref()
                .or(class.min_trace_width.as_ref());
            let (Some(clearance), Some(width)) = (&class.min_clearance, width) else {
                omissions.push(RoutingProblemOmission::IncompleteNetClass(class.id.clone()));
                continue;
            };
            for logical in &class.nets {
                let net = aliases
                    .get(logical)
                    .ok_or_else(|| RoutingAdapterError::MissingNetAlias(logical.clone()))?;
                rules.push(SpecctraRouteRuleRecord {
                    net: Some(net),
                    layer: None,
                    clearance: clearance.clone(),
                    width: width.clone(),
                });
            }
        }

        let mut keepouts = Vec::new();
        for keepout in &layout.keepouts {
            let layers = match &keepout.scope {
                KeepoutScope::All => vec![None],
                KeepoutScope::Copper(layers) => layers.iter().copied().map(Some).collect(),
                KeepoutScope::Vias => {
                    omissions.push(RoutingProblemOmission::ViaOnlyKeepout(keepout.id.clone()));
                    continue;
                }
                KeepoutScope::Components => continue,
            };
            if !is_orthogonal_loop(&keepout.boundary) {
                omissions.push(RoutingProblemOmission::NonOrthogonalKeepout(
                    keepout.id.clone(),
                ));
                continue;
            }
            keepouts.extend(layers.into_iter().map(|layer| SpecctraKeepoutRecord {
                layer,
                keepout: MeanderKeepout::OrthogonalPolygon {
                    vertices: keepout.boundary.clone(),
                },
            }));
        }

        Ok(Self {
            problem: RoutingProblem {
                aliases,
                net_aliases,
                layer_aliases,
                terminals,
                existing: SpecctraRoute::with_curves(traces, vias, arcs, beziers),
                existing_vias: layout.vias.iter().chain(&stitching.vias).cloned().collect(),
                rules,
                net_classes: resolved_net_classes,
                via_styles: layout.rules.via_styles.clone(),
                keepouts,
                route_constraint_regions: layout.rules.route_constraint_regions.clone(),
                route_rule_regions: layout.rules.route_rule_regions.clone(),
                escape_policies: layout.rules.escape_policies.clone(),
                length_tuning_patterns: layout.rules.length_tuning_patterns.clone(),
                phase_tuning_groups: layout.rules.phase_tuning_groups.clone(),
            },
            omissions,
            stitching_realizations: stitching.evidence,
        })
    }
}

impl RoutingSolution {
    /// Evaluates deterministic exact route-quality metrics against one retained problem.
    ///
    /// The lower bound is the obstacle-free Euclidean minimum spanning tree over
    /// placed terminal centers. It is deliberately a benchmark baseline rather
    /// than a proof of optimal routability. Curves remain typed unmeasurable
    /// cases instead of crossing an implicit float boundary.
    pub fn quality_report(&self, problem: &RoutingProblem) -> RoutingQualityReport {
        let mut terminals = BTreeMap::<NetId, Vec<&Point2>>::new();
        for terminal in &problem.terminals {
            terminals
                .entry(terminal.net.clone())
                .or_default()
                .push(&terminal.center);
        }
        let mut issues = Vec::new();
        let mut nets = Vec::new();
        for (net, points) in terminals
            .into_iter()
            .filter(|(_, terminals)| terminals.len() >= 2)
        {
            let routes = self
                .routes
                .iter()
                .filter(|route| route.net == net)
                .collect::<Vec<_>>();
            let lower_bound = euclidean_mst_lower_bound(&points);
            if lower_bound.is_none() {
                issues.push(RoutingQualityIssue::IndeterminateMetric(net.clone()));
            }
            let routed_length = if routes.is_empty() {
                issues.push(RoutingQualityIssue::MissingRoute(net.clone()));
                None
            } else {
                straight_routes_length(&routes, &mut issues)
            };
            let excess_length = routed_length
                .as_ref()
                .zip(lower_bound.as_ref())
                .map(|(routed, lower)| routed.clone() - lower.clone());
            let stretch = exact_ratio(routed_length.as_ref(), lower_bound.as_ref());
            nets.push(RoutingQualityNetEvidence {
                net: net.clone(),
                terminals: points.len(),
                routes: routes.len(),
                routed_length,
                euclidean_mst_lower_bound: lower_bound,
                excess_length,
                stretch,
                vias: self.vias.iter().filter(|via| via.net == net).count(),
            });
        }
        let routed_length = sum_optional(nets.iter().map(|net| net.routed_length.as_ref()));
        let lower_bound = sum_optional(
            nets.iter()
                .map(|net| net.euclidean_mst_lower_bound.as_ref()),
        );
        let excess_length = routed_length
            .as_ref()
            .zip(lower_bound.as_ref())
            .map(|(routed, lower)| routed.clone() - lower.clone());
        let stretch = exact_ratio(routed_length.as_ref(), lower_bound.as_ref());
        let vias = nets.iter().map(|net| net.vias).sum();
        let status = if issues.iter().any(|issue| {
            matches!(
                issue,
                RoutingQualityIssue::UnsupportedCurvedRoute(_)
                    | RoutingQualityIssue::IndeterminateMetric(_)
            )
        }) {
            RoutingQualityStatus::Unmeasurable
        } else if issues
            .iter()
            .any(|issue| matches!(issue, RoutingQualityIssue::MissingRoute(_)))
        {
            RoutingQualityStatus::Incomplete
        } else {
            RoutingQualityStatus::Complete
        };
        RoutingQualityReport {
            status,
            nets,
            routed_length,
            euclidean_mst_lower_bound: lower_bound,
            excess_length,
            stretch,
            vias,
            issues,
        }
    }

    /// Maps an accepted hyperpath route candidate back to semantic layout objects.
    pub fn from_hyperpath(
        problem: &RoutingProblem,
        candidate: &SpecctraRoute,
    ) -> Result<Self, RoutingAdapterError> {
        let mut routes = Vec::with_capacity(
            candidate.traces().len() + candidate.arcs().len() + candidate.beziers().len(),
        );
        for (index, trace) in candidate.traces().iter().enumerate() {
            let net = problem
                .aliases
                .logical(trace.net())
                .cloned()
                .ok_or(RoutingAdapterError::UnknownCandidateNet(trace.net()))?;
            routes.push(PcbRoute {
                id: RouteId::new(format!("autoroute-{}", index + 1))
                    .expect("generated route id is non-empty"),
                net,
                layer: trace.layer(),
                width: trace.swept().width().clone(),
                segments: vec![trace.swept().centerline().clone().into()],
            });
        }
        for (index, arc) in candidate.arcs().iter().enumerate() {
            let net = problem
                .aliases
                .logical(arc.net)
                .cloned()
                .ok_or(RoutingAdapterError::UnknownCandidateNet(arc.net))?;
            routes.push(PcbRoute {
                id: RouteId::new(format!("autoroute-arc-{}", index + 1))
                    .expect("generated route id is non-empty"),
                net,
                layer: arc.layer,
                width: arc.width.clone(),
                segments: vec![arc.arc.clone().into()],
            });
        }
        for (index, bezier) in candidate.beziers().iter().enumerate() {
            let net = problem
                .aliases
                .logical(bezier.net)
                .cloned()
                .ok_or(RoutingAdapterError::UnknownCandidateNet(bezier.net))?;
            routes.push(PcbRoute {
                id: RouteId::new(format!("autoroute-bezier-{}", index + 1))
                    .expect("generated route id is non-empty"),
                net,
                layer: bezier.layer,
                width: bezier.width.clone(),
                segments: vec![bezier.bezier.clone().into()],
            });
        }

        let mut vias = Vec::with_capacity(candidate.vias().len());
        let mut retained_existing = BTreeSet::new();
        let mut unknown_via_mask_intent = 0;
        for (index, via) in candidate.vias().iter().enumerate() {
            let net = problem
                .aliases
                .logical(via.net())
                .cloned()
                .ok_or(RoutingAdapterError::UnknownCandidateNet(via.net()))?;
            let drill_diameter = via
                .drill_diameter()
                .cloned()
                .ok_or(RoutingAdapterError::MissingCandidateViaDrill(index))?;
            if let Some((existing_index, existing)) =
                problem
                    .existing_vias
                    .iter()
                    .enumerate()
                    .find(|(existing_index, existing)| {
                        !retained_existing.contains(existing_index)
                            && existing.net == net
                            && existing.start_layer == via.start_layer()
                            && existing.end_layer == via.end_layer()
                            && existing.center == *via.center()
                            && existing.land_diameter == *via.land_diameter()
                            && existing.drill_diameter == drill_diameter
                            && existing.plating == via_plating(via.drill_intent())
                    })
            {
                retained_existing.insert(existing_index);
                vias.push(existing.clone());
                continue;
            }
            unknown_via_mask_intent += 1;
            vias.push(PcbVia {
                id: ViaId::new(format!("autoroute-via-{}", index + 1))
                    .expect("generated via id is non-empty"),
                net,
                start_layer: via.start_layer(),
                end_layer: via.end_layer(),
                center: via.center().clone(),
                land_diameter: via.land_diameter().clone(),
                drill_diameter,
                plating: via_plating(via.drill_intent()),
                mask: crate::ViaMaskIntent::default(),
            });
        }

        let mut omissions = Vec::new();
        if unknown_via_mask_intent != 0 {
            omissions.push(RoutingSolutionOmission::ViaMaskIntentUnavailable(
                unknown_via_mask_intent,
            ));
        }
        Ok(Self {
            routes,
            vias,
            omissions,
        })
    }

    /// Returns a layout whose routed copper is replaced by this candidate.
    pub fn replace_in(&self, layout: &PcbLayout) -> PcbLayout {
        let mut result = layout.clone();
        result.routes.clone_from(&self.routes);
        result.vias.clone_from(&self.vias);
        result
    }

    /// Returns a layout with this candidate appended after checking stable ids.
    pub fn append_to(&self, layout: &PcbLayout) -> Result<PcbLayout, RoutingAdapterError> {
        let route_ids = layout
            .routes
            .iter()
            .map(|route| route.id.as_str())
            .collect::<BTreeSet<_>>();
        if let Some(route) = self
            .routes
            .iter()
            .find(|route| route_ids.contains(route.id.as_str()))
        {
            return Err(RoutingAdapterError::IdentityCollision(
                route.id.as_str().to_owned(),
            ));
        }
        let via_ids = layout
            .vias
            .iter()
            .map(|via| via.id.as_str())
            .collect::<BTreeSet<_>>();
        if let Some(via) = self
            .vias
            .iter()
            .find(|via| via_ids.contains(via.id.as_str()))
        {
            return Err(RoutingAdapterError::IdentityCollision(
                via.id.as_str().to_owned(),
            ));
        }
        let mut result = layout.clone();
        result.routes.extend(self.routes.clone());
        result.vias.extend(self.vias.clone());
        Ok(result)
    }
}

fn straight_routes_length(
    routes: &[&PcbRoute],
    issues: &mut Vec<RoutingQualityIssue>,
) -> Option<Real> {
    let mut total = Real::zero();
    for route in routes {
        for segment in &route.segments {
            let PcbRouteSegment::Line(line) = segment else {
                issues.push(RoutingQualityIssue::UnsupportedCurvedRoute(
                    route.id.clone(),
                ));
                return None;
            };
            let dx = line.end().x.clone() - line.start().x.clone();
            let dy = line.end().y.clone() - line.start().y.clone();
            let length = (dx.clone() * dx + dy.clone() * dy).sqrt().ok()?;
            total += length;
        }
    }
    Some(total)
}

fn euclidean_mst_lower_bound(points: &[&Point2]) -> Option<Real> {
    if points.is_empty() {
        return Some(Real::zero());
    }
    let mut visited = vec![false; points.len()];
    let mut best = vec![None; points.len()];
    best[0] = Some(Real::zero());
    let mut total = Real::zero();
    for _ in 0..points.len() {
        let mut selected = None;
        for (index, distance) in best.iter().enumerate() {
            if visited[index] {
                continue;
            }
            let Some(distance) = distance else {
                continue;
            };
            selected = match selected {
                Some((best_index, best_distance)) => {
                    if distance.partial_cmp(best_distance)? == Ordering::Less {
                        Some((index, distance))
                    } else {
                        Some((best_index, best_distance))
                    }
                }
                None => Some((index, distance)),
            };
        }
        let (selected_index, selected_distance) = selected?;
        visited[selected_index] = true;
        total += selected_distance.clone();
        for index in 0..points.len() {
            if visited[index] {
                continue;
            }
            let distance = euclidean_distance(points[selected_index], points[index])?;
            let replace = match &best[index] {
                Some(current) => distance.partial_cmp(current)? == Ordering::Less,
                None => true,
            };
            if replace {
                best[index] = Some(distance);
            }
        }
    }
    Some(total)
}

fn euclidean_distance(first: &Point2, second: &Point2) -> Option<Real> {
    let dx = first.x.clone() - second.x.clone();
    let dy = first.y.clone() - second.y.clone();
    (dx.clone() * dx + dy.clone() * dy).sqrt().ok()
}

fn sum_optional<'a>(mut values: impl Iterator<Item = Option<&'a Real>>) -> Option<Real> {
    values.try_fold(Real::zero(), |total, value| Some(total + value?.clone()))
}

fn exact_ratio(numerator: Option<&Real>, denominator: Option<&Real>) -> Option<Real> {
    let (Some(numerator), Some(denominator)) = (numerator, denominator) else {
        return None;
    };
    if denominator.partial_cmp(&Real::zero()) != Some(Ordering::Greater) {
        return None;
    }
    (numerator.clone() / denominator.clone()).ok()
}

fn via_plating(intent: ViaDrillIntent) -> Plating {
    match intent {
        ViaDrillIntent::Plated => Plating::Plated,
        ViaDrillIntent::NonPlated => Plating::NonPlated,
        ViaDrillIntent::Unspecified => Plating::Unspecified,
    }
}

fn routing_terminals(
    circuit: &Circuit,
    layout: &PcbLayout,
    aliases: &RoutingNetAliases,
) -> Result<Vec<RoutingTerminal>, RoutingAdapterError> {
    let last_layer = layout
        .stackup
        .layers
        .iter()
        .filter_map(|layer| match layer.kind {
            StackupLayerKind::Conductor(layer) => Some(layer.0),
            _ => None,
        })
        .max()
        .unwrap_or(0);
    let mut terminals = Vec::new();
    for placement in &layout.placements {
        let instance = circuit
            .instances
            .iter()
            .find(|instance| instance.id == placement.instance)
            .expect("validated placement instance exists");
        let pattern = layout
            .land_patterns
            .iter()
            .find(|pattern| pattern.id == placement.land_pattern)
            .expect("validated placement pattern exists");
        for binding in &instance.pins {
            let routing_net = aliases
                .get(&binding.net)
                .ok_or_else(|| RoutingAdapterError::MissingNetAlias(binding.net.clone()))?;
            for mapping in pattern
                .pin_map
                .iter()
                .filter(|mapping| mapping.pin == binding.pin)
            {
                let pad = pattern
                    .pads
                    .iter()
                    .find(|pad| pad.id == mapping.pad)
                    .expect("validated pin mapping pad exists");
                let layers = pad
                    .copper_layers
                    .iter()
                    .map(|layer| match placement.side {
                        BoardSide::Front => *layer,
                        BoardSide::Back => TraceLayer(last_layer.saturating_sub(layer.0)),
                    })
                    .collect();
                terminals.push(RoutingTerminal {
                    instance: instance.id.clone(),
                    pin: binding.pin.clone(),
                    pad: pad.id.clone(),
                    net: binding.net.clone(),
                    routing_net,
                    center: placement.transform_point(&pad.center),
                    layers,
                });
            }
        }
    }
    Ok(terminals)
}

fn is_orthogonal_loop(vertices: &[Point2]) -> bool {
    vertices.len() >= 3
        && vertices
            .iter()
            .zip(vertices.iter().cycle().skip(1))
            .take(vertices.len())
            .all(|(start, end)| start.x == end.x || start.y == end.y)
}
