//! Deterministic negotiated-congestion board routing.
//!
//! Hypercircuit owns net scheduling and rip-up decisions. Accepted geometry is
//! emitted through hyperpath's exact trace/via carriers before it can re-enter
//! semantic PCB layout state.

use std::cmp::{Ordering, Reverse};
use std::collections::{BTreeMap, BTreeSet, BinaryHeap};
use std::fmt::{Display, Formatter};

use hypercurve::{Classification, CurvePolicy};
use hyperlattice::Point2;
use hyperlimit::{
    RingPointLocation, SegmentIntersection, classify_point_ring_even_odd,
    classify_segment_intersection,
};
use hyperpath::{
    LinePathSegment, NetId as RoutingNetId, PcbTrace, PcbViaStack, SpecctraRoute, SweptLineSegment,
    TraceLayer, ViaDrillIntent,
};
use hyperreal::Real;

use crate::{
    BoardBoundaryGeometry, BoardSide, Circuit, DifferentialPair, DifferentialPairId, EscapePolicy,
    EscapePolicyId, KeepoutScope, NetId, PadShape, PcbLayout, PcbPlacement, PcbRouteSegment,
    PlacementPinAccessDirection, PlacementPinAccessIssue, PlacementPinAccessPolicy,
    PlacementPinAccessProbeEvidence, PlacementPinAccessReport, PlacementPinAccessStatus,
    PlacementPinAccessTerminalEvidence, Plating, RouteConstraintRegion, RouteConstraintRegionId,
    RouteDirection, RouteId, RouteRuleRegionId, RoutingAdapterError, RoutingProblemReport,
    RoutingSolution, RoutingTerminal, ViaId, ViaMaskIntent, ViaStyleId, ViaStyleSpan,
};

/// Deterministic coordinate construction for negotiated routing.
#[derive(Clone, Debug, PartialEq)]
pub enum NegotiatedGridMode {
    /// Uniform exact coordinates separated by `grid_pitch`.
    Uniform,
    /// Coarse global coverage plus fine exact coordinates around retained features.
    FeatureAligned {
        /// Positive multiplier applied to `grid_pitch` for global coverage.
        coarse_pitch_multiplier: usize,
        /// Fine-pitch coordinates inserted on each side of every feature.
        feature_halo_steps: usize,
    },
    /// Coarse global coverage plus fine-pitch nodes inside explicit regions.
    ///
    /// Region boundaries and every fine coordinate are retained exactly. Fine
    /// coordinate axes may appear in the grid inventory outside their region,
    /// but only coarse intersections and nodes inside a region are searchable.
    LocallyRefined {
        /// Positive multiplier applied to `grid_pitch` for global coverage.
        coarse_pitch_multiplier: usize,
        /// Exact axis-aligned regions receiving `grid_pitch` capacity.
        regions: Vec<NegotiatedGridRefinementRegion>,
    },
}

/// Exact axis-aligned region receiving locally refined routing capacity.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedGridRefinementRegion {
    /// Inclusive lower-left corner.
    pub min: Point2,
    /// Inclusive upper-right corner.
    pub max: Point2,
}

/// Planar edge topology available to negotiated routing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NegotiatedPlanarTopology {
    /// Adjacent horizontal and vertical grid edges only.
    Orthogonal,
    /// Orthogonal edges plus exact 45-degree diagonals between equal-span cells.
    Octilinear,
    /// Bounded visibility edges between active planar nodes at any exact angle.
    AnyAngle {
        /// Nearest active planar candidates considered from one search node.
        maximum_neighbors_per_node: usize,
    },
}

/// Bounded deterministic policy for negotiated grid routing.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedRoutePolicy {
    /// Exact board-space grid pitch.
    pub grid_pitch: Real,
    /// Uniform or retained-feature-aligned coordinate construction.
    pub grid_mode: NegotiatedGridMode,
    /// Orthogonal-only or certified octilinear planar motion.
    pub planar_topology: NegotiatedPlanarTopology,
    /// Empty selects every net with at least two placed terminals.
    pub nets: Vec<NetId>,
    /// Maximum complete rip-up/reroute passes.
    pub maximum_iterations: usize,
    /// Maximum expanded search states for one terminal connection.
    pub maximum_expansions_per_connection: usize,
    /// Added cost for changing planar direction.
    pub bend_penalty: u64,
    /// Base cost for one adjacent-layer transition.
    pub via_penalty: u64,
    /// Cost multiplier for resources already used during the current pass.
    pub present_congestion_penalty: u64,
    /// Persistent cost added to every conflicted resource after a pass.
    pub history_increment: u64,
    /// Width used when a selected net has no complete net-class rule.
    pub default_trace_width: Real,
    /// Clearance used when a selected net has no complete net-class rule.
    pub default_clearance: Real,
    /// Land diameter for generated adjacent-layer transitions.
    pub via_land_diameter: Real,
    /// Drill diameter for generated adjacent-layer transitions.
    pub via_drill_diameter: Real,
    /// Surface mask policy for generated vias.
    pub via_mask: ViaMaskIntent,
}

impl Default for NegotiatedRoutePolicy {
    fn default() -> Self {
        Self {
            grid_pitch: Real::one(),
            grid_mode: NegotiatedGridMode::Uniform,
            planar_topology: NegotiatedPlanarTopology::Orthogonal,
            nets: Vec::new(),
            maximum_iterations: 32,
            maximum_expansions_per_connection: 100_000,
            bend_penalty: 2,
            via_penalty: 8,
            present_congestion_penalty: 8,
            history_increment: 2,
            default_trace_width: Real::one(),
            default_clearance: Real::zero(),
            via_land_diameter: Real::one(),
            via_drill_diameter: (Real::one() / Real::from(2))
                .expect("default via drill divisor is nonzero"),
            via_mask: ViaMaskIntent::default(),
        }
    }
}

/// Structural policy/problem error before negotiated search can start.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum NegotiatedRouterError {
    /// Circuit/layout routing handoff failed structural validation.
    InvalidProblem(RoutingAdapterError),
    /// A positive bound or physical dimension was invalid.
    InvalidPolicy,
    /// Grid construction exceeded the caller's per-connection node bound.
    GridTooLarge,
    /// The exact board-boundary query carrier could not be constructed.
    InvalidBoardBoundary(String),
    /// A selected net is absent or has fewer than two placed terminals.
    UnroutableSelectedNet(NetId),
    /// A retained non-selected route contains geometry the grid obstacle model cannot certify.
    UnsupportedFixedRoute(RouteId),
    /// Explicit selection included only one member of a retained differential pair.
    IncompleteSelectedDifferentialPair(DifferentialPairId),
    /// Pair terminals cannot be represented as one constant-spacing translated route.
    UnsupportedDifferentialPair(DifferentialPairId),
    /// One selected net belongs to more than one retained differential pair.
    OverlappingDifferentialPairNet(NetId),
}

impl Display for NegotiatedRouterError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidProblem(error) => {
                write!(formatter, "invalid PCB routing problem: {error:?}")
            }
            Self::InvalidPolicy => formatter.write_str("invalid negotiated routing policy"),
            Self::GridTooLarge => {
                formatter.write_str("routing grid exceeds the configured search bound")
            }
            Self::InvalidBoardBoundary(error) => {
                write!(formatter, "invalid exact board boundary: {error}")
            }
            Self::UnroutableSelectedNet(net) => {
                write!(
                    formatter,
                    "selected net {} has fewer than two terminals",
                    net.as_str()
                )
            }
            Self::UnsupportedFixedRoute(route) => write!(
                formatter,
                "fixed route {} contains unsupported curved geometry",
                route.as_str()
            ),
            Self::IncompleteSelectedDifferentialPair(pair) => write!(
                formatter,
                "selected nets include only one member of differential pair {}",
                pair.as_str()
            ),
            Self::UnsupportedDifferentialPair(pair) => write!(
                formatter,
                "differential pair {} does not have compatible constant-spacing terminals",
                pair.as_str()
            ),
            Self::OverlappingDifferentialPairNet(net) => write!(
                formatter,
                "net {} belongs to more than one selected differential pair",
                net.as_str()
            ),
        }
    }
}

impl std::error::Error for NegotiatedRouterError {}

/// Why a selected net could not produce a complete tree in one pass.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum NegotiatedRouteFailure {
    /// A placed terminal did not lie exactly on the authored routing grid.
    OffGridTerminal { net: NetId, terminal: usize },
    /// No legal path connected this terminal to the growing net tree.
    NoPath { net: NetId, terminal: usize },
    /// The per-connection expansion bound was reached.
    ExpansionLimit { net: NetId, terminal: usize },
    /// Exact containment, clearance, or ordering could not be decided.
    Indeterminate { net: NetId, terminal: usize },
    /// No joint path connected both members of a retained differential pair.
    DifferentialPairNoPath(DifferentialPairId),
    /// The joint differential-pair expansion bound was reached.
    DifferentialPairExpansionLimit(DifferentialPairId),
    /// Exact pair spacing, containment, clearance, or ordering was indeterminate.
    DifferentialPairIndeterminate(DifferentialPairId),
}

/// Evidence from one complete rip-up/reroute pass.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NegotiatedRouteIteration {
    /// Zero-based pass index.
    pub iteration: usize,
    /// Search states removed from the priority queues during this pass.
    pub expanded_states: usize,
    /// Grid resources occupied by more than one net after the pass.
    pub conflicted_resources: usize,
    /// Nets that did not build a complete terminal tree.
    pub failed_nets: usize,
}

/// One exact routed grid node retained for solver replay or visualization.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedRouteNodeState {
    /// Exact board-space coordinate.
    pub center: Point2,
    /// Physical conductor layer.
    pub layer: TraceLayer,
}

/// Successful provisional paths for one logical net during one routing pass.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedRouteNetState {
    /// Logical net.
    pub net: NetId,
    /// Exact node sequences for each terminal-tree connection.
    pub paths: Vec<Vec<NegotiatedRouteNodeState>>,
}

/// Exact geometry of one over-subscribed negotiated-routing resource.
#[derive(Clone, Debug, PartialEq)]
pub enum NegotiatedRouteConflictGeometry {
    /// Multiple nets claim one planar grid node.
    Node { center: Point2, layer: TraceLayer },
    /// Multiple nets claim one planar grid edge.
    Segment {
        start: Point2,
        end: Point2,
        layer: TraceLayer,
    },
    /// Opposing diagonal edges cross inside one exact grid cell.
    DiagonalCell {
        lower_left: Point2,
        upper_right: Point2,
        layer: TraceLayer,
    },
    /// Multiple nets claim one adjacent-layer transition.
    Via {
        center: Point2,
        start_layer: TraceLayer,
        end_layer: TraceLayer,
    },
}

/// Source-addressable conflict retained from one provisional routing pass.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedRouteConflictState {
    /// Exact claimed resource geometry.
    pub geometry: NegotiatedRouteConflictGeometry,
    /// Conflicting logical nets in stable identity order.
    pub nets: Vec<NetId>,
}

/// Replayable exact state after one complete rip-up/reroute pass.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedRouteIterationState {
    /// Zero-based pass index.
    pub iteration: usize,
    /// Every successfully constructed provisional net path.
    pub nets: Vec<NegotiatedRouteNetState>,
    /// Every resource claimed by more than one logical net.
    pub conflicts: Vec<NegotiatedRouteConflictState>,
    /// Typed connections that could not be constructed in this pass.
    pub failures: Vec<NegotiatedRouteFailure>,
}

/// Audited coordinate inventory used by one negotiated routing run.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedGridEvidence {
    /// Caller-selected coordinate construction.
    pub mode: NegotiatedGridMode,
    /// Final exact X-coordinate count.
    pub x_coordinates: usize,
    /// Final exact Y-coordinate count.
    pub y_coordinates: usize,
    /// Conductor-layer count.
    pub conductor_layers: usize,
    /// X coordinates added by retained feature alignment and fine halos.
    pub injected_x_coordinates: usize,
    /// Y coordinates added by retained feature alignment and fine halos.
    pub injected_y_coordinates: usize,
    /// Searchable planar nodes after local activation is applied.
    pub active_planar_nodes: usize,
    /// Searchable planar nodes added beyond the coarse global mesh.
    pub refined_planar_nodes: usize,
}

/// Deterministic work accounting for one negotiated-routing run.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NegotiatedRouteWorkEvidence {
    /// Searchable grid nodes across every exact coordinate and conductor layer.
    pub grid_nodes: usize,
    /// Configured complete rip-up/reroute pass bound.
    pub iteration_budget: usize,
    /// Complete passes actually executed.
    pub iterations_executed: usize,
    /// Configured expansion bound for one terminal connection.
    pub expansion_budget_per_connection: usize,
    /// Search states removed from every priority queue over the complete run.
    pub expanded_states_total: usize,
    /// Largest per-pass expansion count.
    pub peak_expanded_states_per_iteration: usize,
    /// Final-pass connections that reached their expansion bound.
    pub expansion_limit_failures: usize,
    /// Whether the run ended by consuming its complete pass budget.
    pub iteration_budget_exhausted: bool,
}

/// Exact post-route evidence for one atomically routed differential pair.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedDifferentialPairNeckdownEvidence {
    /// Exact trace width emitted through both terminal fanouts.
    pub trace_width: Real,
    /// Retained reduced edge-to-edge terminal spacing.
    pub spacing: Real,
    /// Exact symmetric centerline transition length for each member.
    pub transition_length: Real,
    /// Planar grid edges in the two source-side fanout legs.
    pub source_planar_edges: usize,
    /// Planar grid edges in the two target-side fanout legs.
    pub target_planar_edges: usize,
}

/// Exact post-route evidence for one atomically routed differential pair.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedDifferentialPairEvidence {
    /// Retained pair identity.
    pub pair: DifferentialPairId,
    /// Exact center-to-center separation implied by widths and authored edge gap.
    pub center_spacing: Real,
    /// Positive-member planar centerline length.
    pub positive_length: Real,
    /// Negative-member planar centerline length.
    pub negative_length: Real,
    /// Absolute planar length difference.
    pub skew: Real,
    /// Synchronized vias emitted for each member.
    pub paired_vias: usize,
    /// Bounded terminal neck-down use, when terminal spacing required it.
    pub neckdown: Option<NegotiatedDifferentialPairNeckdownEvidence>,
}

/// Audited use of one retained polygonal route-constraint region.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NegotiatedRouteConstraintEvidence {
    /// Retained region identity.
    pub region: RouteConstraintRegionId,
    /// Accepted planar grid edges governed by this region.
    pub constrained_planar_edges: usize,
    /// Accepted adjacent-layer transitions governed by this region.
    pub constrained_vias: usize,
}

/// Exact-predicate use evidence for one regional width/clearance rule.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NegotiatedRouteRuleRegionEvidence {
    /// Retained regional-rule identity.
    pub region: RouteRuleRegionId,
    /// Accepted planar edges touching the governed region.
    pub constrained_planar_edges: usize,
    /// Accepted adjacent-layer transitions touching the governed region.
    pub constrained_vias: usize,
    /// Governed planar edges receiving the regional width floor.
    pub width_override_edges: usize,
    /// Governed planar/via edges receiving the regional clearance floor.
    pub clearance_override_edges: usize,
}

/// Audited use of one retained terminal-local escape policy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NegotiatedEscapePolicyEvidence {
    /// Retained escape-policy identity.
    pub policy: EscapePolicyId,
    /// Accepted planar grid edges inside governed escape envelopes.
    pub constrained_planar_edges: usize,
    /// Accepted adjacent-layer transitions inside governed escape envelopes.
    pub constrained_vias: usize,
}

/// Generated-via construction evidence for one routed logical net.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedViaStyleEvidence {
    /// Routed logical net.
    pub net: NetId,
    /// Selected named construction, or `None` for class/policy fallback dimensions.
    pub style: Option<ViaStyleId>,
    /// Number of generated transitions using this construction.
    pub generated_vias: usize,
    /// Exact emitted copper land diameter.
    pub land_diameter: Real,
    /// Exact emitted finished drill diameter.
    pub drill_diameter: Real,
    /// Emitted drill plating intent.
    pub plating: Plating,
    /// Emitted front/back mask intent.
    pub mask: ViaMaskIntent,
}

/// Terminal status of a negotiated routing run.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NegotiatedRouteStatus {
    /// Every selected net was routed without shared resources.
    Complete,
    /// The iteration bound ended with congestion or unrouted nets.
    IterationLimit,
}

/// Audited negotiated route result.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedRouteReport {
    /// Final run status.
    pub status: NegotiatedRouteStatus,
    /// Exact caller policy retained for deterministic replay.
    pub policy: NegotiatedRoutePolicy,
    /// Selected logical nets in deterministic order.
    pub selected_nets: Vec<NetId>,
    /// Differential-pair constraints routed atomically.
    pub routed_differential_pairs: Vec<DifferentialPairId>,
    /// Exact spacing, length, skew, and synchronized-via evidence for routed pairs.
    pub differential_pair_evidence: Vec<NegotiatedDifferentialPairEvidence>,
    /// Exact-predicate use counts for retained polygonal route constraints.
    pub route_constraint_evidence: Vec<NegotiatedRouteConstraintEvidence>,
    /// Exact-predicate use counts for retained regional width/clearance rules.
    pub route_rule_region_evidence: Vec<NegotiatedRouteRuleRegionEvidence>,
    /// Exact-distance use counts for retained package escape policies.
    pub escape_policy_evidence: Vec<NegotiatedEscapePolicyEvidence>,
    /// Per-net named/fallback via construction and generated-use evidence.
    pub via_style_evidence: Vec<NegotiatedViaStyleEvidence>,
    /// Per-pass congestion and search evidence.
    pub iterations: Vec<NegotiatedRouteIteration>,
    /// Exact replay/visualization state retained after every routing pass.
    pub iteration_states: Vec<NegotiatedRouteIterationState>,
    /// Exact coordinate inventory used by every search pass.
    pub grid: NegotiatedGridEvidence,
    /// Aggregate deterministic search-work and budget evidence.
    pub work: NegotiatedRouteWorkEvidence,
    /// Failures from the final pass.
    pub failures: Vec<NegotiatedRouteFailure>,
    /// Conflict-free hyperpath route when complete.
    pub route: Option<SpecctraRoute>,
    /// Conflict-free semantic mapping when complete.
    pub solution: Option<RoutingSolution>,
}

impl NegotiatedRouteReport {
    /// Replaces only selected-net routes/vias, preserving unrelated authored copper.
    pub fn apply_to(&self, layout: &PcbLayout) -> Option<PcbLayout> {
        let solution = self.solution.as_ref()?;
        let selected = self.selected_nets.iter().collect::<BTreeSet<_>>();
        let mut result = layout.clone();
        result.routes.retain(|route| !selected.contains(&route.net));
        result.vias.retain(|via| !selected.contains(&via.net));
        result.routes.extend(solution.routes.clone());
        result.vias.extend(solution.vias.clone());
        Some(result)
    }
}

/// Bounded policy for automatically synthesizing sparse routing capacity.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedAdaptiveRoutePolicy {
    /// Electrical, physical, congestion, and work policy used by every round.
    ///
    /// `grid_mode` is retained but replaced by the audited coarse/refined mode
    /// selected for each adaptive round.
    pub route_policy: NegotiatedRoutePolicy,
    /// Coarse global pitch as a positive multiple of `route_policy.grid_pitch`.
    pub coarse_pitch_multiplier: usize,
    /// Fine-pitch steps added around each conflict or failed-net envelope.
    pub refinement_padding_steps: usize,
    /// Maximum refined reruns after the initial coarse pass.
    pub maximum_refinement_rounds: usize,
    /// Maximum merged exact refinement regions admitted to a rerun.
    pub maximum_regions: usize,
}

impl Default for NegotiatedAdaptiveRoutePolicy {
    fn default() -> Self {
        Self {
            route_policy: NegotiatedRoutePolicy::default(),
            coarse_pitch_multiplier: 4,
            refinement_padding_steps: 2,
            maximum_refinement_rounds: 4,
            maximum_regions: 32,
        }
    }
}

/// Terminal status of bounded automatic routing-capacity refinement.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NegotiatedAdaptiveRouteStatus {
    /// A coarse or refined round completed without conflicts or failures.
    Complete,
    /// The configured number of refined reruns was consumed.
    RefinementLimit,
    /// The next exact refinement set exceeded the caller's region bound.
    RegionLimit,
    /// The last incomplete round produced no new exact refinement capacity.
    NoProgress,
}

/// One fully replayable adaptive-routing round and its proposed next mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedAdaptiveRouteRound {
    /// Zero-based adaptive round; round zero is the coarse pass.
    pub round: usize,
    /// Complete ordinary negotiated-routing evidence for this mesh.
    pub report: NegotiatedRouteReport,
    /// Merged exact regions proposed after this round, including a refused set.
    pub proposed_regions: Vec<NegotiatedGridRefinementRegion>,
}

/// Audited result of bounded automatic local-capacity synthesis.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiatedAdaptiveRouteReport {
    /// Why adaptation terminated.
    pub status: NegotiatedAdaptiveRouteStatus,
    /// Exact caller policy retained for replay.
    pub policy: NegotiatedAdaptiveRoutePolicy,
    /// Every coarse/refined routing run in deterministic order.
    pub rounds: Vec<NegotiatedAdaptiveRouteRound>,
    /// Final merged local-capacity regions.
    pub refinement_regions: Vec<NegotiatedGridRefinementRegion>,
}

impl NegotiatedAdaptiveRouteReport {
    /// Last ordinary negotiated-routing report.
    pub fn final_report(&self) -> &NegotiatedRouteReport {
        &self
            .rounds
            .last()
            .expect("adaptive routing always executes its coarse round")
            .report
    }

    /// Applies the complete final solution while preserving unrelated copper.
    pub fn apply_to(&self, layout: &PcbLayout) -> Option<PcbLayout> {
        self.final_report().apply_to(layout)
    }
}

impl PcbLayout {
    /// Routes on a coarse mesh, synthesizing bounded exact local capacity from
    /// retained conflicts and failed-net envelopes until routing completes or
    /// an adaptive bound is reached.
    pub fn adaptive_negotiated_autoroute(
        &self,
        circuit: &Circuit,
        policy: NegotiatedAdaptiveRoutePolicy,
    ) -> Result<NegotiatedAdaptiveRouteReport, NegotiatedRouterError> {
        if policy.coarse_pitch_multiplier == 0
            || policy.refinement_padding_steps == 0
            || policy.maximum_regions == 0
        {
            return Err(NegotiatedRouterError::InvalidPolicy);
        }
        let mut validated_route_policy = policy.route_policy.clone();
        validated_route_policy.grid_mode = NegotiatedGridMode::FeatureAligned {
            coarse_pitch_multiplier: policy.coarse_pitch_multiplier,
            feature_halo_steps: 0,
        };
        validate_policy(&validated_route_policy)?;
        let handoff = RoutingProblemReport::from_layout(circuit, self)
            .map_err(NegotiatedRouterError::InvalidProblem)?;
        let boundary = self
            .outline
            .boundary_geometry()
            .map_err(|error| NegotiatedRouterError::InvalidBoardBoundary(error.to_string()))?;
        let (board_min, board_max) = boundary.exterior_bounds();
        let coarse_pitch = policy.route_policy.grid_pitch.clone()
            * Real::from(policy.coarse_pitch_multiplier as u128);
        let coarse_xs = grid_axis(
            board_min.x.clone(),
            board_max.x.clone(),
            &coarse_pitch,
            policy.route_policy.maximum_expansions_per_connection,
        )?;
        let coarse_ys = grid_axis(
            board_min.y.clone(),
            board_max.y.clone(),
            &coarse_pitch,
            policy.route_policy.maximum_expansions_per_connection,
        )?;
        let mut regions = Vec::<NegotiatedGridRefinementRegion>::new();
        let mut rounds = Vec::<NegotiatedAdaptiveRouteRound>::new();
        for round in 0..=policy.maximum_refinement_rounds {
            let mut round_policy = policy.route_policy.clone();
            round_policy.grid_mode = if round == 0 {
                NegotiatedGridMode::FeatureAligned {
                    coarse_pitch_multiplier: policy.coarse_pitch_multiplier,
                    feature_halo_steps: 0,
                }
            } else {
                NegotiatedGridMode::LocallyRefined {
                    coarse_pitch_multiplier: policy.coarse_pitch_multiplier,
                    regions: regions.clone(),
                }
            };
            let report = self.negotiated_autoroute(circuit, round_policy)?;
            if report.status == NegotiatedRouteStatus::Complete {
                rounds.push(NegotiatedAdaptiveRouteRound {
                    round,
                    report,
                    proposed_regions: Vec::new(),
                });
                return Ok(NegotiatedAdaptiveRouteReport {
                    status: NegotiatedAdaptiveRouteStatus::Complete,
                    policy,
                    rounds,
                    refinement_regions: regions,
                });
            }
            let proposed = synthesize_refinement_regions(
                self, &handoff, &report, &regions, &board_min, &board_max, &coarse_xs, &coarse_ys,
                &policy,
            )?;
            let no_progress = proposed == regions;
            let region_limit = proposed.len() > policy.maximum_regions;
            rounds.push(NegotiatedAdaptiveRouteRound {
                round,
                report,
                proposed_regions: proposed.clone(),
            });
            if region_limit {
                return Ok(NegotiatedAdaptiveRouteReport {
                    status: NegotiatedAdaptiveRouteStatus::RegionLimit,
                    policy,
                    rounds,
                    refinement_regions: regions,
                });
            }
            if no_progress {
                return Ok(NegotiatedAdaptiveRouteReport {
                    status: NegotiatedAdaptiveRouteStatus::NoProgress,
                    policy,
                    rounds,
                    refinement_regions: regions,
                });
            }
            if round == policy.maximum_refinement_rounds {
                return Ok(NegotiatedAdaptiveRouteReport {
                    status: NegotiatedAdaptiveRouteStatus::RefinementLimit,
                    policy,
                    rounds,
                    refinement_regions: regions,
                });
            }
            regions = proposed;
        }
        unreachable!("inclusive bounded loop always returns")
    }

    /// Routes selected placed-terminal nets with deterministic negotiated congestion.
    pub fn negotiated_autoroute(
        &self,
        circuit: &Circuit,
        policy: NegotiatedRoutePolicy,
    ) -> Result<NegotiatedRouteReport, NegotiatedRouterError> {
        let handoff = RoutingProblemReport::from_layout(circuit, self)
            .map_err(NegotiatedRouterError::InvalidProblem)?;
        validate_policy(&policy)?;
        let grid = Grid::new(self, &handoff, &policy)?;
        let mut selected = if policy.nets.is_empty() {
            handoff
                .problem
                .terminals
                .iter()
                .fold(BTreeMap::<NetId, usize>::new(), |mut counts, terminal| {
                    *counts.entry(terminal.net.clone()).or_default() += 1;
                    counts
                })
                .into_iter()
                .filter_map(|(net, count)| (count >= 2).then_some(net))
                .collect::<Vec<_>>()
        } else {
            let mut nets = policy.nets.clone();
            nets.sort();
            nets.dedup();
            nets
        };
        selected.sort();
        for net in &selected {
            if handoff
                .problem
                .terminals
                .iter()
                .filter(|terminal| &terminal.net == net)
                .count()
                < 2
            {
                return Err(NegotiatedRouterError::UnroutableSelectedNet(net.clone()));
            }
        }
        let selected_set = selected.iter().collect::<BTreeSet<_>>();
        let mut selected_pairs = Vec::<&DifferentialPair>::new();
        let mut paired_nets = BTreeSet::<NetId>::new();
        for pair in &self.rules.differential_pairs {
            let positive = selected_set.contains(&pair.positive);
            let negative = selected_set.contains(&pair.negative);
            match (positive, negative) {
                (true, true) => {
                    selected_pairs.push(pair);
                    if !paired_nets.insert(pair.positive.clone()) {
                        return Err(NegotiatedRouterError::OverlappingDifferentialPairNet(
                            pair.positive.clone(),
                        ));
                    }
                    if !paired_nets.insert(pair.negative.clone()) {
                        return Err(NegotiatedRouterError::OverlappingDifferentialPairNet(
                            pair.negative.clone(),
                        ));
                    }
                }
                (true, false) | (false, true) => {
                    return Err(NegotiatedRouterError::IncompleteSelectedDifferentialPair(
                        pair.id.clone(),
                    ));
                }
                (false, false) => {}
            }
        }
        selected_pairs.sort_by(|left, right| left.id.cmp(&right.id));
        let selected_pair_ids = selected_pairs
            .iter()
            .map(|pair| pair.id.clone())
            .collect::<Vec<_>>();
        validate_selected_spacing(self, &handoff, &selected, &grid, &policy)?;
        let pair_problems = selected_pairs
            .iter()
            .map(|pair| prepare_differential_pair(self, &handoff, &grid, pair, &policy))
            .collect::<Result<Vec<_>, _>>()?;
        let fixed = FixedObstacles::from_layout(self, &handoff, &selected, &policy)?;

        let mut history = BTreeMap::<Resource, u64>::new();
        let mut iteration_evidence = Vec::new();
        let mut iteration_states = Vec::new();
        let mut final_failures = Vec::new();
        for iteration in 0..policy.maximum_iterations {
            let mut occupancy = BTreeMap::<Resource, BTreeSet<NetId>>::new();
            let mut routes = BTreeMap::new();
            let mut failures = Vec::new();
            let mut expanded_states = 0;
            let router = RouterContext {
                layout: self,
                handoff: &handoff,
                grid: &grid,
                policy: &policy,
                history: &history,
                fixed: &fixed,
            };
            for problem in &pair_problems {
                match route_differential_pair(&router, problem, &occupancy, &mut expanded_states) {
                    Ok((positive, negative)) => {
                        for resource in &positive.resources {
                            occupancy
                                .entry(*resource)
                                .or_default()
                                .insert(problem.pair.positive.clone());
                        }
                        for resource in &negative.resources {
                            occupancy
                                .entry(*resource)
                                .or_default()
                                .insert(problem.pair.negative.clone());
                        }
                        routes.insert(problem.pair.positive.clone(), positive);
                        routes.insert(problem.pair.negative.clone(), negative);
                    }
                    Err(failure) => failures.push(failure),
                }
            }
            for net in selected.iter().filter(|net| !paired_nets.contains(*net)) {
                match route_net(&router, net, &occupancy, &mut expanded_states) {
                    Ok(route) => {
                        for resource in &route.resources {
                            occupancy.entry(*resource).or_default().insert(net.clone());
                        }
                        routes.insert(net.clone(), route);
                    }
                    Err(failure) => failures.push(failure),
                }
            }
            account_inter_route_clearance(self, &handoff, &grid, &policy, &routes, &mut occupancy)?;
            let conflicts = occupancy
                .iter()
                .filter_map(|(resource, users)| (users.len() > 1).then_some(*resource))
                .collect::<Vec<_>>();
            iteration_evidence.push(NegotiatedRouteIteration {
                iteration,
                expanded_states,
                conflicted_resources: conflicts.len(),
                failed_nets: failures.iter().map(failed_net_count).sum(),
            });
            iteration_states.push(retain_iteration_state(
                iteration, &routes, &occupancy, &grid, &failures,
            ));
            final_failures = failures;
            if conflicts.is_empty() && final_failures.is_empty() {
                let differential_pair_evidence = pair_problems
                    .iter()
                    .map(|problem| differential_pair_evidence(problem, &routes, &grid))
                    .collect::<Result<Vec<_>, _>>()?;
                let route_constraint_evidence = route_constraint_evidence(self, &grid, &routes)?;
                let route_rule_region_evidence = route_rule_region_evidence(self, &grid, &routes)?;
                let escape_policy_evidence =
                    escape_policy_evidence(self, &handoff, &grid, &routes)?;
                let route = lower_routes(self, &handoff, &grid, &routes, &policy)?;
                let mut solution = RoutingSolution::from_hyperpath(&handoff.problem, &route)
                    .map_err(NegotiatedRouterError::InvalidProblem)?;
                stabilize_solution_ids(self, &mut solution, &policy);
                solution.omissions.retain(|omission| {
                    !matches!(
                        omission,
                        crate::RoutingSolutionOmission::ViaMaskIntentUnavailable(_)
                    )
                });
                let via_style_evidence = via_style_evidence(self, &selected, &solution, &policy);
                let work = route_work_evidence(
                    &policy,
                    &grid,
                    &iteration_evidence,
                    &final_failures,
                    false,
                );
                return Ok(NegotiatedRouteReport {
                    status: NegotiatedRouteStatus::Complete,
                    policy: policy.clone(),
                    selected_nets: selected,
                    routed_differential_pairs: selected_pair_ids,
                    differential_pair_evidence,
                    route_constraint_evidence,
                    route_rule_region_evidence,
                    escape_policy_evidence,
                    via_style_evidence,
                    iterations: iteration_evidence,
                    iteration_states,
                    grid: grid.evidence(&policy.grid_mode),
                    work,
                    failures: Vec::new(),
                    route: Some(route),
                    solution: Some(solution),
                });
            }
            for resource in conflicts {
                *history.entry(resource).or_default() = history
                    .get(&resource)
                    .copied()
                    .unwrap_or_default()
                    .saturating_add(policy.history_increment);
            }
        }
        let work = route_work_evidence(&policy, &grid, &iteration_evidence, &final_failures, true);
        Ok(NegotiatedRouteReport {
            status: NegotiatedRouteStatus::IterationLimit,
            policy: policy.clone(),
            selected_nets: selected,
            routed_differential_pairs: selected_pair_ids,
            differential_pair_evidence: Vec::new(),
            route_constraint_evidence: Vec::new(),
            route_rule_region_evidence: Vec::new(),
            escape_policy_evidence: Vec::new(),
            via_style_evidence: Vec::new(),
            iterations: iteration_evidence,
            iteration_states,
            grid: grid.evidence(&policy.grid_mode),
            work,
            failures: final_failures,
            route: None,
            solution: None,
        })
    }
}

pub(crate) fn placement_pin_access_report(
    layout: &PcbLayout,
    circuit: &Circuit,
    policy: &PlacementPinAccessPolicy,
) -> PlacementPinAccessReport {
    let mut report = PlacementPinAccessReport {
        policy: policy.clone(),
        terminals: Vec::new(),
        issues: Vec::new(),
    };
    let positive = |value: &Real| value.partial_cmp(&Real::zero()) == Some(Ordering::Greater);
    let nonnegative = |value: &Real| {
        matches!(
            value.partial_cmp(&Real::zero()),
            Some(Ordering::Equal | Ordering::Greater)
        )
    };
    if !positive(&policy.probe_distance)
        || !positive(&policy.minimum_trace_width)
        || !nonnegative(&policy.minimum_clearance)
    {
        report.issues.push(PlacementPinAccessIssue::InvalidPolicy);
        return report;
    }
    let handoff = match RoutingProblemReport::from_layout(circuit, layout) {
        Ok(handoff) => handoff,
        Err(_) => {
            report.issues.push(PlacementPinAccessIssue::InvalidProblem);
            return report;
        }
    };
    let boundary = match layout.outline.boundary_geometry() {
        Ok(boundary) => boundary,
        Err(error) => {
            report
                .issues
                .push(PlacementPinAccessIssue::InvalidBoardBoundary(
                    error.to_string(),
                ));
            return report;
        }
    };
    let route_policy = NegotiatedRoutePolicy {
        default_trace_width: policy.minimum_trace_width.clone(),
        default_clearance: policy.minimum_clearance.clone(),
        ..NegotiatedRoutePolicy::default()
    };
    let fixed = match FixedObstacles::from_layout(layout, &handoff, &[], &route_policy) {
        Ok(fixed) => fixed,
        Err(NegotiatedRouterError::UnsupportedFixedRoute(route)) => {
            report
                .issues
                .push(PlacementPinAccessIssue::UnsupportedFixedRoute(route));
            return report;
        }
        Err(_) => {
            report.issues.push(PlacementPinAccessIssue::InvalidProblem);
            return report;
        }
    };
    let directions = [
        PlacementPinAccessDirection::NegativeX,
        PlacementPinAccessDirection::PositiveX,
        PlacementPinAccessDirection::NegativeY,
        PlacementPinAccessDirection::PositiveY,
    ];
    for terminal in &handoff.problem.terminals {
        let mut evidence = PlacementPinAccessTerminalEvidence {
            instance: terminal.instance.clone(),
            pin: terminal.pin.clone(),
            pad: terminal.pad.clone(),
            net: terminal.net.clone(),
            center: terminal.center.clone(),
            probes: Vec::new(),
        };
        for layer in &terminal.layers {
            for direction in directions {
                let end = pin_access_probe_end(&terminal.center, direction, &policy.probe_distance);
                let status = pin_access_probe_status(
                    layout,
                    &handoff,
                    &boundary,
                    &fixed,
                    &route_policy,
                    terminal,
                    *layer,
                    direction,
                    &end,
                    policy,
                );
                evidence.probes.push(PlacementPinAccessProbeEvidence {
                    layer: *layer,
                    direction,
                    status,
                });
            }
        }
        report.terminals.push(evidence);
    }
    report
}

fn pin_access_probe_end(
    start: &Point2,
    direction: PlacementPinAccessDirection,
    distance: &Real,
) -> Point2 {
    match direction {
        PlacementPinAccessDirection::NegativeX => {
            Point2::new(start.x.clone() - distance.clone(), start.y.clone())
        }
        PlacementPinAccessDirection::PositiveX => {
            Point2::new(start.x.clone() + distance.clone(), start.y.clone())
        }
        PlacementPinAccessDirection::NegativeY => {
            Point2::new(start.x.clone(), start.y.clone() - distance.clone())
        }
        PlacementPinAccessDirection::PositiveY => {
            Point2::new(start.x.clone(), start.y.clone() + distance.clone())
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn pin_access_probe_status(
    layout: &PcbLayout,
    handoff: &RoutingProblemReport,
    boundary: &BoardBoundaryGeometry,
    fixed: &FixedObstacles,
    route_policy: &NegotiatedRoutePolicy,
    terminal: &RoutingTerminal,
    layer: TraceLayer,
    direction: PlacementPinAccessDirection,
    end: &Point2,
    policy: &PlacementPinAccessPolicy,
) -> PlacementPinAccessStatus {
    let start = &terminal.center;
    let route_direction = match direction {
        PlacementPinAccessDirection::NegativeX | PlacementPinAccessDirection::PositiveX => {
            RouteDirection::Horizontal
        }
        PlacementPinAccessDirection::NegativeY | PlacementPinAccessDirection::PositiveY => {
            RouteDirection::Vertical
        }
    };
    let Some((rule_width, rule_clearance)) =
        edge_net_rule(layout, handoff, &terminal.net, route_policy, start, end)
    else {
        return PlacementPinAccessStatus::Indeterminate;
    };
    let Some(width) = exact_max(&rule_width, &policy.minimum_trace_width) else {
        return PlacementPinAccessStatus::Indeterminate;
    };
    let Some(clearance) = exact_max(&rule_clearance, &policy.minimum_clearance) else {
        return PlacementPinAccessStatus::Indeterminate;
    };
    let Ok(half_width) = width / Real::from(2) else {
        return PlacementPinAccessStatus::Indeterminate;
    };
    let expanded = half_width.clone() + clearance.clone();
    let Some(physical) = segment_is_legal(layout, boundary, start, end, &expanded, layer) else {
        return PlacementPinAccessStatus::Indeterminate;
    };
    if !physical {
        return PlacementPinAccessStatus::Blocked;
    }
    for region in &layout.rules.route_constraint_regions {
        if !constraint_selects_net(&region.nets, &terminal.net) {
            continue;
        }
        let Some(touches) = edge_touches_region(start, end, &region.boundary) else {
            return PlacementPinAccessStatus::Indeterminate;
        };
        if touches
            && (!region.allowed_layers.contains(&layer)
                || !region.allowed_directions.contains(&route_direction))
        {
            return PlacementPinAccessStatus::Blocked;
        }
    }
    for escape in &layout.rules.escape_policies {
        if escape.instances.contains(&terminal.instance)
            && constraint_selects_net(&escape.nets, &terminal.net)
            && (!escape.allowed_layers.contains(&layer)
                || !escape.allowed_directions.contains(&route_direction))
        {
            return PlacementPinAccessStatus::Blocked;
        }
    }
    match segment_clear_of_fixed(
        start,
        end,
        ClearanceSubject {
            physical_radius: &half_width,
            clearance: &clearance,
            net: &terminal.net,
            paired_net: None,
        },
        layer,
        fixed,
    ) {
        Some(true) => PlacementPinAccessStatus::Accessible,
        Some(false) => PlacementPinAccessStatus::Blocked,
        None => PlacementPinAccessStatus::Indeterminate,
    }
}

fn route_work_evidence(
    policy: &NegotiatedRoutePolicy,
    grid: &Grid,
    iterations: &[NegotiatedRouteIteration],
    failures: &[NegotiatedRouteFailure],
    iteration_budget_exhausted: bool,
) -> NegotiatedRouteWorkEvidence {
    NegotiatedRouteWorkEvidence {
        grid_nodes: grid.planar_node_count().saturating_mul(grid.layers.len()),
        iteration_budget: policy.maximum_iterations,
        iterations_executed: iterations.len(),
        expansion_budget_per_connection: policy.maximum_expansions_per_connection,
        expanded_states_total: iterations.iter().fold(0_usize, |total, iteration| {
            total.saturating_add(iteration.expanded_states)
        }),
        peak_expanded_states_per_iteration: iterations
            .iter()
            .map(|iteration| iteration.expanded_states)
            .max()
            .unwrap_or_default(),
        expansion_limit_failures: failures
            .iter()
            .filter(|failure| {
                matches!(
                    failure,
                    NegotiatedRouteFailure::ExpansionLimit { .. }
                        | NegotiatedRouteFailure::DifferentialPairExpansionLimit(_)
                )
            })
            .count(),
        iteration_budget_exhausted,
    }
}

fn failed_net_count(failure: &NegotiatedRouteFailure) -> usize {
    match failure {
        NegotiatedRouteFailure::DifferentialPairNoPath(_)
        | NegotiatedRouteFailure::DifferentialPairExpansionLimit(_)
        | NegotiatedRouteFailure::DifferentialPairIndeterminate(_) => 2,
        _ => 1,
    }
}

fn retain_iteration_state(
    iteration: usize,
    routes: &BTreeMap<NetId, NetRoute>,
    occupancy: &BTreeMap<Resource, BTreeSet<NetId>>,
    grid: &Grid,
    failures: &[NegotiatedRouteFailure],
) -> NegotiatedRouteIterationState {
    let nets = routes
        .iter()
        .map(|(net, route)| NegotiatedRouteNetState {
            net: net.clone(),
            paths: route
                .paths
                .iter()
                .map(|path| {
                    path.iter()
                        .map(|node| NegotiatedRouteNodeState {
                            center: grid.point(*node),
                            layer: grid.layers[node.layer],
                        })
                        .collect()
                })
                .collect(),
        })
        .collect();
    let conflicts = occupancy
        .iter()
        .filter(|(_, nets)| nets.len() > 1)
        .map(|(resource, nets)| NegotiatedRouteConflictState {
            geometry: match resource {
                Resource::Node(node) => NegotiatedRouteConflictGeometry::Node {
                    center: grid.point(*node),
                    layer: grid.layers[node.layer],
                },
                Resource::Edge(first, second) => NegotiatedRouteConflictGeometry::Segment {
                    start: grid.point(*first),
                    end: grid.point(*second),
                    layer: grid.layers[first.layer],
                },
                Resource::DiagonalCell(lower_left, upper_right) => {
                    NegotiatedRouteConflictGeometry::DiagonalCell {
                        lower_left: grid.point(*lower_left),
                        upper_right: grid.point(*upper_right),
                        layer: grid.layers[lower_left.layer],
                    }
                }
                Resource::Via(first, second) => {
                    let first_layer = grid.layers[first.layer];
                    let second_layer = grid.layers[second.layer];
                    let (start_layer, end_layer) = if first_layer <= second_layer {
                        (first_layer, second_layer)
                    } else {
                        (second_layer, first_layer)
                    };
                    NegotiatedRouteConflictGeometry::Via {
                        center: grid.point(*first),
                        start_layer,
                        end_layer,
                    }
                }
            },
            nets: nets.iter().cloned().collect(),
        })
        .collect();
    NegotiatedRouteIterationState {
        iteration,
        nets,
        conflicts,
        failures: failures.to_vec(),
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
struct Node {
    x: usize,
    y: usize,
    layer: usize,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
enum Resource {
    Node(Node),
    Edge(Node, Node),
    DiagonalCell(Node, Node),
    Via(Node, Node),
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
enum Direction {
    Horizontal,
    Vertical,
    DiagonalRising,
    DiagonalFalling,
    Arbitrary,
    Via,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct SearchState {
    node: Node,
    direction: Option<Direction>,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct PairNodes {
    positive: Node,
    negative: Node,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct PairSearchState {
    nodes: PairNodes,
    direction: Option<Direction>,
}

#[derive(Clone, Debug)]
struct Grid {
    xs: Vec<Real>,
    ys: Vec<Real>,
    layers: Vec<TraceLayer>,
    boundary: BoardBoundaryGeometry,
    injected_x_coordinates: usize,
    injected_y_coordinates: usize,
    active_planar_nodes: Option<BTreeSet<(usize, usize)>>,
    coarse_planar_nodes: usize,
}

impl Grid {
    fn new(
        layout: &PcbLayout,
        handoff: &RoutingProblemReport,
        policy: &NegotiatedRoutePolicy,
    ) -> Result<Self, NegotiatedRouterError> {
        let boundary = layout
            .outline
            .boundary_geometry()
            .map_err(|error| NegotiatedRouterError::InvalidBoardBoundary(error.to_string()))?;
        let (min, max) = boundary.exterior_bounds();
        let (
            (mut xs, mut injected_x_coordinates),
            (mut ys, mut injected_y_coordinates),
            local_refinement,
        ) = match &policy.grid_mode {
            NegotiatedGridMode::Uniform => (
                (
                    grid_axis(
                        min.x.clone(),
                        max.x.clone(),
                        &policy.grid_pitch,
                        policy.maximum_expansions_per_connection,
                    )?,
                    0,
                ),
                (
                    grid_axis(
                        min.y.clone(),
                        max.y.clone(),
                        &policy.grid_pitch,
                        policy.maximum_expansions_per_connection,
                    )?,
                    0,
                ),
                None,
            ),
            NegotiatedGridMode::FeatureAligned {
                coarse_pitch_multiplier,
                feature_halo_steps,
            } => {
                let coarse_pitch =
                    policy.grid_pitch.clone() * Real::from(*coarse_pitch_multiplier as u128);
                let x_features = routing_feature_coordinates(layout, handoff, Axis::X);
                let y_features = routing_feature_coordinates(layout, handoff, Axis::Y);
                (
                    adaptive_grid_axis(
                        min.x.clone(),
                        max.x.clone(),
                        &coarse_pitch,
                        &policy.grid_pitch,
                        &x_features,
                        *feature_halo_steps,
                        policy.maximum_expansions_per_connection,
                    )?,
                    adaptive_grid_axis(
                        min.y.clone(),
                        max.y.clone(),
                        &coarse_pitch,
                        &policy.grid_pitch,
                        &y_features,
                        *feature_halo_steps,
                        policy.maximum_expansions_per_connection,
                    )?,
                    None,
                )
            }
            NegotiatedGridMode::LocallyRefined {
                coarse_pitch_multiplier,
                regions,
            } => {
                let coarse_pitch =
                    policy.grid_pitch.clone() * Real::from(*coarse_pitch_multiplier as u128);
                let coarse_xs = grid_axis(
                    min.x.clone(),
                    max.x.clone(),
                    &coarse_pitch,
                    policy.maximum_expansions_per_connection,
                )?;
                let coarse_ys = grid_axis(
                    min.y.clone(),
                    max.y.clone(),
                    &coarse_pitch,
                    policy.maximum_expansions_per_connection,
                )?;
                let mut local_xs = coarse_xs.clone();
                let mut local_ys = coarse_ys.clone();
                for region in regions {
                    validate_refinement_region(region, &min, &max, &coarse_xs, &coarse_ys)?;
                    local_xs.extend(refinement_grid_axis(
                        region.min.x.clone(),
                        region.max.x.clone(),
                        &policy.grid_pitch,
                        policy.maximum_expansions_per_connection,
                    )?);
                    local_ys.extend(refinement_grid_axis(
                        region.min.y.clone(),
                        region.max.y.clone(),
                        &policy.grid_pitch,
                        policy.maximum_expansions_per_connection,
                    )?);
                }
                sort_dedup_coordinates(&mut local_xs)?;
                sort_dedup_coordinates(&mut local_ys)?;
                if local_xs.len() > policy.maximum_expansions_per_connection
                    || local_ys.len() > policy.maximum_expansions_per_connection
                {
                    return Err(NegotiatedRouterError::GridTooLarge);
                }
                for region in regions {
                    validate_refined_axis_spacing(
                        &local_xs,
                        &region.min.x,
                        &region.max.x,
                        &policy.grid_pitch,
                    )?;
                    validate_refined_axis_spacing(
                        &local_ys,
                        &region.min.y,
                        &region.max.y,
                        &policy.grid_pitch,
                    )?;
                }
                let injected_x = local_xs.len().saturating_sub(coarse_xs.len());
                let injected_y = local_ys.len().saturating_sub(coarse_ys.len());
                (
                    (local_xs, injected_x),
                    (local_ys, injected_y),
                    Some((coarse_xs, coarse_ys, regions)),
                )
            }
        };
        if matches!(&policy.grid_mode, NegotiatedGridMode::FeatureAligned { .. }) {
            for geometry in differential_pair_grid_geometry(layout, handoff, policy)? {
                for point in geometry.endpoint_points {
                    if coordinate_is_in_bounds(&point.x, &min.x, &max.x)? && !xs.contains(&point.x)
                    {
                        xs.push(point.x);
                        injected_x_coordinates = injected_x_coordinates.saturating_add(1);
                    }
                    if coordinate_is_in_bounds(&point.y, &min.y, &max.y)? && !ys.contains(&point.y)
                    {
                        ys.push(point.y);
                        injected_y_coordinates = injected_y_coordinates.saturating_add(1);
                    }
                }
                sort_dedup_coordinates(&mut xs)?;
                sort_dedup_coordinates(&mut ys)?;
                if xs.len() > policy.maximum_expansions_per_connection
                    || ys.len() > policy.maximum_expansions_per_connection
                {
                    return Err(NegotiatedRouterError::GridTooLarge);
                }
                injected_x_coordinates =
                    injected_x_coordinates.saturating_add(close_axis_translation(
                        &mut xs,
                        &geometry.translation.x,
                        &min.x,
                        &max.x,
                        policy.maximum_expansions_per_connection,
                    )?);
                injected_y_coordinates =
                    injected_y_coordinates.saturating_add(close_axis_translation(
                        &mut ys,
                        &geometry.translation.y,
                        &min.y,
                        &max.y,
                        policy.maximum_expansions_per_connection,
                    )?);
            }
        }
        let layers = layout
            .stackup
            .layers
            .iter()
            .filter_map(|layer| match layer.kind {
                crate::StackupLayerKind::Conductor(layer) => Some(layer),
                _ => None,
            })
            .collect::<Vec<_>>();
        let (active_planar_nodes, coarse_planar_nodes) =
            if let Some((coarse_xs, coarse_ys, regions)) = local_refinement {
                let mut active = BTreeSet::new();
                for (x, x_coordinate) in xs.iter().enumerate() {
                    for (y, y_coordinate) in ys.iter().enumerate() {
                        let coarse =
                            coarse_xs.contains(x_coordinate) && coarse_ys.contains(y_coordinate);
                        let refined = regions.iter().any(|region| {
                            coordinate_is_in_bounds(x_coordinate, &region.min.x, &region.max.x)
                                == Ok(true)
                                && coordinate_is_in_bounds(
                                    y_coordinate,
                                    &region.min.y,
                                    &region.max.y,
                                ) == Ok(true)
                        });
                        if coarse || refined {
                            active.insert((x, y));
                        }
                    }
                }
                (
                    Some(active),
                    coarse_xs.len().saturating_mul(coarse_ys.len()),
                )
            } else {
                (None, xs.len().saturating_mul(ys.len()))
            };
        let planar_nodes = active_planar_nodes
            .as_ref()
            .map_or_else(|| xs.len().saturating_mul(ys.len()), BTreeSet::len);
        if planar_nodes.saturating_mul(layers.len())
            > policy.maximum_expansions_per_connection.saturating_mul(16)
        {
            return Err(NegotiatedRouterError::GridTooLarge);
        }
        Ok(Self {
            xs,
            ys,
            layers,
            boundary,
            injected_x_coordinates,
            injected_y_coordinates,
            active_planar_nodes,
            coarse_planar_nodes,
        })
    }

    fn evidence(&self, mode: &NegotiatedGridMode) -> NegotiatedGridEvidence {
        NegotiatedGridEvidence {
            mode: mode.clone(),
            x_coordinates: self.xs.len(),
            y_coordinates: self.ys.len(),
            conductor_layers: self.layers.len(),
            injected_x_coordinates: self.injected_x_coordinates,
            injected_y_coordinates: self.injected_y_coordinates,
            active_planar_nodes: self.planar_node_count(),
            refined_planar_nodes: self
                .planar_node_count()
                .saturating_sub(self.coarse_planar_nodes),
        }
    }

    fn is_active(&self, node: Node) -> bool {
        self.active_planar_nodes
            .as_ref()
            .is_none_or(|active| active.contains(&(node.x, node.y)))
    }

    fn planar_node_count(&self) -> usize {
        self.active_planar_nodes.as_ref().map_or_else(
            || self.xs.len().saturating_mul(self.ys.len()),
            BTreeSet::len,
        )
    }

    fn point(&self, node: Node) -> Point2 {
        Point2::new(self.xs[node.x].clone(), self.ys[node.y].clone())
    }

    fn planar_step_units(
        &self,
        first: Node,
        second: Node,
        policy: &NegotiatedRoutePolicy,
    ) -> Option<u64> {
        let dx = self.xs[first.x].clone() - self.xs[second.x].clone();
        let dy = self.ys[first.y].clone() - self.ys[second.y].clone();
        let span = (dx.clone() * dx + dy.clone() * dy).sqrt().ok()?;
        span_units(
            &span,
            &policy.grid_pitch,
            policy.maximum_expansions_per_connection,
        )
    }

    fn euclidean_units(
        &self,
        first: Node,
        second: Node,
        policy: &NegotiatedRoutePolicy,
    ) -> Option<u64> {
        let dx = self.xs[first.x].clone() - self.xs[second.x].clone();
        let dy = self.ys[first.y].clone() - self.ys[second.y].clone();
        let span = (dx.clone() * dx + dy.clone() * dy).sqrt().ok()?;
        span_units(
            &span,
            &policy.grid_pitch,
            policy.maximum_expansions_per_connection,
        )
    }

    fn terminal_nodes(&self, point: &Point2, layers: &[TraceLayer]) -> Option<Vec<Node>> {
        let x = self.xs.iter().position(|value| value == &point.x)?;
        let y = self.ys.iter().position(|value| value == &point.y)?;
        let nodes = layers
            .iter()
            .filter_map(|layer| {
                self.layers
                    .iter()
                    .position(|candidate| candidate == layer)
                    .map(|layer| Node { x, y, layer })
                    .filter(|node| self.is_active(*node))
            })
            .collect::<Vec<_>>();
        (!nodes.is_empty()).then_some(nodes)
    }
}

#[derive(Clone, Debug, Default)]
struct NetRoute {
    paths: Vec<Vec<Node>>,
    resources: BTreeSet<Resource>,
    width_overrides: BTreeMap<Resource, Real>,
}

struct RouterContext<'a> {
    layout: &'a PcbLayout,
    handoff: &'a RoutingProblemReport,
    grid: &'a Grid,
    policy: &'a NegotiatedRoutePolicy,
    history: &'a BTreeMap<Resource, u64>,
    fixed: &'a FixedObstacles,
}

struct SearchContext<'a> {
    router: &'a RouterContext<'a>,
    net: &'a NetId,
    terminal_index: usize,
    via_half_land: &'a Real,
    via_style: &'a ResolvedViaStyle,
    paired_net: Option<&'a NetId>,
    occupancy: &'a BTreeMap<Resource, BTreeSet<NetId>>,
}

#[derive(Clone, Debug)]
struct PreparedDifferentialPair {
    pair: DifferentialPair,
    translation: Point2,
    sources: BTreeSet<PairNodes>,
    targets: Vec<PairNodes>,
    source_escapes: BTreeMap<PairNodes, DifferentialPairEscape>,
    target_escapes: BTreeMap<PairNodes, DifferentialPairEscape>,
    neckdown_transition_length: Option<Real>,
    positive_half_width: Real,
    negative_half_width: Real,
    positive_via_half_land: Real,
    negative_via_half_land: Real,
    positive_via_style: ResolvedViaStyle,
    negative_via_style: ResolvedViaStyle,
}

#[derive(Clone, Debug)]
struct DifferentialPairEscape {
    positive: Vec<Node>,
    negative: Vec<Node>,
}

#[derive(Clone, Debug, PartialEq)]
struct ResolvedViaStyle {
    id: Option<ViaStyleId>,
    land_diameter: Real,
    drill_diameter: Real,
    plating: Plating,
    mask: ViaMaskIntent,
    allowed_spans: Vec<ViaStyleSpan>,
}

impl ResolvedViaStyle {
    fn allows(&self, first: TraceLayer, second: TraceLayer) -> bool {
        if self.allowed_spans.is_empty() {
            return true;
        }
        let (start_layer, end_layer) = if first <= second {
            (first, second)
        } else {
            (second, first)
        };
        self.allowed_spans
            .iter()
            .any(|span| span.start_layer == start_layer && span.end_layer == end_layer)
    }

    fn half_land(&self) -> Result<Real, NegotiatedRouterError> {
        (self.land_diameter.clone() / Real::from(2))
            .map_err(|_| NegotiatedRouterError::InvalidPolicy)
    }
}

#[derive(Clone, Debug)]
struct FixedLine {
    net: NetId,
    layer: TraceLayer,
    start: Point2,
    end: Point2,
    radius: Real,
    clearance: Real,
}

#[derive(Clone, Debug)]
struct FixedDisc {
    net: NetId,
    layers: Vec<TraceLayer>,
    center: Point2,
    radius: Real,
    clearance: Real,
}

#[derive(Clone, Debug)]
struct FixedPad {
    net: Option<NetId>,
    layers: Vec<TraceLayer>,
    placement: PcbPlacement,
    center: Point2,
    rotation_degrees: Real,
    shape: PadShape,
    clearance: Real,
}

#[derive(Clone, Debug, Default)]
struct FixedObstacles {
    lines: Vec<FixedLine>,
    discs: Vec<FixedDisc>,
    pads: Vec<FixedPad>,
}

struct ClearanceSubject<'a> {
    physical_radius: &'a Real,
    clearance: &'a Real,
    net: &'a NetId,
    paired_net: Option<&'a NetId>,
}

impl FixedObstacles {
    fn from_layout(
        layout: &PcbLayout,
        handoff: &RoutingProblemReport,
        selected: &[NetId],
        policy: &NegotiatedRoutePolicy,
    ) -> Result<Self, NegotiatedRouterError> {
        let selected = selected.iter().collect::<BTreeSet<_>>();
        let mut result = Self::default();
        for route in &layout.routes {
            if selected.contains(&route.net) {
                continue;
            }
            let radius = (route.width.clone() / Real::from(2))
                .expect("validated route width has a nonzero divisor");
            let (_, clearance) = net_rule(handoff, &route.net, policy);
            for segment in &route.segments {
                let PcbRouteSegment::Line(segment) = segment else {
                    return Err(NegotiatedRouterError::UnsupportedFixedRoute(
                        route.id.clone(),
                    ));
                };
                result.lines.push(FixedLine {
                    net: route.net.clone(),
                    layer: route.layer,
                    start: segment.start().clone(),
                    end: segment.end().clone(),
                    radius: radius.clone(),
                    clearance: clearance.clone(),
                });
            }
        }
        let stitching = layout.realize_stitching_vias();
        for via in layout
            .vias
            .iter()
            .filter(|via| !selected.contains(&via.net))
            .chain(&stitching.vias)
        {
            let (_, clearance) = net_rule(handoff, &via.net, policy);
            result.discs.push(FixedDisc {
                net: via.net.clone(),
                layers: layout
                    .stackup
                    .layers
                    .iter()
                    .filter_map(|layer| match layer.kind {
                        crate::StackupLayerKind::Conductor(layer)
                            if layer >= via.start_layer && layer <= via.end_layer =>
                        {
                            Some(layer)
                        }
                        _ => None,
                    })
                    .collect(),
                center: via.center.clone(),
                radius: (via.land_diameter.clone() / Real::from(2))
                    .expect("validated via land has a nonzero divisor"),
                clearance,
            });
        }
        for placement in &layout.placements {
            let Some(pattern) = layout
                .land_patterns
                .iter()
                .find(|pattern| pattern.id == placement.land_pattern)
            else {
                continue;
            };
            for pad in &pattern.pads {
                let terminal = handoff.problem.terminals.iter().find(|terminal| {
                    terminal.instance == placement.instance && terminal.pad == pad.id
                });
                let net = terminal.map(|terminal| terminal.net.clone());
                let clearance = net
                    .as_ref()
                    .map_or_else(Real::zero, |net| net_rule(handoff, net, policy).1);
                let layers = terminal.map_or_else(
                    || placed_pad_layers(layout, placement, &pad.copper_layers),
                    |terminal| terminal.layers.clone(),
                );
                result.pads.push(FixedPad {
                    net,
                    layers,
                    placement: placement.clone(),
                    center: pad.center.clone(),
                    rotation_degrees: pad.rotation_degrees.clone(),
                    shape: pad.shape.clone(),
                    clearance,
                });
            }
        }
        Ok(result)
    }
}

fn placed_pad_layers(
    layout: &PcbLayout,
    placement: &PcbPlacement,
    layers: &[TraceLayer],
) -> Vec<TraceLayer> {
    if placement.side == BoardSide::Front {
        return layers.to_vec();
    }
    let last = layout
        .stackup
        .layers
        .iter()
        .filter_map(|candidate| match candidate.kind {
            crate::StackupLayerKind::Conductor(layer) => Some(layer.0),
            _ => None,
        })
        .max()
        .unwrap_or_default();
    layers
        .iter()
        .map(|layer| TraceLayer(last.saturating_sub(layer.0)))
        .collect()
}

fn real_abs(value: &Real) -> Real {
    if value.partial_cmp(&Real::zero()) == Some(Ordering::Less) {
        -value.clone()
    } else {
        value.clone()
    }
}

fn validate_policy(policy: &NegotiatedRoutePolicy) -> Result<(), NegotiatedRouterError> {
    let positive = |value: &Real| value.partial_cmp(&Real::zero()) == Some(Ordering::Greater);
    let nonnegative = |value: &Real| {
        matches!(
            value.partial_cmp(&Real::zero()),
            Some(Ordering::Equal | Ordering::Greater)
        )
    };
    if !positive(&policy.grid_pitch)
        || matches!(
            &policy.grid_mode,
            NegotiatedGridMode::FeatureAligned {
                coarse_pitch_multiplier: 0,
                ..
            }
        )
        || matches!(
            &policy.grid_mode,
            NegotiatedGridMode::FeatureAligned {
                feature_halo_steps,
                ..
            } if *feature_halo_steps > policy.maximum_expansions_per_connection
        )
        || matches!(
            &policy.grid_mode,
            NegotiatedGridMode::LocallyRefined {
                coarse_pitch_multiplier: 0,
                ..
            }
        )
        || matches!(
            &policy.grid_mode,
            NegotiatedGridMode::LocallyRefined { regions, .. }
                if regions.is_empty()
                    || regions.len() > policy.maximum_expansions_per_connection
        )
        || matches!(
            policy.planar_topology,
            NegotiatedPlanarTopology::AnyAngle {
                maximum_neighbors_per_node: 0
            }
        )
        || matches!(
            policy.planar_topology,
            NegotiatedPlanarTopology::AnyAngle {
                maximum_neighbors_per_node
            } if maximum_neighbors_per_node > policy.maximum_expansions_per_connection
        )
        || policy.maximum_iterations == 0
        || policy.maximum_expansions_per_connection == 0
        || !positive(&policy.default_trace_width)
        || !nonnegative(&policy.default_clearance)
        || !positive(&policy.via_land_diameter)
        || !positive(&policy.via_drill_diameter)
        || policy.via_drill_diameter > policy.via_land_diameter
    {
        return Err(NegotiatedRouterError::InvalidPolicy);
    }
    Ok(())
}

fn validate_selected_spacing(
    layout: &PcbLayout,
    handoff: &RoutingProblemReport,
    selected: &[NetId],
    grid: &Grid,
    policy: &NegotiatedRoutePolicy,
) -> Result<(), NegotiatedRouterError> {
    let octilinear_spacing = if policy.planar_topology == NegotiatedPlanarTopology::Octilinear {
        minimum_octilinear_track_spacing(grid)?
    } else {
        None
    };
    for first in selected {
        let (first_width, first_clearance) = maximum_net_rule(layout, handoff, first, policy)?;
        let first_via = resolved_via_style(layout, first, policy).half_land()?;
        let first_half =
            (first_width / Real::from(2)).map_err(|_| NegotiatedRouterError::InvalidPolicy)?;
        for second in selected {
            let (second_width, second_clearance) =
                maximum_net_rule(layout, handoff, second, policy)?;
            let second_via = resolved_via_style(layout, second, policy).half_land()?;
            let second_half =
                (second_width / Real::from(2)).map_err(|_| NegotiatedRouterError::InvalidPolicy)?;
            let clearance = exact_max(&first_clearance, &second_clearance)
                .ok_or(NegotiatedRouterError::InvalidPolicy)?;
            let requirements = [
                first_half.clone() + second_half + clearance.clone(),
                first_via.clone() + second_via.clone() + clearance.clone(),
                first_half.clone() + second_via + clearance,
            ];
            if requirements.iter().any(|required| {
                !matches!(
                    policy.grid_pitch.partial_cmp(required),
                    Some(Ordering::Equal | Ordering::Greater)
                ) || octilinear_spacing.as_ref().is_some_and(|available| {
                    !matches!(
                        available.partial_cmp(required),
                        Some(Ordering::Equal | Ordering::Greater)
                    )
                })
            }) {
                return Err(NegotiatedRouterError::InvalidPolicy);
            }
        }
    }
    Ok(())
}

fn minimum_octilinear_track_spacing(grid: &Grid) -> Result<Option<Real>, NegotiatedRouterError> {
    let root_two = Real::from(2)
        .sqrt()
        .map_err(|_| NegotiatedRouterError::InvalidPolicy)?;
    let mut minimum = None::<Real>;
    for x in 0..grid.xs.len() {
        for y in 0..grid.ys.len() {
            let node = Node { x, y, layer: 0 };
            if !grid.is_active(node) {
                continue;
            }
            for (x_forward, y_forward) in [(true, true), (true, false)] {
                let Some(neighbor) = active_diagonal_neighbor(grid, node, x_forward, y_forward)
                else {
                    continue;
                };
                let span = real_abs(&(grid.xs[neighbor.x].clone() - grid.xs[node.x].clone()));
                let spacing =
                    (span / root_two.clone()).map_err(|_| NegotiatedRouterError::InvalidPolicy)?;
                minimum = Some(match minimum {
                    Some(current) => match current.partial_cmp(&spacing) {
                        Some(Ordering::Less | Ordering::Equal) => current,
                        Some(Ordering::Greater) => spacing,
                        None => return Err(NegotiatedRouterError::InvalidPolicy),
                    },
                    None => spacing,
                });
            }
        }
    }
    Ok(minimum)
}

fn exact_max(first: &Real, second: &Real) -> Option<Real> {
    match first.partial_cmp(second)? {
        Ordering::Less => Some(second.clone()),
        Ordering::Equal | Ordering::Greater => Some(first.clone()),
    }
}

fn exact_min(first: &Real, second: &Real) -> Option<Real> {
    match first.partial_cmp(second)? {
        Ordering::Greater => Some(second.clone()),
        Ordering::Equal | Ordering::Less => Some(first.clone()),
    }
}

fn prepare_differential_pair(
    layout: &PcbLayout,
    handoff: &RoutingProblemReport,
    grid: &Grid,
    pair: &DifferentialPair,
    policy: &NegotiatedRoutePolicy,
) -> Result<PreparedDifferentialPair, NegotiatedRouterError> {
    let positive = handoff
        .problem
        .terminals
        .iter()
        .filter(|terminal| terminal.net == pair.positive)
        .collect::<Vec<_>>();
    let negative = handoff
        .problem
        .terminals
        .iter()
        .filter(|terminal| terminal.net == pair.negative)
        .collect::<Vec<_>>();
    if positive.len() != 2 || negative.len() != 2 {
        return Err(NegotiatedRouterError::UnsupportedDifferentialPair(
            pair.id.clone(),
        ));
    }

    let direct = point_delta(&positive[0].center, &negative[0].center)
        == point_delta(&positive[1].center, &negative[1].center);
    let reversed = point_delta(&positive[0].center, &negative[1].center)
        == point_delta(&positive[1].center, &negative[0].center);
    let negative_order = if direct {
        [0, 1]
    } else if reversed {
        [1, 0]
    } else {
        return Err(NegotiatedRouterError::UnsupportedDifferentialPair(
            pair.id.clone(),
        ));
    };

    let (positive_width, positive_clearance) =
        maximum_net_rule(layout, handoff, &pair.positive, policy)?;
    let (negative_width, negative_clearance) =
        maximum_net_rule(layout, handoff, &pair.negative, policy)?;
    let positive_half_width =
        (positive_width / Real::from(2)).map_err(|_| NegotiatedRouterError::InvalidPolicy)?;
    let negative_half_width =
        (negative_width / Real::from(2)).map_err(|_| NegotiatedRouterError::InvalidPolicy)?;
    let positive_via_style = resolved_via_style(layout, &pair.positive, policy);
    let negative_via_style = resolved_via_style(layout, &pair.negative, policy);
    let positive_via_half_land = positive_via_style.half_land()?;
    let negative_via_half_land = negative_via_style.half_land()?;
    let required_trace_separation =
        positive_half_width.clone() + negative_half_width.clone() + pair.spacing.clone();
    let trace_separation_squared =
        required_trace_separation.clone() * required_trace_separation.clone();
    let start_separation =
        point_distance_squared(&positive[0].center, &negative[negative_order[0]].center);
    let end_separation =
        point_distance_squared(&positive[1].center, &negative[negative_order[1]].center);
    let terminal_translation =
        point_delta(&positive[0].center, &negative[negative_order[0]].center);
    let (translation, neckdown_transition_length, positive_pair_points, negative_pair_points) =
        if start_separation.partial_cmp(&trace_separation_squared) == Some(Ordering::Equal)
            && end_separation.partial_cmp(&trace_separation_squared) == Some(Ordering::Equal)
        {
            (
                Point2::new(terminal_translation.0, terminal_translation.1),
                None,
                positive
                    .iter()
                    .map(|terminal| terminal.center.clone())
                    .collect::<Vec<_>>(),
                negative
                    .iter()
                    .map(|terminal| terminal.center.clone())
                    .collect::<Vec<_>>(),
            )
        } else {
            let neckdown = pair.neckdown.as_ref().ok_or_else(|| {
                NegotiatedRouterError::UnsupportedDifferentialPair(pair.id.clone())
            })?;
            let neckdown_separation = neckdown.trace_width.clone() + neckdown.spacing.clone();
            let neckdown_squared = neckdown_separation.clone() * neckdown_separation.clone();
            if start_separation.partial_cmp(&neckdown_squared) != Some(Ordering::Equal)
                || end_separation.partial_cmp(&neckdown_squared) != Some(Ordering::Equal)
            {
                return Err(NegotiatedRouterError::UnsupportedDifferentialPair(
                    pair.id.clone(),
                ));
            }
            if neckdown_separation.partial_cmp(&required_trace_separation) != Some(Ordering::Less) {
                return Err(NegotiatedRouterError::UnsupportedDifferentialPair(
                    pair.id.clone(),
                ));
            }
            let transition_length = ((required_trace_separation.clone() - neckdown_separation)
                / Real::from(2))
            .map_err(|_| NegotiatedRouterError::InvalidPolicy)?;
            if transition_length.partial_cmp(&neckdown.maximum_transition_length)
                == Some(Ordering::Greater)
            {
                return Err(NegotiatedRouterError::UnsupportedDifferentialPair(
                    pair.id.clone(),
                ));
            }
            let mut positive_points = Vec::with_capacity(2);
            let mut negative_points = negative
                .iter()
                .map(|terminal| terminal.center.clone())
                .collect::<Vec<_>>();
            let mut translation = None;
            for index in 0..2 {
                let (positive_point, negative_point, endpoint_translation) = expanded_pair_centers(
                    &positive[index].center,
                    &negative[negative_order[index]].center,
                    &required_trace_separation,
                    &transition_length,
                )
                .ok_or_else(|| {
                    NegotiatedRouterError::UnsupportedDifferentialPair(pair.id.clone())
                })?;
                if translation
                    .as_ref()
                    .is_some_and(|candidate| candidate != &endpoint_translation)
                {
                    return Err(NegotiatedRouterError::UnsupportedDifferentialPair(
                        pair.id.clone(),
                    ));
                }
                translation = Some(endpoint_translation);
                positive_points.push(positive_point);
                negative_points[negative_order[index]] = negative_point;
            }
            (
                translation.expect("two differential-pair endpoints produce a translation"),
                Some(transition_length),
                positive_points,
                negative_points,
            )
        };

    let positive_nodes = positive
        .iter()
        .map(|terminal| grid.terminal_nodes(&terminal.center, &terminal.layers))
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| NegotiatedRouterError::UnsupportedDifferentialPair(pair.id.clone()))?;
    let negative_nodes = negative
        .iter()
        .map(|terminal| grid.terminal_nodes(&terminal.center, &terminal.layers))
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| NegotiatedRouterError::UnsupportedDifferentialPair(pair.id.clone()))?;
    let positive_pair_nodes = positive
        .iter()
        .zip(&positive_pair_points)
        .map(|(terminal, point)| grid.terminal_nodes(point, &terminal.layers))
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| NegotiatedRouterError::UnsupportedDifferentialPair(pair.id.clone()))?;
    let negative_pair_nodes = negative
        .iter()
        .zip(&negative_pair_points)
        .map(|(terminal, point)| grid.terminal_nodes(point, &terminal.layers))
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| NegotiatedRouterError::UnsupportedDifferentialPair(pair.id.clone()))?;
    let source_escapes = pair_endpoint_escapes(
        grid,
        &positive_nodes[0],
        &negative_nodes[negative_order[0]],
        &positive_pair_nodes[0],
        &negative_pair_nodes[negative_order[0]],
        false,
    )
    .ok_or_else(|| NegotiatedRouterError::UnsupportedDifferentialPair(pair.id.clone()))?;
    let target_escapes = pair_endpoint_escapes(
        grid,
        &positive_nodes[1],
        &negative_nodes[negative_order[1]],
        &positive_pair_nodes[1],
        &negative_pair_nodes[negative_order[1]],
        true,
    )
    .ok_or_else(|| NegotiatedRouterError::UnsupportedDifferentialPair(pair.id.clone()))?;
    let sources = source_escapes.keys().copied().collect::<BTreeSet<_>>();
    let targets = target_escapes.keys().copied().collect::<Vec<_>>();
    if sources.is_empty() || targets.is_empty() {
        return Err(NegotiatedRouterError::UnsupportedDifferentialPair(
            pair.id.clone(),
        ));
    }

    let pair_clearance = exact_max(&positive_clearance, &negative_clearance)
        .ok_or(NegotiatedRouterError::InvalidPolicy)?;
    let required_via_separation =
        positive_via_half_land.clone() + negative_via_half_land.clone() + pair_clearance;
    let required_via_squared = required_via_separation.clone() * required_via_separation;
    if trace_separation_squared.partial_cmp(&required_via_squared) == Some(Ordering::Less) {
        return Err(NegotiatedRouterError::UnsupportedDifferentialPair(
            pair.id.clone(),
        ));
    }

    Ok(PreparedDifferentialPair {
        pair: pair.clone(),
        translation,
        sources,
        targets,
        source_escapes,
        target_escapes,
        neckdown_transition_length,
        positive_half_width,
        negative_half_width,
        positive_via_half_land,
        negative_via_half_land,
        positive_via_style,
        negative_via_style,
    })
}

fn point_delta(positive: &Point2, negative: &Point2) -> (Real, Real) {
    (
        negative.x.clone() - positive.x.clone(),
        negative.y.clone() - positive.y.clone(),
    )
}

fn expanded_pair_centers(
    positive: &Point2,
    negative: &Point2,
    separation: &Real,
    transition: &Real,
) -> Option<(Point2, Point2, Point2)> {
    let (x, y) = point_delta(positive, negative);
    let zero = Real::zero();
    let mut positive = positive.clone();
    let mut negative = negative.clone();
    let translation = if y == zero {
        match x.partial_cmp(&zero)? {
            Ordering::Greater => {
                positive.x -= transition.clone();
                negative.x += transition.clone();
                Point2::new(separation.clone(), zero)
            }
            Ordering::Less => {
                positive.x += transition.clone();
                negative.x -= transition.clone();
                Point2::new(-separation.clone(), zero)
            }
            Ordering::Equal => return None,
        }
    } else if x == zero {
        match y.partial_cmp(&zero)? {
            Ordering::Greater => {
                positive.y -= transition.clone();
                negative.y += transition.clone();
                Point2::new(zero, separation.clone())
            }
            Ordering::Less => {
                positive.y += transition.clone();
                negative.y -= transition.clone();
                Point2::new(zero, -separation.clone())
            }
            Ordering::Equal => return None,
        }
    } else {
        return None;
    };
    Some((positive, negative, translation))
}

fn common_layer_pairs(positive: &[Node], negative: &[Node]) -> BTreeSet<PairNodes> {
    positive
        .iter()
        .flat_map(|positive| {
            negative.iter().filter_map(move |negative| {
                (positive.layer == negative.layer).then_some(PairNodes {
                    positive: *positive,
                    negative: *negative,
                })
            })
        })
        .collect()
}

fn pair_endpoint_escapes(
    grid: &Grid,
    original_positive: &[Node],
    original_negative: &[Node],
    paired_positive: &[Node],
    paired_negative: &[Node],
    reverse: bool,
) -> Option<BTreeMap<PairNodes, DifferentialPairEscape>> {
    let mut result = BTreeMap::new();
    for pair in common_layer_pairs(paired_positive, paired_negative) {
        let positive = original_positive
            .iter()
            .find(|node| node.layer == pair.positive.layer)?;
        let negative = original_negative
            .iter()
            .find(|node| node.layer == pair.negative.layer)?;
        let mut positive_path = axis_node_path(grid, *positive, pair.positive)?;
        let mut negative_path = axis_node_path(grid, *negative, pair.negative)?;
        if reverse {
            positive_path.reverse();
            negative_path.reverse();
        }
        result.insert(
            pair,
            DifferentialPairEscape {
                positive: positive_path,
                negative: negative_path,
            },
        );
    }
    Some(result)
}

fn axis_node_path(grid: &Grid, first: Node, last: Node) -> Option<Vec<Node>> {
    if first.layer != last.layer || (first.x != last.x && first.y != last.y) {
        return None;
    }
    if first.x == last.x {
        let ys = if first.y <= last.y {
            (first.y..=last.y).collect::<Vec<_>>()
        } else {
            (last.y..=first.y).rev().collect::<Vec<_>>()
        };
        return Some(
            ys.into_iter()
                .map(|y| Node { y, ..first })
                .filter(|node| grid.is_active(*node))
                .collect::<Vec<_>>(),
        );
    }
    let xs = if first.x <= last.x {
        (first.x..=last.x).collect::<Vec<_>>()
    } else {
        (last.x..=first.x).rev().collect::<Vec<_>>()
    };
    Some(
        xs.into_iter()
            .map(|x| Node { x, ..first })
            .filter(|node| grid.is_active(*node))
            .collect::<Vec<_>>(),
    )
}

fn grid_axis(
    min: Real,
    max: Real,
    pitch: &Real,
    cap: usize,
) -> Result<Vec<Real>, NegotiatedRouterError> {
    let mut values = Vec::new();
    let mut value = min;
    loop {
        match value.partial_cmp(&max) {
            Some(Ordering::Greater) => break,
            Some(Ordering::Less | Ordering::Equal) => values.push(value.clone()),
            None => return Err(NegotiatedRouterError::InvalidPolicy),
        }
        if values.len() > cap {
            return Err(NegotiatedRouterError::GridTooLarge);
        }
        value += pitch.clone();
    }
    Ok(values)
}

#[derive(Clone)]
struct RefinementBounds {
    min: Point2,
    max: Point2,
}

#[allow(clippy::too_many_arguments)]
fn synthesize_refinement_regions(
    layout: &PcbLayout,
    handoff: &RoutingProblemReport,
    report: &NegotiatedRouteReport,
    existing: &[NegotiatedGridRefinementRegion],
    board_min: &Point2,
    board_max: &Point2,
    coarse_xs: &[Real],
    coarse_ys: &[Real],
    policy: &NegotiatedAdaptiveRoutePolicy,
) -> Result<Vec<NegotiatedGridRefinementRegion>, NegotiatedRouterError> {
    let mut bounds = Vec::<RefinementBounds>::new();
    if let Some(state) = report.iteration_states.last() {
        bounds.extend(
            state
                .conflicts
                .iter()
                .map(|conflict| match &conflict.geometry {
                    NegotiatedRouteConflictGeometry::Node { center, .. }
                    | NegotiatedRouteConflictGeometry::Via { center, .. } => RefinementBounds {
                        min: center.clone(),
                        max: center.clone(),
                    },
                    NegotiatedRouteConflictGeometry::Segment { start, end, .. } => {
                        bounds_from_points([start, end])
                    }
                    NegotiatedRouteConflictGeometry::DiagonalCell {
                        lower_left,
                        upper_right,
                        ..
                    } => RefinementBounds {
                        min: lower_left.clone(),
                        max: upper_right.clone(),
                    },
                }),
        );
    }
    let mut failed_nets = BTreeSet::<NetId>::new();
    for failure in &report.failures {
        match failure {
            NegotiatedRouteFailure::OffGridTerminal { net, .. }
            | NegotiatedRouteFailure::NoPath { net, .. }
            | NegotiatedRouteFailure::ExpansionLimit { net, .. }
            | NegotiatedRouteFailure::Indeterminate { net, .. } => {
                failed_nets.insert(net.clone());
            }
            NegotiatedRouteFailure::DifferentialPairNoPath(pair)
            | NegotiatedRouteFailure::DifferentialPairExpansionLimit(pair)
            | NegotiatedRouteFailure::DifferentialPairIndeterminate(pair) => {
                if let Some(pair) = layout
                    .rules
                    .differential_pairs
                    .iter()
                    .find(|candidate| &candidate.id == pair)
                {
                    failed_nets.insert(pair.positive.clone());
                    failed_nets.insert(pair.negative.clone());
                }
            }
        }
    }
    for net in failed_nets {
        let points = handoff
            .problem
            .terminals
            .iter()
            .filter(|terminal| terminal.net == net)
            .map(|terminal| &terminal.center)
            .collect::<Vec<_>>();
        if !points.is_empty() {
            bounds.push(bounds_from_points(points));
        }
    }
    let mut regions = existing.to_vec();
    for bounds in bounds {
        if let Some(region) = refinement_region_from_bounds(
            &bounds,
            board_min,
            board_max,
            coarse_xs,
            coarse_ys,
            &policy.route_policy.grid_pitch,
            policy.refinement_padding_steps,
            policy.route_policy.maximum_expansions_per_connection,
        )? {
            regions.push(region);
        }
    }
    merge_refinement_regions(regions)
}

fn bounds_from_points<'a>(points: impl IntoIterator<Item = &'a Point2>) -> RefinementBounds {
    let mut points = points.into_iter();
    let first = points
        .next()
        .expect("routing hotspot bounds always contain a point");
    points.fold(
        RefinementBounds {
            min: first.clone(),
            max: first.clone(),
        },
        |mut bounds, point| {
            bounds.min.x = exact_min(&bounds.min.x, &point.x)
                .expect("retained exact X coordinates are orderable");
            bounds.min.y = exact_min(&bounds.min.y, &point.y)
                .expect("retained exact Y coordinates are orderable");
            bounds.max.x = exact_max(&bounds.max.x, &point.x)
                .expect("retained exact X coordinates are orderable");
            bounds.max.y = exact_max(&bounds.max.y, &point.y)
                .expect("retained exact Y coordinates are orderable");
            bounds
        },
    )
}

#[allow(clippy::too_many_arguments)]
fn refinement_region_from_bounds(
    bounds: &RefinementBounds,
    board_min: &Point2,
    board_max: &Point2,
    coarse_xs: &[Real],
    coarse_ys: &[Real],
    pitch: &Real,
    padding_steps: usize,
    cap: usize,
) -> Result<Option<NegotiatedGridRefinementRegion>, NegotiatedRouterError> {
    let padding = pitch.clone() * Real::from(padding_steps as u128);
    let raw_min_x = exact_max(&board_min.x, &(bounds.min.x.clone() - padding.clone()))
        .ok_or(NegotiatedRouterError::InvalidPolicy)?;
    let raw_max_x = exact_min(&board_max.x, &(bounds.max.x.clone() + padding.clone()))
        .ok_or(NegotiatedRouterError::InvalidPolicy)?;
    let raw_min_y = exact_max(&board_min.y, &(bounds.min.y.clone() - padding.clone()))
        .ok_or(NegotiatedRouterError::InvalidPolicy)?;
    let raw_max_y = exact_min(&board_max.y, &(bounds.max.y.clone() + padding))
        .ok_or(NegotiatedRouterError::InvalidPolicy)?;
    let Some(mut min_x) = snap_down_to_pitch(&raw_min_x, &board_min.x, pitch, cap) else {
        return Ok(None);
    };
    let Some(mut max_x) = snap_up_to_pitch(&raw_max_x, &board_min.x, pitch, cap) else {
        return Ok(None);
    };
    let Some(mut min_y) = snap_down_to_pitch(&raw_min_y, &board_min.y, pitch, cap) else {
        return Ok(None);
    };
    let Some(mut max_y) = snap_up_to_pitch(&raw_max_y, &board_min.y, pitch, cap) else {
        return Ok(None);
    };
    if max_x.partial_cmp(&board_max.x) == Some(Ordering::Greater)
        || max_y.partial_cmp(&board_max.y) == Some(Ordering::Greater)
    {
        return Ok(None);
    }
    include_coarse_coordinate(&mut min_x, &mut max_x, coarse_xs)?;
    include_coarse_coordinate(&mut min_y, &mut max_y, coarse_ys)?;
    if min_x.partial_cmp(&max_x) != Some(Ordering::Less)
        || min_y.partial_cmp(&max_y) != Some(Ordering::Less)
    {
        return Ok(None);
    }
    Ok(Some(NegotiatedGridRefinementRegion {
        min: Point2::new(min_x, min_y),
        max: Point2::new(max_x, max_y),
    }))
}

fn snap_down_to_pitch(value: &Real, base: &Real, pitch: &Real, cap: usize) -> Option<Real> {
    let distance = value.clone() - base.clone();
    let units = span_units(&distance, pitch, cap)?;
    let mut candidate = base.clone() + pitch.clone() * Real::from(units as u128);
    if candidate.partial_cmp(value)? == Ordering::Greater {
        candidate -= pitch.clone();
    }
    Some(candidate)
}

fn snap_up_to_pitch(value: &Real, base: &Real, pitch: &Real, cap: usize) -> Option<Real> {
    let distance = value.clone() - base.clone();
    let units = span_units(&distance, pitch, cap)?;
    Some(base.clone() + pitch.clone() * Real::from(units as u128))
}

fn include_coarse_coordinate(
    min: &mut Real,
    max: &mut Real,
    coarse: &[Real],
) -> Result<(), NegotiatedRouterError> {
    if coarse
        .iter()
        .any(|coordinate| coordinate_is_in_bounds(coordinate, min, max) == Ok(true))
    {
        return Ok(());
    }
    let nearest = coarse
        .iter()
        .min_by(|first, second| {
            let first_distance = exact_min(
                &real_abs(&((*first).clone() - min.clone())),
                &real_abs(&((*first).clone() - max.clone())),
            )
            .expect("exact coarse coordinate distance is orderable");
            let second_distance = exact_min(
                &real_abs(&((*second).clone() - min.clone())),
                &real_abs(&((*second).clone() - max.clone())),
            )
            .expect("exact coarse coordinate distance is orderable");
            first_distance
                .partial_cmp(&second_distance)
                .unwrap_or(Ordering::Equal)
        })
        .ok_or(NegotiatedRouterError::InvalidPolicy)?
        .clone();
    *min = exact_min(min, &nearest).ok_or(NegotiatedRouterError::InvalidPolicy)?;
    *max = exact_max(max, &nearest).ok_or(NegotiatedRouterError::InvalidPolicy)?;
    Ok(())
}

fn merge_refinement_regions(
    mut regions: Vec<NegotiatedGridRefinementRegion>,
) -> Result<Vec<NegotiatedGridRefinementRegion>, NegotiatedRouterError> {
    regions.sort_by(|first, second| {
        first
            .min
            .x
            .partial_cmp(&second.min.x)
            .unwrap_or(Ordering::Equal)
            .then_with(|| {
                first
                    .min
                    .y
                    .partial_cmp(&second.min.y)
                    .unwrap_or(Ordering::Equal)
            })
    });
    let mut merged = Vec::<NegotiatedGridRefinementRegion>::new();
    for mut candidate in regions {
        while let Some(index) = merged
            .iter()
            .position(|region| refinement_regions_overlap(region, &candidate))
        {
            let region = merged.remove(index);
            candidate.min.x = exact_min(&candidate.min.x, &region.min.x)
                .ok_or(NegotiatedRouterError::InvalidPolicy)?;
            candidate.min.y = exact_min(&candidate.min.y, &region.min.y)
                .ok_or(NegotiatedRouterError::InvalidPolicy)?;
            candidate.max.x = exact_max(&candidate.max.x, &region.max.x)
                .ok_or(NegotiatedRouterError::InvalidPolicy)?;
            candidate.max.y = exact_max(&candidate.max.y, &region.max.y)
                .ok_or(NegotiatedRouterError::InvalidPolicy)?;
        }
        merged.push(candidate);
    }
    merged.sort_by(|first, second| {
        first
            .min
            .x
            .partial_cmp(&second.min.x)
            .unwrap_or(Ordering::Equal)
            .then_with(|| {
                first
                    .min
                    .y
                    .partial_cmp(&second.min.y)
                    .unwrap_or(Ordering::Equal)
            })
    });
    Ok(merged)
}

fn refinement_regions_overlap(
    first: &NegotiatedGridRefinementRegion,
    second: &NegotiatedGridRefinementRegion,
) -> bool {
    first.min.x.partial_cmp(&second.max.x) != Some(Ordering::Greater)
        && second.min.x.partial_cmp(&first.max.x) != Some(Ordering::Greater)
        && first.min.y.partial_cmp(&second.max.y) != Some(Ordering::Greater)
        && second.min.y.partial_cmp(&first.max.y) != Some(Ordering::Greater)
}

fn refinement_grid_axis(
    min: Real,
    max: Real,
    pitch: &Real,
    cap: usize,
) -> Result<Vec<Real>, NegotiatedRouterError> {
    let values = grid_axis(min, max.clone(), pitch, cap)?;
    if values.last() != Some(&max) {
        return Err(NegotiatedRouterError::InvalidPolicy);
    }
    Ok(values)
}

fn validate_refinement_region(
    region: &NegotiatedGridRefinementRegion,
    board_min: &Point2,
    board_max: &Point2,
    coarse_xs: &[Real],
    coarse_ys: &[Real],
) -> Result<(), NegotiatedRouterError> {
    let ordered = region.min.x.partial_cmp(&region.max.x) == Some(Ordering::Less)
        && region.min.y.partial_cmp(&region.max.y) == Some(Ordering::Less);
    let in_bounds = coordinate_is_in_bounds(&region.min.x, &board_min.x, &board_max.x)?
        && coordinate_is_in_bounds(&region.max.x, &board_min.x, &board_max.x)?
        && coordinate_is_in_bounds(&region.min.y, &board_min.y, &board_max.y)?
        && coordinate_is_in_bounds(&region.max.y, &board_min.y, &board_max.y)?;
    let intersects_coarse_mesh = coarse_xs.iter().any(|coordinate| {
        coordinate_is_in_bounds(coordinate, &region.min.x, &region.max.x) == Ok(true)
    }) && coarse_ys.iter().any(|coordinate| {
        coordinate_is_in_bounds(coordinate, &region.min.y, &region.max.y) == Ok(true)
    });
    if !ordered || !in_bounds || !intersects_coarse_mesh {
        return Err(NegotiatedRouterError::InvalidPolicy);
    }
    Ok(())
}

fn validate_refined_axis_spacing(
    values: &[Real],
    min: &Real,
    max: &Real,
    pitch: &Real,
) -> Result<(), NegotiatedRouterError> {
    let local = values
        .iter()
        .filter(|value| coordinate_is_in_bounds(value, min, max) == Ok(true))
        .collect::<Vec<_>>();
    if local.windows(2).any(|pair| {
        let spacing = pair[1].clone() - pair[0].clone();
        spacing.partial_cmp(pitch) == Some(Ordering::Less)
    }) {
        return Err(NegotiatedRouterError::InvalidPolicy);
    }
    Ok(())
}

#[derive(Clone, Copy)]
enum Axis {
    X,
    Y,
}

fn axis_coordinate(point: &Point2, axis: Axis) -> Real {
    match axis {
        Axis::X => point.x.clone(),
        Axis::Y => point.y.clone(),
    }
}

fn routing_feature_coordinates(
    layout: &PcbLayout,
    handoff: &RoutingProblemReport,
    axis: Axis,
) -> Vec<Real> {
    let mut values = handoff
        .problem
        .terminals
        .iter()
        .map(|terminal| axis_coordinate(&terminal.center, axis))
        .collect::<Vec<_>>();
    for placement in &layout.placements {
        values.push(axis_coordinate(&placement.position, axis));
        if let Some(pattern) = layout
            .land_patterns
            .iter()
            .find(|pattern| pattern.id == placement.land_pattern)
        {
            values.extend(
                pattern
                    .pads
                    .iter()
                    .map(|pad| axis_coordinate(&placement.transform_point(&pad.center), axis)),
            );
        }
    }
    for contour in std::iter::once(&layout.outline.exterior).chain(&layout.outline.cutouts) {
        for segment in contour.segments() {
            values.push(axis_coordinate(segment.start(), axis));
            values.push(axis_coordinate(segment.end(), axis));
        }
    }
    for route in &layout.routes {
        for segment in &route.segments {
            values.push(axis_coordinate(segment.start(), axis));
            values.push(axis_coordinate(segment.end(), axis));
        }
    }
    values.extend(
        layout
            .vias
            .iter()
            .map(|via| axis_coordinate(&via.center, axis)),
    );
    for boundary in layout
        .zones
        .iter()
        .map(|zone| &zone.boundary)
        .chain(layout.keepouts.iter().map(|keepout| &keepout.boundary))
        .chain(
            layout
                .rules
                .route_rule_regions
                .iter()
                .map(|region| &region.boundary),
        )
        .chain(
            layout
                .rules
                .route_constraint_regions
                .iter()
                .map(|region| &region.boundary),
        )
    {
        values.extend(boundary.iter().map(|point| axis_coordinate(point, axis)));
    }
    values
}

fn adaptive_grid_axis(
    min: Real,
    max: Real,
    coarse_pitch: &Real,
    fine_pitch: &Real,
    features: &[Real],
    halo_steps: usize,
    cap: usize,
) -> Result<(Vec<Real>, usize), NegotiatedRouterError> {
    let mut values = grid_axis(min.clone(), max.clone(), coarse_pitch, cap)?;
    let base_len = values.len();
    values.push(min.clone());
    values.push(max.clone());
    for feature in features {
        if !coordinate_is_in_bounds(feature, &min, &max)? {
            continue;
        }
        values.push(feature.clone());
        let mut offset = Real::zero();
        for _ in 0..halo_steps {
            offset += fine_pitch.clone();
            let before = feature.clone() - offset.clone();
            if coordinate_is_in_bounds(&before, &min, &max)? {
                values.push(before);
            }
            let after = feature.clone() + offset.clone();
            if coordinate_is_in_bounds(&after, &min, &max)? {
                values.push(after);
            }
            if values.len() > cap.saturating_mul(4) {
                return Err(NegotiatedRouterError::GridTooLarge);
            }
        }
    }
    sort_dedup_coordinates(&mut values)?;
    if values.len() > cap {
        return Err(NegotiatedRouterError::GridTooLarge);
    }
    let injected = values.len().saturating_sub(base_len);
    Ok((values, injected))
}

struct DifferentialPairGridGeometry {
    translation: Point2,
    endpoint_points: Vec<Point2>,
}

fn differential_pair_grid_geometry(
    layout: &PcbLayout,
    handoff: &RoutingProblemReport,
    policy: &NegotiatedRoutePolicy,
) -> Result<Vec<DifferentialPairGridGeometry>, NegotiatedRouterError> {
    let mut geometries = Vec::new();
    for pair in &layout.rules.differential_pairs {
        let positive = handoff
            .problem
            .terminals
            .iter()
            .filter(|terminal| terminal.net == pair.positive)
            .collect::<Vec<_>>();
        let negative = handoff
            .problem
            .terminals
            .iter()
            .filter(|terminal| terminal.net == pair.negative)
            .collect::<Vec<_>>();
        if positive.len() != 2 || negative.len() != 2 {
            continue;
        }
        let direct = point_delta(&positive[0].center, &negative[0].center)
            == point_delta(&positive[1].center, &negative[1].center);
        let negative_index = if direct {
            0
        } else if point_delta(&positive[0].center, &negative[1].center)
            == point_delta(&positive[1].center, &negative[0].center)
        {
            1
        } else {
            continue;
        };
        let negative_order = [negative_index, 1 - negative_index];
        let (positive_width, _) = maximum_net_rule(layout, handoff, &pair.positive, policy)?;
        let (negative_width, _) = maximum_net_rule(layout, handoff, &pair.negative, policy)?;
        let positive_half_width =
            (positive_width / Real::from(2)).map_err(|_| NegotiatedRouterError::InvalidPolicy)?;
        let negative_half_width =
            (negative_width / Real::from(2)).map_err(|_| NegotiatedRouterError::InvalidPolicy)?;
        let nominal =
            positive_half_width.clone() + negative_half_width.clone() + pair.spacing.clone();
        let nominal_squared = nominal.clone() * nominal.clone();
        let start_squared =
            point_distance_squared(&positive[0].center, &negative[negative_order[0]].center);
        let end_squared =
            point_distance_squared(&positive[1].center, &negative[negative_order[1]].center);
        if start_squared.partial_cmp(&nominal_squared) == Some(Ordering::Equal)
            && end_squared.partial_cmp(&nominal_squared) == Some(Ordering::Equal)
        {
            let (x, y) = point_delta(&positive[0].center, &negative[negative_order[0]].center);
            geometries.push(DifferentialPairGridGeometry {
                translation: Point2::new(x, y),
                endpoint_points: Vec::new(),
            });
            continue;
        }
        let Some(neckdown) = &pair.neckdown else {
            continue;
        };
        let reduced = neckdown.trace_width.clone() + neckdown.spacing.clone();
        let reduced_squared = reduced.clone() * reduced.clone();
        if start_squared.partial_cmp(&reduced_squared) != Some(Ordering::Equal)
            || end_squared.partial_cmp(&reduced_squared) != Some(Ordering::Equal)
        {
            continue;
        }
        if reduced.partial_cmp(&nominal) != Some(Ordering::Less) {
            continue;
        }
        let transition = ((nominal.clone() - reduced) / Real::from(2))
            .map_err(|_| NegotiatedRouterError::InvalidPolicy)?;
        let mut endpoint_points = Vec::with_capacity(4);
        let mut translation = None;
        for index in 0..2 {
            let (positive_point, negative_point, endpoint_translation) = expanded_pair_centers(
                &positive[index].center,
                &negative[negative_order[index]].center,
                &nominal,
                &transition,
            )
            .ok_or_else(|| NegotiatedRouterError::UnsupportedDifferentialPair(pair.id.clone()))?;
            if translation
                .as_ref()
                .is_some_and(|candidate| candidate != &endpoint_translation)
            {
                return Err(NegotiatedRouterError::UnsupportedDifferentialPair(
                    pair.id.clone(),
                ));
            }
            translation = Some(endpoint_translation);
            endpoint_points.push(positive_point);
            endpoint_points.push(negative_point);
        }
        if let Some(translation) = translation {
            geometries.push(DifferentialPairGridGeometry {
                translation,
                endpoint_points,
            });
        }
    }
    Ok(geometries)
}

fn close_axis_translation(
    values: &mut Vec<Real>,
    translation: &Real,
    min: &Real,
    max: &Real,
    cap: usize,
) -> Result<usize, NegotiatedRouterError> {
    let original_len = values.len();
    let translated = values
        .iter()
        .map(|value| value.clone() + translation.clone())
        .collect::<Vec<_>>();
    for value in translated {
        if coordinate_is_in_bounds(&value, min, max)? {
            values.push(value);
        }
    }
    sort_dedup_coordinates(values)?;
    if values.len() > cap {
        return Err(NegotiatedRouterError::GridTooLarge);
    }
    Ok(values.len().saturating_sub(original_len))
}

fn coordinate_is_in_bounds(
    value: &Real,
    min: &Real,
    max: &Real,
) -> Result<bool, NegotiatedRouterError> {
    let after_min = value
        .partial_cmp(min)
        .ok_or(NegotiatedRouterError::InvalidPolicy)?
        != Ordering::Less;
    let before_max = value
        .partial_cmp(max)
        .ok_or(NegotiatedRouterError::InvalidPolicy)?
        != Ordering::Greater;
    Ok(after_min && before_max)
}

fn sort_dedup_coordinates(values: &mut Vec<Real>) -> Result<(), NegotiatedRouterError> {
    let mut indeterminate = false;
    values.sort_by(|first, second| {
        first.partial_cmp(second).unwrap_or_else(|| {
            indeterminate = true;
            Ordering::Equal
        })
    });
    if indeterminate {
        return Err(NegotiatedRouterError::InvalidPolicy);
    }
    values.dedup();
    Ok(())
}

fn span_units(span: &Real, pitch: &Real, cap: usize) -> Option<u64> {
    let mut covered = Real::zero();
    let mut units = 0_u64;
    while covered.partial_cmp(span)? == Ordering::Less {
        if units as u128 >= cap as u128 {
            return None;
        }
        covered += pitch.clone();
        units = units.saturating_add(1);
    }
    Some(units)
}

fn route_net(
    router: &RouterContext<'_>,
    net: &NetId,
    occupancy: &BTreeMap<Resource, BTreeSet<NetId>>,
    expanded_total: &mut usize,
) -> Result<NetRoute, NegotiatedRouteFailure> {
    let terminals = router
        .handoff
        .problem
        .terminals
        .iter()
        .filter(|terminal| &terminal.net == net)
        .collect::<Vec<_>>();
    let mut terminal_nodes = Vec::new();
    for (index, terminal) in terminals.iter().enumerate() {
        terminal_nodes.push(
            router
                .grid
                .terminal_nodes(&terminal.center, &terminal.layers)
                .ok_or_else(|| NegotiatedRouteFailure::OffGridTerminal {
                    net: net.clone(),
                    terminal: index,
                })?,
        );
    }
    let via_style = resolved_via_style(router.layout, net, router.policy);
    let via_half_land = via_style
        .half_land()
        .expect("validated via style has a positive land");
    let mut route = NetRoute::default();
    let mut tree = terminal_nodes[0].iter().copied().collect::<BTreeSet<_>>();
    for (terminal_index, targets) in terminal_nodes.iter().enumerate().skip(1) {
        let search = SearchContext {
            router,
            net,
            terminal_index,
            occupancy,
            via_half_land: &via_half_land,
            via_style: &via_style,
            paired_net: None,
        };
        let path = search.search_connection(&tree, targets, expanded_total)?;
        for pair in path.windows(2) {
            route
                .resources
                .extend(transition_resources(router.grid, pair[0], pair[1]));
        }
        for node in &path {
            route.resources.insert(Resource::Node(*node));
            tree.insert(*node);
        }
        if path.is_empty() {
            return Err(NegotiatedRouteFailure::NoPath {
                net: net.clone(),
                terminal: terminal_index,
            });
        }
        route.paths.push(path);
    }
    Ok(route)
}

fn route_differential_pair(
    router: &RouterContext<'_>,
    problem: &PreparedDifferentialPair,
    occupancy: &BTreeMap<Resource, BTreeSet<NetId>>,
    expanded_total: &mut usize,
) -> Result<(NetRoute, NetRoute), NegotiatedRouteFailure> {
    let positive_search = SearchContext {
        router,
        net: &problem.pair.positive,
        terminal_index: 1,
        via_half_land: &problem.positive_via_half_land,
        via_style: &problem.positive_via_style,
        paired_net: Some(&problem.pair.negative),
        occupancy,
    };
    let negative_search = SearchContext {
        router,
        net: &problem.pair.negative,
        terminal_index: 1,
        via_half_land: &problem.negative_via_half_land,
        via_style: &problem.negative_via_style,
        paired_net: Some(&problem.pair.positive),
        occupancy,
    };
    let target_set = problem.targets.iter().copied().collect::<BTreeSet<_>>();
    let mut queue = BinaryHeap::<Reverse<(u64, u64, PairSearchState)>>::new();
    let mut distance = BTreeMap::<PairSearchState, u64>::new();
    let mut previous = BTreeMap::<PairSearchState, PairSearchState>::new();
    for source in &problem.sources {
        let escape = problem
            .source_escapes
            .get(source)
            .expect("prepared pair sources have escape geometry");
        match differential_pair_escape_is_legal(
            &positive_search,
            &negative_search,
            escape,
            problem
                .neckdown_transition_length
                .as_ref()
                .and(problem.pair.neckdown.as_ref())
                .map(|neckdown| &neckdown.trace_width),
        ) {
            Some(true) => {}
            Some(false) => continue,
            None => {
                return Err(NegotiatedRouteFailure::DifferentialPairIndeterminate(
                    problem.pair.id.clone(),
                ));
            }
        }
        let state = PairSearchState {
            nodes: *source,
            direction: None,
        };
        distance.insert(state, 0);
        let estimate = pair_heuristic(*source, &problem.targets, router.grid, router.policy)
            .ok_or_else(|| {
                NegotiatedRouteFailure::DifferentialPairIndeterminate(problem.pair.id.clone())
            })?;
        queue.push(Reverse((estimate, 0, state)));
    }
    let mut expanded = 0;
    while let Some(Reverse((_, cost, state))) = queue.pop() {
        if distance.get(&state).copied() != Some(cost) {
            continue;
        }
        expanded += 1;
        *expanded_total = expanded_total.saturating_add(1);
        if expanded > router.policy.maximum_expansions_per_connection {
            return Err(NegotiatedRouteFailure::DifferentialPairExpansionLimit(
                problem.pair.id.clone(),
            ));
        }
        if target_set.contains(&state.nodes) {
            let target_escape = problem
                .target_escapes
                .get(&state.nodes)
                .expect("prepared pair targets have escape geometry");
            match differential_pair_escape_is_legal(
                &positive_search,
                &negative_search,
                target_escape,
                problem
                    .neckdown_transition_length
                    .as_ref()
                    .and(problem.pair.neckdown.as_ref())
                    .map(|neckdown| &neckdown.trace_width),
            ) {
                Some(true) => {}
                Some(false) => continue,
                None => {
                    return Err(NegotiatedRouteFailure::DifferentialPairIndeterminate(
                        problem.pair.id.clone(),
                    ));
                }
            }
            let mut path = vec![state.nodes];
            let mut cursor = state;
            while let Some(parent) = previous.get(&cursor).copied() {
                path.push(parent.nodes);
                cursor = parent;
            }
            path.reverse();
            path.dedup();
            let source_escape = problem
                .source_escapes
                .get(&path[0])
                .expect("accepted pair path starts at a prepared source");
            let mut positive = source_escape.positive.clone();
            positive.extend(path.iter().skip(1).map(|nodes| nodes.positive));
            positive.extend(target_escape.positive.iter().skip(1).copied());
            positive.dedup();
            let mut negative = source_escape.negative.clone();
            negative.extend(path.iter().skip(1).map(|nodes| nodes.negative));
            negative.extend(target_escape.negative.iter().skip(1).copied());
            negative.dedup();
            let mut positive_route = net_route_from_path(router.grid, positive);
            let mut negative_route = net_route_from_path(router.grid, negative);
            if problem.neckdown_transition_length.is_some() {
                let width = &problem
                    .pair
                    .neckdown
                    .as_ref()
                    .expect("used neckdown retains its policy")
                    .trace_width;
                retain_escape_width(
                    &mut positive_route,
                    [&source_escape.positive, &target_escape.positive],
                    width,
                );
                retain_escape_width(
                    &mut negative_route,
                    [&source_escape.negative, &target_escape.negative],
                    width,
                );
            }
            return Ok((positive_route, negative_route));
        }
        for (next, direction) in pair_neighbors(
            router.grid,
            state.nodes,
            &problem.translation,
            router.policy.planar_topology,
        ) {
            let Some(positive_legal) =
                positive_search.edge_is_legal(state.nodes.positive, next.positive, direction)
            else {
                return Err(NegotiatedRouteFailure::DifferentialPairIndeterminate(
                    problem.pair.id.clone(),
                ));
            };
            let Some(negative_legal) =
                negative_search.edge_is_legal(state.nodes.negative, next.negative, direction)
            else {
                return Err(NegotiatedRouteFailure::DifferentialPairIndeterminate(
                    problem.pair.id.clone(),
                ));
            };
            if !positive_legal || !negative_legal {
                continue;
            }
            let mut step = if direction == Direction::Via {
                router.policy.via_penalty.saturating_mul(2)
            } else {
                router
                    .grid
                    .planar_step_units(state.nodes.positive, next.positive, router.policy)
                    .and_then(|positive| {
                        router
                            .grid
                            .planar_step_units(state.nodes.negative, next.negative, router.policy)
                            .map(|negative| positive.saturating_add(negative))
                    })
                    .ok_or_else(|| {
                        NegotiatedRouteFailure::DifferentialPairIndeterminate(
                            problem.pair.id.clone(),
                        )
                    })?
            };
            if state.direction.is_some_and(|old| old != direction)
                && direction != Direction::Via
                && state.direction != Some(Direction::Via)
            {
                step = step.saturating_add(router.policy.bend_penalty);
            }
            step = step
                .saturating_add(transition_congestion(
                    router,
                    occupancy,
                    &problem.pair.positive,
                    state.nodes.positive,
                    next.positive,
                ))
                .saturating_add(transition_congestion(
                    router,
                    occupancy,
                    &problem.pair.negative,
                    state.nodes.negative,
                    next.negative,
                ));
            let next_state = PairSearchState {
                nodes: next,
                direction: Some(direction),
            };
            let next_cost = cost.saturating_add(step);
            if distance
                .get(&next_state)
                .is_none_or(|known| next_cost < *known)
            {
                distance.insert(next_state, next_cost);
                previous.insert(next_state, state);
                let estimate = pair_heuristic(next, &problem.targets, router.grid, router.policy)
                    .ok_or_else(|| {
                    NegotiatedRouteFailure::DifferentialPairIndeterminate(problem.pair.id.clone())
                })?;
                queue.push(Reverse((
                    next_cost.saturating_add(estimate),
                    next_cost,
                    next_state,
                )));
            }
        }
    }
    Err(NegotiatedRouteFailure::DifferentialPairNoPath(
        problem.pair.id.clone(),
    ))
}

fn differential_pair_escape_is_legal(
    positive_search: &SearchContext<'_>,
    negative_search: &SearchContext<'_>,
    escape: &DifferentialPairEscape,
    width: Option<&Real>,
) -> Option<bool> {
    for (search, path) in [
        (positive_search, &escape.positive),
        (negative_search, &escape.negative),
    ] {
        for edge in path.windows(2) {
            let direction = direction_between(search.router.grid, edge[0], edge[1]);
            if !search.edge_is_legal_with_width(edge[0], edge[1], direction, width)? {
                return Some(false);
            }
        }
    }
    Some(true)
}

fn retain_escape_width<'a>(
    route: &mut NetRoute,
    paths: impl IntoIterator<Item = &'a Vec<Node>>,
    width: &Real,
) {
    for path in paths {
        for edge in path.windows(2) {
            route
                .width_overrides
                .insert(resource(edge[0], edge[1]), width.clone());
        }
    }
}

fn net_route_from_path(grid: &Grid, path: Vec<Node>) -> NetRoute {
    let mut resources = path
        .iter()
        .copied()
        .map(Resource::Node)
        .collect::<BTreeSet<_>>();
    resources.extend(
        path.windows(2)
            .flat_map(|pair| transition_resources(grid, pair[0], pair[1])),
    );
    NetRoute {
        paths: vec![path],
        resources,
        width_overrides: BTreeMap::new(),
    }
}

fn differential_pair_evidence(
    problem: &PreparedDifferentialPair,
    routes: &BTreeMap<NetId, NetRoute>,
    grid: &Grid,
) -> Result<NegotiatedDifferentialPairEvidence, NegotiatedRouterError> {
    let positive = routes.get(&problem.pair.positive).ok_or_else(|| {
        NegotiatedRouterError::UnsupportedDifferentialPair(problem.pair.id.clone())
    })?;
    let negative = routes.get(&problem.pair.negative).ok_or_else(|| {
        NegotiatedRouterError::UnsupportedDifferentialPair(problem.pair.id.clone())
    })?;
    let positive_length = net_route_planar_length(positive, grid);
    let negative_length = net_route_planar_length(negative, grid);
    let skew = real_abs(&(positive_length.clone() - negative_length.clone()));
    if problem
        .pair
        .max_skew
        .as_ref()
        .is_some_and(|maximum| skew.partial_cmp(maximum) == Some(Ordering::Greater))
    {
        return Err(NegotiatedRouterError::UnsupportedDifferentialPair(
            problem.pair.id.clone(),
        ));
    }
    let positive_vias = positive
        .resources
        .iter()
        .filter(|resource| matches!(resource, Resource::Via(_, _)))
        .count();
    let negative_vias = negative
        .resources
        .iter()
        .filter(|resource| matches!(resource, Resource::Via(_, _)))
        .count();
    if positive_vias != negative_vias {
        return Err(NegotiatedRouterError::UnsupportedDifferentialPair(
            problem.pair.id.clone(),
        ));
    }
    Ok(NegotiatedDifferentialPairEvidence {
        pair: problem.pair.id.clone(),
        center_spacing: problem.positive_half_width.clone()
            + problem.negative_half_width.clone()
            + problem.pair.spacing.clone(),
        positive_length,
        negative_length,
        skew,
        paired_vias: positive_vias,
        neckdown: problem.neckdown_transition_length.as_ref().map(|length| {
            let source = problem
                .source_escapes
                .values()
                .next()
                .expect("prepared pair has source escape geometry");
            let target = problem
                .target_escapes
                .values()
                .next()
                .expect("prepared pair has target escape geometry");
            NegotiatedDifferentialPairNeckdownEvidence {
                trace_width: problem
                    .pair
                    .neckdown
                    .as_ref()
                    .expect("used neckdown retains its policy")
                    .trace_width
                    .clone(),
                spacing: problem
                    .pair
                    .neckdown
                    .as_ref()
                    .expect("used neckdown retains its policy")
                    .spacing
                    .clone(),
                transition_length: length.clone(),
                source_planar_edges: source
                    .positive
                    .len()
                    .saturating_sub(1)
                    .saturating_add(source.negative.len().saturating_sub(1)),
                target_planar_edges: target
                    .positive
                    .len()
                    .saturating_sub(1)
                    .saturating_add(target.negative.len().saturating_sub(1)),
            }
        }),
    })
}

fn net_route_planar_length(route: &NetRoute, grid: &Grid) -> Real {
    route
        .paths
        .iter()
        .flat_map(|path| path.windows(2))
        .filter(|pair| pair[0].layer == pair[1].layer)
        .fold(Real::zero(), |length, pair| {
            let start = grid.point(pair[0]);
            let end = grid.point(pair[1]);
            let dx = end.x - start.x;
            let dy = end.y - start.y;
            length
                + (dx.clone() * dx + dy.clone() * dy)
                    .sqrt()
                    .expect("exact route segment length is nonnegative")
        })
}

fn route_constraint_evidence(
    layout: &PcbLayout,
    grid: &Grid,
    routes: &BTreeMap<NetId, NetRoute>,
) -> Result<Vec<NegotiatedRouteConstraintEvidence>, NegotiatedRouterError> {
    layout
        .rules
        .route_constraint_regions
        .iter()
        .map(|region| route_region_use(region, grid, routes))
        .collect()
}

fn route_rule_region_evidence(
    layout: &PcbLayout,
    grid: &Grid,
    routes: &BTreeMap<NetId, NetRoute>,
) -> Result<Vec<NegotiatedRouteRuleRegionEvidence>, NegotiatedRouterError> {
    layout
        .rules
        .route_rule_regions
        .iter()
        .map(|region| {
            let mut planar = 0_usize;
            let mut vias = 0_usize;
            for (net, route) in routes {
                if !constraint_selects_net(&region.nets, net) {
                    continue;
                }
                for pair in route.paths.iter().flat_map(|path| path.windows(2)) {
                    if !edge_touches_region(
                        &grid.point(pair[0]),
                        &grid.point(pair[1]),
                        &region.boundary,
                    )
                    .ok_or(NegotiatedRouterError::InvalidPolicy)?
                    {
                        continue;
                    }
                    if pair[0].layer == pair[1].layer {
                        planar = planar.saturating_add(1);
                    } else {
                        vias = vias.saturating_add(1);
                    }
                }
            }
            Ok(NegotiatedRouteRuleRegionEvidence {
                region: region.id.clone(),
                constrained_planar_edges: planar,
                constrained_vias: vias,
                width_override_edges: region.min_trace_width.as_ref().map_or(0, |_| planar),
                clearance_override_edges: region
                    .min_clearance
                    .as_ref()
                    .map_or(0, |_| planar.saturating_add(vias)),
            })
        })
        .collect()
}

fn route_region_use(
    region: &RouteConstraintRegion,
    grid: &Grid,
    routes: &BTreeMap<NetId, NetRoute>,
) -> Result<NegotiatedRouteConstraintEvidence, NegotiatedRouterError> {
    let mut planar = 0_usize;
    let mut vias = 0_usize;
    for (net, route) in routes {
        if !constraint_selects_net(&region.nets, net) {
            continue;
        }
        for pair in route.paths.iter().flat_map(|path| path.windows(2)) {
            if edge_touches_region(&grid.point(pair[0]), &grid.point(pair[1]), &region.boundary)
                .ok_or(NegotiatedRouterError::InvalidPolicy)?
            {
                if pair[0].layer == pair[1].layer {
                    planar = planar.saturating_add(1);
                } else {
                    vias = vias.saturating_add(1);
                }
            }
        }
    }
    Ok(NegotiatedRouteConstraintEvidence {
        region: region.id.clone(),
        constrained_planar_edges: planar,
        constrained_vias: vias,
    })
}

fn escape_policy_evidence(
    layout: &PcbLayout,
    handoff: &RoutingProblemReport,
    grid: &Grid,
    routes: &BTreeMap<NetId, NetRoute>,
) -> Result<Vec<NegotiatedEscapePolicyEvidence>, NegotiatedRouterError> {
    layout
        .rules
        .escape_policies
        .iter()
        .map(|policy| escape_policy_use(policy, handoff, grid, routes))
        .collect()
}

fn escape_policy_use(
    policy: &EscapePolicy,
    handoff: &RoutingProblemReport,
    grid: &Grid,
    routes: &BTreeMap<NetId, NetRoute>,
) -> Result<NegotiatedEscapePolicyEvidence, NegotiatedRouterError> {
    let mut planar = 0_usize;
    let mut vias = 0_usize;
    for (net, route) in routes {
        if !constraint_selects_net(&policy.nets, net) {
            continue;
        }
        let terminals = handoff
            .problem
            .terminals
            .iter()
            .filter(|terminal| {
                &terminal.net == net && policy.instances.contains(&terminal.instance)
            })
            .collect::<Vec<_>>();
        for pair in route.paths.iter().flat_map(|path| path.windows(2)) {
            let start = grid.point(pair[0]);
            let end = grid.point(pair[1]);
            let applies = terminals.iter().try_fold(false, |applies, terminal| {
                let inside = segment_touches_manhattan_ball(
                    &terminal.center,
                    &start,
                    &end,
                    &policy.max_distance,
                )
                .ok_or(NegotiatedRouterError::InvalidPolicy)?;
                Ok::<_, NegotiatedRouterError>(applies || inside)
            })?;
            if applies {
                if pair[0].layer == pair[1].layer {
                    planar = planar.saturating_add(1);
                } else {
                    vias = vias.saturating_add(1);
                }
            }
        }
    }
    Ok(NegotiatedEscapePolicyEvidence {
        policy: policy.id.clone(),
        constrained_planar_edges: planar,
        constrained_vias: vias,
    })
}

struct RoutedPlanarEdge {
    net: NetId,
    first: Node,
    second: Node,
    start: Point2,
    end: Point2,
    layer: TraceLayer,
    half_width: Real,
    clearance: Real,
}

fn account_inter_route_clearance(
    layout: &PcbLayout,
    handoff: &RoutingProblemReport,
    grid: &Grid,
    policy: &NegotiatedRoutePolicy,
    routes: &BTreeMap<NetId, NetRoute>,
    occupancy: &mut BTreeMap<Resource, BTreeSet<NetId>>,
) -> Result<(), NegotiatedRouterError> {
    let mut edges = Vec::<RoutedPlanarEdge>::new();
    for (net, route) in routes {
        for pair in route.paths.iter().flat_map(|path| path.windows(2)) {
            if pair[0].layer != pair[1].layer {
                continue;
            }
            let start = grid.point(pair[0]);
            let end = grid.point(pair[1]);
            let (rule_width, clearance) = edge_net_rule(layout, handoff, net, policy, &start, &end)
                .ok_or(NegotiatedRouterError::InvalidPolicy)?;
            let width = route
                .width_overrides
                .get(&resource(pair[0], pair[1]))
                .cloned()
                .unwrap_or(rule_width);
            let half_width =
                (width / Real::from(2)).map_err(|_| NegotiatedRouterError::InvalidPolicy)?;
            edges.push(RoutedPlanarEdge {
                net: net.clone(),
                first: pair[0],
                second: pair[1],
                start,
                end,
                layer: grid.layers[pair[0].layer],
                half_width,
                clearance,
            });
        }
    }
    for first_index in 0..edges.len() {
        for second_index in first_index + 1..edges.len() {
            let first = &edges[first_index];
            let second = &edges[second_index];
            if first.net == second.net
                || first.layer != second.layer
                || retained_differential_pair(layout, &first.net, &second.net)
            {
                continue;
            }
            let clearance = exact_max(&first.clearance, &second.clearance)
                .ok_or(NegotiatedRouterError::InvalidPolicy)?;
            let required = first.half_width.clone() + second.half_width.clone() + clearance;
            let distance_squared = segment_segment_distance_squared(
                &first.start,
                &first.end,
                &second.start,
                &second.end,
            )
            .ok_or(NegotiatedRouterError::InvalidPolicy)?;
            if distance_squared.partial_cmp(&(required.clone() * required)) != Some(Ordering::Less)
            {
                continue;
            }
            for edge in [first, second] {
                let users = occupancy
                    .entry(resource(edge.first, edge.second))
                    .or_default();
                users.insert(first.net.clone());
                users.insert(second.net.clone());
            }
        }
    }
    Ok(())
}

fn retained_differential_pair(layout: &PcbLayout, first: &NetId, second: &NetId) -> bool {
    layout.rules.differential_pairs.iter().any(|pair| {
        (&pair.positive == first && &pair.negative == second)
            || (&pair.positive == second && &pair.negative == first)
    })
}

fn transition_congestion(
    router: &RouterContext<'_>,
    occupancy: &BTreeMap<Resource, BTreeSet<NetId>>,
    net: &NetId,
    first: Node,
    second: Node,
) -> u64 {
    transition_resources(router.grid, first, second)
        .into_iter()
        .chain([Resource::Node(second)])
        .fold(0_u64, |cost, resource| {
            let present = occupancy.get(&resource).map_or(0, |users| {
                users.iter().filter(|user| *user != net).count() as u64
            });
            cost.saturating_add(present.saturating_mul(router.policy.present_congestion_penalty))
                .saturating_add(router.history.get(&resource).copied().unwrap_or_default())
        })
}

impl SearchContext<'_> {
    fn search_connection(
        &self,
        sources: &BTreeSet<Node>,
        targets: &[Node],
        expanded_total: &mut usize,
    ) -> Result<Vec<Node>, NegotiatedRouteFailure> {
        let target_set = targets.iter().copied().collect::<BTreeSet<_>>();
        let mut queue = BinaryHeap::<Reverse<(u64, u64, SearchState)>>::new();
        let mut distance = BTreeMap::<SearchState, u64>::new();
        let mut previous = BTreeMap::<SearchState, SearchState>::new();
        for source in sources {
            let state = SearchState {
                node: *source,
                direction: None,
            };
            distance.insert(state, 0);
            let estimate = heuristic(*source, targets, self.router.grid, self.router.policy)
                .ok_or_else(|| NegotiatedRouteFailure::Indeterminate {
                    net: self.net.clone(),
                    terminal: self.terminal_index,
                })?;
            queue.push(Reverse((estimate, 0, state)));
        }
        let mut expanded = 0;
        while let Some(Reverse((_, cost, state))) = queue.pop() {
            if distance.get(&state).copied() != Some(cost) {
                continue;
            }
            expanded += 1;
            *expanded_total = expanded_total.saturating_add(1);
            if expanded > self.router.policy.maximum_expansions_per_connection {
                return Err(NegotiatedRouteFailure::ExpansionLimit {
                    net: self.net.clone(),
                    terminal: self.terminal_index,
                });
            }
            if target_set.contains(&state.node) {
                let mut path = vec![state.node];
                let mut cursor = state;
                while let Some(parent) = previous.get(&cursor).copied() {
                    path.push(parent.node);
                    cursor = parent;
                }
                path.reverse();
                path.dedup();
                return Ok(path);
            }
            for (next, direction) in neighbors(
                self.router.grid,
                state.node,
                self.router.policy.planar_topology,
            ) {
                let Some(legal) = self.edge_is_legal(state.node, next, direction) else {
                    return Err(NegotiatedRouteFailure::Indeterminate {
                        net: self.net.clone(),
                        terminal: self.terminal_index,
                    });
                };
                if !legal {
                    continue;
                }
                let mut step = if direction == Direction::Via {
                    self.router.policy.via_penalty
                } else {
                    self.router
                        .grid
                        .planar_step_units(state.node, next, self.router.policy)
                        .ok_or_else(|| NegotiatedRouteFailure::Indeterminate {
                            net: self.net.clone(),
                            terminal: self.terminal_index,
                        })?
                };
                if state.direction.is_some_and(|old| old != direction)
                    && direction != Direction::Via
                    && state.direction != Some(Direction::Via)
                {
                    step = step.saturating_add(self.router.policy.bend_penalty);
                }
                step = step.saturating_add(transition_congestion(
                    self.router,
                    self.occupancy,
                    self.net,
                    state.node,
                    next,
                ));
                let next_state = SearchState {
                    node: next,
                    direction: Some(direction),
                };
                let next_cost = cost.saturating_add(step);
                if distance
                    .get(&next_state)
                    .is_none_or(|known| next_cost < *known)
                {
                    distance.insert(next_state, next_cost);
                    previous.insert(next_state, state);
                    let estimate = heuristic(next, targets, self.router.grid, self.router.policy)
                        .ok_or_else(|| NegotiatedRouteFailure::Indeterminate {
                        net: self.net.clone(),
                        terminal: self.terminal_index,
                    })?;
                    queue.push(Reverse((
                        next_cost.saturating_add(estimate),
                        next_cost,
                        next_state,
                    )));
                }
            }
        }
        Err(NegotiatedRouteFailure::NoPath {
            net: self.net.clone(),
            terminal: self.terminal_index,
        })
    }

    fn edge_is_legal(&self, first: Node, second: Node, direction: Direction) -> Option<bool> {
        self.edge_is_legal_with_width(first, second, direction, None)
    }

    fn edge_is_legal_with_width(
        &self,
        first: Node,
        second: Node,
        direction: Direction,
        width_override: Option<&Real>,
    ) -> Option<bool> {
        let start = self.router.grid.point(first);
        let end = self.router.grid.point(second);
        let (mut width, clearance) = edge_net_rule(
            self.router.layout,
            self.router.handoff,
            self.net,
            self.router.policy,
            &start,
            &end,
        )?;
        if let Some(width_override) = width_override {
            width.clone_from(width_override);
        }
        let trace_half_width = (width / Real::from(2)).ok()?;
        if !authored_constraints_allow_edge(
            self.router.layout,
            &self.router.handoff.problem.terminals,
            self.router.grid,
            self.net,
            first,
            second,
            direction,
        )? {
            return Some(false);
        }
        if direction == Direction::Via {
            let expanded = self.via_half_land.clone() + clearance.clone();
            if !point_is_legal(
                self.router.layout,
                &self.router.grid.boundary,
                &start,
                &expanded,
                None,
            )? {
                return Some(false);
            }
            let first_layer = self.router.grid.layers[first.layer];
            let second_layer = self.router.grid.layers[second.layer];
            if !self.via_style.allows(first_layer, second_layer) {
                return Some(false);
            }
            return point_clear_of_fixed(
                &start,
                ClearanceSubject {
                    physical_radius: self.via_half_land,
                    clearance: &clearance,
                    net: self.net,
                    paired_net: self.paired_net,
                },
                &[first_layer, second_layer],
                self.router.fixed,
            );
        }
        let layer = self.router.grid.layers[first.layer];
        let expanded = trace_half_width.clone() + clearance.clone();
        if !segment_is_legal(
            self.router.layout,
            &self.router.grid.boundary,
            &start,
            &end,
            &expanded,
            layer,
        )? {
            return Some(false);
        }
        segment_clear_of_fixed(
            &start,
            &end,
            ClearanceSubject {
                physical_radius: &trace_half_width,
                clearance: &clearance,
                net: self.net,
                paired_net: self.paired_net,
            },
            layer,
            self.router.fixed,
        )
    }
}

fn authored_constraints_allow_edge(
    layout: &PcbLayout,
    terminals: &[RoutingTerminal],
    grid: &Grid,
    net: &NetId,
    first: Node,
    second: Node,
    direction: Direction,
) -> Option<bool> {
    let start = grid.point(first);
    let end = grid.point(second);
    for region in &layout.rules.route_constraint_regions {
        if !constraint_selects_net(&region.nets, net)
            || !edge_touches_region(&start, &end, &region.boundary)?
        {
            continue;
        }
        if !constraint_allows_transition(
            &region.allowed_layers,
            &region.allowed_directions,
            region.allow_vias,
            grid,
            first,
            second,
            direction,
        ) {
            return Some(false);
        }
    }
    for policy in &layout.rules.escape_policies {
        if !constraint_selects_net(&policy.nets, net) {
            continue;
        }
        let applies = terminals
            .iter()
            .filter(|terminal| {
                &terminal.net == net && policy.instances.contains(&terminal.instance)
            })
            .try_fold(false, |applies, terminal| {
                Some(
                    applies
                        || segment_touches_manhattan_ball(
                            &terminal.center,
                            &start,
                            &end,
                            &policy.max_distance,
                        )?,
                )
            })?;
        if applies
            && !constraint_allows_transition(
                &policy.allowed_layers,
                &policy.allowed_directions,
                policy.allow_vias,
                grid,
                first,
                second,
                direction,
            )
        {
            return Some(false);
        }
    }
    Some(true)
}

fn constraint_selects_net(selected: &[NetId], net: &NetId) -> bool {
    selected.is_empty() || selected.contains(net)
}

fn constraint_allows_transition(
    layers: &[TraceLayer],
    directions: &[RouteDirection],
    allow_vias: bool,
    grid: &Grid,
    first: Node,
    second: Node,
    direction: Direction,
) -> bool {
    if direction == Direction::Via {
        allow_vias
            && layers.contains(&grid.layers[first.layer])
            && layers.contains(&grid.layers[second.layer])
    } else {
        layers.contains(&grid.layers[first.layer])
            && directions.contains(&route_direction(direction))
    }
}

fn route_direction(direction: Direction) -> RouteDirection {
    match direction {
        Direction::Horizontal => RouteDirection::Horizontal,
        Direction::Vertical => RouteDirection::Vertical,
        Direction::DiagonalRising => RouteDirection::DiagonalRising,
        Direction::DiagonalFalling => RouteDirection::DiagonalFalling,
        Direction::Arbitrary => RouteDirection::Arbitrary,
        Direction::Via => unreachable!("via direction has no planar route direction"),
    }
}

fn edge_touches_region(start: &Point2, end: &Point2, boundary: &[Point2]) -> Option<bool> {
    if classify_point_ring_even_odd(boundary, start).value()? != RingPointLocation::Outside
        || classify_point_ring_even_odd(boundary, end).value()? != RingPointLocation::Outside
    {
        return Some(true);
    }
    for index in 0..boundary.len() {
        if classify_segment_intersection(
            start,
            end,
            &boundary[index],
            &boundary[(index + 1) % boundary.len()],
        )
        .value()?
        .intersects()
        {
            return Some(true);
        }
    }
    Some(false)
}

fn segment_touches_manhattan_ball(
    point: &Point2,
    start: &Point2,
    end: &Point2,
    radius: &Real,
) -> Option<bool> {
    let dx = end.x.clone() - start.x.clone();
    let dy = end.y.clone() - start.y.clone();
    let mut parameters = vec![Real::zero(), Real::one()];
    for (origin, coordinate, delta) in [(&start.x, &point.x, &dx), (&start.y, &point.y, &dy)] {
        if delta.partial_cmp(&Real::zero())? == Ordering::Equal {
            continue;
        }
        let parameter = ((coordinate.clone() - origin.clone()) / delta.clone()).ok()?;
        if matches!(
            parameter.partial_cmp(&Real::zero()),
            Some(Ordering::Equal | Ordering::Greater)
        ) && matches!(
            parameter.partial_cmp(&Real::one()),
            Some(Ordering::Equal | Ordering::Less)
        ) {
            parameters.push(parameter);
        }
    }
    parameters
        .into_iter()
        .try_fold(false, |touches, parameter| {
            if touches {
                return Some(true);
            }
            let candidate = Point2::new(
                start.x.clone() + dx.clone() * parameter.clone(),
                start.y.clone() + dy.clone() * parameter,
            );
            let distance = real_abs(&(candidate.x - point.x.clone()))
                + real_abs(&(candidate.y - point.y.clone()));
            Some(distance.partial_cmp(radius)? != Ordering::Greater)
        })
}

fn neighbors(
    grid: &Grid,
    node: Node,
    topology: NegotiatedPlanarTopology,
) -> Vec<(Node, Direction)> {
    let mut result = Vec::with_capacity(10);
    if let NegotiatedPlanarTopology::AnyAngle {
        maximum_neighbors_per_node,
    } = topology
    {
        result.extend(visibility_neighbors(grid, node, maximum_neighbors_per_node));
    } else {
        for (axis, forward, direction) in [
            (Axis::X, false, Direction::Horizontal),
            (Axis::X, true, Direction::Horizontal),
            (Axis::Y, false, Direction::Vertical),
            (Axis::Y, true, Direction::Vertical),
        ] {
            if let Some(next) = active_axis_neighbor(grid, node, axis, forward) {
                result.push((next, direction));
            }
        }
        if topology == NegotiatedPlanarTopology::Octilinear {
            for (x_forward, y_forward, direction) in [
                (false, false, Direction::DiagonalRising),
                (true, true, Direction::DiagonalRising),
                (false, true, Direction::DiagonalFalling),
                (true, false, Direction::DiagonalFalling),
            ] {
                if let Some(next) = active_diagonal_neighbor(grid, node, x_forward, y_forward) {
                    result.push((next, direction));
                }
            }
        }
    }
    if node.layer > 0 {
        result.push((
            Node {
                layer: node.layer - 1,
                ..node
            },
            Direction::Via,
        ));
    }
    if node.layer + 1 < grid.layers.len() {
        result.push((
            Node {
                layer: node.layer + 1,
                ..node
            },
            Direction::Via,
        ));
    }
    result
}

fn pair_neighbors(
    grid: &Grid,
    nodes: PairNodes,
    translation: &Point2,
    topology: NegotiatedPlanarTopology,
) -> Vec<(PairNodes, Direction)> {
    let mut result = Vec::with_capacity(10);
    if let NegotiatedPlanarTopology::AnyAngle {
        maximum_neighbors_per_node,
    } = topology
    {
        result.extend(pair_visibility_neighbors(
            grid,
            nodes,
            translation,
            maximum_neighbors_per_node,
        ));
    } else {
        for (axis, forward, direction) in [
            (Axis::X, false, Direction::Horizontal),
            (Axis::X, true, Direction::Horizontal),
            (Axis::Y, false, Direction::Vertical),
            (Axis::Y, true, Direction::Vertical),
        ] {
            if let Some(positive) = active_axis_neighbor(grid, nodes.positive, axis, forward)
                && let Some(negative) = translated_node(grid, positive, translation)
            {
                result.push((PairNodes { positive, negative }, direction));
            }
        }
        if topology == NegotiatedPlanarTopology::Octilinear {
            for (x_forward, y_forward, direction) in [
                (false, false, Direction::DiagonalRising),
                (true, true, Direction::DiagonalRising),
                (false, true, Direction::DiagonalFalling),
                (true, false, Direction::DiagonalFalling),
            ] {
                if let Some(positive) =
                    active_diagonal_neighbor(grid, nodes.positive, x_forward, y_forward)
                    && let Some(negative) = translated_node(grid, positive, translation)
                {
                    result.push((PairNodes { positive, negative }, direction));
                }
            }
        }
    }
    if nodes.positive.layer > 0 && nodes.negative.layer > 0 {
        result.push((
            PairNodes {
                positive: Node {
                    layer: nodes.positive.layer - 1,
                    ..nodes.positive
                },
                negative: Node {
                    layer: nodes.negative.layer - 1,
                    ..nodes.negative
                },
            },
            Direction::Via,
        ));
    }
    if nodes.positive.layer + 1 < grid.layers.len() && nodes.negative.layer + 1 < grid.layers.len()
    {
        result.push((
            PairNodes {
                positive: Node {
                    layer: nodes.positive.layer + 1,
                    ..nodes.positive
                },
                negative: Node {
                    layer: nodes.negative.layer + 1,
                    ..nodes.negative
                },
            },
            Direction::Via,
        ));
    }
    result
}

fn visibility_neighbors(grid: &Grid, node: Node, cap: usize) -> Vec<(Node, Direction)> {
    let mut candidates = Vec::<(Real, Node, Direction)>::new();
    for x in 0..grid.xs.len() {
        for y in 0..grid.ys.len() {
            let candidate = Node { x, y, ..node };
            if candidate == node || !grid.is_active(candidate) {
                continue;
            }
            let dx = grid.xs[x].clone() - grid.xs[node.x].clone();
            let dy = grid.ys[y].clone() - grid.ys[node.y].clone();
            let distance_squared = dx.clone() * dx + dy.clone() * dy;
            candidates.push((
                distance_squared,
                candidate,
                direction_between(grid, node, candidate),
            ));
        }
    }
    candidates.sort_by(|first, second| {
        first
            .0
            .partial_cmp(&second.0)
            .unwrap_or(Ordering::Equal)
            .then_with(|| first.1.cmp(&second.1))
    });
    candidates.truncate(cap);
    candidates
        .into_iter()
        .map(|(_, node, direction)| (node, direction))
        .collect()
}

fn pair_visibility_neighbors(
    grid: &Grid,
    nodes: PairNodes,
    translation: &Point2,
    cap: usize,
) -> Vec<(PairNodes, Direction)> {
    let mut candidates = Vec::<(Real, PairNodes, Direction)>::new();
    for x in 0..grid.xs.len() {
        for y in 0..grid.ys.len() {
            let positive = Node {
                x,
                y,
                ..nodes.positive
            };
            if positive == nodes.positive || !grid.is_active(positive) {
                continue;
            }
            let Some(negative) = translated_node(grid, positive, translation) else {
                continue;
            };
            let dx = grid.xs[x].clone() - grid.xs[nodes.positive.x].clone();
            let dy = grid.ys[y].clone() - grid.ys[nodes.positive.y].clone();
            let distance_squared = dx.clone() * dx + dy.clone() * dy;
            candidates.push((
                distance_squared,
                PairNodes { positive, negative },
                direction_between(grid, nodes.positive, positive),
            ));
        }
    }
    candidates.sort_by(|first, second| {
        first
            .0
            .partial_cmp(&second.0)
            .unwrap_or(Ordering::Equal)
            .then_with(|| first.1.cmp(&second.1))
    });
    candidates.truncate(cap);
    candidates
        .into_iter()
        .map(|(_, nodes, direction)| (nodes, direction))
        .collect()
}

fn active_axis_neighbor(grid: &Grid, node: Node, axis: Axis, forward: bool) -> Option<Node> {
    let length = match axis {
        Axis::X => grid.xs.len(),
        Axis::Y => grid.ys.len(),
    };
    let mut index = match axis {
        Axis::X => node.x,
        Axis::Y => node.y,
    };
    loop {
        index = if forward {
            index.checked_add(1).filter(|index| *index < length)?
        } else {
            index.checked_sub(1)?
        };
        let candidate = match axis {
            Axis::X => Node { x: index, ..node },
            Axis::Y => Node { y: index, ..node },
        };
        if grid.is_active(candidate) {
            return Some(candidate);
        }
    }
}

fn active_diagonal_neighbor(
    grid: &Grid,
    node: Node,
    x_forward: bool,
    y_forward: bool,
) -> Option<Node> {
    let x_indices = directional_indices(node.x, grid.xs.len(), x_forward);
    let y_indices = directional_indices(node.y, grid.ys.len(), y_forward);
    let mut best = None::<(Real, Node)>;
    for x in x_indices {
        let dx = real_abs(&(grid.xs[x].clone() - grid.xs[node.x].clone()));
        for y in &y_indices {
            let dy = real_abs(&(grid.ys[*y].clone() - grid.ys[node.y].clone()));
            if dx != dy {
                continue;
            }
            let candidate = Node { x, y: *y, ..node };
            if !grid.is_active(candidate) {
                continue;
            }
            if best
                .as_ref()
                .is_none_or(|(span, _)| dx.partial_cmp(span) == Some(Ordering::Less))
            {
                best = Some((dx.clone(), candidate));
            }
        }
    }
    best.map(|(_, node)| node)
}

fn directional_indices(start: usize, length: usize, forward: bool) -> Vec<usize> {
    if forward {
        (start.saturating_add(1)..length).collect()
    } else {
        (0..start).rev().collect()
    }
}

fn translated_node(grid: &Grid, positive: Node, translation: &Point2) -> Option<Node> {
    let point = grid.point(positive);
    let translated_x = point.x + translation.x.clone();
    let translated_y = point.y + translation.y.clone();
    let node = Node {
        x: grid.xs.iter().position(|value| value == &translated_x)?,
        y: grid.ys.iter().position(|value| value == &translated_y)?,
        layer: positive.layer,
    };
    grid.is_active(node).then_some(node)
}

fn resource(first: Node, second: Node) -> Resource {
    let (first, second) = if first <= second {
        (first, second)
    } else {
        (second, first)
    };
    if first.layer != second.layer {
        Resource::Via(first, second)
    } else {
        Resource::Edge(first, second)
    }
}

fn diagonal_cell(first: Node, second: Node) -> Resource {
    let lower_left = Node {
        x: first.x.min(second.x),
        y: first.y.min(second.y),
        layer: first.layer,
    };
    let upper_right = Node {
        x: first.x.max(second.x),
        y: first.y.max(second.y),
        layer: first.layer,
    };
    Resource::DiagonalCell(lower_left, upper_right)
}

fn diagonal_breakpoints(grid: &Grid, first: Node, second: Node) -> Vec<Node> {
    let (left, right) = if first.x <= second.x {
        (first, second)
    } else {
        (second, first)
    };
    let rising = left.y < right.y;
    let mut points = Vec::new();
    for x in left.x..=right.x {
        let offset = grid.xs[x].clone() - grid.xs[left.x].clone();
        let y_coordinate = if rising {
            grid.ys[left.y].clone() + offset
        } else {
            grid.ys[left.y].clone() - offset
        };
        let Some(y) = grid
            .ys
            .iter()
            .position(|coordinate| coordinate == &y_coordinate)
        else {
            continue;
        };
        let node = Node {
            x,
            y,
            layer: first.layer,
        };
        if y >= first.y.min(second.y) && y <= first.y.max(second.y) {
            points.push(node);
        }
    }
    points
}

fn transition_resources(grid: &Grid, first: Node, second: Node) -> Vec<Resource> {
    let mut resources = vec![resource(first, second)];
    let exact_diagonal = first.layer == second.layer
        && first.x != second.x
        && first.y != second.y
        && real_abs(&(grid.xs[first.x].clone() - grid.xs[second.x].clone()))
            == real_abs(&(grid.ys[first.y].clone() - grid.ys[second.y].clone()));
    if exact_diagonal {
        let points = diagonal_breakpoints(grid, first, second);
        resources.extend(
            points
                .iter()
                .skip(1)
                .take(points.len().saturating_sub(2))
                .copied()
                .map(Resource::Node),
        );
        resources.extend(
            points
                .windows(2)
                .map(|pair| diagonal_cell(pair[0], pair[1])),
        );
    }
    resources
}

fn heuristic(
    node: Node,
    targets: &[Node],
    grid: &Grid,
    policy: &NegotiatedRoutePolicy,
) -> Option<u64> {
    targets
        .iter()
        .map(|target| {
            Some(grid.euclidean_units(node, *target, policy)?.saturating_add(
                (node.layer.abs_diff(target.layer) as u64).saturating_mul(policy.via_penalty),
            ))
        })
        .collect::<Option<Vec<_>>>()?
        .into_iter()
        .min()
}

fn pair_heuristic(
    nodes: PairNodes,
    targets: &[PairNodes],
    grid: &Grid,
    policy: &NegotiatedRoutePolicy,
) -> Option<u64> {
    targets
        .iter()
        .map(|target| {
            Some(
                grid.euclidean_units(nodes.positive, target.positive, policy)?
                    .saturating_add(grid.euclidean_units(
                        nodes.negative,
                        target.negative,
                        policy,
                    )?)
                    .saturating_add(
                        (nodes.positive.layer.abs_diff(target.positive.layer) as u64)
                            .saturating_mul(policy.via_penalty)
                            .saturating_mul(2),
                    ),
            )
        })
        .collect::<Option<Vec<_>>>()?
        .into_iter()
        .min()
}

fn point_is_legal(
    layout: &PcbLayout,
    boundary: &BoardBoundaryGeometry,
    point: &Point2,
    radius: &Real,
    layer: Option<TraceLayer>,
) -> Option<bool> {
    match boundary
        .contains_disc(point, radius.clone(), &CurvePolicy::certified())
        .ok()?
    {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Some(false),
        Classification::Uncertain(_) => return None,
    }
    for keepout in &layout.keepouts {
        let applies = match keepout.scope {
            KeepoutScope::All => true,
            KeepoutScope::Vias => layer.is_none(),
            KeepoutScope::Copper(ref layers) => layer.is_some_and(|layer| layers.contains(&layer)),
            KeepoutScope::Components => false,
        };
        if applies && !circle_disjoint_from_polygon(&keepout.boundary, point, radius)? {
            return Some(false);
        }
    }
    Some(true)
}

fn segment_is_legal(
    layout: &PcbLayout,
    boundary: &BoardBoundaryGeometry,
    start: &Point2,
    end: &Point2,
    radius: &Real,
    layer: TraceLayer,
) -> Option<bool> {
    match boundary
        .contains_segment(start, end, radius.clone(), &CurvePolicy::certified())
        .ok()?
    {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Some(false),
        Classification::Uncertain(_) => return None,
    }
    for keepout in &layout.keepouts {
        let applies = matches!(keepout.scope, KeepoutScope::All)
            || matches!(&keepout.scope, KeepoutScope::Copper(layers) if layers.contains(&layer));
        if applies && !segment_clear_of_ring(start, end, &keepout.boundary, radius)? {
            return Some(false);
        }
    }
    Some(true)
}

fn point_clear_of_fixed(
    point: &Point2,
    subject: ClearanceSubject<'_>,
    layers: &[TraceLayer],
    fixed: &FixedObstacles,
) -> Option<bool> {
    for line in &fixed.lines {
        if &line.net == subject.net
            || subject.paired_net.is_some_and(|paired| paired == &line.net)
            || !layers.contains(&line.layer)
        {
            continue;
        }
        let pair_clearance = exact_max(subject.clearance, &line.clearance)?;
        let required = subject.physical_radius.clone() + line.radius.clone() + pair_clearance;
        if point_segment_distance_squared(point, &line.start, &line.end)?
            .partial_cmp(&(required.clone() * required))?
            == Ordering::Less
        {
            return Some(false);
        }
    }
    for disc in &fixed.discs {
        if &disc.net == subject.net
            || subject.paired_net.is_some_and(|paired| paired == &disc.net)
            || !disc.layers.iter().any(|layer| layers.contains(layer))
        {
            continue;
        }
        let pair_clearance = exact_max(subject.clearance, &disc.clearance)?;
        let required = subject.physical_radius.clone() + disc.radius.clone() + pair_clearance;
        if point_distance_squared(point, &disc.center)
            .partial_cmp(&(required.clone() * required))?
            == Ordering::Less
        {
            return Some(false);
        }
    }
    for pad in &fixed.pads {
        if pad.net.as_ref() == Some(subject.net)
            || subject
                .paired_net
                .is_some_and(|paired| pad.net.as_ref() == Some(paired))
            || !pad.layers.iter().any(|layer| layers.contains(layer))
        {
            continue;
        }
        let pair_clearance = exact_max(subject.clearance, &pad.clearance)?;
        let required = subject.physical_radius.clone() + pair_clearance;
        if !point_clear_of_pad(point, pad, &required)? {
            return Some(false);
        }
    }
    Some(true)
}

fn segment_clear_of_fixed(
    start: &Point2,
    end: &Point2,
    subject: ClearanceSubject<'_>,
    layer: TraceLayer,
    fixed: &FixedObstacles,
) -> Option<bool> {
    for line in &fixed.lines {
        if &line.net == subject.net
            || subject.paired_net.is_some_and(|paired| paired == &line.net)
            || line.layer != layer
        {
            continue;
        }
        let pair_clearance = exact_max(subject.clearance, &line.clearance)?;
        let required = subject.physical_radius.clone() + line.radius.clone() + pair_clearance;
        if segment_segment_distance_squared(start, end, &line.start, &line.end)?
            .partial_cmp(&(required.clone() * required))?
            == Ordering::Less
        {
            return Some(false);
        }
    }
    for disc in &fixed.discs {
        if &disc.net == subject.net
            || subject.paired_net.is_some_and(|paired| paired == &disc.net)
            || !disc.layers.contains(&layer)
        {
            continue;
        }
        let pair_clearance = exact_max(subject.clearance, &disc.clearance)?;
        let required = subject.physical_radius.clone() + disc.radius.clone() + pair_clearance;
        if point_segment_distance_squared(&disc.center, start, end)?
            .partial_cmp(&(required.clone() * required))?
            == Ordering::Less
        {
            return Some(false);
        }
    }
    for pad in &fixed.pads {
        if pad.net.as_ref() == Some(subject.net)
            || subject
                .paired_net
                .is_some_and(|paired| pad.net.as_ref() == Some(paired))
            || !pad.layers.contains(&layer)
        {
            continue;
        }
        let pair_clearance = exact_max(subject.clearance, &pad.clearance)?;
        let required = subject.physical_radius.clone() + pair_clearance;
        if !segment_clear_of_pad(start, end, pad, &required)? {
            return Some(false);
        }
    }
    Some(true)
}

fn point_clear_of_pad(point: &Point2, pad: &FixedPad, radius: &Real) -> Option<bool> {
    let point = point_in_pad_coordinates(point, pad);
    match &pad.shape {
        PadShape::Circle { diameter } => {
            let pad_radius = (diameter.clone() / Real::from(2)).ok()?;
            let required = radius.clone() + pad_radius;
            Some(
                point_distance_squared(&point, &Point2::new(Real::zero(), Real::zero()))
                    .partial_cmp(&(required.clone() * required))?
                    != Ordering::Less,
            )
        }
        PadShape::Rectangle { width, height } => {
            point_clear_of_axis_rectangle(&point, &half_real(width)?, &half_real(height)?, radius)
        }
        PadShape::RoundedRectangle {
            width,
            height,
            corner_radius,
        } => {
            let half_width = half_real(width)? - corner_radius.clone();
            let half_height = half_real(height)? - corner_radius.clone();
            let required = radius.clone() + corner_radius.clone();
            point_clear_of_axis_rectangle(&point, &half_width, &half_height, &required)
        }
        PadShape::Obround { width, height } => {
            let min_dimension = exact_min(width, height)?;
            let corner_radius = half_real(&min_dimension)?;
            let half_width = half_real(width)? - corner_radius.clone();
            let half_height = half_real(height)? - corner_radius.clone();
            let required = radius.clone() + corner_radius;
            point_clear_of_axis_rectangle(&point, &half_width, &half_height, &required)
        }
        PadShape::Polygon { vertices } => circle_disjoint_from_polygon(vertices, &point, radius),
    }
}

fn segment_clear_of_pad(
    start: &Point2,
    end: &Point2,
    pad: &FixedPad,
    radius: &Real,
) -> Option<bool> {
    segment_clear_of_placed_pad(
        start,
        end,
        &pad.placement,
        &pad.center,
        &pad.rotation_degrees,
        &pad.shape,
        radius,
    )
}

pub(crate) fn segment_clear_of_placed_pad(
    start: &Point2,
    end: &Point2,
    placement: &PcbPlacement,
    pad_center: &Point2,
    pad_rotation_degrees: &Real,
    shape: &PadShape,
    radius: &Real,
) -> Option<bool> {
    let start = point_in_placed_pad_coordinates(start, placement, pad_center, pad_rotation_degrees);
    let end = point_in_placed_pad_coordinates(end, placement, pad_center, pad_rotation_degrees);
    match shape {
        PadShape::Circle { diameter } => {
            let pad_radius = (diameter.clone() / Real::from(2)).ok()?;
            let required = radius.clone() + pad_radius;
            Some(
                point_segment_distance_squared(
                    &Point2::new(Real::zero(), Real::zero()),
                    &start,
                    &end,
                )?
                .partial_cmp(&(required.clone() * required))?
                    != Ordering::Less,
            )
        }
        PadShape::Rectangle { width, height } => segment_clear_of_axis_rectangle(
            &start,
            &end,
            &half_real(width)?,
            &half_real(height)?,
            radius,
        ),
        PadShape::RoundedRectangle {
            width,
            height,
            corner_radius,
        } => {
            let half_width = half_real(width)? - corner_radius.clone();
            let half_height = half_real(height)? - corner_radius.clone();
            let required = radius.clone() + corner_radius.clone();
            segment_clear_of_axis_rectangle(&start, &end, &half_width, &half_height, &required)
        }
        PadShape::Obround { width, height } => {
            let min_dimension = exact_min(width, height)?;
            let corner_radius = half_real(&min_dimension)?;
            let half_width = half_real(width)? - corner_radius.clone();
            let half_height = half_real(height)? - corner_radius.clone();
            let required = radius.clone() + corner_radius;
            segment_clear_of_axis_rectangle(&start, &end, &half_width, &half_height, &required)
        }
        PadShape::Polygon { vertices } => {
            capsule_disjoint_from_polygon(&start, &end, vertices, radius)
        }
    }
}

fn point_in_pad_coordinates(point: &Point2, pad: &FixedPad) -> Point2 {
    point_in_placed_pad_coordinates(point, &pad.placement, &pad.center, &pad.rotation_degrees)
}

fn point_in_placed_pad_coordinates(
    point: &Point2,
    placement: &PcbPlacement,
    pad_center: &Point2,
    pad_rotation_degrees: &Real,
) -> Point2 {
    let point = placement.inverse_transform_point(point);
    let x = point.x - pad_center.x.clone();
    let y = point.y - pad_center.y.clone();
    let pad_radians = pad_rotation_degrees.clone().to_radians();
    let pad_sin = pad_radians.clone().sin();
    let pad_cos = pad_radians.cos();
    Point2::new(
        x.clone() * pad_cos.clone() + y.clone() * pad_sin.clone(),
        -x * pad_sin + y * pad_cos,
    )
}

fn half_real(value: &Real) -> Option<Real> {
    (value.clone() / Real::from(2)).ok()
}

fn point_clear_of_axis_rectangle(
    point: &Point2,
    half_width: &Real,
    half_height: &Real,
    radius: &Real,
) -> Option<bool> {
    let dx = positive_part(&(real_abs(&point.x) - half_width.clone()))?;
    let dy = positive_part(&(real_abs(&point.y) - half_height.clone()))?;
    let distance_squared = dx.clone() * dx + dy.clone() * dy;
    Some(distance_squared.partial_cmp(&(radius.clone() * radius.clone()))? != Ordering::Less)
}

fn positive_part(value: &Real) -> Option<Real> {
    match value.partial_cmp(&Real::zero())? {
        Ordering::Greater => Some(value.clone()),
        Ordering::Equal | Ordering::Less => Some(Real::zero()),
    }
}

fn segment_clear_of_axis_rectangle(
    start: &Point2,
    end: &Point2,
    half_width: &Real,
    half_height: &Real,
    radius: &Real,
) -> Option<bool> {
    let width_positive = half_width.partial_cmp(&Real::zero())? == Ordering::Greater;
    let height_positive = half_height.partial_cmp(&Real::zero())? == Ordering::Greater;
    match (width_positive, height_positive) {
        (true, true) => capsule_disjoint_from_polygon(
            start,
            end,
            &[
                Point2::new(-half_width.clone(), -half_height.clone()),
                Point2::new(half_width.clone(), -half_height.clone()),
                Point2::new(half_width.clone(), half_height.clone()),
                Point2::new(-half_width.clone(), half_height.clone()),
            ],
            radius,
        ),
        (true, false) => {
            let required = radius.clone() * radius.clone();
            Some(
                segment_segment_distance_squared(
                    start,
                    end,
                    &Point2::new(-half_width.clone(), Real::zero()),
                    &Point2::new(half_width.clone(), Real::zero()),
                )?
                .partial_cmp(&required)?
                    != Ordering::Less,
            )
        }
        (false, true) => {
            let required = radius.clone() * radius.clone();
            Some(
                segment_segment_distance_squared(
                    start,
                    end,
                    &Point2::new(Real::zero(), -half_height.clone()),
                    &Point2::new(Real::zero(), half_height.clone()),
                )?
                .partial_cmp(&required)?
                    != Ordering::Less,
            )
        }
        (false, false) => {
            let required = radius.clone() * radius.clone();
            Some(
                point_segment_distance_squared(
                    &Point2::new(Real::zero(), Real::zero()),
                    start,
                    end,
                )?
                .partial_cmp(&required)?
                    != Ordering::Less,
            )
        }
    }
}

fn capsule_disjoint_from_polygon(
    start: &Point2,
    end: &Point2,
    points: &[Point2],
    radius: &Real,
) -> Option<bool> {
    if points.len() < 3
        || classify_point_ring_even_odd(points, start).value()? != RingPointLocation::Outside
        || classify_point_ring_even_odd(points, end).value()? != RingPointLocation::Outside
    {
        return Some(false);
    }
    segment_clear_of_ring(start, end, points, radius)
}

fn segment_clear_of_ring(
    start: &Point2,
    end: &Point2,
    ring: &[Point2],
    radius: &Real,
) -> Option<bool> {
    let required = radius.clone() * radius.clone();
    for index in 0..ring.len() {
        let a = &ring[index];
        let b = &ring[(index + 1) % ring.len()];
        if classify_segment_intersection(start, end, a, b).value()? != SegmentIntersection::Disjoint
        {
            return Some(false);
        }
        if segment_segment_distance_squared(start, end, a, b)?.partial_cmp(&required)?
            == Ordering::Less
        {
            return Some(false);
        }
    }
    Some(true)
}

fn point_distance_squared(first: &Point2, second: &Point2) -> Real {
    let dx = first.x.clone() - second.x.clone();
    let dy = first.y.clone() - second.y.clone();
    dx.clone() * dx + dy.clone() * dy
}

pub(crate) fn segment_segment_distance_squared(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
) -> Option<Real> {
    if classify_segment_intersection(a, b, c, d).value()? != SegmentIntersection::Disjoint {
        return Some(Real::zero());
    }
    [
        point_segment_distance_squared(a, c, d)?,
        point_segment_distance_squared(b, c, d)?,
        point_segment_distance_squared(c, a, b)?,
        point_segment_distance_squared(d, a, b)?,
    ]
    .into_iter()
    .try_fold(None, |best: Option<Real>, value| match best {
        Some(best) => Some(Some(if value.partial_cmp(&best)? == Ordering::Less {
            value
        } else {
            best
        })),
        None => Some(Some(value)),
    })?
}

pub(crate) fn point_segment_distance_squared(
    point: &Point2,
    start: &Point2,
    end: &Point2,
) -> Option<Real> {
    let dx = end.x.clone() - start.x.clone();
    let dy = end.y.clone() - start.y.clone();
    let length_squared = dx.clone() * dx.clone() + dy.clone() * dy.clone();
    if length_squared.partial_cmp(&Real::zero())? != Ordering::Greater {
        return None;
    }
    let px = point.x.clone() - start.x.clone();
    let py = point.y.clone() - start.y.clone();
    let projection = px.clone() * dx.clone() + py.clone() * dy.clone();
    if projection.partial_cmp(&Real::zero())? != Ordering::Greater {
        return Some(px.clone() * px + py.clone() * py);
    }
    if projection.partial_cmp(&length_squared)? != Ordering::Less {
        let ex = point.x.clone() - end.x.clone();
        let ey = point.y.clone() - end.y.clone();
        return Some(ex.clone() * ex + ey.clone() * ey);
    }
    let cross = px * dy - py * dx;
    (cross.clone() * cross / length_squared).ok()
}

fn circle_disjoint_from_polygon(points: &[Point2], center: &Point2, radius: &Real) -> Option<bool> {
    if classify_point_ring_even_odd(points, center).value()? != RingPointLocation::Outside {
        return Some(false);
    }
    polygon_edges_clear(points, center, radius)
}

fn polygon_edges_clear(points: &[Point2], center: &Point2, radius: &Real) -> Option<bool> {
    let required = radius.clone() * radius.clone();
    for index in 0..points.len() {
        if point_segment_distance_squared(
            center,
            &points[index],
            &points[(index + 1) % points.len()],
        )?
        .partial_cmp(&required)?
            == Ordering::Less
        {
            return Some(false);
        }
    }
    Some(true)
}

fn net_rule(
    handoff: &RoutingProblemReport,
    net: &NetId,
    policy: &NegotiatedRoutePolicy,
) -> (Real, Real) {
    let Some(alias) = handoff.problem.aliases.get(net) else {
        return (
            policy.default_trace_width.clone(),
            policy.default_clearance.clone(),
        );
    };
    handoff
        .problem
        .rules
        .iter()
        .find(|rule| rule.net == Some(alias))
        .map_or_else(
            || {
                (
                    policy.default_trace_width.clone(),
                    policy.default_clearance.clone(),
                )
            },
            |rule| (rule.width.clone(), rule.clearance.clone()),
        )
}

fn maximum_net_rule(
    layout: &PcbLayout,
    handoff: &RoutingProblemReport,
    net: &NetId,
    policy: &NegotiatedRoutePolicy,
) -> Result<(Real, Real), NegotiatedRouterError> {
    let (mut width, mut clearance) = net_rule(handoff, net, policy);
    for region in &layout.rules.route_rule_regions {
        if !constraint_selects_net(&region.nets, net) {
            continue;
        }
        if let Some(regional) = &region.min_trace_width {
            width = exact_max(&width, regional).ok_or(NegotiatedRouterError::InvalidPolicy)?;
        }
        if let Some(regional) = &region.min_clearance {
            clearance =
                exact_max(&clearance, regional).ok_or(NegotiatedRouterError::InvalidPolicy)?;
        }
    }
    Ok((width, clearance))
}

fn edge_net_rule(
    layout: &PcbLayout,
    handoff: &RoutingProblemReport,
    net: &NetId,
    policy: &NegotiatedRoutePolicy,
    start: &Point2,
    end: &Point2,
) -> Option<(Real, Real)> {
    let (mut width, mut clearance) = net_rule(handoff, net, policy);
    for region in &layout.rules.route_rule_regions {
        if !constraint_selects_net(&region.nets, net)
            || !edge_touches_region(start, end, &region.boundary)?
        {
            continue;
        }
        if let Some(regional) = &region.min_trace_width {
            width = exact_max(&width, regional)?;
        }
        if let Some(regional) = &region.min_clearance {
            clearance = exact_max(&clearance, regional)?;
        }
    }
    Some((width, clearance))
}

fn resolved_via_style(
    layout: &PcbLayout,
    net: &NetId,
    policy: &NegotiatedRoutePolicy,
) -> ResolvedViaStyle {
    let resolved = layout
        .rules
        .resolve_net_classes()
        .expect("validated layout has resolvable net classes");
    if let Some(class) = resolved.iter().find(|class| class.nets.contains(net)) {
        if let Some(style_id) = &class.preferred_via_style
            && let Some(style) = layout
                .rules
                .via_styles
                .iter()
                .find(|style| style.id == *style_id)
        {
            return ResolvedViaStyle {
                id: Some(style.id.clone()),
                land_diameter: style.land_diameter.clone(),
                drill_diameter: style.drill_diameter.clone(),
                plating: style.plating,
                mask: style.mask.clone(),
                allowed_spans: style.allowed_spans.clone(),
            };
        }
        if let (Some(land_diameter), Some(drill_diameter)) = (
            &class.preferred_via_land_diameter,
            &class.preferred_via_drill_diameter,
        ) {
            return ResolvedViaStyle {
                id: None,
                land_diameter: land_diameter.clone(),
                drill_diameter: drill_diameter.clone(),
                plating: Plating::Plated,
                mask: policy.via_mask.clone(),
                allowed_spans: Vec::new(),
            };
        }
    }
    ResolvedViaStyle {
        id: None,
        land_diameter: policy.via_land_diameter.clone(),
        drill_diameter: policy.via_drill_diameter.clone(),
        plating: Plating::Plated,
        mask: policy.via_mask.clone(),
        allowed_spans: Vec::new(),
    }
}

fn via_drill_intent(plating: Plating) -> ViaDrillIntent {
    match plating {
        Plating::Plated => ViaDrillIntent::Plated,
        Plating::NonPlated => ViaDrillIntent::NonPlated,
        Plating::Unspecified => ViaDrillIntent::Unspecified,
    }
}

fn lower_routes(
    layout: &PcbLayout,
    handoff: &RoutingProblemReport,
    grid: &Grid,
    routes: &BTreeMap<NetId, NetRoute>,
    policy: &NegotiatedRoutePolicy,
) -> Result<SpecctraRoute, NegotiatedRouterError> {
    let mut traces = Vec::new();
    let mut vias = Vec::new();
    let mut emitted_vias = BTreeSet::new();
    let emission = PlanarEmissionContext {
        layout,
        handoff,
        grid,
        policy,
    };
    for (net, route) in routes {
        let via_style = resolved_via_style(layout, net, policy);
        let alias = handoff
            .problem
            .aliases
            .get(net)
            .ok_or_else(|| NegotiatedRouterError::UnroutableSelectedNet(net.clone()))?;
        for path in &route.paths {
            let mut planar_start = 0;
            for index in 1..path.len() {
                if path[index - 1].layer != path[index].layer {
                    emit_planar_spans(
                        &mut traces,
                        &emission,
                        net,
                        alias,
                        &path[planar_start..index],
                        &route.width_overrides,
                    )?;
                    let first_layer = grid.layers[path[index - 1].layer];
                    let second_layer = grid.layers[path[index].layer];
                    let (start_layer, end_layer) = if first_layer <= second_layer {
                        (first_layer, second_layer)
                    } else {
                        (second_layer, first_layer)
                    };
                    let (first, second) = if path[index - 1] <= path[index] {
                        (path[index - 1], path[index])
                    } else {
                        (path[index], path[index - 1])
                    };
                    if emitted_vias.insert((alias, first, second)) {
                        vias.push(
                            PcbViaStack::with_drill_intent(
                                alias,
                                start_layer,
                                end_layer,
                                grid.point(path[index]),
                                via_style.land_diameter.clone(),
                                via_style.drill_diameter.clone(),
                                via_drill_intent(via_style.plating),
                            )
                            .map_err(|_| NegotiatedRouterError::InvalidPolicy)?,
                        );
                    }
                    planar_start = index;
                }
            }
            emit_planar_spans(
                &mut traces,
                &emission,
                net,
                alias,
                &path[planar_start..],
                &route.width_overrides,
            )?;
        }
    }
    Ok(SpecctraRoute::with_vias(traces, vias))
}

struct PlanarEmissionContext<'a> {
    layout: &'a PcbLayout,
    handoff: &'a RoutingProblemReport,
    grid: &'a Grid,
    policy: &'a NegotiatedRoutePolicy,
}

fn emit_planar_spans(
    output: &mut Vec<PcbTrace>,
    context: &PlanarEmissionContext<'_>,
    logical_net: &NetId,
    routing_net: RoutingNetId,
    nodes: &[Node],
    width_overrides: &BTreeMap<Resource, Real>,
) -> Result<(), NegotiatedRouterError> {
    if nodes.len() < 2 {
        return Ok(());
    }
    let mut start = 0;
    let mut width = planar_edge_width(context, logical_net, nodes[0], nodes[1], width_overrides)?;
    let mut direction = direction_between(context.grid, nodes[0], nodes[1]);
    for index in 2..=nodes.len() {
        let (next_width, next_direction) = if index == nodes.len() {
            (None, None)
        } else {
            let next_width = planar_edge_width(
                context,
                logical_net,
                nodes[index - 1],
                nodes[index],
                width_overrides,
            )?;
            (
                Some(next_width),
                Some(direction_between(
                    context.grid,
                    nodes[index - 1],
                    nodes[index],
                )),
            )
        };
        let continues_straight = index < nodes.len()
            && next_direction == Some(direction)
            && nodes_continue_straight(
                context.grid,
                nodes[index - 2],
                nodes[index - 1],
                nodes[index],
            );
        if next_width.as_ref() != Some(&width) || !continues_straight {
            let segment = LinePathSegment::new(
                context.grid.point(nodes[start]),
                context.grid.point(nodes[index - 1]),
            );
            let swept = SweptLineSegment::new(segment, width.clone())
                .map_err(|_| NegotiatedRouterError::InvalidPolicy)?;
            output.push(PcbTrace::new(
                routing_net,
                context.grid.layers[nodes[start].layer],
                swept,
            ));
            start = index - 1;
            if let (Some(next_width), Some(next_direction)) = (next_width, next_direction) {
                width = next_width;
                direction = next_direction;
            }
        }
    }
    Ok(())
}

fn nodes_continue_straight(grid: &Grid, first: Node, middle: Node, last: Node) -> bool {
    let first_dx = grid.xs[middle.x].clone() - grid.xs[first.x].clone();
    let first_dy = grid.ys[middle.y].clone() - grid.ys[first.y].clone();
    let second_dx = grid.xs[last.x].clone() - grid.xs[middle.x].clone();
    let second_dy = grid.ys[last.y].clone() - grid.ys[middle.y].clone();
    first_dx.clone() * second_dy.clone() - first_dy.clone() * second_dx.clone() == Real::zero()
        && first_dx * second_dx + first_dy * second_dy > Real::zero()
}

fn planar_edge_width(
    context: &PlanarEmissionContext<'_>,
    logical_net: &NetId,
    first: Node,
    second: Node,
    width_overrides: &BTreeMap<Resource, Real>,
) -> Result<Real, NegotiatedRouterError> {
    if let Some(width) = width_overrides.get(&resource(first, second)) {
        return Ok(width.clone());
    }
    let start = context.grid.point(first);
    let end = context.grid.point(second);
    edge_net_rule(
        context.layout,
        context.handoff,
        logical_net,
        context.policy,
        &start,
        &end,
    )
    .map(|(width, _)| width)
    .ok_or(NegotiatedRouterError::InvalidPolicy)
}

fn direction_between(grid: &Grid, first: Node, second: Node) -> Direction {
    if first.layer != second.layer {
        Direction::Via
    } else if first.x != second.x && first.y != second.y {
        let dx = grid.xs[second.x].clone() - grid.xs[first.x].clone();
        let dy = grid.ys[second.y].clone() - grid.ys[first.y].clone();
        if real_abs(&dx) != real_abs(&dy) {
            Direction::Arbitrary
        } else if (first.x < second.x) == (first.y < second.y) {
            Direction::DiagonalRising
        } else {
            Direction::DiagonalFalling
        }
    } else if first.x != second.x {
        Direction::Horizontal
    } else {
        Direction::Vertical
    }
}

fn stabilize_solution_ids(
    layout: &PcbLayout,
    solution: &mut RoutingSolution,
    policy: &NegotiatedRoutePolicy,
) {
    let mut route_counts = BTreeMap::<NetId, usize>::new();
    for route in &mut solution.routes {
        let count = route_counts.entry(route.net.clone()).or_default();
        route.id = RouteId::new(format!("negotiated-{}-{}", route.net.as_str(), *count))
            .expect("generated route id is nonempty");
        *count += 1;
    }
    let mut via_counts = BTreeMap::<NetId, usize>::new();
    for via in &mut solution.vias {
        let style = resolved_via_style(layout, &via.net, policy);
        let count = via_counts.entry(via.net.clone()).or_default();
        via.id = ViaId::new(format!("negotiated-{}-via-{}", via.net.as_str(), *count))
            .expect("generated via id is nonempty");
        via.land_diameter.clone_from(&style.land_diameter);
        via.drill_diameter.clone_from(&style.drill_diameter);
        via.plating = style.plating;
        via.mask = style.mask;
        *count += 1;
    }
}

fn via_style_evidence(
    layout: &PcbLayout,
    selected: &[NetId],
    solution: &RoutingSolution,
    policy: &NegotiatedRoutePolicy,
) -> Vec<NegotiatedViaStyleEvidence> {
    selected
        .iter()
        .map(|net| {
            let style = resolved_via_style(layout, net, policy);
            NegotiatedViaStyleEvidence {
                net: net.clone(),
                style: style.id,
                generated_vias: solution.vias.iter().filter(|via| via.net == *net).count(),
                land_diameter: style.land_diameter,
                drill_diameter: style.drill_diameter,
                plating: style.plating,
                mask: style.mask,
            }
        })
        .collect()
}
