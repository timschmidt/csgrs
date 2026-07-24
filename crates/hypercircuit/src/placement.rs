//! Deterministic placement proposals from retained PCB constraints.
//!
//! The solver uses conservative axis-aligned bounds around transformed
//! courtyard, body, or pad envelopes. It is intentionally a proposal layer:
//! `HyperDrcHandoff` remains the release authority for exact component
//! overlap and board-readiness certification when the `drc` feature is enabled.

use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
};

use hypercurve::{Classification, CurvePolicy};
use hyperlattice::Point2;
use hyperpath::TraceLayer;
use hyperreal::RealSign;

use crate::{
    BoardBoundaryGeometry, BoardSide, Circuit, CircuitInstanceId, LandPattern,
    LandPatternGraphicPrimitive, LayerRole, NetId, PadId, PadShape, PcbLayout, PcbPlacement,
    PinRef, PlacementConstraintKind, PlacementResolutionIssue, Real, RouteId,
};

/// Conservative source used to bound one package during placement search.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlacementEnvelopeSource {
    /// One or more retained courtyard graphics.
    Courtyard,
    /// The retained package-body outline.
    Body,
    /// Copper-pad extents used because no courtyard or body exists.
    Pads,
}

/// Exact physical probe used to score cardinal fanout access at every placed pad.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementPinAccessPolicy {
    /// Exact centerline length tested outward from every pad center.
    pub probe_distance: Real,
    /// Minimum trace width tested, combined with retained net/regional rules by exact maximum.
    pub minimum_trace_width: Real,
    /// Minimum clearance tested, combined with retained net/regional rules by exact maximum.
    pub minimum_clearance: Real,
}

/// One cardinal planar fanout direction.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum PlacementPinAccessDirection {
    /// Decreasing board X.
    NegativeX,
    /// Increasing board X.
    PositiveX,
    /// Decreasing board Y.
    NegativeY,
    /// Increasing board Y.
    PositiveY,
}

/// Exact-predicate result for one layer-local cardinal access probe.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlacementPinAccessStatus {
    /// The complete trace envelope is legal.
    Accessible,
    /// A retained physical or authored-policy constraint blocks the probe.
    Blocked,
    /// Exact geometry could not decide the probe.
    Indeterminate,
}

/// Source-addressable result for one layer-local cardinal access probe.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PlacementPinAccessProbeEvidence {
    /// Physical conductor layer tested.
    pub layer: TraceLayer,
    /// Cardinal direction tested.
    pub direction: PlacementPinAccessDirection,
    /// Exact-predicate outcome.
    pub status: PlacementPinAccessStatus,
}

/// Complete access evidence for one logical/physical placed terminal.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementPinAccessTerminalEvidence {
    /// Logical instance owning the terminal.
    pub instance: CircuitInstanceId,
    /// Logical pin bound to the terminal net.
    pub pin: PinRef,
    /// Physical pad carrying the terminal.
    pub pad: PadId,
    /// Logical net whose route rules govern the probe.
    pub net: NetId,
    /// Exact board-space pad center.
    pub center: Point2,
    /// Deterministically ordered layer/direction outcomes.
    pub probes: Vec<PlacementPinAccessProbeEvidence>,
}

/// Structural reason a pin-access audit could not evaluate every terminal.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PlacementPinAccessIssue {
    /// Probe distance, width, or clearance was structurally invalid.
    InvalidPolicy,
    /// Circuit or layout routing handoff was invalid.
    InvalidProblem,
    /// The exact board-boundary carrier could not be built.
    InvalidBoardBoundary(String),
    /// Existing curved copper cannot enter the current fixed-obstacle model.
    UnsupportedFixedRoute(RouteId),
}

/// Aggregate exact pin-access cost used by placement ranking.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct PlacementPinAccessScore {
    /// Terminals with no decided accessible layer/direction.
    pub fully_blocked_terminals: usize,
    /// Decided blocked layer/direction probes.
    pub blocked_probes: usize,
    /// Decided accessible layer/direction probes.
    pub accessible_probes: usize,
    /// Probes whose exact predicates were indeterminate.
    pub indeterminate_probes: usize,
    /// False when structural setup prevented a complete audit.
    pub evaluation_complete: bool,
}

/// Detailed exact access audit over every mapped placed terminal.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementPinAccessReport {
    /// Retained audit policy.
    pub policy: PlacementPinAccessPolicy,
    /// Source-addressable terminal evidence.
    pub terminals: Vec<PlacementPinAccessTerminalEvidence>,
    /// Structural setup failures.
    pub issues: Vec<PlacementPinAccessIssue>,
}

impl PlacementPinAccessReport {
    /// Reduces detailed evidence to the lexicographic placement cost.
    pub fn score(&self) -> PlacementPinAccessScore {
        let mut score = PlacementPinAccessScore {
            evaluation_complete: self.issues.is_empty(),
            ..PlacementPinAccessScore::default()
        };
        for terminal in &self.terminals {
            let mut accessible = 0_usize;
            let mut indeterminate = 0_usize;
            for probe in &terminal.probes {
                match probe.status {
                    PlacementPinAccessStatus::Accessible => {
                        accessible += 1;
                        score.accessible_probes += 1;
                    }
                    PlacementPinAccessStatus::Blocked => score.blocked_probes += 1,
                    PlacementPinAccessStatus::Indeterminate => {
                        indeterminate += 1;
                        score.indeterminate_probes += 1;
                    }
                }
            }
            if accessible == 0 && indeterminate == 0 {
                score.fully_blocked_terminals += 1;
            }
        }
        score
    }
}

/// Search policy for deterministic component placement.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementSolvePolicy {
    /// Exact spacing between candidate origins.
    pub grid_pitch: Real,
    /// Exact conservative clearance between same-side placement envelopes.
    pub clearance: Real,
    /// Hard bound on candidates examined for each movable component.
    pub max_candidates_per_component: usize,
    /// Reconsider legal authored origins instead of only repairing collisions.
    ///
    /// When false, a legal authored origin is preserved. When true, every
    /// bounded candidate participates in the connectivity/displacement score.
    pub optimize_legal_authored_positions: bool,
    /// Prefer candidates whose logical-net routing boxes overlap less.
    ///
    /// This is an exact placement-stage routability proxy, not a claim that
    /// detailed routing will succeed. Hyperpath routing remains authoritative.
    pub minimize_routing_congestion: bool,
    /// Optional exact Manhattan radius inside which same-side origins
    /// accumulate local density pressure.
    ///
    /// `None` disables density scoring. The radius uses the layout's authored
    /// length unit and must be strictly positive when present.
    pub density_radius: Option<Real>,
    /// Optional exact cardinal pad-access feedback evaluated for every candidate.
    pub pin_access: Option<PlacementPinAccessPolicy>,
}

impl Default for PlacementSolvePolicy {
    fn default() -> Self {
        Self {
            grid_pitch: Real::one(),
            clearance: Real::zero(),
            max_candidates_per_component: 100_000,
            optimize_legal_authored_positions: false,
            minimize_routing_congestion: false,
            density_radius: None,
            pin_access: None,
        }
    }
}

/// Exact lexicographic cost retained for one placement proposal.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementCandidateScore {
    /// Exact global cardinal pad-access cost after applying this candidate.
    pub pin_access: PlacementPinAccessScore,
    /// Exact overlap pressure between distinct logical-net routing boxes.
    pub routing_congestion: Real,
    /// Exact same-side origin pressure inside the authored density radius.
    pub density_pressure: Real,
    /// Sum of Manhattan pad-to-pad distances for shared logical nets.
    pub connectivity_length: Real,
    /// Squared origin displacement from the authored/resolved position.
    pub displacement_squared: Real,
    /// Number of authored rotation/side attributes changed.
    pub orientation_changes: u8,
}

/// One deterministic movement accepted by the placement solver.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementMove {
    /// Logical instance moved.
    pub instance: CircuitInstanceId,
    /// Authored/resolved origin before collision search.
    pub from: Point2,
    /// Accepted collision-free origin.
    pub to: Point2,
    /// Authored/resolved exact rotation.
    pub from_rotation_degrees: Real,
    /// Accepted exact rotation.
    pub to_rotation_degrees: Real,
    /// Authored/resolved physical side.
    pub from_side: BoardSide,
    /// Accepted physical side.
    pub to_side: BoardSide,
    /// Envelope source used during search.
    pub envelope_source: PlacementEnvelopeSource,
    /// Number of candidates examined, including the accepted candidate.
    pub candidates_tested: usize,
    /// Cost at the authored/resolved origin, even when it was geometrically illegal.
    pub authored_score: PlacementCandidateScore,
    /// Cost of the accepted candidate.
    pub accepted_score: PlacementCandidateScore,
}

/// Explicit limitation or failure produced during placement search.
#[derive(Clone, Debug, PartialEq)]
pub enum PlacementSolveIssue {
    /// The source circuit/layout failed structural validation.
    InvalidLayout,
    /// The exact board-boundary query carrier could not be constructed.
    InvalidBoardBoundary(String),
    /// The search pitch must be structurally positive.
    NonPositiveGridPitch,
    /// Clearance must be structurally non-negative.
    NegativeClearance,
    /// At least one candidate must be permitted.
    ZeroCandidateLimit,
    /// An enabled density radius must be structurally positive.
    NonPositiveDensityRadius,
    /// An enabled pin-access probe distance must be structurally positive.
    NonPositivePinAccessProbeDistance,
    /// An enabled pin-access trace width must be structurally positive.
    NonPositivePinAccessTraceWidth,
    /// An enabled pin-access clearance must be structurally non-negative.
    NegativePinAccessClearance,
    /// Structural pin-access audit setup failed.
    PinAccess(PlacementPinAccessIssue),
    /// One final terminal retained exact-predicate uncertainty.
    IndeterminatePinAccess {
        /// Logical instance owning the terminal.
        instance: CircuitInstanceId,
        /// Logical pin bound to the terminal net.
        pin: PinRef,
        /// Physical pad carrying the terminal.
        pad: PadId,
        /// Number of indeterminate layer/direction probes.
        probes: usize,
    },
    /// Existing exact placement constraints could not be resolved.
    ConstraintResolution(PlacementResolutionIssue),
    /// No courtyard, body, or pad envelope was available.
    MissingEnvelope(CircuitInstanceId),
    /// A fixed/relative placement envelope extends outside the board bounds.
    LockedOutsideBoard(CircuitInstanceId),
    /// Two fixed/relative same-side envelopes collide.
    LockedCollision {
        first: CircuitInstanceId,
        second: CircuitInstanceId,
    },
    /// No legal candidate was found within the bounded deterministic search.
    NoCandidate {
        instance: CircuitInstanceId,
        candidates_tested: usize,
    },
    /// Exact board-containment predicates could not be decided.
    IndeterminateBoardContainment(CircuitInstanceId),
    /// Exact candidate costs could not be ordered.
    IndeterminateCandidateScore(CircuitInstanceId),
}

/// Deterministic provisional placement plus an audit of every limitation.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementSolveReport {
    /// Placements in authored order after exact constraints and collision search.
    pub placements: Vec<PcbPlacement>,
    /// Accepted origin changes in authored placement order.
    pub moves: Vec<PlacementMove>,
    /// Policy, constraint, envelope, or search failures.
    pub issues: Vec<PlacementSolveIssue>,
    /// Final source-addressable pin-access evidence when requested.
    pub pin_access: Option<PlacementPinAccessReport>,
}

impl PlacementSolveReport {
    /// True when every placement received a bounded collision-free proposal.
    pub fn is_solved(&self) -> bool {
        self.issues.is_empty()
    }

    /// Returns a layout clone containing the proposed placements.
    pub fn apply_to(&self, source: &PcbLayout) -> PcbLayout {
        let mut solved = source.clone();
        solved.placements.clone_from(&self.placements);
        solved
    }
}

#[derive(Clone, Debug)]
struct Bounds {
    min_x: Real,
    min_y: Real,
    max_x: Real,
    max_y: Real,
}

impl Bounds {
    fn from_points(points: impl IntoIterator<Item = Point2>) -> Option<Self> {
        let mut points = points.into_iter();
        let first = points.next()?;
        let mut bounds = Self {
            min_x: first.x.clone(),
            min_y: first.y.clone(),
            max_x: first.x,
            max_y: first.y,
        };
        for point in points {
            if point.x < bounds.min_x {
                bounds.min_x = point.x.clone();
            }
            if point.y < bounds.min_y {
                bounds.min_y = point.y.clone();
            }
            if point.x > bounds.max_x {
                bounds.max_x = point.x.clone();
            }
            if point.y > bounds.max_y {
                bounds.max_y = point.y;
            }
        }
        Some(bounds)
    }

    fn include(&mut self, other: &Self) {
        if other.min_x < self.min_x {
            self.min_x.clone_from(&other.min_x);
        }
        if other.min_y < self.min_y {
            self.min_y.clone_from(&other.min_y);
        }
        if other.max_x > self.max_x {
            self.max_x.clone_from(&other.max_x);
        }
        if other.max_y > self.max_y {
            self.max_y.clone_from(&other.max_y);
        }
    }

    fn expanded(&self, amount: &Real) -> Self {
        Self {
            min_x: self.min_x.clone() - amount.clone(),
            min_y: self.min_y.clone() - amount.clone(),
            max_x: self.max_x.clone() + amount.clone(),
            max_y: self.max_y.clone() + amount.clone(),
        }
    }

    fn collides(&self, other: &Self, clearance: &Real) -> bool {
        !(self.max_x.clone() + clearance.clone() <= other.min_x
            || other.max_x.clone() + clearance.clone() <= self.min_x
            || self.max_y.clone() + clearance.clone() <= other.min_y
            || other.max_y.clone() + clearance.clone() <= self.min_y)
    }
}

impl PcbLayout {
    /// Resolves exact constraints, then places unconstrained components on a
    /// deterministic row-major grid without conservative same-side collisions.
    ///
    /// Fixed and relative constraint participants are treated as locked so
    /// their equations cannot be invalidated. The authored origin is tried
    /// before grid candidates. Board containment uses the outline bounding box;
    /// exact courtyard/body overlap and detailed board-profile readiness should
    /// be certified through HyperDRC after applying a successful report.
    pub fn solve_placement(
        &self,
        circuit: &Circuit,
        policy: &PlacementSolvePolicy,
    ) -> PlacementSolveReport {
        let mut report = PlacementSolveReport {
            placements: self.placements.clone(),
            moves: Vec::new(),
            issues: Vec::new(),
            pin_access: None,
        };
        if !self.validate(circuit).is_valid() {
            report.issues.push(PlacementSolveIssue::InvalidLayout);
            return report;
        }
        if policy.grid_pitch.structural_facts().sign != Some(RealSign::Positive) {
            report
                .issues
                .push(PlacementSolveIssue::NonPositiveGridPitch);
        }
        if !matches!(
            policy.clearance.structural_facts().sign,
            Some(RealSign::Zero | RealSign::Positive)
        ) {
            report.issues.push(PlacementSolveIssue::NegativeClearance);
        }
        if policy.max_candidates_per_component == 0 {
            report.issues.push(PlacementSolveIssue::ZeroCandidateLimit);
        }
        if policy
            .density_radius
            .as_ref()
            .is_some_and(|radius| radius.structural_facts().sign != Some(RealSign::Positive))
        {
            report
                .issues
                .push(PlacementSolveIssue::NonPositiveDensityRadius);
        }
        if let Some(pin_access) = &policy.pin_access {
            if pin_access.probe_distance.structural_facts().sign != Some(RealSign::Positive) {
                report
                    .issues
                    .push(PlacementSolveIssue::NonPositivePinAccessProbeDistance);
            }
            if pin_access.minimum_trace_width.structural_facts().sign != Some(RealSign::Positive) {
                report
                    .issues
                    .push(PlacementSolveIssue::NonPositivePinAccessTraceWidth);
            }
            if !matches!(
                pin_access.minimum_clearance.structural_facts().sign,
                Some(RealSign::Zero | RealSign::Positive)
            ) {
                report
                    .issues
                    .push(PlacementSolveIssue::NegativePinAccessClearance);
            }
        }
        if !report.issues.is_empty() {
            return report;
        }

        let resolved = self.resolve_placement_constraints(circuit);
        let hard_constraint_issues = resolved
            .issues
            .iter()
            .filter(|issue| {
                matches!(
                    issue,
                    PlacementResolutionIssue::InvalidLayout
                        | PlacementResolutionIssue::RelativeCycle(_)
                )
            })
            .cloned()
            .collect::<Vec<_>>();
        if !hard_constraint_issues.is_empty() {
            report.placements = resolved.placements;
            report.issues.extend(
                hard_constraint_issues
                    .into_iter()
                    .map(PlacementSolveIssue::ConstraintResolution),
            );
            return report;
        }
        report.placements = resolved.placements;

        let board_boundary = match self.outline.boundary_geometry() {
            Ok(boundary) => boundary,
            Err(error) => {
                report
                    .issues
                    .push(PlacementSolveIssue::InvalidBoardBoundary(error.to_string()));
                return report;
            }
        };
        let (board_min, board_max) = board_boundary.exterior_bounds();
        let board_bounds = Bounds {
            min_x: board_min.x,
            min_y: board_min.y,
            max_x: board_max.x,
            max_y: board_max.y,
        };
        let patterns = self
            .land_patterns
            .iter()
            .map(|pattern| (&pattern.id, pattern))
            .collect::<BTreeMap<_, _>>();
        let mut local_envelopes = BTreeMap::new();
        for placement in &report.placements {
            let Some(pattern) = patterns.get(&placement.land_pattern) else {
                continue;
            };
            match local_envelope(pattern) {
                Some(envelope) => {
                    local_envelopes.insert(placement.instance.clone(), envelope);
                }
                None => report.issues.push(PlacementSolveIssue::MissingEnvelope(
                    placement.instance.clone(),
                )),
            }
        }

        let mut locked = BTreeSet::new();
        for constraint in &self.placement_constraints {
            match &constraint.kind {
                PlacementConstraintKind::Fixed { instance, .. } => {
                    locked.insert(instance.clone());
                }
                PlacementConstraintKind::Relative {
                    instance, anchor, ..
                } => {
                    locked.insert(instance.clone());
                    locked.insert(anchor.clone());
                }
                PlacementConstraintKind::AlignX { .. }
                | PlacementConstraintKind::AlignY { .. }
                | PlacementConstraintKind::Within { .. }
                | PlacementConstraintKind::AllowedRotations { .. }
                | PlacementConstraintKind::AllowedSides { .. } => {}
            }
        }

        let mut positions = report
            .placements
            .iter()
            .map(|placement| (placement.instance.clone(), placement.position.clone()))
            .collect::<BTreeMap<_, _>>();
        let mut occupied = Vec::<(CircuitInstanceId, BoardSide, Bounds)>::new();
        for index in 0..report.placements.len() {
            if !locked.contains(&report.placements[index].instance) {
                continue;
            }
            let authored = report.placements[index].clone();
            let instance = authored.instance.clone();
            let Some((local, source)) = local_envelopes.get(&instance) else {
                continue;
            };
            let authored_score = candidate_score(
                self,
                circuit,
                &authored,
                &authored,
                &report.placements,
                policy,
            );
            let rotations = allowed_rotations(self, &instance, &authored.rotation_degrees);
            let sides = allowed_sides(self, &instance, authored.side);
            let mut candidates = vec![authored.clone()];
            for rotation in rotations {
                for side in &sides {
                    let mut candidate = authored.clone();
                    candidate.rotation_degrees.clone_from(&rotation);
                    candidate.side = *side;
                    if !candidates.contains(&candidate) {
                        candidates.push(candidate);
                    }
                }
            }
            let mut accepted: Option<(PcbPlacement, Bounds, PlacementCandidateScore)> = None;
            let mut tested = 0_usize;
            let mut indeterminate_board = false;
            let mut indeterminate_score = false;
            for candidate in candidates
                .into_iter()
                .take(policy.max_candidates_per_component)
            {
                tested += 1;
                let bounds = transformed_bounds(local, &candidate);
                match candidate_allowed(
                    self,
                    &board_boundary,
                    &instance,
                    &candidate,
                    &positions,
                    &bounds,
                    &occupied,
                    &policy.clearance,
                ) {
                    Some(true) => {
                        let score = candidate_score(
                            self,
                            circuit,
                            &candidate,
                            &authored,
                            &report.placements,
                            policy,
                        );
                        if candidate == authored && !policy.optimize_legal_authored_positions {
                            accepted = Some((candidate, bounds, score));
                            break;
                        }
                        match &accepted {
                            None => accepted = Some((candidate, bounds, score)),
                            Some((_, _, best_score)) => {
                                match compare_candidate_scores(&score, best_score) {
                                    Some(Ordering::Less) => {
                                        accepted = Some((candidate, bounds, score));
                                    }
                                    Some(Ordering::Equal | Ordering::Greater) => {}
                                    None => indeterminate_score = true,
                                }
                            }
                        }
                    }
                    Some(false) => {}
                    None => indeterminate_board = true,
                }
            }
            if indeterminate_board {
                report
                    .issues
                    .push(PlacementSolveIssue::IndeterminateBoardContainment(
                        instance.clone(),
                    ));
            }
            if indeterminate_score {
                report
                    .issues
                    .push(PlacementSolveIssue::IndeterminateCandidateScore(
                        instance.clone(),
                    ));
            }
            let Some((candidate, bounds, accepted_score)) = accepted else {
                let authored_bounds = transformed_bounds(local, &authored);
                if envelope_inside_board(&authored_bounds, &board_boundary) == Some(false) {
                    report
                        .issues
                        .push(PlacementSolveIssue::LockedOutsideBoard(instance));
                } else if let Some((other, _, _)) = occupied.iter().find(|(_, side, bounds)| {
                    *side == authored.side && authored_bounds.collides(bounds, &policy.clearance)
                }) {
                    report.issues.push(PlacementSolveIssue::LockedCollision {
                        first: other.clone(),
                        second: instance,
                    });
                } else {
                    report.issues.push(PlacementSolveIssue::NoCandidate {
                        instance,
                        candidates_tested: tested,
                    });
                }
                continue;
            };
            if candidate != authored {
                report.moves.push(PlacementMove {
                    instance: instance.clone(),
                    from: authored.position,
                    to: candidate.position.clone(),
                    from_rotation_degrees: authored.rotation_degrees,
                    to_rotation_degrees: candidate.rotation_degrees.clone(),
                    from_side: authored.side,
                    to_side: candidate.side,
                    envelope_source: *source,
                    candidates_tested: tested,
                    authored_score,
                    accepted_score,
                });
            }
            occupied.push((instance, candidate.side, bounds));
            report.placements[index] = candidate;
        }

        for index in 0..report.placements.len() {
            if locked.contains(&report.placements[index].instance) {
                continue;
            }
            let instance = report.placements[index].instance.clone();
            let authored = report.placements[index].clone();
            let Some((local, source)) = local_envelopes.get(&instance) else {
                continue;
            };
            let mut tested = 0_usize;
            let mut accepted: Option<(PcbPlacement, Bounds, PlacementCandidateScore)> = None;
            let mut indeterminate_board = false;
            let mut indeterminate_score = false;

            let authored_bounds = transformed_bounds(local, &authored);
            let mut blockers = occupied.clone();
            for future in report.placements.iter().skip(index + 1) {
                let Some((future_local, _)) = local_envelopes.get(&future.instance) else {
                    continue;
                };
                let future_bounds = transformed_bounds(future_local, future);
                if future.side != authored.side
                    || !authored_bounds.collides(&future_bounds, &policy.clearance)
                {
                    blockers.push((future.instance.clone(), future.side, future_bounds));
                }
            }
            let authored_score = candidate_score(
                self,
                circuit,
                &authored,
                &authored,
                &report.placements,
                policy,
            );
            tested += 1;
            match candidate_allowed(
                self,
                &board_boundary,
                &instance,
                &authored,
                &positions,
                &authored_bounds,
                &blockers,
                &policy.clearance,
            ) {
                Some(true) => {
                    accepted = Some((authored.clone(), authored_bounds, authored_score.clone()));
                    if !policy.optimize_legal_authored_positions {
                        report.placements[index] = authored.clone();
                        occupied.push((instance, authored.side, accepted.unwrap().1));
                        continue;
                    }
                }
                Some(false) => {}
                None => indeterminate_board = true,
            }

            let rotations = allowed_rotations(self, &instance, &authored.rotation_degrees);
            let sides = allowed_sides(self, &instance, authored.side);
            let mut y = board_bounds.min_y.clone();
            while y <= board_bounds.max_y && tested < policy.max_candidates_per_component {
                let mut x = board_bounds.min_x.clone();
                while x <= board_bounds.max_x && tested < policy.max_candidates_per_component {
                    let position = Point2::new(x.clone(), y.clone());
                    x += policy.grid_pitch.clone();
                    for rotation in &rotations {
                        for side in &sides {
                            if tested >= policy.max_candidates_per_component {
                                break;
                            }
                            let mut candidate = authored.clone();
                            candidate.position.clone_from(&position);
                            candidate.rotation_degrees.clone_from(rotation);
                            candidate.side = *side;
                            if candidate == authored {
                                continue;
                            }
                            tested += 1;
                            let bounds = transformed_bounds(local, &candidate);
                            match candidate_allowed(
                                self,
                                &board_boundary,
                                &instance,
                                &candidate,
                                &positions,
                                &bounds,
                                &blockers,
                                &policy.clearance,
                            ) {
                                Some(true) => {
                                    let score = candidate_score(
                                        self,
                                        circuit,
                                        &candidate,
                                        &authored,
                                        &report.placements,
                                        policy,
                                    );
                                    match &accepted {
                                        None => accepted = Some((candidate, bounds, score)),
                                        Some((_, _, best_score)) => {
                                            match compare_candidate_scores(&score, best_score) {
                                                Some(Ordering::Less) => {
                                                    accepted = Some((candidate, bounds, score));
                                                }
                                                Some(Ordering::Equal | Ordering::Greater) => {}
                                                None => indeterminate_score = true,
                                            }
                                        }
                                    }
                                }
                                Some(false) => {}
                                None => indeterminate_board = true,
                            }
                        }
                    }
                }
                y += policy.grid_pitch.clone();
            }

            if indeterminate_board {
                report
                    .issues
                    .push(PlacementSolveIssue::IndeterminateBoardContainment(
                        instance.clone(),
                    ));
            }
            if indeterminate_score {
                report
                    .issues
                    .push(PlacementSolveIssue::IndeterminateCandidateScore(
                        instance.clone(),
                    ));
            }
            let Some((position, bounds, accepted_score)) = accepted else {
                report.issues.push(PlacementSolveIssue::NoCandidate {
                    instance,
                    candidates_tested: tested,
                });
                continue;
            };
            if position != authored {
                report.moves.push(PlacementMove {
                    instance: instance.clone(),
                    from: authored.position.clone(),
                    to: position.position.clone(),
                    from_rotation_degrees: authored.rotation_degrees.clone(),
                    to_rotation_degrees: position.rotation_degrees.clone(),
                    from_side: authored.side,
                    to_side: position.side,
                    envelope_source: *source,
                    candidates_tested: tested,
                    authored_score,
                    accepted_score,
                });
            }
            positions.insert(instance.clone(), position.position.clone());
            occupied.push((instance, position.side, bounds));
            report.placements[index] = position;
        }
        let certified = report.apply_to(self).resolve_placement_constraints(circuit);
        report.issues.extend(
            certified
                .issues
                .into_iter()
                .map(PlacementSolveIssue::ConstraintResolution),
        );
        if let Some(policy) = &policy.pin_access {
            let solved = report.apply_to(self);
            let pin_access = solved.placement_pin_access(circuit, policy);
            report.issues.extend(
                pin_access
                    .issues
                    .iter()
                    .cloned()
                    .map(PlacementSolveIssue::PinAccess),
            );
            report
                .issues
                .extend(pin_access.terminals.iter().filter_map(|terminal| {
                    let probes = terminal
                        .probes
                        .iter()
                        .filter(|probe| probe.status == PlacementPinAccessStatus::Indeterminate)
                        .count();
                    (probes != 0).then(|| PlacementSolveIssue::IndeterminatePinAccess {
                        instance: terminal.instance.clone(),
                        pin: terminal.pin.clone(),
                        pad: terminal.pad.clone(),
                        probes,
                    })
                }));
            report.pin_access = Some(pin_access);
        }
        report
    }

    /// Audits exact cardinal fanout access for every mapped placed terminal.
    pub fn placement_pin_access(
        &self,
        circuit: &Circuit,
        policy: &PlacementPinAccessPolicy,
    ) -> PlacementPinAccessReport {
        crate::autoroute::placement_pin_access_report(self, circuit, policy)
    }
}

#[allow(clippy::too_many_arguments)]
fn candidate_allowed(
    layout: &PcbLayout,
    board_boundary: &BoardBoundaryGeometry,
    instance: &CircuitInstanceId,
    candidate: &PcbPlacement,
    positions: &BTreeMap<CircuitInstanceId, Point2>,
    bounds: &Bounds,
    occupied: &[(CircuitInstanceId, BoardSide, Bounds)],
    clearance: &Real,
) -> Option<bool> {
    if occupied.iter().any(|(_, other_side, other)| {
        *other_side == candidate.side && bounds.collides(other, clearance)
    }) {
        return Some(false);
    }
    match envelope_inside_board(bounds, board_boundary) {
        Some(true) => {}
        decision => return decision,
    }
    Some(layout.placement_constraints.iter().all(|constraint| {
        match &constraint.kind {
            PlacementConstraintKind::Within {
                instance: constrained,
                min,
                max,
            } if constrained == instance => {
                min.x <= candidate.position.x
                    && candidate.position.x <= max.x
                    && min.y <= candidate.position.y
                    && candidate.position.y <= max.y
            }
            PlacementConstraintKind::AlignX { instances } if instances.contains(instance) => {
                alignment_target(instances, instance, positions, |point| &point.x)
                    .is_none_or(|target| candidate.position.x == *target)
            }
            PlacementConstraintKind::AlignY { instances } if instances.contains(instance) => {
                alignment_target(instances, instance, positions, |point| &point.y)
                    .is_none_or(|target| candidate.position.y == *target)
            }
            PlacementConstraintKind::AllowedRotations {
                instance: constrained,
                rotations_degrees,
            } if constrained == instance => rotations_degrees.contains(&candidate.rotation_degrees),
            PlacementConstraintKind::AllowedSides {
                instance: constrained,
                sides,
            } if constrained == instance => sides.contains(&candidate.side),
            _ => true,
        }
    }))
}

fn candidate_score(
    layout: &PcbLayout,
    circuit: &Circuit,
    candidate: &PcbPlacement,
    authored: &PcbPlacement,
    placements: &[PcbPlacement],
    policy: &PlacementSolvePolicy,
) -> PlacementCandidateScore {
    let endpoints = electrical_endpoints(layout, circuit, candidate, &candidate.position);
    let mut connectivity_length = Real::zero();
    for other in placements {
        if other.instance == candidate.instance {
            continue;
        }
        for (other_net, other_point) in
            electrical_endpoints(layout, circuit, other, &other.position)
        {
            for (net, point) in &endpoints {
                if net == &other_net {
                    connectivity_length += (point.x.clone() - other_point.x.clone()).abs()
                        + (point.y.clone() - other_point.y.clone()).abs();
                }
            }
        }
    }
    let dx = candidate.position.x.clone() - authored.position.x.clone();
    let dy = candidate.position.y.clone() - authored.position.y.clone();
    PlacementCandidateScore {
        pin_access: candidate_pin_access_score(layout, circuit, candidate, placements, policy),
        routing_congestion: if policy.minimize_routing_congestion {
            routing_congestion(layout, circuit, candidate, placements)
        } else {
            Real::zero()
        },
        density_pressure: policy
            .density_radius
            .as_ref()
            .map_or_else(Real::zero, |radius| {
                density_pressure(candidate, placements, radius)
            }),
        connectivity_length,
        displacement_squared: dx.clone() * dx + dy.clone() * dy,
        orientation_changes: u8::from(candidate.rotation_degrees != authored.rotation_degrees)
            + u8::from(candidate.side != authored.side),
    }
}

fn candidate_pin_access_score(
    layout: &PcbLayout,
    circuit: &Circuit,
    candidate: &PcbPlacement,
    placements: &[PcbPlacement],
    policy: &PlacementSolvePolicy,
) -> PlacementPinAccessScore {
    let Some(policy) = &policy.pin_access else {
        return PlacementPinAccessScore {
            evaluation_complete: true,
            ..PlacementPinAccessScore::default()
        };
    };
    let mut candidate_layout = layout.clone();
    candidate_layout.placements = placements.to_vec();
    if let Some(placement) = candidate_layout
        .placements
        .iter_mut()
        .find(|placement| placement.instance == candidate.instance)
    {
        placement.clone_from(candidate);
    }
    candidate_layout
        .placement_pin_access(circuit, policy)
        .score()
}

fn density_pressure(candidate: &PcbPlacement, placements: &[PcbPlacement], radius: &Real) -> Real {
    placements
        .iter()
        .filter(|other| other.instance != candidate.instance && other.side == candidate.side)
        .fold(Real::zero(), |pressure, other| {
            let distance = (candidate.position.x.clone() - other.position.x.clone()).abs()
                + (candidate.position.y.clone() - other.position.y.clone()).abs();
            if distance < *radius {
                pressure + radius.clone() - distance
            } else {
                pressure
            }
        })
}

fn routing_congestion(
    layout: &PcbLayout,
    circuit: &Circuit,
    candidate: &PcbPlacement,
    placements: &[PcbPlacement],
) -> Real {
    let mut endpoints = BTreeMap::<crate::NetId, Vec<Point2>>::new();
    for placement in placements {
        let effective = if placement.instance == candidate.instance {
            candidate
        } else {
            placement
        };
        for (net, point) in electrical_endpoints(layout, circuit, effective, &effective.position) {
            endpoints.entry(net).or_default().push(point);
        }
    }
    let boxes = endpoints
        .into_values()
        .filter(|points| points.len() >= 2)
        .filter_map(Bounds::from_points)
        .collect::<Vec<_>>();
    let mut pressure = Real::zero();
    for (index, first) in boxes.iter().enumerate() {
        for second in boxes.iter().skip(index + 1) {
            let Some(overlap_x) =
                interval_overlap(&first.min_x, &first.max_x, &second.min_x, &second.max_x)
            else {
                continue;
            };
            let Some(overlap_y) =
                interval_overlap(&first.min_y, &first.max_y, &second.min_y, &second.max_y)
            else {
                continue;
            };
            pressure += overlap_x + overlap_y + Real::one();
        }
    }
    pressure
}

fn interval_overlap(
    first_min: &Real,
    first_max: &Real,
    second_min: &Real,
    second_max: &Real,
) -> Option<Real> {
    let low = if first_min >= second_min {
        first_min
    } else {
        second_min
    };
    let high = if first_max <= second_max {
        first_max
    } else {
        second_max
    };
    (low <= high).then(|| high.clone() - low.clone())
}

fn electrical_endpoints(
    layout: &PcbLayout,
    circuit: &Circuit,
    placement: &PcbPlacement,
    position: &Point2,
) -> Vec<(crate::NetId, Point2)> {
    let Some(instance) = circuit
        .instances
        .iter()
        .find(|instance| instance.id == placement.instance)
    else {
        return Vec::new();
    };
    let Some(pattern) = layout
        .land_patterns
        .iter()
        .find(|pattern| pattern.id == placement.land_pattern)
    else {
        return Vec::new();
    };
    let mut transformed = placement.clone();
    transformed.position.clone_from(position);
    pattern
        .pin_map
        .iter()
        .filter_map(|mapping| {
            let binding = instance
                .pins
                .iter()
                .find(|binding| binding.pin == mapping.pin)?;
            let pad = pattern.pads.iter().find(|pad| pad.id == mapping.pad)?;
            Some((
                binding.net.clone(),
                transformed.transform_point(&pad.center),
            ))
        })
        .collect()
}

fn compare_candidate_scores(
    candidate: &PlacementCandidateScore,
    current: &PlacementCandidateScore,
) -> Option<Ordering> {
    if !candidate.pin_access.evaluation_complete
        || !current.pin_access.evaluation_complete
        || candidate.pin_access.indeterminate_probes != 0
        || current.pin_access.indeterminate_probes != 0
    {
        return None;
    }
    match candidate
        .pin_access
        .fully_blocked_terminals
        .cmp(&current.pin_access.fully_blocked_terminals)
    {
        Ordering::Equal => {}
        ordering => return Some(ordering),
    }
    match candidate
        .pin_access
        .blocked_probes
        .cmp(&current.pin_access.blocked_probes)
    {
        Ordering::Equal => {}
        ordering => return Some(ordering),
    }
    match candidate
        .routing_congestion
        .partial_cmp(&current.routing_congestion)?
    {
        Ordering::Equal => {}
        ordering => return Some(ordering),
    }
    match candidate
        .density_pressure
        .partial_cmp(&current.density_pressure)?
    {
        Ordering::Equal => {}
        ordering => return Some(ordering),
    }
    match candidate
        .connectivity_length
        .partial_cmp(&current.connectivity_length)?
    {
        Ordering::Equal => candidate
            .displacement_squared
            .partial_cmp(&current.displacement_squared)
            .map(|ordering| {
                if ordering == Ordering::Equal {
                    candidate
                        .orientation_changes
                        .cmp(&current.orientation_changes)
                } else {
                    ordering
                }
            }),
        ordering => Some(ordering),
    }
}

fn envelope_inside_board(bounds: &Bounds, boundary: &BoardBoundaryGeometry) -> Option<bool> {
    match boundary
        .contains_axis_aligned_box(
            &Point2::new(bounds.min_x.clone(), bounds.min_y.clone()),
            &Point2::new(bounds.max_x.clone(), bounds.max_y.clone()),
            &CurvePolicy::certified(),
        )
        .ok()?
    {
        Classification::Decided(decision) => Some(decision),
        Classification::Uncertain(_) => None,
    }
}

fn alignment_target<'a>(
    instances: &[CircuitInstanceId],
    candidate: &CircuitInstanceId,
    positions: &'a BTreeMap<CircuitInstanceId, Point2>,
    coordinate: impl Fn(&'a Point2) -> &'a Real,
) -> Option<&'a Real> {
    instances
        .iter()
        .find(|instance| *instance != candidate)
        .and_then(|instance| positions.get(instance))
        .map(coordinate)
}

fn allowed_rotations(
    layout: &PcbLayout,
    instance: &CircuitInstanceId,
    authored: &Real,
) -> Vec<Real> {
    let constraints = layout
        .placement_constraints
        .iter()
        .filter_map(|constraint| match &constraint.kind {
            PlacementConstraintKind::AllowedRotations {
                instance: constrained,
                rotations_degrees,
            } if constrained == instance => Some(rotations_degrees),
            _ => None,
        })
        .collect::<Vec<_>>();
    let Some(first) = constraints.first() else {
        return vec![authored.clone()];
    };
    let mut rotations = Vec::new();
    if first.contains(authored)
        && constraints
            .iter()
            .all(|constraint| constraint.contains(authored))
    {
        rotations.push(authored.clone());
    }
    for rotation in *first {
        if !rotations.contains(rotation)
            && constraints
                .iter()
                .all(|constraint| constraint.contains(rotation))
        {
            rotations.push(rotation.clone());
        }
    }
    rotations
}

fn allowed_sides(
    layout: &PcbLayout,
    instance: &CircuitInstanceId,
    authored: BoardSide,
) -> Vec<BoardSide> {
    let constraints = layout
        .placement_constraints
        .iter()
        .filter_map(|constraint| match &constraint.kind {
            PlacementConstraintKind::AllowedSides {
                instance: constrained,
                sides,
            } if constrained == instance => Some(sides),
            _ => None,
        })
        .collect::<Vec<_>>();
    let Some(first) = constraints.first() else {
        return vec![authored];
    };
    let mut sides = Vec::new();
    if first.contains(&authored)
        && constraints
            .iter()
            .all(|constraint| constraint.contains(&authored))
    {
        sides.push(authored);
    }
    for side in *first {
        if !sides.contains(side)
            && constraints
                .iter()
                .all(|constraint| constraint.contains(side))
        {
            sides.push(*side);
        }
    }
    sides
}

fn transformed_bounds(local: &Bounds, placement: &PcbPlacement) -> Bounds {
    Bounds::from_points(
        [
            Point2::new(local.min_x.clone(), local.min_y.clone()),
            Point2::new(local.min_x.clone(), local.max_y.clone()),
            Point2::new(local.max_x.clone(), local.min_y.clone()),
            Point2::new(local.max_x.clone(), local.max_y.clone()),
        ]
        .iter()
        .map(|point| placement.transform_point(point)),
    )
    .expect("four envelope corners are nonempty")
}

fn local_envelope(pattern: &LandPattern) -> Option<(Bounds, PlacementEnvelopeSource)> {
    let mut courtyard: Option<Bounds> = None;
    let mut stroke_expansion = Real::zero();
    for graphic in &pattern.graphics {
        if graphic.layer != LayerRole::Courtyard {
            continue;
        }
        let bounds = match &graphic.primitive {
            LandPatternGraphicPrimitive::Line { start, end } => {
                Bounds::from_points([start.clone(), end.clone()])
            }
            LandPatternGraphicPrimitive::Circle { center, radius } => Some(Bounds {
                min_x: center.x.clone() - radius.clone(),
                min_y: center.y.clone() - radius.clone(),
                max_x: center.x.clone() + radius.clone(),
                max_y: center.y.clone() + radius.clone(),
            }),
            LandPatternGraphicPrimitive::Polygon { vertices, .. } => {
                Bounds::from_points(vertices.iter().cloned())
            }
            LandPatternGraphicPrimitive::Text { .. } => None,
        };
        if let Some(bounds) = bounds {
            match &mut courtyard {
                Some(combined) => combined.include(&bounds),
                None => courtyard = Some(bounds),
            }
        }
        if let Some(width) = &graphic.stroke_width {
            let half = (width.clone() / Real::from(2)).expect("two is nonzero");
            if half > stroke_expansion {
                stroke_expansion = half;
            }
        }
    }
    if let Some(courtyard) = courtyard {
        return Some((
            courtyard.expanded(&stroke_expansion),
            PlacementEnvelopeSource::Courtyard,
        ));
    }
    if let Some(body) = &pattern.body
        && let Some(bounds) = Bounds::from_points(body.outline.iter().cloned())
    {
        return Some((bounds, PlacementEnvelopeSource::Body));
    }

    let mut pads: Option<Bounds> = None;
    for pad in &pattern.pads {
        let bounds = pad_bounds(pad);
        match &mut pads {
            Some(combined) => combined.include(&bounds),
            None => pads = Some(bounds),
        }
    }
    pads.map(|bounds| (bounds, PlacementEnvelopeSource::Pads))
}

fn pad_bounds(pad: &crate::LandPatternPad) -> Bounds {
    if let PadShape::Circle { diameter } = &pad.shape {
        let half = (diameter.clone() / Real::from(2)).expect("two is nonzero");
        return Bounds {
            min_x: pad.center.x.clone() - half.clone(),
            min_y: pad.center.y.clone() - half.clone(),
            max_x: pad.center.x.clone() + half.clone(),
            max_y: pad.center.y.clone() + half,
        };
    }
    let points = match &pad.shape {
        PadShape::Circle { .. } => unreachable!("circle returned above"),
        PadShape::Rectangle { width, height }
        | PadShape::RoundedRectangle { width, height, .. }
        | PadShape::Obround { width, height } => {
            let half_width = (width.clone() / Real::from(2)).expect("two is nonzero");
            let half_height = (height.clone() / Real::from(2)).expect("two is nonzero");
            vec![
                Point2::new(-half_width.clone(), -half_height.clone()),
                Point2::new(half_width.clone(), -half_height.clone()),
                Point2::new(half_width.clone(), half_height.clone()),
                Point2::new(-half_width, half_height),
            ]
        }
        PadShape::Polygon { vertices } => vertices.clone(),
    };
    let radians = pad.rotation_degrees.clone().to_radians();
    let sin = radians.clone().sin();
    let cos = radians.cos();
    Bounds::from_points(points.into_iter().map(|point| {
        Point2::new(
            point.x.clone() * cos.clone() - point.y.clone() * sin.clone() + pad.center.x.clone(),
            point.x * sin.clone() + point.y * cos.clone() + pad.center.y.clone(),
        )
    }))
    .expect("validated non-circular pad has bounds")
}
