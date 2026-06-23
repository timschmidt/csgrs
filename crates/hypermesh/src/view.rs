//! Borrowed exact views of retained mesh data.

use std::{cell::RefCell, rc::Rc};

use super::ExactMesh;
use super::arrangement3d::{ArrangementView, ExactArrangement};
use super::boolean::{
    ExactArrangementCellComplexShortcutFacts, ExactBooleanOperation, ExactBooleanRequest,
    materialize_boolean_exact_request_with_prepared_pair,
};
use super::bounds::{
    BroadPhaseScratch, CandidateFacePairPlan, ExactAabb3, ExactAabbBroadPhase, ExactBroadPhase,
    PreparedMeshBounds,
};
use super::error::ExactMeshError;
use super::error::{ExactMeshBlocker, ExactMeshBlockerKind};
use super::graph::{
    ExactIntersectionGraph, build_unvalidated_intersection_graph_from_prepared_pair_rc,
    build_validated_intersection_graph_from_prepared_pair,
};
use super::intersection::{
    MeshFacePairClassification, MeshFacePairRelation, classify_mesh_face_pair_unchecked,
};
use super::regularization::ExactRegularizationPolicy;
use super::validation::ExactMeshValidationPolicy;
use hyperlimit::{ApproximationPolicy, MeshSource, Point3, PredicateUse};
use hyperreal::Real;

/// Borrowed exact view of an [`ExactMesh`].
#[derive(Clone, Copy, Debug)]
pub struct ExactMeshRef<'a> {
    mesh: &'a ExactMesh,
}

/// Alias for the borrowed exact mesh view.
pub type MeshView<'a> = ExactMeshRef<'a>;

/// Borrowed face view.
#[derive(Clone, Copy, Debug)]
pub struct FaceRef<'a> {
    mesh: &'a ExactMesh,
    index: usize,
}

/// Borrowed vertex view.
#[derive(Clone, Copy, Debug)]
pub struct VertexRef<'a> {
    mesh: &'a ExactMesh,
    index: usize,
}

/// Borrowed triangle view.
#[derive(Clone, Copy, Debug)]
pub struct TriangleRef<'a> {
    mesh: &'a ExactMesh,
    index: usize,
}

/// Borrowed edge view.
#[derive(Clone, Copy, Debug)]
pub struct EdgeRef<'a> {
    mesh: &'a ExactMesh,
    index: usize,
}

/// Borrowed exact mesh view with prepared broad-phase acceleration facts.
#[derive(Debug)]
pub struct PreparedMeshView<'a> {
    view: ExactMeshRef<'a>,
    bounds: PreparedMeshBounds<'a>,
}

/// Owned borrowed mesh-pair cache with certificate-validated broad-phase facts.
#[derive(Debug)]
pub struct PreparedMeshPair<'left, 'right> {
    left: PreparedMeshView<'left>,
    right: PreparedMeshView<'right>,
    plan: CandidateFacePairPlan,
    broad_phase_summary: PreparedMeshPairBroadPhaseSummary,
    candidate_pair_capacity_hint: usize,
    scratch: RefCell<BroadPhaseScratch>,
    candidate_face_pairs: RefCell<Option<Vec<[usize; 2]>>>,
    broad_phase_traversal_summary: RefCell<Option<PreparedMeshPairBroadPhaseTraversalSummary>>,
    face_pair_classifications: RefCell<Option<Vec<MeshFacePairClassification>>>,
    face_pair_classification_counts: RefCell<Option<PreparedMeshPairClassificationCounts>>,
    intersection_graph: RefCell<Option<Rc<ExactIntersectionGraph>>>,
    intersection_graph_counts: RefCell<Option<PreparedMeshPairIntersectionGraphCounts>>,
    intersection_graph_validated: RefCell<bool>,
    arrangement: RefCell<Option<Rc<ExactArrangement>>>,
    arrangement_counts: RefCell<Option<PreparedMeshPairArrangementCounts>>,
    arrangement_shortcut_facts: RefCell<Option<ExactArrangementCellComplexShortcutFacts>>,
    union_result: RefCell<Option<Result<ExactMesh, ExactMeshError>>>,
    union_result_outcome: RefCell<Option<PreparedMeshPairResultOutcome>>,
    intersection_result: RefCell<Option<Result<ExactMesh, ExactMeshError>>>,
    intersection_result_outcome: RefCell<Option<PreparedMeshPairResultOutcome>>,
    difference_result: RefCell<Option<Result<ExactMesh, ExactMeshError>>>,
    difference_result_outcome: RefCell<Option<PreparedMeshPairResultOutcome>>,
    xor_result: RefCell<Option<Result<ExactMesh, ExactMeshError>>>,
    xor_result_outcome: RefCell<Option<PreparedMeshPairResultOutcome>>,
}

/// Borrowed prepared pair view with retained broad-phase pair planning.
#[derive(Debug)]
pub struct PreparedMeshPairView<'pair, 'left, 'right> {
    left: &'pair PreparedMeshView<'left>,
    right: &'pair PreparedMeshView<'right>,
    plan: CandidateFacePairPlan,
    broad_phase_summary: PreparedMeshPairBroadPhaseSummary,
    candidate_pair_capacity_hint: usize,
}

/// Cheap status for retained facts inside a prepared mesh-pair session.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PreparedMeshPairCacheStatus {
    source_pair: PreparedMeshPairFactState,
    broad_phase_traversal: PreparedMeshPairFactState,
    retained_broad_phase_traversal_summary: Option<PreparedMeshPairBroadPhaseTraversalSummary>,
    candidate_face_pairs: PreparedMeshPairFactState,
    face_pair_classifications: PreparedMeshPairFactState,
    face_pair_classification_counts: PreparedMeshPairFactState,
    retained_face_pair_classification_counts: Option<PreparedMeshPairClassificationCounts>,
    intersection_graph: PreparedMeshPairFactState,
    retained_intersection_graph_counts: Option<PreparedMeshPairIntersectionGraphCounts>,
    arrangement: PreparedMeshPairFactState,
    retained_arrangement_counts: Option<PreparedMeshPairArrangementCounts>,
    arrangement_shortcut_facts: PreparedMeshPairFactState,
    union_result: PreparedMeshPairFactState,
    union_result_outcome: Option<PreparedMeshPairResultOutcome>,
    intersection_result: PreparedMeshPairFactState,
    intersection_result_outcome: Option<PreparedMeshPairResultOutcome>,
    difference_result: PreparedMeshPairFactState,
    difference_result_outcome: Option<PreparedMeshPairResultOutcome>,
    xor_result: PreparedMeshPairFactState,
    xor_result_outcome: Option<PreparedMeshPairResultOutcome>,
}

/// Compact source/freshness stamp for retained exact mesh facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactMeshSourceStamp {
    source: MeshSource,
    approximation: ApproximationPolicy,
    source_identity: u64,
    construction_version: u64,
    vertex_count: usize,
    face_count: usize,
}

impl ExactMeshSourceStamp {
    /// Return the retained source category.
    pub const fn source(self) -> MeshSource {
        self.source
    }

    /// Return the retained approximation boundary policy.
    pub const fn approximation(self) -> ApproximationPolicy {
        self.approximation
    }

    /// Return the deterministic identity fingerprint for the retained source provenance.
    pub const fn source_identity(self) -> u64 {
        self.source_identity
    }

    /// Return the retained construction version for facts derived from this source.
    pub const fn construction_version(self) -> u64 {
        self.construction_version
    }

    /// Return the retained vertex count covered by this stamp.
    pub const fn vertex_count(self) -> usize {
        self.vertex_count
    }

    /// Return the retained face count covered by this stamp.
    pub const fn face_count(self) -> usize {
        self.face_count
    }

    fn from_mesh(mesh: &ExactMesh) -> Self {
        let provenance = mesh.provenance();
        Self {
            source: provenance.source.source,
            approximation: provenance.source.approximation,
            source_identity: source_provenance_identity(provenance),
            construction_version: provenance.construction_version,
            vertex_count: mesh.facts().mesh.vertex_count,
            face_count: mesh.facts().mesh.face_count,
        }
    }
}

fn source_provenance_identity(provenance: &hyperlimit::ConstructionProvenance) -> u64 {
    let mut hash = 0xcbf29ce484222325u64;
    hash = fnv1a_u64(hash, mesh_source_tag(provenance.source.source));
    hash = fnv1a_u64(
        hash,
        approximation_policy_tag(provenance.source.approximation),
    );
    for &byte in provenance.source.label.as_bytes() {
        hash = fnv1a_u8(hash, byte);
    }
    hash
}

const fn mesh_source_tag(source: MeshSource) -> u64 {
    match source {
        MeshSource::Exact => 0x01,
        MeshSource::LossyF64 => 0x02,
        MeshSource::HypermeshAdapter => 0x03,
        MeshSource::ExternalAdapter => 0x04,
    }
}

const fn approximation_policy_tag(approximation: ApproximationPolicy) -> u64 {
    match approximation {
        ApproximationPolicy::ExactOnly => 0x11,
        ApproximationPolicy::EdgeOnly => 0x12,
        ApproximationPolicy::ExplicitApproximateDecision => 0x13,
    }
}

const fn fnv1a_u64(mut hash: u64, value: u64) -> u64 {
    let mut shift = 0;
    while shift < 64 {
        hash = fnv1a_u8(hash, ((value >> shift) & 0xff) as u8);
        shift += 8;
    }
    hash
}

const fn fnv1a_u8(hash: u64, byte: u8) -> u64 {
    (hash ^ byte as u64).wrapping_mul(0x100000001b3)
}

/// Retained broad-phase plan chosen for a prepared mesh-pair session.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PreparedMeshPairPlanKind {
    /// Whole-mesh or face-level bounds prove that no candidate face pairs are needed.
    Empty,
    /// A sorted-axis sweep plan was retained for candidate traversal.
    Sweep,
    /// No certified sweep axis was retained, so candidate traversal falls back to exact quadratic checks.
    Quadratic,
}

/// Retained broad-phase planning provenance for a prepared mesh-pair session.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PreparedMeshPairBroadPhaseSummary {
    left_source: ExactMeshSourceStamp,
    right_source: ExactMeshSourceStamp,
    plan: PreparedMeshPairPlanKind,
    left_face_count: usize,
    right_face_count: usize,
    face_pair_product: usize,
    candidate_pair_upper_bound: usize,
    candidate_pair_capacity_hint: usize,
    active_face_capacity_hint: Option<usize>,
    sweep_axis: Option<PreparedMeshPairSweepAxis>,
    sweep_direction: Option<PreparedMeshPairSweepDirection>,
    sweep_active_set: Option<PreparedMeshPairSweepActiveSet>,
}

impl PreparedMeshPairBroadPhaseSummary {
    /// Return the retained left source/freshness stamp.
    pub const fn left_source(self) -> ExactMeshSourceStamp {
        self.left_source
    }

    /// Return the retained right source/freshness stamp.
    pub const fn right_source(self) -> ExactMeshSourceStamp {
        self.right_source
    }

    /// Return the retained broad-phase candidate traversal plan.
    pub const fn plan(self) -> PreparedMeshPairPlanKind {
        self.plan
    }

    /// Return the left face count used when this pair plan was selected.
    pub const fn left_face_count(self) -> usize {
        self.left_face_count
    }

    /// Return the right face count used when this pair plan was selected.
    pub const fn right_face_count(self) -> usize {
        self.right_face_count
    }

    /// Return the exact Cartesian face-pair product before broad-phase rejection.
    pub const fn face_pair_product(self) -> usize {
        self.face_pair_product
    }

    /// Return the retained upper bound on pairs the broad phase may inspect.
    pub const fn candidate_pair_upper_bound(self) -> usize {
        self.candidate_pair_upper_bound
    }

    /// Return the bounded vector reserve hint used by retained pair stages.
    pub const fn candidate_pair_capacity_hint(self) -> usize {
        self.candidate_pair_capacity_hint
    }

    /// Return the retained sweep active-set capacity hint, when the plan uses a sweep.
    pub const fn active_face_capacity_hint(self) -> Option<usize> {
        self.active_face_capacity_hint
    }

    /// Return the retained sweep axis, when the plan uses a sweep.
    pub const fn sweep_axis(self) -> Option<PreparedMeshPairSweepAxis> {
        self.sweep_axis
    }

    /// Return the retained sweep driver direction, when the plan uses a sweep.
    pub const fn sweep_direction(self) -> Option<PreparedMeshPairSweepDirection> {
        self.sweep_direction
    }

    /// Return retained sweep active-set storage strategy, when the plan uses a sweep.
    pub const fn sweep_active_set(self) -> Option<PreparedMeshPairSweepActiveSet> {
        self.sweep_active_set
    }

    const fn from_plan(
        left_source: ExactMeshSourceStamp,
        right_source: ExactMeshSourceStamp,
        plan: CandidateFacePairPlan,
        left_face_count: usize,
        right_face_count: usize,
        candidate_pair_capacity_hint: usize,
    ) -> Self {
        Self {
            left_source,
            right_source,
            plan: PreparedMeshPairPlanKind::from_candidate_plan(plan),
            left_face_count,
            right_face_count,
            face_pair_product: left_face_count.saturating_mul(right_face_count),
            candidate_pair_upper_bound: plan
                .candidate_pair_upper_bound(left_face_count, right_face_count),
            candidate_pair_capacity_hint,
            active_face_capacity_hint: plan.active_face_capacity_hint(),
            sweep_axis: PreparedMeshPairSweepAxis::from_candidate_axis_index(
                plan.sweep_axis_index(),
            ),
            sweep_direction: PreparedMeshPairSweepDirection::from_candidate_direction(
                plan.sweep_is_left_driven(),
            ),
            sweep_active_set: PreparedMeshPairSweepActiveSet::from_sparse_flag(
                plan.sweep_uses_sparse_active_set(left_face_count, right_face_count),
            ),
        }
    }
}

/// Retained counts from an executed broad-phase candidate traversal.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PreparedMeshPairBroadPhaseTraversalSummary {
    broad_phase: PreparedMeshPairBroadPhaseSummary,
    candidate_pair_count: usize,
}

impl PreparedMeshPairBroadPhaseTraversalSummary {
    /// Return the retained broad-phase plan that produced this traversal.
    pub const fn broad_phase_summary(self) -> PreparedMeshPairBroadPhaseSummary {
        self.broad_phase
    }

    /// Return the number of candidate face pairs emitted by the traversal.
    pub const fn candidate_pair_count(self) -> usize {
        self.candidate_pair_count
    }

    /// Return the full face-pair product covered by the traversal.
    pub const fn face_pair_product(self) -> usize {
        self.broad_phase.face_pair_product()
    }

    /// Return the number of face pairs rejected by exact broad-phase bounds.
    pub fn broad_phase_rejection_count(self) -> usize {
        self.face_pair_product()
            .saturating_sub(self.candidate_pair_count)
    }

    /// Return the slack between the plan's candidate upper bound and emitted pairs.
    pub fn candidate_upper_bound_slack(self) -> usize {
        self.broad_phase
            .candidate_pair_upper_bound()
            .saturating_sub(self.candidate_pair_count)
    }

    /// Return whether the traversal saturated the retained candidate upper bound.
    pub fn candidate_upper_bound_saturated(self) -> bool {
        self.candidate_upper_bound_slack() == 0
    }

    const fn from_broad_phase(
        broad_phase: PreparedMeshPairBroadPhaseSummary,
        candidate_pair_count: usize,
    ) -> Self {
        Self {
            broad_phase,
            candidate_pair_count,
        }
    }
}

/// Retained active-set storage strategy for a prepared sweep traversal.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PreparedMeshPairSweepActiveSet {
    /// Sparse active-face list maintenance is selected for low active occupancy.
    Sparse,
    /// Marked active-face storage is selected for denser active occupancy.
    Marked,
}

impl PreparedMeshPairSweepActiveSet {
    const fn from_sparse_flag(sparse: Option<bool>) -> Option<Self> {
        match sparse {
            Some(true) => Some(Self::Sparse),
            Some(false) => Some(Self::Marked),
            None => None,
        }
    }
}

/// Retained broad-phase sweep axis for a prepared mesh-pair session.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PreparedMeshPairSweepAxis {
    /// Sweep along the X axis.
    X,
    /// Sweep along the Y axis.
    Y,
    /// Sweep along the Z axis.
    Z,
}

impl PreparedMeshPairSweepAxis {
    const fn from_candidate_axis_index(axis: Option<usize>) -> Option<Self> {
        match axis {
            Some(0) => Some(Self::X),
            Some(1) => Some(Self::Y),
            Some(2) => Some(Self::Z),
            _ => None,
        }
    }
}

/// Retained broad-phase sweep driver direction for a prepared mesh-pair session.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PreparedMeshPairSweepDirection {
    /// Left mesh faces drive the sweep active set.
    LeftDriven,
    /// Right mesh faces drive the sweep active set.
    RightDriven,
}

impl PreparedMeshPairSweepDirection {
    const fn from_candidate_direction(left_driven: Option<bool>) -> Option<Self> {
        match left_driven {
            Some(true) => Some(Self::LeftDriven),
            Some(false) => Some(Self::RightDriven),
            None => None,
        }
    }
}

/// Retained summary counts for a prepared intersection graph.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PreparedMeshPairIntersectionGraphCounts {
    face_pairs: usize,
    events: usize,
    has_unknowns: bool,
    coplanar_overlap_graphs: usize,
}

impl PreparedMeshPairIntersectionGraphCounts {
    /// Return retained graph face-pair record count.
    pub const fn face_pair_count(self) -> usize {
        self.face_pairs
    }

    /// Return retained graph event count.
    pub const fn event_count(self) -> usize {
        self.events
    }

    /// Return whether retained graph evidence contains an undecided relation.
    pub const fn has_unknowns(self) -> bool {
        self.has_unknowns
    }

    /// Return retained coplanar overlap graph count.
    pub const fn coplanar_overlap_graph_count(self) -> usize {
        self.coplanar_overlap_graphs
    }

    fn from_graph(graph: &ExactIntersectionGraph) -> Self {
        Self {
            face_pairs: graph.face_pairs.len(),
            events: graph.event_count(),
            has_unknowns: graph.has_unknowns(),
            coplanar_overlap_graphs: graph.coplanar_overlap_graph_count(),
        }
    }
}

/// Retained topology counts for a prepared arrangement.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PreparedMeshPairArrangementCounts {
    complete: bool,
    vertices: usize,
    edges: usize,
    face_cells: usize,
    regions: usize,
    volume_regions: usize,
    volume_adjacencies: usize,
    lower_dimensional_artifacts: usize,
    blockers: usize,
}

impl PreparedMeshPairArrangementCounts {
    /// Return whether the retained arrangement completed without blockers.
    pub const fn is_complete(self) -> bool {
        self.complete
    }

    /// Return the retained arrangement vertex count.
    pub const fn vertex_count(self) -> usize {
        self.vertices
    }

    /// Return the retained arrangement edge count.
    pub const fn edge_count(self) -> usize {
        self.edges
    }

    /// Return the retained arrangement face-cell count.
    pub const fn face_cell_count(self) -> usize {
        self.face_cells
    }

    /// Return the retained connected face-cell region count.
    pub const fn region_count(self) -> usize {
        self.regions
    }

    /// Return the retained volume-region count.
    pub const fn volume_region_count(self) -> usize {
        self.volume_regions
    }

    /// Return the retained volume-adjacency count.
    pub const fn volume_adjacency_count(self) -> usize {
        self.volume_adjacencies
    }

    /// Return the retained lower-dimensional artifact count.
    pub const fn lower_dimensional_artifact_count(self) -> usize {
        self.lower_dimensional_artifacts
    }

    /// Return the retained arrangement blocker count.
    pub const fn blocker_count(self) -> usize {
        self.blockers
    }

    fn from_arrangement(arrangement: &ExactArrangement) -> Self {
        Self {
            complete: arrangement.is_complete(),
            vertices: arrangement.vertices.len(),
            edges: arrangement.edges.len(),
            face_cells: arrangement.face_cells.len(),
            regions: arrangement.shells_or_regions.as_ref().map_or(0, Vec::len),
            volume_regions: arrangement.volume_regions.as_ref().map_or(0, Vec::len),
            volume_adjacencies: arrangement.volume_adjacencies.as_ref().map_or(0, Vec::len),
            lower_dimensional_artifacts: arrangement.lower_dimensional_artifacts.len(),
            blockers: arrangement.blockers.len(),
        }
    }
}

/// Retained outcome summary for a prepared named boolean result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PreparedMeshPairResultOutcome {
    /// The retained result is an exact mesh with retained construction facts.
    Mesh {
        /// Output vertex count.
        vertices: usize,
        /// Output triangle count.
        triangles: usize,
    },
    /// The retained result is a typed exact blocker set.
    Blocked {
        /// Number of retained blockers.
        blockers: usize,
        /// First retained blocker kind, when the error carried at least one blocker.
        first_blocker: Option<ExactMeshBlockerKind>,
    },
}

impl PreparedMeshPairResultOutcome {
    /// Return whether the retained result is a mesh.
    pub const fn is_mesh(self) -> bool {
        matches!(self, Self::Mesh { .. })
    }

    /// Return whether the retained result is a typed blocker set.
    pub const fn is_blocked(self) -> bool {
        matches!(self, Self::Blocked { .. })
    }

    /// Return the retained output vertex count, when the result is a mesh.
    pub const fn vertex_count(self) -> Option<usize> {
        match self {
            Self::Mesh { vertices, .. } => Some(vertices),
            Self::Blocked { .. } => None,
        }
    }

    /// Return the retained output triangle count, when the result is a mesh.
    pub const fn triangle_count(self) -> Option<usize> {
        match self {
            Self::Mesh { triangles, .. } => Some(triangles),
            Self::Blocked { .. } => None,
        }
    }

    /// Return the retained blocker count, when the result is blocked.
    pub const fn blocker_count(self) -> Option<usize> {
        match self {
            Self::Mesh { .. } => None,
            Self::Blocked { blockers, .. } => Some(blockers),
        }
    }

    /// Return the first retained blocker kind, when the result is blocked.
    pub const fn first_blocker_kind(self) -> Option<ExactMeshBlockerKind> {
        match self {
            Self::Mesh { .. } => None,
            Self::Blocked { first_blocker, .. } => first_blocker,
        }
    }

    fn from_result(result: &Result<ExactMesh, ExactMeshError>) -> Self {
        match result {
            Ok(mesh) => Self::Mesh {
                vertices: mesh.vertices().len(),
                triangles: mesh.triangle_count(),
            },
            Err(error) => Self::Blocked {
                blockers: error.blockers().len(),
                first_blocker: error.blockers().first().map(ExactMeshBlocker::kind),
            },
        }
    }
}

/// Certificate state for retained facts inside a prepared mesh-pair session.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PreparedMeshPairFactState {
    /// The fact has not been computed for this session.
    Missing,
    /// The retained fact was built for source stamps that no longer match this session.
    Stale,
    /// The fact is retained but cannot be consumed by cheap certificate checks yet.
    CertificateBlocked,
    /// The fact is retained and its certificate is current for this session.
    Current,
}

/// Retained decision counts for coarse exact face-pair classifications.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PreparedMeshPairClassificationCounts {
    face_pairs: usize,
    plane_separated: usize,
    coplanar_touching: usize,
    coplanar_overlapping: usize,
    candidates: usize,
    unknown: usize,
    graph_required: usize,
}

impl PreparedMeshPairClassificationCounts {
    /// Return the total number of retained coarse face-pair classifications.
    pub const fn face_pair_count(self) -> usize {
        self.face_pairs
    }

    /// Return the number of pairs rejected by exact plane-side predicates.
    pub const fn plane_separated_count(self) -> usize {
        self.plane_separated
    }

    /// Return the number of coplanar touching pairs retained for graph construction.
    pub const fn coplanar_touching_count(self) -> usize {
        self.coplanar_touching
    }

    /// Return the number of coplanar overlapping pairs retained for graph construction.
    pub const fn coplanar_overlapping_count(self) -> usize {
        self.coplanar_overlapping
    }

    /// Return the number of non-coplanar candidate pairs retained for graph construction.
    pub const fn candidate_count(self) -> usize {
        self.candidates
    }

    /// Return the number of pairs with undecided coarse predicates.
    pub const fn unknown_count(self) -> usize {
        self.unknown
    }

    /// Return the number of retained classifications that require graph construction.
    pub const fn graph_required_count(self) -> usize {
        self.graph_required
    }

    fn from_classifications(classifications: &[MeshFacePairClassification]) -> Self {
        let mut counts = Self::default();
        for classification in classifications {
            counts.record(classification);
        }
        counts
    }

    pub(crate) fn record(&mut self, classification: &MeshFacePairClassification) {
        self.face_pairs = self.face_pairs.saturating_add(1);
        match classification.relation {
            MeshFacePairRelation::PlaneSeparated => self.plane_separated += 1,
            MeshFacePairRelation::CoplanarTouching => self.coplanar_touching += 1,
            MeshFacePairRelation::CoplanarOverlapping => self.coplanar_overlapping += 1,
            MeshFacePairRelation::Candidate => self.candidates += 1,
            MeshFacePairRelation::Unknown => self.unknown += 1,
        }
        if classification.needs_graph_construction() {
            self.graph_required += 1;
        }
    }
}

impl PreparedMeshPairFactState {
    /// Return whether a later stage can consume this fact through a cheap certificate check.
    pub const fn is_current(self) -> bool {
        matches!(self, Self::Current)
    }

    /// Return whether the fact exists but lacks a current cheap certificate.
    pub const fn is_certificate_blocked(self) -> bool {
        matches!(self, Self::CertificateBlocked)
    }

    /// Convert a non-current state into a typed blocker for callers that require a current fact.
    pub fn blocker(self, fact: &'static str) -> Option<ExactMeshBlocker> {
        match self {
            Self::Missing => Some(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                format!("prepared mesh-pair session is missing retained {fact} evidence"),
            )),
            Self::CertificateBlocked => Some(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                format!(
                    "prepared mesh-pair session retained {fact} evidence without a current certificate"
                ),
            )),
            Self::Stale => Some(ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!(
                    "prepared mesh-pair session retained {fact} evidence for stale source stamps"
                ),
            )),
            Self::Current => None,
        }
    }

    /// Require a current retained fact and return a typed blocker otherwise.
    pub fn require_current(self, fact: &'static str) -> Result<(), ExactMeshError> {
        self.blocker(fact)
            .map_or(Ok(()), |blocker| Err(ExactMeshError::one(blocker)))
    }
}

impl PreparedMeshPairCacheStatus {
    /// Require retained pair source stamps to match the current session meshes.
    pub fn require_current_sources(self) -> Result<(), ExactMeshError> {
        self.source_pair.require_current("source-pair stamp")
    }

    /// Return the certificate state for retained broad-phase traversal counts.
    pub const fn broad_phase_traversal(self) -> PreparedMeshPairFactState {
        self.broad_phase_traversal
    }

    /// Require retained broad-phase traversal counts with current certificates.
    pub fn require_current_broad_phase_traversal(self) -> Result<(), ExactMeshError> {
        self.broad_phase_traversal
            .require_current("broad-phase traversal summary")
    }

    /// Return retained broad-phase traversal counts after requiring current evidence.
    pub fn current_broad_phase_traversal_summary(
        self,
    ) -> Result<PreparedMeshPairBroadPhaseTraversalSummary, ExactMeshError> {
        self.require_current_broad_phase_traversal()?;
        self.retained_broad_phase_traversal_summary
            .ok_or_else(|| {
                ExactMeshError::one(ExactMeshBlocker::new(
                    ExactMeshBlockerKind::MissingRequiredEvidence,
                    "prepared mesh-pair session retained broad-phase traversal state without traversal counts",
                ))
            })
    }

    /// Return the certificate state for retained broad-phase candidate pairs.
    pub const fn candidate_face_pairs(self) -> PreparedMeshPairFactState {
        self.candidate_face_pairs
    }

    /// Require retained broad-phase candidate pairs with current certificates.
    pub fn require_current_candidate_face_pairs(self) -> Result<(), ExactMeshError> {
        self.candidate_face_pairs
            .require_current("broad-phase candidate face pairs")
    }

    /// Return retained broad-phase candidate pair count after requiring current evidence.
    pub fn current_candidate_face_pair_count(self) -> Result<usize, ExactMeshError> {
        self.current_broad_phase_traversal_summary()
            .map(PreparedMeshPairBroadPhaseTraversalSummary::candidate_pair_count)
    }

    /// Return the certificate state for coarse face-pair classifications.
    pub const fn face_pair_classifications(self) -> PreparedMeshPairFactState {
        self.face_pair_classifications
    }

    /// Return the certificate state for coarse face-pair classification counts.
    pub const fn face_pair_classification_counts(self) -> PreparedMeshPairFactState {
        self.face_pair_classification_counts
    }

    /// Require retained coarse face-pair classifications with current certificates.
    pub fn require_current_face_pair_classifications(self) -> Result<(), ExactMeshError> {
        self.face_pair_classifications
            .require_current("face-pair classification")
    }

    /// Return the retained coarse face-pair classification count after requiring current evidence.
    pub fn current_face_pair_classification_count(self) -> Result<usize, ExactMeshError> {
        self.current_face_pair_classification_counts()
            .map(PreparedMeshPairClassificationCounts::face_pair_count)
    }

    /// Return retained coarse face-pair decision counts after requiring current evidence.
    pub fn current_face_pair_classification_counts(
        self,
    ) -> Result<PreparedMeshPairClassificationCounts, ExactMeshError> {
        self.face_pair_classification_counts
            .require_current("face-pair classification counts")?;
        self.retained_face_pair_classification_counts
            .ok_or_else(|| {
                ExactMeshError::one(ExactMeshBlocker::new(
                    ExactMeshBlockerKind::MissingRequiredEvidence,
                    "prepared mesh-pair session retained face-pair classification evidence without decision counts",
                ))
            })
    }

    /// Require a retained exact intersection graph with a current replay certificate.
    pub fn require_current_intersection_graph(self) -> Result<(), ExactMeshError> {
        self.intersection_graph
            .require_current("intersection graph")
    }

    /// Return retained exact intersection graph counts after requiring a current certificate.
    pub fn current_intersection_graph_counts(
        self,
    ) -> Result<PreparedMeshPairIntersectionGraphCounts, ExactMeshError> {
        self.require_current_intersection_graph()?;
        self.retained_intersection_graph_counts.ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session retained an intersection graph certificate without graph counts",
            ))
        })
    }

    /// Return the certificate state for the retained prepared arrangement.
    pub const fn arrangement(self) -> PreparedMeshPairFactState {
        self.arrangement
    }

    /// Require a retained arrangement built from current certificates.
    pub fn require_current_arrangement(self) -> Result<(), ExactMeshError> {
        self.arrangement.require_current("arrangement")
    }

    /// Return retained arrangement topology counts after requiring current evidence.
    pub fn current_arrangement_counts(
        self,
    ) -> Result<PreparedMeshPairArrangementCounts, ExactMeshError> {
        self.require_current_arrangement()?;
        self.retained_arrangement_counts.ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session retained arrangement evidence without topology counts",
            ))
        })
    }

    /// Return the certificate state for arrangement shortcut facts.
    pub const fn arrangement_shortcut_facts(self) -> PreparedMeshPairFactState {
        self.arrangement_shortcut_facts
    }

    /// Require retained arrangement shortcut facts with current certificates.
    pub fn require_current_arrangement_shortcut_facts(self) -> Result<(), ExactMeshError> {
        self.arrangement_shortcut_facts
            .require_current("arrangement shortcut facts")
    }

    /// Return the certificate state for the prepared union result or error.
    pub const fn union_result(self) -> PreparedMeshPairFactState {
        self.union_result
    }

    /// Require a retained prepared union result or error.
    pub fn require_current_union_result(self) -> Result<(), ExactMeshError> {
        self.union_result.require_current("union result")
    }

    /// Return the retained prepared union outcome after requiring current evidence.
    pub fn current_union_result_outcome(
        self,
    ) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        self.current_result_outcome(self.union_result, self.union_result_outcome, "union result")
    }

    /// Return the certificate state for the prepared intersection result or error.
    pub const fn intersection_result(self) -> PreparedMeshPairFactState {
        self.intersection_result
    }

    /// Require a retained prepared intersection result or error.
    pub fn require_current_intersection_result(self) -> Result<(), ExactMeshError> {
        self.intersection_result
            .require_current("intersection result")
    }

    /// Return the retained prepared intersection outcome after requiring current evidence.
    pub fn current_intersection_result_outcome(
        self,
    ) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        self.current_result_outcome(
            self.intersection_result,
            self.intersection_result_outcome,
            "intersection result",
        )
    }

    /// Return the certificate state for the prepared difference result or error.
    pub const fn difference_result(self) -> PreparedMeshPairFactState {
        self.difference_result
    }

    /// Require a retained prepared difference result or error.
    pub fn require_current_difference_result(self) -> Result<(), ExactMeshError> {
        self.difference_result.require_current("difference result")
    }

    /// Return the retained prepared difference outcome after requiring current evidence.
    pub fn current_difference_result_outcome(
        self,
    ) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        self.current_result_outcome(
            self.difference_result,
            self.difference_result_outcome,
            "difference result",
        )
    }

    /// Return the certificate state for the prepared symmetric-difference result or error.
    pub const fn xor_result(self) -> PreparedMeshPairFactState {
        self.xor_result
    }

    /// Require a retained prepared symmetric-difference result or error.
    pub fn require_current_xor_result(self) -> Result<(), ExactMeshError> {
        self.xor_result.require_current("xor result")
    }

    /// Return the retained prepared symmetric-difference outcome after requiring current evidence.
    pub fn current_xor_result_outcome(
        self,
    ) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        self.current_result_outcome(self.xor_result, self.xor_result_outcome, "xor result")
    }

    fn current_result_outcome(
        self,
        state: PreparedMeshPairFactState,
        outcome: Option<PreparedMeshPairResultOutcome>,
        fact: &'static str,
    ) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        state.require_current(fact)?;
        outcome.ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                format!(
                    "prepared mesh-pair session retained {fact} evidence without an outcome summary"
                ),
            ))
        })
    }
}

impl<'a> ExactMeshRef<'a> {
    /// Borrow an exact mesh as a replayable view.
    pub(crate) const fn new(mesh: &'a ExactMesh) -> Self {
        Self { mesh }
    }

    pub(crate) const fn mesh(self) -> &'a ExactMesh {
        self.mesh
    }

    /// Return the retained source/freshness stamp for this exact mesh.
    pub fn source_stamp(self) -> ExactMeshSourceStamp {
        ExactMeshSourceStamp::from_mesh(self.mesh)
    }

    /// Return exact vertices.
    pub fn vertices(self) -> &'a [Point3] {
        self.mesh.vertices()
    }

    /// Borrow retained whole-mesh bounds as exact min/max corners.
    pub fn mesh_bounds(self) -> Option<(&'a Point3, &'a Point3)> {
        self.mesh.bounds().mesh().map(bounds_corners)
    }

    /// Borrow retained bounds for one face as exact min/max corners.
    pub fn face_bounds(self, index: usize) -> Option<(&'a Point3, &'a Point3)> {
        self.mesh.bounds().face(index).map(bounds_corners)
    }

    /// Borrow retained bounds for one face, returning a typed blocker when absent.
    pub fn require_face_bounds(
        self,
        index: usize,
    ) -> Result<(&'a Point3, &'a Point3), ExactMeshError> {
        self.face_bounds(index)
            .ok_or_else(|| missing_retained_face_bounds("face", index))
    }

    /// Borrow one vertex by index.
    pub fn vertex(self, index: usize) -> Option<VertexRef<'a>> {
        (index < self.mesh.vertices().len()).then_some(VertexRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Borrow one vertex by index, returning a typed blocker when absent.
    pub fn require_vertex(self, index: usize) -> Result<VertexRef<'a>, ExactMeshError> {
        self.vertex(index).ok_or_else(|| {
            ExactMeshError::one(
                ExactMeshBlocker::new(
                    ExactMeshBlockerKind::IndexOutOfBounds,
                    format!(
                        "mesh vertex index {index} is out of bounds for {} retained vertices",
                        self.vertex_count()
                    ),
                )
                .with_vertex(index),
            )
        })
    }

    /// Return copied triangle index rows.
    pub fn triangle_indices(self) -> impl ExactSizeIterator<Item = [usize; 3]> + 'a {
        self.mesh.triangle_indices()
    }

    /// Retained vertex count.
    pub const fn vertex_count(self) -> usize {
        self.mesh.facts().mesh.vertex_count
    }

    /// Retained face count.
    pub const fn face_count(self) -> usize {
        self.mesh.facts().mesh.face_count
    }

    /// Retained undirected edge count.
    pub const fn edge_count(self) -> usize {
        self.mesh.facts().mesh.edge_count
    }

    /// Retained Euler characteristic `V - E + F`.
    pub const fn euler_characteristic(self) -> isize {
        self.mesh.facts().mesh.euler_characteristic
    }

    /// Retained boundary edge count.
    pub const fn boundary_edge_count(self) -> usize {
        self.mesh.facts().mesh.boundary_edges
    }

    /// Retained non-manifold edge count.
    pub const fn non_manifold_edge_count(self) -> usize {
        self.mesh.facts().mesh.non_manifold_edges
    }

    /// Retained non-manifold vertex-link count.
    pub const fn non_manifold_vertex_count(self) -> usize {
        self.mesh.facts().mesh.non_manifold_vertices
    }

    /// Retained degenerate triangle count.
    pub const fn degenerate_triangle_count(self) -> usize {
        self.mesh.facts().mesh.degenerate_triangles
    }

    /// Whether retained facts certify a closed two-manifold mesh.
    pub const fn is_closed_manifold(self) -> bool {
        self.mesh.facts().mesh.closed_manifold
    }

    /// Whether retained facts record exact rational coordinates for every vertex.
    pub const fn has_exact_rational_coordinates(self) -> bool {
        self.mesh.facts().mesh.fixed_coordinates_exact_rational
    }

    /// Replay retained bounds, topology facts, and provenance against the source mesh.
    pub fn validate_retained_state(self) -> Result<(), ExactMeshError> {
        self.mesh.validate_retained_state()
    }

    /// Replay retained exact bounds against the source mesh.
    pub fn validate_retained_bounds(self) -> Result<(), ExactMeshError> {
        self.mesh.validate_retained_bounds()
    }

    /// Validate retained exact bounds without recomputing them.
    pub fn validate_retained_bounds_certificate(self) -> Result<(), ExactMeshError> {
        self.mesh.validate_retained_bounds_certificate()
    }

    /// Borrow one face by index.
    pub fn face(self, index: usize) -> Option<FaceRef<'a>> {
        (index < self.mesh.triangles().len()).then_some(FaceRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Borrow one face by index, returning a typed blocker when absent.
    pub fn require_face(self, index: usize) -> Result<FaceRef<'a>, ExactMeshError> {
        self.face(index).ok_or_else(|| {
            ExactMeshError::one(
                ExactMeshBlocker::new(
                    ExactMeshBlockerKind::IndexOutOfBounds,
                    format!(
                        "mesh face index {index} is out of bounds for {} retained faces",
                        self.face_count()
                    ),
                )
                .with_face(index),
            )
        })
    }

    /// Borrow one triangle by index.
    pub fn triangle(self, index: usize) -> Option<TriangleRef<'a>> {
        (index < self.mesh.triangles().len()).then_some(TriangleRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Borrow one triangle by index, returning a typed blocker when absent.
    pub fn require_triangle(self, index: usize) -> Result<TriangleRef<'a>, ExactMeshError> {
        self.triangle(index).ok_or_else(|| {
            ExactMeshError::one(
                ExactMeshBlocker::new(
                    ExactMeshBlockerKind::IndexOutOfBounds,
                    format!(
                        "mesh triangle index {index} is out of bounds for {} retained faces",
                        self.face_count()
                    ),
                )
                .with_face(index),
            )
        })
    }

    /// Borrow one retained edge by index.
    pub fn edge(self, index: usize) -> Option<EdgeRef<'a>> {
        (index < self.mesh.facts().edges.len()).then_some(EdgeRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Borrow one retained edge by index, returning a typed blocker when absent.
    pub fn require_edge(self, index: usize) -> Result<EdgeRef<'a>, ExactMeshError> {
        self.edge(index).ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::IndexOutOfBounds,
                format!(
                    "mesh edge index {index} is out of bounds for {} retained edges",
                    self.edge_count()
                ),
            ))
        })
    }

    /// Iterate borrowed vertices.
    pub fn vertex_refs(self) -> impl Iterator<Item = VertexRef<'a>> + 'a {
        (0..self.mesh.vertices().len()).map(move |index| VertexRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Iterate borrowed faces.
    pub fn faces(self) -> impl Iterator<Item = FaceRef<'a>> + 'a {
        (0..self.mesh.triangles().len()).map(move |index| FaceRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Iterate borrowed triangles.
    pub fn triangle_refs(self) -> impl Iterator<Item = TriangleRef<'a>> + 'a {
        (0..self.mesh.triangles().len()).map(move |index| TriangleRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Iterate retained edges.
    pub fn edges(self) -> impl Iterator<Item = EdgeRef<'a>> + 'a {
        (0..self.mesh.facts().edges.len()).map(move |index| EdgeRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Prepare certificate-validated broad-phase facts for repeated pair queries.
    pub fn prepare_broad_phase(self) -> Result<PreparedMeshView<'a>, ExactMeshError> {
        self.validate_retained_bounds_certificate()?;
        Ok(self.prepare_broad_phase_after_certificate())
    }

    /// Prepare certificate-validated broad-phase facts for this mesh pair.
    pub fn prepare_broad_phase_pair<'b>(
        self,
        right: ExactMeshRef<'b>,
    ) -> Result<PreparedMeshPair<'a, 'b>, ExactMeshError> {
        let left = self.prepare_broad_phase()?;
        let right = right.prepare_broad_phase()?;
        Ok(PreparedMeshPair::new(left, right))
    }

    pub(crate) fn prepare_broad_phase_after_certificate(self) -> PreparedMeshView<'a> {
        PreparedMeshView {
            view: self,
            bounds: self.mesh.bounds().prepare(),
        }
    }

    /// Visit broad-phase candidate face pairs after certificate-validating both meshes.
    pub fn visit_candidate_face_pairs<'b>(
        self,
        right: ExactMeshRef<'b>,
        visit: &mut impl FnMut([usize; 2]),
    ) -> Result<(), ExactMeshError> {
        self.validate_retained_bounds_certificate()?;
        right.validate_retained_bounds_certificate()?;
        let result = ExactAabbBroadPhase::default().try_visit_candidate_face_pairs_one_shot(
            self.mesh.bounds(),
            right.mesh.bounds(),
            &mut |pair| {
                visit(pair);
                Ok::<(), ()>(())
            },
        );
        debug_assert!(result.is_ok());
        Ok(())
    }

    /// Materialize this view after a row-major exact homogeneous affine transform.
    pub fn transform(self, matrix: [[Real; 4]; 4]) -> Result<ExactMesh, ExactMeshError> {
        self.mesh.transform(matrix)
    }

    /// Materialize this view with every triangle orientation reversed.
    pub fn inverse(self) -> Result<ExactMesh, ExactMeshError> {
        self.mesh.inverse()
    }

    /// Materialize the exact closed union of this view and `right`.
    pub fn union(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.prepare_broad_phase_pair(right)?.union()
    }

    /// Materialize the exact closed intersection of this view and `right`.
    pub fn intersection(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.prepare_broad_phase_pair(right)?.intersection()
    }

    /// Materialize the exact closed difference of this view minus `right`.
    pub fn difference(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.prepare_broad_phase_pair(right)?.difference()
    }

    /// Materialize the exact closed symmetric difference of this view and `right`.
    pub fn xor(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.prepare_broad_phase_pair(right)?.xor()
    }
}

impl<'a> PreparedMeshView<'a> {
    /// Return the underlying borrowed mesh view.
    pub const fn view(&self) -> ExactMeshRef<'a> {
        self.view
    }

    /// Borrow retained whole-mesh bounds as exact min/max corners.
    pub fn mesh_bounds(&self) -> Option<(&'a Point3, &'a Point3)> {
        self.view.mesh_bounds()
    }

    /// Prepare a certificate-validated pair view that reuses its broad-phase plan.
    pub fn pair_with<'pair, 'right>(
        &'pair self,
        right: &'pair PreparedMeshView<'right>,
    ) -> PreparedMeshPairView<'pair, 'a, 'right> {
        let broad_phase = ExactAabbBroadPhase::default();
        let plan = broad_phase.candidate_face_pair_plan(&self.bounds, &right.bounds);
        let candidate_pair_capacity_hint =
            plan.bounded_capacity_hint(self.view.face_count(), right.view.face_count());
        let broad_phase_summary = PreparedMeshPairBroadPhaseSummary::from_plan(
            self.view.source_stamp(),
            right.view.source_stamp(),
            plan,
            self.view.face_count(),
            right.view.face_count(),
            candidate_pair_capacity_hint,
        );
        PreparedMeshPairView {
            left: self,
            right,
            plan,
            broad_phase_summary,
            candidate_pair_capacity_hint,
        }
    }

    /// Visit certificate-validated broad-phase candidate face pairs.
    pub fn visit_candidate_face_pairs<'b>(
        &self,
        right: &PreparedMeshView<'b>,
        visit: &mut impl FnMut([usize; 2]),
    ) {
        self.pair_with(right).visit_candidate_face_pairs(visit);
    }

    /// Visit certificate-validated candidate face pairs and allow the visitor to stop early.
    pub fn try_visit_candidate_face_pairs<'b, E>(
        &self,
        right: &PreparedMeshView<'b>,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<(), E> {
        self.pair_with(right).try_visit_candidate_face_pairs(visit)
    }
}

impl<'left, 'right> PreparedMeshPair<'left, 'right> {
    fn new(left: PreparedMeshView<'left>, right: PreparedMeshView<'right>) -> Self {
        let broad_phase = ExactAabbBroadPhase::default();
        let plan = broad_phase.candidate_face_pair_plan(&left.bounds, &right.bounds);
        let candidate_pair_capacity_hint =
            plan.bounded_capacity_hint(left.view.face_count(), right.view.face_count());
        let broad_phase_summary = PreparedMeshPairBroadPhaseSummary::from_plan(
            left.view.source_stamp(),
            right.view.source_stamp(),
            plan,
            left.view.face_count(),
            right.view.face_count(),
            candidate_pair_capacity_hint,
        );
        Self {
            left,
            right,
            plan,
            broad_phase_summary,
            candidate_pair_capacity_hint,
            scratch: RefCell::new(BroadPhaseScratch::default()),
            candidate_face_pairs: RefCell::new(None),
            broad_phase_traversal_summary: RefCell::new(None),
            face_pair_classifications: RefCell::new(None),
            face_pair_classification_counts: RefCell::new(None),
            intersection_graph: RefCell::new(None),
            intersection_graph_counts: RefCell::new(None),
            intersection_graph_validated: RefCell::new(false),
            arrangement: RefCell::new(None),
            arrangement_counts: RefCell::new(None),
            arrangement_shortcut_facts: RefCell::new(None),
            union_result: RefCell::new(None),
            union_result_outcome: RefCell::new(None),
            intersection_result: RefCell::new(None),
            intersection_result_outcome: RefCell::new(None),
            difference_result: RefCell::new(None),
            difference_result_outcome: RefCell::new(None),
            xor_result: RefCell::new(None),
            xor_result_outcome: RefCell::new(None),
        }
    }

    /// Return the left prepared mesh view.
    pub const fn left(&self) -> &PreparedMeshView<'left> {
        &self.left
    }

    /// Return the right prepared mesh view.
    pub const fn right(&self) -> &PreparedMeshView<'right> {
        &self.right
    }

    /// Borrow this pair cache as a lightweight pair view.
    pub const fn as_view(&self) -> PreparedMeshPairView<'_, 'left, 'right> {
        PreparedMeshPairView {
            left: &self.left,
            right: &self.right,
            plan: self.plan,
            broad_phase_summary: self.broad_phase_summary,
            candidate_pair_capacity_hint: self.candidate_pair_capacity_hint,
        }
    }

    /// Return a bounded storage hint for candidate face-pair traversal.
    pub const fn candidate_face_pair_capacity_hint(&self) -> usize {
        self.candidate_pair_capacity_hint
    }

    /// Return retained broad-phase planning provenance for this pair session.
    pub const fn broad_phase_summary(&self) -> PreparedMeshPairBroadPhaseSummary {
        self.broad_phase_summary
    }

    /// Return the retained broad-phase plan kind.
    pub const fn candidate_pair_plan(&self) -> PreparedMeshPairPlanKind {
        PreparedMeshPairPlanKind::from_candidate_plan(self.plan)
    }

    /// Return whether retained pair-source stamps still match the borrowed meshes.
    pub fn sources_are_current(&self) -> bool {
        self.broad_phase_summary.left_source() == self.left.view.source_stamp()
            && self.broad_phase_summary.right_source() == self.right.view.source_stamp()
    }

    /// Return whether broad-phase traversal counts have been retained.
    pub fn has_retained_broad_phase_traversal_summary(&self) -> bool {
        self.broad_phase_traversal_summary.borrow().is_some()
    }

    /// Return whether broad-phase traversal counts are retained and source-current.
    pub fn broad_phase_traversal_summary_is_current(&self) -> bool {
        self.cache_status().broad_phase_traversal().is_current()
    }

    /// Return retained broad-phase traversal counts, if present.
    pub fn retained_broad_phase_traversal_summary(
        &self,
    ) -> Option<PreparedMeshPairBroadPhaseTraversalSummary> {
        *self.broad_phase_traversal_summary.borrow()
    }

    /// Return retained broad-phase rejection count, if traversal counts are present.
    pub fn retained_broad_phase_rejection_count(&self) -> Option<usize> {
        self.retained_broad_phase_traversal_summary()
            .map(PreparedMeshPairBroadPhaseTraversalSummary::broad_phase_rejection_count)
    }

    /// Return retained broad-phase candidate upper-bound slack, if present.
    pub fn retained_candidate_upper_bound_slack(&self) -> Option<usize> {
        self.retained_broad_phase_traversal_summary()
            .map(PreparedMeshPairBroadPhaseTraversalSummary::candidate_upper_bound_slack)
    }

    /// Return whether the retained broad-phase candidate upper bound saturated, if present.
    pub fn retained_candidate_upper_bound_saturated(&self) -> Option<bool> {
        self.retained_broad_phase_traversal_summary()
            .map(PreparedMeshPairBroadPhaseTraversalSummary::candidate_upper_bound_saturated)
    }

    /// Require retained broad-phase traversal counts with current source certificates.
    pub fn current_broad_phase_traversal_summary(
        &self,
    ) -> Result<PreparedMeshPairBroadPhaseTraversalSummary, ExactMeshError> {
        self.cache_status().current_broad_phase_traversal_summary()
    }

    /// Return whether candidate face-pair records have been retained.
    pub fn has_retained_candidate_face_pairs(&self) -> bool {
        self.candidate_face_pairs.borrow().is_some()
    }

    /// Return whether candidate face-pair records are retained and source-current.
    pub fn candidate_face_pairs_are_current(&self) -> bool {
        self.cache_status().candidate_face_pairs().is_current()
    }

    /// Execute and retain broad-phase traversal counts without storing candidate records.
    pub fn prepare_broad_phase_traversal_summary(
        &self,
    ) -> PreparedMeshPairBroadPhaseTraversalSummary {
        if let Some(summary) = *self.broad_phase_traversal_summary.borrow() {
            return summary;
        }

        let mut candidate_pair_count = 0usize;
        let result = self.try_visit_candidate_face_pairs_uncached(&mut |_| {
            candidate_pair_count = candidate_pair_count.saturating_add(1);
            Ok::<(), ()>(())
        });
        debug_assert!(result.is_ok());
        self.retain_broad_phase_traversal_count(candidate_pair_count)
    }

    /// Build and retain broad-phase candidate face pairs, returning the retained count.
    pub fn prepare_candidate_face_pairs(&self) -> usize {
        self.ensure_candidate_face_pairs();
        self.candidate_face_pairs
            .borrow()
            .as_ref()
            .map_or(0, Vec::len)
    }

    /// Build and retain coarse face-pair classification records, returning the retained count.
    pub fn prepare_face_pair_classifications(&self) -> usize {
        self.ensure_face_pair_classifications();
        self.face_pair_classifications
            .borrow()
            .as_ref()
            .map_or(0, Vec::len)
    }

    /// Build and retain coarse face-pair classification counts without storing records.
    pub fn prepare_face_pair_classification_counts(&self) -> PreparedMeshPairClassificationCounts {
        self.ensure_face_pair_classification_counts();
        self.face_pair_classification_counts
            .borrow()
            .as_ref()
            .copied()
            .expect("prepared mesh-pair session did not retain face-pair classification counts after preparation")
    }

    /// Return a cheap summary of retained facts in this prepared pair session.
    fn cache_status(&self) -> PreparedMeshPairCacheStatus {
        let sources_current = self.sources_are_current();
        let candidate_face_pairs_retained = self.candidate_face_pairs.borrow().is_some();
        let broad_phase_traversal_summary = *self.broad_phase_traversal_summary.borrow();
        let face_pair_classifications_retained = self.face_pair_classifications.borrow().is_some();
        let face_pair_classification_counts = *self.face_pair_classification_counts.borrow();
        let graph_counts = *self.intersection_graph_counts.borrow();
        let graph_retained = self.intersection_graph.borrow().is_some();
        let arrangement_retained = self.arrangement.borrow().is_some();
        let union_retained = self.union_result.borrow().is_some();
        let intersection_retained = self.intersection_result.borrow().is_some();
        let difference_retained = self.difference_result.borrow().is_some();
        let xor_retained = self.xor_result.borrow().is_some();
        PreparedMeshPairCacheStatus {
            source_pair: source_pair_state(sources_current),
            broad_phase_traversal: retained_current_state(
                broad_phase_traversal_summary.is_some(),
                sources_current,
            ),
            retained_broad_phase_traversal_summary: broad_phase_traversal_summary,
            candidate_face_pairs: retained_current_state(
                candidate_face_pairs_retained,
                sources_current,
            ),
            face_pair_classifications: retained_current_state(
                face_pair_classifications_retained,
                sources_current,
            ),
            face_pair_classification_counts: retained_current_state(
                face_pair_classification_counts.is_some(),
                sources_current,
            ),
            retained_face_pair_classification_counts: face_pair_classification_counts,
            intersection_graph: if graph_retained {
                retained_certificate_state(
                    *self.intersection_graph_validated.borrow(),
                    sources_current,
                )
            } else {
                PreparedMeshPairFactState::Missing
            },
            retained_intersection_graph_counts: graph_counts,
            arrangement: retained_current_state(arrangement_retained, sources_current),
            retained_arrangement_counts: *self.arrangement_counts.borrow(),
            arrangement_shortcut_facts: retained_current_state(
                self.arrangement_shortcut_facts.borrow().is_some(),
                sources_current,
            ),
            union_result: retained_current_state(union_retained, sources_current),
            union_result_outcome: *self.union_result_outcome.borrow(),
            intersection_result: retained_current_state(intersection_retained, sources_current),
            intersection_result_outcome: *self.intersection_result_outcome.borrow(),
            difference_result: retained_current_state(difference_retained, sources_current),
            difference_result_outcome: *self.difference_result_outcome.borrow(),
            xor_result: retained_current_state(xor_retained, sources_current),
            xor_result_outcome: *self.xor_result_outcome.borrow(),
        }
    }

    /// Require this pair session's source stamps to match its current mesh views.
    pub fn require_current_sources(&self) -> Result<(), ExactMeshError> {
        self.cache_status().require_current_sources()
    }

    fn intersection_graph_state(&self) -> PreparedMeshPairFactState {
        if self.intersection_graph.borrow().is_none() {
            PreparedMeshPairFactState::Missing
        } else {
            retained_certificate_state(
                *self.intersection_graph_validated.borrow(),
                self.sources_are_current(),
            )
        }
    }

    /// Return retained exact intersection graph counts after requiring a current certificate.
    pub fn current_intersection_graph_counts(
        &self,
    ) -> Result<PreparedMeshPairIntersectionGraphCounts, ExactMeshError> {
        self.cache_status().current_intersection_graph_counts()
    }

    /// Require a retained exact intersection graph with a current source certificate.
    pub fn require_current_intersection_graph(&self) -> Result<(), ExactMeshError> {
        self.cache_status().require_current_intersection_graph()
    }

    /// Return the retained broad-phase candidate pair count after requiring current evidence.
    pub fn current_candidate_face_pair_count(&self) -> Result<usize, ExactMeshError> {
        self.cache_status().current_candidate_face_pair_count()
    }

    /// Return whether retained face-pair classification records are present.
    pub fn has_retained_face_pair_classifications(&self) -> bool {
        self.face_pair_classifications.borrow().is_some()
    }

    /// Return whether retained face-pair classification records are source-current.
    pub fn face_pair_classifications_are_current(&self) -> bool {
        self.cache_status().face_pair_classifications().is_current()
    }

    /// Return whether retained face-pair classification counts are present.
    pub fn has_retained_face_pair_classification_counts(&self) -> bool {
        self.face_pair_classification_counts.borrow().is_some()
    }

    /// Return whether retained face-pair classification counts are source-current.
    pub fn face_pair_classification_counts_are_current(&self) -> bool {
        self.cache_status()
            .face_pair_classification_counts()
            .is_current()
    }

    /// Return retained face-pair classification record count, if records are present.
    pub fn retained_face_pair_classification_count(&self) -> Option<usize> {
        self.face_pair_classifications
            .borrow()
            .as_ref()
            .map(Vec::len)
    }

    /// Return retained face-pair classification decision counts, if present.
    pub fn retained_face_pair_classification_counts(
        &self,
    ) -> Option<PreparedMeshPairClassificationCounts> {
        *self.face_pair_classification_counts.borrow()
    }

    /// Borrow retained broad-phase candidate pairs without rebuilding missing evidence.
    pub fn with_current_candidate_face_pairs<R>(
        &self,
        query: impl FnOnce(&[[usize; 2]]) -> R,
    ) -> Result<R, ExactMeshError> {
        self.require_current_candidate_face_pairs()?;
        let candidate_face_pairs = self.candidate_face_pairs.borrow();
        let pairs = candidate_face_pairs.as_deref().ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session retained broad-phase candidate-pair state without candidate records",
            ))
        })?;
        Ok(query(pairs))
    }

    /// Require retained broad-phase candidate face pairs with current certificates.
    pub fn require_current_candidate_face_pairs(&self) -> Result<(), ExactMeshError> {
        self.cache_status().require_current_candidate_face_pairs()
    }

    /// Return retained arrangement topology counts after requiring current evidence.
    pub fn current_arrangement_counts(
        &self,
    ) -> Result<PreparedMeshPairArrangementCounts, ExactMeshError> {
        self.cache_status().current_arrangement_counts()
    }

    /// Return retained arrangement topology counts, if present.
    pub fn retained_arrangement_counts(&self) -> Option<PreparedMeshPairArrangementCounts> {
        *self.arrangement_counts.borrow()
    }

    /// Return whether arrangement records are present.
    pub fn has_retained_arrangement(&self) -> bool {
        self.arrangement.borrow().is_some()
    }

    /// Return whether the retained arrangement is present and source-current.
    pub fn arrangement_is_current(&self) -> bool {
        self.cache_status().arrangement().is_current()
    }

    /// Return whether arrangement shortcut facts are retained and source-current.
    pub fn arrangement_shortcut_facts_are_current(&self) -> bool {
        self.cache_status()
            .arrangement_shortcut_facts()
            .is_current()
    }

    /// Return whether arrangement shortcut facts are present.
    pub fn has_retained_arrangement_shortcut_facts(&self) -> bool {
        self.arrangement_shortcut_facts.borrow().is_some()
    }

    /// Require a retained arrangement with current source certificates.
    pub fn require_current_arrangement(&self) -> Result<(), ExactMeshError> {
        self.cache_status().require_current_arrangement()
    }

    /// Build and retain the exact arrangement, returning retained topology counts.
    pub fn prepare_arrangement(&self) -> Result<PreparedMeshPairArrangementCounts, ExactMeshError> {
        self.retained_arrangement()?;
        self.current_arrangement_counts()
    }

    /// Build and retain arrangement shortcut facts for this prepared pair.
    pub fn prepare_arrangement_shortcut_facts(&self) -> Result<(), ExactMeshError> {
        self.arrangement_cell_complex_shortcut_facts();
        self.require_current_arrangement_shortcut_facts()
    }

    /// Require retained arrangement shortcut facts with current source certificates.
    pub fn require_current_arrangement_shortcut_facts(&self) -> Result<(), ExactMeshError> {
        self.cache_status()
            .require_current_arrangement_shortcut_facts()
    }

    /// Return the retained coarse face-pair classification count after requiring current evidence.
    pub fn current_face_pair_classification_count(&self) -> Result<usize, ExactMeshError> {
        self.cache_status().current_face_pair_classification_count()
    }

    /// Return retained coarse face-pair decision counts after requiring current evidence.
    pub fn current_face_pair_classification_counts(
        &self,
    ) -> Result<PreparedMeshPairClassificationCounts, ExactMeshError> {
        self.cache_status()
            .current_face_pair_classification_counts()
    }

    /// Require retained coarse face-pair classifications with current certificates.
    pub fn require_current_face_pair_classifications(&self) -> Result<(), ExactMeshError> {
        self.cache_status()
            .require_current_face_pair_classifications()
    }

    /// Return retained intersection graph counts, if present.
    pub fn retained_intersection_graph_counts(
        &self,
    ) -> Option<PreparedMeshPairIntersectionGraphCounts> {
        *self.intersection_graph_counts.borrow()
    }

    /// Return whether an intersection graph is retained.
    pub fn has_retained_intersection_graph(&self) -> bool {
        self.intersection_graph.borrow().is_some()
    }

    /// Return retained intersection graph face-pair count, if present.
    pub fn retained_intersection_graph_face_pair_count(&self) -> Option<usize> {
        self.retained_intersection_graph_counts()
            .map(|counts| counts.face_pair_count())
    }

    /// Return retained intersection graph event count, if present.
    pub fn retained_intersection_graph_event_count(&self) -> Option<usize> {
        self.retained_intersection_graph_counts()
            .map(|counts| counts.event_count())
    }

    /// Return whether the retained intersection graph is source-current.
    pub fn intersection_graph_is_current(&self) -> bool {
        self.intersection_graph_state().is_current()
    }

    /// Return whether retained intersection graph source certification is blocked.
    pub fn intersection_graph_is_certificate_blocked(&self) -> bool {
        self.intersection_graph_state().is_certificate_blocked()
    }

    /// Build and retain the exact intersection graph without certifying source replay.
    ///
    /// The returned counts are retained for later status checks, but the graph
    /// remains certificate-blocked until [`Self::prepare_current_intersection_graph`]
    /// validates its retained source handles.
    pub fn prepare_intersection_graph(
        &self,
    ) -> Result<PreparedMeshPairIntersectionGraphCounts, ExactMeshError> {
        let graph = build_unvalidated_intersection_graph_from_prepared_pair_rc(self)?;
        Ok(PreparedMeshPairIntersectionGraphCounts::from_graph(&graph))
    }

    /// Build, retain, and source-certify the exact intersection graph.
    pub fn prepare_current_intersection_graph(
        &self,
    ) -> Result<PreparedMeshPairIntersectionGraphCounts, ExactMeshError> {
        build_validated_intersection_graph_from_prepared_pair(self)?;
        self.current_intersection_graph_counts()
    }

    /// Build a retained arrangement from this pair session and run `query` on its borrowed view.
    ///
    /// The pair's retained intersection graph is source-certified first. The
    /// arrangement builder then consumes that current graph certificate instead
    /// of replay-building the graph from the source meshes.
    pub fn with_arrangement_view<R>(
        &self,
        query: impl for<'a> FnOnce(ArrangementView<'a>) -> R,
    ) -> Result<R, ExactMeshError> {
        let arrangement = self.retained_arrangement()?;
        Ok(query(arrangement.view()))
    }

    /// Query a retained arrangement view without rebuilding missing evidence.
    pub fn with_current_arrangement_view<R>(
        &self,
        query: impl for<'a> FnOnce(ArrangementView<'a>) -> R,
    ) -> Result<R, ExactMeshError> {
        self.require_current_arrangement()?;
        let arrangement = self.arrangement.borrow();
        let arrangement = arrangement.as_ref().ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session retained arrangement state without arrangement records",
            ))
        })?;
        Ok(query(arrangement.view()))
    }

    fn retained_arrangement(&self) -> Result<Rc<ExactArrangement>, ExactMeshError> {
        if let Some(arrangement) = self.arrangement.borrow().clone() {
            return Ok(arrangement);
        }

        let graph = build_validated_intersection_graph_from_prepared_pair(self)?;
        let arrangement = ExactArrangement::from_source_certified_intersection_graph_with_policy(
            graph.as_ref().clone(),
            self.left.view().mesh(),
            self.right.view().mesh(),
            ExactRegularizationPolicy::default(),
        )?;
        let counts = PreparedMeshPairArrangementCounts::from_arrangement(&arrangement);
        let arrangement = Rc::new(arrangement);
        *self.arrangement.borrow_mut() = Some(Rc::clone(&arrangement));
        *self.arrangement_counts.borrow_mut() = Some(counts);
        Ok(arrangement)
    }

    /// Visit retained coarse face-pair classifications for this prepared mesh pair.
    #[cfg(test)]
    pub(crate) fn try_visit_face_pair_classifications<E>(
        &self,
        visit: &mut impl FnMut(&MeshFacePairClassification) -> Result<(), E>,
    ) -> Result<(), E> {
        self.ensure_face_pair_classifications();
        let classifications = self.face_pair_classifications.borrow();
        for classification in classifications.as_deref().unwrap_or(&[]) {
            visit(classification)?;
        }
        Ok(())
    }

    pub(crate) fn with_current_face_pair_classifications<R>(
        &self,
        query: impl FnOnce(&[MeshFacePairClassification]) -> R,
    ) -> Result<R, ExactMeshError> {
        self.require_current_face_pair_classifications()?;
        let classifications = self.face_pair_classifications.borrow();
        let classifications = classifications.as_deref().ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session retained face-pair classification state without classification records",
            ))
        })?;
        Ok(query(classifications))
    }

    pub(crate) fn retain_broad_phase_traversal_count(
        &self,
        candidate_pair_count: usize,
    ) -> PreparedMeshPairBroadPhaseTraversalSummary {
        if let Some(summary) = *self.broad_phase_traversal_summary.borrow() {
            return summary;
        }
        let summary = PreparedMeshPairBroadPhaseTraversalSummary::from_broad_phase(
            self.broad_phase_summary,
            candidate_pair_count,
        );
        *self.broad_phase_traversal_summary.borrow_mut() = Some(summary);
        summary
    }

    pub(crate) fn retain_face_pair_classification_counts(
        &self,
        counts: PreparedMeshPairClassificationCounts,
    ) -> PreparedMeshPairClassificationCounts {
        if let Some(retained) = *self.face_pair_classification_counts.borrow() {
            return retained;
        }
        *self.face_pair_classification_counts.borrow_mut() = Some(counts);
        counts
    }

    fn ensure_face_pair_classification_counts(&self) {
        if self.face_pair_classification_counts.borrow().is_some() {
            return;
        }

        let mut counts = PreparedMeshPairClassificationCounts::default();
        let mut candidate_pair_count = 0usize;
        if let Some(candidate_face_pairs) = self.candidate_face_pairs.borrow().as_deref() {
            candidate_pair_count = candidate_face_pairs.len();
            for &[left_face, right_face] in candidate_face_pairs {
                let classification = classify_mesh_face_pair_unchecked(
                    self.left.view.mesh,
                    left_face,
                    self.right.view.mesh,
                    right_face,
                );
                counts.record(&classification);
            }
        } else {
            let result =
                self.try_visit_candidate_face_pairs_uncached(&mut |[left_face, right_face]| {
                    candidate_pair_count = candidate_pair_count.saturating_add(1);
                    let classification = classify_mesh_face_pair_unchecked(
                        self.left.view.mesh,
                        left_face,
                        self.right.view.mesh,
                        right_face,
                    );
                    counts.record(&classification);
                    Ok::<(), ()>(())
                });
            debug_assert!(result.is_ok());
        }
        self.retain_broad_phase_traversal_count(candidate_pair_count);
        self.retain_face_pair_classification_counts(counts);
    }

    fn ensure_face_pair_classifications(&self) {
        if self.face_pair_classifications.borrow().is_some() {
            return;
        }

        let mut classifications = Vec::with_capacity(self.candidate_face_pair_capacity_hint());
        let mut candidate_pair_count = 0usize;
        if let Some(candidate_face_pairs) = self.candidate_face_pairs.borrow().as_deref() {
            candidate_pair_count = candidate_face_pairs.len();
            for &[left_face, right_face] in candidate_face_pairs {
                classifications.push(classify_mesh_face_pair_unchecked(
                    self.left.view.mesh,
                    left_face,
                    self.right.view.mesh,
                    right_face,
                ));
            }
        } else {
            let result =
                self.try_visit_candidate_face_pairs_uncached(&mut |[left_face, right_face]| {
                    candidate_pair_count = candidate_pair_count.saturating_add(1);
                    classifications.push(classify_mesh_face_pair_unchecked(
                        self.left.view.mesh,
                        left_face,
                        self.right.view.mesh,
                        right_face,
                    ));
                    Ok::<(), ()>(())
                });
            debug_assert!(result.is_ok());
        }
        self.retain_broad_phase_traversal_count(candidate_pair_count);
        let counts = PreparedMeshPairClassificationCounts::from_classifications(&classifications);
        *self.face_pair_classifications.borrow_mut() = Some(classifications);
        *self.face_pair_classification_counts.borrow_mut() = Some(counts);
    }

    pub(crate) fn cached_intersection_graph(&self) -> Option<Rc<ExactIntersectionGraph>> {
        self.intersection_graph.borrow().clone()
    }

    pub(crate) fn cached_arrangement(&self) -> Option<Rc<ExactArrangement>> {
        self.arrangement.borrow().clone()
    }

    pub(crate) fn retain_intersection_graph(
        &self,
        graph: ExactIntersectionGraph,
    ) -> Rc<ExactIntersectionGraph> {
        let counts = PreparedMeshPairIntersectionGraphCounts::from_graph(&graph);
        let graph = Rc::new(graph);
        *self.intersection_graph.borrow_mut() = Some(Rc::clone(&graph));
        *self.intersection_graph_counts.borrow_mut() = Some(counts);
        *self.intersection_graph_validated.borrow_mut() = false;
        self.clear_graph_dependent_retained_facts();
        graph
    }

    fn clear_graph_dependent_retained_facts(&self) {
        *self.arrangement.borrow_mut() = None;
        *self.arrangement_counts.borrow_mut() = None;
        *self.union_result.borrow_mut() = None;
        *self.union_result_outcome.borrow_mut() = None;
        *self.intersection_result.borrow_mut() = None;
        *self.intersection_result_outcome.borrow_mut() = None;
        *self.difference_result.borrow_mut() = None;
        *self.difference_result_outcome.borrow_mut() = None;
        *self.xor_result.borrow_mut() = None;
        *self.xor_result_outcome.borrow_mut() = None;
    }

    #[cfg(test)]
    pub(crate) fn has_validated_intersection_graph(&self) -> bool {
        self.intersection_graph_state() == PreparedMeshPairFactState::Current
    }

    pub(crate) fn certify_intersection_graph_source_replay(&self) {
        *self.intersection_graph_validated.borrow_mut() = true;
    }

    pub(crate) fn arrangement_cell_complex_shortcut_facts(
        &self,
    ) -> ExactArrangementCellComplexShortcutFacts {
        if let Some(facts) = self.arrangement_shortcut_facts.borrow().clone() {
            return facts;
        }
        let facts = ExactArrangementCellComplexShortcutFacts::from_sources(
            self.left.view().mesh(),
            self.right.view().mesh(),
        );
        *self.arrangement_shortcut_facts.borrow_mut() = Some(facts.clone());
        facts
    }

    #[cfg(test)]
    pub(crate) fn has_cached_intersection_graph(&self) -> bool {
        self.intersection_graph.borrow().is_some()
    }

    #[cfg(test)]
    pub(crate) fn has_cached_arrangement_shortcut_facts(&self) -> bool {
        self.arrangement_shortcut_facts.borrow().is_some()
    }

    /// Materialize the exact closed union using this retained pair session.
    pub fn union(&self) -> Result<ExactMesh, ExactMeshError> {
        self.named_boolean_mesh(ExactBooleanOperation::Union)
    }

    /// Retain the exact closed union result or blocker summary for this pair session.
    pub fn prepare_union_result(&self) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        let _ = self.union();
        self.current_union_result_outcome()
    }

    /// Return the retained union result or cached error without materializing it.
    pub fn current_union_result(&self) -> Result<ExactMesh, ExactMeshError> {
        self.current_named_boolean_mesh(ExactBooleanOperation::Union)
    }

    /// Return the retained union outcome without materializing the mesh.
    pub fn current_union_result_outcome(
        &self,
    ) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        self.cache_status().current_union_result_outcome()
    }

    /// Return whether a current retained union result or blocker exists.
    pub fn union_result_is_current(&self) -> bool {
        self.cache_status().union_result().is_current()
    }

    /// Return the retained union outcome summary, if present.
    pub fn retained_union_result_outcome(&self) -> Option<PreparedMeshPairResultOutcome> {
        *self.union_result_outcome.borrow()
    }

    /// Return whether a union result or blocker has been retained.
    pub fn has_retained_union_result(&self) -> bool {
        self.union_result.borrow().is_some()
    }

    /// Require a retained union result or retained union blocker.
    pub fn require_current_union_result(&self) -> Result<(), ExactMeshError> {
        self.cache_status().require_current_union_result()
    }

    /// Materialize the exact closed intersection using this retained pair session.
    pub fn intersection(&self) -> Result<ExactMesh, ExactMeshError> {
        self.named_boolean_mesh(ExactBooleanOperation::Intersection)
    }

    /// Retain the exact closed intersection result or blocker summary for this pair session.
    pub fn prepare_intersection_result(
        &self,
    ) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        let _ = self.intersection();
        self.current_intersection_result_outcome()
    }

    /// Return the retained intersection result or cached error without materializing it.
    pub fn current_intersection_result(&self) -> Result<ExactMesh, ExactMeshError> {
        self.current_named_boolean_mesh(ExactBooleanOperation::Intersection)
    }

    /// Return the retained intersection outcome without materializing the mesh.
    pub fn current_intersection_result_outcome(
        &self,
    ) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        self.cache_status().current_intersection_result_outcome()
    }

    /// Return whether a current retained intersection result or blocker exists.
    pub fn intersection_result_is_current(&self) -> bool {
        self.cache_status().intersection_result().is_current()
    }

    /// Return the retained intersection outcome summary, if present.
    pub fn retained_intersection_result_outcome(&self) -> Option<PreparedMeshPairResultOutcome> {
        *self.intersection_result_outcome.borrow()
    }

    /// Return whether an intersection result or blocker has been retained.
    pub fn has_retained_intersection_result(&self) -> bool {
        self.intersection_result.borrow().is_some()
    }

    /// Require a retained intersection result or retained intersection blocker.
    pub fn require_current_intersection_result(&self) -> Result<(), ExactMeshError> {
        self.cache_status().require_current_intersection_result()
    }

    /// Materialize the exact closed difference of the left mesh minus the right mesh.
    pub fn difference(&self) -> Result<ExactMesh, ExactMeshError> {
        self.named_boolean_mesh(ExactBooleanOperation::Difference)
    }

    /// Retain the exact closed difference result or blocker summary for this pair session.
    pub fn prepare_difference_result(
        &self,
    ) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        let _ = self.difference();
        self.current_difference_result_outcome()
    }

    /// Return the retained difference result or cached error without materializing it.
    pub fn current_difference_result(&self) -> Result<ExactMesh, ExactMeshError> {
        self.current_named_boolean_mesh(ExactBooleanOperation::Difference)
    }

    /// Return the retained difference outcome without materializing the mesh.
    pub fn current_difference_result_outcome(
        &self,
    ) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        self.cache_status().current_difference_result_outcome()
    }

    /// Return whether a current retained difference result or blocker exists.
    pub fn difference_result_is_current(&self) -> bool {
        self.cache_status().difference_result().is_current()
    }

    /// Return the retained difference outcome summary, if present.
    pub fn retained_difference_result_outcome(&self) -> Option<PreparedMeshPairResultOutcome> {
        *self.difference_result_outcome.borrow()
    }

    /// Return whether a difference result or blocker has been retained.
    pub fn has_retained_difference_result(&self) -> bool {
        self.difference_result.borrow().is_some()
    }

    /// Require a retained difference result or retained difference blocker.
    pub fn require_current_difference_result(&self) -> Result<(), ExactMeshError> {
        self.cache_status().require_current_difference_result()
    }

    /// Materialize the exact closed symmetric difference of the prepared meshes.
    pub fn xor(&self) -> Result<ExactMesh, ExactMeshError> {
        if let Some(result) = self.xor_result.borrow().clone() {
            return result;
        }

        let result = (|| {
            let left_only = self.difference()?;
            let reverse_pair = self
                .right
                .view()
                .prepare_broad_phase_pair(self.left.view())?;
            let right_only = reverse_pair.difference()?;
            let union_pair = left_only
                .view()
                .prepare_broad_phase_pair(right_only.view())?;
            union_pair.union()
        })();
        retain_boolean_result(&self.xor_result, &self.xor_result_outcome, &result);
        result
    }

    /// Retain the exact closed symmetric-difference result or blocker summary.
    pub fn prepare_xor_result(&self) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        let _ = self.xor();
        self.current_xor_result_outcome()
    }

    /// Return the retained symmetric-difference result or cached error without materializing it.
    pub fn current_xor_result(&self) -> Result<ExactMesh, ExactMeshError> {
        self.xor_result
            .borrow()
            .clone()
            .unwrap_or_else(|| missing_retained_result("xor result"))
    }

    /// Return the retained symmetric-difference outcome without materializing the mesh.
    pub fn current_xor_result_outcome(
        &self,
    ) -> Result<PreparedMeshPairResultOutcome, ExactMeshError> {
        self.cache_status().current_xor_result_outcome()
    }

    /// Return whether a current retained symmetric-difference result or blocker exists.
    pub fn xor_result_is_current(&self) -> bool {
        self.cache_status().xor_result().is_current()
    }

    /// Return the retained symmetric-difference outcome summary, if present.
    pub fn retained_xor_result_outcome(&self) -> Option<PreparedMeshPairResultOutcome> {
        *self.xor_result_outcome.borrow()
    }

    /// Return whether a symmetric-difference result or blocker has been retained.
    pub fn has_retained_xor_result(&self) -> bool {
        self.xor_result.borrow().is_some()
    }

    /// Require a retained symmetric-difference result or retained blocker.
    pub fn require_current_xor_result(&self) -> Result<(), ExactMeshError> {
        self.cache_status().require_current_xor_result()
    }

    fn named_boolean_mesh(
        &self,
        operation: ExactBooleanOperation,
    ) -> Result<ExactMesh, ExactMeshError> {
        if let Some(result) = self.cached_named_boolean_mesh(operation) {
            return result;
        }

        let request = ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED);
        let result = materialize_boolean_exact_request_with_prepared_pair(self, request)
            .map(|result| result.into_mesh());
        self.retain_named_boolean_mesh(operation, &result);
        result
    }

    fn current_named_boolean_mesh(
        &self,
        operation: ExactBooleanOperation,
    ) -> Result<ExactMesh, ExactMeshError> {
        self.cached_named_boolean_mesh(operation)
            .unwrap_or_else(|| missing_retained_result(named_boolean_result_name(operation)))
    }

    fn cached_named_boolean_mesh(
        &self,
        operation: ExactBooleanOperation,
    ) -> Option<Result<ExactMesh, ExactMeshError>> {
        match operation {
            ExactBooleanOperation::Union => self.union_result.borrow().clone(),
            ExactBooleanOperation::Intersection => self.intersection_result.borrow().clone(),
            ExactBooleanOperation::Difference => self.difference_result.borrow().clone(),
            ExactBooleanOperation::SelectedRegions(_) => None,
        }
    }

    fn retain_named_boolean_mesh(
        &self,
        operation: ExactBooleanOperation,
        result: &Result<ExactMesh, ExactMeshError>,
    ) {
        let target = match operation {
            ExactBooleanOperation::Union => (&self.union_result, &self.union_result_outcome),
            ExactBooleanOperation::Intersection => {
                (&self.intersection_result, &self.intersection_result_outcome)
            }
            ExactBooleanOperation::Difference => {
                (&self.difference_result, &self.difference_result_outcome)
            }
            ExactBooleanOperation::SelectedRegions(_) => return,
        };
        retain_boolean_result(target.0, target.1, result);
    }

    /// Visit certificate-validated broad-phase candidate face pairs using the cached pair plan.
    pub fn visit_candidate_face_pairs(&self, visit: &mut impl FnMut([usize; 2])) {
        let result = self.try_visit_candidate_face_pairs(&mut |pair| {
            visit(pair);
            Ok::<(), ()>(())
        });
        debug_assert!(result.is_ok());
    }

    /// Visit certificate-validated candidate face pairs and allow the visitor to stop early.
    pub fn try_visit_candidate_face_pairs<E>(
        &self,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<(), E> {
        let candidate_face_pairs = self.candidate_face_pairs.borrow();
        if let Some(candidate_face_pairs) = candidate_face_pairs.as_deref() {
            for &pair in candidate_face_pairs {
                visit(pair)?;
            }
            return Ok(());
        }

        drop(candidate_face_pairs);
        let mut candidate_pair_count = 0usize;
        self.try_visit_candidate_face_pairs_uncached(&mut |pair| {
            candidate_pair_count = candidate_pair_count.saturating_add(1);
            visit(pair)
        })?;
        self.retain_broad_phase_traversal_count(candidate_pair_count);
        Ok(())
    }

    fn ensure_candidate_face_pairs(&self) {
        if self.candidate_face_pairs.borrow().is_some() {
            return;
        }

        let mut candidate_face_pairs = Vec::with_capacity(self.candidate_face_pair_capacity_hint());
        let result = self.try_visit_candidate_face_pairs_uncached(&mut |pair| {
            candidate_face_pairs.push(pair);
            Ok::<(), ()>(())
        });
        debug_assert!(result.is_ok());
        self.retain_broad_phase_traversal_count(candidate_face_pairs.len());
        *self.candidate_face_pairs.borrow_mut() = Some(candidate_face_pairs);
    }

    fn try_visit_candidate_face_pairs_uncached<E>(
        &self,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<(), E> {
        let broad_phase = ExactAabbBroadPhase::default();
        if let Ok(mut scratch) = self.scratch.try_borrow_mut() {
            return broad_phase.try_visit_candidate_face_pairs_with_plan_and_scratch(
                &self.left.bounds,
                &self.right.bounds,
                self.plan,
                &mut scratch,
                visit,
            );
        }

        let mut local_scratch = BroadPhaseScratch::default();
        broad_phase.try_visit_candidate_face_pairs_with_plan_and_scratch(
            &self.left.bounds,
            &self.right.bounds,
            self.plan,
            &mut local_scratch,
            visit,
        )
    }

    pub(crate) fn try_visit_unretained_candidate_face_pairs<E>(
        &self,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<(), E> {
        self.try_visit_candidate_face_pairs_uncached(visit)
    }
}

fn missing_retained_result(fact: &'static str) -> Result<ExactMesh, ExactMeshError> {
    Err(ExactMeshError::one(ExactMeshBlocker::new(
        ExactMeshBlockerKind::MissingRequiredEvidence,
        format!("prepared mesh-pair session is missing retained {fact} evidence"),
    )))
}

fn retain_boolean_result(
    result_slot: &RefCell<Option<Result<ExactMesh, ExactMeshError>>>,
    outcome_slot: &RefCell<Option<PreparedMeshPairResultOutcome>>,
    result: &Result<ExactMesh, ExactMeshError>,
) {
    *outcome_slot.borrow_mut() = Some(PreparedMeshPairResultOutcome::from_result(result));
    *result_slot.borrow_mut() = Some(result.clone());
}

const fn named_boolean_result_name(operation: ExactBooleanOperation) -> &'static str {
    match operation {
        ExactBooleanOperation::Union => "union result",
        ExactBooleanOperation::Intersection => "intersection result",
        ExactBooleanOperation::Difference => "difference result",
        ExactBooleanOperation::SelectedRegions(_) => "selected-region result",
    }
}

const fn source_pair_state(sources_current: bool) -> PreparedMeshPairFactState {
    if sources_current {
        PreparedMeshPairFactState::Current
    } else {
        PreparedMeshPairFactState::Stale
    }
}

const fn retained_current_state(
    retained: bool,
    sources_current: bool,
) -> PreparedMeshPairFactState {
    if !retained {
        PreparedMeshPairFactState::Missing
    } else if sources_current {
        PreparedMeshPairFactState::Current
    } else {
        PreparedMeshPairFactState::Stale
    }
}

const fn retained_certificate_state(
    certificate_current: bool,
    sources_current: bool,
) -> PreparedMeshPairFactState {
    if !sources_current {
        PreparedMeshPairFactState::Stale
    } else if certificate_current {
        PreparedMeshPairFactState::Current
    } else {
        PreparedMeshPairFactState::CertificateBlocked
    }
}

impl PreparedMeshPairPlanKind {
    /// Return whether broad-phase bounds proved there are no candidate face pairs.
    pub const fn is_empty(self) -> bool {
        matches!(self, Self::Empty)
    }

    /// Return whether candidate traversal uses a retained sorted-axis sweep.
    pub const fn is_sweep(self) -> bool {
        matches!(self, Self::Sweep)
    }

    /// Return whether candidate traversal falls back to exact quadratic checks.
    pub const fn is_quadratic(self) -> bool {
        matches!(self, Self::Quadratic)
    }

    const fn from_candidate_plan(plan: CandidateFacePairPlan) -> Self {
        match plan {
            CandidateFacePairPlan::Empty => Self::Empty,
            CandidateFacePairPlan::Sweep { .. } => Self::Sweep,
            CandidateFacePairPlan::Quadratic => Self::Quadratic,
        }
    }
}

impl<'pair, 'left, 'right> PreparedMeshPairView<'pair, 'left, 'right> {
    /// Return the left prepared mesh view.
    pub const fn left(&self) -> &'pair PreparedMeshView<'left> {
        self.left
    }

    /// Return the right prepared mesh view.
    pub const fn right(&self) -> &'pair PreparedMeshView<'right> {
        self.right
    }

    /// Return a bounded storage hint for candidate face-pair traversal.
    pub const fn candidate_face_pair_capacity_hint(&self) -> usize {
        self.candidate_pair_capacity_hint
    }

    /// Return retained broad-phase planning provenance for this borrowed pair view.
    pub const fn broad_phase_summary(&self) -> PreparedMeshPairBroadPhaseSummary {
        self.broad_phase_summary
    }

    /// Visit certificate-validated broad-phase candidate face pairs using the cached pair plan.
    pub fn visit_candidate_face_pairs(&self, visit: &mut impl FnMut([usize; 2])) {
        let result = self.try_visit_candidate_face_pairs(&mut |pair| {
            visit(pair);
            Ok::<(), ()>(())
        });
        debug_assert!(result.is_ok());
    }

    /// Visit certificate-validated candidate face pairs and allow the visitor to stop early.
    pub fn try_visit_candidate_face_pairs<E>(
        &self,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<(), E> {
        let broad_phase = ExactAabbBroadPhase::default();
        broad_phase.try_visit_candidate_face_pairs_with_plan(
            &self.left.bounds,
            &self.right.bounds,
            self.plan,
            visit,
        )
    }
}

impl<'a> VertexRef<'a> {
    /// Vertex index in the source mesh.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Exact vertex coordinates.
    pub fn point(self) -> &'a Point3 {
        &self.mesh.vertices()[self.index]
    }

    /// Whether retained facts certify exact rational coordinates for this vertex.
    pub fn has_exact_rational_coordinates(self) -> Result<bool, ExactMeshError> {
        retained_vertex_facts(self.mesh, self.index)
            .map(|facts| facts.fixed_coordinates_exact_rational)
    }

    /// Whether retained facts record sparse coordinate support for this vertex.
    pub fn has_sparse_coordinate_support(self) -> Result<bool, ExactMeshError> {
        retained_vertex_facts(self.mesh, self.index).map(|facts| facts.sparse_support)
    }

    /// Retained incident face count.
    pub fn incident_face_count(self) -> Result<usize, ExactMeshError> {
        retained_vertex_facts(self.mesh, self.index).map(|facts| facts.incident_faces)
    }

    /// Retained incident undirected edge count.
    pub fn incident_edge_count(self) -> Result<usize, ExactMeshError> {
        retained_vertex_facts(self.mesh, self.index).map(|facts| facts.incident_edges)
    }

    /// Whether retained facts classify the vertex link as isolated.
    pub fn has_isolated_link(self) -> Result<bool, ExactMeshError> {
        self.has_vertex_link(super::facts::VertexLinkKind::Isolated)
    }

    /// Whether retained facts classify the vertex link as a closed-manifold circle.
    pub fn has_circle_link(self) -> Result<bool, ExactMeshError> {
        self.has_vertex_link(super::facts::VertexLinkKind::Circle)
    }

    /// Whether retained facts classify the vertex link as a boundary-manifold disk.
    pub fn has_disk_link(self) -> Result<bool, ExactMeshError> {
        self.has_vertex_link(super::facts::VertexLinkKind::Disk)
    }

    /// Whether retained facts classify the vertex link as non-manifold.
    pub fn has_non_manifold_link(self) -> Result<bool, ExactMeshError> {
        self.has_vertex_link(super::facts::VertexLinkKind::NonManifold)
    }

    fn has_vertex_link(self, link: super::facts::VertexLinkKind) -> Result<bool, ExactMeshError> {
        retained_vertex_facts(self.mesh, self.index).map(|facts| facts.link == link)
    }
}

impl<'a> FaceRef<'a> {
    /// Face index in the source mesh.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Triangle vertex indices for this face.
    pub fn vertex_indices(self) -> [usize; 3] {
        self.mesh.triangles()[self.index].0
    }

    /// Borrow retained face bounds as exact min/max corners.
    pub fn bounds(self) -> Result<(&'a Point3, &'a Point3), ExactMeshError> {
        self.mesh
            .bounds()
            .face(self.index)
            .map(bounds_corners)
            .ok_or_else(|| missing_retained_face_bounds("face", self.index))
    }

    /// Borrow the face vertices.
    pub fn vertex_refs(self) -> Result<[VertexRef<'a>; 3], ExactMeshError> {
        vertex_refs(self.mesh, self.index, self.vertex_indices())
    }

    /// Retained directed edge rows in face winding order.
    pub fn directed_edges(self) -> Result<[[usize; 2]; 3], ExactMeshError> {
        retained_face_facts(self.mesh, self.index).map(|facts| facts.oriented.directed_edges)
    }

    /// Whether retained predicate evidence certified a non-degenerate triangle.
    pub fn is_non_degenerate(self) -> Result<bool, ExactMeshError> {
        retained_face_facts(self.mesh, self.index).map(|facts| facts.triangle.non_degenerate)
    }

    /// Predicate evidence retained while certifying triangle degeneracy.
    pub fn degeneracy_predicates(self) -> Result<&'a [PredicateUse], ExactMeshError> {
        retained_face_facts(self.mesh, self.index)
            .map(|facts| facts.triangle.degeneracy_predicates.as_slice())
    }

    /// Retained exact oriented plane normal.
    pub fn plane_normal(self) -> Result<&'a [Real; 3], ExactMeshError> {
        retained_face_facts(self.mesh, self.index).map(|facts| &facts.plane.normal)
    }

    /// Retained exact oriented plane offset.
    pub fn plane_offset(self) -> Result<&'a Real, ExactMeshError> {
        retained_face_facts(self.mesh, self.index).map(|facts| &facts.plane.offset)
    }

    /// Retained exact oriented plane coefficients.
    pub fn plane_coefficients(self) -> Result<(&'a [Real; 3], &'a Real), ExactMeshError> {
        let facts = retained_face_facts(self.mesh, self.index)?;
        Ok((&facts.plane.normal, &facts.plane.offset))
    }

    /// Exact face vertices.
    pub fn vertices(self) -> Result<[&'a Point3; 3], ExactMeshError> {
        triangle_vertices(self.mesh, self.index, self.vertex_indices())
    }
}

impl<'a> TriangleRef<'a> {
    /// Triangle index in the source mesh.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Triangle vertex indices.
    pub fn vertex_indices(self) -> [usize; 3] {
        self.mesh.triangles()[self.index].0
    }

    /// Borrow retained triangle bounds as exact min/max corners.
    pub fn bounds(self) -> Result<(&'a Point3, &'a Point3), ExactMeshError> {
        self.mesh
            .bounds()
            .face(self.index)
            .map(bounds_corners)
            .ok_or_else(|| missing_retained_face_bounds("triangle", self.index))
    }

    /// Borrow the triangle vertices.
    pub fn vertex_refs(self) -> Result<[VertexRef<'a>; 3], ExactMeshError> {
        vertex_refs(self.mesh, self.index, self.vertex_indices())
    }

    /// Retained directed edge rows in triangle winding order.
    pub fn directed_edges(self) -> Result<[[usize; 2]; 3], ExactMeshError> {
        retained_face_facts(self.mesh, self.index).map(|facts| facts.oriented.directed_edges)
    }

    /// Whether retained predicate evidence certified a non-degenerate triangle.
    pub fn is_non_degenerate(self) -> Result<bool, ExactMeshError> {
        retained_face_facts(self.mesh, self.index).map(|facts| facts.triangle.non_degenerate)
    }

    /// Predicate evidence retained while certifying triangle degeneracy.
    pub fn degeneracy_predicates(self) -> Result<&'a [PredicateUse], ExactMeshError> {
        retained_face_facts(self.mesh, self.index)
            .map(|facts| facts.triangle.degeneracy_predicates.as_slice())
    }

    /// Retained exact oriented plane normal.
    pub fn plane_normal(self) -> Result<&'a [Real; 3], ExactMeshError> {
        retained_face_facts(self.mesh, self.index).map(|facts| &facts.plane.normal)
    }

    /// Retained exact oriented plane offset.
    pub fn plane_offset(self) -> Result<&'a Real, ExactMeshError> {
        retained_face_facts(self.mesh, self.index).map(|facts| &facts.plane.offset)
    }

    /// Retained exact oriented plane coefficients.
    pub fn plane_coefficients(self) -> Result<(&'a [Real; 3], &'a Real), ExactMeshError> {
        let facts = retained_face_facts(self.mesh, self.index)?;
        Ok((&facts.plane.normal, &facts.plane.offset))
    }

    /// Exact triangle vertices.
    pub fn vertices(self) -> Result<[&'a Point3; 3], ExactMeshError> {
        triangle_vertices(self.mesh, self.index, self.vertex_indices())
    }
}

impl<'a> EdgeRef<'a> {
    /// Edge index in the retained edge-fact table.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Retained endpoint vertex indices.
    pub fn vertex_indices(self) -> [usize; 2] {
        self.mesh.facts().edges[self.index].vertices
    }

    /// Borrow the edge endpoint vertices.
    pub fn vertex_refs(self) -> Result<[VertexRef<'a>; 2], ExactMeshError> {
        let [a, b] = self.vertex_indices();
        require_retained_edge_endpoint(self.mesh, self.index, a)?;
        require_retained_edge_endpoint(self.mesh, self.index, b)?;
        Ok([
            VertexRef {
                mesh: self.mesh,
                index: a,
            },
            VertexRef {
                mesh: self.mesh,
                index: b,
            },
        ])
    }

    /// Return exact endpoint bounds as min/max corners.
    pub fn bounds(self) -> Result<(Point3, Point3), ExactMeshError> {
        let [a, b] = self.vertices()?;
        let bounds = ExactAabb3::from_points(&[a.clone(), b.clone()]).ok_or_else(|| {
            ExactMeshError::one(
                ExactMeshBlocker::new(
                    ExactMeshBlockerKind::MissingRequiredEvidence,
                    format!("mesh edge {} has no retained endpoint bounds", self.index),
                )
                .with_edge(self.vertex_indices()),
            )
        })?;
        Ok((bounds.min, bounds.max))
    }

    /// Retained incident face count.
    pub fn incident_face_count(self) -> usize {
        self.mesh.facts().edges[self.index].incident_faces
    }

    /// Retained directed use counts for the canonical edge orientation.
    pub fn directed_use_counts(self) -> [usize; 2] {
        self.mesh.facts().edges[self.index].directed_uses
    }

    /// Whether retained facts classify this edge as a closed-manifold edge.
    pub fn is_closed_manifold_edge(self) -> bool {
        let facts = &self.mesh.facts().edges[self.index];
        facts.is_closed_manifold_edge()
    }

    /// Exact edge endpoints.
    pub fn vertices(self) -> Result<[&'a Point3; 2], ExactMeshError> {
        let [a, b] = self.vertex_indices();
        let start =
            self.mesh.vertices().get(a).ok_or_else(|| {
                retained_edge_endpoint_error(self.index, self.vertex_indices(), a)
            })?;
        let end =
            self.mesh.vertices().get(b).ok_or_else(|| {
                retained_edge_endpoint_error(self.index, self.vertex_indices(), b)
            })?;
        Ok([start, end])
    }
}

fn triangle_vertices(
    mesh: &ExactMesh,
    face: usize,
    triangle: [usize; 3],
) -> Result<[&Point3; 3], ExactMeshError> {
    let [a, b, c] = triangle;
    let a = mesh
        .vertices()
        .get(a)
        .ok_or_else(|| retained_face_vertex_error(face, triangle, a))?;
    let b = mesh
        .vertices()
        .get(b)
        .ok_or_else(|| retained_face_vertex_error(face, triangle, b))?;
    let c = mesh
        .vertices()
        .get(c)
        .ok_or_else(|| retained_face_vertex_error(face, triangle, c))?;
    Ok([a, b, c])
}

fn vertex_refs(
    mesh: &ExactMesh,
    face: usize,
    triangle: [usize; 3],
) -> Result<[VertexRef<'_>; 3], ExactMeshError> {
    let [a, b, c] = triangle;
    require_retained_face_vertex(mesh, face, triangle, a)?;
    require_retained_face_vertex(mesh, face, triangle, b)?;
    require_retained_face_vertex(mesh, face, triangle, c)?;
    Ok([
        VertexRef { mesh, index: a },
        VertexRef { mesh, index: b },
        VertexRef { mesh, index: c },
    ])
}

fn bounds_corners(bounds: &ExactAabb3) -> (&Point3, &Point3) {
    (&bounds.min, &bounds.max)
}

fn missing_retained_face_bounds(kind: &'static str, face: usize) -> ExactMeshError {
    ExactMeshError::one(
        ExactMeshBlocker::new(
            ExactMeshBlockerKind::MissingRequiredEvidence,
            format!("mesh {kind} {face} has no retained exact bounds"),
        )
        .with_face(face),
    )
}

fn retained_vertex_facts(
    mesh: &ExactMesh,
    vertex: usize,
) -> Result<&super::facts::VertexFacts, ExactMeshError> {
    mesh.facts().vertices.get(vertex).ok_or_else(|| {
        ExactMeshError::one(
            ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!("retained mesh vertex {vertex} has no retained vertex fact row"),
            )
            .with_vertex(vertex),
        )
    })
}

fn retained_face_facts(
    mesh: &ExactMesh,
    face: usize,
) -> Result<&super::facts::FaceFacts, ExactMeshError> {
    mesh.facts().faces.get(face).ok_or_else(|| {
        ExactMeshError::one(
            ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!("retained mesh face {face} has no retained face fact row"),
            )
            .with_face(face),
        )
    })
}

fn require_retained_edge_endpoint(
    mesh: &ExactMesh,
    edge: usize,
    vertex: usize,
) -> Result<(), ExactMeshError> {
    if vertex < mesh.vertices().len() {
        Ok(())
    } else {
        let edge_vertices = mesh.facts().edges[edge].vertices;
        Err(retained_edge_endpoint_error(edge, edge_vertices, vertex))
    }
}

fn require_retained_face_vertex(
    mesh: &ExactMesh,
    face: usize,
    triangle: [usize; 3],
    vertex: usize,
) -> Result<(), ExactMeshError> {
    if vertex < mesh.vertices().len() {
        Ok(())
    } else {
        Err(retained_face_vertex_error(face, triangle, vertex))
    }
}

fn retained_face_vertex_error(face: usize, triangle: [usize; 3], vertex: usize) -> ExactMeshError {
    ExactMeshError::one(
        ExactMeshBlocker::new(
            ExactMeshBlockerKind::StaleFactReplay,
            format!(
                "retained mesh face {face} with vertex row {triangle:?} references missing vertex {vertex}"
            ),
        )
        .with_face(face)
        .with_vertex(vertex),
    )
}

fn retained_edge_endpoint_error(
    edge: usize,
    edge_vertices: [usize; 2],
    vertex: usize,
) -> ExactMeshError {
    ExactMeshError::one(
        ExactMeshBlocker::new(
            ExactMeshBlockerKind::StaleFactReplay,
            format!("retained mesh edge {edge} references missing vertex {vertex}"),
        )
        .with_edge(edge_vertices)
        .with_vertex(vertex),
    )
}
