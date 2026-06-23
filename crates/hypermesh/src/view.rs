//! Borrowed exact views of retained mesh data.

use std::{cell::RefCell, rc::Rc};

use super::ExactMesh;
use super::arrangement3d::{ArrangementView, ExactArrangement};
use super::boolean::{
    ExactArrangementCellComplexShortcutFacts, ExactBooleanOperation, ExactBooleanRequest,
    materialize_boolean_exact_request_with_prepared_pair,
};
use super::bounds::{
    BroadPhaseScratch, CandidateFacePairPlan, ExactAabb3, ExactAabbBroadPhase, PreparedMeshBounds,
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
use hyperlimit::{Point3, PredicateUse};
use hyperreal::Real;

/// Borrowed exact view of an [`ExactMesh`].
#[derive(Clone, Copy, Debug)]
pub struct ExactMeshRef<'a> {
    mesh: &'a ExactMesh,
}

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
pub struct PreparedMeshPairCacheStatus {
    candidate_pair_plan: PreparedMeshPairPlanKind,
    candidate_pair_capacity_hint: usize,
    broad_phase_summary: PreparedMeshPairBroadPhaseSummary,
    candidate_face_pairs: PreparedMeshPairFactState,
    retained_candidate_face_pair_count: Option<usize>,
    face_pair_classifications: PreparedMeshPairFactState,
    retained_face_pair_classification_count: Option<usize>,
    retained_face_pair_classification_counts: Option<PreparedMeshPairClassificationCounts>,
    intersection_graph: PreparedMeshPairFactState,
    retained_intersection_graph_face_pair_count: Option<usize>,
    retained_intersection_graph_event_count: Option<usize>,
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
        plan: CandidateFacePairPlan,
        left_face_count: usize,
        right_face_count: usize,
        candidate_pair_capacity_hint: usize,
    ) -> Self {
        Self {
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
pub enum PreparedMeshPairFactState {
    /// The fact has not been computed for this session.
    Missing,
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
        let mut counts = Self {
            face_pairs: classifications.len(),
            ..Self::default()
        };
        for classification in classifications {
            match classification.relation {
                MeshFacePairRelation::PlaneSeparated => counts.plane_separated += 1,
                MeshFacePairRelation::CoplanarTouching => counts.coplanar_touching += 1,
                MeshFacePairRelation::CoplanarOverlapping => counts.coplanar_overlapping += 1,
                MeshFacePairRelation::Candidate => counts.candidates += 1,
                MeshFacePairRelation::Unknown => counts.unknown += 1,
            }
            if classification.needs_graph_construction() {
                counts.graph_required += 1;
            }
        }
        counts
    }
}

impl PreparedMeshPairFactState {
    /// Return whether a later stage can consume this fact through a cheap certificate check.
    pub const fn is_current(self) -> bool {
        matches!(self, Self::Current)
    }

    /// Return whether this session retains the fact in any state.
    pub const fn is_retained(self) -> bool {
        !matches!(self, Self::Missing)
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
    /// Return the retained broad-phase candidate traversal plan.
    pub const fn candidate_pair_plan(self) -> PreparedMeshPairPlanKind {
        self.candidate_pair_plan
    }

    /// Return the bounded storage hint for candidate face-pair traversal.
    pub const fn candidate_pair_capacity_hint(self) -> usize {
        self.candidate_pair_capacity_hint
    }

    /// Return retained broad-phase planning provenance for this session.
    pub const fn broad_phase_summary(self) -> PreparedMeshPairBroadPhaseSummary {
        self.broad_phase_summary
    }

    /// Return the certificate state for retained broad-phase candidate pairs.
    pub const fn candidate_face_pairs(self) -> PreparedMeshPairFactState {
        self.candidate_face_pairs
    }

    /// Return the retained broad-phase candidate pair count, when cached.
    pub const fn retained_candidate_face_pair_count(self) -> Option<usize> {
        self.retained_candidate_face_pair_count
    }

    /// Require retained broad-phase candidate pairs with current certificates.
    pub fn require_current_candidate_face_pairs(self) -> Result<(), ExactMeshError> {
        self.candidate_face_pairs
            .require_current("broad-phase candidate face pairs")
    }

    /// Return retained broad-phase candidate pair count after requiring current evidence.
    pub fn current_candidate_face_pair_count(self) -> Result<usize, ExactMeshError> {
        self.require_current_candidate_face_pairs()?;
        self.retained_candidate_face_pair_count.ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session retained broad-phase candidate pairs without a count",
            ))
        })
    }

    /// Return retained face-pair product rejected by broad-phase bounds, when candidates are cached.
    pub fn retained_broad_phase_rejection_count(self) -> Option<usize> {
        self.retained_candidate_face_pair_count.map(|candidates| {
            self.broad_phase_summary
                .face_pair_product()
                .saturating_sub(candidates)
        })
    }

    /// Return retained candidate upper-bound slack, when candidates are cached.
    pub fn retained_candidate_upper_bound_slack(self) -> Option<usize> {
        self.retained_candidate_face_pair_count.map(|candidates| {
            self.broad_phase_summary
                .candidate_pair_upper_bound()
                .saturating_sub(candidates)
        })
    }

    /// Return whether the retained candidate count saturated the planned upper bound.
    pub fn retained_candidate_upper_bound_saturated(self) -> Option<bool> {
        self.retained_candidate_upper_bound_slack()
            .map(|slack| slack == 0)
    }

    /// Return the certificate state for coarse face-pair classifications.
    pub const fn face_pair_classifications(self) -> PreparedMeshPairFactState {
        self.face_pair_classifications
    }

    /// Return the retained coarse face-pair classification count, when cached.
    pub const fn retained_face_pair_classification_count(self) -> Option<usize> {
        self.retained_face_pair_classification_count
    }

    /// Return retained coarse face-pair decision counts, when cached.
    pub const fn retained_face_pair_classification_counts(
        self,
    ) -> Option<PreparedMeshPairClassificationCounts> {
        self.retained_face_pair_classification_counts
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
        self.require_current_face_pair_classifications()?;
        self.retained_face_pair_classification_counts
            .ok_or_else(|| {
                ExactMeshError::one(ExactMeshBlocker::new(
                    ExactMeshBlockerKind::MissingRequiredEvidence,
                    "prepared mesh-pair session retained face-pair classification evidence without decision counts",
                ))
            })
    }

    /// Return the certificate state for the exact intersection graph.
    pub const fn intersection_graph(self) -> PreparedMeshPairFactState {
        self.intersection_graph
    }

    /// Return the retained graph face-pair count, when cached.
    pub const fn retained_intersection_graph_face_pair_count(self) -> Option<usize> {
        self.retained_intersection_graph_face_pair_count
    }

    /// Return the retained graph event count, when cached.
    pub const fn retained_intersection_graph_event_count(self) -> Option<usize> {
        self.retained_intersection_graph_event_count
    }

    /// Return retained graph summary counts, when cached.
    pub const fn retained_intersection_graph_counts(
        self,
    ) -> Option<PreparedMeshPairIntersectionGraphCounts> {
        self.retained_intersection_graph_counts
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

    /// Return retained arrangement topology counts, when cached.
    pub const fn retained_arrangement_counts(self) -> Option<PreparedMeshPairArrangementCounts> {
        self.retained_arrangement_counts
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

    /// Return the retained prepared union outcome, when cached.
    pub const fn union_result_outcome(self) -> Option<PreparedMeshPairResultOutcome> {
        self.union_result_outcome
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

    /// Return the retained prepared intersection outcome, when cached.
    pub const fn intersection_result_outcome(self) -> Option<PreparedMeshPairResultOutcome> {
        self.intersection_result_outcome
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

    /// Return the retained prepared difference outcome, when cached.
    pub const fn difference_result_outcome(self) -> Option<PreparedMeshPairResultOutcome> {
        self.difference_result_outcome
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

    /// Return the retained prepared symmetric-difference outcome, when cached.
    pub const fn xor_result_outcome(self) -> Option<PreparedMeshPairResultOutcome> {
        self.xor_result_outcome
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

    /// Build and retain broad-phase candidate face pairs, returning the retained count.
    pub fn prepare_candidate_face_pairs(&self) -> usize {
        self.ensure_candidate_face_pairs();
        self.candidate_face_pairs
            .borrow()
            .as_ref()
            .map_or(0, Vec::len)
    }

    /// Build and retain coarse face-pair classifications, returning the retained count.
    pub fn prepare_face_pair_classifications(&self) -> usize {
        self.prepare_face_pair_classification_counts()
            .face_pair_count()
    }

    /// Build and retain coarse face-pair classifications, returning retained decision counts.
    pub fn prepare_face_pair_classification_counts(&self) -> PreparedMeshPairClassificationCounts {
        self.ensure_face_pair_classifications();
        self.face_pair_classification_counts
            .borrow()
            .as_ref()
            .copied()
            .unwrap_or_default()
    }

    /// Return a cheap summary of retained facts in this prepared pair session.
    pub fn cache_status(&self) -> PreparedMeshPairCacheStatus {
        let candidate_face_pair_count = self.candidate_face_pairs.borrow().as_ref().map(Vec::len);
        let face_pair_classification_counts = *self.face_pair_classification_counts.borrow();
        let graph_counts = *self.intersection_graph_counts.borrow();
        let graph_retained = self.intersection_graph.borrow().is_some();
        let arrangement_retained = self.arrangement.borrow().is_some();
        let union_retained = self.union_result.borrow().is_some();
        let intersection_retained = self.intersection_result.borrow().is_some();
        let difference_retained = self.difference_result.borrow().is_some();
        let xor_retained = self.xor_result.borrow().is_some();
        PreparedMeshPairCacheStatus {
            candidate_pair_plan: PreparedMeshPairPlanKind::from_candidate_plan(self.plan),
            candidate_pair_capacity_hint: self.candidate_face_pair_capacity_hint(),
            broad_phase_summary: self.broad_phase_summary,
            candidate_face_pairs: retained_current_state(candidate_face_pair_count.is_some()),
            retained_candidate_face_pair_count: candidate_face_pair_count,
            face_pair_classifications: retained_current_state(
                face_pair_classification_counts.is_some(),
            ),
            retained_face_pair_classification_count: face_pair_classification_counts
                .map(PreparedMeshPairClassificationCounts::face_pair_count),
            retained_face_pair_classification_counts: face_pair_classification_counts,
            intersection_graph: if graph_retained {
                retained_certificate_state(*self.intersection_graph_validated.borrow())
            } else {
                PreparedMeshPairFactState::Missing
            },
            retained_intersection_graph_face_pair_count: graph_counts
                .map(PreparedMeshPairIntersectionGraphCounts::face_pair_count),
            retained_intersection_graph_event_count: graph_counts
                .map(PreparedMeshPairIntersectionGraphCounts::event_count),
            retained_intersection_graph_counts: graph_counts,
            arrangement: retained_current_state(arrangement_retained),
            retained_arrangement_counts: *self.arrangement_counts.borrow(),
            arrangement_shortcut_facts: retained_current_state(
                self.arrangement_shortcut_facts.borrow().is_some(),
            ),
            union_result: retained_current_state(union_retained),
            union_result_outcome: *self.union_result_outcome.borrow(),
            intersection_result: retained_current_state(intersection_retained),
            intersection_result_outcome: *self.intersection_result_outcome.borrow(),
            difference_result: retained_current_state(difference_retained),
            difference_result_outcome: *self.difference_result_outcome.borrow(),
            xor_result: retained_current_state(xor_retained),
            xor_result_outcome: *self.xor_result_outcome.borrow(),
        }
    }

    pub(crate) fn intersection_graph_state(&self) -> PreparedMeshPairFactState {
        if self.intersection_graph.borrow().is_none() {
            PreparedMeshPairFactState::Missing
        } else {
            retained_certificate_state(*self.intersection_graph_validated.borrow())
        }
    }

    /// Return retained exact intersection graph counts after requiring a current certificate.
    pub fn current_intersection_graph_counts(
        &self,
    ) -> Result<PreparedMeshPairIntersectionGraphCounts, ExactMeshError> {
        self.cache_status().current_intersection_graph_counts()
    }

    /// Return the retained broad-phase candidate pair count after requiring current evidence.
    pub fn current_candidate_face_pair_count(&self) -> Result<usize, ExactMeshError> {
        self.cache_status().current_candidate_face_pair_count()
    }

    /// Borrow retained broad-phase candidate pairs without rebuilding missing evidence.
    pub fn with_current_candidate_face_pairs<R>(
        &self,
        query: impl FnOnce(&[[usize; 2]]) -> R,
    ) -> Result<R, ExactMeshError> {
        self.cache_status().require_current_candidate_face_pairs()?;
        let candidate_face_pairs = self.candidate_face_pairs.borrow();
        let pairs = candidate_face_pairs.as_deref().ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session retained broad-phase candidate-pair state without candidate records",
            ))
        })?;
        Ok(query(pairs))
    }

    /// Return retained arrangement topology counts after requiring current evidence.
    pub fn current_arrangement_counts(
        &self,
    ) -> Result<PreparedMeshPairArrangementCounts, ExactMeshError> {
        self.cache_status().current_arrangement_counts()
    }

    /// Build and retain the exact arrangement, returning retained topology counts.
    pub fn prepare_arrangement(&self) -> Result<PreparedMeshPairArrangementCounts, ExactMeshError> {
        self.retained_arrangement()?;
        self.current_arrangement_counts()
    }

    /// Build and retain arrangement shortcut facts for this prepared pair.
    pub fn prepare_arrangement_shortcut_facts(&self) -> Result<(), ExactMeshError> {
        self.arrangement_cell_complex_shortcut_facts();
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

    fn ensure_face_pair_classifications(&self) {
        if self.face_pair_classifications.borrow().is_some() {
            return;
        }

        let mut classifications = Vec::with_capacity(self.candidate_face_pair_capacity_hint());
        self.ensure_candidate_face_pairs();
        let candidate_face_pairs = self.candidate_face_pairs.borrow();
        for &[left_face, right_face] in candidate_face_pairs.as_deref().unwrap_or(&[]) {
            classifications.push(classify_mesh_face_pair_unchecked(
                self.left.view.mesh,
                left_face,
                self.right.view.mesh,
                right_face,
            ));
        }
        let counts = PreparedMeshPairClassificationCounts::from_classifications(&classifications);
        *self.face_pair_classifications.borrow_mut() = Some(classifications);
        *self.face_pair_classification_counts.borrow_mut() = Some(counts);
    }

    pub(crate) fn cached_intersection_graph(&self) -> Option<Rc<ExactIntersectionGraph>> {
        self.intersection_graph.borrow().clone()
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
        self.ensure_candidate_face_pairs();
        let candidate_face_pairs = self.candidate_face_pairs.borrow();
        for &pair in candidate_face_pairs.as_deref().unwrap_or(&[]) {
            visit(pair)?;
        }
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

const fn retained_current_state(retained: bool) -> PreparedMeshPairFactState {
    if retained {
        PreparedMeshPairFactState::Current
    } else {
        PreparedMeshPairFactState::Missing
    }
}

const fn retained_certificate_state(certificate_current: bool) -> PreparedMeshPairFactState {
    if certificate_current {
        PreparedMeshPairFactState::Current
    } else {
        PreparedMeshPairFactState::CertificateBlocked
    }
}

impl PreparedMeshPairPlanKind {
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
    pub fn has_exact_rational_coordinates(self) -> bool {
        self.mesh.facts().vertices[self.index].fixed_coordinates_exact_rational
    }

    /// Whether retained facts record sparse coordinate support for this vertex.
    pub fn has_sparse_coordinate_support(self) -> bool {
        self.mesh.facts().vertices[self.index].sparse_support
    }

    /// Retained incident face count.
    pub fn incident_face_count(self) -> usize {
        self.mesh.facts().vertices[self.index].incident_faces
    }

    /// Retained incident undirected edge count.
    pub fn incident_edge_count(self) -> usize {
        self.mesh.facts().vertices[self.index].incident_edges
    }

    /// Whether retained facts classify the vertex link as isolated.
    pub fn has_isolated_link(self) -> bool {
        matches!(
            self.mesh.facts().vertices[self.index].link,
            super::facts::VertexLinkKind::Isolated
        )
    }

    /// Whether retained facts classify the vertex link as a closed-manifold circle.
    pub fn has_circle_link(self) -> bool {
        matches!(
            self.mesh.facts().vertices[self.index].link,
            super::facts::VertexLinkKind::Circle
        )
    }

    /// Whether retained facts classify the vertex link as a boundary-manifold disk.
    pub fn has_disk_link(self) -> bool {
        matches!(
            self.mesh.facts().vertices[self.index].link,
            super::facts::VertexLinkKind::Disk
        )
    }

    /// Whether retained facts classify the vertex link as non-manifold.
    pub fn has_non_manifold_link(self) -> bool {
        matches!(
            self.mesh.facts().vertices[self.index].link,
            super::facts::VertexLinkKind::NonManifold
        )
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
    pub fn bounds(self) -> (&'a Point3, &'a Point3) {
        self.mesh
            .bounds()
            .face(self.index)
            .map(bounds_corners)
            .expect("face reference index must have retained bounds")
    }

    /// Borrow the face vertices.
    pub fn vertex_refs(self) -> [VertexRef<'a>; 3] {
        vertex_refs(self.mesh, self.vertex_indices())
    }

    /// Retained directed edge rows in face winding order.
    pub fn directed_edges(self) -> [[usize; 2]; 3] {
        self.mesh.facts().faces[self.index].oriented.directed_edges
    }

    /// Whether retained predicate evidence certified a non-degenerate triangle.
    pub fn is_non_degenerate(self) -> bool {
        self.mesh.facts().faces[self.index].triangle.non_degenerate
    }

    /// Predicate evidence retained while certifying triangle degeneracy.
    pub fn degeneracy_predicates(self) -> &'a [PredicateUse] {
        &self.mesh.facts().faces[self.index]
            .triangle
            .degeneracy_predicates
    }

    /// Retained exact oriented plane normal.
    pub fn plane_normal(self) -> &'a [Real; 3] {
        &self.mesh.facts().faces[self.index].plane.normal
    }

    /// Retained exact oriented plane offset.
    pub fn plane_offset(self) -> &'a Real {
        &self.mesh.facts().faces[self.index].plane.offset
    }

    /// Retained exact oriented plane coefficients.
    pub fn plane_coefficients(self) -> (&'a [Real; 3], &'a Real) {
        (self.plane_normal(), self.plane_offset())
    }

    /// Exact face vertices.
    pub fn vertices(self) -> [&'a Point3; 3] {
        triangle_vertices(self.mesh, self.vertex_indices())
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
    pub fn bounds(self) -> (&'a Point3, &'a Point3) {
        self.mesh
            .bounds()
            .face(self.index)
            .map(bounds_corners)
            .expect("triangle reference index must have retained bounds")
    }

    /// Borrow the triangle vertices.
    pub fn vertex_refs(self) -> [VertexRef<'a>; 3] {
        vertex_refs(self.mesh, self.vertex_indices())
    }

    /// Retained directed edge rows in triangle winding order.
    pub fn directed_edges(self) -> [[usize; 2]; 3] {
        self.mesh.facts().faces[self.index].oriented.directed_edges
    }

    /// Whether retained predicate evidence certified a non-degenerate triangle.
    pub fn is_non_degenerate(self) -> bool {
        self.mesh.facts().faces[self.index].triangle.non_degenerate
    }

    /// Predicate evidence retained while certifying triangle degeneracy.
    pub fn degeneracy_predicates(self) -> &'a [PredicateUse] {
        &self.mesh.facts().faces[self.index]
            .triangle
            .degeneracy_predicates
    }

    /// Retained exact oriented plane normal.
    pub fn plane_normal(self) -> &'a [Real; 3] {
        &self.mesh.facts().faces[self.index].plane.normal
    }

    /// Retained exact oriented plane offset.
    pub fn plane_offset(self) -> &'a Real {
        &self.mesh.facts().faces[self.index].plane.offset
    }

    /// Retained exact oriented plane coefficients.
    pub fn plane_coefficients(self) -> (&'a [Real; 3], &'a Real) {
        (self.plane_normal(), self.plane_offset())
    }

    /// Exact triangle vertices.
    pub fn vertices(self) -> [&'a Point3; 3] {
        triangle_vertices(self.mesh, self.vertex_indices())
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
    pub fn vertex_refs(self) -> [VertexRef<'a>; 2] {
        let [a, b] = self.vertex_indices();
        [
            VertexRef {
                mesh: self.mesh,
                index: a,
            },
            VertexRef {
                mesh: self.mesh,
                index: b,
            },
        ]
    }

    /// Return exact endpoint bounds as min/max corners.
    pub fn bounds(self) -> (Point3, Point3) {
        let [a, b] = self.vertices();
        let bounds = ExactAabb3::from_points(&[a.clone(), b.clone()])
            .expect("edge reference must have two endpoint points");
        (bounds.min, bounds.max)
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
    pub fn vertices(self) -> [&'a Point3; 2] {
        let [a, b] = self.vertex_indices();
        [&self.mesh.vertices()[a], &self.mesh.vertices()[b]]
    }
}

fn triangle_vertices(mesh: &ExactMesh, triangle: [usize; 3]) -> [&Point3; 3] {
    let [a, b, c] = triangle;
    [
        &mesh.vertices()[a],
        &mesh.vertices()[b],
        &mesh.vertices()[c],
    ]
}

fn vertex_refs(mesh: &ExactMesh, triangle: [usize; 3]) -> [VertexRef<'_>; 3] {
    let [a, b, c] = triangle;
    [
        VertexRef { mesh, index: a },
        VertexRef { mesh, index: b },
        VertexRef { mesh, index: c },
    ]
}

fn bounds_corners(bounds: &ExactAabb3) -> (&Point3, &Point3) {
    (&bounds.min, &bounds.max)
}
