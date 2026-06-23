//! Borrowed exact views of retained mesh data.

use std::{cell::RefCell, rc::Rc};

use super::ExactMesh;
use super::boolean::{
    ExactArrangementCellComplexShortcutFacts, ExactBooleanOperation, ExactBooleanRequest,
    materialize_boolean_exact_request_with_prepared_pair,
};
use super::bounds::{
    BroadPhaseScratch, CandidateFacePairPlan, ExactAabbBroadPhase, PreparedMeshBounds,
};
use super::error::ExactMeshError;
use super::error::{ExactMeshBlocker, ExactMeshBlockerKind};
use super::graph::ExactIntersectionGraph;
use super::intersection::{MeshFacePairClassification, classify_mesh_face_pair_unchecked};
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
    candidate_pair_capacity_hint: usize,
    scratch: RefCell<BroadPhaseScratch>,
    face_pair_classifications: RefCell<Option<Vec<MeshFacePairClassification>>>,
    intersection_graph: RefCell<Option<Rc<ExactIntersectionGraph>>>,
    intersection_graph_counts: RefCell<Option<RetainedIntersectionGraphCounts>>,
    intersection_graph_validated: RefCell<bool>,
    arrangement_shortcut_facts: RefCell<Option<ExactArrangementCellComplexShortcutFacts>>,
    union_result: RefCell<Option<Result<ExactMesh, ExactMeshError>>>,
    intersection_result: RefCell<Option<Result<ExactMesh, ExactMeshError>>>,
    difference_result: RefCell<Option<Result<ExactMesh, ExactMeshError>>>,
    xor_result: RefCell<Option<Result<ExactMesh, ExactMeshError>>>,
}

/// Borrowed prepared pair view with retained broad-phase pair planning.
#[derive(Debug)]
pub struct PreparedMeshPairView<'pair, 'left, 'right> {
    left: &'pair PreparedMeshView<'left>,
    right: &'pair PreparedMeshView<'right>,
    plan: CandidateFacePairPlan,
    candidate_pair_capacity_hint: usize,
}

/// Cheap status for retained facts inside a prepared mesh-pair session.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PreparedMeshPairCacheStatus {
    candidate_pair_plan: PreparedMeshPairPlanKind,
    candidate_pair_capacity_hint: usize,
    face_pair_classifications: PreparedMeshPairFactState,
    retained_face_pair_classification_count: Option<usize>,
    intersection_graph: PreparedMeshPairFactState,
    retained_intersection_graph_face_pair_count: Option<usize>,
    retained_intersection_graph_event_count: Option<usize>,
    arrangement_shortcut_facts: PreparedMeshPairFactState,
    union_result: PreparedMeshPairFactState,
    intersection_result: PreparedMeshPairFactState,
    difference_result: PreparedMeshPairFactState,
    xor_result: PreparedMeshPairFactState,
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct RetainedIntersectionGraphCounts {
    face_pairs: usize,
    events: usize,
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

    /// Return the certificate state for coarse face-pair classifications.
    pub const fn face_pair_classifications(self) -> PreparedMeshPairFactState {
        self.face_pair_classifications
    }

    /// Return the retained coarse face-pair classification count, when cached.
    pub const fn retained_face_pair_classification_count(self) -> Option<usize> {
        self.retained_face_pair_classification_count
    }

    /// Require retained coarse face-pair classifications with current certificates.
    pub fn require_current_face_pair_classifications(self) -> Result<(), ExactMeshError> {
        self.face_pair_classifications
            .require_current("face-pair classification")
    }

    /// Return the retained coarse face-pair classification count after requiring current evidence.
    pub fn current_face_pair_classification_count(self) -> Result<usize, ExactMeshError> {
        self.require_current_face_pair_classifications()?;
        self.retained_face_pair_classification_count
            .ok_or_else(|| {
                ExactMeshError::one(ExactMeshBlocker::new(
                    ExactMeshBlockerKind::MissingRequiredEvidence,
                    "prepared mesh-pair session retained face-pair classification evidence without a count",
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

    /// Require a retained exact intersection graph with a current replay certificate.
    pub fn require_current_intersection_graph(self) -> Result<(), ExactMeshError> {
        self.intersection_graph
            .require_current("intersection graph")
    }

    /// Return retained exact intersection graph counts after requiring a current certificate.
    pub fn current_intersection_graph_counts(self) -> Result<(usize, usize), ExactMeshError> {
        self.require_current_intersection_graph()?;
        match (
            self.retained_intersection_graph_face_pair_count,
            self.retained_intersection_graph_event_count,
        ) {
            (Some(face_pairs), Some(events)) => Ok((face_pairs, events)),
            _ => Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session retained an intersection graph certificate without graph counts",
            ))),
        }
    }

    /// Return the certificate state for arrangement shortcut facts.
    pub const fn arrangement_shortcut_facts(self) -> PreparedMeshPairFactState {
        self.arrangement_shortcut_facts
    }

    /// Return the certificate state for the prepared union result or error.
    pub const fn union_result(self) -> PreparedMeshPairFactState {
        self.union_result
    }

    /// Return the certificate state for the prepared intersection result or error.
    pub const fn intersection_result(self) -> PreparedMeshPairFactState {
        self.intersection_result
    }

    /// Return the certificate state for the prepared difference result or error.
    pub const fn difference_result(self) -> PreparedMeshPairFactState {
        self.difference_result
    }

    /// Return the certificate state for the prepared symmetric-difference result or error.
    pub const fn xor_result(self) -> PreparedMeshPairFactState {
        self.xor_result
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

    /// Borrow one vertex by index.
    pub fn vertex(self, index: usize) -> Option<VertexRef<'a>> {
        (index < self.mesh.vertices().len()).then_some(VertexRef {
            mesh: self.mesh,
            index,
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

    /// Borrow one triangle by index.
    pub fn triangle(self, index: usize) -> Option<TriangleRef<'a>> {
        (index < self.mesh.triangles().len()).then_some(TriangleRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Borrow one retained edge by index.
    pub fn edge(self, index: usize) -> Option<EdgeRef<'a>> {
        (index < self.mesh.facts().edges.len()).then_some(EdgeRef {
            mesh: self.mesh,
            index,
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

    /// Prepare a certificate-validated pair view that reuses its broad-phase plan.
    pub fn pair_with<'pair, 'right>(
        &'pair self,
        right: &'pair PreparedMeshView<'right>,
    ) -> PreparedMeshPairView<'pair, 'a, 'right> {
        let broad_phase = ExactAabbBroadPhase::default();
        let plan = broad_phase.candidate_face_pair_plan(&self.bounds, &right.bounds);
        let candidate_pair_capacity_hint =
            plan.bounded_capacity_hint(self.view.face_count(), right.view.face_count());
        PreparedMeshPairView {
            left: self,
            right,
            plan,
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
        Self {
            left,
            right,
            plan,
            candidate_pair_capacity_hint,
            scratch: RefCell::new(BroadPhaseScratch::default()),
            face_pair_classifications: RefCell::new(None),
            intersection_graph: RefCell::new(None),
            intersection_graph_counts: RefCell::new(None),
            intersection_graph_validated: RefCell::new(false),
            arrangement_shortcut_facts: RefCell::new(None),
            union_result: RefCell::new(None),
            intersection_result: RefCell::new(None),
            difference_result: RefCell::new(None),
            xor_result: RefCell::new(None),
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
            candidate_pair_capacity_hint: self.candidate_pair_capacity_hint,
        }
    }

    /// Return a bounded storage hint for candidate face-pair traversal.
    pub const fn candidate_face_pair_capacity_hint(&self) -> usize {
        self.candidate_pair_capacity_hint
    }

    /// Build and retain coarse face-pair classifications, returning the retained count.
    pub fn prepare_face_pair_classifications(&self) -> usize {
        self.ensure_face_pair_classifications();
        self.face_pair_classifications
            .borrow()
            .as_ref()
            .map(Vec::len)
            .unwrap_or(0)
    }

    /// Return a cheap summary of retained facts in this prepared pair session.
    pub fn cache_status(&self) -> PreparedMeshPairCacheStatus {
        let face_pair_classification_count = self
            .face_pair_classifications
            .borrow()
            .as_ref()
            .map(Vec::len);
        let graph_counts = *self.intersection_graph_counts.borrow();
        let graph_retained = self.intersection_graph.borrow().is_some();
        PreparedMeshPairCacheStatus {
            candidate_pair_plan: PreparedMeshPairPlanKind::from_candidate_plan(self.plan),
            candidate_pair_capacity_hint: self.candidate_face_pair_capacity_hint(),
            face_pair_classifications: retained_current_state(
                face_pair_classification_count.is_some(),
            ),
            retained_face_pair_classification_count: face_pair_classification_count,
            intersection_graph: if graph_retained {
                retained_certificate_state(*self.intersection_graph_validated.borrow())
            } else {
                PreparedMeshPairFactState::Missing
            },
            retained_intersection_graph_face_pair_count: graph_counts
                .map(|counts| counts.face_pairs),
            retained_intersection_graph_event_count: graph_counts.map(|counts| counts.events),
            arrangement_shortcut_facts: retained_current_state(
                self.arrangement_shortcut_facts.borrow().is_some(),
            ),
            union_result: retained_current_state(self.union_result.borrow().is_some()),
            intersection_result: retained_current_state(
                self.intersection_result.borrow().is_some(),
            ),
            difference_result: retained_current_state(self.difference_result.borrow().is_some()),
            xor_result: retained_current_state(self.xor_result.borrow().is_some()),
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
    pub fn current_intersection_graph_counts(&self) -> Result<(usize, usize), ExactMeshError> {
        self.cache_status().current_intersection_graph_counts()
    }

    /// Return the retained coarse face-pair classification count after requiring current evidence.
    pub fn current_face_pair_classification_count(&self) -> Result<usize, ExactMeshError> {
        self.cache_status().current_face_pair_classification_count()
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
        self.visit_candidate_face_pairs(&mut |[left_face, right_face]| {
            classifications.push(classify_mesh_face_pair_unchecked(
                self.left.view.mesh,
                left_face,
                self.right.view.mesh,
                right_face,
            ));
        });
        *self.face_pair_classifications.borrow_mut() = Some(classifications);
    }

    pub(crate) fn cached_intersection_graph(&self) -> Option<Rc<ExactIntersectionGraph>> {
        self.intersection_graph.borrow().clone()
    }

    pub(crate) fn retain_intersection_graph(
        &self,
        graph: ExactIntersectionGraph,
    ) -> Rc<ExactIntersectionGraph> {
        let counts = RetainedIntersectionGraphCounts {
            face_pairs: graph.face_pairs.len(),
            events: graph.event_count(),
        };
        let graph = Rc::new(graph);
        *self.intersection_graph.borrow_mut() = Some(Rc::clone(&graph));
        *self.intersection_graph_counts.borrow_mut() = Some(counts);
        *self.intersection_graph_validated.borrow_mut() = false;
        graph
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

    /// Materialize the exact closed intersection using this retained pair session.
    pub fn intersection(&self) -> Result<ExactMesh, ExactMeshError> {
        self.named_boolean_mesh(ExactBooleanOperation::Intersection)
    }

    /// Materialize the exact closed difference of the left mesh minus the right mesh.
    pub fn difference(&self) -> Result<ExactMesh, ExactMeshError> {
        self.named_boolean_mesh(ExactBooleanOperation::Difference)
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
        *self.xor_result.borrow_mut() = Some(result.clone());
        result
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
            ExactBooleanOperation::Union => &self.union_result,
            ExactBooleanOperation::Intersection => &self.intersection_result,
            ExactBooleanOperation::Difference => &self.difference_result,
            ExactBooleanOperation::SelectedRegions(_) => return,
        };
        *target.borrow_mut() = Some(result.clone());
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
