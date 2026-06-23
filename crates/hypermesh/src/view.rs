//! Borrowed exact views of retained mesh data.

use std::cell::RefCell;

use super::ExactMesh;
use super::boolean::{
    ExactArrangementCellComplexShortcutFacts, ExactBooleanOperation, ExactBooleanRequest,
    materialize_boolean_exact_request_with_prepared_pair,
};
use super::bounds::{
    BroadPhaseScratch, CandidateFacePairPlan, ExactAabbBroadPhase, ExactBroadPhaseStrategy,
    PreparedMeshBounds,
};
use super::error::ExactMeshError;
use super::graph::ExactIntersectionGraph;
use super::intersection::{MeshFacePairClassification, classify_mesh_face_pair_unchecked};
use super::validation::ExactMeshValidationPolicy;
use hyperlimit::Point3;
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
    scratch: RefCell<BroadPhaseScratch>,
    face_pair_classifications: RefCell<Option<Vec<MeshFacePairClassification>>>,
    intersection_graph: RefCell<Option<ExactIntersectionGraph>>,
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
}

/// Cheap status for retained facts inside a prepared mesh-pair session.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PreparedMeshPairCacheStatus {
    candidate_pair_capacity_hint: usize,
    retains_face_pair_classifications: bool,
    retains_intersection_graph: bool,
    intersection_graph_source_validated: bool,
    retains_arrangement_shortcut_facts: bool,
    retains_union_result: bool,
    retains_intersection_result: bool,
    retains_difference_result: bool,
    retains_xor_result: bool,
}

impl PreparedMeshPairCacheStatus {
    /// Return the bounded storage hint for candidate face-pair traversal.
    pub const fn candidate_pair_capacity_hint(self) -> usize {
        self.candidate_pair_capacity_hint
    }

    /// Return whether coarse face-pair classifications have been retained.
    pub const fn retains_face_pair_classifications(self) -> bool {
        self.retains_face_pair_classifications
    }

    /// Return whether the exact intersection graph has been retained.
    pub const fn retains_intersection_graph(self) -> bool {
        self.retains_intersection_graph
    }

    /// Return whether the retained graph has replay-validated against its sources.
    pub const fn intersection_graph_source_validated(self) -> bool {
        self.intersection_graph_source_validated
    }

    /// Return whether arrangement shortcut facts have been retained.
    pub const fn retains_arrangement_shortcut_facts(self) -> bool {
        self.retains_arrangement_shortcut_facts
    }

    /// Return whether the prepared union result or error has been retained.
    pub const fn retains_union_result(self) -> bool {
        self.retains_union_result
    }

    /// Return whether the prepared intersection result or error has been retained.
    pub const fn retains_intersection_result(self) -> bool {
        self.retains_intersection_result
    }

    /// Return whether the prepared difference result or error has been retained.
    pub const fn retains_difference_result(self) -> bool {
        self.retains_difference_result
    }

    /// Return whether the prepared symmetric-difference result or error has been retained.
    pub const fn retains_xor_result(self) -> bool {
        self.retains_xor_result
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

    pub(crate) fn prepare_broad_phase_pair_after_certificate<'b>(
        self,
        right: ExactMeshRef<'b>,
    ) -> PreparedMeshPair<'a, 'b> {
        PreparedMeshPair::new(
            self.prepare_broad_phase_after_certificate(),
            right.prepare_broad_phase_after_certificate(),
        )
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
        self.mesh.union(right.mesh)
    }

    /// Materialize the exact closed intersection of this view and `right`.
    pub fn intersection(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.mesh.intersection(right.mesh)
    }

    /// Materialize the exact closed difference of this view minus `right`.
    pub fn difference(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.mesh.difference(right.mesh)
    }

    /// Materialize the exact closed symmetric difference of this view and `right`.
    pub fn xor(self, right: ExactMeshRef<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.mesh.xor(right.mesh)
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
        PreparedMeshPairView {
            left: self,
            right,
            plan,
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
        Self {
            left,
            right,
            plan,
            scratch: RefCell::new(BroadPhaseScratch::default()),
            face_pair_classifications: RefCell::new(None),
            intersection_graph: RefCell::new(None),
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
        }
    }

    /// Return a bounded storage hint for candidate face-pair traversal.
    pub fn candidate_face_pair_capacity_hint(&self) -> usize {
        self.as_view().candidate_face_pair_capacity_hint()
    }

    /// Return a cheap summary of retained facts in this prepared pair session.
    pub fn cache_status(&self) -> PreparedMeshPairCacheStatus {
        PreparedMeshPairCacheStatus {
            candidate_pair_capacity_hint: self.candidate_face_pair_capacity_hint(),
            retains_face_pair_classifications: self.face_pair_classifications.borrow().is_some(),
            retains_intersection_graph: self.intersection_graph.borrow().is_some(),
            intersection_graph_source_validated: *self.intersection_graph_validated.borrow(),
            retains_arrangement_shortcut_facts: self.arrangement_shortcut_facts.borrow().is_some(),
            retains_union_result: self.union_result.borrow().is_some(),
            retains_intersection_result: self.intersection_result.borrow().is_some(),
            retains_difference_result: self.difference_result.borrow().is_some(),
            retains_xor_result: self.xor_result.borrow().is_some(),
        }
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

    pub(crate) fn cached_intersection_graph(&self) -> Option<ExactIntersectionGraph> {
        self.intersection_graph.borrow().clone()
    }

    pub(crate) fn retain_intersection_graph(&self, graph: ExactIntersectionGraph) {
        *self.intersection_graph.borrow_mut() = Some(graph);
        *self.intersection_graph_validated.borrow_mut() = false;
    }

    pub(crate) fn has_validated_intersection_graph(&self) -> bool {
        *self.intersection_graph_validated.borrow()
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
    pub fn candidate_face_pair_capacity_hint(&self) -> usize {
        self.plan
            .bounded_capacity_hint(self.left.view.face_count(), self.right.view.face_count())
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

impl<'a> FaceRef<'a> {
    /// Face index in the source mesh.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Triangle vertex indices for this face.
    pub fn vertex_indices(self) -> [usize; 3] {
        self.mesh.triangles()[self.index].0
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
