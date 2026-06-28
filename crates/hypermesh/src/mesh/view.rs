//! Borrowed exact views of retained mesh data.

use std::{cell::RefCell, rc::Rc};

use super::ExactMesh;
use super::arrangement3d::regularization::ExactRegularizationPolicy;
use super::arrangement3d::{ArrangementView, ExactArrangement};
use super::boolean::evidence::ExactArrangementCellComplexShortcutFacts;
use super::boolean::{ExactBooleanOperation, materialize_closed_named_boolean_with_prepared_pair};
use super::bounds::{BroadPhaseScratch, CandidateFacePairPlan, PreparedMeshBounds};
use super::error::ExactMeshError;
use super::error::{ExactMeshBlocker, ExactMeshBlockerKind};
use super::graph::{ExactIntersectionGraph, build_validated_intersection_graph_from_prepared_pair};
use hyperlimit::{ApproximationPolicy, MeshSource, Point3, PredicateUse};
use hyperreal::Real;

/// Borrowed exact view of an [`ExactMesh`].
#[derive(Clone, Copy, Debug)]
pub struct MeshView<'a> {
    mesh: &'a ExactMesh,
}

/// Borrowed face/triangle view.
#[derive(Clone, Copy, Debug)]
pub struct FaceRef<'a> {
    mesh: &'a ExactMesh,
    index: usize,
}

/// Alias for the borrowed triangle view.
pub type TriangleRef<'a> = FaceRef<'a>;

/// Borrowed vertex view.
#[derive(Clone, Copy, Debug)]
pub struct VertexRef<'a> {
    mesh: &'a ExactMesh,
    index: usize,
}

/// Borrowed edge view.
#[derive(Clone, Copy, Debug)]
pub struct EdgeRef<'a> {
    mesh: &'a ExactMesh,
    index: usize,
}

/// Owned borrowed mesh-pair cache with certificate-validated broad-phase facts.
#[derive(Debug)]
pub(crate) struct PreparedMeshPair<'left, 'right> {
    left_view: MeshView<'left>,
    right_view: MeshView<'right>,
    left_bounds: PreparedMeshBounds<'left>,
    right_bounds: PreparedMeshBounds<'right>,
    plan: CandidateFacePairPlan,
    left_source: ExactMeshSourceStamp,
    right_source: ExactMeshSourceStamp,
    candidate_pair_capacity_hint: usize,
    scratch: RefCell<BroadPhaseScratch>,
    intersection_graph: RefCell<Option<Rc<ExactIntersectionGraph>>>,
    intersection_graph_validated: RefCell<bool>,
    arrangement: RefCell<Option<Rc<ExactArrangement>>>,
    arrangement_shortcut_facts: RefCell<Option<ExactArrangementCellComplexShortcutFacts>>,
}

/// Compact source/freshness stamp for retained exact mesh facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ExactMeshSourceStamp {
    source: MeshSource,
    approximation: ApproximationPolicy,
    source_identity: u64,
    construction_version: u64,
    vertex_count: usize,
    edge_count: usize,
    face_count: usize,
}

impl ExactMeshSourceStamp {
    fn from_mesh(mesh: &ExactMesh) -> Self {
        let provenance = mesh.provenance();
        Self {
            source: provenance.source.source,
            approximation: provenance.source.approximation,
            source_identity: exact_mesh_source_identity(mesh),
            construction_version: provenance.construction_version,
            vertex_count: mesh.facts().mesh.vertex_count,
            edge_count: mesh.facts().mesh.edge_count,
            face_count: mesh.facts().mesh.face_count,
        }
    }
}

fn exact_mesh_source_identity(mesh: &ExactMesh) -> u64 {
    let facts = &mesh.facts().mesh;
    let provenance = mesh.provenance();
    let mut hash = 0xcbf29ce484222325u64;
    hash = fnv1a_u64(
        hash,
        match provenance.source.source {
            MeshSource::Exact => 0x01,
            MeshSource::LossyF64 => 0x02,
            MeshSource::HypermeshAdapter => 0x03,
            MeshSource::ExternalAdapter => 0x04,
        },
    );
    hash = fnv1a_u64(
        hash,
        match provenance.source.approximation {
            ApproximationPolicy::ExactOnly => 0x11,
            ApproximationPolicy::EdgeOnly => 0x12,
            ApproximationPolicy::ExplicitApproximateDecision => 0x13,
        },
    );
    hash = fnv1a_str(hash, provenance.source.label.as_str());

    hash = fnv1a_u64(hash, facts.vertex_count as u64);
    hash = fnv1a_u64(hash, facts.face_count as u64);
    hash = fnv1a_u64(hash, facts.edge_count as u64);
    hash = fnv1a_u64(hash, facts.euler_characteristic as i64 as u64);
    hash = fnv1a_u64(hash, facts.boundary_edges as u64);
    hash = fnv1a_u64(hash, facts.non_manifold_edges as u64);
    hash = fnv1a_u64(hash, facts.duplicate_directed_edges as u64);
    hash = fnv1a_u64(hash, facts.degenerate_triangles as u64);
    hash = fnv1a_u64(hash, facts.non_manifold_vertices as u64);
    hash = fnv1a_u64(hash, facts.closed_manifold as u64);
    hash = fnv1a_u64(hash, facts.fixed_coordinates_exact_rational as u64);

    for vertex in mesh.vertices() {
        hash = fnv1a_real(hash, &vertex.x);
        hash = fnv1a_real(hash, &vertex.y);
        hash = fnv1a_real(hash, &vertex.z);
    }
    for triangle in mesh.triangles() {
        hash = fnv1a_u64(hash, triangle.0[0] as u64);
        hash = fnv1a_u64(hash, triangle.0[1] as u64);
        hash = fnv1a_u64(hash, triangle.0[2] as u64);
    }

    hash
}

fn fnv1a_real(hash: u64, value: &Real) -> u64 {
    if let Some(rational) = value.exact_rational_ref() {
        let hash = fnv1a_u64(hash, 0x524154);
        fnv1a_str(hash, &rational.to_string())
    } else {
        let hash = fnv1a_u64(hash, 0x5245414c);
        fnv1a_str(hash, &format!("{value:?}"))
    }
}

fn fnv1a_str(mut hash: u64, text: &str) -> u64 {
    hash = fnv1a_u64(hash, text.len() as u64);
    for &byte in text.as_bytes() {
        hash = fnv1a_u8(hash, byte);
    }
    hash
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

impl PreparedMeshPairFactState {
    /// Convert a non-current state into a typed blocker for callers that require a current fact.
    fn blocker(self, fact: &'static str) -> Option<ExactMeshBlocker> {
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
    fn require_current(self, fact: &'static str) -> Result<(), ExactMeshError> {
        self.blocker(fact)
            .map_or(Ok(()), |blocker| Err(ExactMeshError::one(blocker)))
    }
}

impl<'a> MeshView<'a> {
    /// Borrow an exact mesh as a replayable view.
    pub(crate) const fn new(mesh: &'a ExactMesh) -> Self {
        Self { mesh }
    }

    pub(crate) const fn mesh(self) -> &'a ExactMesh {
        self.mesh
    }

    fn source_stamp(self) -> ExactMeshSourceStamp {
        ExactMeshSourceStamp::from_mesh(self.mesh)
    }

    /// Return exact vertices.
    pub fn vertices(self) -> &'a [Point3] {
        self.mesh.vertices()
    }

    /// Borrow retained whole-mesh bounds as exact min/max corners.
    pub fn mesh_bounds(self) -> Option<(&'a Point3, &'a Point3)> {
        self.mesh
            .bounds()
            .mesh()
            .map(|bounds| (&bounds.min, &bounds.max))
    }

    /// Borrow retained bounds for one face as exact min/max corners.
    pub fn face_bounds(self, index: usize) -> Option<(&'a Point3, &'a Point3)> {
        self.mesh
            .bounds()
            .face(index)
            .map(|bounds| (&bounds.min, &bounds.max))
    }

    /// Borrow retained bounds for one edge as exact min/max corners.
    pub fn edge_bounds(self, index: usize) -> Option<(&'a Point3, &'a Point3)> {
        self.mesh
            .bounds()
            .edge(index)
            .map(|bounds| (&bounds.min, &bounds.max))
    }

    /// Borrow retained bounds for one face, returning a typed blocker when absent.
    pub fn require_face_bounds(
        self,
        index: usize,
    ) -> Result<(&'a Point3, &'a Point3), ExactMeshError> {
        self.face_bounds(index)
            .ok_or_else(|| missing_retained_face_bounds(index))
    }

    /// Borrow retained bounds for one edge, returning a typed blocker when absent.
    pub fn require_edge_bounds(
        self,
        index: usize,
    ) -> Result<(&'a Point3, &'a Point3), ExactMeshError> {
        self.edge_bounds(index).ok_or_else(|| {
            let mut blocker = ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                format!("mesh edge {index} has no retained exact bounds"),
            );
            if let Some(facts) = self.mesh.facts().edges.get(index) {
                blocker = blocker.with_edge(facts.vertices);
            }
            ExactMeshError::one(blocker)
        })
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
        self.face(index)
    }

    /// Borrow one triangle by index, returning a typed blocker when absent.
    pub fn require_triangle(self, index: usize) -> Result<TriangleRef<'a>, ExactMeshError> {
        self.require_face(index)
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
        self.faces()
    }

    /// Iterate retained edges.
    pub fn edges(self) -> impl Iterator<Item = EdgeRef<'a>> + 'a {
        (0..self.mesh.facts().edges.len()).map(move |index| EdgeRef {
            mesh: self.mesh,
            index,
        })
    }

    /// Prepare certificate-validated broad-phase facts for this mesh pair.
    pub(crate) fn prepare_broad_phase_pair<'b>(
        self,
        right: MeshView<'b>,
    ) -> Result<PreparedMeshPair<'a, 'b>, ExactMeshError> {
        self.validate_retained_bounds_certificate()?;
        right.validate_retained_bounds_certificate()?;
        let left_bounds = self.mesh.bounds().prepare();
        let right_bounds = right.mesh.bounds().prepare();
        Ok(PreparedMeshPair::new(
            self,
            right,
            left_bounds,
            right_bounds,
        ))
    }

    /// Build a retained arrangement for this mesh pair and query its borrowed view.
    pub fn with_arrangement_view<R>(
        self,
        right: MeshView<'_>,
        query: impl for<'arrangement> FnOnce(ArrangementView<'arrangement>) -> R,
    ) -> Result<R, ExactMeshError> {
        let pair = self.prepare_broad_phase_pair(right)?;
        pair.with_arrangement_view(query)
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
    pub fn union(self, right: MeshView<'_>) -> Result<ExactMesh, ExactMeshError> {
        let pair = self.prepare_broad_phase_pair(right)?;
        materialize_closed_named_boolean_with_prepared_pair(&pair, ExactBooleanOperation::Union)
    }

    /// Materialize the exact closed intersection of this view and `right`.
    pub fn intersection(self, right: MeshView<'_>) -> Result<ExactMesh, ExactMeshError> {
        let pair = self.prepare_broad_phase_pair(right)?;
        materialize_closed_named_boolean_with_prepared_pair(
            &pair,
            ExactBooleanOperation::Intersection,
        )
    }

    /// Materialize the exact closed difference of this view minus `right`.
    pub fn difference(self, right: MeshView<'_>) -> Result<ExactMesh, ExactMeshError> {
        let pair = self.prepare_broad_phase_pair(right)?;
        materialize_closed_named_boolean_with_prepared_pair(
            &pair,
            ExactBooleanOperation::Difference,
        )
    }

    /// Materialize the exact closed symmetric difference of this view and `right`.
    pub fn xor(self, right: MeshView<'_>) -> Result<ExactMesh, ExactMeshError> {
        let left_only = self.difference(right)?;
        let right_only = right.difference(self)?;
        left_only.view().union(right_only.view())
    }
}

impl<'left, 'right> PreparedMeshPair<'left, 'right> {
    fn new(
        left_view: MeshView<'left>,
        right_view: MeshView<'right>,
        left_bounds: PreparedMeshBounds<'left>,
        right_bounds: PreparedMeshBounds<'right>,
    ) -> Self {
        let left_source = left_view.source_stamp();
        let right_source = right_view.source_stamp();
        let plan = left_bounds.candidate_face_pair_plan(&right_bounds);
        let candidate_pair_capacity_hint =
            plan.bounded_capacity_hint(left_view.face_count(), right_view.face_count());
        Self {
            left_view,
            right_view,
            left_bounds,
            right_bounds,
            plan,
            left_source,
            right_source,
            candidate_pair_capacity_hint,
            scratch: RefCell::new(BroadPhaseScratch::default()),
            intersection_graph: RefCell::new(None),
            intersection_graph_validated: RefCell::new(false),
            arrangement: RefCell::new(None),
            arrangement_shortcut_facts: RefCell::new(None),
        }
    }

    pub(crate) const fn left_mesh(&self) -> &'left ExactMesh {
        self.left_view.mesh()
    }

    pub(crate) const fn right_mesh(&self) -> &'right ExactMesh {
        self.right_view.mesh()
    }

    pub(crate) const fn candidate_pair_capacity_hint(&self) -> usize {
        self.candidate_pair_capacity_hint
    }

    fn sources_current(&self) -> bool {
        self.left_source == self.left_view.source_stamp()
            && self.right_source == self.right_view.source_stamp()
    }

    /// Build a retained arrangement from this pair session and run `query` on its borrowed view.
    ///
    /// The pair's retained intersection graph is source-certified first. The
    /// arrangement builder then consumes that current graph certificate instead
    /// of replay-building the graph from the source meshes.
    pub(crate) fn with_arrangement_view<R>(
        &self,
        query: impl for<'arrangement> FnOnce(ArrangementView<'arrangement>) -> R,
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
            self.left_mesh(),
            self.right_mesh(),
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )?;
        let arrangement = Rc::new(arrangement);
        *self.arrangement.borrow_mut() = Some(Rc::clone(&arrangement));
        Ok(arrangement)
    }

    pub(crate) fn current_intersection_graph(
        &self,
    ) -> Result<Rc<ExactIntersectionGraph>, ExactMeshError> {
        let state = if self.intersection_graph.borrow().is_none() {
            PreparedMeshPairFactState::Missing
        } else if !self.sources_current() {
            PreparedMeshPairFactState::Stale
        } else if *self.intersection_graph_validated.borrow() {
            PreparedMeshPairFactState::Current
        } else {
            PreparedMeshPairFactState::CertificateBlocked
        };
        state.require_current("intersection graph")?;
        self.intersection_graph.borrow().clone().ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session retained intersection graph state without graph records",
            ))
        })
    }

    pub(crate) fn retained_intersection_graph_for_validation(
        &self,
    ) -> Result<Option<Rc<ExactIntersectionGraph>>, ExactMeshError> {
        if !self.sources_current() {
            let state = if self.intersection_graph.borrow().is_some() {
                PreparedMeshPairFactState::Stale
            } else {
                PreparedMeshPairFactState::Missing
            };
            state.require_current("intersection graph")?;
        }
        Ok(self.intersection_graph.borrow().clone())
    }

    pub(crate) fn retained_arrangement_for_reuse(&self) -> Option<Rc<ExactArrangement>> {
        self.arrangement.borrow().clone()
    }

    pub(crate) fn retain_intersection_graph(
        &self,
        graph: ExactIntersectionGraph,
    ) -> Rc<ExactIntersectionGraph> {
        let graph = Rc::new(graph);
        *self.intersection_graph.borrow_mut() = Some(Rc::clone(&graph));
        *self.intersection_graph_validated.borrow_mut() = false;
        *self.arrangement.borrow_mut() = None;
        graph
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
            self.left_mesh(),
            self.right_mesh(),
        );
        *self.arrangement_shortcut_facts.borrow_mut() = Some(facts.clone());
        facts
    }

    pub(crate) fn try_visit_candidate_face_pairs_uncached<E>(
        &self,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<(), E> {
        if let Ok(mut scratch) = self.scratch.try_borrow_mut() {
            return self
                .left_bounds
                .try_visit_candidate_face_pairs_with_plan_and_scratch(
                    &self.right_bounds,
                    self.plan,
                    &mut scratch,
                    visit,
                );
        }

        let mut local_scratch = BroadPhaseScratch::default();
        self.left_bounds
            .try_visit_candidate_face_pairs_with_plan_and_scratch(
                &self.right_bounds,
                self.plan,
                &mut local_scratch,
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

    /// Retained incident face indices in face order.
    pub fn incident_face_indices(self) -> Result<&'a [usize], ExactMeshError> {
        let facts = retained_vertex_facts(self.mesh, self.index)?;
        for &face in &facts.incident_face_indices {
            if face >= self.mesh.facts().faces.len() {
                return Err(ExactMeshError::one(
                    ExactMeshBlocker::new(
                        ExactMeshBlockerKind::StaleFactReplay,
                        format!(
                            "retained mesh vertex {} references incident face {face}, but only {} retained faces exist",
                            self.index,
                            self.mesh.facts().faces.len()
                        ),
                    )
                    .with_vertex(self.index)
                    .with_face(face),
                ));
            }
        }
        Ok(facts.incident_face_indices.as_slice())
    }

    /// Retained incident edge indices in canonical edge-fact order.
    pub fn incident_edge_indices(self) -> Result<&'a [usize], ExactMeshError> {
        let facts = retained_vertex_facts(self.mesh, self.index)?;
        for &edge in &facts.incident_edge_indices {
            if edge >= self.mesh.facts().edges.len() {
                return Err(ExactMeshError::one(
                    ExactMeshBlocker::new(
                        ExactMeshBlockerKind::StaleFactReplay,
                        format!(
                            "retained mesh vertex {} references incident edge row {edge}, but only {} retained edges exist",
                            self.index,
                            self.mesh.facts().edges.len()
                        ),
                    )
                    .with_vertex(self.index),
                ));
            }
        }
        Ok(facts.incident_edge_indices.as_slice())
    }

    /// Iterate borrowed incident faces from retained adjacency facts.
    pub fn incident_faces(
        self,
    ) -> Result<impl ExactSizeIterator<Item = FaceRef<'a>> + 'a, ExactMeshError> {
        let indices = self.incident_face_indices()?;
        Ok(indices.iter().copied().map(move |index| FaceRef {
            mesh: self.mesh,
            index,
        }))
    }

    /// Iterate borrowed incident edges from retained adjacency facts.
    pub fn incident_edges(
        self,
    ) -> Result<impl ExactSizeIterator<Item = EdgeRef<'a>> + 'a, ExactMeshError> {
        let indices = self.incident_edge_indices()?;
        Ok(indices.iter().copied().map(move |index| EdgeRef {
            mesh: self.mesh,
            index,
        }))
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
            .map(|bounds| (&bounds.min, &bounds.max))
            .ok_or_else(|| missing_retained_face_bounds(self.index))
    }

    /// Borrow the face vertices.
    pub fn vertex_refs(self) -> Result<[VertexRef<'a>; 3], ExactMeshError> {
        let triangle = self.vertex_indices();
        let [a, b, c] = triangle;
        let vertex_count = self.mesh.vertices().len();
        for vertex in triangle {
            if vertex >= vertex_count {
                return Err(retained_face_vertex_error(self.index, triangle, vertex));
            }
        }
        Ok([
            VertexRef {
                mesh: self.mesh,
                index: a,
            },
            VertexRef {
                mesh: self.mesh,
                index: b,
            },
            VertexRef {
                mesh: self.mesh,
                index: c,
            },
        ])
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
        let triangle = self.vertex_indices();
        let [a, b, c] = triangle;
        let a = self
            .mesh
            .vertices()
            .get(a)
            .ok_or_else(|| retained_face_vertex_error(self.index, triangle, a))?;
        let b = self
            .mesh
            .vertices()
            .get(b)
            .ok_or_else(|| retained_face_vertex_error(self.index, triangle, b))?;
        let c = self
            .mesh
            .vertices()
            .get(c)
            .ok_or_else(|| retained_face_vertex_error(self.index, triangle, c))?;
        Ok([a, b, c])
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
        let edge_vertices @ [a, b] = self.vertex_indices();
        for vertex in edge_vertices {
            if vertex >= self.mesh.vertices().len() {
                return Err(retained_edge_endpoint_error(
                    self.index,
                    edge_vertices,
                    vertex,
                ));
            }
        }
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

    /// Borrow retained edge bounds as exact min/max corners.
    pub fn bounds(self) -> Result<(&'a Point3, &'a Point3), ExactMeshError> {
        self.mesh
            .bounds()
            .edge(self.index)
            .map(|bounds| (&bounds.min, &bounds.max))
            .ok_or_else(|| {
                ExactMeshError::one(
                    ExactMeshBlocker::new(
                        ExactMeshBlockerKind::MissingRequiredEvidence,
                        format!("mesh edge {} has no retained endpoint bounds", self.index),
                    )
                    .with_edge(self.vertex_indices()),
                )
            })
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

fn missing_retained_face_bounds(face: usize) -> ExactMeshError {
    ExactMeshError::one(
        ExactMeshBlocker::new(
            ExactMeshBlockerKind::MissingRequiredEvidence,
            format!("mesh face {face} has no retained exact bounds"),
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

#[cfg(test)]
mod tests {
    use super::*;
    use hyperlimit::SourceProvenance;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn tetra(offset: [i64; 3]) -> ExactMesh {
        let [ox, oy, oz] = offset;
        ExactMesh::new(
            vec![
                p(ox, oy, oz),
                p(ox + 1, oy, oz),
                p(ox, oy + 1, oz),
                p(ox, oy, oz + 1),
            ],
            vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]],
            SourceProvenance::exact("view test tetra"),
        )
        .unwrap()
    }

    #[test]
    fn prepared_mesh_pair_streams_candidate_facts_internally() {
        let left = tetra([0, 0, 0]);
        let overlapping = tetra([0, 0, 0]);
        let disjoint = tetra([5, 0, 0]);

        left.view().validate_retained_bounds().unwrap();
        left.view().validate_retained_bounds_certificate().unwrap();

        let mut disjoint_candidates = Vec::new();
        left.view()
            .prepare_broad_phase_pair(disjoint.view())
            .unwrap()
            .try_visit_candidate_face_pairs_uncached(&mut |pair| {
                disjoint_candidates.push(pair);
                Ok::<(), ()>(())
            })
            .unwrap();
        assert!(disjoint_candidates.is_empty());

        let mut direct_pair_candidates = Vec::new();
        left.view()
            .prepare_broad_phase_pair(overlapping.view())
            .unwrap()
            .try_visit_candidate_face_pairs_uncached(&mut |pair| {
                direct_pair_candidates.push(pair);
                Ok::<(), ()>(())
            })
            .unwrap();
        direct_pair_candidates.sort_unstable();
        assert!(!direct_pair_candidates.is_empty());
        assert!(
            direct_pair_candidates
                .iter()
                .all(|[left_face, right_face]| {
                    *left_face < left.view().face_count()
                        && *right_face < overlapping.view().face_count()
                })
        );

        let mut owned_pair_candidates = Vec::new();
        let prepared_pair = left
            .view()
            .prepare_broad_phase_pair(overlapping.view())
            .unwrap();
        prepared_pair
            .try_visit_candidate_face_pairs_uncached(&mut |pair| {
                owned_pair_candidates.push(pair);
                Ok::<(), ()>(())
            })
            .unwrap();
        owned_pair_candidates.sort_unstable();
        assert_eq!(owned_pair_candidates, direct_pair_candidates);
    }

    #[test]
    fn prepared_pair_uncached_candidate_visitor_can_stop_early() {
        let left = tetra([0, 0, 0]);
        let right = tetra([0, 0, 0]);
        let prepared_pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

        let mut visited = 0;
        let result = prepared_pair.try_visit_candidate_face_pairs_uncached(&mut |_| {
            visited += 1;
            Err("stop")
        });

        assert_eq!(result, Err("stop"));
        assert_eq!(visited, 1);
    }

    #[test]
    fn retained_prepared_arrangement_survives_named_boolean() {
        let left = tetra([0, 0, 0]);
        let right = tetra([1, 0, 0]);
        let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

        pair.with_arrangement_view(|view| {
            view.validate_retained_state().unwrap();
        })
        .unwrap();

        let intersection = left.view().intersection(right.view()).unwrap();
        assert!(pair.retained_arrangement_for_reuse().is_some());
        intersection.view().validate_retained_state().unwrap();
    }
}
