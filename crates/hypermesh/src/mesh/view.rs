//! Borrowed exact views of retained mesh data.

use std::{cell::RefCell, rc::Rc};

use super::ExactMesh;
use super::arrangement3d::regularization::ExactRegularizationPolicy;
use super::arrangement3d::{ArrangementView, ExactArrangement};
use super::boolean::evidence::ExactArrangementCellComplexShortcutFacts;
use super::boolean::{
    ExactBooleanOperation, ExactBooleanRequest,
    materialize_boolean_exact_request_with_prepared_pair,
};
use super::bounds::{
    BroadPhaseScratch, CandidateFacePairPlan, ExactAabb3, ExactAabbBroadPhase, ExactBroadPhase,
    PreparedMeshBounds,
};
use super::error::ExactMeshError;
use super::error::{ExactMeshBlocker, ExactMeshBlockerKind};
use super::graph::{ExactIntersectionGraph, build_validated_intersection_graph_from_prepared_pair};
use super::validation::ExactMeshValidationPolicy;
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

/// Borrowed exact mesh view with prepared broad-phase acceleration facts.
#[derive(Debug)]
pub(crate) struct PreparedMeshView<'a> {
    view: MeshView<'a>,
    bounds: PreparedMeshBounds<'a>,
}

/// Owned borrowed mesh-pair cache with certificate-validated broad-phase facts.
#[derive(Debug)]
pub struct PreparedMeshPair<'left, 'right> {
    left: PreparedMeshView<'left>,
    right: PreparedMeshView<'right>,
    plan: CandidateFacePairPlan,
    left_source: ExactMeshSourceStamp,
    right_source: ExactMeshSourceStamp,
    candidate_pair_capacity_hint: usize,
    scratch: RefCell<BroadPhaseScratch>,
    candidate_face_pairs: RefCell<Option<Vec<[usize; 2]>>>,
    intersection_graph: RefCell<Option<Rc<ExactIntersectionGraph>>>,
    intersection_graph_validated: RefCell<bool>,
    arrangement: RefCell<Option<Rc<ExactArrangement>>>,
    arrangement_shortcut_facts: RefCell<Option<ExactArrangementCellComplexShortcutFacts>>,
    union_result: PreparedMeshPairResultCache,
    intersection_result: PreparedMeshPairResultCache,
    difference_result: PreparedMeshPairResultCache,
    xor_result: PreparedMeshPairResultCache,
}

#[derive(Debug, Default)]
struct PreparedMeshPairResultCache {
    result: RefCell<Option<Result<ExactMesh, ExactMeshError>>>,
}

impl PreparedMeshPairResultCache {
    fn cached(&self) -> Option<Result<ExactMesh, ExactMeshError>> {
        self.result.borrow().clone()
    }

    fn retain(&self, result: &Result<ExactMesh, ExactMeshError>) {
        *self.result.borrow_mut() = Some(result.clone());
    }

    fn clear(&self) {
        *self.result.borrow_mut() = None;
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PreparedMeshPairNamedBoolean {
    Union,
    Intersection,
    Difference,
}

impl PreparedMeshPairNamedBoolean {
    const fn exact_operation(self) -> ExactBooleanOperation {
        match self {
            Self::Union => ExactBooleanOperation::Union,
            Self::Intersection => ExactBooleanOperation::Intersection,
            Self::Difference => ExactBooleanOperation::Difference,
        }
    }
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
    let mut hash = source_provenance_identity(mesh.provenance());

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
        hash = fnv1a_point3(hash, vertex);
    }
    for triangle in mesh.triangles() {
        hash = fnv1a_u64(hash, triangle.0[0] as u64);
        hash = fnv1a_u64(hash, triangle.0[1] as u64);
        hash = fnv1a_u64(hash, triangle.0[2] as u64);
    }

    hash
}

fn source_provenance_identity(provenance: &hyperlimit::ConstructionProvenance) -> u64 {
    let mut hash = 0xcbf29ce484222325u64;
    hash = fnv1a_u64(hash, mesh_source_tag(provenance.source.source));
    hash = fnv1a_u64(
        hash,
        approximation_policy_tag(provenance.source.approximation),
    );
    fnv1a_str(hash, provenance.source.label.as_str())
}

fn fnv1a_point3(mut hash: u64, point: &Point3) -> u64 {
    hash = fnv1a_real(hash, &point.x);
    hash = fnv1a_real(hash, &point.y);
    fnv1a_real(hash, &point.z)
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
        self.mesh.bounds().mesh().map(bounds_corners)
    }

    /// Borrow retained bounds for one face as exact min/max corners.
    pub fn face_bounds(self, index: usize) -> Option<(&'a Point3, &'a Point3)> {
        self.mesh.bounds().face(index).map(bounds_corners)
    }

    /// Borrow retained bounds for one edge as exact min/max corners.
    pub fn edge_bounds(self, index: usize) -> Option<(&'a Point3, &'a Point3)> {
        self.mesh.bounds().edge(index).map(bounds_corners)
    }

    /// Borrow retained bounds for one face, returning a typed blocker when absent.
    pub fn require_face_bounds(
        self,
        index: usize,
    ) -> Result<(&'a Point3, &'a Point3), ExactMeshError> {
        self.face_bounds(index)
            .ok_or_else(|| missing_retained_face_bounds("face", index))
    }

    /// Borrow retained bounds for one edge, returning a typed blocker when absent.
    pub fn require_edge_bounds(
        self,
        index: usize,
    ) -> Result<(&'a Point3, &'a Point3), ExactMeshError> {
        self.edge_bounds(index)
            .ok_or_else(|| missing_retained_edge_bounds(self.mesh, index))
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

    fn prepare_broad_phase(self) -> Result<PreparedMeshView<'a>, ExactMeshError> {
        self.validate_retained_bounds_certificate()?;
        Ok(PreparedMeshView {
            view: self,
            bounds: self.mesh.bounds().prepare(),
        })
    }

    /// Prepare certificate-validated broad-phase facts for this mesh pair.
    pub fn prepare_broad_phase_pair<'b>(
        self,
        right: MeshView<'b>,
    ) -> Result<PreparedMeshPair<'a, 'b>, ExactMeshError> {
        let left = self.prepare_broad_phase()?;
        let right = right.prepare_broad_phase()?;
        Ok(PreparedMeshPair::new(left, right))
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
        self.prepare_broad_phase_pair(right)?.union()
    }

    /// Materialize the exact closed intersection of this view and `right`.
    pub fn intersection(self, right: MeshView<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.prepare_broad_phase_pair(right)?.intersection()
    }

    /// Materialize the exact closed difference of this view minus `right`.
    pub fn difference(self, right: MeshView<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.prepare_broad_phase_pair(right)?.difference()
    }

    /// Materialize the exact closed symmetric difference of this view and `right`.
    pub fn xor(self, right: MeshView<'_>) -> Result<ExactMesh, ExactMeshError> {
        self.prepare_broad_phase_pair(right)?.xor()
    }
}

impl<'a> PreparedMeshView<'a> {
    /// Return the underlying borrowed mesh view.
    pub(crate) const fn view(&self) -> MeshView<'a> {
        self.view
    }

    pub(crate) fn retained_pair_plan<'right>(
        &self,
        right: &PreparedMeshView<'right>,
    ) -> (CandidateFacePairPlan, usize) {
        let broad_phase = ExactAabbBroadPhase::default();
        let plan = broad_phase.candidate_face_pair_plan(&self.bounds, &right.bounds);
        let candidate_pair_capacity_hint =
            plan.bounded_capacity_hint(self.view.face_count(), right.view.face_count());
        (plan, candidate_pair_capacity_hint)
    }
}

impl<'left, 'right> PreparedMeshPair<'left, 'right> {
    fn new(left: PreparedMeshView<'left>, right: PreparedMeshView<'right>) -> Self {
        let left_source = left.view.source_stamp();
        let right_source = right.view.source_stamp();
        let (plan, candidate_pair_capacity_hint) = left.retained_pair_plan(&right);
        Self {
            left,
            right,
            plan,
            left_source,
            right_source,
            candidate_pair_capacity_hint,
            scratch: RefCell::new(BroadPhaseScratch::default()),
            candidate_face_pairs: RefCell::new(None),
            intersection_graph: RefCell::new(None),
            intersection_graph_validated: RefCell::new(false),
            arrangement: RefCell::new(None),
            arrangement_shortcut_facts: RefCell::new(None),
            union_result: PreparedMeshPairResultCache::default(),
            intersection_result: PreparedMeshPairResultCache::default(),
            difference_result: PreparedMeshPairResultCache::default(),
            xor_result: PreparedMeshPairResultCache::default(),
        }
    }

    pub(crate) const fn left(&self) -> &PreparedMeshView<'left> {
        &self.left
    }

    pub(crate) const fn right(&self) -> &PreparedMeshView<'right> {
        &self.right
    }

    pub(crate) const fn candidate_pair_capacity_hint(&self) -> usize {
        self.candidate_pair_capacity_hint
    }

    fn sources_current(&self) -> bool {
        self.left_source == self.left.view.source_stamp()
            && self.right_source == self.right.view.source_stamp()
    }

    fn require_current_retained(
        &self,
        retained: bool,
        fact: &'static str,
    ) -> Result<(), ExactMeshError> {
        retained_current_state(retained, self.sources_current()).require_current(fact)
    }

    /// Borrow retained broad-phase candidate pairs without rebuilding missing evidence.
    pub fn with_current_candidate_face_pairs<R>(
        &self,
        query: impl FnOnce(&[[usize; 2]]) -> R,
    ) -> Result<R, ExactMeshError> {
        self.require_current_retained(
            self.candidate_face_pairs.borrow().is_some(),
            "broad-phase candidate face pairs",
        )?;
        let candidate_face_pairs = self.candidate_face_pairs.borrow();
        let pairs = candidate_face_pairs.as_deref().ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "prepared mesh-pair session retained broad-phase candidate-pair state without candidate records",
            ))
        })?;
        Ok(query(pairs))
    }

    /// Build and retain broad-phase candidate face pairs, then run `query` on them.
    pub fn with_candidate_face_pairs<R>(
        &self,
        query: impl FnOnce(&[[usize; 2]]) -> R,
    ) -> Result<R, ExactMeshError> {
        self.ensure_candidate_face_pairs();
        self.with_current_candidate_face_pairs(query)
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
        self.require_current_retained(self.arrangement.borrow().is_some(), "arrangement")?;
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
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )?;
        let arrangement = Rc::new(arrangement);
        *self.arrangement.borrow_mut() = Some(Rc::clone(&arrangement));
        Ok(arrangement)
    }

    pub(crate) fn retained_intersection_graph(
        &self,
        require_current_certificate: bool,
    ) -> Option<Rc<ExactIntersectionGraph>> {
        let graph = self.intersection_graph.borrow().clone()?;
        if require_current_certificate
            && retained_certificate_state(
                *self.intersection_graph_validated.borrow(),
                self.sources_current(),
            ) != PreparedMeshPairFactState::Current
        {
            return None;
        }
        Some(graph)
    }

    pub(crate) fn with_retained_arrangement<R>(
        &self,
        query: impl FnOnce(&ExactArrangement) -> R,
    ) -> Option<R> {
        let arrangement = self.arrangement.borrow();
        arrangement
            .as_ref()
            .map(|arrangement| query(arrangement.as_ref()))
    }

    pub(crate) fn retain_intersection_graph(
        &self,
        graph: ExactIntersectionGraph,
    ) -> Rc<ExactIntersectionGraph> {
        let graph = Rc::new(graph);
        *self.intersection_graph.borrow_mut() = Some(Rc::clone(&graph));
        *self.intersection_graph_validated.borrow_mut() = false;
        self.clear_graph_dependent_retained_facts();
        graph
    }

    fn clear_graph_dependent_retained_facts(&self) {
        *self.arrangement.borrow_mut() = None;
        self.union_result.clear();
        self.intersection_result.clear();
        self.difference_result.clear();
        self.xor_result.clear();
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

    /// Materialize the exact closed union using this retained pair session.
    pub fn union(&self) -> Result<ExactMesh, ExactMeshError> {
        self.named_boolean_mesh(PreparedMeshPairNamedBoolean::Union)
    }

    /// Materialize the exact closed intersection using this retained pair session.
    pub fn intersection(&self) -> Result<ExactMesh, ExactMeshError> {
        self.named_boolean_mesh(PreparedMeshPairNamedBoolean::Intersection)
    }

    /// Materialize the exact closed difference of the left mesh minus the right mesh.
    pub fn difference(&self) -> Result<ExactMesh, ExactMeshError> {
        self.named_boolean_mesh(PreparedMeshPairNamedBoolean::Difference)
    }

    /// Materialize the exact closed symmetric difference of the prepared meshes.
    pub fn xor(&self) -> Result<ExactMesh, ExactMeshError> {
        if let Some(result) = self.xor_result.cached() {
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
        self.xor_result.retain(&result);
        result
    }

    fn named_boolean_mesh(
        &self,
        operation: PreparedMeshPairNamedBoolean,
    ) -> Result<ExactMesh, ExactMeshError> {
        if let Some(result) = self.cached_named_boolean_mesh(operation) {
            return result;
        }

        let request = ExactBooleanRequest::new(
            operation.exact_operation(),
            ExactMeshValidationPolicy::CLOSED,
        );
        let result = materialize_boolean_exact_request_with_prepared_pair(self, request)
            .map(|result| result.into_mesh());
        self.retain_named_boolean_mesh(operation, &result);
        result
    }

    fn cached_named_boolean_mesh(
        &self,
        operation: PreparedMeshPairNamedBoolean,
    ) -> Option<Result<ExactMesh, ExactMeshError>> {
        self.named_boolean_cache(operation).cached()
    }

    fn retain_named_boolean_mesh(
        &self,
        operation: PreparedMeshPairNamedBoolean,
        result: &Result<ExactMesh, ExactMeshError>,
    ) {
        self.named_boolean_cache(operation).retain(result);
    }

    fn named_boolean_cache(
        &self,
        operation: PreparedMeshPairNamedBoolean,
    ) -> &PreparedMeshPairResultCache {
        match operation {
            PreparedMeshPairNamedBoolean::Union => &self.union_result,
            PreparedMeshPairNamedBoolean::Intersection => &self.intersection_result,
            PreparedMeshPairNamedBoolean::Difference => &self.difference_result,
        }
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
        self.try_visit_candidate_face_pairs_uncached(&mut |pair| visit(pair))?;
        Ok(())
    }

    fn ensure_candidate_face_pairs(&self) {
        if self.candidate_face_pairs.borrow().is_some() {
            return;
        }

        let mut candidate_face_pairs = Vec::with_capacity(self.candidate_pair_capacity_hint);
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

    pub(crate) fn try_visit_unretained_candidate_face_pairs<E>(
        &self,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<(), E> {
        self.try_visit_candidate_face_pairs_uncached(visit)
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
        require_retained_vertex_incident_faces(
            self.mesh,
            self.index,
            &facts.incident_face_indices,
        )?;
        Ok(facts.incident_face_indices.as_slice())
    }

    /// Retained incident edge indices in canonical edge-fact order.
    pub fn incident_edge_indices(self) -> Result<&'a [usize], ExactMeshError> {
        let facts = retained_vertex_facts(self.mesh, self.index)?;
        require_retained_vertex_incident_edges(
            self.mesh,
            self.index,
            &facts.incident_edge_indices,
        )?;
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

    /// Borrow retained edge bounds as exact min/max corners.
    pub fn bounds(self) -> Result<(&'a Point3, &'a Point3), ExactMeshError> {
        self.mesh
            .bounds()
            .edge(self.index)
            .map(bounds_corners)
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

fn missing_retained_edge_bounds(mesh: &ExactMesh, edge: usize) -> ExactMeshError {
    let mut blocker = ExactMeshBlocker::new(
        ExactMeshBlockerKind::MissingRequiredEvidence,
        format!("mesh edge {edge} has no retained exact bounds"),
    );
    if let Some(facts) = mesh.facts().edges.get(edge) {
        blocker = blocker.with_edge(facts.vertices);
    }
    ExactMeshError::one(blocker)
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

fn require_retained_vertex_incident_faces(
    mesh: &ExactMesh,
    vertex: usize,
    faces: &[usize],
) -> Result<(), ExactMeshError> {
    for &face in faces {
        if face >= mesh.facts().faces.len() {
            return Err(ExactMeshError::one(
                ExactMeshBlocker::new(
                    ExactMeshBlockerKind::StaleFactReplay,
                    format!(
                        "retained mesh vertex {vertex} references incident face {face}, but only {} retained faces exist",
                        mesh.facts().faces.len()
                    ),
                )
                .with_vertex(vertex)
                .with_face(face),
            ));
        }
    }
    Ok(())
}

fn require_retained_vertex_incident_edges(
    mesh: &ExactMesh,
    vertex: usize,
    edges: &[usize],
) -> Result<(), ExactMeshError> {
    for &edge in edges {
        if edge >= mesh.facts().edges.len() {
            return Err(ExactMeshError::one(
                ExactMeshBlocker::new(
                    ExactMeshBlockerKind::StaleFactReplay,
                    format!(
                        "retained mesh vertex {vertex} references incident edge row {edge}, but only {} retained edges exist",
                        mesh.facts().edges.len()
                    ),
                )
                .with_vertex(vertex),
            ));
        }
    }
    Ok(())
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
