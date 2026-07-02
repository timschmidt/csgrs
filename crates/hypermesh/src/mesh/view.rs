//! Borrowed exact views of retained mesh data.

use super::arrangement3d::ArrangementView;
use super::boolean::{ExactBooleanOperation, materialize_boolean_operation};
use super::error::{MeshBlocker, MeshBlockerKind, MeshError};
use super::prepared::PreparedMeshPair;
use super::validation::MeshValidationMode;
use super::{ExactAffineTransform3, Mesh, Triangle, reverse_triangle};
use hyperlimit::{Point3, PredicateUse, SourceProvenance};
use hyperreal::Real;
use std::cmp::Ordering;

/// Borrowed exact view of an [`Mesh`].
#[derive(Clone, Copy, Debug)]
pub struct MeshView<'a> {
    pub(crate) mesh: &'a Mesh,
}

/// Borrowed face/triangle view.
#[derive(Clone, Copy, Debug)]
pub struct FaceRef<'a> {
    mesh: &'a Mesh,
    index: usize,
}

/// Borrowed triangle geometry view.
#[derive(Clone, Copy, Debug)]
pub struct TriangleRef<'a> {
    mesh: &'a Mesh,
    index: usize,
}

/// Borrowed vertex view.
#[derive(Clone, Copy, Debug)]
pub struct VertexRef<'a> {
    mesh: &'a Mesh,
    index: usize,
}

/// Borrowed edge view.
#[derive(Clone, Copy, Debug)]
pub struct EdgeRef<'a> {
    mesh: &'a Mesh,
    index: usize,
}

impl<'a> MeshView<'a> {
    fn current_row_count(self, kind: RetainedRowKind) -> usize {
        current_row_count(self.mesh, kind)
    }

    const fn vertex_ref(self, index: usize) -> VertexRef<'a> {
        VertexRef {
            mesh: self.mesh,
            index,
        }
    }

    const fn face_ref(self, index: usize) -> FaceRef<'a> {
        FaceRef {
            mesh: self.mesh,
            index,
        }
    }

    const fn triangle_ref(self, index: usize) -> TriangleRef<'a> {
        TriangleRef {
            mesh: self.mesh,
            index,
        }
    }

    const fn edge_ref(self, index: usize) -> EdgeRef<'a> {
        EdgeRef {
            mesh: self.mesh,
            index,
        }
    }

    /// Return exact vertices.
    pub fn vertices(self) -> &'a [Point3] {
        &self.mesh.vertices()[..self.current_row_count(RetainedRowKind::Vertex)]
    }

    /// Borrow retained whole-mesh bounds as exact min/max corners.
    pub fn mesh_bounds(self) -> Result<Option<(&'a Point3, &'a Point3)>, MeshError> {
        require_current_vertex_count(self.mesh)?;
        match (
            self.mesh.vertices().is_empty(),
            self.mesh.bounds().mesh.as_ref(),
        ) {
            (true, None) => Ok(None),
            (true, Some(_)) => Err(MeshError::one(MeshBlocker::new(
                MeshBlockerKind::StaleFactReplay,
                "empty mesh has retained whole-mesh bounds",
            ))),
            (false, Some(bounds)) => Ok(Some((&bounds.min, &bounds.max))),
            (false, None) => Err(MeshError::one(MeshBlocker::new(
                MeshBlockerKind::MissingRequiredEvidence,
                "nonempty mesh has no retained whole-mesh bounds",
            ))),
        }
    }

    /// Borrow retained bounds for one face as exact min/max corners.
    pub fn face_bounds(self, index: usize) -> Result<(&'a Point3, &'a Point3), MeshError> {
        self.face(index)?.bounds()
    }

    /// Borrow retained bounds for one edge as exact min/max corners.
    pub fn edge_bounds(self, index: usize) -> Result<(&'a Point3, &'a Point3), MeshError> {
        self.edge(index)?.bounds()
    }

    /// Borrow one vertex by index.
    pub fn vertex(self, index: usize) -> Result<VertexRef<'a>, MeshError> {
        require_current_row(self.mesh, RetainedRowKind::Vertex, index)?;
        Ok(self.vertex_ref(index))
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
    pub fn validate_retained_state(self) -> Result<(), MeshError> {
        self.mesh.validate_retained_state()
    }

    /// Replay retained exact bounds against the source mesh.
    pub fn validate_retained_bounds(self) -> Result<(), MeshError> {
        self.mesh.validate_retained_bounds()
    }

    /// Validate retained exact bounds without recomputing them.
    pub fn validate_retained_bounds_certificate(self) -> Result<(), MeshError> {
        self.mesh.validate_retained_bounds_certificate()
    }

    /// Borrow one face by index.
    pub fn face(self, index: usize) -> Result<FaceRef<'a>, MeshError> {
        require_current_row(self.mesh, RetainedRowKind::Face, index)?;
        Ok(self.face_ref(index))
    }

    /// Borrow one triangle by retained face index.
    pub fn triangle(self, index: usize) -> Result<TriangleRef<'a>, MeshError> {
        self.face(index).map(|face| face.triangle())
    }

    /// Borrow one retained edge by index.
    pub fn edge(self, index: usize) -> Result<EdgeRef<'a>, MeshError> {
        require_current_row(self.mesh, RetainedRowKind::Edge, index)?;
        Ok(self.edge_ref(index))
    }

    /// Iterate borrowed vertices.
    pub fn vertex_refs(self) -> impl ExactSizeIterator<Item = VertexRef<'a>> + 'a {
        let count = self.current_row_count(RetainedRowKind::Vertex);
        (0..count).map(move |index| self.vertex_ref(index))
    }

    /// Iterate borrowed faces.
    pub fn faces(self) -> impl ExactSizeIterator<Item = FaceRef<'a>> + 'a {
        let count = self.current_row_count(RetainedRowKind::Face);
        (0..count).map(move |index| self.face_ref(index))
    }

    /// Iterate borrowed triangles.
    pub fn triangles(self) -> impl ExactSizeIterator<Item = TriangleRef<'a>> + 'a {
        let count = self.current_row_count(RetainedRowKind::Face);
        (0..count).map(move |index| self.triangle_ref(index))
    }

    /// Iterate retained edges.
    pub fn edges(self) -> impl ExactSizeIterator<Item = EdgeRef<'a>> + 'a {
        let count = self.current_row_count(RetainedRowKind::Edge);
        (0..count).map(move |index| self.edge_ref(index))
    }

    /// Prepare certificate-validated broad-phase facts for this mesh pair.
    pub(crate) fn prepare_broad_phase_pair<'b>(
        self,
        right: MeshView<'b>,
    ) -> Result<PreparedMeshPair<'a, 'b>, MeshError> {
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
    ) -> Result<R, MeshError> {
        let pair = self.prepare_broad_phase_pair(right)?;
        pair.with_arrangement_view(query)
    }

    /// Materialize this view after a row-major exact homogeneous affine transform.
    pub fn transform(self, matrix: [[Real; 4]; 4]) -> Result<Mesh, MeshError> {
        let transform = ExactAffineTransform3::from_homogeneous_rows(matrix)?;
        let vertices = self
            .vertices()
            .iter()
            .map(|point| transform.transform_point(point))
            .collect::<Vec<_>>();
        let triangles = match transform.orientation()? {
            Ordering::Less => self
                .triangles()
                .map(|triangle| reverse_triangle(&Triangle(triangle.vertex_indices())))
                .collect(),
            Ordering::Equal | Ordering::Greater => self
                .triangles()
                .map(|triangle| Triangle(triangle.vertex_indices()))
                .collect(),
        };
        Mesh::new_with_validation_mode_and_version(
            vertices,
            triangles,
            SourceProvenance::exact("exact affine mesh transform"),
            self.mesh.validation_mode(),
            self.mesh.next_construction_version(),
        )
    }

    /// Materialize this view with every triangle orientation reversed.
    pub fn inverse(self) -> Result<Mesh, MeshError> {
        Mesh::new_with_validation_mode_and_version(
            self.vertices().to_vec(),
            self.triangles()
                .map(|triangle| reverse_triangle(&Triangle(triangle.vertex_indices())))
                .collect(),
            SourceProvenance::exact("exact inverse mesh orientation"),
            self.mesh.validation_mode(),
            self.mesh.next_construction_version(),
        )
    }

    /// Materialize the exact closed union of this view and `right`.
    pub fn union(self, right: MeshView<'_>) -> Result<Mesh, MeshError> {
        self.materialize_closed_named_boolean(right, ExactBooleanOperation::Union)
    }

    /// Materialize the exact closed intersection of this view and `right`.
    pub fn intersection(self, right: MeshView<'_>) -> Result<Mesh, MeshError> {
        self.materialize_closed_named_boolean(right, ExactBooleanOperation::Intersection)
    }

    /// Materialize the exact closed difference of this view minus `right`.
    pub fn difference(self, right: MeshView<'_>) -> Result<Mesh, MeshError> {
        self.materialize_closed_named_boolean(right, ExactBooleanOperation::Difference)
    }

    /// Materialize the exact closed symmetric difference of this view and `right`.
    pub fn xor(self, right: MeshView<'_>) -> Result<Mesh, MeshError> {
        let left_only = self.difference(right)?;
        let right_only = right.difference(self)?;
        left_only.view().union(right_only.view())
    }

    fn materialize_closed_named_boolean(
        self,
        right: MeshView<'_>,
        operation: ExactBooleanOperation,
    ) -> Result<Mesh, MeshError> {
        let pair = self.prepare_broad_phase_pair(right)?;
        materialize_boolean_operation(
            pair.left_view.mesh,
            pair.right_view.mesh,
            operation,
            MeshValidationMode::CLOSED,
            None,
            Some(&pair),
        )
        .map(|result| result.mesh)
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

    /// Retained incident face indices in face order.
    pub fn incident_face_indices(self) -> Result<&'a [usize], MeshError> {
        let facts = &self.mesh.facts().vertices[self.index];
        validate_retained_vertex_incident_rows(
            self.index,
            RetainedRowKind::Face,
            facts.incident_face_indices.as_slice(),
            current_row_count(self.mesh, RetainedRowKind::Face),
        )?;
        Ok(facts.incident_face_indices.as_slice())
    }

    /// Retained incident edge indices in canonical edge-fact order.
    pub fn incident_edge_indices(self) -> Result<&'a [usize], MeshError> {
        let facts = &self.mesh.facts().vertices[self.index];
        validate_retained_vertex_incident_rows(
            self.index,
            RetainedRowKind::Edge,
            facts.incident_edge_indices.as_slice(),
            current_row_count(self.mesh, RetainedRowKind::Edge),
        )?;
        Ok(facts.incident_edge_indices.as_slice())
    }

    /// Iterate borrowed incident faces from retained adjacency facts.
    pub fn incident_faces(
        self,
    ) -> Result<impl ExactSizeIterator<Item = FaceRef<'a>> + 'a, MeshError> {
        let indices = self.incident_face_indices()?;
        Ok(indices
            .iter()
            .copied()
            .map(move |index| self.mesh.view().face_ref(index)))
    }

    /// Iterate borrowed incident edges from retained adjacency facts.
    pub fn incident_edges(
        self,
    ) -> Result<impl ExactSizeIterator<Item = EdgeRef<'a>> + 'a, MeshError> {
        let indices = self.incident_edge_indices()?;
        Ok(indices
            .iter()
            .copied()
            .map(move |index| self.mesh.view().edge_ref(index)))
    }

    /// Whether retained facts classify the vertex link as isolated.
    pub fn has_isolated_link(self) -> bool {
        self.mesh.facts().vertices[self.index].link == super::facts::VertexLinkKind::Isolated
    }

    /// Whether retained facts classify the vertex link as a closed-manifold circle.
    pub fn has_circle_link(self) -> bool {
        self.mesh.facts().vertices[self.index].link == super::facts::VertexLinkKind::Circle
    }

    /// Whether retained facts classify the vertex link as a boundary-manifold disk.
    pub fn has_disk_link(self) -> bool {
        self.mesh.facts().vertices[self.index].link == super::facts::VertexLinkKind::Disk
    }

    /// Whether retained facts classify the vertex link as non-manifold.
    pub fn has_non_manifold_link(self) -> bool {
        self.mesh.facts().vertices[self.index].link == super::facts::VertexLinkKind::NonManifold
    }
}

impl<'a> FaceRef<'a> {
    /// Face index in the source mesh.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Borrow this face's triangle geometry.
    pub const fn triangle(self) -> TriangleRef<'a> {
        self.mesh.view().triangle_ref(self.index)
    }

    /// Triangle vertex indices for this face.
    pub fn vertex_indices(self) -> [usize; 3] {
        self.mesh.facts().faces[self.index].triangle.vertices
    }

    /// Borrow retained face bounds as exact min/max corners.
    pub fn bounds(self) -> Result<(&'a Point3, &'a Point3), MeshError> {
        require_current_row(self.mesh, RetainedRowKind::Face, self.index)?;
        self.mesh
            .bounds()
            .faces
            .get(self.index)
            .map(|bounds| (&bounds.min, &bounds.max))
            .ok_or_else(|| missing_retained_face_bounds(self.index))
    }

    /// Borrow the face vertices.
    pub fn vertex_refs(self) -> Result<[VertexRef<'a>; 3], MeshError> {
        let triangle = self.vertex_indices();
        retained_vertex_refs(
            self.mesh,
            RetainedVertexReference::Face {
                face: self.index,
                triangle,
            },
            triangle,
        )
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
        self.mesh.facts().faces[self.index]
            .triangle
            .degeneracy_predicates
            .as_slice()
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
        let facts = &self.mesh.facts().faces[self.index];
        (&facts.plane.normal, &facts.plane.offset)
    }

    pub(crate) fn plane(self) -> &'a super::facts::FacePlaneFacts {
        &self.mesh.facts().faces[self.index].plane
    }

    /// Exact face vertices.
    pub fn vertices(self) -> Result<[&'a Point3; 3], MeshError> {
        let [a, b, c] = self.vertex_refs()?;
        let vertices = self.mesh.vertices();
        Ok([&vertices[a.index], &vertices[b.index], &vertices[c.index]])
    }
}

impl<'a> TriangleRef<'a> {
    /// Retained face index that owns this triangle row.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Triangle vertex indices.
    pub fn vertex_indices(self) -> [usize; 3] {
        self.mesh.facts().faces[self.index].triangle.vertices
    }

    /// Borrow the triangle vertex references.
    pub fn vertex_refs(self) -> Result<[VertexRef<'a>; 3], MeshError> {
        self.mesh.view().face_ref(self.index).vertex_refs()
    }

    /// Exact triangle vertices.
    pub fn vertices(self) -> Result<[&'a Point3; 3], MeshError> {
        self.mesh.view().face_ref(self.index).vertices()
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
    pub fn vertex_refs(self) -> Result<[VertexRef<'a>; 2], MeshError> {
        let edge_vertices = self.vertex_indices();
        retained_vertex_refs(
            self.mesh,
            RetainedVertexReference::Edge {
                edge: self.index,
                edge_vertices,
            },
            edge_vertices,
        )
    }

    /// Borrow retained edge bounds as exact min/max corners.
    pub fn bounds(self) -> Result<(&'a Point3, &'a Point3), MeshError> {
        require_current_row(self.mesh, RetainedRowKind::Edge, self.index)?;
        self.mesh
            .bounds()
            .edges
            .get(self.index)
            .map(|bounds| (&bounds.min, &bounds.max))
            .ok_or_else(|| missing_retained_edge_bounds(self.mesh, self.index, "endpoint bounds"))
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
        facts.incident_faces == 2 && facts.directed_uses[0] == 1 && facts.directed_uses[1] == 1
    }

    /// Exact edge endpoints.
    pub fn vertices(self) -> Result<[&'a Point3; 2], MeshError> {
        let [a, b] = self.vertex_refs()?;
        let vertices = self.mesh.vertices();
        Ok([&vertices[a.index], &vertices[b.index]])
    }
}

fn missing_retained_face_bounds(face: usize) -> MeshError {
    MeshError::one(
        MeshBlocker::new(
            MeshBlockerKind::MissingRequiredEvidence,
            format!("mesh face {face} has no retained exact bounds"),
        )
        .with_face(face),
    )
}

fn missing_retained_edge_bounds(mesh: &Mesh, edge: usize, label: &str) -> MeshError {
    let mut blocker = MeshBlocker::new(
        MeshBlockerKind::MissingRequiredEvidence,
        format!("mesh edge {edge} has no retained {label}"),
    );
    if let Some(facts) = mesh.facts().edges.get(edge) {
        blocker = blocker.with_edge(facts.vertices);
    }
    MeshError::one(blocker)
}

#[derive(Clone, Copy)]
enum RetainedRowKind {
    Vertex,
    Face,
    Edge,
}

fn index_out_of_bounds(kind: RetainedRowKind, index: usize, count: usize) -> MeshError {
    let blocker = match kind {
        RetainedRowKind::Vertex => MeshBlocker::new(
            MeshBlockerKind::IndexOutOfBounds,
            format!("mesh vertex index {index} is out of bounds for {count} source vertices"),
        )
        .with_vertex(index),
        RetainedRowKind::Face => MeshBlocker::new(
            MeshBlockerKind::IndexOutOfBounds,
            format!("mesh face index {index} is out of bounds for {count} retained faces"),
        )
        .with_face(index),
        RetainedRowKind::Edge => MeshBlocker::new(
            MeshBlockerKind::IndexOutOfBounds,
            format!("mesh edge index {index} is out of bounds for {count} retained edges"),
        ),
    };
    MeshError::one(blocker)
}

fn stale_retained_row_error(
    mesh: &Mesh,
    kind: RetainedRowKind,
    index: usize,
    retained_count: usize,
) -> MeshError {
    let mut blocker = match kind {
        RetainedRowKind::Vertex => MeshBlocker::new(
            MeshBlockerKind::StaleFactReplay,
            format!(
                "retained mesh vertex {index} has a retained vertex row beyond summary vertex count {retained_count}"
            ),
        )
        .with_vertex(index),
        RetainedRowKind::Face => MeshBlocker::new(
            MeshBlockerKind::StaleFactReplay,
            format!(
                "retained mesh face {index} has a retained face row beyond summary face count {retained_count}"
            ),
        )
        .with_face(index),
        RetainedRowKind::Edge => MeshBlocker::new(
            MeshBlockerKind::StaleFactReplay,
            format!(
                "retained mesh edge {index} has a retained edge row beyond summary edge count {retained_count}"
            ),
        ),
    };
    if let RetainedRowKind::Edge = kind
        && let Some(facts) = mesh.facts().edges.get(index)
    {
        blocker = blocker.with_edge(facts.vertices);
    }
    MeshError::one(blocker)
}

fn stale_retained_vertex_incident_row_error(
    vertex: usize,
    kind: RetainedRowKind,
    index: usize,
    current_count: usize,
) -> MeshError {
    let blocker = match kind {
        RetainedRowKind::Face => MeshBlocker::new(
            MeshBlockerKind::StaleFactReplay,
            format!(
                "retained mesh vertex {vertex} references incident face {index}, but only {current_count} current retained faces exist"
            ),
        )
        .with_vertex(vertex)
        .with_face(index),
        RetainedRowKind::Edge => MeshBlocker::new(
            MeshBlockerKind::StaleFactReplay,
            format!(
                "retained mesh vertex {vertex} references incident edge row {index}, but only {current_count} current retained edges exist"
            ),
        )
        .with_vertex(vertex),
        RetainedRowKind::Vertex => {
            unreachable!("vertex incident rows only reference retained faces or retained edges")
        }
    };
    MeshError::one(blocker)
}

fn validate_retained_vertex_incident_rows(
    vertex: usize,
    kind: RetainedRowKind,
    indices: &[usize],
    current_count: usize,
) -> Result<(), MeshError> {
    for &index in indices {
        if index >= current_count {
            return Err(stale_retained_vertex_incident_row_error(
                vertex,
                kind,
                index,
                current_count,
            ));
        }
    }
    Ok(())
}

fn current_row_count(mesh: &Mesh, kind: RetainedRowKind) -> usize {
    match kind {
        RetainedRowKind::Vertex => mesh
            .vertices()
            .len()
            .min(mesh.facts().vertices.len())
            .min(mesh.facts().mesh.vertex_count),
        RetainedRowKind::Face => mesh.facts().faces.len().min(mesh.facts().mesh.face_count),
        RetainedRowKind::Edge => mesh.facts().edges.len().min(mesh.facts().mesh.edge_count),
    }
}

fn require_current_vertex_count(mesh: &Mesh) -> Result<(), MeshError> {
    let source_rows = mesh.vertices().len();
    let retained_rows = mesh.facts().vertices.len();
    let summary_rows = mesh.facts().mesh.vertex_count;
    if retained_rows != source_rows {
        return Err(MeshError::one(MeshBlocker::new(
            MeshBlockerKind::StaleFactReplay,
            format!(
                "retained mesh has {retained_rows} vertex fact rows for {source_rows} source vertices"
            ),
        )));
    }
    if summary_rows != source_rows {
        return Err(MeshError::one(MeshBlocker::new(
            MeshBlockerKind::StaleFactReplay,
            format!(
                "retained mesh summary has {summary_rows} vertices for {source_rows} source vertices"
            ),
        )));
    }
    Ok(())
}

fn require_current_row(mesh: &Mesh, kind: RetainedRowKind, index: usize) -> Result<(), MeshError> {
    match kind {
        RetainedRowKind::Vertex => {
            if index >= mesh.vertices().len() {
                return Err(index_out_of_bounds(kind, index, mesh.vertices().len()));
            }
            if mesh.facts().vertices.get(index).is_none() {
                return Err(MeshError::one(
                    MeshBlocker::new(
                        MeshBlockerKind::StaleFactReplay,
                        format!("retained mesh vertex {index} has no retained vertex fact row"),
                    )
                    .with_vertex(index),
                ));
            }
            if index >= mesh.facts().mesh.vertex_count {
                return Err(stale_retained_row_error(
                    mesh,
                    kind,
                    index,
                    mesh.facts().mesh.vertex_count,
                ));
            }
        }
        RetainedRowKind::Face => {
            let retained_rows = mesh.facts().faces.len();
            let summary_rows = mesh.facts().mesh.face_count;
            if index >= retained_rows && index >= summary_rows {
                return Err(index_out_of_bounds(kind, index, summary_rows));
            }
            if index >= retained_rows {
                return Err(MeshError::one(
                    MeshBlocker::new(
                        MeshBlockerKind::StaleFactReplay,
                        format!("retained mesh face {index} has no retained face fact row"),
                    )
                    .with_face(index),
                ));
            }
            if index >= summary_rows {
                return Err(stale_retained_row_error(mesh, kind, index, summary_rows));
            }
        }
        RetainedRowKind::Edge => {
            let retained_rows = mesh.facts().edges.len();
            let summary_rows = mesh.facts().mesh.edge_count;
            if index >= retained_rows && index >= summary_rows {
                return Err(index_out_of_bounds(kind, index, summary_rows));
            }
            if index >= retained_rows {
                return Err(MeshError::one(MeshBlocker::new(
                    MeshBlockerKind::StaleFactReplay,
                    format!("retained mesh edge {index} has no retained edge fact row"),
                )));
            }
            if index >= summary_rows {
                return Err(stale_retained_row_error(mesh, kind, index, summary_rows));
            }
        }
    }
    Ok(())
}

#[derive(Clone, Copy)]
enum RetainedVertexReference {
    Face {
        face: usize,
        triangle: [usize; 3],
    },
    Edge {
        edge: usize,
        edge_vertices: [usize; 2],
    },
}

fn retained_vertex_refs<'a, const N: usize>(
    mesh: &'a Mesh,
    reference: RetainedVertexReference,
    vertices: [usize; N],
) -> Result<[VertexRef<'a>; N], MeshError> {
    let vertex_count = current_row_count(mesh, RetainedRowKind::Vertex);
    for vertex in vertices {
        if vertex >= vertex_count {
            return Err(retained_vertex_reference_error(reference, vertex));
        }
    }
    Ok(std::array::from_fn(|index| {
        mesh.view().vertex_ref(vertices[index])
    }))
}

fn retained_vertex_reference_error(reference: RetainedVertexReference, vertex: usize) -> MeshError {
    let blocker = match reference {
        RetainedVertexReference::Face { face, triangle } => MeshBlocker::new(
            MeshBlockerKind::StaleFactReplay,
            format!(
                "retained mesh face {face} with vertex row {triangle:?} references missing vertex {vertex}"
            ),
        )
        .with_face(face)
        .with_vertex(vertex),
        RetainedVertexReference::Edge {
            edge,
            edge_vertices,
        } => MeshBlocker::new(
            MeshBlockerKind::StaleFactReplay,
            format!("retained mesh edge {edge} references missing vertex {vertex}"),
        )
        .with_edge(edge_vertices)
        .with_vertex(vertex),
    };
    MeshError::one(blocker)
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperlimit::SourceProvenance;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn tetra(offset: [i64; 3]) -> Mesh {
        let [ox, oy, oz] = offset;
        Mesh::new(
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
    fn borrowed_view_rejects_missing_retained_vertex_row() {
        let mut mesh = tetra([0, 0, 0]);
        let stale_vertex = mesh.vertices().len() - 1;
        mesh.facts.vertices.pop();

        let view = mesh.view();

        assert_eq!(
            view.mesh_bounds().unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert_eq!(view.vertices().len(), stale_vertex);
        assert_eq!(view.vertex_refs().count(), stale_vertex);
        assert_eq!(
            view.vertex(stale_vertex).unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
    }

    #[test]
    fn borrowed_view_reports_missing_retained_mesh_bounds() {
        let empty = Mesh::new(
            Vec::new(),
            Vec::new(),
            SourceProvenance::exact("empty view test mesh"),
        )
        .unwrap();
        assert_eq!(empty.view().mesh_bounds().unwrap(), None);

        let mut mesh = tetra([0, 0, 0]);
        mesh.bounds.mesh = None;

        let error = mesh.view().mesh_bounds().unwrap_err();
        assert_eq!(
            error.blockers()[0].kind(),
            MeshBlockerKind::MissingRequiredEvidence
        );
    }

    #[test]
    fn borrowed_view_rejects_missing_retained_face_row() {
        let mut mesh = tetra([0, 0, 0]);
        let stale_face = mesh.facts().mesh.face_count - 1;
        mesh.facts.faces.pop();

        let view = mesh.view();

        assert_eq!(view.triangles().len(), stale_face);
        assert_eq!(view.faces().count(), stale_face);
        assert_eq!(
            view.triangle(stale_face).unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert_eq!(
            view.face(stale_face).unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
    }

    #[test]
    fn borrowed_view_rejects_missing_retained_edge_row() {
        let mut mesh = tetra([0, 0, 0]);
        let stale_edge = mesh.facts().mesh.edge_count - 1;
        mesh.facts.edges.pop();

        let view = mesh.view();

        assert_eq!(view.edges().count(), stale_edge);
        assert_eq!(
            view.edge(stale_edge).unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
    }

    #[test]
    fn borrowed_view_rejects_stale_retained_summary_counts() {
        let mut mesh = tetra([0, 0, 0]);
        let stale_vertex = mesh.facts.mesh.vertex_count - 1;
        let stale_face = mesh.facts.mesh.face_count - 1;
        let stale_edge = mesh.facts.mesh.edge_count - 1;
        let face_with_stale_vertex = mesh
            .facts()
            .faces
            .iter()
            .take(stale_face)
            .position(|face| face.triangle.vertices.contains(&stale_vertex))
            .unwrap();
        let edge_with_stale_vertex = mesh
            .facts()
            .edges
            .iter()
            .take(stale_edge)
            .position(|edge| edge.vertices.contains(&stale_vertex))
            .unwrap();
        mesh.facts.mesh.vertex_count -= 1;
        mesh.facts.mesh.face_count -= 1;
        mesh.facts.mesh.edge_count -= 1;
        mesh.facts.vertices[0].incident_face_indices = vec![stale_face];
        mesh.facts.vertices[0].incident_edge_indices = vec![stale_edge];

        let view = mesh.view();
        let vertex = view.vertex(0).unwrap();

        assert_eq!(
            view.mesh_bounds().unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert_eq!(view.vertices().len(), stale_vertex);
        assert_eq!(view.vertex_refs().count(), stale_vertex);
        assert_eq!(
            view.vertex(stale_vertex).unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert!(view.inverse().is_err());
        assert!(
            view.transform([
                [Real::from(1), Real::from(0), Real::from(0), Real::from(0)],
                [Real::from(0), Real::from(1), Real::from(0), Real::from(0)],
                [Real::from(0), Real::from(0), Real::from(1), Real::from(0)],
                [Real::from(0), Real::from(0), Real::from(0), Real::from(1)],
            ])
            .is_err()
        );
        let stale_incident_face_error = vertex.incident_face_indices().unwrap_err();
        let stale_incident_face = &stale_incident_face_error.blockers()[0];
        assert_eq!(stale_incident_face.kind(), MeshBlockerKind::StaleFactReplay);
        assert_eq!(stale_incident_face.vertex(), Some(0));
        assert_eq!(stale_incident_face.face(), Some(stale_face));
        let stale_incident_edge_error = vertex.incident_edge_indices().unwrap_err();
        let stale_incident_edge = &stale_incident_edge_error.blockers()[0];
        assert_eq!(stale_incident_edge.kind(), MeshBlockerKind::StaleFactReplay);
        assert_eq!(stale_incident_edge.vertex(), Some(0));

        assert_eq!(view.triangles().len(), stale_face);
        assert_eq!(view.faces().count(), stale_face);
        assert_eq!(
            view.triangle(stale_face).unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert_eq!(
            view.face_bounds(stale_face).unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert_eq!(
            view.face(stale_face).unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert_eq!(
            view.face_bounds(stale_face).unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert_eq!(
            FaceRef {
                mesh: &mesh,
                index: stale_face,
            }
            .bounds()
            .unwrap_err()
            .blockers()[0]
                .kind(),
            MeshBlockerKind::StaleFactReplay
        );
        let face = view.face(face_with_stale_vertex).unwrap();
        assert_eq!(
            face.vertex_refs().unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert_eq!(
            face.vertices().unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );

        assert_eq!(view.edges().count(), stale_edge);
        assert_eq!(
            view.edge_bounds(stale_edge).unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert_eq!(
            view.edge(stale_edge).unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert_eq!(
            view.edge_bounds(stale_edge).unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert_eq!(
            EdgeRef {
                mesh: &mesh,
                index: stale_edge,
            }
            .bounds()
            .unwrap_err()
            .blockers()[0]
                .kind(),
            MeshBlockerKind::StaleFactReplay
        );
        let edge = view.edge(edge_with_stale_vertex).unwrap();
        assert_eq!(
            edge.vertex_refs().unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
        assert_eq!(
            edge.vertices().unwrap_err().blockers()[0].kind(),
            MeshBlockerKind::StaleFactReplay
        );
    }

    #[test]
    fn borrowed_view_blockers_retain_object_provenance() {
        let mesh = tetra([0, 0, 0]);
        let view = mesh.view();

        let vertex_error = view.vertex(mesh.vertices().len()).unwrap_err();
        let vertex_blocker = &vertex_error.blockers()[0];
        assert_eq!(vertex_blocker.kind(), MeshBlockerKind::IndexOutOfBounds);
        assert_eq!(vertex_blocker.vertex(), Some(mesh.vertices().len()));

        let face_error = view.face(mesh.facts().faces.len()).unwrap_err();
        let face_blocker = &face_error.blockers()[0];
        assert_eq!(face_blocker.kind(), MeshBlockerKind::IndexOutOfBounds);
        assert_eq!(face_blocker.face(), Some(mesh.facts().faces.len()));

        let edge_error = view.edge(mesh.facts().edges.len()).unwrap_err();
        let edge_blocker = &edge_error.blockers()[0];
        assert_eq!(edge_blocker.kind(), MeshBlockerKind::IndexOutOfBounds);

        let missing_bounds_error = missing_retained_edge_bounds(&mesh, 0, "exact bounds");
        let missing_bounds_blocker = &missing_bounds_error.blockers()[0];
        assert_eq!(
            missing_bounds_blocker.kind(),
            MeshBlockerKind::MissingRequiredEvidence
        );
        assert_eq!(
            missing_bounds_blocker.edge(),
            Some(mesh.facts().edges[0].vertices)
        );
    }

    #[test]
    fn borrowed_view_materializes_transform_and_inverse() {
        let mesh = tetra([0, 0, 0]);
        let translated = mesh
            .view()
            .transform([
                [Real::one(), Real::zero(), Real::zero(), Real::from(2)],
                [Real::zero(), Real::one(), Real::zero(), Real::from(3)],
                [Real::zero(), Real::zero(), Real::one(), Real::from(4)],
                [Real::zero(), Real::zero(), Real::zero(), Real::one()],
            ])
            .unwrap();

        assert_eq!(translated.vertices()[0], p(2, 3, 4));
        assert_eq!(
            translated.provenance().construction_version,
            mesh.provenance().construction_version + 1
        );
        translated.view().validate_retained_state().unwrap();

        let inverted = mesh.view().inverse().unwrap();

        assert_eq!(inverted.triangles()[0].0, [0, 1, 2]);
        assert_eq!(
            inverted.provenance().construction_version,
            mesh.provenance().construction_version + 1
        );
        inverted.view().validate_retained_state().unwrap();
    }
}
