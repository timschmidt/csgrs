//! Shared exact surface-mesh vocabulary audits.
//!
//! [`crate::ExactVoxelSurfaceTriangleMesh`] is already an exact handoff: it
//! stores lattice vertices and indexed triangles instead of primitive-float
//! display coordinates. This module adds the next downstream-facing vocabulary
//! check. It audits the emitted indexed mesh itself so a consumer can validate
//! source-face split records, index bounds, triangle degeneracy, and indexed
//! edge incidence before treating the mesh as a shared Hyper surface artifact.
//!
//! A representation boundary is exact only when its object-level facts are
//! replayable and its blockers are named. The indexed vertex/edge/face
//! vocabulary retains exact grid-lattice vertices and source voxel-face
//! identities.

use std::collections::{BTreeMap, BTreeSet};

use crate::{ExactSurfaceFaceKey, ExactVoxelSurfaceTriangleMesh};

/// Undirected edge between two indexed mesh vertices.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ExactSurfaceTriangleMeshEdge {
    /// Lower vertex index in deterministic order.
    pub a: u32,
    /// Upper vertex index in deterministic order.
    pub b: u32,
}

/// Shared exact surface-mesh vocabulary audit.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExactSurfaceTriangleMeshVocabularyReport {
    /// Number of vertices supplied by the indexed mesh.
    pub input_vertices: usize,
    /// Number of triangles supplied by the indexed mesh.
    pub input_triangles: usize,
    /// Number of source voxel faces represented by at least one triangle.
    pub source_faces: usize,
    /// Number of source faces claimed by the topology report.
    pub topology_source_faces: usize,
    /// Number of vertex indices referenced by triangles after de-duplication.
    pub referenced_vertices: usize,
    /// Triangle vertex indices that are outside the vertex array.
    pub out_of_bounds_indices: Vec<(usize, u32)>,
    /// Triangles with repeated vertex indices.
    pub degenerate_triangles: Vec<usize>,
    /// Triangles whose split ordinal is not `0` or `1`.
    pub invalid_split_triangles: Vec<usize>,
    /// Duplicate `(source face, split)` records.
    pub duplicate_source_splits: Vec<(ExactSurfaceFaceKey, u8)>,
    /// Source faces whose emitted triangle count is not exactly two.
    pub source_faces_with_wrong_triangle_count: Vec<(ExactSurfaceFaceKey, usize)>,
    /// Number of indexed triangle-edge records audited.
    pub triangle_edge_records: usize,
    /// Unique undirected indexed mesh edges.
    pub unique_index_edges: usize,
    /// Indexed edges incident to exactly one triangle.
    pub boundary_index_edges: Vec<ExactSurfaceTriangleMeshEdge>,
    /// Indexed edges incident to exactly two triangles.
    pub manifold_index_edges: usize,
    /// Indexed edges incident to more than two triangles, with incidence counts.
    pub nonmanifold_index_edges: Vec<(ExactSurfaceTriangleMeshEdge, usize)>,
    /// Whether topology, triangle handoff, source-face records, and indexed
    /// edge incidence agree as shared exact surface-mesh vocabulary.
    pub exact_shared_mesh_vocabulary_ready: bool,
}

/// Audits an exact voxel-surface triangle mesh as shared mesh vocabulary.
///
/// This function deliberately replays the indexed mesh instead of trusting the
/// handoff report alone. Each triangle must reference in-bounds lattice
/// vertices, be non-degenerate in index space, retain a valid source-face split
/// ordinal, and contribute to a two-triangle source-face record. The indexed
/// triangle edges must also form a closed two-manifold. Those checks do not
/// repair topology; they expose why a downstream mesh vocabulary handoff is
/// blocked.
pub fn audit_exact_surface_triangle_mesh_vocabulary(
    mesh: &ExactVoxelSurfaceTriangleMesh,
) -> ExactSurfaceTriangleMeshVocabularyReport {
    let mut referenced_vertices = BTreeSet::new();
    let mut out_of_bounds_indices = Vec::new();
    let mut degenerate_triangles = Vec::new();
    let mut invalid_split_triangles = Vec::new();
    let mut seen_source_splits = BTreeSet::new();
    let mut duplicate_source_splits = Vec::new();
    let mut source_counts = BTreeMap::<ExactSurfaceFaceKey, usize>::new();
    let mut edge_incidence = BTreeMap::<ExactSurfaceTriangleMeshEdge, usize>::new();
    let mut triangle_edge_records = 0_usize;

    for (triangle_index, triangle) in mesh.triangles.iter().enumerate() {
        let mut in_bounds = true;
        for index in triangle.vertices {
            if usize::try_from(index)
                .ok()
                .is_some_and(|index| index < mesh.vertices.len())
            {
                referenced_vertices.insert(index);
            } else {
                in_bounds = false;
                out_of_bounds_indices.push((triangle_index, index));
            }
        }

        if triangle.vertices[0] == triangle.vertices[1]
            || triangle.vertices[1] == triangle.vertices[2]
            || triangle.vertices[2] == triangle.vertices[0]
        {
            degenerate_triangles.push(triangle_index);
        }

        if triangle.split > 1 {
            invalid_split_triangles.push(triangle_index);
        }
        if !seen_source_splits.insert((triangle.source_face, triangle.split)) {
            duplicate_source_splits.push((triangle.source_face, triangle.split));
        }
        *source_counts.entry(triangle.source_face).or_insert(0) += 1;

        if in_bounds {
            for edge in triangle_edges(triangle.vertices) {
                if !edge.is_degenerate() {
                    triangle_edge_records += 1;
                    *edge_incidence.entry(edge).or_insert(0) += 1;
                }
            }
        }
    }

    let source_faces_with_wrong_triangle_count = source_counts
        .iter()
        .filter_map(|(source_face, count)| (*count != 2).then_some((*source_face, *count)))
        .collect::<Vec<_>>();
    let mut boundary_index_edges = Vec::new();
    let mut manifold_index_edges = 0_usize;
    let mut nonmanifold_index_edges = Vec::new();
    for (edge, count) in &edge_incidence {
        match *count {
            0 => unreachable!("edge incidence map never stores zero counts"),
            1 => boundary_index_edges.push(*edge),
            2 => manifold_index_edges += 1,
            count => nonmanifold_index_edges.push((*edge, count)),
        }
    }

    let topology_source_faces = mesh.report.topology.unique_faces;
    let exact_shared_mesh_vocabulary_ready = mesh.report.exact_triangle_surface_mesh_ready
        && mesh.report.topology.exact_surface_topology_ready
        && mesh.vertices.len() == mesh.report.exact_vertices
        && mesh.triangles.len() == mesh.report.exact_triangles
        && mesh.triangles.len() == mesh.report.face_triangle_records
        && mesh.report.exact_face_identity_preserved
        && !mesh.vertices.is_empty()
        && !mesh.triangles.is_empty()
        && source_counts.len() == topology_source_faces
        && mesh.triangles.len() == topology_source_faces.saturating_mul(2)
        && referenced_vertices.len() == mesh.vertices.len()
        && out_of_bounds_indices.is_empty()
        && degenerate_triangles.is_empty()
        && invalid_split_triangles.is_empty()
        && duplicate_source_splits.is_empty()
        && source_faces_with_wrong_triangle_count.is_empty()
        && boundary_index_edges.is_empty()
        && nonmanifold_index_edges.is_empty();

    ExactSurfaceTriangleMeshVocabularyReport {
        input_vertices: mesh.vertices.len(),
        input_triangles: mesh.triangles.len(),
        source_faces: source_counts.len(),
        topology_source_faces,
        referenced_vertices: referenced_vertices.len(),
        out_of_bounds_indices,
        degenerate_triangles,
        invalid_split_triangles,
        duplicate_source_splits,
        source_faces_with_wrong_triangle_count,
        triangle_edge_records,
        unique_index_edges: edge_incidence.len(),
        boundary_index_edges,
        manifold_index_edges,
        nonmanifold_index_edges,
        exact_shared_mesh_vocabulary_ready,
    }
}

impl ExactSurfaceTriangleMeshEdge {
    fn new(a: u32, b: u32) -> Self {
        if a <= b {
            Self { a, b }
        } else {
            Self { a: b, b: a }
        }
    }

    fn is_degenerate(&self) -> bool {
        self.a == self.b
    }
}

fn triangle_edges(vertices: [u32; 3]) -> [ExactSurfaceTriangleMeshEdge; 3] {
    [
        ExactSurfaceTriangleMeshEdge::new(vertices[0], vertices[1]),
        ExactSurfaceTriangleMeshEdge::new(vertices[1], vertices[2]),
        ExactSurfaceTriangleMeshEdge::new(vertices[2], vertices[0]),
    ]
}
