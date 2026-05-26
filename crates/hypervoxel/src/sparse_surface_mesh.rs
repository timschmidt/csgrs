//! Sparse-grid exact surface triangle mesh handoff.
//!
//! This module is the canonical sparse-storage counterpart to the paged and
//! SVO surface handoffs. It composes the exact sparse exposed-face extraction,
//! exact lattice-vertex triangle mesh construction, and shared mesh vocabulary
//! audit into one replayable report. That keeps downstream crates from
//! rebuilding the same proof boundary from partial facts.
//!
//! The gate follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997: exactness is a property of the whole
//! geometric system, not a scalar type alone. The surface is accepted only
//! when the retained sparse-grid shell, indexed triangle records, and mesh
//! vocabulary all replay. The indexed mesh vocabulary follows Botsch et al.,
//! *Polygon Mesh Processing*, AK Peters, 2010, while retaining exact
//! grid-lattice vertices instead of primitive-float display coordinates.

use crate::{
    ExactFaceExtractionReport, ExactSurfaceTriangleMeshVocabularyReport,
    ExactVoxelSurfaceTriangleMesh, HypervoxelResult, SparseVoxelGrid,
    audit_exact_surface_triangle_mesh_vocabulary, exact_voxel_surface_triangle_mesh_from_faces,
    extract_exposed_faces_with_report,
};

/// Report for exact sparse-grid surface triangle mesh handoff.
#[derive(Clone, Debug, PartialEq)]
pub struct SparseExactSurfaceTriangleMeshReport {
    /// Exact exposed-face shell replayed directly from sparse storage.
    pub shell: ExactFaceExtractionReport,
    /// Exact indexed triangle mesh built from the accepted shell.
    pub mesh: ExactVoxelSurfaceTriangleMesh,
    /// Shared mesh-vocabulary audit over the indexed triangle records.
    pub vocabulary: ExactSurfaceTriangleMeshVocabularyReport,
    /// Whether the sparse shell, mesh handoff, and vocabulary all replay as
    /// one exact sparse-grid surface artifact.
    pub exact_sparse_triangle_mesh_ready: bool,
}

/// Builds a report-bearing exact surface triangle mesh from sparse storage.
///
/// The sparse grid remains the source object. This function first extracts
/// exact exposed faces with [`crate::extract_exposed_faces_with_report`], then
/// lowers only the accepted shell to the exact indexed triangle vocabulary via
/// [`crate::exact_voxel_surface_triangle_mesh_from_faces`], and finally
/// replays the emitted mesh using
/// [`crate::audit_exact_surface_triangle_mesh_vocabulary`].
///
/// Empty grids, unknown cells, lossy adapter cells, open shells, duplicate
/// faces, and malformed indexed mesh records do not get repaired here; their
/// blockers are retained in the nested reports. This is the Yap-style
/// representation boundary: proposal or storage facts are useful, but only
/// replayed object structure can become exact topology.
pub fn sparse_exact_surface_triangle_mesh_with_report(
    grid: &SparseVoxelGrid,
) -> HypervoxelResult<SparseExactSurfaceTriangleMeshReport> {
    let shell = extract_exposed_faces_with_report(grid)?;
    let mesh = exact_voxel_surface_triangle_mesh_from_faces(&shell.faces);
    let vocabulary = audit_exact_surface_triangle_mesh_vocabulary(&mesh);
    let exact_sparse_triangle_mesh_ready = shell.exact_shell_ready
        && mesh.report.exact_triangle_surface_mesh_ready
        && vocabulary.exact_shared_mesh_vocabulary_ready;

    Ok(SparseExactSurfaceTriangleMeshReport {
        shell,
        mesh,
        vocabulary,
        exact_sparse_triangle_mesh_ready,
    })
}
