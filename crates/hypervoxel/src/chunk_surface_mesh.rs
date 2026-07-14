//! Page-backed exact voxel-surface triangle mesh handoff.
//!
//! [`crate::ChunkPagedSparseGrid`] can accelerate exact shell extraction by
//! grouping cells into integer pages, but page layout is not itself topology.
//! This module threads that page-backed shell evidence into the exact
//! lattice-vertex triangle mesh handoff from [`crate::surface_mesh`].
//!
//! An accelerated representation may be consumed as exact only after replaying
//! retained object facts and reporting blockers. The indexed triangle
//! vocabulary uses exact grid-lattice vertices rather than primitive-float
//! coordinates.

use crate::{
    ChunkPagedExactFaceExtractionReport, ChunkPagedSparseGrid,
    ExactSurfaceTriangleMeshVocabularyReport, ExactVoxelSurfaceTriangleMesh, HypervoxelResult,
    audit_exact_surface_triangle_mesh_vocabulary, exact_voxel_surface_triangle_mesh_from_faces,
    extract_chunk_paged_exposed_faces_with_report,
};

/// Page-backed exact voxel-surface triangle mesh report.
#[derive(Clone, Debug, PartialEq)]
pub struct ChunkPagedExactSurfaceTriangleMeshReport {
    /// Exact page-backed shell consumed by the handoff.
    pub shell: ChunkPagedExactFaceExtractionReport,
    /// Exact indexed triangle mesh produced from the shell.
    pub mesh: ExactVoxelSurfaceTriangleMesh,
    /// Shared indexed mesh vocabulary audit over the emitted triangle mesh.
    pub vocabulary: ExactSurfaceTriangleMeshVocabularyReport,
    /// Whether page-backed shell extraction and exact triangle mesh handoff
    /// both produced replay-ready shared vocabulary evidence.
    pub exact_paged_triangle_mesh_ready: bool,
}

/// Builds an exact indexed triangle mesh from chunk-paged sparse storage.
///
/// Pages schedule the shell extraction, but the emitted mesh is accepted only
/// when exact neighbor lookup certifies a non-empty shell and the topology
/// audit accepts that shell as a closed manifold. Empty, unknown, lossy, open,
/// duplicate, mixed-depth, or nonmanifold cases return a report with blockers
/// and no exact triangle mesh.
pub fn chunk_paged_exact_surface_triangle_mesh_with_report(
    grid: &ChunkPagedSparseGrid,
) -> HypervoxelResult<ChunkPagedExactSurfaceTriangleMeshReport> {
    let shell = extract_chunk_paged_exposed_faces_with_report(grid)?;
    let mesh = exact_voxel_surface_triangle_mesh_from_faces(&shell.faces);
    let vocabulary = audit_exact_surface_triangle_mesh_vocabulary(&mesh);
    let exact_paged_triangle_mesh_ready =
        shell.exact_paged_shell_ready && vocabulary.exact_shared_mesh_vocabulary_ready;
    Ok(ChunkPagedExactSurfaceTriangleMeshReport {
        shell,
        mesh,
        vocabulary,
        exact_paged_triangle_mesh_ready,
    })
}
