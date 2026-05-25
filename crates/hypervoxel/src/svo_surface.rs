//! Exact surface replay from Hyper-owned SVO-DAG storage.
//!
//! The SVO-DAG backend is a compressed storage representation. Exact shell
//! consumers need more than a root node id: they need a replayable path from
//! compressed storage to canonical cells, then to exact exposed faces, then to
//! shared surface topology. This module ties those reports together without
//! letting the SVO layout decide topology directly.
//!
//! This follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7(1-2), 1997: acceleration and compression can be used only when
//! exact object facts are retained and replayed. The SVO sharing idea follows
//! Kämpe, Sintorn, and Assarsson, "High Resolution Sparse Voxel DAGs," *ACM
//! Transactions on Graphics* 32(4), 2013, while the exposed-face and
//! vertex/edge/face reports keep the combinatorial mesh vocabulary explicit.

use crate::{
    ExactFaceExtractionReport, ExactVoxelFace, ExactVoxelSurfaceTopologyReport, HypervoxelResult,
    SvoSparseReplayReport, SvoVoxelGrid, audit_exact_voxel_surface_topology,
    extract_exposed_faces_with_report,
};

/// Exact SVO-DAG surface replay report.
#[derive(Clone, Debug, PartialEq)]
pub struct SvoSurfaceReplayReport {
    /// Sparse replay evidence from compressed SVO storage.
    pub sparse_replay: SvoSparseReplayReport,
    /// Exact exposed-face extraction report from the replayed sparse grid.
    pub shell: ExactFaceExtractionReport,
    /// Exact vertex/edge/face topology audit for the extracted shell.
    pub topology: ExactVoxelSurfaceTopologyReport,
    /// Number of exact faces emitted for downstream consumers.
    pub exact_faces: usize,
    /// Whether exact SVO storage replay, exact shell extraction, and exact
    /// closed-surface topology all succeeded.
    pub exact_svo_surface_replay_ready: bool,
}

/// Replays compressed SVO-DAG storage into exact exposed surface faces.
///
/// The returned faces are the canonical exact exposed faces from the replayed
/// sparse grid. SVO nodes only schedule the replay; the surface decision still
/// comes from exact cell/neighbor facts, and the topology audit must accept the
/// resulting face incidence before this path is exact-ready.
pub fn extract_svo_exposed_faces_with_report(
    grid: &SvoVoxelGrid,
) -> HypervoxelResult<(Vec<ExactVoxelFace>, SvoSurfaceReplayReport)> {
    let (sparse, sparse_replay) = grid.replay_sparse_grid_with_report()?;
    let shell = extract_exposed_faces_with_report(&sparse)?;
    let topology = audit_exact_voxel_surface_topology(&shell.faces);
    let exact_faces = shell.exact_faces;
    let exact_svo_surface_replay_ready = sparse_replay.exact_sparse_replay_ready
        && shell.exact_shell_ready
        && topology.exact_surface_topology_ready
        && exact_faces > 0;
    let faces = shell.faces.clone();
    Ok((
        faces,
        SvoSurfaceReplayReport {
            sparse_replay,
            shell,
            topology,
            exact_faces,
            exact_svo_surface_replay_ready,
        },
    ))
}
