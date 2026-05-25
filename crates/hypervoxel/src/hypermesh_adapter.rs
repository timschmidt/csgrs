//! Exact `hypermesh` solid handoff adapter.
//!
//! This module consumes `hypermesh`'s retained exact mesh vocabulary and lowers
//! it into [`crate::ExactTriangleSolidMesh`] without treating an arbitrary
//! triangle soup as solid evidence. The adapter accepts only fresh
//! `hypermesh::exact::ExactSolidHandoffReport` state whose source provenance is
//! exact, then copies exact retained vertex coordinates into `hypervoxel`'s
//! triangle-solid carrier. This follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997: downstream topology is
//! admitted only when the owning object replays its exact structure and
//! proof-producing predicates at the handoff boundary.

use crate::{
    ExactTriangle3, ExactTriangleSolidMesh, ExactTriangleSurfaceMesh, GridSource, HypervoxelResult,
};

/// Explicit blocker for [`adapt_hypermesh_exact_solid`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HypermeshTriangleSolidAdapterBlocker {
    /// `hypermesh` did not accept the mesh as exact closed-solid evidence.
    SolidHandoffNotReady,
    /// The supplied or replayed solid report is stale for the current mesh.
    StaleSolidHandoff,
    /// The retained mesh came from a lossy source adapter, not exact caller
    /// coordinates.
    SourceNotExact,
    /// The exact handoff did not retain proof-producing predicate evidence.
    PredicateReplayNotReady,
    /// A triangle referenced a vertex outside the retained vertex buffer.
    MissingVertex,
    /// The emitted triangle count did not match the retained solid handoff.
    TriangleCountMismatch,
}

/// Report for a `hypermesh` to `hypervoxel` exact solid adaptation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct HypermeshTriangleSolidAdapterReport {
    /// Number of retained source vertices observed in `hypermesh`.
    pub source_vertex_count: usize,
    /// Number of retained source triangles observed in `hypermesh`.
    pub source_triangle_count: usize,
    /// Number of triangles emitted into the `hypervoxel` carrier.
    pub emitted_triangle_count: usize,
    /// Explicit blockers encountered while replaying the source handoff.
    pub blockers: Vec<HypermeshTriangleSolidAdapterBlocker>,
    /// Whether [`HypermeshTriangleSolidAdapter::solid`] may be consumed as an
    /// exact closed triangle-solid handoff.
    pub exact_triangle_solid_ready: bool,
}

/// Result of adapting a retained `hypermesh` exact solid.
#[derive(Clone, Debug, PartialEq)]
pub struct HypermeshTriangleSolidAdapter {
    /// Adapted triangle solid when all handoff gates passed.
    pub solid: Option<ExactTriangleSolidMesh>,
    /// Report describing the adapter decision.
    pub report: HypermeshTriangleSolidAdapterReport,
}

/// Adapt a retained `hypermesh` exact solid into a `hypervoxel` triangle solid.
///
/// The optional `solid_handoff` parameter lets callers pass a report they have
/// already retained. The adapter validates it against the current mesh before
/// use; a stale report is rejected instead of being trusted as cached topology.
/// That freshness replay is the same object-level discipline required by Yap
/// (1997), and mirrors `hypermesh`'s own exact handoff boundary.
pub fn adapt_hypermesh_exact_solid(
    mesh: &hypermesh::exact::ExactMesh,
    solid_handoff: Option<&hypermesh::exact::ExactSolidHandoffReport>,
    source: Option<GridSource>,
) -> HypervoxelResult<HypermeshTriangleSolidAdapter> {
    let source_vertex_count = mesh.vertices().len();
    let source_triangle_count = mesh.triangles().len();
    let mut blockers = Vec::new();

    let handoff = match solid_handoff {
        Some(report) => match report.validate_against_mesh(mesh) {
            Ok(()) => Some(report.clone()),
            Err(_) => {
                blockers.push(HypermeshTriangleSolidAdapterBlocker::StaleSolidHandoff);
                None
            }
        },
        None => match mesh.solid_handoff() {
            Ok(report) => Some(report),
            Err(_) => {
                blockers.push(HypermeshTriangleSolidAdapterBlocker::SolidHandoffNotReady);
                None
            }
        },
    };

    if let Some(report) = &handoff {
        if !report.source_is_exact() {
            blockers.push(HypermeshTriangleSolidAdapterBlocker::SourceNotExact);
        }
        if !report.proof_predicate_ready {
            blockers.push(HypermeshTriangleSolidAdapterBlocker::PredicateReplayNotReady);
        }
    }

    let mut emitted = Vec::new();
    if blockers.is_empty() {
        for (face, triangle) in mesh.triangles().iter().enumerate() {
            let Some(a) = mesh.vertices().get(triangle.0[0]) else {
                blockers.push(HypermeshTriangleSolidAdapterBlocker::MissingVertex);
                break;
            };
            let Some(b) = mesh.vertices().get(triangle.0[1]) else {
                blockers.push(HypermeshTriangleSolidAdapterBlocker::MissingVertex);
                break;
            };
            let Some(c) = mesh.vertices().get(triangle.0[2]) else {
                blockers.push(HypermeshTriangleSolidAdapterBlocker::MissingVertex);
                break;
            };
            emitted.push(ExactTriangle3::new(
                [
                    a.coordinates().0.clone(),
                    b.coordinates().0.clone(),
                    c.coordinates().0.clone(),
                ],
                Some(face as u64),
            ));
        }
    }

    if blockers.is_empty() && emitted.len() != source_triangle_count {
        blockers.push(HypermeshTriangleSolidAdapterBlocker::TriangleCountMismatch);
    }
    if let Some(report) = &handoff {
        if blockers.is_empty() && emitted.len() != report.audit.face_count {
            blockers.push(HypermeshTriangleSolidAdapterBlocker::TriangleCountMismatch);
        }
    }

    let exact_triangle_solid_ready = blockers.is_empty() && !emitted.is_empty();
    let solid = exact_triangle_solid_ready.then(|| {
        ExactTriangleSolidMesh::new(
            ExactTriangleSurfaceMesh::new(emitted.clone(), source, true),
            true,
        )
    });
    let report = HypermeshTriangleSolidAdapterReport {
        source_vertex_count,
        source_triangle_count,
        emitted_triangle_count: emitted.len(),
        blockers,
        exact_triangle_solid_ready,
    };
    Ok(HypermeshTriangleSolidAdapter { solid, report })
}
