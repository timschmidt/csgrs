//! Exact `hypermesh` input adapter.
//!
//! This module validates a retained [`hypermesh::InputMesh`] as a closed,
//! consistently wound solid before lowering it into
//! [`crate::ExactTriangleSolidMesh`]. It copies the retained exact coordinates
//! directly into `hypervoxel`'s triangle-solid carrier. This follows Yap, "Towards Exact Geometric
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
/// `hypermesh::prepare_input` owns the closed-solid and PWN validation. The
/// adapter does not infer solid evidence from an unchecked triangle soup.
pub fn adapt_hypermesh_exact_solid(
    mesh: &hypermesh::InputMesh,
    source: Option<GridSource>,
) -> HypervoxelResult<HypermeshTriangleSolidAdapter> {
    let source_vertex_count = mesh.positions.len();
    let source_triangle_count = mesh.triangles.len();
    let mut blockers = Vec::new();

    if hypermesh::prepare_input(&[mesh.as_ref()]).is_err() {
        blockers.push(HypermeshTriangleSolidAdapterBlocker::SolidHandoffNotReady);
    }

    let mut emitted = Vec::new();
    if blockers.is_empty() {
        for (face, triangle) in mesh.triangles.iter().enumerate() {
            let [ia, ib, ic] = triangle.indices();
            let Some(a) = mesh.positions.get(ia) else {
                blockers.push(HypermeshTriangleSolidAdapterBlocker::MissingVertex);
                break;
            };
            let Some(b) = mesh.positions.get(ib) else {
                blockers.push(HypermeshTriangleSolidAdapterBlocker::MissingVertex);
                break;
            };
            let Some(c) = mesh.positions.get(ic) else {
                blockers.push(HypermeshTriangleSolidAdapterBlocker::MissingVertex);
                break;
            };
            emitted.push(ExactTriangle3::new(
                [
                    [a.x.clone(), a.y.clone(), a.z.clone()],
                    [b.x.clone(), b.y.clone(), b.z.clone()],
                    [c.x.clone(), c.y.clone(), c.z.clone()],
                ],
                Some(face as u64),
            ));
        }
    }

    if blockers.is_empty() && emitted.len() != source_triangle_count {
        blockers.push(HypermeshTriangleSolidAdapterBlocker::TriangleCountMismatch);
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
