//! Exact voxelization preflight handoff for retained BREP shells.
//!
//! `hypervoxel` owns grid frames, sparse storage, and voxelization reports.
//! `hyperbrep` owns retained shell evidence. This module bridges the two by
//! packaging exact BREP triangles and an exact shell AABB fixture for voxel
//! broad-phase/scheduling, and preparing the exact retained triangle solid
//! that `hypervoxel` can voxelize.

use hypervoxel::{
    ExactBox, ExactTriangle3, ExactTriangleSolidMesh, ExactTriangleSurfaceMesh, GridFrame,
    GridSource, PreparedExactTriangleSolidMesh, PreparedExactTriangleSolidMeshReport,
};

use crate::bounds::BrepShellBoundsReport;
use crate::topology::BrepShell;
use crate::triangle::BrepExactTriangleMeshHandoffReport;

/// Explicit blocker for BREP-to-voxel preflight handoff.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepVoxelHandoffBlocker {
    /// Exact shell bounds were not available.
    ShellBoundsNotReady,
    /// Exact retained-BREP triangle lowering was not available.
    TriangleMeshNotReady,
    /// The caller required a frame/source match and the frame had no source.
    MissingFrameSource,
    /// The frame source did not match the expected source/version.
    StaleFrameSource,
    /// Exact retained triangles could not be prepared as a `hypervoxel`
    /// closed-solid schedule.
    TriangleSolidPreparationNotReady,
}

/// BREP-side exact voxel handoff preflight.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepVoxelHandoffReport {
    /// Grid frame supplied by the voxel owner.
    pub frame: GridFrame,
    /// Expected source/version, when the caller wants freshness replay.
    pub expected_source: Option<GridSource>,
    /// Exact shell bounds used to build the AABB fixture.
    pub bounds: BrepShellBoundsReport,
    /// Exact triangle handoff shared with mesh/physics consumers.
    pub triangle_mesh: BrepExactTriangleMeshHandoffReport,
    /// Exact AABB fixture in the supplied frame's coordinate system.
    pub exact_aabb_fixture: Option<ExactBox>,
    /// Exact triangle-solid handoff consumable by `hypervoxel`.
    pub exact_triangle_solid: Option<ExactTriangleSolidMesh>,
    /// Prepared exact triangle-solid schedule for `hypervoxel` voxelization.
    pub prepared_triangle_solid: Option<PreparedExactTriangleSolidMesh>,
    /// Prepared schedule report, retained even when the prepared object is
    /// unavailable.
    pub prepared_triangle_solid_report: Option<PreparedExactTriangleSolidMeshReport>,
    /// Whether the AABB broad-phase fixture is ready for `hypervoxel`.
    pub exact_aabb_handoff_ready: bool,
    /// Whether exact retained triangles are available for a future
    /// triangle-voxelization owner.
    pub exact_triangle_source_ready: bool,
    /// Whether exact retained triangle-solid voxelization is available now.
    pub exact_triangle_voxelization_ready: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepVoxelHandoffBlocker>,
}

impl BrepVoxelHandoffReport {
    /// Build a BREP-to-voxel preflight report from retained shell evidence.
    ///
    /// This follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997): the voxel owner receives exact
    /// object evidence and named blockers rather than an approximate triangle
    /// rasterization. The AABB fixture uses `hypervoxel::ExactBox`; exact
    /// triangle-solid voxelization is exposed only after the retained BREP
    /// triangle handoff lowers into [`ExactTriangleSolidMesh`] and replays
    /// through [`PreparedExactTriangleSolidMesh`]. The handoff follows the
    /// BREP topology model of Mäntylä, *An Introduction to Solid Modeling*
    /// (1988), while the prepared voxel schedule follows the exact
    /// broad/narrow-phase separation used by Yap (1997).
    pub fn from_shell_frame(
        shell: &BrepShell,
        frame: GridFrame,
        expected_source: Option<GridSource>,
    ) -> Self {
        let bounds = shell.shell_bounds_report();
        let triangle_mesh = shell.exact_triangle_mesh_handoff_report();
        let mut blockers = Vec::new();

        if !bounds.exact_bounds_ready {
            blockers.push(BrepVoxelHandoffBlocker::ShellBoundsNotReady);
        }
        if !triangle_mesh.exact_triangle_mesh_ready {
            blockers.push(BrepVoxelHandoffBlocker::TriangleMeshNotReady);
        }
        if let Some(expected) = expected_source.as_ref() {
            match frame.source() {
                Some(source) if source == expected => {}
                Some(_) => blockers.push(BrepVoxelHandoffBlocker::StaleFrameSource),
                None => blockers.push(BrepVoxelHandoffBlocker::MissingFrameSource),
            }
        }
        let exact_aabb_fixture = match (&bounds.min, &bounds.max) {
            (Some(min), Some(max)) if bounds.exact_bounds_ready => Some(ExactBox::new(
                [min.x.clone(), min.y.clone(), min.z.clone()],
                [max.x.clone(), max.y.clone(), max.z.clone()],
                frame.source().cloned(),
            )),
            _ => None,
        };
        let exact_triangle_solid = if triangle_mesh.exact_triangle_mesh_ready {
            Some(triangle_solid_from_brep_triangles(
                &triangle_mesh,
                frame.source().cloned(),
            ))
        } else {
            None
        };
        let (prepared_triangle_solid, prepared_triangle_solid_report) =
            match exact_triangle_solid.clone() {
                Some(solid) => match PreparedExactTriangleSolidMesh::prepare(solid) {
                    Ok(prepared) => {
                        let report = prepared.report().clone();
                        (Some(prepared), Some(report))
                    }
                    Err(_) => {
                        blockers.push(BrepVoxelHandoffBlocker::TriangleSolidPreparationNotReady);
                        (None, None)
                    }
                },
                None => (None, None),
            };
        let exact_aabb_handoff_ready = exact_aabb_fixture.is_some()
            && !blockers.iter().any(|blocker| {
                matches!(
                    blocker,
                    BrepVoxelHandoffBlocker::ShellBoundsNotReady
                        | BrepVoxelHandoffBlocker::MissingFrameSource
                        | BrepVoxelHandoffBlocker::StaleFrameSource
                )
            });
        let exact_triangle_source_ready = triangle_mesh.exact_triangle_mesh_ready;
        let exact_triangle_voxelization_ready = prepared_triangle_solid.is_some()
            && !blockers.iter().any(|blocker| {
                matches!(
                    blocker,
                    BrepVoxelHandoffBlocker::TriangleMeshNotReady
                        | BrepVoxelHandoffBlocker::MissingFrameSource
                        | BrepVoxelHandoffBlocker::StaleFrameSource
                        | BrepVoxelHandoffBlocker::TriangleSolidPreparationNotReady
                )
            });

        Self {
            frame,
            expected_source,
            bounds,
            triangle_mesh,
            exact_aabb_fixture,
            exact_triangle_solid,
            prepared_triangle_solid,
            prepared_triangle_solid_report,
            exact_aabb_handoff_ready,
            exact_triangle_source_ready,
            exact_triangle_voxelization_ready,
            blockers,
        }
    }
}

fn triangle_solid_from_brep_triangles(
    triangle_mesh: &BrepExactTriangleMeshHandoffReport,
    source: Option<GridSource>,
) -> ExactTriangleSolidMesh {
    let triangles = triangle_mesh
        .triangles
        .iter()
        .enumerate()
        .map(|(index, triangle)| {
            ExactTriangle3::new(
                triangle
                    .vertices
                    .clone()
                    .map(|point| [point.x, point.y, point.z]),
                Some(index as u64),
            )
        })
        .collect::<Vec<_>>();
    ExactTriangleSolidMesh::new(ExactTriangleSurfaceMesh::new(triangles, source, true), true)
}

impl BrepShell {
    /// Package this shell for exact `hypervoxel` broad-phase/intake preflight.
    pub fn voxel_handoff_report(
        &self,
        frame: GridFrame,
        expected_source: Option<GridSource>,
    ) -> BrepVoxelHandoffReport {
        BrepVoxelHandoffReport::from_shell_frame(self, frame, expected_source)
    }
}
