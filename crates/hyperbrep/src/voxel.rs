//! Exact voxelization preflight handoff for retained BREP shells.
//!
//! `hypervoxel` owns grid frames, sparse storage, and voxelization reports.
//! `hyperbrep` owns retained shell evidence. This module bridges the two by
//! packaging exact BREP triangles and an exact shell AABB fixture for voxel
//! broad-phase/scheduling, while explicitly blocking any claim that general
//! triangle voxelization has already happened.

use hypervoxel::{ExactBox, GridFrame, GridSource};

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
    /// General exact triangle voxelization is not implemented in this crate.
    TriangleVoxelizationUnavailable,
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
    /// Whether the AABB broad-phase fixture is ready for `hypervoxel`.
    pub exact_aabb_handoff_ready: bool,
    /// Whether exact retained triangles are available for a future
    /// triangle-voxelization owner.
    pub exact_triangle_source_ready: bool,
    /// Whether general exact triangle voxelization is available now.
    pub exact_triangle_voxelization_ready: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepVoxelHandoffBlocker>,
}

impl BrepVoxelHandoffReport {
    /// Build a BREP-to-voxel preflight report from retained shell evidence.
    ///
    /// This follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997): the voxel owner receives exact
    /// object evidence and named unavailable work rather than an approximate
    /// triangle rasterization. The AABB fixture uses `hypervoxel::ExactBox`,
    /// while full triangle/solid voxelization remains a future cross-crate
    /// algorithm.
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
        blockers.push(BrepVoxelHandoffBlocker::TriangleVoxelizationUnavailable);

        let exact_aabb_fixture = match (&bounds.min, &bounds.max) {
            (Some(min), Some(max)) if bounds.exact_bounds_ready => Some(ExactBox::new(
                [min.x.clone(), min.y.clone(), min.z.clone()],
                [max.x.clone(), max.y.clone(), max.z.clone()],
                frame.source().cloned(),
            )),
            _ => None,
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
        let exact_triangle_voxelization_ready = false;

        Self {
            frame,
            expected_source,
            bounds,
            triangle_mesh,
            exact_aabb_fixture,
            exact_aabb_handoff_ready,
            exact_triangle_source_ready,
            exact_triangle_voxelization_ready,
            blockers,
        }
    }
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
