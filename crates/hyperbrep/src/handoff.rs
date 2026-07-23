//! Exact retained-BREP downstream handoff reports.
//!
//! These handoffs package retained BREP evidence for downstream crates such as
//! `hypervoxel`, `hyperphysics`, `hyperpath`, and `hyperpack`. They are not
//! tessellation handoffs: derived meshes remain separate artifacts with their
//! own provenance and replay reports.

use crate::bounds::BrepShellBoundsReport;
use crate::solid::BrepSolidReadinessReport;
use crate::topology::BrepShell;
use crate::validation::BrepShellValidationReport;
use crate::volume::BrepShellVolumeReport;

/// Explicit blocker for exact retained-BREP surface handoff.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepExactSurfaceHandoffBlocker {
    /// Retained shell validation did not reach exact surface-boundary readiness.
    ShellValidationNotReady,
    /// Exact shell bounds could not be derived.
    ShellBoundsNotReady,
    /// The shell has no retained face evidence.
    EmptyShell,
}

/// Explicit blocker for exact retained-BREP solid handoff.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepExactSolidHandoffBlocker {
    /// Retained surface-boundary evidence is not ready.
    SurfaceHandoffNotReady,
    /// Solid readiness did not reach exact closed-boundary readiness.
    SolidReadinessNotReady,
    /// Exact signed-volume/orientation replay is not ready.
    VolumeNotReady,
}

/// Exact retained-surface handoff for downstream consumers.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepExactSurfaceHandoffReport {
    /// Shell validation replayed at handoff time.
    pub validation: BrepShellValidationReport,
    /// Exact shell bounds replayed at handoff time.
    pub bounds: BrepShellBoundsReport,
    /// Number of retained faces.
    pub face_count: usize,
    /// Number of retained vertices.
    pub vertex_count: usize,
    /// Whether the retained shell is closed as solid topology.
    pub closed_shell: bool,
    /// Whether this handoff has nonempty retained topology evidence.
    pub nonempty_topology: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepExactSurfaceHandoffBlocker>,
    /// Whether downstream crates may consume this as exact retained surface
    /// boundary evidence.
    pub exact_surface_handoff_ready: bool,
    /// Whether this report is over retained BREP topology rather than a
    /// derived mesh.
    pub retained_brep_only: bool,
}

/// Exact retained-solid handoff for downstream consumers.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepExactSolidHandoffReport {
    /// Surface-boundary handoff replayed at solid handoff time.
    pub surface: BrepExactSurfaceHandoffReport,
    /// Solid readiness replayed at handoff time.
    pub solid: BrepSolidReadinessReport,
    /// Exact signed-volume/orientation replayed at handoff time.
    pub volume: BrepShellVolumeReport,
    /// Explicit blockers.
    pub blockers: Vec<BrepExactSolidHandoffBlocker>,
    /// Whether downstream crates may consume this as exact retained closed-solid
    /// boundary evidence.
    pub exact_solid_handoff_ready: bool,
    /// Whether this report is over retained BREP topology rather than a
    /// derived mesh.
    pub retained_brep_only: bool,
}

impl BrepExactSurfaceHandoffReport {
    /// Build an exact retained-surface handoff from current shell evidence.
    ///
    /// Downstream crates receive a replayed object-evidence package, not a
    /// naked boolean. Open surfaces are acceptable when their retained face
    /// boundaries, support surfaces, and bounds are exact-ready; closed-solid
    /// interpretation belongs to [`BrepExactSolidHandoffReport`].
    pub fn from_shell(shell: &BrepShell) -> Self {
        let validation = shell.shell_validation_report();
        let bounds = validation.bounds.clone();
        let face_count = shell.faces.len();
        let vertex_count = shell.vertices.len();
        let nonempty_topology = face_count > 0 && vertex_count > 0;
        let closed_shell = validation.exact_closed_shell_ready;

        let mut blockers = Vec::new();
        if !nonempty_topology {
            blockers.push(BrepExactSurfaceHandoffBlocker::EmptyShell);
        }
        if !validation.exact_surface_boundary_ready {
            blockers.push(BrepExactSurfaceHandoffBlocker::ShellValidationNotReady);
        }
        if !bounds.exact_bounds_ready {
            blockers.push(BrepExactSurfaceHandoffBlocker::ShellBoundsNotReady);
        }
        let exact_surface_handoff_ready = blockers.is_empty();

        Self {
            validation,
            bounds,
            face_count,
            vertex_count,
            closed_shell,
            nonempty_topology,
            blockers,
            exact_surface_handoff_ready,
            retained_brep_only: true,
        }
    }
}

impl BrepExactSolidHandoffReport {
    /// Build an exact retained-solid handoff from current shell evidence.
    ///
    /// This is the retained-BREP counterpart to `hypermesh`'s exact solid
    /// handoff: it requires a ready retained surface handoff, closed-shell solid
    /// readiness and exact signed-volume
    /// replay. Derived triangulations remain separate mesh handoffs and are not
    /// promoted to trusted solid topology.
    pub fn from_shell(shell: &BrepShell) -> Self {
        let surface = BrepExactSurfaceHandoffReport::from_shell(shell);
        let solid = shell.solid_readiness_report();
        let volume = solid.volume.clone();

        let mut blockers = Vec::new();
        if !surface.exact_surface_handoff_ready {
            blockers.push(BrepExactSolidHandoffBlocker::SurfaceHandoffNotReady);
        }
        if !solid.exact_solid_boundary_ready {
            blockers.push(BrepExactSolidHandoffBlocker::SolidReadinessNotReady);
        }
        if !volume.exact_volume_ready {
            blockers.push(BrepExactSolidHandoffBlocker::VolumeNotReady);
        }
        let exact_solid_handoff_ready = blockers.is_empty();

        Self {
            surface,
            solid,
            volume,
            blockers,
            exact_solid_handoff_ready,
            retained_brep_only: true,
        }
    }
}

impl BrepShell {
    /// Build an exact retained-surface handoff report.
    pub fn exact_surface_handoff(&self) -> BrepExactSurfaceHandoffReport {
        BrepExactSurfaceHandoffReport::from_shell(self)
    }

    /// Build an exact retained-solid handoff report.
    pub fn exact_solid_handoff(&self) -> BrepExactSolidHandoffReport {
        BrepExactSolidHandoffReport::from_shell(self)
    }
}
