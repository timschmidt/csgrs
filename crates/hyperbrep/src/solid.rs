//! Solid-readiness reports.
//!
//! A retained BREP shell is not automatically a solid. This module aggregates
//! closure, per-face validation, exact shell bounds, and optional construction
//! freshness into a report-bearing solid handoff gate for downstream consumers.

use crate::bounds::BrepShellBoundsReport;
use crate::provenance::{BrepConstructionManifest, BrepConstructionProvenanceReport};
use crate::report::BrepShellClosureReport;
use crate::topology::BrepShell;
use crate::validation::BrepFaceValidationReport;
use crate::volume::BrepShellVolumeReport;

/// Explicit blocker for solid-readiness handoff.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepSolidReadinessBlocker {
    /// Shell closure/topology audit is not exact-ready.
    ShellClosureNotReady,
    /// Exact shell bounds could not be derived.
    ShellBoundsNotReady,
    /// At least one face validation report is not exact-ready.
    FaceValidationNotReady,
    /// Optional construction provenance was supplied but is stale or rejected.
    ConstructionNotFresh,
    /// The shell has no faces.
    EmptyShell,
    /// Exact volume or orientation proof is not implemented yet.
    VolumeReplayUnavailable,
}

/// Exact solid-readiness report for one retained shell.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepSolidReadinessReport {
    /// Shell closure replayed at report time.
    pub shell_closure: BrepShellClosureReport,
    /// Exact shell AABB/support facts.
    pub shell_bounds: BrepShellBoundsReport,
    /// Per-face validation reports.
    pub faces: Vec<BrepFaceValidationReport>,
    /// Optional construction freshness replay.
    pub construction: Option<BrepConstructionProvenanceReport>,
    /// Exact signed-volume/orientation evidence.
    pub volume: BrepShellVolumeReport,
    /// Number of exact-ready faces.
    pub ready_face_count: usize,
    /// Number of blocked faces.
    pub blocked_face_count: usize,
    /// Whether retained topology is closed and exact-ready.
    pub closed_shell_ready: bool,
    /// Whether every face is exact-ready.
    pub all_faces_ready: bool,
    /// Whether exact shell bounds are ready.
    pub exact_bounds_ready: bool,
    /// Whether optional construction evidence is fresh, or no construction
    /// evidence was requested.
    pub construction_fresh: bool,
    /// Whether exact volume/orientation replay is available.
    pub exact_volume_ready: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepSolidReadinessBlocker>,
    /// Whether the shell is ready as exact solid boundary evidence for
    /// downstream consumers.
    pub exact_solid_boundary_ready: bool,
}

impl BrepSolidReadinessReport {
    /// Build a solid-readiness report from retained BREP shell evidence.
    ///
    /// This is intentionally stricter than a mesh/export gate and intentionally
    /// narrower than a full CAD kernel. It follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997): solid
    /// consumers may use the shell only when closure, face evidence, bounds,
    /// source freshness, and exact signed volume replay as exact/certified
    /// facts. The volume component uses the determinant-based algebraic
    /// certificate described by Mirtich, "Fast and Accurate Computation of
    /// Polyhedral Mass Properties," *Journal of Graphics Tools* 1.2 (1996),
    /// but blocks instead of guessing whenever the retained BREP evidence is
    /// not exact-ready.
    pub fn from_shell(shell: &BrepShell, construction: Option<&BrepConstructionManifest>) -> Self {
        let shell_closure = shell.audit_closure();
        let shell_bounds = shell.shell_bounds_report();
        let faces = shell
            .faces
            .iter()
            .map(|face| shell.face_validation_report(face.id, None))
            .collect::<Vec<_>>();
        let construction = construction.map(|manifest| manifest.report(shell));
        let volume = shell.shell_volume_report();
        let ready_face_count = faces.iter().filter(|face| face.exact_face_ready).count();
        let blocked_face_count = faces.len().saturating_sub(ready_face_count);
        let closed_shell_ready = shell_closure.exact_shell_ready;
        let all_faces_ready = !faces.is_empty() && blocked_face_count == 0;
        let exact_bounds_ready = shell_bounds.exact_bounds_ready;
        let construction_fresh = construction
            .as_ref()
            .is_none_or(|report| report.construction_fresh);
        let exact_volume_ready = volume.exact_volume_ready;

        let mut blockers = Vec::new();
        if shell.faces.is_empty() {
            blockers.push(BrepSolidReadinessBlocker::EmptyShell);
        }
        if !closed_shell_ready {
            blockers.push(BrepSolidReadinessBlocker::ShellClosureNotReady);
        }
        if !exact_bounds_ready {
            blockers.push(BrepSolidReadinessBlocker::ShellBoundsNotReady);
        }
        if !all_faces_ready {
            blockers.push(BrepSolidReadinessBlocker::FaceValidationNotReady);
        }
        if !construction_fresh {
            blockers.push(BrepSolidReadinessBlocker::ConstructionNotFresh);
        }
        if !exact_volume_ready {
            blockers.push(BrepSolidReadinessBlocker::VolumeReplayUnavailable);
        }

        let exact_solid_boundary_ready = blockers.is_empty()
            && closed_shell_ready
            && all_faces_ready
            && exact_bounds_ready
            && exact_volume_ready
            && construction_fresh;
        Self {
            shell_closure,
            shell_bounds,
            faces,
            construction,
            volume,
            ready_face_count,
            blocked_face_count,
            closed_shell_ready,
            all_faces_ready,
            exact_bounds_ready,
            construction_fresh,
            exact_volume_ready,
            blockers,
            exact_solid_boundary_ready,
        }
    }
}

impl BrepShell {
    /// Build a solid-readiness report from retained shell evidence.
    pub fn solid_readiness_report(
        &self,
        construction: Option<&BrepConstructionManifest>,
    ) -> BrepSolidReadinessReport {
        BrepSolidReadinessReport::from_shell(self, construction)
    }
}
