//! Display/export adapter certificates.
//!
//! Exported meshes, polylines, and external BREP files are operational
//! artifacts. They may carry exact replay evidence, but they are not the
//! authoritative BREP topology. This module gives outbound adapters the same
//! report-bearing boundary as imports and tessellation handoffs.

use std::collections::BTreeSet;

use crate::tessellation::BrepMeshHandoffReport;

/// Display/export route family.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum BrepExportFormat {
    /// Wavefront OBJ triangle mesh.
    Obj,
    /// glTF triangle mesh or scene.
    Gltf,
    /// Polyline/edge preview.
    Polyline,
    /// STEP-style external BREP data.
    Step,
    /// Native/external BREP exchange package.
    ExternalBrep,
    /// Format was not declared.
    Unknown,
}

impl BrepExportFormat {
    /// Returns whether this format exports a derived mesh.
    pub const fn requires_mesh_handoff(self) -> bool {
        matches!(self, Self::Obj | Self::Gltf)
    }

    /// Returns whether this format attempts to export external BREP topology.
    pub const fn is_external_brep(self) -> bool {
        matches!(self, Self::Step | Self::ExternalBrep)
    }
}

/// Scalar lowering used by an export route.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum BrepExportScalarPolicy {
    /// Exact values were lowered to primitive `f32`.
    F32,
    /// Exact values were lowered to primitive `f64`.
    F64,
    /// Exact decimal/rational strings were emitted.
    ExactText,
    /// Scalar policy was not declared.
    Unknown,
}

/// Manifest for one BREP display/export route.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepExportManifest {
    /// Export format.
    pub format: BrepExportFormat,
    /// Scalar lowering policy.
    pub scalar_policy: BrepExportScalarPolicy,
    /// Source BREP object ids retained in the exported artifact.
    pub source_object_ids: Vec<String>,
    /// Number of emitted primitives, such as triangles, lines, or faces.
    pub exported_primitives: usize,
    /// Number of scalar coordinates emitted.
    pub exported_coordinates: usize,
    /// Number of emitted scalar coordinates known finite after lowering.
    pub finite_exported_coordinates: usize,
    /// Whether material or face labels were preserved explicitly.
    pub labels_preserved: bool,
    /// Whether the export route claims exact replay evidence.
    pub exact_replay_declared: bool,
}

impl BrepExportManifest {
    /// Build a report without reading the generated file bytes.
    pub fn report(&self, mesh_handoff: Option<&BrepMeshHandoffReport>) -> BrepExportReport {
        BrepExportReport::from_manifest(self, mesh_handoff)
    }
}

/// Explicit blocker for display/export readiness.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepExportBlocker {
    /// Export format was not declared.
    UnknownFormat,
    /// Scalar lowering policy was not declared.
    UnknownScalarPolicy,
    /// Export retained no source object ids.
    MissingSourceObjectIds,
    /// Export emitted no primitives.
    EmptyExport,
    /// Some lowered coordinates were non-finite.
    NonFiniteExportCoordinates,
    /// Mesh format did not include a mesh handoff report.
    MissingMeshHandoff,
    /// Mesh handoff was present but not ready as exact derived surface evidence.
    MeshHandoffNotReady,
    /// External BREP export attempted without exact replay evidence.
    ExternalBrepReplayMissing,
}

/// Report for a BREP display/export route.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepExportReport {
    /// Export format.
    pub format: BrepExportFormat,
    /// Scalar lowering policy.
    pub scalar_policy: BrepExportScalarPolicy,
    /// Number of source object ids retained by the route.
    pub source_object_count: usize,
    /// Number of emitted primitives.
    pub exported_primitives: usize,
    /// Number of scalar coordinates emitted.
    pub exported_coordinates: usize,
    /// Number of finite coordinates emitted.
    pub finite_exported_coordinates: usize,
    /// Whether all emitted coordinates are finite.
    pub all_exported_coordinates_finite: bool,
    /// Whether material or face labels were preserved explicitly.
    pub labels_preserved: bool,
    /// Whether exact replay evidence was declared by the exporter.
    pub exact_replay_declared: bool,
    /// Whether the route had ready mesh handoff evidence when needed.
    pub mesh_handoff_ready: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepExportBlocker>,
    /// Whether this route is ready as an export artifact.
    pub export_ready: bool,
    /// Whether this output remains an adapter/export artifact, not authoritative BREP topology.
    pub export_adapter_only: bool,
}

impl BrepExportReport {
    /// Build an export report from manifest facts and optional mesh handoff.
    ///
    /// File artifacts remain adapter products even when exact object evidence
    /// and replay certificates accompany them. A ready OBJ or glTF route is a
    /// derived mesh view; source BREP topology remains in `BrepShell`.
    pub fn from_manifest(
        manifest: &BrepExportManifest,
        mesh_handoff: Option<&BrepMeshHandoffReport>,
    ) -> Self {
        let mut blockers = BTreeSet::new();
        if manifest.format == BrepExportFormat::Unknown {
            blockers.insert(BrepExportBlocker::UnknownFormat);
        }
        if manifest.scalar_policy == BrepExportScalarPolicy::Unknown {
            blockers.insert(BrepExportBlocker::UnknownScalarPolicy);
        }
        if manifest.source_object_ids.is_empty()
            || manifest
                .source_object_ids
                .iter()
                .any(|id| id.trim().is_empty())
        {
            blockers.insert(BrepExportBlocker::MissingSourceObjectIds);
        }
        if manifest.exported_primitives == 0 {
            blockers.insert(BrepExportBlocker::EmptyExport);
        }
        let all_exported_coordinates_finite =
            manifest.exported_coordinates == manifest.finite_exported_coordinates;
        if !all_exported_coordinates_finite {
            blockers.insert(BrepExportBlocker::NonFiniteExportCoordinates);
        }

        let mesh_handoff_ready = if manifest.format.requires_mesh_handoff() {
            match mesh_handoff {
                Some(report) if report.exact_surface_handoff_ready => true,
                Some(_) => {
                    blockers.insert(BrepExportBlocker::MeshHandoffNotReady);
                    false
                }
                None => {
                    blockers.insert(BrepExportBlocker::MissingMeshHandoff);
                    false
                }
            }
        } else {
            true
        };
        if manifest.format.is_external_brep() && !manifest.exact_replay_declared {
            blockers.insert(BrepExportBlocker::ExternalBrepReplayMissing);
        }

        let blockers = blockers.into_iter().collect::<Vec<_>>();
        Self {
            format: manifest.format,
            scalar_policy: manifest.scalar_policy,
            source_object_count: manifest.source_object_ids.len(),
            exported_primitives: manifest.exported_primitives,
            exported_coordinates: manifest.exported_coordinates,
            finite_exported_coordinates: manifest.finite_exported_coordinates,
            all_exported_coordinates_finite,
            labels_preserved: manifest.labels_preserved,
            exact_replay_declared: manifest.exact_replay_declared,
            mesh_handoff_ready,
            export_ready: blockers.is_empty(),
            blockers,
            export_adapter_only: true,
        }
    }
}
