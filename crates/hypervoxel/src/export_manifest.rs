//! Preview/export adapter manifests.
//!
//! Meshes, Bevy scenes, glTF files, OBJ text, SDF previews, and VTM-style
//! exports are useful operational views of a voxel artifact, but they are not
//! exact source geometry unless an exact replay says so. These manifests keep
//! adapter status explicit. The marching-cubes/SDF boundary is intentionally
//! only a preview route here; Lorensen and Cline, "Marching Cubes,"
//! *Computer Graphics* 21(4), 1987, is a display extraction idea, while Yap's
//! exact-geometric-computation model keeps exact predicates and combinatorial
//! truth in the source/grid reports.

use crate::{FreshnessStatus, LegacyAdapterKind, LegacyAdapterStatus};

/// Preview/export route family.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PreviewExportFormat {
    /// Wavefront OBJ preview text.
    Obj,
    /// glTF preview asset.
    Gltf,
    /// Bevy scene or asset route.
    BevyScene,
    /// VTM-style multi-block visualization export.
    Vtm,
    /// Continuous signed-distance-field preview.
    ContinuousSdfPreview,
}

/// Scalar lowering used by a preview/export route.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PreviewScalarPolicy {
    /// No metric scalar lowering occurs.
    None,
    /// Exact scalars are printed as exact strings.
    ExactString,
    /// Exact scalars are lowered to primitive floats.
    PrimitiveFloat,
}

/// Manifest for a preview/export route.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PreviewExportManifest {
    /// Route family.
    pub format: PreviewExportFormat,
    /// Number of exact voxel faces or cells consumed.
    pub exact_input_primitives: usize,
    /// Number of exported display primitives.
    pub exported_primitives: usize,
    /// Scalar lowering policy.
    pub scalar_policy: PreviewScalarPolicy,
    /// Whether exported topology is known to preserve exact grid topology.
    pub preserves_grid_topology: bool,
    /// Whether material/display labels are explicit in the export.
    pub has_explicit_labels: bool,
}

/// Report for a preview/export route.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PreviewExportReport {
    /// Route family.
    pub format: PreviewExportFormat,
    /// Input primitive count.
    pub exact_input_primitives: usize,
    /// Output primitive count.
    pub exported_primitives: usize,
    /// Whether the route is exact replay or preview-only.
    pub freshness: FreshnessStatus,
    /// Explicit adapter status.
    pub adapter: LegacyAdapterStatus,
}

impl PreviewExportManifest {
    /// Builds a preview/export report from manifest facts only.
    pub fn report(&self) -> PreviewExportReport {
        let exact = self.preserves_grid_topology
            && self.has_explicit_labels
            && !matches!(self.scalar_policy, PreviewScalarPolicy::PrimitiveFloat);
        PreviewExportReport {
            format: self.format,
            exact_input_primitives: self.exact_input_primitives,
            exported_primitives: self.exported_primitives,
            freshness: if exact {
                FreshnessStatus::Current
            } else {
                FreshnessStatus::Unknown
            },
            adapter: if exact {
                LegacyAdapterStatus::exact(kind(self.format), "preview/export manifest")
            } else {
                LegacyAdapterStatus::lossy(kind(self.format), "preview/export manifest")
            },
        }
    }
}

fn kind(format: PreviewExportFormat) -> LegacyAdapterKind {
    match format {
        PreviewExportFormat::Obj | PreviewExportFormat::Gltf => LegacyAdapterKind::GreedyMesh,
        PreviewExportFormat::BevyScene | PreviewExportFormat::ContinuousSdfPreview => {
            LegacyAdapterKind::PreviewRenderer
        }
        PreviewExportFormat::Vtm => LegacyAdapterKind::VtmExport,
    }
}
