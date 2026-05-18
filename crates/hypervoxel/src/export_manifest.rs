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
    /// Whether at least one exact voxel primitive was consumed.
    ///
    /// A preview/export route with no input primitives can still describe a
    /// valid empty export, but it is not topology replay evidence. Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997, keeps exact claims attached to explicit object facts.
    pub has_input_primitives: bool,
    /// Whether at least one display/export primitive was emitted.
    pub has_exported_primitives: bool,
    /// Whether the route is exact replay or preview-only.
    pub freshness: FreshnessStatus,
    /// Whether exact voxel-grid topology was preserved by the route.
    pub exact_grid_topology_replay: bool,
    /// Whether this export can certify source CAD/mesh geometry.
    pub source_geometry_replay: bool,
    /// Explicit adapter status.
    pub adapter: LegacyAdapterStatus,
}

impl PreviewExportManifest {
    /// Builds a preview/export report from manifest facts only.
    pub fn report(&self) -> PreviewExportReport {
        let has_input_primitives = self.exact_input_primitives > 0;
        let has_exported_primitives = self.exported_primitives > 0;
        let exact = has_input_primitives
            && has_exported_primitives
            && self.preserves_grid_topology
            && self.has_explicit_labels
            && !matches!(self.scalar_policy, PreviewScalarPolicy::PrimitiveFloat)
            && !matches!(self.format, PreviewExportFormat::ContinuousSdfPreview);
        PreviewExportReport {
            format: self.format,
            exact_input_primitives: self.exact_input_primitives,
            exported_primitives: self.exported_primitives,
            has_input_primitives,
            has_exported_primitives,
            freshness: if exact {
                FreshnessStatus::Current
            } else {
                FreshnessStatus::Unknown
            },
            exact_grid_topology_replay: exact,
            // Preview/export routes are derived views of voxel artifacts, not
            // source CAD or mesh certificates. Lorensen and Cline's marching
            // cubes work is valuable display extraction; Yap's exact geometry
            // boundary keeps source-geometry truth in proof-carrying source
            // objects and voxelization reports.
            source_geometry_replay: false,
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
