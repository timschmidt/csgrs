//! Explicit status for legacy/lossy adapters.

/// Legacy adapter family.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LegacyAdapterKind {
    /// Voxelis-style OBJ triangle voxelizer.
    VoxelisObjVoxelize,
    /// Greedy mesh generation for display/export.
    GreedyMesh,
    /// Bevy or other rendering preview.
    PreviewRenderer,
    /// VTM-style interchange.
    VtmExport,
    /// Generic import/export manifest or file adapter.
    ImportExport,
}

/// Report carried when a primitive-float adapter participates in a result.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LegacyAdapterStatus {
    /// Adapter family.
    pub kind: LegacyAdapterKind,
    /// Human-readable policy or tolerance summary.
    pub policy: String,
    /// Whether an exact replay validated the adapter's boundary decisions.
    pub exact_replay: bool,
}

impl LegacyAdapterStatus {
    /// Creates a lossy adapter status with no exact replay.
    pub fn lossy(kind: LegacyAdapterKind, policy: impl Into<String>) -> Self {
        Self {
            kind,
            policy: policy.into(),
            exact_replay: false,
        }
    }

    /// Creates an adapter status that has exact replay metadata.
    pub fn exact(kind: LegacyAdapterKind, policy: impl Into<String>) -> Self {
        Self {
            kind,
            policy: policy.into(),
            exact_replay: true,
        }
    }
}
