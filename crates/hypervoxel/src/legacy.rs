//! Explicit status for legacy/lossy adapters.
//!
//! Primitive-float stages may produce preview or interoperability values, but
//! their adapter family is not retained as exact geometry evidence.

/// Legacy adapter family.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LegacyAdapterKind {
    /// Voxelis SVO-DAG storage/interner compatibility check.
    VoxelisStorage,
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

/// Marker carried when a primitive-float adapter participates in a result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LegacyAdapterStatus {
    /// Adapter family.
    pub kind: LegacyAdapterKind,
}

impl LegacyAdapterStatus {
    /// Creates an adapter marker.
    pub const fn new(kind: LegacyAdapterKind) -> Self {
        Self { kind }
    }
}
