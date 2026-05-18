//! Explicit status for legacy/lossy adapters.
//!
//! Legacy adapters are admissible only as named, auditable boundary stages.
//! This follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7(1-2), 1997, pp. 3-23: a primitive-float stage may propose
//! geometry, but exact consumers need explicit provenance for the approximation
//! policy before they can replay or reject its combinatorial decisions.

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

    /// Returns whether the adapter declared a non-empty policy/provenance text.
    ///
    /// Empty or whitespace-only policy strings are not useful audit evidence:
    /// they name an adapter family but do not expose the numeric or replay
    /// convention that separated the adapter from exact topology. Under Yap's
    /// exact-geometric-computation model, that missing boundary contract keeps
    /// the adapter outside exact replay even when a boolean says replay exists.
    pub fn has_policy(&self) -> bool {
        !self.policy.trim().is_empty()
    }

    /// Returns whether the adapter can participate in exact replay gates.
    ///
    /// Exact replay requires both the replay flag and explicit policy
    /// provenance. Callers that only need to display raw adapter status may read
    /// [`Self::exact_replay`] directly; exact topology, audit, and numeric
    /// contract gates should use this method.
    pub fn exact_replay_ready(&self) -> bool {
        self.exact_replay && self.has_policy()
    }
}
