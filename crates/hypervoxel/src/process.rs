//! Process-grid artifact reports.
//!
//! `hypervoxel` does not own CAM, resin chemistry, or physics. It can,
//! however, package exact occupancy/sample grids with process provenance so
//! `hyperpath` and `hyperphysics` can consume them without guessing whether a
//! grid is additive material, subtractive removal, dose, conversion, or support
//! state. The boundary follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997: store exact object facts and explicit
//! provenance, not hidden interpretation.

use crate::{FreshnessStatus, GridSource, VoxelAggregateFacts};

/// Intended process role for a voxel artifact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ProcessGridRole {
    /// Material added by a process.
    AdditiveOccupancy,
    /// Material removed by a process.
    SubtractiveOccupancy,
    /// Support/accessibility mask.
    SupportMask,
    /// Photopolymer exposure dose field.
    PhotopolymerDose,
    /// Photopolymer conversion/gel-state field.
    PhotopolymerConversion,
    /// Porous, scaffold, fluid, thermal, EM, or mechanical sample field.
    PhysicalFieldSample,
    /// Swept-volume broad-phase or process cache.
    SweptVolumeCache,
    /// Stock/removal or remaining-material cache.
    StockRemovalCache,
    /// Controller-facing preview artifact.
    ControllerPreview,
}

/// Provenance for a swept-volume or path-derived voxel cache.
///
/// `hypervoxel` intentionally stores only the replay facts needed to identify
/// the cache. Exact path geometry and clearance predicates remain in
/// `hyperpath`/`hyperlimit`; this report lets callers reject stale or lossy
/// process grids before they are used as broad-phase evidence. This follows
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7(1-2), 1997, by keeping the source construction and approximation policy
/// explicit next to the voxel artifact.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SweptVolumeProvenance {
    /// Path, route, exposure, or tool-sweep source.
    pub source: Option<GridSource>,
    /// Expected source version for exact path replay, when known.
    pub expected_source_version: Option<u64>,
    /// Tool, beam, nozzle, cutter, or trace-width label supplied by a domain crate.
    pub tool_or_beam: Option<String>,
    /// Whether exact path/source replay is available outside this voxel cache.
    pub exact_source_replay_available: bool,
    /// Whether this voxel cache is only a conservative broad-phase artifact.
    pub broad_phase_only: bool,
    /// Human-readable quantization or sampling policy.
    pub quantization_policy: String,
}

/// Safety report for path/process-derived voxel caches.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SweptVolumeReport {
    /// Source freshness/provenance.
    pub source: Option<GridSource>,
    /// Whether the source version is current enough to replay exact path facts.
    pub source_freshness: FreshnessStatus,
    /// Tool, beam, nozzle, cutter, or trace-width label supplied by a domain crate.
    pub tool_or_beam: Option<String>,
    /// Whether a non-empty tool/beam identity was supplied.
    ///
    /// A swept voxel cache without tool or beam identity is an acceleration
    /// artifact, not exact process/path evidence. Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997, keeps
    /// exact object claims bound to their source objects; here that includes
    /// the process object that generated the swept volume.
    pub has_tool_or_beam: bool,
    /// Whether exact replay is available in the owning domain crate.
    pub exact_source_replay_available: bool,
    /// Whether downstream consumers must treat the grid as broad-phase only.
    pub broad_phase_only: bool,
    /// Whether a non-empty quantization/sampling policy was supplied.
    pub has_quantization_policy: bool,
    /// Whether this artifact is acceptable as exact path/clearance truth.
    pub can_stand_in_for_exact_path: bool,
    /// Human-readable quantization or sampling policy.
    pub quantization_policy: String,
}

impl SweptVolumeProvenance {
    /// Builds a swept-volume safety report.
    ///
    /// Exact path evidence is only ready when the path/source construction can
    /// still be replayed. This is the process-grid analogue of Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997:
    /// cached voxel facts may accelerate a query, but a stale or unversioned
    /// cache is not a certificate for the source object.
    pub fn report(&self) -> SweptVolumeReport {
        let source_freshness = match (&self.source, self.expected_source_version) {
            (Some(source), Some(expected)) if source.version == expected => {
                FreshnessStatus::Current
            }
            (Some(_), Some(_)) => FreshnessStatus::Stale,
            _ => FreshnessStatus::Unknown,
        };
        let has_tool_or_beam = self
            .tool_or_beam
            .as_ref()
            .is_some_and(|label| !label.trim().is_empty());
        let has_quantization_policy = !self.quantization_policy.trim().is_empty();
        SweptVolumeReport {
            source: self.source.clone(),
            source_freshness,
            tool_or_beam: self.tool_or_beam.clone(),
            has_tool_or_beam,
            exact_source_replay_available: self.exact_source_replay_available,
            broad_phase_only: self.broad_phase_only,
            has_quantization_policy,
            can_stand_in_for_exact_path: self.exact_source_replay_available
                && source_freshness == FreshnessStatus::Current
                && has_tool_or_beam
                && has_quantization_policy
                && !self.broad_phase_only,
            quantization_policy: self.quantization_policy.clone(),
        }
    }
}

/// Provenance report for a process-oriented voxel grid.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProcessGridArtifact {
    /// Artifact role.
    pub role: ProcessGridRole,
    /// Source path, exposure, material lot, or simulation handle.
    pub source: Option<GridSource>,
    /// Process-state labels or IDs supplied by the owning domain crate.
    pub process_tags: Vec<String>,
    /// Conservative aggregate facts for the grid.
    pub aggregate: VoxelAggregateFacts,
    /// Optional swept-volume/path provenance for process caches.
    pub swept_volume: Option<SweptVolumeProvenance>,
}

impl ProcessGridArtifact {
    /// Creates a process-grid artifact report.
    pub fn new(
        role: ProcessGridRole,
        source: Option<GridSource>,
        process_tags: Vec<String>,
        aggregate: VoxelAggregateFacts,
    ) -> Self {
        Self {
            role,
            source,
            process_tags,
            aggregate,
            swept_volume: None,
        }
    }

    /// Attaches swept-volume provenance to a process grid.
    pub fn with_swept_volume(mut self, swept_volume: SweptVolumeProvenance) -> Self {
        self.swept_volume = Some(swept_volume);
        self
    }
}
