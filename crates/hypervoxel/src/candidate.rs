//! Candidate-evaluation reports for search and packing consumers.
//!
//! Metaheuristics may propose grid resolutions, support masks, dose schedules,
//! compression policies, or LOD choices, but the proposal is not admissible
//! until exact/certified voxel reports replay it. These small reports give
//! `hyperevolution`, `hyperpack`, and process planners a common boundary
//! without moving their objective functions into `hypervoxel`. The rule follows
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7(1-2), 1997: a candidate is accepted because its exact object reports are
//! structurally replayable, not because a sampled scalar score looks good.

use crate::{AggregateCertainty, FreshnessStatus};

/// Proposed voxel candidate family.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelCandidateKind {
    /// Grid resolution, origin, pitch, or depth candidate.
    GridResolution,
    /// Support or process mask candidate.
    SupportOrProcessMask,
    /// Material-region assignment candidate.
    MaterialRegionAssignment,
    /// Exposure, dose, conversion, or gel-threshold schedule candidate.
    ExposureDoseSchedule,
    /// Compression, paging, or LOD policy candidate.
    CompressionOrLodPolicy,
    /// Packing, nesting, or broad-phase occupancy candidate.
    PackingOccupancy,
}

/// Manifest for replaying a proposed voxel candidate.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelCandidateManifest {
    /// Candidate family.
    pub kind: VoxelCandidateKind,
    /// Source/report freshness after replay.
    pub freshness: FreshnessStatus,
    /// Aggregate certainty after replay.
    pub aggregate_certainty: AggregateCertainty,
    /// Number of explicit unknown facts preserved by replay.
    pub unknown_count: usize,
    /// Number of lossy adapter facts preserved by replay.
    pub lossy_count: usize,
    /// Whether exact source or constraint replay succeeded.
    pub exact_replay_available: bool,
    /// Number of exact replay facts retained for this candidate.
    pub exact_evidence_count: usize,
}

/// Result of checking whether a voxel candidate can be promoted.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelCandidateReport {
    /// Candidate family.
    pub kind: VoxelCandidateKind,
    /// Source/report freshness after replay.
    pub freshness: FreshnessStatus,
    /// Aggregate certainty after replay.
    pub aggregate_certainty: AggregateCertainty,
    /// Number of explicit unknown facts preserved by replay.
    pub unknown_count: usize,
    /// Number of lossy adapter facts preserved by replay.
    pub lossy_count: usize,
    /// Whether exact source or constraint replay succeeded.
    pub exact_replay_available: bool,
    /// Number of exact replay facts retained for this candidate.
    pub exact_evidence_count: usize,
    /// Whether at least one exact replay fact was retained.
    ///
    /// A candidate can be fresh and structurally exact but still carry no
    /// replayed object facts. Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997, requires exact decisions to rest
    /// on explicit object evidence rather than vacuous status checks.
    pub has_exact_evidence: bool,
    /// Whether a downstream optimizer may promote this candidate as exact.
    pub promotable_as_exact: bool,
}

impl VoxelCandidateManifest {
    /// Builds a candidate report from replay facts.
    pub fn report(&self) -> VoxelCandidateReport {
        let has_exact_evidence = self.exact_evidence_count > 0;
        let promotable_as_exact = self.freshness == FreshnessStatus::Current
            && self.aggregate_certainty == AggregateCertainty::Exact
            && self.unknown_count == 0
            && self.lossy_count == 0
            && has_exact_evidence
            && self.exact_replay_available;
        VoxelCandidateReport {
            kind: self.kind,
            freshness: self.freshness,
            aggregate_certainty: self.aggregate_certainty,
            unknown_count: self.unknown_count,
            lossy_count: self.lossy_count,
            exact_replay_available: self.exact_replay_available,
            exact_evidence_count: self.exact_evidence_count,
            has_exact_evidence,
            promotable_as_exact,
        }
    }
}
