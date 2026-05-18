//! Voxel artifact manifests for downstream indexes.
//!
//! `hyperparts`, `hyperpack`, `hyperphysics`, and `hypercircuit` may index or
//! request voxel artifacts, but they should not infer voxel semantics from a
//! filename, cache key, or primitive preview. This module provides a compact
//! manifest/report pair for cataloging grid artifacts with their role,
//! freshness, aggregate certainty, storage replay status, and handoff
//! destinations. Following Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997, the artifact remains an object with
//! explicit provenance and certification status rather than an unqualified
//! sampled scalar field.

use crate::{
    AggregateCertainty, FreshnessStatus, StorageReplayStatus, VoxelAggregateFacts,
    VoxelHandoffDomain,
};

/// Stable caller-supplied identifier for a voxel artifact.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VoxelArtifactId(pub String);

/// Role of a cataloged voxel artifact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelArtifactRole {
    /// Exact/certified occupancy cache.
    OccupancyCache,
    /// Material-region grid.
    MaterialRegionGrid,
    /// Field/sample grid.
    FieldSampleGrid,
    /// Process/support/swept-volume grid.
    ProcessGrid,
    /// Preview/export artifact.
    PreviewArtifact,
    /// Storage/interchange snapshot.
    StorageSnapshot,
}

/// Manifest for a voxel artifact in an external index.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelArtifactManifest {
    /// Stable artifact identifier.
    pub id: VoxelArtifactId,
    /// Artifact role.
    pub role: VoxelArtifactRole,
    /// Source/report freshness.
    pub freshness: FreshnessStatus,
    /// Aggregate facts for the artifact.
    pub aggregate: VoxelAggregateFacts,
    /// Storage replay status.
    pub storage_replay: StorageReplayStatus,
    /// Number of required side-table links missing from the artifact.
    pub missing_side_table_links: usize,
    /// Domains this artifact is intended to serve.
    pub intended_domains: Vec<VoxelHandoffDomain>,
}

/// Report for a cataloged voxel artifact.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelArtifactReport {
    /// Stable artifact identifier.
    pub id: VoxelArtifactId,
    /// Artifact role.
    pub role: VoxelArtifactRole,
    /// Source/report freshness.
    pub freshness: FreshnessStatus,
    /// Aggregate certainty.
    pub aggregate_certainty: AggregateCertainty,
    /// Storage replay status.
    pub storage_replay: StorageReplayStatus,
    /// Number of required side-table links missing from the artifact.
    pub missing_side_table_links: usize,
    /// Domains this artifact is intended to serve.
    pub intended_domains: Vec<VoxelHandoffDomain>,
    /// Whether the artifact id is non-empty after trimming whitespace.
    ///
    /// A catalog entry without a stable id cannot be replayed or cross-checked
    /// by a downstream index. Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997, frames exactness as structured
    /// object evidence; an unnamed artifact is not durable object evidence.
    pub stable_id_ready: bool,
    /// Whether the artifact declares at least one intended consumer domain.
    ///
    /// Indexing without a destination domain leaves the object's admission
    /// policy ambiguous. The domain is not a physical law, but it is part of
    /// the provenance envelope needed before exact evidence is promoted.
    pub intended_domain_ready: bool,
    /// Whether this role may ever be promoted as exact evidence.
    pub role_supports_exact_indexing: bool,
    /// Whether the aggregate packet contains retained voxel evidence.
    ///
    /// A catalog entry may be current and well named, but an empty aggregate
    /// packet is not evidence for indexed voxel content. This mirrors Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997: exact indexing is an object-evidence claim, not just a
    /// metadata claim.
    pub has_aggregate_evidence: bool,
    /// Whether the artifact can be indexed as exact evidence.
    pub indexable_as_exact: bool,
}

impl VoxelArtifactManifest {
    /// Builds an artifact report from manifest facts.
    pub fn report(&self) -> VoxelArtifactReport {
        let role_supports_exact_indexing = self.role.supports_exact_indexing();
        let stable_id_ready = !self.id.0.trim().is_empty();
        let intended_domain_ready = !self.intended_domains.is_empty();
        let has_aggregate_evidence = self.aggregate.child_count > 0;
        let indexable_as_exact = role_supports_exact_indexing
            && stable_id_ready
            && intended_domain_ready
            && has_aggregate_evidence
            && self.freshness == FreshnessStatus::Current
            && self.aggregate.certainty == AggregateCertainty::Exact
            && self.storage_replay == StorageReplayStatus::Exact
            && self.missing_side_table_links == 0;
        VoxelArtifactReport {
            id: self.id.clone(),
            role: self.role,
            freshness: self.freshness,
            aggregate_certainty: self.aggregate.certainty,
            storage_replay: self.storage_replay,
            missing_side_table_links: self.missing_side_table_links,
            intended_domains: self.intended_domains.clone(),
            stable_id_ready,
            intended_domain_ready,
            role_supports_exact_indexing,
            has_aggregate_evidence,
            indexable_as_exact,
        }
    }
}

impl VoxelArtifactRole {
    /// Returns whether this artifact family can stand as exact voxel evidence.
    ///
    /// Preview artifacts are adapter outputs by definition. Even when their
    /// inputs were exact and their storage is deterministic, Yap's exact object
    /// boundary requires the exact voxel artifact to be indexed instead of a
    /// display/export derivative; see Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997.
    pub fn supports_exact_indexing(self) -> bool {
        !matches!(self, Self::PreviewArtifact)
    }
}
