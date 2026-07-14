//! Domain handoff reports for voxel artifacts.
//!
//! `hypervoxel` stores grid artifacts and conservative facts; it does not own
//! physical laws, toolpath planning, part identity, or circuit semantics. These
//! reports give domain crates a checked handoff surface with source freshness,
//! side-table link, and aggregate-status visibility. The object and its
//! provenance remain explicit instead of hiding assumptions in payload IDs.

use crate::{AggregateCertainty, FreshnessStatus, GridSource, VoxelAggregateFacts};

/// Downstream domain that can consume a voxel artifact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelHandoffDomain {
    /// Physical simulation/material-property crate.
    Hyperphysics,
    /// Toolpath and swept-volume crate.
    Hyperpath,
    /// Part/catalog/provenance crate.
    Hyperparts,
    /// Circuit/EM candidate extraction crate.
    Hypercircuit,
}

/// Status of side-table references needed by a handoff.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SideTableLinkStatus {
    /// All required links were supplied.
    Complete,
    /// Some links are missing and must remain unknown.
    Missing,
    /// No links are required for this handoff.
    NotRequired,
}

/// Manifest for handing a voxel artifact to a domain crate.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelHandoffManifest {
    /// Destination domain.
    pub domain: VoxelHandoffDomain,
    /// Source artifact or model version.
    pub source: Option<GridSource>,
    /// Expected source version at handoff time.
    pub expected_source: Option<GridSource>,
    /// Number of material/field/process links required by the destination.
    pub required_side_table_links: usize,
    /// Number of required links supplied by the artifact side tables.
    pub supplied_side_table_links: usize,
    /// Conservative aggregate facts for the artifact being handed off.
    pub aggregate: VoxelAggregateFacts,
}

/// Report for a domain handoff.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelHandoffReport {
    /// Destination domain.
    pub domain: VoxelHandoffDomain,
    /// Source freshness.
    pub freshness: FreshnessStatus,
    /// Side-table link status.
    pub side_table_links: SideTableLinkStatus,
    /// Conservative aggregate facts exposed to the destination.
    pub aggregate: VoxelAggregateFacts,
    /// Aggregate certainty exposed beside the full aggregate packet.
    pub aggregate_certainty: AggregateCertainty,
    /// Whether the aggregate packet contains at least one retained child fact.
    ///
    /// Exact aggregate certainty over an empty packet is still an absence
    /// report, not handoff evidence for a voxel artifact.
    pub has_aggregate_evidence: bool,
    /// Whether the destination may consume this handoff as exact voxel evidence.
    pub exact_handoff_ready: bool,
}

impl VoxelHandoffManifest {
    /// Builds a handoff report with explicit freshness and link status.
    pub fn report(&self) -> VoxelHandoffReport {
        let freshness = match (&self.source, &self.expected_source) {
            (Some(source), Some(expected)) if source == expected => FreshnessStatus::Current,
            (Some(_), Some(_)) => FreshnessStatus::Stale,
            _ => FreshnessStatus::Unknown,
        };
        let side_table_links = if self.required_side_table_links == 0 {
            SideTableLinkStatus::NotRequired
        } else if self.supplied_side_table_links >= self.required_side_table_links {
            SideTableLinkStatus::Complete
        } else {
            SideTableLinkStatus::Missing
        };
        let has_aggregate_evidence = self.aggregate.child_count > 0;
        let exact_handoff_ready = freshness == FreshnessStatus::Current
            && self.aggregate.certainty == AggregateCertainty::Exact
            && has_aggregate_evidence
            && side_table_links != SideTableLinkStatus::Missing;
        VoxelHandoffReport {
            domain: self.domain,
            freshness,
            side_table_links,
            aggregate: self.aggregate.clone(),
            aggregate_certainty: self.aggregate.certainty,
            has_aggregate_evidence,
            exact_handoff_ready,
        }
    }
}
