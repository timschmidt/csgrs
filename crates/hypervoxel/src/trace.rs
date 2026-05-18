//! Trace dimensions for exact voxel operations.
//!
//! Benchmarks and downstream diagnostics need to distinguish exact predicate
//! work, storage/interner work, lossy adapter lowering, and domain handoff
//! checks. This module keeps those dimensions semantic rather than timing-only.
//! That matches Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7(1-2), 1997: exact systems should preserve the structure of the
//! operation so later decisions can see which object-level facts were proved
//! and which adapter routes were merely replayed.

use std::collections::BTreeSet;

/// Named operation dimension for tracing or benchmark grouping.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum VoxelTraceDimension {
    /// Exact grid-frame construction and validation.
    GridFrameConstruction,
    /// Source version/freshness checks.
    SourceVersionCheck,
    /// Exact voxelization predicate batches.
    ExactVoxelizationPredicateBatch,
    /// Conservative occupancy/material/field aggregate propagation.
    OccupancyAggregatePropagation,
    /// SVO-DAG interning and node reuse.
    SvoDagInterning,
    /// Deterministic batch edits.
    BatchedEdits,
    /// Prepared voxel query handles.
    PreparedQuery,
    /// Conservative LOD aggregate queries.
    LodAggregateQuery,
    /// Image-stack import/export lowering.
    ImageStackIoLowering,
    /// Common voxel-interchange import/export lowering.
    VoxelInterchangeLowering,
    /// Lossy mesh/export lowering.
    LossyMeshExportLowering,
    /// Voxel/mesh/physics/path/circuit handoff reports.
    DomainHandoffReport,
}

/// Trace manifest for one exact or adapter-backed voxel operation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelTraceManifest {
    /// Human-readable operation label.
    pub operation: String,
    /// Dimensions touched by the operation.
    pub dimensions: Vec<VoxelTraceDimension>,
    /// Number of exact predicate calls or certified comparisons.
    pub exact_predicate_count: usize,
    /// Number of primitive-float/lossy adapter operations.
    pub lossy_adapter_count: usize,
    /// Number of explicit unknown outcomes preserved in reports.
    pub unknown_count: usize,
}

/// Auditable trace summary.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelTraceReport {
    /// Human-readable operation label.
    pub operation: String,
    /// Deduplicated dimensions in deterministic order.
    pub dimensions: Vec<VoxelTraceDimension>,
    /// Number of distinct dimensions.
    pub dimension_count: usize,
    /// Number of exact predicate calls or certified comparisons.
    pub exact_predicate_count: usize,
    /// Number of primitive-float/lossy adapter operations.
    pub lossy_adapter_count: usize,
    /// Number of explicit unknown outcomes preserved in reports.
    pub unknown_count: usize,
    /// Whether the trace carries at least one semantic operation dimension.
    pub has_operation_dimension: bool,
    /// Whether the trace carries at least one exact predicate/comparison.
    pub has_exact_evidence: bool,
    /// Whether this trace contains any lossy adapter work.
    pub has_lossy_adapter_work: bool,
    /// Whether this trace preserved uncertainty explicitly.
    pub has_unknowns: bool,
    /// Whether the trace can be consumed as exact operation evidence.
    ///
    /// This is intentionally stricter than "contains exact predicates": a trace
    /// may include exact work and still be unsuitable as exact evidence if any
    /// step lowered through a primitive-float adapter or ended in an explicit
    /// unknown. A vacuous trace is also rejected: at least one semantic
    /// operation dimension and one exact predicate/comparison must be present.
    /// That follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997, by keeping exact decisions
    /// separated from adapter replay, undecided predicates, and empty timing
    /// shells.
    pub exact_trace_evidence_ready: bool,
}

impl VoxelTraceManifest {
    /// Builds a deterministic trace report.
    pub fn report(&self) -> VoxelTraceReport {
        let dimensions = self
            .dimensions
            .iter()
            .copied()
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        let has_operation_dimension = !dimensions.is_empty();
        let has_exact_evidence = self.exact_predicate_count > 0;
        VoxelTraceReport {
            operation: self.operation.clone(),
            dimension_count: dimensions.len(),
            dimensions,
            exact_predicate_count: self.exact_predicate_count,
            lossy_adapter_count: self.lossy_adapter_count,
            unknown_count: self.unknown_count,
            has_operation_dimension,
            has_exact_evidence,
            has_lossy_adapter_work: self.lossy_adapter_count > 0,
            has_unknowns: self.unknown_count > 0,
            exact_trace_evidence_ready: has_operation_dimension
                && has_exact_evidence
                && self.lossy_adapter_count == 0
                && self.unknown_count == 0,
        }
    }
}
