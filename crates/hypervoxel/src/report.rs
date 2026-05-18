//! Report-bearing voxelization and prepared-grid handles.
//!
//! Voxelization reports keep exact predicate accounting next to the resulting
//! grid. This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997: the combinatorial classification
//! made by predicates is part of the exact object-level state, not debug text
//! that can be dropped after cells are written.

use crate::{
    FreshnessStatus::Current, GridFrame, GridSource, LegacyAdapterStatus, VoxelAggregateFacts,
};

/// Freshness of source-dependent grid data.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FreshnessStatus {
    /// Source versions match.
    Current,
    /// Source version is stale.
    Stale,
    /// Freshness could not be checked.
    Unknown,
}

/// Quantization policy for mapping exact geometry into cells.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum QuantizationPolicy {
    /// Include every cell touched by the source.
    ConservativeCover,
    /// Include only cells certified fully inside the source.
    ConservativeInterior,
    /// Preserve boundary cells explicitly.
    BoundaryPreserving,
    /// Sample an unsigned address-space distance preview.
    UnsignedDistanceSampling,
    /// Sample a signed address-space distance preview.
    SignedDistanceSampling,
    /// Rasterize exact material-region membership into payload IDs.
    MaterialRegionRasterization,
    /// Rasterize process exposure or dose state into payload IDs.
    ProcessExposureGrid,
    /// Adapter-only preview policy.
    LossyPreview,
}

/// Boundary-cell representation policy.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BoundaryPolicy {
    /// Keep boundary cells as boundary.
    KeepBoundary,
    /// Mark boundary cells unknown.
    BoundaryAsUnknown,
    /// Adapter chose a side; not exact.
    LossySideChoice,
}

/// Complete policy for a voxelization pass.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VoxelizationPolicy {
    /// Quantization behavior.
    pub quantization: QuantizationPolicy,
    /// Boundary behavior.
    pub boundary: BoundaryPolicy,
}

impl VoxelizationPolicy {
    /// Conservative cover with explicit boundary cells.
    pub fn conservative_cover() -> Self {
        Self {
            quantization: QuantizationPolicy::ConservativeCover,
            boundary: BoundaryPolicy::KeepBoundary,
        }
    }

    /// Returns whether this policy can certify cells without adapter evidence.
    ///
    /// Distance and process policies are still exact report roles when their
    /// inputs are exact, but they are not the same claim as topological
    /// occupancy. The distinction follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997: a report must name
    /// the predicate or construction it certifies rather than letting callers
    /// infer topology from an approximate-looking grid value.
    pub fn is_exact_semantic_role(&self) -> bool {
        !matches!(self.quantization, QuantizationPolicy::LossyPreview)
    }

    /// Returns whether this policy claims topological occupancy.
    pub fn is_occupancy_policy(&self) -> bool {
        matches!(
            self.quantization,
            QuantizationPolicy::ConservativeCover
                | QuantizationPolicy::ConservativeInterior
                | QuantizationPolicy::BoundaryPreserving
                | QuantizationPolicy::MaterialRegionRasterization
        )
    }
}

/// Exact predicate classification accounting for one voxelization pass.
///
/// The counts are classifier outcomes before an output policy chooses whether
/// boundary cells are stored, omitted, marked unknown, or sent through a lossy
/// adapter. Keeping this separate prevents a conservative-interior policy from
/// erasing evidence that boundary predicates were evaluated.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct VoxelPredicateCertificateReport {
    /// Cells certified fully inside the source.
    pub inside_cells: usize,
    /// Cells certified fully outside the source.
    pub outside_cells: usize,
    /// Cells certified as boundary or partial-overlap cells.
    pub boundary_cells: usize,
    /// Cells whose predicate ordering could not be certified.
    pub unknown_cells: usize,
}

impl VoxelPredicateCertificateReport {
    /// Builds a predicate report from explicit classifier counts.
    pub fn from_counts(
        inside_cells: usize,
        outside_cells: usize,
        boundary_cells: usize,
        unknown_cells: usize,
    ) -> Self {
        Self {
            inside_cells,
            outside_cells,
            boundary_cells,
            unknown_cells,
        }
    }

    /// Returns the number of cells with certified predicate outcomes.
    pub fn certified_cells(&self) -> usize {
        self.inside_cells + self.outside_cells + self.boundary_cells
    }

    /// Returns the full number of classified cells, including unknowns.
    pub fn classified_cells(&self) -> usize {
        self.certified_cells() + self.unknown_cells
    }
}

/// Report from a voxelization or import pass.
#[derive(Clone, Debug, PartialEq)]
pub struct VoxelizationReport {
    /// Source used for voxelization.
    pub source: Option<GridSource>,
    /// Grid frame used by the result.
    pub frame: GridFrame,
    /// Voxelization policy.
    pub policy: VoxelizationPolicy,
    /// Exact/certified aggregate facts for the result.
    pub aggregate: VoxelAggregateFacts,
    /// Explicit unknown cell count.
    pub unknown_cells: usize,
    /// Explicit boundary cell count.
    pub boundary_cells: usize,
    /// Predicate certificate accounting before output policy lowering.
    pub predicate_certificates: VoxelPredicateCertificateReport,
    /// Optional lossy adapter status.
    pub legacy_adapter: Option<LegacyAdapterStatus>,
}

impl VoxelizationReport {
    /// Computes freshness against the grid frame source.
    pub fn freshness(&self) -> FreshnessStatus {
        match (self.frame.source(), self.source.as_ref()) {
            (Some(frame), Some(report)) if frame == report => Current,
            (Some(_), Some(_)) => FreshnessStatus::Stale,
            _ => FreshnessStatus::Unknown,
        }
    }
}

/// Prepared grid handle with retained aggregate facts.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedVoxelGrid<S> {
    /// Grid frame.
    pub frame: GridFrame,
    /// Storage backend or handle.
    pub storage: S,
    /// Retained aggregate facts.
    pub aggregate: VoxelAggregateFacts,
    /// Optional source report.
    pub report: Option<VoxelizationReport>,
}

impl<S> PreparedVoxelGrid<S> {
    /// Creates a prepared grid handle.
    pub fn new(frame: GridFrame, storage: S, aggregate: VoxelAggregateFacts) -> Self {
        Self {
            frame,
            storage,
            aggregate,
            report: None,
        }
    }

    /// Attaches a voxelization report.
    pub fn with_report(mut self, report: VoxelizationReport) -> Self {
        self.report = Some(report);
        self
    }
}
