//! Continuous-field voxel handoff intake.
//!
//! Continuous implicit/SDF fields are owned by crates such as `hypersdf`;
//! `hypervoxel` owns grid frames, cell payloads, aggregate facts, and storage.
//! This module is the intake boundary between those roles. It accepts explicit
//! cell rows that have already been classified by an exact/certified
//! continuous-field predicate, validates frame/address/source readiness, and
//! only then offers materialization into voxel storage. This follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
//! 1997: sampled grid artifacts must preserve the exact object-level evidence
//! that justified their combinatorial labels.

use std::collections::BTreeSet;

use crate::{
    BoundaryPolicy, FreshnessStatus, GridCoordinateSystem, GridFrame, GridSource, HypervoxelError,
    HypervoxelResult, PreparedVoxelGrid, QuantizationPolicy, SparseVoxelGrid, VoxelAddress,
    VoxelAggregateFacts, VoxelCell, VoxelPredicateCertificateReport, VoxelizationPolicy,
    VoxelizationReport,
};

/// One externally classified continuous-field cell row.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContinuousFieldVoxelCell {
    /// Exact voxel address for this classified cell.
    pub address: VoxelAddress,
    /// Conservative cell payload supplied by the continuous-field owner.
    pub cell: VoxelCell,
}

impl ContinuousFieldVoxelCell {
    /// Construct one externally classified cell row.
    pub const fn new(address: VoxelAddress, cell: VoxelCell) -> Self {
        Self { address, cell }
    }
}

/// Manifest for taking exact continuous-field classifications into `hypervoxel`.
#[derive(Clone, Debug, PartialEq)]
pub struct ContinuousFieldVoxelManifest {
    /// Exact target grid frame.
    pub frame: GridFrame,
    /// Source continuous field version that produced the rows.
    pub source: Option<GridSource>,
    /// Expected source version at intake time.
    pub expected_source: Option<GridSource>,
    /// Expected number of classified rows from the source.
    pub expected_cell_count: usize,
    /// Explicit classified rows.
    pub cells: Vec<ContinuousFieldVoxelCell>,
}

/// Declared row order for a continuous-field voxel interchange.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ContinuousFieldVoxelRowOrder {
    /// Rows are explicit address/cell pairs and do not rely on implicit order.
    ExplicitAddresses,
    /// Rows are sorted by increasing Morton/Z-order code at the frame depth.
    MortonAscending,
    /// Rows are dense z-major, then y, then x-fast order.
    ZMajorYThenXFast,
    /// Producer did not declare row ordering.
    Unknown,
}

/// Producer-declared interchange metadata for continuous-field voxel rows.
///
/// This is a lightweight ABI contract between a continuous-field owner such as
/// `hypersdf` and the `hypervoxel` intake side. It lets the consumer validate
/// row count, frame depth, coordinate-system declaration, dimensions, source
/// freshness, and row-order declaration before any storage materialization.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ContinuousFieldVoxelInterchangeManifest {
    /// Source continuous field version that produced the rows.
    pub source: Option<GridSource>,
    /// Expected source version at intake time.
    pub expected_source: Option<GridSource>,
    /// Declared coordinate-system family.
    pub coordinate_system: GridCoordinateSystem,
    /// Declared row ordering.
    pub row_order: ContinuousFieldVoxelRowOrder,
    /// Declared frame depth.
    pub declared_depth: u8,
    /// Declared cell dimensions.
    pub declared_dimensions: [u64; 3],
    /// Declared number of rows.
    pub declared_cell_count: usize,
}

/// Validation report for [`ContinuousFieldVoxelInterchangeManifest`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ContinuousFieldVoxelInterchangeReport {
    /// Source freshness relative to the expected source.
    pub freshness: FreshnessStatus,
    /// Whether declared depth matches the exact frame.
    pub depth_matches_frame: bool,
    /// Whether declared dimensions match the exact frame.
    pub dimensions_match_frame: bool,
    /// Whether declared row count matches supplied rows and frame volume.
    pub cell_count_matches: bool,
    /// Whether coordinate system was explicitly declared.
    pub coordinate_system_declared: bool,
    /// Whether row ordering was explicitly declared.
    pub row_order_declared: bool,
    /// Whether the interchange metadata is ready for exact intake.
    pub exact_interchange_ready: bool,
}

/// Intake report for externally classified continuous-field cells.
#[derive(Clone, Debug, PartialEq)]
pub struct ContinuousFieldVoxelReport {
    /// Source freshness relative to the expected source.
    pub freshness: FreshnessStatus,
    /// Expected classified cell count.
    pub expected_cell_count: usize,
    /// Supplied classified cell count.
    pub supplied_cell_count: usize,
    /// Number of duplicate addresses in the supplied rows.
    pub duplicate_address_count: usize,
    /// Number of supplied addresses validated against the frame.
    pub frame_validated_cell_count: usize,
    /// Whether every supplied row is at the frame's finest depth.
    pub finest_depth_only: bool,
    /// Whether the supplied rows exactly match the expected row count.
    pub complete_expected_cover: bool,
    /// Whether every supplied cell has exact-ready cell evidence.
    pub exact_cell_evidence_ready: bool,
    /// Whether this intake can be materialized as exact voxel evidence.
    pub exact_materialization_ready: bool,
    /// Predicate-style accounting derived from conservative occupancy rows.
    pub predicate_certificates: VoxelPredicateCertificateReport,
    /// Conservative aggregate facts over supplied rows.
    pub aggregate: VoxelAggregateFacts,
}

impl ContinuousFieldVoxelManifest {
    /// Build an intake report without allocating storage.
    pub fn report(&self) -> ContinuousFieldVoxelReport {
        let freshness = match (&self.source, &self.expected_source) {
            (Some(source), Some(expected)) if source == expected => FreshnessStatus::Current,
            (Some(_), Some(_)) => FreshnessStatus::Stale,
            _ => FreshnessStatus::Unknown,
        };

        let mut seen = BTreeSet::new();
        let mut duplicate_address_count = 0_usize;
        let mut frame_validated_cell_count = 0_usize;
        let mut finest_depth_only = true;
        let mut exact_cell_evidence_ready = true;
        let mut inside_cells = 0_usize;
        let mut outside_cells = 0_usize;
        let mut boundary_cells = 0_usize;
        let mut unknown_cells = 0_usize;

        for row in &self.cells {
            if !seen.insert(row.address) {
                duplicate_address_count += 1;
            }
            if row.address.depth <= self.frame.depth() {
                frame_validated_cell_count += 1;
            }
            finest_depth_only &= row.address.depth == self.frame.depth();

            let cell_report = row.cell.report();
            exact_cell_evidence_ready &= cell_report.exact_cell_evidence_ready;
            match row.cell.occupancy {
                crate::OccupancyState::Filled => inside_cells += 1,
                crate::OccupancyState::Empty => outside_cells += 1,
                crate::OccupancyState::Boundary | crate::OccupancyState::Mixed => {
                    boundary_cells += 1;
                }
                crate::OccupancyState::Unknown | crate::OccupancyState::LossyAdapterValue => {
                    unknown_cells += 1;
                }
            }
        }

        let supplied_cell_count = self.cells.len();
        let complete_expected_cover =
            self.expected_cell_count > 0 && supplied_cell_count == self.expected_cell_count;
        let predicate_certificates = VoxelPredicateCertificateReport::from_counts(
            inside_cells,
            outside_cells,
            boundary_cells,
            unknown_cells,
        );
        let aggregate = VoxelAggregateFacts::from_cells(self.cells.iter().map(|row| &row.cell));
        let exact_materialization_ready = freshness == FreshnessStatus::Current
            && duplicate_address_count == 0
            && frame_validated_cell_count == supplied_cell_count
            && finest_depth_only
            && complete_expected_cover
            && exact_cell_evidence_ready
            && predicate_certificates.is_fully_certified()
            && !aggregate.has_unknown
            && !aggregate.has_lossy;

        ContinuousFieldVoxelReport {
            freshness,
            expected_cell_count: self.expected_cell_count,
            supplied_cell_count,
            duplicate_address_count,
            frame_validated_cell_count,
            finest_depth_only,
            complete_expected_cover,
            exact_cell_evidence_ready,
            exact_materialization_ready,
            predicate_certificates,
            aggregate,
        }
    }

    /// Validate producer-declared interchange metadata against this exact frame.
    ///
    /// The manifest does not inspect payload semantics; that is handled by
    /// [`Self::report`]. This method checks the object/provenance envelope that
    /// lets independent crates agree on which exact grid and row stream they
    /// are discussing before materialization.
    pub fn interchange_report(
        &self,
        manifest: &ContinuousFieldVoxelInterchangeManifest,
    ) -> ContinuousFieldVoxelInterchangeReport {
        let freshness = match (&manifest.source, &manifest.expected_source) {
            (Some(source), Some(expected)) if source == expected => FreshnessStatus::Current,
            (Some(_), Some(_)) => FreshnessStatus::Stale,
            _ => FreshnessStatus::Unknown,
        };
        let cells_per_axis = self.frame.cells_per_axis();
        let expected_dimensions = [cells_per_axis, cells_per_axis, cells_per_axis];
        let expected_cell_count = cells_per_axis
            .checked_mul(cells_per_axis)
            .and_then(|area| area.checked_mul(cells_per_axis))
            .and_then(|volume| usize::try_from(volume).ok());
        let depth_matches_frame = manifest.declared_depth == self.frame.depth();
        let dimensions_match_frame = manifest.declared_dimensions == expected_dimensions;
        let cell_count_matches = manifest.declared_cell_count == self.cells.len()
            && expected_cell_count == Some(manifest.declared_cell_count);
        let coordinate_system_declared =
            !matches!(manifest.coordinate_system, GridCoordinateSystem::Unknown);
        let row_order_declared =
            !matches!(manifest.row_order, ContinuousFieldVoxelRowOrder::Unknown);
        let exact_interchange_ready = freshness == FreshnessStatus::Current
            && depth_matches_frame
            && dimensions_match_frame
            && cell_count_matches
            && coordinate_system_declared
            && row_order_declared;
        ContinuousFieldVoxelInterchangeReport {
            freshness,
            depth_matches_frame,
            dimensions_match_frame,
            cell_count_matches,
            coordinate_system_declared,
            row_order_declared,
            exact_interchange_ready,
        }
    }

    /// Build a standard voxelization report from the intake report.
    pub fn voxelization_report(&self) -> VoxelizationReport {
        let report = self.report();
        VoxelizationReport {
            source: self.source.clone(),
            frame: self.frame.clone(),
            policy: VoxelizationPolicy {
                quantization: QuantizationPolicy::ConservativeCover,
                boundary: BoundaryPolicy::KeepBoundary,
            },
            aggregate: report.aggregate,
            unknown_cells: report.predicate_certificates.unknown_cells,
            boundary_cells: report.predicate_certificates.boundary_cells,
            predicate_certificates: report.predicate_certificates,
            legacy_adapter: None,
        }
    }

    /// Materialize a prepared sparse grid from exact-ready intake rows.
    ///
    /// The method still returns a grid for incomplete or uncertain rows, but
    /// the attached report exposes that the result is not exact topology. This
    /// keeps storage construction deterministic while preserving Yap's rule
    /// that uncertain evidence remains visible to consumers.
    pub fn materialize_sparse_grid(&self) -> HypervoxelResult<PreparedVoxelGrid<SparseVoxelGrid>> {
        let mut grid = SparseVoxelGrid::new(self.frame.clone());
        for row in &self.cells {
            grid.set(row.address, row.cell)?;
        }
        let aggregate = grid.stored_aggregate();
        Ok(PreparedVoxelGrid::new(self.frame.clone(), grid, aggregate)
            .with_report(self.voxelization_report()))
    }
}

/// Build a finest-depth address for a continuous-field intake row.
pub fn continuous_field_address(
    frame: &GridFrame,
    xyz: [u64; 3],
) -> HypervoxelResult<VoxelAddress> {
    VoxelAddress::new(frame.depth(), xyz).map_err(|error| match error {
        HypervoxelError::DepthTooLarge {
            depth,
            max_supported,
        } => HypervoxelError::DepthTooLarge {
            depth,
            max_supported,
        },
        other => other,
    })
}
