//! Exact voxel-grid handoff reports for `hypervoxel` consumers.
//!
//! `hypersdf` owns the continuous field; `hypervoxel` owns grid frames and
//! storage. This module bridges the two without allocating voxel storage or
//! importing `hypervoxel` types: it constructs exact cell AABBs from a retained
//! grid frame, replays SDF cell predicates, and reports whether the frame shape
//! is ready for `hypervoxel`-style octree indexing. This follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997): sampled artifacts must carry the exact object/provenance package
//! that made their combinatorial labels meaningful.

use core::cmp::Ordering;

use hyperlimit::{Point3, PredicateOutcome, compare_reals};
use hyperreal::Real;

use crate::handoff::SdfVoxelHandoffReport;
use crate::status::{SdfCellClassificationReport, SdfCellLocation, SdfFreshness, SdfMetricStatus};

/// Length unit declared for a voxel-grid handoff.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfVoxelLengthUnit {
    /// Unitless model coordinates.
    Unitless,
    /// Meters.
    Meter,
    /// Millimeters.
    Millimeter,
    /// Micrometers.
    Micrometer,
    /// Nanometers.
    Nanometer,
}

/// Source/provenance for a voxel-grid handoff frame.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SdfVoxelGridSource {
    /// Stable source identifier, such as a shape id or construction path.
    pub id: String,
    /// Monotonic construction version.
    pub version: u64,
}

impl SdfVoxelGridSource {
    /// Construct a source/version pair.
    pub fn new(id: impl Into<String>, version: u64) -> Self {
        Self {
            id: id.into(),
            version,
        }
    }
}

/// Exact axis-aligned voxel-cell grid requested from an SDF.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfVoxelCellGrid {
    /// Exact minimum corner of cell `(0, 0, 0)`.
    pub origin: Point3,
    /// Exact positive per-axis cell pitch.
    pub step: Point3,
    /// Number of cells on each axis.
    pub dimensions: [u32; 3],
    /// Declared source unit.
    pub units: SdfVoxelLengthUnit,
    /// Optional source/version metadata.
    pub source: Option<SdfVoxelGridSource>,
}

impl SdfVoxelCellGrid {
    /// Construct a unitless exact voxel-cell grid.
    pub const fn new(origin: Point3, step: Point3, dimensions: [u32; 3]) -> Self {
        Self {
            origin,
            step,
            dimensions,
            units: SdfVoxelLengthUnit::Unitless,
            source: None,
        }
    }

    /// Return a copy with declared length units.
    pub const fn with_units(mut self, units: SdfVoxelLengthUnit) -> Self {
        self.units = units;
        self
    }

    /// Return a copy with source/version metadata.
    pub fn with_source(mut self, source: SdfVoxelGridSource) -> Self {
        self.source = Some(source);
        self
    }

    /// Return the number of cells, validating dimensions and host indexing.
    pub fn cell_count(&self) -> Result<usize, SdfVoxelGridError> {
        if self.dimensions.contains(&0) {
            return Err(SdfVoxelGridError::EmptyDimension);
        }
        let count = self
            .dimensions
            .iter()
            .try_fold(1_u64, |acc, dimension| {
                acc.checked_mul(u64::from(*dimension))
            })
            .ok_or(SdfVoxelGridError::TooManyCells)?;
        usize::try_from(count).map_err(|_| SdfVoxelGridError::TooManyCells)
    }

    /// Validate exact positive cell pitch on all axes.
    pub fn validate_positive_step(&self) -> Result<(), SdfVoxelGridError> {
        validate_positive(&self.step.x, 0)?;
        validate_positive(&self.step.y, 1)?;
        validate_positive(&self.step.z, 2)
    }

    /// Return the `hypervoxel` octree depth when dimensions are a cubic power of two.
    pub fn hypervoxel_depth(&self) -> Option<u8> {
        let [nx, ny, nz] = self.dimensions;
        if nx != ny || nx != nz || !nx.is_power_of_two() {
            return None;
        }
        u8::try_from(nx.trailing_zeros()).ok()
    }

    fn cell_bounds(&self, x: u32, y: u32, z: u32) -> (Point3, Point3) {
        let min = Point3::new(
            &self.origin.x + &(&self.step.x * &Real::from(x)),
            &self.origin.y + &(&self.step.y * &Real::from(y)),
            &self.origin.z + &(&self.step.z * &Real::from(z)),
        );
        let max = Point3::new(
            &self.origin.x + &(&self.step.x * &Real::from(x + 1)),
            &self.origin.y + &(&self.step.y * &Real::from(y + 1)),
            &self.origin.z + &(&self.step.z * &Real::from(z + 1)),
        );
        (min, max)
    }
}

/// Input validation error for exact voxel-grid handoff.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfVoxelGridError {
    /// At least one grid dimension is zero.
    EmptyDimension,
    /// Cell count overflows host indexing.
    TooManyCells,
    /// A cell pitch axis was certified non-positive.
    NonPositiveStep {
        /// Axis index.
        axis: usize,
    },
    /// A cell pitch axis could not be certified positive.
    UnknownStepSign {
        /// Axis index.
        axis: usize,
    },
}

/// Conservative occupancy label exported toward voxel/grid storage.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfVoxelOccupancy {
    /// Cell is certified outside/empty.
    Empty,
    /// Cell is certified inside/filled.
    Filled,
    /// Cell touches or may cross the boundary.
    Boundary,
    /// Cell could not be classified.
    Unknown,
}

impl SdfVoxelOccupancy {
    /// Convert SDF conservative cell location to voxel occupancy.
    pub const fn from_cell_location(location: SdfCellLocation) -> Self {
        match location {
            SdfCellLocation::ConservativeInside => Self::Filled,
            SdfCellLocation::Boundary => Self::Boundary,
            SdfCellLocation::ConservativeOutside => Self::Empty,
            SdfCellLocation::Unknown => Self::Unknown,
        }
    }
}

/// One exact voxel-cell handoff row.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfVoxelCellHandoff {
    /// Integer cell coordinate in x/y/z order.
    pub index: [u32; 3],
    /// Conservative occupancy label for voxel storage.
    pub occupancy: SdfVoxelOccupancy,
    /// Exact/certified SDF cell classification used to derive occupancy.
    pub classification: SdfCellClassificationReport,
}

/// Coordinate-system family declared for a voxel-grid interchange stream.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfVoxelCoordinateSystem {
    /// Producer did not declare the coordinate-system family.
    Unknown,
    /// Axis-aligned Hyper stack grid coordinates, with integer cell indices.
    HyperGrid,
}

/// Declared row order for a voxel-grid interchange stream.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfVoxelRowOrder {
    /// Producer did not declare row ordering.
    Unknown,
    /// Rows are explicit address/cell pairs and do not rely on implicit order.
    ExplicitAddresses,
    /// Rows are sorted by increasing Morton/Z-order code at the frame depth.
    MortonAscending,
    /// Rows are dense z-major, then y, then x-fast order.
    ZMajorYThenXFast,
}

/// Dependency-light producer manifest for `hypervoxel`-style intake.
///
/// This is the producer-side counterpart to `hypervoxel`'s continuous-field
/// intake manifest. It intentionally contains only stable scalar metadata and
/// provenance, not consumer storage types. Following Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997), the exact
/// predicate evidence and the sampled artifact metadata are kept together so
/// downstream crates do not infer combinatorial meaning from a row vector alone.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SdfHypervoxelInterchangeManifest {
    /// Source continuous field version that produced the rows.
    pub source: Option<SdfVoxelGridSource>,
    /// Declared coordinate-system family.
    pub coordinate_system: SdfVoxelCoordinateSystem,
    /// Declared row ordering.
    pub row_order: SdfVoxelRowOrder,
    /// Declared octree/frame depth, when the grid is octree-addressable.
    pub declared_depth: Option<u8>,
    /// Declared cell dimensions.
    pub declared_dimensions: [u32; 3],
    /// Declared number of rows.
    pub declared_cell_count: usize,
}

/// Validation report for [`SdfHypervoxelInterchangeManifest`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SdfHypervoxelInterchangeReport {
    /// Whether a source/version was declared for freshness checks downstream.
    pub source_declared: bool,
    /// Whether coordinate system was explicitly declared.
    pub coordinate_system_declared: bool,
    /// Whether row ordering was explicitly declared.
    pub row_order_declared: bool,
    /// Whether declared depth matches the exact frame.
    pub depth_matches_frame: bool,
    /// Whether declared dimensions match the exact frame.
    pub dimensions_match_frame: bool,
    /// Whether declared row count matches supplied rows and frame volume.
    pub cell_count_matches: bool,
    /// Whether the frame is compatible with `hypervoxel` octree addressing.
    pub frame_ready: bool,
    /// Whether every exported row has a classified occupancy label.
    pub payload_classified: bool,
    /// Whether this producer manifest is ready for exact downstream intake.
    pub exact_interchange_ready: bool,
}

/// Exact frame readiness facts for a `hypervoxel` handoff.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfHypervoxelFrameReport {
    /// Whether all cell pitches were certified positive.
    pub positive_step: bool,
    /// Whether dimensions are equal on all axes.
    pub cubic_dimensions: bool,
    /// Whether cubic dimensions are a power of two.
    pub power_of_two_dimensions: bool,
    /// Octree depth implied by dimensions when available.
    pub depth: Option<u8>,
    /// Whether this frame is ready for `hypervoxel`'s current octree address model.
    pub hypervoxel_frame_ready: bool,
}

/// Conservative SDF-to-voxel handoff report.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfHypervoxelHandoffReport {
    /// Exact voxel-cell grid.
    pub grid: SdfVoxelCellGrid,
    /// Frame readiness facts.
    pub frame: SdfHypervoxelFrameReport,
    /// Metric claim of the source expression.
    pub metric_status: SdfMetricStatus,
    /// Prepared-source freshness.
    pub freshness: SdfFreshness,
    /// Number of cells visited.
    pub cell_count: usize,
    /// Number of empty cells.
    pub empty_count: usize,
    /// Number of filled cells.
    pub filled_count: usize,
    /// Number of boundary cells.
    pub boundary_count: usize,
    /// Number of unknown cells.
    pub unknown_count: usize,
    /// Per-cell handoff rows in z-major, y, x-fast order.
    pub cells: Vec<SdfVoxelCellHandoff>,
}

impl SdfHypervoxelHandoffReport {
    /// Build a report from exact cell classifications.
    pub fn from_classifications(
        grid: SdfVoxelCellGrid,
        classifications: Vec<SdfCellClassificationReport>,
        metric_status: SdfMetricStatus,
        freshness: SdfFreshness,
    ) -> Self {
        let depth = grid.hypervoxel_depth();
        let positive_step = grid.validate_positive_step().is_ok();
        let [nx, ny, nz] = grid.dimensions;
        let cubic_dimensions = nx == ny && nx == nz;
        let power_of_two_dimensions = cubic_dimensions && nx.is_power_of_two();
        let frame = SdfHypervoxelFrameReport {
            positive_step,
            cubic_dimensions,
            power_of_two_dimensions,
            depth,
            hypervoxel_frame_ready: positive_step && depth.is_some(),
        };

        let mut empty_count = 0_usize;
        let mut filled_count = 0_usize;
        let mut boundary_count = 0_usize;
        let mut unknown_count = 0_usize;
        let mut cells = Vec::with_capacity(classifications.len());
        let mut iter = classifications.into_iter();
        for z in 0..nz {
            for y in 0..ny {
                for x in 0..nx {
                    let Some(classification) = iter.next() else {
                        break;
                    };
                    let occupancy = SdfVoxelOccupancy::from_cell_location(classification.location);
                    match occupancy {
                        SdfVoxelOccupancy::Empty => empty_count += 1,
                        SdfVoxelOccupancy::Filled => filled_count += 1,
                        SdfVoxelOccupancy::Boundary => boundary_count += 1,
                        SdfVoxelOccupancy::Unknown => unknown_count += 1,
                    }
                    cells.push(SdfVoxelCellHandoff {
                        index: [x, y, z],
                        occupancy,
                        classification,
                    });
                }
            }
        }
        Self {
            grid,
            frame,
            metric_status,
            freshness,
            cell_count: cells.len(),
            empty_count,
            filled_count,
            boundary_count,
            unknown_count,
            cells,
        }
    }

    /// Return a legacy conservative cell report for consumers not yet using
    /// the frame-aware handoff.
    pub fn as_voxel_handoff_report(&self) -> SdfVoxelHandoffReport {
        SdfVoxelHandoffReport::from_cells(
            self.cells
                .iter()
                .map(|cell| cell.classification.clone())
                .collect(),
            self.metric_status,
            self.freshness,
        )
    }

    /// Emit dependency-light metadata for a downstream `hypervoxel` intake.
    ///
    /// The row order matches the construction loop in
    /// [`SdfHypervoxelHandoffReport::from_classifications`]: z-major, then y,
    /// then x-fast. Consumers can compare this envelope with their own exact
    /// frame before admitting the row stream into storage.
    pub fn interchange_manifest(&self) -> SdfHypervoxelInterchangeManifest {
        SdfHypervoxelInterchangeManifest {
            source: self.grid.source.clone(),
            coordinate_system: SdfVoxelCoordinateSystem::HyperGrid,
            row_order: SdfVoxelRowOrder::ZMajorYThenXFast,
            declared_depth: self.frame.depth,
            declared_dimensions: self.grid.dimensions,
            declared_cell_count: self.cell_count,
        }
    }

    /// Validate a producer manifest against this exact handoff report.
    pub fn interchange_report(
        &self,
        manifest: &SdfHypervoxelInterchangeManifest,
    ) -> SdfHypervoxelInterchangeReport {
        let source_declared = manifest.source.is_some();
        let coordinate_system_declared = !matches!(
            manifest.coordinate_system,
            SdfVoxelCoordinateSystem::Unknown
        );
        let row_order_declared = !matches!(manifest.row_order, SdfVoxelRowOrder::Unknown);
        let depth_matches_frame = manifest.declared_depth == self.frame.depth;
        let dimensions_match_frame = manifest.declared_dimensions == self.grid.dimensions;
        let cell_count_matches = self.grid.cell_count().ok() == Some(manifest.declared_cell_count)
            && manifest.declared_cell_count == self.cells.len()
            && manifest.declared_cell_count == self.cell_count;
        let frame_ready = self.frame.hypervoxel_frame_ready;
        let payload_classified = self.unknown_count == 0;
        let exact_interchange_ready = self.is_self_consistent()
            && source_declared
            && coordinate_system_declared
            && row_order_declared
            && depth_matches_frame
            && dimensions_match_frame
            && cell_count_matches
            && frame_ready
            && payload_classified;
        SdfHypervoxelInterchangeReport {
            source_declared,
            coordinate_system_declared,
            row_order_declared,
            depth_matches_frame,
            dimensions_match_frame,
            cell_count_matches,
            frame_ready,
            payload_classified,
            exact_interchange_ready,
        }
    }

    /// Returns whether all cells were classified and the frame is octree-ready.
    pub const fn hypervoxel_ready(&self) -> bool {
        self.frame.hypervoxel_frame_ready && self.unknown_count == 0
    }

    /// Validate summary counts and per-cell reports.
    pub fn is_self_consistent(&self) -> bool {
        let mut empty_count = 0_usize;
        let mut filled_count = 0_usize;
        let mut boundary_count = 0_usize;
        let mut unknown_count = 0_usize;
        for cell in &self.cells {
            if cell.occupancy != SdfVoxelOccupancy::from_cell_location(cell.classification.location)
                || !cell.classification.is_self_consistent()
            {
                return false;
            }
            match cell.occupancy {
                SdfVoxelOccupancy::Empty => empty_count += 1,
                SdfVoxelOccupancy::Filled => filled_count += 1,
                SdfVoxelOccupancy::Boundary => boundary_count += 1,
                SdfVoxelOccupancy::Unknown => unknown_count += 1,
            }
        }
        self.grid.cell_count().ok() == Some(self.cell_count)
            && self.cell_count == self.cells.len()
            && self.empty_count == empty_count
            && self.filled_count == filled_count
            && self.boundary_count == boundary_count
            && self.unknown_count == unknown_count
            && self.frame.depth == self.grid.hypervoxel_depth()
    }
}

pub(crate) fn voxel_cell_bounds(grid: &SdfVoxelCellGrid) -> Vec<(Point3, Point3)> {
    let [nx, ny, nz] = grid.dimensions;
    let mut cells = Vec::with_capacity(grid.cell_count().unwrap_or(0));
    for z in 0..nz {
        for y in 0..ny {
            for x in 0..nx {
                cells.push(grid.cell_bounds(x, y, z));
            }
        }
    }
    cells
}

fn validate_positive(value: &Real, axis: usize) -> Result<(), SdfVoxelGridError> {
    match compare_reals(value, &Real::zero()) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => Ok(()),
        PredicateOutcome::Decided { .. } => Err(SdfVoxelGridError::NonPositiveStep { axis }),
        PredicateOutcome::Unknown { .. } => Err(SdfVoxelGridError::UnknownStepSign { axis }),
    }
}
