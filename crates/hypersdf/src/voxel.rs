//! Exact voxel-grid classification for `hypervoxel` consumers.

use core::cmp::Ordering;

use hyperlimit::{Point3, PredicateOutcome, compare_reals};
use hyperreal::Real;

use crate::status::{SdfCellClassificationReport, SdfCellLocation};

/// Length unit declared for a voxel grid.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfVoxelLengthUnit {
    Unitless,
    Meter,
    Millimeter,
    Micrometer,
    Nanometer,
}

/// Exact axis-aligned voxel-cell grid requested from an SDF.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfVoxelCellGrid {
    pub origin: Point3,
    pub step: Point3,
    pub dimensions: [u32; 3],
    pub units: SdfVoxelLengthUnit,
}

impl SdfVoxelCellGrid {
    pub const fn new(origin: Point3, step: Point3, dimensions: [u32; 3]) -> Self {
        Self {
            origin,
            step,
            dimensions,
            units: SdfVoxelLengthUnit::Unitless,
        }
    }

    pub const fn with_units(mut self, units: SdfVoxelLengthUnit) -> Self {
        self.units = units;
        self
    }

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

    pub fn validate_positive_step(&self) -> Result<(), SdfVoxelGridError> {
        validate_positive(&self.step.x, 0)?;
        validate_positive(&self.step.y, 1)?;
        validate_positive(&self.step.z, 2)
    }

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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfVoxelGridError {
    EmptyDimension,
    TooManyCells,
    NonPositiveStep { axis: usize },
    UnknownStepSign { axis: usize },
}

/// Conservative occupancy label exported toward voxel storage.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfVoxelOccupancy {
    Empty,
    Filled,
    Boundary,
    Unknown,
}

impl SdfVoxelOccupancy {
    pub const fn from_cell_location(location: SdfCellLocation) -> Self {
        match location {
            SdfCellLocation::ConservativeInside => Self::Filled,
            SdfCellLocation::Boundary => Self::Boundary,
            SdfCellLocation::ConservativeOutside => Self::Empty,
            SdfCellLocation::Unknown => Self::Unknown,
        }
    }
}

/// One classified voxel cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SdfVoxelCell {
    pub index: [u32; 3],
    pub occupancy: SdfVoxelOccupancy,
}

/// Classified cells for one exact grid.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfVoxelBatch {
    pub grid: SdfVoxelCellGrid,
    pub cells: Vec<SdfVoxelCell>,
}

impl SdfVoxelBatch {
    pub(crate) fn from_classifications(
        grid: SdfVoxelCellGrid,
        classifications: Vec<SdfCellClassificationReport>,
    ) -> Self {
        let [nx, ny, nz] = grid.dimensions;
        let mut cells = Vec::with_capacity(classifications.len());
        let mut iter = classifications.into_iter();
        for z in 0..nz {
            for y in 0..ny {
                for x in 0..nx {
                    let Some(classification) = iter.next() else {
                        return Self { grid, cells };
                    };
                    cells.push(SdfVoxelCell {
                        index: [x, y, z],
                        occupancy: SdfVoxelOccupancy::from_cell_location(classification.location),
                    });
                }
            }
        }
        Self { grid, cells }
    }

    pub fn is_complete(&self) -> bool {
        self.grid.cell_count().ok() == Some(self.cells.len())
    }

    pub fn has_unknown(&self) -> bool {
        self.cells
            .iter()
            .any(|cell| cell.occupancy == SdfVoxelOccupancy::Unknown)
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
    match compare_reals(value, &Real::from(0)) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => Ok(()),
        PredicateOutcome::Decided { .. } => Err(SdfVoxelGridError::NonPositiveStep { axis }),
        PredicateOutcome::Unknown { .. } => Err(SdfVoxelGridError::UnknownStepSign { axis }),
    }
}
