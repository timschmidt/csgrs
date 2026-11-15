//! Source-bound native voxel ranges, using Hypervoxel's exact frame authority.
use std::fmt;

use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hypersolve::expression_program::{ExpressionBudget, ExpressionError};
use hypervoxel::{
    ContinuousFieldVoxelBatch, ContinuousFieldVoxelCell, GridFrame, HypervoxelError,
    MaterialRegionId, VoxelAddress, continuous_field_address,
};

use super::voxel_cell_from_sdf;
use crate::{SdfCellEnclosure, SdfProgram, SdfProgramError, SdfVoxelOccupancy};

/// Native range precision and a separate limit on retained cell records.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SdfVoxelEnclosureOptions {
    /// Grid spacing exponent for coordinate enclosures and scalar evaluation.
    /// This is not an output-width or spatial-resolution promise.
    pub precision: i32,
    /// Maximum number of finest-depth cells in the complete frame.
    pub max_cells: usize,
}
impl Default for SdfVoxelEnclosureOptions {
    fn default() -> Self {
        Self {
            precision: -48,
            max_cells: 4096,
        }
    }
}

/// Original native failure or explicit retained-cell capacity exhaustion.
#[derive(Clone, Debug)]
pub enum SdfVoxelEnclosureError {
    /// Source range, graph work/size, domain or cancellation diagnostic.
    Program(SdfProgramError),
    /// A frame endpoint could not be enclosed within the arithmetic contract.
    Coordinate {
        /// Exact cell whose coordinate conversion failed.
        address: VoxelAddress,
        /// Coordinate in x/y/z order.
        axis: usize,
        /// Original native interval error.
        source: IntervalError,
    },
    /// Original grid-address or cell-bounds error.
    Grid(HypervoxelError),
    /// The retained or exported record buffer could not be reserved.
    /// This preserves allocator/capacity failure; it is not an interval result.
    Storage(std::collections::TryReserveError),
    /// Complete traversal would exceed the declared cell-record capacity.
    Capacity {
        /// Configured maximum cell records.
        limit: usize,
        /// Exact number of required records, before any traversal allocation.
        required: u64,
    },
}
impl fmt::Display for SdfVoxelEnclosureError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SDF voxel enclosure: {self:?}")
    }
}
impl std::error::Error for SdfVoxelEnclosureError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Program(e) => Some(e),
            Self::Coordinate { source, .. } => Some(source),
            Self::Grid(e) => Some(e),
            Self::Storage(e) => Some(e),
            Self::Capacity { .. } => None,
        }
    }
}
impl From<ExpressionError> for SdfVoxelEnclosureError {
    fn from(e: ExpressionError) -> Self {
        Self::Program(SdfProgramError::Expression(e))
    }
}
impl From<SdfProgramError> for SdfVoxelEnclosureError {
    fn from(e: SdfProgramError) -> Self {
        Self::Program(e)
    }
}
impl From<HypervoxelError> for SdfVoxelEnclosureError {
    fn from(e: HypervoxelError) -> Self {
        Self::Grid(e)
    }
}

/// An exact address associated with a retained whole-cell range proof.
#[derive(Clone, Debug)]
pub struct SdfVoxelEnclosureCell {
    address: VoxelAddress,
    enclosure: SdfCellEnclosure,
}
impl SdfVoxelEnclosureCell {
    /// Finest-depth address in the batch's immutable frame.
    pub fn address(&self) -> VoxelAddress {
        self.address
    }
    /// Original native proof. Its dyadic domain encloses the exact frame cell;
    /// it need not equal that cell for nondyadic coordinates.
    pub fn enclosure(&self) -> &SdfCellEnclosure {
        &self.enclosure
    }
}

/// Complete frame coverage retaining every native source-bound range.
///
/// Records are immutable and ordered by z, then y, then x (x varies fastest).
/// A zero-containing range is Unknown, not a certified boundary crossing.
/// Failed attempts never publish a partial instance. Cloning this report is
/// outside the construction budgets, as is scalar destruction.
#[derive(Clone, Debug)]
pub struct SdfVoxelEnclosureBatch {
    frame: GridFrame,
    options: SdfVoxelEnclosureOptions,
    cells: Vec<SdfVoxelEnclosureCell>,
}
impl SdfVoxelEnclosureBatch {
    /// Authoritative frame, including exact pitch/origin and declared units.
    pub fn frame(&self) -> &GridFrame {
        &self.frame
    }
    /// The precision and capacity used for this complete result.
    pub fn options(&self) -> SdfVoxelEnclosureOptions {
        self.options
    }
    /// Every finest-depth cell with its retained native proof.
    pub fn cells(&self) -> &[SdfVoxelEnclosureCell] {
        &self.cells
    }
    /// Shared original implicit field and validated program association.
    pub fn source(&self) -> &SdfProgram {
        self.cells[0].enclosure.source()
    }

    /// Export conservative occupancy using the same immutable frame.
    ///
    /// The native report retains source evidence; the returned public intake
    /// rows are ordinary storage data, not a replacement for that proof.
    /// Unknown remains Unknown and Hypervoxel's exact materializer rejects it.
    /// Work and entry checks include the retained and exported records, with
    /// cancellation checked again before publication. Scalar frame cloning and
    /// heap bytes retain ExpressionBudget's exclusions.
    pub fn continuous_field_batch(
        &self,
        material: MaterialRegionId,
        graph: &mut ExpressionBudget,
    ) -> Result<ContinuousFieldVoxelBatch, SdfVoxelEnclosureError> {
        graph.charge(0)?;
        let definition = self.source().program().definition();
        graph.check_entries(&[
            definition.variables.len(),
            definition.nodes.len(),
            definition.outputs.len(),
            self.cells.len(),
            self.cells.len(),
            6,
        ])?;
        graph.charge(self.cells.len())?;
        let mut cells = Vec::new();
        cells
            .try_reserve_exact(self.cells.len())
            .map_err(SdfVoxelEnclosureError::Storage)?;
        for row in &self.cells {
            graph.charge(1)?;
            let location = row.enclosure.location().expect("nonempty frame cell");
            cells.push(ContinuousFieldVoxelCell::new(
                row.address,
                voxel_cell_from_sdf(SdfVoxelOccupancy::from_cell_location(location), material),
            ));
        }
        graph.charge(1)?;
        let result = ContinuousFieldVoxelBatch {
            frame: self.frame.clone(),
            cells,
        };
        graph.charge(0)?;
        Ok(result)
    }
}

impl SdfProgram {
    /// Certify every finest-depth cell through native whole-cell enclosures.
    ///
    /// Hypervoxel constructs exact cell bounds. Each endpoint is outwardly
    /// enclosed via Hyperinterval before the original field is evaluated.
    /// This transports evidence to the same frame without fitting or sampling.
    /// Cell capacity is checked before traversal/storage allocation. Graph entry
    /// checks count the source definition, two evaluator node arrays, all retained
    /// cell records and 24 working records; these are not heap-byte bounds.
    /// Graph work and cancellation and one shared DyadicBudget span all cells.
    ///
    /// Exact frame scalar arithmetic, Hyperreal refinement, scalar cloning and
    /// destruction keep the native APIs' accounting exclusions. Cancellation
    /// cannot interrupt them. A failure aborts the entire batch; no partial or
    /// cached failure becomes permanent evidence. Unsupported source forms must
    /// already have been rejected by SdfProgram::compile.
    pub fn classify_voxel_grid(
        &self,
        frame: GridFrame,
        options: SdfVoxelEnclosureOptions,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<SdfVoxelEnclosureBatch, SdfVoxelEnclosureError> {
        graph.charge(0)?;
        let side = frame.cells_per_axis();
        // GridFrame's maximum depth is 21: its cube fits u64 on every target.
        let required = side * side * side;
        if required > options.max_cells as u64 {
            return Err(SdfVoxelEnclosureError::Capacity {
                limit: options.max_cells,
                required,
            });
        }
        let count = usize::try_from(required).map_err(|_| ExpressionError::SizeExhausted)?;
        let definition = self.program().definition();
        graph.check_entries(&[
            definition.variables.len(),
            definition.nodes.len(),
            definition.outputs.len(),
            definition.nodes.len(),
            definition.nodes.len(),
            count,
            24,
        ])?;
        graph.charge(count)?;
        let mut cells = Vec::new();
        cells
            .try_reserve_exact(count)
            .map_err(SdfVoxelEnclosureError::Storage)?;
        for z in 0..side {
            for y in 0..side {
                for x in 0..side {
                    graph.charge(1)?;
                    let address = continuous_field_address(&frame, [x, y, z])?;
                    let bounds = address.bounds(&frame)?;
                    let mut cell = core::array::from_fn(|_| DyadicInterval::empty());
                    for (axis, lane) in cell.iter_mut().enumerate() {
                        let error = |source| SdfVoxelEnclosureError::Coordinate {
                            address,
                            axis,
                            source,
                        };
                        graph.charge(1)?;
                        let lower = DyadicInterval::from_real(
                            &bounds.min[axis],
                            options.precision,
                            arithmetic,
                        )
                        .map_err(error)?;
                        graph.charge(1)?;
                        let upper = DyadicInterval::from_real(
                            &bounds.max[axis],
                            options.precision,
                            arithmetic,
                        )
                        .map_err(error)?;
                        graph.charge(1)?;
                        *lane = lower.hull(&upper, arithmetic).map_err(error)?;
                    }
                    let enclosure = self.image(cell, options.precision, graph, arithmetic)?;
                    graph.charge(1)?;
                    cells.push(SdfVoxelEnclosureCell { address, enclosure });
                }
            }
        }
        graph.charge(1)?;
        let result = SdfVoxelEnclosureBatch {
            frame,
            options,
            cells,
        };
        graph.charge(0)?;
        Ok(result)
    }
}
