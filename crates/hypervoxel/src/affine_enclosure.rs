//! Conservative transformed boxes retaining their original exact definition.
use hyperlattice::interval::hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice::{BoxN, IntervalMatrix, IntervalVector};

use crate::{CellBounds, ExactAffineTransform, HypervoxelResult};

/// Certified Cartesian enclosure of an exact affine image of a source box.
///
/// Source endpoints denote their coordinatewise closed hull, including reversed
/// or singleton coordinates. The enclosure can contain points outside the exact
/// correlated image. It does not establish occupancy, surface or topology.
/// Original endpoints and transform remain immutable and authoritative.
#[derive(Clone, Debug)]
pub struct AffineBoundsEnclosure {
    transform: ExactAffineTransform,
    source: CellBounds,
    precision: i32,
    bounds: BoxN<DyadicInterval, 3>,
}
impl AffineBoundsEnclosure {
    /// Original exact map, including any shared scalar expressions.
    pub fn transform(&self) -> &ExactAffineTransform {
        &self.transform
    }
    /// Original exact endpoint pairs, without rounding or reordered storage.
    pub fn source(&self) -> &CellBounds {
        &self.source
    }
    /// Requested scalar enclosure spacing exponent; not an output-width promise.
    pub fn precision(&self) -> i32 {
        self.precision
    }
    /// A containing Cartesian box, for conservative downstream queries.
    pub fn bounds(&self) -> &BoxN<DyadicInterval, 3> {
        &self.bounds
    }
}

impl ExactAffineTransform {
    /// Enclose the affine image without requiring exact term ordering.
    ///
    /// Outward scalar conversions and Cartesian matrix/vector operations share
    /// one dyadic budget. All dimensions and loops are fixed at three. Native
    /// scalar, size and work failures abort the report and retain their causes.
    /// A later independent budget can retry. Hyperreal refinement, original
    /// exact-value cloning/destruction and heap bytes retain the native budget's
    /// exclusions; this is not a total-work or cancellation guarantee.
    ///
    /// This preserves the source rather than replacing it by rounded geometry.
    /// Use `map_bounds` when exact corner extrema are required. Reversed source
    /// endpoints denote their closed hull here, as in that exact construction.
    /// Singular maps and singleton source boxes are valid.
    pub fn enclose_bounds(
        &self,
        source: &CellBounds,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> HypervoxelResult<AffineBoundsEnclosure> {
        let mut lanes: [DyadicInterval; 3] = core::array::from_fn(|_| DyadicInterval::empty());
        for (axis, lane) in lanes.iter_mut().enumerate() {
            let a = DyadicInterval::from_real(&source.min[axis], precision, budget)?;
            let b = DyadicInterval::from_real(&source.max[axis], precision, budget)?;
            *lane = a.hull(&b, budget)?;
        }
        let source_box = IntervalVector::new(lanes)?;
        let mut rows: [[DyadicInterval; 3]; 3] =
            core::array::from_fn(|_| core::array::from_fn(|_| DyadicInterval::empty()));
        for (row, values) in rows.iter_mut().enumerate() {
            for (column, value) in values.iter_mut().enumerate() {
                *value = DyadicInterval::from_real(&self.matrix[row][column], precision, budget)?;
            }
        }
        let matrix = IntervalMatrix::new(rows)?;
        let mut translation = core::array::from_fn(|_| DyadicInterval::empty());
        for (axis, value) in translation.iter_mut().enumerate() {
            *value = DyadicInterval::from_real(&self.translation[axis], precision, budget)?;
        }
        let bounds = matrix
            .mul_vector(&source_box, budget)?
            .add(&IntervalVector::new(translation)?, budget)?;
        Ok(AffineBoundsEnclosure {
            transform: self.clone(),
            source: source.clone(),
            precision,
            bounds,
        })
    }
}
