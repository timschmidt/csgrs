//! Exact closed dyadic partitions and explicit outward grid transport.
//!
//! Bisection retains two closed boxes meeting at the exact midpoint slice;
//! regridding can enlarge a box. Both delegate arithmetic to Hyperinterval and
//! share one caller budget through construction and retained coordinate copies.
//! Neither operation certifies a solver decision or rolls back consumed work.
use crate::{IntervalLatticeError, IntervalVector, array_result};
use hyperinterval::{DyadicBudget, DyadicInterval};

/// Axis-checked exact partitions of a nonempty, nonsingleton selected coordinate.
impl<const N: usize> IntervalVector<DyadicInterval, N> {
    /// Split one coordinate exactly at its midpoint, retaining both closed halves.
    ///
    /// The children union equals the source and they intersect on the midpoint
    /// slice. Other coordinates retain their exact intervals. An empty source or
    /// singleton selected coordinate returns `None`; invalid indices reject even
    /// for empty sources. With sufficient budget, an unrepresentable exact half
    /// returns typed overflow, never a rounded split or a nonprogressing partition.
    /// All arithmetic and retained scalar copies share the caller's budget.
    ///
    /// Checks the axis before emptiness and singleton status; these early outcomes
    /// consume no scalar work. Otherwise constructs the exact half-width r first,
    /// then intersects the selected interval with its translate by -r for the left
    /// child and +r for the right. For endpoints a < b this gives the closed halves
    /// from a to (a+b)/2 and from (a+b)/2 to b, without midpoint enclosure rounding.
    ///
    /// Both selected child lanes are computed before copying other coordinates,
    /// left array first and right array second, each in index order. Selected lanes
    /// are moved once from temporary Options; other lanes use the scalar power-one
    /// copy path. Resource failures can precede representability errors. Failure
    /// returns neither child, leaves the source unchanged and retains prior charges.
    pub fn bisect_axis(
        &self,
        axis: usize,
        budget: &mut DyadicBudget,
    ) -> Result<Option<[Self; 2]>, IntervalLatticeError> {
        if axis >= N {
            return Err(IntervalLatticeError::IndexOutOfBounds {
                index: axis,
                dimension: N,
            });
        }
        let Some(lanes) = self.lanes() else {
            return Ok(None);
        };
        let lane = &lanes[axis];
        if lane.is_singleton() {
            return Ok(None);
        }
        let radius = lane.radius(budget)?;
        let mut left = Some(lane.intersection(&lane.sub(&radius, budget)?, budget)?);
        let mut right = Some(lane.intersection(&lane.add(&radius, budget)?, budget)?);
        let left = array_result(|i| {
            if i == axis {
                Ok(left.take().expect("axis visited once"))
            } else {
                lanes[i].checked_pow(1, budget)
            }
        })?;
        let right = array_result(|i| {
            if i == axis {
                Ok(right.take().expect("axis visited once"))
            } else {
                lanes[i].checked_pow(1, budget)
            }
        })?;
        Ok(Some([Self::new(left)?, Self::new(right)?]))
    }
}

/// Explicit per-coordinate outward projection onto a requested dyadic grid.
impl<const N: usize> IntervalVector<DyadicInterval, N> {
    /// Outward coordinate enclosure on `2^precision`, preserving empty/unit sets.
    /// All rounding and retained scalar copies share the caller's budget.
    ///
    /// Precision specifies grid spacing, not a bound on total work or output width.
    /// Finer grids preserve each scalar set exactly; coarser grids round lower
    /// endpoints down and upper endpoints up. Native normalization can produce a
    /// stored exponent different from the requested precision. This returned box
    /// does not retain a separate definition of the pre-rounding source.
    ///
    /// Visits coordinates in index order and returns the first scalar error, with
    /// no partial box or budget refund. Even an exact finer-grid copy can exhaust
    /// resources. Empty and zero-dimensional unit inputs invoke no scalar regridding.
    pub fn regrid(
        &self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalLatticeError> {
        match self.lanes() {
            None => Self::empty(),
            Some(lanes) => Self::new(array_result(|i| lanes[i].regrid(precision, budget))?),
        }
    }
}
