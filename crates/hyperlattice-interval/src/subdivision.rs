//! Exact closed dyadic Cartesian partitions.
use crate::{IntervalLatticeError, IntervalVector, array_result};
use hyperinterval::{DyadicBudget, DyadicInterval};

impl<const N: usize> IntervalVector<DyadicInterval, N> {
    /// Split one coordinate exactly at its midpoint, retaining both closed halves.
    ///
    /// The children union equals the source and they intersect on the midpoint
    /// facet. Other coordinates retain their exact intervals. An empty source or
    /// singleton selected coordinate returns `None`; invalid indices reject even
    /// for empty sources. An unrepresentable exact half returns typed overflow,
    /// never a rounded split or a nonprogressing successful partition.
    /// All arithmetic and retained scalar copies share the caller's budget.
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

impl<const N: usize> IntervalVector<DyadicInterval, N> {
    /// Outward coordinate enclosure on `2^precision`, preserving empty/unit sets.
    /// All rounding and retained scalar copies share the caller's budget.
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
