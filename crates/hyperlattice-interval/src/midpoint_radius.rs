use crate::{
    IntervalArithmetic, IntervalLatticeError, IntervalMatrix, IntervalVector, array_result,
};

/// Exact midpoint/radius definition of a Cartesian interval vector.
///
/// Each source lane `[a,b]` defines center `(a+b)/2` and radius `(b-a)/2`.
/// The checked source is retained once, so half-steps need not fit the endpoint
/// family and arbitrary-precision data is not duplicated into two coordinates.
/// Reconstruction preserves the exact original set. Numerical projections are
/// explicitly named enclosures; they never replace this retained definition.
/// An empty source remains empty, distinct from the zero-dimensional unit.
/// Equality compares represented source sets; serde validates those source sets.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(transparent)]
pub struct MidpointRadiusVector<I: IntervalArithmetic, const N: usize> {
    source: IntervalVector<I, N>,
}

impl<I: IntervalArithmetic, const N: usize> IntervalVector<I, N> {
    /// Retains an exact midpoint/radius decomposition without computing or
    /// rounding coordinates. Ownership transfers without scalar copies or work.
    ///
    /// ```
    /// # #[cfg(feature = "fixed")] {
    /// use hyperlattice_interval::{IntervalVector, hyperinterval::FixedInterval};
    /// type I = FixedInterval<i64, 63>;
    /// let source = IntervalVector::new([I::new(0, 1).unwrap()]).unwrap();
    /// let form = source.midpoint_radius();
    /// // Both exact values are 2^-64, between adjacent endpoint lattice points.
    /// assert_eq!(form.midpoint_enclosure(&mut ()).unwrap(), source);
    /// assert_eq!(form.radius_enclosure(&mut ()).unwrap(), source);
    /// assert_eq!(form.into_intervals(), source);
    /// # }
    /// ```
    pub fn midpoint_radius(self) -> MidpointRadiusVector<I, N> {
        MidpointRadiusVector { source: self }
    }
}

impl<I: IntervalArithmetic, const N: usize> MidpointRadiusVector<I, N> {
    /// Exact original interval form, borrowed without copying or projection.
    pub const fn intervals(&self) -> &IntervalVector<I, N> {
        &self.source
    }

    /// Reconstructs the exact original intervals, transferring owned data.
    /// This succeeds even when a numerical center/radius projection overflows.
    pub fn into_intervals(self) -> IntervalVector<I, N> {
        self.source
    }

    /// Whether the represented source is empty.
    pub const fn is_empty(&self) -> bool {
        self.source.is_empty()
    }

    /// Outward enclosure of the exact center vector. The returned lanes can
    /// be nonsingletons; they are not an implicitly chosen point vector.
    /// All scalar work shares the caller context, with no partial result.
    pub fn midpoint_enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        self.project(context, I::midpoint_enclosure)
    }

    /// Outward enclosure of the exact radius vector. Fixed endpoint storage
    /// can overflow. This is not a radius about a separately rounded center;
    /// combine enclosures using interval arithmetic or reconstruct exactly.
    pub fn radius_enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        self.project(context, I::radius_enclosure)
    }

    fn project(
        &self,
        context: &mut I::Context,
        projection: impl Fn(&I, &mut I::Context) -> Result<I, hyperinterval::IntervalError>,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        match self.source.lanes() {
            None => IntervalVector::empty(),
            Some(lanes) => IntervalVector::new(array_result(|i| projection(&lanes[i], context))?),
        }
    }
}

/// Exact midpoint/radius definition of a Cartesian matrix set.
///
/// Uses the same retained-bound semantics as [`MidpointRadiusVector`], in row
/// order. Zero-dimensional unit matrices remain distinct from empty matrices.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(transparent)]
pub struct MidpointRadiusMatrix<I: IntervalArithmetic, const R: usize, const C: usize> {
    source: IntervalMatrix<I, R, C>,
}

impl<I: IntervalArithmetic, const R: usize, const C: usize> IntervalMatrix<I, R, C> {
    /// Retains an exact matrix midpoint/radius decomposition by ownership transfer.
    /// No point is rounded, selected or materialized during construction.
    pub fn midpoint_radius(self) -> MidpointRadiusMatrix<I, R, C> {
        MidpointRadiusMatrix { source: self }
    }
}

impl<I: IntervalArithmetic, const R: usize, const C: usize> MidpointRadiusMatrix<I, R, C> {
    /// Exact original interval matrix, borrowed without copying or projection.
    pub const fn intervals(&self) -> &IntervalMatrix<I, R, C> {
        &self.source
    }

    /// Reconstructs the exact original matrix set, transferring owned data.
    pub fn into_intervals(self) -> IntervalMatrix<I, R, C> {
        self.source
    }

    /// Whether the represented matrix set is empty.
    pub const fn is_empty(&self) -> bool {
        self.source.is_empty()
    }

    /// Outward enclosure of the exact midpoint matrix. Entries can remain
    /// nonsingletons; all work shares the caller context in row-major order.
    pub fn midpoint_enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalMatrix<I, R, C>, IntervalLatticeError> {
        self.project(context, I::midpoint_enclosure)
    }

    /// Outward enclosure of the exact radius matrix. No partial matrix escapes
    /// on failure. The retained exact definition remains unchanged.
    pub fn radius_enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalMatrix<I, R, C>, IntervalLatticeError> {
        self.project(context, I::radius_enclosure)
    }

    fn project(
        &self,
        context: &mut I::Context,
        projection: impl Fn(&I, &mut I::Context) -> Result<I, hyperinterval::IntervalError>,
    ) -> Result<IntervalMatrix<I, R, C>, IntervalLatticeError> {
        match self.source.rows() {
            None => IntervalMatrix::empty(),
            Some(rows) => IntervalMatrix::new(array_result(|r| {
                array_result(|c| projection(&rows[r][c], context))
            })?),
        }
    }
}

#[cfg(feature = "serde")]
mod serialization {
    use super::*;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    impl<I: IntervalArithmetic + Serialize, const N: usize> Serialize for MidpointRadiusVector<I, N> {
        fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            self.source.serialize(serializer)
        }
    }

    impl<'de, I: IntervalArithmetic + Deserialize<'de>, const N: usize> Deserialize<'de>
        for MidpointRadiusVector<I, N>
    {
        fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            Ok(IntervalVector::deserialize(deserializer)?.midpoint_radius())
        }
    }

    impl<I: IntervalArithmetic + Serialize, const R: usize, const C: usize> Serialize
        for MidpointRadiusMatrix<I, R, C>
    {
        fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            self.source.serialize(serializer)
        }
    }

    impl<'de, I: IntervalArithmetic + Deserialize<'de>, const R: usize, const C: usize>
        Deserialize<'de> for MidpointRadiusMatrix<I, R, C>
    {
        fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            Ok(IntervalMatrix::deserialize(deserializer)?.midpoint_radius())
        }
    }
}
