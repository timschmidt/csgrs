//! Exact midpoint/radius definitions retained through their original Cartesian bounds.
//!
//! A source interval defines two exact halves even when neither half is a point in
//! the chosen scalar family. These forms store that source once and materialize
//! outward enclosures only on request. Reconstruction moves the source back out;
//! it does not recombine separately rounded projections. Earlier correlations lost
//! when forming the source box are not recovered by this representation.
//!
//! Scalar projection semantics and failures belong to Hyperinterval. Queries share
//! a caller context, preserve source data on failure and return no partial output.
//! Clone and serialization have no arithmetic context and are caller-owned work.

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
/// Derived equality delegates to the retained interval carrier; serde validates
/// those source intervals rather than trusting serialized center/radius claims.
/// This is a decomposition of bounds, distinct from independently authored center
/// and radius components in [`crate::CenteredVector`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(transparent)]
pub struct MidpointRadiusVector<I: IntervalArithmetic, const N: usize> {
    /// Sole authoritative storage, preserving canonical empty/unit states and exact endpoints.
    /// There is no separately materialized center, radius or cached projection.
    source: IntervalVector<I, N>,
}

/// Ownership-only conversion from a checked Cartesian vector to its retained halves.
impl<I: IntervalArithmetic, const N: usize> IntervalVector<I, N> {
    /// Retains an exact midpoint/radius decomposition without computing or
    /// rounding coordinates. Ownership transfers without scalar copies or work.
    /// Empty and unit products keep their identity; no arithmetic budget is required.
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

/// Source access, exact reconstruction and explicit scalar-family projections.
impl<I: IntervalArithmetic, const N: usize> MidpointRadiusVector<I, N> {
    /// Exact original interval form, borrowed without copying or projection.
    /// The immutable borrow cannot replace source endpoints with rounded halves.
    pub const fn intervals(&self) -> &IntervalVector<I, N> {
        &self.source
    }

    /// Reconstructs the exact original intervals, transferring owned data.
    /// This succeeds even when a numerical center/radius projection overflows.
    /// No arithmetic, validation replay or caller-context reservation is needed.
    pub fn into_intervals(self) -> IntervalVector<I, N> {
        self.source
    }

    /// Whether the represented source is empty.
    /// Reads the source carrier tag; a zero-dimensional unit is nonempty.
    pub const fn is_empty(&self) -> bool {
        self.source.is_empty()
    }

    /// Outward enclosure of the exact center vector. The returned lanes can
    /// be nonsingletons; they are not an implicitly chosen point vector.
    /// All scalar work shares the caller context, with no partial result.
    /// Lanes run in increasing index order and the first scalar error propagates.
    /// Empty/unit sources invoke no scalar projection, and prior charges are not refunded.
    pub fn midpoint_enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        self.project(context, I::midpoint_enclosure)
    }

    /// Outward enclosure of the exact radius vector. Fixed endpoint storage
    /// can overflow. This is not a radius about a separately rounded center;
    /// combine enclosures using interval arithmetic or reconstruct exactly.
    /// Projects in index order through the shared context. Failure leaves the exact
    /// source available for reconstruction or retry and does not refund prior work.
    pub fn radius_enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        self.project(context, I::radius_enclosure)
    }

    /// Apply a scalar center/radius projection to retained lanes in index order.
    ///
    /// Empty input constructs a validated empty vector without calling projection.
    /// Otherwise array_result stops at the first error and discards initialized
    /// results; successful lanes are then passed through the vector constructor.
    /// Unit input calls no projection. Borrowed source data is never replaced, and
    /// callbacks share context without rollback. No projection result is cached.
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
/// Exactly one source matrix is stored, with no duplicate center/radius matrices.
/// It retains Cartesian bounds, not independently authored components of a
/// [`crate::CenteredMatrix`]. Projection failures do not prevent exact reconstruction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(transparent)]
pub struct MidpointRadiusMatrix<I: IntervalArithmetic, const R: usize, const C: usize> {
    /// Sole authoritative row-major source, including canonical empty and unit products.
    /// Projection queries borrow it without replacing entries or retaining caches.
    source: IntervalMatrix<I, R, C>,
}

/// Ownership-only conversion from a checked interval matrix to its retained halves.
impl<I: IntervalArithmetic, const R: usize, const C: usize> IntervalMatrix<I, R, C> {
    /// Retains an exact matrix midpoint/radius decomposition by ownership transfer.
    /// No point is rounded, selected or materialized during construction.
    /// Empty/unit identity and shape are preserved without scalar copies or a context.
    pub fn midpoint_radius(self) -> MidpointRadiusMatrix<I, R, C> {
        MidpointRadiusMatrix { source: self }
    }
}

/// Immutable matrix source access, reconstruction and ordered enclosure projections.
impl<I: IntervalArithmetic, const R: usize, const C: usize> MidpointRadiusMatrix<I, R, C> {
    /// Exact original interval matrix, borrowed without copying or projection.
    /// The caller can inspect the retained row-major data without computing either half.
    pub const fn intervals(&self) -> &IntervalMatrix<I, R, C> {
        &self.source
    }

    /// Reconstructs the exact original matrix set, transferring owned data.
    /// No projection or context is required, even after an earlier projection failure.
    pub fn into_intervals(self) -> IntervalMatrix<I, R, C> {
        self.source
    }

    /// Whether the represented matrix set is empty.
    /// Reads the source tag; a unit with either dimension zero is nonempty.
    pub const fn is_empty(&self) -> bool {
        self.source.is_empty()
    }

    /// Outward enclosure of the exact midpoint matrix. Entries can remain
    /// nonsingletons; all work shares the caller context in row-major order.
    /// Empty/unit sources invoke no scalar projection. The first error aborts without
    /// a partial matrix; consumed work is not refunded and the source remains intact.
    pub fn midpoint_enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalMatrix<I, R, C>, IntervalLatticeError> {
        self.project(context, I::midpoint_enclosure)
    }

    /// Outward enclosure of the exact radius matrix. No partial matrix escapes
    /// on failure. The retained exact definition remains unchanged.
    /// Entries are half-widths about their exact source midpoints, not about separately
    /// rounded center entries. All calls share context in row-major order, without rollback.
    pub fn radius_enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalMatrix<I, R, C>, IntervalLatticeError> {
        self.project(context, I::radius_enclosure)
    }

    /// Project retained entries row-major using one shared arithmetic context.
    ///
    /// Empty input skips projection and constructs validated empty storage. For a
    /// nonempty matrix, nested array_result calls discard partial rows on the first
    /// scalar error; only complete success reaches the matrix constructor. Either
    /// zero dimension means no callback calls. Source data stays authoritative and
    /// prior context charges survive failure; this helper stores no projection cache.
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
/// Encode retained source carriers and rebuild checked midpoint/radius definitions.
///
/// No projected center/radius values or validity flags are serialized. Source decoding
/// checks scalar values and shape before this ownership-only wrapper is constructed.
/// Format errors use the serializer/deserializer's own error type; no arithmetic
/// context accounts for serialization, parsing, allocation or caller-owned input.
mod serialization {
    use super::*;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    /// Use exactly the original vector carrier's serialization, without a wrapper field.
    impl<I: IntervalArithmetic + Serialize, const N: usize> Serialize for MidpointRadiusVector<I, N> {
        /// Borrow and serialize source bounds; no half is computed and format errors propagate.
        fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            self.source.serialize(serializer)
        }
    }

    /// Validate a decoded vector through its native constructors, then retain it once.
    impl<'de, I: IntervalArithmetic + Deserialize<'de>, const N: usize> Deserialize<'de>
        for MidpointRadiusVector<I, N>
    {
        /// Propagate source decode/validation errors before the infallible ownership conversion.
        fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            Ok(IntervalVector::deserialize(deserializer)?.midpoint_radius())
        }
    }

    /// Reuse the original row-major matrix serialization without materializing either half.
    impl<I: IntervalArithmetic + Serialize, const R: usize, const C: usize> Serialize
        for MidpointRadiusMatrix<I, R, C>
    {
        /// Serialize the borrowed source carrier, preserving its empty/unit representation.
        fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            self.source.serialize(serializer)
        }
    }

    /// Decode and validate matrix dimensions and scalars before retaining their definition.
    impl<'de, I: IntervalArithmetic + Deserialize<'de>, const R: usize, const C: usize>
        Deserialize<'de> for MidpointRadiusMatrix<I, R, C>
    {
        /// Delegate all validation to matrix replay, then move the checked source into this form.
        fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            Ok(IntervalMatrix::deserialize(deserializer)?.midpoint_radius())
        }
    }
}
