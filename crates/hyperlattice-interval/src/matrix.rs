//! Cartesian matrix sets, projections and ordered interval linear algebra.
//!
//! Dimensions are part of the type; rows contain independent scalar intervals.
//! Scalar backends own rounding and failure. Products materialize Cartesian output
//! enclosures, while retained correlated affine definitions live in the affine module.

use crate::{IntervalArithmetic, IntervalLatticeError, IntervalVector, array_result};

/// Cartesian set of row-major `R` by `C` matrices, or the canonical empty set.
/// All entries vary independently. Empty dimensions have a singleton unit
/// product unless the explicit empty constructor is used.
///
/// Public access is immutable. Constructors validate the scalar family even when
/// R or C is zero; native deserialization uses those constructors. Derived equality
/// compares stored backend values, not the definitions of prior correlated images.
/// Reusing a matrix as both operands still treats its entries independently.
///
/// Storage is inline in R*C. Scalar allocation/work depends on the backend; dyadic
/// calls share one context across nested loops. Explicit Clone is outside that
/// accounting. Fallible operations retain no partial result, leave borrowed inputs
/// available for retry and do not refund work already charged to the context.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IntervalMatrix<I: IntervalArithmetic, const R: usize, const C: usize> {
    /// None is canonical empty; Some holds R rows of C nonempty scalar intervals.
    /// A zero-sized Some is the unit product, not empty. Internal users must preserve
    /// this constructor invariant; public access supplies only immutable borrows.
    pub(crate) rows: Option<[[I; C]; R]>,
}

/// Construction, set operations, rectangular products and accounted projections.
impl<I: IntervalArithmetic, const R: usize, const C: usize> IntervalMatrix<I, R, C> {
    /// Constructs from row-major scalar intervals; singleton entries lift a point
    /// matrix. Any empty entry canonicalizes the whole matrix set to empty.
    /// Family validation precedes row-major inspection, including for zero dimensions.
    /// Ownership is moved without scalar copies, rounding or context reservations.
    pub fn new(rows: [[I; C]; R]) -> Result<Self, IntervalLatticeError> {
        I::validate_family()?;
        Ok(Self {
            rows: if rows.iter().flatten().any(I::is_empty) {
                None
            } else {
                Some(rows)
            },
        })
    }

    /// Constructs the empty set after validating the scalar family.
    /// No row or entry arithmetic is performed, regardless of the declared dimensions.
    pub fn empty() -> Result<Self, IntervalLatticeError> {
        I::validate_family()?;
        Ok(Self { rows: None })
    }

    /// Whether the matrix set is empty.
    /// Reads the canonical storage tag without scalar predicates or context work.
    pub const fn is_empty(&self) -> bool {
        self.rows.is_none()
    }

    /// Whether exactly one matrix is represented, including empty dimensions.
    /// Empty is false; present entries are inspected row-major until a nonsingleton.
    pub fn is_singleton(&self) -> bool {
        self.rows
            .as_ref()
            .is_some_and(|rows| rows.iter().flatten().all(I::is_singleton))
    }

    /// Borrowed row-major entries, or `None` for empty. No scalar copies occur.
    /// Some preserves the array shape even when one dimension is zero.
    pub const fn rows(&self) -> Option<&[[I; C]; R]> {
        self.rows.as_ref()
    }

    /// Transfers owned entries without copying scalar data.
    /// Consumes the carrier, retaining the distinction between None and a zero-sized Some.
    pub fn into_rows(self) -> Option<[[I; C]; R]> {
        self.rows
    }

    /// Smallest Cartesian matrix box containing both inputs.
    ///
    /// This is generally larger than their union. With one empty operand, entries
    /// of the other are copied through backend accounting in row-major order; an
    /// identity can therefore exhaust dyadic work. Two empty inputs need no copies.
    /// Nonempty pairwise hulls also run row-major and stop on the first scalar error.
    pub fn hull(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        match (&self.rows, &other.rows) {
            (None, None) => Self::empty(),
            (Some(rows), None) | (None, Some(rows)) => Self::new(array_result(|r| {
                array_result(|c| rows[r][c].copy(context))
            })?),
            (Some(a), Some(b)) => Self::new(array_result(|r| {
                array_result(|c| a[r][c].hull(&b[r][c], context))
            })?),
        }
    }

    /// Exact Cartesian intersection, canonicalizing empty entries.
    /// An empty operand skips entry work. Otherwise all intersections run row-major
    /// before canonicalization: an empty early entry does not hide a later error.
    pub fn intersection(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.zip(other, context, I::intersection)
    }

    /// Set inclusion. The empty set is included in every matrix set.
    /// Nonempty entries are checked row-major until the first false or error.
    /// Empty-set shortcuts and unit/unit inclusion perform no context work.
    pub fn subset_of(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<bool, IntervalLatticeError> {
        self.relation(other, context, I::subset_of)
    }

    /// Inclusion in the other's real interior (unit products are open).
    /// Empty is included even in empty. Positive-dimensional nonempty products need
    /// strict scalar inclusion at every entry, checked row-major with early false/error.
    pub fn interior_subset_of(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<bool, IntervalLatticeError> {
        self.relation(other, context, I::interior_subset_of)
    }

    /// Entrywise sum enclosure.
    /// Empty operands give empty. Otherwise visits entries row-major using backend
    /// rounding and one context, stopping at the first error with no partial matrix.
    pub fn add(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.zip(other, context, I::add)
    }

    /// Entrywise difference enclosure, treating inputs independently.
    /// A matrix minus itself need not be zero when entries are nonsingletons.
    /// Empty operands give empty; evaluation and failure order are row-major.
    pub fn sub(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.zip(other, context, I::sub)
    }

    /// Matrix-vector product enclosure. Singleton coefficients or lanes support
    /// point-matrix or point-vector products without a separate arithmetic path.
    /// Rows are evaluated in order, each dot product from column zero upward.
    ///
    /// Output lanes can share input variables, but their correlations are discarded.
    /// An empty operand gives empty even with a zero output dimension. For nonempty
    /// inputs, C=0 gives R scalar zeros; R=0 invokes no dot products and gives the
    /// unit vector. All products and sums share context and preserve the first error.
    pub fn mul_vector(
        &self,
        vector: &IntervalVector<I, C>,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, R>, IntervalLatticeError> {
        match (&self.rows, vector.lanes()) {
            (Some(a), Some(b)) => {
                IntervalVector::new(array_result(|r| crate::dot(a[r].iter().zip(b), context))?)
            }
            _ => IntervalVector::empty(),
        }
    }

    /// Matrix product enclosure. Entries are evaluated in row-major order and
    /// each dot product in increasing shared index. No budget is reset per row.
    ///
    /// The result has shape R by K; shared input entries may correlate its entries,
    /// but those relations are not retained. Empty operands give empty. Nonempty
    /// inputs with C=0 produce scalar zeros; a zero output dimension produces the
    /// unit matrix without dot products. Partial rows are discarded on scalar failure.
    pub fn mul_matrix<const K: usize>(
        &self,
        other: &IntervalMatrix<I, C, K>,
        context: &mut I::Context,
    ) -> Result<IntervalMatrix<I, R, K>, IntervalLatticeError> {
        match (&self.rows, &other.rows) {
            (Some(a), Some(b)) => IntervalMatrix::new(array_result(|r| {
                array_result(|k| crate::dot((0..C).map(|c| (&a[r][c], &b[c][k])), context))
            })?),
            _ => IntervalMatrix::empty(),
        }
    }

    /// Encloses `A*x+b`, with independent matrix, vector and translation sets.
    /// Correlation between output coordinates is not retained. This does not
    /// construct the exact correlated affine image of a box.
    ///
    /// Checks empty translation first, avoiding even a product that would overflow.
    /// Otherwise finishes the entire matrix-vector enclosure before adding translation
    /// coordinatewise. Both phases share context, with no refund if the second fails.
    pub fn affine_enclosure(
        &self,
        vector: &IntervalVector<I, C>,
        translation: &IntervalVector<I, R>,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, R>, IntervalLatticeError> {
        // Empty translation makes the entire source domain empty; do not perform
        // arithmetic that has no image (and could overflow).
        if translation.is_empty() {
            return IntervalVector::empty();
        }
        self.mul_vector(vector, context)?.add(translation, context)
    }

    /// Transposes with backend accounting for owned entry copies.
    /// Copies in output row-major order: input columns first, then rows. Empty maps
    /// to empty; zero-sized unit matrices remain units with the dimensions swapped.
    /// A nonempty transpose can exhaust dyadic resources even though its image is exact.
    pub fn transpose(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalMatrix<I, C, R>, IntervalLatticeError> {
        match &self.rows {
            Some(rows) => IntervalMatrix::new(array_result(|c| {
                array_result(|r| rows[r][c].copy(context))
            })?),
            None => IntervalMatrix::empty(),
        }
    }

    /// Projects a row, copying with backend accounting. Invalid indices are
    /// rejected even for empty input; projecting an empty set remains empty.
    /// Checks index < R before emptiness or context work. Valid nonempty rows copy
    /// columns in order; when C=0 the projection is the unit vector without copies.
    pub fn row(
        &self,
        index: usize,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, C>, IntervalLatticeError> {
        if index >= R {
            return Err(IntervalLatticeError::IndexOutOfBounds {
                index,
                dimension: R,
            });
        }
        match &self.rows {
            Some(rows) => IntervalVector::new(array_result(|c| rows[index][c].copy(context))?),
            None => IntervalVector::empty(),
        }
    }

    /// Projects a column with backend accounting and checked index.
    /// Checks index < C before emptiness or context work, including on empty input.
    /// Empty projects to empty; valid nonempty columns copy rows in order. R=0
    /// yields the unit vector without copies, while scalar failures remain typed.
    pub fn column(
        &self,
        index: usize,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, R>, IntervalLatticeError> {
        if index >= C {
            return Err(IntervalLatticeError::IndexOutOfBounds {
                index,
                dimension: C,
            });
        }
        match &self.rows {
            Some(rows) => IntervalVector::new(array_result(|r| rows[r][index].copy(context))?),
            None => IntervalVector::empty(),
        }
    }

    /// Lift a scalar operation across two nonempty matrices in row-major order.
    ///
    /// An empty operand skips op and constructs validated empty storage. Otherwise
    /// nested array_result calls stop at the first error and discard initialized
    /// entries/rows; Self::new canonicalizes only after complete success. An empty
    /// intermediate result does not skip later entries. Zero-sized units call no op.
    /// One mutable context is shared throughout, and its prior charges are not undone.
    fn zip(
        &self,
        other: &Self,
        context: &mut I::Context,
        mut op: impl FnMut(&I, &I, &mut I::Context) -> Result<I, hyperinterval::IntervalError>,
    ) -> Result<Self, IntervalLatticeError> {
        match (&self.rows, &other.rows) {
            (Some(a), Some(b)) => Self::new(array_result(|r| {
                array_result(|c| op(&a[r][c], &b[r][c], context))
            })?),
            _ => Self::empty(),
        }
    }

    /// Conjoin entry relations row-major with canonical Cartesian empty-set rules.
    ///
    /// Empty self returns true; nonempty self against empty returns false, without
    /// calling op. Otherwise stop at the first false or error; later entries spend
    /// no work and cannot change that outcome. Unit/unit returns true without calls.
    /// Used for closed and interior inclusion, with all attempted calls sharing context.
    fn relation(
        &self,
        other: &Self,
        context: &mut I::Context,
        mut op: impl FnMut(&I, &I, &mut I::Context) -> Result<bool, hyperinterval::IntervalError>,
    ) -> Result<bool, IntervalLatticeError> {
        match (&self.rows, &other.rows) {
            (None, _) => Ok(true),
            (Some(_), None) => Ok(false),
            (Some(a), Some(b)) => {
                for (a, b) in a.iter().flatten().zip(b.iter().flatten()) {
                    if !op(a, b, context)? {
                        return Ok(false);
                    }
                }
                Ok(true)
            }
        }
    }
}
