use crate::{IntervalArithmetic, IntervalLatticeError, IntervalVector, array_result};

/// Cartesian set of row-major `R` by `C` matrices, or the canonical empty set.
/// All entries vary independently. Empty dimensions have a singleton unit
/// product unless the explicit empty constructor is used.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IntervalMatrix<I: IntervalArithmetic, const R: usize, const C: usize> {
    pub(crate) rows: Option<[[I; C]; R]>,
}

impl<I: IntervalArithmetic, const R: usize, const C: usize> IntervalMatrix<I, R, C> {
    /// Constructs from row-major scalar intervals; singleton entries lift a point
    /// matrix. Any empty entry canonicalizes the whole matrix set to empty.
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
    pub fn empty() -> Result<Self, IntervalLatticeError> {
        I::validate_family()?;
        Ok(Self { rows: None })
    }

    /// Whether the matrix set is empty.
    pub const fn is_empty(&self) -> bool {
        self.rows.is_none()
    }

    /// Whether exactly one matrix is represented, including empty dimensions.
    pub fn is_singleton(&self) -> bool {
        self.rows
            .as_ref()
            .is_some_and(|rows| rows.iter().flatten().all(I::is_singleton))
    }

    /// Borrowed row-major entries, or `None` for empty. No scalar copies occur.
    pub const fn rows(&self) -> Option<&[[I; C]; R]> {
        self.rows.as_ref()
    }

    /// Transfers owned entries without copying scalar data.
    pub fn into_rows(self) -> Option<[[I; C]; R]> {
        self.rows
    }

    /// Smallest Cartesian matrix box containing both inputs.
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
    pub fn intersection(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.zip(other, context, I::intersection)
    }

    /// Set inclusion. The empty set is included in every matrix set.
    pub fn subset_of(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<bool, IntervalLatticeError> {
        self.relation(other, context, I::subset_of)
    }

    /// Inclusion in the other's real interior (unit products are open).
    pub fn interior_subset_of(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<bool, IntervalLatticeError> {
        self.relation(other, context, I::interior_subset_of)
    }

    /// Entrywise sum enclosure.
    pub fn add(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.zip(other, context, I::add)
    }

    /// Entrywise difference enclosure, treating inputs independently.
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
