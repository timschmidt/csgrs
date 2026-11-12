use crate::{
    IntervalArithmetic, IntervalLatticeError, IntervalMatrix, IntervalVector, array_result,
};
use hyperinterval::IntervalError;

// No scalar formulas live here: each absolute image, sum, maximum and difference
// delegates to Hyperinterval through the narrow backend adapter.
fn sum_abs<'a, I: IntervalArithmetic + 'a>(
    values: impl Iterator<Item = &'a I>,
    context: &mut I::Context,
) -> Result<I, IntervalError> {
    let mut sum = I::zero()?;
    for value in values {
        let absolute = value.abs(context)?;
        sum = sum.add(&absolute, context)?;
    }
    Ok(sum)
}

fn include_max<I: IntervalArithmetic>(
    maximum: &mut Option<I>,
    next: I,
    context: &mut I::Context,
) -> Result<(), IntervalError> {
    *maximum = Some(match maximum {
        None => next,
        Some(previous) => previous.max_image(&next, context)?,
    });
    Ok(())
}

impl<I: IntervalArithmetic, const N: usize> IntervalVector<I, N> {
    /// Encloses `{ sum_i |x_i| : x in self }`, including its minimum.
    ///
    /// Scalar absolute images are added in coordinate order with one shared
    /// context. Rational/dyadic success gives the exact range; primitive rounding
    /// can widen it and overflow is typed. Empty has empty image; the unique
    /// zero-dimensional vector has norm zero. No midpoint or point is selected.
    ///
    /// ```
    /// # #[cfg(feature = "fixed")] {
    /// use hyperlattice_interval::{IntervalVector, hyperinterval::FixedInterval};
    /// type I = FixedInterval<i32, 0>;
    /// let vector = IntervalVector::new([I::new(-2, 1).unwrap(), I::new(3, 4).unwrap()]).unwrap();
    /// assert_eq!(vector.norm_one(&mut ()).unwrap(), I::new(3, 6).unwrap());
    /// # }
    /// ```
    pub fn norm_one(&self, context: &mut I::Context) -> Result<I, IntervalLatticeError> {
        match self.lanes() {
            None => Ok(I::empty()?),
            Some(lanes) => Ok(sum_abs(lanes.iter(), context)?),
        }
    }

    /// Encloses `{ max_i |x_i| : x in self }`, including interior minima.
    /// Empty has empty image; dimension zero has norm zero. The scalar context
    /// covers every absolute image and maximum in increasing coordinate order.
    pub fn norm_infinity(&self, context: &mut I::Context) -> Result<I, IntervalLatticeError> {
        let Some(lanes) = self.lanes() else {
            return Ok(I::empty()?);
        };
        let mut maximum = None;
        for lane in lanes {
            let absolute = lane.abs(context)?;
            include_max(&mut maximum, absolute, context)?;
        }
        Ok(match maximum {
            Some(value) => value,
            None => I::zero()?,
        })
    }
}

impl<I: IntervalArithmetic, const R: usize, const C: usize> IntervalMatrix<I, R, C> {
    /// Encloses the induced one-norm range: maximum absolute column sum.
    ///
    /// The matrix set is Cartesian, so each column's extrema can be attained
    /// independently. Rational/dyadic success gives the exact range. Primitive
    /// sums round outward or fail with typed overflow. Empty has empty image;
    /// a nonempty matrix set with either dimension zero has norm zero.
    pub fn norm_one(&self, context: &mut I::Context) -> Result<I, IntervalLatticeError> {
        let Some(rows) = self.rows() else {
            return Ok(I::empty()?);
        };
        let mut maximum = None;
        for column in 0..C {
            let sum = sum_abs(rows.iter().map(|row| &row[column]), context)?;
            include_max(&mut maximum, sum, context)?;
        }
        Ok(match maximum {
            Some(value) => value,
            None => I::zero()?,
        })
    }

    /// Encloses the induced infinity-norm range: maximum absolute row sum.
    /// Rows and their entries are processed in increasing order. One context
    /// covers all operations, including maximum reduction; failures leave the
    /// source reusable. Empty and zero-dimension behavior matches `norm_one`.
    pub fn norm_infinity(&self, context: &mut I::Context) -> Result<I, IntervalLatticeError> {
        let Some(rows) = self.rows() else {
            return Ok(I::empty()?);
        };
        let mut maximum = None;
        for row in rows {
            let sum = sum_abs(row.iter(), context)?;
            include_max(&mut maximum, sum, context)?;
        }
        Ok(match maximum {
            Some(value) => value,
            None => I::zero()?,
        })
    }
}

impl<I: IntervalArithmetic, const N: usize> IntervalMatrix<I, N, N> {
    /// Encloses the logarithmic one-norm (matrix measure) range:
    /// `max_j (a_jj + sum_{i != j} |a_ij|)`. The diagonal retains its sign.
    ///
    /// For positive dimension, rational/dyadic success gives the exact range;
    /// primitive arithmetic may widen it or overflow. Empty input gives empty.
    /// For the nonempty 0×0 matrix, `lim_{h->0+} (norm(I+hA)-1)/h` is negative
    /// infinity under the zero-dimensional induced-norm convention. This finite
    /// scalar API reports [`IntervalError::UnboundedImage`] rather than zero.
    ///
    /// This is numerical evidence, not a solver uniqueness/contraction status.
    pub fn measure_one(&self, context: &mut I::Context) -> Result<I, IntervalLatticeError> {
        self.measure::<true>(context)
    }

    /// Encloses the logarithmic infinity-norm range:
    /// `max_i (a_ii + sum_{j != i} |a_ij|)`. Negative measures are retained.
    /// Shares the empty/zero-dimension and resource contract of `measure_one`.
    pub fn measure_infinity(&self, context: &mut I::Context) -> Result<I, IntervalLatticeError> {
        self.measure::<false>(context)
    }

    /// Per-row ranges of `|a_ii| - sum_{j != i} |a_ij|`.
    ///
    /// Strictly positive lower bounds supply diagonal-dominance evidence;
    /// a range containing zero is not a strict proof. No decision status is
    /// synthesized here. Empty input yields an empty vector; a nonempty 0×0
    /// matrix yields the unit vector product. Scalar failures propagate.
    pub fn row_dominance_margins(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        self.dominance_margins::<false>(context)
    }

    /// Per-column ranges of `|a_jj| - sum_{i != j} |a_ij|`, with the same
    /// evidence and resource contract as `row_dominance_margins`.
    pub fn column_dominance_margins(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        self.dominance_margins::<true>(context)
    }

    fn off_diagonal_sum<const COLUMN: bool>(
        rows: &[[I; N]; N],
        line: usize,
        context: &mut I::Context,
    ) -> Result<I, IntervalError> {
        sum_abs(
            (0..N).filter(|&index| index != line).map(|index| {
                if COLUMN {
                    &rows[index][line]
                } else {
                    &rows[line][index]
                }
            }),
            context,
        )
    }

    fn measure<const COLUMN: bool>(
        &self,
        context: &mut I::Context,
    ) -> Result<I, IntervalLatticeError> {
        let Some(rows) = self.rows() else {
            return Ok(I::empty()?);
        };
        if N == 0 {
            return Err(IntervalError::UnboundedImage.into());
        }
        let mut maximum = None;
        for line in 0..N {
            let sum = Self::off_diagonal_sum::<COLUMN>(rows, line, context)?;
            let measure = rows[line][line].add(&sum, context)?;
            include_max(&mut maximum, measure, context)?;
        }
        Ok(maximum.expect("positive dimension has at least one line"))
    }

    fn dominance_margins<const COLUMN: bool>(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        match self.rows() {
            None => IntervalVector::empty(),
            Some(rows) => IntervalVector::new(array_result(|line| {
                let sum = Self::off_diagonal_sum::<COLUMN>(rows, line, context)?;
                let diagonal = rows[line][line].abs(context)?;
                diagonal.sub(&sum, context)
            })?),
        }
    }
}
