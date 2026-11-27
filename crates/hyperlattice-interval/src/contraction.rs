//! Identity remainders and norm margins for interval contraction arithmetic.
//!
//! These are Cartesian enclosure operations built on existing matrix norms and
//! scalar subtraction. They supply evidence for consumers that separately
//! establish the hypotheses of a contraction or interval root theorem.

use crate::{IntervalArithmetic, IntervalLatticeError, IntervalMatrix, array_result};

/// Square-matrix remainders and margins with shared scalar context accounting.
impl<I: IntervalArithmetic, const N: usize> IntervalMatrix<I, N, N> {
    /// Encloses `{ Id - A : A in self }`, subtracting from the identity matrix.
    ///
    /// Entries are evaluated in row-major order with one shared scalar context.
    /// Empty maps to empty. The nonempty 0×0 matrix maps to itself without
    /// constructing scalar one. Otherwise one must be representable: fixed
    /// i32/F31 and i64/F63 return typed overflow, even if some output entries
    /// would fit. All scalar subtraction failures propagate without partial output.
    /// Positive dimensions construct one before zero, then subtract each entry
    /// from its identity entry. The unit product constructs neither scalar
    /// constant because both array callbacks are skipped. These constants do
    /// not charge the context; entry subtractions do, without refund on failure.
    ///
    /// This supplies the `Id - C*J(X)` remainder in Krawczyk arithmetic after
    /// a matrix product. It does not establish that `J(X)` encloses a Jacobian,
    /// that `C` is a point preconditioner, or that a root exists or is unique.
    /// Rational/dyadic success gives the exact Cartesian image; primitive
    /// subtraction may widen it. No exact correlated image is implied when
    /// the input was itself an enclosure of a correlated matrix expression.
    pub fn identity_minus(&self, context: &mut I::Context) -> Result<Self, IntervalLatticeError> {
        let Some(rows) = self.rows() else {
            return Self::empty();
        };
        // The unit product requires no entries and may use a family without +1.
        if N == 0 {
            return Self::new(array_result(|_| array_result(|_| I::zero()))?);
        }
        let one = I::one()?;
        let zero = I::zero()?;
        Self::new(array_result(|r| {
            array_result(|c| {
                let identity = if r == c { &one } else { &zero };
                identity.sub(&rows[r][c], context)
            })
        })?)
    }

    /// Encloses `{ 1 - ||A||_1 : A in self }` using the induced one-norm.
    ///
    /// A strictly positive lower endpoint supplies a uniform norm-contraction
    /// bound. Containing zero is inconclusive; a negative bound is not a
    /// no-root claim. Solver decisions, Jacobian validity and self-mapping
    /// checks remain the consumer's responsibility.
    ///
    /// Empty maps to empty. The nonempty 0×0 margin is one. Nonempty queries
    /// first construct exact one (typed overflow for fixed i32/F31 or i64/F63),
    /// then evaluate the existing norm and subtract it, sharing the context.
    /// Rational/dyadic success gives the exact range. Primitive arithmetic may
    /// widen or overflow, including in an intermediate norm. Constant
    /// construction has the same accounting exclusion as scalar zero.
    /// The unit-product value one is conditional on representable constants
    /// and successful scalar subtraction; unlike `identity_minus`, this query
    /// can exhaust a dyadic work budget even in dimension zero.
    pub fn contraction_margin_one(
        &self,
        context: &mut I::Context,
    ) -> Result<I, IntervalLatticeError> {
        if self.is_empty() {
            return Ok(I::empty()?);
        }
        let one = I::one()?;
        let norm = self.norm_one(context)?;
        Ok(one.sub(&norm, context)?)
    }

    /// Encloses `{ 1 - ||A||_infinity : A in self }` using the maximum absolute
    /// row sum. Shares the evidence, empty/unit and resource contract of
    /// [`Self::contraction_margin_one`].
    pub fn contraction_margin_infinity(
        &self,
        context: &mut I::Context,
    ) -> Result<I, IntervalLatticeError> {
        if self.is_empty() {
            return Ok(I::empty()?);
        }
        let one = I::one()?;
        let norm = self.norm_infinity(context)?;
        Ok(one.sub(&norm, context)?)
    }
}
