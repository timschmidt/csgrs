//! Arithmetic aligns retained coordinate fields before combining expressions.

use super::*;
use hypersolve::bivariate_arithmetic::bivariate_add;

#[derive(Clone, Copy)]
enum Operation {
    Add,
    Subtract,
    Multiply,
    Divide,
}

impl CurvePointRationalForm2 {
    /// Adds two rational forms in a certified common point field.
    ///
    /// Polynomial-fiber images, native rational parameter images, similarities
    /// and represented constants are pulled back at their owning layer. A join
    /// of independent unavailable fields remains a typed blocker. Both original
    /// operands, including their denominator guards, remain queryable.
    pub fn add(&self, other: &Self) -> ExactCurveResult<Self> {
        self.combine(other, Operation::Add)
    }

    /// Subtracts in a common retained field without independent root images.
    pub fn subtract(&self, other: &Self) -> ExactCurveResult<Self> {
        self.combine(other, Operation::Subtract)
    }

    /// Multiplies in a common retained field, preserving both original guards.
    pub fn multiply(&self, other: &Self) -> ExactCurveResult<Self> {
        self.combine(other, Operation::Multiply)
    }

    /// Divides in a common retained field. A zero original divisor is rejected
    /// before any cancellation, including `zero / zero`.
    pub fn divide(&self, other: &Self) -> ExactCurveResult<Self> {
        if other.compare_to_real(&Real::zero())? == Ordering::Equal {
            return Err(ExactCurveError::invalid_unattributed(
                CurveOperation2::Construction,
                crate::CurveError::ZeroRationalPointDenominator,
            ));
        }
        self.combine(other, Operation::Divide)
    }

    /// Scales a query without changing its original source or pole guard.
    pub fn scaled(&self, factor: &Real) -> ExactCurveResult<Self> {
        let mut form = self.source.rational_form(
            bivariate_scale(self.numerator.clone(), factor),
            self.denominator.clone(),
        )?;
        form.operands = self.operands.clone();
        Ok(form)
    }

    fn combine(&self, other: &Self, operation: Operation) -> ExactCurveResult<Self> {
        let (source, numerator, denominator) = resolve(CurveOperation2::Construction, |policy| {
            use crate::bezier_offset::{PointFormSource2, common_point_form_sources};
            let [first, second] = match common_point_form_sources(
                PointFormSource2::new(
                    self.source.clone(),
                    self.numerator.clone(),
                    self.denominator.clone(),
                ),
                PointFormSource2::new(
                    other.source.clone(),
                    other.numerator.clone(),
                    other.denominator.clone(),
                ),
                policy,
            )? {
                Classification::Decided(forms) => forms,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            // Keeping one already shared denominator prevents finite sums
            // (including dot products) from multiplying its degree at every
            // addition. This is structural equality, not a guessed identity.
            let same_denominator = first.denominator == second.denominator;
            let common_denominator =
                matches!(operation, Operation::Add | Operation::Subtract) && same_denominator;
            let numerator = match operation {
                Operation::Add if common_denominator => {
                    bivariate_add(&first.numerator, &second.numerator)
                }
                Operation::Subtract if common_denominator => {
                    bivariate_subtract(&first.numerator, &second.numerator)
                }
                Operation::Add | Operation::Subtract => {
                    let a = bivariate_multiply(&first.numerator, &second.denominator);
                    let b = bivariate_multiply(&second.numerator, &first.denominator);
                    if matches!(operation, Operation::Add) {
                        bivariate_add(&a, &b)
                    } else {
                        bivariate_subtract(&a, &b)
                    }
                }
                Operation::Multiply => bivariate_multiply(&first.numerator, &second.numerator),
                Operation::Divide => bivariate_multiply(&first.numerator, &second.denominator),
            };
            let denominator = if common_denominator {
                first.denominator.clone()
            } else {
                bivariate_multiply(
                    &first.denominator,
                    if matches!(operation, Operation::Divide) {
                        &second.numerator
                    } else {
                        &second.denominator
                    },
                )
            };
            // Division by b/e is defined only where the original e is
            // nonzero. Keep that guard in the replayable denominator even
            // though algebraic inversion moves e into the numerator. An
            // already shared denominator retains the same guard once.
            let (numerator, denominator) =
                if matches!(operation, Operation::Divide) && !same_denominator {
                    (
                        bivariate_multiply(&numerator, &second.denominator),
                        bivariate_multiply(&denominator, &second.denominator),
                    )
                } else {
                    (numerator, denominator)
                };
            Ok(Classification::Decided((
                first.source,
                numerator,
                denominator,
            )))
        })?;
        let mut result = source.rational_form(numerator, denominator)?;
        result.operands = Some(std::sync::Arc::new([self.clone(), other.clone()]));
        Ok(result)
    }
}
