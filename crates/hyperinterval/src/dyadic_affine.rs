use super::{BigInt, DyadicBudget, DyadicInterval, IntervalError, Zero};

impl DyadicInterval {
    /// Exact image under multiplication by a dyadic scalar.
    ///
    /// The scalar is a borrowed mantissa and a base-two exponent. Its sign
    /// determines which endpoint is the minimum. A nonconstant product uses
    /// two endpoint multiplications and shares any exponent repair with the
    /// same budget. Empty maps to empty, and zero times a nonempty set is zero.
    #[inline]
    pub fn scale(
        &self,
        factor: (&BigInt, i32),
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalError> {
        let Some((lower, upper, exponent)) = self.bounds() else {
            return Ok(Self::empty());
        };
        let (factor, factor_exponent) = factor;
        if factor.is_zero() || self.bits() == 0 {
            return Self::point(BigInt::zero(), 0);
        }
        let (lower, upper) = if factor.sign() == num_bigint::Sign::Minus {
            (upper, lower)
        } else {
            (lower, upper)
        };
        let wide = i64::from(exponent) + i64::from(factor_exponent);
        let grid = wide.clamp(i64::from(i32::MIN), i64::from(i32::MAX));
        let shifted;
        let factor = if wide < grid {
            // Here the source exponent is negative, so normalization left at
            // least one odd endpoint. The factor must supply the entire shift.
            let shift = (grid - wide) as u64;
            budget.reserve(factor.bits(), 1, false)?;
            if factor.trailing_zeros().expect("nonzero factor") < shift {
                return Err(IntervalError::Overflow);
            }
            shifted = factor >> usize::try_from(shift).map_err(|_| IntervalError::Overflow)?;
            &shifted
        } else if wide > grid {
            let shift = (wide - grid) as u64;
            budget.reserve(factor.bits().saturating_add(shift), 1, false)?;
            shifted = factor << usize::try_from(shift).map_err(|_| IntervalError::Overflow)?;
            &shifted
        } else {
            factor
        };
        budget.reserve(self.bits().saturating_add(factor.bits()), 2, true)?;
        Self::new(lower * factor, upper * factor, grid as i32)
    }

    /// Exact image of `factor * x + offset` for two dyadic scalars.
    ///
    /// Each scalar is a borrowed mantissa and its base-two exponent. The
    /// mantissas need not be normalized. A negative factor reverses endpoint
    /// order, a zero factor maps a nonempty input to the offset, and an empty
    /// input maps to empty. A zero offset requires no grid alignment.
    ///
    /// The two endpoint products, any exponent repair and common-grid addition
    /// share the supplied budget. An endpoint requiring a grid finer than the
    /// minimum stored exponent returns [`IntervalError::Overflow`].
    #[inline]
    pub fn affine(
        &self,
        factor: (&BigInt, i32),
        offset: (&BigInt, i32),
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalError> {
        let product = self.scale(factor, budget)?;
        if product.is_empty() || offset.0.is_zero() {
            return Ok(product);
        }
        let bits = product.bits();
        let grid = if bits == 0 {
            offset.1
        } else {
            product.exponent.min(offset.1)
        };
        // The product owns its endpoints. Translate them in place on the
        // finer grid, aligning the common offset only once.
        let (mut lower, mut upper) = product.bounds.expect("nonempty product").endpoints();
        let mut product_bits = bits;
        if bits != 0 && product.exponent > grid {
            let shift = (i64::from(product.exponent) - i64::from(grid)) as u64;
            product_bits = bits.saturating_add(shift);
            budget.reserve(product_bits, 2, false)?;
            let shift = usize::try_from(shift).map_err(|_| IntervalError::Overflow)?;
            lower <<= shift;
            upper <<= shift;
        }
        let shifted;
        let translation = if offset.1 > grid {
            let shift = (i64::from(offset.1) - i64::from(grid)) as u64;
            budget.reserve(offset.0.bits().saturating_add(shift), 1, false)?;
            shifted = offset.0 << usize::try_from(shift).map_err(|_| IntervalError::Overflow)?;
            &shifted
        } else {
            offset.0
        };
        let sum_bits = product_bits
            .max(translation.bits())
            .saturating_add(u64::from(bits != 0));
        budget.reserve(sum_bits, 2, false)?;
        lower += translation;
        upper += translation;
        Self::new(lower, upper, grid)
    }
}
