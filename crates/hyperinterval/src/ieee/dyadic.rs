//! Directed projection of arbitrary finite dyadic endpoints onto IEEE grids.
use super::Format;
use crate::{DyadicBudget, IntervalError};
use num_bigint::{BigInt, Sign};
use num_traits::ToPrimitive;

impl Format {
    pub(crate) fn enclose_dyadic(
        self,
        mantissa: &BigInt,
        exponent: i32,
        budget: &mut DyadicBudget,
    ) -> Result<(u64, u64), IntervalError> {
        let magnitude = mantissa.magnitude();
        let bits = magnitude.bits();
        // Reserve before limb scans/shifts. Three linear stages cover input
        // inspection, retained significand and discarded-tail inspection. The
        // carry needs at most fraction+2 bits. No exponent-sized shift occurs.
        budget.reserve(bits.max(u64::from(self.fraction()) + 2), 3, false)?;
        if bits == 0 {
            return Ok((0, 0));
        }
        let negative = mantissa.sign() == Sign::Minus;
        let top = i128::from(exponent) + i128::from(bits) - 1;
        if top > i128::from(self.bias()) {
            return Err(IntervalError::Overflow);
        }
        if top < i128::from(self.minimum()) {
            return Ok(self.signed(0, 1, negative));
        }
        // top is now between the smallest subnormal and largest normal
        // exponents. The output grid and significand fit the existing encoder.
        let grid = self.minimum().max(top as i32 - self.fraction() as i32);
        let shift = i64::from(grid) - i64::from(exponent);
        let (floor, remainder) = if shift > 0 {
            let shift = usize::try_from(shift).map_err(|_| IntervalError::SizeExhausted)?;
            let floor = (magnitude >> shift)
                .to_u128()
                .expect("retained IEEE significand has at most 53 bits");
            let remainder = magnitude.trailing_zeros().expect("nonzero magnitude") < shift as u64;
            (floor, remainder)
        } else {
            let floor = magnitude
                .to_u128()
                .expect("unshifted IEEE significand has at most 53 bits")
                << -shift;
            (floor, false)
        };
        let lower = self.encode(floor, grid)?;
        let upper = self.encode(floor + u128::from(remainder), grid)?;
        Ok(self.signed(lower, upper, negative))
    }
}
