//! Budgeted outward projection from exact rational endpoints.
use super::{Dyadic, Format};
use crate::{DyadicBudget, IntervalError, dyadic::admitted_shift};
use num_bigint::Sign;
use num_traits::ToPrimitive;

impl Format {
    #[inline]
    pub(crate) fn rational_bound(
        self,
        value: &hyperreal::Rational,
        upper: bool,
        budget: &mut DyadicBudget,
    ) -> Result<u64, IntervalError> {
        budget.reserve(1, 1, false)?;
        let (sign, numerator, denominator) = value.ratio_parts();
        if numerator.bits() == 0 {
            return Ok(0);
        }
        // The leading exponent is either order - 1 or order. Inspecting stored
        // lengths needs no reduction, allocation or exponent-sized shift.
        let order = i128::from(numerator.bits()) - i128::from(denominator.bits());
        if order > i128::from(self.bias()) + 1 {
            return Err(IntervalError::Overflow);
        }
        if order < i128::from(self.minimum()) {
            let (lo, hi) = self.signed(0, 1, sign == Sign::Minus);
            return Ok(if upper { hi } else { lo });
        }
        // This grid divides the IEEE spacing on either possible binade. An
        // outward projection onto it cannot skip a representable endpoint.
        let grid = self
            .minimum()
            .max(order as i32 - 1 - self.fraction() as i32);
        let shift = -i64::from(grid);
        let amount = admitted_shift(numerator.bits(), denominator.bits(), shift, budget)?;
        let scaled;
        let (n, d) = if shift >= 0 {
            scaled = numerator << amount;
            (&scaled, denominator)
        } else {
            scaled = denominator << amount;
            (numerator, &scaled)
        };
        // Increasing the magnitude moves a negative endpoint downward.
        // Compute only the directed quotient needed by this endpoint.
        let negative = sign == Sign::Minus;
        let directed = if upper != negative {
            num_integer::Integer::div_ceil(n, d)
        } else {
            n / d
        };
        budget.reserve(u64::from(self.fraction()) + 3, 1, false)?;
        let magnitude = directed.to_u128().ok_or(IntervalError::Overflow)?;
        let (lo, hi) = self.rounded(Dyadic {
            negative,
            magnitude,
            exponent: grid,
        })?;
        Ok(if upper { hi } else { lo })
    }
}
