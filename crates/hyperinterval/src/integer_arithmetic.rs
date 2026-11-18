use crate::bounds::Bounds;
use crate::{IntegerRange, IntervalError, MidpointRounding};

macro_rules! arithmetic {
    ($($endpoint:ty),*) => { $(
        impl IntegerRange<$endpoint> {
            /// Exact pointwise absolute-value image, with zero minimum across
            /// zero. A signed minimum that cannot be reflected returns Overflow.
            /// Unsigned ranges are unchanged, including their maximum endpoint.
            pub fn abs(&self) -> Result<Self, IntervalError> {
                let Some((a, b)) = self.bounds() else { return Ok(*self); };
                let signs = self.signs();
                if !signs.contains_negative() { return Ok(*self); }
                let upper = a.checked_neg().ok_or(IntervalError::Overflow)?;
                if signs.contains_zero() {
                    Self::new(0, upper.max(b))
                } else {
                    Self::new(b.checked_neg().ok_or(IntervalError::Overflow)?, upper)
                }
            }

            /// Exact endpoint distance, widened to u128; empty returns `None`.
            ///
            /// This is the width, not the cardinality. The cardinality of the
            /// full u128 range would require 129 bits.
            pub fn width(&self) -> Option<u128> {
                self.bounds().map(|(lower, upper)| upper.abs_diff(lower) as u128)
            }

            /// Midpoint with explicit integer rounding; empty returns `None`.
            #[inline(always)]
            pub fn midpoint(&self, rounding: MidpointRounding) -> Option<$endpoint> {
                let (lower, upper) = self.bounds()?;
                Some(crate::bounds::lattice_midpoint!(lower, upper, rounding, $endpoint))
            }

            /// Outward integer radius around any supported rounded midpoint.
            pub fn radius(&self) -> Option<u128> {
                self.width().map(|width| width / 2 + width % 2)
            }

            /// Exact discrete Minkowski sum, with checked endpoint arithmetic.
            pub fn checked_add(&self, other: &Self) -> Result<Self, IntervalError> {
                match (self.bounds(), other.bounds()) {
                    (Some((a, b)), Some((c, d))) => {
                        let lower = a.checked_add(c).ok_or(IntervalError::Overflow)?;
                        let upper = b.checked_add(d).ok_or(IntervalError::Overflow)?;
                        // Addition is monotone in each operand. Once both
                        // checked sums succeed, a <= b and c <= d prove order.
                        Ok(Self::from_bounds(Bounds::ordered(lower, upper)))
                    }
                    _ => Ok(Self::empty()),
                }
            }

            /// Exact discrete Minkowski difference, rejecting overflow/unsigned underflow.
            pub fn checked_sub(&self, other: &Self) -> Result<Self, IntervalError> {
                match (self.bounds(), other.bounds()) {
                    (Some((a, b)), Some((c, d))) => Self::new(
                        a.checked_sub(d).ok_or(IntervalError::Overflow)?,
                        b.checked_sub(c).ok_or(IntervalError::Overflow)?),
                    _ => Ok(Self::empty()),
                }
            }

            /// Exact reflected set; unsigned nonzero values cannot be represented.
            pub fn checked_neg(&self) -> Result<Self, IntervalError> {
                match self.bounds() {
                    Some((a, b)) => Self::new(b.checked_neg().ok_or(IntervalError::Overflow)?, a.checked_neg().ok_or(IntervalError::Overflow)?),
                    None => Ok(Self::empty()),
                }
            }

            /// Smallest integer range enclosing every pairwise product.
            ///
            /// Integer products can have gaps; the returned range is their
            /// hull, not a claim that each integer in the hull is attained.
            pub fn checked_mul(&self, other: &Self) -> Result<Self, IntervalError> {
                match (self.bounds(), other.bounds()) {
                    (Some((a, b)), Some((c, d))) => {
                        let products = [a.checked_mul(c), a.checked_mul(d), b.checked_mul(c), b.checked_mul(d)];
                        let [a, b, c, d] = products;
                        let a = a.ok_or(IntervalError::Overflow)?;
                        let b = b.ok_or(IntervalError::Overflow)?;
                        let c = c.ok_or(IntervalError::Overflow)?;
                        let d = d.ok_or(IntervalError::Overflow)?;
                        Self::new(a.min(b).min(c).min(d), a.max(b).max(c).max(d))
                    }
                    _ => Ok(Self::empty()),
                }
            }

            /// Smallest integer hull after multiplication by one exact scalar.
            ///
            /// Scaling can leave gaps between attained integers. A negative
            /// factor reverses endpoint order; zero maps every nonempty range
            /// to zero, and empty stays empty. Unrepresentable products return
            /// `Overflow`. Known scalar sign avoids a general four-corner product.
            #[inline]
            pub fn scale(&self, factor: $endpoint) -> Result<Self, IntervalError> {
                let Some((a, b)) = self.bounds() else { return Ok(Self::empty()); };
                let a = a.checked_mul(factor).ok_or(IntervalError::Overflow)?;
                let b = b.checked_mul(factor).ok_or(IntervalError::Overflow)?;
                let (lower, upper) = if factor.cmp(&0).is_lt() { (b, a) } else { (a, b) };
                // Multiplication by a nonnegative scalar is monotone; a
                // negative scalar reverses order. Checked products remain exact.
                Ok(Self::from_bounds(Bounds::ordered(lower, upper)))
            }

            /// Smallest integer hull of pointwise squares, with zero-aware bounds.
            pub fn square(&self) -> Result<Self, IntervalError> {
                self.checked_pow(2)
            }

            /// Smallest integer hull of a nonnegative integer power.
            ///
            /// Exponent zero maps every nonempty set to one, including zero.
            /// Empty maps to empty. Primitive checked exponentiation is bounded
            /// by the 32-bit exponent and never wraps.
            pub fn checked_pow(&self, exponent: u32) -> Result<Self, IntervalError> {
                match self.bounds() {
                    Some((a, b)) => {
                        let aa = a.checked_pow(exponent).ok_or(IntervalError::Overflow)?;
                        let bb = b.checked_pow(exponent).ok_or(IntervalError::Overflow)?;
                        let lower = if exponent != 0 && exponent % 2 == 0 && self.contains_point(0) { 0 } else { aa.min(bb) };
                        Self::new(lower, aa.max(bb))
                    }
                    None => Ok(Self::empty()),
                }
            }

            /// Smallest integer hull of `factor * x + offset` for members of this range.
            ///
            /// Returns `Overflow` only when a final endpoint is unrepresentable.
            /// An out-of-range product can be brought back into range by the offset.
            /// Empty maps to empty. When the factor has magnitude above one, the
            /// hull also contains integers skipped between successive image values.
            pub fn affine(&self, factor: $endpoint, offset: $endpoint) -> Result<Self, IntervalError> {
                let Some((a, b)) = self.bounds() else {
                    return Ok(Self::empty());
                };
                let reversed = factor.cmp(&0).is_lt();
                let factor_magnitude = factor.abs_diff(0);
                let image = |x: $endpoint| {
                    // Any two representable values differ by at most the unsigned
                    // maximum. A larger displacement cannot be rescued by any offset.
                    let distance = x.abs_diff(0).checked_mul(factor_magnitude)
                        .ok_or(IntervalError::Overflow)?;
                    let subtract = x.cmp(&0).is_lt() != reversed;
                    let limit = if subtract { <$endpoint>::MIN } else { <$endpoint>::MAX };
                    if distance > offset.abs_diff(limit) {
                        return Err(IntervalError::Overflow);
                    }
                    // The final mathematical value fits. Modular arithmetic applies
                    // its displacement even when the signed distance itself cannot fit.
                    Ok(if subtract {
                        offset.wrapping_sub(distance as $endpoint)
                    } else {
                        offset.wrapping_add(distance as $endpoint)
                    })
                };
                let (lower, upper) = if reversed { (b, a) } else { (a, b) };
                Ok(Self::from_bounds(Bounds::ordered(image(lower)?, image(upper)?)))
            }
        }
    )* };
}

arithmetic!(
    i8, i16, i32, i64, i128, isize, u8, u16, u32, u64, u128, usize
);
