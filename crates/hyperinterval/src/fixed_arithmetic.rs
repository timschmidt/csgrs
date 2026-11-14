//! Widened arithmetic for the sealed i32/i64 fixed-point families.

use crate::integer_quotient::positive_quotient_bounds as quotient_bounds;
use crate::{DomainCoverage, FixedInterval, IntervalError, MidpointRounding, RangeEnclosure};

#[inline]
fn scaled_ratio(numerator: i128, denominator: i128, scale: i128) -> (i128, i128) {
    if denominator < 0 {
        quotient_bounds(-numerator * scale, -denominator)
    } else {
        quotient_bounds(numerator * scale, denominator)
    }
}

macro_rules! arithmetic {
    ($endpoint:ty) => {
        impl<const FRAC: u32> FixedInterval<$endpoint, FRAC> {
            /// Exact pointwise absolute-value image on the existing lattice.
            /// Crossing zero gives lower bound zero. A signed minimum whose
            /// positive reflection cannot fit returns Overflow, never saturation.
            pub fn abs(&self) -> Result<Self, IntervalError> {
                let Some((a, b)) = self.bounds() else {
                    return Ok(*self);
                };
                if a >= 0 {
                    return Ok(*self);
                }
                let upper = a.checked_neg().ok_or(IntervalError::Overflow)?;
                if b >= 0 {
                    Self::new(0, upper.max(b))
                } else {
                    Self::new(b.checked_neg().ok_or(IntervalError::Overflow)?, upper)
                }
            }

            fn scale() -> Result<i128, IntervalError> {
                if FRAC >= <$endpoint>::BITS {
                    return Err(IntervalError::InvalidScale);
                }
                Ok(1_i128 << FRAC)
            }

            fn from_wide(lower: i128, upper: i128) -> Result<Self, IntervalError> {
                let lower = <$endpoint>::try_from(lower).map_err(|_| IntervalError::Overflow)?;
                let upper = <$endpoint>::try_from(upper).map_err(|_| IntervalError::Overflow)?;
                Self::new(lower, upper)
            }

            /// Smallest outward lattice enclosure of an exact i64 ratio.
            ///
            /// A zero denominator returns `ContainsZero`; unrepresentable
            /// endpoints return `Overflow`. All intermediate products fit i128.
            pub fn from_ratio(numerator: i64, denominator: i64) -> Result<Self, IntervalError> {
                let scale = Self::scale()?;
                if denominator == 0 {
                    return Err(IntervalError::ContainsZero);
                }
                let (lower, upper) =
                    scaled_ratio(i128::from(numerator), i128::from(denominator), scale);
                Self::from_wide(lower, upper)
            }

            /// Exact width in lattice units, including the full signed range.
            /// Empty has no width and returns `None`.
            pub fn width_ulps(&self) -> Option<u64> {
                self.bounds()
                    .map(|(lower, upper)| (i128::from(upper) - i128::from(lower)) as u64)
            }

            /// Exact scale-tagged unsigned endpoint distance; empty has no width.
            /// This includes widths larger than either signed endpoint can hold.
            pub fn width(&self) -> Option<crate::FixedWidth<FRAC>> {
                self.width_ulps()
                    .map(|n| crate::FixedWidth::new(n).expect("source scale is valid"))
            }

            /// Changes scale with exact refinement or outward-rounded coarsening.
            ///
            /// No exact source point is lost. Refinement can overflow the
            /// endpoint storage; invalid destination scales fail even for empty.
            pub fn rescale<const DESTINATION: u32>(
                &self,
            ) -> Result<FixedInterval<$endpoint, DESTINATION>, IntervalError> {
                FixedInterval::<$endpoint, DESTINATION>::scale()?;
                match self.bounds() {
                    Some((lower, upper)) => {
                        let (lower, upper) = if DESTINATION >= FRAC {
                            let factor = 1_i128 << (DESTINATION - FRAC);
                            (i128::from(lower) * factor, i128::from(upper) * factor)
                        } else {
                            let divisor = 1_i128 << (FRAC - DESTINATION);
                            (
                                quotient_bounds(i128::from(lower), divisor).0,
                                quotient_bounds(i128::from(upper), divisor).1,
                            )
                        };
                        FixedInterval::<$endpoint, DESTINATION>::from_wide(lower, upper)
                    }
                    None => FixedInterval::empty(),
                }
            }

            /// Continuous real hull of a discrete integer range, converted exactly.
            ///
            /// This explicit conversion fills the gaps between integers and may
            /// overflow when scaled numerators cannot fit the endpoint storage.
            #[cfg(feature = "integer")]
            pub fn from_integer_range(
                range: &crate::IntegerRange<$endpoint>,
            ) -> Result<Self, IntervalError> {
                let scale = Self::scale()?;
                match range.bounds() {
                    Some((lower, upper)) => {
                        Self::from_wide(i128::from(lower) * scale, i128::from(upper) * scale)
                    }
                    None => Self::empty(),
                }
            }

            /// Exact discrete set of integers contained in this real interval.
            #[cfg(feature = "integer")]
            pub fn integer_points(&self) -> Result<crate::IntegerRange<$endpoint>, IntervalError> {
                let scale = Self::scale()?;
                match self.bounds() {
                    Some((lower, upper)) => {
                        let lower = quotient_bounds(i128::from(lower), scale).1;
                        let upper = quotient_bounds(i128::from(upper), scale).0;
                        if lower > upper {
                            return Ok(crate::IntegerRange::empty());
                        }
                        // Dividing by scale >= 1 keeps every surviving integer
                        // inside the original signed endpoint range.
                        crate::IntegerRange::new(lower as $endpoint, upper as $endpoint)
                    }
                    None => Ok(crate::IntegerRange::empty()),
                }
            }

            /// Midpoint lattice numerator with explicit rounding; empty returns `None`.
            pub fn midpoint(&self, rounding: MidpointRounding) -> Option<$endpoint> {
                self.bounds().map(|(lower, upper)| {
                    let width = i128::from(upper) - i128::from(lower);
                    let floor = i128::from(lower) + width / 2;
                    let increment = width % 2 != 0
                        && match rounding {
                            MidpointRounding::Floor => false,
                            MidpointRounding::Ceiling => true,
                            MidpointRounding::NearestTiesEven => floor % 2 != 0,
                        };
                    // The rounded midpoint lies between representable endpoints.
                    (floor + i128::from(increment)) as $endpoint
                })
            }

            /// Outward radius in lattice units around any supported rounded midpoint.
            /// Empty returns `None`; odd widths round the half-width upward.
            pub fn radius_ulps(&self) -> Option<u64> {
                self.width_ulps().map(|width| width / 2 + width % 2)
            }

            /// Tight outward enclosure of the exact midpoint on this lattice.
            /// Unlike `midpoint`, this returns both neighbors when the exact
            /// half-step is not representable. Empty maps to empty.
            pub fn midpoint_enclosure(&self) -> Result<Self, IntervalError> {
                self.half_measure_enclosure(false)
            }

            /// Tight outward enclosure of the exact half-width on this lattice.
            /// A radius outside signed endpoint storage returns `Overflow`.
            /// This measures the source radius, not one about a rounded center.
            pub fn radius_enclosure(&self) -> Result<Self, IntervalError> {
                self.half_measure_enclosure(true)
            }

            fn half_measure_enclosure(&self, difference: bool) -> Result<Self, IntervalError> {
                match self.bounds() {
                    None => Self::empty(),
                    Some((a, b)) => {
                        let numerator = if difference {
                            i128::from(b) - i128::from(a)
                        } else {
                            i128::from(a) + i128::from(b)
                        };
                        let (lo, hi) = quotient_bounds(numerator, 2);
                        Self::from_wide(lo, hi)
                    }
                }
            }

            /// Exact Minkowski sum, or `Overflow` if an endpoint does not fit.
            pub fn checked_add(&self, other: &Self) -> Result<Self, IntervalError> {
                match (self.bounds(), other.bounds()) {
                    (Some((a, b)), Some((c, d))) => Self::from_wide(
                        i128::from(a) + i128::from(c),
                        i128::from(b) + i128::from(d),
                    ),
                    _ => Self::empty(),
                }
            }

            /// Exact Minkowski difference, with widened intermediates and no saturation.
            pub fn checked_sub(&self, other: &Self) -> Result<Self, IntervalError> {
                match (self.bounds(), other.bounds()) {
                    (Some((a, b)), Some((c, d))) => Self::from_wide(
                        i128::from(a) - i128::from(d),
                        i128::from(b) - i128::from(c),
                    ),
                    _ => Self::empty(),
                }
            }

            /// Exact reflected set, or `Overflow` at an unrepresentable negation.
            pub fn checked_neg(&self) -> Result<Self, IntervalError> {
                match self.bounds() {
                    Some((lower, upper)) => Self::from_wide(-i128::from(upper), -i128::from(lower)),
                    None => Self::empty(),
                }
            }

            /// Tight outward-rounded product hull using i128 intermediates.
            pub fn checked_mul(&self, other: &Self) -> Result<Self, IntervalError> {
                let scale = Self::scale()?;
                match (self.bounds(), other.bounds()) {
                    (Some((a, b)), Some((c, d))) => {
                        let products = [
                            i128::from(a) * i128::from(c),
                            i128::from(a) * i128::from(d),
                            i128::from(b) * i128::from(c),
                            i128::from(b) * i128::from(d),
                        ];
                        let lower = products[0]
                            .min(products[1])
                            .min(products[2])
                            .min(products[3]);
                        let upper = products[0]
                            .max(products[1])
                            .max(products[2])
                            .max(products[3]);
                        Self::from_wide(
                            quotient_bounds(lower, scale).0,
                            quotient_bounds(upper, scale).1,
                        )
                    }
                    _ => Self::empty(),
                }
            }

            /// Tight outward-rounded quotient hull when zero is excluded.
            ///
            /// A zero-containing denominator is rejected even if the numerator
            /// is empty. Otherwise an empty operand produces an empty image.
            pub fn checked_div(&self, other: &Self) -> Result<Self, IntervalError> {
                let scale = Self::scale()?;
                if other.contains_point(0) {
                    return Err(IntervalError::ContainsZero);
                }
                match (self.bounds(), other.bounds()) {
                    (Some((a, b)), Some((c, d))) => {
                        let ratios = [
                            scaled_ratio(i128::from(a), i128::from(c), scale),
                            scaled_ratio(i128::from(a), i128::from(d), scale),
                            scaled_ratio(i128::from(b), i128::from(c), scale),
                            scaled_ratio(i128::from(b), i128::from(d), scale),
                        ];
                        let lower = ratios[0]
                            .0
                            .min(ratios[1].0)
                            .min(ratios[2].0)
                            .min(ratios[3].0);
                        let upper = ratios[0]
                            .1
                            .max(ratios[1].1)
                            .max(ratios[2].1)
                            .max(ratios[3].1);
                        Self::from_wide(lower, upper)
                    }
                    _ => Self::empty(),
                }
            }

            /// Tight reciprocal enclosure, without requiring 1 to fit this lattice.
            ///
            /// See also [`Self::div_integer`] for division by an exact integer
            /// that need not be representable as a fixed-point denominator.
            pub fn reciprocal(&self) -> Result<Self, IntervalError> {
                let scale = Self::scale()?;
                if self.contains_point(0) {
                    return Err(IntervalError::ContainsZero);
                }
                match self.bounds() {
                    Some((lower, upper)) => {
                        let (image_lower, _) = scaled_ratio(scale, i128::from(upper), scale);
                        let (_, image_upper) = scaled_ratio(scale, i128::from(lower), scale);
                        Self::from_wide(image_lower, image_upper)
                    }
                    None => Self::empty(),
                }
            }

            /// Tight outward enclosure of `self / divisor`, retaining this grid.
            ///
            /// `divisor` is an exact integer, not a lattice numerator. It need
            /// not fit as a point of this fixed-point family. Zero is rejected
            /// with [`IntervalError::ContainsZero`] even for an empty numerator;
            /// otherwise empty maps to empty. Negative divisors reverse order.
            /// An unrepresentable endpoint returns [`IntervalError::Overflow`],
            /// including a signed minimum divided by -1. There is no saturation,
            /// allocation, or implicit rescaling.
            ///
            /// ```
            /// use hyperinterval::FixedInterval;
            /// // This format cannot represent the point 3, but raw [-5,7]/3
            /// // has tight outward bounds [-2,3] on the original 2^-63 grid.
            /// let x = FixedInterval::<i64, 63>::new(-5, 7)?;
            /// assert_eq!(x.div_integer(3)?.bounds(), Some((-2, 3)));
            /// # Ok::<(), hyperinterval::IntervalError>(())
            /// ```
            #[inline]
            pub fn div_integer(&self, divisor: i64) -> Result<Self, IntervalError> {
                Self::scale()?;
                if divisor == 0 {
                    return Err(IntervalError::ContainsZero);
                }
                let Some((a, b)) = self.bounds() else {
                    return Self::empty();
                };
                let (lower, upper, denominator) = if divisor < 0 {
                    (-i128::from(b), -i128::from(a), -i128::from(divisor))
                } else {
                    (i128::from(a), i128::from(b), i128::from(divisor))
                };
                Self::from_wide(
                    quotient_bounds(lower, denominator).0,
                    quotient_bounds(upper, denominator).1,
                )
            }

            /// Tight outward principal square root on this fixed-point lattice.
            /// Negative source members are excluded with `Partial` coverage;
            /// strictly negative nonempty input returns `EmptyDomain`. Empty
            /// maps to empty with `Complete` coverage. An unrepresentable upper
            /// bound returns `Overflow`, including rounding to one at a maximal
            /// fractional scale. Uses bounded integer arithmetic, no allocation.
            pub fn sqrt(&self) -> Result<RangeEnclosure<Self>, IntervalError> {
                Self::scale()?;
                let Some((a, b)) = self.bounds() else {
                    return Ok(RangeEnclosure {
                        range: Self::empty()?,
                        domain: DomainCoverage::Complete,
                    });
                };
                if b < 0 {
                    return Err(IntervalError::EmptyDomain);
                }
                let domain = if a < 0 {
                    DomainCoverage::Partial
                } else {
                    DomainCoverage::Complete
                };
                // sqrt(m / 2^FRAC) * 2^FRAC = sqrt(m * 2^FRAC).
                // At most 63 magnitude bits + 63 scale bits, fitting u128.
                let (lower, lower_ceil) =
                    crate::sqrt::integer_sqrt_bounds((a.max(0) as u128) << FRAC);
                let upper = if a == b {
                    lower_ceil
                } else {
                    crate::sqrt::integer_sqrt_bounds((b as u128) << FRAC).1
                };
                Ok(RangeEnclosure {
                    range: Self::from_wide(lower as i128, upper as i128)?,
                    domain,
                })
            }

            /// Tight zero-aware square enclosure.
            pub fn square(&self) -> Result<Self, IntervalError> {
                let scale = Self::scale()?;
                match self.bounds() {
                    Some((a, b)) => {
                        let aa = i128::from(a) * i128::from(a);
                        let bb = i128::from(b) * i128::from(b);
                        let lower = if a <= 0 && 0 <= b { 0 } else { aa.min(bb) };
                        Self::from_wide(
                            quotient_bounds(lower, scale).0,
                            quotient_bounds(aa.max(bb), scale).1,
                        )
                    }
                    None => Self::empty(),
                }
            }

            /// Outward enclosure of a nonnegative integer power.
            ///
            /// Uses at most 32 squarings and 32 products, with outward rounding
            /// at each step. This may be wider than the smallest lattice hull.
            /// Intermediate overflow is reported explicitly. For exponent zero,
            /// every nonempty input maps to one, including a zero singleton.
            pub fn checked_pow(&self, mut exponent: u32) -> Result<Self, IntervalError> {
                if self.is_empty() {
                    return Self::empty();
                }
                if exponent == 0 {
                    return Self::from_wide(Self::scale()?, Self::scale()?);
                }
                let mut base = *self;
                let mut result: Option<Self> = None;
                while exponent != 0 {
                    if exponent & 1 != 0 {
                        result = Some(match result {
                            Some(value) => value.checked_mul(&base)?,
                            None => base,
                        });
                    }
                    exponent >>= 1;
                    if exponent != 0 {
                        base = base.square()?;
                    }
                }
                // Nonzero initial exponent has at least one set bit.
                Ok(result.expect("nonzero exponent has a set bit"))
            }

            /// Tight outward affine image `coefficient * self + offset`.
            ///
            /// The coefficient and offset are lattice numerators. Accumulation
            /// happens before rounding/narrowing, preserving cancellation.
            pub fn affine(
                &self,
                coefficient: $endpoint,
                offset: $endpoint,
            ) -> Result<Self, IntervalError> {
                let scale = Self::scale()?;
                match self.bounds() {
                    Some((a, b)) => {
                        let a = i128::from(a) * i128::from(coefficient);
                        let b = i128::from(b) * i128::from(coefficient);
                        let offset = i128::from(offset) * scale;
                        Self::from_wide(
                            quotient_bounds(a.min(b) + offset, scale).0,
                            quotient_bounds(a.max(b) + offset, scale).1,
                        )
                    }
                    None => Self::empty(),
                }
            }
        }
    };
}

arithmetic!(i32);
arithmetic!(i64);
