use crate::IntervalError;
use core::cmp::Ordering;

// Private IEEE encoding authority. No native floating-point comparison is
// needed for finite validation, zero canonicalization or endpoint order.
#[derive(Clone, Copy)]
pub(crate) struct Format {
    sign: u64,
    infinity: u64,
}

pub(crate) const BINARY32: Format = Format {
    sign: 1 << 31,
    infinity: 0x7f80_0000,
};
pub(crate) const BINARY64: Format = Format {
    sign: 1 << 63,
    infinity: 0x7ff0_0000_0000_0000,
};

impl Format {
    // Keep the integer argument boundary opaque to the caller's float bitcast.
    // Inlining lets LLVM replace the magnitude-zero bit test with `fcmp`, which
    // misclassifies subnormals under DAZ. Release-mode MXCSR tests and generated
    // code inspection qualify this barrier on the supported compiler/target.
    #[inline(never)]
    pub(crate) fn finite(self, bits: u64) -> Result<u64, IntervalError> {
        if bits & self.infinity == self.infinity {
            Err(IntervalError::NonFiniteEndpoint)
        } else if bits & (self.sign - 1) == 0 {
            Ok(0)
        } else {
            Ok(bits)
        }
    }

    fn key(self, bits: u64) -> u64 {
        if bits & self.sign != 0 {
            self.sign - (bits & (self.sign - 1))
        } else {
            self.sign + bits
        }
    }

    pub(crate) fn compare(self, a: u64, b: u64) -> Ordering {
        self.key(a).cmp(&self.key(b))
    }
}

#[cfg(feature = "float-enclosure")]
#[derive(Clone, Copy)]
struct Dyadic {
    negative: bool,
    magnitude: u128,
    exponent: i32,
}

#[cfg(feature = "float-enclosure")]
impl Format {
    fn fraction(self) -> u32 {
        self.infinity.trailing_zeros()
    }
    fn bias(self) -> i32 {
        ((self.infinity >> self.fraction()) >> 1) as i32
    }
    fn minimum(self) -> i32 {
        1 - self.bias() - self.fraction() as i32
    }

    pub(crate) fn one(self) -> u64 {
        (self.bias() as u64) << self.fraction()
    }

    pub(crate) fn negate(self, bits: u64) -> u64 {
        if bits == 0 { 0 } else { bits ^ self.sign }
    }

    pub(crate) fn next_up(self, bits: u64) -> Result<u64, IntervalError> {
        let next = if bits & self.sign != 0 {
            bits - 1
        } else {
            bits + 1
        };
        self.finite(next).map_err(|_| IntervalError::Overflow)
    }

    pub(crate) fn next_down(self, bits: u64) -> Result<u64, IntervalError> {
        self.next_up(self.negate(bits))
            .map(|next| self.negate(next))
    }

    fn decode(self, bits: u64) -> Dyadic {
        let f = self.fraction();
        let field = (bits & self.infinity) >> f;
        let tail = bits & ((1 << f) - 1);
        Dyadic {
            negative: bits & self.sign != 0,
            magnitude: u128::from(if field == 0 { tail } else { tail | (1 << f) }),
            exponent: if field == 0 {
                self.minimum()
            } else {
                field as i32 - self.bias() - f as i32
            },
        }
    }

    #[cfg(feature = "dyadic")]
    pub(crate) fn exact_parts(self, bits: u64) -> (i64, i32) {
        let value = self.decode(bits);
        // All input significands have at most 53 bits, including their sign.
        let magnitude = value.magnitude as i64;
        (
            if value.negative {
                -magnitude
            } else {
                magnitude
            },
            value.exponent,
        )
    }

    // A coefficient on an output grid; magnitude has at most F+2 bits,
    // and any extra bit is the carry from rounding into the next binade.
    fn encode(self, magnitude: u128, grid: i32) -> Result<u64, IntervalError> {
        if magnitude == 0 {
            return Ok(0);
        }
        let high = 127 - magnitude.leading_zeros() as i32;
        let top = grid + high;
        if top > self.bias() {
            return Err(IntervalError::Overflow);
        }
        let f = self.fraction() as i32;
        if top < 1 - self.bias() {
            debug_assert!(grid >= self.minimum());
            return Ok((magnitude << (grid - self.minimum())) as u64);
        }
        let significand = if high <= f {
            magnitude << (f - high)
        } else {
            magnitude >> (high - f)
        };
        debug_assert!(high <= f || magnitude == 1u128 << high);
        Ok(((top + self.bias()) as u64) << f | ((significand as u64) & ((1u64 << f) - 1)))
    }

    // Principal root of a nonnegative finite endpoint. For x=m*2^e with
    // h=floor(log2(m)), sqrt(x) has top exponent floor((e+h)/2).
    // Its target grid g=top-F yields a nonnegative shift e-2g=2F-h+parity.
    // Thus m << (e-2g) has at most 2F+2 <= 106 bits. Every nonzero
    // binary32/64 root is normal, including roots of subnormal inputs.
    pub(crate) fn sqrt(self, bits: u64) -> Result<(u64, u64), IntervalError> {
        let value = self.decode(bits);
        debug_assert!(!value.negative);
        if value.magnitude == 0 {
            return Ok((0, 0));
        }
        let high = 127 - value.magnitude.leading_zeros() as i32;
        let grid = (value.exponent + high).div_euclid(2) - self.fraction() as i32;
        let shift = value.exponent - 2 * grid;
        let (lower, upper) = crate::sqrt::integer_sqrt_bounds(value.magnitude << shift);
        Ok((self.encode(lower, grid)?, self.encode(upper, grid)?))
    }

    fn signed(self, lower: u64, upper: u64, negative: bool) -> (u64, u64) {
        if negative {
            (self.negate(upper), self.negate(lower))
        } else {
            (lower, upper)
        }
    }

    fn rounded(self, value: Dyadic) -> Result<(u64, u64), IntervalError> {
        let m = value.magnitude;
        if m == 0 {
            return Ok((0, 0));
        }
        let top = value.exponent + 127 - m.leading_zeros() as i32;
        if top > self.bias() {
            return Err(IntervalError::Overflow);
        }
        let grid = self.minimum().max(top - self.fraction() as i32);
        let shift = grid - value.exponent;
        let (floor, remainder) = if shift >= 128 {
            (0, true)
        } else if shift > 0 {
            (m >> shift, m & ((1u128 << shift) - 1) != 0)
        } else {
            debug_assert!(m.leading_zeros() >= (-shift) as u32);
            (m << -shift, false)
        };
        let lower = self.encode(floor, grid)?;
        let upper = self.encode(floor + u128::from(remainder), grid)?;
        Ok(self.signed(lower, upper, value.negative))
    }

    pub(crate) fn add(self, a: u64, b: u64) -> Result<(u64, u64), IntervalError> {
        if a == 0 {
            return Ok((b, b));
        }
        if b == 0 {
            return Ok((a, a));
        }
        let normalize = |mut x: Dyadic| {
            let shift = self.fraction() + 1 - (128 - x.magnitude.leading_zeros());
            x.magnitude <<= shift;
            x.exponent -= shift as i32;
            x
        };
        let mut x = normalize(self.decode(a));
        let mut y = normalize(self.decode(b));
        let mut large = a;
        if x.exponent < y.exponent {
            core::mem::swap(&mut x, &mut y);
            large = b;
        }
        let gap = x.exponent - y.exponent;
        if gap > 74 {
            return if y.negative {
                Ok((self.next_down(large)?, large))
            } else {
                Ok((large, self.next_up(large)?))
            };
        }
        let xm = x.magnitude << gap;
        let (negative, magnitude) = if x.negative == y.negative {
            (x.negative, xm + y.magnitude)
        } else if xm >= y.magnitude {
            (x.negative, xm - y.magnitude)
        } else {
            (y.negative, y.magnitude - xm)
        };
        self.rounded(Dyadic {
            negative,
            magnitude,
            exponent: y.exponent,
        })
    }

    pub(crate) fn mul(self, a: u64, b: u64) -> Result<(u64, u64), IntervalError> {
        let x = self.decode(a);
        let y = self.decode(b);
        self.rounded(Dyadic {
            negative: x.negative != y.negative,
            magnitude: x.magnitude * y.magnitude,
            exponent: x.exponent + y.exponent,
        })
    }

    // Add two exact dyadics with at most 106 magnitude bits each. Choose a
    // common work grid giving the dominant operand bit 126, leaving one carry
    // bit. At most the smaller operand loses a fractional tail. A lost tail
    // implies a top-bit separation of at least 21, so cancellation cannot make
    // the result small or its IEEE spacing finer than the work grid. Therefore
    // the tail can change the final bounds only at an exact IEEE grid point.
    fn add_wide(self, x: Dyadic, y: Dyadic) -> Result<(u64, u64), IntervalError> {
        if x.magnitude == 0 {
            return self.rounded(y);
        }
        if y.magnitude == 0 {
            return self.rounded(x);
        }
        let top = |v: Dyadic| v.exponent + 127 - v.magnitude.leading_zeros() as i32;
        let grid = top(x).max(top(y)) - 126;
        let align = |v: Dyadic| {
            let shift = v.exponent - grid;
            let (magnitude, lost) = if shift >= 0 {
                debug_assert!(shift < 128 && v.magnitude.leading_zeros() >= shift as u32);
                (v.magnitude << shift, false)
            } else if -shift >= 128 {
                (0, true)
            } else {
                (
                    v.magnitude >> -shift,
                    v.magnitude & ((1u128 << -shift) - 1) != 0,
                )
            };
            (
                magnitude,
                if !lost {
                    0
                } else if v.negative {
                    -1
                } else {
                    1
                },
            )
        };
        let (a, atail) = align(x);
        let (b, btail) = align(y);
        debug_assert!(atail == 0 || btail == 0);
        let (negative, magnitude) = if x.negative == y.negative {
            (x.negative, a + b)
        } else if a >= b {
            (x.negative, a - b)
        } else {
            (y.negative, b - a)
        };
        let (lo, hi) = self.rounded(Dyadic {
            negative,
            magnitude,
            exponent: grid,
        })?;
        match (lo == hi, atail + btail) {
            (true, -1) => Ok((self.next_down(lo)?, hi)),
            (true, 1) => Ok((lo, self.next_up(hi)?)),
            _ => Ok((lo, hi)),
        }
    }

    pub(crate) fn affine(
        self,
        x: u64,
        scale: u64,
        offset: u64,
    ) -> Result<(u64, u64), IntervalError> {
        let x = self.decode(x);
        let scale = self.decode(scale);
        self.add_wide(
            Dyadic {
                negative: x.negative != scale.negative,
                magnitude: x.magnitude * scale.magnitude,
                exponent: x.exponent + scale.exponent,
            },
            self.decode(offset),
        )
    }

    pub(crate) fn div(self, a: u64, b: u64) -> Result<(u64, u64), IntervalError> {
        if b == 0 {
            return Err(IntervalError::ContainsZero);
        }
        if a == 0 {
            return Ok((0, 0));
        }
        let x = self.decode(a);
        let y = self.decode(b);
        let mut order = y.magnitude.leading_zeros() as i32 - x.magnitude.leading_zeros() as i32;
        let below = if order >= 0 {
            x.magnitude < y.magnitude << order
        } else {
            x.magnitude << -order < y.magnitude
        };
        if below {
            order -= 1;
        }
        let exponent = x.exponent - y.exponent;
        let top = exponent + order;
        if top > self.bias() {
            return Err(IntervalError::Overflow);
        }
        let grid = self.minimum().max(top - self.fraction() as i32);
        let shift = exponent - grid;
        let (floor, remainder) = if shift >= 0 {
            debug_assert!(shift < 128 && x.magnitude.leading_zeros() >= shift as u32);
            let n = x.magnitude << shift;
            (n / y.magnitude, !n.is_multiple_of(y.magnitude))
        } else if -shift > y.magnitude.leading_zeros() as i32 {
            (0, true)
        } else {
            let d = y.magnitude << -shift;
            (x.magnitude / d, !x.magnitude.is_multiple_of(d))
        };
        Ok(self.signed(
            self.encode(floor, grid)?,
            self.encode(floor + u128::from(remainder), grid)?,
            x.negative != y.negative,
        ))
    }

    pub(crate) fn power(self, bits: u64, mut exponent: u32) -> Result<(u64, u64), IntervalError> {
        let negative = bits & self.sign != 0 && !exponent.is_multiple_of(2);
        let mut base = (bits & (self.sign - 1), bits & (self.sign - 1));
        let mut result = (self.one(), self.one());
        while exponent != 0 {
            if exponent & 1 != 0 {
                result = (self.mul(result.0, base.0)?.0, self.mul(result.1, base.1)?.1);
            }
            exponent >>= 1;
            if exponent != 0 {
                base = (self.mul(base.0, base.0)?.0, self.mul(base.1, base.1)?.1);
            }
        }
        Ok(self.signed(result.0, result.1, negative))
    }
}

#[cfg(all(test, feature = "float-enclosure"))]
#[path = "ieee_tests.rs"]
mod tests;
