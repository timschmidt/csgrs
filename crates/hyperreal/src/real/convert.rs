use crate::RealSign;
use crate::{Computable, Problem, Rational, Real};

macro_rules! impl_integer_conversion {
    ($T:ty) => {
        impl From<$T> for Real {
            #[inline]
            fn from(n: $T) -> Real {
                // Integer identity conversion is a public hot path through
                // hyperlattice and hyperlimit. Keep 0 and 1 on dedicated
                // Real constructors instead of paying BigInt rational import.
                if n == 0 {
                    return Real::zero();
                }
                if n == 1 {
                    return Real::one();
                }
                // Let Rational pick the signed/unsigned primitive path; it
                // avoids materializing a BigInt just to build Real::new.
                Real::new(Rational::from(n))
            }
        }
    };
}

impl_integer_conversion!(i8);
impl_integer_conversion!(i16);
impl_integer_conversion!(i32);
impl_integer_conversion!(i64);
impl_integer_conversion!(i128);
impl_integer_conversion!(u8);
impl_integer_conversion!(u16);
impl_integer_conversion!(u32);
impl_integer_conversion!(u64);
impl_integer_conversion!(u128);

impl From<Rational> for Real {
    fn from(rational: Rational) -> Real {
        Real::new(rational)
    }
}

impl TryFrom<f32> for Real {
    type Error = Problem;

    fn try_from(n: f32) -> Result<Real, Self::Error> {
        // Import floats as exact dyadic rationals. That preserves structural
        // facts and avoids immediately lowering user data to Computable nodes.
        let rational: Rational = n.try_into()?;
        Ok(Real::new(rational))
    }
}

impl TryFrom<f64> for Real {
    type Error = Problem;

    fn try_from(n: f64) -> Result<Real, Self::Error> {
        // Same exact dyadic import as f32; the public Real constructor keeps the
        // value in the rational class so matrix inputs stay cheap.
        let rational: Rational = n.try_into()?;
        Ok(Real::new(rational))
    }
}

impl Real {
    #[inline]
    pub(crate) fn fold_ref(&self) -> Computable {
        use crate::real::Class;

        // Keep the rational scale separate until a generic computable kernel is
        // unavoidable. Folding `a * class` eagerly would erase exact classes
        // that sign, sqrt, log, and trig shortcuts can still exploit.
        let mut c = if self.rational.is_one() {
            self.computable_clone()
        } else if self.class == Class::One {
            Computable::rational(self.rational.clone())
        } else {
            self.computable_clone()
                .multiply_rational(self.rational.clone())
        };

        if let Some(s) = &self.signal {
            c.abort(s.clone());
        }
        c
    }

    #[inline]
    pub(crate) fn fold(self) -> Computable {
        // Owned folding mirrors `fold_ref` but moves the computable when the
        // rational scale is one; scalar transcendental kernels hit this path.
        let crate::Real {
            rational,
            class,
            computable,
            signal,
        } = self;
        if rational.is_one() {
            let mut c = computable.unwrap_or_else(Computable::one);
            if let Some(s) = signal {
                c.abort(s.clone());
            }
            c
        } else if class == crate::real::Class::One {
            Computable::rational(rational)
        } else {
            let mut c = computable
                .unwrap_or_else(Computable::one)
                .multiply_rational(rational);
            if let Some(s) = signal {
                c.abort(s);
            }
            c
        }
    }
}

use crate::computable::Precision;

// (Significand, Exponent)
fn sig_exp_32(c: Computable, mut msd: Precision) -> (u32, u32) {
    const SIG_BITS: u32 = 0x007f_ffff;
    const OVERSIZE: u32 = SIG_BITS.next_power_of_two() << 1;

    if msd <= -126 {
        // Subnormal output needs the fixed minimum precision, independent of
        // the discovered MSD.
        let sig = c
            .approx(-149)
            .magnitude()
            .try_into()
            .expect("Magnitude of the top bits should fit in a u32");
        // It is possible for that top bit to be set, so we're not a denormal
        if sig > SIG_BITS {
            (sig & SIG_BITS, 1)
        } else {
            (sig, 0)
        }
    } else {
        // Normal output requests just enough bits for the f32 significand, then
        // repairs the exponent if rounding carried into a new top bit.
        let mut sig: u32 = c
            .approx(msd - 24)
            .magnitude()
            .try_into()
            .expect("Magnitude of the top bits should fit in a u32");
        // Almost (but not quite) two orders of binary magnitude range
        while sig >= OVERSIZE {
            msd += 1;
            sig >>= 1;
        }
        (sig & SIG_BITS, (126 + msd) as u32)
    }
}

impl From<Real> for f32 {
    fn from(r: Real) -> f32 {
        use num::bigint::Sign::*;

        const NEG_BITS: u32 = 0x8000_0000;
        const EXP_BITS: u32 = 0x7f80_0000;
        const SIG_BITS: u32 = 0x007f_ffff;
        debug_assert_eq!(NEG_BITS + EXP_BITS + SIG_BITS, u32::MAX);

        let c = r.fold();
        let neg = match c.sign() {
            NoSign => {
                return 0.0;
            }
            Plus => 0,
            Minus => 1,
        };

        let Some(msd) = c.iter_msd_stop(-150) else {
            // Below the f32 subnormal floor, round to signed zero.
            return match neg {
                0 => 0.0,
                1 => -0.0,
                _ => unreachable!(),
            };
        };
        if msd > 127 {
            // Above the finite f32 range, saturate to signed infinity.
            return match neg {
                0 => f32::INFINITY,
                1 => f32::NEG_INFINITY,
                _ => unreachable!(),
            };
        }
        let (sig_bits, exp) = sig_exp_32(c, msd);
        let neg_bits: u32 = neg << NEG_BITS.trailing_zeros();
        let exp_bits: u32 = exp << EXP_BITS.trailing_zeros();
        let bits = neg_bits | exp_bits | sig_bits;
        f32::from_bits(bits)
    }
}

// (Significand, Exponent)
fn sig_exp_64(c: Computable, mut msd: Precision) -> (u64, u64) {
    const SIG_BITS: u64 = 0x000f_ffff_ffff_ffff;
    const OVERSIZE: u64 = SIG_BITS.next_power_of_two() << 1;

    if msd <= -1022 {
        // Subnormal f64 path mirrors f32 with the wider significand and lower
        // minimum precision.
        let sig = c
            .approx(-1074)
            .magnitude()
            .try_into()
            .expect("Magnitude of the top bits should fit in a u64");
        if sig > SIG_BITS {
            (sig & SIG_BITS, 1)
        } else {
            (sig, 0)
        }
    } else {
        // Normal f64 path requests 53 useful bits and handles one-bit carry from
        // rounding by shifting the significand and bumping the exponent.
        let mut sig: u64 = c
            .approx(msd - 53)
            .magnitude()
            .try_into()
            .expect("Magnitude of the top bits should fit in a u64");
        // Almost (but not quite) two orders of binary magnitude range
        while sig >= OVERSIZE {
            msd += 1;
            sig >>= 1;
        }
        (sig & SIG_BITS, (1022 + msd) as u64)
    }
}

impl From<Real> for f64 {
    fn from(r: Real) -> f64 {
        use num::bigint::Sign::*;

        const NEG_BITS: u64 = 0x8000_0000_0000_0000;
        const EXP_BITS: u64 = 0x7ff0_0000_0000_0000;
        const SIG_BITS: u64 = 0x000f_ffff_ffff_ffff;
        debug_assert_eq!(NEG_BITS + EXP_BITS + SIG_BITS, u64::MAX);

        let c = r.fold();
        let neg = match c.sign() {
            NoSign => {
                return 0.0;
            }
            Plus => 0,
            Minus => 1,
        };

        let Some(msd) = c.iter_msd_stop(-1075) else {
            // Too small for f64, including subnormal precision.
            return match neg {
                0 => 0.0,
                1 => -0.0,
                _ => unreachable!(),
            };
        };
        if msd > 1023 {
            // Too large for finite f64.
            return match neg {
                0 => f64::INFINITY,
                1 => f64::NEG_INFINITY,
                _ => unreachable!(),
            };
        }
        let (sig_bits, exp) = sig_exp_64(c, msd);
        let neg_bits: u64 = neg << NEG_BITS.trailing_zeros();
        let exp_bits: u64 = exp << EXP_BITS.trailing_zeros();
        let bits = neg_bits | exp_bits | sig_bits;
        f64::from_bits(bits)
    }
}

impl Real {
    /// Return a finite borrowed `f64` approximation, or `None` on overflow.
    #[inline]
    pub fn to_f64_approx(&self) -> Option<f64> {
        const NEG_BITS: u64 = 0x8000_0000_0000_0000;
        const EXP_BITS: u64 = 0x7ff0_0000_0000_0000;

        if matches!(self.class, crate::real::Class::One)
            && let fast @ Some(_) = self.rational.to_f64_approx()
        {
            // Exact rationals can often be rounded to f64 without touching the
            // lazy computable tree. This matters for matrix/predicate code that
            // asks for approximate centers of plain scalar data.
            return fast;
        }

        let c = self.fold_ref();
        let sign = match self.refine_sign_until(-1075) {
            // Borrowed conversion refuses to do unbounded refinement for sign.
            // Returning signed zero here keeps approximate-center users from
            // accidentally forcing exact evaluation of unresolved expressions.
            Some(sign) => sign,
            None => return Some(0.0),
        };
        let neg = match sign {
            RealSign::Zero => return Some(0.0),
            RealSign::Positive => 0,
            RealSign::Negative => 1,
        };

        let Some(msd) = c.iter_msd_stop(-1075) else {
            // Magnitude below f64's minimum representable scale.
            return Some(0.0);
        };
        if msd > 1023 {
            // Unlike `From<Real> for f64`, borrowed approximate conversion
            // reports overflow as None so callers can distinguish saturation.
            return None;
        }
        let (sig_bits, exp) = sig_exp_64(c, msd);
        let neg_bits: u64 = neg << NEG_BITS.trailing_zeros();
        let exp_bits: u64 = exp << EXP_BITS.trailing_zeros();
        let bits = neg_bits | exp_bits | sig_bits;
        let value = f64::from_bits(bits);
        value.is_finite().then_some(value)
    }
}

#[cfg(test)]
mod tests {
    use num::bigint::ToBigInt;
    use num::{BigInt, One};

    use super::*;

    #[test]
    fn zero() {
        let f: f32 = 0.0;
        let d: f64 = 0.0;
        let a: Real = f.try_into().unwrap();
        let b: Real = d.try_into().unwrap();
        let zero = Real::zero();
        assert_eq!(a, zero);
        assert_eq!(b, zero);
    }

    #[test]
    fn infinity() {
        let f = f32::INFINITY;
        let d = f64::NEG_INFINITY;
        let a: Problem = <f32 as TryInto<Real>>::try_into(f).unwrap_err();
        let b: Problem = <f64 as TryInto<Real>>::try_into(d).unwrap_err();
        assert_eq!(a, Problem::Infinity);
        assert_eq!(b, Problem::Infinity);
    }

    #[test]
    fn nans() {
        let f = f32::NAN;
        let d = f64::NAN;
        let a: Problem = <f32 as TryInto<Real>>::try_into(f).unwrap_err();
        let b: Problem = <f64 as TryInto<Real>>::try_into(d).unwrap_err();
        assert_eq!(a, Problem::NotANumber);
        assert_eq!(b, Problem::NotANumber);
    }

    #[test]
    fn half_to_float() {
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        let f: f32 = half.clone().into();
        let d: f64 = half.into();
        assert_eq!(f, 0.5);
        assert_eq!(d, 0.5);
    }

    #[test]
    fn half_from_float() {
        let half = 0.5_f32;
        let correct = Real::new(Rational::fraction(1, 2).unwrap());
        let answer: Real = half.try_into().unwrap();
        assert_eq!(answer, correct);
        let half = 0.5_f64;
        let correct = Real::new(Rational::fraction(1, 2).unwrap());
        let answer: Real = half.try_into().unwrap();
        assert_eq!(answer, correct);
    }

    #[test]
    fn negative_half() {
        let half = Real::new(Rational::fraction(-1, 2).unwrap());
        let f: f32 = half.clone().into();
        let d: f64 = half.into();
        assert_eq!(f, -0.5);
        assert_eq!(d, -0.5);
    }

    #[test]
    fn rational() {
        let f: f32 = 27.0;
        let d: f32 = 81.0;
        let a: Real = f.try_into().unwrap();
        let b: Real = d.try_into().unwrap();
        let third = Real::new(Rational::fraction(1, 3).unwrap());
        let answer = (a / b).unwrap();
        assert_eq!(answer, third);
    }

    #[test]
    fn too_small() {
        let r: Real = f32::from_bits(1).try_into().unwrap();
        let s = r * Real::new(Rational::fraction(1, 3).unwrap());
        let f: f32 = s.into();
        assert_eq!(f, 0.0_f32);
        let r: Real = f64::from_bits(1).try_into().unwrap();
        let s = r * Real::new(Rational::fraction(1, 3).unwrap());
        let f: f64 = s.into();
        assert_eq!(f, 0.0_f64);
    }

    #[test]
    fn repr_f32() {
        let f: f32 = 1.234_567_9;
        let a: Real = f.try_into().unwrap();
        let correct = Real::new(Rational::fraction(5178153, 4194304).unwrap());
        assert_eq!(a, correct);
    }

    #[test]
    fn repr_f64() {
        let f: f64 = 1.23456789;
        let a: Real = f.try_into().unwrap();
        let correct = Real::new(Rational::fraction(5559999489367579, 4503599627370496).unwrap());
        assert_eq!(a, correct);
    }

    #[test]
    fn fold_nine() {
        let nine = Real::new(Rational::new(9));
        let c = nine.fold();
        assert_eq!(c.approx(3), BigInt::one());
        let nine: BigInt = ToBigInt::to_bigint(&9).unwrap();
        assert_eq!(c.approx(0), nine);
    }

    #[test]
    fn zero_roundtrip() {
        let zero = 0.0_f32;
        let zero: Real = zero.try_into().unwrap();
        assert_eq!(zero, Real::zero());
        let zero: f32 = zero.into();
        assert_eq!(zero, 0.0);
        let zero = 0.0_f64;
        let zero: Real = zero.try_into().unwrap();
        assert_eq!(zero, Real::zero());
        let zero: f64 = zero.into();
        assert_eq!(zero, 0.0);
    }

    fn roundtrip<T>(f: T) -> T
    where
        T: TryInto<Real> + From<Real>,
        <T as TryInto<Real>>::Error: std::fmt::Debug,
    {
        let mid: Real = f.try_into().unwrap();
        mid.into()
    }

    #[test]
    fn big_roundtrip() {
        assert_eq!(f32::MAX, roundtrip(f32::MAX));
        assert_eq!(f64::MAX, roundtrip(f64::MAX));
        assert_eq!(f32::MIN, roundtrip(f32::MIN));
        assert_eq!(f64::MIN, roundtrip(f64::MIN));
    }

    #[test]
    fn small_roundtrip() {
        assert_eq!(f32::MIN_POSITIVE * 3.0, roundtrip(f32::MIN_POSITIVE * 3.0));
        assert_eq!(f64::MIN_POSITIVE * 3.0, roundtrip(f64::MIN_POSITIVE * 3.0));
        assert_eq!(f32::MIN_POSITIVE, roundtrip(f32::MIN_POSITIVE));
        assert_eq!(f64::MIN_POSITIVE, roundtrip(f64::MIN_POSITIVE));
    }

    #[test]
    fn arbitrary_roundtrip() {
        assert_eq!(0.123_456_79_f32, roundtrip(0.123_456_79_f32));
        assert_eq!(987654321_f32, roundtrip(987654321_f32));
        assert_eq!(0.123456789_f64, roundtrip(0.123456789_f64));
        assert_eq!(987654321_f64, roundtrip(987654321_f64));
    }

    #[test]
    fn almost_two() {
        // Largest f32 which is smaller than two
        let h = f32::from_bits(0x3fff_ffff);
        assert_eq!(format!("{h:#.7}"), "1.9999999");
        let r: Real = h.try_into().unwrap();
        assert_eq!(format!("{r:#.7}"), "1.9999999");
        let j: f32 = r.into();
        assert_eq!(h, j);
        // Largest f64 which is smaller than two
        let h = f64::from_bits(0x3fff_ffff_ffff_ffff);
        assert_eq!(format!("{h:#.16}"), "1.9999999999999998");
        let r: Real = h.try_into().unwrap();
        assert_eq!(format!("{r:#.16}"), "1.9999999999999998");
        let j: f64 = r.into();
        assert_eq!(h, j);
    }

    #[test]
    fn subnormal_roundtrip() {
        let before = 1.234e-310_f64;
        assert_ne!(before, 0.0);
        assert_eq!(before, roundtrip(before));
        let before = 1.234e-41_f32;
        assert_ne!(before, 0.0);
        assert_eq!(before, roundtrip(before));
        // Large but still subnormal
        let sub = f32::from_bits(0x7c0000);
        assert_eq!(sub, roundtrip(sub));
        let sub = f64::from_bits(0x000f_ffff_0000_0000);
        assert_eq!(sub, roundtrip(sub));
    }

    // Sometimes during conversion the approximation fits without shifting, that's fine
    // but none of our other tests for f32 catch that
    #[test]
    fn bit_conversion() {
        let r = Real::new(Rational::fraction(2, 5).unwrap()) * Real::pi();
        let f: f32 = r.sin().into();
        assert_eq!(f, 0.951_056_54);
    }

    // Our Pi isn't exactly equal to the IEEE approximations since it's more accurate
    #[test]
    fn pi() {
        let f: f32 = Real::pi().into();
        assert!(std::f32::consts::PI.to_bits().abs_diff(f.to_bits()) < 2);
        let f: f64 = Real::pi().into();
        assert!(std::f64::consts::PI.to_bits().abs_diff(f.to_bits()) < 2);
    }

    #[test]
    fn max_u64_f32() {
        let max_u64: Rational = u64::MAX.into();
        let r = Real::new(max_u64);
        let f: f32 = r.into();
        assert_eq!(f, u64::MAX as f32);
    }

    #[test]
    fn max_u64_f64() {
        let max_u64: Rational = u64::MAX.into();
        let r = Real::new(max_u64);
        let d: f64 = r.into();
        assert_eq!(d, u64::MAX as f64);
    }

    #[test]
    fn borrowed_f64_approx_finite_values() {
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        assert_eq!(half.to_f64_approx(), Some(0.5));

        let one_third = Real::new(Rational::fraction(1, 3).unwrap());
        assert_eq!(one_third.to_f64_approx(), Some(1.0 / 3.0));

        let pi = Real::pi().to_f64_approx().unwrap();
        assert!(std::f64::consts::PI.to_bits().abs_diff(pi.to_bits()) < 2);
    }

    #[test]
    fn borrowed_f64_approx_underflow_and_overflow() {
        let tiny = Real::new(
            Rational::from_bigint_fraction(BigInt::from(1), num::BigUint::from(1_u8) << 1200)
                .unwrap(),
        );
        assert_eq!(tiny.to_f64_approx(), Some(0.0));

        let negative_tiny = -tiny;
        assert_eq!(negative_tiny.to_f64_approx(), Some(0.0));

        let huge = Real::new(Rational::from_bigint(BigInt::from(1_u8) << 1200));
        assert_eq!(huge.to_f64_approx(), None);

        let negative_huge = -huge;
        assert_eq!(negative_huge.to_f64_approx(), None);
    }

    #[test]
    fn borrowed_f64_approx_tracks_negative_finite_values() {
        let value = -(Real::new(Rational::new(2)).sqrt().unwrap());
        let approx = value.to_f64_approx().unwrap();
        assert!(approx.is_sign_negative());
        assert!((approx + std::f64::consts::SQRT_2).abs() < 1e-15);
    }
}
