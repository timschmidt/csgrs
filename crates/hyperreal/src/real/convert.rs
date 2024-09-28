use crate::{Computable, Rational, Real, RealProblem};
use num::{BigInt, BigUint, One};

impl From<i64> for Real {
    fn from(n: i64) -> Real {
        Real::new(Rational::new(n))
    }
}

impl From<i32> for Real {
    fn from(n: i32) -> Real {
        Real::new(Rational::new(n.into()))
    }
}

impl From<Rational> for Real {
    fn from(rational: Rational) -> Real {
        Real::new(rational)
    }
}

fn signed(n: Rational, neg: bool) -> Real {
    if neg {
        Real::new(-n)
    } else {
        Real::new(n)
    }
}

impl TryFrom<f32> for Real {
    type Error = RealProblem;

    fn try_from(n: f32) -> Result<Real, Self::Error> {
        const NEG_BITS: u32 = 0x8000_0000;
        const EXP_BITS: u32 = 0x7f80_0000;
        const SIG_BITS: u32 = 0x007f_ffff;
        debug_assert_eq!(NEG_BITS + EXP_BITS + SIG_BITS, u32::MAX);

        let bits = n.to_bits();
        let neg = (bits & NEG_BITS) == NEG_BITS;
        let exp = (bits & EXP_BITS) >> EXP_BITS.trailing_zeros();
        let sig = bits & SIG_BITS;
        match exp {
            0 => {
                if sig == 0 {
                    Ok(Real::new(Rational::zero()))
                } else {
                    let numerator: BigInt = sig.into();
                    let denominator = BigUint::one() << 149;
                    Ok(signed(
                        Rational::from_bigint_fraction(numerator, denominator),
                        neg,
                    ))
                }
            }
            1..=150 => {
                let n = SIG_BITS + 1 + sig;
                let numerator: BigInt = n.into();
                let denominator = BigUint::one() << (150 - exp);
                Ok(signed(
                    Rational::from_bigint_fraction(numerator, denominator),
                    neg,
                ))
            }
            151..=254 => {
                let n = SIG_BITS + 1 + sig;
                let mut big: BigInt = n.into();
                big <<= exp - 150;
                Ok(signed(Rational::from_bigint(big), neg))
            }
            255 => {
                if sig == 0 {
                    Err(RealProblem::Infinity)
                } else {
                    Err(RealProblem::NotANumber)
                }
            }
            _ => unreachable!(),
        }
    }
}

impl TryFrom<f64> for Real {
    type Error = RealProblem;

    fn try_from(n: f64) -> Result<Real, Self::Error> {
        const NEG_BITS: u64 = 0x8000_0000_0000_0000;
        const EXP_BITS: u64 = 0x7ff0_0000_0000_0000;
        const SIG_BITS: u64 = 0x000f_ffff_ffff_ffff;
        debug_assert_eq!(NEG_BITS + EXP_BITS + SIG_BITS, u64::MAX);

        let bits = n.to_bits();
        let neg = (bits & NEG_BITS) == NEG_BITS;
        let exp = (bits & EXP_BITS) >> EXP_BITS.trailing_zeros();
        let sig = bits & SIG_BITS;
        match exp {
            0 => {
                if sig == 0 {
                    Ok(Real::new(Rational::zero()))
                } else {
                    let numerator: BigInt = sig.into();
                    let denominator = BigUint::one() << 1074;
                    Ok(signed(
                        Rational::from_bigint_fraction(numerator, denominator),
                        neg,
                    ))
                }
            }
            1..=1075 => {
                let n = SIG_BITS + 1 + sig;
                let numerator: BigInt = n.into();
                let denominator = BigUint::one() << (1075 - exp);
                Ok(signed(
                    Rational::from_bigint_fraction(numerator, denominator),
                    neg,
                ))
            }
            1076..=2046 => {
                let n = SIG_BITS + 1 + sig;
                let mut big: BigInt = n.into();
                big <<= exp - 1075;
                Ok(signed(Rational::from_bigint(big), neg))
            }
            2047 => {
                if sig == 0 {
                    Err(RealProblem::Infinity)
                } else {
                    Err(RealProblem::NotANumber)
                }
            }
            _ => unreachable!(),
        }
    }
}

impl Real {
    fn fold(self) -> Computable {
        let rat = Computable::rational(self.rational);
        rat.multiply(self.computable)
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

        let msd = c.iter_msd();
        if msd < -150 {
            return match neg {
                0 => 0.0,
                1 => -0.0,
                _ => unreachable!(),
            };
        }
        if msd > 127 {
            return match neg {
                0 => f32::INFINITY,
                1 => f32::NEG_INFINITY,
                _ => unreachable!(),
            };
        }
        let (sig, exp): (u32, u32) = if msd < -126 {
            (c.approx(-149).try_into().unwrap(), 0)
        } else {
            (c.approx(msd - 23).try_into().unwrap(), (127 + msd) as u32)
        };
        let sig_bits = sig & SIG_BITS;
        let neg_bits: u32 = neg << NEG_BITS.trailing_zeros();
        let exp_bits: u32 = exp << EXP_BITS.trailing_zeros();
        let bits = neg_bits | exp_bits | sig_bits;
        f32::from_bits(bits)
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

        let msd = c.iter_msd();
        if msd < -1075 {
            return match neg {
                0 => 0.0,
                1 => -0.0,
                _ => unreachable!(),
            };
        }
        if msd > 1023 {
            return match neg {
                0 => f64::INFINITY,
                1 => f64::NEG_INFINITY,
                _ => unreachable!(),
            };
        }
        let (sig, exp): (u64, u64) = if msd < -1022 {
            (c.approx(-1074).try_into().unwrap(), 0)
        } else {
            (c.approx(msd - 52).try_into().unwrap(), (1023 + msd) as u64)
        };
        let sig_bits = sig & SIG_BITS;
        let neg_bits: u64 = neg << NEG_BITS.trailing_zeros();
        let exp_bits: u64 = exp << EXP_BITS.trailing_zeros();
        let bits = neg_bits | exp_bits | sig_bits;
        f64::from_bits(bits)
    }
}

#[cfg(test)]
mod tests {
    use num::bigint::ToBigInt;

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
    fn rational() {
        let f: f32 = 27.0;
        let d: f32 = 81.0;
        let a: Real = f.try_into().unwrap();
        let b: Real = d.try_into().unwrap();
        let third = Real::new(Rational::fraction(1, 3));
        let answer = (a / b).unwrap();
        assert_eq!(answer, third);
    }

    #[test]
    fn repr_f32() {
        let f: f32 = 1.23456789;
        let a: Real = f.try_into().unwrap();
        let correct = Real::new(Rational::fraction(5178153, 4194304));
        assert_eq!(a, correct);
    }

    #[test]
    fn repr_f64() {
        let f: f64 = 1.23456789;
        let a: Real = f.try_into().unwrap();
        let correct = Real::new(Rational::fraction(5559999489367579, 4503599627370496));
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

    #[test]
    fn big_roundtrip() {
        let before = f32::MAX;
        let mid: Real = before.try_into().unwrap();
        let after: f32 = mid.into();
        assert_eq!(before, after);
        let before = f64::MAX;
        let mid: Real = before.try_into().unwrap();
        let after: f64 = mid.into();
        assert_eq!(before, after);
    }

    #[test]
    fn arbitrary_roundtrip() {
        let before = 0.123456789_f32;
        let mid: Real = before.try_into().unwrap();
        let after: f32 = mid.into();
        assert_eq!(before, after);
        let before = 987654321_f32;
        let mid: Real = before.try_into().unwrap();
        let after: f32 = mid.into();
        assert_eq!(before, after);
        let before = 0.123456789_f64;
        let mid: Real = before.try_into().unwrap();
        let after: f64 = mid.into();
        assert_eq!(before, after);
        let before = 987654321_f64;
        let mid: Real = before.try_into().unwrap();
        let after: f64 = mid.into();
        assert_eq!(before, after);
    }

    #[test]
    fn small_roundtrip() {
        let before = f32::MIN_POSITIVE;
        let mid: Real = before.try_into().unwrap();
        let after: f32 = mid.into();
        assert_eq!(before, after);
        let before = f64::MIN_POSITIVE;
        let mid: Real = before.try_into().unwrap();
        let after: f64 = mid.into();
        assert_eq!(before, after);
    }

    #[test]
    fn subnormal_roundtrip_64() {
        let before = 1.234e-310_f64;
        assert_ne!(before, 0.0);
        let mid: Real = before.try_into().unwrap();
        let after: f64 = mid.into();
        assert_eq!(before, after);
    }

    #[test]
    fn subnormal_roundtrip_32() {
        let before = 1.234e-41_f32;
        assert_ne!(before, 0.0);
        let mid: Real = before.try_into().unwrap();
        let after: f32 = mid.into();
        assert_eq!(before, after);
    }

    #[test]
    fn pi() {
        let f: f32 = Real::pi().into();
        assert_eq!(f, std::f32::consts::PI);
        let f: f64 = Real::pi().into();
        assert_eq!(f, std::f64::consts::PI);
    }
}
