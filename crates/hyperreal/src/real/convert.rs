use crate::{Rational, Real, RealProblem};
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

#[cfg(test)]
mod tests {
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
}
