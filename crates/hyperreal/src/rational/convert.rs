use crate::Rational;
use num::{BigInt, BigUint, One};

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum FloatProblem {
    NotANumber,
    Infinity,
}

// TODO: implement Error

fn signed(n: Rational, neg: bool) -> Rational {
    if neg {
        -n
    } else {
        n
    }
}

impl TryFrom<f32> for Rational {
    type Error = FloatProblem;

    fn try_from(n: f32) -> Result<Rational, Self::Error> {
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
                    Ok(Rational::zero())
                } else {
                    let numerator: BigInt = sig.into();
                    let denominator = BigUint::one() << 149;
                    Ok(signed(
                        Rational::from_bigint_fraction(numerator, denominator).unwrap(),
                        neg,
                    ))
                }
            }
            1..=150 => {
                let n = SIG_BITS + 1 + sig;
                let numerator: BigInt = n.into();
                let denominator = BigUint::one() << (150 - exp);
                Ok(signed(
                    Rational::from_bigint_fraction(numerator, denominator).unwrap(),
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
                    Err(FloatProblem::Infinity)
                } else {
                    Err(FloatProblem::NotANumber)
                }
            }
            _ => unreachable!(),
        }
    }
}

impl TryFrom<f64> for Rational {
    type Error = FloatProblem;

    fn try_from(n: f64) -> Result<Rational, Self::Error> {
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
                    Ok(Rational::zero())
                } else {
                    let numerator: BigInt = sig.into();
                    let denominator = BigUint::one() << 1074;
                    Ok(signed(
                        Rational::from_bigint_fraction(numerator, denominator).unwrap(),
                        neg,
                    ))
                }
            }
            1..=1075 => {
                let n = SIG_BITS + 1 + sig;
                let numerator: BigInt = n.into();
                let denominator = BigUint::one() << (1075 - exp);
                Ok(signed(
                    Rational::from_bigint_fraction(numerator, denominator).unwrap(),
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
                    Err(FloatProblem::Infinity)
                } else {
                    Err(FloatProblem::NotANumber)
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
        let a: Rational = f.try_into().unwrap();
        let b: Rational = d.try_into().unwrap();
        let zero = Rational::zero();
        assert_eq!(a, zero);
        assert_eq!(b, zero);
    }

    #[test]
    fn half_from_float() {
        let half = 0.5_f32;
        let correct = Rational::fraction(1, 2).unwrap();
        let answer: Rational = half.try_into().unwrap();
        assert_eq!(answer, correct);
        let half = 0.5_f64;
        let answer: Rational = half.try_into().unwrap();
        assert_eq!(answer, correct);
    }

    #[test]
    fn repr_f32() {
        let f: f32 = 1.23456789;
        let a: Rational = f.try_into().unwrap();
        let correct = Rational::fraction(5178153, 4194304).unwrap();
        assert_eq!(a, correct);
    }

    #[test]
    fn repr_f64() {
        let f: f64 = 1.23456789;
        let a: Rational = f.try_into().unwrap();
        let correct = Rational::fraction(5559999489367579, 4503599627370496).unwrap();
        assert_eq!(a, correct);
    }
}
