use crate::{Problem, Rational};
use num::bigint::ToBigInt;
use num::{BigInt, BigUint, One};

macro_rules! impl_integer_conversion {
    ($T:ty) => {
        impl From<$T> for Rational {
            #[inline]
            fn from(n: $T) -> Rational {
                Self::from_bigint(ToBigInt::to_bigint(&n).unwrap())
            }
        }

        impl TryFrom<Rational> for $T {
            type Error = Problem;

            fn try_from(n: Rational) -> Result<$T, Self::Error> {
                if let Some(i) = n.to_big_integer() {
                    <$T>::try_from(i).map_err(|_| Problem::OutOfRange)
                } else {
                    Err(Problem::NotAnInteger)
                }
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

fn signed(n: Rational, neg: bool) -> Rational {
    if neg {
        -n
    } else {
        n
    }
}

impl TryFrom<f32> for Rational {
    type Error = Problem;

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
                    Err(Problem::Infinity)
                } else {
                    Err(Problem::NotANumber)
                }
            }
            _ => unreachable!(),
        }
    }
}

impl TryFrom<f64> for Rational {
    type Error = Problem;

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
                    Err(Problem::Infinity)
                } else {
                    Err(Problem::NotANumber)
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
    fn signed_integers() {
        let one: Rational = i8::MAX.into();
        let two: Rational = i16::MAX.into();
        let three: Rational = i32::MAX.into();
        let four: Rational = i64::MAX.into();
        assert_eq!(one, Rational::new(0x7f));
        assert_eq!(two, Rational::new(0x7fff));
        assert_eq!(three, Rational::new(0x7fff_ffff));
        assert_eq!(four, Rational::new(0x7fff_ffff_ffff_ffff));
    }

    #[test]
    fn unsigned_integers() {
        let one: Rational = u8::MAX.into();
        let two: Rational = u16::MAX.into();
        let three: Rational = u32::MAX.into();
        assert_eq!(one, Rational::new(0xff));
        assert_eq!(two, Rational::new(0xffff));
        assert_eq!(three, Rational::new(0xffff_ffff));
    }

    #[test]
    fn nines() {
        let nine = Rational::new(99);
        let n_i8: i8 = nine.clone().try_into().unwrap();
        let n_u32: u32 = nine.clone().try_into().unwrap();
        let n_i64: i64 = nine.clone().try_into().unwrap();
        let n_u128: u128 = nine.clone().try_into().unwrap();
        assert_eq!(n_i8, 99);
        assert_eq!(n_u32, 99);
        assert_eq!(n_i64, 99);
        assert_eq!(n_u128, 99);
    }

    #[test]
    fn not_int() {
        let almost_pi = Rational::fraction(22,7).unwrap();
        let problem = <Rational as TryInto<i16>>::try_into(almost_pi).unwrap_err();
        assert_eq!(problem, Problem::NotAnInteger);
        let almost_pi = Rational::fraction(22,7).unwrap();
        let three: u32 = almost_pi.trunc().try_into().unwrap();
        assert_eq!(three, 3);
    }

    #[test]
    fn huge() {
        let huge = Rational::new(123_456_789);
        let problem = <Rational as TryInto<i16>>::try_into(huge).unwrap_err();
        assert_eq!(problem, Problem::OutOfRange);
    }

    #[test]
    fn negative() {
        let minus_100 = Rational::new(-100);
        let problem = <Rational as TryInto<u8>>::try_into(minus_100).unwrap_err();
        assert_eq!(problem, Problem::OutOfRange);
    }

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
