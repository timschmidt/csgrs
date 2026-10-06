use crate::verified::float::{FloatParts, decode_f32, decode_f64};
use crate::{Problem, Rational};
use num::BigUint;
use num::bigint::Sign;

macro_rules! impl_signed_integer_conversion {
    ($T:ty) => {
        impl From<$T> for Rational {
            #[inline]
            fn from(n: $T) -> Rational {
                // Split sign from unsigned_abs because Rational stores exactly
                // that shape; no signed BigInt is needed for primitive inputs.
                Self::from_primitive_integer(
                    if n < 0 { Sign::Minus } else { Sign::Plus },
                    u128::from(n.unsigned_abs()),
                )
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

macro_rules! impl_unsigned_integer_conversion {
    ($T:ty) => {
        impl From<$T> for Rational {
            #[inline]
            fn from(n: $T) -> Rational {
                // Unsigned primitives are already narrow magnitudes, so let the
                // shared constructor use canonical small-integer storage before
                // materializing a BigUint for larger values.
                Self::from_primitive_integer(Sign::Plus, u128::from(n))
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

impl_signed_integer_conversion!(i8);
impl_signed_integer_conversion!(i16);
impl_signed_integer_conversion!(i32);
impl_signed_integer_conversion!(i64);
impl_signed_integer_conversion!(i128);
impl_unsigned_integer_conversion!(u8);
impl_unsigned_integer_conversion!(u16);
impl_unsigned_integer_conversion!(u32);
impl_unsigned_integer_conversion!(u64);
impl_unsigned_integer_conversion!(u128);

fn integer_from_unsigned_magnitude(n: BigUint, neg: bool) -> Rational {
    // Float decomposition produces an unsigned significand plus a sign bit,
    // matching Rational's internal representation without signed bigint work.
    Rational::from_integer_magnitude(if neg { Sign::Minus } else { Sign::Plus }, n)
}

fn pow2_fraction_u32(numerator: u32, denominator_shift: u32, neg: bool) -> Rational {
    if numerator == 0 {
        return Rational::zero();
    }
    // Binary floats import as numerator / 2^k. Strip common powers of two here
    // so the resulting Rational is reduced without a general BigUint gcd.
    let shift = numerator.trailing_zeros().min(denominator_shift);
    let numerator = numerator >> shift;
    let denominator_shift = denominator_shift - shift;
    Rational::from_reduced_dyadic_word(
        if neg { Sign::Minus } else { Sign::Plus },
        u64::from(numerator),
        denominator_shift,
    )
}

fn pow2_fraction_u64(numerator: u64, denominator_shift: u32, neg: bool) -> Rational {
    if numerator == 0 {
        return Rational::zero();
    }
    // Same reduced-dyadic path as f32. This is the hot finite-binary64-to-exact-dyadic import path
    // used by hyperlattice and hyperlimit benchmark construction.
    let shift = numerator.trailing_zeros().min(denominator_shift);
    let numerator = numerator >> shift;
    let denominator_shift = denominator_shift - shift;
    Rational::from_reduced_dyadic_word(
        if neg { Sign::Minus } else { Sign::Plus },
        numerator,
        denominator_shift,
    )
}

impl TryFrom<f32> for Rational {
    type Error = Problem;

    fn try_from(n: f32) -> Result<Rational, Self::Error> {
        match decode_f32(n.to_bits()) {
            FloatParts::Finite {
                negative,
                significand,
                exponent,
            } => {
                if exponent <= 0 {
                    // The decoder proves that binary32 significands fit 24 bits.
                    Ok(pow2_fraction_u32(
                        significand as u32,
                        (-exponent) as u32,
                        negative,
                    ))
                } else {
                    Ok(integer_from_unsigned_magnitude(
                        BigUint::from(significand) << exponent as u32,
                        negative,
                    ))
                }
            }
            FloatParts::Infinity => Err(Problem::Infinity),
            FloatParts::NotANumber => Err(Problem::NotANumber),
        }
    }
}

impl TryFrom<f64> for Rational {
    type Error = Problem;

    fn try_from(n: f64) -> Result<Rational, Self::Error> {
        let rational = match decode_f64(n.to_bits()) {
            FloatParts::Finite {
                negative,
                significand,
                exponent,
            } => {
                if exponent <= 0 {
                    pow2_fraction_u64(significand, (-exponent) as u32, negative)
                } else {
                    integer_from_unsigned_magnitude(
                        BigUint::from(significand) << exponent as u32,
                        negative,
                    )
                }
            }
            FloatParts::Infinity => return Err(Problem::Infinity),
            FloatParts::NotANumber => return Err(Problem::NotANumber),
        };
        rational.mark_exact_f64_view();
        Ok(rational)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use num::{BigInt, One};

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
        let almost_pi = Rational::fraction(22, 7).unwrap();
        let problem = <Rational as TryInto<i16>>::try_into(almost_pi).unwrap_err();
        assert_eq!(problem, Problem::NotAnInteger);
        let almost_pi = Rational::fraction(22, 7).unwrap();
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
        let f: f32 = 1.234_567_9;
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

    #[test]
    fn reduced_binary_fraction_f64() {
        let value: Rational = 0.75_f64.try_into().unwrap();
        assert_eq!(value, Rational::fraction(3, 4).unwrap());
        assert_eq!(*value.numerator(), BigUint::from(3_u8));
        assert_eq!(*value.denominator(), BigUint::from(4_u8));
        assert!(value.has_exact_f64_view());
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn float_import_retains_the_reduced_dyadic_fact() {
        crate::dispatch_trace::reset();
        crate::dispatch_trace::with_recording(|| {
            let value = Rational::try_from(1.234_567_89_f64).unwrap();
            assert!(value.is_dyadic());
        });
        let trace = crate::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("rational", "retained-facts", "dyadic-hit"),
            1
        );
        assert_eq!(
            trace.path_count("rational", "retained-facts", "dyadic-learned"),
            0
        );
    }

    #[test]
    fn f64_import_retains_exact_view_for_normal_subnormal_and_integer_values() {
        for source in [
            -987_654_321.0,
            -0.0,
            f64::from_bits(1),
            0.123_456_789,
            f64::MAX,
        ] {
            let value = Rational::try_from(source).unwrap();
            assert!(value.has_exact_f64_view());
            if source.is_normal() || source == 0.0 {
                let expected = if source == 0.0 { 0.0 } else { source };
                assert_eq!(
                    value.dyadic_to_f64_exact().map(f64::to_bits),
                    Some(expected.to_bits())
                );
            }
        }
    }

    #[test]
    fn dyadic_arithmetic_stays_reduced() {
        let three_eighths = Rational::fraction(3, 8).unwrap();
        let five_sixteenths = Rational::fraction(5, 16).unwrap();

        assert_eq!(
            three_eighths.clone() + five_sixteenths.clone(),
            Rational::fraction(11, 16).unwrap()
        );
        assert_eq!(
            three_eighths.clone() - five_sixteenths.clone(),
            Rational::fraction(1, 16).unwrap()
        );
        assert_eq!(
            three_eighths.clone() * five_sixteenths.clone(),
            Rational::fraction(15, 128).unwrap()
        );
        assert_eq!(
            three_eighths / five_sixteenths,
            Rational::fraction(6, 5).unwrap()
        );
    }

    #[test]
    fn reduced_subnormal_f64() {
        let value: Rational = f64::from_bits(2).try_into().unwrap();
        let correct =
            Rational::from_bigint_fraction(BigInt::from(1), BigUint::one() << 1073).unwrap();
        assert_eq!(value, correct);
    }
}
