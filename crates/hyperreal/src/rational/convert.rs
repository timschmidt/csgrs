//! Exact primitive ingress and checked integer egress for the rational carrier.
//!
//! Fixed-width integer conversions preserve their full range. Finite IEEE binary32
//! and binary64 inputs become their exact dyadic values, including subnormals;
//! both signed zeros become canonical rational zero. NaNs and infinities produce
//! distinct typed errors. No float import recovers an intended decimal spelling.
//!
//! For example, binary64 `0.1` differs from the exact decimal one tenth:
//!
//! ```
//! use hyperreal::{Problem, Rational};
//! let tenth = Rational::fraction(1, 10).unwrap();
//! assert_ne!(Rational::try_from(0.1_f64).unwrap(), tenth);
//! assert_eq!("0.1".parse::<Rational>().unwrap(), tenth);
//! assert_eq!(Rational::try_from(f64::INFINITY), Err(Problem::Infinity));
//! ```
//!
//! Integer export consumes the rational, canonicalizes lazy storage as needed,
//! and rejects fractional values before checking the destination range. It does
//! not round or truncate. These adapters have no policy parameter, total-work
//! budget or typed allocation failure; arbitrary-width export inputs can require
//! normalization and magnitude cloning. The private dyadic helpers admit larger
//! shifts than their bounded IEEE callers normally supply.

use crate::verified::float::{FloatParts, decode_f32, decode_f64};
use crate::{Problem, Rational};
use num::BigUint;
use num::bigint::Sign;

/// Generate exact ingress and checked egress for signed fixed-width primitives.
///
/// Invoked for i8, i16, i32, i64 and i128. Ingress splits the sign from
/// unsigned_abs, including the most negative value without signed overflow.
/// Egress uses the shared integer query before the target's checked BigInt
/// conversion, preserving NotAnInteger versus OutOfRange and that error order.
macro_rules! impl_signed_integer_conversion {
    ($T:ty) => {
        /// Preserve every signed source integer exactly in the rational representation.
        impl From<$T> for Rational {
            /// Split sign and unsigned magnitude, then reuse canonical zero/unit/small storage.
            /// The minimum signed value is accepted through unsigned_abs; larger magnitudes
            /// use the exact owned-integer path without an intermediate signed BigInt.
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

        /// Consume an exact rational only when its integer value fits this target type.
        /// Fractional inputs return NotAnInteger even if their magnitude is also too large.
        /// Integral values outside the target range return OutOfRange.
        impl TryFrom<Rational> for $T {
            /// Distinguish a fractional value from an integer outside the destination range.
            type Error = Problem;

            /// Canonicalize and clone an exact integer magnitude, then check the target range.
            /// No truncation or rounding is performed. Lazy normalization and BigInt creation
            /// can allocate and have no local work, memory or cancellation budget.
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

/// Generate exact ingress and checked egress for unsigned fixed-width primitives.
///
/// Invoked for u8, u16, u32, u64 and u128. Ingress passes a nonnegative
/// magnitude to the shared constructor. Egress first requires an integer;
/// negative integers and oversized positive integers then produce OutOfRange.
macro_rules! impl_unsigned_integer_conversion {
    ($T:ty) => {
        /// Preserve every unsigned source integer exactly in the rational representation.
        impl From<$T> for Rational {
            /// Pass the full unsigned magnitude to the shared exact integer constructor.
            /// Zero, units and eligible small values reuse canonical storage; the larger
            /// integer path remains exact and does not pass through floating point.
            #[inline]
            fn from(n: $T) -> Rational {
                // Unsigned primitives are already narrow magnitudes, so let the
                // shared constructor use canonical small-integer storage before
                // materializing a BigUint for larger values.
                Self::from_primitive_integer(Sign::Plus, u128::from(n))
            }
        }

        /// Consume an exact rational only when its integer value fits this target type.
        /// Fractional inputs return NotAnInteger even if their magnitude is also too large.
        /// Integral values outside the target range return OutOfRange.
        impl TryFrom<Rational> for $T {
            /// Distinguish a fractional value from an integer outside the destination range.
            type Error = Problem;

            /// Canonicalize and clone an exact integer magnitude, then check the target range.
            /// No truncation or rounding is performed. Lazy normalization and BigInt creation
            /// can allocate and have no local work, memory or cancellation budget.
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

/// Import an owned unsigned float significand already shifted to an integer.
///
/// Applies the decoded sign and delegates denominator-one construction, including
/// canonical zero. The helper accepts any finite BigUint magnitude, not just IEEE
/// widths, and adds no resource budget or numeric validation error.
fn integer_from_unsigned_magnitude(n: BigUint, neg: bool) -> Rational {
    // Float decomposition produces an unsigned significand plus a sign bit,
    // matching Rational's internal representation without signed bigint work.
    Rational::from_integer_magnitude(if neg { Sign::Minus } else { Sign::Plus }, n)
}

/// Construct the exact signed ratio numerator/2^denominator_shift from a u32 word.
///
/// Zero returns canonical zero independently of sign and shift. Otherwise removes
/// min(trailing_zeros, denominator_shift) powers of two and supplies the proven
/// nonzero, reduced word to the shared dyadic constructor, retaining its known
/// dyadic facts. Every u32 shift is arithmetically admitted, but denominator
/// allocation has no local resource bound; the binary32 decoder uses at most
/// 149. The Boolean sign denotes negation, not a separate negative zero.
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

/// Construct the exact signed ratio numerator/2^denominator_shift from a u64 word.
///
/// Zero returns canonical zero independently of sign and shift. Otherwise removes
/// min(trailing_zeros, denominator_shift) powers of two and supplies the proven
/// nonzero, reduced word to the shared dyadic constructor, retaining its known
/// dyadic facts. Every u32 shift is arithmetically admitted, but denominator
/// allocation has no local resource bound; the binary64 decoder uses at most
/// 1074. The Boolean sign denotes negation, not a separate negative zero.
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

/// Import each finite binary32 value as the exact rational encoded by its bits.
///
/// Includes normal and subnormal values; both signed zeros become canonical zero.
/// Infinity and NaN are rejected separately. Decimal intent, NaN payloads and a
/// zero sign are not part of the rational value.
impl TryFrom<f32> for Rational {
    /// Reject infinities as Infinity and every NaN encoding as NotANumber.
    type Error = Problem;

    /// Decode a signed significand times a power of two without rounding.
    ///
    /// The decoder bounds significands to 24 bits and exponents to -149..=104;
    /// nonpositive exponents use exact power-of-two cancellation, while positive
    /// exponents shift an owned integer magnitude. These bounds justify the casts
    /// and negated exponent used here.
    /// This adapter does not explicitly mark the exact-binary64-view fact; the Real
    /// float adapter establishes that widening fact when it wraps this rational.
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

/// Import each finite binary64 value as the exact rational encoded by its bits.
///
/// Includes normal and subnormal values; both signed zeros become canonical zero.
/// Infinity and NaN are rejected separately. Decimal intent, NaN payloads and a
/// zero sign are not part of the rational value.
impl TryFrom<f64> for Rational {
    /// Reject infinities as Infinity and every NaN encoding as NotANumber.
    type Error = Problem;

    /// Decode a signed significand times a power of two without rounding.
    ///
    /// The decoder bounds significands to 53 bits and exponents to -1074..=971;
    /// nonpositive exponents use exact power-of-two cancellation, while positive
    /// exponents shift an owned integer magnitude. These bounds justify the casts
    /// and negated exponent used here.
    /// Every finite success records exact binary64 representability on the rational
    /// owner; that fact stores neither the original bits nor a signed-zero distinction.
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

/// Finite regression witnesses for primitive ingress, checked egress and retained facts.
///
/// Literal integer/fraction expectations test exact values; the trace fixture pins
/// one retained dyadic hit and no learned fact. This collection is not exhaustive
/// across primitive widths, float encodings or normalization/cache transitions.
#[cfg(test)]
mod tests {
    use super::*;
    use num::{BigInt, One};

    /// Check positive maxima for signed 8-, 16-, 32- and 64-bit ingress.
    /// Literal integer expectations omit negative minima and the i128 conversion.
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

    /// Check unsigned 8-, 16- and 32-bit maxima against literal integer values.
    /// This fixture does not cover the generated u64/u128 implementations.
    #[test]
    fn unsigned_integers() {
        let one: Rational = u8::MAX.into();
        let two: Rational = u16::MAX.into();
        let three: Rational = u32::MAX.into();
        assert_eq!(one, Rational::new(0xff));
        assert_eq!(two, Rational::new(0xffff));
        assert_eq!(three, Rational::new(0xffff_ffff));
    }

    /// Export the integer 99 exactly to i8, u32, i64 and u128.
    /// These positive in-range examples do not exercise every generated target.
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

    /// Require NotAnInteger for 22/7, then explicitly truncate before unsigned export.
    /// The separate truncation produces three; the conversion itself never truncates.
    #[test]
    fn not_int() {
        let almost_pi = Rational::fraction(22, 7).unwrap();
        let problem = <Rational as TryInto<i16>>::try_into(almost_pi).unwrap_err();
        assert_eq!(problem, Problem::NotAnInteger);
        let almost_pi = Rational::fraction(22, 7).unwrap();
        let three: u32 = almost_pi.trunc().try_into().unwrap();
        assert_eq!(three, 3);
    }

    /// Require OutOfRange when the integral value 123456789 is exported to i16.
    #[test]
    fn huge() {
        let huge = Rational::new(123_456_789);
        let problem = <Rational as TryInto<i16>>::try_into(huge).unwrap_err();
        assert_eq!(problem, Problem::OutOfRange);
    }

    /// Require OutOfRange when the negative integer -100 is exported to u8.
    #[test]
    fn negative() {
        let minus_100 = Rational::new(-100);
        let problem = <Rational as TryInto<u8>>::try_into(minus_100).unwrap_err();
        assert_eq!(problem, Problem::OutOfRange);
    }

    /// Import positive binary32 and binary64 zero as canonical rational zero.
    /// Negative-zero behavior is covered separately by the retained-view fixture.
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

    /// Import binary32 and binary64 one half against the exact fraction 1/2.
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

    /// Pin the binary32 value of 1.2345679 to the exact ratio 5178153/4194304.
    /// The literal denotes its binary rounding, not that decimal rational.
    #[test]
    fn repr_f32() {
        let f: f32 = 1.234_567_9;
        let a: Rational = f.try_into().unwrap();
        let correct = Rational::fraction(5178153, 4194304).unwrap();
        assert_eq!(a, correct);
    }

    /// Pin binary64 1.23456789 to the exact ratio 5559999489367579/4503599627370496.
    /// This checks the encoded binary value rather than its decimal spelling.
    #[test]
    fn repr_f64() {
        let f: f64 = 1.23456789;
        let a: Rational = f.try_into().unwrap();
        let correct = Rational::fraction(5559999489367579, 4503599627370496).unwrap();
        assert_eq!(a, correct);
    }

    /// Check imported binary64 0.75 against reduced 3/4 and the exact-view fact.
    /// Canonical numerator/denominator observations do not independently prove the
    /// original stored parts were reduced before any accessor ran.
    #[test]
    fn reduced_binary_fraction_f64() {
        let value: Rational = 0.75_f64.try_into().unwrap();
        assert_eq!(value, Rational::fraction(3, 4).unwrap());
        assert_eq!(*value.numerator(), BigUint::from(3_u8));
        assert_eq!(*value.denominator(), BigUint::from(4_u8));
        assert!(value.has_exact_f64_view());
    }

    /// Require one retained dyadic hit and zero learned facts after a binary64 import.
    /// The recording window includes construction and is_dyadic for 1.23456789;
    /// these named counts do not measure every event or allocation.
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

    /// Check exact-view retention for five finite inputs, including negative zero.
    /// The set includes a negative integer, the smallest positive subnormal, one
    /// ordinary fraction and the largest finite value. Normal values and zero
    /// round-trip through the exact dyadic view by bits, with zero made positive;
    /// the subnormal case checks only the retained fact, not its exported bits.
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

    /// Consume 3/8 and 5/16 in four scalar operations with literal exact expectations.
    /// Addition, subtraction and multiplication remain dyadic; division gives 6/5.
    /// Value equality alone does not inspect raw storage or force a particular path.
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

    /// Import binary64 bit pattern two as the exact reduced fraction 1/2^1073.
    /// Equality checks the numeric value; this test does not inspect raw storage.
    #[test]
    fn reduced_subnormal_f64() {
        let value: Rational = f64::from_bits(2).try_into().unwrap();
        let correct =
            Rational::from_bigint_fraction(BigInt::from(1), BigUint::one() << 1073).unwrap();
        assert_eq!(value, correct);
    }
}
