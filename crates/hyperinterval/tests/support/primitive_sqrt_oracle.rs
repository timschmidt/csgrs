//! Independent MPFR directed roots of exact finite IEEE inputs.
use hyperinterval::IntervalError;
use rug::{Float, Rational, float::Round};

macro_rules! ieee_root {
    ($name:ident, $float:ty, $bits:ty, $convert:ident, $from:ident) => {
        pub fn $name(value: $float) -> Result<($bits, $bits), IntervalError> {
            if value < 0.0 {
                return Err(IntervalError::EmptyDomain);
            }
            // Ingress is exact at 128 bits. Same-direction nested rounding
            // retains the tight target floor/ceiling after the MPFR root.
            let mut lower = Float::with_val(128, value);
            let mut upper = lower.clone();
            lower.sqrt_round(Round::Down);
            upper.sqrt_round(Round::Up);
            let lower = lower.$convert(Round::Down);
            let upper = upper.$convert(Round::Up);
            let source = Rational::$from(value).unwrap();
            let lo = Rational::$from(lower).unwrap();
            let hi = Rational::$from(upper).unwrap();
            assert!(lo.square() <= source);
            assert!(hi.square() >= source);
            let inward_lower = <$float>::from_bits(lower.to_bits() + 1);
            assert!(Rational::$from(inward_lower).unwrap().square() > source);
            if upper != 0.0 {
                let inward_upper = <$float>::from_bits(upper.to_bits() - 1);
                assert!(Rational::$from(inward_upper).unwrap().square() < source);
            }
            Ok((
                if lower == 0.0 { 0 } else { lower.to_bits() },
                if upper == 0.0 { 0 } else { upper.to_bits() },
            ))
        }
    };
}
ieee_root!(sqrt_f32, f32, u32, to_f32_round, from_f32);
ieee_root!(sqrt_f64, f64, u64, to_f64_round, from_f64);
