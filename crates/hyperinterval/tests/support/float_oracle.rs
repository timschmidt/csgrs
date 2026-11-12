use hyperinterval::IntervalError;
use rug::{Float, Rational, float::Round};

// Independently directed MPFR conversion from exact GMP rationals. Precision
// 128 is finer than every target normal/subnormal grid; same-direction nested
// rounding retains the tight target floor/ceiling.
macro_rules! oracle {
    ($name:ident, $bits:ty, $convert:ident) => {
        pub fn $name(value: &Rational) -> Result<($bits, $bits), IntervalError> {
            crate::mpfr_cache::cleanup_on_thread_exit();
            let (lo, _) = Float::with_val_round(128, value, Round::Down);
            let (hi, _) = Float::with_val_round(128, value, Round::Up);
            let (lo, hi) = (lo.$convert(Round::Down), hi.$convert(Round::Up));
            if !lo.is_finite() || !hi.is_finite() {
                return Err(IntervalError::Overflow);
            }
            Ok((
                if lo == 0.0 { 0 } else { lo.to_bits() },
                if hi == 0.0 { 0 } else { hi.to_bits() },
            ))
        }
    };
}

oracle!(round_f32, u32, to_f32_round);
oracle!(round_f64, u64, to_f64_round);
