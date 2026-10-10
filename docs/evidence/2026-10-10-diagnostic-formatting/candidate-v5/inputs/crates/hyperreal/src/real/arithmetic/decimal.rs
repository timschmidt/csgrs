//! Checked adaptive decimal output and an explicitly diagnostic display view.

use super::Real;
use super::scientific::{MAX_DECIMAL_DIGITS, MAX_RATIONAL_BITS, round_rational};
use crate::{Problem, Rational, ScientificCase};
use core::fmt;
use num::bigint::BigUint;
use num::{Integer, One};

impl Real {
    /// Return a certified rounded decimal with a deterministic notation choice.
    ///
    /// Values whose magnitude is in `[2^-19, 2^60)` use fixed notation with
    /// `decimal_places` digits after the decimal point. Other values, including
    /// exact zero, use lowercase scientific notation with `decimal_places`
    /// digits after the leading digit. Rounding is to nearest, with exact ties
    /// going to an even last digit. A fixed-notation result can round to zero;
    /// it retains the sign of the source. This is a lossy presentation boundary,
    /// not exact serialization or a source for geometric predicates.
    ///
    /// The magnitude window resembles the older alternate formatter's choice,
    /// but is defined by exact values rather than a cached leading-bit estimate.
    /// Values at a window boundary can therefore select a different notation.
    ///
    /// # Errors and limits
    ///
    /// Uses the precision schedule, resource guards and abort handling of
    /// [`Self::try_scientific`]. Returns [`Problem::UnknownZero`] if the finite
    /// enclosure search cannot separate the value from zero, and
    /// [`Problem::Exhausted`] if rounding or notation remains ambiguous, work
    /// is aborted, or a conversion resource limit is exceeded. A positive
    /// `min_precision` returns [`Problem::OutOfRange`]. The requested refinement
    /// floor does not bound all work within an underlying approximation.
    ///
    /// ```
    /// use hyperreal::Real;
    /// let ordinary: Real = "0.125".parse()?;
    /// let tiny: Real = "1e-40".parse()?;
    /// assert_eq!(ordinary.try_decimal(3, -128)?, "0.125");
    /// assert_eq!(tiny.try_decimal(3, -128)?, "1.000e-40");
    /// # Ok::<(), hyperreal::Problem>(())
    /// ```
    pub fn try_decimal(
        &self,
        decimal_places: usize,
        min_precision: i32,
    ) -> Result<String, Problem> {
        self.try_decimal_rounding(decimal_places, min_precision, |value| {
            round_adaptive(value, decimal_places)
        })
    }

    /// Borrow an adaptive decimal view for human-readable diagnostics.
    ///
    /// The formatter's precision selects the decimal places as in
    /// [`Self::try_decimal`]. Without an explicit precision, it uses 32 places
    /// and removes trailing fractional zeros. Numeric conversion failures are
    /// printed as nonnumeric markers such as `[UnknownZero]` or `[Exhausted]`.
    /// The view never substitutes numeric zero for a failed conversion.
    ///
    /// Use [`Self::try_decimal`] when the caller must handle a typed failure.
    /// This view is for logs, diagnostic messages and interactive answers;
    /// its output must not be used as machine numeric serialization. Existing
    /// scalar formatting traits keep their original implementation.
    ///
    /// ```
    /// use hyperreal::Real;
    /// let value: Real = "1/8".parse().unwrap();
    /// assert_eq!(format!("{:.3}", value.diagnostic_decimal(-128)), "0.125");
    /// assert_eq!(format!("{}", value.diagnostic_decimal(-128)), "0.125");
    /// ```
    pub fn diagnostic_decimal(&self, min_precision: i32) -> impl fmt::Display + '_ {
        struct DiagnosticDecimal<'a> {
            value: &'a Real,
            min_precision: i32,
        }

        impl fmt::Display for DiagnosticDecimal<'_> {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                let precision = f.precision();
                let mut text = match self
                    .value
                    .try_decimal(precision.unwrap_or(32), self.min_precision)
                {
                    Ok(text) => text,
                    Err(problem) => return write!(f, "[{problem:?}]"),
                };
                if precision.is_none() {
                    let end = text.find('e').unwrap_or(text.len());
                    if text[..end].contains('.') {
                        let trimmed = text[..end]
                            .trim_end_matches('0')
                            .trim_end_matches('.')
                            .len();
                        // Restrict trimming to the mantissa; exponent zeros
                        // belong to the value and must never be removed.
                        text.replace_range(trimmed..end, "");
                    }
                }
                match text.strip_prefix('-') {
                    Some(magnitude) => f.pad_integral(false, "", magnitude),
                    None => f.pad_integral(true, "", &text),
                }
            }
        }

        DiagnosticDecimal {
            value: self,
            min_precision,
        }
    }
}

fn round_adaptive(value: &Rational, places: usize) -> Result<String, Problem> {
    if value.numerator().bits() > MAX_RATIONAL_BITS
        || value.denominator().bits() > MAX_RATIONAL_BITS
    {
        return Err(Problem::Exhausted);
    }
    let magnitude = if value.is_negative() {
        -value
    } else {
        value.clone()
    };
    let minimum = Rational::fraction(1, 1 << 19).expect("nonzero denominator");
    let maximum = Rational::new(1 << 60);
    if magnitude < minimum || magnitude >= maximum {
        return round_rational(value, places, ScientificCase::Lower);
    }
    round_fixed(value, places)
}

fn round_fixed(value: &Rational, places: usize) -> Result<String, Problem> {
    if places > MAX_DECIMAL_DIGITS {
        return Err(Problem::Exhausted);
    }
    let numerator = value.numerator() * BigUint::from(10_u8).pow(places as u32);
    let denominator = value.denominator();
    let (mut rounded, remainder) = numerator.div_rem(denominator);
    let twice_remainder = remainder << 1_usize;
    if &twice_remainder > denominator || (&twice_remainder == denominator && rounded.is_odd()) {
        rounded += BigUint::one();
    }
    let digits = rounded.to_str_radix(10);
    let mut text = String::new();
    text.try_reserve(places + 24)
        .map_err(|_| Problem::Exhausted)?;
    if value.is_negative() {
        text.push('-');
    }
    if places == 0 {
        text.push_str(&digits);
    } else if digits.len() > places {
        let split = digits.len() - places;
        text.push_str(&digits[..split]);
        text.push('.');
        text.push_str(&digits[split..]);
    } else {
        text.push_str("0.");
        text.extend(core::iter::repeat_n('0', places - digits.len()));
        text.push_str(&digits);
    }
    Ok(text)
}
