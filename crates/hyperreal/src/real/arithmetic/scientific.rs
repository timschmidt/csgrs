//! Checked scientific output from exact rational rounding certificates.

use super::Real;
use crate::{Problem, Rational, ZeroKnowledge};
use num::bigint::{BigUint, Sign};
use num::{Integer, One};

/// Case of the exponent marker in [`Real::try_scientific`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScientificCase {
    /// Use `e`, as in `1.250000e-3`.
    Lower,
    /// Use `E`, as in `1.250000E-3`.
    Upper,
}

/// Maximum requested fractional digits and exponent accepted by temporary powers of ten.
pub(super) const MAX_DECIMAL_DIGITS: usize = 1_000_000;
/// Maximum negative computable precision magnitude requested by checked formatting; not total work.
const MAX_REFINEMENT_BITS: i32 = 1_000_000;
/// Maximum bit length of either exact rational part admitted to rounding arithmetic.
pub(super) const MAX_RATIONAL_BITS: u64 = 4_000_000;

/// Checked scientific formatting and the shared enclosure-to-string certificate search.
impl Real {
    /// Return a certified, rounded scientific decimal or a typed failure.
    ///
    /// `decimal_places` counts digits after the single leading digit. Rounding
    /// is to nearest, with exact halfway cases going to an even last digit.
    /// The exponent has no redundant `+` or leading zeros. Exact zero is
    /// unsigned and has exponent zero. For example, three places produce
    /// `1.250e-1`, not a fixed-point decimal with three fractional places.
    /// This is a lossy presentation boundary: the returned decimal is a rounded
    /// value, not an exact serialization or an input to an exact predicate.
    ///
    /// Exact rationals are rounded directly. Other values are enclosed at
    /// computable precisions `0, -16, -40, ...`, ending at `min_precision`.
    /// Output is returned only when both exact enclosure endpoints round to
    /// the same scientific string. A rational scale on this value also scales
    /// those enclosures; `min_precision` is the underlying computable precision,
    /// not a guaranteed absolute error bound on the scaled value.
    ///
    /// # Errors and limits
    ///
    /// * [`Problem::UnknownZero`]: the finite enclosure search cannot separate
    ///   this value from zero. An inconclusive search never formats as zero.
    /// * [`Problem::Exhausted`]: a requested enclosure proves nonzero but rounding
    ///   remains ambiguous, evaluation is aborted, or a formatting resource limit is
    ///   exceeded. An exact rational halfway case is decided directly; an
    ///   unresolved expression at the same boundary can remain ambiguous.
    /// * [`Problem::OutOfRange`]: `min_precision` is positive.
    ///
    /// At most 1,000,000 decimal places and 1,000,000 refinement bits are
    /// accepted. Rational endpoint numerators/denominators are limited to
    /// 4,000,000 bits, and temporary powers of ten to exponent 1,000,000.
    /// These guards bound this formatter's requests and arithmetic expansion.
    /// They do not bound all work inside structural analysis or a single
    /// underlying approximation. Existing evaluation abort flags are honored
    /// around numeric refinement; direct exact-rational proofs need none.
    ///
    /// This checked API does not change the existing formatting traits. Those
    /// traits cannot return these typed failures and may still seek a leading
    /// bit for an unresolved zero. Use this method when failure must be explicit.
    ///
    /// ```
    /// use hyperreal::{Real, ScientificCase};
    /// let value: Real = "1/8".parse().unwrap();
    /// assert_eq!(value.try_scientific(3, -128, ScientificCase::Lower)?, "1.250e-1");
    /// assert_eq!(value.try_scientific(3, -128, ScientificCase::Upper)?, "1.250E-1");
    /// # Ok::<(), hyperreal::Problem>(())
    /// ```
    pub fn try_scientific(
        &self,
        decimal_places: usize,
        min_precision: i32,
        case: ScientificCase,
    ) -> Result<String, Problem> {
        self.try_decimal_rounding(decimal_places, min_precision, |value| {
            round_rational(value, decimal_places, case)
        })
    }

    /// Certify a formatting string by exact value or agreement of enclosing endpoint strings.
    /// Validate requests first, then use exact rationals before checking abort.
    /// Other values check abort, consult structural zero, and request scaled
    /// rational enclosures at 0, -16, -40, ... clamped to min_precision.
    ///
    /// The callback must be sound on each Rational, with equal endpoint strings
    /// implying identical output throughout the enclosed interval. The two
    /// callers supply nearest-even rounding with distinguishable notation regions;
    /// this is not valid for an arbitrary string-valued callback. Callback errors
    /// propagate immediately. Enclosure-based success is rechecked for cancellation.
    ///
    /// Only enclosures requested in this search set nonzero_proved: a cached
    /// sign alone does not turn UnknownZero into Exhausted. Reaching the floor
    /// returns the former if no enclosure excluded zero, otherwise the latter.
    /// Failed interval production is Exhausted. Guards bound requests, not the
    /// entire cost of structural analysis or one underlying approximation.
    pub(super) fn try_decimal_rounding(
        &self,
        decimal_places: usize,
        min_precision: i32,
        round: impl Fn(&Rational) -> Result<String, Problem>,
    ) -> Result<String, Problem> {
        if min_precision > 0 {
            return Err(Problem::OutOfRange);
        }
        if decimal_places > MAX_DECIMAL_DIGITS || min_precision < -MAX_REFINEMENT_BITS {
            return Err(Problem::Exhausted);
        }
        if let Some(value) = self.exact_rational_ref() {
            return round(value);
        }
        if self.is_aborted() {
            return Err(Problem::Exhausted);
        }
        let zero = self.structural_facts().zero;
        if zero == ZeroKnowledge::Zero {
            return round(&Rational::zero());
        }
        // Use the enclosures requested by this call to distinguish the two
        // refinement failures. A finer cached sign alone must not change an
        // otherwise identical coarse search from UnknownZero to Exhausted.
        let mut nonzero_proved = false;
        let mut precision = 0_i32;
        loop {
            let [lower, upper] = self
                .certified_rational_interval(precision)
                .ok_or(Problem::Exhausted)?;
            if lower.sign() == Sign::Plus || upper.sign() == Sign::Minus {
                nonzero_proved = true;
                let lower_text = round(&lower)?;
                let upper_text = round(&upper)?;
                // Both callers use monotone nearest-even rounding within each
                // notation region. Adaptive fixed/scientific regions have
                // distinct spellings, so equal endpoint strings certify every
                // enclosed value and its notation choice.
                if lower_text == upper_text {
                    return if self.is_aborted() {
                        Err(Problem::Exhausted)
                    } else {
                        Ok(lower_text)
                    };
                }
            }
            if precision == min_precision {
                return Err(if nonzero_proved {
                    Problem::Exhausted
                } else {
                    Problem::UnknownZero
                });
            }
            // Validated bounds keep this arithmetic well inside i32. Clamp the
            // last step so no request goes beyond the caller's chosen floor.
            precision = ((precision * 3) / 2 - 16).max(min_precision);
        }
    }
}

/// Construct exact 10^exponent within the decimal expansion cap, otherwise Exhausted.
/// The cap also makes the usize-to-u32 cast exact; allocation by BigUint is not
/// independently budgeted or cancellable through this helper.
fn ten_to(exponent: usize) -> Result<BigUint, Problem> {
    if exponent > MAX_DECIMAL_DIGITS {
        return Err(Problem::Exhausted);
    }
    Ok(BigUint::from(10_u8).pow(exponent as u32))
}

/// Produce normalized scientific digits by exact integer quotient/remainder rounding.
/// The caller must bound decimal_places by MAX_DECIMAL_DIGITS before entry:
/// reservation and the zero path use that precondition before ten_to is called.
/// Nonzero rational parts are bit-limited; decimal digit counts bracket the
/// exponent and one comparison corrects it. Ties select an even significand;
/// a carry into the next decade increments the exponent and renormalizes digits.
/// Zero is unsigned with exponent zero. Preserve case only in the exponent marker.
/// Fallible string reservation and guarded powers report Exhausted; this does
/// not turn every internal bigint/string allocation into a typed memory limit.
pub(super) fn round_rational(
    value: &Rational,
    decimal_places: usize,
    case: ScientificCase,
) -> Result<String, Problem> {
    let mut text = String::new();
    text.try_reserve(decimal_places + 32)
        .map_err(|_| Problem::Exhausted)?;
    let mut exponent = 0_i64;
    let digits = if value.sign() == Sign::NoSign {
        "0".repeat(decimal_places + 1)
    } else {
        let numerator = value.numerator();
        let denominator = value.denominator();
        if numerator.bits() > MAX_RATIONAL_BITS || denominator.bits() > MAX_RATIONAL_BITS {
            return Err(Problem::Exhausted);
        }
        // Decimal digit lengths bracket the normalized ratio in [0.1, 10).
        // One exact comparison then selects its true decimal exponent.
        exponent =
            numerator.to_str_radix(10).len() as i64 - denominator.to_str_radix(10).len() as i64;
        let power = ten_to(exponent.unsigned_abs() as usize)?;
        let (mut numerator, denominator) = if exponent >= 0 {
            (numerator.clone(), denominator * power)
        } else {
            (numerator * power, denominator.clone())
        };
        if numerator < denominator {
            numerator *= 10_u8;
            exponent -= 1;
        }
        numerator *= ten_to(decimal_places)?;
        let (mut rounded, remainder) = numerator.div_rem(&denominator);
        let twice_remainder = remainder << 1_usize;
        if twice_remainder > denominator || (twice_remainder == denominator && rounded.is_odd()) {
            rounded += BigUint::one();
        }
        let mut digits = rounded.to_str_radix(10);
        if digits.len() > decimal_places + 1 {
            // A rounding carry produces exactly 10^(decimal_places + 1).
            rounded /= 10_u8;
            exponent += 1;
            digits = rounded.to_str_radix(10);
        }
        if value.sign() == Sign::Minus {
            text.push('-');
        }
        digits
    };
    text.push_str(&digits[..1]);
    if decimal_places != 0 {
        text.push('.');
        text.push_str(&digits[1..]);
    }
    text.push(match case {
        ScientificCase::Lower => 'e',
        ScientificCase::Upper => 'E',
    });
    text.push_str(&exponent.to_string());
    Ok(text)
}
