//! Approximate fixed/scientific presentation of retained computable reals.
//!
//! These diagnostic formatters extract decimal digits from one guarded binary
//! sample; they do not certify correctly rounded decimal text or preserve an
//! exact interchange representation. Integer divisor scaling and strict remainder
//! comparisons are not a universal decimal tie rule. Use Real's checked decimal
//! or scientific APIs when rounding or numerical failure must be reported.
//!
//! Fixed formatting bounds leading-bit search by its output precision; scientific
//! formatting uses the much deeper default magnitude search. Neither bounds total
//! child work, allocation or output size. Attached cancellation follows the native
//! approximation machinery without a formatting receipt. Width, alignment, fill,
//! zero padding and alternate flags are not implemented; precision and explicit
//! plus are consumed. A displayed zero is not an exact-zero proof.

use crate::computable::{Precision, unsigned};
use crate::{Computable, RealSign};
use core::fmt;
use num::{BigUint, Zero};

/// Remove trailing numeric zero digits from a most-significant-first buffer.
/// Digits are numeric bytes, not ASCII. Keep the first nonzero suffix boundary;
/// an all-zero buffer becomes empty. This supports default trailing-zero
/// suppression and does not alter the separate decimal exponent.
fn trim(num: &mut Vec<u8>) {
    loop {
        match num.pop() {
            Some(0) => (),
            Some(other) => {
                num.push(other);
                return;
            }
            None => return,
        }
    }
}

/// Increment a most-significant-first decimal mantissa, reporting an exponent carry.
/// Require numeric decimal digits. Restore zeros after the rightmost non-nine;
/// if all digits are nine, keep their original count as one followed by zeros
/// and return true so the caller increases the exponent. Empty input becomes
/// one with a carry. This is not a general fixed-width integer increment.
/// The signed local nine counter and allocation have no explicit size budget.
fn up(num: &mut Vec<u8>) -> bool {
    let mut flag = false;
    let mut nines = 0;
    loop {
        match num.pop() {
            Some(9) => {
                nines += 1;
            }
            Some(other) => {
                num.push(other + 1);
                break;
            }
            None => {
                num.push(1);
                flag = true;
                nines -= 1;
                break;
            }
        }
    }
    for _ in 0..nines {
        num.push(0);
    }
    flag
}

/// Default number of fractional digits: fixed places or scientific mantissa places.
/// Unspecified precision also enables trailing-zero suppression.
const DEFAULT_PRECISION: usize = 32;

/// Plan guarded binary sample bits from decimal places and a leading-bit estimate.
/// Use `floor((places+4)*32/9)`, with 32 default places, and add only a positive
/// msd. This supports a fixed absolute precision when Display clamps a tiny
/// magnitude to zero; it is not a proof of correctly rounded decimal output.
/// Unchecked usize arithmetic and positive-msd addition can overflow, and the
/// conversion to Precision panics when it does not fit. No resource receipt
/// or total-work bound is returned.
fn enough_bits(msd: Precision, prec: Option<usize>) -> Precision {
    /// Convert decimal places to the formatter's integer guard-bit estimate.
    /// Evaluate `(p+4)*32/9` before the checked conversion to Precision. Oversized
    /// requests can overflow the usize intermediates or panic at conversion;
    /// this nested planner does not validate a resource envelope.
    fn bits(p: usize) -> Precision {
        let b = ((p + 4) * 32) / 9;
        b.try_into().expect("Bits of precision should fit")
    }

    let bits = bits(prec.unwrap_or(DEFAULT_PRECISION)) as Precision;
    if msd > 0 { bits + msd } else { bits }
}

/// Decimal placement request independent of the binary approximation precision.
/// Copy/Clone preserve the request; Debug exposes the variant/count for diagnostics.
#[derive(Copy, Clone, Debug)]
enum Places {
    /// Scientific mantissa places, excluding its single leading digit.
    Exp(
        /// Requested digits after the mantissa decimal point.
        usize,
    ),
    /// Fixed decimal places after the units position.
    Zero(
        /// Requested fractional places, independent of the leading decimal exponent.
        usize,
    ),
}

/// Translate a placement request into the extracted significant-digit count.
impl Places {
    /// Return n+1 for scientific notation, or max(n+exp+1,0) for fixed notation.
    /// The fixed branch casts usize to i32 and adds without checked overflow;
    /// callers must keep the request and exponent in the usable arithmetic range.
    /// Zero count sends digit extraction to its separate rounding branch.
    fn digits(self, exp: i32) -> usize {
        use Places::*;

        match self {
            Exp(n) => n + 1,
            Zero(n) => {
                let places = n as i32 + exp + 1;
                if places < 0 { 0 } else { places as usize }
            }
        }
    }
}

/// Produce numeric zero digits and exponent zero for an already-zero sample.
/// With stop enabled return a single zero. Otherwise emit the requested count
/// at exponent zero, including the scientific leading digit. This describes
/// the sample only and does not certify the represented real is zero.
fn zeroes(places: Places, stop: bool) -> (Vec<u8>, i32) {
    if stop {
        (vec![0], 0)
    } else {
        let count = places.digits(0);
        (vec![0; count], 0)
    }
}

/// Extract numeric decimal mantissa digits and a leading exponent from a binary sample.
/// The sampled magnitude denotes `magn * 2^(msd-bits)`. Normalize an integer
/// divisor by powers of two, five and ten, then extract the requested count
/// with integer quotient/remainder operations. Integer division while shrinking
/// the divisor discards remainders; this is not exact rational decimal scaling.
///
/// Increment only when twice the final remainder exceeds the divisor. An
/// all-nine increment raises the exponent; a first quotient ten also becomes
/// leading one with an increased exponent. A zero requested count has its own
/// strict threshold branch. These rules do not establish a uniform decimal tie
/// policy (native fixed formatting sends 1/2 to 1 but 3/2 to 1 at zero places).
/// With stop enabled trim trailing zeros after ordinary extraction; zero and
/// zero-count returns have separate handling.
///
/// Callers must provide a valid sample/precision plan that keeps the divisor
/// nonzero for extraction and digits in range. Arithmetic, vector reservation
/// and byte conversion are not typed numerical/resource failures. This helper
/// does not account for the source sample's uncertainty or poll cancellation.
fn digits(
    magn: &BigUint,
    places: Places,
    bits: Precision,
    msd: Precision,
    stop: bool,
) -> (Vec<u8>, i32) {
    let mut exp: i32 = 0;
    let mut divisor = unsigned::ONE.clone();
    let mut excess = msd - bits;

    if *magn == BigUint::zero() {
        return zeroes(places, stop);
    }

    // If we have enough bits already then just divide off the powers of two
    if excess < 0 {
        divisor <<= bits - msd;
    }

    // Regardless, adjust until we've calculated the decimal exponent
    loop {
        while divisor <= *magn {
            if excess > 0 {
                excess -= 1;
                exp += 1;
                divisor *= &*unsigned::FIVE;
            } else {
                exp += 1;
                divisor *= &*unsigned::TEN;
            }
        }
        while divisor > *magn {
            exp -= 1;
            divisor /= &*unsigned::TEN;
        }
        if excess <= 0 {
            break;
        }
    }

    let count = places.digits(exp);
    // If we're not actually here to calculate any digits, but rounding occurs...
    if count == 0 {
        divisor *= &*unsigned::FIVE;
        if magn > &divisor {
            return (vec![1], exp + 1);
        } else {
            return (vec![], exp);
        }
    }

    let mut num: Vec<u8> = Vec::with_capacity(count);
    let mut left = magn.clone();

    for k in 0..count {
        if k > 0 {
            left *= &*unsigned::TEN;
        }
        let digit = &left / &divisor;
        left -= &digit * &divisor;
        if k == 0 && digit == *unsigned::TEN {
            exp += 1;
            num.push(1);
        } else {
            num.push(digit.try_into().unwrap());
        }
    }
    left *= &*unsigned::TWO;
    if left > divisor && up(&mut num) {
        // All nines rounded up
        exp += 1;
    }

    if stop {
        trim(&mut num);
    }

    (num, exp)
}

/// Approximate fixed-point decimal presentation with precision-limited magnitude search.
/// Use 32 fractional places by default, omitting trailing zeros, or exactly the
/// explicit fractional count. A certified negative sign can produce negative
/// zero text; unresolved sign is not a sign certificate. Width/alignment/fill,
/// zero padding and alternate formatting are ignored.
///
/// This native formatter does not certify decimal rounding or return typed
/// numerical/resource/cancellation outcomes. Use [`crate::Real::try_decimal`]
/// for checked rounding (with its own notation-selection contract).
impl fmt::Display for Computable {
    /// Bound leading-bit search by the guarded fixed output precision, sample and write digits.
    /// Clamp the magnitude estimate at zero and pad explicit fractional places.
    /// Propagate only formatter write errors; native evaluation can still panic or
    /// produce unqualified interrupted output.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Fixed decimal places need an absolute error bound, not a proof that
        // the value is nonzero. Stop the leading-bit search below the guarded
        // output precision so cancellation and arbitrarily tiny values remain
        // productive. Zero is also a safe scale when no leading bit is found.
        let stop = -enough_bits(0, f.precision());
        // A retained exact MSD can be far below `stop`. Fixed decimal places
        // still need only the absolute output precision, even in that case.
        let msd = self.iter_msd_stop(stop).unwrap_or(0).max(0);
        let bits = enough_bits(msd, f.precision());
        let appr = self.approx(msd - bits);
        if self.sign_until(msd - bits) == Some(RealSign::Negative) {
            f.write_str("-")?;
        } else if f.sign_plus() {
            // Even for zero
            f.write_str("+")?;
        }
        let mut dp = f.precision().unwrap_or(DEFAULT_PRECISION);
        let (num, mut exp) = digits(appr.magnitude(), Places::Zero(dp), bits, msd, true);
        let mut num = num.into_iter().peekable();

        if exp < 0 {
            f.write_str("0")?;
        }
        while exp >= 0 {
            let digit = num.next().unwrap_or_default();
            write!(f, "{digit}")?;
            exp -= 1;
        }
        // Decimal point or early exit if we won't write any decimal places
        if dp == 0 {
            return Ok(());
        }
        if f.precision().is_none() && num.peek().is_none() {
            return Ok(());
        }
        f.write_str(".")?;

        // After the decimal point
        while exp < -1 && dp > 0 {
            f.write_str("0")?;
            exp += 1;
            dp -= 1;
        }
        for digit in num {
            if dp == 0 {
                return Ok(());
            }
            dp -= 1;
            write!(f, "{digit}")?;
        }
        if f.precision().is_none() {
            return Ok(());
        }
        for _ in 0..dp {
            f.write_str("0")?;
        }
        Ok(())
    }
}

/// Approximate scientific presentation with an uppercase E exponent marker.
/// Use explicit mantissa places or default 32 with trailing zeros suppressed.
/// The default leading-bit search can demand deep refinement for tiny or
/// unresolved values; it supplies no total-work or memory bound. Only precision
/// and explicit plus are honored among the formatting controls used here.
/// Use [`crate::Real::try_scientific`] for checked rounding and typed failures.
impl fmt::UpperExp for Computable {
    /// Find the leading-bit estimate, sample, extract mantissa digits and write the exponent.
    /// Explicit precision preserves mantissa zeros; default precision trims them.
    /// Propagate writer errors only, with no independent cancellation or numerical
    /// validation step around the underlying approximation.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let msd = self.iter_msd();
        let precision = f.precision();
        // Precision does not include the first digit before the decimal point
        let exact = precision.unwrap_or(DEFAULT_PRECISION);
        let bits = enough_bits(msd, f.precision());
        let appr = self.approx(msd - bits);
        if self.sign_until(msd - bits) == Some(RealSign::Negative) {
            f.write_str("-")?;
        } else if f.sign_plus() {
            // Even for zero
            f.write_str("+")?;
        }

        let (num, exp) = digits(
            appr.magnitude(),
            Places::Exp(exact),
            bits,
            msd,
            precision.is_none(),
        );

        for (n, digit) in num.into_iter().enumerate() {
            if n == 1 {
                f.write_str(".")?;
            }
            write!(f, "{digit}")?;
        }
        write!(f, "E{exp}")?;
        Ok(())
    }
}

/// Approximate scientific presentation with a lowercase e exponent marker.
/// Share the uppercase formatter's sample/digit strategy and default precision,
/// including deep leading-bit search and its resource limitations. Width,
/// alignment, fill, zero padding and alternate flags are ignored.
/// Use [`crate::Real::try_scientific`] for checked decimal rounding/failure receipts.
impl fmt::LowerExp for Computable {
    /// Find the leading-bit estimate, sample, extract mantissa digits and write the exponent.
    /// Explicit precision preserves mantissa zeros; default precision trims them.
    /// Propagate writer errors only, with no independent cancellation or numerical
    /// validation step around the underlying approximation.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let msd = self.iter_msd();
        let precision = f.precision();
        // Precision does not include the first digit before the decimal point
        let exact = precision.unwrap_or(DEFAULT_PRECISION);
        let bits = enough_bits(msd, f.precision());
        let appr = self.approx(msd - bits);
        if self.sign_until(msd - bits) == Some(RealSign::Negative) {
            f.write_str("-")?;
        } else if f.sign_plus() {
            // Even for zero
            f.write_str("+")?;
        }

        let (num, exp) = digits(
            appr.magnitude(),
            Places::Exp(exact),
            bits,
            msd,
            precision.is_none(),
        );

        for (n, digit) in num.into_iter().enumerate() {
            if n == 1 {
                f.write_str(".")?;
            }
            write!(f, "{digit}")?;
        }
        write!(f, "e{exp}")?;
        Ok(())
    }
}

#[cfg(test)]
/// Literal formatting regressions, digit-helper cases and precision-demand checks.
/// The fixtures exercise presentation of exact sources; they do not make the
/// diagnostic formatter a certified decimal conversion API.
mod tests {
    use super::*;
    use crate::Rational;

    #[test]
    /// Check scientific output for the squared exact 10^-12 fixture.
    /// Exercise zero/two/four/eight explicit mantissa places, both exponent cases
    /// and default trailing-zero suppression for 10^-24.
    fn exp_smol() {
        let smol = Computable::rational(Rational::fraction(1, 1_000_000_000_000).unwrap());
        let ans = smol.clone().multiply(smol);
        assert_eq!(format!("{ans:.0e}"), "1e-24");
        assert_eq!(format!("{ans:.2E}"), "1.00E-24");
        assert_eq!(format!("{ans:.4e}"), "1.0000e-24");
        assert_eq!(format!("{ans:.8E}"), "1.00000000E-24");
        assert_eq!(format!("{ans:e}"), "1e-24");
    }

    #[test]
    /// Check scientific digit counts and internal zeros for 11, 101, 10001 and 1000000001.
    /// Compare literal outputs at several explicit precisions and the default;
    /// these are formatting regressions, not a universal rounding proof.
    fn pinch() {
        let ans = Computable::rational(Rational::new(11));
        assert_eq!(format!("{ans:.0e}"), "1e1");
        assert_eq!(format!("{ans:.1E}"), "1.1E1");
        assert_eq!(format!("{ans:e}"), "1.1e1");
        let ans = Computable::rational(Rational::new(101));
        assert_eq!(format!("{ans:.0e}"), "1e2");
        assert_eq!(format!("{ans:.1E}"), "1.0E2");
        assert_eq!(format!("{ans:.2e}"), "1.01e2");
        assert_eq!(format!("{ans:e}"), "1.01e2");
        let ans = Computable::rational(Rational::new(10001));
        assert_eq!(format!("{ans:.0e}"), "1e4");
        assert_eq!(format!("{ans:.2E}"), "1.00E4");
        assert_eq!(format!("{ans:.4e}"), "1.0001e4");
        assert_eq!(format!("{ans:e}"), "1.0001e4");
        let ans = Computable::rational(Rational::new(1_000_000_001));
        assert_eq!(format!("{ans:.0e}"), "1e9");
        assert_eq!(format!("{ans:.8E}"), "1.00000000E9");
        assert_eq!(format!("{ans:.10e}"), "1.0000000010e9");
        assert_eq!(format!("{ans:e}"), "1.000000001e9");
    }

    #[test]
    /// Check scientific carries from near powers of ten and rounding 12346.
    /// Assert the retained exponent/mantissa strings, including all-nine carry
    /// to the next decimal exponent and the three-place 1.235e4 case.
    fn almost() {
        let ans = Computable::rational(Rational::fraction(99, 10).unwrap());
        assert_eq!(format!("{ans:.0e}"), "1e1");
        let ans = Computable::rational(Rational::fraction(999, 10).unwrap());
        assert_eq!(format!("{ans:.0e}"), "1e2");
        let ans = Computable::rational(Rational::fraction(9999, 10).unwrap());
        assert_eq!(format!("{ans:.0e}"), "1e3");
        let ans = Computable::rational(Rational::new(12346));
        assert_eq!(format!("{ans:.0E}"), "1E4");
        assert_eq!(format!("{ans:.3e}"), "1.235e4");
    }

    #[test]
    /// Check sign, exponent and zero padding for the exact value -5*10^37.
    /// The fixture is assembled through multiplication and compared with literal
    /// scientific strings at explicit/default mantissa precision.
    fn exp_huge_neg() {
        let huge = Computable::rational(Rational::new(1_000_000_000_000_000_000));
        let minus_fifty = Computable::rational(Rational::new(-50));
        let ans = huge.clone().multiply(huge).multiply(minus_fifty);
        assert_eq!(format!("{ans:.4e}"), "-5.0000e37");
        assert_eq!(format!("{ans:.0e}"), "-5e37");
        assert_eq!(format!("{ans:.2e}"), "-5.00e37");
        assert_eq!(format!("{ans:.8e}"), "-5.00000000e37");
        assert_eq!(format!("{ans:e}"), "-5e37");
    }

    #[test]
    /// Check scientific formatting of the repeated-square value 10^594/3.
    /// Assert exponent 593, the leading repeating threes and explicit/default
    /// mantissa lengths without materializing an expected huge decimal integer.
    fn exp_bigger() {
        let mut huge = Computable::rational(Rational::new(1_000_000_000_000_000_000));
        huge = huge.clone().multiply(huge);
        huge = huge.clone().multiply(huge);
        huge = huge.clone().multiply(huge);
        huge = huge.clone().multiply(huge);
        let fraction =
            Computable::rational(Rational::fraction(1_000_000_000_000_000_000, 3).unwrap());
        let ans = huge.clone().multiply(huge).multiply(fraction);
        assert_eq!(format!("{ans:.4e}"), "3.3333e593");
        assert_eq!(format!("{ans:.0e}"), "3e593");
        assert_eq!(format!("{ans:.2e}"), "3.33e593");
        assert_eq!(format!("{ans:.8e}"), "3.33333333e593");
        assert_eq!(format!("{ans:e}"), "3.33333333333333333333333333333333e593");
    }

    #[test]
    /// Check the repeating two-thirds fraction in scientific notation.
    /// Literal expectations cover zero/two/four/eight places and the default
    /// 32 mantissa places, including the rounded final digit.
    fn exp_two_thirds() {
        let tt = Computable::rational(Rational::fraction(2, 3).unwrap());
        assert_eq!(format!("{tt:.0e}"), "7e-1");
        assert_eq!(format!("{tt:.2e}"), "6.67e-1");
        assert_eq!(format!("{tt:.4e}"), "6.6667e-1");
        assert_eq!(format!("{tt:.8e}"), "6.66666667e-1");
        assert_eq!(format!("{tt:e}"), "6.66666666666666666666666666666667e-1");
    }

    #[test]
    /// Check lowercase scientific pi against fixed reference strings through 32 places.
    /// Explicit precision retains the last zero at 32 places; default formatting
    /// trims it. This finite corpus does not prove all precision/rounding cases.
    fn exp_pi() {
        let pi = Computable::pi();
        assert_eq!(format!("{pi:.2e}"), "3.14e0");
        assert_eq!(format!("{pi:.4e}"), "3.1416e0");
        assert_eq!(format!("{pi:.8e}"), "3.14159265e0");
        assert_eq!(format!("{pi:.16e}"), "3.1415926535897932e0");
        assert_eq!(format!("{pi:.32e}"), "3.14159265358979323846264338327950e0");
        assert_eq!(format!("{pi:e}"), "3.1415926535897932384626433832795e0");
    }

    #[test]
    /// Check fixed output of exact 8*10^-9 across coarse and finer precision.
    /// Expect displayed zero at coarse places, a last-place increment at eight
    /// places and the nonzero default string; zero text is not a value assertion.
    fn disp_tiny() {
        let tiny = Computable::rational(Rational::fraction(8, 1_000_000_000).unwrap());
        assert_eq!(format!("{tiny:.0}"), "0");
        assert_eq!(format!("{tiny:.2}"), "0.00");
        assert_eq!(format!("{tiny:.8}"), "0.00000001");
        assert_eq!(format!("{tiny}"), "0.000000008");
    }

    #[test]
    /// Check coarse displayed zero and default nonzero text for exact 4*10^-6.
    /// Explicit precision retains requested fractional zeros.
    fn disp_small() {
        let smol = Computable::rational(Rational::fraction(4, 1_000_000).unwrap());
        assert_eq!(format!("{smol:.0}"), "0");
        assert_eq!(format!("{smol:.2}"), "0.00");
        assert_eq!(format!("{smol}"), "0.000004");
    }

    #[test]
    /// Check an exact nine-digit integer with zero, two and default fractional places.
    /// Explicit places pad zeros while the default omits the decimal suffix.
    fn disp_big() {
        let big = Computable::rational(Rational::new(123456789));
        assert_eq!(format!("{big:.0}"), "123456789");
        assert_eq!(format!("{big:.2}"), "123456789.00");
        assert_eq!(format!("{big}"), "123456789");
    }

    #[test]
    /// Check rational zero under fixed and scientific formatting.
    /// Assert default fixed zero, explicit fractional padding and exponent zero
    /// in both a padded uppercase and zero-place lowercase case.
    fn actual_zero() {
        let zero = Computable::rational(Rational::zero());
        assert_eq!(format!("{zero}"), "0");
        assert_eq!(format!("{zero:.10}"), "0.0000000000");
        assert_eq!(format!("{zero:.5E}"), "0.00000E0");
        assert_eq!(format!("{zero:.0e}"), "0e0");
    }

    #[test]
    /// Check fixed text and sampled precision for sqrt(2)^2-2 at five place counts.
    /// Assert ordinary/explicit-plus zero strings and a cached precision no finer
    /// than the guarded demand minus two bits. This checks precision demand,
    /// not a bound on total work, allocations or wall time.
    fn disp_cancellation_zero_has_bounded_demand() {
        for places in [0, 1, 8, 32, 128] {
            let radical = Computable::rational(Rational::new(2)).sqrt();
            let zero = radical
                .clone()
                .multiply(radical)
                .add(Computable::rational(Rational::new(-2)));
            let expected = if places == 0 {
                "0".to_owned()
            } else {
                format!("0.{}", "0".repeat(places))
            };
            assert_eq!(format!("{zero:.places$}"), expected);
            assert_eq!(format!("{zero:+.places$}"), format!("+{expected}"));
            let (precision, _) = zero.internal.cache_snapshot().unwrap();
            assert!(precision >= -enough_bits(0, Some(places)) - 2);
        }
    }

    #[test]
    /// Check both signs of sqrt(2)*2^-4096 before and after finer cache warming.
    /// Assert two-place zero strings, including negative zero, and the initial
    /// cached-precision floor. Refine to -4200 and require the same two-place and
    /// default text. The demand assertion is not a total resource bound.
    fn disp_tiny_computable_has_bounded_demand() {
        for negative in [false, true] {
            let tiny = Computable::rational(Rational::new(2))
                .sqrt()
                .shift_left(-4096);
            let tiny = if negative { tiny.negate() } else { tiny };
            let expected = if negative { "-0.00" } else { "0.00" };
            assert_eq!(format!("{tiny:.2}"), expected);
            let (precision, _) = tiny.internal.cache_snapshot().unwrap();
            assert!(precision >= -enough_bits(0, Some(2)) - 2);
            let _ = tiny.approx(-4200);
            assert_eq!(format!("{tiny:.2}"), expected);
            assert_eq!(format!("{tiny}"), if negative { "-0" } else { "0" });
        }
    }

    #[test]
    /// Check five positive rational examples that display as zero at zero places.
    /// The expected string tests presentation only, not equality to exact zero.
    fn disp_too_small() {
        let ratios = [(1, 3), (1, 4), (2, 5), (1, 6), (3, 7)];
        for ratio in ratios {
            let ans = Computable::rational(Rational::fraction(ratio.0, ratio.1).unwrap());
            assert_eq!(format!("{ans:.0}"), "0");
        }
    }

    #[test]
    /// Check five positive fractions displayed as one at zero places.
    /// The corpus includes the existing 1/2 tie behavior; it does not establish a
    /// uniform ties-to-even or ties-away rule for the formatter.
    fn disp_one() {
        let ratios = [(1, 2), (3, 4), (3, 5), (5, 6), (4, 7)];
        for ratio in ratios {
            let ans = Computable::rational(Rational::fraction(ratio.0, ratio.1).unwrap());
            assert_eq!(format!("{ans:.0}"), "1");
        }
    }

    #[test]
    /// Check fixed one-third output at zero/two/default fractional precision.
    /// Compare literal repeating-digit strings without treating them as exact values.
    fn disp_one_third() {
        let ot = Computable::rational(Rational::fraction(1, 3).unwrap());
        assert_eq!(format!("{ot:.0}"), "0");
        assert_eq!(format!("{ot:.2}"), "0.33");
        assert_eq!(format!("{ot}"), "0.33333333333333333333333333333333");
    }

    #[test]
    /// Check fixed output of the composed value 60*pi at six precision choices.
    /// Reference strings include explicit trailing zero padding and the default
    /// 32 fractional places; the source remains an exact retained product.
    fn disp_sixty_pi() {
        let pi = Computable::pi();
        let sixty = Computable::rational(Rational::new(60));
        let sp = pi.multiply(sixty);
        assert_eq!(format!("{sp:.0}"), "188");
        assert_eq!(format!("{sp:.2}"), "188.50");
        assert_eq!(format!("{sp:.8}"), "188.49555922");
        assert_eq!(format!("{sp:.16}"), "188.4955592153875943");
        assert_eq!(format!("{sp:.32}"), "188.49555921538759430775860299677017");
        assert_eq!(format!("{sp}"), "188.49555921538759430775860299677017");
    }

    /// Recognize exactly 0. followed by the requested number of ASCII ones.
    /// Check byte length, prefix and every remaining character. The test caller
    /// supplies place counts for which the length addition fits usize.
    fn check_ninth(s: &str, places: usize) -> bool {
        if s.len() != places + 2 {
            return false;
        }
        let Some(rest) = s.strip_prefix("0.") else {
            return false;
        };
        rest.chars().all(|c| c == '1')
    }

    #[test]
    /// Check one-ninth at default precision and 3, 17, 123 and 4567 explicit places.
    /// Use the shared local string predicate to verify every repeating digit and
    /// length; the large case exercises output allocation without claiming a budget.
    fn disp_ninth() {
        let ninth = Computable::rational(Rational::fraction(1, 9).unwrap());
        assert_eq!(format!("{ninth}"), "0.11111111111111111111111111111111");
        for places in [3, 17, 123, 4567] {
            let formatted = format!("{ninth:.*}", places);
            assert!(
                check_ninth(&formatted, places),
                "{formatted} was not formatted as expected to {places} decimal places"
            );
        }
    }

    #[test]
    /// Check all-nine carry without trimming for the sample 1-2^-129.
    /// With 32 mantissa places, require exponent zero and one followed by 32 zeros.
    /// The integer fixture is 2^129-1 in units of 2^-129.
    fn digits_round_exp_false() {
        let magn: BigUint = "680564733841876926926749214863536422911".parse().unwrap();
        let (v, exp) = digits(&magn, Places::Exp(32), 128, -1, false);
        assert_eq!(exp, 0);
        assert_eq!(
            v,
            [
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0
            ]
        );
    }

    #[test]
    /// Check the same 1-2^-129 sample under fixed 32-place extraction with trimming.
    /// Require exponent zero and a single retained one after the carry.
    fn digits_round_zero_true() {
        let magn: BigUint = "680564733841876926926749214863536422911".parse().unwrap();
        let (v, exp) = digits(&magn, Places::Zero(32), 128, -1, true);
        assert_eq!(exp, 0);
        assert_eq!(v, [1]);
    }

    #[test]
    /// Check default scientific rounding of 38- and 39-digit all-nine fractions.
    /// Both retained exact rationals must display as 1e0 at the default precision.
    fn disp_almost_one_exp() {
        let dp38: Rational = "0.99999999999999999999999999999999999999".parse().unwrap();
        let almost = Computable::rational(dp38);
        assert_eq!(format!("{almost:e}"), "1e0");
        let dp39: Rational = "0.999999999999999999999999999999999999999".parse().unwrap();
        let almost = Computable::rational(dp39);
        assert_eq!(format!("{almost:e}"), "1e0");
    }

    #[test]
    /// Check default fixed rounding of 38- and 39-digit all-nine fractions.
    /// Both exact inputs are below one but their default presentation is one.
    fn disp_almost_one() {
        let dp38: Rational = "0.99999999999999999999999999999999999999".parse().unwrap();
        let almost = Computable::rational(dp38);
        assert_eq!(format!("{almost}"), "1");
        let dp39: Rational = "0.999999999999999999999999999999999999999".parse().unwrap();
        let almost = Computable::rational(dp39);
        assert_eq!(format!("{almost}"), "1");
    }

    #[test]
    /// Check fixed pi against literal reference digits through 3062 decimal places.
    /// Cover explicit and default trailing-zero behavior plus the 1000- and
    /// 3062-place strings. These exact text regressions do not certify every input
    /// or precision supported by the generic formatter.
    fn disp_pi() {
        let pi = Computable::pi();
        assert_eq!(format!("{pi}"), "3.1415926535897932384626433832795");
        assert_eq!(format!("{pi:.2}"), "3.14");
        assert_eq!(format!("{pi:.4}"), "3.1416");
        assert_eq!(format!("{pi:.8}"), "3.14159265");
        assert_eq!(format!("{pi:.16}"), "3.1415926535897932");
        assert_eq!(format!("{pi:.32}"), "3.14159265358979323846264338327950");
        assert_eq!(
            format!("{pi:.1000}"),
            "3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679821480865132823066470938446095505822317253594081284811174502841027019385211055596446229489549303819644288109756659334461284756482337867831652712019091456485669234603486104543266482133936072602491412737245870066063155881748815209209628292540917153643678925903600113305305488204665213841469519415116094330572703657595919530921861173819326117931051185480744623799627495673518857527248912279381830119491298336733624406566430860213949463952247371907021798609437027705392171762931767523846748184676694051320005681271452635608277857713427577896091736371787214684409012249534301465495853710507922796892589235420199561121290219608640344181598136297747713099605187072113499999983729780499510597317328160963185950244594553469083026425223082533446850352619311881710100031378387528865875332083814206171776691473035982534904287554687311595628638823537875937519577818577805321712268066130019278766111959092164201989"
        );
        assert_eq!(
            format!("{pi:.3062}"),
            "3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651328230664709384460955058223172535940812848111745028410270193852110555964462294895493038196442881097566593344612847564823378678316527120190914564856692346034861045432664821339360726024914127372458700660631558817488152092096282925409171536436789259036001133053054882046652138414695194151160943305727036575959195309218611738193261179310511854807446237996274956735188575272489122793818301194912983367336244065664308602139494639522473719070217986094370277053921717629317675238467481846766940513200056812714526356082778577134275778960917363717872146844090122495343014654958537105079227968925892354201995611212902196086403441815981362977477130996051870721134999999837297804995105973173281609631859502445945534690830264252230825334468503526193118817101000313783875288658753320838142061717766914730359825349042875546873115956286388235378759375195778185778053217122680661300192787661119590921642019893809525720106548586327886593615338182796823030195203530185296899577362259941389124972177528347913151557485724245415069595082953311686172785588907509838175463746493931925506040092770167113900984882401285836160356370766010471018194295559619894676783744944825537977472684710404753464620804668425906949129331367702898915210475216205696602405803815019351125338243003558764024749647326391419927260426992279678235478163600934172164121992458631503028618297455570674983850549458858692699569092721079750930295532116534498720275596023648066549911988183479775356636980742654252786255181841757467289097777279380008164706001614524919217321721477235014144197356854816136115735255213347574184946843852332390739414333454776241686251898356948556209921922218427255025425688767179049460165346680498862723279178608578438382796797668145410095388378636095068006422512520511739298489608412848862694560424196528502221066118630674427862203919494504712371378696095636437191728746776465757396241389086583264599581339047802759009946576407895126946839835259570982582262052248940772671947826848260147699090264013639443745530506820349625245174939965143142980919065925093722169646151570985838741059788595977297549893016175392846813826868386894277415599185592524595395943104997252468084598727364469584865383673622262609912460805124388439045124413654976278079771569143599770012961608944169486855584840635342207222582848864815845602850601684273945226746767889525213852254995466672782398645659611635488623057745649803559363456817432411251507606947945109659609402522887971089314566913686722874894056010150330861792868092087476091782493858900971490967598526136554978189312978482168299894872265880485756401427047755513237964145152374623436454285844479526586782105114135473573952311342716610213596953623144295248493718711014576540359027993440374200731057853906219838744780847848968332144571386875194350643021845319104848100537061468067491927819119793995206141966342875444064374512371819217999839101591956181467514269123974894090718649423196156794520809514655022523160388193014209376213785595663893778708"
        );
    }

    #[test]
    /// Check fixed two-thirds output at zero/two/four/eight/default places.
    /// Literal expected strings exercise final-digit increments in a repeating fraction.
    fn disp_two_thirds() {
        let tt = Computable::rational(Rational::fraction(2, 3).unwrap());
        assert_eq!(format!("{tt:.0}"), "1");
        assert_eq!(format!("{tt:.2}"), "0.67");
        assert_eq!(format!("{tt:.4}"), "0.6667");
        assert_eq!(format!("{tt:.8}"), "0.66666667");
        assert_eq!(format!("{tt}"), "0.66666666666666666666666666666667");
    }

    #[test]
    /// Check fixed output of 10^594/3 after repeated exact squaring and multiplication.
    /// Strip threes from both ends and require only the decimal point, or empty text
    /// at zero places. This assertion checks digit shape, not the integer length
    /// or an independent exact-value reconstruction.
    fn disp_threes() {
        let mut huge = Computable::rational(Rational::new(1_000_000_000_000_000_000));
        huge = huge.clone().multiply(huge);
        huge = huge.clone().multiply(huge);
        huge = huge.clone().multiply(huge);
        huge = huge.clone().multiply(huge);
        let fraction =
            Computable::rational(Rational::fraction(1_000_000_000_000_000_000, 3).unwrap());
        let ans = huge.clone().multiply(huge).multiply(fraction);
        assert_eq!(format!("{ans:.4}").trim_matches('3'), ".");
        assert_eq!(format!("{ans:.0}").trim_matches('3'), "");
        assert_eq!(format!("{ans:.2}").trim_matches('3'), ".");
        assert_eq!(format!("{ans:.8}").trim_matches('3'), ".");
        assert_eq!(format!("{ans}").trim_matches('3'), ".");
    }
}
