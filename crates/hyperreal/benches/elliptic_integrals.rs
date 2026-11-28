//! Carlson RF enclosure timings against an independently formulated MPFR AGM point.
//!
//! The fixed complement y=2^-bits approaches the complete-integral singularity at
//! y=0. Native output is an exact expression followed by a certified rational
//! enclosure; MPFR output is a rounded point. Acceptance checks and timing scopes
//! are described on `main`. These different completed contracts do not establish
//! a kernel speedup or a universal correctness claim.
//!
//! These are custom executables, not Criterion harnesses: arguments are ignored and
//! JSON rows go to stdout without updating Markdown reports. Domain, enclosure and
//! acceptance failures panic. A row is printed before its accepted-count assertion;
//! consumers must check both counts and process success. Fixed repetition counts
//! and precision requests do not impose a total-work, memory or cancellation bound.
//! No explicit MPFR cache-cleanup guard is installed here.

use hyperreal::{Rational, Real};
use rug::{Float, float::Constant};
use std::{hint::black_box, time::Instant};

/// Round an exact signed rational to a 1024-bit MPFR validation point.
///
/// Transfer numerator and denominator as integer decimal strings, restore the sign,
/// and round the resulting rug rational with MPFR's default nearest rounding.
/// This avoids mixed-fraction Display parsing but supplies no directed endpoint
/// bound. Integer parsing is unwrapped; conversion is outside the timed loop.
fn mpfr_rational(value: &Rational) -> Float {
    let n: rug::Integer = value.numerator().to_string().parse().unwrap();
    let d: rug::Integer = value.denominator().to_string().parse().unwrap();
    let r = rug::Rational::from((n, d));
    Float::with_val(
        1024,
        if value.sign() == num::bigint::Sign::Minus {
            -r
        } else {
            r
        },
    )
}

/// Approximate RF(0, 2^-bits, 1) as pi / (2 AGM(1, sqrt(2^-bits))).
///
/// Construct the complement and pi at the requested MPFR precision; square root,
/// AGM and arithmetic produce rounded points, not an enclosing interval. Current
/// callers use bits 1, 20 or 128 and precision 256 (timed) or 1024 (expected).
/// No general input-range, rounding-error or resource proof is provided here.
fn reference(bits: u32, precision: u32) -> Float {
    let y = Float::with_val(precision, 1) >> bits;
    let agm = Float::with_val(precision, 1).agm(&y.sqrt());
    Float::with_val(precision, Constant::Pi) / (agm * 2)
}

/// Construct exact RF(0, 2^-bits, 1) from a rational power of one half.
///
/// The finite nonnegative exponent gives a positive complement, so the fixed
/// fixtures satisfy Carlson RF's nonnegative domain with exactly one zero argument.
/// Power and RF results are unwrapped, turning a construction failure into a panic.
/// The returned exact expression can be refined later; construction may itself do
/// admission work and is not promised to be evaluation-free or resource-bounded.
fn native(bits: u32) -> Real {
    let y = Real::new(Rational::fraction(1, 2).unwrap().powi(bits.into()).unwrap());
    Real::elliptic_rf(&Real::zero(), &y, &Real::one()).unwrap()
}

/// Emit 45 aggregate timing rows for three complements, five trials and three modes.
///
/// For each complement, compute a 1024-bit AGM expected point outside timing. Before
/// every mode, construct and request a -112 native enclosure, even for the MPFR
/// mode. Cold rows reconstruct native expressions ten times; warm rows clone the
/// pre-refined expression 100 times; MPFR rows compute 256-bit AGM points 100 times.
/// Cold denotes fresh result expressions, not fresh-process or cleared-cache work.
///
/// The timer includes construction or cloning, evaluation, vector allocation and
/// row retention. Expected-point preparation, warm-up, validation, printing and
/// retained-row destruction are outside it. Native expression temporaries can be
/// destroyed inside the loop; only their enclosures are stored.
///
/// Validation requires exact rational width below 2^-96 and containment of the
/// expected point between nearest-rounded 1024-bit endpoint conversions for each
/// native row. MPFR rows require absolute point difference strictly below 2^-96.
/// The endpoint conversions are not directed, and the expected point is not a
/// certified enclosure of the mathematical answer. Both MPFR paths use the same
/// AGM formula. All repeats must pass, but this finite check is not an independent
/// rounding proof or a matched-output performance comparison.
fn main() {
    for bits in [1, 20, 128] {
        let expected = reference(bits, 1024);
        for trial in 0..5 {
            for mode in [
                "native_cold_enclosure",
                "native_warm_enclosure",
                "mpfr_agm_point",
            ] {
                let warm = native(bits);
                let _ = warm.certified_rational_interval(-112).unwrap();
                let repeats = if mode == "native_cold_enclosure" {
                    10
                } else {
                    100
                };
                let start = Instant::now();
                let mut rows = Vec::new();
                for _ in 0..repeats {
                    if mode == "mpfr_agm_point" {
                        let value = black_box(reference(bits, 256));
                        rows.push(Err(value));
                    } else {
                        let value = if mode == "native_cold_enclosure" {
                            native(bits)
                        } else {
                            warm.clone()
                        };
                        rows.push(Ok(black_box(
                            value.certified_rational_interval(-112).unwrap(),
                        )));
                    }
                }
                let elapsed_ns = start.elapsed().as_nanos();
                let mut accepted = 0;
                for row in rows {
                    let valid = match row {
                        Ok([lo, hi]) => {
                            mpfr_rational(&lo) <= expected
                                && expected <= mpfr_rational(&hi)
                                && hi - lo
                                    < Rational::fraction(1, 2).unwrap().powi(96.into()).unwrap()
                        }
                        Err(value) => (value - &expected).abs() < (Float::with_val(1024, 1) >> 96),
                    };
                    accepted += u32::from(valid);
                }
                println!(
                    "{}",
                    serde_json::json!({
                        "complement": format!("2^-{bits}"), "trial": trial, "mode": mode,
                        "repeats": repeats, "accepted": accepted, "elapsed_ns": elapsed_ns,
                        "timing": "construction/evaluation and row storage; excludes validation",
                        "contract": "native exact expression plus certified enclosure; MPFR point only; 96-bit acceptance",
                    })
                );
                assert_eq!(accepted, repeats);
            }
        }
    }
}
