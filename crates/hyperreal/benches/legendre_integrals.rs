//! Complete and unwrapped Legendre F/E enclosure timings versus MPFR point work.
//!
//! The fixed native inputs use the parameter m (squared modulus), including negative
//! m, and radian amplitudes. The shared reference uses AGM for complete integrals
//! and a fixed binomial/sine-moment series for incomplete integrals. It returns
//! nearest-rounded points rather than directed error bounds. Comparing those points
//! with native exact expressions plus enclosures does not establish a kernel speedup.
//!
//! These are custom executables, not Criterion harnesses: arguments are ignored and
//! JSON rows go to stdout without updating Markdown reports. Domain, enclosure and
//! acceptance failures panic. A row is printed before its accepted-count assertion;
//! consumers must check both counts and process success. Fixed repetition counts
//! and precision requests do not impose a total-work, memory or cancellation bound.
//! No explicit MPFR cache-cleanup guard is installed here.

use hyperreal::{Rational, Real};
use std::{hint::black_box, time::Instant};

/// Development-only complete AGM, incomplete series and exact MPFR-point import.
///
/// Imported point values preserve their binary rational value, not exact equality
/// to the integral. The helper's domain, truncation and rounding limitations apply.
#[path = "../tests/support/legendre_oracle.rs"]
mod oracle;

/// Emit 120 timing rows covering four cases, two kinds, five trials and three modes.
///
/// Complete cases use m=1/2 and m=-3; their zero amplitude field is unused by the
/// integral calculation. Incomplete cases use (phi,m)=(3/4,1/2) and (15/2,-3/4).
/// Decimal strings are parsed exactly. Both kinds select K/F when `second` is false
/// and E when true. Each reference call computes both F/E or K/E before cloning
/// the selected result; this extra work remains in the timed MPFR path.
///
/// Adopt one 1024-bit reference point as an exact binary-rational Real expectation
/// outside timing. Before every mode, including MPFR, construct and request a -112
/// native enclosure. Cold iterations construct three fresh native expressions;
/// warm iterations clone the prepared expression 25 times; MPFR iterations compute
/// 25 reference points. Cold does not clear process/global caches.
///
/// Timing includes construction/cloning, enclosure or point evaluation, vector
/// allocation and result storage; it excludes preparation, validation, printing and
/// retained-row destruction. Temporary expression destruction can occur in the loop.
/// Each native row must contain the adopted reference rational and have exact width
/// strictly below 2^-96. For MPFR rows, the -112 enclosure of point minus expectation
/// must lie within the inclusive bounds -2^-96 and 2^-96.
///
/// Every repeat must pass. The MPFR comparison repeats the same reference algorithm;
/// its rounded expectation is not the exact integral or a directed oracle enclosure.
/// The finite checks therefore do not certify arbitrary inputs, oracle rounding or
/// comparable completed output contracts.
fn main() {
    for (name, complete, amplitude, parameter) in [
        ("complete_oblate", true, "0", "0.5"),
        ("complete_prolate", true, "0", "-3"),
        ("incomplete_principal", false, "0.75", "0.5"),
        ("incomplete_unwrapped", false, "7.5", "-0.75"),
    ] {
        let phi: Real = amplitude.parse().unwrap();
        let m: Real = parameter.parse().unwrap();
        for second in [false, true] {
            let reference = || {
                let pair = if complete {
                    oracle::complete(&(Real::one() - &m))
                } else {
                    oracle::series(&phi, &m)
                };
                pair[usize::from(second)].clone()
            };
            let native = || {
                match (complete, second) {
                    (true, false) => Real::elliptic_k(&m),
                    (true, true) => Real::elliptic_e_complete(&m),
                    (false, false) => phi.clone().elliptic_f(&m),
                    (false, true) => phi.clone().elliptic_e(&m),
                }
                .unwrap()
            };
            let expected = oracle::native(reference());
            for trial in 0..5 {
                for mode in [
                    "native_cold_enclosure",
                    "native_warm_enclosure",
                    "mpfr_point",
                ] {
                    let warm = native();
                    let _ = warm.certified_rational_interval(-112).unwrap();
                    let repeats = if mode == "native_cold_enclosure" {
                        3
                    } else {
                        25
                    };
                    let start = Instant::now();
                    let mut rows = Vec::new();
                    for _ in 0..repeats {
                        if mode == "mpfr_point" {
                            rows.push(Err(black_box(reference())));
                        } else {
                            let value = if mode == "native_cold_enclosure" {
                                native()
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
                        let tolerance = Rational::fraction(1, 2).unwrap().powi(96.into()).unwrap();
                        let valid = match row {
                            Ok([lo, hi]) => {
                                Real::new(lo.clone()) <= expected
                                    && expected <= Real::new(hi.clone())
                                    && hi - lo < tolerance
                            }
                            Err(point) => {
                                let [lo, hi] = (oracle::native(point) - &expected)
                                    .certified_rational_interval(-112)
                                    .unwrap();
                                lo >= -tolerance.clone() && hi <= tolerance
                            }
                        };
                        accepted += u32::from(valid);
                    }
                    println!(
                        "{}",
                        serde_json::json!({
                            "case": name, "second_kind": second, "trial": trial, "mode": mode,
                            "repeats": repeats, "accepted": accepted, "elapsed_ns": elapsed_ns,
                            "timing": "construction/evaluation and row storage; excludes validation",
                            "contract": "native exact expression plus enclosure; MPFR 1024-bit point only; 96-bit acceptance",
                        })
                    );
                    assert_eq!(accepted, repeats);
                }
            }
        }
    }
}
