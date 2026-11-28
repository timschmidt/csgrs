//! Unwrapped inverse-Legendre enclosure timings versus a rounded MPFR root reference.
//!
//! Native results retain the inverse equation for F or E. The development reference
//! uses safeguarded Newton iteration over independent forward-series evaluations
//! and checks its final rounded residual. That point calculation and native exact
//! expressions with certified enclosures have different output contracts; the rows
//! do not establish a kernel speedup or a directed error proof for the MPFR root.
//!
//! These are custom executables, not Criterion harnesses: arguments are ignored and
//! JSON rows go to stdout without updating Markdown reports. Domain, enclosure and
//! acceptance failures panic. A row is printed before its accepted-count assertion;
//! consumers must check both counts and process success. Fixed repetition counts
//! and precision requests do not impose a total-work, memory or cancellation bound.
//! No explicit MPFR cache-cleanup guard is installed here.

use hyperreal::{Rational, Real};
use std::{hint::black_box, time::Instant};

/// Independent 1024-bit safeguarded inverse reference with a final residual check.
///
/// The helper admits rounded abs(u)<4 and abs(m)<=3/4, caps Newton steps at 24 and
/// requires final residual magnitude below 2^-800. It panics on failed fixture
/// conditions and supplies a rounded point, not a certified root enclosure.
#[path = "../tests/support/inverse_legendre_oracle.rs"]
mod inverse_oracle;
/// Shared forward series/adapters used by the inverse reference and point import.
///
/// The benchmark calls its exact binary-rational MPFR import through the `oracle`
/// alias. The sibling inverse helper also uses its input conversions and series;
/// only the complete AGM function is unused in this benchmark.
#[path = "../tests/support/legendre_oracle.rs"]
#[allow(dead_code)] // Complete AGM is unused by this benchmark and its inverse oracle.
mod legendre_oracle;
use legendre_oracle as oracle;

/// Emit 120 rows for four argument/parameter pairs, two inverses and three modes.
///
/// The exact pairs (u,m) are (3/4,1/2), (3/4,-3/4), (7/2,1/2) and (-7/2,-3/4).
/// For each, test the continuous inverse of F (Jacobi amplitude) and inverse of E,
/// five trials each; amplitudes are not wrapped to a principal interval. Parse
/// inputs and adopt one 1024-bit reference point as an exact binary rational before
/// timing. Each reference evaluation solves using both forward-series components.
///
/// Before every mode, even MPFR, construct and request a -112 native enclosure.
/// Cold rows then construct three fresh expressions; warm rows clone that prepared
/// expression 25 times; MPFR rows compute 25 inverse points. Cold does not reset
/// process/global caches. The timer includes construction/cloning, evaluation,
/// vector allocation and row storage, plus expression temporaries destroyed in the
/// loop. Preparation, validation, printing and retained-row drops are excluded.
///
/// Each native enclosure must contain the adopted reference rational and have exact
/// width below 2^-96. Each MPFR point-minus-expectation enclosure requested at -112
/// must lie inside inclusive bounds -2^-96 and 2^-96. All repeats must be accepted.
/// Repeating the same MPFR solver as the expectation checks reproducibility against
/// that rounded point, not an independent exact root or a directed rounding proof.
/// Construction/interval/reference failures are unwrapped or asserted, not reported
/// as typed benchmark outcomes.
fn main() {
    for (name, argument, parameter) in [
        ("principal_oblate", "0.75", "0.5"),
        ("principal_prolate", "0.75", "-0.75"),
        ("unwrapped_positive", "3.5", "0.5"),
        ("unwrapped_negative", "-3.5", "-0.75"),
    ] {
        let u: Real = argument.parse().unwrap();
        let m: Real = parameter.parse().unwrap();
        for second in [false, true] {
            let reference = || inverse_oracle::inverse(&u, &m, second);
            let native = || {
                if second {
                    u.clone().inverse_elliptic_e(&m)
                } else {
                    u.clone().jacobi_amplitude(&m)
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
