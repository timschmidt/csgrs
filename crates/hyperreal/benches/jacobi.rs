//! Shared native Jacobi triplet enclosure timings versus MPFR sn/cn/dn points.
//!
//! Native evaluation constructs one exact amplitude shared by all three components;
//! the independent reference inverts F numerically, then evaluates sine, cosine and
//! the positive square root for dn. The result contracts differ: three native exact
//! expressions and enclosures versus three rounded points. These rows alone establish
//! neither a kernel speedup nor certified MPFR rounding.
//!
//! These are custom executables, not Criterion harnesses: arguments are ignored and
//! JSON rows go to stdout without updating Markdown reports. Domain, enclosure and
//! acceptance failures panic. A row is printed before its accepted-count assertion;
//! consumers must check both counts and process success. Fixed repetition counts
//! and precision requests do not impose a total-work, memory or cancellation bound.
//! No explicit MPFR cache-cleanup guard is installed here.

use hyperreal::{Rational, Real};
use std::{hint::black_box, time::Instant};
/// Residual-checked safeguarded MPFR inverse used by the sibling Jacobi reference.
///
/// Its rounded-input domain and final-residual assertions remain fixture checks;
/// this benchmark does not add a directed root enclosure or cleanup guard.
#[path = "../tests/support/inverse_legendre_oracle.rs"]
mod inverse_oracle;
/// Independent 1024-bit point calculation of sn, cn and positive-branch dn.
///
/// Consumes the sibling inverse and forward adapters, inheriting rounded abs(u)<4
/// and abs(m)<=3/4 guards and the inverse's final residual check.
#[path = "../tests/support/jacobi_oracle.rs"]
mod jacobi_oracle;
/// Forward series and scalar adapters shared with the inverse/Jacobi references.
///
/// Also adopts reference points as exact binary-rational Real expectations. This
/// preserves a rounded point's value, not exact equality to the Jacobi component;
/// the complete AGM helper is not used by this benchmark.
#[path = "../tests/support/legendre_oracle.rs"]
#[allow(dead_code)] // Complete AGM is covered by tests; this bench uses the series inverse.
mod legendre_oracle;

/// Emit 60 triplet timing rows for four pairs, five trials and three modes.
///
/// Use exact (u,m) pairs (3/4,1/2), (3/4,-3/4), (7/2,1/2) and (-7/2,-3/4).
/// The argument is a Jacobi input, not an amplitude. Adopt three 1024-bit reference
/// points as exact binary rationals outside timing. Before every mode, including
/// MPFR, construct a native triplet and request all three -112 enclosures.
///
/// Cold rows construct three fresh shared triplets; warm rows clone the prepared
/// triplet 25 times; MPFR rows compute 25 reference triplets. Cold does not reset
/// process/global caches. Timing includes triplet construction/cloning, all three
/// evaluations, vector allocation and row storage, with native temporaries dropped
/// inside the loop. Preparation, validation, printing and retained rows' destruction
/// are excluded. Sharing within a triplet is part of the native workload.
///
/// A native row passes only if all three enclosures contain their adopted reference
/// rationals and have exact widths below 2^-96. An MPFR row passes only if every
/// point-minus-expectation enclosure requested at -112 lies within inclusive bounds
/// -2^-96 and 2^-96. The fixed arrays ensure three components are compared; accepted
/// counts whole triplets, not individual components. All repeats must pass, but the
/// expectation repeats the same MPFR reference and is not a certified enclosure of
/// the mathematical answer. No identity or arbitrary-input conformance claim follows.
fn main() {
    for (name, argument, parameter) in [
        ("principal_oblate", "0.75", "0.5"),
        ("principal_prolate", "0.75", "-0.75"),
        ("unwrapped_positive", "3.5", "0.5"),
        ("unwrapped_negative", "-3.5", "-0.75"),
    ] {
        let u: Real = argument.parse().unwrap();
        let m: Real = parameter.parse().unwrap();
        let reference = || jacobi_oracle::values(&u, &m);
        let native = || {
            let (sn, cn, dn) = u.clone().jacobi_sncndn(&m).unwrap();
            [sn, cn, dn]
        };
        let expected = reference().map(legendre_oracle::native);
        for trial in 0..5 {
            for mode in [
                "native_cold_enclosure",
                "native_warm_enclosure",
                "mpfr_point",
            ] {
                let warm = native();
                for value in &warm {
                    let _ = value.certified_rational_interval(-112).unwrap();
                }
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
                        let values = if mode == "native_cold_enclosure" {
                            native()
                        } else {
                            warm.clone()
                        };
                        rows.push(Ok(black_box(
                            values.map(|v| v.certified_rational_interval(-112).unwrap()),
                        )));
                    }
                }
                let elapsed_ns = start.elapsed().as_nanos();
                let mut accepted = 0;
                for row in rows {
                    let tolerance = Rational::fraction(1, 2).unwrap().powi(96.into()).unwrap();
                    let valid = match row {
                        Ok(bounds) => {
                            bounds
                                .into_iter()
                                .zip(&expected)
                                .all(|([lo, hi], expected)| {
                                    Real::new(lo.clone()) <= *expected
                                        && *expected <= Real::new(hi.clone())
                                        && hi - lo < tolerance
                                })
                        }
                        Err(points) => {
                            points.into_iter().zip(&expected).all(|(point, expected)| {
                                let [lo, hi] = (legendre_oracle::native(point) - expected)
                                    .certified_rational_interval(-112)
                                    .unwrap();
                                lo >= -tolerance.clone() && hi <= tolerance
                            })
                        }
                    };
                    accepted += u32::from(valid);
                }
                println!(
                    "{}",
                    serde_json::json!({
                        "case": name, "trial": trial, "mode": mode, "repeats": repeats,
                        "accepted": accepted, "elapsed_ns": elapsed_ns,
                        "timing": "triplet construction/evaluation and row storage; excludes validation",
                        "contract": "three native exact expressions plus enclosures; three MPFR 1024-bit points only; 96-bit acceptance",
                    })
                );
                assert_eq!(accepted, repeats);
            }
        }
    }
}
