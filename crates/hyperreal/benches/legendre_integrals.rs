//! Exact native enclosures versus independent MPFR point calculations.
use hyperreal::{Rational, Real};
use std::{hint::black_box, time::Instant};

#[path = "../tests/support/legendre_oracle.rs"]
mod oracle;

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
