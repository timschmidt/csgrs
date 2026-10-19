//! Full native triplet enclosures versus independent MPFR point calculations.
use hyperreal::{Rational, Real};
use std::{hint::black_box, time::Instant};
#[path = "../tests/support/inverse_legendre_oracle.rs"]
mod inverse_oracle;
#[path = "../tests/support/jacobi_oracle.rs"]
mod jacobi_oracle;
#[path = "../tests/support/legendre_oracle.rs"]
#[allow(dead_code)] // Complete AGM is covered by tests; this bench uses the series inverse.
mod legendre_oracle;

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
