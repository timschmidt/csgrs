//! Completion-checked native enclosure and MPFR point-reference timings.
//! The output contracts differ; these rows do not establish a kernel speedup.
use hyperreal::{Rational, Real};
use rug::{Float, float::Constant};
use std::{hint::black_box, time::Instant};

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

fn reference(bits: u32, precision: u32) -> Float {
    let y = Float::with_val(precision, 1) >> bits;
    let agm = Float::with_val(precision, 1).agm(&y.sqrt());
    Float::with_val(precision, Constant::Pi) / (agm * 2)
}

fn native(bits: u32) -> Real {
    let y = Real::new(Rational::fraction(1, 2).unwrap().powi(bits.into()).unwrap());
    Real::elliptic_rf(&Real::zero(), &y, &Real::one()).unwrap()
}

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
