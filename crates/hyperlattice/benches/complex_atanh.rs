//! Complete principal atanh enclosures and the independent MPFR series oracle.
use hyperlattice::{Complex, Real};
use std::{hint::black_box, time::Instant};
#[path = "../tests/support/complex_atanh_oracle.rs"]
mod oracle;

fn components(argument: &Complex) -> [Real; 2] {
    let value = argument.clone().atanh().unwrap();
    [value.re, value.im]
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--test");
    let local = Complex::new("0.125".parse().unwrap(), "0.0625".parse().unwrap());
    for (text, argument) in [
        ("local", local.clone()),
        ("reciprocal", local.reciprocal().unwrap()),
        ("axis", Complex::new(Real::zero(), "0.25".parse().unwrap())),
    ] {
        oracle::compare(&argument);
        let reference = oracle::reference(&argument);
        let warm = components(&argument);
        let expected = warm
            .clone()
            .map(|v| v.certified_rational_interval(-112).unwrap());
        for trial in 0..if smoke { 1 } else { 5 } {
            for mode in ["native_cold", "native_warm", "mpfr_series"] {
                let repeats = if smoke || mode == "mpfr_series" { 1 } else { 5 };
                let start = Instant::now();
                let mut rows = Vec::new();
                for _ in 0..repeats {
                    rows.push(if mode == "mpfr_series" {
                        Err(black_box(oracle::reference(&argument)))
                    } else {
                        let values = if mode == "native_cold" {
                            components(&argument)
                        } else {
                            warm.clone()
                        };
                        Ok(black_box(
                            values.map(|v| v.certified_rational_interval(-112).unwrap()),
                        ))
                    });
                }
                let elapsed_ns = start.elapsed().as_nanos();
                let mut accepted = 0;
                for row in rows {
                    let valid = match row {
                        Ok(native) => reference
                            .iter()
                            .zip(native)
                            .all(|(a, b)| a.agrees_with(&b, 96)),
                        Err(reference) => reference
                            .iter()
                            .zip(&expected)
                            .all(|(a, b)| a.agrees_with(b, 96)),
                    };
                    accepted += usize::from(valid);
                }
                println!(
                    "{}",
                    serde_json::json!({
                        "regime":text,"trial":trial,"mode":mode,"repeats":repeats,"accepted":accepted,"elapsed_ns":elapsed_ns,
                        "timing":"construction/evaluation and row storage; validation excluded; scalar globals and inputs may be warm",
                        "contract":"two finite principal atanh enclosures with width below 2^-96 and independent overlap; no comparative speedup claim",
                    })
                );
                assert_eq!(accepted, repeats);
            }
        }
    }
}
