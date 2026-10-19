//! Completed native six-component enclosures and an independent MPFR ODE oracle.
use hyperlattice::{Complex, Real};
use std::{hint::black_box, time::Instant};
#[path = "../tests/support/complex_jacobi_oracle.rs"]
mod oracle;

fn components(argument: &Complex, m: &Real) -> [Real; 6] {
    let (s, c, d) = argument.clone().jacobi_sncndn(m).unwrap();
    [s.re, s.im, c.re, c.im, d.re, d.im]
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--test");
    for text in ["0.5", "-3", "0"] {
        let argument = Complex::new("0.125".parse().unwrap(), "0.0625".parse().unwrap());
        let m: Real = text.parse().unwrap();
        oracle::compare(&argument, &m);
        let reference = oracle::reference(&argument, &m);
        let warm = components(&argument, &m);
        let expected = warm
            .clone()
            .map(|v| v.certified_rational_interval(-112).unwrap());
        for trial in 0..if smoke { 1 } else { 5 } {
            for mode in ["native_cold", "native_warm", "mpfr_ode"] {
                let repeats = if smoke || mode == "mpfr_ode" { 1 } else { 5 };
                let start = Instant::now();
                let mut rows = Vec::new();
                for _ in 0..repeats {
                    rows.push(if mode == "mpfr_ode" {
                        Err(black_box(oracle::reference(&argument, &m)))
                    } else {
                        let values = if mode == "native_cold" {
                            components(&argument, &m)
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
                        "parameter":text,"trial":trial,"mode":mode,"repeats":repeats,
                        "accepted":accepted,"elapsed_ns":elapsed_ns,
                        "timing":"construction/evaluation and row storage; validation excluded; scalar globals and inputs may be warm",
                        "contract":"six finite enclosures with width below 2^-96 and independent overlap; no comparative speedup claim",
                    })
                );
                assert_eq!(accepted, repeats);
            }
        }
    }
}
