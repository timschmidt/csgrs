//! Timing harness for completed native triplet enclosures and a local MPFR ODE reference.
//!
//! Each row includes construction/evaluation and retained row storage; interval
//! validation follows the timer. Native construction and clone reuse can share
//! warmed inputs/global scalar state. These labels do not prove independent cold
//! caches, and different requested precisions do not support a comparative speedup
//! claim. Failures panic rather than count incomplete work as accepted.

use hyperlattice::{Complex, Real};
use std::{hint::black_box, time::Instant};
/// Shared local ODE reference and ordered-width/overlap checks, independent of production rectangular continuation.
#[path = "../tests/support/complex_jacobi_oracle.rs"]
mod oracle;

/// Construct the native triplet and return its 6 scalar components in reference order; native domain errors unwrap-panic.
fn components(argument: &Complex, m: &Real) -> [Real; 6] {
    let (s, c, d) = argument.clone().jacobi_sncndn(m).unwrap();
    [s.re, s.im, c.re, c.im, d.re, d.im]
}

/// Emit JSON timing/acceptance rows for m=1/2,-3,0 at z=1/8+i/16.
///
/// An exact --test argument selects one trial and one repeat; ordinary runs use
/// five trials, five native repeats and one MPFR repeat. Before timing, compare
/// production with the oracle and prepare a reference, native outputs and -112
/// bounds. Each timed row retains all results until validation, using width
/// threshold 2^-96 and independent overlap for 6 components. Print elapsed_ns
/// and accepted counts, then assert every repeat passed. No warmup isolation,
/// statistical aggregation, total-work cap or process timeout is implemented.
fn main() {
    let smoke = std::env::args().any(|arg| arg == "--test");
    for text in ["0.5", "-3", "0"] {
        let argument = Complex::new("0.125".parse().unwrap(), "0.0625".parse().unwrap());
        let m: Real = text.parse().unwrap();
        oracle::compare(&argument, &m, false);
        let reference = oracle::reference(&argument, &m, false);
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
                        Err(black_box(oracle::reference(&argument, &m, false)))
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
