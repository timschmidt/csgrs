//! Completed exact certificate projections, with cold replay and retained proof reuse.
use hyperreal::{Rational, Real};
use std::{hint::black_box, time::Instant};
#[path = "../tests/support/representations.rs"]
mod representations;

fn residuals([v, m, r, a, tau]: [Real; 5]) -> [Real; 3] {
    let h = (Real::one() / (Real::one() + &r * &r)).unwrap();
    let decoded_v = (&a / (Real::one() + &a * &a).sqrt().unwrap()).unwrap();
    [h - (Real::one() - m), decoded_v - &v, tau.sin() - v]
}

fn main() {
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        let v = (&case.value / (&case.value + Real::one())).unwrap();
        let m = (&v / Real::from(2)).unwrap();
        let r = (&m / (Real::one() - &m)).unwrap().sqrt().unwrap();
        let a = (&v / (Real::one() - &v * &v).sqrt().unwrap()).unwrap();
        let tau = v.clone().asin().unwrap();
        let payload = [v, m, r, a, tau];
        let wire = serde_json::to_string(&payload).unwrap();
        for trial in 0..5 {
            for mode in ["cold_replay", "retained_certificate"] {
                let warm = residuals(serde_json::from_str(&wire).unwrap());
                for value in &warm {
                    assert_eq!(value.exact_rational_normal_form(), Some(Rational::zero()));
                }
                let repeats = 10;
                let start = Instant::now();
                let mut rows = Vec::new();
                for _ in 0..repeats {
                    let values = if mode == "cold_replay" {
                        residuals(serde_json::from_str(&wire).unwrap())
                    } else {
                        warm.clone()
                    };
                    rows.push(black_box(values.map(|v| v.exact_rational_normal_form())));
                }
                let elapsed_ns = start.elapsed().as_nanos();
                let accepted = rows
                    .iter()
                    .filter(|row| row.iter().all(|r| *r == Some(Rational::zero())))
                    .count();
                assert_eq!(serde_json::to_string(&payload).unwrap(), wire);
                println!(
                    "{}",
                    serde_json::json!({
                        "family":case.certificate,"trial":trial,"mode":mode,
                        "repeats":repeats,"accepted":accepted,"elapsed_ns":elapsed_ns,
                        "serialized_input_bytes":wire.len(),
                        "contract":"three exact zero certificates; parameter, meridian and time reconstruction",
                        "timing":"cold: deserialize, reconstruct residuals, prove, store rows; retained: clone residuals, reuse certificates, store rows; validation excluded",
                        "cache":"cold payloads have no retained input facts; process-global shared constant caches may be warm",
                    })
                );
                assert_eq!(accepted, repeats);
            }
        }
    }
}
