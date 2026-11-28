//! Exact rational-relation projection after JSON replay and with retained zero proofs.
//!
//! Twenty-two positive scalar fixtures produce bounded parameters and selected root/
//! inverse-trigonometric encodings. Each measured result is a triple of exact zero
//! projections, not a sampled numerical residual. Same-process JSON replay reconstructs
//! objects but does not reset process-global constant caches. Decoding can perform
//! admission proofs and establish new evidence; it does not load saved live proof
//! caches. The retained mode clones
//! already-proved residual expressions; it does not repeat the original construction.
//!
//! This custom serde-enabled executable ignores arguments and emits JSON to stdout.
//! Rows are printed before their accepted-count assertion, so process success and
//! accepted counts both matter. Fixed repetitions and bounded proof-search stages
//! are not a complete wall-time/memory/cancellation contract; failures panic here.

use hyperreal::{Rational, Real};
use std::{hint::black_box, time::Instant};
/// Shared 22-case positive scalar catalogue and expected public structural kinds.
///
/// The benchmark validates those public categories before encoding; labels do not
/// prove a private enum variant or complete coverage of all values in a family.
#[path = "../tests/support/representations.rs"]
mod representations;

/// Reconstruct three defining-relation residuals from `[v, m, r, a, tau]`.
///
/// Return 1/(1+r^2)-(1-m), a/sqrt(1+a^2)-v, and sin(tau)-v, in that order.
/// The fixture sets r=sqrt(m/(1-m)), a=v/sqrt(1-v^2), and tau=asin(v), using
/// 0<=m<1 and abs(v)<1 with positive square roots. Those relations imply all three
/// zeros. This helper constructs residuals without checking those relations or
/// proving zero; callers perform the exact rational projections. Construction
/// failures are unwrapped and panic rather than becoming zero residuals.
fn residuals([v, m, r, a, tau]: [Real; 5]) -> [Real; 3] {
    let h = (Real::one() / (Real::one() + &r * &r)).unwrap();
    let decoded_v = (&a / (Real::one() + &a * &a).sqrt().unwrap()).unwrap();
    [h - (Real::one() - m), decoded_v - &v, tau.sin() - v]
}

/// Emit 220 aggregate rows: 22 fixtures, five trials and two modes, ten repeats each.
///
/// For positive fixture x, set v=x/(x+1) in (0,1), m=v/2 in (0,1/2), then form
/// r, a and tau as described by residuals. Check the fixture's public structural
/// kind and serialize `[v,m,r,a,tau]` once before all timing. Before every mode,
/// including cold replay, deserialize, reconstruct residuals and require three
/// Some(0) exact rational projections to prepare warm evidence.
///
/// Cold timing includes fresh JSON deserialization, residual construction, exact
/// projection and vector storage. Retained timing includes cloning those proved
/// residuals, querying their projections and storing results. Temporary Real arrays
/// are consumed/dropped inside the loop; stored `Option<Rational>` triples are checked
/// and dropped afterward. Setup, zero validation, serialization comparison and
/// printing are excluded from elapsed_ns.
///
/// Every repeat must yield three Some(0) projections; None means no proof and fails
/// acceptance. Re-serializing the original payload must equal its original JSON
/// bytes after each timing row. This checks source-byte stability, not fresh-process
/// replay, hostile imports, complete value equality or a universal proof-search
/// resource bound. Each accepted count represents a whole triple.
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
