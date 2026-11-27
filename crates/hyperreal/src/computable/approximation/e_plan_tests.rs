//! Qualification of the e term planner, direct enclosures, shared refinement and replay.
//! These finite requests do not establish a global resource or cancellation-latency bound.

use super::{BigInt, BigUint, Computable, One, Rational, e, e_terms_for_precision};
use rug::{Float, Integer, float::Round};
use std::collections::BTreeSet;
use std::sync::{Arc, atomic::AtomicBool, atomic::Ordering};

/// Require the one-unit interval around actual to strictly contain directed MPFR bounds for scaled e.
///
/// The selected fixtures fit their integer endpoints exactly in the chosen
/// precision (at least 256 bits, otherwise abs(p)+192). Compute exp(1) with
/// both directed roundings, apply the binary scale, and register MPFR thread
/// cleanup. This fixture helper has no allocation limit for arbitrary p.
fn assert_e_enclosure(actual: &BigInt, p: i32) {
    crate::mpfr_test::cleanup_on_thread_exit();
    let precision = p.unsigned_abs().max(64) + 192;
    let mut lower = Float::with_val(precision, 1);
    let mut upper = lower.clone();
    lower.exp_round(Round::Down);
    upper.exp_round(Round::Up);
    if p < 0 {
        lower <<= p.unsigned_abs();
        upper <<= p.unsigned_abs();
    } else {
        lower >>= p as u32;
        upper >>= p as u32;
    }
    let integer = Integer::from_str_radix(&actual.to_str_radix(16), 16).unwrap();
    let bottom = Float::with_val(precision, Integer::from(&integer - 1));
    let top = Float::with_val(precision, Integer::from(&integer + 1));
    assert!(bottom < lower && top > upper, "e at precision {p}");
}

#[test]
/// Compare planner outputs against exact factorial bit thresholds over 4,151 distinct requests.
///
/// Combine the dense -4096..=16 range, selected factorial-boundary neighbors,
/// and larger requests through -262144. Require no early truncation; allowing
/// at most one extra term is a finite-corpus cost sentinel, not a universal
/// minimality theorem or a measured runtime bound.
fn term_planner_preserves_exact_factorial_thresholds() {
    let mut requests: BTreeSet<i32> = (-4_096..=16).collect();
    let selected: BTreeSet<u32> = [
        20, 21, 22, 31, 32, 33, 63, 64, 65, 127, 128, 129, 255, 256, 257, 511, 512, 513, 1_023,
        1_024, 1_025, 2_047, 2_048, 2_049, 4_095, 4_096, 4_097, 8_191, 8_192,
    ]
    .into_iter()
    .collect();
    let mut factorial = BigUint::one();
    for n in 1..=8_192_u32 {
        factorial *= n;
        if selected.contains(&n) {
            for delta in [-1, 0, 1] {
                let needed = factorial.bits() as i32 + delta;
                if needed > 4 {
                    requests.insert(4 - needed);
                }
            }
        }
    }
    for bits in [16_384, 32_768, 65_536, 120_700, 262_144] {
        requests.insert(-bits);
    }
    let mut factorial = BigUint::one();
    let mut k = 1_u32;
    for p in requests.into_iter().rev() {
        let needed = if p < 0 {
            u64::from(p.unsigned_abs()) + 4
        } else {
            4
        };
        while factorial.bits() <= needed {
            k += 1;
            factorial *= k;
        }
        let exact = k - 1;
        let planned = e_terms_for_precision(p);
        assert!(planned >= exact, "planner stops early at {p}");
        // A finite-corpus cost sentinel, not a general minimality guarantee.
        assert!(planned <= exact + 1, "excess sampled work at {p}");
    }
}

#[test]
/// Check 93 direct e evaluations against strict directed MPFR enclosures.
/// The requests span -262144 through 8 with dense coarse and selected fine
/// boundaries. Direct calls bypass the shared constant cache, so a warm cache
/// cannot substitute for exercising this kernel.
fn direct_e_kernel_has_strict_directed_enclosures() {
    let mut requests: BTreeSet<i32> = (-64..=8).collect();
    for p in [
        -127, -128, -129, -255, -256, -257, -511, -512, -513, -1_023, -1_024, -1_025, -4_095,
        -4_096, -4_097, -16_384, -32_768, -65_536, -120_700, -262_144,
    ] {
        requests.insert(p);
    }
    for p in requests {
        assert_e_enclosure(&e(p), p);
    }
}

#[test]
/// Check seven nonmonotone requests through one shared e handle and freshly constructed handles.
/// Also check exp(1) at -16384. Require directed enclosures without assuming
/// that other tests left the process-wide cache cold or isolated.
fn shared_e_refines_and_coarsens_within_its_enclosure() {
    // Other tests may already have warmed the process-wide constant cache.
    // This checks the public contract without assuming exclusive cold state.
    let shared = Computable::e();
    for p in [-64, -4_096, -32_768, -128, 0, -65_536, -256] {
        assert_e_enclosure(&shared.approx(p), p);
        assert_e_enclosure(&Computable::e().approx(p), p);
    }
    let exp_one = Computable::rational(Rational::one()).exp();
    assert_e_enclosure(&exp_one.approx(-16_384), -16_384);
}

#[test]
/// Verify a stopped shared-e handle can later produce an enclosure after clearing the signal.
/// Discard the stopped result; this test does not assert that inner work ceased
/// or inspect the cache immediately after abort. With serde, replay JSON and
/// check four further precisions in this process. This is recovery/enclosure
/// evidence, not an interruption-latency or fresh-process replay test.
fn shared_e_recovers_from_cancellation_and_serialization() {
    let stop = Arc::new(AtomicBool::new(true));
    let mut shared = Computable::e();
    shared.abort(stop.clone());
    drop(shared.approx(-4_096));
    stop.store(false, Ordering::Relaxed);
    assert_e_enclosure(&shared.approx(-4_096), -4_096);
    #[cfg(feature = "serde")]
    {
        let encoded = serde_json::to_string(&shared).unwrap();
        let restored: Computable = serde_json::from_str(&encoded).unwrap();
        for p in [-8_192, -128, -32_768, 0] {
            assert_e_enclosure(&restored.approx(p), p);
        }
    }
}
