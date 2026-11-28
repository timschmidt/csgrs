//! Exact parameter-encoding identities, replay and shared-proof regression fixtures.
//!
//! Positive scalar representatives are mapped into bounded domains; negative values
//! and selected endpoints extend the corpus. Exact rational zero projections test
//! defining relations and subsequent sine consumption. Feature-gated replay tests
//! separately check JSON/CBOR stability and ordered enclosures; they do not equate
//! those byte/order checks with complete mathematical equivalence or containment.

use hyperreal::{Rational, Real};
/// Shared finite scalar constructions; expected public categories are fixture assertions.
#[path = "support/representations.rs"]
mod representations;

/// Verify parameter, signed meridian and inverse-sine recovery for 22 scalar fixtures.
///
/// Check each public structural category, map positive x to u=x/(x+1), and use
/// m=u/2. Require exact zero projections for parameter reconstruction and for both
/// signs of meridian and sine recovery: five residuals per fixture. Separately test
/// sin(asin(v))=v at v=0,1,-1; meridian encoding is not applied at the poles.
/// Some(0), rather than approximate closeness or an undecided result, is required.
/// This is finite identity coverage, not every value of every certificate family.
#[test]
fn bounded_parameter_encodings_retain_all_scalar_family_values() {
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        let value = &case.value;
        let unit = (value / (value + Real::one())).unwrap();
        let m = (&unit / Real::from(2)).unwrap();
        let h = Real::one() - &m;
        let r = (m.clone() / &h).unwrap().sqrt().unwrap();
        let decoded_h = (Real::one() / (Real::one() + &r * &r)).unwrap();
        assert_eq!(
            (decoded_h - h).exact_rational_normal_form(),
            Some(Rational::zero()),
            "parameter {}",
            case.certificate
        );
        for sign in [-1, 1] {
            let v = Real::from(sign) * &unit;
            let a = (v.clone() / (Real::one() - &v * &v).sqrt().unwrap()).unwrap();
            let decoded_v = (a.clone() / (Real::one() + &a * &a).sqrt().unwrap()).unwrap();
            assert_eq!(
                (decoded_v - &v).exact_rational_normal_form(),
                Some(Rational::zero()),
                "meridian {} / {sign}",
                case.certificate
            );
            let decoded_time = v.clone().asin().unwrap().sin();
            assert_eq!(
                (decoded_time - v).exact_rational_normal_form(),
                Some(Rational::zero()),
                "time {} / {sign}",
                case.certificate
            );
        }
    }
    for v in [Real::zero(), Real::one(), -Real::one()] {
        assert_eq!(
            (v.clone().asin().unwrap().sin() - v).exact_rational_normal_form(),
            Some(Rational::zero())
        );
    }
}

/// Serde-only replay, endpoint and concurrent-proof fixtures.
///
/// All replay occurs in the same process and may share global constants. No total
/// work/memory or cancellation limit, malformed-wire corpus or fresh-process claim
/// is supplied by these tests.
#[cfg(feature = "serde")]
mod serialization {
    use super::*;
    /// JSON round-trip the five authoritative encoding values into fresh objects.
    ///
    /// Serialization and deserialization are unwrapped. This does not reset global
    /// caches or compare the serialized bytes before/after the round trip.
    fn replay(value: &[Real; 5]) -> [Real; 5] {
        serde_json::from_str(&serde_json::to_string(value).unwrap()).unwrap()
    }

    /// Build `[v,m,sqrt(m/(1-m)),v/sqrt(1-v^2),asin(v)]` for valid fixture inputs.
    ///
    /// Callers supply 0<=m<1 and abs(v)<1, including exact near-endpoint offsets.
    /// Positive-root and inverse-trig relations remain in the exact expressions;
    /// invalid/unresolved construction domains panic through unwrapped results.
    fn encoded(v: Real, m: Real) -> [Real; 5] {
        let r = (&m / (Real::one() - &m)).unwrap().sqrt().unwrap();
        let a = (&v / (Real::one() - &v * &v).sqrt().unwrap()).unwrap();
        let tau = v.clone().asin().unwrap();
        [v, m, r, a, tau]
    }

    /// Require all three reconstructed residuals, and then their sines, to prove zero.
    ///
    /// Decode the parameter complement, signed meridian and inverse-sine encodings;
    /// compare by exact rational normal form Some(0). Applying sine to each residual
    /// exercises subsequent use of its exact relation. The context labels failures;
    /// None or a nonzero rational fails. No floating tolerance or sampling is used.
    fn accept([v, m, r, a, tau]: [Real; 5], context: &str) {
        let decoded_h = (Real::one() / (Real::one() + &r * &r)).unwrap();
        let decoded_v = (&a / (Real::one() + &a * &a).sqrt().unwrap()).unwrap();
        for (label, residual) in [
            ("parameter", decoded_h - (Real::one() - m)),
            ("meridian", decoded_v - &v),
            ("time", tau.sin() - v),
        ] {
            assert_eq!(
                residual.exact_rational_normal_form(),
                Some(Rational::zero()),
                "{context}: {label}"
            );
            assert_eq!(
                residual.sin().exact_rational_normal_form(),
                Some(Rational::zero())
            );
        }
    }

    /// Exercise 88 JSON replays: 22 fixtures, both signs and cold/display-warmed modes.
    ///
    /// Assert each source public category, encode v with each sign and m from the
    /// positive bounded magnitude, then replay. Warm mode requests lossy float views
    /// of all five decoded values but ignores their return values. Both modes require
    /// all exact residual and subsequent-sine proofs through accept; neither mode
    /// clears process-global caches or asserts JSON byte equality.
    #[test]
    fn serialized_parameter_encodings_replay_cold_and_after_display_warming() {
        for case in representations::optimized_certificate_representatives() {
            assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
            let unit = (&case.value / (&case.value + Real::one())).unwrap();
            let m = (&unit / Real::from(2)).unwrap();
            for sign in [-1, 1] {
                let source = encoded(Real::from(sign) * &unit, m.clone());
                for warm in [false, true] {
                    let decoded = replay(&source);
                    if warm {
                        for value in &decoded {
                            let _ = value.to_f64_lossy();
                        }
                    }
                    accept(
                        decoded,
                        &format!("{} sign={sign} warm={warm}", case.certificate),
                    );
                }
            }
        }
    }

    /// Check nine exact JSON-replayed boundary combinations without endpoint snapping.
    ///
    /// Use delta=10^-100, m in {0,delta,1-delta} and v in {0,-1+delta,1-delta}.
    /// Every combination remains inside the meridian/root domains and must pass all
    /// six exact zero projections in accept. These are finite near-endpoint witnesses,
    /// not a proof for all small offsets or the excluded singular parameter m=1.
    #[test]
    fn parameter_zero_and_exact_near_endpoint_offsets_survive_replay() {
        let delta: Real = "1e-100".parse().unwrap();
        for m in [Real::zero(), delta.clone(), Real::one() - &delta] {
            for v in [Real::zero(), -Real::one() + &delta, Real::one() - &delta] {
                accept(replay(&encoded(v, m.clone())), "boundary");
            }
        }
    }

    /// Check CBOR replay, concurrent exact proofs and serialization stability for 22 cases.
    ///
    /// Encode negative bounded v with m from its positive magnitude, decode CBOR and
    /// record the decoded JSON. Four scoped workers each call accept three times on
    /// clones sharing the decoded values; joining propagates worker panics. Require
    /// unchanged JSON afterward. Then request all five enclosures at precisions
    /// -32,-192,-32, checking only endpoint order, and repeat exact proofs and JSON
    /// byte equality after each precision stage. Finally require the decoded CBOR
    /// bytes to equal the original source encoding.
    ///
    /// This gives 15 accept calls per fixture (12 concurrent plus three sequential).
    /// Ordered endpoints alone do not prove containment, overlap or nested refinement;
    /// those properties are not asserted here. Byte stability and exact relation
    /// proofs remain separate checks, with no fresh-process or latency guarantee.
    #[test]
    fn binary_replay_and_concurrent_proofs_preserve_authoritative_values() {
        for case in representations::optimized_certificate_representatives() {
            let unit = (&case.value / (&case.value + Real::one())).unwrap();
            let m = (&unit / Real::from(2)).unwrap();
            let source = encoded(-unit, m);
            let mut cbor = Vec::new();
            ciborium::ser::into_writer(&source, &mut cbor).unwrap();
            let decoded: [Real; 5] = ciborium::de::from_reader(cbor.as_slice()).unwrap();
            let wire = serde_json::to_string(&decoded).unwrap();
            std::thread::scope(|scope| {
                let jobs: Vec<_> = (0..4)
                    .map(|_| {
                        let decoded = &decoded;
                        let label = case.certificate;
                        scope.spawn(move || {
                            for _ in 0..3 {
                                accept(decoded.clone(), label);
                            }
                        })
                    })
                    .collect();
                for job in jobs {
                    job.join().unwrap();
                }
            });
            assert_eq!(serde_json::to_string(&decoded).unwrap(), wire);
            for precision in [-32, -192, -32] {
                for value in &decoded {
                    let [lo, hi] = value.certified_rational_interval(precision).unwrap();
                    assert!(lo <= hi);
                }
                accept(decoded.clone(), case.certificate);
                assert_eq!(serde_json::to_string(&decoded).unwrap(), wire);
            }
            let mut after = Vec::new();
            ciborium::ser::into_writer(&decoded, &mut after).unwrap();
            assert_eq!(after, cbor);
        }
    }
}
