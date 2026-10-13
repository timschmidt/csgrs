use super::*;
use std::sync::{Arc, atomic::AtomicBool};

fn root(family: u8) -> Real {
    match family {
        0 => Real::from(2).sqrt().unwrap(),
        1 => (Real::from(3) + Real::from(5).sqrt().unwrap())
            .sqrt()
            .unwrap(),
        2 => Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap(),
        _ => unreachable!(),
    }
}

fn retained_signs(family: u8, square: bool) {
    let root = root(family);
    let lower = root.certified_rational_interval(-1024).unwrap()[0].clone();
    for reverse in [false, true] {
        // Spell both subtraction orders so the negative source need not be
        // represented only by negating its outer rational scale.
        let delta = if reverse {
            Real::from(lower.clone()) - &root
        } else {
            &root - Real::from(lower.clone())
        };
        let delta_sign = if reverse {
            RealSign::Negative
        } else {
            RealSign::Positive
        };
        assert_eq!(delta.certified_sign_until(-1100).sign(), Some(delta_sign));
        let value = if square { &delta * &delta } else { delta };
        for (numerator, denominator) in [(1, 1), (-7, 1), (7, 13), (-7, 13)] {
            let expected = if (reverse && !square) ^ (numerator < 0) {
                RealSign::Negative
            } else {
                RealSign::Positive
            };
            let original = &value * Real::from(Rational::fraction(numerator, denominator).unwrap());
            assert_eq!(original.certified_sign_until(0).sign(), Some(expected));
            assert_eq!(original.immediate_sign(), Some(expected));
            let original_clone = original.clone();
            let compact = original.compact_quadratic_tower().unwrap();
            // These must precede equality reduction, which could prove and
            // cache an otherwise unavailable sign in the reconstructed graph.
            assert_eq!(
                compact.immediate_sign(),
                Some(expected),
                "reverse={reverse}, scale={numerator}/{denominator}"
            );
            for precision in [0, -512, 0] {
                assert_eq!(
                    compact.certified_sign_until(precision).sign(),
                    Some(expected)
                );
            }
            let clone = compact.clone();
            assert_eq!(clone.certified_sign_until(0).sign(), Some(expected));
            let repeated = compact.compact_quadratic_tower().unwrap();
            assert_eq!(repeated.immediate_sign(), Some(expected));
            assert_eq!(repeated.certified_sign_until(0).sign(), Some(expected));
            assert_eq!(
                original_clone.certified_sign_until(0).sign(),
                Some(expected)
            );
            assert_eq!(original.certified_sign_until(0).sign(), Some(expected));
            assert_eq!(
                (&compact - &original).quadratic_tower_sign(),
                Some(RealSign::Zero)
            );
            assert_eq!(
                (&repeated - &original).quadratic_tower_sign(),
                Some(RealSign::Zero)
            );
        }
    }
}

#[test]
fn quadratic_linear_retains_sign() {
    retained_signs(0, false);
}

#[test]
fn quadratic_square_retains_sign() {
    retained_signs(0, true);
}

#[test]
fn nested_linear_retains_sign() {
    retained_signs(1, false);
}

#[test]
fn nested_square_retains_sign() {
    retained_signs(1, true);
}

#[test]
fn biquadratic_linear_retains_sign() {
    retained_signs(2, false);
}

#[test]
fn biquadratic_square_retains_sign() {
    retained_signs(2, true);
}

#[test]
fn rational_zero_and_symbolic_compactions_preserve_signs() {
    let sum = root(2);
    let rational = &sum * &sum - Real::from(2) * Real::from(6).sqrt().unwrap();
    let zero = &rational - Real::from(5);
    assert_eq!(zero.quadratic_tower_sign(), Some(RealSign::Zero));
    for (numerator, denominator) in [(7, 13), (-7, 13)] {
        let scale = Rational::fraction(numerator, denominator).unwrap();
        let compact = (&rational * Real::from(scale.clone()))
            .compact_quadratic_tower()
            .unwrap();
        assert_eq!(
            compact.exact_rational_ref(),
            Some(&(Rational::from(5) * scale.clone()))
        );
        let compact_zero = (&zero * Real::from(scale.clone()))
            .compact_quadratic_tower()
            .unwrap();
        assert_eq!(compact_zero.exact_rational_ref(), Some(&Rational::zero()));
        assert_eq!(compact_zero.immediate_sign(), Some(RealSign::Zero));
        let symbolic = Real::from(2).sqrt().unwrap() * Real::from(scale);
        let compact_symbolic = symbolic.compact_quadratic_tower().unwrap();
        assert_eq!(compact_symbolic.immediate_sign(), symbolic.immediate_sign());
    }
    // A zero result must not corrupt a shared nonzero constant's sign.
    assert_eq!(Real::one().immediate_sign(), Some(RealSign::Positive));
    assert_eq!(
        Real::from(2).sqrt().unwrap().immediate_sign(),
        Some(RealSign::Positive)
    );
}

#[test]
fn compaction_preserves_final_abort_signal() {
    let mut value = root(1) - Real::from(2);
    assert_eq!(value.quadratic_tower_sign(), Some(RealSign::Positive));
    let signal = Arc::new(AtomicBool::new(false));
    value.abort(signal.clone());
    let compact = value.compact_quadratic_tower().unwrap();
    assert!(Arc::ptr_eq(compact.abort_signal().unwrap(), &signal));
    signal.store(true, std::sync::atomic::Ordering::Relaxed);
    assert!(compact.is_aborted());
    let repeated = compact.compact_quadratic_tower().unwrap();
    assert!(Arc::ptr_eq(repeated.abort_signal().unwrap(), &signal));
    assert!(repeated.is_aborted());
    assert!(value.compact_quadratic_tower().unwrap().is_aborted());
}

#[test]
fn compaction_keeps_unknown_and_unsupported_sources_honest() {
    let root = root(1);
    let lower = root.certified_rational_interval(-1024).unwrap()[0].clone();
    let unknown = &root - Real::from(lower);
    assert_eq!(unknown.immediate_sign(), None);
    let opaque = Real::pi() + &root;
    for (value, covered) in [(unknown, true), (opaque, false)] {
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::reset();
        #[cfg(feature = "dispatch-trace")]
        let _recording = crate::dispatch_trace::recording_scope();
        let compact = value.compact_quadratic_tower();
        #[cfg(feature = "dispatch-trace")]
        assert!(
            crate::dispatch_trace::snapshot()
                .iter()
                .all(|row| !(row.layer == "computable" && row.operation == "approx"))
        );
        assert_eq!(compact.is_some(), covered);
        if let Some(compact) = compact {
            // Strengthening Unknown is allowed; an invented zero/negative is not.
            assert!(matches!(
                compact.immediate_sign(),
                None | Some(RealSign::Positive)
            ));
            assert_eq!(
                (&compact - &value).quadratic_tower_sign(),
                Some(RealSign::Zero)
            );
        }
    }
}

#[test]
fn denested_negative_tower_preserves_payload_sign() {
    let outer = root(1);
    // The existing sqrt fold denests sqrt(3 + sqrt(5)) into a sum.
    // Rebuilding its negative keeps the outer scale and a negative payload.
    let negative = Real::one() - (&outer + Real::one());
    assert_eq!(negative.rational.sign(), Sign::Plus);
    assert_eq!(
        negative.certified_sign_until(0).sign(),
        Some(RealSign::Negative)
    );
    for numerator in [-7, 7] {
        let original = &negative * Real::from(Rational::fraction(numerator, 13).unwrap());
        let expected = if numerator < 0 {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        assert_eq!(original.immediate_sign(), Some(expected));
        let compact = original.compact_quadratic_tower().unwrap();
        assert_eq!(compact.rational.sign(), original.rational.sign());
        assert_eq!(compact.immediate_sign(), Some(expected));
        assert_eq!(compact.certified_sign_until(0).sign(), Some(expected));
        assert_eq!(
            compact.computable_ref().immediate_sign(),
            Some(RealSign::Negative)
        );
        assert_eq!(
            (&compact - &original).quadratic_tower_sign(),
            Some(RealSign::Zero)
        );
    }
    assert_eq!(
        outer.certified_sign_until(0).sign(),
        Some(RealSign::Positive)
    );
}

#[test]
fn normalized_negative_tower_scale_preserves_payload_sign() {
    let outer = (Real::from(3) + Real::from(2).sqrt().unwrap())
        .sqrt()
        .unwrap();
    // Cancellation reconstructs -outer with a negative rational scale,
    // even though the original graph's outer rational scale is positive.
    let negative = Real::one() - (&outer + Real::one());
    assert_eq!(negative.rational.sign(), Sign::Plus);
    assert_eq!(
        negative.certified_sign_until(0).sign(),
        Some(RealSign::Negative)
    );
    for numerator in [-7, 7] {
        let original = &negative * Real::from(Rational::fraction(numerator, 13).unwrap());
        let expected = if numerator < 0 {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        assert_eq!(original.immediate_sign(), Some(expected));
        let compact = original.compact_quadratic_tower().unwrap();
        assert_ne!(compact.rational.sign(), original.rational.sign());
        assert_eq!(compact.immediate_sign(), Some(expected));
        assert_eq!(compact.certified_sign_until(0).sign(), Some(expected));
        assert_eq!(
            compact.computable_ref().immediate_sign(),
            Some(RealSign::Positive)
        );
        assert_eq!(
            (&compact - &original).quadratic_tower_sign(),
            Some(RealSign::Zero)
        );
    }
    assert_eq!(
        outer.certified_sign_until(0).sign(),
        Some(RealSign::Positive)
    );
}
