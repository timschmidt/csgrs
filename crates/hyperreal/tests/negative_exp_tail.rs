//! Regression for the default-stack abort exposed by inverse transverse Mercator.
use hyperreal::{CertifiedRealSign, Rational, Real, RealSign};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[path = "support/representations.rs"]
mod representations;

fn small(value: &Real, bits: i32) {
    let [lower, upper] = value.certified_rational_interval(-bits - 8).unwrap();
    let tolerance = Rational::fraction(1, 2).unwrap().powi(bits.into()).unwrap();
    assert!(lower >= -tolerance.clone() && upper <= tolerance);
}

fn positive(value: &Real) {
    assert!(matches!(
        value.certified_sign_until(-128),
        CertifiedRealSign::Known {
            sign: RealSign::Positive,
            ..
        }
    ));
}

#[test]
fn huge_negative_exp_composes_on_the_default_thread_stack() {
    // The test harness uses the ordinary Rust thread stack. Do not increase it:
    // the baseline failed while building/evaluating 1065 nested squares.
    let huge: Real = "1e320".parse().unwrap();
    for input in [-huge.clone(), -&huge + Real::pi(), -&huge * Real::e()] {
        let exponential = input.clone().exp().unwrap();
        positive(&exponential);
        let composed = exponential.clone().atan().unwrap();
        for bits in [128, 512, 32, 256] {
            small(&composed, bits);
            small(&composed.clone().sin(), bits);
            small(&exponential, bits);
            positive(&exponential);
            small(&(input.clone().expm1() + Real::one()), bits);
        }
    }
}

#[test]
fn all_certificate_families_keep_negative_tail_composition() {
    let huge: Real = "1e320".parse().unwrap();
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(
            case.value.detailed_facts().symbolic.kind,
            case.public_kind,
            "{}",
            case.certificate
        );
        let exponential = (-&huge * case.value)
            .exp()
            .unwrap_or_else(|e| panic!("{}: {e:?}", case.certificate));
        let angle = exponential.clone().atan().unwrap();
        small(&angle, 192);
        let _ = angle.to_f64_lossy();
        small(&angle, 256);
        positive(&exponential);
    }
}

#[test]
fn zero_approximation_does_not_replace_value_or_prevent_finer_refinement() {
    for precisions in [[0, -128, -4, -256], [-256, 0, -128, -4]] {
        let value = Real::from(-8).exp().unwrap().atan().unwrap();
        for precision in precisions {
            let [lower, upper] = value.certified_rational_interval(precision).unwrap();
            if precision <= -128 {
                assert!(lower > Rational::zero());
                assert!(upper < Rational::fraction(1, 1000).unwrap());
            }
        }
    }
}

#[test]
fn ordinary_integer_log_collapse_still_produces_exact_rationals() {
    for base in [2, 3, 5, 17] {
        for exponent in -20..=20 {
            let value = (Real::from(exponent) * Real::from(base).ln().unwrap())
                .exp()
                .unwrap();
            assert_eq!(
                value.exact_rational().unwrap(),
                Rational::new(base).powi(exponent.into()).unwrap()
            );
        }
    }
}

#[test]
fn cancelled_clone_and_concurrent_readers_preserve_the_original() {
    let value = "-1e320"
        .parse::<Real>()
        .unwrap()
        .exp()
        .unwrap()
        .atan()
        .unwrap();
    let signal = Arc::new(AtomicBool::new(true));
    let mut cancelled = value.clone();
    cancelled.abort(signal.clone());
    assert!(cancelled.certified_rational_interval(-128).is_none());
    signal.store(false, Ordering::Relaxed);
    small(&cancelled, 192);
    std::thread::scope(|scope| {
        for bits in [32, 128, 256, 512] {
            let value = value.clone();
            scope.spawn(move || small(&value, bits));
        }
    });
    small(&value, 512);
}

#[cfg(feature = "serde")]
#[test]
fn cold_and_warm_serialization_keep_the_nonzero_expression() {
    for case in representations::optimized_certificate_representatives() {
        let value = (-"1e320".parse::<Real>().unwrap() * case.value)
            .exp()
            .unwrap_or_else(|e| panic!("{}: {e:?}", case.certificate));
        let cold = value.to_json();
        let decoded = Real::from_json(&cold).unwrap();
        small(&decoded.clone().atan().unwrap(), 192);
        positive(&decoded);
        small(&value.clone().atan().unwrap(), 256);
        assert_eq!(cold, value.to_json());
        let mut bytes = Vec::new();
        ciborium::ser::into_writer(&value, &mut bytes).unwrap();
        let decoded: Real = ciborium::de::from_reader(bytes.as_slice()).unwrap();
        small(&decoded.clone().atan().unwrap(), 128);
        positive(&decoded);
    }
}
