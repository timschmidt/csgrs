//! Huge negative exponential tails: retained positivity, composition and replay.
//!
//! These regressions exercise the ordinary test-harness stack configuration
//! without setting a larger stack. Interval smallness, nonzero sign, exact
//! identities and serialization stability remain separate assertions.
use hyperreal::{CertifiedRealSign, Rational, Real, RealSign};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[path = "support/representations.rs"]
/// Shared finite positive-value fixture inventory; labels alone do not prove storage identity.
mod representations;

/// Require a certified interval to lie inside [-2^-bits,2^-bits].
/// Request eight finer bits and construct the tolerance exactly. The fixture
/// uses positive, modest bit counts whose precision arithmetic fits i32.
/// Containing zero is allowed: this helper alone does not prove nonzero value,
/// positivity, correct rounding or agreement with an independent oracle.
fn small(value: &Real, bits: i32) {
    let [lower, upper] = value.certified_rational_interval(-bits - 8).unwrap();
    let tolerance = Rational::fraction(1, 2).unwrap().powi(bits.into()).unwrap();
    assert!(lower >= -tolerance.clone() && upper <= tolerance);
}

/// Require a certified positive sign at refinement floor -128.
/// Do not prescribe the proof source or work required to obtain it. This
/// separately prevents smallness checks from accepting a value replaced by zero.
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
/// Exercise three huge negative operands through exp, atan, sine and expm1-plus-one.
/// Use -10^320, -10^320+pi and -10^320*e, then request smallness at bit counts
/// 128,512,32,256 while retaining a positive exponential sign. The test does
/// not set a larger thread stack; successful completion is a finite regression,
/// not a measured stack bound or a guarantee for arbitrary expression depth.
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
/// Exercise the shared inventory's 22 construction cases after checking their expected public StructuralKind.
/// Multiply each positive fixture by -10^320, then require exp/atan composition
/// to remain small at 192 and 256 bits across a lossy float request, while exp
/// retains a certified positive sign. The float result is not an exact source
/// or an asserted oracle. This finite inventory does not cover every value or
/// prove each diagnostic label equals a private enum variant.
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
/// Refine atan(exp(-8)) in two coarse/fine request orders.
/// Require strictly positive interval bounds below 1/1000 at -128 and -256;
/// coarser requests only need to return an interval. Despite the name, this
/// test does not explicitly assert that a coarse integer approximation is zero.
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
/// Require exp(k*ln(b)) to return exactly b^k for four positive integer bases and 41 exponents.
/// Use bases 2,3,5,17 and k from -20 through 20, giving 164 exact-rational
/// assertions against Rational::powi. No approximation tolerance is accepted.
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
/// Reject an interval from a pre-stopped clone, clear its signal, then retry successfully.
/// After clearing, four scoped workers refine clones at different bit counts,
/// and the original is checked again after joining. This observes one schedule;
/// it neither cancels running workers nor measures cancellation latency or stack use.
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
/// Replay all 22 inventory-derived negative exponentials through JSON and native CBOR.
/// Check atan smallness and exponential positivity after each import, and require
/// the original JSON bytes to remain equal after refinement. The CBOR path has
/// no byte-equality assertion. Cold denotes a newly constructed expression,
/// not reset shared caches or a fresh process; serialization flags are not proof.
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
