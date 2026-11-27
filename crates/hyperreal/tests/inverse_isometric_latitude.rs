use hyperreal::{Problem, Rational, Real};

#[path = "support/mpfr.rs"]
/// Register thread-exit cleanup for the test-only MPFR backend.
mod mpfr;
#[path = "support/representations.rs"]
/// Supply the shared finite inventory of 22 positive scalar construction recipes.
mod representations;

use num::{BigInt, BigUint};
use rug::{Float, float::Constant};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

/// Construct an exact rational Real fixture; a zero denominator is a test-author error.
fn fraction(n: i64, d: u64) -> Real {
    Real::new(Rational::fraction(n, d).unwrap())
}

/// Require the entire computed error enclosure to lie within +/-2^(-bits).
/// Request eight guard bits from the scalar enclosure API and compare exact
/// rational endpoints. This is an absolute-error assertion, conditional on that
/// API's enclosure contract, rather than a relative or correctly rounded test.
fn enclosed_error(actual: &Real, expected: &Real, bits: i32) {
    let [lo, hi] = (actual - expected)
        .certified_rational_interval(-bits - 8)
        .unwrap();
    let tolerance = Rational::fraction(1, 2).unwrap().powi(bits.into()).unwrap();
    assert!(
        lo >= -tolerance.clone() && hi <= tolerance,
        "error [{lo}, {hi}]"
    );
}

#[test]
/// Check three analytic inverse fixtures, odd symmetry and subsequent sine/cosine.
/// Use sin(phi)=3/5, tan(phi)=3/4 and s=ln(2), with spherical, oblate and prolate
/// axis ratios. Demand 160-bit absolute inverse agreement and 120-bit trig
/// agreement. The expected angle is independently constructed as atan(3/4).
fn independently_constructed_oblate_prolate_and_sphere_inverses() {
    // sin(phi)=3/5, tan(phi)=3/4, asinh(tan(phi))=ln(2).
    let expected = fraction(3, 4).atan().unwrap();
    let spherical = Real::from(2).ln().unwrap();
    let eccentricity = fraction(4, 5);
    let oblate = &spherical - &eccentricity * fraction(12, 25).atanh().unwrap();
    let prolate = &spherical + &eccentricity * fraction(12, 25).atan().unwrap();
    for (psi, ratio) in [
        (spherical, Real::one()),
        (oblate, fraction(3, 5)),
        (
            prolate,
            (Real::from(41).sqrt().unwrap() / Real::from(5)).unwrap(),
        ),
    ] {
        let phi = psi.clone().inverse_isometric_latitude(&ratio).unwrap();
        enclosed_error(&phi, &expected, 160);
        enclosed_error(
            &(-psi).inverse_isometric_latitude(&ratio).unwrap(),
            &-expected.clone(),
            160,
        );
        enclosed_error(&phi.clone().sin(), &fraction(3, 5), 120);
        enclosed_error(&phi.cos(), &fraction(4, 5), 120);
    }
}

#[test]
/// Reject zero/negative axis ratios even at psi=0, then accept the valid exact equator.
/// The typed NotANumber assertions prevent the zero shortcut from hiding these
/// invalid domains; they do not exhaust unresolved or malformed graph inputs.
fn axis_domain_is_checked_before_equator_identity() {
    for ratio in [Real::zero(), Real::from(-1)] {
        assert_eq!(
            Real::zero().inverse_isometric_latitude(&ratio),
            Err(Problem::NotANumber)
        );
    }
    assert_eq!(
        Real::zero()
            .inverse_isometric_latitude(&fraction(3, 5))
            .unwrap(),
        Real::zero()
    );
}

/// Convert the midpoint of a certified Real enclosure to a 768-bit MPFR test value.
/// Transfer exact numerator, denominator and sign explicitly; Display may contain
/// mixed fractions and is not a rational wire format. This midpoint conversion is
/// a rounded oracle input, not a directed MPFR enclosure or production scalar.
/// The caller registers MPFR thread cleanup.
fn mpfr_value(value: &Real, precision: i32) -> Float {
    let [lo, hi] = value.certified_rational_interval(precision).unwrap();
    let middle = (lo + hi) / Rational::new(2);
    // Display can use mixed fractions ("1 2/3"); that is not a wire format.
    let numerator: rug::Integer = middle.numerator().to_string().parse().unwrap();
    let denominator: rug::Integer = middle.denominator().to_string().parse().unwrap();
    let rational = rug::Rational::from((numerator, denominator));
    let rational = if middle.sign() == num::bigint::Sign::Minus {
        -rational
    } else {
        rational
    };
    Float::with_val(768, rational)
}

/// Build an independent numerical oracle by 260 bisections directly in geodetic latitude.
/// Evaluate the oblate/prolate forward equation with 768-bit MPFR using input
/// midpoints enclosed at precision -640, then adopt the final MPFR midpoint's
/// exact binary rational as a test reference. Fixed iteration and nearest rounding
/// make this a finite oracle, not a certified solve of the original exact inputs.
/// Register thread-local MPFR cleanup; callers choose tolerances within its useful
/// resolution, or use an analytic reference for the 512-bit case.
fn mpfr_inverse(psi: &Real, axis_ratio: &Real) -> Real {
    mpfr::cleanup_on_thread_exit();
    let psi = mpfr_value(psi, -640);
    let axis = mpfr_value(axis_ratio, -640);
    let square = axis.square();
    let oblate = square < 1;
    let eccentricity = (Float::with_val(768, 1) - square).abs().sqrt();
    let mut hi: Float = Float::with_val(768, Constant::Pi) / 2;
    let mut lo = -hi.clone();
    for _ in 0..260 {
        let mid: Float = Float::with_val(768, &lo + &hi) / 2;
        let argument = Float::with_val(768, &eccentricity * mid.clone().sin());
        let correction = Float::with_val(
            768,
            &eccentricity
                * if oblate {
                    argument.atanh()
                } else {
                    argument.atan()
                },
        );
        let f = if oblate {
            mid.clone().tan().asinh() - correction
        } else {
            mid.clone().tan().asinh() + correction
        };
        if f < psi {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let midpoint: Float = Float::with_val(768, lo + hi) / 2;
    let exact = midpoint.to_rational().unwrap();
    Real::new(
        Rational::from_bigint_fraction(
            exact.numer().to_string().parse::<BigInt>().unwrap(),
            exact.denom().to_string().parse::<BigUint>().unwrap(),
        )
        .unwrap(),
    )
}

#[test]
/// Compare a 35-point psi/axis grid to MPFR and exercise ten requested precisions.
/// The grid uses 192-bit absolute error. Nine schedule entries test reference
/// containment in a newly constructed result's enclosure; the -512 entry instead
/// checks the independent atan(3/4) analytic fixture beyond the 260-step oracle.
/// This finite schedule is not coverage of every representable precision.
fn independent_mpfr_grid_and_precision_schedule() {
    for psi in [-10, -3, -1, 0, 1, 3, 10] {
        for axis in [
            fraction(1, 100),
            fraction(3, 5),
            Real::one(),
            fraction(5, 3),
            Real::from(100),
        ] {
            let input = Real::from(psi);
            let expected = mpfr_inverse(&input, &axis);
            let actual = input.inverse_isometric_latitude(&axis).unwrap();
            enclosed_error(&actual, &expected, 192);
        }
    }
    // Approximation contract at every precision, not just display accuracy.
    let input = fraction(3, 7);
    let axis = fraction(3, 5);
    let expected = mpfr_inverse(&input, &axis);
    for p in [-512, -193, -64, -7, -3, -2, -1, 0, 1, 2] {
        let value = input.clone().inverse_isometric_latitude(&axis).unwrap();
        if p == -512 {
            // Independent analytic fixture supplies a reference beyond the
            // MPFR oracle's 260-step bisection precision.
            let psi =
                Real::from(2).ln().unwrap() - fraction(4, 5) * fraction(12, 25).atanh().unwrap();
            enclosed_error(
                &psi.inverse_isometric_latitude(&axis).unwrap(),
                &fraction(3, 4).atan().unwrap(),
                512,
            );
        } else {
            let [lo, hi] = value.certified_rational_interval(p).unwrap();
            let [reference, _] = expected.certified_rational_interval(0).unwrap();
            assert!(lo <= reference && reference <= hi, "precision {p}");
        }
    }
}

#[test]
/// Exercise 1e-80 and +/-1e320 inputs on three axis ratios and both near-sphere perturbations.
/// Compare tiny inputs with the psi/(b/a)^2 leading term at 512 bits, huge inputs
/// with the appropriate limiting pole at 256 bits, and axis ratios 1+/-1e-80
/// with the spherical expression at 240 bits. Pole proximity is an absolute-error
/// claim; it does not identify a finite input's latitude with the pole.
fn tiny_huge_and_near_spherical_inputs_keep_their_meaning() {
    let tiny: Real = "1e-80".parse().unwrap();
    for axis in [fraction(3, 5), Real::one(), fraction(5, 3)] {
        let actual = tiny.clone().inverse_isometric_latitude(&axis).unwrap();
        let leading = (tiny.clone() / (&axis * &axis)).unwrap();
        // The next term is cubic, far below this absolute bound.
        enclosed_error(&actual, &leading, 512);
        let huge: Real = "1e320".parse().unwrap();
        let pole = (Real::pi() / Real::from(2)).unwrap();
        enclosed_error(
            &huge.clone().inverse_isometric_latitude(&axis).unwrap(),
            &pole,
            256,
        );
        enclosed_error(
            &(-huge).inverse_isometric_latitude(&axis).unwrap(),
            &-pole,
            256,
        );
    }
    let input = fraction(7, 3);
    let sphere = input.clone().sinh().unwrap().atan().unwrap();
    for delta in [tiny.clone(), -tiny] {
        let axis = Real::one() + delta;
        enclosed_error(
            &input.clone().inverse_isometric_latitude(&axis).unwrap(),
            &sphere,
            240,
        );
    }
}

#[test]
/// Recover atan(3/4) at 160-bit absolute accuracy for axis ratios 1e-40 and 1e40.
/// Construct each isometric input independently from sin(phi)=3/5 and ln(2),
/// retaining the respective oblate/prolate elementary correction exactly.
fn extreme_axis_ratios_preserve_an_independently_constructed_latitude() {
    let expected = fraction(3, 4).atan().unwrap();
    for (text, oblate) in [("1e-40", true), ("1e40", false)] {
        let axis: Real = text.parse().unwrap();
        let e2 = Real::one() - &axis * &axis;
        let e = e2.abs().sqrt().unwrap();
        let argument = &e * fraction(3, 5);
        let spherical = Real::from(2).ln().unwrap();
        let psi = if oblate {
            spherical - &e * argument.atanh().unwrap()
        } else {
            spherical + &e * argument.atan().unwrap()
        };
        enclosed_error(
            &psi.inverse_isometric_latitude(&axis).unwrap(),
            &expected,
            160,
        );
    }
}

#[test]
/// Reject an already aborted enclosure, recover after clearing the flag, then refine four clones.
/// Compare with the MPFR reference at 160 bits and concurrent precisions 32,
/// 128,192,64. This is clone/retry evidence within one process, not serialized
/// replay, a mid-solve cancellation test or a cache-internals inspection.
fn cancellation_and_concurrent_refinement_do_not_poison_replay() {
    let original = fraction(3, 7)
        .inverse_isometric_latitude(&fraction(3, 5))
        .unwrap();
    let expected = mpfr_inverse(&fraction(3, 7), &fraction(3, 5));
    let signal = Arc::new(AtomicBool::new(true));
    let mut stopped = original.clone();
    stopped.abort(signal.clone());
    assert!(stopped.certified_rational_interval(-160).is_none());
    enclosed_error(&original, &expected, 160);
    signal.store(false, Ordering::Relaxed);
    enclosed_error(&stopped, &expected, 160);
    std::thread::scope(|scope| {
        for bits in [32, 128, 192, 64] {
            let value = original.clone();
            let expected = expected.clone();
            scope.spawn(move || enclosed_error(&value, &expected, bits));
        }
    });
}

#[test]
/// Exercise all 484 ordered pairs of the 22 positive scalar fixture recipes.
/// Check the input recipe's public structural category, inverse accuracy at 96
/// bits against MPFR, and subsequent sine/cosine at 80 bits. These named finite
/// recipes do not enumerate all values or all possible private expression graphs.
fn every_scalar_family_pair_survives_inverse_and_trigonometry() {
    let cases = representations::optimized_certificate_representatives();
    assert_eq!(cases.len(), 22);
    for input in &cases {
        assert_eq!(
            input.value.detailed_facts().symbolic.kind,
            input.public_kind
        );
        for axis in &cases {
            let expected = mpfr_inverse(&input.value, &axis.value);
            let latitude = input
                .value
                .clone()
                .inverse_isometric_latitude(&axis.value)
                .unwrap_or_else(|e| panic!("{} / {}: {e}", input.certificate, axis.certificate));
            enclosed_error(&latitude, &expected, 96);
            enclosed_error(&latitude.clone().sin(), &expected.clone().sin(), 80);
            enclosed_error(&latitude.cos(), &expected.cos(), 80);
        }
    }
}

#[cfg(feature = "serde")]
#[test]
/// Check native replay and trig composition for each of the 22 positive input recipes.
/// Require the inverse node tag, compatible cold/fine signs, 160/192-bit numerical
/// agreement, unchanged JSON after warming, and CBOR replay followed by sine.
/// All reimports occur in the same process; this is not a fresh-process cache or
/// malformed-payload qualification. Deterministic JSON and represented value are
/// checked separately.
fn cold_warm_json_and_cbor_retain_the_defining_equation() {
    for case in representations::optimized_certificate_representatives() {
        let expected = mpfr_inverse(&case.value, &fraction(3, 5));
        let value = case
            .value
            .inverse_isometric_latitude(&fraction(3, 5))
            .unwrap();
        let cold = value.to_json();
        assert!(cold.contains("InverseIsometricLatitude"));
        let reconstructed = Real::from_json(&cold).unwrap();
        let cold_sign = reconstructed.certified_sign_until(0);
        // A decoded opaque argument such as pi-2 need not have a zero-bit
        // sign certificate. Refinement must recover its positive sign; any
        // already definite cold answer must agree, never be contradicted.
        assert!(matches!(
            reconstructed.certified_sign_until(-160),
            hyperreal::CertifiedRealSign::Known {
                sign: hyperreal::RealSign::Positive,
                ..
            }
        ));
        if let hyperreal::CertifiedRealSign::Known { sign, .. } = cold_sign {
            assert_eq!(sign, hyperreal::RealSign::Positive);
        }
        if case.certificate == "One" {
            assert!(matches!(
                cold_sign,
                hyperreal::CertifiedRealSign::Known { .. }
            ));
        }
        enclosed_error(&reconstructed, &expected, 160);
        let _ = value.to_f64_lossy();
        enclosed_error(&value, &expected, 192);
        assert_eq!(cold, value.to_json());
        let mut bytes = Vec::new();
        ciborium::ser::into_writer(&value, &mut bytes).unwrap();
        let restored: Real = ciborium::de::from_reader(bytes.as_slice()).unwrap();
        enclosed_error(&restored, &expected, 160);
        enclosed_error(&restored.sin(), &expected.sin(), 128);
    }
}

#[test]
/// Compare four signed improper rational fixtures against direct 768-bit MPFR division.
/// This guards explicit numerator/denominator/sign transfer against the earlier
/// mixed-fraction Display parsing error and registers backend cleanup.
fn oracle_adapter_preserves_mixed_fraction_values() {
    mpfr::cleanup_on_thread_exit();
    for (n, d) in [(5, 3), (-5, 3), (7, 4), (-7, 4)] {
        let reference = Float::with_val(768, n) / Float::with_val(768, d);
        assert_eq!(mpfr_value(&fraction(n, d as u64), -640), reference);
    }
}
