//! Finite branch, scaling, replay, and oracle regressions for complex atanh.
//!
//! Exact axis/error assertions are separate from bounded interval comparisons.
//! The independent-formula oracle has documented MPFR rounding gaps; passing it
//! is not a universal certified-enclosure or exact-equality proof. Test names do
//! not establish exhaustive scalar coverage or fresh-process replay.

use hyperlattice::{Complex, Problem, Rational, Real, RealSign};

/// Check selected real/imaginary axis identities structurally, Infinity at exact ±1, and NotANumber at exact ±2; real scalar methods supply the axis expectations.
#[test]
fn axes_and_principal_cuts_have_explicit_semantics() {
    for x in [
        Real::zero(),
        "0.5".parse().unwrap(),
        "-0.5".parse().unwrap(),
    ] {
        assert_eq!(
            Complex::from(x.clone()).atanh().unwrap(),
            Complex::from(x.atanh().unwrap())
        );
    }
    for y in [
        Real::zero(),
        Real::pi(),
        -Real::e(),
        "1e300".parse().unwrap(),
    ] {
        assert_eq!(
            Complex::new(Real::zero(), y.clone()).atanh().unwrap(),
            Complex::new(Real::zero(), y.atan().unwrap())
        );
    }
    for x in [Real::one(), -Real::one()] {
        assert_eq!(Complex::from(x).atanh(), Err(Problem::Infinity));
    }
    for x in [Real::from(2), Real::from(-2)] {
        assert_eq!(Complex::from(x).atanh(), Err(Problem::NotANumber));
    }
}

/// Shared thread-local MPFR cache cleanup; used by the two independent-reference tests.
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr;
/// Local-series/reciprocal reference with documented radius/tail rounding limitations; also consumed by a benchmark.
#[path = "support/complex_atanh_oracle.rs"]
mod oracle;
/// Shared finite labeled scalar recipes; labels are not distinct private-tag assertions in this test.
#[path = "../../hyperreal/tests/support/representations.rs"]
mod representations;

/// Compare nine small fixtures and ten large nonreal fixtures against the series/reciprocal reference, requiring overlap and narrow rounded widths rather than exact equality.
#[test]
fn independent_series_and_reciprocal_enclosures() {
    mpfr::cleanup_on_thread_exit();
    for x in ["-0.5", "0", "0.5"] {
        for y in ["-0.125", "0", "0.125"] {
            oracle::compare(&Complex::new(x.parse().unwrap(), y.parse().unwrap()));
        }
    }
    for x in ["-4", "0", "4", "1e300", "-1e300"] {
        for y in ["-2", "2"] {
            oracle::compare(&Complex::new(x.parse().unwrap(), y.parse().unwrap()));
        }
    }
}

/// Visit the shared finite recipe list, assert each advertised public kind, then compare scaled values and reciprocals.
///
/// For each current recipe use small=v/(4*(v+1)), both real signs, and imaginary
/// part small/2. The recipe list currently has 22 entries; this test does not assert
/// its count or distinguish every private representation. Oracle limitations remain.
#[test]
fn every_scalar_family_reaches_the_independent_reference() {
    mpfr::cleanup_on_thread_exit();
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(
            case.value.detailed_facts().symbolic.kind,
            case.public_kind,
            "{}",
            case.certificate
        );
        let small = (&case.value / (Real::from(4) * (&case.value + Real::one()))).unwrap();
        for sign in [-1, 1] {
            let z = Complex::new(Real::from(sign) * &small, (&small / Real::from(2)).unwrap());
            oracle::compare(&z);
            oracle::compare(&z.reciprocal().unwrap());
        }
    }
}

/// Require a certified difference enclosure at -bits-16 to lie within ±2^-bits.
///
/// Uses exact rational bounds and panics on unavailable enclosure or invalid
/// arithmetic. This establishes the requested finite tolerance, not exact equality;
/// callers supply positive small bits, and no overflow/work guard is added.
fn close(a: &Real, b: &Real, bits: i32) {
    let [lo, hi] = (a - b).certified_rational_interval(-bits - 16).unwrap();
    let tolerance = Rational::fraction(1, 2).unwrap().powi(bits.into()).unwrap();
    assert!(
        lo >= -tolerance.clone() && hi <= tolerance,
        "difference [{lo}, {hi}]"
    );
}

/// Apply close to real then imaginary components using the same absolute tolerance; no relative-error or branch proof follows from closeness alone.
fn complex_close(a: &Complex, b: &Complex, bits: i32) {
    close(&a.re, &b.re, bits);
    close(&a.im, &b.im, bits);
}

/// Build tanh(x+iy) from sinh(2x), sin(2y), and cosh(2x)+cos(2y).
///
/// This forward identity is distinct from the inverse implementation, but shares
/// native Real arithmetic/transcendentals. All construction errors unwrap-panic;
/// fixture selection, not this helper, excludes poles.
fn forward_tanh(z: Complex) -> Complex {
    let twice_re = Real::from(2) * z.re;
    let twice_im = Real::from(2) * z.im;
    let denominator = twice_re.clone().cosh().unwrap() + twice_im.clone().cos();
    Complex::new(
        (twice_re.sinh().unwrap() / &denominator).unwrap(),
        (twice_im.sin() / denominator).unwrap(),
    )
}

/// Check conjugation, oddness, and forward tanh within 2^-192 for five fixtures, plus an exact positive sign for pi/2-|Im(atanh(z))|.
#[test]
fn conjugation_parity_and_forward_composition_preserve_the_branch() {
    for (x, y) in [
        ("0.125", "0.25"),
        ("4", "2"),
        ("-4", "2"),
        ("0.75", "-0.75"),
        ("1", "1"),
    ] {
        let z = Complex::new(x.parse().unwrap(), y.parse().unwrap());
        let value = z.clone().atanh().unwrap();
        complex_close(
            &z.clone().conjugate().atanh().unwrap(),
            &value.clone().conjugate(),
            192,
        );
        complex_close(&(-z.clone()).atanh().unwrap(), &(-value.clone()), 192);
        complex_close(&forward_tanh(value.clone()), &z, 192);
        let margin = (Real::pi() / Real::from(2)).unwrap() - value.im.abs();
        assert_eq!(
            margin.certified_sign_until(-256).sign(),
            Some(RealSign::Positive)
        );
    }
}

/// Check scaled leading corrections near ±1 and the exterior cuts at ±2 from both imaginary signs.
///
/// Use delta=10^-40 and 10^-320. Normalize real corrections by delta^2 and imaginary
/// corrections by delta, then require 2^-64 closeness to analytic coefficients.
/// These finite asymptotic checks preserve the original hard cases but do not prove
/// a uniform remainder bound or all branch-cut behavior.
#[test]
fn both_banks_and_tiny_pole_offsets_remain_distinct() {
    for text in ["1e-40", "1e-320"] {
        let delta: Real = text.parse().unwrap();
        let delta2 = &delta * &delta;
        for sx in [-1, 1] {
            for sy in [-1, 1] {
                let z = Complex::new(Real::from(sx), Real::from(sy) * &delta);
                let result = z.atanh().unwrap();
                let main = (Real::from(sx)
                    * (Real::from(2).ln().unwrap() - delta.clone().ln().unwrap())
                    / Real::from(2))
                .unwrap();
                close(
                    &((&result.re - main) / &delta2).unwrap(),
                    &(Real::from(sx) / Real::from(16)).unwrap(),
                    64,
                );
                let main = (Real::from(sy) * Real::pi() / Real::from(4)).unwrap();
                close(
                    &((&result.im - main) / &delta).unwrap(),
                    &(Real::from(sy) / Real::from(4)).unwrap(),
                    64,
                );
                let z = Complex::new(Real::from(2 * sx), Real::from(sy) * &delta);
                let result = z.atanh().unwrap();
                let main = (Real::from(sx) * Real::from(3).ln().unwrap() / Real::from(2)).unwrap();
                close(
                    &((&result.re - main) / &delta2).unwrap(),
                    &(Real::from(-2 * sx) / Real::from(9)).unwrap(),
                    64,
                );
                let main = (Real::from(sy) * Real::pi() / Real::from(2)).unwrap();
                close(
                    &((&result.im - main) / &delta).unwrap(),
                    &(Real::from(-sy) / Real::from(3)).unwrap(),
                    64,
                );
            }
        }
    }
}

/// Use opaque sin(e)^2+cos(e)^2-1 to reproduce unresolved zero sign and cut refusal.
///
/// Assert no immediate/refined sign, require Exhausted with real part two, and
/// compare two cut-free interior/imaginary-axis constructions within 2^-192.
/// The fixture is mathematically zero; unavailable sign knowledge is not nonzero.
#[test]
fn unknown_signs_are_not_silently_classified_as_cuts() {
    let zero = Real::e().sin().powi(2.into()).unwrap() + Real::e().cos().powi(2.into()).unwrap()
        - Real::one();
    assert_eq!(zero.immediate_sign(), None);
    assert_eq!(
        zero.certified_sign_until(Real::PARTIAL_CMP_MIN_PRECISION)
            .sign(),
        None
    );
    assert_eq!(
        Complex::new(Real::from(2), zero.clone()).atanh(),
        Err(Problem::Exhausted)
    );
    let x: Real = "0.25".parse().unwrap();
    complex_close(
        &Complex::new(x.clone(), zero.clone()).atanh().unwrap(),
        &Complex::from(x.atanh().unwrap()),
        192,
    );
    let y: Real = "0.125".parse().unwrap();
    complex_close(
        &Complex::new(zero, y.clone()).atanh().unwrap(),
        &Complex::new(Real::zero(), y.atan().unwrap()),
        192,
    );
}

/// Exercise interval requests, cancellation/retry, and concurrent reads on two result components.
///
/// Require successive overlap in the 32→512→64→256 sequence, not nested intervals.
/// Attach an already-set signal to a clone, require no -1024 enclosure, clear it,
/// and compare within 2^-256. Four reader threads require available intervals only;
/// they do not compare every reader's values or establish independent cold caches.
#[test]
fn precision_cancellation_and_concurrent_readers_preserve_values() {
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };
    let value = Complex::new(
        (Real::pi() / Real::from(8)).unwrap(),
        (Real::e() / Real::from(8)).unwrap(),
    )
    .atanh()
    .unwrap();
    for component in [&value.re, &value.im] {
        let mut previous: Option<[Rational; 2]> = None;
        for bits in [32, 512, 64, 256] {
            let current = component.certified_rational_interval(-bits).unwrap();
            if let Some(old) = previous {
                assert!(old[0] <= current[1] && current[0] <= old[1]);
            }
            previous = Some(current);
        }
        let signal = Arc::new(AtomicBool::new(true));
        let mut aborted = component.clone();
        aborted.abort(signal.clone());
        assert!(aborted.certified_rational_interval(-1024).is_none());
        signal.store(false, Ordering::Relaxed);
        close(&aborted, component, 256);
        std::thread::scope(|scope| {
            for bits in [32, 128, 256, 512] {
                scope
                    .spawn(move || assert!(component.certified_rational_interval(-bits).is_some()));
            }
        });
    }
}

/// Round-trip four result component pairs through JSON in the same process.
///
/// Require 2^-256 closeness and deterministic reserialization text. This does not
/// establish exact value equality, fresh-process independence, or replay of the
/// original complex input/operation and its branch-decision evidence.
#[cfg(feature = "serde")]
#[test]
fn serialized_components_replay_the_selected_branch() {
    for (x, y) in [("0.125", "0.25"), ("-4", "2"), ("4", "-2"), ("1", "0.0001")] {
        let value = Complex::new(x.parse().unwrap(), y.parse().unwrap())
            .atanh()
            .unwrap();
        let values = [value.re, value.im];
        let serialized = serde_json::to_string(&values).unwrap();
        let decoded: [Real; 2] = serde_json::from_str(&serialized).unwrap();
        for (a, b) in values.iter().zip(&decoded) {
            close(a, b, 256);
        }
        assert_eq!(serialized, serde_json::to_string(&decoded).unwrap());
    }
}

/// Use cos(e)+i*sin(e), check imaginary output near pi/4 and forward composition near the input at 2^-192; no explicit unit-norm knowledge assertion is made.
#[test]
fn unit_circle_without_a_norm_zero_certificate_remains_admitted() {
    let angle = Real::e();
    let z = Complex::new(angle.clone().cos(), angle.sin());
    let value = z.clone().atanh().unwrap();
    close(&value.im, &(Real::pi() / Real::from(4)).unwrap(), 192);
    complex_close(&forward_tanh(value), &z, 192);
}

/// Check 10^-320 near-zero linear scaling and 10^320 far-field corrections for both imaginary signs, normalized before 2^-128 closeness comparisons.
#[test]
fn tiny_values_and_huge_reciprocal_corrections_are_not_erased() {
    let tiny: Real = "1e-320".parse().unwrap();
    let huge: Real = "1e320".parse().unwrap();
    for sign in [-1, 1] {
        let value = Complex::new(tiny.clone(), Real::from(sign) * &tiny)
            .atanh()
            .unwrap();
        close(&(value.re / &tiny).unwrap(), &Real::one(), 128);
        close(&(value.im / &tiny).unwrap(), &Real::from(sign), 128);
        let value = Complex::new(huge.clone(), Real::from(sign) * &huge)
            .atanh()
            .unwrap();
        close(&(value.re * &huge), &"0.5".parse().unwrap(), 128);
        let limit = (Real::from(sign) * Real::pi() / Real::from(2)).unwrap();
        close(
            &((value.im - limit) * &huge),
            &(Real::from(-sign) / Real::from(2)).unwrap(),
            128,
        );
    }
}
