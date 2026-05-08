//! In-depth fuzz testing of complex number operations and algebraic laws.
//!
//! Every arithmetic operator, all ownership variants, `powi` / `powi_checked`
//! across multiple exponents, and the `^` operator are exercised. Algebraic
//! invariants that hold exactly or by interval-arithmetic bounds are checked
//! as secondary guards.
//!
//! Run with: `cargo fuzz run complex_ops` from the `fuzz/` directory.

#![no_main]

use arbitrary::Arbitrary;
use libfuzzer_sys::fuzz_target;
use realistic_blas::{ApproxBackend, Complex, ZeroStatus};

#[derive(Arbitrary, Debug)]
struct Input {
    z: Complex<ApproxBackend>,
    w: Complex<ApproxBackend>,
}

fuzz_target!(|input: Input| {
    let Input { z, w } = input;

    // ── No-panic: owned and borrowed arithmetic ──────────────────────────────
    let _ = z.clone() + w.clone();
    let _ = z.clone() - w.clone();
    let _ = z.clone() * w.clone();
    let _ = z.clone() / w.clone();
    let _ = &z + &w;
    let _ = &z - &w;
    let _ = &z * &w;
    let _ = z.clone() + &w;
    let _ = z.clone() - &w;
    let _ = z.clone() * &w;
    let _ = &z + w.clone();
    let _ = &z - w.clone();
    let _ = &z * w.clone();
    let _ = -z.clone();
    let _ = -&z;

    // ── No-panic: scalar divisor ─────────────────────────────────────────────
    let _ = z.clone() / w.re.clone();
    let _ = z.clone() / &w.re;

    // ── No-panic: reciprocal and checked variants ─────────────────────────────
    let _ = z.clone().reciprocal();
    let _ = z.clone().reciprocal_checked();

    // ── No-panic: division with checked denominator ──────────────────────────
    let _ = z.clone().div_checked(w.clone());
    let _ = z.clone().div_real_checked(w.re.clone());

    // ── No-panic: norm and conjugate ─────────────────────────────────────────
    let _ = z.norm_squared();
    let _ = z.clone().conjugate();

    // ── No-panic: powi across a range of exponents ───────────────────────────
    for e in [0_i64, 1, 2, 3, 4, 5, -1, -2, -3, 7, -7] {
        let _ = z.clone().powi(e);
        let _ = z.clone().powi_checked(e);
    }

    // ── No-panic: ^ (BitXor) operator (delegates to powi) ────────────────────
    let _ = z.clone() ^ 0_i64;
    let _ = z.clone() ^ 1_i64;
    let _ = z.clone() ^ 2_i64;
    let _ = z.clone() ^ -1_i64;
    let _ = z.clone() ^ -3_i64;

    // ── Invariant: double negation is exact identity ─────────────────────────
    assert_eq!(-(-z.clone()), z, "double negation must be identity");

    // ── Invariant: conjugate is an involution ────────────────────────────────
    // conjugate only flips the sign of im; two flips return the original exactly.
    assert_eq!(
        z.clone().conjugate().conjugate(),
        z,
        "conjugate must be an involution"
    );

    // ── Invariant: z + (−z) components are within error bounds of zero ───────
    let zero_candidate = z.clone() + (-z.clone());
    assert_ne!(
        zero_candidate.re.zero_status(),
        ZeroStatus::NonZero,
        "real part of z + (-z) must be within error bounds of zero"
    );
    assert_ne!(
        zero_candidate.im.zero_status(),
        ZeroStatus::NonZero,
        "imaginary part of z + (-z) must be within error bounds of zero"
    );

    // ── Invariant: commutativity ─────────────────────────────────────────────
    assert_eq!(
        z.clone() + w.clone(),
        w.clone() + z.clone(),
        "complex addition must be commutative"
    );
    assert_eq!(
        z.clone() * w.clone(),
        w.clone() * z.clone(),
        "complex multiplication must be commutative"
    );

    // ── Invariant: additive identity ─────────────────────────────────────────
    let complex_zero = Complex::<ApproxBackend>::zero();
    assert_eq!(z.clone() + complex_zero.clone(), z, "z + 0 must equal z");
    assert_eq!(complex_zero + z.clone(), z, "0 + z must equal z");

    // ── Invariant: multiplicative identity ───────────────────────────────────
    let complex_one = Complex::<ApproxBackend>::one();
    assert_eq!(z.clone() * complex_one.clone(), z, "z * 1 must equal z");
    assert_eq!(complex_one * z.clone(), z, "1 * z must equal z");

    // ── Invariant: z · conj(z) equals norm_squared ───────────────────────────
    // re(z·conj(z)) = re²+im² (product_epsilon is symmetric) == norm_squared.
    let z_times_conj = z.clone() * z.clone().conjugate();
    assert_eq!(
        z_times_conj.re,
        z.norm_squared(),
        "real part of z·conj(z) must equal norm_squared"
    );

    // im(z·conj(z)) = re·(−im) + im·re = 0 exactly in IEEE 754 (a + (−a) = 0).
    assert_ne!(
        z_times_conj.im.zero_status(),
        ZeroStatus::NonZero,
        "imaginary part of z·conj(z) must be within error bounds of zero"
    );

    // ── Invariant: powi(NonZero, 0) is the identity ──────────────────────────
    // powi returns Complex::one() directly for exponent 0 on non-zero inputs.
    let is_zero_re = z.re.zero_status() == ZeroStatus::Zero;
    let is_zero_im = z.im.zero_status() == ZeroStatus::Zero;
    if !is_zero_re || !is_zero_im {
        if let Ok(result) = z.clone().powi(0) {
            assert_eq!(
                result,
                Complex::<ApproxBackend>::one(),
                "powi(non-zero z, 0) must equal 1"
            );
        }
    }

    // ── Invariant: owned + borrowed operators agree ───────────────────────────
    assert_eq!(
        z.clone() + w.clone(),
        z.clone() + &w,
        "Complex + Complex must equal Complex + &Complex"
    );
    assert_eq!(
        z.clone() * w.clone(),
        z.clone() * &w,
        "Complex * Complex must equal Complex * &Complex"
    );

    // ── Invariant: zero complex number has zero norm ──────────────────────────
    // norm_squared = 0·0 + 0·0 = 0 exactly (every cross term in product_epsilon
    // vanishes when both value and epsilon are 0.0).
    assert!(
        Complex::<ApproxBackend>::zero().norm_squared().definitely_zero(),
        "norm_squared of Complex::zero() must be exactly zero"
    );
});
