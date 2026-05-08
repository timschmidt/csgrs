//! In-depth fuzz testing of scalar operations and algebraic laws.
//!
//! Covers all arithmetic operations across every ownership variant, free
//! functions (`powi`, `reciprocal`, `sinh`, `cosh`, `tanh`, …), and a
//! comprehensive set of algebraic invariants for [`ApproxBackend`] scalars.
//!
//! Run with: `cargo +nightly fuzz run scalar_ops` from the `fuzz/` directory.

#![no_main]

use arbitrary::Arbitrary;
use libfuzzer_sys::fuzz_target;
use realistic_blas::{
    ApproxBackend, HyperrealBackend, Scalar, ZeroStatus,
    cosh, powi, reciprocal, reciprocal_checked, reciprocal_ref, reciprocal_ref_checked, sinh,
    tanh,
};

#[derive(Debug)]
struct Input<Backend: realistic_blas::Backend> {
    a: Scalar<Backend>,
    b: Scalar<Backend>,
    /// Bounded exponent so powi computation stays tractable.
    exp: i8,
}

impl<'a, Backend: realistic_blas::Backend> Arbitrary<'a> for Input<Backend> where Scalar<Backend>: Arbitrary<'a> {
    fn arbitrary(u: &mut arbitrary::Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self {
            a: Arbitrary::arbitrary(u)?,
            b: Arbitrary::arbitrary(u)?,
            exp: Arbitrary::arbitrary(u)?,
        })
    }
}

fuzz_target!(|input: (Input<ApproxBackend>, Input<HyperrealBackend>)| {
    let (approx_input, hyperreal_input) = input;
    scalar_fuzz(approx_input);
    scalar_fuzz(hyperreal_input);
});

fn scalar_fuzz<Backend: realistic_blas::Backend>(input: Input<Backend>) {
    let Input { a, b, exp } = input;

    // ── No-panic: owned and borrowed arithmetic ──────────────────────────────
    // Every ownership combination is exercised so all four impl variants are hit.
    let _ = a.clone() + b.clone();
    let _ = a.clone() - b.clone();
    let _ = a.clone() * b.clone();
    let _ = a.clone() / b.clone();
    let _ = &a + &b;
    let _ = &a - &b;
    let _ = &a * &b;
    let _ = &a / &b;
    let _ = a.clone() + &b;
    let _ = a.clone() - &b;
    let _ = a.clone() * &b;
    let _ = a.clone() / &b;
    let _ = &a + b.clone();
    let _ = &a - b.clone();
    let _ = &a * b.clone();
    let _ = &a / b.clone();
    let _ = -a.clone();

    // ── No-panic: unary operations ───────────────────────────────────────────
    let _ = a.clone().inverse();
    let _ = a.inverse_ref();
    let _ = a.clone().sqrt();
    let _ = a.clone().exp();
    let _ = a.clone().ln();
    let _ = a.clone().log10();
    let _ = a.clone().sin();
    let _ = a.clone().cos();
    let _ = a.clone().tan();
    let _ = a.clone().asin();
    let _ = a.clone().acos();
    let _ = a.clone().atan();
    let _ = a.clone().asinh();
    let _ = a.clone().acosh();
    let _ = a.clone().atanh();
    let _ = a.clone().pow(b.clone());

    // ── No-panic: free function variants ────────────────────────────────────
    let _ = reciprocal(a.clone());
    let _ = reciprocal_ref(&a);
    let _ = reciprocal_checked(a.clone());
    let _ = reciprocal_ref_checked(&a);
    let _ = sinh(a.clone());
    let _ = cosh(a.clone());
    let _ = tanh(a.clone());

    // ── No-panic: powi with a range of exponents ────────────────────────────
    for e in [0_i64, 1, 2, 3, 4, 5, -1, -2, -3, i64::from(exp)] {
        let _ = powi(a.clone(), e);
    }

    // ── No-panic: query and classification methods ───────────────────────────
    let _ = a.zero_status();
    let _ = a.structural_facts();
    let _ = a.to_f64_approx();
    let _ = a.definitely_zero();
    let _ = a.refine_sign_until(0);
    let _ = a.refine_sign_until(53);

    // ── Invariant: double negation is exact identity ─────────────────────────
    assert_eq!(-(-a.clone()), a, "double negation must be identity");

    // ── Invariant: a + (−a) is within error bounds of zero ──────────────────
    assert_ne!(
        (a.clone() + (-a.clone())).zero_status(),
        ZeroStatus::NonZero,
        "a + (-a) must be within error bounds of zero"
    );

    // ── Invariant: commutativity ─────────────────────────────────────────────
    assert_eq!(a.clone() + b.clone(), b.clone() + a.clone(), "addition must be commutative");
    assert_eq!(a.clone() * b.clone(), b.clone() * a.clone(), "multiplication must be commutative");

    // ── Invariant: owned and borrowed operators agree ────────────────────────
    // Both paths delegate to add_refs / mul_refs so results must be identical.
    assert_eq!(
        a.clone() + b.clone(),
        a.clone() + &b,
        "Scalar + Scalar must equal Scalar + &Scalar"
    );
    assert_eq!(
        a.clone() * b.clone(),
        a.clone() * &b,
        "Scalar * Scalar must equal Scalar * &Scalar"
    );

    // ── Invariant: additive identity ─────────────────────────────────────────
    let zero = Scalar::<Backend>::zero();
    let one = Scalar::<Backend>::one();
    assert_eq!(a.clone() + zero.clone(), a, "a + 0 must equal a");
    assert_eq!(zero.clone() + a.clone(), a, "0 + a must equal a");

    // ── Invariant: multiplicative identity ───────────────────────────────────
    assert_eq!(a.clone() * one.clone(), a, "a * 1 must equal a");
    assert_eq!(one.clone() * a.clone(), a, "1 * a must equal a");

    // ── Invariant: zero annihilator ──────────────────────────────────────────
    // a * 0 produces value=0.0 and epsilon=0.0 exactly (all cross terms vanish).
    assert!(
        (a.clone() * zero.clone()).definitely_zero(),
        "a * 0 must be exactly zero"
    );

    // ── Invariant: zero_status and definitely_zero consistency ───────────────
    if a.zero_status() == ZeroStatus::NonZero {
        assert!(
            !a.definitely_zero(),
            "NonZero zero_status contradicts definitely_zero()"
        );
    }
    if a.definitely_zero() {
        assert_eq!(
            a.zero_status(),
            ZeroStatus::Zero,
            "definitely_zero() implies Zero status"
        );
    }

    // ── Invariant: ApproxBackend never claims exact rational representation ───
    assert!(
        !a.structural_facts().exact_rational,
        "ApproxBackend must not claim exact_rational"
    );

    // ── Invariant: powi(a, 1) is the identity ────────────────────────────────
    match powi(a.clone(), 1) {
        Ok(result) => assert_eq!(result, a, "powi(a, 1) must equal a"),
        Err(e) => panic!("powi(a, 1) must always succeed; got {e:?}"),
    }

    // ── Invariant: powi(a, 2) equals a * a ──────────────────────────────────
    // Both paths route through mul_refs on identical operands so results must
    // be bit-identical, not merely within error bounds.
    let a_squared = a.clone() * a.clone();
    match powi(a.clone(), 2) {
        Ok(result) => assert_eq!(result, a_squared, "powi(a, 2) must equal a * a"),
        Err(e) => panic!("powi(a, 2) must always succeed; got {e:?}"),
    }

    // ── Invariant: powi(NonZero, 0) is the multiplicative identity ───────────
    if a.zero_status() == ZeroStatus::NonZero {
        match powi(a.clone(), 0) {
            Ok(result) => assert_eq!(result, one, "powi(NonZero, 0) must equal 1"),
            Err(e) => panic!("powi(NonZero, 0) must succeed; got {e:?}"),
        }
    }
}
