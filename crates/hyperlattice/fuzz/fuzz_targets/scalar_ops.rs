//! Fuzz selected scalar operations and algebraic identities from exact dyadic seeds.
//!
//! Local generation selects sanitized finite f64 or i128 values. Calls all four
//! ownership combinations of binary arithmetic, selected unary/transcendental APIs,
//! checked reciprocals, integer powers, and structural/lossy queries. General powers
//! use half-integer exponents from -4 to 4; the integer-power list also includes an
//! input i8 exponent. Seeds can span the finite binary64 range, so input magnitudes
//! and nested evaluation cost are not small merely because exponents are bounded.
//!
//! Most discarded Result values exercise panic resistance, not successful completion
//! or correctness of the returned error/value. Assertions compare exact Real carriers
//! or structural statuses. A check against NonZero also accepts Unknown and is not a
//! zero proof or a numerical error bound. No independent total-work/memory limit is
//! enforced; magnitude/exponent/recipe limits are not a completed-workload guarantee.
//! The typed libFuzzer macro uses Arbitrary input decoding before the callback; manual
//! Input implementations retain its default zero lower size hint. Macro-generated
//! entry functions are not individually authored/documented Rust items here.

#![no_main]

use arbitrary::{Arbitrary, Error, Unstructured};
use hyperlattice::{Real, ZeroKnowledge, reciprocal_checked, reciprocal_ref_checked};
use libfuzzer_sys::fuzz_target;

/// Decoded controls for this target; field order determines input consumption and the generated fixtures.
#[derive(Debug)]
struct Input {
    /// First exact dyadic/integer seed, decoded before b.
    a: Real,
    /// Second exact dyadic/integer seed; zero and large finite magnitudes are permitted.
    b: Real,
    /// Full i8 exponent (-128..=127); also selects half-integer general-power exponents via remainder modulo 17.
    exp: i8,
}

impl<'a> Arbitrary<'a> for Input {
    /// Generate fields in declaration order from one cursor, propagating the first error.
    ///
    /// No custom size hint is supplied; current primitive decoding can zero-pad short
    /// input, including synthesizing all-zero operands from an empty byte slice.
    fn arbitrary(u: &mut arbitrary::Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self {
            a: arbitrary_real(u)?,
            b: arbitrary_real(u)?,
            exp: Arbitrary::arbitrary(u)?,
        })
    }
}

/// Decode a u64 bit pattern as f64, retaining finite values and replacing every NaN/infinity with +0.0.
///
/// Finite subnormals and negative zero are reachable at this primitive stage. This
/// is a fuzz-input sanitization rule, not a valid Real conversion of a nonfinite
/// number. Arbitrary 1.4.2 zero-pads short integer input; missing bytes need not fail.
fn finite_f64(u: &mut Unstructured<'_>) -> arbitrary::Result<f64> {
    let bits = u64::arbitrary(u)?;
    let f = f64::from_bits(bits);
    if f.is_finite() { Ok(f) } else { Ok(0.0) }
}

/// Choose a finite binary64 import or an i128 integer with ratio(1,2), then construct an exact Real.
///
/// The two routes produce dyadic rationals/integers, not arbitrary symbolic or
/// non-dyadic rational data. Selection follows input bytes rather than a uniform
/// mathematical-value distribution. Native float-conversion errors map to
/// IncorrectFormat. Current primitive decoding zero-pads missing bytes, so even
/// empty input can produce zero; this helper does not enforce a minimum input size.
fn arbitrary_real(u: &mut Unstructured<'_>) -> arbitrary::Result<Real> {
    if u.ratio(1, 2)? {
        Real::try_from(finite_f64(u)?).map_err(|_| Error::IncorrectFormat)
    } else {
        Ok(Real::from(i128::arbitrary(u)?))
    }
}

fuzz_target!(|input: Input| {
    scalar_fuzz(input);
});

/// Run the listed scalar calls, then exact identities and conservative structural consistency assertions.
///
/// Most fallible outcomes are discarded. Cancellation a+(-a) is checked only to be
/// not NonZero; powers 1 and 2 must succeed and match the input/product, while the
/// power-zero assertion runs only for structurally NonZero input. Lossy exports and
/// sign refinement at precision arguments 0 and 53 are not checked for accuracy.
fn scalar_fuzz(input: Input) {
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
    // Keep general powers representative without allowing an arbitrary exact
    // integer exponent to allocate gigabytes before the remaining API is
    // exercised. Half-integers cover both integer and rational exponent paths.
    let pow_exponent = f64::from(i16::from(exp).rem_euclid(17) - 8) / 2.0;
    let pow_exponent = Real::try_from(pow_exponent).expect("finite dyadic exponent");
    let _ = a.clone().pow(pow_exponent);

    // ── No-panic: checked reciprocals and hyperbolic operations ──────────────
    let _ = reciprocal_checked(a.clone());
    let _ = reciprocal_ref_checked(&a);
    let _ = Real::sinh(a.clone());
    let _ = Real::cosh(a.clone());
    let _ = Real::tanh(a.clone());

    // ── No-panic: powi with a range of exponents ────────────────────────────
    for e in [0_i64, 1, 2, 3, 4, 5, -1, -2, -3, i64::from(exp)] {
        let _ = Real::powi_i64(a.clone(), e);
    }

    // ── No-panic: query and classification methods ───────────────────────────
    let _ = a.zero_status();
    let _ = a.structural_facts();
    let _ = a.to_f64_lossy();
    let _ = a.definitely_zero();
    let _ = a.refine_sign_until(0);
    let _ = a.refine_sign_until(53);

    // ── Invariant: double negation is exact identity ─────────────────────────
    assert_eq!(-(-a.clone()), a, "double negation must be identity");

    // ── Invariant: cancellation must not classify NonZero (Unknown passes) ──
    assert_ne!(
        (a.clone() + (-a.clone())).zero_status(),
        ZeroKnowledge::NonZero,
        "a + (-a) must be within error bounds of zero"
    );

    // ── Invariant: commutativity ─────────────────────────────────────────────
    assert_eq!(
        a.clone() + b.clone(),
        b.clone() + a.clone(),
        "addition must be commutative"
    );
    assert_eq!(
        a.clone() * b.clone(),
        b.clone() * a.clone(),
        "multiplication must be commutative"
    );

    // ── Invariant: owned and borrowed operators agree ────────────────────────
    // These ownership forms are compared through the same scalar arithmetic API.
    assert_eq!(
        a.clone() + b.clone(),
        a.clone() + &b,
        "Real + Real must equal Real + &Real"
    );
    assert_eq!(
        a.clone() * b.clone(),
        a.clone() * &b,
        "Real * Real must equal Real * &Real"
    );

    // ── Invariant: additive identity ─────────────────────────────────────────
    let zero = Real::zero();
    let one = Real::one();
    assert_eq!(a.clone() + zero.clone(), a, "a + 0 must equal a");
    assert_eq!(zero.clone() + a.clone(), a, "0 + a must equal a");

    // ── Invariant: multiplicative identity ───────────────────────────────────
    assert_eq!(a.clone() * one.clone(), a, "a * 1 must equal a");
    assert_eq!(one.clone() * a.clone(), a, "1 * a must equal a");

    // ── Invariant: zero annihilator ──────────────────────────────────────────
    // Multiplication by exact zero must yield a structurally definite zero.
    assert!(
        (a.clone() * zero.clone()).definitely_zero(),
        "a * 0 must be exactly zero"
    );

    // ── Invariant: zero_status and definitely_zero consistency ───────────────
    if a.zero_status() == ZeroKnowledge::NonZero {
        assert!(
            !a.definitely_zero(),
            "NonZero zero_status contradicts definitely_zero()"
        );
    }
    if a.definitely_zero() {
        assert_eq!(
            a.zero_status(),
            ZeroKnowledge::Zero,
            "definitely_zero() implies Zero status"
        );
    }

    // ── Invariant: Real::powi_i64(a, 1) is the identity ────────────────────────────────
    match Real::powi_i64(a.clone(), 1) {
        Ok(result) => assert_eq!(result, a, "Real::powi_i64(a, 1) must equal a"),
        Err(e) => panic!("Real::powi_i64(a, 1) must always succeed; got {e:?}"),
    }

    // ── Invariant: Real::powi_i64(a, 2) equals a * a ──────────────────────────────────
    // Both paths route through mul_refs on identical operands so results must
    // compare equal as Real values; this is not a serialized-bit comparison.
    let a_squared = a.clone() * a.clone();
    match Real::powi_i64(a.clone(), 2) {
        Ok(result) => assert_eq!(result, a_squared, "Real::powi_i64(a, 2) must equal a * a"),
        Err(e) => panic!("Real::powi_i64(a, 2) must always succeed; got {e:?}"),
    }

    // ── Invariant: Real::powi_i64(NonZero, 0) is the multiplicative identity ───────────
    if a.zero_status() == ZeroKnowledge::NonZero {
        match Real::powi_i64(a.clone(), 0) {
            Ok(result) => assert_eq!(result, one, "Real::powi_i64(NonZero, 0) must equal 1"),
            Err(e) => panic!("Real::powi_i64(NonZero, 0) must succeed; got {e:?}"),
        }
    }
}
