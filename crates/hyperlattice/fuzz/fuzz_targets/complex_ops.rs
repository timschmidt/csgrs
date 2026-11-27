//! Fuzz selected Complex arithmetic and algebraic laws over dyadic coordinate seeds.
//!
//! The feature-gated aggregate generator supplies z then w. Calls four ownership
//! forms of complex arithmetic, owned-left scalar division, reciprocal/domain-checked
//! methods, conjugation/norms, powers at a fixed exponent list, and selected ^ forms.
//! Does not cover every complex API. Exact identities and ownership-result agreement
//! supplement calls whose Results are discarded; neither is an independent kernel.
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

use arbitrary::Arbitrary;
use hyperlattice::{Complex, ZeroKnowledge};
use libfuzzer_sys::fuzz_target;

/// Decoded controls for this target; field order determines input consumption and the generated fixtures.
#[derive(Debug)]
struct Input {
    /// First complex seed, with independently generated dyadic real/imaginary coordinates.
    z: Complex,
    /// Second complex seed, also used as a complex divisor and as the source of a real divisor.
    w: Complex,
}

impl<'a> Arbitrary<'a> for Input {
    /// Generate fields in declaration order from one cursor, propagating the first error.
    ///
    /// No custom size hint is supplied; current primitive decoding can zero-pad short
    /// input, including synthesizing all-zero operands from an empty byte slice.
    fn arbitrary(u: &mut arbitrary::Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self {
            z: Arbitrary::arbitrary(u)?,
            w: Arbitrary::arbitrary(u)?,
        })
    }
}

fuzz_target!(|input: Input| {
    complex_fuzz(input);
});

/// Run the selected complex operations and assertions using the two decoded seeds.
///
/// Norm/conjugate and ownership comparisons share the same scalar implementation.
/// Imaginary cancellation only needs status other than NonZero. The exponent-zero
/// identity is asserted only when not both components are known zero and powi returns
/// Ok; an error skips that assertion. This is not a completeness check for inversion.
fn complex_fuzz(input: Input) {
    let Input { z, w } = input;

    // ── No-panic: owned and borrowed arithmetic ──────────────────────────────
    let _ = z.clone() + w.clone();
    let _ = z.clone() - w.clone();
    let _ = z.clone() * w.clone();
    let _ = z.clone() / w.clone();
    let _ = &z + &w;
    let _ = &z - &w;
    let _ = &z * &w;
    let _ = &z / &w;
    let _ = z.clone() + &w;
    let _ = z.clone() - &w;
    let _ = z.clone() * &w;
    let _ = z.clone() / &w;
    let _ = &z + w.clone();
    let _ = &z - w.clone();
    let _ = &z * w.clone();
    let _ = &z / w.clone();
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

    // ── Invariant: cancellation components must not classify NonZero ────────
    let zero_candidate = z.clone() + (-z.clone());
    assert_ne!(
        zero_candidate.re.zero_status(),
        ZeroKnowledge::NonZero,
        "real part of z + (-z) must be within error bounds of zero"
    );
    assert_ne!(
        zero_candidate.im.zero_status(),
        ZeroKnowledge::NonZero,
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
    let complex_zero = Complex::zero();
    assert_eq!(z.clone() + complex_zero.clone(), z, "z + 0 must equal z");
    assert_eq!(complex_zero + z.clone(), z, "0 + z must equal z");

    // ── Invariant: multiplicative identity ───────────────────────────────────
    let complex_one = Complex::one();
    assert_eq!(z.clone() * complex_one.clone(), z, "z * 1 must equal z");
    assert_eq!(complex_one * z.clone(), z, "1 * z must equal z");

    // ── Invariant: z · conj(z) equals norm_squared ───────────────────────────
    // The real component is compared with the same scalar norm-squared arithmetic.
    let z_times_conj = z.clone() * z.clone().conjugate();
    assert_eq!(
        z_times_conj.re,
        z.norm_squared(),
        "real part of z·conj(z) must equal norm_squared"
    );

    // Imaginary cancellation is required only not to classify NonZero; Unknown passes.
    assert_ne!(
        z_times_conj.im.zero_status(),
        ZeroKnowledge::NonZero,
        "imaginary part of z·conj(z) must be within error bounds of zero"
    );

    // ── Invariant: powi(NonZero, 0) is the identity ──────────────────────────
    // powi returns Complex::one() directly for exponent 0 on non-zero inputs.
    let is_zero_re = z.re.zero_status() == ZeroKnowledge::Zero;
    let is_zero_im = z.im.zero_status() == ZeroKnowledge::Zero;
    if (!is_zero_re || !is_zero_im)
        && let Ok(result) = z.clone().powi(0)
    {
        assert_eq!(result, Complex::one(), "powi(non-zero z, 0) must equal 1");
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
    assert_eq!(
        z.clone() / w.clone(),
        z.clone() / &w,
        "Complex / Complex must equal Complex / &Complex"
    );
    assert_eq!(
        z.clone() / w.clone(),
        &z / w.clone(),
        "Complex / Complex must equal &Complex / Complex"
    );
    assert_eq!(
        z.clone() / w.clone(),
        &z / &w,
        "Complex / Complex must equal &Complex / &Complex"
    );

    // ── Invariant: zero complex number has zero norm ──────────────────────────
    // The norm-squared expression for exact complex zero must be definitely_zero.
    assert!(
        Complex::zero().norm_squared().definitely_zero(),
        "norm_squared of Complex::zero() must be exactly zero"
    );
}
