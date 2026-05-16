//! In-depth fuzz testing of Vector3 and Vector4 operations and algebraic laws.
//!
//! All arithmetic operators across every ownership variant are exercised,
//! including scalar-broadcast operators, abort-aware division and normalization,
//! and a suite of algebraic invariants that hold for interval arithmetic.
//!
//! Run with: `cargo fuzz run vector_ops` from the `fuzz/` directory.

#![no_main]

use std::sync::{Arc, atomic::AtomicBool};

use arbitrary::Arbitrary;
use hyperlattice::{Real, Vector3, Vector4, ZeroStatus};
use libfuzzer_sys::fuzz_target;

#[derive(Debug)]
struct Input {
    v3a: Vector3,
    v3b: Vector3,
    v4a: Vector4,
    v4b: Vector4,
}

impl<'a> Arbitrary<'a> for Input {
    fn arbitrary(u: &mut arbitrary::Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self {
            v3a: Arbitrary::arbitrary(u)?,
            v3b: Arbitrary::arbitrary(u)?,
            v4a: Arbitrary::arbitrary(u)?,
            v4b: Arbitrary::arbitrary(u)?,
        })
    }
}

fuzz_target!(|input: Input| {
    vector_fuzz(input);
});

fn vector_fuzz(input: Input) {
    let Input { v3a, v3b, v4a, v4b } = input;

    let signal = Arc::new(AtomicBool::new(false));

    // ── Vector3: no-panic — vector-vector arithmetic ─────────────────────────
    let _ = v3a.clone() + v3b.clone();
    let _ = v3a.clone() - v3b.clone();
    let _ = &v3a + &v3b;
    let _ = &v3a - &v3b;
    let _ = v3a.clone() + &v3b;
    let _ = v3a.clone() - &v3b;
    let _ = &v3a + v3b.clone();
    let _ = &v3a - v3b.clone();
    let _ = -v3a.clone();
    let _ = -&v3a;

    // ── Vector3: no-panic — scalar broadcast arithmetic ──────────────────────
    let _ = v3a.clone() * v3a[0].clone();
    let _ = v3a.clone() * &v3a[0];
    let _ = v3a.clone() + v3b[0].clone();
    let _ = v3a.clone() + &v3b[0];
    let _ = v3a.clone() - v3b[0].clone();
    let _ = v3a.clone() - &v3b[0];
    let _ = v3a.clone() / v3b[0].clone();
    let _ = v3a.clone() / &v3b[0];

    // ── Vector3: no-panic — geometric operations ─────────────────────────────
    let _ = v3a.dot(&v3b);
    let _ = v3a.shared_scale_view();
    let _ = v3b.shared_scale_view();
    let _ = v3a.dot_with_abort(&v3b, &signal);
    let _ = v3a.magnitude();
    let _ = v3a.magnitude_with_abort(&signal);
    let _ = v3a.normalize();
    let _ = v3a.normalize_checked();
    let _ = v3a.normalize_checked_with_abort(&signal);
    let _ = v3a.clone().div_scalar_checked(v3b[0].clone());
    let _ = v3a
        .clone()
        .div_scalar_checked_with_abort(v3b[0].clone(), &signal);

    // ── Vector4: no-panic — vector-vector arithmetic ─────────────────────────
    let _ = v4a.clone() + v4b.clone();
    let _ = v4a.clone() - v4b.clone();
    let _ = &v4a + &v4b;
    let _ = &v4a - &v4b;
    let _ = v4a.clone() + &v4b;
    let _ = v4a.clone() - &v4b;
    let _ = &v4a + v4b.clone();
    let _ = &v4a - v4b.clone();
    let _ = -v4a.clone();
    let _ = -&v4a;

    // ── Vector4: no-panic — scalar broadcast arithmetic ──────────────────────
    let _ = v4a.clone() * v4a[0].clone();
    let _ = v4a.clone() * &v4a[0];
    let _ = v4a.clone() + v4b[0].clone();
    let _ = v4a.clone() + &v4b[0];
    let _ = v4a.clone() - v4b[0].clone();
    let _ = v4a.clone() - &v4b[0];
    let _ = v4a.clone() / v4b[0].clone();
    let _ = v4a.clone() / &v4b[0];

    // ── Vector4: no-panic — geometric operations ─────────────────────────────
    let _ = v4a.dot(&v4b);
    let _ = v4a.shared_scale_view();
    let _ = v4b.shared_scale_view();
    let _ = v4a.dot_with_abort(&v4b, &signal);
    let _ = v4a.magnitude();
    let _ = v4a.magnitude_with_abort(&signal);
    let _ = v4a.normalize();
    let _ = v4a.normalize_checked();
    let _ = v4a.normalize_checked_with_abort(&signal);
    let _ = v4a.clone().div_scalar_checked(v4b[0].clone());
    let _ = v4a
        .clone()
        .div_scalar_checked_with_abort(v4b[0].clone(), &signal);

    // ── Vector3: algebraic invariants ────────────────────────────────────────

    // −(−v) is the exact identity for each component.
    assert_eq!(
        -(-v3a.clone()),
        v3a,
        "Vector3 double negation must be identity"
    );

    // Each component of v + (−v) must be within error bounds of zero.
    let zero3 = v3a.clone() + (-v3a.clone());
    for i in 0..3 {
        assert_ne!(
            zero3[i].zero_status(),
            ZeroStatus::NonZero,
            "Vector3 component {i} of v + (-v) must be within error bounds of zero"
        );
    }

    // Vector addition is commutative (scalar add is commutative at every component).
    assert_eq!(
        v3a.clone() + v3b.clone(),
        v3b.clone() + v3a.clone(),
        "Vector3 addition must be commutative"
    );

    // Dot product is symmetric.
    assert_eq!(
        v3a.dot(&v3b),
        v3b.dot(&v3a),
        "Vector3 dot product must be commutative"
    );

    // Real multiplicative identity: v * 1 == v component-wise.
    let one = Real::one();
    assert_eq!(
        v3a.clone() * one.clone(),
        v3a,
        "Vector3 * 1 must equal the vector"
    );

    // Real zero annihilator: every component of v * 0 is exactly zero.
    let scalar_zero = Real::zero();
    let scaled_zero3 = v3a.clone() * scalar_zero.clone();
    for i in 0..3 {
        assert!(
            scaled_zero3[i].definitely_zero(),
            "Vector3 component {i} of v * 0 must be exactly zero"
        );
    }

    // Owned + borrowed agrees with owned + owned (same internal path).
    assert_eq!(
        v3a.clone() + v3b.clone(),
        v3a.clone() + &v3b,
        "Vector3 + Vector3 must equal Vector3 + &Vector3"
    );

    // ── Vector4: algebraic invariants ────────────────────────────────────────

    assert_eq!(
        -(-v4a.clone()),
        v4a,
        "Vector4 double negation must be identity"
    );

    let zero4 = v4a.clone() + (-v4a.clone());
    for i in 0..4 {
        assert_ne!(
            zero4[i].zero_status(),
            ZeroStatus::NonZero,
            "Vector4 component {i} of v + (-v) must be within error bounds of zero"
        );
    }

    assert_eq!(
        v4a.clone() + v4b.clone(),
        v4b.clone() + v4a.clone(),
        "Vector4 addition must be commutative"
    );

    assert_eq!(
        v4a.dot(&v4b),
        v4b.dot(&v4a),
        "Vector4 dot product must be commutative"
    );

    assert_eq!(
        v4a.clone() * one.clone(),
        v4a,
        "Vector4 * 1 must equal the vector"
    );

    let scaled_zero4 = v4a.clone() * scalar_zero.clone();
    for i in 0..4 {
        assert!(
            scaled_zero4[i].definitely_zero(),
            "Vector4 component {i} of v * 0 must be exactly zero"
        );
    }

    assert_eq!(
        v4a.clone() + v4b.clone(),
        v4a.clone() + &v4b,
        "Vector4 + Vector4 must equal Vector4 + &Vector4"
    );
}
