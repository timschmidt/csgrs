//! In-depth fuzz testing of Matrix3 and Matrix4 operations and algebraic laws.
//!
//! All arithmetic operators across every ownership variant, matrix-vector
//! products, `powi` / `powi_checked_with_abort`, division operators, and the
//! `^` operator are exercised. Algebraic invariants including identity-matrix
//! laws, determinant properties, and powi structural laws are checked.
//!
//! Run with: `cargo fuzz run matrix_ops` from the `fuzz/` directory.

#![no_main]

use std::sync::{Arc, atomic::AtomicBool};

use arbitrary::Arbitrary;
use libfuzzer_sys::fuzz_target;
use realistic_blas::{ApproxBackend, HyperrealBackend, Matrix3, Matrix4, Scalar, Vector3, Vector4, ZeroStatus};

#[derive(Debug)]
struct Input<Backend: realistic_blas::Backend> {
    m3a: Matrix3<Backend>,
    m3b: Matrix3<Backend>,
    m4a: Matrix4<Backend>,
    m4b: Matrix4<Backend>,
    v3: Vector3<Backend>,
    v4: Vector4<Backend>,
}

impl<'a, Backend: realistic_blas::Backend> Arbitrary<'a> for Input<Backend>
where Matrix3<Backend>: Arbitrary<'a>, Matrix4<Backend>: Arbitrary<'a>,
    Vector3<Backend>: Arbitrary<'a>, Vector4<Backend>: Arbitrary<'a>
{
    fn arbitrary(u: &mut arbitrary::Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self {
            m3a: Arbitrary::arbitrary(u)?,
            m3b: Arbitrary::arbitrary(u)?,
            m4a: Arbitrary::arbitrary(u)?,
            m4b: Arbitrary::arbitrary(u)?,
            v3: Arbitrary::arbitrary(u)?,
            v4: Arbitrary::arbitrary(u)?,
        })
    }
}

fuzz_target!(|input: (Input<ApproxBackend>, Input<HyperrealBackend>)| {
    let (approx_input, hyperreal_input) = input;
    matrix_fuzz(approx_input);
    matrix_fuzz(hyperreal_input);
});

fn matrix_fuzz<Backend: realistic_blas::Backend>(input: Input<Backend>) {
    let Input { m3a, m3b, m4a, m4b, v3, v4 } = input;

    let signal = Arc::new(AtomicBool::new(false));

    // ── Matrix3: no-panic — matrix-matrix arithmetic ─────────────────────────
    let _ = m3a.clone() + m3b.clone();
    let _ = m3a.clone() - m3b.clone();
    let _ = m3a.clone() * m3b.clone();
    let _ = &m3a + &m3b;
    let _ = &m3a - &m3b;
    let _ = &m3a * &m3b;
    let _ = m3a.clone() + &m3b;
    let _ = m3a.clone() - &m3b;
    let _ = m3a.clone() * &m3b;
    let _ = &m3a + m3b.clone();
    let _ = &m3a - m3b.clone();
    let _ = &m3a * m3b.clone();
    let _ = -m3a.clone();
    let _ = -&m3a;

    // ── Matrix3: no-panic — scalar broadcast arithmetic ──────────────────────
    let _ = m3a.clone() * m3b[0][0].clone();
    let _ = m3a.clone() * &m3b[0][0];
    let _ = m3a.clone() + m3b[0][0].clone();
    let _ = m3a.clone() + &m3b[0][0];
    let _ = m3a.clone() - m3b[0][0].clone();
    let _ = m3a.clone() - &m3b[0][0];
    let _ = m3a.clone() / m3b[0][0].clone();
    let _ = m3a.clone() / &m3b[0][0];

    // ── Matrix3: no-panic — matrix-vector products ───────────────────────────
    let _ = m3a.clone() * v3.clone();
    let _ = m3a.clone() * &v3;
    let _ = &m3a * v3.clone();
    let _ = &m3a * &v3;

    // ── Matrix3: no-panic — structural operations ────────────────────────────
    let _ = m3a.clone().transpose();
    let _ = m3a.clone().determinant();
    let _ = m3a.clone().inverse();
    let _ = m3a.clone().inverse_checked();
    let _ = m3a.clone().inverse_checked_with_abort(&signal);

    // ── Matrix3: no-panic — powi and division ────────────────────────────────
    for e in [-3_i32, -2, -1, 0, 1, 2, 3, 4, 5] {
        let _ = m3a.clone().powi(e);
        let _ = m3a.clone().powi_checked(e);
        let _ = m3a.clone().powi_checked_with_abort(e, &signal);
    }
    let _ = m3a.clone() / m3b.clone();
    let _ = m3a.clone() / &m3b;
    let _ = &m3a / m3b.clone();
    let _ = m3a.clone().div_matrix_checked(m3b.clone());
    let _ = m3a.clone().div_matrix_checked_with_abort(m3b.clone(), &signal);
    let _ = m3a.clone().div_scalar_checked(m3b[0][0].clone());
    let _ = m3a.clone().div_scalar_checked_with_abort(m3b[0][0].clone(), &signal);

    // ── Matrix3: no-panic — ^ (BitXor) operator ──────────────────────────────
    let _ = m3a.clone() ^ 0_i32;
    let _ = m3a.clone() ^ 1_i32;
    let _ = m3a.clone() ^ 2_i32;
    let _ = m3a.clone() ^ -1_i32;

    // ── Matrix4: no-panic — matrix-matrix arithmetic ─────────────────────────
    let _ = m4a.clone() + m4b.clone();
    let _ = m4a.clone() - m4b.clone();
    let _ = m4a.clone() * m4b.clone();
    let _ = &m4a + &m4b;
    let _ = &m4a - &m4b;
    let _ = &m4a * &m4b;
    let _ = m4a.clone() + &m4b;
    let _ = m4a.clone() - &m4b;
    let _ = m4a.clone() * &m4b;
    let _ = &m4a + m4b.clone();
    let _ = &m4a - m4b.clone();
    let _ = &m4a * m4b.clone();
    let _ = -m4a.clone();
    let _ = -&m4a;

    // ── Matrix4: no-panic — scalar broadcast arithmetic ──────────────────────
    let _ = m4a.clone() * m4b[0][0].clone();
    let _ = m4a.clone() * &m4b[0][0];
    let _ = m4a.clone() + m4b[0][0].clone();
    let _ = m4a.clone() + &m4b[0][0];
    let _ = m4a.clone() - m4b[0][0].clone();
    let _ = m4a.clone() - &m4b[0][0];
    let _ = m4a.clone() / m4b[0][0].clone();
    let _ = m4a.clone() / &m4b[0][0];

    // ── Matrix4: no-panic — matrix-vector products ───────────────────────────
    let _ = m4a.clone() * v4.clone();
    let _ = m4a.clone() * &v4;
    let _ = &m4a * v4.clone();
    let _ = &m4a * &v4;

    // ── Matrix4: no-panic — structural operations ────────────────────────────
    let _ = m4a.clone().transpose();
    let _ = m4a.clone().determinant();
    let _ = m4a.clone().inverse();
    let _ = m4a.clone().inverse_checked();
    let _ = m4a.clone().inverse_checked_with_abort(&signal);

    // ── Matrix4: no-panic — powi and division ────────────────────────────────
    for e in [-3_i32, -2, -1, 0, 1, 2, 3, 4, 5] {
        let _ = m4a.clone().powi(e);
        let _ = m4a.clone().powi_checked(e);
        let _ = m4a.clone().powi_checked_with_abort(e, &signal);
    }
    let _ = m4a.clone() / m4b.clone();
    let _ = m4a.clone() / &m4b;
    let _ = &m4a / m4b.clone();
    let _ = m4a.clone().div_matrix_checked(m4b.clone());
    let _ = m4a.clone().div_matrix_checked_with_abort(m4b.clone(), &signal);
    let _ = m4a.clone().div_scalar_checked(m4b[0][0].clone());
    let _ = m4a.clone().div_scalar_checked_with_abort(m4b[0][0].clone(), &signal);

    // ── Matrix4: no-panic — ^ (BitXor) operator ──────────────────────────────
    let _ = m4a.clone() ^ 0_i32;
    let _ = m4a.clone() ^ 1_i32;
    let _ = m4a.clone() ^ 2_i32;
    let _ = m4a.clone() ^ -1_i32;

    // ── Matrix3: algebraic invariants ────────────────────────────────────────

    // −(−M) is the exact identity for every entry.
    assert_eq!(-(-m3a.clone()), m3a, "Matrix3 double negation must be identity");

    // Transpose is an involution: (M^T)^T == M exactly (clone then rearrange).
    assert_eq!(
        m3a.clone().transpose().transpose(),
        m3a,
        "Matrix3 transpose must be an involution"
    );

    // M + (−M): every entry must be within error bounds of zero.
    let zero3 = m3a.clone() + (-m3a.clone());
    for row in 0..3 {
        for col in 0..3 {
            assert_ne!(
                zero3[row][col].zero_status(),
                ZeroStatus::NonZero,
                "Matrix3 entry [{row}][{col}] of M + (-M) must be within error bounds of zero"
            );
        }
    }

    // Addition is commutative (element-wise commutativity).
    assert_eq!(
        m3a.clone() + m3b.clone(),
        m3b.clone() + m3a.clone(),
        "Matrix3 addition must be commutative"
    );

    // Identity multiplication: I·M == M and M·I == M.
    let i3 = Matrix3::<Backend>::identity();
    assert_eq!(i3.clone() * m3a.clone(), m3a, "I · M must equal M for Matrix3");
    assert_eq!(m3a.clone() * i3.clone(), m3a, "M · I must equal M for Matrix3");

    // powi(M, 0) always returns the identity, regardless of M.
    match m3a.clone().powi(0) {
        Ok(result) => assert_eq!(result, i3.clone(), "Matrix3 powi(M, 0) must equal identity"),
        Err(e) => panic!("Matrix3 powi(M, 0) must succeed; got {e:?}"),
    }

    // powi(M, 1) returns M unchanged.
    match m3a.clone().powi(1) {
        Ok(result) => assert_eq!(result, m3a, "Matrix3 powi(M, 1) must equal M"),
        Err(e) => panic!("Matrix3 powi(M, 1) must succeed; got {e:?}"),
    }

    // powi(M, 2) == M * M (same multiply_arrays call with identical inputs).
    let m3a_sq = m3a.clone() * m3a.clone();
    match m3a.clone().powi(2) {
        Ok(result) => assert_eq!(result, m3a_sq, "Matrix3 powi(M, 2) must equal M * M"),
        Err(e) => panic!("Matrix3 powi(M, 2) must succeed; got {e:?}"),
    }

    // determinant(I) == 1: all integer arithmetic on exact scalars, so the
    // computed value is 1.0 and PartialEq (interval) trivially holds.
    let one = Scalar::<Backend>::one();
    assert_eq!(
        i3.clone().determinant(),
        one.clone(),
        "det(I₃) must equal 1"
    );

    // M · 0_vector == 0_vector: every dot product with a zero right-hand side
    // produces value=0.0, epsilon=0.0 exactly (all cross terms vanish).
    let zero_v3 = Vector3::<Backend>::zero();
    let mv_zero3 = m3a.clone() * zero_v3.clone();
    for i in 0..3 {
        assert!(
            mv_zero3[i].definitely_zero(),
            "Matrix3 · 0 component {i} must be exactly zero"
        );
    }

    // I · v == v for any vector (dot of a basis row with v returns v's component).
    assert_eq!(i3.clone() * v3.clone(), v3, "I₃ · v must equal v for Vector3");

    // Owned + borrowed agrees with owned + owned.
    assert_eq!(
        m3a.clone() + m3b.clone(),
        m3a.clone() + &m3b,
        "Matrix3 + Matrix3 must equal Matrix3 + &Matrix3"
    );

    // ── Matrix4: algebraic invariants ────────────────────────────────────────

    assert_eq!(-(-m4a.clone()), m4a, "Matrix4 double negation must be identity");

    assert_eq!(
        m4a.clone().transpose().transpose(),
        m4a,
        "Matrix4 transpose must be an involution"
    );

    let zero4 = m4a.clone() + (-m4a.clone());
    for row in 0..4 {
        for col in 0..4 {
            assert_ne!(
                zero4[row][col].zero_status(),
                ZeroStatus::NonZero,
                "Matrix4 entry [{row}][{col}] of M + (-M) must be within error bounds of zero"
            );
        }
    }

    assert_eq!(
        m4a.clone() + m4b.clone(),
        m4b.clone() + m4a.clone(),
        "Matrix4 addition must be commutative"
    );

    let i4 = Matrix4::<Backend>::identity();
    assert_eq!(i4.clone() * m4a.clone(), m4a, "I · M must equal M for Matrix4");
    assert_eq!(m4a.clone() * i4.clone(), m4a, "M · I must equal M for Matrix4");

    match m4a.clone().powi(0) {
        Ok(result) => assert_eq!(result, i4.clone(), "Matrix4 powi(M, 0) must equal identity"),
        Err(e) => panic!("Matrix4 powi(M, 0) must succeed; got {e:?}"),
    }

    match m4a.clone().powi(1) {
        Ok(result) => assert_eq!(result, m4a, "Matrix4 powi(M, 1) must equal M"),
        Err(e) => panic!("Matrix4 powi(M, 1) must succeed; got {e:?}"),
    }

    let m4a_sq = m4a.clone() * m4a.clone();
    match m4a.clone().powi(2) {
        Ok(result) => assert_eq!(result, m4a_sq, "Matrix4 powi(M, 2) must equal M * M"),
        Err(e) => panic!("Matrix4 powi(M, 2) must succeed; got {e:?}"),
    }

    assert_eq!(i4.clone().determinant(), one, "det(I₄) must equal 1");

    let zero_v4 = Vector4::<Backend>::zero();
    let mv_zero4 = m4a.clone() * zero_v4.clone();
    for i in 0..4 {
        assert!(
            mv_zero4[i].definitely_zero(),
            "Matrix4 · 0 component {i} must be exactly zero"
        );
    }

    assert_eq!(i4 * v4.clone(), v4, "I₄ · v must equal v for Vector4");

    assert_eq!(
        m4a.clone() + m4b.clone(),
        m4a.clone() + &m4b,
        "Matrix4 + Matrix4 must equal Matrix4 + &Matrix4"
    );
}
