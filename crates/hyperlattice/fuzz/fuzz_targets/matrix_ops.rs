//! Fuzz selected Matrix3/Matrix4 operations and facts over unconstrained dyadic seeds.
//!
//! Exercises matrix arithmetic ownership forms, owned-left scalar broadcast, matrix-
//! vector products, determinant/inverse/power/division, and one-element batch
//! transforms. Abort-aware calls use a false flag that is never toggled. Singular,
//! non-affine, and otherwise unstructured inputs are admitted. Fact helpers check
//! internal consistency, not an independent classification oracle; inverse success
//! and inverse-product recovery are not generally asserted.
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

use std::sync::{Arc, atomic::AtomicBool};

use arbitrary::Arbitrary;
use hyperlattice::{
    Matrix3, Matrix3StructuralFacts, Matrix3TransformKind, Matrix4, Matrix4StructuralFacts,
    Matrix4TransformKind, Real, Vector3, Vector4, ZeroKnowledge,
};
use libfuzzer_sys::fuzz_target;

/// Decoded controls for this target; field order determines input consumption and the generated fixtures.
#[derive(Debug)]
struct Input {
    /// Unconstrained m3 matrix seed; no invertibility or affine constraint.
    m3a: Matrix3,
    /// Unconstrained m3 matrix seed; no invertibility or affine constraint.
    m3b: Matrix3,
    /// Unconstrained m4 matrix seed; no invertibility or affine constraint.
    m4a: Matrix4,
    /// Unconstrained m4 matrix seed; no invertibility or affine constraint.
    m4b: Matrix4,
    /// Generated vector for immediate multiplication/batch comparisons.
    v3: Vector3,
    /// Generated vector for immediate multiplication/batch comparisons.
    v4: Vector4,
}

impl<'a> Arbitrary<'a> for Input {
    /// Generate fields in declaration order from one cursor, propagating the first error.
    ///
    /// No custom size hint is supplied; current primitive decoding can zero-pad short
    /// input, including synthesizing all-zero operands from an empty byte slice.
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

fuzz_target!(|input: Input| {
    matrix_fuzz(input);
});

/// Check generated fact consistency, execute the listed matrix calls, then selected exact identities.
///
/// Powers 0/1/2 must succeed and match identity/input/product. Immediate one-element
/// batch transforms match multiplication. Additive cancellation accepts Unknown;
/// zero-vector products must be definitely_zero. General inverse/division Results
/// are discarded, and the abort flag remains false throughout.
fn matrix_fuzz(input: Input) {
    let Input {
        m3a,
        m3b,
        m4a,
        m4b,
        v3,
        v4,
    } = input;

    let signal = Arc::new(AtomicBool::new(false));

    assert_matrix3_fact_helpers(m3a.structural_facts());
    assert_matrix4_fact_helpers(m4a.structural_facts());

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
    let _ = m3a
        .clone()
        .div_matrix_checked_with_abort(m3b.clone(), &signal);
    let _ = m3a.clone().div_scalar_checked(m3b[0][0].clone());
    let _ = m3a
        .clone()
        .div_scalar_checked_with_abort(m3b[0][0].clone(), &signal);
    assert_eq!(
        m3b.transform_vec3_batch(std::slice::from_ref(&v3)),
        vec![m3b.clone() * v3.clone()],
        "Matrix3 immediate batch transform must match multiplication"
    );

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
    let _ = m4a
        .clone()
        .div_matrix_checked_with_abort(m4b.clone(), &signal);
    let _ = m4a.clone().div_scalar_checked(m4b[0][0].clone());
    let _ = m4a
        .clone()
        .div_scalar_checked_with_abort(m4b[0][0].clone(), &signal);
    assert_eq!(
        m4b.transform_vec4_batch(std::slice::from_ref(&v4)),
        vec![m4b.clone() * v4.clone()],
        "Matrix4 immediate batch transform must match multiplication"
    );

    // ── Matrix4: no-panic — ^ (BitXor) operator ──────────────────────────────
    let _ = m4a.clone() ^ 0_i32;
    let _ = m4a.clone() ^ 1_i32;
    let _ = m4a.clone() ^ 2_i32;
    let _ = m4a.clone() ^ -1_i32;

    // ── Matrix3: algebraic invariants ────────────────────────────────────────

    // −(−M) is the exact identity for every entry.
    assert_eq!(
        -(-m3a.clone()),
        m3a,
        "Matrix3 double negation must be identity"
    );

    // Transpose is an involution: (M^T)^T == M exactly (clone then rearrange).
    assert_eq!(
        m3a.clone().transpose().transpose(),
        m3a,
        "Matrix3 transpose must be an involution"
    );

    // Each cancellation entry must not classify NonZero; Unknown is accepted.
    let zero3 = m3a.clone() + (-m3a.clone());
    for row in 0..3 {
        for col in 0..3 {
            assert_ne!(
                zero3[row][col].zero_status(),
                ZeroKnowledge::NonZero,
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
    let i3 = Matrix3::identity();
    assert_eq!(
        i3.clone() * m3a.clone(),
        m3a,
        "I · M must equal M for Matrix3"
    );
    assert_eq!(
        m3a.clone() * i3.clone(),
        m3a,
        "M · I must equal M for Matrix3"
    );

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

    // Compare the exact identity determinant with the scalar multiplicative identity.
    let one = Real::one();
    assert_eq!(
        i3.clone().determinant(),
        one.clone(),
        "det(I₃) must equal 1"
    );

    // Every component of M times the exact zero vector must be definitely_zero.
    let zero_v3 = Vector3::zero();
    let mv_zero3 = m3a.clone() * zero_v3.clone();
    for i in 0..3 {
        assert!(
            mv_zero3[i].definitely_zero(),
            "Matrix3 · 0 component {i} must be exactly zero"
        );
    }

    // I · v == v for any vector (dot of a basis row with v returns v's component).
    assert_eq!(
        i3.clone() * v3.clone(),
        v3,
        "I₃ · v must equal v for Vector3"
    );

    // Owned + borrowed agrees with owned + owned.
    assert_eq!(
        m3a.clone() + m3b.clone(),
        m3a.clone() + &m3b,
        "Matrix3 + Matrix3 must equal Matrix3 + &Matrix3"
    );

    // ── Matrix4: algebraic invariants ────────────────────────────────────────

    assert_eq!(
        -(-m4a.clone()),
        m4a,
        "Matrix4 double negation must be identity"
    );

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
                ZeroKnowledge::NonZero,
                "Matrix4 entry [{row}][{col}] of M + (-M) must be within error bounds of zero"
            );
        }
    }

    assert_eq!(
        m4a.clone() + m4b.clone(),
        m4b.clone() + m4a.clone(),
        "Matrix4 addition must be commutative"
    );

    let i4 = Matrix4::identity();
    assert_eq!(
        i4.clone() * m4a.clone(),
        m4a,
        "I · M must equal M for Matrix4"
    );
    assert_eq!(
        m4a.clone() * i4.clone(),
        m4a,
        "M · I must equal M for Matrix4"
    );

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

    assert_eq!(
        i4.clone().determinant(),
        one.clone(),
        "det(I₄) must equal 1"
    );

    let zero_v4 = Vector4::zero();
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

/// Check 3D fact accessors agree internally on zero rows/columns, schedule fallback, and transform flags.
///
/// The fallback equivalence does not independently prove shape-driven and
/// exact-rational-driven categories disjoint. Row/column index 3 must return None.
/// Transform assertions test implications from the returned tag, not geometric
/// classification against original matrix coefficients.
fn assert_matrix3_fact_helpers(facts: Matrix3StructuralFacts) {
    let hint = facts.determinant_schedule_hint();
    assert_eq!(
        hint.requires_generic_real_fallback(),
        !hint.is_shape_driven() && !hint.is_exact_rational_driven(),
        "Matrix3 determinant schedule categories must stay disjoint and exhaustive"
    );

    let mut any_zero_row = false;
    let mut any_zero_column = false;
    for row in 0..3 {
        let count = facts.row_known_zero_count(row).unwrap();
        let zero = facts.row_is_known_zero(row).unwrap();
        assert_eq!(zero, count == 3);
        any_zero_row |= zero;
    }
    for column in 0..3 {
        let count = facts.column_known_zero_count(column).unwrap();
        let zero = facts.column_is_known_zero(column).unwrap();
        assert_eq!(zero, count == 3);
        any_zero_column |= zero;
    }
    assert_eq!(facts.has_known_zero_row(), any_zero_row);
    assert_eq!(facts.has_known_zero_column(), any_zero_column);
    assert_eq!(facts.has_known_zero_lane(), any_zero_row || any_zero_column);
    assert_eq!(facts.row_is_known_zero(3), None);
    assert_eq!(facts.column_is_known_zero(3), None);

    match facts.transform_kind {
        Matrix3TransformKind::Identity => assert!(facts.is_identity),
        Matrix3TransformKind::AffineTranslation => assert!(facts.is_affine_translation),
        Matrix3TransformKind::AffineDiagonalLinear => assert!(facts.is_affine),
        Matrix3TransformKind::Affine => assert!(facts.is_affine),
        Matrix3TransformKind::Projective => assert!(!facts.is_affine),
    }
}

/// Check 4D zero-row/column accessors, schedule fallback, and transform-tag implications.
///
/// Index 4 must return None. Signed-permutation and affine-diagonal tags imply their
/// stored helper flags; the original coefficients are not independently checked.
/// The fallback equivalence alone does not prove the other schedule categories disjoint.
fn assert_matrix4_fact_helpers(facts: Matrix4StructuralFacts) {
    let hint = facts.determinant_schedule_hint();
    assert_eq!(
        hint.requires_generic_real_fallback(),
        !hint.is_shape_driven() && !hint.is_exact_rational_driven(),
        "Matrix4 determinant schedule categories must stay disjoint and exhaustive"
    );

    let mut any_zero_row = false;
    let mut any_zero_column = false;
    for row in 0..4 {
        let count = facts.row_known_zero_count(row).unwrap();
        let zero = facts.row_is_known_zero(row).unwrap();
        assert_eq!(zero, count == 4);
        any_zero_row |= zero;
    }
    for column in 0..4 {
        let count = facts.column_known_zero_count(column).unwrap();
        let zero = facts.column_is_known_zero(column).unwrap();
        assert_eq!(zero, count == 4);
        any_zero_column |= zero;
    }
    assert_eq!(facts.has_known_zero_row(), any_zero_row);
    assert_eq!(facts.has_known_zero_column(), any_zero_column);
    assert_eq!(facts.has_known_zero_lane(), any_zero_row || any_zero_column);
    assert_eq!(facts.row_is_known_zero(4), None);
    assert_eq!(facts.column_is_known_zero(4), None);

    match facts.transform_kind {
        Matrix4TransformKind::Identity => assert!(facts.is_identity),
        Matrix4TransformKind::SignedPermutation => assert!(facts.is_signed_permutation()),
        Matrix4TransformKind::AffineTranslation => assert!(facts.is_affine_translation),
        Matrix4TransformKind::AffineDiagonalLinear => {
            assert!(facts.is_affine);
            assert!(facts.linear_is_diagonal);
        }
        Matrix4TransformKind::Affine => assert!(facts.is_affine),
        Matrix4TransformKind::Projective => {
            assert!(!facts.is_affine);
            assert!(!facts.is_signed_permutation());
        }
    }
}
