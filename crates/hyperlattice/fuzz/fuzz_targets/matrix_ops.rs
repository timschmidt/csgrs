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
use hyperlattice::{
    Matrix3, Matrix3StructuralFacts, Matrix3TransformKind, Matrix4, Matrix4StructuralFacts,
    Matrix4TransformKind, MatrixPreparedCacheState, Real, Vector3, Vector4, ZeroStatus,
};
use libfuzzer_sys::fuzz_target;

#[derive(Debug)]
struct Input {
    m3a: Matrix3,
    m3b: Matrix3,
    m4a: Matrix4,
    m4b: Matrix4,
    v3: Vector3,
    v4: Vector4,
}

impl<'a> Arbitrary<'a> for Input {
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
    let mut prepared3 = m3b.prepare();
    let facts3 = prepared3.structural_facts();
    assert_matrix3_fact_helpers(facts3);
    assert_eq!(
        prepared3.determinant_schedule_hint(),
        facts3.determinant_schedule_hint(),
        "PreparedMatrix3 must reuse the same determinant schedule hint as its cached facts"
    );
    let _ = prepared3.exact_facts();
    let _ = prepared3.right_divisor().determinant_schedule_hint();
    assert_prepared_matrix3_cache_progress(&mut prepared3, "generated Matrix3");
    assert_eq!(
        prepared3.transform_vector(&v3),
        m3b.transform_vec3_handle().transform_vector(&v3),
        "PreparedMatrix3 transform_vector must match the retained transform handle"
    );
    assert_eq!(
        prepared3.transform_vector_batch(std::slice::from_ref(&v3)),
        m3b.transform_vec3_handle()
            .transform_vector_batch(std::slice::from_ref(&v3)),
        "PreparedMatrix3 transform_vector_batch must match the retained transform handle"
    );
    let _ = prepared3.inverse();
    let _ = prepared3.inverse_checked();
    let _ = prepared3.inverse_checked_with_abort(&signal);
    let _ = prepared3.divide_left(m3a.clone());
    let _ = prepared3.divide_left_checked(m3a.clone());
    let _ = prepared3.divide_left_checked_with_abort(m3a.clone(), &signal);

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
    let mut prepared4 = m4b.prepare();
    let facts4 = prepared4.structural_facts();
    assert_matrix4_fact_helpers(facts4);
    assert_eq!(
        prepared4.determinant_schedule_hint(),
        facts4.determinant_schedule_hint(),
        "PreparedMatrix4 must reuse the same determinant schedule hint as its cached facts"
    );
    let _ = prepared4.exact_facts();
    let _ = prepared4.right_divisor().determinant_schedule_hint();
    assert_prepared_matrix4_cache_progress(&mut prepared4, "generated Matrix4");
    assert_eq!(
        prepared4.transform_vector(&v4),
        m4b.transform_vec4_handle().transform_vector(&v4),
        "PreparedMatrix4 transform_vector must match the retained transform handle"
    );
    assert_eq!(
        prepared4.transform_vector_batch(std::slice::from_ref(&v4)),
        m4b.transform_vec4_handle()
            .transform_vector_batch(std::slice::from_ref(&v4)),
        "PreparedMatrix4 transform_vector_batch must match the retained transform handle"
    );
    assert_eq!(
        prepared4.transform_direction_vector(&v4),
        m4b.transform_vec4_handle().transform_direction_vector(&v4),
        "PreparedMatrix4 transform_direction_vector must match the retained transform handle"
    );
    assert_eq!(
        prepared4.transform_direction_batch(std::slice::from_ref(&v4)),
        m4b.transform_vec4_handle()
            .transform_direction_batch(std::slice::from_ref(&v4)),
        "PreparedMatrix4 transform_direction_batch must match the retained transform handle"
    );
    assert_eq!(
        prepared4.transform_point_vector(&v4),
        m4b.transform_vec4_handle().transform_point_vector(&v4),
        "PreparedMatrix4 transform_point_vector must match the retained transform handle"
    );
    assert_eq!(
        prepared4.transform_point_batch(std::slice::from_ref(&v4)),
        m4b.transform_vec4_handle()
            .transform_point_batch(std::slice::from_ref(&v4)),
        "PreparedMatrix4 transform_point_batch must match the retained transform handle"
    );
    let _ = prepared4.inverse();
    let _ = prepared4.inverse_checked();
    let _ = prepared4.inverse_checked_with_abort(&signal);
    let _ = prepared4.divide_left(m4a.clone());
    let _ = prepared4.divide_exact_rational_left(m4a.clone());
    let _ = prepared4.divide_left_checked(m4a.clone());
    let _ = prepared4.divide_left_checked_with_abort(m4a.clone(), &signal);

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

    // determinant(I) == 1: all integer arithmetic on exact scalars, so the
    // computed value is 1.0 and PartialEq (interval) trivially holds.
    let one = Real::one();
    assert_eq!(
        i3.clone().determinant(),
        one.clone(),
        "det(I₃) must equal 1"
    );
    let mut prepared_i3 = i3.prepare();
    assert_prepared_matrix3_cache_progress(&mut prepared_i3, "identity Matrix3");
    assert!(
        prepared_i3.inverse().is_ok(),
        "prepared identity Matrix3 inverse must succeed"
    );
    assert!(
        prepared_i3.cache_state().has_shared_adjugate_path(),
        "prepared identity Matrix3 inverse must warm the shared adjugate path"
    );
    assert!(
        prepared_i3.cache_state().inverse,
        "prepared identity Matrix3 inverse must retain the scaled inverse cache"
    );

    // M · 0_vector == 0_vector: every dot product with a zero right-hand side
    // produces value=0.0, epsilon=0.0 exactly (all cross terms vanish).
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
    let mut prepared_i4 = i4.prepare();
    assert_prepared_matrix4_cache_progress(&mut prepared_i4, "identity Matrix4");
    assert!(
        prepared_i4.inverse().is_ok(),
        "prepared identity Matrix4 inverse must succeed"
    );
    let identity4_state = prepared_i4.cache_state();
    assert!(
        identity4_state.has_shared_adjugate_path(),
        "prepared identity Matrix4 inverse must warm the shared adjugate path"
    );
    assert!(
        identity4_state.minor_factors,
        "prepared identity Matrix4 inverse must retain the shared six-minor factor cache"
    );
    assert!(
        identity4_state.inverse,
        "prepared identity Matrix4 inverse must retain the scaled inverse cache"
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

fn assert_prepared_matrix3_cache_progress(
    prepared: &mut hyperlattice::PreparedMatrix3<'_>,
    label: &str,
) {
    let cold = prepared.cache_state();
    assert!(
        cold.is_cold(),
        "{label}: freshly prepared Matrix3 handles must start with cold determinant/adjugate caches"
    );

    let _ = prepared.determinant();
    let determinant_state = prepared.cache_state();
    assert!(
        determinant_state.determinant,
        "{label}: determinant() must retain the exact determinant cache"
    );
    assert!(
        determinant_state.is_warm(),
        "{label}: determinant() must make the prepared Matrix3 cache observably warm"
    );
    assert!(
        !determinant_state.minor_factors,
        "{label}: Matrix3 must not report the Matrix4-only minor-factor cache"
    );
    assert_cache_state_monotonic(cold, determinant_state, label);
}

fn assert_prepared_matrix4_cache_progress(
    prepared: &mut hyperlattice::PreparedMatrix4<'_>,
    label: &str,
) {
    let cold = prepared.cache_state();
    assert!(
        cold.is_cold(),
        "{label}: freshly prepared Matrix4 handles must start with cold determinant/factor/adjugate caches"
    );

    let _ = prepared.determinant();
    let determinant_state = prepared.cache_state();
    assert!(
        determinant_state.determinant,
        "{label}: determinant() must retain the exact determinant cache"
    );
    assert!(
        determinant_state.minor_factors,
        "{label}: Matrix4 determinant() must retain the shared six-minor factor cache"
    );
    assert!(
        determinant_state.is_warm(),
        "{label}: determinant() must make the prepared Matrix4 cache observably warm"
    );
    assert_cache_state_monotonic(cold, determinant_state, label);
}

fn assert_cache_state_monotonic(
    before: MatrixPreparedCacheState,
    after: MatrixPreparedCacheState,
    label: &str,
) {
    // Yap's exact-geometric-computation model treats prepared objects as the
    // validity boundary for derived algebraic facts. A later kernel may add
    // determinant, reciprocal, adjugate, factor, or inverse facts, but it must
    // not silently discard a fact that was already valid for the same borrowed
    // immutable matrix; see Yap, "Towards Exact Geometric Computation,"
    // Computational Geometry 7.1-2 (1997).
    assert!(
        !before.determinant || after.determinant,
        "{label}: determinant cache state must be monotonic"
    );
    assert!(
        !before.reciprocal_determinant || after.reciprocal_determinant,
        "{label}: reciprocal determinant cache state must be monotonic"
    );
    assert!(
        !before.minor_factors || after.minor_factors,
        "{label}: minor-factor cache state must be monotonic"
    );
    assert!(
        !before.adjugate || after.adjugate,
        "{label}: adjugate cache state must be monotonic"
    );
    assert!(
        !before.inverse || after.inverse,
        "{label}: inverse cache state must be monotonic"
    );
}

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
