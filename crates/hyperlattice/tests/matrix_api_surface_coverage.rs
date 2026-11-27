//! Matrix API exercise coverage over fixed rational and symbolic fixtures.
//!
//! Combines successful calls, selected exact identities and specific typed-error
//! assertions. A discarded Result does not establish success; a discarded value
//! only exercises construction. The suite does not assert every result, dispatch
//! route, formatting string or cancellation outcome, and its name is not a claim
//! of exhaustive mathematical support. Independent value/representation regressions
//! also live in the matrix integration tests.
//!
//! Private helpers retain their exact integer, rational or π-dependent inputs.
//! The shared unknown_zero fixture is a mathematically nonzero sine difference with
//! initially unavailable cheap zero knowledge, not an authored exact zero.

/// Shared exact literal, fresh abort-flag and unknown-status scalar fixtures.
mod common;

use std::sync::atomic::Ordering;

use common::{abort_signal, frac, r, unknown_zero};
use hyperlattice::{
    Matrix3, Matrix4, MatrixDeterminantScheduleHint, Point3, Problem, Real, SignedAxis4, Vector3,
    Vector4,
};

/// Builds the fully nonzero symmetric integer fixture `diag(1,2,3) + 1*1ᵀ`.
///
/// Here `1` is the three-entry all-ones column vector.
/// Its determinant is 17. It supplies a nonsingular nonaffine 3×3 operand for
/// arithmetic, right-division and structural-fact API exercises.
fn dense3() -> Matrix3 {
    Matrix3::new([[r(2), r(1), r(1)], [r(1), r(3), r(1)], [r(1), r(1), r(4)]])
}

/// Builds the fully nonzero symmetric integer fixture `diag(4,5,6,7) + 1*1ᵀ`.
///
/// Here `1` is the four-entry all-ones column vector.
/// Its determinant is 1478. The nonaffine bottom row and nonzero cross terms
/// exercise general 4×4 arithmetic rather than a homogeneous affine shortcut.
fn dense4() -> Matrix4 {
    Matrix4::new([
        [r(5), r(1), r(1), r(1)],
        [r(1), r(6), r(1), r(1)],
        [r(1), r(1), r(7), r(1)],
        [r(1), r(1), r(1), r(8)],
    ])
}

/// Builds a fully nonzero symmetric 3×3 fixture with π+5 at entry (0,0).
///
/// The remaining entries are integers and its determinant is `175 + 39*π`.
/// This is a nonsingular symbolic operand; construction does not itself assert
/// which nonzero proofs or inverse schedules will be recognized.
fn symbolic_dense3() -> Matrix3 {
    Matrix3::new([
        [Real::pi() + r(5), r(1), r(2)],
        [r(1), r(6), r(3)],
        [r(2), r(3), r(8)],
    ])
}

/// Builds a fully nonzero symmetric 4×4 fixture with π+9 at entry (0,0).
///
/// Its determinant is `7416 + 906*π`; the nonaffine matrix supplies symbolic
/// inverse/division inputs. A caller that discards a fallible result does not
/// establish completion merely because this mathematical matrix is nonsingular.
fn symbolic_dense4() -> Matrix4 {
    Matrix4::new([
        [Real::pi() + r(9), r(1), r(2), r(3)],
        [r(1), r(10), r(3), r(4)],
        [r(2), r(3), r(11), r(5)],
        [r(3), r(4), r(5), r(12)],
    ])
}

/// Builds a fully nonzero rational 3×3 fixture containing non-dyadic entries.
///
/// Positive diagonal entries strictly exceed each row's off-diagonal absolute sum,
/// so the matrix is nonsingular. The 3/4 diagonal entry is dyadic; the fixture name
/// means the aggregate is not all dyadic, not that every coefficient is non-dyadic.
fn non_dyadic_dense3() -> Matrix3 {
    Matrix3::new([
        [frac(2, 3), frac(1, 5), frac(1, 7)],
        [frac(1, 11), frac(3, 4), frac(1, 13)],
        [frac(1, 17), frac(1, 19), frac(4, 5)],
    ])
}

/// Builds a fully nonzero rational 4×4 fixture containing non-dyadic entries.
///
/// Strict row diagonal dominance makes it nonsingular. Mixed denominators exercise
/// rational inverse composition; the dyadic 3/4 entry remains part of the fixture.
fn non_dyadic_dense4() -> Matrix4 {
    Matrix4::new([
        [frac(2, 3), frac(1, 5), frac(1, 7), frac(1, 11)],
        [frac(1, 13), frac(3, 4), frac(1, 17), frac(1, 19)],
        [frac(1, 23), frac(1, 29), frac(4, 5), frac(1, 31)],
        [frac(1, 37), frac(1, 41), frac(1, 43), frac(5, 6)],
    ])
}

/// Builds an upper-triangular 3×3 integer fixture with diagonal (2,4,5) and determinant 40.
///
/// Nonzero upper entries exercise substitution; its bottom-right value 5 makes
/// it nonaffine in the 2D homogeneous convention.
fn upper3() -> Matrix3 {
    Matrix3::new([[r(2), r(1), r(3)], [r(0), r(4), r(2)], [r(0), r(0), r(5)]])
}

/// Transposes upper3 into a lower-triangular fixture with the same diagonal and determinant.
///
/// Mirrors the substitution direction without adding an independent set of coefficients.
fn lower3() -> Matrix3 {
    upper3().transpose()
}

/// Builds an upper-triangular 4×4 integer fixture with diagonal (2,4,5,6) and determinant 240.
///
/// Every strict upper entry is nonzero. The bottom-right value 6 distinguishes
/// this general triangular operand from an affine homogeneous transform.
fn upper4() -> Matrix4 {
    Matrix4::new([
        [r(2), r(1), r(3), r(2)],
        [r(0), r(4), r(2), r(1)],
        [r(0), r(0), r(5), r(3)],
        [r(0), r(0), r(0), r(6)],
    ])
}

/// Transposes upper4 into a lower-triangular fixture with determinant 240.
///
/// Retains the diagonal while moving nonzero off-diagonal coefficients below it.
fn lower4() -> Matrix4 {
    upper4().transpose()
}

/// Builds the 2D affine map `(x,y) -> (2*x+3, 4*y+5)` in homogeneous 3×3 form.
///
/// Its linear block is diagonal and determinant 8; the full matrix includes
/// translation and is not diagonal.
fn affine3_diagonal() -> Matrix3 {
    Matrix3::new([[r(2), r(0), r(3)], [r(0), r(4), r(5)], [r(0), r(0), r(1)]])
}

/// Builds a 2D affine fixture with linear block `[[2,1],[1,3]]` and translation (3,4).
///
/// The dense linear block has determinant 5. The full homogeneous matrix retains
/// its zero bottom spatial entries, so dense refers to the linear block.
fn affine3_dense() -> Matrix3 {
    Matrix3::new([[r(2), r(1), r(3)], [r(1), r(3), r(4)], [r(0), r(0), r(1)]])
}

/// Builds a 2D affine fixture with upper-triangular linear block `[[2,1],[0,4]]`.
///
/// Translation is (3,5), the bottom row is (0,0,1), and determinant is 8.
/// This separates affine upper-triangular dispatch from diagonal-linear handling.
fn affine3_upper() -> Matrix3 {
    Matrix3::new([[r(2), r(1), r(3)], [r(0), r(4), r(5)], [r(0), r(0), r(1)]])
}

/// Embeds dense3 as a 3D affine linear block with translation (3,4,5).
///
/// The bottom row is (0,0,0,1) and determinant is 17. Dense describes the linear
/// block, not the full matrix, whose homogeneous spatial row is exactly zero.
fn affine4_dense() -> Matrix4 {
    Matrix4::new([
        [r(2), r(1), r(1), r(3)],
        [r(1), r(3), r(1), r(4)],
        [r(1), r(1), r(4), r(5)],
        [r(0), r(0), r(0), r(1)],
    ])
}

/// Builds the affine map `(x,y,z) -> (2*x+3, 3*y+4, 5*z+5)`.
///
/// Its diagonal linear block has determinant 30, with unit homogeneous weight.
/// Nonzero translation distinguishes it from a full diagonal matrix.
fn affine4_diagonal() -> Matrix4 {
    Matrix4::new([
        [r(2), r(0), r(0), r(3)],
        [r(0), r(3), r(0), r(4)],
        [r(0), r(0), r(5), r(5)],
        [r(0), r(0), r(0), r(1)],
    ])
}

/// Exercises twenty-six arithmetic ownership forms for compatible matrix/vector operands.
///
/// Evaluates each supplied expression once, binds scalar two, and invokes matrix
/// addition/subtraction, scalar arithmetic, negation, matrix products and vector
/// products. Scalar divisions must succeed via unwrap; other outputs are discarded.
/// There are no value or trace assertions here, and matrix right division is not
/// part of this macro.
macro_rules! exercise_matrix_operators {
    ($left:expr, $right:expr, $vector:expr) => {{
        let left = $left;
        let right = $right;
        let vector = $vector;
        let scalar = r(2);

        let _ = left.clone() + right.clone();
        let _ = left.clone() + &right;
        let _ = &left + right.clone();
        let _ = &left + &right;
        let _ = left.clone() - right.clone();
        let _ = left.clone() - &right;
        let _ = &left - right.clone();
        let _ = &left - &right;
        let _ = left.clone() + scalar.clone();
        let _ = left.clone() + &scalar;
        let _ = left.clone() - scalar.clone();
        let _ = left.clone() - &scalar;
        let _ = -left.clone();
        let _ = -&left;
        let _ = left.clone() * scalar.clone();
        let _ = left.clone() * &scalar;
        let _ = (left.clone() / scalar.clone()).unwrap();
        let _ = (left.clone() / &scalar).unwrap();

        let _ = left.clone() * right.clone();
        let _ = left.clone() * &right;
        let _ = &left * right.clone();
        let _ = &left * &right;
        let _ = left.clone() * vector.clone();
        let _ = left.clone() * &vector;
        let _ = &left * vector.clone();
        let _ = &left * &vector;
    }};
}

/// Checks that each of the eight named schedule hints belongs to exactly one classifier group.
///
/// Also calls structural accessors on selected zero, identity, rational, symbolic
/// and signed-permutation fixtures, including one out-of-range diagonal index.
/// Those accessor results are discarded: the test does not verify their values
/// or exhaust all masks, indices, matrices or determinant algorithms.
#[test]
fn structural_fact_accessors_and_schedule_classifiers_are_exhaustive() {
    for hint in [
        MatrixDeterminantScheduleHint::StructurallyZero,
        MatrixDeterminantScheduleHint::Diagonal,
        MatrixDeterminantScheduleHint::Triangular,
        MatrixDeterminantScheduleHint::SparseSupport,
        MatrixDeterminantScheduleHint::SharedDenominator,
        MatrixDeterminantScheduleHint::Dyadic,
        MatrixDeterminantScheduleHint::ExactRational,
        MatrixDeterminantScheduleHint::GenericRealFallback,
    ] {
        let shape = hint.is_shape_driven();
        let exact = hint.is_exact_rational_driven();
        let generic = hint.requires_generic_real_fallback();
        assert_eq!(shape as u8 + exact as u8 + generic as u8, 1);
    }

    for matrix in [
        Matrix3::zero(),
        Matrix3::identity(),
        dense3(),
        symbolic_dense3(),
    ] {
        let facts = matrix.structural_facts();
        let _ = facts.is_zero();
        for index in 0..=3 {
            let _ = facts.entry_known_zero(index, index);
            let _ = facts.entry_known_one(index, index);
            let _ = facts.row_zero_mask(index);
            let _ = facts.column_zero_mask(index);
            let _ = facts.row_known_zero_count(index);
            let _ = facts.column_known_zero_count(index);
            let _ = facts.row_is_known_zero(index);
            let _ = facts.column_is_known_zero(index);
            let _ = facts.row_has_sparse_support(index);
            let _ = facts.column_has_sparse_support(index);
        }
        let _ = facts.has_known_zero_row();
        let _ = facts.has_known_zero_column();
        let _ = facts.has_known_zero_lane();
        let _ = facts.all_rows_have_sparse_support();
        let _ = facts.all_columns_have_sparse_support();
        let _ = facts.determinant_schedule_hint();
        let _ = matrix.exact_facts();
    }

    let signed_rows = [
        SignedAxis4::NegW,
        SignedAxis4::NegZ,
        SignedAxis4::NegY,
        SignedAxis4::NegX,
    ];
    for matrix in [
        Matrix4::zero(),
        Matrix4::identity(),
        dense4(),
        symbolic_dense4(),
        Matrix4::signed_permutation(signed_rows),
    ] {
        let facts = matrix.structural_facts();
        let _ = facts.is_zero();
        let _ = facts.is_signed_permutation();
        for index in 0..=4 {
            let _ = facts.entry_known_zero(index, index);
            let _ = facts.entry_known_one(index, index);
            let _ = facts.row_zero_mask(index);
            let _ = facts.column_zero_mask(index);
            let _ = facts.row_known_zero_count(index);
            let _ = facts.column_known_zero_count(index);
            let _ = facts.row_is_known_zero(index);
            let _ = facts.column_is_known_zero(index);
            let _ = facts.row_has_sparse_support(index);
            let _ = facts.column_has_sparse_support(index);
        }
        let _ = facts.has_known_zero_row();
        let _ = facts.has_known_zero_column();
        let _ = facts.has_known_zero_lane();
        let _ = facts.all_rows_have_sparse_support();
        let _ = facts.all_columns_have_sparse_support();
        let _ = facts.determinant_schedule_hint();
        let _ = matrix.exact_facts();
    }
}

/// Exercises operator ownership forms, powers, scalar division, indexing and formatting in both sizes.
///
/// Checks transpose involution, selected inverse aliases/negative-one powers,
/// rational inverse composition, typed scalar errors and indexed writes. Other
/// successful calls are unwrapped without value assertions; symbolic inverse
/// Results are discarded. Formatting is invoked without checking strings, and
/// inactive abort flags do not test cancellation.
#[test]
fn common_matrix_surface_covers_powers_scalars_ownership_and_formatting() {
    let m3 = dense3();
    let m4 = dense4();
    exercise_matrix_operators!(
        m3.clone(),
        Matrix3::identity(),
        Vector3::new([r(1), r(2), r(3)])
    );
    exercise_matrix_operators!(
        m4.clone(),
        Matrix4::identity(),
        Vector4::new([r(1), r(2), r(3), r(4)])
    );

    assert_eq!(m3.transpose().transpose(), m3);
    assert_eq!(m4.transpose().transpose(), m4);
    assert_eq!(
        m3.clone().reciprocal().unwrap(),
        m3.clone().inverse().unwrap()
    );
    assert_eq!(
        m4.clone().reciprocal_checked().unwrap(),
        m4.clone().inverse_checked().unwrap()
    );

    for exponent in 0..=7 {
        let _ = m3.clone().powi(exponent).unwrap();
        let _ = m3.clone().powi_checked(exponent).unwrap();
        let _ = m4.clone().powi(exponent).unwrap();
        let _ = m4.clone().powi_checked(exponent).unwrap();
    }
    assert_eq!(m3.clone().powi(-1).unwrap(), m3.clone().inverse().unwrap());
    assert_eq!(
        m4.clone().powi_checked(-1).unwrap(),
        m4.clone().inverse_checked().unwrap()
    );
    let _ = m3.clone().powi(-2).unwrap();
    let _ = m4.clone().powi_checked(-2).unwrap();
    let _ = (m3.clone() ^ 5).unwrap();
    let _ = (m4.clone() ^ 5).unwrap();

    let signal = abort_signal();
    let _ = Matrix3::identity().inverse().unwrap();
    let _ = Matrix3::identity().inverse_checked().unwrap();
    let _ = Matrix3::identity()
        .inverse_checked_with_abort(&signal)
        .unwrap();
    let _ = Matrix4::identity().inverse().unwrap();
    let _ = Matrix4::identity().inverse_checked().unwrap();
    let _ = Matrix4::identity()
        .inverse_checked_with_abort(&signal)
        .unwrap();
    let _ = symbolic_dense3().inverse();
    let _ = symbolic_dense3().inverse_checked();
    let _ = symbolic_dense3().inverse_checked_with_abort(&signal);
    let _ = symbolic_dense4().inverse();
    let _ = symbolic_dense4().inverse_checked();
    let _ = symbolic_dense4().inverse_checked_with_abort(&signal);
    let rational3 = non_dyadic_dense3();
    assert_eq!(
        rational3.clone() * rational3.clone().inverse().unwrap(),
        Matrix3::identity()
    );
    let _ = rational3.clone().inverse_checked().unwrap();
    let _ = rational3.inverse_checked_with_abort(&signal).unwrap();
    let rational4 = non_dyadic_dense4();
    assert_eq!(
        rational4.clone() * rational4.clone().inverse().unwrap(),
        Matrix4::identity()
    );
    let _ = rational4.clone().inverse_checked().unwrap();
    let _ = rational4.inverse_checked_with_abort(&signal).unwrap();
    let _ = dense4()
        .div_matrix_checked_with_abort(dense4(), &signal)
        .unwrap();
    let _ = Matrix4::affine_nonuniform_scale([r(2), r(3), r(5)]);
    let _ = Matrix3::identity().powi(2).unwrap();
    let _ = Matrix3::identity().powi(3).unwrap();
    let _ = Matrix4::identity().powi(2).unwrap();
    let _ = Matrix4::identity().powi(3).unwrap();
    let _ = m3.clone().powi_checked_with_abort(-2, &signal).unwrap();
    let _ = m4.clone().powi_checked_with_abort(-2, &signal).unwrap();
    let _ = m3.clone().div_scalar_checked(r(2)).unwrap();
    let _ = m4.clone().div_scalar_checked(r(2)).unwrap();
    let _ = m3
        .clone()
        .div_scalar_checked_with_abort(r(2), &signal)
        .unwrap();
    let _ = m4
        .clone()
        .div_scalar_checked_with_abort(r(2), &signal)
        .unwrap();
    assert_eq!(
        m3.clone().div_scalar_checked(unknown_zero()),
        Err(Problem::UnknownZero)
    );
    assert_eq!(
        m4.clone().div_scalar_checked(r(0)),
        Err(Problem::DivideByZero)
    );

    let mut indexed3 = Matrix3::zero();
    indexed3[1][2] = r(7);
    assert_eq!(indexed3[1][2], r(7));
    let mut indexed4 = Matrix4::zero();
    indexed4[3][2] = r(9);
    assert_eq!(indexed4[3][2], r(9));
    let _ = format!("{indexed3}");
    let _ = format!("{indexed3:#}");
    let _ = format!("{indexed4}");
    let _ = format!("{indexed4:#}");

    let diagonal3 = Matrix3::diagonal([r(2), r(3), r(5)]);
    let identity3 = Matrix3::identity();
    let _ = identity3.clone() * m3.clone();
    let _ = identity3.clone() * &m3;
    let _ = &identity3 * m3.clone();
    let _ = &identity3 * &m3;
    let _ = diagonal3.clone() * m3.clone();
    let _ = diagonal3.clone() * &m3;
    let _ = &diagonal3 * m3.clone();
    let _ = &diagonal3 * &m3;
    let _ = m3.clone() * diagonal3.clone();
    let _ = m3.clone() * &diagonal3;
    let _ = &m3 * diagonal3.clone();
    let _ = &m3 * &diagonal3;

    let diagonal4 = Matrix4::diagonal([r(2), r(3), r(5), r(7)]);
    let identity4 = Matrix4::identity();
    let _ = identity4.clone() * m4.clone();
    let _ = identity4.clone() * &m4;
    let _ = &identity4 * m4.clone();
    let _ = &identity4 * &m4;
    let _ = diagonal4.clone() * m4.clone();
    let _ = diagonal4.clone() * &m4;
    let _ = &diagonal4 * m4.clone();
    let _ = &diagonal4 * &m4;
    let _ = m4.clone() * diagonal4.clone();
    let _ = m4.clone() * &diagonal4;
    let _ = &m4 * diagonal4.clone();
    let _ = &m4 * &diagonal4;
    let _ = m4 * &upper4();
}

/// Exercises triangular, diagonal, uniform and affine inversion/division entry points in both sizes.
///
/// Selected inverse products must equal identity; most remaining fallible calls
/// are only required to succeed. Fused 4D vector calls include direction, point
/// and unknown-weight fixtures. This does not validate arbitrary shape evidence,
/// assert every quotient, or count arithmetic/dispatch work. Abort flags remain unset.
#[test]
fn triangular_diagonal_and_uniform_known_shape_apis_cover_both_sizes() {
    let signal = abort_signal();
    let left3 = dense3();
    let up3 = upper3();
    let low3 = lower3();
    assert_eq!(
        up3.clone() * up3.clone().upper_triangular_inverse().unwrap(),
        Matrix3::identity()
    );
    assert_eq!(
        low3.clone() * low3.clone().lower_triangular_inverse().unwrap(),
        Matrix3::identity()
    );
    let _ = up3.clone().upper_triangular_inverse_checked().unwrap();
    let _ = low3.clone().lower_triangular_inverse_checked().unwrap();
    let _ = left3.clone().div_upper_triangular(up3.clone()).unwrap();
    let _ = left3
        .clone()
        .div_upper_triangular_checked(up3.clone())
        .unwrap();
    let _ = left3.clone().div_lower_triangular(low3.clone()).unwrap();
    let _ = left3
        .clone()
        .div_lower_triangular_checked(low3.clone())
        .unwrap();
    let diagonal3 = [r(2), r(3), r(5)];
    let d3 = Matrix3::diagonal(diagonal3.clone());
    assert_eq!(
        d3.clone() * Matrix3::diagonal_inverse(diagonal3.clone()).unwrap(),
        Matrix3::identity()
    );
    let _ = left3.clone().div_diagonal(diagonal3.clone()).unwrap();
    let _ = left3.clone().div_matrix_checked(d3.clone()).unwrap();
    let _ = left3
        .clone()
        .div_matrix_checked_with_abort(d3.clone(), &signal)
        .unwrap();
    let _ = left3
        .div_diagonal_vector(diagonal3, &Vector3::new([r(2), r(3), r(5)]))
        .unwrap();
    assert_eq!(
        Matrix3::uniform_scale(r(3)) * Matrix3::uniform_scale_inverse(r(3)).unwrap(),
        Matrix3::identity()
    );
    let affine_diagonal3 = affine3_diagonal();
    assert_eq!(
        affine_diagonal3.clone() * affine_diagonal3.clone().inverse().unwrap(),
        Matrix3::identity()
    );
    let _ = affine_diagonal3.clone().inverse_checked().unwrap();
    let _ = affine_diagonal3
        .inverse_checked_with_abort(&signal)
        .unwrap();
    let _ = affine3_dense().inverse().unwrap();
    let _ = affine3_dense().inverse_checked().unwrap();
    let _ = affine3_dense().inverse_checked_with_abort(&signal).unwrap();

    let left4 = dense4();
    let up4 = upper4();
    let low4 = lower4();
    assert_eq!(
        up4.clone() * up4.clone().upper_triangular_inverse().unwrap(),
        Matrix4::identity()
    );
    assert_eq!(
        low4.clone() * low4.clone().lower_triangular_inverse().unwrap(),
        Matrix4::identity()
    );
    let _ = up4.clone().upper_triangular_inverse_checked().unwrap();
    let _ = low4.clone().lower_triangular_inverse_checked().unwrap();
    let _ = left4.clone().div_upper_triangular(up4.clone()).unwrap();
    let _ = left4
        .clone()
        .div_upper_triangular_checked(up4.clone())
        .unwrap();
    let _ = left4.clone().div_lower_triangular(low4.clone()).unwrap();
    let _ = left4
        .clone()
        .div_lower_triangular_checked(low4.clone())
        .unwrap();
    let diagonal4 = [r(2), r(3), r(5), r(7)];
    let d4 = Matrix4::diagonal(diagonal4.clone());
    assert_eq!(
        d4.clone() * Matrix4::diagonal_inverse(diagonal4.clone()).unwrap(),
        Matrix4::identity()
    );
    let _ = left4.clone().div_diagonal(diagonal4.clone()).unwrap();
    let direction = Vector4::new([r(2), r(3), r(5), r(0)]);
    let point = Vector4::new([r(2), r(3), r(5), r(1)]);
    let unknown = Vector4::new([r(2), r(3), r(5), unknown_zero()]);
    let _ = left4
        .div_diagonal_vector(diagonal4.clone(), &direction)
        .unwrap();
    let _ = d4.div_diagonal_vector(diagonal4.clone(), &point).unwrap();
    let _ = dense4()
        .div_diagonal_vector(diagonal4.clone(), &unknown)
        .unwrap();
    let _ = Matrix4::identity()
        .div_diagonal_vector(diagonal4.clone(), &unknown)
        .unwrap();
    let _ = Matrix4::diagonal([r(2), r(3), r(5), r(7)])
        .div_diagonal_vector(diagonal4.clone(), &unknown)
        .unwrap();
    let _ = dense4()
        .div_diagonal_direction_vector(diagonal4, &direction)
        .unwrap();
    assert_eq!(
        Matrix4::uniform_scale(r(3)) * Matrix4::uniform_scale_inverse(r(3)).unwrap(),
        Matrix4::identity()
    );
    let affine_diagonal4 = affine4_diagonal();
    assert_eq!(
        affine_diagonal4.clone() * affine_diagonal4.clone().inverse().unwrap(),
        Matrix4::identity()
    );
    let _ = affine_diagonal4.clone().inverse_checked().unwrap();
    let _ = affine_diagonal4
        .inverse_checked_with_abort(&signal)
        .unwrap();
    let affine_translation4 = Matrix4::affine_translation([r(3), r(5), r(7)]);
    assert_eq!(
        affine_translation4.clone() * affine_translation4.clone().inverse().unwrap(),
        Matrix4::identity()
    );
    let _ = affine_translation4.clone().inverse_checked().unwrap();
    let _ = affine_translation4
        .inverse_checked_with_abort(&signal)
        .unwrap();
    let _ = affine4_dense().inverse().unwrap();
    let _ = affine4_dense().inverse_checked().unwrap();
    let _ = affine4_dense().inverse_checked_with_abort(&signal).unwrap();
}

/// Exercises right division across ownership forms and selected affine/projective operand shapes.
///
/// Ordinary symbolic-divisor calls must succeed, but their checked Results are
/// discarded. Rational shape combinations and symbolic dividends with rational
/// divisors are unwrapped without quotient comparisons. Explicit assertions require
/// UnknownZero for each spatial pivot of a fresh diagonal-linear affine 4×4 divisor,
/// with affine and projective dividends. The flag is unset; no route counts or
/// cancellation guarantees are asserted.
#[test]
fn generic_and_affine_right_division_dispatches_cover_owned_borrowed_checked_and_abort() {
    let signal = abort_signal();
    let left3_projective = dense3();
    let left3_affine = affine3_dense();
    let right3_symbolic = symbolic_dense3();
    let _ = (left3_projective.clone() / right3_symbolic.clone()).unwrap();
    let _ = (left3_projective.clone() / &right3_symbolic).unwrap();
    let _ = (&left3_projective / right3_symbolic.clone()).unwrap();
    let _ = (&left3_projective / &right3_symbolic).unwrap();
    let _ = left3_projective
        .clone()
        .div_matrix_checked(right3_symbolic.clone());
    let _ = left3_projective
        .clone()
        .div_matrix_checked_with_abort(right3_symbolic, &signal);

    let translation3 = Matrix3::new([[r(1), r(0), r(3)], [r(0), r(1), r(5)], [r(0), r(0), r(1)]]);
    let cases3 = [
        Matrix3::identity(),
        Matrix3::diagonal([r(2), r(3), r(5)]),
        translation3,
        affine3_diagonal(),
        affine3_upper(),
        affine3_dense(),
        upper3(),
        lower3(),
    ];
    for right in cases3 {
        let _ = (left3_projective.clone() / right.clone()).unwrap();
        let _ = (&left3_projective / &right).unwrap();
        let _ = left3_projective
            .clone()
            .div_matrix_checked(right.clone())
            .unwrap();
        let _ = left3_projective
            .clone()
            .div_matrix_checked_with_abort(right.clone(), &signal)
            .unwrap();
        let _ = (left3_affine.clone() / right.clone()).unwrap();
        let _ = (&left3_affine / &right).unwrap();
        let _ = left3_affine
            .clone()
            .div_matrix_checked(right.clone())
            .unwrap();
        let _ = left3_affine
            .clone()
            .div_matrix_checked_with_abort(right, &signal)
            .unwrap();
    }

    let left4_projective = dense4();
    let left4_affine = affine4_dense();
    let right4_symbolic = symbolic_dense4();
    let _ = (left4_projective.clone() / right4_symbolic.clone()).unwrap();
    let _ = (left4_projective.clone() / &right4_symbolic).unwrap();
    let _ = (&left4_projective / right4_symbolic.clone()).unwrap();
    let _ = (&left4_projective / &right4_symbolic).unwrap();
    let _ = left4_projective
        .clone()
        .div_matrix_checked(right4_symbolic.clone());
    let _ = left4_projective
        .clone()
        .div_matrix_checked_with_abort(right4_symbolic, &signal);

    let translation4 = Matrix4::affine_translation([r(3), r(5), r(7)]);
    let diagonal_affine4 = affine4_diagonal();
    let affine_upper4 = Matrix4::new([
        [r(2), r(1), r(3), r(7)],
        [r(0), r(4), r(2), r(8)],
        [r(0), r(0), r(5), r(9)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let cases4 = [
        Matrix4::identity(),
        Matrix4::diagonal([r(2), r(3), r(5), r(7)]),
        translation4,
        diagonal_affine4,
        affine4_dense(),
        affine_upper4,
        upper4(),
        lower4(),
    ];
    for right in cases4 {
        let _ = (left4_projective.clone() / right.clone()).unwrap();
        let _ = (&left4_projective / &right).unwrap();
        let _ = left4_projective
            .clone()
            .div_matrix_checked(right.clone())
            .unwrap();
        let _ = left4_projective
            .clone()
            .div_matrix_checked_with_abort(right.clone(), &signal)
            .unwrap();
        let _ = (left4_affine.clone() / right.clone()).unwrap();
        let _ = (&left4_affine / &right).unwrap();
        let _ = left4_affine
            .clone()
            .div_matrix_checked(right.clone())
            .unwrap();
        let _ = left4_affine
            .clone()
            .div_matrix_checked_with_abort(right, &signal)
            .unwrap();
    }

    let exact_right3 = dense3();
    let symbolic_left3 = symbolic_dense3();
    let _ = (symbolic_left3.clone() / exact_right3.clone()).unwrap();
    let _ = (&symbolic_left3 / &exact_right3).unwrap();
    let _ = symbolic_left3
        .clone()
        .div_matrix_checked(exact_right3.clone())
        .unwrap();
    let _ = symbolic_left3
        .div_matrix_checked_with_abort(exact_right3, &signal)
        .unwrap();

    let exact_right4 = dense4();
    let symbolic_left4 = symbolic_dense4();
    let _ = (symbolic_left4.clone() / exact_right4.clone()).unwrap();
    let _ = (&symbolic_left4 / &exact_right4).unwrap();
    let _ = symbolic_left4
        .clone()
        .div_matrix_checked(exact_right4.clone())
        .unwrap();
    let _ = symbolic_left4
        .div_matrix_checked_with_abort(exact_right4, &signal)
        .unwrap();

    for unknown_index in 0..3 {
        let diagonal: [hyperlattice::Real; 3] = std::array::from_fn(|index| {
            if index == unknown_index {
                unknown_zero()
            } else {
                r(index as i32 + 2)
            }
        });
        let unknown_affine4 = Matrix4::new([
            [diagonal[0].clone(), r(0), r(0), r(3)],
            [r(0), diagonal[1].clone(), r(0), r(4)],
            [r(0), r(0), diagonal[2].clone(), r(5)],
            [r(0), r(0), r(0), r(1)],
        ]);
        assert_eq!(
            left4_projective
                .clone()
                .div_matrix_checked(unknown_affine4.clone()),
            Err(Problem::UnknownZero)
        );
        assert_eq!(
            left4_projective
                .clone()
                .div_matrix_checked_with_abort(unknown_affine4.clone(), &signal),
            Err(Problem::UnknownZero)
        );
        assert_eq!(
            left4_affine
                .clone()
                .div_matrix_checked(unknown_affine4.clone()),
            Err(Problem::UnknownZero)
        );
        assert_eq!(
            left4_affine
                .clone()
                .div_matrix_checked_with_abort(unknown_affine4, &signal),
            Err(Problem::UnknownZero)
        );
    }
}

/// Exercises object-specific construction, rotations, storage and homogeneous transforms.
///
/// Checks translation/orthonormal inverse composition using an identity linear
/// block, one negative reversed signed permutation's inverse, identity alignment,
/// zero-axis and unit-antipode errors, and selected row-storage behavior. Most
/// rotation and transform results are discarded. Literal projective point/vector
/// expectations check weight scaling; 3D vector wrappers are compared with products.
/// This is selected API coverage, not exhaustive rotation or transform validation.
#[test]
fn affine_rotation_signed_permutation_and_transform_apis_cover_object_routes() {
    let translation = [r(3), r(5), r(7)];
    let transform = Matrix4::affine_translation(translation.clone());
    assert_eq!(
        transform.clone() * Matrix4::affine_translation_inverse(translation.clone()),
        Matrix4::identity()
    );
    assert_eq!(
        transform
            .clone()
            .div_affine_translation(translation.clone()),
        Matrix4::identity()
    );

    let identity_linear = [[r(1), r(0), r(0)], [r(0), r(1), r(0)], [r(0), r(0), r(1)]];
    let rigid = Matrix4::affine_orthonormal(identity_linear.clone(), translation.clone());
    let rigid_inverse =
        Matrix4::affine_orthonormal_inverse(identity_linear.clone(), translation.clone());
    assert_eq!(rigid.clone() * rigid_inverse, Matrix4::identity());
    assert_eq!(
        rigid.div_affine_orthonormal(identity_linear, translation),
        Matrix4::identity()
    );

    let positive_rows = [
        SignedAxis4::PosX,
        SignedAxis4::PosY,
        SignedAxis4::PosZ,
        SignedAxis4::PosW,
    ];
    let negative_rows = [
        SignedAxis4::NegW,
        SignedAxis4::NegZ,
        SignedAxis4::NegY,
        SignedAxis4::NegX,
    ];
    let signed = Matrix4::signed_permutation(negative_rows);
    assert_eq!(
        signed.clone() * Matrix4::signed_permutation_inverse(negative_rows),
        Matrix4::identity()
    );
    let vector = Vector4::new([r(2), r(3), r(5), r(7)]);
    assert_eq!(
        Matrix4::transform_signed_permutation_vector(positive_rows, &vector),
        vector
    );
    let _ = Matrix4::transform_signed_permutation_vector(negative_rows, &vector);
    let _ =
        Matrix4::transform_signed_permutation_batch(negative_rows, std::slice::from_ref(&vector));
    let _ = dense4().div_signed_permutation(negative_rows);

    let quarter_turn = Real::pi() / r(2);
    let quarter_turn = quarter_turn.unwrap();
    let _ = Matrix4::rotation_x(quarter_turn.clone());
    let _ = Matrix4::rotation_y(quarter_turn.clone());
    let _ = Matrix4::rotation_z(quarter_turn.clone());
    let _ = Matrix4::rotation_axis_angle(&Vector3::x(), quarter_turn.clone()).unwrap();
    let _ = Matrix4::rotation_axis_angle(&-Vector3::x(), quarter_turn.clone()).unwrap();
    let _ = Matrix4::rotation_axis_angle(&Vector3::y(), quarter_turn.clone()).unwrap();
    let _ = Matrix4::rotation_axis_angle(&Vector3::z(), quarter_turn.clone()).unwrap();
    let _ = Matrix4::rotation_axis_angle(&Vector3::new([r(1), r(1), r(1)]), quarter_turn).unwrap();
    assert_eq!(
        Matrix4::rotation_axis_angle(&Vector3::zero(), r(1)),
        Err(Problem::DivideByZero)
    );
    assert_eq!(
        Matrix4::rotation_between_vectors(&Vector3::x(), &Vector3::x()).unwrap(),
        Matrix4::identity()
    );
    let _ = Matrix4::rotation_between_vectors(&Vector3::x(), &-Vector3::x()).unwrap();
    let _ = Matrix4::rotation_between_vectors(&Vector3::y(), &-Vector3::y()).unwrap();
    let _ = Matrix4::rotation_between_vectors(&Vector3::x(), &Vector3::y()).unwrap();
    let _ = Matrix4::rotation_between_unit_vectors(&Vector3::x(), &Vector3::y()).unwrap();
    assert_eq!(
        Matrix4::rotation_between_unit_vectors(&Vector3::x(), &-Vector3::x()),
        Err(Problem::DivideByZero)
    );

    let values: Vec<_> = (1..=16).map(r).collect();
    assert!(Matrix4::from_row_slice(&values[..15]).is_none());
    assert!(Matrix4::from_row_slice(&values).is_some());
    let row_major = Matrix4::from_row_major(std::array::from_fn(|index| r(index as i32 + 1)));
    assert_eq!(row_major[3][3], r(16));

    let point = Point3::new(r(1), r(2), r(3));
    let direction = Vector3::new([r(1), r(2), r(3)]);
    let homogeneous_point = Vector4::new([r(1), r(2), r(3), r(1)]);
    let homogeneous_direction = Vector4::new([r(1), r(2), r(3), r(0)]);
    let homogeneous_unknown = Vector4::new([r(1), r(2), r(3), unknown_zero()]);
    let _ = transform.transform_point3(&point).unwrap();
    let _ = transform
        .transform_point3_batch(std::slice::from_ref(&point))
        .unwrap();
    let _ = transform.transform_direction3(&direction);
    let _ = transform.transform_direction3_batch(std::slice::from_ref(&direction));
    let _ = transform.transform_vec4_point(&homogeneous_point);
    let _ = transform.transform_vec4_direction(&homogeneous_direction);
    let _ = transform.transform_vec4(&homogeneous_unknown);
    let _ = transform.transform_vec4_point_batch(std::slice::from_ref(&homogeneous_point));
    let _ = transform.transform_vec4_direction_batch(std::slice::from_ref(&homogeneous_direction));
    let _ = transform.transform_vec4_batch(&[
        homogeneous_point.clone(),
        homogeneous_direction.clone(),
        homogeneous_unknown.clone(),
    ]);

    let partial_translation = Matrix4::new([
        [r(2), r(1), r(1), r(3)],
        [r(1), r(3), r(1), r(0)],
        [r(1), r(1), r(4), r(5)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let _ =
        partial_translation.transform_vec4_point_batch(std::slice::from_ref(&homogeneous_point));
    let _ = partial_translation
        .transform_vec4_batch(&[homogeneous_point.clone(), homogeneous_direction.clone()]);
    let no_translation = Matrix4::new([
        [r(2), r(1), r(1), r(0)],
        [r(1), r(3), r(1), r(0)],
        [r(1), r(1), r(4), r(0)],
        [r(1), r(1), r(1), r(0)],
    ]);
    let _ = no_translation.transform_vec4_point_batch(std::slice::from_ref(&homogeneous_point));
    let _ = no_translation
        .transform_vec4_batch(&[homogeneous_point.clone(), homogeneous_direction.clone()]);
    let _ = upper4().transform_vec4_batch(std::slice::from_ref(&homogeneous_point));
    let _ = affine4_diagonal()
        .transform_vec4_batch(&[homogeneous_point.clone(), homogeneous_direction.clone()]);
    assert_eq!(
        Matrix4::diagonal([r(2), r(3), r(5), r(7)]).transform_vec4_point(&homogeneous_point),
        Vector4::new([r(2), r(6), r(15), r(7)])
    );
    let projective = Matrix4::diagonal([r(2), r(3), r(5), r(2)]);
    assert_eq!(
        projective.transform_point3(&point).unwrap(),
        Point3::new(r(1), r(3), frac(15, 2))
    );
    let _ = affine4_dense().transform_vec4_direction(&Vector4::new([r(1), r(2), r(3), r(0)]));
    let _ =
        affine4_dense().transform_vec4_direction_batch(&[Vector4::new([r(1), r(2), r(3), r(0)])]);

    let matrix3 = dense3();
    let vector3 = Vector3::new([r(1), r(2), r(3)]);
    assert_eq!(matrix3.transform_vec3(&vector3), &matrix3 * &vector3);
    assert_eq!(
        matrix3.transform_vec3_batch(std::slice::from_ref(&vector3)),
        vec![&matrix3 * &vector3]
    );
    let _ = Matrix3::identity().transform_vec3(&vector3);
    let _ = Matrix3::identity().transform_vec3_batch(std::slice::from_ref(&vector3));
    let _ = Matrix3::diagonal([r(2), r(3), r(5)]).transform_vec3(&vector3);
    let _ =
        Matrix3::diagonal([r(2), r(3), r(5)]).transform_vec3_batch(std::slice::from_ref(&vector3));

    let unknown_vector = Vector4::new([r(1), r(2), r(3), unknown_zero()]);
    let _ = Matrix4::identity().transform_vec4(&unknown_vector);
    let _ = Matrix4::identity().transform_vec4_batch(std::slice::from_ref(&unknown_vector));
    let _ = Matrix4::diagonal([r(2), r(3), r(5), r(7)]).transform_vec4(&unknown_vector);
    let _ = Matrix4::diagonal([r(2), r(3), r(5), r(7)])
        .transform_vec4_batch(std::slice::from_ref(&unknown_vector));
}

/// Checks selected singular and unknown-pivot errors in ordinary and checked matrix APIs.
///
/// Asserts DivideByZero for singular inverses, UnknownZero for unknown diagonal
/// checked inverses, and UnknownZero at every 4×4 diagonal divisor position.
/// Several other fallible calls are discarded. In particular, the already-set
/// abort flag's inverse Results are not asserted, so this test does not establish
/// cancellation behavior or latency; the final square-root Result is also discarded.
#[test]
fn pivot_errors_and_unknown_zero_checked_paths_are_explicit() {
    let signal = abort_signal();
    let singular3 = Matrix3::new([[r(0), r(1), r(0)], [r(0), r(2), r(0)], [r(0), r(0), r(1)]]);
    assert_eq!(singular3.clone().inverse(), Err(Problem::DivideByZero));
    assert_eq!(
        singular3.clone().inverse_checked(),
        Err(Problem::DivideByZero)
    );
    let _ = Matrix3::identity().div_matrix_checked(singular3.clone());
    let _ = Matrix3::identity().div_matrix_checked_with_abort(singular3, &signal);

    let unknown3 = Matrix3::diagonal([unknown_zero(), r(1), r(1)]);
    assert_eq!(
        unknown3.clone().inverse_checked(),
        Err(Problem::UnknownZero)
    );
    let _ = unknown3.inverse_checked_with_abort(&signal);
    let known3 = Matrix3::diagonal([r(2), r(3), r(5)]);
    let _ = known3.clone().inverse_checked().unwrap();
    let _ = known3.inverse_checked_with_abort(&signal).unwrap();

    let singular4 = Matrix4::new([
        [r(0), r(1), r(0), r(0)],
        [r(0), r(2), r(0), r(0)],
        [r(0), r(0), r(1), r(0)],
        [r(0), r(0), r(0), r(1)],
    ]);
    assert_eq!(singular4.clone().inverse(), Err(Problem::DivideByZero));
    assert_eq!(
        singular4.clone().inverse_checked(),
        Err(Problem::DivideByZero)
    );
    let _ = Matrix4::identity().div_matrix_checked(singular4.clone());
    let _ = Matrix4::identity().div_matrix_checked_with_abort(singular4, &signal);

    let unknown4 = Matrix4::diagonal([unknown_zero(), r(1), r(1), r(1)]);
    assert_eq!(
        unknown4.clone().inverse_checked(),
        Err(Problem::UnknownZero)
    );
    let _ = unknown4.inverse_checked_with_abort(&signal);
    for unknown_index in 0..4 {
        let diagonal = std::array::from_fn(|index| {
            if index == unknown_index {
                unknown_zero()
            } else {
                r(index as i32 + 2)
            }
        });
        let unknown_divisor4 = Matrix4::diagonal(diagonal);
        assert_eq!(
            dense4().div_matrix_checked(unknown_divisor4.clone()),
            Err(Problem::UnknownZero)
        );
        assert_eq!(
            dense4().div_matrix_checked_with_abort(unknown_divisor4, &signal),
            Err(Problem::UnknownZero)
        );
    }

    let active = abort_signal();
    active.store(true, Ordering::Relaxed);
    let _ = dense3().inverse_checked_with_abort(&active);
    let _ = dense4().inverse_checked_with_abort(&active);
    let _ = Real::sqrt(frac(2, 1));
}
