//! Fixed matrix/vector regressions for exact algebra, rejection and retained zero facts.
//!
//! Matrix identities compare related library operations on selected rational fixtures;
//! they are composition checks, not an independent inverse oracle or universal proof.
//! Symbolic transform assertions inspect zero knowledge, while the named 3×3
//! exact-and-symbolic inverse test currently supplies only a dyadic matrix.

/// Shared exact rational, scalar-status and abort fixtures used by this integration target.
mod common;

use common::{frac, r, unknown_zero};
use hyperlattice::{Matrix3, Matrix4, Problem, Real, Vector3, Vector4, ZeroKnowledge};

/// Checks identity products, transpose involution, two-sided inverse, self-division and powers 0/1/-1.
///
/// Requires the supplied fixture's ordinary inverse to succeed; unwrap panics on
/// failure. The computed inverse and other matrix paths share implementation, so
/// these 3×3 composition checks are not an independent literal inverse oracle.
fn assert_matrix3_inverse_invariants(matrix: Matrix3) {
    let inverse = matrix.clone().inverse().unwrap();
    assert_eq!(matrix.clone() * Matrix3::identity(), matrix);
    assert_eq!(Matrix3::identity() * matrix.clone(), matrix);
    assert_eq!(matrix.transpose().transpose(), matrix);
    assert_eq!(matrix.clone() * inverse.clone(), Matrix3::identity());
    assert_eq!(inverse * matrix.clone(), Matrix3::identity());
    assert_eq!(
        (matrix.clone() / matrix.clone()).unwrap(),
        Matrix3::identity()
    );
    assert_eq!(matrix.clone().powi(0).unwrap(), Matrix3::identity());
    assert_eq!(matrix.clone().powi(1).unwrap(), matrix);
    assert_eq!(matrix.clone().powi(-1).unwrap(), matrix.inverse().unwrap());
}

/// Checks the same identity, inverse, self-division and power relations for a 4×4 fixture.
///
/// Requires successful ordinary inversion. Both left/right inverse products are
/// asserted, while the -1 power is compared with a newly computed inverse.
fn assert_matrix4_inverse_invariants(matrix: Matrix4) {
    let inverse = matrix.clone().inverse().unwrap();
    assert_eq!(matrix.clone() * Matrix4::identity(), matrix);
    assert_eq!(Matrix4::identity() * matrix.clone(), matrix);
    assert_eq!(matrix.transpose().transpose(), matrix);
    assert_eq!(matrix.clone() * inverse.clone(), Matrix4::identity());
    assert_eq!(inverse * matrix.clone(), Matrix4::identity());
    assert_eq!(
        (matrix.clone() / matrix.clone()).unwrap(),
        Matrix4::identity()
    );
    assert_eq!(matrix.clone().powi(0).unwrap(), Matrix4::identity());
    assert_eq!(matrix.clone().powi(1).unwrap(), matrix);
    assert_eq!(matrix.clone().powi(-1).unwrap(), matrix.inverse().unwrap());
}

/// Runs the 3×3 invariant helper on one dense dyadic-rational matrix.
///
/// Despite the test name, this fixture contains no symbolic coefficients.
#[test]
fn matrix3_adversarial_exact_and_symbolic_kernels_obey_algebra() {
    assert_matrix3_inverse_invariants(Matrix3::new([
        [frac(9, 8), frac(3, 16), frac(-5, 8)],
        [frac(7, 4), frac(-11, 8), frac(13, 16)],
        [frac(5, 8), frac(17, 16), frac(19, 8)],
    ]));
}

/// Runs 4×4 composition invariants on a dense mixed-denominator matrix and a sparse integer matrix.
#[test]
fn matrix4_adversarial_exact_kernels_obey_algebra() {
    assert_matrix4_inverse_invariants(Matrix4::new([
        [frac(11, 10), frac(2, 10), frac(3, 10), frac(4, 10)],
        [frac(5, 10), frac(17, 10), frac(7, 10), frac(-8, 10)],
        [frac(9, 10), frac(-10, 10), frac(23, 10), frac(12, 10)],
        [frac(-13, 10), frac(14, 10), frac(-15, 10), frac(19, 10)],
    ]));
    assert_matrix4_inverse_invariants(Matrix4::new([
        [r(2), r(0), r(1), r(0)],
        [r(1), r(3), r(0), r(1)],
        [r(0), r(2), r(1), r(0)],
        [r(1), r(0), r(0), r(2)],
    ]));
}

/// Checks det(A*B)=det(A)*det(B) for one diagonal rational pair in each dimension.
///
/// All four operands are diagonal; this does not exercise a general dense product identity.
#[test]
fn determinant_product_identity_holds_for_mixed_matrices() {
    let a = Matrix3::new([[r(2), r(0), r(0)], [r(0), r(3), r(0)], [r(0), r(0), r(5)]]);
    let b = Matrix3::new([
        [frac(5, 3), r(0), r(0)],
        [r(0), frac(7, 5), r(0)],
        [r(0), r(0), frac(11, 7)],
    ]);
    assert_eq!(
        (a.clone() * b.clone()).determinant(),
        a.determinant() * b.determinant()
    );

    let c = Matrix4::new([
        [r(2), r(0), r(0), r(0)],
        [r(0), r(3), r(0), r(0)],
        [r(0), r(0), r(5), r(0)],
        [r(0), r(0), r(0), r(7)],
    ]);
    let d = Matrix4::new([
        [frac(3, 2), r(0), r(0), r(0)],
        [r(0), frac(5, 3), r(0), r(0)],
        [r(0), r(0), frac(7, 5), r(0)],
        [r(0), r(0), r(0), frac(11, 7)],
    ]);
    assert_eq!(
        (c.clone() * d.clone()).determinant(),
        c.determinant() * d.determinant()
    );
}

/// Accepts zero-or-unknown rejection for a dependent-row matrix and requires UnknownZero for an opaque pivot.
///
/// The fresh sine-difference pivot is mathematically nonzero; its cheap status is
/// unknown. The first assertion does not require a definite singularity proof.
#[test]
fn singular_and_unknown_pivot_cases_remain_rejected_by_checked_paths() {
    let singular = Matrix3::new([[r(1), r(2), r(3)], [r(2), r(4), r(6)], [r(0), r(0), r(1)]]);
    assert!(matches!(
        singular.inverse_checked(),
        Err(Problem::DivideByZero | Problem::UnknownZero)
    ));

    let unknown_pivot = Matrix3::new([
        [unknown_zero(), r(0), r(0)],
        [r(0), r(1), r(0)],
        [r(0), r(0), r(1)],
    ]);
    assert_eq!(unknown_pivot.inverse_checked(), Err(Problem::UnknownZero));
}

/// Checks zero/nonzero status of the four coordinates of a sparse π/e direction transform.
///
/// The e coefficient multiplies zero. Despite the name, no sign or coordinate-value
/// assertion is made; the retained facts must be nonzero, zero, nonzero, zero.
#[test]
fn sparse_symbolic_transform_preserves_zero_and_sign_facts() {
    let matrix = Matrix4::new([
        [Real::pi(), r(0), Real::e(), r(0)],
        [r(0), r(0), r(0), r(0)],
        [r(0), frac(-2, 1), r(0), r(0)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let direction = Vector4::new([r(3), r(7), r(0), r(0)]);
    let transformed = matrix * direction;

    assert_eq!(transformed[0].zero_status(), ZeroKnowledge::NonZero);
    assert_eq!(transformed[1].zero_status(), ZeroKnowledge::Zero);
    assert_eq!(transformed[2].zero_status(), ZeroKnowledge::NonZero);
    assert_eq!(transformed[3].zero_status(), ZeroKnowledge::Zero);
}

/// Checks unit squared norm for (6,8,0) and exact arithmetic cancellation of 1000001-1000000.
///
/// The latter difference is expressed through a rational product before a dot sum;
/// this is arithmetic cancellation, not a cooperative abort test.
#[test]
fn vector_normalize_and_dot_invariants_cover_cancellation_and_huge_tiny_mix() {
    let v = Vector3::new([r(6), r(8), r(0)]);
    let unit = v.normalize_checked().unwrap();
    assert_eq!(unit.dot(&unit), r(1));

    let huge_tiny = Vector3::new([
        r(1_000_000) * (r(1) + frac(1, 1_000_000)),
        -r(1_000_000),
        r(0),
    ]);
    assert_eq!(huge_tiny.dot(&Vector3::new([r(1), r(1), r(0)])), r(1));
}
