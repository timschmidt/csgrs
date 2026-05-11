mod common;

use common::{frac, r, unknown_zero};
use realistic_blas::{Matrix3, Matrix4, Problem, Scalar, Vector3, Vector4, ZeroStatus};

fn assert_matrix3_inverse_invariants(matrix: Matrix3) {
    let inverse = matrix.clone().inverse().unwrap();
    assert_eq!(matrix.clone() * Matrix3::identity(), matrix);
    assert_eq!(Matrix3::identity() * matrix.clone(), matrix);
    assert_eq!(matrix.transpose().transpose(), matrix);
    assert_eq!(matrix.clone() * inverse.clone(), Matrix3::identity());
    assert_eq!(inverse * matrix.clone(), Matrix3::identity());
    assert_eq!((matrix.clone() / matrix.clone()).unwrap(), Matrix3::identity());
    assert_eq!(matrix.clone().powi(0).unwrap(), Matrix3::identity());
    assert_eq!(matrix.clone().powi(1).unwrap(), matrix);
    assert_eq!(matrix.clone().powi(-1).unwrap(), matrix.inverse().unwrap());
}

fn assert_matrix4_inverse_invariants(matrix: Matrix4) {
    let inverse = matrix.clone().inverse().unwrap();
    assert_eq!(matrix.clone() * Matrix4::identity(), matrix);
    assert_eq!(Matrix4::identity() * matrix.clone(), matrix);
    assert_eq!(matrix.transpose().transpose(), matrix);
    assert_eq!(matrix.clone() * inverse.clone(), Matrix4::identity());
    assert_eq!(inverse * matrix.clone(), Matrix4::identity());
    assert_eq!((matrix.clone() / matrix.clone()).unwrap(), Matrix4::identity());
    assert_eq!(matrix.clone().powi(0).unwrap(), Matrix4::identity());
    assert_eq!(matrix.clone().powi(1).unwrap(), matrix);
    assert_eq!(matrix.clone().powi(-1).unwrap(), matrix.inverse().unwrap());
}

#[test]
fn matrix3_adversarial_exact_and_symbolic_kernels_obey_algebra() {
    assert_matrix3_inverse_invariants(Matrix3::new([
        [frac(9, 8), frac(3, 16), frac(-5, 8)],
        [frac(7, 4), frac(-11, 8), frac(13, 16)],
        [frac(5, 8), frac(17, 16), frac(19, 8)],
    ]));
}

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

#[test]
fn determinant_product_identity_holds_for_mixed_matrices() {
    let a = Matrix3::new([[r(2), r(0), r(0)], [r(0), r(3), r(0)], [r(0), r(0), r(5)]]);
    let b = Matrix3::new([[frac(5, 3), r(0), r(0)], [r(0), frac(7, 5), r(0)], [r(0), r(0), frac(11, 7)]]);
    assert_eq!((a.clone() * b.clone()).determinant(), a.determinant() * b.determinant());

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
    assert_eq!((c.clone() * d.clone()).determinant(), c.determinant() * d.determinant());
}

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

#[test]
fn sparse_symbolic_transform_preserves_zero_and_sign_facts() {
    let matrix = Matrix4::new([
        [Scalar::pi(), r(0), Scalar::e(), r(0)],
        [r(0), r(0), r(0), r(0)],
        [r(0), frac(-2, 1), r(0), r(0)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let direction = Vector4::new([r(3), r(7), r(0), r(0)]);
    let transformed = matrix * direction;

    assert_eq!(transformed[0].zero_status(), ZeroStatus::NonZero);
    assert_eq!(transformed[1].zero_status(), ZeroStatus::Zero);
    assert_eq!(transformed[2].zero_status(), ZeroStatus::NonZero);
    assert_eq!(transformed[3].zero_status(), ZeroStatus::Zero);
}

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
