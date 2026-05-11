mod common;

use common::{frac, r, unknown_zero};
use realistic_blas::{Complex, Matrix3, Matrix4, Problem, Scalar, Vector3, Vector4, ZeroStatus};

#[test]
fn complex_zero_and_unknown_norms_are_rejected_by_checked_division() {
    let zero = Complex::zero();
    let numerator = Complex::new(r(1), r(2));
    assert_eq!(numerator.clone().div_checked(zero), Err(Problem::DivideByZero));

    let unknown = Complex::new(unknown_zero(), r(0));
    assert_eq!(numerator.div_checked(unknown), Err(Problem::UnknownZero));
}

#[test]
fn matrix_power_edges_do_not_hide_domain_errors() {
    let singular = Matrix3::new([
        [r(1), r(2), r(3)],
        [r(1), r(2), r(3)],
        [r(0), r(0), r(1)],
    ]);

    assert_eq!(singular.clone().powi(0).unwrap(), Matrix3::identity());
    assert!(matches!(
        singular.clone().powi(-1),
        Err(Problem::DivideByZero | Problem::UnknownZero)
    ));
    assert!(matches!(
        singular.powi_checked(-2),
        Err(Problem::DivideByZero | Problem::UnknownZero)
    ));
}

#[test]
fn hidden_singular_matrix_by_symbolic_cancellation_is_detected() {
    let pi_minus_pi = Scalar::pi() - Scalar::pi();
    let matrix = Matrix3::new([
        [r(1), r(2), r(3)],
        [r(1) + pi_minus_pi, r(2), r(3)],
        [r(0), r(0), r(1)],
    ]);

    assert_eq!(matrix.determinant().zero_status(), ZeroStatus::Zero);
    assert!(matches!(
        matrix.inverse_checked(),
        Err(Problem::DivideByZero | Problem::UnknownZero)
    ));
}

#[test]
fn matrix_division_and_inverse_are_consistent_under_cached_determinant_warming() {
    let matrix = Matrix4::new([
        [frac(11, 10), frac(2, 10), frac(3, 10), frac(4, 10)],
        [frac(5, 10), frac(17, 10), frac(7, 10), frac(-8, 10)],
        [frac(9, 10), frac(-10, 10), frac(23, 10), frac(12, 10)],
        [frac(-13, 10), frac(14, 10), frac(-15, 10), frac(19, 10)],
    ]);
    let warmed_det = matrix.determinant();
    let _ = warmed_det.to_f64_approx();

    assert_eq!(matrix.clone() * matrix.clone().inverse_checked().unwrap(), Matrix4::identity());
    assert_eq!(
        matrix.clone().div_matrix_checked(matrix.clone()).unwrap(),
        Matrix4::identity()
    );
}

#[test]
fn homogeneous_transform_helpers_match_direct_multiplication_for_points_and_directions() {
    let transform = Matrix4::new([
        [r(2), r(0), r(0), r(7)],
        [r(0), r(3), r(0), r(-5)],
        [r(0), r(0), r(4), r(11)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let point = Vector4::new([r(13), r(-17), r(19), r(1)]);
    let direction = Vector4::new([r(13), r(-17), r(19), r(0)]);

    assert_eq!(transform.transform_vec4_point(&point), transform.clone() * point);
    assert_eq!(
        transform.transform_vec4_direction(&direction),
        transform.clone() * direction.clone()
    );
    assert_eq!(transform.transform_vec4_direction(&direction)[3].zero_status(), ZeroStatus::Zero);
}

#[test]
fn normalization_checked_rejects_unknown_zero_norm_but_preserves_exact_unit_vectors() {
    let unknown = Vector3::new([unknown_zero(), r(0), r(0)]);
    assert_eq!(unknown.normalize_checked(), Err(Problem::UnknownZero));

    let exact = Vector3::new([frac(3, 5), frac(4, 5), r(0)]);
    assert_eq!(exact.normalize_checked().unwrap(), exact);
}

