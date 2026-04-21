mod common;

use common::{r, unknown_zero};
use realistic_blas::{BlasProblem, Matrix3, Matrix4, Problem, Vector4, zero};

#[test]
fn matrix3_inverse_and_power() {
    let matrix = Matrix3::new([[r(1), r(2), r(3)], [r(0), r(1), r(4)], [r(5), r(6), r(0)]]);

    assert_eq!(matrix.determinant(), r(1));
    assert_eq!(
        matrix.clone() * matrix.clone().inverse().unwrap(),
        Matrix3::identity()
    );
    assert_eq!((matrix.clone() ^ 0).unwrap(), Matrix3::identity());
    assert_eq!((matrix.clone() ^ 1).unwrap(), matrix);
}

#[test]
fn matrix4_identity_and_vector_multiply() {
    let vector = Vector4::new([r(1), r(2), r(3), r(4)]);
    assert_eq!(Matrix4::identity() * vector.clone(), vector);
}

#[test]
fn matrix_scalar_add_and_subtract_are_componentwise() {
    let matrix = Matrix3::new([[r(1), r(2), r(3)], [r(4), r(5), r(6)], [r(7), r(8), r(9)]]);

    assert_eq!(
        matrix.clone() + r(1),
        Matrix3::new([[r(2), r(3), r(4)], [r(5), r(6), r(7)], [r(8), r(9), r(10)],])
    );
    assert_eq!(
        matrix - r(2),
        Matrix3::new([[r(-1), r(0), r(1)], [r(2), r(3), r(4)], [r(5), r(6), r(7)],])
    );
}

#[test]
fn checked_matrix_inverse_rejects_singular_matrices() {
    let singular = Matrix3::new([[r(1), r(2), r(3)], [r(1), r(2), r(3)], [r(0), r(0), r(1)]]);
    let invertible = Matrix3::new([[r(1), r(2), r(3)], [r(0), r(1), r(4)], [r(5), r(6), r(0)]]);

    assert_eq!(singular.clone().inverse(), Err(Problem::DivideByZero));
    assert_eq!(singular.clone().reciprocal(), Err(Problem::DivideByZero));
    assert_eq!(singular.clone().powi(-1), Err(Problem::DivideByZero));
    assert_eq!(
        Matrix3::identity() / singular.clone(),
        Err(Problem::DivideByZero)
    );
    assert_eq!(Matrix3::identity() / zero(), Err(Problem::DivideByZero));
    assert_eq!(
        singular.inverse_checked(),
        Err(BlasProblem::Real(Problem::DivideByZero))
    );
    assert_eq!(
        invertible.clone() * invertible.clone().inverse_checked().unwrap(),
        Matrix3::identity()
    );
}

#[test]
fn checked_matrix_inverse_rejects_unknown_zero_pivots() {
    let matrix = Matrix3::new([
        [unknown_zero(), r(0), r(0)],
        [r(0), r(1), r(0)],
        [r(0), r(0), r(1)],
    ]);

    assert_eq!(matrix.inverse_checked(), Err(BlasProblem::UnknownZero));
}
