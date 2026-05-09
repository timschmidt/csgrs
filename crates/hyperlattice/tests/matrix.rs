mod common;

use common::{abort_signal, frac, r, unknown_zero};
use realistic_blas::{Matrix3, Matrix4, Problem, Vector4, zero};

fn assert_singular_error<T: std::fmt::Debug>(result: Result<T, Problem>) {
    assert!(matches!(
        result,
        Err(Problem::DivideByZero | Problem::UnknownZero)
    ));
}

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
fn matrix3_negative_power_matches_repeated_inverse_product() {
    let matrix = Matrix3::new([[r(2), r(1), r(0)], [r(0), r(3), r(1)], [r(1), r(0), r(2)]]);
    let inverse = matrix.clone().inverse().unwrap();

    assert_eq!(matrix.powi(-2).unwrap(), inverse.clone() * inverse);
}

#[test]
fn matrix4_identity_and_vector_multiply() {
    let vector = Vector4::new([r(1), r(2), r(3), r(4)]);
    assert_eq!(Matrix4::identity() * vector.clone(), vector);
}

#[test]
fn matrix4_inverse_and_determinant_handle_general_integer_matrix() {
    let matrix = Matrix4::new([
        [r(1), r(2), r(3), r(4)],
        [r(0), r(1), r(4), r(2)],
        [r(5), r(6), r(0), r(1)],
        [r(2), r(7), r(1), r(3)],
    ]);

    assert_eq!(matrix.determinant(), r(-198));
    assert_eq!(
        matrix.clone() * matrix.inverse().unwrap(),
        Matrix4::identity()
    );
}

#[test]
fn matrix4_negative_power_matches_repeated_inverse_product() {
    let matrix = Matrix4::new([
        [r(2), r(0), r(1), r(0)],
        [r(1), r(3), r(0), r(1)],
        [r(0), r(2), r(1), r(0)],
        [r(1), r(0), r(0), r(2)],
    ]);
    let inverse = matrix.clone().inverse().unwrap();

    assert_eq!(matrix.powi(-2).unwrap(), inverse.clone() * inverse);
}

#[cfg(feature = "hyperreal-backend")]
#[test]
fn targeted_fractional_matrix_forms_round_trip_exactly() {
    let dyadic3 = Matrix3::new([
        [frac(9, 8), frac(3, 16), frac(-5, 8)],
        [frac(7, 4), frac(-11, 8), frac(13, 16)],
        [frac(5, 8), frac(17, 16), frac(19, 8)],
    ]);
    let equal_den4 = Matrix4::new([
        [frac(11, 10), frac(2, 10), frac(3, 10), frac(4, 10)],
        [frac(5, 10), frac(17, 10), frac(7, 10), frac(-8, 10)],
        [frac(9, 10), frac(-10, 10), frac(23, 10), frac(12, 10)],
        [frac(-13, 10), frac(14, 10), frac(-15, 10), frac(19, 10)],
    ]);
    let mixed_prime4 = Matrix4::new([
        [frac(7, 3), frac(-5, 7), frac(11, 13), frac(13, 5)],
        [frac(17, 11), frac(-19, 17), frac(23, 19), frac(-29, 23)],
        [frac(31, 29), frac(37, 31), frac(-41, 37), frac(43, 41)],
        [frac(-47, 43), frac(53, 47), frac(59, 53), frac(61, 59)],
    ]);

    let dyadic3_inverse = dyadic3.clone().inverse().unwrap();
    assert_eq!(dyadic3.clone() * dyadic3_inverse, Matrix3::identity());
    assert_eq!(
        dyadic3.clone().powi(-2).unwrap(),
        dyadic3.clone().inverse().unwrap() * dyadic3.inverse().unwrap()
    );

    let equal_den4_inverse = equal_den4.clone().inverse().unwrap();
    assert_eq!(equal_den4.clone() * equal_den4_inverse, Matrix4::identity());
    assert_eq!(
        equal_den4.clone().powi(-2).unwrap(),
        equal_den4.clone().inverse().unwrap() * equal_den4.inverse().unwrap()
    );

    let mixed_prime4_inverse = mixed_prime4.clone().inverse().unwrap();
    assert_eq!(
        mixed_prime4.clone() * mixed_prime4_inverse,
        Matrix4::identity()
    );
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
fn matrix_display_forwards_real_formatting() {
    let half = frac(1, 2);
    let quarter = frac(1, 4);
    let matrix = Matrix3::new([
        [half, r(2), r(3)],
        [r(4), quarter, r(6)],
        [r(7), r(8), r(9)],
    ]);

    #[cfg(feature = "hyperreal-backend")]
    assert_eq!(format!("{matrix}"), "[[1/2, 2, 3], [4, 1/4, 6], [7, 8, 9]]");
    #[cfg(not(feature = "hyperreal-backend"))]
    assert_eq!(
        format!("{matrix}"),
        "[[0.5, 2, 3], [4, 0.25, 6], [7, 8, 9]]"
    );
    assert_eq!(
        format!("{matrix:#}"),
        "[[0.5, 2, 3], [4, 0.25, 6], [7, 8, 9]]"
    );
}

#[test]
fn checked_matrix_inverse_rejects_singular_matrices() {
    let singular = Matrix3::new([[r(1), r(2), r(3)], [r(1), r(2), r(3)], [r(0), r(0), r(1)]]);
    let invertible = Matrix3::new([[r(1), r(2), r(3)], [r(0), r(1), r(4)], [r(5), r(6), r(0)]]);
    let signal = abort_signal();

    assert_singular_error(singular.clone().inverse());
    assert_singular_error(singular.clone().reciprocal());
    assert_singular_error(singular.clone().powi(-1));
    assert_singular_error(Matrix3::identity() / singular.clone());
    assert_eq!(Matrix3::identity() / zero(), Err(Problem::DivideByZero));
    assert_singular_error(singular.inverse_checked());
    assert_eq!(
        invertible.clone() * invertible.clone().inverse_checked().unwrap(),
        Matrix3::identity()
    );
    assert_eq!(
        invertible.clone()
            * invertible
                .clone()
                .inverse_checked_with_abort(&signal)
                .unwrap(),
        Matrix3::identity()
    );
    assert_eq!(
        invertible
            .clone()
            .powi_checked_with_abort(-1, &signal)
            .unwrap(),
        invertible.inverse_checked_with_abort(&signal).unwrap()
    );
}

#[test]
fn matrix3_division_solves_right_division() {
    let numerator = Matrix3::new([[r(3), r(1), r(4)], [r(1), r(5), r(9)], [r(2), r(6), r(5)]]);
    let divisor = Matrix3::new([[r(2), r(0), r(1)], [r(1), r(3), r(0)], [r(0), r(2), r(1)]]);

    let quotient = (numerator.clone() / divisor.clone()).unwrap();

    assert_eq!(quotient * divisor, numerator);
}

#[test]
fn checked_matrix3_division_matches_ordinary_right_division() {
    let numerator = Matrix3::new([[r(4), r(2), r(7)], [r(0), r(3), r(1)], [r(5), r(8), r(6)]]);
    let divisor = Matrix3::new([[r(1), r(2), r(0)], [r(0), r(1), r(3)], [r(2), r(0), r(1)]]);
    let signal = abort_signal();

    let ordinary = (numerator.clone() / divisor.clone()).unwrap();

    assert_eq!(
        numerator
            .clone()
            .div_matrix_checked(divisor.clone())
            .unwrap(),
        ordinary
    );
    assert_eq!(
        numerator
            .div_matrix_checked_with_abort(divisor, &signal)
            .unwrap(),
        ordinary
    );
}

#[test]
fn matrix4_division_solves_right_division() {
    let numerator = Matrix4::new([
        [r(1), r(3), r(5), r(7)],
        [r(2), r(4), r(6), r(8)],
        [r(9), r(7), r(5), r(3)],
        [r(8), r(6), r(4), r(2)],
    ]);
    let divisor = Matrix4::new([
        [r(2), r(0), r(1), r(0)],
        [r(1), r(3), r(0), r(1)],
        [r(0), r(2), r(1), r(0)],
        [r(1), r(0), r(0), r(2)],
    ]);

    let quotient = (numerator.clone() / divisor.clone()).unwrap();

    assert_eq!(quotient * divisor, numerator);
}

#[test]
fn checked_matrix_inverse_rejects_unknown_zero_pivots() {
    let matrix = Matrix3::new([
        [unknown_zero(), r(0), r(0)],
        [r(0), r(1), r(0)],
        [r(0), r(0), r(1)],
    ]);
    let signal = abort_signal();

    assert_eq!(matrix.clone().inverse_checked(), Err(Problem::UnknownZero));
    assert_eq!(
        matrix.clone().inverse_checked_with_abort(&signal),
        Err(Problem::UnknownZero)
    );
    assert_eq!(
        Matrix3::identity().div_matrix_checked_with_abort(matrix, &signal),
        Err(Problem::UnknownZero)
    );
}

#[test]
fn checked_matrix_scalar_division_accepts_abort_signal() {
    let matrix = Matrix3::new([
        [r(2), r(4), r(6)],
        [r(8), r(10), r(12)],
        [r(14), r(16), r(18)],
    ]);
    let signal = abort_signal();

    assert_eq!(
        matrix.div_scalar_checked_with_abort(r(2), &signal).unwrap(),
        Matrix3::new([[r(1), r(2), r(3)], [r(4), r(5), r(6)], [r(7), r(8), r(9)]])
    );
}

#[cfg(not(feature = "hyperreal-backend"))]
#[test]
fn ordinary_matrix_scalar_division_returns_unknown_zero() {
    let divisor = realistic_blas::Scalar::approx(0.0, 0.25).unwrap();

    assert_eq!(Matrix3::identity() / divisor, Err(Problem::UnknownZero));
}

#[test]
fn ordinary_matrix_inverse_prefers_known_nonzero_pivots() {
    let matrix = Matrix3::new([
        [unknown_zero(), r(0), r(1)],
        [r(1), r(1), r(0)],
        [r(0), r(1), r(1)],
    ]);

    assert!(matrix.inverse().is_ok());
}
