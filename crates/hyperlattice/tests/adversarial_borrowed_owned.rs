mod common;

use common::{frac, r};
use hyperlattice::{Complex, Matrix3, Matrix4, Scalar, Vector3, Vector4};

#[test]
fn scalar_owned_and_borrowed_paths_match_for_adversarial_values() {
    let cases: [(Scalar, Scalar); 5] = [
        (frac(1, 3), frac(5, 7)),
        (frac(-11, 13), frac(17, 19)),
        (frac(1 << 20, 3), frac(-99, 70)),
        (
            Scalar::try_from(0.125).unwrap(),
            Scalar::try_from(-0.25).unwrap(),
        ),
        (r(-64), r(8)),
    ];

    for (left, right) in cases {
        assert_eq!(&left + &right, left.clone() + right.clone());
        assert_eq!(&left - &right, left.clone() - right.clone());
        assert_eq!(&left * &right, left.clone() * right.clone());
        if right.zero_status() == hyperlattice::ZeroStatus::NonZero {
            assert_eq!(
                (&left / &right).unwrap(),
                (left.clone() / right.clone()).unwrap()
            );
        }
    }
}

#[test]
fn complex_owned_and_borrowed_division_paths_match_after_cache_warming() {
    let lhs = Complex::new(frac(11, 7), frac(-13, 5));
    let rhs = Complex::new(frac(7, 5), frac(-17, 11));
    let scalar = frac(13, 8);

    let _ = lhs.norm_squared().to_f64_approx();
    let _ = rhs.norm_squared().to_f64_approx();

    assert_eq!(&lhs + &rhs, lhs.clone() + rhs.clone());
    assert_eq!(&lhs - &rhs, lhs.clone() - rhs.clone());
    assert_eq!(&lhs * &rhs, lhs.clone() * rhs.clone());
    assert_eq!((&lhs / &rhs).unwrap(), (lhs.clone() / rhs.clone()).unwrap());
    assert_eq!((lhs.clone() / &scalar).unwrap(), (lhs / scalar).unwrap());
}

#[test]
fn vector_owned_borrowed_and_cached_paths_are_semantically_flat() {
    let lhs = Vector4::new([frac(11, 7), frac(1, 3), r(0), frac(-13, 5)]);
    let rhs = Vector4::new([frac(-5, 7), frac(17, 11), r(0), frac(99, 70)]);
    let scalar = frac(9, 4);

    let _ = lhs.dot(&rhs).to_f64_approx();

    assert_eq!(&lhs + &rhs, lhs.clone() + rhs.clone());
    assert_eq!(&lhs - &rhs, lhs.clone() - rhs.clone());
    assert_eq!(-&lhs, -lhs.clone());
    assert_eq!(lhs.clone() + &scalar, lhs.clone() + scalar.clone());
    assert_eq!(lhs.clone() - &scalar, lhs.clone() - scalar.clone());
    assert_eq!(lhs.clone() * &scalar, lhs.clone() * scalar.clone());
    assert_eq!((lhs.clone() / &scalar).unwrap(), (lhs / scalar).unwrap());
}

#[test]
fn matrix_owned_borrowed_and_checked_paths_match_on_mixed_forms() {
    let lhs = Matrix3::new([
        [frac(11, 7), frac(1, 3), r(0)],
        [r(0), frac(17, 11), frac(-5, 7)],
        [frac(13, 5), r(0), frac(9, 4)],
    ]);
    let rhs = Matrix3::new([[r(2), r(0), r(1)], [r(1), r(3), r(0)], [r(0), r(2), r(1)]]);
    let vector = Vector3::new([frac(2, 3), Scalar::pi(), r(-1)]);
    let scalar = frac(7, 3);

    let _ = lhs.determinant().to_f64_approx();

    assert_eq!(&lhs + &rhs, lhs.clone() + rhs.clone());
    assert_eq!(&lhs - &rhs, lhs.clone() - rhs.clone());
    assert_eq!(&lhs * &rhs, lhs.clone() * rhs.clone());
    assert_eq!((&lhs / &rhs).unwrap(), (lhs.clone() / rhs.clone()).unwrap());
    assert_eq!(&lhs * &vector, lhs.clone() * vector.clone());
    assert_eq!(lhs.clone() + &scalar, lhs.clone() + scalar.clone());
    assert_eq!(lhs.clone() - &scalar, lhs.clone() - scalar.clone());
    assert_eq!(lhs.clone() * &scalar, lhs.clone() * scalar.clone());
    assert_eq!(
        (lhs.clone() / &scalar).unwrap(),
        (lhs.clone() / scalar.clone()).unwrap()
    );
    assert_eq!(
        lhs.clone().div_matrix_checked(rhs.clone()).unwrap(),
        (lhs / rhs).unwrap()
    );
}

#[test]
fn matrix4_checked_abort_paths_match_non_abort_paths() {
    let matrix = Matrix4::new([
        [r(2), r(0), r(1), r(0)],
        [r(1), r(3), r(0), r(1)],
        [r(0), r(2), r(1), r(0)],
        [r(1), r(0), r(0), r(2)],
    ]);
    let signal = common::abort_signal();

    assert_eq!(
        matrix.clone().inverse_checked_with_abort(&signal).unwrap(),
        matrix.clone().inverse_checked().unwrap()
    );
    assert_eq!(
        matrix.clone().powi_checked_with_abort(-2, &signal).unwrap(),
        matrix.clone().powi_checked(-2).unwrap()
    );
    assert_eq!(
        matrix
            .clone()
            .div_matrix_checked_with_abort(matrix.clone(), &signal)
            .unwrap(),
        Matrix4::identity()
    );
}
