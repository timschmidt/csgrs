mod common;

use common::{frac, r};
use hyperlattice::{Complex, Matrix3, Real, Vector3};

#[test]
fn borrowed_scalar_ops_match_owned_ops() {
    let a = r(6);
    let b = r(3);

    assert_eq!(&a + &b, a.clone() + b.clone());
    assert_eq!(&a - &b, a.clone() - b.clone());
    assert_eq!(&a * &b, a.clone() * b.clone());
    assert_eq!((&a / &b).unwrap(), (a / b).unwrap());
}

#[test]
fn borrowed_vector_ops_match_owned_ops() {
    let lhs = Vector3::new([r(1), r(2), r(3)]);
    let rhs = Vector3::new([r(4), r(5), r(6)]);
    let scalar: Real = r(2);

    assert_eq!(&lhs + &rhs, lhs.clone() + rhs.clone());
    assert_eq!(&lhs - &rhs, lhs.clone() - rhs.clone());
    assert_eq!(-&lhs, -lhs.clone());
    assert_eq!(lhs.clone() + &scalar, lhs.clone() + scalar.clone());
    assert_eq!(lhs.clone() - &scalar, lhs.clone() - scalar.clone());
    assert_eq!(lhs.clone() * &scalar, lhs.clone() * scalar.clone());
    assert_eq!((lhs.clone() / &scalar).unwrap(), (lhs / scalar).unwrap());
}

#[test]
fn borrowed_matrix_ops_match_owned_ops() {
    let lhs = Matrix3::new([[r(1), r(2), r(3)], [r(0), r(1), r(4)], [r(5), r(6), r(0)]]);
    let rhs = Matrix3::new([[r(2), r(0), r(1)], [r(1), r(3), r(0)], [r(0), r(2), r(1)]]);
    let vector = Vector3::new([r(7), r(8), r(9)]);
    let scalar: Real = r(2);

    assert_eq!(&lhs + &rhs, lhs.clone() + rhs.clone());
    assert_eq!(&lhs - &rhs, lhs.clone() - rhs.clone());
    assert_eq!(&lhs * &rhs, lhs.clone() * rhs.clone());
    assert_eq!((&lhs / &rhs).unwrap(), (lhs.clone() / rhs.clone()).unwrap());
    assert_eq!(&lhs * &vector, lhs.clone() * vector.clone());
    assert_eq!(lhs.clone() + &scalar, lhs.clone() + scalar.clone());
    assert_eq!(lhs.clone() - &scalar, lhs.clone() - scalar.clone());
    assert_eq!(lhs.clone() * &scalar, lhs.clone() * scalar.clone());
    assert_eq!((lhs.clone() / &scalar).unwrap(), (lhs / scalar).unwrap());
}

#[test]
fn borrowed_complex_ops_match_owned_ops() {
    let lhs = Complex::new(r(3), r(4));
    let rhs = Complex::new(r(1), r(-2));
    let scalar: Real = frac(2, 1);

    assert_eq!(&lhs + &rhs, lhs.clone() + rhs.clone());
    assert_eq!(&lhs - &rhs, lhs.clone() - rhs.clone());
    assert_eq!(-&lhs, -lhs.clone());
    assert_eq!(&lhs * &rhs, lhs.clone() * rhs.clone());
    assert_eq!((&lhs / &rhs).unwrap(), (lhs.clone() / rhs).unwrap());
    assert_eq!((lhs.clone() / &scalar).unwrap(), (lhs / scalar).unwrap());
}
