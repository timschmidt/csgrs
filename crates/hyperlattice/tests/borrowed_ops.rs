//! Finite ownership-form agreement tests for scalar, vector, matrix and complex operations.
//!
//! Each test compares wrappers using the same native implementation on selected
//! rational fixtures. These structural-equality checks do not independently prove
//! all arithmetic, identity of dispatch/cache behavior, or a performance claim.

/// Shared exact integer and fraction constructors for the ownership comparisons.
mod common;

use common::{frac, r};
use hyperlattice::{Complex, Matrix3, Real, Vector3};

/// Compare borrowed/owned addition, subtraction, multiplication and valid division for scalars six and three.
#[test]
fn borrowed_scalar_ops_match_owned_ops() {
    let a = r(6);
    let b = r(3);

    assert_eq!(&a + &b, a.clone() + b.clone());
    assert_eq!(&a - &b, a.clone() - b.clone());
    assert_eq!(&a * &b, a.clone() * b.clone());
    assert_eq!((&a / &b).unwrap(), (a / b).unwrap());
}

/// Compare selected Vector3 ownership forms for addition, subtraction, negation and scalar operations; results use structural equality on rational fixtures.
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

/// Compare selected Matrix3 ownership forms for matrix arithmetic, matrix-vector multiplication and scalar operations on fixed invertible rational data.
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

/// Compare selected complex ownership forms for arithmetic, negation and scalar division on fixed rational inputs; no independent branch or zero-domain expectations are supplied.
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
