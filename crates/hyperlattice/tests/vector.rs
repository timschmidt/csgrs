mod common;

use common::{r, unknown_zero};
use realistic_blas::{BlasProblem, Problem, Rational, Vector3, Vector4, one, zero};

#[test]
fn vector_dot_and_normalize() {
    let v = Vector3::new([r(3), r(4), r(0)]);
    assert_eq!(v.dot(&v), r(25));

    let normalized = v.normalize().unwrap();
    assert_eq!(normalized.dot(&normalized), one());
}

#[test]
fn vector_scalar_add_and_subtract_are_componentwise() {
    let vector = Vector4::new([r(1), r(2), r(3), r(4)]);

    assert_eq!(
        vector.clone() + r(10),
        Vector4::new([r(11), r(12), r(13), r(14)])
    );
    assert_eq!(vector - r(1), Vector4::new([r(0), r(1), r(2), r(3)]));
}

#[test]
fn checked_vector_operations_reject_zero_divisors() {
    let vector = Vector3::new([r(1), r(2), r(3)]);

    assert_eq!(Vector3::zero().normalize(), Err(Problem::DivideByZero));
    assert_eq!(vector.clone() / zero(), Err(Problem::DivideByZero));
    assert_eq!(
        Vector3::zero().normalize_checked(),
        Err(BlasProblem::Real(Problem::DivideByZero))
    );
    assert_eq!(
        vector.clone().div_scalar_checked(zero()),
        Err(BlasProblem::Real(Problem::DivideByZero))
    );
    assert_eq!(
        vector.div_scalar_checked(r(2)).unwrap(),
        Vector3::new([
            Rational::fraction(1, 2).unwrap().into(),
            r(1),
            Rational::fraction(3, 2).unwrap().into(),
        ])
    );
}

#[test]
fn checked_vector_operations_reject_unknown_zero_divisors() {
    let vector = Vector3::new([r(1), r(2), r(3)]);

    assert_eq!(
        vector.div_scalar_checked(unknown_zero()),
        Err(BlasProblem::UnknownZero)
    );
}
