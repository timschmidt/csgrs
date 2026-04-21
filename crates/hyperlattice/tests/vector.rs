mod common;

use common::{abort_signal, r, unknown_zero};
use realistic_blas::{Problem, Rational, Vector3, Vector4, one, zero};

#[test]
fn vector_dot_and_normalize() {
    let v = Vector3::new([r(3), r(4), r(0)]);
    assert_eq!(v.dot(&v), r(25));

    let normalized = v.normalize().unwrap();
    assert_eq!(normalized.dot(&normalized), one());

    let signal = abort_signal();
    assert_eq!(v.dot_with_abort(&v, &signal), r(25));
    assert_eq!(v.magnitude_with_abort(&signal).unwrap(), r(5));
    assert_eq!(
        v.normalize_checked_with_abort(&signal)
            .unwrap()
            .dot(&normalized),
        one()
    );
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
fn vector_display_forwards_real_formatting() {
    let vector = Vector3::new([
        Rational::fraction(1, 2).unwrap().into(),
        r(2),
        Rational::fraction(3, 4).unwrap().into(),
    ]);

    #[cfg(feature = "realistic-backend")]
    assert_eq!(format!("{vector}"), "[1/2, 2, 3/4]");
    #[cfg(not(feature = "realistic-backend"))]
    assert_eq!(format!("{vector}"), "[0.5, 2, 0.75]");
    assert_eq!(format!("{vector:#}"), "[0.5, 2, 0.75]");
}

#[test]
fn checked_vector_operations_reject_zero_divisors() {
    let vector = Vector3::new([r(1), r(2), r(3)]);

    assert_eq!(Vector3::zero().normalize(), Err(Problem::DivideByZero));
    assert_eq!(vector.clone() / zero(), Err(Problem::DivideByZero));
    assert_eq!(
        Vector3::zero().normalize_checked(),
        Err(Problem::DivideByZero)
    );
    assert_eq!(
        vector.clone().div_scalar_checked(zero()),
        Err(Problem::DivideByZero)
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
    let signal = abort_signal();

    assert_eq!(
        vector.clone().div_scalar_checked(unknown_zero()),
        Err(Problem::UnknownZero)
    );
    assert_eq!(
        vector.div_scalar_checked_with_abort(unknown_zero(), &signal),
        Err(Problem::UnknownZero)
    );
}
