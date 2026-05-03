mod common;

use common::{abort_signal, frac, r, unknown_zero};
use realistic_blas::{Problem, Vector3, Vector4, one, zero};

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
    let vector = Vector3::new([frac(1, 2), r(2), frac(3, 4)]);

    #[cfg(feature = "hyperreal-backend")]
    assert_eq!(format!("{vector}"), "[1/2, 2, 3/4]");
    #[cfg(not(feature = "hyperreal-backend"))]
    assert_eq!(format!("{vector}"), "[0.5, 2, 0.75]");
    assert_eq!(format!("{vector:#}"), "[0.5, 2, 0.75]");
}

#[test]
fn checked_vector_operations_reject_zero_divisors() {
    let vector = Vector3::new([r(1), r(2), r(3)]);
    let zero_vector: Vector3 = Vector3::zero();

    assert_eq!(zero_vector.clone().normalize(), Err(Problem::DivideByZero));
    assert_eq!(vector.clone() / zero(), Err(Problem::DivideByZero));
    assert_eq!(zero_vector.normalize_checked(), Err(Problem::DivideByZero));
    assert_eq!(
        vector.clone().div_scalar_checked(zero()),
        Err(Problem::DivideByZero)
    );
    assert_eq!(
        vector.div_scalar_checked(r(2)).unwrap(),
        Vector3::new([frac(1, 2), r(1), frac(3, 2)])
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

    let unknown_vector = Vector3::new([unknown_zero(), r(0), r(0)]);
    assert_eq!(
        unknown_vector.normalize_checked_with_abort(&signal),
        Err(Problem::UnknownZero)
    );
}

#[cfg(not(feature = "hyperreal-backend"))]
#[test]
fn ordinary_vector_scalar_division_returns_unknown_zero() {
    let vector = Vector3::new([r(1), r(2), r(3)]);
    let divisor = realistic_blas::Scalar::approx(0.0, 0.25).unwrap();

    assert_eq!(vector / divisor, Err(Problem::UnknownZero));
}
