//! Runnable exact-integer vector and identity-matrix quick start.
//!
//! The README contains this whole file verbatim between its quick-start markers;
//! the integration test checks trimmed text equality. The four assertions exercise
//! one rational fixture, not arbitrary symbolic equality or predicate certainty.

use hyperlattice::{Matrix3, Real, Vector3};

/// Import a signed 32-bit integer exactly into Real, without a floating-point conversion.
fn r(value: i32) -> Real {
    value.into()
}

/// Check squared length 25, principal length 5, and identity action/inversion.
///
/// Builds (3, 4, 0), checks its self-dot product and square root, then checks a 3x3
/// identity matrix acting on it and inverting to itself. Assertions use structural
/// PartialEq for these exact rational values. Native scalar errors propagate via
/// BlasResult; this example adds no policy or certainty receipt and prints nothing.
fn main() -> hyperlattice::BlasResult<()> {
    let vector = Vector3::new([r(3), r(4), r(0)]);
    assert_eq!(vector.dot(&vector), r(25));
    assert_eq!(Real::sqrt(vector.dot(&vector))?, r(5));

    let identity = Matrix3::identity();
    assert_eq!(identity.clone() * vector.clone(), vector);
    assert_eq!(identity.inverse()?, Matrix3::identity());
    Ok(())
}
