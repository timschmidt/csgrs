//! Minimal integer-coordinate orientation query with a printed policy outcome.

use hyperlimit::{Point2, Real, Sign, orient2};

/// Assert the triangle's positive orientation, then print the full decision outcome.
///
/// The assertion checks the value; the debug output also exposes its receipt.
fn main() {
    let a = Point2::new(Real::from(0), Real::from(0));
    let b = Point2::new(Real::from(1), Real::from(0));
    let c = Point2::new(Real::from(0), Real::from(1));

    let orientation = orient2(&a, &b, &c, hyperlimit::PredicatePolicy::APPROXIMATE_512);
    assert_eq!(orientation.value(), Some(Sign::Positive));
    println!("{orientation:?}");
}
