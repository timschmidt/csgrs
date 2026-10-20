#![allow(dead_code)]
use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::Real;
use hyperspatial::*;
use std::sync::Arc;

/// An exact zero that no retained scalar relation proves. The scalar layer
/// proves sin(a)^2 + cos(a)^2 = 1, so tests of unresolved decisions use the
/// angle-addition identity sin(e + 1) - sin(e) cos(1) - cos(e) sin(1).
pub fn unresolved_zero() -> Real {
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
}
pub fn context() -> Context {
    Context::new(PredicatePolicy::STRICT, Limits::default())
}
pub fn r(value: &str) -> Real {
    value.parse().unwrap()
}
pub fn input() -> InputProvenance {
    InputProvenance::new("analytical fixture", InputAccuracy::ExactModel)
}
pub fn reference() -> ReferenceFrame {
    ReferenceFrame::fixed("fixture:static-earth").unwrap()
}
pub fn position(
    latitude: Real,
    longitude: Real,
    height: Real,
    ellipsoid: Arc<Ellipsoid>,
    ctx: &mut Context,
) -> GeodeticPosition {
    let latitude = Latitude::new(Angle::degrees(latitude), ctx).unwrap();
    GeodeticPosition::new(
        latitude,
        Longitude::unrolled(Angle::degrees(longitude)),
        Length::metres(height),
        ellipsoid,
        reference(),
        input(),
        ctx,
    )
    .unwrap()
}
pub fn equal(left: &Real, right: &Real) {
    let outcome = compare_reals(left, right, PredicatePolicy::STRICT);
    assert_eq!(
        outcome.value(),
        Some(std::cmp::Ordering::Equal),
        "{outcome:?}: {left} versus {right}"
    );
}
pub fn rounded(actual: &Real, expected: &str, half_unit: &str) {
    let expected = r(expected);
    let half_unit = r(half_unit);
    let lower = compare_reals(actual, &(&expected - &half_unit), PredicatePolicy::STRICT)
        .value()
        .unwrap();
    let upper = compare_reals(actual, &(&expected + &half_unit), PredicatePolicy::STRICT)
        .value()
        .unwrap();
    assert_ne!(
        lower,
        std::cmp::Ordering::Less,
        "{actual} below rounded fixture {expected}"
    );
    assert_ne!(
        upper,
        std::cmp::Ordering::Greater,
        "{actual} above rounded fixture {expected}"
    );
}
