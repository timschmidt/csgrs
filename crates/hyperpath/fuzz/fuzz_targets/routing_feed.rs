#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    AccelerationLimitedFeedProfileClass, LinePathSegment, RouteCertificationError,
    certify_acceleration_limited_feed_time, certify_constant_feed_time,
    certify_symmetric_jerk_limited_feed_time,
};
use hyperreal::{Rational, Real};
use libfuzzer_sys::fuzz_target;

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn positive(byte: u8, max: i64) -> i64 {
    i64::from(byte % u8::try_from(max).unwrap()) + 1
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }

    let feed = positive(data[0], 20);
    let time = positive(data[1], 20);
    let route = vec![LinePathSegment::new(p(0, 0), p(feed * time, 0))];
    let constant =
        certify_constant_feed_time(&route, r(feed), r(time), PredicatePolicy::default()).unwrap();
    assert!(constant.certification.all_satisfied());

    let accel_scale = positive(data[2], 8);
    let triangular_time = positive(data[3], 20);
    let acceleration = 4 * accel_scale;
    let triangular_length = accel_scale * triangular_time * triangular_time;
    let triangular_route = vec![LinePathSegment::new(p(0, 0), p(triangular_length, 0))];
    let triangular = certify_acceleration_limited_feed_time(
        &triangular_route,
        r(acceleration * triangular_time),
        r(acceleration),
        r(triangular_time),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        triangular.profile,
        AccelerationLimitedFeedProfileClass::Triangular
    );
    assert!(triangular.certification.all_satisfied());

    let max_feed = positive(data[4], 20);
    let cruise_time = positive(data[5], 20);
    let trapezoid_length = max_feed * max_feed + max_feed * cruise_time;
    let trapezoid_time = 2 * max_feed + cruise_time;
    let trapezoid_route = vec![LinePathSegment::new(p(0, 0), p(trapezoid_length, 0))];
    let trapezoid = certify_acceleration_limited_feed_time(
        &trapezoid_route,
        r(max_feed),
        r(1),
        r(trapezoid_time),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        trapezoid.profile,
        AccelerationLimitedFeedProfileClass::Trapezoidal
    );
    assert!(trapezoid.certification.all_satisfied());

    let diagonal = vec![LinePathSegment::new(p(0, 0), p(3, 4))];
    assert_eq!(
        certify_acceleration_limited_feed_time(
            &diagonal,
            r(max_feed),
            r(1),
            r(trapezoid_time),
            PredicatePolicy::default(),
        )
        .unwrap_err(),
        RouteCertificationError::UnsupportedRouteGeometry
    );

    let jerk = positive(data[6], 8);
    let quarter_time = positive(data[7], 16);
    let jerk_length = 2 * jerk * quarter_time * quarter_time * quarter_time;
    let jerk_time = 4 * quarter_time;
    let jerk_route = vec![LinePathSegment::new(p(0, 0), p(jerk_length, 0))];
    let jerk_report = certify_symmetric_jerk_limited_feed_time(
        &jerk_route,
        r(jerk * quarter_time * quarter_time),
        r(jerk * quarter_time),
        r(jerk),
        r(jerk_time),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(jerk_report.certification.all_satisfied());

    let limited_report = certify_symmetric_jerk_limited_feed_time(
        &jerk_route,
        r(1),
        r(1),
        r(jerk),
        r(jerk_time),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(limited_report.certification.has_certified_violation());
});
