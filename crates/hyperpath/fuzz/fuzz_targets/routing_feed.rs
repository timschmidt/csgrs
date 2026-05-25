#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    AccelerationLimitedFeedProfileClass, ArcDirection, ExplicitCircularArc, FeedPathElement,
    LinePathSegment, LookaheadFeedSchedule, RouteCertificationError, TangentSpan,
    certify_acceleration_limited_feed_time, certify_acceleration_limited_feed_time_for_path,
    certify_constant_feed_time, certify_constant_feed_time_for_path,
    certify_corner_lookahead_limits, certify_lookahead_feed_schedule,
    certify_symmetric_jerk_limited_feed_time, certify_symmetric_jerk_limited_feed_time_for_path,
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
    let mixed_constant = certify_constant_feed_time_for_path(
        &[FeedPathElement::Line(route[0].clone())],
        r(feed),
        r(time),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(mixed_constant.certification.all_satisfied());

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
    let triangular_mixed = certify_acceleration_limited_feed_time_for_path(
        &[FeedPathElement::Line(triangular_route[0].clone())],
        r(acceleration * triangular_time),
        r(acceleration),
        r(triangular_time),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(triangular_mixed.certification.all_satisfied());

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
    let radius = (r(2) / Real::pi()).unwrap();
    let arc = ExplicitCircularArc::new(
        p(0, 0),
        radius.clone(),
        Point2::new(radius.clone(), r(0)),
        Point2::new(-radius, r(0)),
        ArcDirection::Ccw,
    )
    .unwrap();
    let mixed_jerk_route = vec![
        FeedPathElement::Line(LinePathSegment::new(p(0, 0), p(jerk_length - 2, 0))),
        FeedPathElement::ExplicitArc(arc),
    ];
    let mixed_jerk_report = certify_symmetric_jerk_limited_feed_time_for_path(
        &mixed_jerk_route,
        r(jerk * quarter_time * quarter_time),
        r(jerk * quarter_time),
        r(jerk),
        r(jerk_time),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(mixed_jerk_report.certification.all_satisfied());

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

    let corner_feed = positive(data[0], 20);
    let corner_spans = vec![
        TangentSpan::from_line_segment(&LinePathSegment::new(p(0, 0), p(10, 0))),
        TangentSpan::from_line_segment(&LinePathSegment::new(p(10, 0), p(10, 10))),
    ];
    let corner_report = certify_corner_lookahead_limits(
        &corner_spans,
        r(corner_feed),
        r(corner_feed),
        r(corner_feed * corner_feed),
        r(1),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(corner_report.all_satisfied());

    let reversal_spans = vec![
        TangentSpan::from_line_segment(&LinePathSegment::new(p(0, 0), p(5, 0))),
        TangentSpan::from_line_segment(&LinePathSegment::new(p(5, 0), p(0, 0))),
    ];
    let reversal_report = certify_corner_lookahead_limits(
        &reversal_spans,
        Real::zero(),
        r(1),
        r(1),
        r(1),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(reversal_report.all_satisfied());

    let schedule_line = LinePathSegment::new(p(0, 0), p(corner_feed * corner_feed, 0));
    let schedule_route = vec![FeedPathElement::Line(schedule_line.clone())];
    let schedule_spans = vec![TangentSpan::from_line_segment(&schedule_line)];
    let schedule = LookaheadFeedSchedule {
        entry_feed: Real::zero(),
        corner_feeds: vec![],
        corner_radii: vec![],
        exit_feed: r(corner_feed),
    };
    let schedule_report = certify_lookahead_feed_schedule(
        &schedule_route,
        &schedule_spans,
        &schedule,
        r(corner_feed),
        r(1),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(schedule_report.all_satisfied());
});
