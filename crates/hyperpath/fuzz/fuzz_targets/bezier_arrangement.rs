#![no_main]

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperpath::{
    BezierParameter, CubicBezier, LineCubicAlgebraicPointDomain, LineCubicAlgebraicRootDomain,
    LineCubicBezierAlgebraicBreakpointDomain, LineCubicBezierIntersectionClass, LinePathSegment,
    LineQuadraticBezierIntersectionClass, LineRationalQuadraticBezierIntersectionClass,
    QuadraticBezier, RationalQuadraticBezier, arrange_cubic_beziers,
    arrange_line_segments_with_cubic_beziers, arrange_line_segments_with_quadratic_beziers,
    arrange_line_segments_with_rational_quadratic_beziers, arrange_quadratic_beziers,
    arrange_rational_quadratic_beziers, intersect_axis_aligned_line_cubic_bezier,
    intersect_axis_aligned_line_quadratic_bezier,
    intersect_axis_aligned_line_rational_quadratic_bezier,
};
use hyperreal::{Rational, Real};
use hypersolve::AlgebraicRootPolynomialImageStatus;
use libfuzzer_sys::fuzz_target;

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn p(x: i64, y: i64) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(r(x), r(y))
}

fn pq(x_num: i64, x_den: i64, y_num: i64, y_den: i64) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(
        Real::new(Rational::new(x_num) / Rational::new(x_den)),
        Real::new(Rational::new(y_num) / Rational::new(y_den)),
    )
}

fn signed(byte: u8) -> i64 {
    i64::from(i8::from_ne_bytes([byte]))
}

fn parameter(byte: u8) -> BezierParameter {
    BezierParameter::new(i64::from(byte % 9) + 1, 10).unwrap()
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 18 {
        return;
    }

    let t = parameter(data[0]);
    let quadratic = QuadraticBezier::new(
        p(signed(data[1]), signed(data[2])),
        p(signed(data[3]), signed(data[4])),
        p(signed(data[5]), signed(data[6])),
    );
    let q_report =
        arrange_quadratic_beziers(&[quadratic.clone()], &[vec![t]], PredicatePolicy::default())
            .unwrap();
    assert_eq!(q_report.fragments.len(), 2);
    assert_eq!(q_report.fragments[0].curve.start(), quadratic.start());
    assert_eq!(q_report.fragments[0].curve.end(), &quadratic.eval(t));
    assert_eq!(q_report.fragments[1].curve.start(), &quadratic.eval(t));
    assert_eq!(q_report.fragments[1].curve.end(), quadratic.end());

    let horizontal = LinePathSegment::new(
        p(signed(data[1]), signed(data[2])),
        p(signed(data[5]), signed(data[2])),
    );
    let intersection_report = intersect_axis_aligned_line_quadratic_bezier(
        &horizontal,
        &quadratic,
        PredicatePolicy::default(),
    );
    for event in &intersection_report.intersections {
        assert_eq!(
            compare_reals_with_policy(
                &event.point.y,
                &horizontal.start().y,
                PredicatePolicy::default()
            )
            .value(),
            Some(Ordering::Equal)
        );
    }
    let mixed_report = arrange_line_segments_with_quadratic_beziers(
        std::slice::from_ref(&horizontal),
        std::slice::from_ref(&quadratic),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(mixed_report.events.len(), 1);
    for window in mixed_report.bezier_breakpoints[0].windows(2) {
        assert!(
            compare_reals_with_policy(
                &window[0].parameter,
                &window[1].parameter,
                PredicatePolicy::default()
            )
            .value()
            .is_some()
        );
    }

    let overlap_curve = QuadraticBezier::new(p(0, 0), p(4, 0), p(8, 0));
    let overlap_line = LinePathSegment::new(p(2, 0), p(6, 0));
    let overlap_report = arrange_line_segments_with_quadratic_beziers(
        &[overlap_line],
        &[overlap_curve],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        overlap_report.events[0].class,
        LineQuadraticBezierIntersectionClass::Overlap
    );
    assert_eq!(overlap_report.bezier_breakpoints[0].len(), 4);
    let nonlinear_overlap_curve = QuadraticBezier::new(p(0, 0), p(2, 0), p(8, 0));
    let nonlinear_overlap_line = LinePathSegment::new(p(2, 0), p(6, 0));
    let nonlinear_overlap_report = arrange_line_segments_with_quadratic_beziers(
        &[nonlinear_overlap_line],
        &[nonlinear_overlap_curve],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        nonlinear_overlap_report.events[0].class,
        LineQuadraticBezierIntersectionClass::Overlap
    );
    assert_eq!(nonlinear_overlap_report.bezier_breakpoints[0].len(), 4);

    let cubic = CubicBezier::new(
        p(signed(data[1]), signed(data[2])),
        p(signed(data[7]), signed(data[8])),
        p(signed(data[9]), signed(data[10])),
        p(signed(data[5]), signed(data[6])),
    );
    let c_report =
        arrange_cubic_beziers(&[cubic.clone()], &[vec![t]], PredicatePolicy::default()).unwrap();
    assert_eq!(c_report.fragments.len(), 2);
    assert_eq!(c_report.fragments[0].curve.start(), cubic.start());
    assert_eq!(c_report.fragments[0].curve.end(), &cubic.eval(t));
    assert_eq!(c_report.fragments[1].curve.start(), &cubic.eval(t));
    assert_eq!(c_report.fragments[1].curve.end(), cubic.end());

    let reducible_cubic = CubicBezier::new(p(0, 0), pq(8, 3, 4, 1), pq(16, 3, 4, 1), p(8, 0));
    let cubic_secant_line = LinePathSegment::new(pq(0, 1, 9, 4), pq(8, 1, 9, 4));
    let cubic_intersection_report = intersect_axis_aligned_line_cubic_bezier(
        &cubic_secant_line,
        &reducible_cubic,
        PredicatePolicy::default(),
    );
    for event in &cubic_intersection_report.intersections {
        assert_eq!(
            compare_reals_with_policy(
                &event.point.y,
                &cubic_secant_line.start().y,
                PredicatePolicy::default()
            )
            .value(),
            Some(Ordering::Equal)
        );
    }
    let cubic_mixed_report = arrange_line_segments_with_cubic_beziers(
        &[cubic_secant_line],
        &[reducible_cubic],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        cubic_mixed_report.events[0].class,
        LineCubicBezierIntersectionClass::TwoPoints
    );
    assert_eq!(cubic_mixed_report.cubic_fragments.len(), 3);

    let cubic_overlap_curve = CubicBezier::new(p(0, 0), pq(8, 3, 0, 1), pq(16, 3, 0, 1), p(8, 0));
    let cubic_overlap_line = LinePathSegment::new(p(2, 0), p(6, 0));
    let cubic_overlap_report = arrange_line_segments_with_cubic_beziers(
        &[cubic_overlap_line],
        &[cubic_overlap_curve],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        cubic_overlap_report.events[0].class,
        LineCubicBezierIntersectionClass::Overlap
    );
    assert_eq!(cubic_overlap_report.cubic_breakpoints[0].len(), 4);
    let algebraic_cubic = CubicBezier::new(p(0, 0), pq(1, 3, 0, 1), pq(2, 3, 0, 1), p(1, 1));
    let algebraic_line = LinePathSegment::new(pq(0, 1, 1, 8), pq(1, 1, 1, 8));
    let algebraic_report = intersect_axis_aligned_line_cubic_bezier(
        &algebraic_line,
        &algebraic_cubic,
        PredicatePolicy::default(),
    );
    assert_eq!(
        algebraic_report.class,
        LineCubicBezierIntersectionClass::Unknown
    );
    assert_eq!(algebraic_report.algebraic_support_roots.len(), 1);
    assert_eq!(
        algebraic_report.algebraic_support_roots[0].parameter_domain,
        LineCubicAlgebraicRootDomain::InsideUnitInterval
    );
    assert_eq!(
        &algebraic_report.algebraic_support_roots[0]
            .point_image
            .x
            .status,
        &AlgebraicRootPolynomialImageStatus::Transformed
    );
    assert_eq!(
        &algebraic_report.algebraic_support_roots[0]
            .point_image
            .y
            .status,
        &AlgebraicRootPolynomialImageStatus::Transformed
    );
    assert_eq!(
        algebraic_report.algebraic_support_roots[0]
            .point_image
            .segment_domain,
        LineCubicAlgebraicPointDomain::InsideSegmentBounds
    );
    let algebraic_mixed_report = arrange_line_segments_with_cubic_beziers(
        &[algebraic_line],
        &[algebraic_cubic],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(algebraic_mixed_report.algebraic_breakpoints.len(), 1);
    assert_eq!(
        algebraic_mixed_report.algebraic_breakpoints[0].domain,
        LineCubicBezierAlgebraicBreakpointDomain::InsideLineAndCurve
    );
    assert_eq!(
        &algebraic_mixed_report.algebraic_breakpoints[0]
            .line_parameter
            .status,
        &AlgebraicRootPolynomialImageStatus::Transformed
    );

    let weight = r(i64::from(data[11] % 16));
    let conic = RationalQuadraticBezier::new(
        p(signed(data[12]), signed(data[13])),
        p(signed(data[14]), signed(data[15])),
        p(signed(data[16]), signed(data[17])),
        weight,
    )
    .unwrap();
    let conic_horizontal = LinePathSegment::new(
        p(signed(data[12]), signed(data[13])),
        p(signed(data[16]), signed(data[13])),
    );
    let conic_intersection_report = intersect_axis_aligned_line_rational_quadratic_bezier(
        &conic_horizontal,
        &conic,
        PredicatePolicy::default(),
    );
    for event in &conic_intersection_report.intersections {
        assert_eq!(
            compare_reals_with_policy(
                &event.point.y,
                &conic_horizontal.start().y,
                PredicatePolicy::default()
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    let secant_conic = RationalQuadraticBezier::new(p(0, 0), p(4, 8), p(8, 0), r(1)).unwrap();
    let secant_line = LinePathSegment::new(p(0, 3), p(8, 3));
    let secant_report = arrange_line_segments_with_rational_quadratic_beziers(
        &[secant_line],
        &[secant_conic],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        secant_report.events[0].class,
        LineRationalQuadraticBezierIntersectionClass::TwoPoints
    );
    assert_eq!(secant_report.conic_breakpoints[0].len(), 4);
    assert_eq!(secant_report.conic_fragments.len(), 3);

    let tangent_conic = RationalQuadraticBezier::new(p(0, 0), p(4, 4), p(8, 0), r(1)).unwrap();
    let tangent_line = LinePathSegment::new(p(0, 2), p(8, 2));
    let tangent_report = arrange_line_segments_with_rational_quadratic_beziers(
        &[tangent_line],
        &[tangent_conic],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        tangent_report.events[0].class,
        LineRationalQuadraticBezierIntersectionClass::Tangent
    );
    assert_eq!(tangent_report.conic_breakpoints[0].len(), 3);

    let overlap_conic = RationalQuadraticBezier::new(p(0, 0), p(4, 0), p(8, 0), r(2)).unwrap();
    let overlap_line = LinePathSegment::new(
        hyperlimit::Point2::new(Real::new(Rational::new(28) / Rational::new(11)), r(0)),
        hyperlimit::Point2::new(Real::new(Rational::new(60) / Rational::new(11)), r(0)),
    );
    let overlap_report = arrange_line_segments_with_rational_quadratic_beziers(
        &[overlap_line],
        &[overlap_conic],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        overlap_report.events[0].class,
        LineRationalQuadraticBezierIntersectionClass::Overlap
    );
    assert_eq!(overlap_report.conic_breakpoints[0].len(), 4);

    let nonmonotone_conic = RationalQuadraticBezier::new(p(0, 0), p(8, 0), p(0, 0), r(1)).unwrap();
    let nonmonotone_report = arrange_line_segments_with_rational_quadratic_beziers(
        &[LinePathSegment::new(p(2, 0), p(6, 0))],
        &[nonmonotone_conic],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        nonmonotone_report.events[0].class,
        LineRationalQuadraticBezierIntersectionClass::Unknown
    );

    let r_report =
        arrange_rational_quadratic_beziers(&[conic], &[vec![t]], PredicatePolicy::default())
            .unwrap();
    assert_eq!(r_report.fragments.len(), 2);
    assert_eq!(
        r_report.fragments[0].end_control,
        r_report.fragments[1].start_control
    );
});
