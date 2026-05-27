#![no_main]

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperpath::{
    ArcDirection, BezierParameter, CubicBezier, CurveArrangementCellFaceClass,
    CurveArrangementLoopRoleClass,
    ExplicitCircularArc, LineCubicAlgebraicPointDomain, LineCubicAlgebraicRootDomain,
    LineCubicBezierAlgebraicBreakpointDomain,
    LineCubicBezierAlgebraicBreakpointOrderClass, LineCubicBezierAlgebraicBreakpointSequenceClass,
    LineCubicBezierAlgebraicOverlapBreakpointDomain,
    LineCubicBezierAlgebraicOverlapBreakpointSequenceClass,
    LineCubicBezierAlgebraicOverlapBreakpointSequenceSource, LineCubicBezierIntersectionClass,
    LineCubicBezierSupportOverlapMonotonicity, LineExplicitArcIntersectionClass,
    LineMixedBezierArrangementError, LinePathSegment, LineQuadraticBezierIntersectionClass,
    LineRationalQuadraticBezierAlgebraicBreakpointDomain,
    LineRationalQuadraticBezierAlgebraicBreakpointOrderClass,
    LineRationalQuadraticBezierAlgebraicBreakpointSequenceClass,
    LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource,
    LineRationalQuadraticBezierIntersectionClass, LineRationalQuadraticBezierInverseRootDomain,
    LineRationalQuadraticBezierSupportOverlapMonotonicity, MixedCurveEndpointTangentClass,
    MixedCurveFragmentEndpoint, MixedCurveFragmentRef, MixedCurveFragmentSeparationClass,
    MixedCurveSourceRef, QuadraticBezier, RationalQuadraticBezier, arrange_cubic_beziers,
    arrange_explicit_arcs, arrange_line_segments_with_cubic_beziers,
    arrange_line_segments_with_explicit_arcs, arrange_line_segments_with_mixed_beziers,
    arrange_line_segments_with_mixed_curves, arrange_line_segments_with_quadratic_beziers,
    arrange_line_segments_with_rational_quadratic_beziers, arrange_quadratic_beziers,
    arrange_rational_quadratic_beziers, intersect_axis_aligned_line_cubic_bezier,
    intersect_axis_aligned_line_quadratic_bezier,
    intersect_axis_aligned_line_rational_quadratic_bezier, intersect_line_cubic_bezier,
    intersect_line_quadratic_bezier, intersect_line_rational_quadratic_bezier,
};
use hyperreal::{Rational, Real};
use hypersolve::AlgebraicRootPolynomialImageStatus;
use libfuzzer_sys::fuzz_target;

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn rq(numerator: i64, denominator: i64) -> Real {
    Real::new(Rational::new(numerator) / Rational::new(denominator))
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
    assert_eq!(
        q_report.cell_graph.half_edges.len(),
        q_report.cell_graph.edges.len() * 2
    );
    assert!(q_report.cell_graph.faces.is_empty());

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
    assert_eq!(
        mixed_report.cell_graph.half_edges.len(),
        mixed_report.cell_graph.edges.len() * 2
    );
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
    assert_eq!(
        overlap_report.cell_graph.half_edges.len(),
        overlap_report.cell_graph.edges.len() * 2
    );
    let closed_curve = QuadraticBezier::new(p(0, 0), p(4, 8), p(8, 0));
    let closed_line = LinePathSegment::new(p(0, 0), p(8, 0));
    let closed_report = arrange_line_segments_with_quadratic_beziers(
        &[closed_line],
        &[closed_curve],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(closed_report.cell_graph.faces.iter().any(|face| {
        face.class == CurveArrangementCellFaceClass::Bounded
            && face.signed_area_twice
                == Real::new(Rational::new(64) / Rational::new(3))
    }));
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
    let general_nonlinear_overlap_curve = QuadraticBezier::new(p(0, 0), p(1, 1), p(3, 3));
    let general_nonlinear_overlap_line =
        LinePathSegment::new(pq(9, 16, 9, 16), pq(33, 16, 33, 16));
    let general_nonlinear_overlap_report = arrange_line_segments_with_quadratic_beziers(
        &[general_nonlinear_overlap_line],
        &[general_nonlinear_overlap_curve],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        general_nonlinear_overlap_report.events[0].class,
        LineQuadraticBezierIntersectionClass::Overlap
    );
    assert_eq!(
        general_nonlinear_overlap_report.events[0].intersection.intersections[0].parameter,
        rq(1, 4)
    );
    assert_eq!(
        general_nonlinear_overlap_report.events[0].intersection.intersections[1].parameter,
        rq(3, 4)
    );
    assert_eq!(
        general_nonlinear_overlap_report.bezier_breakpoints[0].len(),
        4
    );
    let general_nonmonotone_overlap_curve = QuadraticBezier::new(p(0, 0), p(4, 4), p(0, 0));
    let general_nonmonotone_overlap_line = LinePathSegment::new(p(1, 1), p(3, 3));
    let general_nonmonotone_overlap_report = arrange_line_segments_with_quadratic_beziers(
        &[general_nonmonotone_overlap_line],
        &[general_nonmonotone_overlap_curve],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        general_nonmonotone_overlap_report.events[0].class,
        LineQuadraticBezierIntersectionClass::Unknown
    );
    assert!(general_nonmonotone_overlap_report.events[0]
        .intersection
        .intersections
        .is_empty());
    assert_eq!(
        general_nonmonotone_overlap_report.bezier_breakpoints[0].len(),
        2
    );

    let diagonal_curve = QuadraticBezier::new(p(0, 0), p(2, 4), p(4, 0));
    let diagonal_line = LinePathSegment::new(p(0, 1), p(4, 3));
    let diagonal_intersection =
        intersect_line_quadratic_bezier(&diagonal_line, &diagonal_curve, PredicatePolicy::default());
    assert_eq!(
        diagonal_intersection.class,
        LineQuadraticBezierIntersectionClass::TwoPoints
    );
    assert_eq!(diagonal_intersection.intersections[0].parameter, rq(1, 4));
    assert_eq!(diagonal_intersection.intersections[1].parameter, rq(1, 2));
    let diagonal_report = arrange_line_segments_with_quadratic_beziers(
        &[diagonal_line],
        &[diagonal_curve],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        diagonal_report.events[0].class,
        LineQuadraticBezierIntersectionClass::TwoPoints
    );
    assert_eq!(diagonal_report.line_breakpoints[0].len(), 4);
    assert_eq!(diagonal_report.bezier_breakpoints[0].len(), 4);

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
    assert_eq!(
        c_report.cell_graph.half_edges.len(),
        c_report.cell_graph.edges.len() * 2
    );
    assert!(c_report.cell_graph.faces.is_empty());

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
    let diagonal_cubic_line = LinePathSegment::new(p(0, 1), p(8, 5));
    let diagonal_cubic_report = intersect_line_cubic_bezier(
        &diagonal_cubic_line,
        &reducible_cubic,
        PredicatePolicy::default(),
    );
    assert_eq!(
        diagonal_cubic_report.class,
        LineCubicBezierIntersectionClass::TwoPoints
    );
    assert_eq!(diagonal_cubic_report.intersections[0].parameter, rq(1, 6));
    assert_eq!(diagonal_cubic_report.intersections[1].parameter, rq(1, 2));
    let diagonal_cubic_arrangement = arrange_line_segments_with_cubic_beziers(
        &[diagonal_cubic_line],
        &[reducible_cubic.clone()],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        diagonal_cubic_arrangement.events[0].class,
        LineCubicBezierIntersectionClass::TwoPoints
    );
    assert_eq!(diagonal_cubic_arrangement.line_breakpoints[0].len(), 4);
    assert_eq!(diagonal_cubic_arrangement.cubic_breakpoints[0].len(), 4);
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
    assert_eq!(
        cubic_mixed_report.cell_graph.half_edges.len(),
        cubic_mixed_report.cell_graph.edges.len() * 2
    );
    let cubic_cell_curve = CubicBezier::new(p(0, 0), p(0, 4), p(8, 4), p(8, 0));
    let cubic_cell_line = LinePathSegment::new(p(0, 0), p(8, 0));
    let cubic_cell_report = arrange_line_segments_with_cubic_beziers(
        &[cubic_cell_line],
        &[cubic_cell_curve],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(cubic_cell_report.cell_graph.faces.iter().any(|face| {
        face.class == CurveArrangementCellFaceClass::Bounded
            && face.signed_area_twice
                == Real::new(Rational::new(192) / Rational::new(5))
    }));

    let q_upper = QuadraticBezier::new(p(0, 0), p(4, 8), p(8, 0));
    let q_lower = QuadraticBezier::new(p(8, 0), p(4, -8), p(0, 0));
    let q_loop_report = arrange_quadratic_beziers(
        &[q_upper, q_lower],
        &[vec![], vec![]],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(q_loop_report.cell_graph.vertices.len(), 2);
    assert_eq!(q_loop_report.cell_graph.edges.len(), 2);
    assert_eq!(q_loop_report.cell_graph.faces.len(), 2);
    assert!(q_loop_report
        .cell_graph
        .loop_roles
        .iter()
        .any(|role| role.class == CurveArrangementLoopRoleClass::Material
            && role.containment_depth == Some(0)
            && role.representative.is_some()));
    let nested_q_report = arrange_quadratic_beziers(
        &[
            QuadraticBezier::new(p(0, 0), p(4, 8), p(8, 0)),
            QuadraticBezier::new(p(8, 0), p(4, -8), p(0, 0)),
            QuadraticBezier::new(p(2, 0), p(4, 3), p(6, 0)),
            QuadraticBezier::new(p(6, 0), p(4, -3), p(2, 0)),
        ],
        &[vec![], vec![], vec![], vec![]],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(nested_q_report
        .cell_graph
        .loop_roles
        .iter()
        .any(|role| role.class == CurveArrangementLoopRoleClass::Hole
            && role.containment_depth == Some(1)));
    let nested_arc_report = arrange_explicit_arcs(
        &[
            ExplicitCircularArc::new(p(4, 0), r(4), p(0, 0), p(8, 0), ArcDirection::Cw).unwrap(),
            ExplicitCircularArc::new(p(4, 0), r(4), p(8, 0), p(0, 0), ArcDirection::Cw).unwrap(),
            ExplicitCircularArc::new(p(4, 0), r(2), p(2, 0), p(6, 0), ArcDirection::Cw).unwrap(),
            ExplicitCircularArc::new(p(4, 0), r(2), p(6, 0), p(2, 0), ArcDirection::Cw).unwrap(),
        ],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(nested_arc_report
        .cell_graph
        .loop_roles
        .iter()
        .any(|role| role.class == CurveArrangementLoopRoleClass::Hole
            && role.containment_depth == Some(1)
            && role.representative.is_some()));

    let c_upper = CubicBezier::new(p(0, 0), p(0, 4), p(8, 4), p(8, 0));
    let c_lower = CubicBezier::new(p(8, 0), p(8, -4), p(0, -4), p(0, 0));
    let c_loop_report =
        arrange_cubic_beziers(&[c_upper, c_lower], &[vec![], vec![]], PredicatePolicy::default())
            .unwrap();
    assert_eq!(c_loop_report.cell_graph.vertices.len(), 2);
    assert_eq!(c_loop_report.cell_graph.edges.len(), 2);
    assert_eq!(c_loop_report.cell_graph.faces.len(), 2);
    let nested_true_cubic_report = arrange_cubic_beziers(
        &[
            CubicBezier::new(p(0, 0), p(0, 6), p(8, 2), p(8, 0)),
            CubicBezier::new(p(8, 0), p(8, -6), p(0, -2), p(0, 0)),
            CubicBezier::new(p(2, 0), p(2, 2), p(6, 1), p(6, 0)),
            CubicBezier::new(p(6, 0), p(6, -2), p(2, -1), p(2, 0)),
        ],
        &[vec![], vec![], vec![], vec![]],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(nested_true_cubic_report
        .cell_graph
        .loop_roles
        .iter()
        .any(|role| role.class == CurveArrangementLoopRoleClass::Hole
            && role.containment_depth == Some(1)
            && role.representative.is_some()));
    let duplicate_quadratic_report = arrange_quadratic_beziers(
        &[
            QuadraticBezier::new(p(0, 0), p(4, 8), p(8, 0)),
            QuadraticBezier::new(p(8, 0), p(4, 8), p(0, 0)),
        ],
        &[vec![], vec![]],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(duplicate_quadratic_report.cell_graph.edges.len(), 1);
    assert_eq!(
        duplicate_quadratic_report.cell_graph.edges[0].fragments.len(),
        2
    );
    let duplicate_cubic_report = arrange_cubic_beziers(
        &[
            CubicBezier::new(p(0, 0), p(2, 8), p(6, 8), p(8, 0)),
            CubicBezier::new(p(8, 0), p(6, 8), p(2, 8), p(0, 0)),
        ],
        &[vec![], vec![]],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(duplicate_cubic_report.cell_graph.edges.len(), 1);
    assert_eq!(duplicate_cubic_report.cell_graph.edges[0].fragments.len(), 2);
    let tangent_quadratic_report = arrange_quadratic_beziers(
        &[
            QuadraticBezier::new(p(-1, 0), p(0, 1), p(1, 0)),
            QuadraticBezier::new(p(1, 0), p(0, -1), p(-1, 0)),
            QuadraticBezier::new(p(3, 1), p(4, 3), p(5, 1)),
            QuadraticBezier::new(p(5, 1), p(4, -1), p(3, 1)),
        ],
        &[vec![], vec![], vec![], vec![]],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        tangent_quadratic_report
            .cell_graph
            .loop_roles
            .iter()
            .filter(|role| role.class == CurveArrangementLoopRoleClass::Material
                && role.containment_depth == Some(0))
            .count(),
        2
    );

    let mixed_line = LinePathSegment::new(p(0, 0), p(20, 0));
    let mixed_quadratic = QuadraticBezier::new(p(0, 0), p(2, 4), p(4, 0));
    let mixed_cubic = CubicBezier::new(p(8, 0), p(8, 3), p(12, 3), p(12, 0));
    let mixed_conic =
        RationalQuadraticBezier::new(p(16, 0), p(18, 4), p(20, 0), r(2)).unwrap();
    let mixed_report = arrange_line_segments_with_mixed_beziers(
        &[mixed_line],
        &[mixed_quadratic],
        &[mixed_cubic],
        &[mixed_conic],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(mixed_report.line_breakpoints[0].len(), 6);
    assert_eq!(mixed_report.cell_graph.edges.len(), 8);
    assert_eq!(
        mixed_report.cell_graph.half_edges.len(),
        mixed_report.cell_graph.edges.len() * 2
    );
    assert_eq!(mixed_report.fragment_separations.len(), 3);
    assert!(mixed_report
        .fragment_separations
        .iter()
        .all(|separation| separation.class
            == MixedCurveFragmentSeparationClass::LeftBeforeRightX));

    let overlapping_mixed_error = arrange_line_segments_with_mixed_beziers(
        &[LinePathSegment::new(p(0, 0), p(8, 0))],
        &[QuadraticBezier::new(p(0, 0), p(4, 8), p(8, 0))],
        &[CubicBezier::new(p(0, 0), p(0, 4), p(8, 4), p(8, 0))],
        &[],
        PredicatePolicy::default(),
    )
    .unwrap_err();
    assert_eq!(
        overlapping_mixed_error,
        LineMixedBezierArrangementError::UnsupportedCurveCurveInteraction {
            left: MixedCurveFragmentRef::Quadratic(0),
            right: MixedCurveFragmentRef::Cubic(0),
        }
    );
    let extrema_separated_report = arrange_line_segments_with_mixed_beziers(
        &[LinePathSegment::new(p(0, 0), p(4, 0))],
        &[QuadraticBezier::new(p(0, 0), p(2, 2), p(4, 0))],
        &[CubicBezier::new(p(0, 2), pq(1, 1, 3, 2), pq(3, 1, 3, 2), p(4, 2))],
        &[],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(extrema_separated_report
        .fragment_separations
        .iter()
        .any(|separation| separation.left == MixedCurveFragmentRef::Quadratic(0)
            && separation.right == MixedCurveFragmentRef::Cubic(0)
            && separation.class == MixedCurveFragmentSeparationClass::LeftBelowRightY));
    assert!(extrema_separated_report
        .fragment_envelopes
        .iter()
        .any(|envelope| envelope.fragment == MixedCurveFragmentRef::Quadratic(0)
            && envelope.source == MixedCurveSourceRef::Quadratic(0)
            && envelope.y_min == r(0)
            && envelope.y_max == r(1)));
    assert!(extrema_separated_report
        .fragment_envelopes
        .iter()
        .any(|envelope| envelope.fragment == MixedCurveFragmentRef::Cubic(0)
            && envelope.source == MixedCurveSourceRef::Cubic(0)
            && envelope.y_min == rq(13, 8)
            && envelope.y_max == r(2)));
    let conic_extrema_separated_report = arrange_line_segments_with_mixed_beziers(
        &[LinePathSegment::new(p(0, 0), p(4, 0))],
        &[QuadraticBezier::new(p(0, 0), p(2, 2), p(4, 0))],
        &[],
        &[RationalQuadraticBezier::new(p(0, 2), p(2, 0), p(4, 2), rq(1, 3)).unwrap()],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(conic_extrema_separated_report
        .fragment_separations
        .iter()
        .any(|separation| separation.left == MixedCurveFragmentRef::Quadratic(0)
            && separation.right == MixedCurveFragmentRef::RationalQuadratic(0)
            && separation.class == MixedCurveFragmentSeparationClass::LeftBelowRightY));
    assert!(conic_extrema_separated_report
        .fragment_envelopes
        .iter()
        .any(|envelope| envelope.fragment == MixedCurveFragmentRef::RationalQuadratic(0)
            && envelope.source == MixedCurveSourceRef::RationalQuadratic(0)
            && envelope.y_min == rq(3, 2)
            && envelope.y_max == r(2)));

    let endpoint_contact_report = arrange_line_segments_with_mixed_beziers(
        &[LinePathSegment::new(p(0, 0), p(8, 0))],
        &[QuadraticBezier::new(p(0, 0), p(2, 2), p(4, 0))],
        &[CubicBezier::new(p(4, 0), p(5, -1), p(7, -1), p(8, 0))],
        &[],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(endpoint_contact_report
        .fragment_separations
        .iter()
        .any(|separation| separation.class == MixedCurveFragmentSeparationClass::EndpointContact
            && separation.left_endpoint == Some(MixedCurveFragmentEndpoint::End)
            && separation.right_endpoint == Some(MixedCurveFragmentEndpoint::Start)
            && separation.endpoint_tangent_class
                == Some(MixedCurveEndpointTangentClass::Collinear)));

    let endpoint_ccw_contact_report = arrange_line_segments_with_mixed_beziers(
        &[LinePathSegment::new(p(0, 0), p(8, 0))],
        &[QuadraticBezier::new(p(0, 0), p(2, 1), p(4, 0))],
        &[CubicBezier::new(p(4, 0), p(5, -2), p(7, -2), p(8, 0))],
        &[],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(endpoint_ccw_contact_report
        .fragment_separations
        .iter()
        .any(|separation| separation.endpoint_tangent_class
            == Some(MixedCurveEndpointTangentClass::CounterClockwise)));

    let endpoint_edge_contact_error = arrange_line_segments_with_mixed_beziers(
        &[LinePathSegment::new(p(0, 0), p(8, 0))],
        &[QuadraticBezier::new(p(0, 0), p(2, 2), p(4, 0))],
        &[CubicBezier::new(p(4, 0), p(5, 1), p(7, 1), p(8, 0))],
        &[],
        PredicatePolicy::default(),
    )
    .unwrap_err();
    assert_eq!(
        endpoint_edge_contact_error,
        LineMixedBezierArrangementError::UnsupportedCurveCurveInteraction {
            left: MixedCurveFragmentRef::Quadratic(0),
            right: MixedCurveFragmentRef::Cubic(0),
        }
    );

    let mixed_curve_report = arrange_line_segments_with_mixed_curves(
        &[LinePathSegment::new(p(0, 0), p(28, 0))],
        &[ExplicitCircularArc::new(p(2, 0), r(2), p(0, 0), p(4, 0), ArcDirection::Cw).unwrap()],
        &[QuadraticBezier::new(p(8, 0), p(10, 4), p(12, 0))],
        &[CubicBezier::new(p(16, 0), p(16, 3), p(20, 3), p(20, 0))],
        &[RationalQuadraticBezier::new(p(24, 0), p(26, 4), p(28, 0), r(2)).unwrap()],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(mixed_curve_report.line_breakpoints[0].len(), 8);
    assert_eq!(mixed_curve_report.cell_graph.edges.len(), 11);
    assert_eq!(
        mixed_curve_report.cell_graph.half_edges.len(),
        mixed_curve_report.cell_graph.edges.len() * 2
    );

    let mixed_cubic_evidence_report = arrange_line_segments_with_mixed_beziers(
        &[LinePathSegment::new(p(2, 0), p(6, 0))],
        &[],
        &[CubicBezier::new(p(0, 0), p(1, 0), p(7, 0), p(8, 0))],
        &[],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        mixed_cubic_evidence_report.cubic_events[0].class,
        LineCubicBezierIntersectionClass::Unknown
    );
    assert_eq!(
        mixed_cubic_evidence_report
            .cubic_algebraic_evidence
            .support_overlaps
            .len(),
        1
    );
    assert_eq!(
        mixed_cubic_evidence_report
            .cubic_algebraic_evidence
            .algebraic_overlap_breakpoints
            .len(),
        6
    );
    assert_eq!(
        mixed_cubic_evidence_report
            .cubic_algebraic_evidence
            .algebraic_overlap_breakpoint_sequences
            .len(),
        2
    );

    let mixed_conic_evidence_report = arrange_line_segments_with_mixed_beziers(
        &[LinePathSegment::new(p(1, 0), p(2, 0))],
        &[],
        &[],
        &[RationalQuadraticBezier::new(p(0, 0), p(8, 0), p(0, 0), r(1)).unwrap()],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        mixed_conic_evidence_report.rational_quadratic_events[0].class,
        LineRationalQuadraticBezierIntersectionClass::Unknown
    );
    assert_eq!(
        mixed_conic_evidence_report
            .rational_quadratic_algebraic_evidence
            .support_overlaps
            .len(),
        1
    );
    assert_eq!(
        mixed_conic_evidence_report
            .rational_quadratic_algebraic_evidence
            .algebraic_breakpoints
            .len(),
        4
    );
    assert_eq!(
        mixed_conic_evidence_report
            .rational_quadratic_algebraic_evidence
            .algebraic_breakpoint_sequences
            .len(),
        2
    );

    let mixed_promoted_conic_siblings_report = arrange_line_segments_with_mixed_beziers(
        &[LinePathSegment::new(p(1, 0), p(3, 0))],
        &[],
        &[],
        &[RationalQuadraticBezier::new(p(0, 0), p(8, 0), p(0, 0), r(1)).unwrap()],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        mixed_promoted_conic_siblings_report
            .rational_quadratic_algebraic_evidence
            .exact_algebraic_breakpoint_promotions
            .len(),
        2
    );
    assert_eq!(
        mixed_promoted_conic_siblings_report
            .rational_quadratic_fragments
            .len(),
        3
    );
    assert_eq!(
        mixed_promoted_conic_siblings_report
            .fragment_separations
            .len(),
        3
    );
    assert!(mixed_promoted_conic_siblings_report
        .fragment_separations
        .iter()
        .all(|separation| separation.class
            == MixedCurveFragmentSeparationClass::SameSourceSibling));

    let overlapping_arc_error = arrange_line_segments_with_mixed_curves(
        &[LinePathSegment::new(p(0, 0), p(8, 0))],
        &[ExplicitCircularArc::new(p(4, 0), r(4), p(0, 0), p(8, 0), ArcDirection::Cw).unwrap()],
        &[QuadraticBezier::new(p(0, 0), p(4, 8), p(8, 0))],
        &[],
        &[],
        PredicatePolicy::default(),
    )
    .unwrap_err();
    assert_eq!(
        overlapping_arc_error,
        LineMixedBezierArrangementError::UnsupportedCurveCurveInteraction {
            left: MixedCurveFragmentRef::ExplicitArc(0),
            right: MixedCurveFragmentRef::Quadratic(0),
        }
    );

    let sweep_tight_arc_report = arrange_line_segments_with_mixed_curves(
        &[LinePathSegment::new(p(0, 0), p(8, 0))],
        &[ExplicitCircularArc::new(p(4, 0), r(4), p(0, 0), p(8, 0), ArcDirection::Cw).unwrap()],
        &[QuadraticBezier::new(p(2, -1), p(4, -3), p(6, -1))],
        &[],
        &[],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(sweep_tight_arc_report.arc_fragments.len(), 1);
    assert_eq!(sweep_tight_arc_report.quadratic_fragments.len(), 1);

    let full_circle_arc =
        ExplicitCircularArc::new(p(0, 0), r(5), p(5, 0), p(5, 0), ArcDirection::Ccw).unwrap();
    let diagonal_arc_line = LinePathSegment::new(p(-6, -8), p(6, 8));
    let diagonal_arc_intersection =
        full_circle_arc.intersect_segment(&diagonal_arc_line, PredicatePolicy::default());
    assert_eq!(
        diagonal_arc_intersection.class,
        LineExplicitArcIntersectionClass::Secant
    );
    assert_eq!(diagonal_arc_intersection.points, vec![p(-3, -4), p(3, 4)]);
    let diagonal_arc_report = arrange_line_segments_with_explicit_arcs(
        &[diagonal_arc_line],
        &[full_circle_arc],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(diagonal_arc_report.line_breakpoints[0].len(), 4);
    assert_eq!(diagonal_arc_report.arc_breakpoints[0].len(), 3);

    let full_circle_overlap_error = arrange_line_segments_with_mixed_curves(
        &[LinePathSegment::new(p(0, 10), p(8, 10))],
        &[ExplicitCircularArc::new(p(4, 0), r(4), p(8, 0), p(8, 0), ArcDirection::Ccw).unwrap()],
        &[QuadraticBezier::new(p(2, -1), p(4, -3), p(6, -1))],
        &[],
        &[],
        PredicatePolicy::default(),
    )
    .unwrap_err();
    assert_eq!(
        full_circle_overlap_error,
        LineMixedBezierArrangementError::UnsupportedCurveCurveInteraction {
            left: MixedCurveFragmentRef::ExplicitArc(0),
            right: MixedCurveFragmentRef::Quadratic(0),
        }
    );

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
    let exact_cubic_overlap_curve = CubicBezier::new(p(0, 0), p(8, 0), p(8, 0), p(0, 0));
    let exact_cubic_overlap_line = LinePathSegment::new(p(0, 0), pq(9, 2, 0, 1));
    let exact_cubic_overlap_report = arrange_line_segments_with_cubic_beziers(
        &[exact_cubic_overlap_line],
        &[exact_cubic_overlap_curve],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(exact_cubic_overlap_report
        .exact_algebraic_overlap_breakpoint_promotions
        .iter()
        .any(|promotion| promotion.parameter
            == Real::new(Rational::new(1) / Rational::new(4))));
    assert!(exact_cubic_overlap_report
        .exact_algebraic_overlap_breakpoint_promotions
        .iter()
        .any(|promotion| promotion.parameter
            == Real::new(Rational::new(3) / Rational::new(4))));
    assert!(exact_cubic_overlap_report
        .algebraic_overlap_endpoint_envelopes
        .iter()
        .any(|envelope| {
            let span =
                &exact_cubic_overlap_report.algebraic_overlap_source_spans[envelope.span];
            span.source == LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Curve(0)
                && span.parameter_lower == rq(1, 4)
                && span.parameter_upper == rq(3, 4)
                && envelope.x_lower == rq(9, 2)
                && envelope.x_upper == r(6)
                && envelope.y_lower == r(0)
                && envelope.y_upper == r(0)
        }));
    assert_eq!(exact_cubic_overlap_report.cubic_breakpoints[0].len(), 4);
    assert_eq!(exact_cubic_overlap_report.cubic_fragments.len(), 3);
    let general_cubic_overlap_curve = CubicBezier::new(
        p(0, 0),
        pq(8, 3, 8, 3),
        pq(16, 3, 16, 3),
        p(8, 8),
    );
    let general_cubic_overlap_line = LinePathSegment::new(p(2, 2), p(6, 6));
    let general_cubic_overlap_report = arrange_line_segments_with_cubic_beziers(
        &[general_cubic_overlap_line],
        &[general_cubic_overlap_curve],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        general_cubic_overlap_report.events[0].class,
        LineCubicBezierIntersectionClass::Overlap
    );
    assert_eq!(
        general_cubic_overlap_report.events[0].intersection.intersections[0].parameter,
        rq(1, 4)
    );
    assert_eq!(
        general_cubic_overlap_report.events[0].intersection.intersections[1].parameter,
        rq(3, 4)
    );
    assert_eq!(general_cubic_overlap_report.cubic_breakpoints[0].len(), 4);
    let general_cubic_nonmonotone_curve = CubicBezier::new(p(0, 0), p(8, 8), p(0, 0), p(0, 0));
    let general_cubic_nonmonotone_line = LinePathSegment::new(p(1, 1), p(3, 3));
    let general_cubic_nonmonotone_report = intersect_line_cubic_bezier(
        &general_cubic_nonmonotone_line,
        &general_cubic_nonmonotone_curve,
        PredicatePolicy::default(),
    );
    assert_eq!(
        general_cubic_nonmonotone_report.class,
        LineCubicBezierIntersectionClass::Unknown
    );
    assert!(general_cubic_nonmonotone_report.intersections.is_empty());
    let nonlinear_cubic_overlap_curve =
        CubicBezier::new(p(0, 0), p(1, 0), p(7, 0), p(8, 0));
    let nonlinear_cubic_overlap_line = LinePathSegment::new(p(-1, 0), p(9, 0));
    let nonlinear_cubic_overlap_report = arrange_line_segments_with_cubic_beziers(
        &[nonlinear_cubic_overlap_line],
        &[nonlinear_cubic_overlap_curve.clone()],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        nonlinear_cubic_overlap_report.events[0].class,
        LineCubicBezierIntersectionClass::Overlap
    );
    assert_eq!(
        nonlinear_cubic_overlap_report.events[0]
            .intersection
            .support_overlap
            .as_ref()
            .unwrap()
            .monotonicity,
        LineCubicBezierSupportOverlapMonotonicity::Monotone
    );
    let nonlinear_cubic_inner_line = LinePathSegment::new(p(2, 0), p(6, 0));
    let nonlinear_cubic_inner_report = intersect_axis_aligned_line_cubic_bezier(
        &nonlinear_cubic_inner_line,
        &nonlinear_cubic_overlap_curve,
        PredicatePolicy::default(),
    );
    assert_eq!(
        nonlinear_cubic_inner_report.class,
        LineCubicBezierIntersectionClass::Unknown
    );
    assert_eq!(
        nonlinear_cubic_inner_report
            .support_overlap
            .as_ref()
            .unwrap()
            .inverse_boundary_roots
            .len(),
        2
    );
    let nonlinear_cubic_mixed_inner_report = arrange_line_segments_with_cubic_beziers(
        &[nonlinear_cubic_inner_line],
        &[nonlinear_cubic_overlap_curve],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(nonlinear_cubic_mixed_inner_report.support_overlaps.len(), 1);
    assert!(
        nonlinear_cubic_mixed_inner_report
            .algebraic_overlap_breakpoints
            .iter()
            .any(|breakpoint| breakpoint.domain
                == LineCubicBezierAlgebraicOverlapBreakpointDomain::InsideLineAndCurve)
    );
    assert!(
        nonlinear_cubic_mixed_inner_report
            .algebraic_overlap_breakpoint_sequences
            .iter()
            .any(|sequence| sequence.source
                == LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Curve(0)
                && sequence.class
                    == LineCubicBezierAlgebraicOverlapBreakpointSequenceClass::Ordered)
    );
    assert!(!nonlinear_cubic_mixed_inner_report
        .algebraic_overlap_source_spans
        .is_empty());
    assert_eq!(
        nonlinear_cubic_mixed_inner_report
            .algebraic_overlap_endpoint_envelopes
            .len(),
        nonlinear_cubic_mixed_inner_report
            .algebraic_overlap_source_spans
            .len()
    );
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
        &[algebraic_cubic.clone()],
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
    assert!(
        algebraic_mixed_report
            .algebraic_breakpoint_orders
            .is_empty()
    );
    assert_eq!(
        algebraic_mixed_report.algebraic_breakpoint_sequences.len(),
        2
    );
    assert!(
        algebraic_mixed_report
            .algebraic_breakpoint_sequences
            .iter()
            .all(|sequence| sequence.class
                == LineCubicBezierAlgebraicBreakpointSequenceClass::Ordered
                && sequence.breakpoints == vec![0]
                && sequence.blockers.is_empty())
    );
    assert_eq!(algebraic_mixed_report.algebraic_source_spans.len(), 4);
    assert_eq!(algebraic_mixed_report.algebraic_endpoint_envelopes.len(), 4);
    assert_eq!(
        algebraic_mixed_report
            .exact_algebraic_breakpoint_promotions
            .len(),
        1
    );
    assert_eq!(
        algebraic_mixed_report.exact_algebraic_breakpoint_promotions[0].cubic_parameter,
        Real::new(Rational::new(1) / Rational::new(2))
    );
    assert_eq!(algebraic_mixed_report.line_breakpoints[0].len(), 3);
    assert_eq!(algebraic_mixed_report.cubic_breakpoints[0].len(), 3);
    let general_algebraic_line = LinePathSegment::new(pq(0, 1, -3, 8), pq(1, 1, 5, 8));
    let general_algebraic_report = intersect_line_cubic_bezier(
        &general_algebraic_line,
        &algebraic_cubic,
        PredicatePolicy::default(),
    );
    assert_eq!(
        general_algebraic_report.class,
        LineCubicBezierIntersectionClass::Unknown
    );
    assert_eq!(general_algebraic_report.algebraic_support_roots.len(), 3);
    assert!(general_algebraic_report.algebraic_support_roots.iter().any(|root| {
        root.parameter_domain == LineCubicAlgebraicRootDomain::InsideUnitInterval
            && root.point_image.segment_domain
                == LineCubicAlgebraicPointDomain::InsideSegmentBounds
            && root.point_image.x.status == AlgebraicRootPolynomialImageStatus::Transformed
            && root.point_image.y.status == AlgebraicRootPolynomialImageStatus::Transformed
    }));
    let general_algebraic_mixed_report = arrange_line_segments_with_cubic_beziers(
        &[general_algebraic_line],
        &[algebraic_cubic],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        general_algebraic_mixed_report
            .algebraic_breakpoints
            .len(),
        2
    );
    assert!(general_algebraic_mixed_report.algebraic_breakpoints.iter().all(
        |breakpoint| breakpoint.domain
            == LineCubicBezierAlgebraicBreakpointDomain::InsideLineAndCurve
            && breakpoint.line_parameter.status
                == AlgebraicRootPolynomialImageStatus::Transformed
    ));
    let three_root_cubic = CubicBezier::new(
        hyperlimit::Point2::new(r(0), Real::new(Rational::new(-2) / Rational::new(25))),
        hyperlimit::Point2::new(
            Real::new(Rational::new(1) / Rational::new(3)),
            Real::new(Rational::new(7) / Rational::new(50)),
        ),
        hyperlimit::Point2::new(
            Real::new(Rational::new(2) / Rational::new(3)),
            Real::new(Rational::new(-7) / Rational::new(50)),
        ),
        hyperlimit::Point2::new(r(1), Real::new(Rational::new(2) / Rational::new(25))),
    );
    let three_root_report = arrange_line_segments_with_cubic_beziers(
        &[LinePathSegment::new(p(0, 0), p(1, 0))],
        &[three_root_cubic],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(three_root_report.algebraic_breakpoints.len(), 3);
    assert_eq!(three_root_report.algebraic_breakpoint_orders.len(), 3);
    assert!(
        three_root_report
            .algebraic_breakpoint_orders
            .iter()
            .all(|order| {
                order.cubic_order == Some(LineCubicBezierAlgebraicBreakpointOrderClass::Before)
                    && order.line_order
                        == Some(LineCubicBezierAlgebraicBreakpointOrderClass::Before)
            })
    );
    assert_eq!(three_root_report.algebraic_breakpoint_sequences.len(), 2);
    assert!(
        three_root_report
            .algebraic_breakpoint_sequences
            .iter()
            .all(|sequence| sequence.class
                == LineCubicBezierAlgebraicBreakpointSequenceClass::Ordered
                && sequence.breakpoints == vec![0, 1, 2]
                && sequence.blockers.is_empty())
    );
    assert_eq!(three_root_report.algebraic_source_spans.len(), 8);
    assert_eq!(three_root_report.algebraic_endpoint_envelopes.len(), 8);
    assert!(three_root_report.algebraic_endpoint_envelopes.iter().any(|envelope| {
        compare_reals_with_policy(
            &envelope.y_upper,
            &Real::new(Rational::new(1) / Rational::new(200)),
            PredicatePolicy::default(),
        )
        .value()
            == Some(std::cmp::Ordering::Greater)
    }));
    assert!(three_root_report.algebraic_endpoint_envelopes.iter().any(|envelope| {
        compare_reals_with_policy(
            &envelope.y_lower,
            &Real::new(Rational::new(-1) / Rational::new(200)),
            PredicatePolicy::default(),
        )
        .value()
            == Some(std::cmp::Ordering::Less)
    }));

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
    let diagonal_conic = RationalQuadraticBezier::new(p(0, 0), p(2, 4), p(4, 0), r(1)).unwrap();
    let diagonal_conic_line = LinePathSegment::new(p(0, 1), p(8, 5));
    let diagonal_conic_report = intersect_line_rational_quadratic_bezier(
        &diagonal_conic_line,
        &diagonal_conic,
        PredicatePolicy::default(),
    );
    assert_eq!(
        diagonal_conic_report.class,
        LineRationalQuadraticBezierIntersectionClass::TwoPoints
    );
    assert_eq!(diagonal_conic_report.intersections[0].parameter, rq(1, 4));
    assert_eq!(diagonal_conic_report.intersections[1].parameter, rq(1, 2));
    let diagonal_conic_arrangement = arrange_line_segments_with_rational_quadratic_beziers(
        &[diagonal_conic_line],
        &[diagonal_conic],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        diagonal_conic_arrangement.events[0].class,
        LineRationalQuadraticBezierIntersectionClass::TwoPoints
    );
    assert_eq!(diagonal_conic_arrangement.line_breakpoints[0].len(), 4);
    assert_eq!(diagonal_conic_arrangement.conic_breakpoints[0].len(), 4);
    let general_overlap_conic =
        RationalQuadraticBezier::new(p(0, 0), p(2, 2), p(4, 4), r(1)).unwrap();
    let general_overlap_line = LinePathSegment::new(p(1, 1), p(3, 3));
    let general_overlap_report = arrange_line_segments_with_rational_quadratic_beziers(
        &[general_overlap_line],
        &[general_overlap_conic],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(
        general_overlap_report.events[0].class,
        LineRationalQuadraticBezierIntersectionClass::Overlap
    );
    assert_eq!(
        general_overlap_report.events[0].intersection.intersections[0].parameter,
        rq(1, 4)
    );
    assert_eq!(
        general_overlap_report.events[0].intersection.intersections[1].parameter,
        rq(3, 4)
    );
    assert_eq!(general_overlap_report.line_fragments.len(), 1);
    assert_eq!(general_overlap_report.conic_fragments.len(), 3);

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
    assert_eq!(secant_report.cell_graph.edges.len(), 6);
    assert_eq!(
        secant_report.cell_graph.half_edges.len(),
        secant_report.cell_graph.edges.len() * 2
    );
    assert_eq!(secant_report.cell_graph.faces.len(), 2);
    assert!(secant_report.cell_graph.faces.iter().any(|face| {
        face.class == CurveArrangementCellFaceClass::Bounded
    }));
    assert!(secant_report.cell_graph.faces.iter().any(|face| {
        face.class == CurveArrangementCellFaceClass::Exterior
    }));

    let log_area_conic = RationalQuadraticBezier::new(p(0, 0), p(4, 8), p(8, 0), r(2)).unwrap();
    let log_area_report = arrange_line_segments_with_rational_quadratic_beziers(
        &[LinePathSegment::new(p(0, 0), p(8, 0))],
        &[log_area_conic.clone()],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(log_area_report.cell_graph.faces.len(), 2);

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
    assert_eq!(
        overlap_report.events[0]
            .intersection
            .support_overlap
            .as_ref()
            .unwrap()
            .monotonicity,
        LineRationalQuadraticBezierSupportOverlapMonotonicity::Monotone
    );
    assert_eq!(overlap_report.support_overlaps.len(), 1);
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
    assert_eq!(nonmonotone_report.support_overlaps.len(), 1);
    assert_eq!(
        nonmonotone_report.support_overlaps[0].overlap.monotonicity,
        LineRationalQuadraticBezierSupportOverlapMonotonicity::NonMonotone
    );
    assert_eq!(
        nonmonotone_report.support_overlaps[0]
            .overlap
            .inverse_boundary_roots
            .len(),
        2
    );
    assert!(
        nonmonotone_report.support_overlaps[0]
            .overlap
            .inverse_boundary_roots
            .iter()
            .flat_map(|boundary| boundary.roots.iter())
            .all(|root| root.parameter_domain
                == LineRationalQuadraticBezierInverseRootDomain::InsideUnitInterval)
    );
    assert_eq!(nonmonotone_report.algebraic_breakpoints.len(), 2);
    assert!(
        nonmonotone_report
            .algebraic_breakpoints
            .iter()
            .all(|breakpoint| breakpoint.domain
                == LineRationalQuadraticBezierAlgebraicBreakpointDomain::InsideLineAndCurve)
    );
    assert_eq!(nonmonotone_report.algebraic_breakpoint_orders.len(), 1);
    assert_ne!(
        nonmonotone_report.algebraic_breakpoint_orders[0].order,
        LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Unknown
    );
    assert_eq!(nonmonotone_report.algebraic_breakpoint_sequences.len(), 2);
    assert!(
        nonmonotone_report
            .algebraic_breakpoint_sequences
            .iter()
            .any(|sequence| sequence.source
                == LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Curve(0)
                && sequence.class
                    == LineRationalQuadraticBezierAlgebraicBreakpointSequenceClass::Ordered
                && sequence.blockers.is_empty())
    );
    assert!(
        nonmonotone_report
            .algebraic_breakpoint_sequences
            .iter()
            .any(|sequence| sequence.source
                == LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Line(0)
                && sequence.class
                    == LineRationalQuadraticBezierAlgebraicBreakpointSequenceClass::Ambiguous
                && !sequence.blockers.is_empty())
    );
    assert_eq!(nonmonotone_report.algebraic_source_spans.len(), 3);
    assert_eq!(
        nonmonotone_report.algebraic_endpoint_envelopes.len(),
        nonmonotone_report.algebraic_source_spans.len()
    );
    assert!(nonmonotone_report.algebraic_endpoint_envelopes.iter().any(|envelope| {
        envelope.x_upper == r(4) && envelope.y_lower == r(0) && envelope.y_upper == r(0)
    }));
    let exact_root_conic = RationalQuadraticBezier::new(p(0, 0), p(8, 0), p(0, 0), r(1)).unwrap();
    let exact_root_report = arrange_line_segments_with_rational_quadratic_beziers(
        &[LinePathSegment::new(p(0, 0), p(3, 0))],
        &[exact_root_conic],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(exact_root_report
        .exact_algebraic_breakpoint_promotions
        .iter()
        .any(|promotion| promotion.parameter == Real::new(Rational::new(1) / Rational::new(4))));
    assert_eq!(exact_root_report.conic_breakpoints[0].len(), 4);

    let r_report =
        arrange_rational_quadratic_beziers(&[conic], &[vec![t]], PredicatePolicy::default())
            .unwrap();
    assert_eq!(r_report.fragments.len(), 2);
    assert_eq!(r_report.cell_graph.edges.len(), 2);
    assert_eq!(
        r_report.cell_graph.half_edges.len(),
        r_report.cell_graph.edges.len() * 2
    );
    assert_eq!(
        r_report.fragments[0].end_control,
        r_report.fragments[1].start_control
    );

    let lower_conic = RationalQuadraticBezier::new(p(8, 0), p(4, -8), p(0, 0), r(2)).unwrap();
    let conic_loop_report = arrange_rational_quadratic_beziers(
        &[log_area_conic, lower_conic],
        &[vec![], vec![]],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(conic_loop_report.cell_graph.vertices.len(), 2);
    assert_eq!(conic_loop_report.cell_graph.edges.len(), 2);
    assert_eq!(conic_loop_report.cell_graph.faces.len(), 2);
    assert!(conic_loop_report
        .cell_graph
        .loop_roles
        .iter()
        .any(|role| role.class == CurveArrangementLoopRoleClass::Material
            && role.containment_depth == Some(0)
            && role.representative.is_some()));
    let nested_conic_loop_report = arrange_rational_quadratic_beziers(
        &[
            RationalQuadraticBezier::new(p(0, 0), p(4, 8), p(8, 0), r(2)).unwrap(),
            RationalQuadraticBezier::new(p(8, 0), p(4, -8), p(0, 0), r(2)).unwrap(),
            RationalQuadraticBezier::new(p(2, 0), p(4, 3), p(6, 0), r(2)).unwrap(),
            RationalQuadraticBezier::new(p(6, 0), p(4, -3), p(2, 0), r(2)).unwrap(),
        ],
        &[vec![], vec![], vec![], vec![]],
        PredicatePolicy::default(),
    )
    .unwrap();
    assert!(nested_conic_loop_report
        .cell_graph
        .loop_roles
        .iter()
        .any(|role| role.class == CurveArrangementLoopRoleClass::Hole
            && role.containment_depth == Some(1)
            && role.representative.is_some()));
});
