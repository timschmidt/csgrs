use crate::{
    BezierAlgebraicEndpointImage2, BezierAlgebraicParameter2, BezierArrangementGraph2,
    BezierLineContact, BezierLineContactKind, BezierMonotoneSpan, BezierParameter2,
    BezierParameterInterval, BezierParameterPolynomial, BezierSplitFragment2, BezierSubcurve2,
    Classification, CubicBezier2, CurveContext, CurveError, Point2, QuadraticBezier2,
    RationalBezier2, RationalQuadraticBezier2, Real, UncertaintyReason,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn policy() -> CurveContext {
    CurveContext::STRICT
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

fn assert_topology_error<T>(result: Result<T, CurveError>) {
    assert!(matches!(result, Err(CurveError::Topology(_))));
}

fn graph(fragments: Vec<crate::BezierArrangementFragment2>) -> BezierArrangementGraph2 {
    BezierArrangementGraph2::from_certified_fragments(fragments)
}

#[test]
fn monotone_span_rejects_reversed_parameter_evidence() {
    assert_topology_error(BezierMonotoneSpan::new(r(1), r(0)));
}

#[test]
fn contact_parameters_follow_their_carrier_domains() {
    for value in [r(-1), r(2)] {
        let parameter = BezierParameter2::Exact(value);
        let contact = BezierLineContact::new(parameter.clone(), BezierLineContactKind::Tangent);
        assert_eq!(contact.parameter(), &parameter);
        assert_eq!(contact.kind(), BezierLineContactKind::Tangent);
    }
}

fn algebraic_midpoint_parameter() -> BezierAlgebraicParameter2 {
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis(vec![r(-1), r(2)], &policy()).unwrap(),
    );
    let interval = decided(BezierParameterInterval::try_new(q(2, 5), q(3, 5), &policy()).unwrap());
    decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap())
}

fn through_origin_with_midpoint_tangent(dx: i32, dy: i32) -> QuadraticBezier2 {
    QuadraticBezier2::new(p(-dx, -dy), p(0, 0), p(dx, dy))
}

fn through_origin_with_horizontal_midpoint_tangent(curvature: i32) -> QuadraticBezier2 {
    QuadraticBezier2::new(
        Point2::new(r(-1), r(curvature)),
        Point2::new(r(0), r(-curvature)),
        Point2::new(r(1), r(curvature)),
    )
}

fn through_origin_with_horizontal_midpoint_tangent_and_third_order(third_y: i32) -> CubicBezier2 {
    CubicBezier2::new(
        Point2::new(q(-1, 2), q(-third_y, 8)),
        Point2::new(q(-1, 6), q(third_y, 8)),
        Point2::new(q(1, 6), q(-third_y, 8)),
        Point2::new(q(1, 2), q(third_y, 8)),
    )
}

fn rational_through_origin_with_horizontal_midpoint_tangent(
    curvature: i32,
) -> RationalQuadraticBezier2 {
    RationalQuadraticBezier2::try_new(
        Point2::new(r(-1), r(curvature)),
        Point2::new(r(0), r(-curvature)),
        Point2::new(r(1), r(curvature)),
        r(1),
        r(1),
        r(1),
    )
    .unwrap()
}

fn algebraic_endpoint_image(
    curve: &QuadraticBezier2,
    parameter: &BezierAlgebraicParameter2,
) -> BezierAlgebraicEndpointImage2 {
    decided(BezierAlgebraicEndpointImage2::quadratic(curve, parameter, &policy()).unwrap())
}

fn algebraic_cubic_endpoint_image(
    curve: &CubicBezier2,
    parameter: &BezierAlgebraicParameter2,
) -> BezierAlgebraicEndpointImage2 {
    decided(BezierAlgebraicEndpointImage2::cubic(curve, parameter, &policy()).unwrap())
}

fn algebraic_rational_endpoint_image(
    curve: &RationalQuadraticBezier2,
    parameter: &BezierAlgebraicParameter2,
) -> BezierAlgebraicEndpointImage2 {
    decided(BezierAlgebraicEndpointImage2::rational_quadratic(curve, parameter, &policy()).unwrap())
}

#[test]
fn tangent_ordered_traversal_resolves_simple_branch_vertex() {
    let first = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0))),
    };
    let upward = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Cubic(CubicBezier2::new(p(2, 0), p(3, 1), p(4, 1), p(5, 0))),
    };
    let straightest = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(3, -1), p(4, 0))),
    };
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, first),
        crate::BezierArrangementFragment2::new(1, 0, upward),
        crate::BezierArrangementFragment2::new(2, 0, straightest),
    ]);
    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));

    assert_eq!(traversal.chains().len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 2]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[1]);
}

#[test]
fn tangent_ordered_traversal_uses_second_order_for_equal_outgoing_tangents() {
    let first = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let first_out = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(3, 1), p(4, 0))),
    };
    let second_out = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(4, 2), p(5, 0))),
    };
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, first),
        crate::BezierArrangementFragment2::new(1, 0, first_out),
        crate::BezierArrangementFragment2::new(2, 0, second_out),
    ]);

    // Match x-coordinates with t=2u-u^2/2, for 0<u<1/2. The second
    // branch lies above the first by u^2*(u^2-8u+10)/2 > 0. Both rays
    // point into the upper half-plane, so the first branch is encountered
    // first counter-clockwise from the incoming horizontal tangent.
    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(traversal.chains().len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);

    let retained_traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(retained_traversal.chains().len(), 2);
    assert_eq!(retained_traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(retained_traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn tangent_ordered_traversal_resolves_equal_nonzero_curvature() {
    let curve = |k: i32| {
        BezierSubcurve2::Cubic(CubicBezier2::new(
            p(0, 0),
            Point2::new(q(1, 3), r(0)),
            Point2::new(q(2, 3), q(1, 3)),
            p(1, 1 + k),
        ))
    };
    let fragment = |index, curve| {
        crate::BezierArrangementFragment2::new(
            index,
            0,
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(r(0)),
                end: BezierParameter2::Exact(r(1)),
                curve,
            },
        )
    };
    let graph = graph(vec![
        fragment(
            0,
            BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                p(-1, 0),
                Point2::new(q(-1, 2), r(0)),
                p(0, 0),
            )),
        ),
        fragment(1, curve(1)),
        fragment(2, curve(2)),
    ]);
    // The graphs y=x^2+x^3 and y=x^2+2x^3 have equal nonzero
    // curvature. At x>0, the first outgoing ray has the smaller angle.
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        {
            let traversal = graph.traverse_retained_with_tangent_order(&policy);
            let traversal = decided(traversal);
            assert_eq!(traversal.chains()[0].fragment_indices(), [0, 1]);
            assert_eq!(traversal.chains()[1].fragment_indices(), [2]);
        }
    }
}

#[test]
fn tangent_ordered_traversal_rejects_equal_second_order_outgoing_tangents() {
    let first = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let first_out = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(3, 1), p(4, 0))),
    };
    let second_out = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(3, 1), p(4, 0))),
    };
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, first),
        crate::BezierArrangementFragment2::new(1, 0, first_out),
        crate::BezierArrangementFragment2::new(2, 0, second_out),
    ]);

    assert_eq!(
        graph.traverse_retained_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn tangent_ordered_traversal_uses_rational_second_order_for_equal_outgoing_tangents() {
    let first = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let upward = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(p(2, 0), p(3, 0), p(4, 1), r(1), r(2), r(3)).unwrap(),
        ),
    };
    let downward = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(p(2, 0), p(3, 0), p(4, -1), r(1), r(2), r(3))
                .unwrap(),
        ),
    };
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, first),
        crate::BezierArrangementFragment2::new(1, 0, upward),
        crate::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(traversal.chains().len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);

    let retained_traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(retained_traversal.chains().len(), 2);
    assert_eq!(retained_traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(retained_traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn tangent_ordered_traversal_rejects_equal_rational_second_order_successors() {
    let first = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let first_out = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(p(2, 0), p(3, 0), p(4, 1), r(1), r(2), r(3)).unwrap(),
        ),
    };
    let second_out = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(p(2, 0), p(3, 0), p(4, 1), r(1), r(2), r(3)).unwrap(),
        ),
    };
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, first),
        crate::BezierArrangementFragment2::new(1, 0, first_out),
        crate::BezierArrangementFragment2::new(2, 0, second_out),
    ]);

    assert_eq!(
        graph.traverse_retained_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn tangent_ordered_traversal_uses_third_order_for_cubic_same_tangent_inflections() {
    let first = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let upward = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Cubic(CubicBezier2::new(p(2, 0), p(3, 0), p(4, 0), p(5, 1))),
    };
    let downward = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Cubic(CubicBezier2::new(p(2, 0), p(3, 0), p(4, 0), p(5, -1))),
    };
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, first),
        crate::BezierArrangementFragment2::new(1, 0, upward),
        crate::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(traversal.chains().len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);

    let retained_traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(retained_traversal.chains().len(), 2);
    assert_eq!(retained_traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(retained_traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn tangent_ordered_traversal_rejects_equal_third_order_cubic_successors() {
    let first = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let first_out = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Cubic(CubicBezier2::new(p(2, 0), p(3, 0), p(4, 0), p(5, 1))),
    };
    let second_out = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Cubic(CubicBezier2::new(p(2, 0), p(3, 0), p(4, 0), p(5, 1))),
    };
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, first),
        crate::BezierArrangementFragment2::new(1, 0, first_out),
        crate::BezierArrangementFragment2::new(2, 0, second_out),
    ]);

    assert_eq!(
        graph.traverse_retained_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn retained_tangent_order_traverses_algebraic_branch_vertex() {
    let parameter = algebraic_midpoint_parameter();
    let algebraic = BezierParameter2::Algebraic(parameter.clone());
    let incoming_curve = through_origin_with_midpoint_tangent(1, 0);
    let upward_curve = through_origin_with_midpoint_tangent(0, 1);
    let downward_curve = through_origin_with_midpoint_tangent(0, -1);
    let incoming = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: BezierParameter2::Exact(r(0)),
        end: algebraic.clone(),
        source_curve: BezierSubcurve2::Quadratic(incoming_curve.clone()),
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&incoming_curve, &parameter)),
    };
    let upward = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: algebraic.clone(),
        end: BezierParameter2::Exact(r(1)),
        source_curve: BezierSubcurve2::Quadratic(upward_curve.clone()),
        start_image: Some(algebraic_endpoint_image(&upward_curve, &parameter)),
        end_image: None,
    };
    let downward = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: algebraic,
        end: BezierParameter2::Exact(r(1)),
        source_curve: BezierSubcurve2::Quadratic(downward_curve.clone()),
        start_image: Some(algebraic_endpoint_image(&downward_curve, &parameter)),
        end_image: None,
    };
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, incoming),
        crate::BezierArrangementFragment2::new(1, 0, upward),
        crate::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));

    assert_eq!(traversal.chains().len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn retained_tangent_order_transforms_reversed_algebraic_endpoints_and_tangents() {
    let parameter = algebraic_midpoint_parameter();
    let algebraic = BezierParameter2::Algebraic(parameter.clone());
    let incoming_curve = through_origin_with_midpoint_tangent(1, 0);
    let source_downward_curve = through_origin_with_midpoint_tangent(0, -1);
    let source_upward_curve = through_origin_with_midpoint_tangent(0, 1);
    let incoming = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: BezierParameter2::Exact(r(0)),
        end: algebraic.clone(),
        source_curve: BezierSubcurve2::Quadratic(incoming_curve.clone()),
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&incoming_curve, &parameter)),
    };
    let upward = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: BezierParameter2::Exact(r(0)),
        end: algebraic.clone(),
        source_curve: BezierSubcurve2::Quadratic(source_downward_curve.clone()),
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&source_downward_curve, &parameter)),
    }
    .reversed()
    .unwrap();
    let downward = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: BezierParameter2::Exact(r(0)),
        end: algebraic,
        source_curve: BezierSubcurve2::Quadratic(source_upward_curve.clone()),
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&source_upward_curve, &parameter)),
    }
    .reversed()
    .unwrap();
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, incoming),
        crate::BezierArrangementFragment2::new(1, 0, upward),
        crate::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(traversal.chains().len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn retained_tangent_order_rejects_equal_algebraic_successors() {
    let parameter = algebraic_midpoint_parameter();
    let algebraic = BezierParameter2::Algebraic(parameter.clone());
    let incoming_curve = through_origin_with_midpoint_tangent(1, 0);
    let first_curve = through_origin_with_midpoint_tangent(0, 1);
    let second_curve = through_origin_with_midpoint_tangent(0, 1);
    let incoming = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: BezierParameter2::Exact(r(0)),
        end: algebraic.clone(),
        source_curve: BezierSubcurve2::Quadratic(incoming_curve.clone()),
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&incoming_curve, &parameter)),
    };
    let first = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: algebraic.clone(),
        end: BezierParameter2::Exact(r(1)),
        source_curve: BezierSubcurve2::Quadratic(first_curve.clone()),
        start_image: Some(algebraic_endpoint_image(&first_curve, &parameter)),
        end_image: None,
    };
    let second = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: algebraic,
        end: BezierParameter2::Exact(r(1)),
        source_curve: BezierSubcurve2::Quadratic(second_curve.clone()),
        start_image: Some(algebraic_endpoint_image(&second_curve, &parameter)),
        end_image: None,
    };
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, incoming),
        crate::BezierArrangementFragment2::new(1, 0, first),
        crate::BezierArrangementFragment2::new(2, 0, second),
    ]);

    assert_eq!(
        graph.traverse_retained_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn retained_tangent_order_uses_algebraic_second_order_for_equal_successors() {
    let parameter = algebraic_midpoint_parameter();
    let algebraic = BezierParameter2::Algebraic(parameter.clone());
    let incoming_curve = through_origin_with_midpoint_tangent(1, 0);
    let upward_curve = through_origin_with_horizontal_midpoint_tangent(1);
    let downward_curve = through_origin_with_horizontal_midpoint_tangent(-1);
    let incoming = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: BezierParameter2::Exact(r(0)),
        end: algebraic.clone(),
        source_curve: BezierSubcurve2::Quadratic(incoming_curve.clone()),
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&incoming_curve, &parameter)),
    };
    let upward = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: algebraic.clone(),
        end: BezierParameter2::Exact(r(1)),
        source_curve: BezierSubcurve2::Quadratic(upward_curve.clone()),
        start_image: Some(algebraic_endpoint_image(&upward_curve, &parameter)),
        end_image: None,
    };
    let downward = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: algebraic,
        end: BezierParameter2::Exact(r(1)),
        source_curve: BezierSubcurve2::Quadratic(downward_curve.clone()),
        start_image: Some(algebraic_endpoint_image(&downward_curve, &parameter)),
        end_image: None,
    };
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, incoming),
        crate::BezierArrangementFragment2::new(1, 0, upward),
        crate::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(traversal.chains().len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn retained_tangent_order_uses_rational_algebraic_second_order_for_equal_successors() {
    let parameter = algebraic_midpoint_parameter();
    let algebraic = BezierParameter2::Algebraic(parameter.clone());
    let incoming_curve = through_origin_with_midpoint_tangent(1, 0);
    let upward_curve = rational_through_origin_with_horizontal_midpoint_tangent(1);
    let downward_curve = rational_through_origin_with_horizontal_midpoint_tangent(-1);
    let incoming = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: BezierParameter2::Exact(r(0)),
        end: algebraic.clone(),
        source_curve: BezierSubcurve2::Quadratic(incoming_curve.clone()),
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&incoming_curve, &parameter)),
    };
    let upward = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: algebraic.clone(),
        end: BezierParameter2::Exact(r(1)),
        source_curve: BezierSubcurve2::RationalQuadratic(upward_curve.clone()),
        start_image: Some(algebraic_rational_endpoint_image(&upward_curve, &parameter)),
        end_image: None,
    };
    let downward = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: algebraic,
        end: BezierParameter2::Exact(r(1)),
        source_curve: BezierSubcurve2::RationalQuadratic(downward_curve.clone()),
        start_image: Some(algebraic_rational_endpoint_image(
            &downward_curve,
            &parameter,
        )),
        end_image: None,
    };
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, incoming),
        crate::BezierArrangementFragment2::new(1, 0, upward),
        crate::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(traversal.chains().len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn retained_tangent_order_uses_algebraic_third_order_for_cubic_same_tangent_inflections() {
    let parameter = algebraic_midpoint_parameter();
    let algebraic = BezierParameter2::Algebraic(parameter.clone());
    let incoming_curve = through_origin_with_midpoint_tangent(1, 0);
    let upward_curve = through_origin_with_horizontal_midpoint_tangent_and_third_order(8);
    let downward_curve = through_origin_with_horizontal_midpoint_tangent_and_third_order(-8);
    let incoming = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: BezierParameter2::Exact(r(0)),
        end: algebraic.clone(),
        source_curve: BezierSubcurve2::Quadratic(incoming_curve.clone()),
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&incoming_curve, &parameter)),
    };
    let upward = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: algebraic.clone(),
        end: BezierParameter2::Exact(r(1)),
        source_curve: BezierSubcurve2::Cubic(upward_curve.clone()),
        start_image: Some(algebraic_cubic_endpoint_image(&upward_curve, &parameter)),
        end_image: None,
    };
    let downward = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start: algebraic,
        end: BezierParameter2::Exact(r(1)),
        source_curve: BezierSubcurve2::Cubic(downward_curve.clone()),
        start_image: Some(algebraic_cubic_endpoint_image(&downward_curve, &parameter)),
        end_image: None,
    };
    let graph = graph(vec![
        crate::BezierArrangementFragment2::new(0, 0, incoming),
        crate::BezierArrangementFragment2::new(1, 0, upward),
        crate::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(traversal.chains().len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);
}

proptest! {
    #[test]
    fn open_quadratic_chain_stays_one_nonclosed_chain(
        middle_y in -16_i32..=16,
    ) {
        let first = QuadraticBezier2::new(p(0, 0), p(1, middle_y), p(2, 0));
        let second = QuadraticBezier2::new(p(2, 0), p(3, -middle_y), p(4, 0));
        let fragments = [first, second]
            .into_iter()
            .enumerate()
            .map(|(source, curve)| {
                let split = decided(curve.split_at_parameters(&[], &policy()).unwrap());
                crate::BezierArrangementFragment2::new(source, 0, split.fragments()[0].clone())
            })
            .collect();
        let graph = graph(fragments);
        let traversal = match graph.traverse_retained_with_tangent_order(&policy()) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                return Err(TestCaseError::fail(format!("unexpected uncertainty: {reason:?}")));
            }
        };

        prop_assert_eq!(traversal.chains().len(), 1);
        prop_assert!(!traversal.chains()[0].is_closed());
        prop_assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    }
}

#[test]
fn native_quadratic_ordering_agrees_with_rational_carriers() {
    use crate::BezierArrangementFragment2;
    fn source(family: usize, second: bool, shifted: bool) -> BezierSubcurve2 {
        let [a, b, c] = if shifted {
            [
                Point2::new(if second { q(-1, 4) } else { q(-1, 2) }, q(1, 4)),
                Point2::new(if second { q(-1, 4) } else { Real::zero() }, q(-1, 4)),
                Point2::new(if second { q(3, 4) } else { q(1, 2) }, q(1, 4)),
            ]
        } else {
            [
                Point2::from_values(0, 0),
                Point2::new(q(1, 2), Real::zero()),
                Point2::from_values(if second { 2 } else { 1 }, 1),
            ]
        };
        if family == 0 {
            BezierSubcurve2::Quadratic(QuadraticBezier2::new(a, b, c))
        } else {
            let conic =
                RationalQuadraticBezier2::try_new(a, b, c, Real::one(), Real::one(), Real::one())
                    .unwrap();
            if family == 1 {
                BezierSubcurve2::RationalQuadratic(conic)
            } else {
                BezierSubcurve2::Rational(RationalBezier2::from(conic))
            }
        }
    }
    fn materialized(index: usize, curve: BezierSubcurve2) -> BezierArrangementFragment2 {
        BezierArrangementFragment2::new(
            index,
            0,
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve,
            },
        )
    }
    fn incoming() -> BezierArrangementFragment2 {
        materialized(
            0,
            BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                Point2::from_values(-1, 0),
                Point2::new(q(-1, 2), Real::zero()),
                Point2::from_values(0, 0),
            )),
        )
    }

    fn check(value: Classification<crate::BezierArrangementTraversal2>) {
        let traversal = decided(value);
        assert_eq!(traversal.chains()[0].fragment_indices(), [0, 2]);
        assert_eq!(traversal.chains()[1].fragment_indices(), [1]);
    }
    // The curves (t,t^2) and (u+u^2,u^2) have the same nonzero curvature.
    // At matched x=t=u+u^2, the first lies above the second by 2u^3+u^4>0.
    // Their order must not depend on a polynomial/conic/rational carrier,
    // or on representing the shared endpoint by a selected parameter.
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parameter = decided(
            BezierAlgebraicParameter2::try_isolate(
                decided(
                    BezierParameterPolynomial::try_new_power_basis(
                        vec![Real::from(-1), Real::from(2)],
                        &policy,
                    )
                    .unwrap(),
                ),
                decided(
                    BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap(),
                ),
                &policy,
            )
            .unwrap(),
        );
        for family in 0..3 {
            let graph = BezierArrangementGraph2::from_certified_fragments(vec![
                incoming(),
                materialized(1, source(family, false, false)),
                materialized(2, source(family, true, false)),
            ]);
            check(graph.traverse_retained_with_tangent_order(&policy));
            let endpoint = |index: usize, second: bool| {
                let source = source(family, second, true);
                let image = decided(
                    BezierAlgebraicEndpointImage2::from_source_curve(&source, &parameter, &policy)
                        .unwrap(),
                );
                BezierArrangementFragment2::new(
                    index,
                    0,
                    BezierSplitFragment2::RetainedBezier {
                        reversed: false,
                        start: BezierParameter2::Algebraic(parameter.clone()),
                        end: BezierParameter2::Exact(Real::one()),
                        source_curve: source,
                        start_image: Some(image),
                        end_image: None,
                    },
                )
            };
            let graph = BezierArrangementGraph2::from_certified_fragments(vec![
                incoming(),
                endpoint(1, false),
                endpoint(2, true),
            ]);
            check(graph.traverse_retained_with_tangent_order(&policy));
        }
    }
}

#[test]
fn mixed_native_and_selected_endpoints_share_tangent_ordering() {
    use crate::BezierArrangementFragment2;
    // The selected source uses t-1/2 at t=1/2; the native source uses t=0.
    // For order 2, y=x^2 precedes y=2*x^2. For order 3, the curve
    // (u+u^2,u^2) lies below (t,t^2) at matched x by 2*u^3+u^4.
    // These independent graph orders survive mixing endpoint carriers,
    // swapping candidate positions and choosing either selected branch.
    fn source(family: usize, second: bool, shifted: bool, order: usize) -> BezierSubcurve2 {
        let second_x = second && order == 3;
        let scale = Real::from(if second && order == 2 { 2 } else { 1 });
        let [a, b, c] = if shifted {
            [
                Point2::new(if second_x { q(-1, 4) } else { q(-1, 2) }, &scale * q(1, 4)),
                Point2::new(
                    if second_x { q(-1, 4) } else { Real::zero() },
                    &scale * q(-1, 4),
                ),
                Point2::new(if second_x { q(3, 4) } else { q(1, 2) }, &scale * q(1, 4)),
            ]
        } else {
            [
                Point2::from_values(0, 0),
                Point2::new(q(1, 2), Real::zero()),
                Point2::new(Real::from(if second_x { 2 } else { 1 }), scale),
            ]
        };
        if family == 0 {
            BezierSubcurve2::Quadratic(QuadraticBezier2::new(a, b, c))
        } else {
            let conic =
                RationalQuadraticBezier2::try_new(a, b, c, Real::one(), Real::one(), Real::one())
                    .unwrap();
            if family == 1 {
                BezierSubcurve2::RationalQuadratic(conic)
            } else {
                BezierSubcurve2::Rational(RationalBezier2::from(conic))
            }
        }
    }
    fn materialized(index: usize, curve: BezierSubcurve2) -> BezierArrangementFragment2 {
        BezierArrangementFragment2::new(
            index,
            0,
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve,
            },
        )
    }
    fn incoming() -> BezierArrangementFragment2 {
        materialized(
            0,
            BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                Point2::from_values(-1, 0),
                Point2::new(q(-1, 2), Real::zero()),
                Point2::from_values(0, 0),
            )),
        )
    }
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parameter = decided(
            BezierAlgebraicParameter2::try_isolate(
                decided(
                    BezierParameterPolynomial::try_new_power_basis(
                        vec![Real::from(-1), Real::from(2)],
                        &policy,
                    )
                    .unwrap(),
                ),
                decided(
                    BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap(),
                ),
                &policy,
            )
            .unwrap(),
        );
        for family in 0..3 {
            for order in [2, 3] {
                for selected_first in [false, true] {
                    for swapped in [false, true] {
                        let endpoint = |index: usize, second: bool| {
                            let selected = selected_first != second;
                            let source = source(family, second, selected, order);
                            if !selected {
                                return materialized(index, source);
                            }
                            let image = decided(
                                BezierAlgebraicEndpointImage2::from_source_curve(
                                    &source, &parameter, &policy,
                                )
                                .unwrap(),
                            );
                            BezierArrangementFragment2::new(
                                index,
                                0,
                                BezierSplitFragment2::RetainedBezier {
                                    reversed: false,
                                    start: BezierParameter2::Algebraic(parameter.clone()),
                                    end: BezierParameter2::Exact(Real::one()),
                                    source_curve: source,
                                    start_image: Some(image),
                                    end_image: None,
                                },
                            )
                        };
                        let graph = BezierArrangementGraph2::from_certified_fragments(vec![
                            incoming(),
                            endpoint(1, swapped),
                            endpoint(2, !swapped),
                        ]);
                        let chosen = if (order == 2) != swapped { 1 } else { 2 };
                        let traversal =
                            decided(graph.traverse_retained_with_tangent_order(&policy));
                        assert_eq!(traversal.chains()[0].fragment_indices(), [0, chosen]);
                        assert_eq!(traversal.chains()[1].fragment_indices(), [3 - chosen]);
                    }
                }
            }
        }
    }
}

#[test]
fn retained_source_tangents_order_without_coordinate_projection() {
    use crate::{BezierAlgebraicTangentVector2, BezierArrangementFragment2, HomogeneousControl2};
    fn line(index: usize, start: Point2, end: Point2) -> BezierArrangementFragment2 {
        let two = Real::from(2);
        let mid = Point2::new(
            ((start.x() + end.x()) / &two).unwrap(),
            ((start.y() + end.y()) / two).unwrap(),
        );
        BezierArrangementFragment2::new(
            index,
            0,
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(Real::zero()),
                end: BezierParameter2::Exact(Real::one()),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(start, mid, end)),
            },
        )
    }
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parameter = decided(
            BezierAlgebraicParameter2::try_isolate(
                decided(
                    BezierParameterPolynomial::try_new_power_basis(
                        vec![-Real::pi(), Real::zero(), Real::zero(), Real::from(4)],
                        &policy,
                    )
                    .unwrap(),
                ),
                decided(
                    BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap(),
                ),
                &policy,
            )
            .unwrap(),
        );
        // C(t)=(P(t),t*P(t)), P(t)=4*t^3-pi. At P(alpha)=0 its point is zero
        // and its nonzero tangent has slope alpha in (0,1).
        let xs = [
            -Real::pi(),
            -Real::pi(),
            -Real::pi(),
            Real::one() - Real::pi(),
            Real::from(4) - Real::pi(),
        ];
        let ys = [
            Real::zero(),
            (-Real::pi() / Real::from(4)).unwrap(),
            (-Real::pi() / Real::from(2)).unwrap(),
            (-Real::from(3) * Real::pi() / Real::from(4)).unwrap(),
            Real::from(4) - Real::pi(),
        ];
        let curve = decided(
            RationalBezier2::from_homogeneous_controls(
                xs.into_iter()
                    .zip(ys)
                    .map(|(x, y)| HomogeneousControl2::new(x, y, Real::one()))
                    .collect(),
                &policy,
            )
            .unwrap(),
        );
        let source = BezierSubcurve2::Rational(curve);
        let image = decided(
            BezierAlgebraicEndpointImage2::from_source_curve(&source, &parameter, &policy).unwrap(),
        );
        let point = decided(image.point().unwrap());
        let represented_point = point.x().and_then(|c| c.representation()).is_some()
            && point.y().and_then(|c| c.representation()).is_some();
        let tangent = decided(image.tangent().unwrap());
        let retained = tangent.retained_parameter().is_some();
        let represented_vector = BezierAlgebraicTangentVector2::from_image(tangent)
            .represented_coordinates()
            .is_some();
        assert!(represented_point && retained && !represented_vector);
        for reversed in [false, true] {
            for swapped in [false, true] {
                let candidate = |index| {
                    BezierArrangementFragment2::new(
                        index,
                        0,
                        BezierSplitFragment2::RetainedBezier {
                            reversed,
                            start: if reversed {
                                BezierParameter2::Exact(Real::zero())
                            } else {
                                BezierParameter2::Algebraic(parameter.clone())
                            },
                            end: if reversed {
                                BezierParameter2::Algebraic(parameter.clone())
                            } else {
                                BezierParameter2::Exact(Real::one())
                            },
                            source_curve: source.clone(),
                            start_image: (!reversed).then(|| image.clone()),
                            end_image: reversed.then(|| image.clone()),
                        },
                    )
                };
                let diagonal = |index| {
                    let end = if reversed {
                        -Real::pi()
                    } else {
                        Real::from(4) - Real::pi()
                    };
                    line(
                        index,
                        Point2::from_values(0, 0),
                        Point2::new(end.clone(), end),
                    )
                };
                let graph = BezierArrangementGraph2::from_certified_fragments(vec![
                    line(
                        0,
                        Point2::from_values(if reversed { 1 } else { -1 }, 0),
                        Point2::from_values(0, 0),
                    ),
                    if swapped { diagonal(1) } else { candidate(1) },
                    if swapped { candidate(2) } else { diagonal(2) },
                ]);
                let traversal = decided(graph.traverse_retained_with_tangent_order(&policy));
                let chosen = if swapped { 2 } else { 1 };
                assert_eq!(traversal.chains()[0].fragment_indices(), [0, chosen]);
                assert_eq!(traversal.chains()[1].fragment_indices(), [3 - chosen]);
            }
        }
    }
}

#[test]
fn retained_source_curvature_orders_without_scalar_projection() {
    use crate::{BezierAlgebraicTangentVector2, BezierArrangementFragment2, HomogeneousControl2};
    fn choose(n: i32, k: i32) -> i32 {
        if n < k {
            return 0;
        }
        (0..k).fold(1, |a, j| a * (n - j)) / (1..=k).product::<i32>()
    }
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let parameter = decided(
            BezierAlgebraicParameter2::try_isolate(
                decided(
                    BezierParameterPolynomial::try_new_power_basis(
                        vec![-Real::pi(), r(0), r(0), r(4)],
                        &policy,
                    )
                    .unwrap(),
                ),
                decided(BezierParameterInterval::try_new(r(0), r(1), &policy).unwrap()),
                &policy,
            )
            .unwrap(),
        );
        for retimed in [false, true] {
            for reflected in [false, true] {
                let candidate = |index, second| {
                    let scale: i32 = if retimed && second { 2 } else { 1 };
                    let parameter = if scale == 1 {
                        parameter.clone()
                    } else {
                        decided(
                            BezierAlgebraicParameter2::try_isolate(
                                decided(
                                    BezierParameterPolynomial::try_new_power_basis(
                                        vec![-Real::pi(), r(0), r(0), r(4 * scale.pow(3))],
                                        &policy,
                                    )
                                    .unwrap(),
                                ),
                                decided(
                                    BezierParameterInterval::try_new(r(0), q(1, scale), &policy)
                                        .unwrap(),
                                ),
                                &policy,
                            )
                            .unwrap(),
                        )
                    };
                    // A=(P,t*P), B=(P,t*P+P^2), P=4*t^3-pi. At the selected
                    // zero of P they have the same nonzero tangent. At matched
                    // x=P(t)>0, B_y-A_y=x^2, so A comes first counter-clockwise.
                    // Reflecting y reverses their angular order. A second chart
                    // t=2*u retains the same branch and exact x^2 separation.
                    let controls = (0..=6)
                        .map(|k| {
                            let x = -Real::pi() + q(scale.pow(3) * choose(k, 3), 5);
                            let mut y = -Real::pi() * q(scale * k, 6)
                                + q(4 * scale.pow(4) * choose(k, 4), 15);
                            if second {
                                y = y + Real::pi() * Real::pi()
                                    - Real::pi() * q(2 * scale.pow(3) * choose(k, 3), 5)
                                    + r(if k == 6 { 16 * scale.pow(6) } else { 0 });
                            }
                            if reflected {
                                y = -y;
                            }
                            HomogeneousControl2::new(x, y, r(1))
                        })
                        .collect();
                    let source = BezierSubcurve2::Rational(decided(
                        RationalBezier2::from_homogeneous_controls(controls, &policy).unwrap(),
                    ));
                    let image = decided(
                        BezierAlgebraicEndpointImage2::from_source_curve(
                            &source, &parameter, &policy,
                        )
                        .unwrap(),
                    );
                    let point = decided(image.point().unwrap());
                    assert!(point.x().and_then(|c| c.representation()).is_some());
                    assert!(point.y().and_then(|c| c.representation()).is_some());
                    let tangent = decided(image.tangent().unwrap());
                    assert!(tangent.retained_parameter().is_some());
                    assert!(
                        BezierAlgebraicTangentVector2::from_image(tangent)
                            .represented_coordinates()
                            .is_none()
                    );
                    BezierArrangementFragment2::new(
                        index,
                        0,
                        BezierSplitFragment2::RetainedBezier {
                            reversed: false,
                            start: BezierParameter2::Algebraic(parameter.clone()),
                            end: BezierParameter2::Exact(q(1, scale)),
                            source_curve: source,
                            start_image: Some(image),
                            end_image: None,
                        },
                    )
                };
                for swapped in [false, true] {
                    let incoming = BezierArrangementFragment2::new(
                        0,
                        0,
                        BezierSplitFragment2::Materialized {
                            start: BezierParameter2::Exact(r(0)),
                            end: BezierParameter2::Exact(r(1)),
                            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                                p(-1, 0),
                                Point2::new(q(-1, 2), r(0)),
                                p(0, 0),
                            )),
                        },
                    );
                    let graph = BezierArrangementGraph2::from_certified_fragments(vec![
                        incoming,
                        candidate(1, swapped),
                        candidate(2, !swapped),
                    ]);
                    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy));
                    let chosen = if swapped != reflected { 2 } else { 1 };
                    assert_eq!(traversal.chains()[0].fragment_indices(), [0, chosen]);
                    assert_eq!(traversal.chains()[1].fragment_indices(), [3 - chosen]);
                }
            }
        }
    }
}

#[test]
fn exact_endpoint_buckets_retain_symbolic_matches() {
    let symbolic = Real::pi();
    assert!(symbolic.exact_rational_ref().is_none());
    let first = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            Point2::new(&symbolic - r(2), r(0)),
            Point2::new(&symbolic - r(1), r(0)),
            Point2::new(symbolic.clone(), r(0)),
        )),
    };
    let second = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(r(0)),
        end: BezierParameter2::Exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            Point2::new(symbolic.clone(), r(0)),
            Point2::new(&symbolic + r(1), r(0)),
            Point2::new(&symbolic + r(2), r(0)),
        )),
    };
    let mut fragments = vec![
        crate::BezierArrangementFragment2::new(0, 0, first),
        crate::BezierArrangementFragment2::new(1, 0, second),
    ];
    // Enough rational fragments to enable exact endpoint bucketing.
    for index in 0..14 {
        let x = 100 + index * 3;
        fragments.push(crate::BezierArrangementFragment2::new(
            usize::try_from(index + 2).unwrap(),
            0,
            BezierSplitFragment2::Materialized {
                start: BezierParameter2::Exact(r(0)),
                end: BezierParameter2::Exact(r(1)),
                curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                    p(x, 0),
                    p(x + 1, 0),
                    p(x + 2, 0),
                )),
            },
        ));
    }
    let graph = graph(fragments);

    let tangent_ordered = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(tangent_ordered.chains().len(), 15);
    assert_eq!(tangent_ordered.chains()[0].fragment_indices(), &[0, 1]);
}

#[test]
fn exact_split_fragments_traverse_as_one_closed_bezier_chain() {
    let upper = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let lower = QuadraticBezier2::new(p(4, 0), p(2, -4), p(0, 0));
    let mut fragments = Vec::new();
    for (source, curve) in [upper, lower].into_iter().enumerate() {
        let split = decided(
            curve
                .split_at_parameters(&[BezierParameter2::Exact(q(1, 2))], &policy())
                .unwrap(),
        );
        for (index, fragment) in split.fragments().iter().enumerate() {
            fragments.push(crate::BezierArrangementFragment2::new(
                source,
                index,
                fragment.clone(),
            ));
        }
    }
    let graph = graph(fragments);
    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));

    assert_eq!(graph.fragments().len(), 4);
    assert_eq!(traversal.chains().len(), 1);
    assert!(traversal.chains()[0].is_closed());
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1, 2, 3]);
}
