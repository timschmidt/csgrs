use hypercurve::{
    BezierAlgebraicEndpointImage2, BezierAlgebraicParameter2, BezierArrangementGraph2,
    BezierParameter2, BezierParameterInterval, BezierParameterPolynomial,
    BezierRetainedLineOverlapExtent2, BezierRetainedOverlap2, BezierRetainedOverlapOrientation2,
    BezierRetainedOverlapRelation2, BezierRetainedOverlapReport2, BezierSplitFragment2,
    BezierSubcurve2, Classification, CubicBezier2, CurveError, CurvePolicy, Point2,
    QuadraticBezier2, RationalQuadraticBezier2, Real, UncertaintyReason,
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

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
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

fn exact(value: Real) -> BezierParameter2 {
    decided(BezierParameter2::exact(value, &policy()).unwrap())
}

fn algebraic_sqrt_half() -> BezierParameter2 {
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis(vec![r(-1), r(0), r(2)], &policy()).unwrap(),
    );
    let interval = decided(BezierParameterInterval::try_new(q(2, 3), q(3, 4), &policy()).unwrap());
    BezierParameter2::algebraic(decided(
        BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap(),
    ))
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
    BezierAlgebraicEndpointImage2::quadratic(curve, parameter, &policy()).unwrap()
}

fn algebraic_cubic_endpoint_image(
    curve: &CubicBezier2,
    parameter: &BezierAlgebraicParameter2,
) -> BezierAlgebraicEndpointImage2 {
    BezierAlgebraicEndpointImage2::cubic(curve, parameter, &policy()).unwrap()
}

fn algebraic_rational_endpoint_image(
    curve: &RationalQuadraticBezier2,
    parameter: &BezierAlgebraicParameter2,
) -> BezierAlgebraicEndpointImage2 {
    BezierAlgebraicEndpointImage2::rational_quadratic(curve, parameter, &policy()).unwrap()
}

#[test]
fn exact_split_fragments_traverse_as_one_closed_bezier_chain() {
    let upper = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let lower = QuadraticBezier2::new(p(4, 0), p(2, -4), p(0, 0));
    let upper_split = decided(
        upper
            .split_at_parameters(&[exact(q(1, 2))], &policy())
            .unwrap(),
    );
    let lower_split = decided(
        lower
            .split_at_parameters(&[exact(q(1, 2))], &policy())
            .unwrap(),
    );

    let graph = BezierArrangementGraph2::from_split_materializations(&[upper_split, lower_split]);
    let traversal = decided(graph.traverse_branch_free(&policy()));

    assert_eq!(graph.len(), 4);
    assert_eq!(traversal.len(), 1);
    assert_eq!(traversal.closed_count(), 1);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1, 2, 3]);
}

#[test]
fn branch_vertex_is_explicit_uncertainty_not_arbitrary_successor() {
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0))),
    };
    let second = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Cubic(CubicBezier2::new(p(2, 0), p(3, 1), p(4, 1), p(5, 0))),
    };
    let third = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(3, -1), p(4, 0))),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, second),
        hypercurve::BezierArrangementFragment2::new(2, 0, third),
    ]);

    assert_eq!(
        graph.traverse_branch_free(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn tangent_ordered_traversal_resolves_simple_branch_vertex() {
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0))),
    };
    let upward = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Cubic(CubicBezier2::new(p(2, 0), p(3, 1), p(4, 1), p(5, 0))),
    };
    let straightest = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(3, -1), p(4, 0))),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, upward),
        hypercurve::BezierArrangementFragment2::new(2, 0, straightest),
    ]);
    let traversal = decided(graph.traverse_with_tangent_order(&policy()));

    assert_eq!(traversal.len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 2]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[1]);
}

#[test]
fn tangent_ordered_traversal_uses_second_order_for_equal_outgoing_tangents() {
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let first_out = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(3, 1), p(4, 0))),
    };
    let second_out = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(4, 2), p(5, 0))),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, first_out),
        hypercurve::BezierArrangementFragment2::new(2, 0, second_out),
    ]);

    let traversal = decided(graph.traverse_with_tangent_order(&policy()));
    assert_eq!(traversal.len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 2]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[1]);

    let retained_traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(retained_traversal.len(), 2);
    assert_eq!(retained_traversal.chains()[0].fragment_indices(), &[0, 2]);
    assert_eq!(retained_traversal.chains()[1].fragment_indices(), &[1]);
}

#[test]
fn tangent_ordered_traversal_rejects_equal_second_order_outgoing_tangents() {
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let first_out = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(3, 1), p(4, 0))),
    };
    let second_out = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(3, 1), p(4, 0))),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, first_out),
        hypercurve::BezierArrangementFragment2::new(2, 0, second_out),
    ]);

    assert_eq!(
        graph.traverse_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    assert_eq!(
        graph.traverse_retained_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn tangent_ordered_traversal_uses_rational_second_order_for_equal_outgoing_tangents() {
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let upward = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(p(2, 0), p(3, 0), p(4, 1), r(1), r(2), r(3)).unwrap(),
        ),
    };
    let downward = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(p(2, 0), p(3, 0), p(4, -1), r(1), r(2), r(3))
                .unwrap(),
        ),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, upward),
        hypercurve::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    let traversal = decided(graph.traverse_with_tangent_order(&policy()));
    assert_eq!(traversal.len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);

    let retained_traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(retained_traversal.len(), 2);
    assert_eq!(retained_traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(retained_traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn tangent_ordered_traversal_rejects_equal_rational_second_order_successors() {
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let first_out = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(p(2, 0), p(3, 0), p(4, 1), r(1), r(2), r(3)).unwrap(),
        ),
    };
    let second_out = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::RationalQuadratic(
            RationalQuadraticBezier2::try_new(p(2, 0), p(3, 0), p(4, 1), r(1), r(2), r(3)).unwrap(),
        ),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, first_out),
        hypercurve::BezierArrangementFragment2::new(2, 0, second_out),
    ]);

    assert_eq!(
        graph.traverse_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    assert_eq!(
        graph.traverse_retained_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn tangent_ordered_traversal_uses_third_order_for_cubic_same_tangent_inflections() {
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let upward = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Cubic(CubicBezier2::new(p(2, 0), p(3, 0), p(4, 0), p(5, 1))),
    };
    let downward = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Cubic(CubicBezier2::new(p(2, 0), p(3, 0), p(4, 0), p(5, -1))),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, upward),
        hypercurve::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    let traversal = decided(graph.traverse_with_tangent_order(&policy()));
    assert_eq!(traversal.len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);

    let retained_traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(retained_traversal.len(), 2);
    assert_eq!(retained_traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(retained_traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn tangent_ordered_traversal_rejects_equal_third_order_cubic_successors() {
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let first_out = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Cubic(CubicBezier2::new(p(2, 0), p(3, 0), p(4, 0), p(5, 1))),
    };
    let second_out = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Cubic(CubicBezier2::new(p(2, 0), p(3, 0), p(4, 0), p(5, 1))),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, first_out),
        hypercurve::BezierArrangementFragment2::new(2, 0, second_out),
    ]);

    assert_eq!(
        graph.traverse_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    assert_eq!(
        graph.traverse_retained_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn algebraic_split_boundary_blocks_graph_traversal() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let split = decided(
        curve
            .split_at_parameters(&[algebraic_sqrt_half(), exact(q(4, 5))], &policy())
            .unwrap(),
    );
    let graph = BezierArrangementGraph2::from_split_materializations(&[split]);

    assert_eq!(
        graph.traverse_branch_free(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn retained_tangent_order_traverses_algebraic_branch_vertex() {
    let parameter = algebraic_midpoint_parameter();
    let algebraic = BezierParameter2::algebraic(parameter.clone());
    let incoming_curve = through_origin_with_midpoint_tangent(1, 0);
    let upward_curve = through_origin_with_midpoint_tangent(0, 1);
    let downward_curve = through_origin_with_midpoint_tangent(0, -1);
    let incoming = BezierSplitFragment2::AlgebraicEndpointImages {
        start: exact(r(0)),
        end: algebraic.clone(),
        source_curve: None,
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&incoming_curve, &parameter)),
    };
    let upward = BezierSplitFragment2::AlgebraicEndpointImages {
        start: algebraic.clone(),
        end: exact(r(1)),
        source_curve: None,
        start_image: Some(algebraic_endpoint_image(&upward_curve, &parameter)),
        end_image: None,
    };
    let downward = BezierSplitFragment2::AlgebraicEndpointImages {
        start: algebraic,
        end: exact(r(1)),
        source_curve: None,
        start_image: Some(algebraic_endpoint_image(&downward_curve, &parameter)),
        end_image: None,
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, incoming),
        hypercurve::BezierArrangementFragment2::new(1, 0, upward),
        hypercurve::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    assert_eq!(
        graph.traverse_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));

    assert_eq!(traversal.len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn retained_tangent_order_rejects_equal_algebraic_successors() {
    let parameter = algebraic_midpoint_parameter();
    let algebraic = BezierParameter2::algebraic(parameter.clone());
    let incoming_curve = through_origin_with_midpoint_tangent(1, 0);
    let first_curve = through_origin_with_midpoint_tangent(0, 1);
    let second_curve = through_origin_with_midpoint_tangent(0, 1);
    let incoming = BezierSplitFragment2::AlgebraicEndpointImages {
        start: exact(r(0)),
        end: algebraic.clone(),
        source_curve: None,
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&incoming_curve, &parameter)),
    };
    let first = BezierSplitFragment2::AlgebraicEndpointImages {
        start: algebraic.clone(),
        end: exact(r(1)),
        source_curve: None,
        start_image: Some(algebraic_endpoint_image(&first_curve, &parameter)),
        end_image: None,
    };
    let second = BezierSplitFragment2::AlgebraicEndpointImages {
        start: algebraic,
        end: exact(r(1)),
        source_curve: None,
        start_image: Some(algebraic_endpoint_image(&second_curve, &parameter)),
        end_image: None,
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, incoming),
        hypercurve::BezierArrangementFragment2::new(1, 0, first),
        hypercurve::BezierArrangementFragment2::new(2, 0, second),
    ]);

    assert_eq!(
        graph.traverse_retained_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn retained_tangent_order_uses_algebraic_second_order_for_equal_successors() {
    let parameter = algebraic_midpoint_parameter();
    let algebraic = BezierParameter2::algebraic(parameter.clone());
    let incoming_curve = through_origin_with_midpoint_tangent(1, 0);
    let upward_curve = through_origin_with_horizontal_midpoint_tangent(1);
    let downward_curve = through_origin_with_horizontal_midpoint_tangent(-1);
    let incoming = BezierSplitFragment2::AlgebraicEndpointImages {
        start: exact(r(0)),
        end: algebraic.clone(),
        source_curve: None,
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&incoming_curve, &parameter)),
    };
    let upward = BezierSplitFragment2::AlgebraicEndpointImages {
        start: algebraic.clone(),
        end: exact(r(1)),
        source_curve: None,
        start_image: Some(algebraic_endpoint_image(&upward_curve, &parameter)),
        end_image: None,
    };
    let downward = BezierSplitFragment2::AlgebraicEndpointImages {
        start: algebraic,
        end: exact(r(1)),
        source_curve: None,
        start_image: Some(algebraic_endpoint_image(&downward_curve, &parameter)),
        end_image: None,
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, incoming),
        hypercurve::BezierArrangementFragment2::new(1, 0, upward),
        hypercurve::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(traversal.len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn retained_tangent_order_uses_rational_algebraic_second_order_for_equal_successors() {
    let parameter = algebraic_midpoint_parameter();
    let algebraic = BezierParameter2::algebraic(parameter.clone());
    let incoming_curve = through_origin_with_midpoint_tangent(1, 0);
    let upward_curve = rational_through_origin_with_horizontal_midpoint_tangent(1);
    let downward_curve = rational_through_origin_with_horizontal_midpoint_tangent(-1);
    let incoming = BezierSplitFragment2::AlgebraicEndpointImages {
        start: exact(r(0)),
        end: algebraic.clone(),
        source_curve: None,
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&incoming_curve, &parameter)),
    };
    let upward = BezierSplitFragment2::AlgebraicEndpointImages {
        start: algebraic.clone(),
        end: exact(r(1)),
        source_curve: None,
        start_image: Some(algebraic_rational_endpoint_image(&upward_curve, &parameter)),
        end_image: None,
    };
    let downward = BezierSplitFragment2::AlgebraicEndpointImages {
        start: algebraic,
        end: exact(r(1)),
        source_curve: None,
        start_image: Some(algebraic_rational_endpoint_image(
            &downward_curve,
            &parameter,
        )),
        end_image: None,
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, incoming),
        hypercurve::BezierArrangementFragment2::new(1, 0, upward),
        hypercurve::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(traversal.len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn retained_tangent_order_uses_algebraic_third_order_for_cubic_same_tangent_inflections() {
    let parameter = algebraic_midpoint_parameter();
    let algebraic = BezierParameter2::algebraic(parameter.clone());
    let incoming_curve = through_origin_with_midpoint_tangent(1, 0);
    let upward_curve = through_origin_with_horizontal_midpoint_tangent_and_third_order(8);
    let downward_curve = through_origin_with_horizontal_midpoint_tangent_and_third_order(-8);
    let incoming = BezierSplitFragment2::AlgebraicEndpointImages {
        start: exact(r(0)),
        end: algebraic.clone(),
        source_curve: None,
        start_image: None,
        end_image: Some(algebraic_endpoint_image(&incoming_curve, &parameter)),
    };
    let upward = BezierSplitFragment2::AlgebraicEndpointImages {
        start: algebraic.clone(),
        end: exact(r(1)),
        source_curve: None,
        start_image: Some(algebraic_cubic_endpoint_image(&upward_curve, &parameter)),
        end_image: None,
    };
    let downward = BezierSplitFragment2::AlgebraicEndpointImages {
        start: algebraic,
        end: exact(r(1)),
        source_curve: None,
        start_image: Some(algebraic_cubic_endpoint_image(&downward_curve, &parameter)),
        end_image: None,
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, incoming),
        hypercurve::BezierArrangementFragment2::new(1, 0, upward),
        hypercurve::BezierArrangementFragment2::new(2, 0, downward),
    ]);

    assert_eq!(
        graph.traverse_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    assert_eq!(traversal.len(), 2);
    assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    assert_eq!(traversal.chains()[1].fragment_indices(), &[2]);
}

#[test]
fn retained_overlap_report_finds_identical_materialized_fragments() {
    let curve = QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0));
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(curve.clone()),
    };
    let second = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(curve),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, second),
    ]);

    let report = decided(BezierRetainedOverlapReport2::from_graph(&graph, &policy()));

    assert_eq!(report.len(), 1);
    assert_eq!(report.overlaps()[0].first_fragment_index(), 0);
    assert_eq!(report.overlaps()[0].second_fragment_index(), 1);
    assert!(matches!(
        report.overlaps()[0].relation(),
        BezierRetainedOverlapRelation2::SameControlPolygon
    ));
}

#[test]
fn retained_overlap_pair_constructor_rejects_unordered_indices() {
    assert_topology_error(BezierRetainedOverlap2::new(
        0,
        0,
        BezierRetainedOverlapRelation2::SameControlPolygon,
    ));
    assert_topology_error(BezierRetainedOverlap2::new(
        2,
        1,
        BezierRetainedOverlapRelation2::SameControlPolygon,
    ));
}

#[test]
fn retained_overlap_report_finds_reversed_degree_elevated_same_image() {
    let quadratic = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let cubic_reversed = CubicBezier2::new(
        p(4, 0),
        Point2::new(q(8, 3), q(8, 3)),
        Point2::new(q(4, 3), q(8, 3)),
        p(0, 0),
    );
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(
            0,
            0,
            BezierSplitFragment2::Materialized {
                start: exact(r(0)),
                end: exact(r(1)),
                curve: BezierSubcurve2::Quadratic(quadratic),
            },
        ),
        hypercurve::BezierArrangementFragment2::new(
            1,
            0,
            BezierSplitFragment2::Materialized {
                start: exact(r(0)),
                end: exact(r(1)),
                curve: BezierSubcurve2::Cubic(cubic_reversed),
            },
        ),
    ]);

    let report = decided(BezierRetainedOverlapReport2::from_graph(&graph, &policy()));

    assert_eq!(report.len(), 1);
    assert!(matches!(
        report.overlaps()[0].relation(),
        BezierRetainedOverlapRelation2::SameCurveImage
    ));
}

#[test]
fn retained_overlap_report_separates_endpoint_touch_from_overlap() {
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0))),
    };
    let second = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(3, -1), p(4, 0))),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, second),
    ]);

    let report = decided(BezierRetainedOverlapReport2::from_graph(&graph, &policy()));

    assert!(report.is_empty());
}

#[test]
fn retained_overlap_report_extracts_partial_line_image_split_ranges() {
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(2, 0), p(4, 0))),
    };
    let second = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(4, 0), p(6, 0))),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, second),
    ]);
    let report = decided(BezierRetainedOverlapReport2::from_graph(&graph, &policy()));

    let splits = decided(report.line_overlap_splits(&policy()));

    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].first_fragment_index(), 0);
    assert_eq!(splits[0].second_fragment_index(), 1);
    assert_eq!(splits[0].overlap_segment().start(), &p(2, 0));
    assert_eq!(splits[0].overlap_segment().end(), &p(4, 0));
    assert_eq!(splits[0].first_line_range().start(), &q(1, 2));
    assert_eq!(splits[0].first_line_range().end(), &r(1));
    assert_eq!(splits[0].second_line_range().start(), &r(0));
    assert_eq!(splits[0].second_line_range().end(), &q(1, 2));
    assert_eq!(
        splits[0].extent(),
        BezierRetainedLineOverlapExtent2::PartialBoth
    );
    assert_eq!(
        graph.traverse_retained_deduplicating_materialized_overlaps(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );

    let bezier_splits = decided(report.linear_bezier_overlap_splits(&graph, &policy()));
    assert_eq!(bezier_splits.len(), 1);
    assert_eq!(bezier_splits[0].first_bezier_range().start(), &q(1, 2));
    assert_eq!(bezier_splits[0].first_bezier_range().end(), &r(1));
    assert_eq!(bezier_splits[0].second_bezier_range().start(), &r(0));
    assert_eq!(bezier_splits[0].second_bezier_range().end(), &q(1, 2));
    assert_eq!(
        bezier_splits[0].extent(),
        BezierRetainedLineOverlapExtent2::PartialBoth
    );

    let refinement = decided(graph.split_retained_linear_overlaps(&policy()));
    assert_eq!(refinement.overlap_report().len(), 1);
    assert_eq!(refinement.split_plan().len(), 1);
    assert_eq!(refinement.resolved_overlaps().len(), 1);
    assert_eq!(refinement.graph().len(), 4);
    assert_eq!(refinement.refined_fragments().len(), 4);
    assert_eq!(
        refinement.resolved_overlaps()[0].first_refined_fragment_index(),
        1
    );
    assert_eq!(
        refinement.resolved_overlaps()[0].second_refined_fragment_index(),
        2
    );
    assert_eq!(
        refinement.resolved_overlaps()[0].orientation(),
        BezierRetainedOverlapOrientation2::Same
    );
    assert_eq!(
        refinement.refined_fragments()[0].original_fragment_index(),
        0
    );
    assert_eq!(
        refinement.refined_fragments()[0].local_range(),
        &hypercurve::ParamRange::new(r(0), q(1, 2))
    );
    assert_eq!(
        refinement.refined_fragments()[1].original_fragment_index(),
        0
    );
    assert_eq!(
        refinement.refined_fragments()[1].local_range(),
        &hypercurve::ParamRange::new(q(1, 2), r(1))
    );
    assert_eq!(
        refinement.refined_fragments()[2].original_fragment_index(),
        1
    );
    assert_eq!(
        refinement.refined_fragments()[2].local_range(),
        &hypercurve::ParamRange::new(r(0), q(1, 2))
    );
    assert_eq!(
        refinement.refined_fragments()[3].original_fragment_index(),
        1
    );
    assert_eq!(
        refinement.refined_fragments()[3].local_range(),
        &hypercurve::ParamRange::new(q(1, 2), r(1))
    );
    let refined = refinement.graph().fragments();
    let BezierSplitFragment2::Materialized {
        start,
        end,
        curve: BezierSubcurve2::Quadratic(overlap_from_first),
    } = refined[1].fragment()
    else {
        panic!("expected exact quadratic overlap fragment from first curve");
    };
    assert_eq!(start, &exact(q(1, 2)));
    assert_eq!(end, &exact(r(1)));
    assert_eq!(overlap_from_first.start(), &p(2, 0));
    assert_eq!(overlap_from_first.end(), &p(4, 0));
    let BezierSplitFragment2::Materialized {
        start,
        end,
        curve: BezierSubcurve2::Quadratic(overlap_from_second),
    } = refined[2].fragment()
    else {
        panic!("expected exact quadratic overlap fragment from second curve");
    };
    assert_eq!(start, &exact(r(0)));
    assert_eq!(end, &exact(q(1, 2)));
    assert_eq!(overlap_from_second.start(), &p(2, 0));
    assert_eq!(overlap_from_second.end(), &p(4, 0));
}

#[test]
fn retained_linear_overlap_refinement_reports_reversed_span_orientation() {
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(2, 0), p(4, 0))),
    };
    let reversed_overlap = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(4, 0), p(3, 0), p(2, 0))),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, reversed_overlap),
    ]);

    let refinement = decided(graph.split_retained_linear_overlaps(&policy()));

    assert_eq!(refinement.graph().len(), 3);
    assert_eq!(refinement.resolved_overlaps().len(), 1);
    let resolved = &refinement.resolved_overlaps()[0];
    assert_eq!(resolved.first_refined_fragment_index(), 1);
    assert_eq!(resolved.second_refined_fragment_index(), 2);
    assert_eq!(resolved.first_original_fragment_index(), 0);
    assert_eq!(resolved.second_original_fragment_index(), 1);
    assert_eq!(
        resolved.first_local_range(),
        &hypercurve::ParamRange::new(q(1, 2), r(1))
    );
    assert_eq!(
        resolved.second_local_range(),
        &hypercurve::ParamRange::new(r(1), r(0))
    );
    assert_eq!(resolved.overlap_segment().start(), &p(2, 0));
    assert_eq!(resolved.overlap_segment().end(), &p(4, 0));
    assert_eq!(
        resolved.orientation(),
        BezierRetainedOverlapOrientation2::Opposite
    );
    assert_eq!(
        resolved.extent(),
        BezierRetainedLineOverlapExtent2::PartialFirstFullSecond
    );
    let traversal = decided(graph.traverse_retained_splitting_linear_overlaps(&policy()));
    assert_eq!(
        traversal.refined_traversal().shadowed_fragment_indices(),
        &[1, 2]
    );
    assert_eq!(traversal.traversal().len(), 1);
    assert_eq!(traversal.traversal().closed_count(), 0);
    assert_eq!(traversal.traversal().chains()[0].fragment_indices(), &[0]);
}

#[test]
fn retained_linear_overlap_traversal_splits_and_consumes_duplicate_span_in_loop() {
    let bottom = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(2, 0), p(4, 0))),
    };
    let overlapping_bottom_tail = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(3, 0), p(4, 0))),
    };
    let right = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(4, 0), p(4, 1), p(4, 2))),
    };
    let top = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(4, 2), p(2, 2), p(0, 2))),
    };
    let left = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 2), p(0, 1), p(0, 0))),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, bottom),
        hypercurve::BezierArrangementFragment2::new(1, 0, overlapping_bottom_tail),
        hypercurve::BezierArrangementFragment2::new(2, 0, right),
        hypercurve::BezierArrangementFragment2::new(3, 0, top),
        hypercurve::BezierArrangementFragment2::new(4, 0, left),
    ]);

    assert_eq!(
        graph.traverse_retained_deduplicating_materialized_overlaps(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    let traversal = decided(graph.traverse_retained_splitting_linear_overlaps(&policy()));

    assert_eq!(traversal.refinement().graph().len(), 6);
    assert_eq!(traversal.refinement().split_plan().len(), 1);
    assert_eq!(traversal.refinement().resolved_overlaps().len(), 1);
    assert_eq!(
        traversal.refinement().resolved_overlaps()[0].orientation(),
        BezierRetainedOverlapOrientation2::Same
    );
    assert_eq!(
        traversal.refined_traversal().shadowed_fragment_indices(),
        &[2]
    );
    assert_eq!(traversal.traversal().len(), 1);
    assert_eq!(traversal.traversal().closed_count(), 1);
    assert_eq!(
        traversal.traversal().chains()[0].fragment_indices(),
        &[0, 1, 3, 4, 5]
    );
    assert_eq!(
        traversal.refinement().refined_fragments()[1].local_range(),
        &hypercurve::ParamRange::new(q(1, 2), r(1))
    );
    assert_eq!(
        traversal.refinement().refined_fragments()[2].local_range(),
        &hypercurve::ParamRange::new(r(0), r(1))
    );
}

#[test]
fn retained_linear_overlap_traversal_cancels_reversed_internal_span_in_loop() {
    let left_bottom = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0))),
    };
    let shared_up = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(2, 1), p(2, 2))),
    };
    let left_top = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 2), p(1, 2), p(0, 2))),
    };
    let left_edge = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 2), p(0, 1), p(0, 0))),
    };
    let right_bottom = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 0), p(3, 0), p(4, 0))),
    };
    let right_edge = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(4, 0), p(4, 1), p(4, 2))),
    };
    let right_top = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(4, 2), p(3, 2), p(2, 2))),
    };
    let shared_down = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(2, 2), p(2, 1), p(2, 0))),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, left_bottom),
        hypercurve::BezierArrangementFragment2::new(0, 1, shared_up),
        hypercurve::BezierArrangementFragment2::new(0, 2, left_top),
        hypercurve::BezierArrangementFragment2::new(0, 3, left_edge),
        hypercurve::BezierArrangementFragment2::new(1, 0, right_bottom),
        hypercurve::BezierArrangementFragment2::new(1, 1, right_edge),
        hypercurve::BezierArrangementFragment2::new(1, 2, right_top),
        hypercurve::BezierArrangementFragment2::new(1, 3, shared_down),
    ]);

    assert_eq!(
        graph.traverse_retained_deduplicating_materialized_overlaps(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    let traversal = decided(graph.traverse_retained_splitting_linear_overlaps(&policy()));

    assert_eq!(traversal.refinement().overlap_report().len(), 1);
    assert!(traversal.refinement().resolved_overlaps().is_empty());
    assert_eq!(
        traversal.refined_traversal().shadowed_fragment_indices(),
        &[1, 7]
    );
    assert_eq!(traversal.traversal().len(), 1);
    assert_eq!(traversal.traversal().closed_count(), 1);
    assert_eq!(
        traversal.traversal().chains()[0].fragment_indices(),
        &[0, 4, 5, 6, 2, 3]
    );
}

#[test]
fn retained_overlap_report_does_not_call_same_curve_image_a_line_split() {
    let curve = QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0));
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(
            0,
            0,
            BezierSplitFragment2::Materialized {
                start: exact(r(0)),
                end: exact(r(1)),
                curve: BezierSubcurve2::Quadratic(curve.clone()),
            },
        ),
        hypercurve::BezierArrangementFragment2::new(
            1,
            0,
            BezierSplitFragment2::Materialized {
                start: exact(r(0)),
                end: exact(r(1)),
                curve: BezierSubcurve2::Quadratic(curve),
            },
        ),
    ]);
    let report = decided(BezierRetainedOverlapReport2::from_graph(&graph, &policy()));

    assert!(decided(report.line_overlap_splits(&policy())).is_empty());
}

#[test]
fn retained_overlap_report_rejects_nonlinear_line_image_bezier_ranges() {
    let first = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(0, 0), p(1, 0), p(4, 0))),
    };
    let second = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(1, 0), p(3, 0), p(5, 0))),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, first),
        hypercurve::BezierArrangementFragment2::new(1, 0, second),
    ]);
    let report = decided(BezierRetainedOverlapReport2::from_graph(&graph, &policy()));

    assert_eq!(decided(report.line_overlap_splits(&policy())).len(), 1);
    assert_eq!(
        report.linear_bezier_overlap_splits(&graph, &policy()),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        graph.split_retained_linear_overlaps(&policy()),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn retained_overlap_report_does_not_sample_algebraic_endpoint_image_fragments() {
    let parameter = algebraic_midpoint_parameter();
    let algebraic = BezierParameter2::algebraic(parameter.clone());
    let curve = through_origin_with_midpoint_tangent(1, 0);
    let fragment = BezierSplitFragment2::AlgebraicEndpointImages {
        start: algebraic.clone(),
        end: algebraic,
        source_curve: None,
        start_image: Some(algebraic_endpoint_image(&curve, &parameter)),
        end_image: Some(algebraic_endpoint_image(&curve, &parameter)),
    };
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(0, 0, fragment.clone()),
        hypercurve::BezierArrangementFragment2::new(1, 0, fragment),
    ]);

    let report = decided(BezierRetainedOverlapReport2::from_graph(&graph, &policy()));

    assert!(report.is_empty());
}

#[test]
fn retained_overlap_traversal_deduplicates_oriented_duplicate_loop_edges() {
    let edges = [
        QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0)),
        QuadraticBezier2::new(p(2, 0), p(2, 1), p(2, 2)),
        QuadraticBezier2::new(p(2, 2), p(1, 2), p(0, 2)),
        QuadraticBezier2::new(p(0, 2), p(0, 1), p(0, 0)),
    ];
    let mut fragments = Vec::new();
    for (edge_index, edge) in edges.iter().cloned().enumerate() {
        for duplicate_index in 0..2 {
            fragments.push(hypercurve::BezierArrangementFragment2::new(
                edge_index,
                duplicate_index,
                BezierSplitFragment2::Materialized {
                    start: exact(r(0)),
                    end: exact(r(1)),
                    curve: BezierSubcurve2::Quadratic(edge.clone()),
                },
            ));
        }
    }
    let graph = BezierArrangementGraph2::new(fragments);

    assert_eq!(
        graph.traverse_retained_with_tangent_order(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    let overlap_traversal =
        decided(graph.traverse_retained_deduplicating_materialized_overlaps(&policy()));

    assert_eq!(overlap_traversal.overlap_report().len(), 4);
    assert_eq!(overlap_traversal.shadowed_fragment_indices(), &[1, 3, 5, 7]);
    assert_eq!(overlap_traversal.traversal().len(), 1);
    assert_eq!(overlap_traversal.traversal().closed_count(), 1);
    assert_eq!(
        overlap_traversal.traversal().chains()[0].fragment_indices(),
        &[0, 2, 4, 6]
    );
}

#[test]
fn retained_overlap_traversal_rejects_reversed_duplicate_as_ownership_boundary() {
    let forward = QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0));
    let reversed = QuadraticBezier2::new(p(2, 0), p(1, 2), p(0, 0));
    let graph = BezierArrangementGraph2::new(vec![
        hypercurve::BezierArrangementFragment2::new(
            0,
            0,
            BezierSplitFragment2::Materialized {
                start: exact(r(0)),
                end: exact(r(1)),
                curve: BezierSubcurve2::Quadratic(forward),
            },
        ),
        hypercurve::BezierArrangementFragment2::new(
            1,
            0,
            BezierSplitFragment2::Materialized {
                start: exact(r(0)),
                end: exact(r(1)),
                curve: BezierSubcurve2::Quadratic(reversed),
            },
        ),
    ]);

    let report = decided(BezierRetainedOverlapReport2::from_graph(&graph, &policy()));
    assert_eq!(report.len(), 1);
    assert_eq!(
        graph.traverse_retained_deduplicating_materialized_overlaps(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

proptest! {
    #[test]
    fn open_quadratic_chain_stays_one_nonclosed_chain(
        middle_y in -16_i32..=16,
    ) {
        let first = QuadraticBezier2::new(p(0, 0), p(1, middle_y), p(2, 0));
        let second = QuadraticBezier2::new(p(2, 0), p(3, -middle_y), p(4, 0));
        let first_split = decided(first.split_at_parameters(&[], &policy()).unwrap());
        let second_split = decided(second.split_at_parameters(&[], &policy()).unwrap());
        let graph = BezierArrangementGraph2::from_split_materializations(&[first_split, second_split]);
        let traversal = match graph.traverse_branch_free(&policy()) {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                return Err(TestCaseError::fail(format!("unexpected uncertainty: {reason:?}")));
            }
        };

        prop_assert_eq!(traversal.len(), 1);
        prop_assert_eq!(traversal.closed_count(), 0);
        prop_assert_eq!(traversal.chains()[0].fragment_indices(), &[0, 1]);
    }
}
