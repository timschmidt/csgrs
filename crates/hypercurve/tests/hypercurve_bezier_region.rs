use hypercurve::{
    BezierAlgebraicEndpointImage2, BezierAlgebraicParameter2, BezierArrangementFragment2,
    BezierArrangementGraph2, BezierParameter2, BezierParameterInterval, BezierParameterPolynomial,
    BezierRegion2, BezierRetainedRegion2, BezierSplitFragment2, Classification, CurvePolicy,
    Point2, QuadraticBezier2, RationalQuadraticBezier2, Real, UncertaintyReason,
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

fn exact(value: Real) -> BezierParameter2 {
    decided(BezierParameter2::exact(value, &policy()).unwrap())
}

fn algebraic_midpoint_parameter() -> BezierAlgebraicParameter2 {
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis(vec![r(-1), r(2)], &policy()).unwrap(),
    );
    let interval = decided(BezierParameterInterval::try_new(q(2, 5), q(3, 5), &policy()).unwrap());
    decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap())
}

fn algebraic_image(curve: &QuadraticBezier2) -> BezierAlgebraicEndpointImage2 {
    BezierAlgebraicEndpointImage2::quadratic(curve, &algebraic_midpoint_parameter(), &policy())
        .unwrap()
}

fn line_midpoint_curve(start_x: i32, mid_x: i32, end_x: i32) -> QuadraticBezier2 {
    QuadraticBezier2::new(p(start_x, 0), p(mid_x, 0), p(end_x, 0))
}

#[test]
fn closed_polynomial_arrangement_materializes_retained_region_with_exact_area() {
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
    let region = decided(BezierRegion2::from_arrangement_traversal(
        &graph, &traversal,
    ));

    assert_eq!(region.len(), 1);
    assert_eq!(region.boundary_loops()[0].len(), 4);
    assert_eq!(region.signed_area().unwrap(), Some(q(-32, 3)));
}

#[test]
fn open_arrangement_chain_does_not_materialize_region() {
    let first = QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0));
    let second = QuadraticBezier2::new(p(2, 0), p(3, -1), p(4, 0));
    let first_split = decided(first.split_at_parameters(&[], &policy()).unwrap());
    let second_split = decided(second.split_at_parameters(&[], &policy()).unwrap());
    let graph = BezierArrangementGraph2::from_split_materializations(&[first_split, second_split]);
    let traversal = decided(graph.traverse_branch_free(&policy()));

    assert_eq!(
        BezierRegion2::from_arrangement_traversal(&graph, &traversal),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn conic_region_boundary_materializes_but_area_is_explicitly_unsupported() {
    let upper =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 0), p(2, 2), p(4, 0), q(1, 2)).unwrap();
    let lower = RationalQuadraticBezier2::try_unit_end_weights(p(4, 0), p(2, -2), p(0, 0), q(1, 2))
        .unwrap();
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
    let region = decided(BezierRegion2::from_arrangement_traversal(
        &graph, &traversal,
    ));

    assert_eq!(region.len(), 1);
    assert_eq!(region.boundary_loops()[0].len(), 4);
    assert_eq!(region.signed_area().unwrap(), None);
}

#[test]
fn retained_region_materializes_closed_algebraic_carrier_loop_without_area_sampling() {
    let parameter = BezierParameter2::algebraic(algebraic_midpoint_parameter());
    let p0_right = algebraic_image(&line_midpoint_curve(-1, 0, 1));
    let p1_right = algebraic_image(&line_midpoint_curve(0, 1, 2));
    let p1_left = algebraic_image(&line_midpoint_curve(2, 1, 0));
    let p0_left = algebraic_image(&line_midpoint_curve(1, 0, -1));
    let first = BezierSplitFragment2::AlgebraicEndpointImages {
        start: parameter.clone(),
        end: parameter.clone(),
        start_image: Some(p0_right),
        end_image: Some(p1_right),
    };
    let second = BezierSplitFragment2::AlgebraicEndpointImages {
        start: parameter.clone(),
        end: parameter,
        start_image: Some(p1_left),
        end_image: Some(p0_left),
    };
    let graph = BezierArrangementGraph2::new(vec![
        BezierArrangementFragment2::new(0, 0, first),
        BezierArrangementFragment2::new(1, 0, second),
    ]);
    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));

    assert_eq!(
        BezierRegion2::from_arrangement_traversal(&graph, &traversal),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    let retained = decided(BezierRetainedRegion2::from_retained_arrangement_traversal(
        &graph, &traversal,
    ));

    assert_eq!(retained.len(), 1);
    assert_eq!(retained.boundary_loops()[0].len(), 2);
    assert!(retained.has_algebraic_fragments());
    assert_eq!(retained.signed_area().unwrap(), None);
}

#[test]
fn retained_region_rejects_unresolved_carriers_even_when_marked_closed() {
    let parameter = BezierParameter2::algebraic(algebraic_midpoint_parameter());
    let graph = BezierArrangementGraph2::new(vec![BezierArrangementFragment2::new(
        0,
        0,
        BezierSplitFragment2::Unresolved {
            start: parameter.clone(),
            end: parameter,
        },
    )]);
    let traversal = hypercurve::BezierArrangementTraversal2::new(vec![
        hypercurve::BezierArrangementChain2::new(vec![0], true),
    ]);

    assert_eq!(
        BezierRetainedRegion2::from_retained_arrangement_traversal(&graph, &traversal),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

proptest! {
    #[test]
    fn symmetric_quadratic_lens_area_scales_exactly(
        height in 1_i32..=12,
    ) {
        let upper = QuadraticBezier2::new(p(0, 0), p(2, height), p(4, 0));
        let lower = QuadraticBezier2::new(p(4, 0), p(2, -height), p(0, 0));
        let graph = BezierArrangementGraph2::from_split_materializations(&[
            decided(upper.split_at_parameters(&[exact(q(1, 2))], &policy()).unwrap()),
            decided(lower.split_at_parameters(&[exact(q(1, 2))], &policy()).unwrap()),
        ]);
        let traversal = decided(graph.traverse_branch_free(&policy()));
        let region = decided(BezierRegion2::from_arrangement_traversal(&graph, &traversal));

        prop_assert_eq!(region.signed_area().unwrap(), Some(q(-8 * height, 3)));
    }
}
