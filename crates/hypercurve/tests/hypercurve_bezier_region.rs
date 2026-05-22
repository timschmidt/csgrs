use hypercurve::{
    BezierArrangementGraph2, BezierParameter2, BezierRegion2, Classification, CurvePolicy, Point2,
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

fn exact(value: Real) -> BezierParameter2 {
    decided(BezierParameter2::exact(value, &policy()).unwrap())
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
