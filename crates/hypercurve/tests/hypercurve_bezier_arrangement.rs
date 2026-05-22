use hypercurve::{
    BezierAlgebraicParameter2, BezierArrangementGraph2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierSplitFragment2, BezierSubcurve2, Classification, CubicBezier2,
    CurvePolicy, Point2, QuadraticBezier2, Real, UncertaintyReason,
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

fn algebraic_midpoint() -> BezierParameter2 {
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis(vec![r(-1), r(2)], &policy()).unwrap(),
    );
    let interval = decided(BezierParameterInterval::try_new(q(2, 5), q(3, 5), &policy()).unwrap());
    BezierParameter2::algebraic(decided(
        BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap(),
    ))
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
fn algebraic_split_boundary_blocks_graph_traversal() {
    let curve = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let split = decided(
        curve
            .split_at_parameters(&[algebraic_midpoint(), exact(q(3, 4))], &policy())
            .unwrap(),
    );
    let graph = BezierArrangementGraph2::from_split_materializations(&[split]);

    assert_eq!(
        graph.traverse_branch_free(&policy()),
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
