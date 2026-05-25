use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierAlgebraicParameter2, BezierArrangementGraph2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierRetainedOverlapReport2, Classification, CurvePolicy,
    CurveResult, Point2, QuadraticBezier2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("benchmark unexpectedly uncertain: {reason:?}"),
    }
}

fn main() -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    let split = [decided(BezierParameter2::exact(q(1, 2), &policy)?)];
    let mut materializations = Vec::new();
    for index in 0..64 {
        let curve = QuadraticBezier2::new(
            p(index * 2, 0),
            p(index * 2 + 1, if index % 2 == 0 { 2 } else { -2 }),
            p(index * 2 + 2, 0),
        );
        materializations.push(decided(curve.split_at_parameters(&split, &policy)?));
    }
    let graph = BezierArrangementGraph2::from_split_materializations(&materializations);

    let iterations = 5_000_u32;
    let started = Instant::now();
    let mut total = 0_usize;
    for _ in 0..iterations {
        let traversal = decided(graph.traverse_with_tangent_order(&policy));
        total += black_box(traversal.len());
        total += black_box(
            match BezierRetainedOverlapReport2::from_graph(&graph, &policy) {
                Classification::Decided(report) => {
                    let split_count = match report.line_overlap_splits(&policy) {
                        Classification::Decided(splits) => splits.len(),
                        Classification::Uncertain(_) => 0,
                    };
                    let bezier_split_count =
                        match report.linear_bezier_overlap_splits(&graph, &policy) {
                            Classification::Decided(splits) => splits.len(),
                            Classification::Uncertain(_) => 0,
                        };
                    report.len() + split_count + bezier_split_count
                }
                Classification::Uncertain(_) => 0,
            },
        );
        total += black_box(
            match graph.traverse_retained_deduplicating_materialized_overlaps(&policy) {
                Classification::Decided(report) => report.shadowed_fragment_indices().len(),
                Classification::Uncertain(_) => 0,
            },
        );
        total += black_box(match graph.split_retained_linear_overlaps(&policy) {
            Classification::Decided(refinement) => {
                refinement.graph().len() + refinement.refined_fragments().len()
            }
            Classification::Uncertain(_) => 0,
        });
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_arrangement_tangent_order: {iterations} iterations in {elapsed:?} ({:?}/iter), total={total}",
        elapsed / iterations
    );

    let algebraic_parameter =
        BezierParameter2::algebraic(decided(BezierAlgebraicParameter2::try_isolate(
            decided(BezierParameterPolynomial::try_new_power_basis(
                vec![r(-1), r(2)],
                &policy,
            )?),
            decided(BezierParameterInterval::try_new(q(2, 5), q(3, 5), &policy)?),
            &policy,
        )?));
    let algebraic_curve = QuadraticBezier2::new(p(-1, 0), p(0, 0), p(1, 0));
    let algebraic_split =
        decided(algebraic_curve.split_at_parameters(&[algebraic_parameter], &policy)?);
    let retained_graph = BezierArrangementGraph2::from_split_materializations(&[algebraic_split]);
    let started = Instant::now();
    let mut retained_total = 0_usize;
    for _ in 0..iterations {
        let traversal = decided(retained_graph.traverse_retained_with_tangent_order(&policy));
        retained_total += black_box(traversal.len());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_arrangement_retained_tangent_order: {iterations} iterations in {elapsed:?} ({:?}/iter), total={retained_total}",
        elapsed / iterations
    );

    Ok(())
}
