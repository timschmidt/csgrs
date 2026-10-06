#![cfg(feature = "dispatch-trace")]

use hypergraphics::{
    Color3, ExactMesh, ExactVertex, Point3, Primitive, Rational, Real, TriangleOrientation,
};

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn point(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

#[test]
fn near_coplanar_exact_orientation_does_not_request_approximation() {
    let n = 1_000_000_i64;
    let color = Color3::new(0.2, 0.4, 0.8).expect("finite color");
    let mesh = ExactMesh::new(
        Primitive::Triangles,
        vec![
            ExactVertex::new(point(Real::zero(), Real::zero(), Real::zero()), color),
            ExactVertex::new(point(Real::from(n), Real::zero(), Real::one()), color),
            ExactVertex::new(point(Real::zero(), Real::from(n), Real::one()), color),
        ],
    );
    let denominator = n as u64;
    let query = point(
        Real::one(),
        Real::one(),
        q(2, denominator) + q(1, denominator * denominator),
    );

    hyperreal::dispatch_trace::reset();
    let _recording = hyperreal::dispatch_trace::recording_scope();
    assert!(matches!(
        mesh.triangle_orientation_against(0, &query, hyperlimit::PredicatePolicy::STRICT)
            .expect("triangle query"),
        TriangleOrientation::Positive(_) | TriangleOrientation::Negative(_)
    ));

    let correlation = hyperreal::dispatch_trace::snapshot_trace().correlation_summary();
    assert!(correlation.dispatch_events > 0);
    assert!(correlation.predicate_events > 0);
    assert_eq!(correlation.approximation_events, 0);
    assert_eq!(correlation.unknown_fact_events, 0);
}
