use hypertri::{Point2, PredicatePolicy, Real, TriangulationContext};

fn main() -> hypertri::Result<()> {
    let points = vec![
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(1), Real::from(0)),
        Point2::new(Real::from(0), Real::from(1)),
    ];

    let context = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);
    let triangles = hypertri::earcut(&context, &points, &[])?.value;
    assert_eq!(triangles.len(), 3);
    Ok(())
}
