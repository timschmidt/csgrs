use hypertri::{PointD, PredicatePolicy, Real, TriangulationContext, nd};

fn main() -> hypertri::Result<()> {
    let points = vec![
        PointD::new(vec![Real::from(0), Real::from(0)]),
        PointD::new(vec![Real::from(1), Real::from(0)]),
        PointD::new(vec![Real::from(0), Real::from(1)]),
    ];

    let context = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);
    let complex = nd::delaunay_complex(&context, &points)?.value;
    complex.validate(&context)?;
    assert_eq!(complex.cells().len(), 1);
    Ok(())
}
