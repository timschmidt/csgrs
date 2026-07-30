use hypertri::{Constraint, Point2, PredicatePolicy, Real, TriangulationContext, cdt};

fn main() -> hypertri::Result<()> {
    let points = vec![
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(2), Real::from(0)),
        Point2::new(Real::from(2), Real::from(2)),
        Point2::new(Real::from(0), Real::from(2)),
    ];

    let context = TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);
    let delaunay = cdt::delaunay(&context, &points)?.value;
    let constrained = cdt::constrained_delaunay(
        &context,
        &points,
        &[Constraint::new(0, 1), Constraint::new(1, 2)],
    )?
    .value;
    delaunay.validate(&context)?;
    constrained.validate(&context)?;
    Ok(())
}
