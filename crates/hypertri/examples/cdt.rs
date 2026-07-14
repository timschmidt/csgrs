use hypertri::{Constraint, Point2, Real, cdt};

fn main() -> hypertri::Result<()> {
    let points = vec![
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(2), Real::from(0)),
        Point2::new(Real::from(2), Real::from(2)),
        Point2::new(Real::from(0), Real::from(2)),
    ];

    let delaunay = cdt::delaunay(&points)?;
    let constrained =
        cdt::constrained_delaunay(&points, &[Constraint::new(0, 1), Constraint::new(1, 2)])?;
    delaunay.validate()?;
    constrained.validate()?;
    Ok(())
}
