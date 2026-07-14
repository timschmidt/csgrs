use hypertri::{PointD, Real, nd};

fn main() -> hypertri::Result<()> {
    let points = vec![
        PointD::new(vec![Real::from(0), Real::from(0)]),
        PointD::new(vec![Real::from(1), Real::from(0)]),
        PointD::new(vec![Real::from(0), Real::from(1)]),
    ];

    let complex = nd::delaunay_complex(&points)?;
    complex.validate()?;
    assert_eq!(complex.cells().len(), 1);
    Ok(())
}
