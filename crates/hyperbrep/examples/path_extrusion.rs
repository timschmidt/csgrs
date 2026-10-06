use hyperbrep::{Real, builder};
use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let point = |x, y| Point2::new(Real::from(x), Real::from(y));
    let line = |x0, y0, x1, y1| LineSeg2::try_new(point(x0, y0), point(x1, y1)).map(Curve2::from);
    let boundary = CurvePath2::try_new(vec![
        Curve2::try_nurbs(
            2,
            vec![point(0, 0), point(2, 0), point(4, 0)],
            vec![Real::one(), Real::from(2), Real::from(3)],
            vec![
                Real::from(2),
                Real::from(2),
                Real::from(2),
                Real::from(5),
                Real::from(5),
                Real::from(5),
            ],
        )?,
        line(4, 0, 4, 3)?,
        line(4, 3, 0, 3)?,
        line(0, 3, 0, 0)?,
    ])?;
    let (model, solid) = builder::extrude_path(&boundary, Real::zero(), Real::from(2))?;
    let volume = model.solid_volume(solid)?;
    println!("faces: {}, exact volume: {volume}", model.counts().faces);
    assert_eq!(volume, Real::from(24));
    Ok(())
}
