use hyperbrep::{Point3, Real, Vector3, builder};
use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let point = |x, y| Point2::new(Real::from(x), Real::from(y));
    let line = |x0, y0, x1, y1| LineSeg2::try_new(point(x0, y0), point(x1, y1)).map(Curve2::from);
    let boundary = CurvePath2::try_new(vec![
        line(0, 0, 4, 0)?,
        line(4, 0, 4, 3)?,
        line(4, 3, 0, 3)?,
        line(0, 3, 0, 0)?,
    ])?;
    let (model, face) =
        builder::planar_face(&boundary, &[], Point3::origin(), Vector3::x(), Vector3::y())?;
    let area = model.face_area(face)?;
    println!("edges: {}, exact area: {area}", model.counts().edges);
    assert_eq!(area, Real::from(12));
    Ok(())
}
