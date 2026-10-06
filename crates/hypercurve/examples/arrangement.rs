use hypercurve::{CurveRegion2, FillRule, LineSeg2, Point2, RegionPointLocation, Segment2};
use hyperreal::Real;

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn line(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> hypercurve::CurveResult<LineSeg2> {
    LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let boundary = vec![
        line(0, 0, 4, 0)?,
        line(4, 0, 4, 4)?,
        line(4, 4, 0, 4)?,
        line(0, 4, 0, 0)?,
    ];

    let region = CurveRegion2::arrange_unordered_segments(
        &boundary.into_iter().map(Segment2::Line).collect::<Vec<_>>(),
        FillRule::NonZero,
    )?;
    assert_eq!(
        region.classify_point(&p(2, 2).into())?,
        RegionPointLocation::Inside
    );

    Ok(())
}
