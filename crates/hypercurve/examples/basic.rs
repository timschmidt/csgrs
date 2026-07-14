use hypercurve::{
    BezierDegree, Classification, Contour2, CurvePolicy, LineSeg2, Point2, QuadraticBezier2,
    Region2, Segment2,
};
use hyperreal::Real;

fn main() -> hypercurve::CurveResult<()> {
    let p = |x, y| Point2::new(Real::from(x), Real::from(y));
    let bezier = QuadraticBezier2::new(p(0, 0), p(1, 2), p(2, 0));
    assert_eq!(bezier.structural_facts().degree, BezierDegree::Quadratic);

    let boundary = [
        ((0, 0), (2, 0)),
        ((2, 0), (2, 2)),
        ((2, 2), (0, 2)),
        ((0, 2), (0, 0)),
    ]
    .into_iter()
    .map(|(start, end)| LineSeg2::try_new(p(start.0, start.1), p(end.0, end.1)).map(Segment2::Line))
    .collect::<hypercurve::CurveResult<Vec<_>>>()?;

    let region = Region2::from_material_contours(vec![Contour2::try_new(boundary)?]);
    assert!(matches!(
        region.classify_point(&p(1, 1), &CurvePolicy::certified()),
        Classification::Decided(_)
    ));
    Ok(())
}
