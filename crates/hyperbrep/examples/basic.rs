use hyperbrep::vertical_prism_shell;
use hypercurve::{Contour2, CurvePolicy, CurveRegion2, LineSeg2, Point2, Segment2};
use hyperreal::Real;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let p = |x, y| Point2::new(Real::from(x), Real::from(y));
    let boundary = [(0, 0), (2, 0), (2, 3), (0, 3), (0, 0)]
        .windows(2)
        .map(|pair| {
            LineSeg2::try_new(p(pair[0].0, pair[0].1), p(pair[1].0, pair[1].1)).map(Segment2::Line)
        })
        .collect::<hypercurve::CurveResult<Vec<_>>>()?;
    let contour = Contour2::try_new(boundary)?;
    let region =
        CurveRegion2::try_from_native_material_contours(vec![contour], &CurvePolicy::certified())?;

    let shell = vertical_prism_shell(&region, Real::from(0), Real::from(4))?;
    let solid = shell.solid_readiness_report();
    assert!(solid.exact_solid_boundary_ready);
    assert!(solid.exact_volume_ready);
    Ok(())
}
