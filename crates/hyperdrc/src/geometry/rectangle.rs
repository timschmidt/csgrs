//! Rectangles whose native corners precede every report projection.

use super::{Coord, LineString, Polygon, exact_real};
use crate::Scalar;
use hypercurve::{Contour2, CurveRegion2, LineSeg2, Point2, Segment2};

/// Construct a native rectangle from finite center, size and degree inputs.
///
/// Each input is imported exactly as its binary64 dyadic before halving sizes,
/// rotation or translation. Rotation uses native sine/cosine of the exact
/// degree value. Signed dimensions denote the same regularized rectangle as
/// their absolute values; a zero dimension denotes the empty regularized set.
/// Non-finite inputs, uncertified construction and exhausted shared budgets
/// remain attached failures, observable through the checked polygon methods.
///
/// The historical finite corner calculation is retained only as a report view,
/// including signed winding. That view can collapse at large coordinates and
/// is empty if a corner cannot be represented finitely; it never replaces the
/// native rectangle. Retry denied construction from its inputs in a fresh scope.
pub fn rect_polygon(center: [f64; 2], size: [f64; 2], angle_degrees: f64) -> Polygon<f64> {
    if !center
        .into_iter()
        .chain(size)
        .chain([angle_degrees])
        .all(f64::is_finite)
    {
        return Polygon::from_exact_region(CurveRegion2::empty(), LineString(vec![]), vec![])
            .with_exact_construction_error("rectangle contains a non-finite parameter");
    }
    let native = rectangle_region(
        &center.map(exact_real),
        &size.map(exact_real),
        &exact_real(angle_degrees),
    );
    let view = report_ring(center, size, angle_degrees);
    match native {
        Ok(region) => Polygon::from_exact_region(region, view, vec![]),
        Err(error) => Polygon::from_exact_region(CurveRegion2::empty(), view, vec![])
            .with_exact_construction_error(error.to_string()),
    }
}

pub(crate) fn rectangle_region(
    center: &[Scalar; 2],
    size: &[Scalar; 2],
    angle_degrees: &Scalar,
) -> Result<CurveRegion2, crate::PcbGeometryUncertainty> {
    crate::operations::evaluate("rectangle-construction", None, || {
        if size.contains(&Scalar::zero()) {
            return Ok(CurveRegion2::empty());
        }
        let [x, y] = size.clone().map(|side| crate::scalar::half(&side.abs()));
        let points = [
            Point2::new(-x.clone(), -y.clone()),
            Point2::new(x.clone(), -y.clone()),
            Point2::new(x.clone(), y.clone()),
            Point2::new(-x, y),
        ];
        let failure = |error| crate::operations::failure("rectangle-construction", None, error);
        let edges = points
            .iter()
            .cloned()
            .zip(points.iter().cloned().cycle().skip(1))
            .map(|(a, b)| LineSeg2::try_new(a, b).map(Segment2::Line).map_err(failure))
            .collect::<Result<_, _>>()?;
        let contour = Contour2::try_new(edges).map_err(failure)?;
        let local = CurveRegion2::try_from_native_material_contours(vec![contour])
            .map_err(|error| crate::operations::failure("rectangle-construction", None, error))?;
        let angle = (angle_degrees * Scalar::pi() / Scalar::from(180_u16))
            .expect("180 is a nonzero exact degree denominator");
        let cosine = angle.clone().cos();
        let sine = angle.sin();
        local
            .transform_affine(
                &cosine,
                &(-sine.clone()),
                &sine,
                &cosine,
                &center[0],
                &center[1],
            )
            .map_err(|error| crate::operations::failure("rectangle-construction", None, error))
    })
}

fn report_ring(center: [f64; 2], size: [f64; 2], angle_degrees: f64) -> LineString<f64> {
    let [x, y] = size.map(|side| side / 2.0);
    let theta = angle_degrees.to_radians();
    let cosine = theta.cos();
    let sine = theta.sin();
    let mut points = [[-x, -y], [x, -y], [x, y], [-x, y]]
        .map(|[x, y]| Coord {
            x: center[0] + x * cosine - y * sine,
            y: center[1] + x * sine + y * cosine,
        })
        .to_vec();
    if points
        .iter()
        .any(|point| !point.x.is_finite() || !point.y.is_finite())
    {
        return LineString(vec![]);
    }
    if points.first() != points.last() {
        points.push(points[0]);
    }
    LineString(points)
}
