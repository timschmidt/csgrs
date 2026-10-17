//! Exact linear strokes delegated to the native curve kernel.

use super::{Coord, LineString, Polygon, exact_real};
use crate::{PcbGeometryUncertainty, Scalar};
use hypercurve::{CurvePath2, CurveRegion2, LineSeg2, OffsetCap, OffsetCornerStyle2, Point2};

/// Construct the rectangular, butt-capped stroke between finite endpoints.
///
/// Finite inputs are lifted before subtraction, normalization or halving.
/// Non-finite inputs, nonpositive widths and identical endpoints retain the
/// adapter's `None` contract. For otherwise admitted inputs, native construction
/// failures remain attached to the polygon and observable through its checked
/// queries. The historical finite corner calculation supplies only a report
/// ring; collapse or overflow in that view cannot replace the native stroke.
pub fn line_polygon(start: [f64; 2], end: [f64; 2], width: f64) -> Option<Polygon<f64>> {
    if !start
        .into_iter()
        .chain(end)
        .chain([width])
        .all(f64::is_finite)
        || width <= 0.0
        || start == end
    {
        return None;
    }
    let native = linear_stroke_region(
        &start.map(exact_real),
        &end.map(exact_real),
        &exact_real(width),
        OffsetCap::Butt,
    );
    let view = report_ring(start, end, width);
    Some(match native {
        Ok(region) => Polygon::from_exact_region(region, view, vec![]),
        Err(error) => Polygon::from_exact_region(CurveRegion2::empty(), view, vec![])
            .with_exact_construction_error(error.to_string()),
    })
}

pub(crate) fn linear_stroke_region(
    start: &[Scalar; 2],
    end: &[Scalar; 2],
    width: &Scalar,
    cap: OffsetCap,
) -> Result<CurveRegion2, PcbGeometryUncertainty> {
    crate::operations::evaluate("linear-stroke-construction", None, || {
        let failure =
            |detail| crate::operations::failure("linear-stroke-construction", None, detail);
        match crate::scalar::try_compare(width, &Scalar::zero())? {
            std::cmp::Ordering::Less => return Err(failure("negative stroke width".to_owned())),
            std::cmp::Ordering::Equal => return Ok(CurveRegion2::empty()),
            std::cmp::Ordering::Greater => {}
        }
        let half_width = crate::scalar::half(width);
        if crate::scalar::try_compare(&start[0], &end[0])?.is_eq()
            && crate::scalar::try_compare(&start[1], &end[1])?.is_eq()
        {
            return match cap {
                OffsetCap::Round => super::circle_region(start, &half_width),
                OffsetCap::Butt => Ok(CurveRegion2::empty()),
                OffsetCap::Square => Err(failure("a square cap requires a direction".to_owned())),
            };
        }
        let line = LineSeg2::try_new(
            Point2::new(start[0].clone(), start[1].clone()),
            Point2::new(end[0].clone(), end[1].clone()),
        )
        .map_err(|error| failure(error.to_string()))?;
        let path =
            CurvePath2::try_new(vec![line.into()]).map_err(|error| failure(error.to_string()))?;
        CurveRegion2::stroke_path(&path, half_width, &OffsetCornerStyle2::Round, cap)
            .map_err(|error| failure(error.to_string()))
    })
}

fn report_ring(start: [f64; 2], end: [f64; 2], width: f64) -> LineString<f64> {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    let length = (dx * dx + dy * dy).sqrt();
    let nx = -dy / length * width / 2.0;
    let ny = dx / length * width / 2.0;
    let points = [
        [start[0] + nx, start[1] + ny],
        [end[0] + nx, end[1] + ny],
        [end[0] - nx, end[1] - ny],
        [start[0] - nx, start[1] - ny],
        [start[0] + nx, start[1] + ny],
    ];
    if !points.iter().flatten().all(|value| value.is_finite()) {
        return LineString(vec![]);
    }
    LineString(points.into_iter().map(|[x, y]| Coord { x, y }).collect())
}
