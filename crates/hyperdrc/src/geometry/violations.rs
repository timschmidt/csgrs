//! Conversion from raw geometry into reportable violation shapes.

use geo::{Area, Coord, LineString, MultiPolygon};

use crate::Scalar;
use crate::report::ViolationPolygon;

/// Run the `multipolygon_to_shapes` design-readiness check or report helper.
#[cfg(test)]
pub fn multipolygon_to_shapes(
    multipolygon: &MultiPolygon<f64>,
    min_area: f64,
) -> Vec<ViolationPolygon> {
    multipolygon
        .0
        .iter()
        .filter_map(|polygon| {
            let area = polygon.unsigned_area();
            (area > min_area).then(|| ViolationPolygon {
                area,
                exterior: ring_to_coordinates(polygon.exterior()),
                holes: polygon
                    .interiors()
                    .iter()
                    .map(ring_to_coordinates)
                    .collect(),
            })
        })
        .collect()
}

/// Convert projected polygons to report shapes while keeping the area release
/// predicate in the exact scalar domain.
///
/// Coordinates enter from the finite report projection and are lifted exactly
/// as dyadics for the shoelace sum. The stored `ViolationPolygon::area` remains
/// `f64` because it is report/output data.
pub fn multipolygon_to_shapes_scalar(
    multipolygon: &MultiPolygon<f64>,
    min_area: &Scalar,
) -> Vec<ViolationPolygon> {
    multipolygon
        .0
        .iter()
        .filter_map(|polygon| {
            let area = polygon_area_scalar(polygon)?;
            (&area > min_area).then(|| ViolationPolygon {
                area: polygon.unsigned_area(),
                exterior: ring_to_coordinates(polygon.exterior()),
                holes: polygon
                    .interiors()
                    .iter()
                    .map(ring_to_coordinates)
                    .collect(),
            })
        })
        .collect()
}

pub(crate) fn polygon_area_scalar(polygon: &geo::Polygon<f64>) -> Option<Scalar> {
    let exterior = ring_area_scalar(polygon.exterior())?;
    let holes = Scalar::sum_owned(
        polygon
            .interiors()
            .iter()
            .map(ring_area_scalar)
            .collect::<Option<Vec<_>>>()?,
    );
    Some((exterior - holes).abs())
}

pub(crate) fn multipolygon_area_scalar(multipolygon: &MultiPolygon<f64>) -> Option<Scalar> {
    Some(Scalar::sum_owned(
        multipolygon
            .0
            .iter()
            .map(polygon_area_scalar)
            .collect::<Option<Vec<_>>>()?,
    ))
}

pub(crate) fn polygon_bounds_scalar(polygon: &geo::Polygon<f64>) -> Option<[Scalar; 4]> {
    let mut coordinates = polygon
        .exterior()
        .0
        .iter()
        .chain(polygon.interiors().iter().flat_map(|ring| ring.0.iter()));
    let first = coordinates.next()?;
    let first_x = Scalar::try_from(first.x).ok()?;
    let first_y = Scalar::try_from(first.y).ok()?;
    let mut bounds = [first_x.clone(), first_y.clone(), first_x, first_y];
    for coordinate in coordinates {
        let x = Scalar::try_from(coordinate.x).ok()?;
        let y = Scalar::try_from(coordinate.y).ok()?;
        if x < bounds[0] {
            bounds[0] = x.clone();
        }
        if y < bounds[1] {
            bounds[1] = y.clone();
        }
        if x > bounds[2] {
            bounds[2] = x;
        }
        if y > bounds[3] {
            bounds[3] = y;
        }
    }
    Some(bounds)
}

fn ring_area_scalar(ring: &LineString<f64>) -> Option<Scalar> {
    let doubled = Scalar::sum_owned(
        ring.0
            .windows(2)
            .map(|edge| {
                let x0 = Scalar::try_from(edge[0].x).ok()?;
                let y0 = Scalar::try_from(edge[0].y).ok()?;
                let x1 = Scalar::try_from(edge[1].x).ok()?;
                let y1 = Scalar::try_from(edge[1].y).ok()?;
                Some(x0 * y1 - x1 * y0)
            })
            .collect::<Option<Vec<_>>>()?,
    );
    (doubled / crate::scalar::scalar("2"))
        .ok()
        .map(|area| area.abs())
}

fn ring_to_coordinates(ring: &LineString<f64>) -> Vec<[f64; 2]> {
    ring.0.iter().map(|Coord { x, y }| [*x, *y]).collect()
}
