//! Conversion from raw geometry into reportable violation shapes.

use csgrs::float_types::Real;
use geo::{Area, Coord, LineString, MultiPolygon};

use crate::report::ViolationPolygon;

pub fn multipolygon_to_shapes(
    multipolygon: &MultiPolygon<Real>,
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

fn ring_to_coordinates(ring: &LineString<Real>) -> Vec<[f64; 2]> {
    ring.0.iter().map(|Coord { x, y }| [*x, *y]).collect()
}
