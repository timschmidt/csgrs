//! Conversion from raw geometry into reportable violation shapes.

use std::cmp::Ordering;

use hyperlimit::compare_reals;

use super::{Coord, LineString, MultiPolygon, Polygon};
use crate::PREDICATE_POLICY;
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
/// The release predicate uses the authoritative native region area. The stored
/// `ViolationPolygon::area` remains `f64` because it is report/output data.
pub fn multipolygon_to_shapes_scalar(
    multipolygon: &MultiPolygon<f64>,
    min_area: &Scalar,
) -> Vec<ViolationPolygon> {
    multipolygon
        .0
        .iter()
        .filter_map(|polygon| {
            let area = polygon_area_scalar(polygon)?;
            (compare_reals(&area, min_area, PREDICATE_POLICY).value() == Some(Ordering::Greater))
                .then(|| ViolationPolygon {
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

pub(crate) fn polygon_area_scalar(polygon: &Polygon<f64>) -> Option<Scalar> {
    polygon.exact_area().cloned()
}

pub(crate) fn multipolygon_area_scalar(multipolygon: &MultiPolygon<f64>) -> Option<Scalar> {
    Some(balanced_scalar_sum(
        multipolygon
            .0
            .iter()
            .map(polygon_area_scalar)
            .collect::<Option<Vec<_>>>()?,
    ))
}

pub(crate) fn balanced_scalar_sum(values: impl IntoIterator<Item = Scalar>) -> Scalar {
    let mut values = values.into_iter().collect::<Vec<_>>();
    if values.is_empty() {
        return Scalar::zero();
    }
    while values.len() > 1 {
        let mut next = Vec::with_capacity(values.len().div_ceil(2));
        let mut pairs = values.into_iter();
        while let Some(left) = pairs.next() {
            next.push(match pairs.next() {
                Some(right) => left + right,
                None => left,
            });
        }
        values = next;
    }
    values
        .pop()
        .expect("nonempty exact sum reduction must retain one value")
}

pub(crate) fn polygon_bounds_scalar(polygon: &Polygon<f64>) -> Option<[Scalar; 4]> {
    polygon.exact_bounds().cloned()
}

fn ring_to_coordinates(ring: &LineString<f64>) -> Vec<[f64; 2]> {
    ring.0.iter().map(|Coord { x, y }| [*x, *y]).collect()
}
