//! Conversion from raw geometry into reportable violation shapes.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};

use super::{Coord, LineString, MultiPolygon, Polygon};
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
            (compare_reals_with_policy(&area, min_area, PredicatePolicy).value()
                == Some(Ordering::Greater))
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
    polygon.exact_area().cloned().or_else(|| {
        let exterior = ring_area_scalar(polygon.exterior())?;
        let holes = balanced_scalar_sum(
            polygon
                .interiors()
                .iter()
                .map(ring_area_scalar)
                .collect::<Option<Vec<_>>>()?,
        );
        Some((exterior - holes).abs())
    })
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

fn ring_area_scalar(ring: &LineString<f64>) -> Option<Scalar> {
    let mut points = ring.0.iter();
    let first = points.next()?;
    let mut previous = first;
    let mut doubled = Scalar::zero();
    for point in points {
        let x0 = Scalar::try_from(previous.x).ok()?;
        let y0 = Scalar::try_from(previous.y).ok()?;
        let x1 = Scalar::try_from(point.x).ok()?;
        let y1 = Scalar::try_from(point.y).ok()?;
        doubled += x0 * y1 - x1 * y0;
        previous = point;
    }
    if previous != first {
        doubled += Scalar::try_from(previous.x).ok()? * Scalar::try_from(first.y).ok()?
            - Scalar::try_from(first.x).ok()? * Scalar::try_from(previous.y).ok()?;
    }
    Some(crate::scalar::half(&doubled).abs())
}
