//! Conversion from raw geometry into reportable violation shapes.

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
/// The release predicate uses the authoritative native region area. The stored
/// `ViolationPolygon::area` remains `f64` because it is report/output data.
/// Missing integrals, unknown thresholds, exhausted budgets and non-finite
/// report output return an error; no partial list is published.
pub fn multipolygon_to_shapes_scalar(
    multipolygon: &MultiPolygon<f64>,
    min_area: &Scalar,
) -> Result<Vec<ViolationPolygon>, crate::PcbGeometryUncertainty> {
    crate::operations::evaluate("report-area-filter", None, || {
        let mut shapes = Vec::new();
        for polygon in &multipolygon.0 {
            let area = polygon_area_scalar(polygon)?;
            if !crate::scalar::try_compare(&area, min_area)?.is_gt() {
                continue;
            }
            let area = polygon.unsigned_area();
            let exterior = ring_to_coordinates(polygon.exterior());
            let holes = polygon
                .interiors()
                .iter()
                .map(ring_to_coordinates)
                .collect::<Vec<_>>();
            if !area.is_finite()
                || exterior
                    .iter()
                    .chain(holes.iter().flatten())
                    .flatten()
                    .any(|value| !value.is_finite())
            {
                return Err(crate::operations::failure(
                    "report-area-filter",
                    None,
                    "report area or coordinates are not finite",
                ));
            }
            shapes.push(ViolationPolygon {
                area,
                exterior,
                holes,
            });
        }
        Ok(shapes)
    })
}

pub(crate) fn polygon_area_scalar(
    polygon: &Polygon<f64>,
) -> Result<Scalar, crate::PcbGeometryUncertainty> {
    polygon.try_area()
}

/// Filled-set area, including overlap regularization. Authored report profiles
/// may overlap or repeat and cannot be summed as independent material.
pub(crate) fn multipolygon_area_scalar(
    multipolygon: &MultiPolygon<f64>,
) -> Result<Scalar, crate::PcbGeometryUncertainty> {
    crate::operations::evaluate("profile-collection-area", None, || {
        match multipolygon.0.as_slice() {
            [] => Ok(Scalar::zero()),
            [polygon] => polygon.try_area(),
            _ => super::polygons_to_profile(multipolygon.0.clone(), None).try_area(),
        }
    })
}

pub(crate) fn polygon_bounds_scalar(polygon: &Polygon<f64>) -> Option<[Scalar; 4]> {
    polygon.exact_bounds().cloned()
}

fn ring_to_coordinates(ring: &LineString<f64>) -> Vec<[f64; 2]> {
    ring.0.iter().map(|Coord { x, y }| [*x, *y]).collect()
}
