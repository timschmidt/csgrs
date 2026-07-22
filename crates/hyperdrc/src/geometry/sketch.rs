//! Profile conversion helpers.
//!
//! Keep these wrappers small and explicit: most checks operate on `Profile`
//! topology, while parsers naturally produce `geo` polygons.

use csgrs::sketch::Profile;
use geo::{Area, LineString, Polygon};
use hypercurve::{Contour2, CurvePolicy, CurveRegion2};

use crate::{LayerMetadata, PcbSketch};

/// Convert one `geo` polygon into a `csgrs::Profile` with layer metadata.
pub fn polygon_to_profile(polygon: Polygon<f64>, metadata: Option<LayerMetadata>) -> PcbSketch {
    polygons_to_profile(vec![polygon], metadata)
}

/// Convert `geo` polygons into a `csgrs::Profile` with layer metadata.
pub fn polygons_to_profile(
    polygons: Vec<Polygon<f64>>,
    metadata: Option<LayerMetadata>,
) -> PcbSketch {
    let (exact_bounds, had_non_finite_input) = exact_input_polygon_bounds(&polygons);
    let mut material = Vec::new();
    let mut holes = Vec::new();
    for polygon in polygons {
        if let Some(contour) = linestring_to_contour(polygon.exterior(), RingRole::Material) {
            material.push(contour);
        }
        holes.extend(
            polygon
                .interiors()
                .iter()
                .filter_map(|ring| linestring_to_contour(ring, RingRole::Hole)),
        );
    }
    if material.is_empty() && holes.is_empty() {
        return PcbSketch::new_with_exact_bounds(
            Profile::empty(),
            metadata,
            exact_bounds,
            had_non_finite_input,
        );
    }
    let region = CurveRegion2::try_from_native_contours(material, holes, &CurvePolicy::certified())
        .unwrap_or_else(|_| CurveRegion2::empty());
    PcbSketch::new_with_exact_bounds(
        Profile::from_curve_region(region),
        metadata,
        exact_bounds,
        had_non_finite_input,
    )
}

fn exact_input_polygon_bounds(polygons: &[Polygon<f64>]) -> (Option<[hyperreal::Real; 4]>, bool) {
    let mut bounds: Option<[f64; 4]> = None;
    for polygon in polygons {
        for coordinate in polygon
            .exterior()
            .0
            .iter()
            .chain(polygon.interiors().iter().flat_map(|ring| ring.0.iter()))
        {
            if !coordinate.x.is_finite() || !coordinate.y.is_finite() {
                return (None, true);
            }
            match &mut bounds {
                Some([min_x, min_y, max_x, max_y]) => {
                    *min_x = min_x.min(coordinate.x);
                    *min_y = min_y.min(coordinate.y);
                    *max_x = max_x.max(coordinate.x);
                    *max_y = max_y.max(coordinate.y);
                }
                None => {
                    bounds = Some([coordinate.x, coordinate.y, coordinate.x, coordinate.y]);
                }
            }
        }
    }
    let Some([min_x, min_y, max_x, max_y]) = bounds else {
        return (None, false);
    };
    (
        Some([
            hyperreal::Real::try_from(min_x).expect("finite polygon bounds promote exactly"),
            hyperreal::Real::try_from(min_y).expect("finite polygon bounds promote exactly"),
            hyperreal::Real::try_from(max_x).expect("finite polygon bounds promote exactly"),
            hyperreal::Real::try_from(max_y).expect("finite polygon bounds promote exactly"),
        ]),
        false,
    )
}

/// Create an empty `csgrs::Profile` with layer metadata.
pub fn empty_profile(metadata: Option<LayerMetadata>) -> PcbSketch {
    PcbSketch::new(Profile::empty(), metadata)
}

#[derive(Clone, Copy)]
enum RingRole {
    Material,
    Hole,
}

fn linestring_to_contour(ring: &LineString<f64>, role: RingRole) -> Option<Contour2> {
    let mut points = ring
        .0
        .iter()
        .map(|coord| [coord.x, coord.y])
        .collect::<Vec<_>>();
    let signed_area = Polygon::new(ring.clone(), vec![]).signed_area();
    let should_reverse = match role {
        RingRole::Material => signed_area < 0.0,
        RingRole::Hole => signed_area > 0.0,
    };
    if should_reverse {
        points.reverse();
    }
    Contour2::from_finite_ring(&points).ok()
}
