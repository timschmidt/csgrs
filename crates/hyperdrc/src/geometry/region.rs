//! Native filled-region conversion helpers.
//!
//! Checks retain exact Hypercurve topology while parsers use local finite
//! coordinate views only until promotion at this boundary.

use hypercurve::{Classification, CurvePolicy, CurveRegion2};

use super::{MultiPolygon, Polygon};
use crate::{LayerMetadata, PcbRegion};

/// Convert one exact-backed polygon into a native filled region with metadata.
pub fn polygon_to_profile(polygon: Polygon<f64>, metadata: Option<LayerMetadata>) -> PcbRegion {
    polygons_to_profile(vec![polygon], metadata)
}

/// Combine exact-backed polygons into a native filled region with metadata.
pub fn polygons_to_profile(
    polygons: Vec<Polygon<f64>>,
    metadata: Option<LayerMetadata>,
) -> PcbRegion {
    let (exact_bounds, had_non_finite_input) = exact_input_polygon_bounds(&polygons);
    let finite_projection = (!had_non_finite_input).then(|| MultiPolygon(polygons.clone()));
    if let Some(error) = polygons
        .iter()
        .find_map(|polygon| polygon.exact_construction_error())
    {
        return PcbRegion::new_with_exact_bounds_and_projection(
            CurveRegion2::empty(),
            metadata,
            exact_bounds,
            had_non_finite_input,
            finite_projection,
        )
        .with_exact_construction_error(error);
    }
    let policy = CurvePolicy::certified();
    let mut material = Vec::new();
    let mut holes = Vec::new();
    for polygon in &polygons {
        let native = match polygon.exact_region().native_contours_fast_path(&policy) {
            Ok(Classification::Decided(native)) => native,
            Ok(Classification::Uncertain(uncertainty)) => {
                return PcbRegion::new_with_exact_bounds_and_projection(
                    CurveRegion2::empty(),
                    metadata,
                    exact_bounds,
                    had_non_finite_input,
                    finite_projection,
                )
                .with_exact_construction_error(format!(
                    "exact native-contour extraction is unresolved: {uncertainty:?}"
                ));
            }
            Err(error) => {
                return PcbRegion::new_with_exact_bounds_and_projection(
                    CurveRegion2::empty(),
                    metadata,
                    exact_bounds,
                    had_non_finite_input,
                    finite_projection,
                )
                .with_exact_construction_error(format!(
                    "exact native-contour extraction failed: {error}"
                ));
            }
        };
        material.extend(native.material_contours().iter().cloned());
        holes.extend(native.hole_contours().iter().cloned());
    }
    if material.is_empty() && holes.is_empty() {
        return PcbRegion::new_with_exact_bounds_and_projection(
            CurveRegion2::empty(),
            metadata,
            exact_bounds,
            had_non_finite_input,
            finite_projection,
        );
    }
    match CurveRegion2::try_from_native_contours(material, holes, &policy) {
        Ok(region) => PcbRegion::new_with_exact_bounds_and_projection(
            region,
            metadata,
            exact_bounds,
            had_non_finite_input,
            finite_projection,
        ),
        Err(error) => PcbRegion::new_with_exact_bounds_and_projection(
            CurveRegion2::empty(),
            metadata,
            exact_bounds,
            had_non_finite_input,
            finite_projection,
        )
        .with_exact_construction_error(format!(
            "combined exact region construction failed: {error}"
        )),
    }
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

/// Create an empty native filled region with layer metadata.
pub fn empty_profile(metadata: Option<LayerMetadata>) -> PcbRegion {
    PcbRegion::new(CurveRegion2::empty(), metadata)
}

#[cfg(test)]
mod tests {
    use super::polygon_to_profile;
    use crate::geometry::{LineString, Polygon};

    #[test]
    fn invalid_exact_polygon_is_not_silently_recast_as_empty() {
        let polygon = Polygon::new(LineString(Vec::new()), Vec::new());
        let region = polygon_to_profile(polygon, None);

        assert!(region.is_empty());
        assert!(
            region
                .exact_construction_error()
                .is_some_and(|detail| detail.contains("fewer than three"))
        );
    }
}
