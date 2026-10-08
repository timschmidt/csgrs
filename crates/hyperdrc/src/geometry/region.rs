//! Native filled-region conversion helpers.
//!
//! Checks retain exact Hypercurve topology while parsers use local finite
//! coordinate views only until promotion at this boundary.

use hypercurve::CurveRegion2;

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
    let mut material = Vec::new();
    let mut holes = Vec::new();
    for polygon in &polygons {
        let native = match polygon.exact_region().native_contours_fast_path() {
            Ok(Some(native)) => native,
            Ok(None) => {
                return PcbRegion::new_with_exact_bounds_and_projection(
                    CurveRegion2::empty(),
                    metadata,
                    exact_bounds,
                    had_non_finite_input,
                    finite_projection,
                )
                .with_exact_construction_error(
                    "exact native-contour extraction found no line/arc boundary".to_owned(),
                );
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
    match CurveRegion2::try_from_native_contours(material, holes) {
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
    let had_non_finite_input = polygons.iter().any(|polygon| {
        polygon
            .exterior()
            .0
            .iter()
            .chain(polygon.interiors().iter().flat_map(|ring| ring.0.iter()))
            .any(|coordinate| !coordinate.x.is_finite() || !coordinate.y.is_finite())
    });
    let bounds = hypercurve::evaluate_under(crate::PREDICATE_POLICY, || {
        let mut bounds: Option<[hyperreal::Real; 4]> = None;
        for polygon in polygons {
            polygon.exact_construction_error().is_none().then_some(())?;
            if polygon.exact_region().is_empty() {
                continue;
            }
            let next = polygon.exact_bounds()?;
            if let Some(bounds) = &mut bounds {
                for axis in 0..4 {
                    let order = crate::scalar::compare(&next[axis], &bounds[axis])?;
                    if (axis < 2 && order.is_lt()) || (axis >= 2 && order.is_gt()) {
                        bounds[axis] = next[axis].clone();
                    }
                }
            } else {
                bounds = Some(next.clone());
            }
        }
        bounds
    })
    .certified()
    .flatten();
    // A missing certificate disables this shortcut. Report samples can be
    // interior to a curve and cannot prove a rejection bound.
    let bounds = if hypersolve::work_budget::exhaustion().is_some() {
        None
    } else {
        bounds
    };
    (bounds, had_non_finite_input)
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
