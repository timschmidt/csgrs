//! Native filled-region conversion helpers.
//!
//! Checks retain exact Hypercurve topology while parsers use local finite
//! coordinate views only until promotion at this boundary.

use hypercurve::CurveRegion2;

use super::{MultiPolygon, Polygon};
use crate::{LayerMetadata, PcbGeometryUncertainty, PcbRegion};

/// Convert one exact-backed polygon into a native filled region with metadata.
pub fn polygon_to_profile(polygon: Polygon<f64>, metadata: Option<LayerMetadata>) -> PcbRegion {
    polygons_to_profile(vec![polygon], metadata)
}

/// Combine exact-backed polygons into a native filled region with metadata.
/// Source or budget failures are retained by the returned owner; its checked
/// queries report that failure. Retry failed construction with the source
/// polygons in a fresh budget. Finite rings remain report-only views.
pub fn polygons_to_profile(
    polygons: Vec<Polygon<f64>>,
    metadata: Option<LayerMetadata>,
) -> PcbRegion {
    let had_non_finite_input = polygons.iter().any(|polygon| {
        polygon
            .exterior()
            .0
            .iter()
            .chain(polygon.interiors().iter().flat_map(|ring| ring.0.iter()))
            .any(|coordinate| !coordinate.x.is_finite() || !coordinate.y.is_finite())
    });
    let source = metadata.as_ref().map(|m| m.name.as_str());
    let construction = crate::operations::evaluate("profile-aggregation", source, || {
        let region =
            aggregate_exact_regions(&polygons).map_err(|detail| crate::PcbGeometryUncertainty {
                operation: "profile-aggregation".into(),
                source: source.map(str::to_owned),
                detail,
            })?;
        Ok((region, exact_input_polygon_bounds(&polygons)))
    });
    let finite_projection = (!had_non_finite_input).then_some(MultiPolygon(polygons));
    match construction {
        Ok((region, exact_bounds)) => PcbRegion::new_with_exact_bounds_and_projection(
            region,
            metadata,
            exact_bounds,
            had_non_finite_input,
            finite_projection,
        ),
        Err(error) => PcbRegion::new_with_exact_bounds_and_projection(
            std::sync::Arc::new(CurveRegion2::empty()),
            metadata,
            None,
            had_non_finite_input,
            finite_projection,
        )
        .with_exact_construction_error(error.to_string()),
    }
}

/// Combine checked PCB owners without requiring a finite report projection.
/// All selected sources are validated before empty or singleton identities.
pub(crate) fn regions_to_profile(
    regions: &[&PcbRegion],
    metadata: Option<LayerMetadata>,
) -> Result<PcbRegion, PcbGeometryUncertainty> {
    let source = metadata.as_ref().map(|m| m.name.as_str());
    let native = crate::operations::evaluate("profile-aggregation", source, || {
        let regions = regions
            .iter()
            .map(|region| {
                region.ensure_exact_geometry("profile-aggregation")?;
                Ok(region.region.clone())
            })
            .collect::<Result<Vec<_>, PcbGeometryUncertainty>>()?;
        aggregate_native_regions(&regions)
            .map_err(|detail| crate::operations::failure("profile-aggregation", source, detail))
    })?;
    Ok(PcbRegion::new_aggregate(native, metadata, regions))
}

fn aggregate_exact_regions(
    polygons: &[Polygon<f64>],
) -> Result<std::sync::Arc<CurveRegion2>, String> {
    let regions = polygons
        .iter()
        .map(|polygon| {
            polygon
                .try_exact_region()
                .map_err(|error| error.to_string())?;
            Ok(polygon.shared_exact_region())
        })
        .collect::<Result<Vec<_>, String>>()?;
    aggregate_native_regions(&regions)
}

fn aggregate_native_regions(
    regions: &[std::sync::Arc<CurveRegion2>],
) -> Result<std::sync::Arc<CurveRegion2>, String> {
    match regions {
        [] => return Ok(std::sync::Arc::new(CurveRegion2::empty())),
        [region] => return Ok(region.clone()),
        _ => {}
    }
    let mut material = Vec::new();
    let mut holes = Vec::new();
    for region in regions {
        let Some(native) = region
            .native_contours_fast_path()
            .map_err(|error| error.to_string())?
        else {
            // The line/arc adapter is only a bulk fast path. Other supported
            // carriers stay native and use the kernel's regularized union.
            return union_native_regions(regions).map_err(|error| error.to_string());
        };
        material.extend(native.material_contours().iter().cloned());
        holes.extend(native.hole_contours().iter().cloned());
    }
    if material.is_empty() && holes.is_empty() {
        return Ok(std::sync::Arc::new(CurveRegion2::empty()));
    }
    CurveRegion2::try_from_native_contours(material, holes)
        .map(std::sync::Arc::new)
        .map_err(|error| error.to_string())
}

fn union_native_regions(
    regions: &[std::sync::Arc<CurveRegion2>],
) -> hypercurve::ExactCurveResult<std::sync::Arc<CurveRegion2>> {
    if let [region] = regions {
        return Ok(region.clone());
    }
    let (left, right) = regions.split_at(regions.len() / 2);
    let left = union_native_regions(left)?;
    let right = union_native_regions(right)?;
    left.boolean_region(&right, hypercurve::BooleanOp::Union)
        .map(std::sync::Arc::new)
}

fn exact_input_polygon_bounds(polygons: &[Polygon<f64>]) -> Option<[hyperreal::Real; 4]> {
    let bounds = hypercurve::evaluate_under(crate::PREDICATE_POLICY, || {
        let mut bounds: Option<[hyperreal::Real; 4]> = None;
        for polygon in polygons {
            polygon.exact_construction_error().is_none().then_some(())?;
            if polygon.try_exact_region().ok()?.is_empty() {
                continue;
            }
            let next = polygon.exact_bounds().ok()??;
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
    if hypersolve::work_budget::exhaustion().is_some() {
        None
    } else {
        bounds
    }
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

        assert!(region.try_is_empty().is_err());
        assert!(
            region
                .exact_construction_error()
                .is_some_and(|detail| detail.contains("fewer than three"))
        );
    }
}
