//! Checked native envelope dimensions used by readiness proxies.

use crate::{PcbGeometryUncertainty, PcbRegion, Scalar};

pub(super) fn drill_radius(diameter: &Scalar) -> Result<Scalar, PcbGeometryUncertainty> {
    if crate::scalar::try_compare(diameter, &Scalar::zero())?.is_lt() {
        return Err(crate::operations::failure(
            "drill-radius",
            None,
            "negative drill diameter",
        ));
    }
    Ok(crate::scalar::half(diameter))
}

/// Signed edge gap of the retained circular drill proxies. Negative gaps mean
/// overlap. Invalid radii and unavailable native distances remain failures.
pub(super) fn drill_edge_gap(
    left: &crate::kicad::DrillFeature,
    right: &crate::kicad::DrillFeature,
) -> Result<Scalar, PcbGeometryUncertainty> {
    crate::operations::evaluate("drill-edge-gap", None, || {
        let left_radius = drill_radius(&left.diameter)?;
        let right_radius = drill_radius(&right.diameter)?;
        let dx = &left.location[0] - &right.location[0];
        let dy = &left.location[1] - &right.location[1];
        let distance = Scalar::hypot2(&dx, &dy)
            .map_err(|error| crate::operations::failure("drill-edge-gap", None, error))?;
        Ok(distance - left_radius - right_radius)
    })
}

/// Required closed-radius predicate over retained native coordinates. Squared
/// comparison avoids introducing a square root merely to decide a threshold.
pub(super) fn points_within_radius(
    left: &[Scalar; 2],
    right: &[Scalar; 2],
    radius: &Scalar,
) -> Result<bool, PcbGeometryUncertainty> {
    crate::operations::evaluate("point-radius-comparison", None, || {
        if crate::scalar::try_compare(radius, &Scalar::zero())?.is_lt() {
            return Ok(false);
        }
        let dx = &left[0] - &right[0];
        let dy = &left[1] - &right[1];
        let squared_distance = &dx * &dx + &dy * &dy;
        Ok(crate::scalar::try_compare(&squared_distance, &(radius * radius))?.is_le())
    })
}

pub(super) fn polygon_center_scalar(
    polygon: &crate::geometry::Polygon<f64>,
) -> Result<Option<[Scalar; 2]>, PcbGeometryUncertainty> {
    crate::operations::evaluate("polygon-center", None, || {
        Ok(polygon.try_bounds()?.map(|bounds| {
            [
                crate::scalar::half(&(&bounds[0] + &bounds[2])),
                crate::scalar::half(&(&bounds[1] + &bounds[3])),
            ]
        }))
    })
}

pub(super) fn report_point(point: &[Scalar; 2]) -> Result<[f64; 2], PcbGeometryUncertainty> {
    crate::operations::evaluate("point-report-projection", None, || {
        let mut result = [0.0; 2];
        for (target, coordinate) in result.iter_mut().zip(point) {
            *target = coordinate
                .to_f64_lossy()
                .filter(|value| value.is_finite())
                .ok_or_else(|| {
                    crate::operations::failure(
                        "point-report-projection",
                        None,
                        "coordinate has no finite report value",
                    )
                })?;
        }
        Ok(result)
    })
}

pub(super) fn polygon_report_center(
    polygon: &crate::geometry::Polygon<f64>,
) -> Result<Option<[f64; 2]>, PcbGeometryUncertainty> {
    polygon_center_scalar(polygon)?
        .as_ref()
        .map(report_point)
        .transpose()
}

pub(super) fn bounding_dimensions(
    region: &PcbRegion,
) -> Result<Option<(Scalar, Scalar)>, PcbGeometryUncertainty> {
    crate::operations::evaluate(
        "region-dimensions",
        region.metadata().as_ref().map(|m| m.name.as_str()),
        || {
            let Some(bounds) = region.try_native_bounds()? else {
                return Ok(None);
            };
            let width = &bounds[2] - &bounds[0];
            let height = &bounds[3] - &bounds[1];
            Ok(Some(
                if super::distance::compare_metrics(&width, &height)?.is_le() {
                    (width, height)
                } else {
                    (height, width)
                },
            ))
        },
    )
}

pub(super) fn minimum_bounding_dimension(
    region: &PcbRegion,
) -> Result<Option<Scalar>, PcbGeometryUncertainty> {
    Ok(bounding_dimensions(region)?.map(|(minimum, _)| minimum))
}

pub(super) fn maximum_bounding_dimension(
    region: &PcbRegion,
) -> Result<Option<Scalar>, PcbGeometryUncertainty> {
    Ok(bounding_dimensions(region)?.map(|(_, maximum)| maximum))
}

/// Readiness proxy: longest native exterior endpoint span, with native envelope
/// size for families without a line/arc view. This is not routed-path length.
/// Missing certificates and source failures cannot select the envelope fallback.
pub(super) fn segment_length_proxy(
    region: &PcbRegion,
) -> Result<Option<Scalar>, PcbGeometryUncertainty> {
    region.evaluate_native("segment-length-proxy", |native| {
        let Some(contours) = native
            .native_contours_fast_path()
            .map_err(|error| region.geometry_uncertainty("segment-length-proxy", error))?
        else {
            return maximum_bounding_dimension(region);
        };
        let mut maximum = None;
        for segment in contours
            .material_contours()
            .iter()
            .flat_map(|contour| contour.segments())
        {
            let dx = segment.end().x() - segment.start().x();
            let dy = segment.end().y() - segment.start().y();
            let length = (&dx * &dx + &dy * &dy)
                .sqrt()
                .map_err(|error| region.geometry_uncertainty("segment-length-proxy", error))?;
            if maximum.as_ref().map_or(Ok(true), |current| {
                super::distance::compare_metrics(&length, current).map(|order| order.is_gt())
            })? {
                maximum = Some(length);
            }
        }
        Ok(maximum)
    })
}
