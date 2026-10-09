//! Checked native envelope dimensions used by readiness proxies.

use crate::{PcbGeometryUncertainty, PcbRegion, Scalar};

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
