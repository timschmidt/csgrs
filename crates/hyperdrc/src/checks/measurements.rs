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
