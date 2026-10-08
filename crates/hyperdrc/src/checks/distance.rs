//! Certified distance queries over authoritative native PCB boundaries.

#[cfg(test)]
mod finite_legacy;
mod native;
#[cfg(test)]
mod native_regressions;

pub(super) use native::{
    PreparedRegionBoundary, compare_metrics, finite_enclosure, minimum_metric,
    point_region_boundary_within, region_boundaries_within, region_boundary_distance,
};
