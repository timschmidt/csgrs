//! Shared memoization for certified probe queries.

use super::{
    halfspace_cell_seed_families_from_optional_report, optional_halfspace_feasibility_report,
};
use crate::error::HypermeshResult;
use crate::geometry::{Aabb, Plane};
use crate::halfspace::limit_plane_families_match_as_sets;
use crate::polygon::ConvexPolygon;
use hyperlattice::Point3;
use hyperlimit::Plane3 as LimitPlane3;

#[derive(Clone, Debug, PartialEq)]
pub(super) struct SurfaceCacheEntry {
    point: Point3,
    on_surface: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct DirectProbeReachabilityCacheEntry {
    start: Point3,
    end: Point3,
    host_support: Plane,
    polygons: Vec<ConvexPolygon>,
    reachable: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct HalfspaceSeedFamilyCacheEntry {
    bounds: Aabb,
    halfspaces: Vec<LimitPlane3>,
    saw_unknown: bool,
    result: HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)>,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct HalfspaceReportCacheEntry {
    halfspaces: Vec<LimitPlane3>,
    saw_unknown: bool,
    report: Option<hyperlimit::HalfspaceFeasibilityReport>,
}

pub(super) fn cached_optional_halfspace_feasibility_report_with(
    cache: &mut Vec<HalfspaceReportCacheEntry>,
    halfspaces: &[LimitPlane3],
    saw_unknown: &mut bool,
) -> HypermeshResult<Option<hyperlimit::HalfspaceFeasibilityReport>> {
    if let Some(existing) = cache
        .iter()
        .rev()
        .find(|existing| limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces))
    {
        *saw_unknown |= existing.saw_unknown;
        return Ok(existing.report.clone());
    }

    let (report, local_unknown) = optional_halfspace_feasibility_report(halfspaces)?;
    cache.push(HalfspaceReportCacheEntry {
        halfspaces: halfspaces.to_vec(),
        saw_unknown: local_unknown,
        report: report.clone(),
    });
    *saw_unknown |= local_unknown;
    Ok(report)
}

pub(super) fn cached_halfspace_cell_seed_families_from_optional_report_with(
    cache: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    saw_unknown: &mut bool,
) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        existing.bounds == *bounds
            && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
    }) {
        *saw_unknown |= existing.saw_unknown;
        return existing.result.clone();
    }

    let mut local_unknown = false;
    let result = halfspace_cell_seed_families_from_optional_report(
        bounds,
        halfspaces,
        report,
        &mut local_unknown,
    );
    cache.push(HalfspaceSeedFamilyCacheEntry {
        bounds: bounds.clone(),
        halfspaces: halfspaces.to_vec(),
        saw_unknown: local_unknown,
        result: result.clone(),
    });
    *saw_unknown |= local_unknown;
    result
}

pub(super) fn cached_surface_query_with(
    cache: &mut Vec<SurfaceCacheEntry>,
    point: &Point3,
    query: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().rev().find(|existing| existing.point == *point) {
        return existing.on_surface.clone();
    }

    let on_surface = query();
    cache.push(SurfaceCacheEntry {
        point: point.clone(),
        on_surface: on_surface.clone(),
    });
    on_surface
}

pub(super) fn cached_direct_probe_reachability_with(
    cache: &mut Vec<DirectProbeReachabilityCacheEntry>,
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    query: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        existing.host_support == *host_support
            && existing.polygons == polygons
            && ((existing.start == *start && existing.end == *end)
                || (existing.start == *end && existing.end == *start))
    }) {
        return existing.reachable.clone();
    }

    let reachable = query();
    cache.push(DirectProbeReachabilityCacheEntry {
        start: start.clone(),
        end: end.clone(),
        host_support: host_support.clone(),
        polygons: polygons.to_vec(),
        reachable: reachable.clone(),
    });
    reachable
}
