//! Certified adjacent-cell reachability and plane-replacement search.

use super::probe_cache::{
    DirectProbeReachabilityCacheEntry, HalfspaceReportCacheEntry, HalfspaceSeedFamilyCacheEntry,
    SurfaceCacheEntry, cached_direct_probe_reachability_with,
    cached_halfspace_cell_seed_families_from_optional_report_with,
    cached_optional_halfspace_feasibility_report_with, cached_surface_query_with,
};
#[cfg(test)]
use super::strict_aabb_targets;
use super::{
    AXIS_ORDERINGS, DefinitionCycleGuardReachabilityCache, DefinitionNoDetourReachabilityCache,
    DefinitionNoPlaneReplacementCycleGuardCache, DefinitionNoPlaneReplacementReachabilityCache,
    DetourArrangementCellState, DetourTarget, DetourTargetFamilyCache,
    DetourTargetFamilyCacheEntry, InteriorBoxAxisIntervalsCache, InteriorBoxDetourTargetBatchCache,
    InteriorLeafPoint, PlaneReplacementAffineCache, PlaneReplacementNoNestedOrderingWarmupBucket,
    PlaneReplacementNoNestedOrderingWarmupCache, PlaneReplacementNoNestedOrderingWarmupCacheEntry,
    PlaneReplacementReachabilityPathBucket, PlaneReplacementReachabilityPathCache,
    PlaneReplacementReachabilityPathCacheEntry, PlaneReplacementReachabilityStepBucket,
    PlaneReplacementReachabilityStepCache, PlaneReplacementReachabilityStepCacheEntry,
    PlaneReplacementReachabilityStepMode, PolygonPointLocation, ProbePoint,
    StrictAabbTargetFamilyCache, StrictAabbTargetFamilyCacheEntry, VisitedDefinitionPoint,
    aabb_from_axis_intervals, adapt_plane_replacement_vertex_to_trace_bounds, affine_from_planes,
    begin_definition_cycle_guard_result, begin_definition_no_detour_reachability_result,
    begin_definition_no_plane_replacement_cycle_guard_result,
    begin_definition_no_plane_replacement_reachability_result, cached_affine_from_planes_with,
    cached_definition_cycle_guard_result, cached_definition_no_detour_reachability_result,
    cached_definition_no_plane_replacement_cycle_guard_result,
    cached_definition_no_plane_replacement_reachability_result, cached_detour_target_family,
    cached_detour_target_family_with, cached_interior_box_axis_intervals_with_surface_queries,
    cached_strict_aabb_target_families, classify_point_in_polygon,
    definition_families_match_as_sets, definition_planes_match_as_sets, detour_arrangement_cell,
    detour_arrangement_cell_state_is_dominated, detour_arrangement_planes,
    detour_target_family_result_from_targets, endpoint_definition_family,
    evaluate_strict_aabb_target_families_with_direct_ranking, first_changed_axis,
    initial_visited_definition_points, interior_box_axis_intervals_with_surface_queries,
    interior_box_detour_targets, normalized_cycle_guard_visited_points, planes_are_coplanar,
    point_is_inside_optional_trace_bounds, point_lies_on_traced_surface,
    point_strictly_between_axis, push_detour_target_family_bucket_entry,
    push_strict_aabb_target_family_bucket_entry, push_unique_detour_target,
    record_detour_arrangement_cell_state,
    search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking_outcome,
    segment_plane_crossing, unique_definition_family, visited_definition_family_contains,
    visited_definition_points_match_as_sets,
};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Aabb, Classification, Plane, classify_point};
use crate::polygon::ConvexPolygon;
use hyperlattice::Point3;

pub(super) fn probe_reaches_adjacent_cell(
    start: &Point3,
    probe: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let Some(sort_axis) = first_changed_axis(start, probe)? else {
        for polygon in polygons {
            if polygon.mesh_index < 0 {
                continue;
            }

            if planes_are_coplanar(&polygon.support, host_support)? {
                continue;
            }

            match classify_point_in_polygon(start, polygon)? {
                PolygonPointLocation::Outside => {}
                PolygonPointLocation::Boundary | PolygonPointLocation::Interior => {
                    return Err(HypermeshError::UnknownClassification);
                }
            }
        }
        return Ok(true);
    };

    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        let start_class = classify_point(start, &polygon.support)?;
        let probe_class = classify_point(probe, &polygon.support)?;

        if start_class == Classification::On {
            if planes_are_coplanar(&polygon.support, host_support)? {
                continue;
            }
            match classify_point_in_polygon(start, polygon)? {
                PolygonPointLocation::Outside => {}
                PolygonPointLocation::Boundary | PolygonPointLocation::Interior => {
                    return Err(HypermeshError::UnknownClassification);
                }
            }
            continue;
        }

        if probe_class == Classification::On {
            match classify_point_in_polygon(probe, polygon)? {
                PolygonPointLocation::Outside => {}
                PolygonPointLocation::Boundary | PolygonPointLocation::Interior => {
                    return Err(HypermeshError::UnknownClassification);
                }
            }
            continue;
        }

        if start_class == probe_class {
            continue;
        }

        let Some(crossing) = segment_plane_crossing(start, probe, &polygon.support)? else {
            continue;
        };
        if point_strictly_between_axis(&crossing, start, probe, sort_axis)? {
            match classify_point_in_polygon(&crossing, polygon)? {
                PolygonPointLocation::Outside => {}
                PolygonPointLocation::Boundary => {
                    return Err(HypermeshError::UnknownClassification);
                }
                PolygonPointLocation::Interior => {
                    return Ok(false);
                }
            }
        }
    }

    Ok(true)
}

pub(super) fn probe_polyline_reaches_adjacent_cell(
    points: &[Point3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    if points.is_empty() {
        return Err(HypermeshError::UnknownClassification);
    }

    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        let classifications = points
            .iter()
            .map(|point| classify_point(point, &polygon.support))
            .collect::<HypermeshResult<Vec<_>>>()?;

        if classifications[0] == Classification::On {
            if planes_are_coplanar(&polygon.support, host_support)? {
                continue;
            }
            if classify_point_in_polygon(&points[0], polygon)? != PolygonPointLocation::Outside {
                return Err(HypermeshError::UnknownClassification);
            }
        }

        let last = points.len() - 1;
        if last > 0
            && classifications[last] == Classification::On
            && classify_point_in_polygon(&points[last], polygon)? != PolygonPointLocation::Outside
        {
            return Err(HypermeshError::UnknownClassification);
        }

        for index in 0..last {
            let start_class = classifications[index];
            let end_class = classifications[index + 1];
            if start_class == Classification::On
                || end_class == Classification::On
                || start_class == end_class
            {
                continue;
            }

            let Some(sort_axis) = first_changed_axis(&points[index], &points[index + 1])? else {
                continue;
            };
            let Some(crossing) =
                segment_plane_crossing(&points[index], &points[index + 1], &polygon.support)?
            else {
                continue;
            };
            if !point_strictly_between_axis(
                &crossing,
                &points[index],
                &points[index + 1],
                sort_axis,
            )? {
                continue;
            }
            match classify_point_in_polygon(&crossing, polygon)? {
                PolygonPointLocation::Outside => {}
                PolygonPointLocation::Boundary => {
                    return Err(HypermeshError::UnknownClassification);
                }
                PolygonPointLocation::Interior => return Ok(false),
            }
        }

        let mut index = 1;
        while index < last {
            if classifications[index] != Classification::On {
                index += 1;
                continue;
            }
            let run_start = index;
            while index + 1 < last && classifications[index + 1] == Classification::On {
                index += 1;
            }
            let run_end = index;
            let mut touches_interior = false;
            for point in &points[run_start..=run_end] {
                match classify_point_in_polygon(point, polygon)? {
                    PolygonPointLocation::Outside => {}
                    PolygonPointLocation::Boundary => {
                        return Err(HypermeshError::UnknownClassification);
                    }
                    PolygonPointLocation::Interior => touches_interior = true,
                }
            }
            if touches_interior && classifications[run_start - 1] != classifications[run_end + 1] {
                return Ok(false);
            }
            if run_start != run_end {
                return Err(HypermeshError::UnknownClassification);
            }
            index += 1;
        }
    }

    Ok(true)
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_from_interior(
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let mut surface_cache = Vec::new();
    let mut plane_replacement_affine = PlaneReplacementAffineCache::default();
    let mut plane_replacement_reachability_paths = PlaneReplacementReachabilityPathCache::default();
    let mut plane_replacement_reachability_steps = PlaneReplacementReachabilityStepCache::default();
    let mut plane_replacement_no_nested_ordering_warmups =
        PlaneReplacementNoNestedOrderingWarmupCache::default();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    let mut definition_cycle_guard_reachability = DefinitionCycleGuardReachabilityCache::default();
    let mut definition_no_step_detour_reachability = DefinitionNoDetourReachabilityCache::default();
    let mut definition_no_plane_replacement_cycle_guard =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let mut definition_no_plane_replacement_reachability =
        DefinitionNoPlaneReplacementReachabilityCache::default();
    let mut halfspace_reports = Vec::new();
    let mut halfspace_seed_families = Vec::new();
    let mut no_step_detour_target_families = DetourTargetFamilyCache::default();
    let mut definition_full_no_detour_reachability = DefinitionNoDetourReachabilityCache::default();
    let mut definition_no_detour_reachability = DefinitionNoDetourReachabilityCache::default();
    let mut direct_probe_reachability = Vec::new();
    let mut detour_target_families = DetourTargetFamilyCache::default();
    probe_reaches_adjacent_cell_from_interior_with_caches(
        interior,
        probe,
        host_support,
        polygons,
        &mut surface_cache,
        &mut plane_replacement_affine,
        &mut plane_replacement_reachability_paths,
        &mut plane_replacement_reachability_steps,
        &mut plane_replacement_no_nested_ordering_warmups,
        &mut interior_box_axis_intervals,
        &mut definition_cycle_guard_reachability,
        &mut definition_no_step_detour_reachability,
        &mut definition_no_plane_replacement_cycle_guard,
        &mut definition_no_plane_replacement_reachability,
        &mut halfspace_reports,
        &mut halfspace_seed_families,
        &mut no_step_detour_target_families,
        &mut definition_full_no_detour_reachability,
        &mut definition_no_detour_reachability,
        &mut direct_probe_reachability,
        &mut detour_target_families,
        None,
    )
}

pub(super) fn probe_reaches_adjacent_cell_from_interior_with_caches(
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    plane_replacement_affine: &mut PlaneReplacementAffineCache,
    plane_replacement_reachability_paths: &mut PlaneReplacementReachabilityPathCache,
    plane_replacement_reachability_steps: &mut PlaneReplacementReachabilityStepCache,
    plane_replacement_no_nested_ordering_warmups: &mut PlaneReplacementNoNestedOrderingWarmupCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    definition_cycle_guard_reachability: &mut DefinitionCycleGuardReachabilityCache,
    definition_no_step_detour_reachability: &mut DefinitionNoDetourReachabilityCache,
    definition_no_plane_replacement_cycle_guard: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    definition_no_plane_replacement_reachability:
        &mut DefinitionNoPlaneReplacementReachabilityCache,
    halfspace_reports: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_families: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    no_step_detour_target_families: &mut DetourTargetFamilyCache,
    definition_full_no_detour_reachability: &mut DefinitionNoDetourReachabilityCache,
    definition_no_detour_reachability: &mut DefinitionNoDetourReachabilityCache,
    direct_probe_reachability: &mut Vec<DirectProbeReachabilityCacheEntry>,
    detour_target_families: &mut DetourTargetFamilyCache,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<bool> {
    if !point_is_inside_optional_trace_bounds(&interior.point, trace_bounds)?
        || !point_is_inside_optional_trace_bounds(&probe.point, trace_bounds)?
    {
        return Err(HypermeshError::UnknownClassification);
    }
    let start_family = endpoint_definition_family(&interior.point, &interior.planes)?;
    let end_family = endpoint_definition_family(&probe.point, &probe.planes)?;
    let saw_unknown = start_family.saw_unknown || end_family.saw_unknown;

    let result = probe_reaches_adjacent_cell_with_cycle_guard_with_caches(
        &interior.point,
        &probe.point,
        host_support,
        polygons,
        &start_family.definitions,
        &end_family.definitions,
        surface_cache,
        plane_replacement_affine,
        plane_replacement_reachability_paths,
        plane_replacement_reachability_steps,
        plane_replacement_no_nested_ordering_warmups,
        interior_box_axis_intervals,
        definition_cycle_guard_reachability,
        definition_no_step_detour_reachability,
        definition_no_plane_replacement_cycle_guard,
        definition_no_plane_replacement_reachability,
        halfspace_reports,
        halfspace_seed_families,
        no_step_detour_target_families,
        definition_full_no_detour_reachability,
        definition_no_detour_reachability,
        direct_probe_reachability,
        detour_target_families,
        trace_bounds,
    );
    match result {
        Ok(false) if saw_unknown => Err(HypermeshError::UnknownClassification),
        result => result,
    }
}

#[cfg(test)]
#[allow(dead_code)]
fn probe_reaches_adjacent_cell_with_cycle_guard(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    let mut surface_cache = Vec::new();
    let mut plane_replacement_affine = PlaneReplacementAffineCache::default();
    let mut plane_replacement_reachability_paths = PlaneReplacementReachabilityPathCache::default();
    let mut plane_replacement_reachability_steps = PlaneReplacementReachabilityStepCache::default();
    let mut plane_replacement_no_nested_ordering_warmups =
        PlaneReplacementNoNestedOrderingWarmupCache::default();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    let mut definition_cycle_guard_reachability = DefinitionCycleGuardReachabilityCache::default();
    let mut no_step_cache = DefinitionNoDetourReachabilityCache::default();
    let mut no_plane_replacement_cycle_guard_cache =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let mut no_plane_replacement_cache = DefinitionNoPlaneReplacementReachabilityCache::default();
    let mut halfspace_reports = Vec::new();
    let mut halfspace_seed_families = Vec::new();
    let mut no_step_detour_target_cache = DetourTargetFamilyCache::default();
    let mut full_no_detour_cache = DefinitionNoDetourReachabilityCache::default();
    let mut no_detour_cache = DefinitionNoDetourReachabilityCache::default();
    let mut direct_probe_reachability_cache = Vec::new();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    probe_reaches_adjacent_cell_with_cycle_guard_with_caches(
        start,
        end,
        host_support,
        polygons,
        start_definitions,
        end_definitions,
        &mut surface_cache,
        &mut plane_replacement_affine,
        &mut plane_replacement_reachability_paths,
        &mut plane_replacement_reachability_steps,
        &mut plane_replacement_no_nested_ordering_warmups,
        &mut interior_box_axis_intervals,
        &mut definition_cycle_guard_reachability,
        &mut no_step_cache,
        &mut no_plane_replacement_cycle_guard_cache,
        &mut no_plane_replacement_cache,
        &mut halfspace_reports,
        &mut halfspace_seed_families,
        &mut no_step_detour_target_cache,
        &mut full_no_detour_cache,
        &mut no_detour_cache,
        &mut direct_probe_reachability_cache,
        &mut detour_target_cache,
        None,
    )
}

fn probe_reaches_adjacent_cell_with_cycle_guard_with_caches(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    plane_replacement_affine: &mut PlaneReplacementAffineCache,
    plane_replacement_reachability_paths: &mut PlaneReplacementReachabilityPathCache,
    plane_replacement_reachability_steps: &mut PlaneReplacementReachabilityStepCache,
    plane_replacement_no_nested_ordering_warmups: &mut PlaneReplacementNoNestedOrderingWarmupCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    definition_cycle_guard_reachability: &mut DefinitionCycleGuardReachabilityCache,
    no_step_cache: &mut DefinitionNoDetourReachabilityCache,
    no_plane_replacement_cycle_guard_cache: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    no_plane_replacement_cache: &mut DefinitionNoPlaneReplacementReachabilityCache,
    halfspace_reports: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_families: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    no_step_detour_target_cache: &mut DetourTargetFamilyCache,
    full_no_detour_cache: &mut DefinitionNoDetourReachabilityCache,
    no_detour_cache: &mut DefinitionNoDetourReachabilityCache,
    direct_probe_reachability_cache: &mut Vec<DirectProbeReachabilityCacheEntry>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<bool> {
    let visited_points =
        initial_visited_definition_points(start, start_definitions, end, end_definitions);
    if let Some(existing) = cached_definition_cycle_guard_result(
        definition_cycle_guard_reachability,
        start,
        end,
        start_definitions,
        end_definitions,
        &visited_points,
    ) {
        return existing;
    }
    let cache_index = begin_definition_cycle_guard_result(
        definition_cycle_guard_reachability,
        start,
        end,
        start_definitions,
        end_definitions,
        &visited_points,
    );
    let mut trace_without_detours =
        |start: &Point3,
         end: &Point3,
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            probe_reaches_adjacent_cell_with_definitions_no_detours_with_caches(
                start,
                end,
                host_support,
                polygons,
                start_definitions,
                end_definitions,
                plane_replacement_affine,
                plane_replacement_reachability_paths,
                plane_replacement_reachability_steps,
                plane_replacement_no_nested_ordering_warmups,
                interior_box_axis_intervals,
                no_step_cache,
                halfspace_reports,
                halfspace_seed_families,
                no_plane_replacement_cycle_guard_cache,
                no_plane_replacement_cache,
                no_step_detour_target_cache,
                full_no_detour_cache,
                no_detour_cache,
                direct_probe_reachability_cache,
                trace_bounds,
            )
        };
    let arrangement_planes = detour_arrangement_planes(polygons);
    let mut detour_batches = InteriorBoxDetourTargetBatchCache::default();
    let result = probe_reaches_adjacent_cell_with_detour_batches_breadth_first_with_surface_query(
        start,
        end,
        start_definitions,
        end_definitions,
        &arrangement_planes,
        surface_cache,
        &mut |point| {
            if !point_is_inside_optional_trace_bounds(point, trace_bounds)? {
                return Ok(true);
            }
            point_lies_on_traced_surface(point, polygons)
        },
        &mut trace_without_detours,
        &mut |batch_start, batch_end, batch_index| {
            if let Some(cached) = cached_detour_target_family(
                detour_target_cache,
                batch_start,
                batch_end,
                trace_bounds,
            ) {
                if batch_index == 0 {
                    cached.targets.clone().map(Some)
                } else {
                    Ok(None)
                }
            } else {
                detour_batches.batch_for(
                    batch_start,
                    batch_end,
                    batch_index,
                    polygons,
                    &arrangement_planes,
                    trace_bounds,
                )
            }
        },
    );
    definition_cycle_guard_reachability.entries[cache_index].result = result.clone();
    result
}

#[cfg(test)]
#[allow(dead_code)]
fn probe_reaches_adjacent_cell_with_definitions_budget(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
) -> HypermeshResult<bool> {
    let mut trace_without_detours =
        |start: &Point3,
         end: &Point3,
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            probe_reaches_adjacent_cell_with_definitions_no_detours(
                start,
                end,
                host_support,
                polygons,
                start_definitions,
                end_definitions,
            )
        };
    let mut detours_for =
        |start: &Point3, end: &Point3| interior_box_detour_targets(start, end, polygons);
    probe_reaches_adjacent_cell_with_definitions_budget_impl(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        remaining_detours,
        &mut trace_without_detours,
        &mut detours_for,
    )
}

pub(super) fn cached_definition_no_detour_reachability_with(
    cache: &mut DefinitionNoDetourReachabilityCache,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    trace: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cached_definition_no_detour_reachability_result(
        cache,
        start,
        end,
        start_definitions,
        end_definitions,
    ) {
        return existing;
    }

    let cache_index = begin_definition_no_detour_reachability_result(
        cache,
        start,
        end,
        start_definitions,
        end_definitions,
    );
    let result = trace();
    cache.entries[cache_index].result = result.clone();
    result
}

#[cfg(test)]
pub(super) fn cached_definition_no_plane_replacement_reachability_with(
    cache: &mut DefinitionNoPlaneReplacementReachabilityCache,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    trace: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cached_definition_no_plane_replacement_reachability_result(
        cache,
        start,
        end,
        start_definitions,
        end_definitions,
    ) {
        return existing;
    }

    let cache_index = begin_definition_no_plane_replacement_reachability_result(
        cache,
        start,
        end,
        start_definitions,
        end_definitions,
    );
    let result = trace();
    cache.entries[cache_index].result = result.clone();
    result
}

#[cfg(test)]
#[allow(dead_code)]
pub(super) fn probe_reaches_adjacent_cell_with_cycle_guard_impl(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut surface_cache = Vec::new();
    probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        visited_points,
        &mut surface_cache,
        &mut |point| point_lies_on_traced_surface(point, polygons),
        trace_without_detours,
        detours_for,
    )
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let no_detour_unknown =
        match trace_without_detours(start, end, start_definitions, end_definitions) {
            Ok(true) => return Ok(true),
            Ok(false) => false,
            Err(HypermeshError::UnknownClassification) => true,
            Err(err) => return Err(err),
        };

    let detour_result =
        probe_reaches_adjacent_cell_via_detours_with_cycle_guard_with_surface_query(
            start,
            end,
            polygons,
            start_definitions,
            end_definitions,
            visited_points,
            surface_cache,
            surface_query,
            trace_without_detours,
            detours_for,
        )?;

    if detour_result {
        Ok(true)
    } else if no_detour_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    }
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_with_definitions_budget_impl(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let no_detour_unknown =
        match trace_without_detours(start, end, start_definitions, end_definitions) {
            Ok(true) => return Ok(true),
            Ok(false) => false,
            Err(HypermeshError::UnknownClassification) => true,
            Err(err) => return Err(err),
        };

    if remaining_detours == 0 {
        return if no_detour_unknown {
            Err(HypermeshError::UnknownClassification)
        } else {
            Ok(false)
        };
    }

    let detour_result = probe_reaches_adjacent_cell_via_detours_with_budget(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        remaining_detours,
        trace_without_detours,
        detours_for,
    )?;

    if detour_result {
        Ok(true)
    } else if no_detour_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    }
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_via_detours_with_cycle_guard(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut surface_cache = Vec::new();
    probe_reaches_adjacent_cell_via_detours_with_cycle_guard_with_surface_query(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        visited_points,
        &mut surface_cache,
        &mut |point| point_lies_on_traced_surface(point, polygons),
        trace_without_detours,
        detours_for,
    )
}

#[cfg(test)]
fn probe_reaches_adjacent_cell_via_detours_with_cycle_guard_with_surface_query(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for detour in detours_for(start, end)? {
        if evaluate_probe_detour_target_with_cycle_guard_with_surface_query(
            &detour,
            start,
            end,
            polygons,
            start_definitions,
            end_definitions,
            visited_points,
            surface_cache,
            surface_query,
            trace_without_detours,
            detours_for,
            &mut saw_unknown,
        )? {
            return Ok(true);
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    }
}

#[cfg(test)]
#[allow(dead_code)]
fn probe_reaches_adjacent_cell_via_interior_box_detours_progressively_with_surface_query(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let (intervals, mut saw_unknown) = cached_interior_box_axis_intervals_with_surface_queries(
        interior_box_axis_intervals,
        start,
        end,
        || {
            interior_box_axis_intervals_with_surface_queries(
                start,
                end,
                polygons,
                &mut |edge_start, edge_end, polygon, axis| {
                    let start_class = classify_point(edge_start, &polygon.support)?;
                    let end_class = classify_point(edge_end, &polygon.support)?;
                    if start_class == Classification::On {
                        return Ok(Some(edge_start.clone()));
                    }
                    if end_class == Classification::On {
                        return Ok(Some(edge_end.clone()));
                    }
                    segment_plane_crossing(edge_start, edge_end, &polygon.support).and_then(
                        |crossing| {
                            if let Some(crossing) = crossing {
                                if !point_strictly_between_axis(
                                    &crossing, edge_start, edge_end, axis,
                                )? {
                                    return Ok(None);
                                }
                                Ok(Some(crossing))
                            } else {
                                Ok(None)
                            }
                        },
                    )
                },
                &mut |crossing, polygon| classify_point_in_polygon(crossing, polygon),
            )
        },
    )?;
    for x in &intervals[0] {
        for y in &intervals[1] {
            for z in &intervals[2] {
                let bounds = aabb_from_axis_intervals([x, y, z])?;
                let detours = match strict_aabb_targets(&bounds) {
                    Ok(detours) => detours,
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
                for detour in detours {
                    if evaluate_probe_detour_target_with_cycle_guard_with_surface_query(
                        &detour,
                        start,
                        end,
                        polygons,
                        start_definitions,
                        end_definitions,
                        visited_points,
                        surface_cache,
                        surface_query,
                        trace_without_detours,
                        detours_for,
                        &mut saw_unknown,
                    )? {
                        return Ok(true);
                    }
                }
            }
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    }
}

#[cfg(test)]
fn evaluate_probe_detour_target_with_cycle_guard_with_surface_query(
    detour: &DetourTarget,
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
    saw_unknown: &mut bool,
) -> HypermeshResult<bool> {
    let start_definition_transition = detour.point == *start
        && !definition_families_match_as_sets(&detour.definitions, start_definitions);
    let end_definition_transition = detour.point == *end
        && !definition_families_match_as_sets(&detour.definitions, end_definitions);
    let zero_length_definition_transition =
        start_definition_transition || end_definition_transition;
    let already_visited =
        visited_definition_family_contains(visited_points, &detour.point, &detour.definitions);
    let on_surface = if already_visited || zero_length_definition_transition {
        false
    } else {
        match cached_surface_query_with(surface_cache, &detour.point, || {
            surface_query(&detour.point)
        }) {
            Ok(on_surface) => on_surface,
            Err(HypermeshError::UnknownClassification) => {
                *saw_unknown = true;
                return Ok(false);
            }
            Err(err) => return Err(err),
        }
    };
    if already_visited || on_surface {
        if detour.uncertified_definition_fallback {
            *saw_unknown = true;
        }
        return Ok(false);
    }

    let mut next_visited_points = visited_points.to_vec();
    if !visited_definition_family_contains(&next_visited_points, &detour.point, &detour.definitions)
    {
        next_visited_points.push(VisitedDefinitionPoint {
            point: detour.point.clone(),
            definitions: detour.definitions.clone(),
        });
    }

    let first_leg = match if start_definition_transition {
        trace_without_detours(start, &detour.point, start_definitions, &detour.definitions)
    } else {
        probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
            start,
            &detour.point,
            polygons,
            start_definitions,
            &detour.definitions,
            &next_visited_points,
            surface_cache,
            surface_query,
            trace_without_detours,
            detours_for,
        )
    } {
        Ok(result) => result,
        Err(HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            return Ok(false);
        }
        Err(err) => return Err(err),
    };
    if !first_leg {
        if detour.uncertified_definition_fallback {
            *saw_unknown = true;
        }
        return Ok(false);
    }

    match if end_definition_transition {
        trace_without_detours(&detour.point, end, &detour.definitions, end_definitions)
    } else {
        probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
            &detour.point,
            end,
            polygons,
            &detour.definitions,
            end_definitions,
            &next_visited_points,
            surface_cache,
            surface_query,
            trace_without_detours,
            detours_for,
        )
    } {
        Ok(true) => Ok(true),
        Ok(false) => {
            if detour.uncertified_definition_fallback {
                *saw_unknown = true;
            }
            Ok(false)
        }
        Err(HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            Ok(false)
        }
        Err(err) => Err(err),
    }
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_via_progressive_detours(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    let mut no_detour_cache = DefinitionNoDetourReachabilityCache::default();
    let mut no_plane_replacement_cycle_guard_cache =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let mut no_plane_replacement_cache = DefinitionNoPlaneReplacementReachabilityCache::default();
    let mut halfspace_report_cache = Vec::new();
    let mut halfspace_seed_family_cache = Vec::new();
    let mut strict_aabb_target_families = StrictAabbTargetFamilyCache::default();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    probe_reaches_adjacent_cell_with_interior_box_detours_without_plane_replacement_from_definitions_with(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        &mut no_detour_cache,
        &mut no_plane_replacement_cycle_guard_cache,
        &mut no_plane_replacement_cache,
        &mut halfspace_report_cache,
        &mut halfspace_seed_family_cache,
        &mut strict_aabb_target_families,
        &mut detour_target_cache,
        &mut interior_box_axis_intervals,
        None,
        |start: &Point3,
         end: &Point3,
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            probe_reaches_adjacent_cell_with_definitions_no_detours(
                start,
                end,
                host_support,
                polygons,
                start_definitions,
                end_definitions,
            )
        },
        |start: &Point3, end: &Point3| interior_box_detour_targets(start, end, polygons),
    )
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_via_detours_with_budget(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    let mut surface_cache = Vec::new();
    for detour in detours_for(start, end)? {
        if detour.point == *start
            || detour.point == *end
            || cached_surface_query_with(&mut surface_cache, &detour.point, || {
                point_lies_on_traced_surface(&detour.point, polygons)
            })?
        {
            if detour.uncertified_definition_fallback {
                saw_unknown = true;
            }
            continue;
        }
        let first_leg = match probe_reaches_adjacent_cell_with_definitions_budget_impl(
            start,
            &detour.point,
            polygons,
            start_definitions,
            &detour.definitions,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        ) {
            Ok(result) => result,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if !first_leg {
            if detour.uncertified_definition_fallback {
                saw_unknown = true;
            }
            continue;
        }
        match probe_reaches_adjacent_cell_with_definitions_budget_impl(
            &detour.point,
            end,
            polygons,
            &detour.definitions,
            end_definitions,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        ) {
            Ok(true) => return Ok(true),
            Ok(false) => {
                if detour.uncertified_definition_fallback {
                    saw_unknown = true;
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    }
}

#[cfg(test)]
fn probe_reaches_adjacent_cell_with_definitions_no_detours(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut path_cache = PlaneReplacementReachabilityPathCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();
    let mut no_nested_ordering_warmup_cache =
        PlaneReplacementNoNestedOrderingWarmupCache::default();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    let mut no_step_cache = DefinitionNoDetourReachabilityCache::default();
    let mut halfspace_reports = Vec::new();
    let mut halfspace_seed_families = Vec::new();
    let mut no_plane_replacement_cycle_guard_cache =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let mut no_plane_replacement_cache = DefinitionNoPlaneReplacementReachabilityCache::default();
    let mut no_step_detour_target_cache = DetourTargetFamilyCache::default();
    let mut full_no_detour_cache = DefinitionNoDetourReachabilityCache::default();
    let mut no_detour_cache = DefinitionNoDetourReachabilityCache::default();
    let mut direct_probe_reachability_cache = Vec::new();
    probe_reaches_adjacent_cell_with_definitions_no_detours_with_caches(
        start,
        end,
        host_support,
        polygons,
        start_definitions,
        end_definitions,
        &mut affine_cache,
        &mut path_cache,
        &mut step_cache,
        &mut no_nested_ordering_warmup_cache,
        &mut interior_box_axis_intervals,
        &mut no_step_cache,
        &mut halfspace_reports,
        &mut halfspace_seed_families,
        &mut no_plane_replacement_cycle_guard_cache,
        &mut no_plane_replacement_cache,
        &mut no_step_detour_target_cache,
        &mut full_no_detour_cache,
        &mut no_detour_cache,
        &mut direct_probe_reachability_cache,
        None,
    )
}

pub(super) fn probe_reaches_adjacent_cell_with_definitions_no_detours_with_caches(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    affine_cache: &mut PlaneReplacementAffineCache,
    path_cache: &mut PlaneReplacementReachabilityPathCache,
    step_cache: &mut PlaneReplacementReachabilityStepCache,
    no_nested_ordering_warmup_cache: &mut PlaneReplacementNoNestedOrderingWarmupCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    no_step_cache: &mut DefinitionNoDetourReachabilityCache,
    halfspace_reports: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_families: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    no_plane_replacement_cycle_guard_cache: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    no_plane_replacement_cache: &mut DefinitionNoPlaneReplacementReachabilityCache,
    no_step_detour_target_cache: &mut DetourTargetFamilyCache,
    full_no_detour_cache: &mut DefinitionNoDetourReachabilityCache,
    no_detour_cache: &mut DefinitionNoDetourReachabilityCache,
    direct_probe_reachability_cache: &mut Vec<DirectProbeReachabilityCacheEntry>,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<bool> {
    let mut strict_aabb_target_families = StrictAabbTargetFamilyCache::default();
    cached_definition_no_detour_reachability_with(
        full_no_detour_cache,
        start,
        end,
        start_definitions,
        end_definitions,
        || {
            let direct_unknown = match cached_direct_probe_reachability_with(
                direct_probe_reachability_cache,
                start,
                end,
                host_support,
                polygons,
                || probe_reaches_adjacent_cell(start, end, host_support, polygons),
            ) {
                Ok(true) => return Ok(true),
                Ok(false) => false,
                Err(HypermeshError::UnknownClassification) => true,
                Err(err) => return Err(err),
            };

            match definition_search_precheck_plan(
                start,
                end,
                start_definitions,
                end_definitions,
                direct_unknown,
                |start_definition, end_definition| {
                    probe_reaches_adjacent_cell_with_definitions_no_step_detours_with_caches(
                        start,
                        end,
                        host_support,
                        polygons,
                        std::slice::from_ref(start_definition),
                        std::slice::from_ref(end_definition),
                        affine_cache,
                        path_cache,
                        step_cache,
                        no_step_cache,
                        direct_probe_reachability_cache,
                        trace_bounds,
                    )
                },
            )? {
                DefinitionSearchPrecheckOutcome::Reaches => Ok(true),
                DefinitionSearchPrecheckOutcome::Search(plan) => {
                    let mut saw_unknown = plan.unknown_if_no_match;
                    for (start_index, end_index) in plan.ordered_pairs {
                        let pair_result = plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement_with_caches(
                            &plan.start_definitions[start_index],
                            &plan.end_definitions[end_index],
                            host_support,
                            polygons,
                            affine_cache,
                            path_cache,
                            step_cache,
                            no_nested_ordering_warmup_cache,
                            interior_box_axis_intervals,
                            no_step_cache,
                            halfspace_reports,
                            halfspace_seed_families,
                            no_detour_cache,
                            no_plane_replacement_cycle_guard_cache,
                            no_plane_replacement_cache,
                            &mut strict_aabb_target_families,
                            no_step_detour_target_cache,
                            direct_probe_reachability_cache,
                            trace_bounds,
                        );
                        match pair_result {
                            Ok(true) => return Ok(true),
                            Ok(false) => {}
                            Err(HypermeshError::UnknownClassification) => saw_unknown = true,
                            Err(err) => return Err(err),
                        }
                    }

                    if saw_unknown {
                        Err(HypermeshError::UnknownClassification)
                    } else {
                        Ok(false)
                    }
                }
            }
        },
    )
}

#[cfg(test)]
#[allow(dead_code)]
fn probe_reaches_adjacent_cell_with_definitions_no_step_detours(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut path_cache = PlaneReplacementReachabilityPathCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();
    let mut no_step_cache = DefinitionNoDetourReachabilityCache::default();
    let mut direct_probe_reachability_cache = Vec::new();
    probe_reaches_adjacent_cell_with_definitions_no_step_detours_with_caches(
        start,
        end,
        host_support,
        polygons,
        start_definitions,
        end_definitions,
        &mut affine_cache,
        &mut path_cache,
        &mut step_cache,
        &mut no_step_cache,
        &mut direct_probe_reachability_cache,
        None,
    )
}

pub(super) fn probe_reaches_adjacent_cell_with_definitions_no_step_detours_with_caches(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    affine_cache: &mut PlaneReplacementAffineCache,
    path_cache: &mut PlaneReplacementReachabilityPathCache,
    step_cache: &mut PlaneReplacementReachabilityStepCache,
    no_step_cache: &mut DefinitionNoDetourReachabilityCache,
    direct_probe_reachability_cache: &mut Vec<DirectProbeReachabilityCacheEntry>,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<bool> {
    let start_family = endpoint_definition_family(start, start_definitions)?;
    let end_family = endpoint_definition_family(end, end_definitions)?;
    let saw_definition_unknown = start_family.saw_unknown || end_family.saw_unknown;
    let result = cached_definition_no_detour_reachability_with(
        no_step_cache,
        start,
        end,
        &start_family.definitions,
        &end_family.definitions,
        || {
            let direct_unknown = match cached_direct_probe_reachability_with(
                direct_probe_reachability_cache,
                start,
                end,
                host_support,
                polygons,
                || probe_reaches_adjacent_cell(start, end, host_support, polygons),
            ) {
                Ok(true) => return Ok(true),
                Ok(false) => false,
                Err(HypermeshError::UnknownClassification) => true,
                Err(err) => return Err(err),
            };

            let ordered_pairs = ordered_definition_pairs_by_no_step_precheck_with(
                &start_family.definitions,
                &end_family.definitions,
                host_support,
                polygons,
                affine_cache,
                direct_probe_reachability_cache,
            )?;
            let mut saw_unknown = direct_unknown;
            for (start_index, end_index) in ordered_pairs {
                match plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches(
                    &start_family.definitions[start_index],
                    &end_family.definitions[end_index],
                    host_support,
                    polygons,
                    affine_cache,
                    path_cache,
                    step_cache,
                    trace_bounds,
                ) {
                    Ok(true) => return Ok(true),
                    Ok(false) => {}
                    Err(HypermeshError::UnknownClassification) => saw_unknown = true,
                    Err(err) => return Err(err),
                }
            }

            if saw_unknown {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(false)
            }
        },
    );
    match result {
        Ok(false) if saw_definition_unknown => Err(HypermeshError::UnknownClassification),
        result => result,
    }
}

fn ordered_definition_pairs_by_no_step_precheck_with(
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    affine_cache: &mut PlaneReplacementAffineCache,
    direct_probe_reachability_cache: &mut Vec<DirectProbeReachabilityCacheEntry>,
) -> HypermeshResult<Vec<(usize, usize)>> {
    let mut scored = Vec::with_capacity(start_definitions.len() * end_definitions.len());
    for (start_index, start_definition) in start_definitions.iter().enumerate() {
        for (end_index, end_definition) in end_definitions.iter().enumerate() {
            scored.push((
                best_plane_replacement_no_step_precheck_key(
                    start_definition,
                    end_definition,
                    host_support,
                    polygons,
                    affine_cache,
                    direct_probe_reachability_cache,
                )?,
                start_index,
                end_index,
            ));
        }
    }
    scored.sort_unstable();
    Ok(scored
        .into_iter()
        .map(|(_, start_index, end_index)| (start_index, end_index))
        .collect())
}

fn best_plane_replacement_no_step_precheck_key(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    affine_cache: &mut PlaneReplacementAffineCache,
    direct_probe_reachability_cache: &mut Vec<DirectProbeReachabilityCacheEntry>,
) -> HypermeshResult<[u8; 3]> {
    let mut best = [u8::MAX; 3];
    for ordering in AXIS_ORDERINGS {
        let key = ordering_no_step_precheck_key(
            &ordering,
            start_planes,
            end_planes,
            affine_cache,
            &mut |current, next, _current_definitions, _next_definitions| {
                cached_direct_probe_reachability_with(
                    direct_probe_reachability_cache,
                    current,
                    next,
                    host_support,
                    polygons,
                    || probe_reaches_adjacent_cell(current, next, host_support, polygons),
                )
            },
        )?;
        if key < best {
            best = key;
        }
    }
    Ok(best)
}

pub(super) fn probe_reaches_adjacent_cell_with_definition_search(
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    mut direct_reaches: impl FnMut() -> HypermeshResult<bool>,
    mut replacement_reaches: impl FnMut(&[Plane; 3], &[Plane; 3]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    let direct_unknown = match direct_reaches() {
        Ok(true) => return Ok(true),
        Ok(false) => false,
        Err(HypermeshError::UnknownClassification) => true,
        Err(err) => return Err(err),
    };

    let start_family = endpoint_definition_family(start, start_definitions)?;
    let end_family = endpoint_definition_family(end, end_definitions)?;

    match definition_pair_reachability_backtracking_unknown(
        &start_family.definitions,
        &end_family.definitions,
        |start_definition, end_definition| replacement_reaches(start_definition, end_definition),
    ) {
        Ok(true) => Ok(true),
        Ok(false) => {
            if direct_unknown || start_family.saw_unknown || end_family.saw_unknown {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(false)
            }
        }
        Err(HypermeshError::UnknownClassification) => Err(HypermeshError::UnknownClassification),
        Err(err) => Err(err),
    }
}

struct DefinitionSearchPrecheckPlan {
    start_definitions: Vec<[Plane; 3]>,
    end_definitions: Vec<[Plane; 3]>,
    ordered_pairs: Vec<(usize, usize)>,
    unknown_if_no_match: bool,
}

enum DefinitionSearchPrecheckOutcome {
    Reaches,
    Search(DefinitionSearchPrecheckPlan),
}

fn definition_search_precheck_plan(
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    direct_unknown: bool,
    mut precheck_reaches: impl FnMut(&[Plane; 3], &[Plane; 3]) -> HypermeshResult<bool>,
) -> HypermeshResult<DefinitionSearchPrecheckOutcome> {
    let start_family = endpoint_definition_family(start, start_definitions)?;
    let end_family = endpoint_definition_family(end, end_definitions)?;

    let mut ordered_pairs = Vec::new();
    let mut saw_unknown = direct_unknown || start_family.saw_unknown || end_family.saw_unknown;

    for (start_index, start_definition) in start_family.definitions.iter().enumerate() {
        for (end_index, end_definition) in end_family.definitions.iter().enumerate() {
            match precheck_reaches(start_definition, end_definition) {
                Ok(true) => return Ok(DefinitionSearchPrecheckOutcome::Reaches),
                Ok(false) => ordered_pairs.push((1usize, start_index, end_index)),
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    ordered_pairs.push((0usize, start_index, end_index));
                }
                Err(err) => return Err(err),
            }
        }
    }

    ordered_pairs.sort_unstable();

    Ok(DefinitionSearchPrecheckOutcome::Search(
        DefinitionSearchPrecheckPlan {
            start_definitions: start_family.definitions,
            end_definitions: end_family.definitions,
            ordered_pairs: ordered_pairs
                .into_iter()
                .map(|(_, start_index, end_index)| (start_index, end_index))
                .collect(),
            unknown_if_no_match: saw_unknown,
        },
    ))
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_with_definition_search_preferring_precheck(
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    mut direct_reaches: impl FnMut() -> HypermeshResult<bool>,
    mut precheck_reaches: impl FnMut(&[Plane; 3], &[Plane; 3]) -> HypermeshResult<bool>,
    mut replacement_reaches: impl FnMut(&[Plane; 3], &[Plane; 3]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    let direct_unknown = match direct_reaches() {
        Ok(true) => return Ok(true),
        Ok(false) => false,
        Err(HypermeshError::UnknownClassification) => true,
        Err(err) => return Err(err),
    };

    match definition_search_precheck_plan(
        start,
        end,
        start_definitions,
        end_definitions,
        direct_unknown,
        |start_definition, end_definition| precheck_reaches(start_definition, end_definition),
    )? {
        DefinitionSearchPrecheckOutcome::Reaches => Ok(true),
        DefinitionSearchPrecheckOutcome::Search(plan) => {
            let mut saw_unknown = plan.unknown_if_no_match;
            for (start_index, end_index) in plan.ordered_pairs {
                match replacement_reaches(
                    &plan.start_definitions[start_index],
                    &plan.end_definitions[end_index],
                ) {
                    Ok(true) => return Ok(true),
                    Ok(false) => {}
                    Err(HypermeshError::UnknownClassification) => saw_unknown = true,
                    Err(err) => return Err(err),
                }
            }

            if saw_unknown {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(false)
            }
        }
    }
}

pub(super) fn definition_pair_reachability_backtracking_unknown(
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    mut reaches: impl FnMut(&[Plane; 3], &[Plane; 3]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    let start_definitions = unique_definition_family(start_definitions);
    let end_definitions = unique_definition_family(end_definitions);

    for start_definition in &start_definitions {
        for end_definition in &end_definitions {
            match reaches(start_definition, end_definition) {
                Ok(true) => return Ok(true),
                Ok(false) => {}
                Err(HypermeshError::UnknownClassification) => saw_unknown = true,
                Err(err) => return Err(err),
            }
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    }
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    let mut no_detour_cache = DefinitionNoDetourReachabilityCache::default();
    let mut no_plane_replacement_cycle_guard_cache =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let mut no_plane_replacement_cache = DefinitionNoPlaneReplacementReachabilityCache::default();
    let mut halfspace_report_cache = Vec::new();
    let mut halfspace_seed_family_cache = Vec::new();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        &mut no_detour_cache,
        &mut no_plane_replacement_cycle_guard_cache,
        &mut no_plane_replacement_cache,
        &mut halfspace_report_cache,
        &mut halfspace_seed_family_cache,
        &mut detour_target_cache,
        &mut interior_box_axis_intervals,
        |start: &Point3,
         end: &Point3,
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            probe_reaches_adjacent_cell_with_definitions_no_step_detours(
                start,
                end,
                host_support,
                polygons,
                start_definitions,
                end_definitions,
            )
        },
        |start: &Point3, end: &Point3| interior_box_detour_targets(start, end, polygons),
    )
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    no_detour_cache: &mut DefinitionNoDetourReachabilityCache,
    no_plane_replacement_cycle_guard_cache: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    no_plane_replacement_cache: &mut DefinitionNoPlaneReplacementReachabilityCache,
    halfspace_report_cache: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_family_cache: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    reach_without_detours: impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for_query: impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut strict_aabb_target_families = StrictAabbTargetFamilyCache::default();
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with_mode(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        no_detour_cache,
        no_plane_replacement_cycle_guard_cache,
        no_plane_replacement_cache,
        halfspace_report_cache,
        halfspace_seed_family_cache,
        &mut strict_aabb_target_families,
        detour_target_cache,
        interior_box_axis_intervals,
        false,
        None,
        reach_without_detours,
        detours_for_query,
    )
}

fn probe_reaches_adjacent_cell_with_interior_box_detours_without_plane_replacement_from_definitions_with(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    no_detour_cache: &mut DefinitionNoDetourReachabilityCache,
    no_plane_replacement_cycle_guard_cache: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    no_plane_replacement_cache: &mut DefinitionNoPlaneReplacementReachabilityCache,
    halfspace_report_cache: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_family_cache: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    strict_aabb_target_families: &mut StrictAabbTargetFamilyCache,
    detour_target_cache: &mut DetourTargetFamilyCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    trace_bounds: Option<&Aabb>,
    reach_without_detours: impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for_query: impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with_mode(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        no_detour_cache,
        no_plane_replacement_cycle_guard_cache,
        no_plane_replacement_cache,
        halfspace_report_cache,
        halfspace_seed_family_cache,
        strict_aabb_target_families,
        detour_target_cache,
        interior_box_axis_intervals,
        true,
        trace_bounds,
        reach_without_detours,
        detours_for_query,
    )
}

fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with_mode(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    no_detour_cache: &mut DefinitionNoDetourReachabilityCache,
    no_plane_replacement_cycle_guard_cache: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    no_plane_replacement_cache: &mut DefinitionNoPlaneReplacementReachabilityCache,
    halfspace_report_cache: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_family_cache: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    strict_aabb_target_families: &mut StrictAabbTargetFamilyCache,
    detour_target_cache: &mut DetourTargetFamilyCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    progressive_interior_box_detours: bool,
    trace_bounds: Option<&Aabb>,
    mut reach_without_detours: impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    mut detours_for_query: impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut trace_without_detours =
        |start: &Point3,
         end: &Point3,
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            cached_definition_no_detour_reachability_with(
                &mut *no_detour_cache,
                start,
                end,
                start_definitions,
                end_definitions,
                || reach_without_detours(start, end, start_definitions, end_definitions),
            )
        };
    if let Some(existing) = cached_definition_no_plane_replacement_reachability_result(
        no_plane_replacement_cache,
        start,
        end,
        start_definitions,
        end_definitions,
    ) {
        return existing;
    }

    let result = if progressive_interior_box_detours {
        let mut surface_cache = Vec::new();
        let arrangement_planes = detour_arrangement_planes(polygons);
        let mut detour_batches = InteriorBoxDetourTargetBatchCache::default();
        probe_reaches_adjacent_cell_with_detour_batches_breadth_first_with_surface_query(
            start,
            end,
            start_definitions,
            end_definitions,
            &arrangement_planes,
            &mut surface_cache,
            &mut |point| {
                if !point_is_inside_optional_trace_bounds(point, trace_bounds)? {
                    return Ok(true);
                }
                point_lies_on_traced_surface(point, polygons)
            },
            &mut trace_without_detours,
            &mut |batch_start, batch_end, batch_index| {
                detour_batches.batch_for(
                    batch_start,
                    batch_end,
                    batch_index,
                    polygons,
                    &arrangement_planes,
                    trace_bounds,
                )
            },
        )
    } else {
        // The cycle-guard evaluator reads this cache. Do not expose the whole-query
        // UnknownClassification placeholder as if it were a completed exact-state result.
        let known_false_cache = &*no_plane_replacement_cache;
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_mode(
            start,
            end,
            polygons,
            &initial_visited_definition_points(start, start_definitions, end, end_definitions),
            start_definitions,
            end_definitions,
            progressive_interior_box_detours,
            no_plane_replacement_cycle_guard_cache,
            known_false_cache,
            halfspace_report_cache,
            halfspace_seed_family_cache,
            strict_aabb_target_families,
            interior_box_axis_intervals,
            &mut trace_without_detours,
            detour_target_cache,
            &mut detours_for_query,
        )
    };
    let cache_index = begin_definition_no_plane_replacement_reachability_result(
        no_plane_replacement_cache,
        start,
        end,
        start_definitions,
        end_definitions,
    );
    no_plane_replacement_cache.entries[cache_index].result = result.clone();
    result
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[VisitedDefinitionPoint],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    no_plane_replacement_cache: &DefinitionNoPlaneReplacementReachabilityCache,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for_query: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut no_plane_replacement_cycle_guard_cache =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let mut halfspace_report_cache = Vec::new();
    let mut halfspace_seed_family_cache = Vec::new();
    let mut strict_aabb_target_families = StrictAabbTargetFamilyCache::default();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_mode(
        start,
        end,
        polygons,
        visited_points,
        start_definitions,
        end_definitions,
        false,
        &mut no_plane_replacement_cycle_guard_cache,
        no_plane_replacement_cache,
        &mut halfspace_report_cache,
        &mut halfspace_seed_family_cache,
        &mut strict_aabb_target_families,
        &mut interior_box_axis_intervals,
        trace_without_detours,
        &mut detour_target_cache,
        detours_for_query,
    )
}

fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_mode(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[VisitedDefinitionPoint],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    progressive_interior_box_detours: bool,
    no_plane_replacement_cycle_guard_cache: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    no_plane_replacement_cache: &DefinitionNoPlaneReplacementReachabilityCache,
    halfspace_report_cache: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_family_cache: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    strict_aabb_target_families: &mut StrictAabbTargetFamilyCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    detours_for_query: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut surface_cache = Vec::new();
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query_mode(
        start,
        end,
        polygons,
        visited_points,
        start_definitions,
        end_definitions,
        progressive_interior_box_detours,
        no_plane_replacement_cycle_guard_cache,
        no_plane_replacement_cache,
        halfspace_report_cache,
        halfspace_seed_family_cache,
        strict_aabb_target_families,
        interior_box_axis_intervals,
        &mut surface_cache,
        &mut |point| point_lies_on_traced_surface(point, polygons),
        trace_without_detours,
        detour_target_cache,
        detours_for_query,
    )
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[VisitedDefinitionPoint],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    no_plane_replacement_cache: &DefinitionNoPlaneReplacementReachabilityCache,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for_query: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut no_plane_replacement_cycle_guard_cache =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let mut halfspace_report_cache = Vec::new();
    let mut halfspace_seed_family_cache = Vec::new();
    let mut strict_aabb_target_families = StrictAabbTargetFamilyCache::default();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query_mode(
        start,
        end,
        polygons,
        visited_points,
        start_definitions,
        end_definitions,
        false,
        &mut no_plane_replacement_cycle_guard_cache,
        no_plane_replacement_cache,
        &mut halfspace_report_cache,
        &mut halfspace_seed_family_cache,
        &mut strict_aabb_target_families,
        &mut interior_box_axis_intervals,
        surface_cache,
        surface_query,
        trace_without_detours,
        &mut detour_target_cache,
        detours_for_query,
    )
}

pub(super) fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query_mode(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[VisitedDefinitionPoint],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    progressive_interior_box_detours: bool,
    no_plane_replacement_cycle_guard_cache: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    no_plane_replacement_cache: &DefinitionNoPlaneReplacementReachabilityCache,
    halfspace_report_cache: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_family_cache: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    strict_aabb_target_families: &mut StrictAabbTargetFamilyCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    detours_for_query: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let initial_visited_points =
        initial_visited_definition_points(start, start_definitions, end, end_definitions);
    let normalized_initial_visited_points = normalized_cycle_guard_visited_points(
        start,
        end,
        start_definitions,
        end_definitions,
        &initial_visited_points,
    );
    let normalized_visited_points = normalized_cycle_guard_visited_points(
        start,
        end,
        start_definitions,
        end_definitions,
        visited_points,
    );
    if let Some(existing) = cached_definition_no_plane_replacement_reachability_result(
        no_plane_replacement_cache,
        start,
        end,
        start_definitions,
        end_definitions,
    ) {
        if visited_definition_points_match_as_sets(
            &normalized_visited_points,
            &normalized_initial_visited_points,
        ) {
            return existing;
        }
        if matches!(existing, Ok(false)) {
            return Ok(false);
        }
    }
    if let Some(existing) = cached_definition_no_plane_replacement_cycle_guard_result(
        no_plane_replacement_cycle_guard_cache,
        start,
        end,
        start_definitions,
        end_definitions,
        visited_points,
    ) {
        return existing;
    }
    let cache_index = begin_definition_no_plane_replacement_cycle_guard_result(
        no_plane_replacement_cycle_guard_cache,
        start,
        end,
        start_definitions,
        end_definitions,
        visited_points,
    );
    let mut saw_unknown = false;
    let direct_result = trace_without_detours(start, end, start_definitions, end_definitions);
    let result = match direct_result {
        Ok(true) => Ok(true),
        Ok(false) | Err(HypermeshError::UnknownClassification) => {
            if matches!(direct_result, Err(HypermeshError::UnknownClassification)) {
                saw_unknown = true;
            }
            if progressive_interior_box_detours
                && cached_detour_target_family(detour_target_cache, start, end, None).is_none()
            {
                let outcome =
                    probe_reaches_adjacent_cell_via_interior_box_detours_without_plane_replacement_progressively_with_surface_query_outcome(
                    start,
                    end,
                    polygons,
                    visited_points,
                    start_definitions,
                    end_definitions,
                    progressive_interior_box_detours,
                    no_plane_replacement_cycle_guard_cache,
                    no_plane_replacement_cache,
                    halfspace_report_cache,
                    halfspace_seed_family_cache,
                    strict_aabb_target_families,
                    interior_box_axis_intervals,
                    surface_cache,
                    surface_query,
                    trace_without_detours,
                    detour_target_cache,
                    detours_for_query,
                );
                if let Some(targets) = outcome.exhausted_targets.clone() {
                    detour_target_cache
                        .entries
                        .push(DetourTargetFamilyCacheEntry {
                            start: start.clone(),
                            end: end.clone(),
                            trace_bounds: None,
                            targets,
                        });
                    let index = detour_target_cache.entries.len() - 1;
                    push_detour_target_family_bucket_entry(
                        &mut detour_target_cache.buckets,
                        start,
                        end,
                        index,
                    );
                }
                match outcome.result {
                    Ok(true) => Ok(true),
                    Ok(false) => {
                        if saw_unknown {
                            Err(HypermeshError::UnknownClassification)
                        } else {
                            Ok(false)
                        }
                    }
                    Err(HypermeshError::UnknownClassification) => {
                        Err(HypermeshError::UnknownClassification)
                    }
                    Err(err) => return Err(err),
                }
            } else {
                let mut found = false;
                for detour in
                    cached_detour_target_family_with(detour_target_cache, start, end, None, || {
                        detours_for_query(start, end)
                    })?
                {
                    if evaluate_probe_detour_target_without_plane_replacement_with_surface_query(
                        &detour,
                        start,
                        end,
                        polygons,
                        visited_points,
                        start_definitions,
                        end_definitions,
                        progressive_interior_box_detours,
                        no_plane_replacement_cycle_guard_cache,
                        no_plane_replacement_cache,
                        halfspace_report_cache,
                        halfspace_seed_family_cache,
                        strict_aabb_target_families,
                        interior_box_axis_intervals,
                        surface_cache,
                        surface_query,
                        trace_without_detours,
                        detour_target_cache,
                        detours_for_query,
                        &mut saw_unknown,
                    )? {
                        found = true;
                        break;
                    }
                }
                if found {
                    Ok(true)
                } else if saw_unknown {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(false)
                }
            }
        }
        Err(err) => return Err(err),
    };
    no_plane_replacement_cycle_guard_cache.entries[cache_index].result = result.clone();
    result
}

pub(super) fn evaluate_probe_detour_target_without_plane_replacement_with_surface_query(
    detour: &DetourTarget,
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[VisitedDefinitionPoint],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    progressive_interior_box_detours: bool,
    no_plane_replacement_cycle_guard_cache: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    no_plane_replacement_cache: &DefinitionNoPlaneReplacementReachabilityCache,
    halfspace_report_cache: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_family_cache: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    strict_aabb_target_families: &mut StrictAabbTargetFamilyCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    detours_for_query: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
    saw_unknown: &mut bool,
) -> HypermeshResult<bool> {
    let start_definition_transition = detour.point == *start
        && !definition_families_match_as_sets(&detour.definitions, start_definitions);
    let end_definition_transition = detour.point == *end
        && !definition_families_match_as_sets(&detour.definitions, end_definitions);
    let zero_length_definition_transition =
        start_definition_transition || end_definition_transition;
    let already_visited =
        visited_definition_family_contains(visited_points, &detour.point, &detour.definitions);
    let on_surface = if already_visited || zero_length_definition_transition {
        false
    } else {
        match cached_surface_query_with(surface_cache, &detour.point, || {
            surface_query(&detour.point)
        }) {
            Ok(on_surface) => on_surface,
            Err(HypermeshError::UnknownClassification) => {
                *saw_unknown = true;
                return Ok(false);
            }
            Err(err) => return Err(err),
        }
    };
    if already_visited || on_surface {
        if detour.uncertified_definition_fallback {
            *saw_unknown = true;
        }
        return Ok(false);
    }

    let mut next_visited_points = visited_points.to_vec();
    if !visited_definition_family_contains(&next_visited_points, &detour.point, &detour.definitions)
    {
        next_visited_points.push(VisitedDefinitionPoint {
            point: detour.point.clone(),
            definitions: detour.definitions.clone(),
        });
    }
    let first_leg_key = direct_precheck_rank(trace_without_detours(
        start,
        &detour.point,
        start_definitions,
        &detour.definitions,
    ))?;
    let second_leg_key = direct_precheck_rank(trace_without_detours(
        &detour.point,
        end,
        &detour.definitions,
        end_definitions,
    ))?;
    let prefer_second_leg = second_leg_key < first_leg_key;
    let mut evaluate_leg = |leg_start: &Point3,
                            leg_end: &Point3,
                            leg_start_definitions: &[[Plane; 3]],
                            leg_end_definitions: &[[Plane; 3]],
                            definition_transition: bool,
                            direct_key: u8|
     -> HypermeshResult<Option<bool>> {
        if definition_transition || direct_key == 0 {
            return match direct_key {
                0 => Ok(Some(true)),
                1 => {
                    *saw_unknown = true;
                    Ok(None)
                }
                2 => Ok(Some(false)),
                _ => unreachable!("unexpected direct precheck rank"),
            };
        }

        match probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query_mode(
            leg_start,
            leg_end,
            polygons,
            &next_visited_points,
            leg_start_definitions,
            leg_end_definitions,
            progressive_interior_box_detours,
            no_plane_replacement_cycle_guard_cache,
            no_plane_replacement_cache,
            halfspace_report_cache,
            halfspace_seed_family_cache,
            strict_aabb_target_families,
            interior_box_axis_intervals,
            surface_cache,
            surface_query,
            trace_without_detours,
            detour_target_cache,
            detours_for_query,
        ) {
            Ok(result) => Ok(Some(result)),
            Err(HypermeshError::UnknownClassification) => {
                *saw_unknown = true;
                Ok(None)
            }
            Err(err) => Err(err),
        }
    };

    let first_result = if prefer_second_leg {
        evaluate_leg(
            &detour.point,
            end,
            &detour.definitions,
            end_definitions,
            end_definition_transition,
            second_leg_key,
        )?
    } else {
        evaluate_leg(
            start,
            &detour.point,
            start_definitions,
            &detour.definitions,
            start_definition_transition,
            first_leg_key,
        )?
    };
    if first_result != Some(true) {
        if detour.uncertified_definition_fallback {
            *saw_unknown = true;
        }
        return Ok(false);
    }

    let second_result = if prefer_second_leg {
        evaluate_leg(
            start,
            &detour.point,
            start_definitions,
            &detour.definitions,
            start_definition_transition,
            first_leg_key,
        )?
    } else {
        evaluate_leg(
            &detour.point,
            end,
            &detour.definitions,
            end_definitions,
            end_definition_transition,
            second_leg_key,
        )?
    };
    match second_result {
        Some(true) => Ok(true),
        Some(false) | None => {
            if detour.uncertified_definition_fallback {
                *saw_unknown = true;
            }
            Ok(false)
        }
    }
}

struct ProgressiveNoPlaneDetourSearchOutcome {
    result: HypermeshResult<bool>,
    exhausted_targets: Option<HypermeshResult<Vec<DetourTarget>>>,
}

#[cfg(test)]
#[allow(dead_code)]
fn probe_reaches_adjacent_cell_via_interior_box_detours_without_plane_replacement_progressively_with_surface_query(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[VisitedDefinitionPoint],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    progressive_interior_box_detours: bool,
    no_plane_replacement_cycle_guard_cache: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    no_plane_replacement_cache: &DefinitionNoPlaneReplacementReachabilityCache,
    halfspace_report_cache: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_family_cache: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    strict_aabb_target_families: &mut StrictAabbTargetFamilyCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    detours_for_query: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    probe_reaches_adjacent_cell_via_interior_box_detours_without_plane_replacement_progressively_with_surface_query_outcome(
        start,
        end,
        polygons,
        visited_points,
        start_definitions,
        end_definitions,
        progressive_interior_box_detours,
        no_plane_replacement_cycle_guard_cache,
        no_plane_replacement_cache,
        halfspace_report_cache,
        halfspace_seed_family_cache,
        strict_aabb_target_families,
        interior_box_axis_intervals,
        surface_cache,
        surface_query,
        trace_without_detours,
        detour_target_cache,
        detours_for_query,
    )
    .result
}

fn probe_reaches_adjacent_cell_via_interior_box_detours_without_plane_replacement_progressively_with_surface_query_outcome(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[VisitedDefinitionPoint],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    progressive_interior_box_detours: bool,
    no_plane_replacement_cycle_guard_cache: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    no_plane_replacement_cache: &DefinitionNoPlaneReplacementReachabilityCache,
    halfspace_report_cache: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_family_cache: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    strict_aabb_target_families: &mut StrictAabbTargetFamilyCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    detours_for_query: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> ProgressiveNoPlaneDetourSearchOutcome {
    let (intervals, mut saw_unknown) = match cached_interior_box_axis_intervals_with_surface_queries(
        interior_box_axis_intervals,
        start,
        end,
        || {
            interior_box_axis_intervals_with_surface_queries(
                start,
                end,
                polygons,
                &mut |edge_start, edge_end, polygon, axis| {
                    let start_class = classify_point(edge_start, &polygon.support)?;
                    let end_class = classify_point(edge_end, &polygon.support)?;
                    if start_class == Classification::On {
                        return Ok(Some(edge_start.clone()));
                    }
                    if end_class == Classification::On {
                        return Ok(Some(edge_end.clone()));
                    }
                    segment_plane_crossing(edge_start, edge_end, &polygon.support).and_then(
                        |crossing| {
                            if let Some(crossing) = crossing {
                                if !point_strictly_between_axis(
                                    &crossing, edge_start, edge_end, axis,
                                )? {
                                    return Ok(None);
                                }
                                Ok(Some(crossing))
                            } else {
                                Ok(None)
                            }
                        },
                    )
                },
                &mut |crossing, polygon| classify_point_in_polygon(crossing, polygon),
            )
        },
    ) {
        Ok(found) => found,
        Err(err) => {
            return ProgressiveNoPlaneDetourSearchOutcome {
                result: Err(err),
                exhausted_targets: None,
            };
        }
    };
    let mut target_family_saw_unknown = saw_unknown;
    let mut exhausted_targets = Vec::new();
    for x in &intervals[0] {
        for y in &intervals[1] {
            for z in &intervals[2] {
                let bounds = match aabb_from_axis_intervals([x, y, z]) {
                    Ok(bounds) => bounds,
                    Err(err) => {
                        return ProgressiveNoPlaneDetourSearchOutcome {
                            result: Err(err),
                            exhausted_targets: None,
                        };
                    }
                };
                let trace_without_detours_cell =
                    std::cell::RefCell::new(&mut *trace_without_detours);
                let halfspace_report_cache_cell =
                    std::cell::RefCell::new(&mut *halfspace_report_cache);
                let halfspace_seed_family_cache_cell =
                    std::cell::RefCell::new(&mut *halfspace_seed_family_cache);
                let interior_box_axis_intervals_cell =
                    std::cell::RefCell::new(&mut *interior_box_axis_intervals);
                let result = if let Some(families) =
                    cached_strict_aabb_target_families(strict_aabb_target_families, &bounds)
                {
                    let families = match families {
                        Ok(families) => families,
                        Err(err) => {
                            return ProgressiveNoPlaneDetourSearchOutcome {
                                result: Err(err),
                                exhausted_targets: None,
                            };
                        }
                    };
                    target_family_saw_unknown |= families.saw_unknown;
                    for target in families
                        .direct_targets
                        .iter()
                        .chain(families.shifted_targets.iter())
                    {
                        push_unique_detour_target(&mut exhausted_targets, target.clone());
                    }
                    evaluate_strict_aabb_target_families_with_direct_ranking(
                        families,
                        &mut |detour| {
                            detour_target_no_plane_refined_rank_with_surface_queries(
                                detour,
                                start,
                                end,
                                polygons,
                                &mut |edge_start, edge_end, polygon, axis| {
                                    let start_class = classify_point(edge_start, &polygon.support)?;
                                    let end_class = classify_point(edge_end, &polygon.support)?;
                                    if start_class == Classification::On {
                                        return Ok(Some(edge_start.clone()));
                                    }
                                    if end_class == Classification::On {
                                        return Ok(Some(edge_end.clone()));
                                    }
                                    segment_plane_crossing(edge_start, edge_end, &polygon.support)
                                        .and_then(|crossing| {
                                            if let Some(crossing) = crossing {
                                                if !point_strictly_between_axis(
                                                    &crossing, edge_start, edge_end, axis,
                                                )? {
                                                    return Ok(None);
                                                }
                                                Ok(Some(crossing))
                                            } else {
                                                Ok(None)
                                            }
                                        })
                                },
                                &mut |crossing, polygon| {
                                    classify_point_in_polygon(crossing, polygon)
                                },
                                start_definitions,
                                end_definitions,
                                &mut **interior_box_axis_intervals_cell.borrow_mut(),
                                &mut **trace_without_detours_cell.borrow_mut(),
                            )
                        },
                        &mut |detour| {
                            evaluate_probe_detour_target_without_plane_replacement_with_surface_query(
                                &detour,
                                start,
                                end,
                                polygons,
                                visited_points,
                                start_definitions,
                                end_definitions,
                                progressive_interior_box_detours,
                                no_plane_replacement_cycle_guard_cache,
                                no_plane_replacement_cache,
                                &mut **halfspace_report_cache_cell.borrow_mut(),
                                &mut **halfspace_seed_family_cache_cell.borrow_mut(),
                                strict_aabb_target_families,
                                &mut **interior_box_axis_intervals_cell.borrow_mut(),
                                surface_cache,
                                surface_query,
                                &mut **trace_without_detours_cell.borrow_mut(),
                                detour_target_cache,
                                detours_for_query,
                                &mut saw_unknown,
                            )
                        },
                    )
                } else {
                    let outcome =
                        search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking_outcome(
                        &bounds,
                        |bounds, halfspaces, report, local_unknown| {
                            let report = match report {
                                Some(report) => Some(report.clone()),
                                None => cached_optional_halfspace_feasibility_report_with(
                                    &mut **halfspace_report_cache_cell.borrow_mut(),
                                    halfspaces,
                                    local_unknown,
                                )?,
                            };
                            cached_halfspace_cell_seed_families_from_optional_report_with(
                                &mut **halfspace_seed_family_cache_cell.borrow_mut(),
                                bounds,
                                halfspaces,
                                report.as_ref(),
                                local_unknown,
                            )
                        },
                        &mut |detour| {
                            detour_target_no_plane_refined_rank_with_surface_queries(
                                detour,
                                start,
                                end,
                                polygons,
                                &mut |edge_start, edge_end, polygon, axis| {
                                    let start_class = classify_point(edge_start, &polygon.support)?;
                                    let end_class = classify_point(edge_end, &polygon.support)?;
                                    if start_class == Classification::On {
                                        return Ok(Some(edge_start.clone()));
                                    }
                                    if end_class == Classification::On {
                                        return Ok(Some(edge_end.clone()));
                                    }
                                    segment_plane_crossing(edge_start, edge_end, &polygon.support)
                                        .and_then(|crossing| {
                                            if let Some(crossing) = crossing {
                                                if !point_strictly_between_axis(
                                                    &crossing, edge_start, edge_end, axis,
                                                )? {
                                                    return Ok(None);
                                                }
                                                Ok(Some(crossing))
                                            } else {
                                                Ok(None)
                                            }
                                        })
                                },
                                &mut |crossing, polygon| {
                                    classify_point_in_polygon(crossing, polygon)
                                },
                                start_definitions,
                                end_definitions,
                                &mut **interior_box_axis_intervals_cell.borrow_mut(),
                                &mut **trace_without_detours_cell.borrow_mut(),
                            )
                        },
                        &mut |detour| {
                            evaluate_probe_detour_target_without_plane_replacement_with_surface_query(
                                &detour,
                                start,
                                end,
                                polygons,
                                visited_points,
                                start_definitions,
                                end_definitions,
                                progressive_interior_box_detours,
                                no_plane_replacement_cycle_guard_cache,
                                no_plane_replacement_cache,
                                &mut **halfspace_report_cache_cell.borrow_mut(),
                                &mut **halfspace_seed_family_cache_cell.borrow_mut(),
                                strict_aabb_target_families,
                                &mut **interior_box_axis_intervals_cell.borrow_mut(),
                                surface_cache,
                                surface_query,
                                &mut **trace_without_detours_cell.borrow_mut(),
                                detour_target_cache,
                                detours_for_query,
                                &mut saw_unknown,
                            )
                        },
                    );
                    if let Some(families) = outcome.exhausted_families.clone() {
                        target_family_saw_unknown |= families.saw_unknown;
                        for target in families
                            .direct_targets
                            .iter()
                            .chain(families.shifted_targets.iter())
                        {
                            push_unique_detour_target(&mut exhausted_targets, target.clone());
                        }
                        strict_aabb_target_families.entries.push(
                            StrictAabbTargetFamilyCacheEntry {
                                bounds: bounds.clone(),
                                families: Ok(families),
                            },
                        );
                        let index = strict_aabb_target_families.entries.len() - 1;
                        push_strict_aabb_target_family_bucket_entry(
                            &mut strict_aabb_target_families.buckets,
                            &bounds,
                            index,
                        );
                    }
                    outcome.result
                };
                match result {
                    Ok(true) => {
                        return ProgressiveNoPlaneDetourSearchOutcome {
                            result: Ok(true),
                            exhausted_targets: None,
                        };
                    }
                    Ok(false) => {}
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                        continue;
                    }
                    Err(err) => {
                        return ProgressiveNoPlaneDetourSearchOutcome {
                            result: Err(err),
                            exhausted_targets: None,
                        };
                    }
                }
            }
        }
    }

    ProgressiveNoPlaneDetourSearchOutcome {
        result: if saw_unknown {
            Err(HypermeshError::UnknownClassification)
        } else {
            Ok(false)
        },
        exhausted_targets: Some(detour_target_family_result_from_targets(
            exhausted_targets,
            target_family_saw_unknown,
        )),
    }
}

fn detour_target_no_plane_refined_rank_with_surface_queries(
    detour: &DetourTarget,
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    interval_crossing: &mut impl FnMut(
        &Point3,
        &Point3,
        &ConvexPolygon,
        usize,
    ) -> HypermeshResult<Option<Point3>>,
    classify_crossing: &mut impl FnMut(&Point3, &ConvexPolygon) -> HypermeshResult<PolygonPointLocation>,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
) -> HypermeshResult<(u8, u8, usize, usize, usize, usize, usize, usize)> {
    let first_key = direct_precheck_rank(trace_without_detours(
        start,
        &detour.point,
        start_definitions,
        &detour.definitions,
    ))?;
    let second_key = direct_precheck_rank(trace_without_detours(
        &detour.point,
        end,
        &detour.definitions,
        end_definitions,
    ))?;
    let first_counts = if first_key == 0 {
        (0, 0, 0)
    } else {
        interior_box_axis_interval_counts_with_surface_queries(
            interior_box_axis_intervals,
            start,
            &detour.point,
            polygons,
            interval_crossing,
            classify_crossing,
        )?
    };
    let second_counts = if second_key == 0 {
        (0, 0, 0)
    } else {
        interior_box_axis_interval_counts_with_surface_queries(
            interior_box_axis_intervals,
            &detour.point,
            end,
            polygons,
            interval_crossing,
            classify_crossing,
        )?
    };
    Ok((
        first_key,
        second_key,
        first_counts.0,
        first_counts.1,
        first_counts.2,
        second_counts.0,
        second_counts.1,
        second_counts.2,
    ))
}

fn interior_box_axis_interval_counts_with_surface_queries(
    cache: &mut InteriorBoxAxisIntervalsCache,
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    interval_crossing: &mut impl FnMut(
        &Point3,
        &Point3,
        &ConvexPolygon,
        usize,
    ) -> HypermeshResult<Option<Point3>>,
    classify_crossing: &mut impl FnMut(&Point3, &ConvexPolygon) -> HypermeshResult<PolygonPointLocation>,
) -> HypermeshResult<(usize, usize, usize)> {
    let (intervals, _) =
        cached_interior_box_axis_intervals_with_surface_queries(cache, start, end, || {
            interior_box_axis_intervals_with_surface_queries(
                start,
                end,
                polygons,
                interval_crossing,
                classify_crossing,
            )
        })?;
    Ok((intervals[0].len(), intervals[1].len(), intervals[2].len()))
}

fn direct_precheck_rank(result: HypermeshResult<bool>) -> HypermeshResult<u8> {
    match result {
        Ok(true) => Ok(0),
        Err(HypermeshError::UnknownClassification) => Ok(1),
        Ok(false) => Ok(2),
        Err(err) => Err(err),
    }
}

#[cfg(test)]
pub(super) fn probe_reaches_adjacent_cell_with_detours_breadth_first_with_surface_query(
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    arrangement_planes: &[Plane],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    probe_reaches_adjacent_cell_with_detour_batches_breadth_first_with_surface_query(
        start,
        end,
        start_definitions,
        end_definitions,
        arrangement_planes,
        surface_cache,
        surface_query,
        trace_without_detours,
        &mut |batch_start, batch_end, batch_index| {
            if batch_index == 0 {
                detours_for(batch_start, batch_end).map(Some)
            } else {
                Ok(None)
            }
        },
    )
}

pub(super) fn probe_reaches_adjacent_cell_with_detour_batches_breadth_first_with_surface_query(
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    arrangement_planes: &[Plane],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detour_batch_for: &mut impl FnMut(
        &Point3,
        &Point3,
        usize,
    ) -> HypermeshResult<Option<Vec<DetourTarget>>>,
) -> HypermeshResult<bool> {
    let start_target = DetourTarget {
        point: start.clone(),
        definitions: start_definitions.to_vec(),
        uncertified_definition_fallback: false,
    };
    let end_target = DetourTarget {
        point: end.clone(),
        definitions: end_definitions.to_vec(),
        uncertified_definition_fallback: false,
    };
    let initial_path = vec![start_target, end_target];
    let mut queue = std::collections::VecDeque::from([(initial_path.clone(), 0usize)]);
    let mut seen_paths = vec![initial_path];
    let mut seen_cells = Vec::<DetourArrangementCellState>::new();
    let mut saw_unknown = false;

    while let Some((path, batch_index)) = queue.pop_front() {
        let mut unresolved_edge = None;
        for index in 0..path.len() - 1 {
            match trace_without_detours(
                &path[index].point,
                &path[index + 1].point,
                &path[index].definitions,
                &path[index + 1].definitions,
            ) {
                Ok(true) => {}
                Ok(false) => {
                    unresolved_edge = Some(index);
                    break;
                }
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    unresolved_edge = Some(index);
                    break;
                }
                Err(err) => return Err(err),
            }
        }

        let Some(edge_index) = unresolved_edge else {
            return Ok(true);
        };
        let edge_start = &path[edge_index];
        let edge_end = &path[edge_index + 1];
        let mut detours = match detour_batch_for(&edge_start.point, &edge_end.point, batch_index) {
            Ok(Some(detours)) => detours,
            Ok(None) => continue,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        detours.sort_by_key(|detour| detour.uncertified_definition_fallback);
        let mut next_paths = Vec::new();
        for detour in detours {
            let definition_transition = (detour.point == edge_start.point
                && !definition_families_match_as_sets(
                    &detour.definitions,
                    &edge_start.definitions,
                ))
                || (detour.point == edge_end.point
                    && !definition_families_match_as_sets(
                        &detour.definitions,
                        &edge_end.definitions,
                    ));
            let exact_state_visited = path.iter().any(|visited| {
                visited.point == detour.point
                    && definition_families_match_as_sets(&visited.definitions, &detour.definitions)
            });
            if exact_state_visited {
                continue;
            }
            let detour_cell = if arrangement_planes.is_empty() {
                None
            } else {
                match detour_arrangement_cell(&detour.point, arrangement_planes) {
                    Ok(cell) => Some(cell),
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                }
            };
            if !definition_transition && let Some(detour_cell) = detour_cell.as_ref() {
                let mut revisits_cell = detour_arrangement_cell_state_is_dominated(
                    &seen_cells,
                    detour_cell,
                    detour.uncertified_definition_fallback,
                );
                if !revisits_cell {
                    for visited in &path {
                        match detour_arrangement_cell(&visited.point, arrangement_planes) {
                            Ok(cell) if cell == *detour_cell => {
                                revisits_cell = true;
                                break;
                            }
                            Ok(_) => {}
                            Err(HypermeshError::UnknownClassification) => {
                                saw_unknown = true;
                            }
                            Err(err) => return Err(err),
                        }
                    }
                }
                if revisits_cell {
                    continue;
                }
            }
            if !definition_transition {
                match cached_surface_query_with(surface_cache, &detour.point, || {
                    surface_query(&detour.point)
                }) {
                    Ok(true) => continue,
                    Ok(false) => {}
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                }
            }

            let mut next_path = path.clone();
            let detour_uncertified_definition_fallback = detour.uncertified_definition_fallback;
            next_path.insert(edge_index + 1, detour);
            if seen_paths.iter().any(|seen| *seen == next_path) {
                continue;
            }
            let mut complete = true;
            for index in 0..next_path.len() - 1 {
                match trace_without_detours(
                    &next_path[index].point,
                    &next_path[index + 1].point,
                    &next_path[index].definitions,
                    &next_path[index + 1].definitions,
                ) {
                    Ok(true) => {}
                    Ok(false) => {
                        complete = false;
                        break;
                    }
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                        complete = false;
                        break;
                    }
                    Err(err) => return Err(err),
                }
            }
            if complete {
                return Ok(true);
            }
            seen_paths.push(next_path.clone());
            if !definition_transition && let Some(cell) = detour_cell {
                record_detour_arrangement_cell_state(
                    &mut seen_cells,
                    cell,
                    detour_uncertified_definition_fallback,
                );
            }
            next_paths.push(next_path);
        }
        if let Some(first) = next_paths.first().cloned() {
            queue.push_back((first, 0));
        }
        queue.push_back((path, batch_index + 1));
        queue.extend(next_paths.into_iter().skip(1).map(|path| (path, 0)));
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    }
}

#[cfg(test)]
#[allow(dead_code)]
fn plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut path_cache = PlaneReplacementReachabilityPathCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();
    let mut no_nested_ordering_warmup_cache =
        PlaneReplacementNoNestedOrderingWarmupCache::default();
    let mut no_step_cache = DefinitionNoDetourReachabilityCache::default();
    let mut halfspace_reports = Vec::new();
    let mut halfspace_seed_families = Vec::new();
    let mut no_detour_cache = DefinitionNoDetourReachabilityCache::default();
    let mut no_plane_replacement_cycle_guard_cache =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let mut no_plane_replacement_cache = DefinitionNoPlaneReplacementReachabilityCache::default();
    let mut strict_aabb_target_families = StrictAabbTargetFamilyCache::default();
    let mut no_step_detour_target_cache = DetourTargetFamilyCache::default();
    let mut direct_probe_reachability_cache = Vec::new();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement_with_caches(
        start_planes,
        end_planes,
        host_support,
        polygons,
        &mut affine_cache,
        &mut path_cache,
        &mut step_cache,
        &mut no_nested_ordering_warmup_cache,
        &mut interior_box_axis_intervals,
        &mut no_step_cache,
        &mut halfspace_reports,
        &mut halfspace_seed_families,
        &mut no_detour_cache,
        &mut no_plane_replacement_cycle_guard_cache,
        &mut no_plane_replacement_cache,
        &mut strict_aabb_target_families,
        &mut no_step_detour_target_cache,
        &mut direct_probe_reachability_cache,
        None,
    )
}

fn plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement_with_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    affine_cache: &mut PlaneReplacementAffineCache,
    path_cache: &mut PlaneReplacementReachabilityPathCache,
    step_cache: &mut PlaneReplacementReachabilityStepCache,
    no_nested_ordering_warmup_cache: &mut PlaneReplacementNoNestedOrderingWarmupCache,
    interior_box_axis_intervals: &mut InteriorBoxAxisIntervalsCache,
    no_step_cache: &mut DefinitionNoDetourReachabilityCache,
    halfspace_reports: &mut Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_families: &mut Vec<HalfspaceSeedFamilyCacheEntry>,
    no_detour_cache: &mut DefinitionNoDetourReachabilityCache,
    no_plane_replacement_cycle_guard_cache: &mut DefinitionNoPlaneReplacementCycleGuardCache,
    no_plane_replacement_cache: &mut DefinitionNoPlaneReplacementReachabilityCache,
    strict_aabb_target_families: &mut StrictAabbTargetFamilyCache,
    no_step_detour_target_cache: &mut DetourTargetFamilyCache,
    direct_probe_reachability_cache: &mut Vec<DirectProbeReachabilityCacheEntry>,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<bool> {
    cached_plane_replacement_reachability_path_with(
        path_cache,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        start_planes,
        end_planes,
        || {
            let mut no_step_affine_cache = PlaneReplacementAffineCache::default();
            let mut no_step_path_cache = PlaneReplacementReachabilityPathCache::default();
            let mut no_step_step_cache = PlaneReplacementReachabilityStepCache::default();
            let ordered = cached_plane_replacement_no_nested_ordering_warmup_with(
                no_nested_ordering_warmup_cache,
                start_planes,
                end_planes,
                &mut no_step_affine_cache,
                &mut no_step_path_cache,
                &mut no_step_step_cache,
                |mut no_step_affine_cache, mut no_step_path_cache, mut no_step_step_cache| {
                    ordered_axis_orderings_by_no_step_precheck_with(
                        start_planes,
                        end_planes,
                        affine_cache,
                        |current, next, current_definitions, next_definitions| {
                            probe_reaches_adjacent_cell_with_definitions_no_step_detours_with_caches(
                                current,
                                next,
                                host_support,
                                polygons,
                                std::slice::from_ref(current_definitions),
                                std::slice::from_ref(next_definitions),
                                &mut no_step_affine_cache,
                                &mut no_step_path_cache,
                                &mut no_step_step_cache,
                                no_step_cache,
                                direct_probe_reachability_cache,
                                trace_bounds,
                            )
                        },
                    )
                },
            )?;
            plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
                &ordered,
                start_planes,
                end_planes,
                PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
                affine_cache,
                step_cache,
                trace_bounds,
                |current, next, current_definitions, next_definitions| {
                    probe_reaches_adjacent_cell_with_interior_box_detours_without_plane_replacement_from_definitions_with(
                        current,
                        next,
                        polygons,
                        current_definitions,
                        next_definitions,
                        no_detour_cache,
                        no_plane_replacement_cycle_guard_cache,
                        no_plane_replacement_cache,
                        halfspace_reports,
                        halfspace_seed_families,
                        strict_aabb_target_families,
                        no_step_detour_target_cache,
                        interior_box_axis_intervals,
                        trace_bounds,
                        |start: &Point3,
                         end: &Point3,
                         start_definitions: &[[Plane; 3]],
                         end_definitions: &[[Plane; 3]]| {
                            probe_reaches_adjacent_cell_with_definitions_no_step_detours_with_caches(
                                start,
                                end,
                                host_support,
                                polygons,
                                start_definitions,
                                end_definitions,
                                &mut no_step_affine_cache,
                                &mut no_step_path_cache,
                                &mut no_step_step_cache,
                                no_step_cache,
                                direct_probe_reachability_cache,
                                trace_bounds,
                            )
                        },
                        |start: &Point3, end: &Point3| {
                            interior_box_detour_targets(start, end, polygons)
                        },
                    )
                },
            )
        },
    )
}

#[cfg(test)]
#[allow(dead_code)]
pub(super) fn plane_replacement_path_reaches_adjacent_cell_without_step_detours(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut path_cache = PlaneReplacementReachabilityPathCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();
    plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches(
        start_planes,
        end_planes,
        host_support,
        polygons,
        &mut affine_cache,
        &mut path_cache,
        &mut step_cache,
        None,
    )
}

pub(super) fn plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    affine_cache: &mut PlaneReplacementAffineCache,
    path_cache: &mut PlaneReplacementReachabilityPathCache,
    step_cache: &mut PlaneReplacementReachabilityStepCache,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<bool> {
    cached_plane_replacement_reachability_path_with(
        path_cache,
        PlaneReplacementReachabilityStepMode::WithoutStepDetours,
        start_planes,
        end_planes,
        || {
            let ordered = ordered_axis_orderings_by_no_step_precheck_with(
                start_planes,
                end_planes,
                affine_cache,
                |current, next, _current_definitions, _next_definitions| {
                    probe_reaches_adjacent_cell(current, next, host_support, polygons)
                },
            )?;
            let result =
                plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
                    &ordered,
                    start_planes,
                    end_planes,
                    PlaneReplacementReachabilityStepMode::WithoutStepDetours,
                    affine_cache,
                    step_cache,
                    trace_bounds,
                    |current, next, _current_definitions, _next_definitions| {
                        probe_reaches_adjacent_cell(current, next, host_support, polygons)
                    },
                );
            match result {
                Err(HypermeshError::UnknownClassification) => {
                    // A shared polyline vertex supplies the incident sides that
                    // independent step checks lack at an endpoint contact.
                    plane_replacement_orderings_reach_adjacent_cell_as_polylines(
                        &ordered,
                        start_planes,
                        end_planes,
                        host_support,
                        polygons,
                        affine_cache,
                        trace_bounds,
                    )
                }
                result => result,
            }
        },
    )
}

fn plane_replacement_orderings_reach_adjacent_cell_as_polylines(
    orderings: &[[usize; 3]],
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    affine_cache: &mut PlaneReplacementAffineCache,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for ordering in orderings {
        let mut current_planes = start_planes.clone();
        let current_point =
            match cached_affine_from_planes_with(&mut *affine_cache, &current_planes, || {
                affine_from_planes(&current_planes)
            }) {
                Ok(point) if point_is_inside_optional_trace_bounds(&point, trace_bounds)? => point,
                Ok(_) => {
                    saw_unknown = true;
                    continue;
                }
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
        let mut points = vec![current_point];
        let mut valid = true;

        for plane_index in ordering.iter().copied() {
            let mut next_planes = current_planes.clone();
            next_planes[plane_index] = end_planes[plane_index].clone();
            if next_planes == current_planes {
                continue;
            }
            let next_point =
                match cached_affine_from_planes_with(&mut *affine_cache, &next_planes, || {
                    affine_from_planes(&next_planes)
                }) {
                    Ok(point) => {
                        if next_planes == *end_planes
                            && !point_is_inside_optional_trace_bounds(&point, trace_bounds)?
                        {
                            saw_unknown = true;
                            valid = false;
                            break;
                        }
                        adapt_plane_replacement_vertex_to_trace_bounds(
                            point,
                            next_planes.clone(),
                            trace_bounds,
                        )?
                        .0
                    }
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                        valid = false;
                        break;
                    }
                    Err(err) => return Err(err),
                };
            if points.last() != Some(&next_point) {
                points.push(next_point);
            }
            current_planes = next_planes;
        }
        if !valid {
            continue;
        }

        match probe_polyline_reaches_adjacent_cell(&points, host_support, polygons) {
            Ok(true) => return Ok(true),
            Ok(false) => {}
            Err(HypermeshError::UnknownClassification) => saw_unknown = true,
            Err(err) => return Err(err),
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    }
}

#[cfg(test)]
pub(super) fn plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    mode: PlaneReplacementReachabilityStepMode,
    affine_cache: &mut PlaneReplacementAffineCache,
    step_cache: &mut PlaneReplacementReachabilityStepCache,
    trace_step: impl FnMut(&Point3, &Point3, &[[Plane; 3]], &[[Plane; 3]]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
        &AXIS_ORDERINGS,
        start_planes,
        end_planes,
        mode,
        affine_cache,
        step_cache,
        None,
        trace_step,
    )
}

pub(super) fn plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
    orderings: &[[usize; 3]],
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    mode: PlaneReplacementReachabilityStepMode,
    affine_cache: &mut PlaneReplacementAffineCache,
    step_cache: &mut PlaneReplacementReachabilityStepCache,
    trace_bounds: Option<&Aabb>,
    mut trace_step: impl FnMut(&Point3, &Point3, &[[Plane; 3]], &[[Plane; 3]]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for ordering in orderings {
        let mut current_planes = start_planes.clone();
        let mut current_point =
            match cached_affine_from_planes_with(&mut *affine_cache, &current_planes, || {
                affine_from_planes(&current_planes)
            }) {
                Ok(point) if point_is_inside_optional_trace_bounds(&point, trace_bounds)? => point,
                Ok(_) => {
                    saw_unknown = true;
                    continue;
                }
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
        let mut current_trace_planes = current_planes.clone();
        let mut valid = true;

        for (_step_index, plane_index) in ordering.iter().copied().enumerate() {
            let mut next_planes = current_planes.clone();
            next_planes[plane_index] = end_planes[plane_index].clone();
            if next_planes == current_planes {
                continue;
            }
            let (next_point, next_trace_planes) =
                match cached_affine_from_planes_with(&mut *affine_cache, &next_planes, || {
                    affine_from_planes(&next_planes)
                }) {
                    Ok(point) => {
                        if next_planes == *end_planes
                            && !point_is_inside_optional_trace_bounds(&point, trace_bounds)?
                        {
                            saw_unknown = true;
                            valid = false;
                            break;
                        }
                        adapt_plane_replacement_vertex_to_trace_bounds(
                            point,
                            next_planes.clone(),
                            trace_bounds,
                        )?
                    }
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                        valid = false;
                        break;
                    }
                    Err(err) => return Err(err),
                };
            let reachable = match cached_plane_replacement_reachability_step_with(
                &mut *step_cache,
                mode,
                &current_point,
                &next_point,
                &current_trace_planes,
                &next_trace_planes,
                || {
                    trace_step(
                        &current_point,
                        &next_point,
                        std::slice::from_ref(&current_trace_planes),
                        std::slice::from_ref(&next_trace_planes),
                    )
                },
            ) {
                Ok(reachable) => reachable,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    valid = false;
                    break;
                }
                Err(err) => return Err(err),
            };
            if !reachable {
                valid = false;
                break;
            }
            current_point = next_point;
            current_trace_planes = next_trace_planes;
            current_planes = next_planes;
        }

        if valid {
            return Ok(true);
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    }
}

pub(super) fn ordered_axis_orderings_by_no_step_precheck_with(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    affine_cache: &mut PlaneReplacementAffineCache,
    mut precheck: impl FnMut(&Point3, &Point3, &[Plane; 3], &[Plane; 3]) -> HypermeshResult<bool>,
) -> HypermeshResult<Vec<[usize; 3]>> {
    let mut ordered = AXIS_ORDERINGS.to_vec();
    let mut scored = Vec::with_capacity(ordered.len());
    for (index, ordering) in ordered.iter().copied().enumerate() {
        scored.push((
            ordering_no_step_precheck_key(
                &ordering,
                start_planes,
                end_planes,
                affine_cache,
                &mut precheck,
            )?,
            index,
        ));
    }
    ordered.sort_by_key(|ordering| {
        let index = AXIS_ORDERINGS
            .iter()
            .position(|candidate| candidate == ordering)
            .unwrap_or(usize::MAX);
        scored[index]
    });
    Ok(ordered)
}

fn ordering_no_step_precheck_key(
    ordering: &[usize; 3],
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    affine_cache: &mut PlaneReplacementAffineCache,
    precheck: &mut impl FnMut(&Point3, &Point3, &[Plane; 3], &[Plane; 3]) -> HypermeshResult<bool>,
) -> HypermeshResult<[u8; 3]> {
    let mut key = [0u8; 3];
    let mut current_planes = start_planes.clone();
    let mut current_point =
        match cached_affine_from_planes_with(&mut *affine_cache, &current_planes, || {
            affine_from_planes(&current_planes)
        }) {
            Ok(point) => point,
            Err(HypermeshError::UnknownClassification) => return Ok([3, 3, 3]),
            Err(err) => return Err(err),
        };

    for (step_index, plane_index) in ordering.iter().copied().enumerate() {
        let mut next_planes = current_planes.clone();
        next_planes[plane_index] = end_planes[plane_index].clone();
        if next_planes == current_planes {
            key[step_index] = 0;
            continue;
        }
        let next_point =
            match cached_affine_from_planes_with(&mut *affine_cache, &next_planes, || {
                affine_from_planes(&next_planes)
            }) {
                Ok(point) => point,
                Err(HypermeshError::UnknownClassification) => {
                    key[step_index] = 3;
                    break;
                }
                Err(err) => return Err(err),
            };
        key[step_index] = match precheck(&current_point, &next_point, &current_planes, &next_planes)
        {
            Ok(true) => 0,
            Err(HypermeshError::UnknownClassification) => 1,
            Ok(false) => 2,
            Err(err) => return Err(err),
        };
        current_point = next_point;
        current_planes = next_planes;
    }

    Ok(key)
}

pub(super) fn cached_plane_replacement_reachability_step_with(
    cache: &mut PlaneReplacementReachabilityStepCache,
    mode: PlaneReplacementReachabilityStepMode,
    current_point: &Point3,
    next_point: &Point3,
    current_planes: &[Plane; 3],
    next_planes: &[Plane; 3],
    trace: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(index) = matching_plane_replacement_reachability_step_bucket_indices(
        &cache.buckets,
        mode,
        current_point,
        next_point,
        current_planes,
        next_planes,
    )
    .and_then(|indices| indices.iter().rev().copied().next())
    {
        return cache.entries[index].result.clone();
    }

    cache
        .entries
        .push(PlaneReplacementReachabilityStepCacheEntry {
            mode,
            current_point: current_point.clone(),
            next_point: next_point.clone(),
            current_planes: current_planes.clone(),
            next_planes: next_planes.clone(),
            result: Err(HypermeshError::UnknownClassification),
        });
    let cache_index = cache.entries.len() - 1;
    push_plane_replacement_reachability_step_bucket_entry(
        &mut cache.buckets,
        mode,
        current_point,
        next_point,
        current_planes,
        next_planes,
        cache_index,
    );
    let result = trace();
    cache.entries[cache_index].result = result.clone();
    result
}

pub(super) fn cached_plane_replacement_reachability_path_with(
    cache: &mut PlaneReplacementReachabilityPathCache,
    mode: PlaneReplacementReachabilityStepMode,
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    trace: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(index) = matching_plane_replacement_reachability_path_bucket_indices(
        &cache.buckets,
        mode,
        start_planes,
        end_planes,
    )
    .and_then(|indices| indices.iter().rev().copied().next())
    {
        return cache.entries[index].result.clone();
    }

    cache
        .entries
        .push(PlaneReplacementReachabilityPathCacheEntry {
            mode,
            start_planes: start_planes.clone(),
            end_planes: end_planes.clone(),
            result: Err(HypermeshError::UnknownClassification),
        });
    let cache_index = cache.entries.len() - 1;
    push_plane_replacement_reachability_path_bucket_entry(
        &mut cache.buckets,
        mode,
        start_planes,
        end_planes,
        cache_index,
    );
    let result = trace();
    cache.entries[cache_index].result = result.clone();
    result
}

pub(super) fn cached_plane_replacement_no_nested_ordering_warmup_with(
    cache: &mut PlaneReplacementNoNestedOrderingWarmupCache,
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    affine_cache: &mut PlaneReplacementAffineCache,
    path_cache: &mut PlaneReplacementReachabilityPathCache,
    step_cache: &mut PlaneReplacementReachabilityStepCache,
    warm: impl FnOnce(
        &mut PlaneReplacementAffineCache,
        &mut PlaneReplacementReachabilityPathCache,
        &mut PlaneReplacementReachabilityStepCache,
    ) -> HypermeshResult<Vec<[usize; 3]>>,
) -> HypermeshResult<Vec<[usize; 3]>> {
    if let Some(index) = matching_plane_replacement_no_nested_ordering_warmup_bucket_indices(
        &cache.buckets,
        start_planes,
        end_planes,
    )
    .and_then(|indices| {
        indices.iter().rev().copied().find(|index| {
            definition_planes_match_as_sets(&cache.entries[*index].start_planes, start_planes)
                && definition_planes_match_as_sets(&cache.entries[*index].end_planes, end_planes)
        })
    }) {
        *affine_cache = cache.entries[index].affine_cache.clone();
        *path_cache = cache.entries[index].path_cache.clone();
        *step_cache = cache.entries[index].step_cache.clone();
        return cache.entries[index].ordered.clone();
    }

    let ordered = warm(affine_cache, path_cache, step_cache);
    cache
        .entries
        .push(PlaneReplacementNoNestedOrderingWarmupCacheEntry {
            start_planes: start_planes.clone(),
            end_planes: end_planes.clone(),
            ordered: ordered.clone(),
            affine_cache: affine_cache.clone(),
            path_cache: path_cache.clone(),
            step_cache: step_cache.clone(),
        });
    let index = cache.entries.len() - 1;
    push_plane_replacement_no_nested_ordering_warmup_bucket_entry(
        &mut cache.buckets,
        start_planes,
        end_planes,
        index,
    );
    ordered
}

fn matching_plane_replacement_no_nested_ordering_warmup_bucket_indices<'a>(
    buckets: &'a [PlaneReplacementNoNestedOrderingWarmupBucket],
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
) -> Option<&'a [usize]> {
    buckets
        .iter()
        .rev()
        .find(|bucket| {
            definition_planes_match_as_sets(&bucket.start_planes, start_planes)
                && definition_planes_match_as_sets(&bucket.end_planes, end_planes)
        })
        .map(|bucket| bucket.indices.as_slice())
}

fn push_plane_replacement_no_nested_ordering_warmup_bucket_entry(
    buckets: &mut Vec<PlaneReplacementNoNestedOrderingWarmupBucket>,
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    index: usize,
) {
    if let Some(existing) = buckets.iter_mut().find(|bucket| {
        definition_planes_match_as_sets(&bucket.start_planes, start_planes)
            && definition_planes_match_as_sets(&bucket.end_planes, end_planes)
    }) {
        existing.indices.push(index);
        return;
    }

    buckets.push(PlaneReplacementNoNestedOrderingWarmupBucket {
        start_planes: start_planes.clone(),
        end_planes: end_planes.clone(),
        indices: vec![index],
    });
}

fn matching_plane_replacement_reachability_step_bucket_indices<'a>(
    buckets: &'a [PlaneReplacementReachabilityStepBucket],
    mode: PlaneReplacementReachabilityStepMode,
    current_point: &Point3,
    next_point: &Point3,
    current_planes: &[Plane; 3],
    next_planes: &[Plane; 3],
) -> Option<&'a [usize]> {
    buckets
        .iter()
        .rev()
        .find(|bucket| {
            bucket.mode == mode
                && ((bucket.current_point == *current_point
                    && bucket.next_point == *next_point
                    && definition_planes_match_as_sets(&bucket.current_planes, current_planes)
                    && definition_planes_match_as_sets(&bucket.next_planes, next_planes))
                    || (bucket.current_point == *next_point
                        && bucket.next_point == *current_point
                        && definition_planes_match_as_sets(&bucket.current_planes, next_planes)
                        && definition_planes_match_as_sets(&bucket.next_planes, current_planes)))
        })
        .map(|bucket| bucket.indices.as_slice())
}

pub(super) fn push_plane_replacement_reachability_step_bucket_entry(
    buckets: &mut Vec<PlaneReplacementReachabilityStepBucket>,
    mode: PlaneReplacementReachabilityStepMode,
    current_point: &Point3,
    next_point: &Point3,
    current_planes: &[Plane; 3],
    next_planes: &[Plane; 3],
    index: usize,
) {
    if let Some(existing) = buckets.iter_mut().find(|bucket| {
        bucket.mode == mode
            && bucket.current_point == *current_point
            && bucket.next_point == *next_point
            && definition_planes_match_as_sets(&bucket.current_planes, current_planes)
            && definition_planes_match_as_sets(&bucket.next_planes, next_planes)
    }) {
        existing.indices.push(index);
        return;
    }

    buckets.push(PlaneReplacementReachabilityStepBucket {
        mode,
        current_point: current_point.clone(),
        next_point: next_point.clone(),
        current_planes: current_planes.clone(),
        next_planes: next_planes.clone(),
        indices: vec![index],
    });
}

fn matching_plane_replacement_reachability_path_bucket_indices<'a>(
    buckets: &'a [PlaneReplacementReachabilityPathBucket],
    mode: PlaneReplacementReachabilityStepMode,
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
) -> Option<&'a [usize]> {
    buckets
        .iter()
        .rev()
        .find(|bucket| {
            bucket.mode == mode
                && ((definition_planes_match_as_sets(&bucket.start_planes, start_planes)
                    && definition_planes_match_as_sets(&bucket.end_planes, end_planes))
                    || (definition_planes_match_as_sets(&bucket.start_planes, end_planes)
                        && definition_planes_match_as_sets(&bucket.end_planes, start_planes)))
        })
        .map(|bucket| bucket.indices.as_slice())
}

pub(super) fn push_plane_replacement_reachability_path_bucket_entry(
    buckets: &mut Vec<PlaneReplacementReachabilityPathBucket>,
    mode: PlaneReplacementReachabilityStepMode,
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    index: usize,
) {
    if let Some(existing) = buckets.iter_mut().find(|bucket| {
        bucket.mode == mode
            && definition_planes_match_as_sets(&bucket.start_planes, start_planes)
            && definition_planes_match_as_sets(&bucket.end_planes, end_planes)
    }) {
        existing.indices.push(index);
        return;
    }

    buckets.push(PlaneReplacementReachabilityPathBucket {
        mode,
        start_planes: start_planes.clone(),
        end_planes: end_planes.clone(),
        indices: vec![index],
    });
}
