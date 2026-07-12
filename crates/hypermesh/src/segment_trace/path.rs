//! Exact segment paths, detour construction, and plane replacement.

#[cfg(test)]
use super::StrictAabbTargetFamilyCacheEntry;
use super::probe_cache::{SurfaceCacheEntry, cached_surface_query_with};
use super::{
    AxisOrderedSegmentTraceCacheEntry, CrossingEvent, DIRECT_TARGET_RANK_REFINEMENT_LIMIT,
    DefinitionNoDetourTraceCacheEntry, DetourTarget, DetourTargetFamilyBucket,
    DetourTargetFamilyCache, DetourTargetFamilyCacheEntry, InteriorBoxAxisIntervalsBucket,
    InteriorBoxAxisIntervalsCache, InteriorBoxAxisIntervalsCacheEntry, InteriorLeafPoint,
    PlaneDefinedPoint, PlaneReplacementAffineBucket, PlaneReplacementAffineCache,
    PlaneReplacementAffineCacheEntry, PlaneReplacementStepCacheEntry, PolygonPointLocation,
    ProbePoint, ShiftedHalfspaceWitness, StrictAabbTargetFamilies, StrictAabbTargetFamilyBucket,
    StrictAabbTargetFamilyCache, TraceAxisSegmentResult, VisitedDefinitionPoint,
    active_planes_from_optional_report, classify_point_in_polygon,
    dedupe_shifted_halfspace_seed_families, dominant_normal_axis,
    extend_shifted_halfspace_seed_families_backtracking_unknown,
    halfspace_cell_seed_families_from_optional_report, optional_halfspace_feasibility_report,
    point_strictly_between_axis, probe_definitions_from_active_halfspaces,
    probe_definitions_or_axis, segment_plane_crossing, shifted_halfspace_cell_witnesses_from_seed,
    shifted_halfspace_seed_families_with_report_seed, shifted_halfspace_witness_family_or_empty,
    sort_crossing_events,
};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{
    Aabb, Classification, Plane, PreparedPoint3, axis_mut, axis_ref, classify_point, classify_real,
    compare_real,
};
use crate::halfspace::aabb_core_halfspaces;
use crate::polygon::ConvexPolygon;
use crate::winding::WindingNumberVector;
use hyperlattice::{Point3, Real, intersect_three_planes};
use hyperlimit::{
    HalfspaceFeasibility, Plane3 as LimitPlane3, PredicateOutcome, Sign,
    classify_plane_aabb3_report,
};

pub(super) fn detour_arrangement_planes(polygons: &[ConvexPolygon]) -> Vec<Plane> {
    let mut planes = Vec::new();
    for polygon in polygons {
        if !planes.iter().any(|existing| existing == &polygon.support) {
            planes.push(polygon.support.clone());
        }
    }
    planes
}

pub(super) fn detour_arrangement_cell(
    point: &Point3,
    arrangement_planes: &[Plane],
) -> HypermeshResult<Vec<Classification>> {
    let point = PreparedPoint3::new(point);
    arrangement_planes
        .iter()
        .map(|plane| point.classify(plane))
        .collect()
}

fn optional_detour_arrangement_cell(
    point: &Point3,
    arrangement_planes: &[Plane],
) -> HypermeshResult<Option<Vec<Classification>>> {
    match detour_arrangement_cell(point, arrangement_planes) {
        Ok(cell) => Ok(Some(cell)),
        Err(HypermeshError::UnknownClassification) => Ok(None),
        Err(err) => Err(err),
    }
}

pub(super) fn strict_aabb_arrangement_cell(
    bounds: &Aabb,
    arrangement_planes: &[Plane],
) -> HypermeshResult<Option<Vec<Classification>>> {
    let mut cell = Vec::with_capacity(arrangement_planes.len());
    for plane in arrangement_planes {
        let limit_plane = LimitPlane3::new(plane.normal.clone(), plane.offset.clone());
        let report = match classify_plane_aabb3_report(&limit_plane, &bounds.min, &bounds.max) {
            PredicateOutcome::Decided { value, .. } => value,
            PredicateOutcome::Unknown { .. } => return Ok(None),
        };
        let side = match (report.lower_sign, report.upper_sign) {
            (Sign::Negative, Sign::Negative | Sign::Zero) => Classification::Negative,
            (Sign::Zero | Sign::Positive, Sign::Positive) => Classification::Positive,
            (Sign::Zero, Sign::Zero) => Classification::On,
            (Sign::Negative, Sign::Positive) => return Ok(None),
            _ => return Ok(None),
        };
        cell.push(side);
    }
    Ok(Some(cell))
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct DetourArrangementCellState {
    cell: Vec<Classification>,
    uncertified_definition_fallback: bool,
}

pub(super) fn detour_arrangement_cell_state_is_dominated(
    seen: &[DetourArrangementCellState],
    cell: &[Classification],
    uncertified_definition_fallback: bool,
) -> bool {
    seen.iter().any(|existing| {
        existing.cell == cell
            && (!existing.uncertified_definition_fallback || uncertified_definition_fallback)
    })
}

pub(super) fn record_detour_arrangement_cell_state(
    seen: &mut Vec<DetourArrangementCellState>,
    cell: Vec<Classification>,
    uncertified_definition_fallback: bool,
) {
    if let Some(existing) = seen.iter_mut().find(|existing| existing.cell == cell) {
        existing.uncertified_definition_fallback &= uncertified_definition_fallback;
    } else {
        seen.push(DetourArrangementCellState {
            cell,
            uncertified_definition_fallback,
        });
    }
}

fn mark_all_interior_points_uncertified(points: &mut Vec<InteriorLeafPoint>) {
    for point in points {
        point.uncertified_definition_fallback = true;
    }
}

fn mark_all_probe_points_uncertified(probes: &mut Vec<ProbePoint>) {
    for probe in probes {
        probe.uncertified_definition_fallback = true;
    }
}

pub(super) fn mark_all_detour_targets_uncertified(targets: &mut Vec<DetourTarget>) {
    for target in targets {
        target.uncertified_definition_fallback = true;
    }
}

fn mark_all_shifted_halfspace_witnesses_uncertified(witnesses: &mut Vec<ShiftedHalfspaceWitness>) {
    for witness in witnesses {
        witness.uncertified_definition_fallback = true;
    }
}

pub(super) fn finalize_interior_point_family(
    points: &mut Vec<InteriorLeafPoint>,
    saw_unknown: bool,
) -> HypermeshResult<()> {
    let saw_unknown = saw_unknown
        || points
            .iter()
            .any(|point| point.uncertified_definition_fallback);
    if points.is_empty() && saw_unknown {
        return Err(HypermeshError::UnknownClassification);
    }
    if saw_unknown {
        mark_all_interior_points_uncertified(points);
    }
    Ok(())
}

pub(super) fn finalize_probe_point_family(
    probes: &mut Vec<ProbePoint>,
    saw_unknown: bool,
) -> HypermeshResult<()> {
    let saw_unknown = saw_unknown
        || probes
            .iter()
            .any(|probe| probe.uncertified_definition_fallback);
    if probes.is_empty() && saw_unknown {
        return Err(HypermeshError::UnknownClassification);
    }
    if saw_unknown {
        mark_all_probe_points_uncertified(probes);
    }
    Ok(())
}

fn finalize_detour_target_family(
    targets: &mut Vec<DetourTarget>,
    saw_unknown: bool,
) -> HypermeshResult<()> {
    let saw_unknown = saw_unknown
        || targets
            .iter()
            .any(|target| target.uncertified_definition_fallback);
    if targets.is_empty() && saw_unknown {
        return Err(HypermeshError::UnknownClassification);
    }
    if saw_unknown {
        mark_all_detour_targets_uncertified(targets);
    }
    Ok(())
}

pub(super) fn finalize_shifted_halfspace_witness_family(
    witnesses: &mut Vec<ShiftedHalfspaceWitness>,
    saw_unknown: bool,
) -> HypermeshResult<()> {
    let saw_unknown = saw_unknown
        || witnesses
            .iter()
            .any(|witness| witness.uncertified_definition_fallback);
    if witnesses.is_empty() && saw_unknown {
        return Err(HypermeshError::UnknownClassification);
    }
    if saw_unknown {
        mark_all_shifted_halfspace_witnesses_uncertified(witnesses);
    }
    Ok(())
}

/// Traces an axis-aligned segment, accumulating polygon winding transitions.
pub fn trace_axis_segment(
    start: &Point3,
    end: &Point3,
    axis: usize,
    start_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<TraceAxisSegmentResult> {
    let mut winding = start_wnv.to_vec();
    let direction = compare_real(axis_ref(end, axis), axis_ref(start, axis))?;
    if direction.is_eq() {
        match point_lies_on_traced_surface(start, polygons) {
            Ok(false) => {}
            Ok(true) | Err(HypermeshError::UnknownClassification) => {
                return Err(HypermeshError::UnknownClassification);
            }
            Err(err) => return Err(err),
        }
        return Ok(TraceAxisSegmentResult {
            winding,
            valid: true,
        });
    }

    let dir_sign = if direction.is_gt() { 1 } else { -1 };
    let mut events = Vec::new();
    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        let normal_axis = axis_ref(&polygon.support.normal, axis);
        if normal_axis.definitely_zero() {
            continue;
        }

        let start_value = polygon.support.expression_at_point(start);
        let end_value = polygon.support.expression_at_point(end);
        let start_class = classify_real(&start_value)?;
        let end_class = classify_real(&end_value)?;
        if start_class == Classification::On {
            match classify_point_in_polygon(start, polygon)? {
                PolygonPointLocation::Outside => {}
                PolygonPointLocation::Boundary | PolygonPointLocation::Interior => {
                    return Err(HypermeshError::UnknownClassification);
                }
            }
            continue;
        }
        if end_class == Classification::On {
            match classify_point_in_polygon(end, polygon)? {
                PolygonPointLocation::Outside => {}
                PolygonPointLocation::Boundary | PolygonPointLocation::Interior => {
                    return Err(HypermeshError::UnknownClassification);
                }
            }
            continue;
        }
        if start_class == end_class {
            continue;
        }

        let Some(crossing) = segment_plane_crossing(start, end, &polygon.support)? else {
            continue;
        };

        if !point_strictly_between_axis(&crossing, start, end, axis)? {
            continue;
        }

        let mut inside = true;
        let mut boundary_edge_count = 0;
        for edge in &polygon.edges {
            match classify_point(&crossing, edge)? {
                Classification::Positive => {
                    inside = false;
                    break;
                }
                Classification::On => boundary_edge_count += 1,
                Classification::Negative => {}
            }
        }
        if !inside {
            continue;
        }

        let normal_sign = match crate::geometry::classify_real(normal_axis)? {
            Classification::Positive => 1,
            Classification::Negative => -1,
            Classification::On => continue,
        };
        let cross_sign = normal_sign * -dir_sign;
        events.push(CrossingEvent {
            point: crossing,
            support: polygon.support.clone(),
            normal_sign,
            cross_sign,
            delta_w: polygon.delta_w.clone(),
            boundary_edge_count,
        });
    }

    let mut accepted = accepted_crossing_events(&events)?;

    sort_crossing_events(&mut accepted, axis, dir_sign)?;

    for event in accepted {
        apply_winding_transition_in_place(&mut winding, event.cross_sign, &event.delta_w)?;
    }

    Ok(TraceAxisSegmentResult {
        winding,
        valid: true,
    })
}

pub(super) const AXIS_ORDERINGS: [[usize; 3]; 6] = [
    [0, 1, 2],
    [0, 2, 1],
    [1, 0, 2],
    [1, 2, 0],
    [2, 0, 1],
    [2, 1, 0],
];

#[cfg(test)]
const MIN_DETOUR_RECURSION_LIMIT: usize = 2;
#[cfg(test)]
const MIN_PLANE_REPLACEMENT_STEP_DETOUR_LIMIT: usize = 1;

/// Traces an axis-aligned polyline using several axis orderings and returns
/// the first valid winding result. If direct L-shaped paths are blocked by
/// exact surface hits, retries through arrangement-coordinate endpoint-box
/// detours.
pub fn trace_segment(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    trace_segment_from_definitions(
        start,
        end,
        winding,
        polygons,
        &[axis_plane_definition(start)],
        &[axis_plane_definition(end)],
    )
}

pub(crate) fn trace_segment_from_definitions(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<WindingNumberVector> {
    let mut no_detour_cache = Vec::new();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    trace_segment_from_definitions_with_caches(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        &mut no_detour_cache,
        &mut detour_target_cache,
        None,
    )
}

pub(super) fn trace_segment_from_definitions_with_caches(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    no_detour_cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<WindingNumberVector> {
    let mut surface_cache = Vec::new();
    let mut axis_ordered_segment_traces = Vec::new();
    let mut plane_replacement_affine = PlaneReplacementAffineCache::default();
    let mut plane_replacement_trace_steps = Vec::new();
    trace_segment_from_definitions_with_caches_and_surface_query(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        &mut surface_cache,
        &mut axis_ordered_segment_traces,
        &mut plane_replacement_affine,
        &mut plane_replacement_trace_steps,
        no_detour_cache,
        detour_target_cache,
        trace_bounds,
    )
}

fn trace_segment_from_definitions_with_caches_and_surface_query(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    axis_ordered_segment_traces: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
    plane_replacement_affine: &mut PlaneReplacementAffineCache,
    plane_replacement_trace_steps: &mut Vec<PlaneReplacementStepCacheEntry>,
    no_detour_cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<WindingNumberVector> {
    let mut no_detour_surface_cache = Vec::new();
    let arrangement_planes = detour_arrangement_planes(polygons);
    let mut trace_without_detours =
        |start: &Point3,
         end: &Point3,
         winding: &[i32],
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            if points_share_open_arrangement_cell(start, end, &arrangement_planes)? {
                return Ok(Some(winding.to_vec()));
            }
            cached_definition_no_detour_trace_with(
                &mut *no_detour_cache,
                start,
                end,
                winding,
                start_definitions,
                end_definitions,
                || {
                    trace_segment_with_definitions_no_detours_with_caches(
                        start,
                        end,
                        winding,
                        polygons,
                        start_definitions,
                        end_definitions,
                        &mut no_detour_surface_cache,
                        axis_ordered_segment_traces,
                        plane_replacement_affine,
                        plane_replacement_trace_steps,
                        trace_bounds,
                    )
                },
            )
        };
    let mut detour_batches = InteriorBoxDetourTargetBatchCache::default();
    trace_segment_with_detour_batches_breadth_first_with_surface_query(
        start,
        end,
        winding,
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
    )
}

pub(super) fn point_is_inside_optional_trace_bounds(
    point: &Point3,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<bool> {
    trace_bounds.map_or(Ok(true), |bounds| bounds.contains_point(point))
}

pub(super) fn trace_bounds_including_point(bounds: &Aabb, point: &Point3) -> HypermeshResult<Aabb> {
    let mut min = bounds.min.clone();
    let mut max = bounds.max.clone();
    for axis in 0..3 {
        if compare_real(axis_ref(point, axis), axis_ref(&min, axis))?.is_lt() {
            *axis_mut(&mut min, axis) = axis_ref(point, axis).clone();
        }
        if compare_real(axis_ref(point, axis), axis_ref(&max, axis))?.is_gt() {
            *axis_mut(&mut max, axis) = axis_ref(point, axis).clone();
        }
    }
    Ok(Aabb::new(min, max))
}

pub(super) fn adapt_plane_replacement_vertex_to_trace_bounds(
    point: Point3,
    planes: [Plane; 3],
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<(Point3, [Plane; 3])> {
    let Some(bounds) = trace_bounds else {
        return Ok((point, planes));
    };
    if bounds.contains_point(&point)? {
        return Ok((point, planes));
    }

    // Keep the replacement walk local by moving an outside intermediate onto
    // exact AABB planes. The resulting legs are still certified by the tracer.
    let mut adapted = point;
    for axis in 0..3 {
        if compare_real(axis_ref(&adapted, axis), axis_ref(&bounds.min, axis))?.is_lt() {
            *axis_mut(&mut adapted, axis) = axis_ref(&bounds.min, axis).clone();
        } else if compare_real(axis_ref(&adapted, axis), axis_ref(&bounds.max, axis))?.is_gt() {
            *axis_mut(&mut adapted, axis) = axis_ref(&bounds.max, axis).clone();
        }
    }
    let definitions = axis_plane_definition(&adapted);
    Ok((adapted, definitions))
}

pub(super) fn points_share_open_arrangement_cell(
    start: &Point3,
    end: &Point3,
    arrangement_planes: &[Plane],
) -> HypermeshResult<bool> {
    let Some(start_cell) = optional_detour_arrangement_cell(start, arrangement_planes)? else {
        return Ok(false);
    };
    let Some(end_cell) = optional_detour_arrangement_cell(end, arrangement_planes)? else {
        return Ok(false);
    };
    Ok(start_cell == end_cell && start_cell.iter().all(|side| *side != Classification::On))
}

#[cfg(test)]
#[allow(dead_code)]
pub(super) fn trace_segment_from_definitions_with_cycle_guard_impl(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[i32],
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<WindingNumberVector> {
    let mut surface_cache = Vec::new();
    trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
        start,
        end,
        winding,
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
pub(super) fn trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[i32],
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<WindingNumberVector> {
    match trace_without_detours(start, end, winding, start_definitions, end_definitions) {
        Ok(Some(winding)) => return Ok(winding),
        Ok(None) => {}
        Err(HypermeshError::UnknownClassification) => {}
        Err(err) => return Err(err),
    }

    if let Some(winding) = trace_segment_via_detours_with_cycle_guard_with_surface_query(
        start,
        end,
        winding,
        polygons,
        &detours_for(start, end)?,
        start_definitions,
        end_definitions,
        visited_points,
        surface_cache,
        surface_query,
        trace_without_detours,
        detours_for,
    )? {
        return Ok(winding);
    }

    Err(HypermeshError::UnknownClassification)
}

#[cfg(test)]
#[allow(dead_code)]
fn trace_segment_from_definitions_with_budget(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
) -> HypermeshResult<WindingNumberVector> {
    let mut trace_without_detours =
        |start: &Point3,
         end: &Point3,
         winding: &[i32],
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            trace_segment_with_definitions_no_detours(
                start,
                end,
                winding,
                polygons,
                start_definitions,
                end_definitions,
            )
        };
    let mut detours_for =
        |start: &Point3, end: &Point3| interior_box_detour_targets(start, end, polygons);
    trace_segment_from_definitions_with_budget_impl(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        remaining_detours,
        &mut trace_without_detours,
        &mut detours_for,
    )
}

#[cfg(test)]
pub(super) fn trace_segment_from_definitions_with_budget_impl(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[i32],
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<WindingNumberVector> {
    match trace_without_detours(start, end, winding, start_definitions, end_definitions) {
        Ok(Some(winding)) => return Ok(winding),
        Ok(None) => {}
        Err(HypermeshError::UnknownClassification) => {}
        Err(err) => return Err(err),
    }

    if remaining_detours == 0 {
        return Err(HypermeshError::UnknownClassification);
    }

    if let Some(winding) = trace_segment_via_detours_with_definitions_budget(
        start,
        end,
        winding,
        polygons,
        &detours_for(start, end)?,
        start_definitions,
        end_definitions,
        remaining_detours,
        trace_without_detours,
        detours_for,
    )? {
        return Ok(winding);
    }

    Err(HypermeshError::UnknownClassification)
}

#[cfg(test)]
pub(super) fn trace_segment_via_detours_with_cycle_guard_with_surface_query(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    detours: &[DetourTarget],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[i32],
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut saw_unknown = false;
    for detour in detours {
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
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            }
        };
        if already_visited || on_surface {
            if detour.uncertified_definition_fallback {
                saw_unknown = true;
            }
            continue;
        }

        let mut next_visited_points = visited_points.to_vec();
        if !visited_definition_family_contains(
            &next_visited_points,
            &detour.point,
            &detour.definitions,
        ) {
            next_visited_points.push(VisitedDefinitionPoint {
                point: detour.point.clone(),
                definitions: detour.definitions.clone(),
            });
        }

        let first_leg = match if start_definition_transition {
            match trace_without_detours(
                start,
                &detour.point,
                winding,
                start_definitions,
                &detour.definitions,
            ) {
                Ok(Some(first_leg)) => Ok(first_leg),
                Ok(None) => Err(HypermeshError::UnknownClassification),
                Err(err) => Err(err),
            }
        } else {
            trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
                start,
                &detour.point,
                winding,
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
            Ok(first_leg) => first_leg,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        let second_leg = match if end_definition_transition {
            match trace_without_detours(
                &detour.point,
                end,
                &first_leg,
                &detour.definitions,
                end_definitions,
            ) {
                Ok(Some(second_leg)) => Ok(second_leg),
                Ok(None) => Err(HypermeshError::UnknownClassification),
                Err(err) => Err(err),
            }
        } else {
            trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
                &detour.point,
                end,
                &first_leg,
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
            Ok(second_leg) => second_leg,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        return Ok(Some(second_leg));
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

pub(super) fn trace_segment_with_detour_batches_breadth_first_with_surface_query(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    arrangement_planes: &[Plane],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[i32],
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
    detour_batch_for: &mut impl FnMut(
        &Point3,
        &Point3,
        usize,
    ) -> HypermeshResult<Option<Vec<DetourTarget>>>,
) -> HypermeshResult<WindingNumberVector> {
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

    while let Some((path, batch_index)) = queue.pop_front() {
        let mut attempt = winding.to_vec();
        let mut unresolved_edge = None;
        for index in 0..path.len() - 1 {
            match trace_without_detours(
                &path[index].point,
                &path[index + 1].point,
                &attempt,
                &path[index].definitions,
                &path[index + 1].definitions,
            ) {
                Ok(Some(next_winding)) => attempt = next_winding,
                Ok(None) | Err(HypermeshError::UnknownClassification) => {
                    unresolved_edge = Some((index, attempt.clone()));
                    break;
                }
                Err(err) => return Err(err),
            }
        }

        let Some((edge_index, edge_winding)) = unresolved_edge else {
            return Ok(attempt);
        };
        let edge_start = &path[edge_index];
        let edge_end = &path[edge_index + 1];
        let mut detours = match detour_batch_for(&edge_start.point, &edge_end.point, batch_index) {
            Ok(Some(detours)) => detours,
            Ok(None) | Err(HypermeshError::UnknownClassification) => continue,
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
                    Err(HypermeshError::UnknownClassification) => continue,
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
                            Ok(_) | Err(HypermeshError::UnknownClassification) => {}
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
                    Ok(true) | Err(HypermeshError::UnknownClassification) => continue,
                    Ok(false) => {}
                    Err(err) => return Err(err),
                }
            }

            let mut next_path = path.clone();
            let detour_uncertified_definition_fallback = detour.uncertified_definition_fallback;
            next_path.insert(edge_index + 1, detour);
            if seen_paths.contains(&next_path) {
                continue;
            }

            let mut next_attempt = edge_winding.clone();
            let mut complete = true;
            for index in edge_index..next_path.len() - 1 {
                match trace_without_detours(
                    &next_path[index].point,
                    &next_path[index + 1].point,
                    &next_attempt,
                    &next_path[index].definitions,
                    &next_path[index + 1].definitions,
                ) {
                    Ok(Some(next_winding)) => next_attempt = next_winding,
                    Ok(None) | Err(HypermeshError::UnknownClassification) => {
                        complete = false;
                        break;
                    }
                    Err(err) => return Err(err),
                }
            }
            if complete {
                return Ok(next_attempt);
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

    Err(HypermeshError::UnknownClassification)
}

pub(super) fn cached_definition_no_detour_trace_with(
    cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    trace: impl FnOnce() -> HypermeshResult<Option<WindingNumberVector>>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        existing.start == *start
            && existing.end == *end
            && existing.winding == winding
            && definition_families_match_as_sets(&existing.start_definitions, start_definitions)
            && definition_families_match_as_sets(&existing.end_definitions, end_definitions)
    }) {
        return existing.result.clone();
    }

    let result = trace();
    cache.push(DefinitionNoDetourTraceCacheEntry {
        start: start.clone(),
        end: end.clone(),
        winding: winding.to_vec(),
        start_definitions: start_definitions.to_vec(),
        end_definitions: end_definitions.to_vec(),
        result: result.clone(),
    });
    result
}

fn matching_detour_target_family_bucket_indices<'a>(
    buckets: &'a [DetourTargetFamilyBucket],
    start: &Point3,
    end: &Point3,
) -> Option<&'a [usize]> {
    buckets
        .iter()
        .rev()
        .find(|existing| {
            (existing.start == *start && existing.end == *end)
                || (existing.start == *end && existing.end == *start)
        })
        .map(|bucket| bucket.indices.as_slice())
}

pub(super) fn push_detour_target_family_bucket_entry(
    buckets: &mut Vec<DetourTargetFamilyBucket>,
    start: &Point3,
    end: &Point3,
    index: usize,
) {
    if let Some(bucket) = buckets.iter_mut().find(|existing| {
        (existing.start == *start && existing.end == *end)
            || (existing.start == *end && existing.end == *start)
    }) {
        bucket.indices.push(index);
        return;
    }
    buckets.push(DetourTargetFamilyBucket {
        start: start.clone(),
        end: end.clone(),
        indices: vec![index],
    });
}

pub(super) fn cached_detour_target_family_with(
    cache: &mut DetourTargetFamilyCache,
    start: &Point3,
    end: &Point3,
    trace_bounds: Option<&Aabb>,
    build: impl FnOnce() -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<Vec<DetourTarget>> {
    if let Some(existing) = cached_detour_target_family(cache, start, end, trace_bounds) {
        return existing.targets.clone();
    }

    let targets = build();
    cache.entries.push(DetourTargetFamilyCacheEntry {
        start: start.clone(),
        end: end.clone(),
        trace_bounds: trace_bounds.cloned(),
        targets: targets.clone(),
    });
    let index = cache.entries.len() - 1;
    push_detour_target_family_bucket_entry(&mut cache.buckets, start, end, index);
    targets
}

pub(super) fn cached_detour_target_family<'a>(
    cache: &'a DetourTargetFamilyCache,
    start: &Point3,
    end: &Point3,
    trace_bounds: Option<&Aabb>,
) -> Option<&'a DetourTargetFamilyCacheEntry> {
    matching_detour_target_family_bucket_indices(&cache.buckets, start, end).and_then(|indices| {
        indices
            .iter()
            .rev()
            .filter_map(|index| cache.entries.get(*index))
            .find(|entry| entry.trace_bounds.as_ref() == trace_bounds)
    })
}

#[cfg(test)]
pub(crate) fn trace_segment_without_detours(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut surface_cache = Vec::new();
    let mut axis_ordered_segment_traces = Vec::new();
    trace_segment_without_detours_with_caches(
        start,
        end,
        winding,
        polygons,
        &mut surface_cache,
        &mut axis_ordered_segment_traces,
    )
}

fn trace_segment_without_detours_with_caches(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    axis_ordered_segment_traces: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let axis_unknown = match trace_axis_ordered_paths_with_caches(
        start,
        end,
        winding,
        polygons,
        surface_cache,
        axis_ordered_segment_traces,
    ) {
        Ok(winding) => return Ok(Some(winding)),
        Err(HypermeshError::UnknownClassification) => true,
        Err(err) => return Err(err),
    };

    let direct_unknown = match trace_direct_segment(start, end, winding, polygons) {
        Ok(traced) if traced.valid => return Ok(Some(traced.winding)),
        Ok(_) => false,
        Err(HypermeshError::UnknownClassification) => true,
        Err(err) => return Err(err),
    };

    if axis_unknown || direct_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

#[cfg(test)]
pub(super) fn trace_segment_via_detours_with_definitions_budget(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    detours: &[DetourTarget],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[i32],
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut saw_unknown = false;
    let mut surface_cache = Vec::new();
    for detour in detours {
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
        let first_leg = match trace_segment_from_definitions_with_budget_impl(
            start,
            &detour.point,
            winding,
            polygons,
            start_definitions,
            &detour.definitions,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        ) {
            Ok(first_leg) => first_leg,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        let second_leg = match trace_segment_from_definitions_with_budget_impl(
            &detour.point,
            end,
            &first_leg,
            polygons,
            &detour.definitions,
            end_definitions,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        ) {
            Ok(second_leg) => second_leg,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        return Ok(Some(second_leg));
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

#[cfg(test)]
pub(super) fn trace_segment_with_definitions_no_detours(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut surface_cache = Vec::new();
    let mut axis_ordered_segment_traces = Vec::new();
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();
    trace_segment_with_definitions_no_detours_with_caches(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        &mut surface_cache,
        &mut axis_ordered_segment_traces,
        &mut affine_cache,
        &mut step_cache,
        None,
    )
}

fn trace_segment_with_definitions_no_detours_with_caches(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    axis_ordered_segment_traces: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
    affine_cache: &mut PlaneReplacementAffineCache,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    if !point_is_inside_optional_trace_bounds(start, trace_bounds)?
        || !point_is_inside_optional_trace_bounds(end, trace_bounds)?
    {
        return Err(HypermeshError::UnknownClassification);
    }

    match trace_segment_without_detours_with_caches(
        start,
        end,
        winding,
        polygons,
        surface_cache,
        axis_ordered_segment_traces,
    ) {
        Ok(Some(winding)) => return Ok(Some(winding)),
        Ok(None) => {}
        Err(HypermeshError::UnknownClassification) => {}
        Err(err) => return Err(err),
    }

    let start_family = endpoint_definition_family(start, start_definitions)?;
    let end_family = endpoint_definition_family(end, end_definitions)?;
    let saw_unknown = start_family.saw_unknown || end_family.saw_unknown;

    let result = definition_pair_trace_backtracking_unknown(
        &start_family.definitions,
        &end_family.definitions,
        |start_definition, end_definition| {
            trace_plane_replacement_path_without_detours_with_shared_caches(
                start_definition,
                end_definition,
                winding,
                polygons,
                surface_cache,
                axis_ordered_segment_traces,
                affine_cache,
                step_cache,
                trace_bounds,
            )
        },
    );
    match result {
        Ok(None) if saw_unknown => Err(HypermeshError::UnknownClassification),
        result => result,
    }
}

pub(super) fn definition_pair_trace_backtracking_unknown(
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    mut trace: impl FnMut(&[Plane; 3], &[Plane; 3]) -> HypermeshResult<WindingNumberVector>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut saw_unknown = false;
    let start_definitions = unique_definition_family(start_definitions);
    let end_definitions = unique_definition_family(end_definitions);

    for start_definition in &start_definitions {
        for end_definition in &end_definitions {
            match trace(start_definition, end_definition) {
                Ok(winding) => return Ok(Some(winding)),
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                }
                Err(err) => return Err(err),
            }
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

#[cfg(test)]
pub(super) fn trace_segment_with_detours_without_plane_replacement_impl(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[i32],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    match trace_without_detours(start, end, winding) {
        Ok(Some(winding)) => return Ok(Some(winding)),
        Ok(None) => {}
        Err(HypermeshError::UnknownClassification) => {}
        Err(err) => return Err(err),
    }

    if remaining_detours == 0 {
        return Ok(None);
    }

    for detour in detours_for(start, end)? {
        if detour.point == *start
            || detour.point == *end
            || point_lies_on_traced_surface(&detour.point, polygons)?
        {
            continue;
        }
        let Some(first_leg) = trace_segment_with_detours_without_plane_replacement_impl(
            start,
            &detour.point,
            winding,
            polygons,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        )?
        else {
            continue;
        };
        let Some(second_leg) = trace_segment_with_detours_without_plane_replacement_impl(
            &detour.point,
            end,
            &first_leg,
            polygons,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        )?
        else {
            continue;
        };
        return Ok(Some(second_leg));
    }

    Ok(None)
}

#[cfg(test)]
pub(crate) fn trace_plane_replacement_path(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();
    trace_plane_replacement_path_with_tracer(
        start_planes,
        end_planes,
        winding,
        polygons,
        |current, next, _current_planes, _next_planes, attempt, polygons| {
            retryable_trace(trace_segment(current, next, attempt, polygons))
        },
        &mut affine_cache,
        &mut step_cache,
    )
}

#[cfg(test)]
#[allow(dead_code)]
fn trace_plane_replacement_path_without_detours(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();
    trace_plane_replacement_path_without_detours_with_caches(
        start_planes,
        end_planes,
        winding,
        polygons,
        &mut affine_cache,
        &mut step_cache,
    )
}

#[cfg(test)]
pub(super) fn trace_plane_replacement_path_without_detours_with_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    affine_cache: &mut PlaneReplacementAffineCache,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
    let mut surface_cache = Vec::new();
    let mut axis_ordered_segment_traces = Vec::new();
    trace_plane_replacement_path_without_detours_with_shared_caches(
        start_planes,
        end_planes,
        winding,
        polygons,
        &mut surface_cache,
        &mut axis_ordered_segment_traces,
        affine_cache,
        step_cache,
        None,
    )
}

fn trace_plane_replacement_path_without_detours_with_shared_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    axis_ordered_segment_traces: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
    affine_cache: &mut PlaneReplacementAffineCache,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_tracer_and_caches(
        start_planes,
        end_planes,
        winding,
        polygons,
        affine_cache,
        step_cache,
        trace_bounds,
        |current, next, _current_planes, _next_planes, attempt, polygons| {
            trace_segment_without_detours_with_caches(
                current,
                next,
                attempt,
                polygons,
                surface_cache,
                axis_ordered_segment_traces,
            )
        },
    )
}

#[cfg(test)]
pub(super) fn trace_plane_replacement_path_with_tracer(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    trace_step: impl FnMut(
        &Point3,
        &Point3,
        &[Plane; 3],
        &[Plane; 3],
        &[i32],
        &[ConvexPolygon],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
    affine_cache: &mut PlaneReplacementAffineCache,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_tracer_and_caches(
        start_planes,
        end_planes,
        winding,
        polygons,
        affine_cache,
        step_cache,
        None,
        trace_step,
    )
}

pub(super) fn trace_plane_replacement_path_with_tracer_and_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    affine_cache: &mut PlaneReplacementAffineCache,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
    trace_bounds: Option<&Aabb>,
    mut trace_step: impl FnMut(
        &Point3,
        &Point3,
        &[Plane; 3],
        &[Plane; 3],
        &[i32],
        &[ConvexPolygon],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
) -> HypermeshResult<WindingNumberVector> {
    for ordering in AXIS_ORDERINGS {
        let mut current_planes = start_planes.clone();
        let mut current_point =
            match cached_affine_from_planes_with(affine_cache, &current_planes, || {
                affine_from_planes(&current_planes)
            }) {
                Ok(point) if point_is_inside_optional_trace_bounds(&point, trace_bounds)? => point,
                Ok(_) => continue,
                Err(HypermeshError::UnknownClassification) => continue,
                Err(err) => return Err(err),
            };
        let mut current_trace_planes = current_planes.clone();
        let mut attempt = winding.to_vec();
        let mut valid = true;

        for plane_index in ordering.iter().copied() {
            let mut next_planes = current_planes.clone();
            next_planes[plane_index] = end_planes[plane_index].clone();
            if next_planes == current_planes {
                continue;
            }
            let (next_point, next_trace_planes) =
                match cached_affine_from_planes_with(affine_cache, &next_planes, || {
                    affine_from_planes(&next_planes)
                }) {
                    Ok(point) => {
                        if next_planes == *end_planes
                            && !point_is_inside_optional_trace_bounds(&point, trace_bounds)?
                        {
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
                        valid = false;
                        break;
                    }
                    Err(err) => return Err(err),
                };
            let next_winding = match cached_plane_replacement_step_with(
                step_cache,
                &current_point,
                &next_point,
                &current_trace_planes,
                &next_trace_planes,
                &attempt,
                || {
                    trace_step(
                        &current_point,
                        &next_point,
                        &current_trace_planes,
                        &next_trace_planes,
                        &attempt,
                        polygons,
                    )
                },
            ) {
                Ok(Some(next_winding)) => next_winding,
                Ok(None) | Err(HypermeshError::UnknownClassification) => {
                    valid = false;
                    break;
                }
                Err(err) => return Err(err),
            };
            attempt = next_winding;
            current_point = next_point;
            current_trace_planes = next_trace_planes;
            current_planes = next_planes;
        }

        if valid {
            return Ok(attempt);
        }
    }

    Err(HypermeshError::UnknownClassification)
}

pub(super) fn cached_affine_from_planes_with(
    cache: &mut PlaneReplacementAffineCache,
    planes: &[Plane; 3],
    compute: impl FnOnce() -> HypermeshResult<Point3>,
) -> HypermeshResult<Point3> {
    if let Some(index) = matching_plane_replacement_affine_bucket_indices(&cache.buckets, planes)
        .and_then(|indices| {
            indices.iter().rev().copied().find(|index| {
                definition_planes_match_as_sets(&cache.entries[*index].planes, planes)
            })
        })
    {
        return cache.entries[index].point.clone();
    }

    let point = compute();
    cache.entries.push(PlaneReplacementAffineCacheEntry {
        planes: planes.clone(),
        point: point.clone(),
    });
    let index = cache.entries.len() - 1;
    push_plane_replacement_affine_bucket_entry(&mut cache.buckets, planes, index);
    point
}

fn matching_plane_replacement_affine_bucket_indices<'a>(
    buckets: &'a [PlaneReplacementAffineBucket],
    planes: &[Plane; 3],
) -> Option<&'a [usize]> {
    buckets
        .iter()
        .rev()
        .find(|bucket| definition_planes_match_as_sets(&bucket.planes, planes))
        .map(|bucket| bucket.indices.as_slice())
}

fn push_plane_replacement_affine_bucket_entry(
    buckets: &mut Vec<PlaneReplacementAffineBucket>,
    planes: &[Plane; 3],
    index: usize,
) {
    if let Some(existing) = buckets
        .iter_mut()
        .find(|bucket| definition_planes_match_as_sets(&bucket.planes, planes))
    {
        existing.indices.push(index);
        return;
    }

    buckets.push(PlaneReplacementAffineBucket {
        planes: planes.clone(),
        indices: vec![index],
    });
}

pub(super) fn cached_plane_replacement_step_with(
    cache: &mut Vec<PlaneReplacementStepCacheEntry>,
    current_point: &Point3,
    next_point: &Point3,
    current_planes: &[Plane; 3],
    next_planes: &[Plane; 3],
    attempt: &[i32],
    trace: impl FnOnce() -> HypermeshResult<Option<WindingNumberVector>>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        existing.current_point == *current_point
            && existing.next_point == *next_point
            && definition_planes_match_as_sets(&existing.current_planes, current_planes)
            && definition_planes_match_as_sets(&existing.next_planes, next_planes)
            && existing.attempt == attempt
    }) {
        return existing.result.clone();
    }

    let result = trace();
    cache.push(PlaneReplacementStepCacheEntry {
        current_point: current_point.clone(),
        next_point: next_point.clone(),
        current_planes: current_planes.clone(),
        next_planes: next_planes.clone(),
        attempt: attempt.to_vec(),
        result: result.clone(),
    });
    result
}

pub(crate) fn affine_from_planes(planes: &[Plane; 3]) -> HypermeshResult<Point3> {
    intersect_three_planes(&planes[0], &planes[1], &planes[2])
        .to_affine_point()
        .map_err(|_| HypermeshError::UnknownClassification)
}

pub(crate) fn axis_plane_definition(point: &Point3) -> [Plane; 3] {
    [
        Plane::axis_aligned(0, point.x.clone()),
        Plane::axis_aligned(1, point.y.clone()),
        Plane::axis_aligned(2, point.z.clone()),
    ]
}

pub(super) fn axis_plane_defined_point(point: &Point3) -> PlaneDefinedPoint {
    PlaneDefinedPoint {
        planes: axis_plane_definition(point),
    }
}

#[cfg(test)]
pub(super) fn retryable_trace<T>(result: HypermeshResult<T>) -> HypermeshResult<Option<T>> {
    match result {
        Ok(value) => Ok(Some(value)),
        Err(HypermeshError::UnknownClassification) => Ok(None),
        Err(err) => Err(err),
    }
}

pub(super) fn apply_winding_transition_in_place(
    winding: &mut [i32],
    sign: i32,
    delta_w: &[i32],
) -> HypermeshResult<()> {
    if winding.len() != delta_w.len() {
        return Err(HypermeshError::UnknownClassification);
    }
    for (value, delta) in winding.iter_mut().zip(delta_w) {
        *value += sign * *delta;
    }
    Ok(())
}

pub(super) fn trace_direct_segment(
    start: &Point3,
    end: &Point3,
    start_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<TraceAxisSegmentResult> {
    let mut winding = start_wnv.to_vec();
    let Some(sort_axis) = first_changed_axis(start, end)? else {
        match point_lies_on_traced_surface(start, polygons) {
            Ok(false) => {}
            Ok(true) | Err(HypermeshError::UnknownClassification) => {
                return Err(HypermeshError::UnknownClassification);
            }
            Err(err) => return Err(err),
        }
        return Ok(TraceAxisSegmentResult {
            winding,
            valid: true,
        });
    };
    let dir_sign = if compare_real(axis_ref(end, sort_axis), axis_ref(start, sort_axis))?.is_gt() {
        1
    } else {
        -1
    };

    let mut events = Vec::new();
    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        let start_value = polygon.support.expression_at_point(start);
        let end_value = polygon.support.expression_at_point(end);
        let start_class = classify_real(&start_value)?;
        let end_class = classify_real(&end_value)?;
        if start_class == Classification::On {
            match classify_point_in_polygon(start, polygon)? {
                PolygonPointLocation::Outside => {}
                PolygonPointLocation::Boundary | PolygonPointLocation::Interior => {
                    return Err(HypermeshError::UnknownClassification);
                }
            }
            continue;
        }
        if end_class == Classification::On {
            match classify_point_in_polygon(end, polygon)? {
                PolygonPointLocation::Outside => {}
                PolygonPointLocation::Boundary | PolygonPointLocation::Interior => {
                    return Err(HypermeshError::UnknownClassification);
                }
            }
            continue;
        }
        if start_class == end_class {
            continue;
        }

        let Some(crossing) = segment_plane_crossing(start, end, &polygon.support)? else {
            continue;
        };

        let mut inside = true;
        let mut boundary_edge_count = 0;
        for edge in &polygon.edges {
            match classify_point(&crossing, edge)? {
                Classification::Positive => {
                    inside = false;
                    break;
                }
                Classification::On => boundary_edge_count += 1,
                Classification::Negative => {}
            }
        }
        if !inside {
            continue;
        }

        let normal_axis = dominant_normal_axis(&polygon.support)?;
        let normal_sign = match classify_real(axis_ref(&polygon.support.normal, normal_axis))? {
            Classification::Positive => 1,
            Classification::Negative => -1,
            Classification::On => continue,
        };
        let cross_sign = match classify_real(&(&start_value - &end_value))? {
            Classification::Positive => 1,
            Classification::Negative => -1,
            Classification::On => continue,
        };
        events.push(CrossingEvent {
            point: crossing,
            support: polygon.support.clone(),
            normal_sign,
            cross_sign,
            delta_w: polygon.delta_w.clone(),
            boundary_edge_count,
        });
    }

    let mut accepted = accepted_crossing_events(&events)?;
    sort_crossing_events(&mut accepted, sort_axis, dir_sign)?;

    for event in accepted {
        apply_winding_transition_in_place(&mut winding, event.cross_sign, &event.delta_w)?;
    }

    Ok(TraceAxisSegmentResult {
        winding,
        valid: true,
    })
}

fn accepted_crossing_events(events: &[CrossingEvent]) -> HypermeshResult<Vec<CrossingEvent>> {
    let mut accepted = Vec::new();
    let mut consumed = vec![false; events.len()];
    for index in 0..events.len() {
        if consumed[index] {
            continue;
        }
        let event = &events[index];
        // Strict events represent individual sheets. Shared-edge crossings are
        // emitted by both adjacent polygons, while vertex incidence is ambiguous.
        let mut strict = Vec::new();
        let mut edge = Vec::new();
        for (other_index, other) in events.iter().enumerate() {
            if consumed[other_index] || !crossing_events_share_transition(event, other) {
                continue;
            }
            consumed[other_index] = true;
            match other.boundary_edge_count {
                0 => strict.push(other),
                1 => edge.push(other),
                _ => return Err(HypermeshError::UnknownClassification),
            }
        }

        if edge.len() % 2 != 0 {
            return Err(HypermeshError::UnknownClassification);
        }
        let paired_edge_crossings = edge.len() / 2;
        accepted.extend(strict.into_iter().cloned());
        accepted.extend(edge.into_iter().take(paired_edge_crossings).cloned());
    }
    Ok(accepted)
}

fn crossing_events_share_transition(left: &CrossingEvent, right: &CrossingEvent) -> bool {
    left.point == right.point
        && left.support == right.support
        && left.normal_sign == right.normal_sign
        && left.cross_sign == right.cross_sign
        && left.delta_w == right.delta_w
}

pub(super) fn first_changed_axis(start: &Point3, end: &Point3) -> HypermeshResult<Option<usize>> {
    for axis in 0..3 {
        if compare_real(axis_ref(start, axis), axis_ref(end, axis))?.is_ne() {
            return Ok(Some(axis));
        }
    }
    Ok(None)
}

#[cfg(test)]
pub(super) fn trace_axis_ordered_paths(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    let mut surface_cache = Vec::new();
    let mut axis_ordered_segment_traces = Vec::new();
    trace_axis_ordered_paths_with_caches(
        start,
        end,
        winding,
        polygons,
        &mut surface_cache,
        &mut axis_ordered_segment_traces,
    )
}

fn trace_axis_ordered_paths_with_caches(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    axis_ordered_segment_traces: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
    trace_axis_ordered_paths_with_queries(
        start,
        end,
        winding,
        polygons,
        |point| {
            cached_surface_query_with(surface_cache, point, || {
                point_lies_on_traced_surface(point, polygons)
            })
        },
        |current, next, axis, attempt, polygons| {
            cached_axis_ordered_segment_trace_with(
                axis_ordered_segment_traces,
                current,
                next,
                axis,
                attempt,
                || trace_axis_segment(current, next, axis, attempt, polygons),
            )
        },
    )
}

#[cfg(test)]
pub(super) fn trace_axis_ordered_paths_with_surface_query(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    point_lies_on_surface: impl FnMut(&Point3) -> HypermeshResult<bool>,
) -> HypermeshResult<WindingNumberVector> {
    trace_axis_ordered_paths_with_queries(
        start,
        end,
        winding,
        polygons,
        point_lies_on_surface,
        |current, next, axis, attempt, polygons| {
            trace_axis_segment(current, next, axis, attempt, polygons)
        },
    )
}

pub(super) fn trace_axis_ordered_paths_with_queries(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    mut point_lies_on_surface: impl FnMut(&Point3) -> HypermeshResult<bool>,
    mut trace_segment_step: impl FnMut(
        &Point3,
        &Point3,
        usize,
        &[i32],
        &[ConvexPolygon],
    ) -> HypermeshResult<TraceAxisSegmentResult>,
) -> HypermeshResult<WindingNumberVector> {
    if start == end {
        match point_lies_on_surface(start) {
            Ok(false) => return Ok(winding.to_vec()),
            Ok(true) | Err(HypermeshError::UnknownClassification) => {
                return Err(HypermeshError::UnknownClassification);
            }
            Err(err) => return Err(err),
        }
    }

    for ordering in AXIS_ORDERINGS {
        let mut current = start.clone();
        let mut attempt = winding.to_vec();
        let mut valid = true;

        for axis in ordering {
            if compare_real(axis_ref(&current, axis), axis_ref(end, axis))?.is_ne() {
                let mut next = current.clone();
                *axis_mut(&mut next, axis) = axis_ref(end, axis).clone();
                if next != *end {
                    match point_lies_on_surface(&next) {
                        Ok(true) => {
                            valid = false;
                            break;
                        }
                        Ok(false) => {}
                        Err(HypermeshError::UnknownClassification) => {
                            valid = false;
                            break;
                        }
                        Err(err) => return Err(err),
                    }
                }
                let traced = match trace_segment_step(&current, &next, axis, &attempt, polygons) {
                    Ok(traced) => traced,
                    Err(HypermeshError::UnknownClassification) => {
                        valid = false;
                        break;
                    }
                    Err(err) => return Err(err),
                };
                attempt = traced.winding;
                valid = traced.valid;
                current = next;
                if !valid {
                    break;
                }
            }
        }

        if valid {
            return Ok(attempt);
        }
    }

    Err(HypermeshError::UnknownClassification)
}

pub(super) fn cached_axis_ordered_segment_trace_with(
    cache: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
    start: &Point3,
    end: &Point3,
    axis: usize,
    attempt: &[i32],
    trace: impl FnOnce() -> HypermeshResult<TraceAxisSegmentResult>,
) -> HypermeshResult<TraceAxisSegmentResult> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        existing.start == *start
            && existing.end == *end
            && existing.axis == axis
            && existing.attempt == attempt
    }) {
        return existing.result.clone();
    }

    let result = trace();
    cache.push(AxisOrderedSegmentTraceCacheEntry {
        start: start.clone(),
        end: end.clone(),
        axis,
        attempt: attempt.to_vec(),
        result: result.clone(),
    });
    result
}

pub(super) fn interior_box_detour_targets(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<DetourTarget>> {
    interior_box_detour_targets_with_queries(
        start,
        end,
        polygons,
        |edge_start, edge_end, polygon, axis| {
            let start_class = classify_point(edge_start, &polygon.support)?;
            let end_class = classify_point(edge_end, &polygon.support)?;
            if start_class == Classification::On {
                return Ok(Some(edge_start.clone()));
            }
            if end_class == Classification::On {
                return Ok(Some(edge_end.clone()));
            }
            segment_plane_crossing(edge_start, edge_end, &polygon.support).and_then(|crossing| {
                if let Some(crossing) = crossing {
                    if !point_strictly_between_axis(&crossing, edge_start, edge_end, axis)? {
                        return Ok(None);
                    }
                    Ok(Some(crossing))
                } else {
                    Ok(None)
                }
            })
        },
        classify_point_in_polygon,
        strict_aabb_targets,
    )
}

pub(super) fn interior_box_detour_targets_with_queries(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    mut crossing_for: impl FnMut(
        &Point3,
        &Point3,
        &ConvexPolygon,
        usize,
    ) -> HypermeshResult<Option<Point3>>,
    mut classify_point_on_polygon: impl FnMut(
        &Point3,
        &ConvexPolygon,
    ) -> HypermeshResult<PolygonPointLocation>,
    mut build: impl FnMut(&Aabb) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<Vec<DetourTarget>> {
    let (intervals, saw_unknown) = interior_box_axis_intervals_with_surface_queries(
        start,
        end,
        polygons,
        &mut crossing_for,
        &mut classify_point_on_polygon,
    )?;
    let targets = collect_detour_targets_from_axis_intervals(&intervals, |bounds| build(bounds))?;
    detour_target_family_result_from_targets(targets, saw_unknown)
}

pub(super) fn interior_box_axis_intervals_with_surface_queries(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    crossing_for: &mut impl FnMut(
        &Point3,
        &Point3,
        &ConvexPolygon,
        usize,
    ) -> HypermeshResult<Option<Point3>>,
    classify_point_on_polygon: &mut impl FnMut(
        &Point3,
        &ConvexPolygon,
    ) -> HypermeshResult<PolygonPointLocation>,
) -> HypermeshResult<(Vec<Vec<(Real, Real)>>, bool)> {
    let mut intervals = vec![Vec::new(), Vec::new(), Vec::new()];
    let mut saw_unknown = false;
    for (axis, axis_intervals) in intervals.iter_mut().enumerate() {
        let start_value = axis_ref(start, axis);
        let end_value = axis_ref(end, axis);
        if compare_real(start_value, end_value)?.is_eq() {
            axis_intervals.push((start_value.clone(), end_value.clone()));
            continue;
        }

        let mut cuts = Vec::new();
        push_unique_ordered_real(&mut cuts, start_value.clone())?;
        push_unique_ordered_real(&mut cuts, end_value.clone())?;
        for polygon in polygons {
            for vertex in polygon.vertices()? {
                let value = axis_ref(&vertex, axis);
                if value_strictly_between(value, start_value, end_value)? {
                    push_unique_ordered_real(&mut cuts, value.clone())?;
                }
            }
            saw_unknown |= add_axis_box_surface_cuts_with_queries(
                &mut cuts,
                start,
                end,
                polygon,
                axis,
                start_value,
                end_value,
                crossing_for,
                classify_point_on_polygon,
            )?;
        }

        for endpoints in cuts.windows(2) {
            axis_intervals.push((endpoints[0].clone(), endpoints[1].clone()));
        }
    }

    Ok((intervals, saw_unknown))
}

fn matching_interior_box_axis_intervals_bucket_indices<'a>(
    buckets: &'a [InteriorBoxAxisIntervalsBucket],
    start: &Point3,
    end: &Point3,
) -> Option<&'a [usize]> {
    buckets
        .iter()
        .rev()
        .find(|existing| {
            (existing.start == *start && existing.end == *end)
                || (existing.start == *end && existing.end == *start)
        })
        .map(|bucket| bucket.indices.as_slice())
}

fn push_interior_box_axis_intervals_bucket_entry(
    buckets: &mut Vec<InteriorBoxAxisIntervalsBucket>,
    start: &Point3,
    end: &Point3,
    index: usize,
) {
    if let Some(bucket) = buckets.iter_mut().find(|existing| {
        (existing.start == *start && existing.end == *end)
            || (existing.start == *end && existing.end == *start)
    }) {
        bucket.indices.push(index);
        return;
    }
    buckets.push(InteriorBoxAxisIntervalsBucket {
        start: start.clone(),
        end: end.clone(),
        indices: vec![index],
    });
}

pub(super) fn cached_interior_box_axis_intervals_with_surface_queries(
    cache: &mut InteriorBoxAxisIntervalsCache,
    start: &Point3,
    end: &Point3,
    query: impl FnOnce() -> HypermeshResult<(Vec<Vec<(Real, Real)>>, bool)>,
) -> HypermeshResult<(Vec<Vec<(Real, Real)>>, bool)> {
    if let Some(entry) =
        matching_interior_box_axis_intervals_bucket_indices(&cache.buckets, start, end).and_then(
            |indices| {
                indices
                    .iter()
                    .rev()
                    .find_map(|index| cache.entries.get(*index))
            },
        )
    {
        return Ok((entry.intervals.clone(), entry.saw_unknown));
    }

    let (intervals, saw_unknown) = query()?;
    cache.entries.push(InteriorBoxAxisIntervalsCacheEntry {
        start: start.clone(),
        end: end.clone(),
        intervals: intervals.clone(),
        saw_unknown,
    });
    let index = cache.entries.len() - 1;
    push_interior_box_axis_intervals_bucket_entry(&mut cache.buckets, start, end, index);
    Ok((intervals, saw_unknown))
}

pub(super) fn aabb_from_axis_intervals(intervals: [&(Real, Real); 3]) -> HypermeshResult<Aabb> {
    let mut min = Point3::origin();
    let mut max = Point3::origin();
    for (axis, (start, end)) in intervals.into_iter().enumerate() {
        if compare_real(start, end)?.is_le() {
            *axis_mut(&mut min, axis) = start.clone();
            *axis_mut(&mut max, axis) = end.clone();
        } else {
            *axis_mut(&mut min, axis) = end.clone();
            *axis_mut(&mut max, axis) = start.clone();
        }
    }
    Ok(Aabb::new(min, max))
}

pub(super) fn strict_aabb_targets(bounds: &Aabb) -> HypermeshResult<Vec<DetourTarget>> {
    let mut cursor = StrictAabbTargetCursor::new(bounds)?;
    let mut targets = Vec::new();
    while let Some(batch) = cursor.next_batch()? {
        for target in batch {
            push_unique_detour_target(&mut targets, target);
        }
    }
    detour_target_family_result_from_targets(targets, cursor.saw_unknown)
}

#[allow(dead_code)]
fn search_strict_aabb_targets_progressively_with_direct_ranking<K: Ord>(
    bounds: &Aabb,
    rank_direct: &mut impl FnMut(&DetourTarget) -> HypermeshResult<K>,
    evaluate: &mut impl FnMut(DetourTarget) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking(
        bounds,
        |bounds, halfspaces, report, saw_unknown| {
            halfspace_cell_seed_families_from_optional_report(
                bounds,
                halfspaces,
                report,
                saw_unknown,
            )
        },
        rank_direct,
        evaluate,
    )
}

pub(super) struct ProgressiveStrictAabbSearchOutcome {
    pub(super) result: HypermeshResult<bool>,
    pub(super) exhausted_families: Option<StrictAabbTargetFamilies>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum StrictAabbTargetCursorStage {
    FrontDirect,
    Shifted,
    DeferredDirect,
    Done,
}

pub(super) struct StrictAabbTargetCursor {
    bounds: Aabb,
    halfspaces: Vec<LimitPlane3>,
    report: Option<hyperlimit::HalfspaceFeasibilityReport>,
    seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    shifted_seeds: Option<Vec<Point3>>,
    next_shifted_seed: usize,
    certified_direct_target_points: Vec<Point3>,
    emitted_targets: Vec<DetourTarget>,
    pub(super) saw_unknown: bool,
    pub(super) stage: StrictAabbTargetCursorStage,
}

impl StrictAabbTargetCursor {
    pub(super) fn new(bounds: &Aabb) -> HypermeshResult<Self> {
        let halfspaces = aabb_core_halfspaces(bounds)?;
        let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces)?;
        let feasible = report
            .as_ref()
            .is_none_or(|report| report.status == HalfspaceFeasibility::Feasible);
        let (seeds, shifted_vertices, shifted_geometry_seeds) = if feasible {
            halfspace_cell_seed_families_from_optional_report(
                bounds,
                &halfspaces,
                report.as_ref(),
                &mut saw_unknown,
            )?
        } else {
            (Vec::new(), Vec::new(), Vec::new())
        };
        let (seeds, shifted_vertices, shifted_geometry_seeds) =
            dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);
        Ok(Self {
            bounds: bounds.clone(),
            halfspaces,
            report,
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
            shifted_seeds: None,
            next_shifted_seed: 0,
            certified_direct_target_points: Vec::new(),
            emitted_targets: Vec::new(),
            saw_unknown,
            stage: if feasible {
                StrictAabbTargetCursorStage::FrontDirect
            } else {
                StrictAabbTargetCursorStage::Done
            },
        })
    }

    pub(super) fn next_batch(&mut self) -> HypermeshResult<Option<Vec<DetourTarget>>> {
        loop {
            let batch = match self.stage {
                StrictAabbTargetCursorStage::FrontDirect => {
                    self.stage = StrictAabbTargetCursorStage::DeferredDirect;
                    self.build_direct_batch(
                        0,
                        self.seeds.len().min(DIRECT_TARGET_RANK_REFINEMENT_LIMIT),
                    )?
                }
                StrictAabbTargetCursorStage::Shifted => {
                    let (batch, exhausted) = self.build_shifted_batch()?;
                    if exhausted {
                        self.stage = StrictAabbTargetCursorStage::Done;
                    }
                    batch
                }
                StrictAabbTargetCursorStage::DeferredDirect => {
                    self.stage = StrictAabbTargetCursorStage::Shifted;
                    self.build_direct_batch(
                        self.seeds.len().min(DIRECT_TARGET_RANK_REFINEMENT_LIMIT),
                        self.seeds.len(),
                    )?
                }
                StrictAabbTargetCursorStage::Done => return Ok(None),
            };
            if !batch.is_empty() {
                return Ok(Some(batch));
            }
        }
    }

    fn build_direct_batch(
        &mut self,
        start_index: usize,
        end_index: usize,
    ) -> HypermeshResult<Vec<DetourTarget>> {
        let mut batch = Vec::new();
        let seeds = self.seeds[start_index..end_index].to_vec();
        for seed in &seeds {
            let target = match build_detour_target(
                seed,
                &self.halfspaces,
                active_planes_from_optional_report(self.report.as_ref(), seed),
                false,
            ) {
                Ok(target) => target,
                Err(HypermeshError::UnknownClassification) => {
                    self.saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
            if !target.uncertified_definition_fallback
                && !self.certified_direct_target_points.contains(&target.point)
            {
                self.certified_direct_target_points
                    .push(target.point.clone());
            }
            self.push_target(&mut batch, target);
        }
        Ok(batch)
    }

    fn build_shifted_batch(&mut self) -> HypermeshResult<(Vec<DetourTarget>, bool)> {
        if self.shifted_seeds.is_none() {
            let report_witness = self
                .report
                .as_ref()
                .and_then(|report| report.witness.as_ref());
            let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
                detour_shifted_seed_families(
                    report_witness,
                    &self.certified_direct_target_points,
                    self.seeds.clone(),
                    std::mem::take(&mut self.shifted_vertices),
                    std::mem::take(&mut self.shifted_geometry_seeds),
                );
            let mut shifted_seeds = strict_shift_seeds;
            shifted_seeds.extend(shifted_vertices);
            shifted_seeds.extend(shifted_geometry_seeds);
            self.shifted_seeds = Some(shifted_seeds);
        }

        loop {
            let Some(seed) = self
                .shifted_seeds
                .as_ref()
                .and_then(|seeds| seeds.get(self.next_shifted_seed))
                .cloned()
            else {
                return Ok((Vec::new(), true));
            };
            self.next_shifted_seed += 1;
            let shifted_witnesses = match shifted_halfspace_cell_witnesses_from_seed(
                &self.bounds,
                &self.halfspaces,
                &seed,
            ) {
                Ok(witnesses) => witnesses,
                Err(HypermeshError::UnknownClassification) => {
                    self.saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
            let mut batch = Vec::new();
            for witness in &shifted_witnesses {
                let target = match build_detour_target_from_shifted_witness(witness) {
                    Ok(target) => target,
                    Err(HypermeshError::UnknownClassification) => {
                        self.saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
                self.push_target(&mut batch, target);
            }
            if !batch.is_empty() {
                return Ok((batch, false));
            }
        }
    }

    fn push_target(&mut self, batch: &mut Vec<DetourTarget>, target: DetourTarget) {
        if self.emitted_targets.iter().any(|existing| {
            existing.point == target.point
                && definition_families_match_as_sets(&existing.definitions, &target.definitions)
        }) {
            return;
        }
        self.emitted_targets.push(target.clone());
        batch.push(target);
    }
}

pub(super) struct InteriorBoxDetourTargetCursor {
    pub(super) bounds: Vec<Aabb>,
    next_bounds: usize,
    current: Option<StrictAabbTargetCursor>,
    emitted_targets: Vec<DetourTarget>,
    pub(super) saw_unknown: bool,
}

impl InteriorBoxDetourTargetCursor {
    pub(super) fn new(
        start: &Point3,
        end: &Point3,
        polygons: &[ConvexPolygon],
        arrangement_planes: &[Plane],
        trace_bounds: Option<&Aabb>,
    ) -> HypermeshResult<Self> {
        let (start_cell, end_cell) = if arrangement_planes.is_empty() {
            (None, None)
        } else {
            (
                optional_detour_arrangement_cell(start, arrangement_planes)?,
                optional_detour_arrangement_cell(end, arrangement_planes)?,
            )
        };
        let (mut bounds, mut saw_unknown) = interior_detour_candidate_bounds(
            start,
            end,
            polygons,
            arrangement_planes,
            start_cell.as_deref(),
            end_cell.as_deref(),
        )?;
        if let Some(trace_bounds) = trace_bounds {
            if bounds.is_empty()
                && strict_aabb_arrangement_cell(trace_bounds, arrangement_planes)?.is_none()
                && !bounds.iter().any(|existing| existing == trace_bounds)
            {
                bounds.push(trace_bounds.clone());
            }
            let (trace_candidates, trace_unknown) = interior_detour_candidate_bounds(
                &trace_bounds.min,
                &trace_bounds.max,
                polygons,
                arrangement_planes,
                start_cell.as_deref(),
                end_cell.as_deref(),
            )?;
            saw_unknown |= trace_unknown;
            for candidate in trace_candidates {
                if !bounds.contains(&candidate) {
                    bounds.push(candidate);
                }
            }
        }
        Ok(Self {
            bounds,
            next_bounds: 0,
            current: None,
            emitted_targets: Vec::new(),
            saw_unknown,
        })
    }

    pub(super) fn next_batch(&mut self) -> HypermeshResult<Option<Vec<DetourTarget>>> {
        loop {
            if let Some(current) = self.current.as_mut() {
                match current.next_batch() {
                    Ok(Some(batch)) => {
                        let mut unique = Vec::new();
                        for target in batch {
                            if let Some(existing) =
                                self.emitted_targets.iter_mut().find(|existing| {
                                    existing.point == target.point
                                        && definition_families_match_as_sets(
                                            &existing.definitions,
                                            &target.definitions,
                                        )
                                })
                            {
                                if existing.uncertified_definition_fallback
                                    && !target.uncertified_definition_fallback
                                {
                                    existing.uncertified_definition_fallback = false;
                                    unique.push(target);
                                }
                                continue;
                            }
                            self.emitted_targets.push(target.clone());
                            unique.push(target);
                        }
                        if !unique.is_empty() {
                            return Ok(Some(unique));
                        }
                    }
                    Ok(None) => {
                        self.saw_unknown |= current.saw_unknown;
                        self.current = None;
                    }
                    Err(HypermeshError::UnknownClassification) => {
                        self.saw_unknown = true;
                        self.current = None;
                    }
                    Err(err) => return Err(err),
                }
                continue;
            }
            let Some(bounds) = self.bounds.get(self.next_bounds) else {
                return Ok(None);
            };
            self.next_bounds += 1;
            match StrictAabbTargetCursor::new(bounds) {
                Ok(cursor) => self.current = Some(cursor),
                Err(HypermeshError::UnknownClassification) => self.saw_unknown = true,
                Err(err) => return Err(err),
            }
        }
    }
}

fn interior_detour_candidate_bounds(
    domain_start: &Point3,
    domain_end: &Point3,
    polygons: &[ConvexPolygon],
    arrangement_planes: &[Plane],
    start_cell: Option<&[Classification]>,
    end_cell: Option<&[Classification]>,
) -> HypermeshResult<(Vec<Aabb>, bool)> {
    let (intervals, saw_unknown) = interior_box_axis_intervals_with_surface_queries(
        domain_start,
        domain_end,
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
            segment_plane_crossing(edge_start, edge_end, &polygon.support).and_then(|crossing| {
                if let Some(crossing) = crossing {
                    if !point_strictly_between_axis(&crossing, edge_start, edge_end, axis)? {
                        return Ok(None);
                    }
                    Ok(Some(crossing))
                } else {
                    Ok(None)
                }
            })
        },
        &mut |crossing, polygon| classify_point_in_polygon(crossing, polygon),
    )?;
    let mut bounds = Vec::new();
    for x in &intervals[0] {
        for y in &intervals[1] {
            for z in &intervals[2] {
                let candidate = aabb_from_axis_intervals([x, y, z])?;
                let single_cell = strict_aabb_arrangement_cell(&candidate, arrangement_planes)?;
                if single_cell.as_ref().is_some_and(|cell| {
                    start_cell == Some(cell.as_slice()) || end_cell == Some(cell.as_slice())
                }) {
                    continue;
                }
                bounds.push(candidate);
            }
        }
    }
    Ok((bounds, saw_unknown))
}

pub(super) struct InteriorBoxDetourTargetBatchCacheEntry {
    start: Point3,
    end: Point3,
    trace_bounds: Option<Aabb>,
    pub(super) cursor: InteriorBoxDetourTargetCursor,
    batches: Vec<Vec<DetourTarget>>,
    exhausted: bool,
}

#[derive(Default)]
pub(super) struct InteriorBoxDetourTargetBatchCache {
    pub(super) entries: Vec<InteriorBoxDetourTargetBatchCacheEntry>,
}

impl InteriorBoxDetourTargetBatchCache {
    pub(super) fn batch_for(
        &mut self,
        start: &Point3,
        end: &Point3,
        batch_index: usize,
        polygons: &[ConvexPolygon],
        arrangement_planes: &[Plane],
        trace_bounds: Option<&Aabb>,
    ) -> HypermeshResult<Option<Vec<DetourTarget>>> {
        let entry_index = if let Some(index) = self.entries.iter().position(|entry| {
            entry.start == *start
                && entry.end == *end
                && entry.trace_bounds.as_ref() == trace_bounds
        }) {
            index
        } else {
            self.entries.push(InteriorBoxDetourTargetBatchCacheEntry {
                start: start.clone(),
                end: end.clone(),
                trace_bounds: trace_bounds.cloned(),
                cursor: InteriorBoxDetourTargetCursor::new(
                    start,
                    end,
                    polygons,
                    arrangement_planes,
                    trace_bounds,
                )?,
                batches: Vec::new(),
                exhausted: false,
            });
            self.entries.len() - 1
        };
        let entry = &mut self.entries[entry_index];
        while entry.batches.len() <= batch_index && !entry.exhausted {
            match entry.cursor.next_batch()? {
                Some(batch) => entry.batches.push(batch),
                None => entry.exhausted = true,
            }
        }
        if let Some(batch) = entry.batches.get(batch_index) {
            Ok(Some(batch.clone()))
        } else if entry.cursor.saw_unknown {
            Err(HypermeshError::UnknownClassification)
        } else {
            Ok(None)
        }
    }
}

#[cfg(test)]
pub(super) fn search_strict_aabb_targets_progressively_with_seed_families(
    bounds: &Aabb,
    mut seed_families_for: impl FnMut(
        &Aabb,
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
        &mut bool,
    ) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)>,
    evaluate: &mut impl FnMut(DetourTarget) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking(
        bounds,
        &mut seed_families_for,
        &mut |_| Ok(()),
        evaluate,
    )
}

fn search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking<K: Ord>(
    bounds: &Aabb,
    mut seed_families_for: impl FnMut(
        &Aabb,
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
        &mut bool,
    ) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)>,
    rank_direct: &mut impl FnMut(&DetourTarget) -> HypermeshResult<K>,
    evaluate: &mut impl FnMut(DetourTarget) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking_outcome(
        bounds,
        &mut seed_families_for,
        rank_direct,
        evaluate,
    )
    .result
}

pub(super) fn search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking_outcome<
    K: Ord,
>(
    bounds: &Aabb,
    mut seed_families_for: impl FnMut(
        &Aabb,
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
        &mut bool,
    ) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)>,
    rank_direct: &mut impl FnMut(&DetourTarget) -> HypermeshResult<K>,
    evaluate: &mut impl FnMut(DetourTarget) -> HypermeshResult<bool>,
) -> ProgressiveStrictAabbSearchOutcome {
    let halfspaces = match aabb_core_halfspaces(bounds) {
        Ok(halfspaces) => halfspaces,
        Err(err) => {
            return ProgressiveStrictAabbSearchOutcome {
                result: Err(err),
                exhausted_families: None,
            };
        }
    };
    let (report, mut saw_unknown) = match optional_halfspace_feasibility_report(&halfspaces) {
        Ok(report) => report,
        Err(err) => {
            return ProgressiveStrictAabbSearchOutcome {
                result: Err(err),
                exhausted_families: None,
            };
        }
    };
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return ProgressiveStrictAabbSearchOutcome {
            result: if saw_unknown {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(false)
            },
            exhausted_families: Some(StrictAabbTargetFamilies {
                direct_targets: Vec::new(),
                shifted_targets: Vec::new(),
                saw_unknown,
            }),
        };
    }

    let mut exhausted_direct_targets = Vec::new();
    let mut exhausted_shifted_targets = Vec::new();

    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        match seed_families_for(bounds, &halfspaces, report.as_ref(), &mut saw_unknown) {
            Ok(families) => families,
            Err(err) => {
                return ProgressiveStrictAabbSearchOutcome {
                    result: Err(err),
                    exhausted_families: None,
                };
            }
        };
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    let refinement_len = seeds.len().min(DIRECT_TARGET_RANK_REFINEMENT_LIMIT);
    let mut certified_direct_target_points = Vec::new();
    let mut front_direct_targets = Vec::with_capacity(refinement_len);
    for seed in seeds.iter().take(refinement_len) {
        let target = match build_detour_target(
            seed,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), seed),
            false,
        ) {
            Ok(target) => target,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => {
                return ProgressiveStrictAabbSearchOutcome {
                    result: Err(err),
                    exhausted_families: None,
                };
            }
        };
        if !target.uncertified_definition_fallback
            && !certified_direct_target_points.contains(&target.point)
        {
            certified_direct_target_points.push(target.point.clone());
        }
        push_unique_detour_target(&mut exhausted_direct_targets, target.clone());
        push_unique_detour_target(&mut front_direct_targets, target);
    }
    let mut ranked_direct_targets = Vec::with_capacity(front_direct_targets.len());
    for (index, target) in front_direct_targets.into_iter().enumerate() {
        let (rank_missing, rank) = match rank_direct(&target) {
            Ok(rank) => (0u8, Some(rank)),
            Err(HypermeshError::UnknownClassification) => (1u8, None),
            Err(err) => {
                return ProgressiveStrictAabbSearchOutcome {
                    result: Err(err),
                    exhausted_families: None,
                };
            }
        };
        ranked_direct_targets.push((rank_missing, rank, index, target));
    }
    ranked_direct_targets.sort_by(|left, right| {
        left.0
            .cmp(&right.0)
            .then_with(|| left.1.cmp(&right.1))
            .then_with(|| left.2.cmp(&right.2))
    });
    for (_, _, _, target) in ranked_direct_targets {
        match evaluate(target.clone()) {
            Ok(true) => {
                return ProgressiveStrictAabbSearchOutcome {
                    result: Ok(true),
                    exhausted_families: None,
                };
            }
            Ok(false) => {
                if target.uncertified_definition_fallback {
                    saw_unknown = true;
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => {
                return ProgressiveStrictAabbSearchOutcome {
                    result: Err(err),
                    exhausted_families: None,
                };
            }
        }
    }

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        detour_shifted_seed_families(
            report_witness,
            &certified_direct_target_points,
            seeds.clone(),
            shifted_vertices,
            shifted_geometry_seeds,
        );
    let shifted_witnesses = match shifted_halfspace_witness_family_or_empty(
        {
            let mut shifted_witnesses = Vec::new();
            let shifted_result = extend_shifted_halfspace_seed_families_backtracking_unknown(
                &mut shifted_witnesses,
                [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds],
                |seed| shifted_halfspace_cell_witnesses_from_seed(bounds, &halfspaces, seed),
            );
            match shifted_result {
                Ok(()) => Ok(shifted_witnesses),
                Err(err) => Err(err),
            }
        },
        &mut saw_unknown,
    ) {
        Ok(witnesses) => witnesses,
        Err(err) => {
            return ProgressiveStrictAabbSearchOutcome {
                result: Err(err),
                exhausted_families: None,
            };
        }
    };
    let mut unique_shifted_targets = Vec::new();
    for witness in &shifted_witnesses {
        let target = match build_detour_target_from_shifted_witness(witness) {
            Ok(target) => target,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => {
                return ProgressiveStrictAabbSearchOutcome {
                    result: Err(err),
                    exhausted_families: None,
                };
            }
        };
        push_unique_detour_target(&mut exhausted_shifted_targets, target.clone());
        push_unique_detour_target(&mut unique_shifted_targets, target);
    }
    for target in unique_shifted_targets {
        match evaluate(target.clone()) {
            Ok(true) => {
                return ProgressiveStrictAabbSearchOutcome {
                    result: Ok(true),
                    exhausted_families: None,
                };
            }
            Ok(false) => {
                if target.uncertified_definition_fallback {
                    saw_unknown = true;
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => {
                return ProgressiveStrictAabbSearchOutcome {
                    result: Err(err),
                    exhausted_families: None,
                };
            }
        }
    }

    let mut deferred_direct_targets = Vec::new();
    for seed in seeds.iter().skip(refinement_len) {
        let target = match build_detour_target(
            seed,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), seed),
            false,
        ) {
            Ok(target) => target,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => {
                return ProgressiveStrictAabbSearchOutcome {
                    result: Err(err),
                    exhausted_families: None,
                };
            }
        };
        push_unique_detour_target(&mut exhausted_direct_targets, target.clone());
        push_unique_detour_target(&mut deferred_direct_targets, target);
    }
    for target in deferred_direct_targets {
        match evaluate(target.clone()) {
            Ok(true) => {
                return ProgressiveStrictAabbSearchOutcome {
                    result: Ok(true),
                    exhausted_families: None,
                };
            }
            Ok(false) => {
                if target.uncertified_definition_fallback {
                    saw_unknown = true;
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => {
                return ProgressiveStrictAabbSearchOutcome {
                    result: Err(err),
                    exhausted_families: None,
                };
            }
        }
    }

    let result = if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(false)
    };

    ProgressiveStrictAabbSearchOutcome {
        result,
        exhausted_families: Some(StrictAabbTargetFamilies {
            direct_targets: exhausted_direct_targets,
            shifted_targets: exhausted_shifted_targets,
            saw_unknown,
        }),
    }
}

pub(super) fn evaluate_strict_aabb_target_families_with_direct_ranking<K: Ord>(
    families: StrictAabbTargetFamilies,
    rank_direct: &mut impl FnMut(&DetourTarget) -> HypermeshResult<K>,
    evaluate: &mut impl FnMut(DetourTarget) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    let StrictAabbTargetFamilies {
        direct_targets,
        shifted_targets,
        mut saw_unknown,
    } = families;
    let refinement_len = direct_targets
        .len()
        .min(DIRECT_TARGET_RANK_REFINEMENT_LIMIT);
    let mut ranked_direct_targets = Vec::with_capacity(refinement_len);
    let mut deferred_direct_targets =
        Vec::with_capacity(direct_targets.len().saturating_sub(refinement_len));
    for (index, target) in direct_targets.into_iter().enumerate() {
        if index < refinement_len {
            let (rank_missing, rank) = match rank_direct(&target) {
                Ok(rank) => (0u8, Some(rank)),
                Err(HypermeshError::UnknownClassification) => (1u8, None),
                Err(err) => return Err(err),
            };
            ranked_direct_targets.push((rank_missing, rank, index, target));
        } else {
            deferred_direct_targets.push(target);
        }
    }
    ranked_direct_targets.sort_by(|left, right| {
        left.0
            .cmp(&right.0)
            .then_with(|| left.1.cmp(&right.1))
            .then_with(|| left.2.cmp(&right.2))
    });
    for (_, _, _, target) in ranked_direct_targets {
        match evaluate(target.clone()) {
            Ok(true) => return Ok(true),
            Ok(false) => {
                if target.uncertified_definition_fallback {
                    saw_unknown = true;
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    for target in shifted_targets {
        match evaluate(target.clone()) {
            Ok(true) => return Ok(true),
            Ok(false) => {
                if target.uncertified_definition_fallback {
                    saw_unknown = true;
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    for target in deferred_direct_targets {
        match evaluate(target.clone()) {
            Ok(true) => return Ok(true),
            Ok(false) => {
                if target.uncertified_definition_fallback {
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
fn strict_aabb_target_families_with_seed_families(
    bounds: &Aabb,
    mut seed_families_for: impl FnMut(
        &Aabb,
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
        &mut bool,
    ) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)>,
) -> HypermeshResult<StrictAabbTargetFamilies> {
    let halfspaces = aabb_core_halfspaces(bounds)?;
    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(StrictAabbTargetFamilies {
            direct_targets: Vec::new(),
            shifted_targets: Vec::new(),
            saw_unknown,
        });
    }

    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        seed_families_for(bounds, &halfspaces, report.as_ref(), &mut saw_unknown)?;
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    let mut certified_direct_target_points = Vec::new();
    let mut direct_targets = Vec::new();
    for seed in &seeds {
        let target = match build_detour_target(
            seed,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), seed),
            false,
        ) {
            Ok(target) => target,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if !target.uncertified_definition_fallback {
            if !certified_direct_target_points
                .iter()
                .any(|existing| *existing == target.point)
            {
                certified_direct_target_points.push(target.point.clone());
            }
        }
        push_unique_detour_target(&mut direct_targets, target);
    }

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        detour_shifted_seed_families(
            report_witness,
            &certified_direct_target_points,
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );
    let shifted_witnesses = shifted_halfspace_witness_family_or_empty(
        {
            let mut shifted_witnesses = Vec::new();
            extend_shifted_halfspace_seed_families_backtracking_unknown(
                &mut shifted_witnesses,
                [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds],
                |seed| shifted_halfspace_cell_witnesses_from_seed(bounds, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;

    let mut shifted_targets = Vec::new();
    for witness in &shifted_witnesses {
        match build_detour_target_from_shifted_witness(witness) {
            Ok(target) => {
                push_unique_detour_target(&mut shifted_targets, target);
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    Ok(StrictAabbTargetFamilies {
        direct_targets,
        shifted_targets,
        saw_unknown,
    })
}

#[cfg(test)]
pub(super) fn cached_strict_aabb_target_families_with_seed_families(
    cache: &mut StrictAabbTargetFamilyCache,
    bounds: &Aabb,
    mut seed_families_for: impl FnMut(
        &Aabb,
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
        &mut bool,
    ) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)>,
) -> HypermeshResult<StrictAabbTargetFamilies> {
    if let Some(families) = cached_strict_aabb_target_families(cache, bounds) {
        return families;
    }
    let families = strict_aabb_target_families_with_seed_families(bounds, &mut seed_families_for);
    cache.entries.push(StrictAabbTargetFamilyCacheEntry {
        bounds: bounds.clone(),
        families: families.clone(),
    });
    let index = cache.entries.len() - 1;
    push_strict_aabb_target_family_bucket_entry(&mut cache.buckets, bounds, index);
    families
}

fn matching_strict_aabb_target_family_bucket_indices<'a>(
    buckets: &'a [StrictAabbTargetFamilyBucket],
    bounds: &Aabb,
) -> Option<&'a [usize]> {
    buckets
        .iter()
        .rev()
        .find(|existing| existing.bounds == *bounds)
        .map(|bucket| bucket.indices.as_slice())
}

pub(super) fn push_strict_aabb_target_family_bucket_entry(
    buckets: &mut Vec<StrictAabbTargetFamilyBucket>,
    bounds: &Aabb,
    index: usize,
) {
    if let Some(bucket) = buckets
        .iter_mut()
        .find(|existing| existing.bounds == *bounds)
    {
        bucket.indices.push(index);
        return;
    }
    buckets.push(StrictAabbTargetFamilyBucket {
        bounds: bounds.clone(),
        indices: vec![index],
    });
}

pub(super) fn cached_strict_aabb_target_families(
    cache: &StrictAabbTargetFamilyCache,
    bounds: &Aabb,
) -> Option<HypermeshResult<StrictAabbTargetFamilies>> {
    matching_strict_aabb_target_family_bucket_indices(&cache.buckets, bounds).and_then(|indices| {
        indices
            .iter()
            .rev()
            .find_map(|index| cache.entries.get(*index))
            .map(|entry| entry.families.clone())
    })
}

pub(super) fn detour_shifted_seed_families(
    report_witness: Option<&Point3>,
    certified_direct_target_points: &[Point3],
    seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
) -> (Vec<Point3>, Vec<Point3>, Vec<Point3>) {
    let _ = certified_direct_target_points;

    shifted_halfspace_seed_families_with_report_seed(
        report_witness,
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    )
}

#[cfg(test)]
pub(super) fn strict_aabb_targets_with_seed_families(
    bounds: &Aabb,
    mut seed_families_for: impl FnMut(
        &Aabb,
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
        &mut bool,
    ) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)>,
) -> HypermeshResult<Vec<DetourTarget>> {
    let families = strict_aabb_target_families_with_seed_families(bounds, &mut seed_families_for)?;
    let mut targets = families.direct_targets;
    targets.extend(families.shifted_targets);
    finalize_detour_target_family(&mut targets, families.saw_unknown)?;
    Ok(targets)
}

pub(super) fn build_detour_target(
    point: &Point3,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    inherited_uncertified_definition_fallback: bool,
) -> HypermeshResult<DetourTarget> {
    let (definitions, uncertified_definition_fallback) = probe_definitions_or_axis(
        point,
        probe_definitions_from_active_halfspaces(point, halfspaces, active_planes, &[]),
    )?;
    Ok(DetourTarget {
        point: point.clone(),
        definitions,
        uncertified_definition_fallback: inherited_uncertified_definition_fallback
            || uncertified_definition_fallback,
    })
}

pub(super) fn build_detour_target_from_shifted_witness(
    witness: &ShiftedHalfspaceWitness,
) -> HypermeshResult<DetourTarget> {
    let mut definitions = Vec::new();
    let mut saw_unknown = false;

    for family in &witness.families {
        match probe_definitions_from_active_halfspaces(
            &witness.point,
            &family.halfspaces,
            family.active_planes,
            &[],
        ) {
            Ok(found) => {
                saw_unknown |= found.saw_unknown;
                extend_unique_definition_families(&mut definitions, found.definitions);
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    let used_axis_fallback = definitions.is_empty() && saw_unknown;
    if used_axis_fallback {
        definitions.push(axis_plane_definition(&witness.point));
    }

    Ok(DetourTarget {
        point: witness.point.clone(),
        definitions,
        uncertified_definition_fallback: witness.uncertified_definition_fallback
            || used_axis_fallback,
    })
}

pub(super) fn push_unique_detour_target(
    targets: &mut Vec<DetourTarget>,
    target: DetourTarget,
) -> bool {
    if let Some(existing) = targets
        .iter_mut()
        .find(|existing| existing.point == target.point)
    {
        let incoming_definitions = target.definitions;
        let incoming_is_fallback = target.uncertified_definition_fallback;
        let existing_covered_by_incoming = existing.definitions.iter().all(|existing_definition| {
            incoming_definitions.iter().any(|incoming_definition| {
                definition_planes_match_as_sets(existing_definition, incoming_definition)
            })
        });
        let mut introduced_new_definition = false;
        for definition in incoming_definitions {
            if !existing
                .definitions
                .iter()
                .any(|candidate| definition_planes_match_as_sets(candidate, &definition))
            {
                existing.definitions.push(definition);
                introduced_new_definition = true;
            }
        }
        if incoming_is_fallback {
            if introduced_new_definition {
                existing.uncertified_definition_fallback = true;
                true
            } else {
                false
            }
        } else {
            if existing_covered_by_incoming {
                existing.uncertified_definition_fallback = false;
            }
            false
        }
    } else {
        let introduced_uncertified_state = target.uncertified_definition_fallback;
        targets.push(target);
        introduced_uncertified_state
    }
}

pub(super) fn detour_target_family_result_from_targets(
    mut targets: Vec<DetourTarget>,
    saw_unknown: bool,
) -> HypermeshResult<Vec<DetourTarget>> {
    finalize_detour_target_family(&mut targets, saw_unknown)?;
    Ok(targets)
}

pub(super) fn extend_unique_definition_families(
    definitions: &mut Vec<[Plane; 3]>,
    fresh: Vec<[Plane; 3]>,
) {
    for definition in fresh {
        if !definitions
            .iter()
            .any(|existing| definition_planes_match_as_sets(existing, &definition))
        {
            definitions.push(definition);
        }
    }
}

#[cfg(test)]
pub(super) fn extend_detour_target_builds_backtracking_unknown<'a, T: 'a>(
    targets: &mut Vec<DetourTarget>,
    candidates: impl IntoIterator<Item = &'a T>,
    mut build: impl FnMut(&'a T) -> HypermeshResult<DetourTarget>,
) -> HypermeshResult<()> {
    let mut saw_hard_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(target) => {
                push_unique_detour_target(targets, target);
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_detour_target_family(targets, saw_hard_unknown)
}

#[cfg(test)]
pub(super) fn extend_detour_target_families_backtracking_unknown(
    targets: &mut Vec<DetourTarget>,
    families: impl IntoIterator<Item = HypermeshResult<Vec<DetourTarget>>>,
) -> HypermeshResult<()> {
    let mut saw_hard_unknown = false;
    for family in families {
        match family {
            Ok(found) => {
                for target in found {
                    push_unique_detour_target(targets, target);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_detour_target_family(targets, saw_hard_unknown)
}

fn extend_disjoint_detour_target_families_backtracking_unknown(
    targets: &mut Vec<DetourTarget>,
    families: impl IntoIterator<Item = HypermeshResult<Vec<DetourTarget>>>,
) -> HypermeshResult<()> {
    let mut saw_hard_unknown = false;
    for family in families {
        match family {
            Ok(found) => targets.extend(found),
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_detour_target_family(targets, saw_hard_unknown)
}

pub(super) fn collect_detour_targets_from_axis_intervals(
    intervals: &[Vec<(Real, Real)>],
    mut build: impl FnMut(&Aabb) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<Vec<DetourTarget>> {
    let mut detours =
        Vec::with_capacity(intervals[0].len() * intervals[1].len() * intervals[2].len());
    let mut families =
        Vec::with_capacity(intervals[0].len() * intervals[1].len() * intervals[2].len());
    for x in &intervals[0] {
        for y in &intervals[1] {
            for z in &intervals[2] {
                let bounds = aabb_from_axis_intervals([x, y, z])?;
                families.push(build(&bounds));
            }
        }
    }
    // Each interval combination defines a distinct strict interior box, so target points
    // produced by different boxes cannot coincide. Avoid cross-box dedupe here.
    extend_disjoint_detour_target_families_backtracking_unknown(&mut detours, families)?;
    Ok(detours)
}

fn add_axis_box_surface_cuts_with_queries(
    cuts: &mut Vec<Real>,
    start: &Point3,
    end: &Point3,
    polygon: &ConvexPolygon,
    axis: usize,
    start_value: &Real,
    end_value: &Real,
    crossing_for: &mut impl FnMut(
        &Point3,
        &Point3,
        &ConvexPolygon,
        usize,
    ) -> HypermeshResult<Option<Point3>>,
    classify_point_on_polygon: &mut impl FnMut(
        &Point3,
        &ConvexPolygon,
    ) -> HypermeshResult<PolygonPointLocation>,
) -> HypermeshResult<bool> {
    let other_axes = other_axes(axis);
    let mut saw_unknown = false;
    for first in [axis_ref(start, other_axes[0]), axis_ref(end, other_axes[0])] {
        for second in [axis_ref(start, other_axes[1]), axis_ref(end, other_axes[1])] {
            let mut edge_start = Point3::origin();
            let mut edge_end = Point3::origin();
            *axis_mut(&mut edge_start, axis) = start_value.clone();
            *axis_mut(&mut edge_end, axis) = end_value.clone();
            *axis_mut(&mut edge_start, other_axes[0]) = first.clone();
            *axis_mut(&mut edge_end, other_axes[0]) = first.clone();
            *axis_mut(&mut edge_start, other_axes[1]) = second.clone();
            *axis_mut(&mut edge_end, other_axes[1]) = second.clone();

            let Some(crossing) = (match crossing_for(&edge_start, &edge_end, polygon, axis) {
                Ok(crossing) => crossing,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            }) else {
                continue;
            };
            let point_location = match classify_point_on_polygon(&crossing, polygon) {
                Ok(point_location) => point_location,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
            if !point_strictly_between_axis(&crossing, &edge_start, &edge_end, axis)? {
                if crossing == edge_start
                    && matches!(
                        point_location,
                        PolygonPointLocation::Boundary | PolygonPointLocation::Interior
                    )
                {
                    saw_unknown = true;
                }
                if crossing == edge_end
                    && matches!(
                        point_location,
                        PolygonPointLocation::Boundary | PolygonPointLocation::Interior
                    )
                {
                    saw_unknown = true;
                }
                continue;
            }
            match point_location {
                PolygonPointLocation::Outside => {}
                PolygonPointLocation::Boundary => {
                    saw_unknown = true;
                }
                PolygonPointLocation::Interior => {
                    let value = axis_ref(&crossing, axis);
                    if value_strictly_between(value, start_value, end_value)? {
                        push_unique_ordered_real(cuts, value.clone())?;
                    }
                }
            }
        }
    }
    Ok(saw_unknown)
}

pub(super) fn other_axes(axis: usize) -> [usize; 2] {
    match axis {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => unreachable!("axis must be in 0..3"),
    }
}

fn value_strictly_between(value: &Real, a: &Real, b: &Real) -> HypermeshResult<bool> {
    let value_to_a = compare_real(value, a)?;
    let value_to_b = compare_real(value, b)?;
    Ok((value_to_a.is_gt() && value_to_b.is_lt()) || (value_to_a.is_lt() && value_to_b.is_gt()))
}

fn push_unique_ordered_real(values: &mut Vec<Real>, value: Real) -> HypermeshResult<()> {
    for (index, existing) in values.iter().enumerate() {
        match compare_real(&value, existing)? {
            std::cmp::Ordering::Equal => return Ok(()),
            std::cmp::Ordering::Less => {
                values.insert(index, value);
                return Ok(());
            }
            std::cmp::Ordering::Greater => {}
        }
    }
    values.push(value);
    Ok(())
}

pub(super) fn point_lies_on_traced_surface(
    point: &Point3,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let point = PreparedPoint3::new(point);
    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        if point.classify(&polygon.support)? != Classification::On {
            continue;
        }

        let mut inside_polygon = true;
        let mut on_edge = false;
        for edge in &polygon.edges {
            match point.classify(edge)? {
                Classification::Positive => {
                    inside_polygon = false;
                    break;
                }
                Classification::On => on_edge = true,
                Classification::Negative => {}
            }
        }
        if inside_polygon {
            if on_edge {
                return Err(HypermeshError::UnknownClassification);
            }
            return Ok(true);
        }
    }
    Ok(false)
}

#[cfg(test)]
pub(crate) fn trace_segment_from_definitions_with_step_detoured_plane_replacement(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<WindingNumberVector> {
    let mut no_detour_cache = Vec::new();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    let mut surface_cache = Vec::new();
    let mut axis_ordered_segment_traces = Vec::new();
    let mut plane_replacement_affine = PlaneReplacementAffineCache::default();
    let mut plane_replacement_trace_steps = Vec::new();
    trace_segment_from_definitions_with_step_detoured_plane_replacement_with_caches(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        &mut surface_cache,
        &mut axis_ordered_segment_traces,
        &mut plane_replacement_affine,
        &mut plane_replacement_trace_steps,
        &mut no_detour_cache,
        &mut detour_target_cache,
        None,
    )
}

pub(crate) fn trace_segment_from_definitions_with_step_detoured_plane_replacement_in_bounds(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    trace_bounds: &Aabb,
) -> HypermeshResult<WindingNumberVector> {
    let mut no_detour_cache = Vec::new();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    let mut surface_cache = Vec::new();
    let mut axis_ordered_segment_traces = Vec::new();
    let mut plane_replacement_affine = PlaneReplacementAffineCache::default();
    let mut plane_replacement_trace_steps = Vec::new();
    trace_segment_from_definitions_with_step_detoured_plane_replacement_with_caches(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        &mut surface_cache,
        &mut axis_ordered_segment_traces,
        &mut plane_replacement_affine,
        &mut plane_replacement_trace_steps,
        &mut no_detour_cache,
        &mut detour_target_cache,
        Some(trace_bounds),
    )
}

pub(super) fn trace_segment_from_definitions_with_step_detoured_plane_replacement_with_caches(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    axis_ordered_segment_traces: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
    plane_replacement_affine: &mut PlaneReplacementAffineCache,
    plane_replacement_trace_steps: &mut Vec<PlaneReplacementStepCacheEntry>,
    no_detour_cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<WindingNumberVector> {
    if !point_is_inside_optional_trace_bounds(start, trace_bounds)?
        || !point_is_inside_optional_trace_bounds(end, trace_bounds)?
    {
        return Err(HypermeshError::UnknownClassification);
    }

    match trace_segment_from_definitions_with_caches_and_surface_query(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        surface_cache,
        axis_ordered_segment_traces,
        plane_replacement_affine,
        plane_replacement_trace_steps,
        no_detour_cache,
        detour_target_cache,
        trace_bounds,
    ) {
        Ok(winding) => return Ok(winding),
        Err(HypermeshError::UnknownClassification) => {}
        Err(err) => return Err(err),
    }

    trace_from_definition_sets_with_step_detoured_plane_replacement(
        start,
        start_definitions,
        end,
        end_definitions,
        winding,
        polygons,
        no_detour_cache,
        detour_target_cache,
        trace_bounds,
    )
}

#[cfg(test)]
pub(super) fn trace_probe_from_reference_definitions(
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    probe_point: &Point3,
    probe_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    let mut no_detour_cache = Vec::new();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    trace_from_definition_sets_with_step_detoured_plane_replacement(
        ref_point,
        ref_definitions,
        probe_point,
        probe_definitions,
        ref_wnv,
        polygons,
        &mut no_detour_cache,
        &mut detour_target_cache,
        None,
    )
}

pub(super) fn trace_from_definition_sets_with_step_detoured_plane_replacement(
    start: &Point3,
    start_definitions: &[[Plane; 3]],
    end: &Point3,
    end_definitions: &[[Plane; 3]],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    no_detour_cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<WindingNumberVector> {
    trace_from_definition_sets_with_step_detoured_plane_replacement_with_query_caches(
        start,
        start_definitions,
        end,
        end_definitions,
        winding,
        polygons,
        no_detour_cache,
        detour_target_cache,
        trace_bounds,
    )
}

fn trace_from_definition_sets_with_step_detoured_plane_replacement_with_query_caches(
    start: &Point3,
    start_definitions: &[[Plane; 3]],
    end: &Point3,
    end_definitions: &[[Plane; 3]],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    no_detour_cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<WindingNumberVector> {
    let start_definitions = endpoint_definition_family(start, start_definitions)?.definitions;
    let end_definitions = endpoint_definition_family(end, end_definitions)?.definitions;
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();

    for start_definition in &start_definitions {
        for end_definition in &end_definitions {
            match trace_plane_replacement_path_with_step_detours_with_query_caches(
                start_definition,
                end_definition,
                winding,
                polygons,
                &mut affine_cache,
                &mut step_cache,
                no_detour_cache,
                detour_target_cache,
                trace_bounds,
            ) {
                Ok(winding) => return Ok(winding),
                Err(HypermeshError::UnknownClassification) => continue,
                Err(err) => return Err(err),
            }
        }
    }

    Err(HypermeshError::UnknownClassification)
}

#[cfg(test)]
#[allow(dead_code)]
fn trace_plane_replacement_path_with_step_detours(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();
    trace_plane_replacement_path_with_step_detours_with_caches(
        start_planes,
        end_planes,
        winding,
        polygons,
        &mut affine_cache,
        &mut step_cache,
    )
}

#[cfg(test)]
fn trace_plane_replacement_path_with_step_detours_with_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    affine_cache: &mut PlaneReplacementAffineCache,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_step_detours_impl(
        start_planes,
        end_planes,
        winding,
        polygons,
        affine_cache,
        step_cache,
        None,
        |current, next, attempt, polygons, current_definitions, next_definitions| {
            match trace_segment_from_definitions(
                current,
                next,
                attempt,
                polygons,
                current_definitions,
                next_definitions,
            ) {
                Ok(winding) => Ok(Some(winding)),
                Err(HypermeshError::UnknownClassification) => {
                    Err(HypermeshError::UnknownClassification)
                }
                Err(err) => Err(err),
            }
        },
    )
}

fn trace_plane_replacement_path_with_step_detours_with_query_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    affine_cache: &mut PlaneReplacementAffineCache,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
    no_detour_cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_cache: &mut DetourTargetFamilyCache,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_step_detours_impl(
        start_planes,
        end_planes,
        winding,
        polygons,
        affine_cache,
        step_cache,
        trace_bounds,
        |current, next, attempt, polygons, current_definitions, next_definitions| {
            match trace_segment_from_definitions_with_caches(
                current,
                next,
                attempt,
                polygons,
                current_definitions,
                next_definitions,
                no_detour_cache,
                detour_target_cache,
                trace_bounds,
            ) {
                Ok(winding) => Ok(Some(winding)),
                Err(HypermeshError::UnknownClassification) => {
                    Err(HypermeshError::UnknownClassification)
                }
                Err(err) => Err(err),
            }
        },
    )
}

pub(super) fn trace_plane_replacement_path_with_step_detours_impl(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    affine_cache: &mut PlaneReplacementAffineCache,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
    trace_bounds: Option<&Aabb>,
    mut trace_step: impl FnMut(
        &Point3,
        &Point3,
        &[i32],
        &[ConvexPolygon],
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<Option<WindingNumberVector>>,
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_tracer_and_caches(
        start_planes,
        end_planes,
        winding,
        polygons,
        affine_cache,
        step_cache,
        trace_bounds,
        |current, next, current_definition, next_definition, attempt, polygons| {
            trace_step(
                current,
                next,
                attempt,
                polygons,
                std::slice::from_ref(current_definition),
                std::slice::from_ref(next_definition),
            )
        },
    )
}

fn append_definition_if_missing(definitions: &mut Vec<[Plane; 3]>, candidate: [Plane; 3]) {
    if definitions
        .iter()
        .all(|definition| !definition_planes_match_as_sets(definition, &candidate))
    {
        definitions.push(candidate);
    }
}

pub(super) struct EndpointDefinitionFamilyState {
    pub(super) definitions: Vec<[Plane; 3]>,
    pub(super) saw_unknown: bool,
}

pub(super) fn endpoint_definition_family(
    point: &Point3,
    definitions: &[[Plane; 3]],
) -> HypermeshResult<EndpointDefinitionFamilyState> {
    let mut matching = Vec::new();
    let mut saw_unknown = false;
    for definition in definitions {
        match affine_from_planes(definition) {
            Ok(defined_point) if defined_point == *point => {
                append_definition_if_missing(&mut matching, definition.clone());
            }
            Ok(_) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    append_definition_if_missing(&mut matching, axis_plane_definition(point));
    Ok(EndpointDefinitionFamilyState {
        definitions: matching,
        saw_unknown,
    })
}

pub(super) fn definition_planes_match_as_sets(left: &[Plane; 3], right: &[Plane; 3]) -> bool {
    let mut matched = [false; 3];
    for left_plane in left {
        let Some((index, _)) = right
            .iter()
            .enumerate()
            .find(|(index, right_plane)| !matched[*index] && *right_plane == left_plane)
        else {
            return false;
        };
        matched[index] = true;
    }
    true
}

pub(super) fn unique_definition_family(definitions: &[[Plane; 3]]) -> Vec<[Plane; 3]> {
    let mut unique = Vec::new();
    for definition in definitions {
        if unique
            .iter()
            .all(|existing| !definition_planes_match_as_sets(existing, definition))
        {
            unique.push(definition.clone());
        }
    }
    unique
}

pub(super) fn definition_families_match_as_sets(left: &[[Plane; 3]], right: &[[Plane; 3]]) -> bool {
    if left.len() != right.len() {
        return false;
    }

    let mut matched = vec![false; right.len()];
    for left_definition in left {
        let Some((index, _)) = right.iter().enumerate().find(|(index, right_definition)| {
            !matched[*index] && definition_planes_match_as_sets(left_definition, right_definition)
        }) else {
            return false;
        };
        matched[index] = true;
    }

    true
}

pub(super) fn initial_visited_definition_points(
    start: &Point3,
    start_definitions: &[[Plane; 3]],
    end: &Point3,
    end_definitions: &[[Plane; 3]],
) -> Vec<VisitedDefinitionPoint> {
    let mut visited = vec![VisitedDefinitionPoint {
        point: start.clone(),
        definitions: start_definitions.to_vec(),
    }];
    if !visited_definition_family_contains(&visited, end, end_definitions) {
        visited.push(VisitedDefinitionPoint {
            point: end.clone(),
            definitions: end_definitions.to_vec(),
        });
    }
    visited
}

pub(super) fn visited_definition_family_contains(
    points: &[VisitedDefinitionPoint],
    candidate: &Point3,
    definitions: &[[Plane; 3]],
) -> bool {
    points.iter().any(|point| {
        point.point == *candidate
            && definition_families_match_as_sets(&point.definitions, definitions)
    })
}

pub(super) fn visited_definition_points_match_as_sets(
    left: &[VisitedDefinitionPoint],
    right: &[VisitedDefinitionPoint],
) -> bool {
    if left.len() != right.len() {
        return false;
    }

    let mut matched = vec![false; right.len()];
    for left_point in left {
        let Some(index) = right.iter().enumerate().position(|(index, right_point)| {
            !matched[index]
                && right_point.point == left_point.point
                && definition_families_match_as_sets(
                    &right_point.definitions,
                    &left_point.definitions,
                )
        }) else {
            return false;
        };
        matched[index] = true;
    }

    true
}

pub(super) fn visited_definition_points_subset_of(
    subset: &[VisitedDefinitionPoint],
    superset: &[VisitedDefinitionPoint],
) -> bool {
    subset.iter().all(|subset_point| {
        superset.iter().any(|superset_point| {
            superset_point.point == subset_point.point
                && definition_families_match_as_sets(
                    &superset_point.definitions,
                    &subset_point.definitions,
                )
        })
    })
}

pub(super) fn normalized_cycle_guard_visited_points(
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
) -> Vec<VisitedDefinitionPoint> {
    visited_points
        .iter()
        .filter(|visited| {
            !(visited.point == *start
                && definition_families_match_as_sets(&visited.definitions, start_definitions)
                || visited.point == *end
                    && definition_families_match_as_sets(&visited.definitions, end_definitions))
        })
        .cloned()
        .collect()
}

#[cfg(test)]
pub(super) fn detour_recursion_limit(polygons: &[ConvexPolygon]) -> usize {
    MIN_DETOUR_RECURSION_LIMIT.max(
        polygons
            .iter()
            .filter(|polygon| polygon.mesh_index >= 0)
            .count(),
    )
}

#[cfg(test)]
#[allow(dead_code)]
pub(super) fn plane_replacement_step_detour_limit(polygons: &[ConvexPolygon]) -> usize {
    MIN_PLANE_REPLACEMENT_STEP_DETOUR_LIMIT.max(
        polygons
            .iter()
            .filter(|polygon| polygon.mesh_index >= 0)
            .count(),
    )
}
