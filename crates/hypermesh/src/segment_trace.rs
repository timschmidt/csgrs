//! Exact segment tracing for winding-number propagation.

use hyperlattice::{HomogeneousPoint3, Point3, Real, intersect_three_planes};
use hyperlimit::{
    HalfspaceFeasibility, Plane3 as LimitPlane3, PredicateOutcome, classify_halfspace_feasibility3,
};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{
    Aabb, Classification, Plane, axis_mut, axis_ref, classify_point, classify_real, compare_real,
};
use crate::polygon::ConvexPolygon;
use crate::winding::{WindingNumberTransitionVector, WindingNumberVector};

/// Result of tracing one axis-aligned segment.
#[derive(Clone, Debug, PartialEq)]
pub struct TraceAxisSegmentResult {
    /// Winding number after accepted crossings.
    pub winding: WindingNumberVector,
    /// Whether the path avoided exact edge hits.
    pub valid: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct CrossingEvent {
    point: Point3,
    support: Plane,
    normal_sign: i32,
    cross_sign: i32,
    delta_w: WindingNumberTransitionVector,
    on_edge: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct PlaneDefinedPoint {
    planes: [Plane; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct InteriorLeafPoint {
    pub(crate) point: Point3,
    planes: Vec<[Plane; 3]>,
    uncertified_definition_fallback: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct ProbePoint {
    point: Point3,
    side: Classification,
    planes: Vec<[Plane; 3]>,
    uncertified_definition_fallback: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct ProbeWindingCacheEntry {
    point: Point3,
    planes: Vec<[Plane; 3]>,
    winding: HypermeshResult<WindingNumberVector>,
}

#[derive(Clone, Debug, PartialEq)]
struct SurfaceCacheEntry {
    point: Point3,
    on_surface: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
struct ProbeReachabilityCacheEntry {
    interior_point: Point3,
    interior_planes: Vec<[Plane; 3]>,
    probe_point: Point3,
    probe_planes: Vec<[Plane; 3]>,
    reachable: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
struct LeafWitnessSeedFamilies {
    seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    saw_unknown: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct PlaneReplacementStepCacheEntry {
    current_point: Point3,
    next_point: Point3,
    current_planes: [Plane; 3],
    next_planes: [Plane; 3],
    attempt: WindingNumberVector,
    result: HypermeshResult<Option<WindingNumberVector>>,
}

#[derive(Clone, Debug, PartialEq)]
struct PlaneReplacementAffineCacheEntry {
    planes: [Plane; 3],
    point: HypermeshResult<Point3>,
}

#[derive(Clone, Debug, PartialEq)]
struct PlaneReplacementReachabilityStepCacheEntry {
    current_point: Point3,
    next_point: Point3,
    current_planes: [Plane; 3],
    next_planes: [Plane; 3],
    result: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
struct AxisOrderedSegmentTraceCacheEntry {
    start: Point3,
    end: Point3,
    axis: usize,
    attempt: WindingNumberVector,
    result: HypermeshResult<TraceAxisSegmentResult>,
}

#[derive(Clone, Debug, PartialEq)]
struct DefinitionNoDetourTraceCacheEntry {
    start: Point3,
    end: Point3,
    winding: WindingNumberVector,
    start_definitions: Vec<[Plane; 3]>,
    end_definitions: Vec<[Plane; 3]>,
    result: HypermeshResult<Option<WindingNumberVector>>,
}

#[derive(Clone, Debug, PartialEq)]
struct DefinitionNoDetourReachabilityCacheEntry {
    start: Point3,
    end: Point3,
    start_definitions: Vec<[Plane; 3]>,
    end_definitions: Vec<[Plane; 3]>,
    result: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
struct DetourTargetFamilyCacheEntry {
    start: Point3,
    end: Point3,
    targets: HypermeshResult<Vec<DetourTarget>>,
}

#[derive(Clone, Debug, PartialEq)]
struct DetourTarget {
    point: Point3,
    definitions: Vec<[Plane; 3]>,
    uncertified_definition_fallback: bool,
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

fn mark_all_detour_targets_uncertified(targets: &mut Vec<DetourTarget>) {
    for target in targets {
        target.uncertified_definition_fallback = true;
    }
}

fn mark_all_shifted_halfspace_witnesses_uncertified(witnesses: &mut Vec<ShiftedHalfspaceWitness>) {
    for witness in witnesses {
        witness.uncertified_definition_fallback = true;
    }
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

        let Some(crossing) = segment_plane_crossing(start, end, &polygon.support)? else {
            continue;
        };

        if !point_strictly_between_axis(&crossing, start, end, axis)? {
            continue;
        }

        let mut inside = true;
        let mut on_edge = false;
        for edge in &polygon.edges {
            match classify_point(&crossing, edge)? {
                Classification::Positive => {
                    inside = false;
                    break;
                }
                Classification::On => on_edge = true,
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
            on_edge,
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

const AXIS_ORDERINGS: [[usize; 3]; 6] = [
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
    let mut detour_target_cache = Vec::new();
    trace_segment_from_definitions_with_caches(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        &mut no_detour_cache,
        &mut detour_target_cache,
    )
}

fn trace_segment_from_definitions_with_caches(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    no_detour_cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
    let mut trace_without_detours =
        |start: &Point3,
         end: &Point3,
         winding: &[i32],
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            cached_definition_no_detour_trace_with(
                &mut *no_detour_cache,
                start,
                end,
                winding,
                start_definitions,
                end_definitions,
                || {
                    trace_segment_with_definitions_no_detours(
                        start,
                        end,
                        winding,
                        polygons,
                        start_definitions,
                        end_definitions,
                    )
                },
            )
        };
    let mut detours_for = |start: &Point3, end: &Point3| {
        cached_detour_target_family_with(&mut *detour_target_cache, start, end, || {
            interior_box_detour_targets(start, end, polygons)
        })
    };
    trace_segment_from_definitions_with_cycle_guard_impl(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        &[start.clone(), end.clone()],
        &mut trace_without_detours,
        &mut detours_for,
    )
}

fn trace_segment_from_definitions_with_cycle_guard_impl(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[Point3],
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

fn trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[Point3],
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
fn trace_segment_from_definitions_with_budget_impl(
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

fn trace_segment_via_detours_with_cycle_guard_with_surface_query(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    detours: &[DetourTarget],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[Point3],
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
        let already_visited = point_family_contains(visited_points, &detour.point);
        let on_surface = if already_visited {
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
        next_visited_points.push(detour.point.clone());

        let first_leg =
            match trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
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
            ) {
                Ok(first_leg) => first_leg,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
        let second_leg =
            match trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
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

fn cached_definition_no_detour_trace_with(
    cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    trace: impl FnOnce() -> HypermeshResult<Option<WindingNumberVector>>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    if let Some(existing) = cache.iter().find(|existing| {
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

fn cached_detour_target_family_with(
    cache: &mut Vec<DetourTargetFamilyCacheEntry>,
    start: &Point3,
    end: &Point3,
    build: impl FnOnce() -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<Vec<DetourTarget>> {
    if let Some(existing) = cache.iter().find(|existing| {
        (existing.start == *start && existing.end == *end)
            || (existing.start == *end && existing.end == *start)
    }) {
        return existing.targets.clone();
    }

    let targets = build();
    cache.push(DetourTargetFamilyCacheEntry {
        start: start.clone(),
        end: end.clone(),
        targets: targets.clone(),
    });
    targets
}

fn trace_segment_without_detours(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<WindingNumberVector>> {
    let axis_unknown = match trace_axis_ordered_paths(start, end, winding, polygons) {
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
fn trace_segment_via_detours_with_definitions_budget(
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

fn trace_segment_with_definitions_no_detours(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<Option<WindingNumberVector>> {
    match trace_segment_without_detours(start, end, winding, polygons) {
        Ok(Some(winding)) => return Ok(Some(winding)),
        Ok(None) => {}
        Err(HypermeshError::UnknownClassification) => {}
        Err(err) => return Err(err),
    }

    let mut start_definitions = start_definitions.to_vec();
    append_definition_if_missing(&mut start_definitions, axis_plane_definition(start));
    let mut end_definitions = end_definitions.to_vec();
    append_definition_if_missing(&mut end_definitions, axis_plane_definition(end));
    let mut affine_cache = Vec::new();
    let mut step_cache = Vec::new();

    definition_pair_trace_backtracking_unknown(
        &start_definitions,
        &end_definitions,
        |start_definition, end_definition| {
            trace_plane_replacement_path_without_detours_with_caches(
                start_definition,
                end_definition,
                winding,
                polygons,
                &mut affine_cache,
                &mut step_cache,
            )
        },
    )
}

fn definition_pair_trace_backtracking_unknown(
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
fn trace_segment_with_detours_without_plane_replacement_impl(
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
    let mut affine_cache = Vec::new();
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
    let mut affine_cache = Vec::new();
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

fn trace_plane_replacement_path_without_detours_with_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_tracer(
        start_planes,
        end_planes,
        winding,
        polygons,
        |current, next, _current_planes, _next_planes, attempt, polygons| {
            trace_segment_without_detours(current, next, attempt, polygons)
        },
        affine_cache,
        step_cache,
    )
}

fn trace_plane_replacement_path_with_tracer(
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
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_tracer_and_caches(
        start_planes,
        end_planes,
        winding,
        polygons,
        affine_cache,
        step_cache,
        trace_step,
    )
}

fn trace_plane_replacement_path_with_tracer_and_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    mut affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    mut step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
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
            match cached_affine_from_planes_with(&mut affine_cache, &current_planes, || {
                affine_from_planes(&current_planes)
            }) {
                Ok(point) => point,
                Err(HypermeshError::UnknownClassification) => continue,
                Err(err) => return Err(err),
            };
        let mut attempt = winding.to_vec();
        let mut valid = true;

        for plane_index in ordering {
            let mut next_planes = current_planes.clone();
            next_planes[plane_index] = end_planes[plane_index].clone();
            let next_point =
                match cached_affine_from_planes_with(&mut affine_cache, &next_planes, || {
                    affine_from_planes(&next_planes)
                }) {
                    Ok(point) => point,
                    Err(HypermeshError::UnknownClassification) => {
                        valid = false;
                        break;
                    }
                    Err(err) => return Err(err),
                };
            if next_point != current_point {
                let next_winding = match cached_plane_replacement_step_with(
                    &mut step_cache,
                    &current_point,
                    &next_point,
                    &current_planes,
                    &next_planes,
                    &attempt,
                    || {
                        trace_step(
                            &current_point,
                            &next_point,
                            &current_planes,
                            &next_planes,
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
            }
            current_planes = next_planes;
        }

        if valid {
            return Ok(attempt);
        }
    }

    Err(HypermeshError::UnknownClassification)
}

fn cached_affine_from_planes_with(
    cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    planes: &[Plane; 3],
    compute: impl FnOnce() -> HypermeshResult<Point3>,
) -> HypermeshResult<Point3> {
    if let Some(existing) = cache
        .iter()
        .find(|existing| definition_planes_match_as_sets(&existing.planes, planes))
    {
        return existing.point.clone();
    }

    let point = compute();
    cache.push(PlaneReplacementAffineCacheEntry {
        planes: planes.clone(),
        point: point.clone(),
    });
    point
}

fn cached_plane_replacement_step_with(
    cache: &mut Vec<PlaneReplacementStepCacheEntry>,
    current_point: &Point3,
    next_point: &Point3,
    current_planes: &[Plane; 3],
    next_planes: &[Plane; 3],
    attempt: &[i32],
    trace: impl FnOnce() -> HypermeshResult<Option<WindingNumberVector>>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    if let Some(existing) = cache.iter().find(|existing| {
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

fn axis_plane_defined_point(point: &Point3) -> PlaneDefinedPoint {
    PlaneDefinedPoint {
        planes: axis_plane_definition(point),
    }
}

#[cfg(test)]
fn retryable_trace<T>(result: HypermeshResult<T>) -> HypermeshResult<Option<T>> {
    match result {
        Ok(value) => Ok(Some(value)),
        Err(HypermeshError::UnknownClassification) => Ok(None),
        Err(err) => Err(err),
    }
}

fn apply_winding_transition_in_place(
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

fn trace_direct_segment(
    start: &Point3,
    end: &Point3,
    start_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<TraceAxisSegmentResult> {
    let mut winding = start_wnv.to_vec();
    let Some(sort_axis) = first_changed_axis(start, end)? else {
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
        if start_class == Classification::On || end_class == Classification::On {
            continue;
        }
        if start_class == end_class {
            continue;
        }

        let Some(crossing) = segment_plane_crossing(start, end, &polygon.support)? else {
            continue;
        };

        let mut inside = true;
        let mut on_edge = false;
        for edge in &polygon.edges {
            match classify_point(&crossing, edge)? {
                Classification::Positive => {
                    inside = false;
                    break;
                }
                Classification::On => on_edge = true,
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
            on_edge,
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
    for (index, event) in events.iter().enumerate() {
        if event.on_edge
            && !events.iter().enumerate().any(|(other_index, other)| {
                other_index != index
                    && other.point == event.point
                    && other.support == event.support
                    && other.normal_sign == event.normal_sign
                    && other.delta_w == event.delta_w
            })
        {
            return Err(HypermeshError::UnknownClassification);
        }

        if accepted.iter().any(|existing: &CrossingEvent| {
            existing.point == event.point
                && existing.support == event.support
                && existing.normal_sign == event.normal_sign
                && existing.delta_w == event.delta_w
        }) {
            continue;
        }

        accepted.push(event.clone());
    }
    Ok(accepted)
}

fn first_changed_axis(start: &Point3, end: &Point3) -> HypermeshResult<Option<usize>> {
    for axis in 0..3 {
        if compare_real(axis_ref(start, axis), axis_ref(end, axis))?.is_ne() {
            return Ok(Some(axis));
        }
    }
    Ok(None)
}

fn trace_axis_ordered_paths(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    let mut surface_cache = Vec::new();
    trace_axis_ordered_paths_with_queries(
        start,
        end,
        winding,
        polygons,
        |point| {
            cached_surface_query_with(&mut surface_cache, point, || {
                point_lies_on_traced_surface(point, polygons)
            })
        },
        |current, next, axis, attempt, polygons| {
            trace_axis_segment(current, next, axis, attempt, polygons)
        },
    )
}

#[cfg(test)]
fn trace_axis_ordered_paths_with_surface_query(
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

fn trace_axis_ordered_paths_with_queries(
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
    let mut segment_cache = Vec::new();

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
                let traced = match cached_axis_ordered_segment_trace_with(
                    &mut segment_cache,
                    &current,
                    &next,
                    axis,
                    &attempt,
                    || trace_segment_step(&current, &next, axis, &attempt, polygons),
                ) {
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

fn cached_axis_ordered_segment_trace_with(
    cache: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
    start: &Point3,
    end: &Point3,
    axis: usize,
    attempt: &[i32],
    trace: impl FnOnce() -> HypermeshResult<TraceAxisSegmentResult>,
) -> HypermeshResult<TraceAxisSegmentResult> {
    if let Some(existing) = cache.iter().find(|existing| {
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

fn interior_box_detour_targets(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<DetourTarget>> {
    interior_box_detour_targets_with_queries(
        start,
        end,
        polygons,
        |edge_start, edge_end, polygon, axis| {
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
        |crossing, polygon| classify_point_in_polygon(crossing, polygon),
        |bounds| strict_aabb_targets(bounds),
    )
}

fn interior_box_detour_targets_with_queries(
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

    let mut targets =
        collect_detour_targets_from_axis_intervals(&intervals, |bounds| build(bounds))?;
    if targets.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_detour_targets_uncertified(&mut targets);
        }
        Ok(targets)
    }
}

fn interior_box_axis_intervals_with_surface_queries(
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

fn aabb_from_axis_intervals(intervals: [&(Real, Real); 3]) -> HypermeshResult<Aabb> {
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

fn strict_aabb_targets(bounds: &Aabb) -> HypermeshResult<Vec<DetourTarget>> {
    let halfspaces = aabb_core_halfspaces(bounds)?;
    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }
    let mut targets = Vec::new();
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            bounds,
            &halfspaces,
            report.as_ref(),
            &mut saw_unknown,
        )?;
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) = dedupe_shifted_halfspace_seed_families(
        report_witness,
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    );

    extend_detour_target_builds_backtracking_unknown(&mut targets, seeds.iter(), |seed| {
        let active_planes = active_planes_from_optional_report(report.as_ref(), seed);
        build_detour_target(seed, &halfspaces, active_planes, false)
    })?;

    let shifted_witnesses = shifted_halfspace_witness_family_or_empty(
        {
            let mut shifted_witnesses = Vec::new();
            extend_shifted_halfspace_seed_families_backtracking_unknown(
                &mut shifted_witnesses,
                [seeds, shifted_vertices, shifted_geometry_seeds],
                |seed| shifted_halfspace_cell_witnesses_from_seed(bounds, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    extend_detour_target_builds_backtracking_unknown(
        &mut targets,
        shifted_witnesses.iter(),
        build_detour_target_from_shifted_witness,
    )?;
    if targets.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_detour_targets_uncertified(&mut targets);
        }
        Ok(targets)
    }
}

fn build_detour_target(
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

fn build_detour_target_from_shifted_witness(
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

    if definitions.is_empty() && saw_unknown {
        definitions.push(axis_plane_definition(&witness.point));
    }

    Ok(DetourTarget {
        point: witness.point.clone(),
        definitions,
        uncertified_definition_fallback: witness.uncertified_definition_fallback || saw_unknown,
    })
}

fn push_unique_detour_target(targets: &mut Vec<DetourTarget>, target: DetourTarget) {
    if let Some(existing) = targets
        .iter_mut()
        .find(|existing| existing.point == target.point)
    {
        existing.uncertified_definition_fallback |= target.uncertified_definition_fallback;
        for definition in target.definitions {
            if !existing
                .definitions
                .iter()
                .any(|candidate| definition_planes_match_as_sets(candidate, &definition))
            {
                existing.definitions.push(definition);
            }
        }
    } else {
        targets.push(target);
    }
}

fn extend_unique_definition_families(definitions: &mut Vec<[Plane; 3]>, fresh: Vec<[Plane; 3]>) {
    for definition in fresh {
        if !definitions
            .iter()
            .any(|existing| definition_planes_match_as_sets(existing, &definition))
        {
            definitions.push(definition);
        }
    }
}

fn extend_detour_target_builds_backtracking_unknown<'a, T: 'a>(
    targets: &mut Vec<DetourTarget>,
    candidates: impl IntoIterator<Item = &'a T>,
    mut build: impl FnMut(&'a T) -> HypermeshResult<DetourTarget>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(target) => {
                saw_unknown |= target.uncertified_definition_fallback;
                push_unique_detour_target(targets, target)
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if targets.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_detour_targets_uncertified(targets);
        }
        Ok(())
    }
}

fn extend_detour_target_families_backtracking_unknown(
    targets: &mut Vec<DetourTarget>,
    families: impl IntoIterator<Item = HypermeshResult<Vec<DetourTarget>>>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for family in families {
        match family {
            Ok(found) => {
                saw_unknown |= found
                    .iter()
                    .any(|target| target.uncertified_definition_fallback);
                for target in found {
                    push_unique_detour_target(targets, target);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if targets.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_detour_targets_uncertified(targets);
        }
        Ok(())
    }
}

fn collect_detour_targets_from_axis_intervals(
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
    extend_detour_target_families_backtracking_unknown(&mut detours, families)?;
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

fn other_axes(axis: usize) -> [usize; 2] {
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

fn point_lies_on_traced_surface(
    point: &Point3,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        if classify_point(point, &polygon.support)? != Classification::On {
            continue;
        }

        let mut inside_polygon = true;
        for edge in &polygon.edges {
            if classify_point(point, edge)? == Classification::Positive {
                inside_polygon = false;
                break;
            }
        }
        if inside_polygon {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Classifies a leaf polygon by tracing from a reference point to an off-face
/// probe and applying the host transition correction.
pub fn classify_leaf_polygon(
    support: &Plane,
    leaf_edges: &[Plane],
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
) -> HypermeshResult<WindingNumberVector> {
    let interior_points = certified_leaf_interior_points(support, leaf_edges)?;
    classify_leaf_polygon_from_interior_points(
        &interior_points,
        support,
        ref_point,
        ref_definitions,
        ref_wnv,
        polygons,
        bounds,
        host_delta_w,
    )
}

pub(crate) fn classify_leaf_polygon_from_interior_points(
    interior_points: &[InteriorLeafPoint],
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
) -> HypermeshResult<WindingNumberVector> {
    let mut probe_winding_cache = Vec::new();
    let mut probe_surface_cache = Vec::new();
    let mut probe_reachability_cache = Vec::new();
    search_leaf_probe_families(
        interior_points,
        |point, positive_side| {
            bounded_probes_from_interior(point, support, bounds, positive_side, polygons)
        },
        |point, _positive_side, probe| {
            if cached_surface_query_with(&mut probe_surface_cache, &probe.point, || {
                point_lies_on_traced_surface(&probe.point, polygons)
            })? {
                return Ok(None);
            }
            if !cached_probe_reachability_with(
                &mut probe_reachability_cache,
                point,
                &probe,
                || probe_reaches_adjacent_cell_from_interior(point, &probe, support, polygons),
            )? {
                return Ok(None);
            }
            let mut winding = cached_probe_winding_with(&mut probe_winding_cache, &probe, || {
                trace_probe_winding(ref_point, ref_definitions, &probe, ref_wnv, polygons)
            })?;
            if probe.side == Classification::Negative {
                apply_winding_transition_in_place(&mut winding, -1, host_delta_w)?;
            }
            Ok(Some(winding))
        },
    )?
    .ok_or(HypermeshError::UnknownClassification)
}

fn cached_probe_winding_with(
    cache: &mut Vec<ProbeWindingCacheEntry>,
    probe: &ProbePoint,
    trace: impl FnOnce() -> HypermeshResult<WindingNumberVector>,
) -> HypermeshResult<WindingNumberVector> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.point == probe.point
            && definition_families_match_as_sets(&existing.planes, &probe.planes)
    }) {
        return existing.winding.clone();
    }

    let winding = trace();
    cache.push(ProbeWindingCacheEntry {
        point: probe.point.clone(),
        planes: probe.planes.clone(),
        winding: winding.clone(),
    });
    winding
}

fn cached_surface_query_with(
    cache: &mut Vec<SurfaceCacheEntry>,
    point: &Point3,
    query: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().find(|existing| existing.point == *point) {
        return existing.on_surface.clone();
    }

    let on_surface = query();
    cache.push(SurfaceCacheEntry {
        point: point.clone(),
        on_surface: on_surface.clone(),
    });
    on_surface
}

fn cached_probe_reachability_with(
    cache: &mut Vec<ProbeReachabilityCacheEntry>,
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
    query: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.interior_point == interior.point
            && definition_families_match_as_sets(&existing.interior_planes, &interior.planes)
            && existing.probe_point == probe.point
            && definition_families_match_as_sets(&existing.probe_planes, &probe.planes)
    }) {
        return existing.reachable.clone();
    }

    let reachable = query();
    cache.push(ProbeReachabilityCacheEntry {
        interior_point: interior.point.clone(),
        interior_planes: interior.planes.clone(),
        probe_point: probe.point.clone(),
        probe_planes: probe.planes.clone(),
        reachable: reachable.clone(),
    });
    reachable
}

fn search_leaf_probe_families<'a>(
    interior_points: &'a [InteriorLeafPoint],
    mut probes_for: impl FnMut(&'a InteriorLeafPoint, bool) -> HypermeshResult<Vec<ProbePoint>>,
    mut handle_probe: impl FnMut(
        &'a InteriorLeafPoint,
        bool,
        ProbePoint,
    ) -> HypermeshResult<Option<WindingNumberVector>>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut saw_unknown = false;

    for point in interior_points {
        for positive_side in [true, false] {
            let probes = match probes_for(point, positive_side) {
                Ok(probes) => probes,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
            if probes.is_empty() && point.uncertified_definition_fallback {
                saw_unknown = true;
            }

            for probe in probes {
                let probe_fallback = probe.uncertified_definition_fallback;
                match handle_probe(point, positive_side, probe) {
                    Ok(Some(winding)) => return Ok(Some(winding)),
                    Ok(None) => {
                        if point.uncertified_definition_fallback || probe_fallback {
                            saw_unknown = true;
                        }
                    }
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                    }
                    Err(err) => return Err(err),
                }
            }
        }
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

fn trace_probe_winding(
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    probe: &ProbePoint,
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    let mut probe_definitions = probe.planes.clone();
    let axis_definition = axis_plane_defined_point(&probe.point).planes;
    if !probe_definitions
        .iter()
        .any(|definition| definition_planes_match_as_sets(definition, &axis_definition))
    {
        probe_definitions.push(axis_definition);
    }
    probe_definitions = unique_definition_family(&probe_definitions);

    trace_segment_from_definitions_with_step_detoured_plane_replacement(
        ref_point,
        &probe.point,
        ref_wnv,
        polygons,
        ref_definitions,
        &probe_definitions,
    )
}

pub(crate) fn trace_segment_from_definitions_with_step_detoured_plane_replacement(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<WindingNumberVector> {
    let mut no_detour_cache = Vec::new();
    let mut detour_target_cache = Vec::new();
    match trace_segment_from_definitions_with_caches(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        &mut no_detour_cache,
        &mut detour_target_cache,
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
        &mut no_detour_cache,
        &mut detour_target_cache,
    )
}

#[cfg(test)]
fn trace_probe_from_reference_definitions(
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    probe_point: &Point3,
    probe_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    let mut no_detour_cache = Vec::new();
    let mut detour_target_cache = Vec::new();
    trace_from_definition_sets_with_step_detoured_plane_replacement(
        ref_point,
        ref_definitions,
        probe_point,
        probe_definitions,
        ref_wnv,
        polygons,
        &mut no_detour_cache,
        &mut detour_target_cache,
    )
}

fn trace_from_definition_sets_with_step_detoured_plane_replacement(
    start: &Point3,
    start_definitions: &[[Plane; 3]],
    end: &Point3,
    end_definitions: &[[Plane; 3]],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    no_detour_cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
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
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
    let mut start_definitions = start_definitions.to_vec();
    append_definition_if_missing(
        &mut start_definitions,
        axis_plane_defined_point(start).planes,
    );
    start_definitions = unique_definition_family(&start_definitions);
    let mut end_definitions = end_definitions.to_vec();
    append_definition_if_missing(&mut end_definitions, axis_plane_defined_point(end).planes);
    end_definitions = unique_definition_family(&end_definitions);
    let mut affine_cache = Vec::new();
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
    let mut affine_cache = Vec::new();
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
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_step_detours_impl(
        start_planes,
        end_planes,
        winding,
        polygons,
        affine_cache,
        step_cache,
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
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
    no_detour_cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_step_detours_impl(
        start_planes,
        end_planes,
        winding,
        polygons,
        affine_cache,
        step_cache,
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

fn trace_plane_replacement_path_with_step_detours_impl(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
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

fn definition_planes_match_as_sets(left: &[Plane; 3], right: &[Plane; 3]) -> bool {
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

fn unique_definition_family(definitions: &[[Plane; 3]]) -> Vec<[Plane; 3]> {
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

fn definition_families_match_as_sets(left: &[[Plane; 3]], right: &[[Plane; 3]]) -> bool {
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

fn point_family_contains(points: &[Point3], candidate: &Point3) -> bool {
    points.iter().any(|point| point == candidate)
}

#[cfg(test)]
fn detour_recursion_limit(polygons: &[ConvexPolygon]) -> usize {
    MIN_DETOUR_RECURSION_LIMIT.max(
        polygons
            .iter()
            .filter(|polygon| polygon.mesh_index >= 0)
            .count(),
    )
}

#[cfg(test)]
#[allow(dead_code)]
fn plane_replacement_step_detour_limit(polygons: &[ConvexPolygon]) -> usize {
    MIN_PLANE_REPLACEMENT_STEP_DETOUR_LIMIT.max(
        polygons
            .iter()
            .filter(|polygon| polygon.mesh_index >= 0)
            .count(),
    )
}

fn probe_reaches_adjacent_cell(
    start: &Point3,
    probe: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let Some(sort_axis) = first_changed_axis(start, probe)? else {
        return Ok(false);
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

fn probe_reaches_adjacent_cell_from_interior(
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let mut start_definitions = interior.planes.clone();
    append_definition_if_missing(
        &mut start_definitions,
        axis_plane_definition(&interior.point),
    );
    start_definitions = unique_definition_family(&start_definitions);
    let mut end_definitions = probe.planes.clone();
    append_definition_if_missing(&mut end_definitions, axis_plane_definition(&probe.point));
    end_definitions = unique_definition_family(&end_definitions);

    probe_reaches_adjacent_cell_with_cycle_guard(
        &interior.point,
        &probe.point,
        host_support,
        polygons,
        &start_definitions,
        &end_definitions,
    )
}

fn probe_reaches_adjacent_cell_with_cycle_guard(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    let mut no_detour_cache = Vec::new();
    let mut detour_target_cache = Vec::new();
    let mut trace_without_detours =
        |start: &Point3,
         end: &Point3,
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            cached_definition_no_detour_reachability_with(
                &mut no_detour_cache,
                start,
                end,
                start_definitions,
                end_definitions,
                || {
                    probe_reaches_adjacent_cell_with_definitions_no_detours(
                        start,
                        end,
                        host_support,
                        polygons,
                        start_definitions,
                        end_definitions,
                    )
                },
            )
        };
    let mut detours_for = |start: &Point3, end: &Point3| {
        cached_detour_target_family_with(&mut detour_target_cache, start, end, || {
            interior_box_detour_targets(start, end, polygons)
        })
    };
    probe_reaches_adjacent_cell_with_cycle_guard_impl(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        &[start.clone(), end.clone()],
        &mut trace_without_detours,
        &mut detours_for,
    )
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

fn cached_definition_no_detour_reachability_with(
    cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    trace: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.start == *start
            && existing.end == *end
            && definition_families_match_as_sets(&existing.start_definitions, start_definitions)
            && definition_families_match_as_sets(&existing.end_definitions, end_definitions)
    }) {
        return existing.result.clone();
    }

    let result = trace();
    cache.push(DefinitionNoDetourReachabilityCacheEntry {
        start: start.clone(),
        end: end.clone(),
        start_definitions: start_definitions.to_vec(),
        end_definitions: end_definitions.to_vec(),
        result: result.clone(),
    });
    result
}

fn probe_reaches_adjacent_cell_with_cycle_guard_impl(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[Point3],
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

fn probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[Point3],
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
fn probe_reaches_adjacent_cell_with_definitions_budget_impl(
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
fn probe_reaches_adjacent_cell_via_detours_with_cycle_guard(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[Point3],
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

fn probe_reaches_adjacent_cell_via_detours_with_cycle_guard_with_surface_query(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[Point3],
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
        let already_visited = point_family_contains(visited_points, &detour.point);
        let on_surface = if already_visited {
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
        next_visited_points.push(detour.point.clone());

        let first_leg = match probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
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
        match probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
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
fn probe_reaches_adjacent_cell_via_detours(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
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
    probe_reaches_adjacent_cell_via_detours_with_cycle_guard(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        &[start.clone(), end.clone()],
        &mut trace_without_detours,
        &mut detours_for,
    )
}

#[cfg(test)]
fn probe_reaches_adjacent_cell_via_detours_with_budget(
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

fn probe_reaches_adjacent_cell_with_definitions_no_detours(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    let mut affine_cache = Vec::new();
    let mut step_cache = Vec::new();
    let mut no_detour_cache = Vec::new();
    let mut detour_target_cache = Vec::new();
    probe_reaches_adjacent_cell_with_definition_search(
        start,
        end,
        start_definitions,
        end_definitions,
        || probe_reaches_adjacent_cell(start, end, host_support, polygons),
        |start_definition, end_definition| {
            plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement_with_caches(
                start_definition,
                end_definition,
                host_support,
                polygons,
                &mut affine_cache,
                &mut step_cache,
                &mut no_detour_cache,
                &mut detour_target_cache,
            )
        },
    )
}

fn probe_reaches_adjacent_cell_with_definitions_no_step_detours(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    let mut affine_cache = Vec::new();
    let mut step_cache = Vec::new();
    probe_reaches_adjacent_cell_with_definition_search(
        start,
        end,
        start_definitions,
        end_definitions,
        || probe_reaches_adjacent_cell(start, end, host_support, polygons),
        |start_definition, end_definition| {
            plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches(
                start_definition,
                end_definition,
                host_support,
                polygons,
                &mut affine_cache,
                &mut step_cache,
            )
        },
    )
}

fn probe_reaches_adjacent_cell_with_definition_search(
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

    let mut start_definitions = start_definitions.to_vec();
    append_definition_if_missing(&mut start_definitions, axis_plane_definition(start));
    let mut end_definitions = end_definitions.to_vec();
    append_definition_if_missing(&mut end_definitions, axis_plane_definition(end));

    match definition_pair_reachability_backtracking_unknown(
        &start_definitions,
        &end_definitions,
        |start_definition, end_definition| replacement_reaches(start_definition, end_definition),
    ) {
        Ok(true) => Ok(true),
        Ok(false) => {
            if direct_unknown {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(false)
            }
        }
        Err(HypermeshError::UnknownClassification) => Err(HypermeshError::UnknownClassification),
        Err(err) => Err(err),
    }
}

fn definition_pair_reachability_backtracking_unknown(
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
        Ok(false)
    }
}

#[cfg(test)]
fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    let mut no_detour_cache = Vec::new();
    let mut detour_target_cache = Vec::new();
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        &mut no_detour_cache,
        &mut detour_target_cache,
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

fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    no_detour_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
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
    let mut detours_for = |start: &Point3, end: &Point3| {
        cached_detour_target_family_with(&mut *detour_target_cache, start, end, || {
            detours_for_query(start, end)
        })
    };
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
        start,
        end,
        polygons,
        &[start.clone(), end.clone()],
        start_definitions,
        end_definitions,
        &mut trace_without_detours,
        &mut detours_for,
    )
}

fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[Point3],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut surface_cache = Vec::new();
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
        start,
        end,
        polygons,
        visited_points,
        start_definitions,
        end_definitions,
        &mut surface_cache,
        &mut |point| point_lies_on_traced_surface(point, polygons),
        trace_without_detours,
        detours_for,
    )
}

fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[Point3],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
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
    match trace_without_detours(start, end, start_definitions, end_definitions) {
        Ok(true) => return Ok(true),
        Ok(false) => {}
        Err(HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }

    for detour in detours_for(start, end)? {
        let already_visited = point_family_contains(visited_points, &detour.point);
        let on_surface = if already_visited {
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
        next_visited_points.push(detour.point.clone());

        let first_leg =
            match probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
            start,
            &detour.point,
            polygons,
            &next_visited_points,
            start_definitions,
            &detour.definitions,
            surface_cache,
            surface_query,
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
        match probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
            &detour.point,
            end,
            polygons,
            &next_visited_points,
            &detour.definitions,
            end_definitions,
            surface_cache,
            surface_query,
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
#[allow(dead_code)]
fn plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let mut affine_cache = Vec::new();
    let mut step_cache = Vec::new();
    let mut no_detour_cache = Vec::new();
    let mut detour_target_cache = Vec::new();
    plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement_with_caches(
        start_planes,
        end_planes,
        host_support,
        polygons,
        &mut affine_cache,
        &mut step_cache,
        &mut no_detour_cache,
        &mut detour_target_cache,
    )
}

fn plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement_with_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    no_detour_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
) -> HypermeshResult<bool> {
    plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
        start_planes,
        end_planes,
        affine_cache,
        step_cache,
        |current, next, current_definitions, next_definitions| {
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
                current,
                next,
                polygons,
                current_definitions,
                next_definitions,
                no_detour_cache,
                detour_target_cache,
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
        },
    )
}

#[cfg(test)]
#[allow(dead_code)]
fn plane_replacement_path_reaches_adjacent_cell_without_step_detours(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let mut affine_cache = Vec::new();
    let mut step_cache = Vec::new();
    plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches(
        start_planes,
        end_planes,
        host_support,
        polygons,
        &mut affine_cache,
        &mut step_cache,
    )
}

fn plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
) -> HypermeshResult<bool> {
    plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
        start_planes,
        end_planes,
        affine_cache,
        step_cache,
        |current, next, _current_definitions, _next_definitions| {
            probe_reaches_adjacent_cell(current, next, host_support, polygons)
        },
    )
}

fn plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    mut trace_step: impl FnMut(&Point3, &Point3, &[[Plane; 3]], &[[Plane; 3]]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for ordering in AXIS_ORDERINGS {
        let mut current_planes = start_planes.clone();
        let mut current_point =
            match cached_affine_from_planes_with(&mut *affine_cache, &current_planes, || {
                affine_from_planes(&current_planes)
            }) {
                Ok(point) => point,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
        let mut valid = true;

        for plane_index in ordering {
            let mut next_planes = current_planes.clone();
            next_planes[plane_index] = end_planes[plane_index].clone();
            let next_point =
                match cached_affine_from_planes_with(&mut *affine_cache, &next_planes, || {
                    affine_from_planes(&next_planes)
                }) {
                    Ok(point) => point,
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                        valid = false;
                        break;
                    }
                    Err(err) => return Err(err),
                };
            if next_point != current_point {
                let reachable = match cached_plane_replacement_reachability_step_with(
                    &mut *step_cache,
                    &current_point,
                    &next_point,
                    &current_planes,
                    &next_planes,
                    || {
                        trace_step(
                            &current_point,
                            &next_point,
                            std::slice::from_ref(&current_planes),
                            std::slice::from_ref(&next_planes),
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
            }
            current_point = next_point;
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

fn cached_plane_replacement_reachability_step_with(
    cache: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    current_point: &Point3,
    next_point: &Point3,
    current_planes: &[Plane; 3],
    next_planes: &[Plane; 3],
    trace: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.current_point == *current_point
            && existing.next_point == *next_point
            && definition_planes_match_as_sets(&existing.current_planes, current_planes)
            && definition_planes_match_as_sets(&existing.next_planes, next_planes)
    }) {
        return existing.result.clone();
    }

    let result = trace();
    cache.push(PlaneReplacementReachabilityStepCacheEntry {
        current_point: current_point.clone(),
        next_point: next_point.clone(),
        current_planes: current_planes.clone(),
        next_planes: next_planes.clone(),
        result: result.clone(),
    });
    result
}

fn planes_are_coplanar(left: &Plane, right: &Plane) -> HypermeshResult<bool> {
    let left_coefficients = [&left.normal.x, &left.normal.y, &left.normal.z, &left.offset];
    let right_coefficients = [
        &right.normal.x,
        &right.normal.y,
        &right.normal.z,
        &right.offset,
    ];

    for i in 0..left_coefficients.len() {
        for j in (i + 1)..left_coefficients.len() {
            let determinant = (left_coefficients[i] * right_coefficients[j])
                - (left_coefficients[j] * right_coefficients[i]);
            if classify_real(&determinant)? != Classification::On {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PolygonPointLocation {
    Outside,
    Boundary,
    Interior,
}

fn classify_point_in_polygon(
    point: &Point3,
    polygon: &ConvexPolygon,
) -> HypermeshResult<PolygonPointLocation> {
    if classify_point(point, &polygon.support)? != Classification::On {
        return Ok(PolygonPointLocation::Outside);
    }
    let mut on_edge = false;
    for edge in &polygon.edges {
        match classify_point(point, edge)? {
            Classification::Positive => return Ok(PolygonPointLocation::Outside),
            Classification::On => on_edge = true,
            Classification::Negative => {}
        }
    }
    if on_edge {
        Ok(PolygonPointLocation::Boundary)
    } else {
        Ok(PolygonPointLocation::Interior)
    }
}

fn segment_plane_crossing(
    start: &Point3,
    end: &Point3,
    plane: &Plane,
) -> HypermeshResult<Option<Point3>> {
    let start_value = plane.expression_at_point(start);
    let end_value = plane.expression_at_point(end);
    let start_class = crate::geometry::classify_real(&start_value)?;
    let end_class = crate::geometry::classify_real(&end_value)?;

    if start_class == Classification::On || end_class == Classification::On {
        return Ok(None);
    }
    if start_class == end_class {
        return Ok(None);
    }

    let denom = &start_value - &end_value;
    let t = (start_value / denom).map_err(|_| HypermeshError::UnknownClassification)?;
    Ok(Some(Point3::new(
        &start.x + &(t.clone() * (&end.x - &start.x)),
        &start.y + &(t.clone() * (&end.y - &start.y)),
        &start.z + &(t * (&end.z - &start.z)),
    )))
}

fn point_strictly_between_axis(
    point: &Point3,
    start: &Point3,
    end: &Point3,
    axis: usize,
) -> HypermeshResult<bool> {
    let start_to_point = compare_real(axis_ref(point, axis), axis_ref(start, axis))?;
    let point_to_end = compare_real(axis_ref(point, axis), axis_ref(end, axis))?;
    Ok((start_to_point.is_gt() && point_to_end.is_lt())
        || (start_to_point.is_lt() && point_to_end.is_gt()))
}

fn sort_crossing_events(
    events: &mut Vec<CrossingEvent>,
    axis: usize,
    dir_sign: i32,
) -> HypermeshResult<()> {
    let mut sorted: Vec<CrossingEvent> = Vec::with_capacity(events.len());
    for event in events.drain(..) {
        let mut insert_at = sorted.len();
        for (index, existing) in sorted.iter().enumerate() {
            let order = compare_real(
                axis_ref(&event.point, axis),
                axis_ref(&existing.point, axis),
            )?;
            if (dir_sign > 0 && order.is_lt()) || (dir_sign < 0 && order.is_gt()) {
                insert_at = index;
                break;
            }
        }
        sorted.insert(insert_at, event);
    }
    *events = sorted;
    Ok(())
}

fn dominant_normal_axis(plane: &Plane) -> HypermeshResult<usize> {
    let abs = [
        plane.normal.x.clone().abs(),
        plane.normal.y.clone().abs(),
        plane.normal.z.clone().abs(),
    ];
    let mut best = 0;
    for axis in 1..3 {
        if compare_real(&abs[axis], &abs[best])?.is_gt() {
            best = axis;
        }
    }
    Ok(best)
}

fn centroid(points: &[Point3]) -> HypermeshResult<Option<Point3>> {
    if points.is_empty() {
        return Ok(None);
    }
    let mut sum = Point3::origin();
    for point in points {
        sum.x += point.x.clone();
        sum.y += point.y.clone();
        sum.z += point.z.clone();
    }
    let denom = Real::from(points.len() as u64);
    Ok(Some(Point3::new(
        (sum.x / denom.clone()).map_err(|_| HypermeshError::UnknownClassification)?,
        (sum.y / denom.clone()).map_err(|_| HypermeshError::UnknownClassification)?,
        (sum.z / denom).map_err(|_| HypermeshError::UnknownClassification)?,
    )))
}

fn halfspace_cell_geometry_seed_candidates(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Vec<Point3>> {
    let vertices = feasible_halfspace_cell_vertices(halfspaces)?;
    halfspace_cell_geometry_seed_candidates_from_vertices(&vertices)
}

fn halfspace_cell_geometry_seed_candidates_from_vertices(
    vertices: &[Point3],
) -> HypermeshResult<Vec<Point3>> {
    Ok(halfspace_centroid_subset_seed_family_from_vertices(vertices)?.seeds)
}

fn halfspace_centroid_subset_seed_family_from_vertices(
    vertices: &[Point3],
) -> HypermeshResult<HalfspaceSeedFamilyState> {
    halfspace_centroid_subset_seed_family_from_vertices_with(vertices, centroid)
}

fn halfspace_centroid_subset_seed_family_from_vertices_with(
    vertices: &[Point3],
    mut center_of: impl FnMut(&[Point3]) -> HypermeshResult<Option<Point3>>,
) -> HypermeshResult<HalfspaceSeedFamilyState> {
    let mut candidates = Vec::new();
    let mut subset = Vec::new();
    let mut saw_unknown = false;
    collect_halfspace_centroid_subset_candidates(
        &mut candidates,
        vertices,
        0,
        &mut subset,
        &mut saw_unknown,
        &mut center_of,
    )?;
    if candidates.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(HalfspaceSeedFamilyState {
            seeds: candidates,
            saw_unknown,
        })
    }
}

fn collect_halfspace_centroid_subset_candidates(
    candidates: &mut Vec<Point3>,
    vertices: &[Point3],
    start: usize,
    subset: &mut Vec<Point3>,
    saw_unknown: &mut bool,
    center_of: &mut impl FnMut(&[Point3]) -> HypermeshResult<Option<Point3>>,
) -> HypermeshResult<()> {
    for index in start..vertices.len() {
        subset.push(vertices[index].clone());
        if subset.len() >= 2 {
            match center_of(subset) {
                Ok(Some(center)) => push_unique_halfspace_seed(candidates, center),
                Ok(None) => {}
                Err(HypermeshError::UnknownClassification) => {
                    *saw_unknown = true;
                }
                Err(err) => return Err(err),
            }
        }
        collect_halfspace_centroid_subset_candidates(
            candidates,
            vertices,
            index + 1,
            subset,
            saw_unknown,
            center_of,
        )?;
        subset.pop();
    }
    Ok(())
}

fn interior_leaf_points(leaf: &ConvexPolygon) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let vertices = leaf.vertices()?;
    if vertices.is_empty() {
        return Ok(Vec::new());
    }

    let mut points = strict_leaf_witness_points(leaf, &vertices)?;
    let witness_points = points
        .iter()
        .map(|point| point.point.clone())
        .collect::<Vec<_>>();
    extend_interior_leaf_points_backtracking_unknown(
        &mut points,
        witness_points.iter(),
        |witness| shifted_edge_interior_points(leaf, witness),
    )?;

    Ok(points)
}

fn strict_leaf_witness_points(
    leaf: &ConvexPolygon,
    vertices: &[Point3],
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    strict_leaf_witness_points_with_seed_families(
        leaf,
        vertices,
        |leaf, vertices, bounds, halfspaces, report| {
            leaf_witness_seed_families(leaf, vertices, bounds, halfspaces, report)
        },
    )
}

fn strict_leaf_witness_points_with_seed_families(
    leaf: &ConvexPolygon,
    vertices: &[Point3],
    mut seed_families_for: impl FnMut(
        &ConvexPolygon,
        &[Point3],
        &Aabb,
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
    ) -> HypermeshResult<LeafWitnessSeedFamilies>,
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let bounds = leaf_bounds(vertices)?;
    let halfspaces = leaf_halfspaces(leaf);
    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }

    let mut points = Vec::new();
    let LeafWitnessSeedFamilies {
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
        saw_unknown: seed_saw_unknown,
    } = seed_families_for(leaf, vertices, &bounds, &halfspaces, report.as_ref())?;
    saw_unknown |= seed_saw_unknown;
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) = dedupe_shifted_halfspace_seed_families(
        report_witness,
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    );

    extend_leaf_point_builds_backtracking_unknown(&mut points, seeds.iter(), |seed| {
        let active_planes = active_planes_from_optional_report(report.as_ref(), seed);
        build_strict_leaf_point(leaf, seed, &halfspaces, active_planes, false)
    })?;

    let shifted_witnesses = shifted_halfspace_witness_family_or_empty(
        {
            let mut shifted_witnesses = Vec::new();
            extend_shifted_halfspace_seed_families_backtracking_unknown(
                &mut shifted_witnesses,
                [seeds, shifted_vertices, shifted_geometry_seeds],
                |seed| shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    extend_leaf_point_builds_backtracking_unknown(
        &mut points,
        shifted_witnesses.iter(),
        |shifted| build_strict_leaf_point_from_shifted_witness(leaf, shifted),
    )?;
    let direct_witnesses = points
        .iter()
        .map(|point| point.point.clone())
        .collect::<Vec<_>>();
    extend_interior_leaf_points_backtracking_unknown(
        &mut points,
        direct_witnesses.iter(),
        |witness| strict_leaf_cell_points(leaf, witness),
    )?;

    if points.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_interior_points_uncertified(&mut points);
        }
        Ok(points)
    }
}

fn leaf_witness_seed_families(
    leaf: &ConvexPolygon,
    _vertices: &[Point3],
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> HypermeshResult<LeafWitnessSeedFamilies> {
    let mut saw_unknown = false;
    let (generic_seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            bounds,
            halfspaces,
            report,
            &mut saw_unknown,
        )?;
    let mut seeds = generic_seeds;

    extend_strict_halfspace_seed_families_backtracking_unknown(
        &mut seeds,
        [collect_strict_halfspace_seed_family(
            Ok(shifted_geometry_seeds.clone()),
            |candidate| point_strictly_inside_leaf(candidate, leaf),
        )],
    )?;

    if seed_family_search_failed_without_any_seed(
        &seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        saw_unknown,
    ) {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(LeafWitnessSeedFamilies {
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
            saw_unknown,
        })
    }
}

#[cfg(test)]
fn strict_leaf_witness_seeds(
    leaf: &ConvexPolygon,
    vertices: &[Point3],
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> HypermeshResult<Vec<Point3>> {
    leaf_witness_seed_families(leaf, vertices, bounds, halfspaces, report)
        .map(|families| families.seeds)
}

fn leaf_bounds(vertices: &[Point3]) -> HypermeshResult<Aabb> {
    let Some(first) = vertices.first() else {
        return Err(HypermeshError::UnknownClassification);
    };

    let mut min = first.clone();
    let mut max = first.clone();
    for vertex in &vertices[1..] {
        for axis in 0..3 {
            if compare_real(axis_ref(vertex, axis), axis_ref(&min, axis))?.is_lt() {
                *axis_mut(&mut min, axis) = axis_ref(vertex, axis).clone();
            }
            if compare_real(axis_ref(vertex, axis), axis_ref(&max, axis))?.is_gt() {
                *axis_mut(&mut max, axis) = axis_ref(vertex, axis).clone();
            }
        }
    }

    Ok(Aabb::new(min, max))
}

fn leaf_halfspaces(leaf: &ConvexPolygon) -> Vec<LimitPlane3> {
    let mut halfspaces = Vec::with_capacity(leaf.edges.len() + 2);
    halfspaces.push(limit_plane_from_plane(&leaf.support));
    halfspaces.push(limit_plane_from_plane(&leaf.support.inverted()));
    for edge in &leaf.edges {
        halfspaces.push(limit_plane_from_plane(edge));
    }
    halfspaces
}

#[cfg(test)]
pub(crate) fn certified_leaf_test_point(
    support: &Plane,
    edges: &[Plane],
) -> HypermeshResult<Option<HomogeneousPoint3>> {
    let points = certified_leaf_interior_points(support, edges)?;
    let Some(point) = points
        .iter()
        .find(|point| !point.planes.is_empty())
        .or_else(|| points.first())
    else {
        return Ok(None);
    };
    Ok(Some(HomogeneousPoint3::new(
        point.point.x.clone(),
        point.point.y.clone(),
        point.point.z.clone(),
        Real::one(),
    )))
}

pub(crate) fn certified_leaf_test_points(
    support: &Plane,
    edges: &[Plane],
) -> HypermeshResult<Vec<HomogeneousPoint3>> {
    Ok(certified_leaf_interior_points(support, edges)?
        .into_iter()
        .map(|point| {
            HomogeneousPoint3::new(point.point.x, point.point.y, point.point.z, Real::one())
        })
        .collect())
}

pub(crate) fn certified_leaf_interior_points(
    support: &Plane,
    edges: &[Plane],
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let leaf = ConvexPolygon {
        support: support.clone(),
        edges: edges.to_vec(),
        mesh_index: -1,
        polygon_index: -1,
        delta_w: WindingNumberTransitionVector::new(),
        approx_bounds: None,
    };
    interior_leaf_points(&leaf)
}

fn shifted_edge_interior_points(
    leaf: &ConvexPolygon,
    strict_interior: &Point3,
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let mut points = Vec::with_capacity(leaf.vertex_count());
    let half = (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;

    for first_edge in 0..leaf.vertex_count() {
        let second_edge = (first_edge + 1) % leaf.vertex_count();
        let first_margin = leaf.edges[first_edge].expression_at_point(strict_interior);
        let second_margin = leaf.edges[second_edge].expression_at_point(strict_interior);
        if classify_real(&first_margin)? != Classification::Negative
            || classify_real(&second_margin)? != Classification::Negative
        {
            continue;
        }

        let first_shifted =
            inward_shifted_edge_plane(&leaf.edges[first_edge], &first_margin, &half);
        let second_shifted =
            inward_shifted_edge_plane(&leaf.edges[second_edge], &second_margin, &half);
        let candidate = intersect_three_planes(&leaf.support, &first_shifted, &second_shifted)
            .to_affine_point()
            .map_err(|_| HypermeshError::UnknownClassification)?;

        if point_strictly_inside_leaf(&candidate, leaf)? {
            push_unique_interior_point(
                &mut points,
                InteriorLeafPoint {
                    point: candidate,
                    planes: vec![[leaf.support.clone(), first_shifted, second_shifted]],
                    uncertified_definition_fallback: false,
                },
            );
        }
    }

    Ok(points)
}

fn inward_shifted_edge_plane(
    edge: &Plane,
    strict_interior_margin: &Real,
    fraction: &Real,
) -> Plane {
    let inward_offset = strict_interior_margin * fraction;
    Plane::new(edge.normal.clone(), &edge.offset - &inward_offset)
}

fn push_unique_interior_point(points: &mut Vec<InteriorLeafPoint>, point: InteriorLeafPoint) {
    if let Some(existing) = points
        .iter_mut()
        .find(|existing| existing.point == point.point)
    {
        for planes in point.planes {
            if !existing
                .planes
                .iter()
                .any(|candidate| definition_planes_match_as_sets(candidate, &planes))
            {
                existing.planes.push(planes);
            }
        }
        existing.uncertified_definition_fallback |= point.uncertified_definition_fallback;
    } else {
        points.push(point);
    }
}

fn extend_interior_leaf_points_backtracking_unknown<'a, T: 'a>(
    points: &mut Vec<InteriorLeafPoint>,
    candidates: impl IntoIterator<Item = &'a T>,
    mut build: impl FnMut(&'a T) -> HypermeshResult<Vec<InteriorLeafPoint>>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(found) => {
                saw_unknown |= found
                    .iter()
                    .any(|point| point.uncertified_definition_fallback);
                for point in found {
                    push_unique_interior_point(points, point);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if points.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_interior_points_uncertified(points);
        }
        Ok(())
    }
}

fn extend_leaf_point_builds_backtracking_unknown<'a, T: 'a>(
    points: &mut Vec<InteriorLeafPoint>,
    candidates: impl IntoIterator<Item = &'a T>,
    mut build: impl FnMut(&'a T) -> HypermeshResult<Option<InteriorLeafPoint>>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(Some(point)) => {
                saw_unknown |= point.uncertified_definition_fallback;
                push_unique_interior_point(points, point)
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if points.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_interior_points_uncertified(points);
        }
        Ok(())
    }
}

fn strict_leaf_cell_points(
    leaf: &ConvexPolygon,
    strict_interior: &Point3,
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let vertices = leaf.vertices()?;
    let bounds = leaf_bounds(&vertices)?;
    let half = (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;
    let mut halfspaces = Vec::with_capacity(leaf.edges.len() + 2);
    halfspaces.push(limit_plane_from_plane(&leaf.support));
    halfspaces.push(limit_plane_from_plane(&leaf.support.inverted()));

    for edge in &leaf.edges {
        let margin = edge.expression_at_point(strict_interior);
        if classify_real(&margin)? != Classification::Negative {
            return Ok(Vec::new());
        }
        halfspaces.push(limit_plane_from_plane(&inward_shifted_edge_plane(
            edge, &margin, &half,
        )));
    }

    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }

    let mut points = Vec::new();
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            &bounds,
            &halfspaces,
            report.as_ref(),
            &mut saw_unknown,
        )?;
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) = dedupe_shifted_halfspace_seed_families(
        report_witness,
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    );
    extend_leaf_point_builds_backtracking_unknown(&mut points, seeds.iter(), |witness| {
        let active_planes = active_planes_from_optional_report(report.as_ref(), witness);
        build_strict_leaf_point(leaf, witness, &halfspaces, active_planes, false)
    })?;

    let shifted_witnesses = shifted_halfspace_witness_family_or_empty(
        {
            let mut shifted_witnesses = Vec::new();
            extend_shifted_halfspace_seed_families_backtracking_unknown(
                &mut shifted_witnesses,
                [seeds, shifted_vertices, shifted_geometry_seeds],
                |seed| shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    extend_leaf_point_builds_backtracking_unknown(
        &mut points,
        shifted_witnesses.iter(),
        |shifted| build_strict_leaf_point_from_shifted_witness(leaf, shifted),
    )?;
    if points.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_interior_points_uncertified(&mut points);
        }
        Ok(points)
    }
}

fn build_strict_leaf_point(
    leaf: &ConvexPolygon,
    witness: &Point3,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    inherited_uncertified_definition_fallback: bool,
) -> HypermeshResult<Option<InteriorLeafPoint>> {
    if !point_strictly_inside_leaf(witness, leaf)? {
        return Ok(None);
    }

    let (planes, uncertified_definition_fallback) =
        match leaf_interior_definitions_from_active_halfspaces(
            witness,
            &leaf.support,
            halfspaces,
            active_planes,
        ) {
            Ok(found) => (found.definitions, found.saw_unknown),
            Err(HypermeshError::UnknownClassification) => {
                (vec![axis_plane_definition(witness)], true)
            }
            Err(err) => return Err(err),
        };
    Ok(Some(InteriorLeafPoint {
        point: witness.clone(),
        planes,
        uncertified_definition_fallback: inherited_uncertified_definition_fallback
            || uncertified_definition_fallback,
    }))
}

fn build_strict_leaf_point_from_shifted_witness(
    leaf: &ConvexPolygon,
    witness: &ShiftedHalfspaceWitness,
) -> HypermeshResult<Option<InteriorLeafPoint>> {
    if !point_strictly_inside_leaf(&witness.point, leaf)? {
        return Ok(None);
    }

    let mut planes = Vec::new();
    let mut saw_unknown = false;
    for family in &witness.families {
        match leaf_interior_definitions_from_active_halfspaces(
            &witness.point,
            &leaf.support,
            &family.halfspaces,
            family.active_planes,
        ) {
            Ok(found) => {
                saw_unknown |= found.saw_unknown;
                extend_unique_definition_families(&mut planes, found.definitions);
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    if planes.is_empty() {
        if saw_unknown {
            planes.push(axis_plane_definition(&witness.point));
        } else {
            return Ok(None);
        }
    }

    Ok(Some(InteriorLeafPoint {
        point: witness.point.clone(),
        planes,
        uncertified_definition_fallback: witness.uncertified_definition_fallback || saw_unknown,
    }))
}

fn witness_active_planes(
    report_witness: Option<&Point3>,
    active_planes: [Option<usize>; 3],
    witness: &Point3,
) -> [Option<usize>; 3] {
    if report_witness.is_some_and(|point| point == witness) {
        active_planes
    } else {
        [None, None, None]
    }
}

fn limit_plane_from_plane(plane: &Plane) -> LimitPlane3 {
    LimitPlane3::new(plane.normal.clone(), plane.offset.clone())
}

fn leaf_interior_definitions_from_active_halfspaces(
    witness: &Point3,
    support: &Plane,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
) -> HypermeshResult<DefinitionFamilyState> {
    let axis_definition = axis_plane_definition(witness);
    let mut definitions = Vec::new();
    let mut saw_unknown = false;
    let mut active = Vec::new();
    for index in active_planes.into_iter().flatten() {
        let Some(halfspace) = halfspaces.get(index) else {
            continue;
        };
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if plane == *support || plane == support.inverted() {
            continue;
        }
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if plane == *support || plane == support.inverted() {
            continue;
        }
        if !compare_real(&plane.expression_at_point(witness), &Real::zero())?.is_eq() {
            continue;
        }
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    if active.len() >= 2 {
        for first in 0..active.len() {
            for second in (first + 1)..active.len() {
                push_verified_leaf_definition(
                    &mut definitions,
                    [
                        support.clone(),
                        active[first].clone(),
                        active[second].clone(),
                    ],
                    witness,
                    &mut saw_unknown,
                )?;
            }
        }
    }

    for plane in &active {
        for axis in axis_definition.iter().cloned() {
            push_verified_leaf_definition(
                &mut definitions,
                [support.clone(), plane.clone(), axis],
                witness,
                &mut saw_unknown,
            )?;
        }
    }

    for first_axis in 0..3 {
        for second_axis in (first_axis + 1)..3 {
            push_verified_leaf_definition(
                &mut definitions,
                [
                    support.clone(),
                    axis_definition[first_axis].clone(),
                    axis_definition[second_axis].clone(),
                ],
                witness,
                &mut saw_unknown,
            )?;
        }
    }

    if definitions.is_empty() {
        return Err(HypermeshError::UnknownClassification);
    }
    Ok(DefinitionFamilyState {
        definitions,
        saw_unknown,
    })
}

fn push_verified_leaf_definition(
    definitions: &mut Vec<[Plane; 3]>,
    definition: [Plane; 3],
    witness: &Point3,
    saw_unknown: &mut bool,
) -> HypermeshResult<()> {
    match intersect_three_planes(&definition[0], &definition[1], &definition[2]).to_affine_point() {
        Ok(point) if point == *witness => {
            if !definitions
                .iter()
                .any(|existing| definition_planes_match_as_sets(existing, &definition))
            {
                definitions.push(definition);
            }
        }
        Ok(_) => {}
        Err(_) => {
            *saw_unknown = true;
        }
    }
    Ok(())
}

fn point_strictly_inside_leaf(point: &Point3, leaf: &ConvexPolygon) -> HypermeshResult<bool> {
    let homogeneous = HomogeneousPoint3::new(
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
        Real::one(),
    );
    leaf.contains_point_strictly(&homogeneous)
}

fn bounded_probes_from_interior(
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    positive_side: bool,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut probes = Vec::new();
    let mut saw_unknown = false;

    extend_probe_families_backtracking_unknown(
        &mut probes,
        adjacent_normal_probes(interior, support, bounds, polygons, positive_side),
        &mut saw_unknown,
    )?;

    for axis in probe_axes(support)? {
        let normal_sign = crate::geometry::classify_real(axis_ref(&support.normal, axis))?;
        if normal_sign == Classification::On {
            continue;
        }

        let direction_positive = (normal_sign == Classification::Positive) == positive_side;
        let axis_value = axis_ref(&interior.point, axis);
        let room = if direction_positive {
            axis_ref(&bounds.max, axis) - axis_value
        } else {
            axis_value - axis_ref(&bounds.min, axis)
        };
        if !compare_real(&room, &Real::zero())?.is_gt() {
            continue;
        }

        extend_probe_families_backtracking_unknown(
            &mut probes,
            adjacent_axis_probes(
                interior,
                support,
                bounds,
                polygons,
                axis,
                direction_positive,
            ),
            &mut saw_unknown,
        )?;
    }

    if probes.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

fn extend_probe_families_backtracking_unknown(
    probes: &mut Vec<ProbePoint>,
    family: HypermeshResult<Vec<ProbePoint>>,
    saw_unknown: &mut bool,
) -> HypermeshResult<()> {
    match family {
        Ok(found) => {
            *saw_unknown |= found
                .iter()
                .any(|probe| probe.uncertified_definition_fallback);
            for probe in found {
                push_unique_probe_point(probes, probe);
            }
            Ok(())
        }
        Err(HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            Ok(())
        }
        Err(err) => Err(err),
    }
}

fn probe_definitions_from_active_halfspaces(
    witness: &Point3,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    extra_planes: &[Plane],
) -> HypermeshResult<DefinitionFamilyState> {
    let axis_definition = axis_plane_definition(witness);
    let mut definitions = Vec::new();
    let mut saw_unknown = false;
    let mut active = Vec::new();

    for plane in extra_planes {
        if !active.iter().any(|existing| existing == plane) {
            active.push(plane.clone());
        }
    }

    for index in active_planes.into_iter().flatten() {
        let Some(halfspace) = halfspaces.get(index) else {
            continue;
        };
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if !compare_real(&plane.expression_at_point(witness), &Real::zero())?.is_eq() {
            continue;
        }
        if !active.iter().any(|existing| existing == &plane) {
            active.push(plane);
        }
    }

    for first in 0..active.len() {
        for second in (first + 1)..active.len() {
            for third in (second + 1)..active.len() {
                push_verified_probe_definition(
                    &mut definitions,
                    [
                        active[first].clone(),
                        active[second].clone(),
                        active[third].clone(),
                    ],
                    witness,
                    &mut saw_unknown,
                )?;
            }
        }
    }

    for first in 0..active.len() {
        for second in (first + 1)..active.len() {
            for axis in 0..3 {
                push_verified_probe_definition(
                    &mut definitions,
                    [
                        active[first].clone(),
                        active[second].clone(),
                        axis_definition[axis].clone(),
                    ],
                    witness,
                    &mut saw_unknown,
                )?;
            }
        }
    }

    for plane in &active {
        for first_axis in 0..3 {
            for second_axis in (first_axis + 1)..3 {
                push_verified_probe_definition(
                    &mut definitions,
                    [
                        plane.clone(),
                        axis_definition[first_axis].clone(),
                        axis_definition[second_axis].clone(),
                    ],
                    witness,
                    &mut saw_unknown,
                )?;
            }
        }
    }

    push_verified_probe_definition(&mut definitions, axis_definition, witness, &mut saw_unknown)?;
    Ok(DefinitionFamilyState {
        definitions,
        saw_unknown,
    })
}

fn probe_definitions_or_axis(
    witness: &Point3,
    result: HypermeshResult<DefinitionFamilyState>,
) -> HypermeshResult<(Vec<[Plane; 3]>, bool)> {
    match result {
        Ok(found) => Ok((found.definitions, found.saw_unknown)),
        Err(HypermeshError::UnknownClassification) => {
            Ok((vec![axis_plane_definition(witness)], true))
        }
        Err(err) => Err(err),
    }
}

fn push_verified_probe_definition(
    definitions: &mut Vec<[Plane; 3]>,
    definition: [Plane; 3],
    witness: &Point3,
    saw_unknown: &mut bool,
) -> HypermeshResult<()> {
    match affine_from_planes(&definition) {
        Ok(point) if point == *witness => {
            if !definitions
                .iter()
                .any(|existing| definition_planes_match_as_sets(existing, &definition))
            {
                definitions.push(definition);
            }
        }
        Ok(_) => {}
        Err(HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
        }
        Err(err) => return Err(err),
    }
    Ok(())
}

fn adjacent_normal_probes(
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    positive_side: bool,
) -> HypermeshResult<Vec<ProbePoint>> {
    adjacent_normal_probes_with_queries(
        interior,
        support,
        bounds,
        polygons,
        positive_side,
        |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        |point, polygon| classify_point_in_polygon(point, polygon),
        |corridor, stop_point| {
            collect_normal_probe_targets(&interior.planes, |definition| {
                if let Some(definition) = definition
                    && !normal_probe_definition_preserves_support_direction(definition, support)?
                {
                    return Ok(Vec::new());
                }
                strict_normal_probe_targets(
                    interior,
                    support,
                    corridor,
                    definition,
                    stop_point,
                    positive_side,
                )
            })
        },
    )
}

fn adjacent_normal_probes_with_queries(
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    positive_side: bool,
    mut denom_for: impl FnMut(&Point3, &Point3, &ConvexPolygon) -> HypermeshResult<Real>,
    mut classify_point_on_polygon: impl FnMut(
        &Point3,
        &ConvexPolygon,
    ) -> HypermeshResult<PolygonPointLocation>,
    mut build: impl FnMut(&Aabb, &Point3) -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let direction = if positive_side {
        support.normal.clone()
    } else {
        Point3::new(
            -support.normal.x.clone(),
            -support.normal.y.clone(),
            -support.normal.z.clone(),
        )
    };

    let (stop_values, mut saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior.point,
        &direction,
        support,
        bounds,
        polygons,
        &mut denom_for,
        &mut classify_point_on_polygon,
    )?;
    let mut probes = Vec::new();

    for stop_t in stop_values {
        if !compare_real(&stop_t, &Real::zero())?.is_gt() {
            continue;
        }
        let stop_point = offset_point(&interior.point, &direction, &stop_t);
        let corridor = bounds_between_points(&interior.point, &stop_point)?;
        extend_probe_families_backtracking_unknown(
            &mut probes,
            build(&corridor, &stop_point),
            &mut saw_unknown,
        )?;
    }

    if probes.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

fn adjacent_normal_probe_stop_values_with_queries(
    interior: &Point3,
    direction: &Point3,
    support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    denom_for: &mut impl FnMut(&Point3, &Point3, &ConvexPolygon) -> HypermeshResult<Real>,
    classify_point_on_polygon: &mut impl FnMut(
        &Point3,
        &ConvexPolygon,
    ) -> HypermeshResult<PolygonPointLocation>,
) -> HypermeshResult<(Vec<Real>, bool)> {
    let Some(bound_stop) = normal_probe_bounds_stop(interior, direction, bounds)? else {
        return Ok((Vec::new(), false));
    };

    let mut stop_values = vec![bound_stop.clone()];
    let mut saw_unknown = false;

    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }
        if planes_are_coplanar(&polygon.support, support)? {
            continue;
        }

        let start_value = polygon.support.expression_at_point(interior);
        let start_class = match classify_real(&start_value) {
            Ok(classification) => classification,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if start_class == Classification::On {
            let point_location = match classify_point_on_polygon(interior, polygon) {
                Ok(point_location) => point_location,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
            match point_location {
                PolygonPointLocation::Outside => {}
                PolygonPointLocation::Boundary => {
                    saw_unknown = true;
                }
                PolygonPointLocation::Interior => {
                    return Ok((Vec::new(), saw_unknown));
                }
            }
            continue;
        }

        let denom = match denom_for(interior, direction, polygon) {
            Ok(denom) => denom,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        let denom_class = match classify_real(&denom) {
            Ok(classification) => classification,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if denom_class == Classification::On {
            continue;
        }
        let crossing_t =
            match ((-start_value) / denom).map_err(|_| HypermeshError::UnknownClassification) {
                Ok(crossing_t) => crossing_t,
                Err(HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
        if !positive_real_strictly_before(&crossing_t, &bound_stop)? {
            continue;
        }

        let crossing = offset_point(interior, direction, &crossing_t);
        let point_location = match classify_point_on_polygon(&crossing, polygon) {
            Ok(point_location) => point_location,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        match point_location {
            PolygonPointLocation::Outside => continue,
            PolygonPointLocation::Boundary => {
                saw_unknown = true;
                continue;
            }
            PolygonPointLocation::Interior => {}
        }

        let mut insert_at = stop_values.len();
        let mut duplicate = false;
        for (index, existing) in stop_values.iter().enumerate() {
            let order = compare_real(&crossing_t, existing)?;
            if order.is_eq() {
                duplicate = true;
                break;
            }
            if order.is_lt() {
                insert_at = index;
                break;
            }
        }
        if !duplicate {
            stop_values.insert(insert_at, crossing_t);
        }
    }

    Ok((stop_values, saw_unknown))
}

fn push_unique_probe_point(probes: &mut Vec<ProbePoint>, probe: ProbePoint) {
    if let Some(existing) = probes
        .iter_mut()
        .find(|existing| existing.point == probe.point && existing.side == probe.side)
    {
        for definition in probe.planes {
            if !existing
                .planes
                .iter()
                .any(|candidate| definition_planes_match_as_sets(candidate, &definition))
            {
                existing.planes.push(definition);
            }
        }
        existing.uncertified_definition_fallback |= probe.uncertified_definition_fallback;
    } else {
        probes.push(probe);
    }
}

fn collect_normal_probe_targets(
    definitions: &[[Plane; 3]],
    mut search: impl FnMut(Option<&[Plane; 3]>) -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut probes = Vec::new();
    let mut saw_unknown = false;
    let definitions = unique_definition_family(definitions);
    for definition in &definitions {
        match search(Some(definition)) {
            Ok(found) => {
                for probe in found {
                    push_unique_probe_point(&mut probes, probe);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    match search(None) {
        Ok(found) => {
            for probe in found {
                push_unique_probe_point(&mut probes, probe);
            }
        }
        Err(HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }
    if probes.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

fn normal_probe_definition_preserves_support_direction(
    definition: &[Plane; 3],
    support: &Plane,
) -> HypermeshResult<bool> {
    Ok(
        classify_real(&dot_direction(&definition[1].normal, &support.normal))?
            == Classification::On
            && classify_real(&dot_direction(&definition[2].normal, &support.normal))?
                == Classification::On,
    )
}

fn strict_normal_probe_targets(
    interior: &InteriorLeafPoint,
    support: &Plane,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
    positive_side: bool,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    halfspaces.push(normal_stop_halfspace(support, stop_point, positive_side));

    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }
    let mut probes = Vec::new();
    let mut extra_planes = Vec::new();
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            corridor,
            &halfspaces,
            report.as_ref(),
            &mut saw_unknown,
        )?;
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) = dedupe_shifted_halfspace_seed_families(
        report_witness,
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    );
    for definition in &interior.planes {
        for plane in &definition[1..] {
            if !extra_planes.iter().any(|existing| existing == plane) {
                extra_planes.push(plane.clone());
            }
        }
    }

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_probe_point(
            witness,
            support,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            &extra_planes,
            false,
        )
    })?;

    let shifted_witnesses = shifted_halfspace_witness_family_or_empty(
        {
            let mut shifted_witnesses = Vec::new();
            extend_shifted_halfspace_seed_families_backtracking_unknown(
                &mut shifted_witnesses,
                [seeds, shifted_vertices, shifted_geometry_seeds],
                |seed| shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    extend_probe_point_builds_backtracking_unknown(
        &mut probes,
        shifted_witnesses.iter(),
        |shifted| build_probe_point_from_shifted_witness(shifted, support, &extra_planes),
    )?;
    if probes.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

fn bounds_between_points(start: &Point3, end: &Point3) -> HypermeshResult<Aabb> {
    let mut min = Point3::origin();
    let mut max = Point3::origin();
    for axis in 0..3 {
        let start_value = axis_ref(start, axis);
        let end_value = axis_ref(end, axis);
        if compare_real(start_value, end_value)?.is_le() {
            *axis_mut(&mut min, axis) = start_value.clone();
            *axis_mut(&mut max, axis) = end_value.clone();
        } else {
            *axis_mut(&mut min, axis) = end_value.clone();
            *axis_mut(&mut max, axis) = start_value.clone();
        }
    }
    Ok(Aabb::new(min, max))
}

fn normal_probe_bounds_stop(
    interior: &Point3,
    direction: &Point3,
    bounds: &Aabb,
) -> HypermeshResult<Option<Real>> {
    let mut stop_t: Option<Real> = None;
    for axis in 0..3 {
        let component = axis_ref(direction, axis);
        match classify_real(component)? {
            Classification::Positive => {
                let room = axis_ref(&bounds.max, axis) - axis_ref(interior, axis);
                if !compare_real(&room, &Real::zero())?.is_gt() {
                    return Ok(None);
                }
                update_positive_stop(
                    &mut stop_t,
                    (room / component.clone())
                        .map_err(|_| HypermeshError::UnknownClassification)?,
                )?;
            }
            Classification::Negative => {
                let room = axis_ref(interior, axis) - axis_ref(&bounds.min, axis);
                if !compare_real(&room, &Real::zero())?.is_gt() {
                    return Ok(None);
                }
                update_positive_stop(
                    &mut stop_t,
                    (room / (-component.clone()))
                        .map_err(|_| HypermeshError::UnknownClassification)?,
                )?;
            }
            Classification::On => {}
        }
    }
    Ok(stop_t)
}

fn update_positive_stop(stop_t: &mut Option<Real>, candidate: Real) -> HypermeshResult<()> {
    if !compare_real(&candidate, &Real::zero())?.is_gt() {
        return Ok(());
    }
    if stop_t
        .as_ref()
        .is_none_or(|current| compare_real(&candidate, current).is_ok_and(|order| order.is_lt()))
    {
        *stop_t = Some(candidate);
    }
    Ok(())
}

fn positive_real_strictly_before(value: &Real, stop: &Real) -> HypermeshResult<bool> {
    Ok(compare_real(value, &Real::zero())?.is_gt() && compare_real(value, stop)?.is_lt())
}

fn dot_direction(left: &Point3, right: &Point3) -> Real {
    Real::signed_product_sum(
        [true, true, true],
        [
            [&left.x, &right.x],
            [&left.y, &right.y],
            [&left.z, &right.z],
        ],
    )
}

fn offset_point(point: &Point3, direction: &Point3, amount: &Real) -> Point3 {
    Point3::new(
        &point.x + &(amount * &direction.x),
        &point.y + &(amount * &direction.y),
        &point.z + &(amount * &direction.z),
    )
}

fn adjacent_axis_probes(
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
    direction_positive: bool,
) -> HypermeshResult<Vec<ProbePoint>> {
    adjacent_axis_probes_with_queries(
        interior,
        support,
        bounds,
        polygons,
        axis,
        direction_positive,
        |interior, endpoint, polygon, _axis| {
            segment_plane_crossing(interior, endpoint, &polygon.support)
        },
        |crossing, polygon| classify_point_in_polygon(crossing, polygon),
        |corridor| {
            collect_axis_probe_targets(&interior.planes, |definition| {
                if let Some(definition) = definition
                    && !axis_probe_definition_preserves_axis_direction(definition, axis)?
                {
                    return Ok(Vec::new());
                }
                strict_axis_probe_targets(
                    interior,
                    support,
                    corridor,
                    axis,
                    direction_positive,
                    definition,
                )
            })
        },
    )
}

fn adjacent_axis_probes_with_queries(
    interior: &InteriorLeafPoint,
    _support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
    direction_positive: bool,
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
    mut build: impl FnMut(&Aabb) -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let (stop_values, mut saw_unknown) = adjacent_axis_probe_stop_values_with_queries(
        &interior.point,
        bounds,
        polygons,
        axis,
        direction_positive,
        &mut crossing_for,
        &mut classify_point_on_polygon,
    )?;
    let start_value = axis_ref(&interior.point, axis);
    let mut probes = Vec::new();

    for stop_value in stop_values {
        if !axis_value_after_start(start_value, &stop_value, direction_positive)? {
            continue;
        }
        let corridor = axis_probe_bounds(&interior.point, axis, &stop_value)?;
        extend_probe_families_backtracking_unknown(
            &mut probes,
            build(&corridor),
            &mut saw_unknown,
        )?;
    }

    if probes.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

fn adjacent_axis_probe_stop_values_with_queries(
    interior: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
    direction_positive: bool,
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
) -> HypermeshResult<(Vec<Real>, bool)> {
    let start_value = axis_ref(interior, axis);
    let bound_value = if direction_positive {
        axis_ref(&bounds.max, axis)
    } else {
        axis_ref(&bounds.min, axis)
    };
    if !axis_value_after_start(start_value, bound_value, direction_positive)? {
        return Ok((Vec::new(), false));
    }

    let mut endpoint = interior.clone();
    *axis_mut(&mut endpoint, axis) = bound_value.clone();
    let mut stop_values = vec![bound_value.clone()];
    let mut saw_unknown = false;

    for polygon in polygons {
        if polygon.mesh_index < 0 {
            continue;
        }

        let Some(crossing) = (match crossing_for(interior, &endpoint, polygon, axis) {
            Ok(crossing) => crossing,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        }) else {
            continue;
        };
        if !point_strictly_between_axis(&crossing, interior, &endpoint, axis)? {
            continue;
        }
        let point_location = match classify_point_on_polygon(&crossing, polygon) {
            Ok(point_location) => point_location,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        match point_location {
            PolygonPointLocation::Outside => continue,
            PolygonPointLocation::Boundary => {
                saw_unknown = true;
                continue;
            }
            PolygonPointLocation::Interior => {}
        }

        let crossing_value = axis_ref(&crossing, axis);
        if !axis_value_after_start(start_value, crossing_value, direction_positive)? {
            continue;
        }

        let mut insert_at = stop_values.len();
        let mut duplicate = false;
        for (index, existing) in stop_values.iter().enumerate() {
            if compare_real(&crossing_value, existing)?.is_eq() {
                duplicate = true;
                break;
            }
            if axis_value_before_stop(&crossing_value, existing, direction_positive)? {
                insert_at = index;
                break;
            }
        }
        if !duplicate {
            stop_values.insert(insert_at, crossing_value.clone());
        }
    }

    Ok((stop_values, saw_unknown))
}

fn axis_probe_bounds(interior: &Point3, axis: usize, stop_value: &Real) -> HypermeshResult<Aabb> {
    let mut min = interior.clone();
    let mut max = interior.clone();
    let start_value = axis_ref(interior, axis);
    if compare_real(start_value, stop_value)?.is_lt() {
        *axis_mut(&mut max, axis) = stop_value.clone();
    } else {
        *axis_mut(&mut min, axis) = stop_value.clone();
    }
    Ok(Aabb::new(min, max))
}

fn collect_axis_probe_targets(
    definitions: &[[Plane; 3]],
    mut search: impl FnMut(Option<&[Plane; 3]>) -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut probes = Vec::new();
    let mut saw_unknown = false;
    let definitions = unique_definition_family(definitions);
    for definition in &definitions {
        match search(Some(definition)) {
            Ok(found) => {
                for probe in found {
                    push_unique_probe_point(&mut probes, probe);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    match search(None) {
        Ok(found) => {
            for probe in found {
                push_unique_probe_point(&mut probes, probe);
            }
        }
        Err(HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }
    if probes.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

fn extend_probe_point_builds_backtracking_unknown<'a, T: 'a>(
    probes: &mut Vec<ProbePoint>,
    candidates: impl IntoIterator<Item = &'a T>,
    mut build: impl FnMut(&'a T) -> HypermeshResult<Option<ProbePoint>>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(Some(probe)) => {
                saw_unknown |= probe.uncertified_definition_fallback;
                push_unique_probe_point(probes, probe)
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if probes.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_probe_points_uncertified(probes);
        }
        Ok(())
    }
}

fn axis_probe_definition_preserves_axis_direction(
    definition: &[Plane; 3],
    axis: usize,
) -> HypermeshResult<bool> {
    Ok(
        classify_real(axis_ref(&definition[1].normal, axis))? == Classification::On
            && classify_real(axis_ref(&definition[2].normal, axis))? == Classification::On,
    )
}

fn strict_axis_probe_targets(
    interior: &InteriorLeafPoint,
    support: &Plane,
    corridor: &Aabb,
    axis: usize,
    positive_side: bool,
    definition: Option<&[Plane; 3]>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }
    let mut probes = Vec::new();
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            corridor,
            &halfspaces,
            report.as_ref(),
            &mut saw_unknown,
        )?;
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) = dedupe_shifted_halfspace_seed_families(
        report_witness,
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    );

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_axis_probe_point(
            witness,
            interior,
            support,
            axis,
            definition,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            false,
        )
    })?;

    let shifted_witnesses = shifted_halfspace_witness_family_or_empty(
        {
            let mut shifted_witnesses = Vec::new();
            extend_shifted_halfspace_seed_families_backtracking_unknown(
                &mut shifted_witnesses,
                [seeds, shifted_vertices, shifted_geometry_seeds],
                |seed| shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    extend_probe_point_builds_backtracking_unknown(
        &mut probes,
        shifted_witnesses.iter(),
        |shifted| {
            build_axis_probe_point_from_shifted_witness(
                shifted, interior, support, axis, definition,
            )
        },
    )?;
    if probes.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

fn build_probe_point(
    witness: &Point3,
    support: &Plane,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    extra_planes: &[Plane],
    inherited_uncertified_definition_fallback: bool,
) -> HypermeshResult<Option<ProbePoint>> {
    if !point_satisfies_halfspaces(witness, halfspaces)? {
        return Ok(None);
    }
    let side = classify_point(witness, support)?;
    if side == Classification::On {
        return Ok(None);
    }

    let shifted_support = Plane::new(
        support.normal.clone(),
        &support.offset - &support.expression_at_point(witness),
    );
    let mut all_extra_planes = vec![shifted_support];
    for plane in extra_planes {
        if !all_extra_planes.iter().any(|existing| existing == plane) {
            all_extra_planes.push(plane.clone());
        }
    }

    let (planes, uncertified_definition_fallback) = probe_definitions_or_axis(
        witness,
        probe_definitions_from_active_halfspaces(
            witness,
            halfspaces,
            active_planes,
            &all_extra_planes,
        ),
    )?;
    Ok(Some(ProbePoint {
        point: witness.clone(),
        side,
        planes,
        uncertified_definition_fallback: inherited_uncertified_definition_fallback
            || uncertified_definition_fallback,
    }))
}

fn build_probe_point_from_shifted_witness(
    witness: &ShiftedHalfspaceWitness,
    support: &Plane,
    extra_planes: &[Plane],
) -> HypermeshResult<Option<ProbePoint>> {
    let side = classify_point(&witness.point, support)?;
    if side == Classification::On {
        return Ok(None);
    }

    let shifted_support = Plane::new(
        support.normal.clone(),
        &support.offset - &support.expression_at_point(&witness.point),
    );
    let mut all_extra_planes = vec![shifted_support];
    for plane in extra_planes {
        if !all_extra_planes.iter().any(|existing| existing == plane) {
            all_extra_planes.push(plane.clone());
        }
    }

    let mut planes = Vec::new();
    let mut saw_unknown = false;
    let mut saw_candidate_family = false;
    for family in &witness.families {
        if !point_satisfies_halfspaces(&witness.point, &family.halfspaces)? {
            continue;
        }
        saw_candidate_family = true;
        match probe_definitions_from_active_halfspaces(
            &witness.point,
            &family.halfspaces,
            family.active_planes,
            &all_extra_planes,
        ) {
            Ok(found) => {
                saw_unknown |= found.saw_unknown;
                extend_unique_definition_families(&mut planes, found.definitions);
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    if planes.is_empty() {
        if saw_unknown && saw_candidate_family {
            planes.push(axis_plane_definition(&witness.point));
        } else {
            return Ok(None);
        }
    }

    Ok(Some(ProbePoint {
        point: witness.point.clone(),
        side,
        planes,
        uncertified_definition_fallback: witness.uncertified_definition_fallback || saw_unknown,
    }))
}

fn build_axis_probe_point(
    witness: &Point3,
    interior: &InteriorLeafPoint,
    support: &Plane,
    axis: usize,
    definition: Option<&[Plane; 3]>,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    inherited_uncertified_definition_fallback: bool,
) -> HypermeshResult<Option<ProbePoint>> {
    if !point_satisfies_halfspaces(witness, halfspaces)? {
        return Ok(None);
    }
    let side = classify_point(witness, support)?;
    if side == Classification::On {
        return Ok(None);
    }

    let (planes, uncertified_definition_fallback) = probe_definitions_or_axis(
        witness,
        axis_probe_definitions(
            interior,
            support,
            axis,
            definition,
            halfspaces,
            active_planes,
            witness,
        ),
    )?;
    Ok(Some(ProbePoint {
        point: witness.clone(),
        side,
        planes,
        uncertified_definition_fallback: inherited_uncertified_definition_fallback
            || uncertified_definition_fallback,
    }))
}

fn build_axis_probe_point_from_shifted_witness(
    witness: &ShiftedHalfspaceWitness,
    interior: &InteriorLeafPoint,
    support: &Plane,
    axis: usize,
    definition: Option<&[Plane; 3]>,
) -> HypermeshResult<Option<ProbePoint>> {
    let side = classify_point(&witness.point, support)?;
    if side == Classification::On {
        return Ok(None);
    }

    let mut planes = Vec::new();
    let mut saw_unknown = false;
    let mut saw_candidate_family = false;
    for family in &witness.families {
        if !point_satisfies_halfspaces(&witness.point, &family.halfspaces)? {
            continue;
        }
        saw_candidate_family = true;
        match axis_probe_definitions(
            interior,
            support,
            axis,
            definition,
            &family.halfspaces,
            family.active_planes,
            &witness.point,
        ) {
            Ok(found) => {
                saw_unknown |= found.saw_unknown;
                extend_unique_definition_families(&mut planes, found.definitions);
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    if planes.is_empty() {
        if saw_unknown && saw_candidate_family {
            planes.push(axis_plane_definition(&witness.point));
        } else {
            return Ok(None);
        }
    }

    Ok(Some(ProbePoint {
        point: witness.point.clone(),
        side,
        planes,
        uncertified_definition_fallback: witness.uncertified_definition_fallback || saw_unknown,
    }))
}

fn axis_probe_definitions(
    interior: &InteriorLeafPoint,
    support: &Plane,
    axis: usize,
    definition: Option<&[Plane; 3]>,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    witness: &Point3,
) -> HypermeshResult<DefinitionFamilyState> {
    let shifted_support = Plane::new(
        support.normal.clone(),
        &support.offset - &support.expression_at_point(witness),
    );
    let axes = other_axes(axis);
    let mut extra_planes = vec![
        shifted_support,
        Plane::axis_aligned(axes[0], axis_ref(&interior.point, axes[0]).clone()),
        Plane::axis_aligned(axes[1], axis_ref(&interior.point, axes[1]).clone()),
    ];
    if let Some(definition) = definition {
        for plane in &definition[1..] {
            if !extra_planes.iter().any(|existing| existing == plane) {
                extra_planes.push(plane.clone());
            }
        }
    }
    probe_definitions_from_active_halfspaces(witness, halfspaces, active_planes, &extra_planes)
}

fn aabb_core_halfspaces(bounds: &Aabb) -> HypermeshResult<Vec<LimitPlane3>> {
    let mut halfspaces = Vec::with_capacity(6);
    for axis in 0..3 {
        let min = axis_ref(&bounds.min, axis);
        let max = axis_ref(&bounds.max, axis);
        halfspaces.push(axis_halfspace(axis, true, min.clone()));
        halfspaces.push(axis_halfspace(axis, false, max.clone()));
    }
    Ok(halfspaces)
}

fn axis_halfspace(axis: usize, lower_bound: bool, value: Real) -> LimitPlane3 {
    let zero = Real::zero();
    let one = Real::one();
    let minus_one = -Real::one();
    let normal = match (axis, lower_bound) {
        (0, true) => Point3::new(minus_one, zero.clone(), zero),
        (1, true) => Point3::new(zero.clone(), minus_one, zero),
        (2, true) => Point3::new(zero.clone(), zero, minus_one),
        (0, false) => Point3::new(one, zero.clone(), zero),
        (1, false) => Point3::new(zero.clone(), one, zero),
        (2, false) => Point3::new(zero.clone(), zero, one),
        _ => panic!("axis must be in 0..3"),
    };
    let offset = if lower_bound { value } else { -value };
    LimitPlane3::new(normal, offset)
}

fn plane_halfspace(plane: &Plane) -> LimitPlane3 {
    LimitPlane3::new(plane.normal.clone(), plane.offset.clone())
}

fn negated_halfspace(halfspace: &LimitPlane3) -> LimitPlane3 {
    LimitPlane3::new(
        Point3::new(
            -halfspace.normal.x.clone(),
            -halfspace.normal.y.clone(),
            -halfspace.normal.z.clone(),
        ),
        -halfspace.offset.clone(),
    )
}

fn push_plane_equality_halfspaces(halfspaces: &mut Vec<LimitPlane3>, plane: &Plane) {
    let halfspace = plane_halfspace(plane);
    halfspaces.push(halfspace.clone());
    halfspaces.push(negated_halfspace(&halfspace));
}

fn support_side_halfspace(plane: &Plane, positive: bool) -> LimitPlane3 {
    if positive {
        LimitPlane3::new(
            Point3::new(
                -plane.normal.x.clone(),
                -plane.normal.y.clone(),
                -plane.normal.z.clone(),
            ),
            -plane.offset.clone(),
        )
    } else {
        LimitPlane3::new(plane.normal.clone(), plane.offset.clone())
    }
}

fn normal_stop_halfspace(plane: &Plane, stop_point: &Point3, positive_side: bool) -> LimitPlane3 {
    let stop_plane = Plane::new(
        plane.normal.clone(),
        &plane.offset - &plane.expression_at_point(stop_point),
    );
    let halfspace = plane_halfspace(&stop_plane);
    if positive_side {
        halfspace
    } else {
        negated_halfspace(&halfspace)
    }
}

#[cfg(test)]
fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    remaining_detours: usize,
    trace_without_detours: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<bool>,
    detours_for: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    if trace_without_detours(start, end)? {
        return Ok(true);
    }

    if remaining_detours == 0 {
        return Ok(false);
    }

    for detour in detours_for(start, end)? {
        if detour.point == *start
            || detour.point == *end
            || point_lies_on_traced_surface(&detour.point, polygons)?
        {
            continue;
        }
        if !probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
            start,
            &detour.point,
            polygons,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        )? {
            continue;
        }
        if probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
            &detour.point,
            end,
            polygons,
            remaining_detours - 1,
            trace_without_detours,
            detours_for,
        )? {
            return Ok(true);
        }
    }

    Ok(false)
}

#[cfg(test)]
fn strict_halfspace_cell_seeds_from_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: &hyperlimit::HalfspaceFeasibilityReport,
) -> HypermeshResult<Vec<Point3>> {
    strict_halfspace_cell_seeds_from_optional_report(bounds, halfspaces, Some(report))
}

#[cfg(test)]
fn strict_halfspace_cell_seeds_from_optional_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> HypermeshResult<Vec<Point3>> {
    let mut saw_unknown = false;
    halfspace_cell_seed_families_from_optional_report(bounds, halfspaces, report, &mut saw_unknown)
        .map(|(strict_seeds, _shifted_vertices, _shifted_geometry_seeds)| strict_seeds)
}

fn push_unique_halfspace_seed(seeds: &mut Vec<Point3>, seed: Point3) {
    if !seeds.iter().any(|existing| existing == &seed) {
        seeds.push(seed);
    }
}

#[derive(Clone, Debug, PartialEq)]
struct HalfspaceSeedFamilyState {
    seeds: Vec<Point3>,
    saw_unknown: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct DefinitionFamilyState {
    definitions: Vec<[Plane; 3]>,
    saw_unknown: bool,
}

fn extend_strict_halfspace_seeds_backtracking_unknown(
    seeds: &mut Vec<Point3>,
    candidates: impl IntoIterator<Item = Point3>,
    mut is_strict_seed: impl FnMut(&Point3) -> HypermeshResult<bool>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for candidate in candidates {
        match is_strict_seed(&candidate) {
            Ok(true) => push_unique_halfspace_seed(seeds, candidate),
            Ok(false) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if seeds.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(())
    }
}

fn collect_strict_halfspace_seed_family(
    candidates: HypermeshResult<Vec<Point3>>,
    mut is_strict_seed: impl FnMut(&Point3) -> HypermeshResult<bool>,
) -> HypermeshResult<HalfspaceSeedFamilyState> {
    let mut seeds = Vec::new();
    let mut saw_unknown = false;
    for candidate in candidates? {
        match is_strict_seed(&candidate) {
            Ok(true) => push_unique_halfspace_seed(&mut seeds, candidate),
            Ok(false) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if seeds.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(HalfspaceSeedFamilyState { seeds, saw_unknown })
    }
}

fn extend_strict_halfspace_seed_families_backtracking_unknown(
    seeds: &mut Vec<Point3>,
    families: impl IntoIterator<Item = HypermeshResult<HalfspaceSeedFamilyState>>,
) -> HypermeshResult<()> {
    let saw_unknown = extend_strict_halfspace_seed_families_collect_unknown(seeds, families)?;
    if seeds.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(())
    }
}

fn extend_strict_halfspace_seed_families_collect_unknown(
    seeds: &mut Vec<Point3>,
    families: impl IntoIterator<Item = HypermeshResult<HalfspaceSeedFamilyState>>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for family in families {
        match family {
            Ok(found) => {
                saw_unknown |= found.saw_unknown;
                for seed in found.seeds {
                    push_unique_halfspace_seed(seeds, seed);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    Ok(saw_unknown)
}

#[derive(Clone, Debug, PartialEq)]
struct ShiftedHalfspaceWitnessFamily {
    halfspaces: Vec<LimitPlane3>,
    active_planes: [Option<usize>; 3],
}

#[derive(Clone, Debug, PartialEq)]
struct ShiftedHalfspaceWitness {
    point: Point3,
    families: Vec<ShiftedHalfspaceWitnessFamily>,
    uncertified_definition_fallback: bool,
}

impl ShiftedHalfspaceWitness {
    fn with_family(
        point: Point3,
        halfspaces: Vec<LimitPlane3>,
        active_planes: [Option<usize>; 3],
        uncertified_definition_fallback: bool,
    ) -> Self {
        Self {
            point,
            families: vec![ShiftedHalfspaceWitnessFamily {
                halfspaces,
                active_planes,
            }],
            uncertified_definition_fallback,
        }
    }
}

fn shifted_halfspace_cell_witnesses_from_seed(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>> {
    let shifted = shifted_halfspace_cell(bounds, halfspaces, seed)?;
    let (shifted_report, mut saw_unknown) = optional_halfspace_feasibility_report(&shifted)?;
    if shifted_report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }

    let mut witnesses = Vec::new();
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            bounds,
            &shifted,
            shifted_report.as_ref(),
            &mut saw_unknown,
        )?;
    let report_witness = shifted_report
        .as_ref()
        .and_then(|report| report.witness.as_ref());
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(
            report_witness,
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        strict_seeds,
        |witness| {
            Ok(vec![ShiftedHalfspaceWitness::with_family(
                witness.clone(),
                shifted.clone(),
                active_planes_from_optional_report(shifted_report.as_ref(), &witness),
                false,
            )])
        },
    )?;
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        shifted_vertices,
        |witness| {
            if !point_strictly_inside_halfspace_cell(&witness, bounds, halfspaces)? {
                return Ok(Vec::new());
            }
            Ok(vec![ShiftedHalfspaceWitness::with_family(
                witness.clone(),
                shifted.clone(),
                [None, None, None],
                false,
            )])
        },
    )?;
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        shifted_geometry_seeds,
        |witness| {
            if !point_strictly_inside_halfspace_cell(&witness, bounds, halfspaces)? {
                return Ok(Vec::new());
            }
            Ok(vec![ShiftedHalfspaceWitness::with_family(
                witness.clone(),
                shifted.clone(),
                [None, None, None],
                false,
            )])
        },
    )?;

    if witnesses.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_shifted_halfspace_witnesses_uncertified(&mut witnesses);
        }
        Ok(witnesses)
    }
}

fn halfspace_cell_seed_families_from_optional_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    saw_unknown: &mut bool,
) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)> {
    let shifted_vertex_family = feasible_halfspace_cell_vertex_family(halfspaces)?;
    *saw_unknown |= shifted_vertex_family.saw_unknown;
    let shifted_vertices = shifted_vertex_family.seeds;
    let shifted_geometry_seed_family =
        halfspace_centroid_subset_seed_family_from_vertices(&shifted_vertices)?;
    *saw_unknown |= shifted_geometry_seed_family.saw_unknown;
    let shifted_geometry_seeds = shifted_geometry_seed_family.seeds;
    let mut strict_seeds = Vec::new();

    *saw_unknown |= extend_strict_halfspace_seed_families_collect_unknown(
        &mut strict_seeds,
        [
            if report.is_some_and(|report| report.status == HalfspaceFeasibility::Feasible)
                && let Some(witness) = report.and_then(|report| report.witness.as_ref())
            {
                collect_strict_halfspace_seed_family(Ok(vec![witness.clone()]), |candidate| {
                    point_strictly_inside_halfspace_cell(candidate, bounds, halfspaces)
                })
            } else {
                Ok(HalfspaceSeedFamilyState {
                    seeds: Vec::new(),
                    saw_unknown: false,
                })
            },
            collect_strict_halfspace_seed_family(Ok(shifted_vertices.clone()), |candidate| {
                point_strictly_inside_halfspace_cell(candidate, bounds, halfspaces)
            }),
            collect_strict_halfspace_seed_family(Ok(shifted_geometry_seeds.clone()), |candidate| {
                point_strictly_inside_halfspace_cell(candidate, bounds, halfspaces)
            }),
        ],
    )?;

    if seed_family_search_failed_without_any_seed(
        &strict_seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        *saw_unknown,
    ) {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok((strict_seeds, shifted_vertices, shifted_geometry_seeds))
    }
}

fn seed_family_search_failed_without_any_seed(
    strict_seeds: &[Point3],
    shifted_vertices: &[Point3],
    shifted_geometry_seeds: &[Point3],
    saw_unknown: bool,
) -> bool {
    strict_seeds.is_empty()
        && shifted_vertices.is_empty()
        && shifted_geometry_seeds.is_empty()
        && saw_unknown
}

#[cfg(test)]
fn shifted_halfspace_cell_vertex_witnesses(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>> {
    let mut witnesses: Vec<ShiftedHalfspaceWitness> = Vec::new();
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        feasible_halfspace_cell_vertices(halfspaces)?,
        |seed| shifted_halfspace_cell_witnesses_from_seed(bounds, halfspaces, seed),
    )?;
    Ok(witnesses)
}

#[cfg(test)]
fn shifted_halfspace_cell_geometry_witnesses(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>> {
    let mut witnesses: Vec<ShiftedHalfspaceWitness> = Vec::new();
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        halfspace_cell_geometry_seed_candidates(halfspaces)?,
        |seed| shifted_halfspace_cell_witnesses_from_seed(bounds, halfspaces, seed),
    )?;
    Ok(witnesses)
}

fn push_unique_shifted_halfspace_witness(
    witnesses: &mut Vec<ShiftedHalfspaceWitness>,
    witness: ShiftedHalfspaceWitness,
) {
    if let Some(existing) = witnesses
        .iter_mut()
        .find(|existing| existing.point == witness.point)
    {
        existing.uncertified_definition_fallback |= witness.uncertified_definition_fallback;
        for family in witness.families {
            if !existing
                .families
                .iter()
                .any(|candidate| shifted_halfspace_witness_families_match(candidate, &family))
            {
                existing.families.push(family);
            }
        }
    } else {
        witnesses.push(witness);
    }
}

fn shifted_halfspace_witness_families_match(
    left: &ShiftedHalfspaceWitnessFamily,
    right: &ShiftedHalfspaceWitnessFamily,
) -> bool {
    limit_plane_families_match_as_sets(&left.halfspaces, &right.halfspaces)
        && active_halfspace_planes_match_as_sets(
            &left.halfspaces,
            left.active_planes,
            &right.halfspaces,
            right.active_planes,
        )
}

fn limit_plane_families_match_as_sets(left: &[LimitPlane3], right: &[LimitPlane3]) -> bool {
    if left.len() != right.len() {
        return false;
    }

    let mut matched = vec![false; right.len()];
    for left_halfspace in left {
        let Some((index, _)) = right.iter().enumerate().find(|(index, right_halfspace)| {
            !matched[*index] && *right_halfspace == left_halfspace
        }) else {
            return false;
        };
        matched[index] = true;
    }
    true
}

fn active_halfspace_planes_match_as_sets(
    left_halfspaces: &[LimitPlane3],
    left_active_planes: [Option<usize>; 3],
    right_halfspaces: &[LimitPlane3],
    right_active_planes: [Option<usize>; 3],
) -> bool {
    let left_planes = mapped_active_halfspace_planes(left_halfspaces, left_active_planes);
    let right_planes = mapped_active_halfspace_planes(right_halfspaces, right_active_planes);
    plane_families_match_as_sets(&left_planes, &right_planes)
}

fn mapped_active_halfspace_planes(
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
) -> Vec<Plane> {
    active_planes
        .into_iter()
        .flatten()
        .filter_map(|index| halfspaces.get(index))
        .map(|halfspace| Plane::new(halfspace.normal.clone(), halfspace.offset.clone()))
        .collect()
}

fn plane_families_match_as_sets(left: &[Plane], right: &[Plane]) -> bool {
    if left.len() != right.len() {
        return false;
    }

    let mut matched = vec![false; right.len()];
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

fn take_new_halfspace_seed_family(points: Vec<Point3>, seen: &mut Vec<Point3>) -> Vec<Point3> {
    let mut fresh = Vec::new();
    for point in points {
        if seen.iter().any(|existing| existing == &point) {
            continue;
        }
        seen.push(point.clone());
        fresh.push(point);
    }
    fresh
}

fn dedupe_shifted_halfspace_seed_families(
    report_witness: Option<&Point3>,
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
) -> (Vec<Point3>, Vec<Point3>, Vec<Point3>) {
    let mut seen = Vec::new();
    let strict_seeds = take_new_halfspace_seed_family(strict_seeds, &mut seen);
    if let Some(report_witness) = report_witness
        && !seen.iter().any(|existing| existing == report_witness)
    {
        seen.push(report_witness.clone());
    }
    let shifted_vertices = take_new_halfspace_seed_family(shifted_vertices, &mut seen);
    let shifted_geometry_seeds = take_new_halfspace_seed_family(shifted_geometry_seeds, &mut seen);
    (strict_seeds, shifted_vertices, shifted_geometry_seeds)
}

fn extend_shifted_halfspace_seed_families_backtracking_unknown(
    witnesses: &mut Vec<ShiftedHalfspaceWitness>,
    families: impl IntoIterator<Item = Vec<Point3>>,
    mut build: impl FnMut(&Point3) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    let mut seen = Vec::new();
    for family in families {
        let fresh = take_new_halfspace_seed_family(family, &mut seen);
        let mut local = Vec::new();
        match extend_shifted_halfspace_witnesses_backtracking_unknown(&mut local, fresh, |seed| {
            build(seed)
        }) {
            Ok(()) => {
                saw_unknown |= local
                    .iter()
                    .any(|witness| witness.uncertified_definition_fallback);
                for witness in local {
                    push_unique_shifted_halfspace_witness(witnesses, witness);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if witnesses.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_shifted_halfspace_witnesses_uncertified(witnesses);
        }
        Ok(())
    }
}

fn extend_shifted_halfspace_witnesses_backtracking_unknown(
    witnesses: &mut Vec<ShiftedHalfspaceWitness>,
    seeds: impl IntoIterator<Item = Point3>,
    mut build: impl FnMut(&Point3) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for seed in seeds {
        match build(&seed) {
            Ok(found) => {
                saw_unknown |= found
                    .iter()
                    .any(|witness| witness.uncertified_definition_fallback);
                for witness in found {
                    push_unique_shifted_halfspace_witness(witnesses, witness);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if witnesses.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_shifted_halfspace_witnesses_uncertified(witnesses);
        }
        Ok(())
    }
}

fn shifted_halfspace_witness_family_or_empty(
    result: HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
    saw_unknown: &mut bool,
) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>> {
    match result {
        Ok(witnesses) => Ok(witnesses),
        Err(HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            Ok(Vec::new())
        }
        Err(err) => Err(err),
    }
}

#[cfg(test)]
fn halfspace_feasibility_report(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<hyperlimit::HalfspaceFeasibilityReport> {
    match classify_halfspace_feasibility3(halfspaces) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { .. } => Err(HypermeshError::UnknownClassification),
    }
}

fn optional_halfspace_feasibility_report(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<(Option<hyperlimit::HalfspaceFeasibilityReport>, bool)> {
    match classify_halfspace_feasibility3(halfspaces) {
        PredicateOutcome::Decided { value, .. } => Ok((Some(value), false)),
        PredicateOutcome::Unknown { .. } => Ok((None, true)),
    }
}

fn active_planes_from_optional_report(
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    witness: &Point3,
) -> [Option<usize>; 3] {
    report.map_or([None, None, None], |report| {
        witness_active_planes(report.witness.as_ref(), report.active_planes, witness)
    })
}

fn feasible_halfspace_cell_vertices(halfspaces: &[LimitPlane3]) -> HypermeshResult<Vec<Point3>> {
    Ok(feasible_halfspace_cell_vertex_family(halfspaces)?.seeds)
}

fn feasible_halfspace_cell_vertex_family(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<HalfspaceSeedFamilyState> {
    feasible_halfspace_cell_vertex_family_with_contains(halfspaces, |point, halfspaces| {
        point_satisfies_halfspaces(point, halfspaces)
    })
}

fn feasible_halfspace_cell_vertex_family_with_contains(
    halfspaces: &[LimitPlane3],
    mut contains: impl FnMut(&Point3, &[LimitPlane3]) -> HypermeshResult<bool>,
) -> HypermeshResult<HalfspaceSeedFamilyState> {
    let mut vertices = Vec::new();
    let mut saw_unknown = false;
    for first in 0..halfspaces.len() {
        for second in (first + 1)..halfspaces.len() {
            for third in (second + 1)..halfspaces.len() {
                let candidate = intersect_three_planes(
                    &halfspaces[first],
                    &halfspaces[second],
                    &halfspaces[third],
                );
                let Ok(point) = candidate.to_affine_point() else {
                    continue;
                };
                match contains(&point, halfspaces) {
                    Ok(true) => {
                        if !vertices.iter().any(|existing| existing == &point) {
                            vertices.push(point);
                        }
                    }
                    Ok(false) => {}
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                    }
                    Err(err) => return Err(err),
                }
            }
        }
    }
    if vertices.is_empty() && saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(HalfspaceSeedFamilyState {
            seeds: vertices,
            saw_unknown,
        })
    }
}

fn feasible_halfspace_cell_vertices_with_contains(
    halfspaces: &[LimitPlane3],
    contains: impl FnMut(&Point3, &[LimitPlane3]) -> HypermeshResult<bool>,
) -> HypermeshResult<Vec<Point3>> {
    Ok(feasible_halfspace_cell_vertex_family_with_contains(halfspaces, contains)?.seeds)
}

fn point_satisfies_halfspaces(point: &Point3, halfspaces: &[LimitPlane3]) -> HypermeshResult<bool> {
    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if classify_point(point, &plane)? == Classification::Positive {
            return Ok(false);
        }
    }
    Ok(true)
}

fn point_strictly_inside_halfspace_cell(
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    if !point_strictly_inside_probe_bounds(point, bounds)? {
        return Ok(false);
    }
    for halfspace in halfspaces {
        if halfspace_is_degenerate_bound(halfspace, bounds)?
            || halfspace_has_opposite_pair(halfspace, halfspaces)
        {
            continue;
        }
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if compare_real(&plane.expression_at_point(point), &Real::zero())?.is_eq() {
            return Ok(false);
        }
    }
    Ok(true)
}

fn point_strictly_inside_probe_bounds(point: &Point3, bounds: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        let min = axis_ref(&bounds.min, axis);
        let max = axis_ref(&bounds.max, axis);
        if compare_real(min, max)?.is_eq() {
            if compare_real(axis_ref(point, axis), min)?.is_ne() {
                return Ok(false);
            }
            continue;
        }
        if !compare_real(axis_ref(point, axis), min)?.is_gt()
            || !compare_real(axis_ref(point, axis), max)?.is_lt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn shifted_halfspace_cell(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    strict_interior: &Point3,
) -> HypermeshResult<Vec<LimitPlane3>> {
    let half = (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;
    let mut shifted = Vec::with_capacity(halfspaces.len());
    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        let value = plane.expression_at_point(strict_interior);
        let keep_closed = compare_real(&value, &Real::zero())?.is_eq()
            || halfspace_is_degenerate_bound(halfspace, bounds)?;
        let offset = if keep_closed {
            halfspace.offset.clone()
        } else {
            &halfspace.offset - &(value * &half)
        };
        shifted.push(LimitPlane3::new(halfspace.normal.clone(), offset));
    }
    Ok(shifted)
}

fn halfspace_is_degenerate_bound(halfspace: &LimitPlane3, bounds: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        let min = axis_ref(&bounds.min, axis);
        let max = axis_ref(&bounds.max, axis);
        if compare_real(min, max)?.is_ne() {
            continue;
        }
        if *halfspace == axis_halfspace(axis, true, min.clone())
            || *halfspace == axis_halfspace(axis, false, min.clone())
        {
            return Ok(true);
        }
    }
    Ok(false)
}

fn halfspace_has_opposite_pair(target: &LimitPlane3, halfspaces: &[LimitPlane3]) -> bool {
    let opposite = negated_halfspace(target);
    halfspaces.iter().any(|halfspace| halfspace == &opposite)
}

fn axis_value_after_start(
    start: &Real,
    value: &Real,
    direction_positive: bool,
) -> HypermeshResult<bool> {
    let order = compare_real(value, start)?;
    Ok((direction_positive && order.is_gt()) || (!direction_positive && order.is_lt()))
}

fn axis_value_before_stop(
    value: &Real,
    stop: &Real,
    direction_positive: bool,
) -> HypermeshResult<bool> {
    let order = compare_real(value, stop)?;
    Ok((direction_positive && order.is_lt()) || (!direction_positive && order.is_gt()))
}

fn probe_axes(support: &Plane) -> HypermeshResult<Vec<usize>> {
    let dominant = dominant_normal_axis(support)?;
    let mut axes = vec![dominant];
    for axis in 0..3 {
        if axis != dominant && !axis_ref(&support.normal, axis).definitely_zero() {
            axes.push(axis);
        }
    }
    Ok(axes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::polygon::{make_quad, make_triangle};

    fn r(value: i32) -> Real {
        value.into()
    }

    fn q(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    fn quadrilateral_halfspace_cell_fixture() -> (Aabb, Vec<LimitPlane3>, Point3) {
        let bounds = Aabb::new(p(0, 0, 0), p(5, 4, 0));
        let support = Plane::axis_aligned(2, r(0));
        let interior = Point3::new(q(9, 4), r(2), r(0));
        let vertices = [p(0, 0, 0), p(4, 0, 0), p(5, 4, 0), p(0, 4, 0)];
        let mut halfspaces = vec![
            limit_plane_from_plane(&support),
            limit_plane_from_plane(&support.inverted()),
        ];

        for index in 0..vertices.len() {
            let next = (index + 1) % vertices.len();
            let mut edge_plane = Plane::from_points(
                &vertices[index],
                &vertices[next],
                &Point3::new(
                    axis_ref(&vertices[index], 0).clone(),
                    axis_ref(&vertices[index], 1).clone(),
                    r(1),
                ),
            );
            if classify_real(&edge_plane.expression_at_point(&interior)).unwrap()
                == Classification::Positive
            {
                edge_plane = edge_plane.inverted();
            }
            halfspaces.push(limit_plane_from_plane(&edge_plane));
        }

        (bounds, halfspaces, Point3::new(q(5, 2), r(2), r(0)))
    }

    fn px(x: Real, y: i32, z: i32) -> Point3 {
        Point3::new(x, r(y), r(z))
    }

    #[test]
    fn trace_retry_only_suppresses_unknown_classification() {
        assert_eq!(
            retryable_trace::<Vec<i32>>(Err(HypermeshError::UnknownClassification)).unwrap(),
            None
        );
        assert_eq!(
            retryable_trace::<Vec<i32>>(Err(HypermeshError::PointAtInfinity)),
            Err(HypermeshError::PointAtInfinity)
        );
    }

    #[test]
    fn trace_axis_segment_rejects_transition_dimension_mismatch() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];

        assert_eq!(
            trace_axis_segment(&p(0, 0, 0), &p(2, 0, 0), 0, &[0, 0], &[wall]),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn trace_axis_segment_reports_unknown_for_unmatched_edge_crossing() {
        let wall = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);

        assert_eq!(
            trace_axis_segment(&p(0, 0, 0), &p(2, 0, 0), 0, &[0], &[wall]),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn trace_direct_segment_reports_unknown_for_unmatched_edge_crossing() {
        let wall = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);

        assert_eq!(
            trace_direct_segment(&p(0, 0, 0), &p(2, 0, 0), &[0], &[wall]),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn centroid_is_fallible_and_reports_empty_input() {
        assert_eq!(centroid(&[]).unwrap(), None);
        assert_eq!(
            centroid(&[p(0, 0, 0), p(2, 2, 2)]).unwrap(),
            Some(p(1, 1, 1))
        );
    }

    fn axis_values(points: &[Point3], axis: usize) -> Vec<Real> {
        let mut values = Vec::new();
        for point in points {
            let value = axis_ref(point, axis).clone();
            if !values.iter().any(|existing| existing == &value) {
                values.push(value);
            }
        }
        values
    }

    #[test]
    fn endpoint_box_detours_are_cut_by_surface_crossings() {
        let slanted = make_triangle(&p(0, 2, -2), &p(0, 2, 2), &p(4, -2, 0), 0, 0);

        let detours = interior_box_detour_targets(&p(0, 0, 0), &p(4, 4, 4), &[slanted]).unwrap();
        let x_values = axis_values(
            &detours
                .iter()
                .map(|target| target.point.clone())
                .collect::<Vec<_>>(),
            0,
        );

        assert!(
            x_values
                .iter()
                .any(|value| compare_real(value, &r(0)).unwrap().is_gt()
                    && compare_real(value, &r(2)).unwrap().is_lt())
        );
        assert!(
            x_values
                .iter()
                .any(|value| compare_real(value, &r(2)).unwrap().is_gt()
                    && compare_real(value, &r(4)).unwrap().is_lt())
        );
    }

    #[test]
    fn strict_aabb_targets_handle_degenerate_axis_boxes() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 6, 0));
        let targets = strict_aabb_targets(&bounds).unwrap();

        assert!(!targets.is_empty());
        for target in targets {
            assert_eq!(target.point.z, r(0));
            assert!(compare_real(&target.point.x, &r(0)).unwrap().is_gt());
            assert!(compare_real(&target.point.x, &r(4)).unwrap().is_lt());
            assert!(compare_real(&target.point.y, &r(0)).unwrap().is_gt());
            assert!(compare_real(&target.point.y, &r(6)).unwrap().is_lt());
            assert!(!target.definitions.is_empty());
        }
    }

    #[test]
    fn detour_target_marking_marks_existing_targets_uncertain() {
        let mut targets = vec![DetourTarget {
            point: p(1, 2, 3),
            definitions: vec![axis_plane_definition(&p(1, 2, 3))],
            uncertified_definition_fallback: false,
        }];

        mark_all_detour_targets_uncertified(&mut targets);

        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn detour_target_build_collection_backtracks_after_uncertified_candidate() {
        let first = p(1, 2, 3);
        let second = p(1, 2, 4);
        let mut targets = Vec::new();

        extend_detour_target_builds_backtracking_unknown(
            &mut targets,
            [&first, &second],
            |point| {
                if *point == first {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(DetourTarget {
                        point: point.clone(),
                        definitions: vec![axis_plane_definition(point)],
                        uncertified_definition_fallback: false,
                    })
                }
            },
        )
        .unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, second);
        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn detour_target_build_collection_reports_unknown_if_all_candidates_are_uncertified() {
        let first = p(1, 2, 3);
        let second = p(1, 2, 4);
        let mut targets = Vec::new();

        let err = extend_detour_target_builds_backtracking_unknown(
            &mut targets,
            [&first, &second],
            |_point| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
        assert!(targets.is_empty());
    }

    #[test]
    fn detour_target_build_collection_marks_later_targets_uncertain_after_uncertain_candidate_result()
     {
        let first = p(1, 2, 3);
        let second = p(1, 2, 4);
        let mut targets = Vec::new();

        extend_detour_target_builds_backtracking_unknown(
            &mut targets,
            [&first, &second],
            |point| {
                Ok(DetourTarget {
                    point: point.clone(),
                    definitions: vec![axis_plane_definition(point)],
                    uncertified_definition_fallback: *point == first,
                })
            },
        )
        .unwrap();

        assert_eq!(targets.len(), 2);
        assert!(
            targets
                .iter()
                .all(|target| target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn detour_target_family_collection_backtracks_after_uncertified_family() {
        let mut targets = Vec::new();

        extend_detour_target_families_backtracking_unknown(
            &mut targets,
            [
                Err(HypermeshError::UnknownClassification),
                Ok(vec![DetourTarget {
                    point: p(1, 2, 4),
                    definitions: vec![axis_plane_definition(&p(1, 2, 4))],
                    uncertified_definition_fallback: false,
                }]),
            ],
        )
        .unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, p(1, 2, 4));
        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn detour_target_family_collection_tracks_unknown_after_uncertain_family_result() {
        let mut targets = Vec::new();

        extend_detour_target_families_backtracking_unknown(
            &mut targets,
            [
                Ok(vec![DetourTarget {
                    point: p(1, 2, 3),
                    definitions: vec![axis_plane_definition(&p(1, 2, 3))],
                    uncertified_definition_fallback: true,
                }]),
                Ok(vec![DetourTarget {
                    point: p(1, 2, 4),
                    definitions: vec![axis_plane_definition(&p(1, 2, 4))],
                    uncertified_definition_fallback: false,
                }]),
            ],
        )
        .unwrap();

        assert_eq!(targets.len(), 2);
        assert!(
            targets
                .iter()
                .all(|target| target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn detour_target_family_collection_reports_unknown_if_all_families_are_uncertified() {
        let mut targets = Vec::new();

        let err = extend_detour_target_families_backtracking_unknown(
            &mut targets,
            [
                Err(HypermeshError::UnknownClassification),
                Err(HypermeshError::UnknownClassification),
            ],
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
        assert!(targets.is_empty());
    }

    #[test]
    fn interior_box_detour_target_collection_backtracks_after_uncertified_box_family() {
        let intervals = vec![
            vec![(r(0), r(1)), (r(1), r(2))],
            vec![(r(0), r(1))],
            vec![(r(0), r(1))],
        ];

        let targets = collect_detour_targets_from_axis_intervals(&intervals, |bounds| {
            if bounds.min == p(0, 0, 0) && bounds.max == p(1, 1, 1) {
                Err(HypermeshError::UnknownClassification)
            } else {
                let point = Point3::new(r(1), q(1, 2), q(1, 2));
                Ok(vec![DetourTarget {
                    point: point.clone(),
                    definitions: vec![axis_plane_definition(&point)],
                    uncertified_definition_fallback: false,
                }])
            }
        })
        .unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, Point3::new(r(1), q(1, 2), q(1, 2)));
        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn axis_box_surface_cut_collection_backtracks_after_uncertified_crossing() {
        let start = p(0, 0, 0);
        let end = p(2, 0, 0);
        let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let (intervals, saw_unknown) = interior_box_axis_intervals_with_surface_queries(
            &start,
            &end,
            &[first, second],
            &mut |_edge_start, _edge_end, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(1) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(p(1, 0, 0)))
                }
            },
            &mut |_crossing, _polygon| Ok(PolygonPointLocation::Interior),
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(intervals[0], vec![(r(0), r(1)), (r(1), r(2))]);
    }

    #[test]
    fn interior_box_detour_target_collection_marks_surviving_targets_uncertain_after_uncertified_surface_cut()
     {
        let start = p(0, 0, 0);
        let end = p(2, 0, 0);
        let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let targets = interior_box_detour_targets_with_queries(
            &start,
            &end,
            &[first, second],
            |_edge_start, _edge_end, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(1) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(p(1, 0, 0)))
                }
            },
            |_crossing, _polygon| Ok(PolygonPointLocation::Interior),
            |bounds| {
                if *bounds == Aabb::new(p(1, 0, 0), p(2, 0, 0)) {
                    let point = p(1, 0, 0);
                    Ok(vec![DetourTarget {
                        point: point.clone(),
                        definitions: vec![axis_plane_definition(&point)],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, p(1, 0, 0));
        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn interior_box_detour_target_collection_reports_unknown_when_surface_cut_family_is_partially_uncertified_and_boxes_fail()
     {
        let start = p(0, 0, 0);
        let end = p(2, 0, 0);
        let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let err = interior_box_detour_targets_with_queries(
            &start,
            &end,
            &[first, second],
            |_edge_start, _edge_end, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(1) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(p(1, 0, 0)))
                }
            },
            |_crossing, _polygon| Ok(PolygonPointLocation::Interior),
            |_bounds| Ok(Vec::new()),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn axis_box_surface_cut_collection_treats_boundary_crossing_as_unknown_and_keeps_later_cut() {
        let start = p(0, 0, 0);
        let end = p(3, 0, 0);
        let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let (intervals, saw_unknown) = interior_box_axis_intervals_with_surface_queries(
            &start,
            &end,
            &[first, second],
            &mut |_edge_start, _edge_end, polygon, _axis| {
                let x = polygon.vertices().unwrap()[0].x.clone();
                Ok(Some(Point3::new(x, r(0), r(0))))
            },
            &mut |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(1) {
                    Ok(PolygonPointLocation::Boundary)
                } else {
                    Ok(PolygonPointLocation::Interior)
                }
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(intervals[0], vec![(r(0), r(1)), (r(1), r(2)), (r(2), r(3))]);
    }

    #[test]
    fn interior_box_detour_target_collection_marks_surviving_targets_uncertain_after_boundary_surface_cut()
     {
        let start = p(0, 0, 0);
        let end = p(3, 0, 0);
        let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let targets = interior_box_detour_targets_with_queries(
            &start,
            &end,
            &[first, second],
            |_edge_start, _edge_end, polygon, _axis| {
                let x = polygon.vertices().unwrap()[0].x.clone();
                Ok(Some(Point3::new(x, r(0), r(0))))
            },
            |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(1) {
                    Ok(PolygonPointLocation::Boundary)
                } else {
                    Ok(PolygonPointLocation::Interior)
                }
            },
            |bounds| {
                if *bounds == Aabb::new(p(2, 0, 0), p(3, 0, 0)) {
                    let point = p(2, 0, 0);
                    Ok(vec![DetourTarget {
                        point: point.clone(),
                        definitions: vec![axis_plane_definition(&point)],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, p(2, 0, 0));
        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn detour_target_build_preserves_inherited_uncertified_definition_fallback() {
        let point = p(1, 1, 1);
        let halfspaces = vec![axis_halfspace(2, false, r(1))];

        let target = build_detour_target(&point, &halfspaces, [None, None, None], true).unwrap();

        assert!(target.uncertified_definition_fallback);
    }

    #[test]
    fn duplicate_detour_targets_merge_permuted_plane_definitions() {
        let point = p(1, 1, 1);
        let definition = axis_plane_definition(&point);
        let permuted = [
            definition[1].clone(),
            definition[2].clone(),
            definition[0].clone(),
        ];
        let mut targets = vec![DetourTarget {
            point: point.clone(),
            definitions: vec![definition],
            uncertified_definition_fallback: false,
        }];

        push_unique_detour_target(
            &mut targets,
            DetourTarget {
                point,
                definitions: vec![permuted.clone()],
                uncertified_definition_fallback: false,
            },
        );

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].definitions.len(), 1);
        assert!(definition_planes_match_as_sets(
            &targets[0].definitions[0],
            &permuted
        ));
    }

    #[test]
    fn shifted_halfspace_cell_vertex_witnesses_return_strict_points() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let witnesses = shifted_halfspace_cell_vertex_witnesses(&bounds, &halfspaces).unwrap();

        assert!(!witnesses.is_empty());
        for witness in &witnesses {
            assert!(
                point_strictly_inside_halfspace_cell(&witness.point, &bounds, &halfspaces).unwrap()
            );
        }
    }

    #[test]
    fn shifted_halfspace_cell_geometry_witnesses_return_strict_points() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let witnesses = shifted_halfspace_cell_geometry_witnesses(&bounds, &halfspaces).unwrap();

        assert!(!witnesses.is_empty());
        for witness in &witnesses {
            assert!(
                point_strictly_inside_halfspace_cell(&witness.point, &bounds, &halfspaces).unwrap()
            );
        }
    }

    #[test]
    fn shifted_halfspace_cell_witnesses_from_seed_returns_only_strict_points() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let witnesses =
            shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, &p(2, 1, 3)).unwrap();

        assert!(!witnesses.is_empty());
        for witness in &witnesses {
            assert!(
                point_strictly_inside_halfspace_cell(&witness.point, &bounds, &halfspaces).unwrap()
            );
        }
    }

    #[test]
    fn feasible_halfspace_cell_vertices_backtrack_after_uncertified_candidate() {
        let halfspaces = vec![
            axis_halfspace(0, true, r(0)),
            axis_halfspace(0, false, r(0)),
            axis_halfspace(1, true, r(0)),
            axis_halfspace(1, false, r(0)),
            axis_halfspace(2, true, r(0)),
            axis_halfspace(2, false, r(1)),
        ];
        let first = p(0, 0, 0);
        let second = p(0, 0, 1);

        let vertices = feasible_halfspace_cell_vertices_with_contains(&halfspaces, |point, _| {
            if point == &first {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(point == &second)
            }
        })
        .unwrap();

        assert_eq!(vertices, vec![second]);
    }

    #[test]
    fn feasible_halfspace_cell_vertex_family_tracks_unknown_after_later_vertex() {
        let halfspaces = vec![
            axis_halfspace(0, true, r(0)),
            axis_halfspace(0, false, r(0)),
            axis_halfspace(1, true, r(0)),
            axis_halfspace(1, false, r(0)),
            axis_halfspace(2, true, r(0)),
            axis_halfspace(2, false, r(1)),
        ];
        let first = p(0, 0, 0);
        let second = p(0, 0, 1);

        let family =
            feasible_halfspace_cell_vertex_family_with_contains(&halfspaces, |point, _| {
                if point == &first {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(point == &second)
                }
            })
            .unwrap();

        assert_eq!(family.seeds, vec![second]);
        assert!(family.saw_unknown);
    }

    #[test]
    fn feasible_halfspace_cell_vertices_report_unknown_if_all_candidates_are_uncertified() {
        let halfspaces = vec![
            axis_halfspace(0, true, r(0)),
            axis_halfspace(0, false, r(0)),
            axis_halfspace(1, true, r(0)),
            axis_halfspace(1, false, r(0)),
            axis_halfspace(2, true, r(0)),
            axis_halfspace(2, false, r(1)),
        ];

        let err = feasible_halfspace_cell_vertices_with_contains(&halfspaces, |_point, _| {
            Err(HypermeshError::UnknownClassification)
        })
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn halfspace_cell_geometry_seed_candidates_from_vertices_matches_direct_query() {
        let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
        let vertices = feasible_halfspace_cell_vertices(&halfspaces).unwrap();

        let from_vertices =
            halfspace_cell_geometry_seed_candidates_from_vertices(&vertices).unwrap();
        let from_query = halfspace_cell_geometry_seed_candidates(&halfspaces).unwrap();

        assert_eq!(from_vertices, from_query);
    }

    #[test]
    fn halfspace_centroid_subset_seed_family_tracks_unknown_after_later_centroid() {
        let vertices = vec![p(0, 0, 0), p(2, 0, 0), p(4, 0, 0)];
        let blocked_subset = vec![vertices[0].clone(), vertices[1].clone()];

        let family =
            halfspace_centroid_subset_seed_family_from_vertices_with(&vertices, |subset| {
                if subset == blocked_subset.as_slice() {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    centroid(subset)
                }
            })
            .unwrap();

        assert!(family.saw_unknown);
        assert!(!family.seeds.is_empty());
    }

    #[test]
    fn shifted_halfspace_cell_witnesses_from_seed_include_shifted_vertex_targets() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let witnesses =
            shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, &p(2, 1, 3)).unwrap();

        assert!(
            witnesses
                .iter()
                .any(|witness| witness.point == Point3::new(r(3), q(5, 2), q(7, 2)))
        );
        assert!(
            witnesses
                .iter()
                .find(|witness| witness.point == Point3::new(r(3), q(5, 2), q(7, 2)))
                .is_some_and(|witness| witness
                    .families
                    .iter()
                    .any(|family| family.active_planes == [None, None, None]))
        );
    }

    #[test]
    fn shifted_halfspace_cell_witnesses_from_seed_include_shifted_geometry_targets() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let witnesses =
            shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, &p(2, 1, 3)).unwrap();

        assert!(
            witnesses
                .iter()
                .any(|witness| witness.point == Point3::new(r(1), q(7, 6), q(13, 6)))
        );
        assert!(
            witnesses
                .iter()
                .find(|witness| witness.point == Point3::new(r(1), q(7, 6), q(13, 6)))
                .is_some_and(|witness| witness
                    .families
                    .iter()
                    .any(|family| family.active_planes == [None, None, None]))
        );
    }

    #[test]
    fn shifted_halfspace_witness_collection_backtracks_after_uncertified_candidate() {
        let mut witnesses = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_shifted_halfspace_witnesses_backtracking_unknown(
            &mut witnesses,
            [first.clone(), second.clone()],
            |candidate| {
                if *candidate == first {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(vec![ShiftedHalfspaceWitness::with_family(
                        candidate.clone(),
                        Vec::new(),
                        [None, None, None],
                        false,
                    )])
                }
            },
        )
        .unwrap();

        assert_eq!(witnesses.len(), 1);
        assert_eq!(witnesses[0].point, second);
        assert!(witnesses[0].uncertified_definition_fallback);
    }

    #[test]
    fn shifted_halfspace_witness_collection_reports_unknown_if_all_candidates_are_uncertified() {
        let mut witnesses = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        let err = extend_shifted_halfspace_witnesses_backtracking_unknown(
            &mut witnesses,
            [first, second],
            |_candidate| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn shifted_halfspace_witness_collection_backtracks_after_uncertified_seed() {
        let first_seed = p(1, 1, 1);
        let second_seed = p(2, 2, 2);
        let kept =
            ShiftedHalfspaceWitness::with_family(p(3, 3, 3), Vec::new(), [None, None, None], false);
        let mut witnesses = Vec::new();

        extend_shifted_halfspace_witnesses_backtracking_unknown(
            &mut witnesses,
            vec![first_seed.clone(), second_seed.clone()],
            |seed| {
                if seed == &first_seed {
                    Err(HypermeshError::UnknownClassification)
                } else if seed == &second_seed {
                    Ok(vec![kept.clone()])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert_eq!(witnesses.len(), 1);
        assert_eq!(witnesses[0].point, kept.point);
        assert!(witnesses[0].uncertified_definition_fallback);
    }

    #[test]
    fn shifted_halfspace_witness_collection_marks_later_witnesses_uncertain_after_uncertain_candidate_result()
     {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut witnesses = Vec::new();

        extend_shifted_halfspace_witnesses_backtracking_unknown(
            &mut witnesses,
            [first.clone(), second.clone()],
            |seed| {
                if *seed == first {
                    Ok(vec![ShiftedHalfspaceWitness::with_family(
                        seed.clone(),
                        Vec::new(),
                        [None, None, None],
                        true,
                    )])
                } else {
                    Ok(vec![ShiftedHalfspaceWitness::with_family(
                        seed.clone(),
                        Vec::new(),
                        [None, None, None],
                        false,
                    )])
                }
            },
        )
        .unwrap();

        assert_eq!(witnesses.len(), 2);
        assert!(
            witnesses
                .iter()
                .all(|witness| witness.uncertified_definition_fallback)
        );
    }

    #[test]
    fn duplicate_shifted_halfspace_witnesses_merge_distinct_active_plane_families() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let point = p(1, 1, 1);
        let mut witnesses = vec![ShiftedHalfspaceWitness::with_family(
            point.clone(),
            vec![halfspaces[0].clone()],
            [None, None, None],
            true,
        )];

        push_unique_shifted_halfspace_witness(
            &mut witnesses,
            ShiftedHalfspaceWitness::with_family(
                point,
                halfspaces.clone(),
                [Some(0), Some(1), None],
                false,
            ),
        );

        assert_eq!(witnesses.len(), 1);
        assert_eq!(witnesses[0].families.len(), 2);
        assert!(
            witnesses[0]
                .families
                .iter()
                .any(|family| family.active_planes == [None, None, None])
        );
        assert!(witnesses[0].families.iter().any(|family| {
            family.active_planes == [Some(0), Some(1), None] && family.halfspaces == halfspaces
        }));
        assert!(witnesses[0].uncertified_definition_fallback);
    }

    #[test]
    fn duplicate_shifted_halfspace_witnesses_merge_permuted_halfspace_families() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut permuted_halfspaces = halfspaces.clone();
        permuted_halfspaces.swap(0, 1);
        let point = p(1, 1, 1);
        let mut witnesses = vec![ShiftedHalfspaceWitness::with_family(
            point.clone(),
            halfspaces,
            [Some(0), Some(1), None],
            false,
        )];

        push_unique_shifted_halfspace_witness(
            &mut witnesses,
            ShiftedHalfspaceWitness::with_family(
                point,
                permuted_halfspaces,
                [Some(0), Some(1), None],
                false,
            ),
        );

        assert_eq!(witnesses.len(), 1);
        assert_eq!(witnesses[0].families.len(), 1);
    }

    #[test]
    fn shifted_halfspace_witness_collection_reports_unknown_if_all_seeds_are_uncertified() {
        let first_seed = p(1, 1, 1);
        let second_seed = p(2, 2, 2);
        let mut witnesses = Vec::new();

        let err = extend_shifted_halfspace_witnesses_backtracking_unknown(
            &mut witnesses,
            vec![first_seed, second_seed],
            |_seed| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn strict_halfspace_cell_seeds_include_direct_strict_feasibility_witness() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let direct = p(2, 1, 3);
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(direct.clone(), [None, None, None]);

        let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

        assert!(seeds.iter().any(|seed| seed == &direct));
    }

    #[test]
    fn strict_halfspace_cell_seeds_include_strict_feasible_vertices() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = vec![
            axis_halfspace(0, true, r(1)),
            axis_halfspace(0, false, r(1)),
            axis_halfspace(1, true, r(2)),
            axis_halfspace(1, false, r(2)),
            axis_halfspace(2, true, r(3)),
            axis_halfspace(2, false, r(3)),
        ];
        let report = hyperlimit::HalfspaceFeasibilityReport::feasible(
            Point3::new(r(1), r(2), r(0)),
            [None, None, None],
        );

        let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

        assert_eq!(seeds, vec![Point3::new(r(1), r(2), r(3))]);
    }

    #[test]
    fn strict_halfspace_cell_seeds_include_strict_geometry_seeds() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let report = hyperlimit::HalfspaceFeasibilityReport::feasible(
            Point3::new(r(0), r(0), r(0)),
            [None, None, None],
        );
        let tetra_center = p(1, 1, 1);

        let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

        assert!(seeds.iter().any(|seed| seed == &p(2, 2, 2)));
        assert!(seeds.iter().any(|seed| seed == &tetra_center));
    }

    #[test]
    fn strict_halfspace_cell_seed_collection_backtracks_after_uncertified_candidate() {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut seeds = Vec::new();

        extend_strict_halfspace_seeds_backtracking_unknown(
            &mut seeds,
            vec![first.clone(), second.clone()],
            |candidate| {
                if candidate == &first {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(candidate == &second)
                }
            },
        )
        .unwrap();

        assert_eq!(seeds, vec![second]);
    }

    #[test]
    fn strict_halfspace_cell_seed_collection_reports_unknown_if_all_candidates_are_uncertified() {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut seeds = Vec::new();

        let err = extend_strict_halfspace_seeds_backtracking_unknown(
            &mut seeds,
            vec![first, second],
            |_candidate| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn strict_halfspace_cell_seed_family_search_backtracks_after_uncertified_earlier_family() {
        let mut seeds = Vec::new();

        extend_strict_halfspace_seed_families_backtracking_unknown(
            &mut seeds,
            [
                Err(HypermeshError::UnknownClassification),
                Ok(HalfspaceSeedFamilyState {
                    seeds: vec![p(2, 2, 2)],
                    saw_unknown: false,
                }),
            ],
        )
        .unwrap();

        assert_eq!(seeds, vec![p(2, 2, 2)]);
    }

    #[test]
    fn strict_halfspace_cell_seed_family_search_tracks_unknown_after_uncertain_family_result() {
        let mut seeds = Vec::new();

        let saw_unknown = extend_strict_halfspace_seed_families_collect_unknown(
            &mut seeds,
            [
                Ok(HalfspaceSeedFamilyState {
                    seeds: vec![p(1, 1, 1)],
                    saw_unknown: true,
                }),
                Ok(HalfspaceSeedFamilyState {
                    seeds: vec![p(2, 2, 2)],
                    saw_unknown: false,
                }),
            ],
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(seeds, vec![p(1, 1, 1), p(2, 2, 2)]);
    }

    #[test]
    fn strict_halfspace_cell_seed_family_search_reports_unknown_if_all_families_are_uncertified() {
        let mut seeds = Vec::new();

        let err = extend_strict_halfspace_seed_families_backtracking_unknown(
            &mut seeds,
            [
                Err(HypermeshError::UnknownClassification),
                Err(HypermeshError::UnknownClassification),
            ],
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn collect_strict_halfspace_seed_family_tracks_unknown_after_later_strict_seed() {
        let family =
            collect_strict_halfspace_seed_family(Ok(vec![p(1, 1, 1), p(2, 2, 2)]), |candidate| {
                if *candidate == p(1, 1, 1) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(true)
                }
            })
            .unwrap();

        assert_eq!(family.seeds, vec![p(2, 2, 2)]);
        assert!(family.saw_unknown);
    }

    #[test]
    fn seed_family_search_failure_allows_later_shifted_seeds_after_unknown_strict_family() {
        assert!(!seed_family_search_failed_without_any_seed(
            &[],
            &[p(1, 1, 1)],
            &[],
            true,
        ));
        assert!(!seed_family_search_failed_without_any_seed(
            &[],
            &[],
            &[p(2, 2, 2)],
            true,
        ));
    }

    #[test]
    fn seed_family_search_failure_reports_unknown_only_when_every_seed_family_is_empty() {
        assert!(seed_family_search_failed_without_any_seed(
            &[],
            &[],
            &[],
            true,
        ));
        assert!(!seed_family_search_failed_without_any_seed(
            &[p(3, 3, 3)],
            &[],
            &[],
            true,
        ));
    }

    #[test]
    fn take_new_halfspace_seed_family_preserves_first_occurrence_order() {
        let mut seen = vec![p(0, 0, 0)];
        let fresh = take_new_halfspace_seed_family(
            vec![p(1, 1, 1), p(0, 0, 0), p(2, 2, 2), p(1, 1, 1)],
            &mut seen,
        );

        assert_eq!(fresh, vec![p(1, 1, 1), p(2, 2, 2)]);
        assert_eq!(seen, vec![p(0, 0, 0), p(1, 1, 1), p(2, 2, 2)]);
    }

    #[test]
    fn shifted_halfspace_seed_families_preserve_direct_report_witness_and_skip_later_duplicates() {
        let witness = p(1, 1, 1);
        let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
            dedupe_shifted_halfspace_seed_families(
                Some(&witness),
                vec![witness.clone(), p(2, 1, 1)],
                vec![p(2, 1, 1), witness.clone(), p(3, 1, 1)],
                vec![p(3, 1, 1), witness.clone(), p(4, 1, 1)],
            );

        assert_eq!(strict_seeds, vec![witness, p(2, 1, 1)]);
        assert_eq!(shifted_vertices, vec![p(3, 1, 1)]);
        assert_eq!(shifted_geometry_seeds, vec![p(4, 1, 1)]);
    }

    #[test]
    fn shifted_halfspace_witness_seed_family_search_skips_duplicate_seed_sources() {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut witnesses = Vec::new();
        let visited = std::cell::RefCell::new(Vec::new());

        extend_shifted_halfspace_seed_families_backtracking_unknown(
            &mut witnesses,
            [
                vec![first.clone(), second.clone()],
                vec![second.clone(), first.clone()],
            ],
            |seed| {
                visited.borrow_mut().push(seed.clone());
                Ok(vec![ShiftedHalfspaceWitness::with_family(
                    seed.clone(),
                    Vec::new(),
                    [None, None, None],
                    false,
                )])
            },
        )
        .unwrap();

        assert_eq!(visited.into_inner(), vec![first.clone(), second.clone()]);
        assert_eq!(
            witnesses
                .into_iter()
                .map(|witness| witness.point)
                .collect::<Vec<_>>(),
            vec![first, second]
        );
    }

    #[test]
    fn shifted_halfspace_witness_seed_family_search_backtracks_after_uncertified_earlier_family() {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut witnesses = Vec::new();

        extend_shifted_halfspace_seed_families_backtracking_unknown(
            &mut witnesses,
            [vec![first.clone()], vec![first, second.clone()]],
            |seed| {
                if *seed == p(2, 2, 2) {
                    Ok(vec![ShiftedHalfspaceWitness::with_family(
                        seed.clone(),
                        Vec::new(),
                        [None, None, None],
                        false,
                    )])
                } else {
                    Err(HypermeshError::UnknownClassification)
                }
            },
        )
        .unwrap();

        assert_eq!(
            witnesses
                .into_iter()
                .map(|witness| witness.point)
                .collect::<Vec<_>>(),
            vec![second]
        );
    }

    #[test]
    fn shifted_halfspace_witness_seed_family_search_marks_existing_witnesses_uncertain_after_later_unknown()
     {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut witnesses = Vec::new();

        extend_shifted_halfspace_seed_families_backtracking_unknown(
            &mut witnesses,
            [vec![first.clone()], vec![second.clone()]],
            |seed| {
                if *seed == first {
                    Ok(vec![ShiftedHalfspaceWitness::with_family(
                        seed.clone(),
                        Vec::new(),
                        [None, None, None],
                        false,
                    )])
                } else {
                    Err(HypermeshError::UnknownClassification)
                }
            },
        )
        .unwrap();

        assert_eq!(witnesses.len(), 1);
        assert_eq!(witnesses[0].point, first);
        assert!(witnesses[0].uncertified_definition_fallback);
    }

    #[test]
    fn shifted_halfspace_witness_seed_family_search_marks_later_witnesses_uncertain_after_uncertain_family_result()
     {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut witnesses = Vec::new();

        extend_shifted_halfspace_seed_families_backtracking_unknown(
            &mut witnesses,
            [vec![first.clone()], vec![second.clone()]],
            |seed| {
                if *seed == first {
                    Ok(vec![ShiftedHalfspaceWitness::with_family(
                        seed.clone(),
                        Vec::new(),
                        [None, None, None],
                        true,
                    )])
                } else {
                    Ok(vec![ShiftedHalfspaceWitness::with_family(
                        seed.clone(),
                        Vec::new(),
                        [None, None, None],
                        false,
                    )])
                }
            },
        )
        .unwrap();

        assert_eq!(witnesses.len(), 2);
        assert!(
            witnesses
                .iter()
                .all(|witness| witness.uncertified_definition_fallback)
        );
    }

    #[test]
    fn strict_halfspace_cell_seeds_include_strict_geometry_seeds_without_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let triangle_center = centroid(&[p(0, 0, 0), p(4, 0, 0), p(4, 4, 4)])
            .unwrap()
            .unwrap();
        let tetra_center = p(1, 1, 1);

        let seeds =
            strict_halfspace_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

        assert!(
            point_strictly_inside_halfspace_cell(&triangle_center, &bounds, &halfspaces).unwrap()
        );
        assert!(point_strictly_inside_halfspace_cell(&tetra_center, &bounds, &halfspaces).unwrap());
        assert!(seeds.iter().any(|seed| seed == &triangle_center));
        assert!(seeds.iter().any(|seed| seed == &tetra_center));
    }

    #[test]
    fn strict_halfspace_cell_seeds_include_strict_edge_midpoints() {
        let (bounds, halfspaces, midpoint) = quadrilateral_halfspace_cell_fixture();

        let seeds =
            strict_halfspace_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

        assert!(point_strictly_inside_halfspace_cell(&midpoint, &bounds, &halfspaces).unwrap());
        assert!(seeds.iter().any(|seed| seed == &midpoint));
    }

    #[test]
    fn strict_halfspace_cell_seeds_include_strict_five_vertex_centroids() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let five_vertex_center = Point3::new(q(8, 5), q(8, 5), q(8, 5));

        let seeds =
            strict_halfspace_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

        assert!(
            point_strictly_inside_halfspace_cell(&five_vertex_center, &bounds, &halfspaces)
                .unwrap()
        );
        assert!(seeds.iter().any(|seed| seed == &five_vertex_center));
    }

    #[test]
    fn strict_leaf_witness_seeds_include_strict_halfspace_triangle_centroid() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let vertices = leaf.vertices().unwrap();
        let bounds = leaf_bounds(&vertices).unwrap();
        let halfspaces = leaf_halfspaces(&leaf);
        let report = halfspace_feasibility_report(&halfspaces).unwrap();
        let center = centroid(&feasible_halfspace_cell_vertices(&halfspaces).unwrap())
            .unwrap()
            .unwrap();

        let seeds =
            strict_leaf_witness_seeds(&leaf, &vertices, &bounds, &halfspaces, Some(&report))
                .unwrap();

        assert!(point_strictly_inside_leaf(&center, &leaf).unwrap());
        assert!(seeds.iter().any(|seed| seed == &center));
    }

    #[test]
    fn strict_leaf_witness_seeds_include_strict_halfspace_geometry_family() {
        let leaf = make_quad(&p(0, 0, 0), &p(4, 0, 0), &p(4, 4, 0), &p(0, 4, 0), 0, 0);
        let vertices = leaf.vertices().unwrap();
        let bounds = leaf_bounds(&vertices).unwrap();
        let halfspaces = leaf_halfspaces(&leaf);
        let report = halfspace_feasibility_report(&halfspaces).unwrap();
        let triangle_center = centroid(&[p(0, 0, 0), p(4, 0, 0), p(4, 4, 0)])
            .unwrap()
            .unwrap();

        let seeds =
            strict_leaf_witness_seeds(&leaf, &vertices, &bounds, &halfspaces, Some(&report))
                .unwrap();

        assert!(point_strictly_inside_leaf(&triangle_center, &leaf).unwrap());
        assert!(seeds.iter().any(|seed| seed == &triangle_center));
    }

    #[test]
    fn shifted_edge_interior_points_move_vertices_inside_by_certified_margins() {
        let leaf = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0);
        let vertices = leaf.vertices().unwrap();
        let center = centroid(&vertices).unwrap().unwrap();
        let points = shifted_edge_interior_points(&leaf, &center).unwrap();

        assert_eq!(points.len(), 3);
        for point in &points {
            assert!(point_strictly_inside_leaf(&point.point, &leaf).unwrap());
        }

        let first = &points[0].point;
        let expected_first_edge_margin =
            (leaf.edges[0].expression_at_point(&center) / Real::from(2)).unwrap();
        let expected_second_edge_margin =
            (leaf.edges[1].expression_at_point(&center) / Real::from(2)).unwrap();

        assert_eq!(
            leaf.edges[0].expression_at_point(first),
            expected_first_edge_margin
        );
        assert_eq!(
            leaf.edges[1].expression_at_point(first),
            expected_second_edge_margin
        );
    }

    #[test]
    fn bounded_probes_include_certified_normal_direction_probe() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let vertices = leaf.vertices().unwrap();
        let center = centroid(&vertices).unwrap().unwrap();
        let interior = shifted_edge_interior_points(&leaf, &center)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("shifted edge construction should retain defining planes");

        let probes =
            bounded_probes_from_interior(&interior, &leaf.support, &bounds, true, &[]).unwrap();

        let probe = probes
            .iter()
            .find(|probe| probe.side == Classification::Positive && !probe.planes.is_empty())
            .expect("normal probe should preserve a shifted plane definition");
        let planes = &probe.planes[0];
        assert_eq!(affine_from_planes(planes).unwrap(), probe.point);
    }

    #[test]
    fn bounded_probe_family_collection_backtracks_after_uncertified_family() {
        let constrained_probe = ProbePoint {
            point: p(1, 1, 1),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let mut probes = Vec::new();
        let mut saw_unknown = false;

        extend_probe_families_backtracking_unknown(
            &mut probes,
            Err(HypermeshError::UnknownClassification),
            &mut saw_unknown,
        )
        .unwrap();
        extend_probe_families_backtracking_unknown(
            &mut probes,
            Ok(vec![constrained_probe.clone()]),
            &mut saw_unknown,
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].point, constrained_probe.point);
    }

    #[test]
    fn bounded_probe_family_collection_reports_unknown_if_all_families_are_uncertified() {
        let mut probes = Vec::new();
        let mut saw_unknown = false;

        extend_probe_families_backtracking_unknown(
            &mut probes,
            Err(HypermeshError::UnknownClassification),
            &mut saw_unknown,
        )
        .unwrap();
        extend_probe_families_backtracking_unknown(
            &mut probes,
            Err(HypermeshError::UnknownClassification),
            &mut saw_unknown,
        )
        .unwrap();

        assert!(saw_unknown);
        assert!(probes.is_empty());
    }

    #[test]
    fn bounded_probe_family_collection_tracks_unknown_after_uncertain_family_result() {
        let uncertain_probe = ProbePoint {
            point: p(1, 1, 1),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: true,
        };
        let certain_probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: false,
        };
        let mut probes = Vec::new();
        let mut saw_unknown = false;

        extend_probe_families_backtracking_unknown(
            &mut probes,
            Ok(vec![uncertain_probe]),
            &mut saw_unknown,
        )
        .unwrap();
        extend_probe_families_backtracking_unknown(
            &mut probes,
            Ok(vec![certain_probe]),
            &mut saw_unknown,
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(probes.len(), 2);
    }

    #[test]
    fn leaf_probe_family_search_backtracks_after_uncertified_probe_family() {
        let first = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let second = InteriorLeafPoint {
            point: p(2, 2, 2),
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: false,
        };
        let winning_probe = ProbePoint {
            point: p(3, 3, 3),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(3, 3, 3))],
            uncertified_definition_fallback: false,
        };

        let winding = search_leaf_probe_families(
            &[first.clone(), second.clone()],
            |point, _positive_side| {
                if point.point == first.point {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(vec![winning_probe.clone()])
                }
            },
            |point, _positive_side, _probe| {
                if point.point == second.point {
                    Ok(Some(vec![1]))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert_eq!(winding, Some(vec![1]));
    }

    #[test]
    fn leaf_probe_family_search_backtracks_after_uncertified_probe_check() {
        let first = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let second = InteriorLeafPoint {
            point: p(2, 2, 2),
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: false,
        };
        let probe = ProbePoint {
            point: p(3, 3, 3),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(3, 3, 3))],
            uncertified_definition_fallback: false,
        };

        let winding = search_leaf_probe_families(
            &[first.clone(), second.clone()],
            |_point, _positive_side| Ok(vec![probe.clone()]),
            |point, _positive_side, _probe| {
                if point.point == first.point {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(vec![2]))
                }
            },
        )
        .unwrap();

        assert_eq!(winding, Some(vec![2]));
    }

    #[test]
    fn leaf_probe_family_search_reports_unknown_if_all_families_are_uncertified() {
        let point = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };

        let err = search_leaf_probe_families(
            &[point],
            |_point, _positive_side| Err(HypermeshError::UnknownClassification),
            |_point, _positive_side, _probe| Ok(Some(vec![1])),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn leaf_probe_family_search_reports_unknown_when_fallback_probe_is_rejected() {
        let point = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: true,
        };

        let err = search_leaf_probe_families(
            &[point],
            |_point, _positive_side| Ok(vec![probe.clone()]),
            |_point, _positive_side, _probe| Ok(None),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn leaf_probe_family_search_reports_unknown_when_fallback_interior_has_no_probes() {
        let point = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: true,
        };

        let err = search_leaf_probe_families(
            &[point],
            |_point, _positive_side| Ok(Vec::new()),
            |_point, _positive_side, _probe| Ok(Some(vec![1])),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn cached_probe_winding_reuses_equivalent_trace_across_probe_sides() {
        let definition = axis_plane_definition(&p(1, 2, 3));
        let positive = ProbePoint {
            point: p(1, 2, 3),
            side: Classification::Positive,
            planes: vec![definition.clone()],
            uncertified_definition_fallback: false,
        };
        let negative = ProbePoint {
            point: p(1, 2, 3),
            side: Classification::Negative,
            planes: vec![definition],
            uncertified_definition_fallback: false,
        };
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_probe_winding_with(&mut cache, &positive, || {
            calls += 1;
            Ok(vec![7])
        })
        .unwrap();
        let second = cached_probe_winding_with(&mut cache, &negative, || {
            calls += 1;
            Ok(vec![9])
        })
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, vec![7]);
        assert_eq!(second, vec![7]);
    }

    #[test]
    fn cached_probe_winding_reuses_permuted_definition_families() {
        let definition = axis_plane_definition(&p(1, 2, 3));
        let permuted = [
            definition[1].clone(),
            definition[2].clone(),
            definition[0].clone(),
        ];
        let first = ProbePoint {
            point: p(1, 2, 3),
            side: Classification::Positive,
            planes: vec![definition],
            uncertified_definition_fallback: false,
        };
        let second = ProbePoint {
            point: p(1, 2, 3),
            side: Classification::Positive,
            planes: vec![permuted],
            uncertified_definition_fallback: false,
        };
        let mut cache = Vec::new();
        let mut calls = 0;

        let first_result = cached_probe_winding_with(&mut cache, &first, || {
            calls += 1;
            Ok(vec![5])
        })
        .unwrap();
        let second_result = cached_probe_winding_with(&mut cache, &second, || {
            calls += 1;
            Ok(vec![9])
        })
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first_result, vec![5]);
        assert_eq!(second_result, vec![5]);
    }

    #[test]
    fn cached_surface_and_probe_reachability_reuse_equivalent_queries() {
        let definition = axis_plane_definition(&p(1, 2, 3));
        let interior = InteriorLeafPoint {
            point: p(0, 0, 0),
            planes: vec![axis_plane_definition(&p(0, 0, 0))],
            uncertified_definition_fallback: false,
        };
        let probe = ProbePoint {
            point: p(1, 2, 3),
            side: Classification::Positive,
            planes: vec![definition],
            uncertified_definition_fallback: false,
        };
        let mut surface_cache = Vec::new();
        let mut reachability_cache = Vec::new();
        let mut surface_calls = 0;
        let mut reachability_calls = 0;

        let first_surface = cached_surface_query_with(&mut surface_cache, &probe.point, || {
            surface_calls += 1;
            Ok(false)
        })
        .unwrap();
        let second_surface = cached_surface_query_with(&mut surface_cache, &probe.point, || {
            surface_calls += 1;
            Ok(true)
        })
        .unwrap();
        let first_reachability =
            cached_probe_reachability_with(&mut reachability_cache, &interior, &probe, || {
                reachability_calls += 1;
                Ok(true)
            })
            .unwrap();
        let second_reachability =
            cached_probe_reachability_with(&mut reachability_cache, &interior, &probe, || {
                reachability_calls += 1;
                Ok(false)
            })
            .unwrap();

        assert_eq!(surface_calls, 1);
        assert!(!first_surface);
        assert!(!second_surface);
        assert_eq!(reachability_calls, 1);
        assert!(first_reachability);
        assert!(second_reachability);
    }

    #[test]
    fn cached_probe_reachability_reuses_permuted_definition_families() {
        let interior_definition = axis_plane_definition(&p(0, 0, 0));
        let interior_permuted = [
            interior_definition[1].clone(),
            interior_definition[2].clone(),
            interior_definition[0].clone(),
        ];
        let probe_definition = axis_plane_definition(&p(1, 2, 3));
        let probe_permuted = [
            probe_definition[1].clone(),
            probe_definition[2].clone(),
            probe_definition[0].clone(),
        ];
        let first_interior = InteriorLeafPoint {
            point: p(0, 0, 0),
            planes: vec![interior_definition],
            uncertified_definition_fallback: false,
        };
        let second_interior = InteriorLeafPoint {
            point: p(0, 0, 0),
            planes: vec![interior_permuted],
            uncertified_definition_fallback: false,
        };
        let first_probe = ProbePoint {
            point: p(1, 2, 3),
            side: Classification::Positive,
            planes: vec![probe_definition],
            uncertified_definition_fallback: false,
        };
        let second_probe = ProbePoint {
            point: p(1, 2, 3),
            side: Classification::Positive,
            planes: vec![probe_permuted],
            uncertified_definition_fallback: false,
        };
        let mut cache = Vec::new();
        let mut calls = 0;

        let first_result =
            cached_probe_reachability_with(&mut cache, &first_interior, &first_probe, || {
                calls += 1;
                Ok(true)
            })
            .unwrap();
        let second_result =
            cached_probe_reachability_with(&mut cache, &second_interior, &second_probe, || {
                calls += 1;
                Ok(false)
            })
            .unwrap();

        assert_eq!(calls, 1);
        assert!(first_result);
        assert!(second_result);
    }

    #[test]
    fn trace_axis_ordered_paths_reuse_equivalent_intermediate_surface_queries() {
        let start = p(0, 0, 0);
        let end = p(1, 1, 0);
        let mut surface_cache = Vec::new();
        let mut query_calls = 0;

        let err = trace_axis_ordered_paths_with_surface_query(&start, &end, &[0], &[], |point| {
            cached_surface_query_with(&mut surface_cache, point, || {
                query_calls += 1;
                Ok(*point == p(1, 0, 0) || *point == p(0, 1, 0))
            })
        })
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
        assert_eq!(query_calls, 2);
    }

    #[test]
    fn trace_axis_ordered_paths_reuse_equivalent_segment_traces() {
        let start = p(0, 0, 0);
        let end = p(1, 1, 0);
        let mut trace_calls = 0;

        let err = trace_axis_ordered_paths_with_queries(
            &start,
            &end,
            &[0],
            &[],
            |_point| Ok(false),
            |_current, _next, _axis, attempt, _polygons| {
                trace_calls += 1;
                Ok(TraceAxisSegmentResult {
                    winding: attempt.to_vec(),
                    valid: false,
                })
            },
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
        assert_eq!(trace_calls, 2);
    }

    #[test]
    fn trace_axis_ordered_paths_try_later_ordering_after_uncertified_surface_query() {
        let start = p(0, 0, 0);
        let end = p(1, 1, 0);

        let winding = trace_axis_ordered_paths_with_queries(
            &start,
            &end,
            &[7],
            &[],
            |point| {
                if *point == p(1, 0, 0) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(false)
                }
            },
            |_current, _next, _axis, attempt, _polygons| {
                Ok(TraceAxisSegmentResult {
                    winding: attempt.to_vec(),
                    valid: true,
                })
            },
        )
        .unwrap();

        assert_eq!(winding, vec![7]);
    }

    #[test]
    fn trace_axis_ordered_paths_try_later_ordering_after_uncertified_segment_step() {
        let start = p(0, 0, 0);
        let end = p(1, 1, 0);

        let winding = trace_axis_ordered_paths_with_queries(
            &start,
            &end,
            &[7],
            &[],
            |_point| Ok(false),
            |current, next, _axis, attempt, _polygons| {
                if *current == start && *next == p(1, 0, 0) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(TraceAxisSegmentResult {
                        winding: attempt.to_vec(),
                        valid: true,
                    })
                }
            },
        )
        .unwrap();

        assert_eq!(winding, vec![7]);
    }

    #[test]
    fn cached_definition_no_detour_trace_reuses_identical_query() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let mut cache = Vec::new();
        let mut trace_calls = 0;

        let first = cached_definition_no_detour_trace_with(
            &mut cache,
            &start,
            &end,
            &[7],
            &start_definitions,
            &end_definitions,
            || {
                trace_calls += 1;
                Ok(Some(vec![7]))
            },
        )
        .unwrap();
        let second = cached_definition_no_detour_trace_with(
            &mut cache,
            &start,
            &end,
            &[7],
            &start_definitions,
            &end_definitions,
            || {
                trace_calls += 1;
                Ok(Some(vec![7]))
            },
        )
        .unwrap();

        assert_eq!(first, Some(vec![7]));
        assert_eq!(second, Some(vec![7]));
        assert_eq!(trace_calls, 1);
    }

    #[test]
    fn cached_definition_no_detour_trace_reuses_permuted_definition_families() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_a = axis_plane_definition(&start);
        let start_b = axis_plane_definition(&p(0, 1, 0));
        let end_a = axis_plane_definition(&end);
        let end_b = axis_plane_definition(&p(1, 1, 0));
        let mut cache = Vec::new();
        let mut trace_calls = 0;

        let first = cached_definition_no_detour_trace_with(
            &mut cache,
            &start,
            &end,
            &[7],
            &[start_a.clone(), start_b.clone()],
            &[end_a.clone(), end_b.clone()],
            || {
                trace_calls += 1;
                Ok(Some(vec![7]))
            },
        )
        .unwrap();
        let second = cached_definition_no_detour_trace_with(
            &mut cache,
            &start,
            &end,
            &[7],
            &[start_b, start_a],
            &[end_b, end_a],
            || {
                trace_calls += 1;
                Ok(Some(vec![7]))
            },
        )
        .unwrap();

        assert_eq!(first, Some(vec![7]));
        assert_eq!(second, Some(vec![7]));
        assert_eq!(trace_calls, 1);
    }

    #[test]
    fn trace_segment_from_definitions_shared_query_caches_reuse_equivalent_calls() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let mut no_detour_cache = Vec::new();
        let mut detour_target_cache = Vec::new();

        let first = trace_segment_from_definitions_with_caches(
            &start,
            &end,
            &[7],
            &[],
            &start_definitions,
            &end_definitions,
            &mut no_detour_cache,
            &mut detour_target_cache,
        )
        .unwrap();
        let no_detour_len = no_detour_cache.len();
        let detour_len = detour_target_cache.len();
        let second = trace_segment_from_definitions_with_caches(
            &start,
            &end,
            &[7],
            &[],
            &start_definitions,
            &end_definitions,
            &mut no_detour_cache,
            &mut detour_target_cache,
        )
        .unwrap();

        assert_eq!(first, vec![7]);
        assert_eq!(second, vec![7]);
        assert_eq!(no_detour_cache.len(), no_detour_len);
        assert_eq!(detour_target_cache.len(), detour_len);
    }

    #[test]
    fn cached_definition_no_detour_reachability_reuses_identical_query() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let mut cache = Vec::new();
        let mut trace_calls = 0;

        let first = cached_definition_no_detour_reachability_with(
            &mut cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            || {
                trace_calls += 1;
                Ok(true)
            },
        )
        .unwrap();
        let second = cached_definition_no_detour_reachability_with(
            &mut cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            || {
                trace_calls += 1;
                Ok(true)
            },
        )
        .unwrap();

        assert!(first);
        assert!(second);
        assert_eq!(trace_calls, 1);
    }

    #[test]
    fn cached_definition_no_detour_reachability_reuses_permuted_definition_families() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_a = axis_plane_definition(&start);
        let start_b = axis_plane_definition(&p(0, 1, 0));
        let end_a = axis_plane_definition(&end);
        let end_b = axis_plane_definition(&p(1, 1, 0));
        let mut cache = Vec::new();
        let mut trace_calls = 0;

        let first = cached_definition_no_detour_reachability_with(
            &mut cache,
            &start,
            &end,
            &[start_a.clone(), start_b.clone()],
            &[end_a.clone(), end_b.clone()],
            || {
                trace_calls += 1;
                Ok(true)
            },
        )
        .unwrap();
        let second = cached_definition_no_detour_reachability_with(
            &mut cache,
            &start,
            &end,
            &[start_b, start_a],
            &[end_b, end_a],
            || {
                trace_calls += 1;
                Ok(true)
            },
        )
        .unwrap();

        assert!(first);
        assert!(second);
        assert_eq!(trace_calls, 1);
    }

    #[test]
    fn cached_detour_target_family_reuses_identical_query() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let target = DetourTarget {
            point: p(0, 1, 0),
            definitions: vec![axis_plane_definition(&p(0, 1, 0))],
            uncertified_definition_fallback: false,
        };
        let mut cache = Vec::new();
        let mut build_calls = 0;

        let first = cached_detour_target_family_with(&mut cache, &start, &end, || {
            build_calls += 1;
            Ok(vec![target.clone()])
        })
        .unwrap();
        let second = cached_detour_target_family_with(&mut cache, &start, &end, || {
            build_calls += 1;
            Ok(vec![target.clone()])
        })
        .unwrap();

        assert_eq!(first, vec![target.clone()]);
        assert_eq!(second, vec![target]);
        assert_eq!(build_calls, 1);
    }

    #[test]
    fn cached_detour_target_family_reuses_reversed_query() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let target = DetourTarget {
            point: p(0, 1, 0),
            definitions: vec![axis_plane_definition(&p(0, 1, 0))],
            uncertified_definition_fallback: false,
        };
        let mut cache = Vec::new();
        let mut build_calls = 0;

        let first = cached_detour_target_family_with(&mut cache, &start, &end, || {
            build_calls += 1;
            Ok(vec![target.clone()])
        })
        .unwrap();
        let second = cached_detour_target_family_with(&mut cache, &end, &start, || {
            build_calls += 1;
            Ok(vec![target.clone()])
        })
        .unwrap();

        assert_eq!(first, vec![target.clone()]);
        assert_eq!(second, vec![target]);
        assert_eq!(build_calls, 1);
    }

    #[test]
    fn detour_trace_cycle_guard_reuses_surface_queries_across_failed_branches() {
        let start = p(0, 0, 0);
        let shared = p(1, 0, 0);
        let outer_b = p(2, 0, 0);
        let outer_a = p(3, 0, 0);
        let end = p(4, 0, 0);
        let outer_targets = vec![
            DetourTarget {
                point: outer_a.clone(),
                definitions: vec![axis_plane_definition(&outer_a)],
                uncertified_definition_fallback: false,
            },
            DetourTarget {
                point: outer_b.clone(),
                definitions: vec![axis_plane_definition(&outer_b)],
                uncertified_definition_fallback: false,
            },
        ];
        let shared_target = DetourTarget {
            point: shared.clone(),
            definitions: vec![axis_plane_definition(&shared)],
            uncertified_definition_fallback: false,
        };
        let mut surface_cache = Vec::new();
        let mut query_calls = 0;

        let err = trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[0],
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &[start.clone(), end.clone()],
            &mut surface_cache,
            &mut |_point| {
                query_calls += 1;
                Ok(false)
            },
            &mut |_from, _to, _winding, _start_definitions, _end_definitions| Ok(None),
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(outer_targets.clone())
                } else if *from == start && (*to == outer_a || *to == outer_b) {
                    Ok(vec![shared_target.clone()])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
        assert_eq!(query_calls, 3);
    }

    #[test]
    fn normal_probe_is_clipped_before_intervening_surface() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let blocker = make_triangle(&p(6, 0, 0), &p(0, 6, 0), &p(0, 0, 6), 1, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let vertices = leaf.vertices().unwrap();
        let center = centroid(&vertices).unwrap().unwrap();
        let interior = shifted_edge_interior_points(&leaf, &center)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("shifted edge construction should retain defining planes");

        let probes =
            adjacent_normal_probes(&interior, &leaf.support, &bounds, &[blocker.clone()], true)
                .unwrap();
        let probe = probes
            .into_iter()
            .find(|probe| probe.side == Classification::Positive)
            .expect("normal corridor should contain a certified probe witness");

        assert!(!probe.planes.is_empty());
        for definition in &probe.planes {
            assert_eq!(affine_from_planes(definition).unwrap(), probe.point);
        }
        let start_value = leaf.support.expression_at_point(&interior.point);
        let probe_value = leaf.support.expression_at_point(&probe.point);
        let blocker_value = blocker.support.expression_at_point(&probe.point);
        assert!(compare_real(&probe_value, &start_value).unwrap().is_gt());
        assert!(compare_real(&blocker_value, &Real::zero()).unwrap().is_lt());
    }

    #[test]
    fn adjacent_normal_probe_stop_values_backtrack_after_uncertified_crossing() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = p(1, 1, 1);
        let direction = support.normal.clone();
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
            &interior,
            &direction,
            &support,
            &bounds,
            &[first, second],
            &mut |_interior, direction, polygon| {
                Ok(dot_direction(&polygon.support.normal, direction))
            },
            &mut |point, polygon| {
                if polygon.vertices().unwrap()[0].x == r(2) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(if *point == p(3, 1, 1) {
                        PolygonPointLocation::Interior
                    } else {
                        PolygonPointLocation::Outside
                    })
                }
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(stop_values, vec![r(2), r(3)]);
    }

    #[test]
    fn adjacent_normal_probe_accepts_later_corridor_after_uncertified_crossing() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let probes = adjacent_normal_probes_with_queries(
            &interior,
            &support,
            &bounds,
            &[first, second],
            true,
            |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
            |point, polygon| {
                if polygon.vertices().unwrap()[0].x == r(2) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(if *point == p(3, 1, 1) {
                        PolygonPointLocation::Interior
                    } else {
                        PolygonPointLocation::Outside
                    })
                }
            },
            |corridor, stop_point| {
                if corridor.max.x == r(3) && *stop_point == p(3, 1, 1) {
                    Ok(vec![ProbePoint {
                        point: p(2, 1, 1),
                        side: Classification::Positive,
                        planes: vec![axis_plane_definition(&p(2, 1, 1))],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].point, p(2, 1, 1));
        assert!(probes[0].uncertified_definition_fallback);
    }

    #[test]
    fn adjacent_normal_probe_reports_unknown_when_corridor_family_is_partially_uncertified_and_later_corridors_fail()
     {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let err = adjacent_normal_probes_with_queries(
            &interior,
            &support,
            &bounds,
            &[first, second],
            true,
            |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
            |point, polygon| {
                if polygon.vertices().unwrap()[0].x == r(2) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(if *point == p(3, 1, 1) {
                        PolygonPointLocation::Interior
                    } else {
                        PolygonPointLocation::Outside
                    })
                }
            },
            |_corridor, _stop_point| Ok(Vec::new()),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn adjacent_normal_probe_stop_values_treat_boundary_start_contact_as_unknown_and_keep_later_corridor()
     {
        let support = Plane::axis_aligned(0, r(0));
        let interior = p(1, 1, 1);
        let direction = support.normal.clone();
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
            &interior,
            &direction,
            &support,
            &bounds,
            &[first, second],
            &mut |_interior, direction, polygon| {
                Ok(dot_direction(&polygon.support.normal, direction))
            },
            &mut |_point, polygon| {
                if polygon.vertices().unwrap()[0].x == r(1) {
                    Ok(PolygonPointLocation::Boundary)
                } else {
                    Ok(PolygonPointLocation::Interior)
                }
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(stop_values, vec![r(2), r(3)]);
    }

    #[test]
    fn adjacent_normal_probe_accepts_later_corridor_after_boundary_start_contact() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let probes = adjacent_normal_probes_with_queries(
            &interior,
            &support,
            &bounds,
            &[first, second],
            true,
            |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
            |_point, polygon| {
                if polygon.vertices().unwrap()[0].x == r(1) {
                    Ok(PolygonPointLocation::Boundary)
                } else {
                    Ok(PolygonPointLocation::Interior)
                }
            },
            |corridor, stop_point| {
                if corridor.max.x == r(3) && *stop_point == p(3, 1, 1) {
                    Ok(vec![ProbePoint {
                        point: p(2, 1, 1),
                        side: Classification::Positive,
                        planes: vec![axis_plane_definition(&p(2, 1, 1))],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].point, p(2, 1, 1));
        assert!(probes[0].uncertified_definition_fallback);
    }

    #[test]
    fn collect_normal_probe_targets_keeps_unrestricted_family_after_definition_hits() {
        let support = Plane::axis_aligned(2, r(0));
        let definition = [
            support.clone(),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let constrained_probe = ProbePoint {
            point: p(1, 1, 1),
            side: Classification::Positive,
            planes: vec![definition.clone()],
            uncertified_definition_fallback: false,
        };
        let unrestricted_probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: false,
        };

        let probes = collect_normal_probe_targets(&[definition], |candidate| match candidate {
            Some(_) => Ok(vec![constrained_probe.clone()]),
            None => Ok(vec![unrestricted_probe.clone()]),
        })
        .unwrap();

        assert_eq!(probes.len(), 2);
        assert!(
            probes
                .iter()
                .any(|probe| probe.point == constrained_probe.point)
        );
        assert!(
            probes
                .iter()
                .any(|probe| probe.point == unrestricted_probe.point)
        );
    }

    #[test]
    fn collect_normal_probe_targets_merges_duplicate_unrestricted_probe_definitions() {
        let support = Plane::axis_aligned(2, r(0));
        let definition_probe = ProbePoint {
            point: p(1, 1, 1),
            side: Classification::Positive,
            planes: vec![[
                support.clone(),
                Plane::axis_aligned(0, r(1)),
                Plane::axis_aligned(1, r(1)),
            ]],
            uncertified_definition_fallback: false,
        };
        let extra_definition = [
            support,
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(2, r(1)),
        ];

        let probes =
            collect_normal_probe_targets(&[definition_probe.planes[0].clone()], |candidate| {
                match candidate {
                    Some(_) => Ok(vec![definition_probe.clone()]),
                    None => Ok(vec![ProbePoint {
                        point: definition_probe.point.clone(),
                        side: definition_probe.side,
                        planes: vec![extra_definition.clone()],
                        uncertified_definition_fallback: false,
                    }]),
                }
            })
            .unwrap();

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].planes.len(), 2);
    }

    #[test]
    fn collect_normal_probe_targets_skips_duplicate_definition_families() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let mut definition_calls = 0;
        let mut unrestricted_calls = 0;

        let probes =
            collect_normal_probe_targets(&[definition.clone(), definition.clone()], |candidate| {
                match candidate {
                    Some(found_definition) => {
                        definition_calls += 1;
                        assert_eq!(found_definition, &definition);
                        Ok(vec![ProbePoint {
                            point: p(0, 0, 1),
                            side: Classification::Positive,
                            planes: vec![definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
                    }
                    None => {
                        unrestricted_calls += 1;
                        Ok(Vec::new())
                    }
                }
            })
            .unwrap();

        assert_eq!(definition_calls, 1);
        assert_eq!(unrestricted_calls, 1);
        assert_eq!(probes.len(), 1);
    }

    #[test]
    fn collect_normal_probe_targets_skips_permuted_definition_families() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let permuted_definition = [
            definition[1].clone(),
            definition[2].clone(),
            definition[0].clone(),
        ];
        let mut definition_calls = 0;
        let mut unrestricted_calls = 0;

        let probes =
            collect_normal_probe_targets(&[definition.clone(), permuted_definition], |candidate| {
                match candidate {
                    Some(found_definition) => {
                        definition_calls += 1;
                        assert!(definition_planes_match_as_sets(
                            found_definition,
                            &definition
                        ));
                        Ok(vec![ProbePoint {
                            point: p(0, 0, 1),
                            side: Classification::Positive,
                            planes: vec![definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
                    }
                    None => {
                        unrestricted_calls += 1;
                        Ok(Vec::new())
                    }
                }
            })
            .unwrap();

        assert_eq!(definition_calls, 1);
        assert_eq!(unrestricted_calls, 1);
        assert_eq!(probes.len(), 1);
    }

    #[test]
    fn collect_normal_probe_targets_backtracks_after_uncertified_definition() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let unrestricted_probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: false,
        };

        let probes = collect_normal_probe_targets(&[definition], |candidate| match candidate {
            Some(_) => Err(HypermeshError::UnknownClassification),
            None => Ok(vec![unrestricted_probe.clone()]),
        })
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].point, unrestricted_probe.point);
    }

    #[test]
    fn collect_normal_probe_targets_report_unknown_if_all_families_are_uncertified() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];

        let err = collect_normal_probe_targets(&[definition], |_candidate| {
            Err(HypermeshError::UnknownClassification)
        })
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn probe_point_build_collection_backtracks_after_uncertified_candidate() {
        let mut probes = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_probe_point_builds_backtracking_unknown(
            &mut probes,
            [first.clone(), second.clone()].iter(),
            |candidate| {
                if *candidate == first {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(ProbePoint {
                        point: candidate.clone(),
                        side: Classification::Positive,
                        planes: vec![axis_plane_definition(candidate)],
                        uncertified_definition_fallback: false,
                    }))
                }
            },
        )
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].point, second);
    }

    #[test]
    fn probe_point_build_collection_marks_existing_probes_uncertain_after_later_unknown() {
        let mut probes = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_probe_point_builds_backtracking_unknown(
            &mut probes,
            [first.clone(), second.clone()].iter(),
            |candidate| {
                if *candidate == second {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(ProbePoint {
                        point: candidate.clone(),
                        side: Classification::Positive,
                        planes: vec![axis_plane_definition(candidate)],
                        uncertified_definition_fallback: false,
                    }))
                }
            },
        )
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert!(probes[0].uncertified_definition_fallback);
    }

    #[test]
    fn probe_point_build_collection_marks_later_probes_uncertain_after_uncertain_candidate_result()
    {
        let mut probes = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_probe_point_builds_backtracking_unknown(
            &mut probes,
            [first.clone(), second.clone()].iter(),
            |candidate| {
                Ok(Some(ProbePoint {
                    point: candidate.clone(),
                    side: Classification::Positive,
                    planes: vec![axis_plane_definition(candidate)],
                    uncertified_definition_fallback: *candidate == first,
                }))
            },
        )
        .unwrap();

        assert_eq!(probes.len(), 2);
        assert!(
            probes
                .iter()
                .all(|probe| probe.uncertified_definition_fallback)
        );
    }

    #[test]
    fn probe_point_build_collection_reports_unknown_if_all_candidates_are_uncertified() {
        let mut probes = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        let err = extend_probe_point_builds_backtracking_unknown(
            &mut probes,
            [first, second].iter(),
            |_candidate| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn adjacent_axis_probe_uses_corridor_witness_and_retains_definition() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };

        let probe = adjacent_axis_probes(&interior, &leaf.support, &bounds, &[], 0, true)
            .unwrap()
            .into_iter()
            .find(|probe| probe.side == Classification::Positive)
            .expect("axis corridor should contain a certified probe witness");

        assert_eq!(probe.side, Classification::Positive);
        assert!(!probe.planes.is_empty());
        for definition in &probe.planes {
            assert_eq!(affine_from_planes(definition).unwrap(), probe.point);
        }
        assert!(compare_real(&probe.point.x, &r(1)).unwrap().is_gt());
        assert!(compare_real(&probe.point.x, &r(4)).unwrap().is_lt());
        assert_eq!(probe.point.y, r(1));
        assert_eq!(probe.point.z, r(1));
    }

    #[test]
    fn adjacent_axis_probe_stop_values_backtrack_after_uncertified_crossing() {
        let interior = p(1, 1, 1);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let (stop_values, saw_unknown) = adjacent_axis_probe_stop_values_with_queries(
            &interior,
            &bounds,
            &[first, second],
            0,
            true,
            &mut |_interior, _endpoint, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(2) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(p(3, 1, 1)))
                }
            },
            &mut |_crossing, _polygon| Ok(PolygonPointLocation::Interior),
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(stop_values, vec![r(3), r(4)]);
    }

    #[test]
    fn adjacent_axis_probe_accepts_later_corridor_after_uncertified_crossing() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let probes = adjacent_axis_probes_with_queries(
            &interior,
            &support,
            &bounds,
            &[first, second],
            0,
            true,
            |_interior, _endpoint, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(2) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(p(3, 1, 1)))
                }
            },
            |_crossing, _polygon| Ok(PolygonPointLocation::Interior),
            |corridor| {
                if corridor.max.x == r(3) {
                    Ok(vec![ProbePoint {
                        point: p(2, 1, 1),
                        side: Classification::Positive,
                        planes: vec![axis_plane_definition(&p(2, 1, 1))],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].point, p(2, 1, 1));
        assert!(probes[0].uncertified_definition_fallback);
    }

    #[test]
    fn adjacent_axis_probe_reports_unknown_when_corridor_family_is_partially_uncertified_and_later_corridors_fail()
     {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let err = adjacent_axis_probes_with_queries(
            &interior,
            &support,
            &bounds,
            &[first, second],
            0,
            true,
            |_interior, _endpoint, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(2) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(p(3, 1, 1)))
                }
            },
            |_crossing, _polygon| Ok(PolygonPointLocation::Interior),
            |_corridor| Ok(Vec::new()),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn adjacent_axis_probe_stop_values_treat_boundary_crossing_as_unknown_and_keep_later_corridor()
    {
        let interior = p(1, 1, 1);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let (stop_values, saw_unknown) = adjacent_axis_probe_stop_values_with_queries(
            &interior,
            &bounds,
            &[first, second],
            0,
            true,
            &mut |_interior, _endpoint, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(2) {
                    Ok(Some(p(2, 1, 1)))
                } else {
                    Ok(Some(p(3, 1, 1)))
                }
            },
            &mut |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(2) {
                    Ok(PolygonPointLocation::Boundary)
                } else {
                    Ok(PolygonPointLocation::Interior)
                }
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(stop_values, vec![r(3), r(4)]);
    }

    #[test]
    fn adjacent_axis_probe_accepts_later_corridor_after_boundary_crossing() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let probes = adjacent_axis_probes_with_queries(
            &interior,
            &support,
            &bounds,
            &[first, second],
            0,
            true,
            |_interior, _endpoint, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(2) {
                    Ok(Some(p(2, 1, 1)))
                } else {
                    Ok(Some(p(3, 1, 1)))
                }
            },
            |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(2) {
                    Ok(PolygonPointLocation::Boundary)
                } else {
                    Ok(PolygonPointLocation::Interior)
                }
            },
            |corridor| {
                if corridor.max.x == r(3) {
                    Ok(vec![ProbePoint {
                        point: p(2, 1, 1),
                        side: Classification::Positive,
                        planes: vec![axis_plane_definition(&p(2, 1, 1))],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].point, p(2, 1, 1));
        assert!(probes[0].uncertified_definition_fallback);
    }

    #[test]
    fn collect_axis_probe_targets_backtracks_after_uncertified_definition() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let unrestricted_probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: false,
        };

        let probes = collect_axis_probe_targets(&[definition], |candidate| match candidate {
            Some(_) => Err(HypermeshError::UnknownClassification),
            None => Ok(vec![unrestricted_probe.clone()]),
        })
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].point, unrestricted_probe.point);
    }

    #[test]
    fn collect_axis_probe_targets_report_unknown_if_all_families_are_uncertified() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];

        let err = collect_axis_probe_targets(&[definition], |_candidate| {
            Err(HypermeshError::UnknownClassification)
        })
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn collect_axis_probe_targets_skips_duplicate_definition_families() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let mut definition_calls = 0;
        let mut unrestricted_calls = 0;

        let probes =
            collect_axis_probe_targets(&[definition.clone(), definition.clone()], |candidate| {
                match candidate {
                    Some(found_definition) => {
                        definition_calls += 1;
                        assert_eq!(found_definition, &definition);
                        Ok(vec![ProbePoint {
                            point: p(1, 0, 0),
                            side: Classification::Positive,
                            planes: vec![definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
                    }
                    None => {
                        unrestricted_calls += 1;
                        Ok(Vec::new())
                    }
                }
            })
            .unwrap();

        assert_eq!(definition_calls, 1);
        assert_eq!(unrestricted_calls, 1);
        assert_eq!(probes.len(), 1);
    }

    #[test]
    fn collect_axis_probe_targets_keeps_unrestricted_family_after_definition_hits() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let constrained_probe = ProbePoint {
            point: p(1, 1, 1),
            side: Classification::Positive,
            planes: vec![definition.clone()],
            uncertified_definition_fallback: false,
        };
        let unrestricted_probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: false,
        };

        let probes = collect_axis_probe_targets(&[definition], |candidate| match candidate {
            Some(_) => Ok(vec![constrained_probe.clone()]),
            None => Ok(vec![unrestricted_probe.clone()]),
        })
        .unwrap();

        assert_eq!(probes.len(), 2);
        assert!(
            probes
                .iter()
                .any(|probe| probe.point == constrained_probe.point)
        );
        assert!(
            probes
                .iter()
                .any(|probe| probe.point == unrestricted_probe.point)
        );
    }

    #[test]
    fn adjacent_axis_probe_preserves_retained_definition_when_axis_direction_allows() {
        let support = Plane::axis_aligned(2, r(0));
        let bounds = Aabb::new(p(0, 0, 0), p(2, 2, 2));
        let retained = [
            support.clone(),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![retained.clone()],
            uncertified_definition_fallback: false,
        };

        let probe = adjacent_axis_probes(&interior, &support, &bounds, &[], 2, true)
            .unwrap()
            .into_iter()
            .find(|probe| {
                probe.side == Classification::Positive
                    && probe
                        .planes
                        .iter()
                        .any(|planes| planes[1] == retained[1] && planes[2] == retained[2])
            })
            .expect("axis-direction probe should preserve retained axis-stable planes");

        assert_eq!(probe.point.x, r(1));
        assert_eq!(probe.point.y, r(1));
        assert!(compare_real(&probe.point.z, &r(0)).unwrap().is_gt());
    }

    #[test]
    fn leaf_classification_uses_certified_slanted_normal_probe() {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let ref_definitions = [axis_plane_definition(&p(0, 0, 0))];

        let winding = classify_leaf_polygon(
            &leaf.support,
            &leaf.edges,
            &p(0, 0, 0),
            &ref_definitions,
            &[0],
            &[leaf.clone()],
            &bounds,
            &leaf.delta_w,
        )
        .unwrap();

        assert_eq!(winding, vec![-1]);
    }

    #[test]
    fn strict_leaf_cell_points_retain_replayable_planes() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let center = p(1, 1, 1);

        let interior = strict_leaf_cell_points(&leaf, &center)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("strict leaf halfspaces should have a feasible witness");

        assert!(point_strictly_inside_leaf(&interior.point, &leaf).unwrap());
        assert!(!interior.planes.is_empty());
        let planes = &interior.planes[0];
        assert_eq!(affine_from_planes(planes).unwrap(), interior.point);
        assert_eq!(planes[0], leaf.support);
    }

    #[test]
    fn strict_leaf_cell_points_include_shifted_leaf_vertices() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let center = p(1, 1, 1);
        let vertices = leaf.vertices().unwrap();
        let bounds = leaf_bounds(&vertices).unwrap();
        let half = (Real::one() / Real::from(2)).unwrap();
        let mut halfspaces = vec![
            limit_plane_from_plane(&leaf.support),
            limit_plane_from_plane(&leaf.support.inverted()),
        ];
        for edge in &leaf.edges {
            let margin = edge.expression_at_point(&center);
            halfspaces.push(limit_plane_from_plane(&inward_shifted_edge_plane(
                edge, &margin, &half,
            )));
        }

        let report = halfspace_feasibility_report(&halfspaces).unwrap();
        let report_witness = report.witness.clone();
        let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();
        let mut direct_points = Vec::new();
        for seed in seeds {
            let active_planes = if report_witness.as_ref().is_some_and(|point| point == &seed) {
                report.active_planes
            } else {
                [None, None, None]
            };
            if let Some(point) =
                build_strict_leaf_point(&leaf, &seed, &halfspaces, active_planes, false).unwrap()
            {
                direct_points.push(point.point);
            }
        }

        let interiors = strict_leaf_cell_points(&leaf, &center).unwrap();
        let shifted = interiors
            .iter()
            .find(|point| !direct_points.iter().any(|direct| direct == &point.point))
            .expect("shifted strict leaf witness family should extend direct seed points");

        assert!(!shifted.planes.is_empty());
    }

    #[test]
    fn strict_leaf_cell_points_return_only_strict_points() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let center = p(1, 1, 1);

        let interiors = strict_leaf_cell_points(&leaf, &center).unwrap();

        assert!(!interiors.is_empty());
        for interior in &interiors {
            assert!(point_strictly_inside_leaf(&interior.point, &leaf).unwrap());
        }
    }

    #[test]
    fn strict_leaf_witness_points_include_shifted_leaf_vertices() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let vertices = leaf.vertices().unwrap();

        let interiors = strict_leaf_witness_points(&leaf, &vertices).unwrap();

        assert!(
            interiors
                .iter()
                .any(|point| point.point == Point3::new(q(1, 2), q(1, 2), r(2)))
        );
        assert!(interiors.iter().all(|point| !point.planes.is_empty()));
    }

    #[test]
    fn strict_leaf_witness_points_extend_direct_family_with_stricter_leaf_cells() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let vertices = leaf.vertices().unwrap();
        let bounds = leaf_bounds(&vertices).unwrap();
        let halfspaces = leaf_halfspaces(&leaf);
        let report = halfspace_feasibility_report(&halfspaces).unwrap();
        let report_witness = report.witness.clone();
        let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

        let mut direct_points = Vec::new();
        for seed in &seeds {
            let active_planes = if report_witness.as_ref().is_some_and(|point| point == seed) {
                report.active_planes
            } else {
                [None, None, None]
            };
            if let Some(point) =
                build_strict_leaf_point(&leaf, seed, &halfspaces, active_planes, false).unwrap()
            {
                direct_points.push(point.point);
            }
        }

        let mut stricter_points = Vec::new();
        for point in &direct_points {
            for stricter in strict_leaf_cell_points(&leaf, point).unwrap() {
                if !direct_points.iter().any(|direct| direct == &stricter.point) {
                    stricter_points.push(stricter.point);
                }
            }
        }

        let interiors = strict_leaf_witness_points(&leaf, &vertices).unwrap();

        assert!(!stricter_points.is_empty());
        assert!(
            stricter_points
                .iter()
                .any(|point| interiors.iter().any(|interior| &interior.point == point))
        );
    }

    #[test]
    fn interior_leaf_point_collection_backtracks_after_uncertified_candidate() {
        let mut points = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_interior_leaf_points_backtracking_unknown(
            &mut points,
            [first.clone(), second.clone()].iter(),
            |candidate| {
                if *candidate == first {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(vec![InteriorLeafPoint {
                        point: candidate.clone(),
                        planes: vec![axis_plane_definition(candidate)],
                        uncertified_definition_fallback: false,
                    }])
                }
            },
        )
        .unwrap();

        assert_eq!(points.len(), 1);
        assert_eq!(points[0].point, second);
    }

    #[test]
    fn interior_leaf_point_collection_marks_existing_points_uncertain_after_later_unknown() {
        let mut points = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_interior_leaf_points_backtracking_unknown(
            &mut points,
            [first.clone(), second.clone()].iter(),
            |candidate| {
                if *candidate == second {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(vec![InteriorLeafPoint {
                        point: candidate.clone(),
                        planes: vec![axis_plane_definition(candidate)],
                        uncertified_definition_fallback: false,
                    }])
                }
            },
        )
        .unwrap();

        assert_eq!(points.len(), 1);
        assert!(points[0].uncertified_definition_fallback);
    }

    #[test]
    fn interior_leaf_point_collection_marks_later_points_uncertain_after_uncertain_candidate_result()
     {
        let mut points = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_interior_leaf_points_backtracking_unknown(
            &mut points,
            [first.clone(), second.clone()].iter(),
            |candidate| {
                Ok(vec![InteriorLeafPoint {
                    point: candidate.clone(),
                    planes: vec![axis_plane_definition(candidate)],
                    uncertified_definition_fallback: *candidate == first,
                }])
            },
        )
        .unwrap();

        assert_eq!(points.len(), 2);
        assert!(
            points
                .iter()
                .all(|point| point.uncertified_definition_fallback)
        );
    }

    #[test]
    fn interior_leaf_point_collection_reports_unknown_if_all_candidates_are_uncertified() {
        let mut points = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        let err = extend_interior_leaf_points_backtracking_unknown(
            &mut points,
            [first, second].iter(),
            |_candidate| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn leaf_point_build_collection_backtracks_after_uncertified_candidate() {
        let mut points = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_leaf_point_builds_backtracking_unknown(
            &mut points,
            [first.clone(), second.clone()].iter(),
            |candidate| {
                if *candidate == first {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(InteriorLeafPoint {
                        point: candidate.clone(),
                        planes: vec![axis_plane_definition(candidate)],
                        uncertified_definition_fallback: false,
                    }))
                }
            },
        )
        .unwrap();

        assert_eq!(points.len(), 1);
        assert_eq!(points[0].point, second);
    }

    #[test]
    fn leaf_point_build_collection_marks_existing_points_uncertain_after_later_unknown() {
        let mut points = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_leaf_point_builds_backtracking_unknown(
            &mut points,
            [first.clone(), second.clone()].iter(),
            |candidate| {
                if *candidate == second {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(InteriorLeafPoint {
                        point: candidate.clone(),
                        planes: vec![axis_plane_definition(candidate)],
                        uncertified_definition_fallback: false,
                    }))
                }
            },
        )
        .unwrap();

        assert_eq!(points.len(), 1);
        assert!(points[0].uncertified_definition_fallback);
    }

    #[test]
    fn leaf_point_build_collection_marks_later_points_uncertain_after_uncertain_candidate_result() {
        let mut points = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        extend_leaf_point_builds_backtracking_unknown(
            &mut points,
            [first.clone(), second.clone()].iter(),
            |candidate| {
                Ok(Some(InteriorLeafPoint {
                    point: candidate.clone(),
                    planes: vec![axis_plane_definition(candidate)],
                    uncertified_definition_fallback: *candidate == first,
                }))
            },
        )
        .unwrap();

        assert_eq!(points.len(), 2);
        assert!(
            points
                .iter()
                .all(|point| point.uncertified_definition_fallback)
        );
    }

    #[test]
    fn leaf_point_build_collection_reports_unknown_if_all_candidates_are_uncertified() {
        let mut points = Vec::new();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        let err = extend_leaf_point_builds_backtracking_unknown(
            &mut points,
            [first, second].iter(),
            |_candidate| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn certified_leaf_test_point_prefers_replayable_interior_witness() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let expected_points = interior_leaf_points(&leaf)
            .unwrap()
            .into_iter()
            .filter(|point| !point.planes.is_empty())
            .map(|point| point.point)
            .collect::<Vec<_>>();

        let point = certified_leaf_test_point(&leaf.support, &leaf.edges)
            .unwrap()
            .expect("triangle leaf should have a certified strict interior point")
            .to_affine_point()
            .unwrap();

        assert!(!expected_points.is_empty());
        assert!(expected_points.iter().any(|expected| expected == &point));
    }

    #[test]
    fn interior_leaf_points_drop_naked_centroid_when_replayable_points_exist() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);

        let points = interior_leaf_points(&leaf).unwrap();

        assert!(!points.is_empty());
        assert!(points.iter().all(|point| !point.planes.is_empty()));
    }

    #[test]
    fn leaf_interior_definitions_include_non_basis_active_halfspaces() {
        let witness = p(1, 1, 1);
        let support = Plane::axis_aligned(2, r(1));
        let halfspaces = vec![
            limit_plane_from_plane(&support),
            limit_plane_from_plane(&support.inverted()),
            LimitPlane3::new(p(1, 0, 0), r(-1)),
            LimitPlane3::new(p(0, 1, 0), r(-1)),
            LimitPlane3::new(p(1, 1, 1), r(-3)),
        ];

        let definitions = leaf_interior_definitions_from_active_halfspaces(
            &witness,
            &support,
            &halfspaces,
            [Some(0), Some(2), Some(3)],
        )
        .unwrap();

        assert!(definitions.definitions.iter().any(|definition| {
            definition[1..]
                .iter()
                .any(|plane| plane.normal == p(1, 1, 1))
        }));
        for definition in &definitions.definitions {
            assert_eq!(definition[0], support);
            assert_eq!(affine_from_planes(definition).unwrap(), witness);
        }
    }

    #[test]
    fn strict_leaf_witness_retains_axis_definition_when_active_replay_fails() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let witness = p(1, 1, 1);
        let halfspaces = vec![limit_plane_from_plane(&leaf.support)];

        let point =
            build_strict_leaf_point(&leaf, &witness, &halfspaces, [Some(9), None, None], false)
                .unwrap()
                .expect("strict witness should still be retained");

        assert_eq!(point.point, witness);
        assert!(!point.uncertified_definition_fallback);
        assert!(point.planes.iter().any(|definition| {
            definition[0] == leaf.support
                && definition[1..]
                    .iter()
                    .filter(|plane| {
                        plane.normal == p(1, 0, 0)
                            || plane.normal == p(0, 1, 0)
                            || plane.normal == p(0, 0, 1)
                    })
                    .count()
                    == 2
        }));
    }

    #[test]
    fn strict_leaf_witness_preserves_inherited_uncertified_definition_fallback() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let witness = p(1, 1, 1);
        let halfspaces = vec![limit_plane_from_plane(&leaf.support)];

        let point = build_strict_leaf_point(&leaf, &witness, &halfspaces, [None, None, None], true)
            .unwrap()
            .expect("strict witness should still be retained");

        assert!(point.uncertified_definition_fallback);
    }

    #[test]
    fn strict_leaf_witness_points_mark_surviving_points_uncertain_after_seed_family_unknown() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let vertices = leaf.vertices().unwrap();

        let points = strict_leaf_witness_points_with_seed_families(
            &leaf,
            &vertices,
            |_leaf, _vertices, _bounds, _halfspaces, _report| {
                Ok(LeafWitnessSeedFamilies {
                    seeds: vec![p(1, 1, 1)],
                    shifted_vertices: Vec::new(),
                    shifted_geometry_seeds: Vec::new(),
                    saw_unknown: true,
                })
            },
        )
        .unwrap();

        assert!(points.iter().any(|point| point.point == p(1, 1, 1)));
        assert!(
            points
                .iter()
                .all(|point| point.uncertified_definition_fallback)
        );
    }

    #[test]
    fn leaf_witness_seed_family_gate_allows_shifted_seed_sources_after_unknown_direct_family() {
        assert!(!seed_family_search_failed_without_any_seed(
            &[],
            &[p(1, 1, 1)],
            &[],
            true,
        ));
        assert!(!seed_family_search_failed_without_any_seed(
            &[],
            &[],
            &[p(1, 1, 1)],
            true,
        ));
    }

    #[test]
    fn strict_leaf_witness_from_shifted_witness_merges_definition_families() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let witness = ShiftedHalfspaceWitness {
            point: p(1, 1, 1),
            families: vec![
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(0, false, r(1))],
                    active_planes: [Some(0), None, None],
                },
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(1, false, r(1))],
                    active_planes: [Some(0), None, None],
                },
            ],
            uncertified_definition_fallback: false,
        };

        let point = build_strict_leaf_point_from_shifted_witness(&leaf, &witness)
            .unwrap()
            .expect("shifted witness should still certify a strict leaf point");

        assert!(point.planes.iter().any(|definition| {
            definition
                .iter()
                .any(|plane| plane.normal == p(1, 0, 0) && plane.offset == r(-1))
        }));
        assert!(point.planes.iter().any(|definition| {
            definition
                .iter()
                .any(|plane| plane.normal == p(0, 1, 0) && plane.offset == r(-1))
        }));
    }

    #[test]
    fn strict_leaf_witness_salvages_coincident_halfspaces_after_invalid_active_index() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let witness = p(1, 1, 1);
        let halfspaces = vec![
            limit_plane_from_plane(&leaf.support),
            axis_halfspace(0, false, r(1)),
        ];

        let point =
            build_strict_leaf_point(&leaf, &witness, &halfspaces, [Some(9), None, None], false)
                .unwrap()
                .expect("strict witness should still be retained");

        assert_eq!(point.point, witness);
        assert!(point.uncertified_definition_fallback);
        assert!(point.planes.iter().any(|definition| {
            definition[1..]
                .iter()
                .any(|plane| plane.normal == p(1, 0, 0) && plane.offset == r(-1))
        }));
    }

    #[test]
    fn witness_active_planes_return_report_planes_only_for_matching_witness() {
        let report_witness = p(1, 2, 3);
        let active_planes = [Some(4), Some(5), None];

        assert_eq!(
            witness_active_planes(Some(&report_witness), active_planes, &report_witness),
            active_planes
        );
        assert_eq!(
            witness_active_planes(Some(&report_witness), active_planes, &p(9, 9, 9)),
            [None, None, None]
        );
    }

    #[test]
    fn probe_definitions_include_non_basis_active_halfspaces() {
        let witness = p(1, 1, 1);
        let shifted_support = Plane::axis_aligned(2, r(1));
        let halfspaces = vec![
            LimitPlane3::new(p(1, 0, 0), r(-1)),
            LimitPlane3::new(p(0, 1, 0), r(-1)),
            LimitPlane3::new(p(1, 1, 1), r(-3)),
        ];

        let definitions = probe_definitions_from_active_halfspaces(
            &witness,
            &halfspaces,
            [Some(0), Some(1), None],
            &[shifted_support],
        )
        .unwrap();

        assert!(definitions.saw_unknown);
        assert!(
            definitions
                .definitions
                .iter()
                .any(|definition| definition.iter().any(|plane| plane.normal == p(1, 1, 1)))
        );
        for definition in &definitions.definitions {
            assert_eq!(affine_from_planes(definition).unwrap(), witness);
        }
    }

    #[test]
    fn probe_definitions_or_axis_falls_back_to_axis_definition() {
        let witness = p(1, 2, 3);

        let (definitions, used_fallback) =
            probe_definitions_or_axis(&witness, Err(HypermeshError::UnknownClassification))
                .unwrap();

        assert_eq!(definitions, vec![axis_plane_definition(&witness)]);
        assert!(used_fallback);
    }

    #[test]
    fn strict_probe_witness_retains_axis_definition_when_active_replay_fails() {
        let support = Plane::axis_aligned(2, r(0));
        let witness = p(1, 1, 1);
        let halfspaces = vec![axis_halfspace(2, false, r(1))];

        let probe = build_probe_point(
            &witness,
            &support,
            &halfspaces,
            [Some(9), None, None],
            &[],
            false,
        )
        .unwrap()
        .expect("strict probe witness should still be retained");

        assert_eq!(probe.point, witness);
        assert!(probe.uncertified_definition_fallback);
        assert!(probe.planes.iter().any(|definition| {
            definition_planes_match_as_sets(definition, &axis_plane_definition(&probe.point))
        }));
    }

    #[test]
    fn strict_probe_witness_preserves_inherited_uncertified_definition_fallback() {
        let support = Plane::axis_aligned(2, r(0));
        let witness = p(1, 1, 1);
        let halfspaces = vec![axis_halfspace(2, false, r(1))];

        let probe = build_probe_point(
            &witness,
            &support,
            &halfspaces,
            [None, None, None],
            &[],
            true,
        )
        .unwrap()
        .expect("strict probe witness should still be retained");

        assert!(probe.uncertified_definition_fallback);
    }

    #[test]
    fn strict_probe_witness_from_shifted_witness_merges_definition_families() {
        let support = Plane::axis_aligned(2, r(0));
        let witness = ShiftedHalfspaceWitness {
            point: p(1, 1, 1),
            families: vec![
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(0, false, r(1))],
                    active_planes: [Some(0), None, None],
                },
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(1, false, r(1))],
                    active_planes: [Some(0), None, None],
                },
            ],
            uncertified_definition_fallback: false,
        };

        let probe = build_probe_point_from_shifted_witness(&witness, &support, &[])
            .unwrap()
            .expect("shifted witness should still certify a strict probe");

        assert!(probe.planes.iter().any(|definition| {
            definition
                .iter()
                .any(|plane| plane.normal == p(1, 0, 0) && plane.offset == r(-1))
        }));
        assert!(probe.planes.iter().any(|definition| {
            definition
                .iter()
                .any(|plane| plane.normal == p(0, 1, 0) && plane.offset == r(-1))
        }));
    }

    #[test]
    fn strict_probe_witness_salvages_coincident_halfspaces_after_invalid_active_index() {
        let support = Plane::axis_aligned(2, r(0));
        let witness = p(1, 1, 1);
        let halfspaces = vec![
            axis_halfspace(2, false, r(1)),
            LimitPlane3::new(p(1, 1, 1), r(-3)),
        ];

        let probe = build_probe_point(
            &witness,
            &support,
            &halfspaces,
            [Some(9), None, None],
            &[],
            false,
        )
        .unwrap()
        .expect("strict probe witness should still be retained");

        assert_eq!(probe.point, witness);
        assert!(probe.uncertified_definition_fallback);
        assert!(probe.planes.iter().any(|definition| {
            definition
                .iter()
                .any(|plane| plane.normal == p(1, 1, 1) && plane.offset == r(-3))
        }));
    }

    #[test]
    fn strict_axis_probe_witness_retains_axis_definition_when_active_replay_fails() {
        let support = Plane::axis_aligned(2, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![axis_plane_definition(&p(1, 1, 0))],
            uncertified_definition_fallback: false,
        };
        let witness = p(2, 1, 1);
        let halfspaces = vec![axis_halfspace(0, false, r(2))];

        let probe = build_axis_probe_point(
            &witness,
            &interior,
            &support,
            0,
            None,
            &halfspaces,
            [Some(9), None, None],
            false,
        )
        .unwrap()
        .expect("strict axis probe witness should still be retained");

        assert_eq!(probe.point, witness);
        assert!(probe.uncertified_definition_fallback);
        assert!(probe.planes.iter().any(|definition| {
            definition_planes_match_as_sets(definition, &axis_plane_definition(&probe.point))
        }));
    }

    #[test]
    fn strict_axis_probe_witness_preserves_inherited_uncertified_definition_fallback() {
        let support = Plane::axis_aligned(2, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![axis_plane_definition(&p(1, 1, 0))],
            uncertified_definition_fallback: false,
        };
        let witness = p(2, 1, 1);
        let halfspaces = vec![axis_halfspace(0, false, r(2))];

        let probe = build_axis_probe_point(
            &witness,
            &interior,
            &support,
            0,
            None,
            &halfspaces,
            [None, None, None],
            true,
        )
        .unwrap()
        .expect("strict axis probe witness should still be retained");

        assert!(probe.uncertified_definition_fallback);
    }

    #[test]
    fn duplicate_probe_points_merge_plane_definitions() {
        let point = p(1, 1, 1);
        let first_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let second_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(2, r(1)),
        ];
        let mut probes = vec![ProbePoint {
            point: point.clone(),
            side: Classification::Positive,
            planes: vec![first_definition.clone()],
            uncertified_definition_fallback: false,
        }];

        push_unique_probe_point(
            &mut probes,
            ProbePoint {
                point,
                side: Classification::Positive,
                planes: vec![second_definition.clone()],
                uncertified_definition_fallback: false,
            },
        );
        push_unique_probe_point(
            &mut probes,
            ProbePoint {
                point: p(1, 1, 1),
                side: Classification::Positive,
                planes: vec![second_definition],
                uncertified_definition_fallback: false,
            },
        );

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].planes.len(), 2);
    }

    #[test]
    fn duplicate_probe_points_merge_permuted_plane_definitions() {
        let point = p(1, 1, 1);
        let definition = axis_plane_definition(&point);
        let permuted = [
            definition[1].clone(),
            definition[2].clone(),
            definition[0].clone(),
        ];
        let mut probes = vec![ProbePoint {
            point: point.clone(),
            side: Classification::Positive,
            planes: vec![definition],
            uncertified_definition_fallback: false,
        }];

        push_unique_probe_point(
            &mut probes,
            ProbePoint {
                point,
                side: Classification::Positive,
                planes: vec![permuted.clone()],
                uncertified_definition_fallback: false,
            },
        );

        assert_eq!(probes.len(), 1);
        assert_eq!(probes[0].planes.len(), 1);
        assert!(definition_planes_match_as_sets(
            &probes[0].planes[0],
            &permuted
        ));
    }

    #[test]
    fn duplicate_interior_points_merge_plane_definitions() {
        let point = p(1, 1, 1);
        let mut points = vec![InteriorLeafPoint {
            point: point.clone(),
            planes: Vec::new(),
            uncertified_definition_fallback: false,
        }];
        let first_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let second_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(2, r(1)),
        ];

        push_unique_interior_point(
            &mut points,
            InteriorLeafPoint {
                point: point.clone(),
                planes: vec![first_definition.clone()],
                uncertified_definition_fallback: false,
            },
        );
        push_unique_interior_point(
            &mut points,
            InteriorLeafPoint {
                point,
                planes: vec![second_definition.clone()],
                uncertified_definition_fallback: false,
            },
        );

        assert_eq!(points.len(), 1);
        assert_eq!(points[0].planes, vec![first_definition, second_definition]);
    }

    #[test]
    fn duplicate_interior_points_merge_permuted_plane_definitions() {
        let point = p(1, 1, 1);
        let definition = axis_plane_definition(&point);
        let permuted = [
            definition[1].clone(),
            definition[2].clone(),
            definition[0].clone(),
        ];
        let mut points = vec![InteriorLeafPoint {
            point: point.clone(),
            planes: vec![definition],
            uncertified_definition_fallback: false,
        }];

        push_unique_interior_point(
            &mut points,
            InteriorLeafPoint {
                point,
                planes: vec![permuted.clone()],
                uncertified_definition_fallback: false,
            },
        );

        assert_eq!(points.len(), 1);
        assert_eq!(points[0].planes.len(), 1);
        assert!(definition_planes_match_as_sets(
            &points[0].planes[0],
            &permuted
        ));
    }

    #[test]
    fn plane_replacement_path_traces_certified_winding_steps() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let start = axis_plane_defined_point(&p(0, 0, 0));
        let end = axis_plane_defined_point(&p(2, 0, 0));

        let winding =
            trace_plane_replacement_path(&start.planes, &end.planes, &[0], &[wall]).unwrap();

        assert_eq!(winding, vec![-1]);
    }

    #[test]
    fn retained_reference_definitions_try_later_plane_replacement_paths() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let invalid_start = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(0, r(2)),
        ];
        let valid_start = axis_plane_defined_point(&p(0, 0, 0));
        let end = axis_plane_defined_point(&p(2, 0, 0));

        let winding = trace_probe_from_reference_definitions(
            &p(0, 0, 0),
            &[invalid_start, valid_start.planes],
            &p(2, 0, 0),
            std::slice::from_ref(&end.planes),
            &[0],
            &[wall],
        )
        .unwrap();

        assert_eq!(winding, vec![-1]);
    }

    #[test]
    fn retained_probe_definitions_try_later_plane_replacement_paths() {
        let ref_point = p(0, 0, 0);
        let ref_definitions = [[
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::from_coefficients(r(1), r(1), r(1), r(0)),
        ]];
        let invalid_probe_definition = [
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(0, r(0)),
        ];
        let probe = ProbePoint {
            point: p(2, 1, 0),
            side: Classification::Positive,
            planes: vec![invalid_probe_definition, axis_plane_definition(&p(2, 1, 0))],
            uncertified_definition_fallback: false,
        };
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];

        assert_eq!(
            trace_segment(&ref_point, &probe.point, &[0], &[wall.clone()]),
            Err(HypermeshError::UnknownClassification)
        );

        let winding =
            trace_probe_winding(&ref_point, &ref_definitions, &probe, &[0], &[wall]).unwrap();

        assert_eq!(winding, vec![0]);
    }

    #[test]
    fn retained_definition_segment_search_continues_after_uncertified_direct_family() {
        let ref_point = p(0, 0, 0);
        let ref_definitions = [[
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::from_coefficients(r(1), r(1), r(1), r(0)),
        ]];
        let invalid_probe_definition = [
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(0, r(0)),
        ];
        let probe_point = p(2, 1, 0);
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];

        assert_eq!(
            trace_segment(&ref_point, &probe_point, &[0], &[wall.clone()]),
            Err(HypermeshError::UnknownClassification)
        );

        let winding = trace_segment_from_definitions_with_step_detoured_plane_replacement(
            &ref_point,
            &probe_point,
            &[0],
            &[wall],
            &ref_definitions,
            &[
                invalid_probe_definition,
                axis_plane_definition(&probe_point),
            ],
        )
        .unwrap();

        assert_eq!(winding, vec![0]);
    }

    #[test]
    fn definition_pair_trace_backtracks_after_uncertified_pair() {
        let start_unknown = axis_plane_definition(&p(0, 0, 0));
        let start_ok = axis_plane_definition(&p(1, 0, 0));
        let end = axis_plane_definition(&p(2, 0, 0));

        let traced = definition_pair_trace_backtracking_unknown(
            &[start_unknown.clone(), start_ok.clone()],
            std::slice::from_ref(&end),
            |start_definition, end_definition| {
                if start_definition == &start_unknown && end_definition == &end {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(vec![1])
                }
            },
        )
        .unwrap();

        assert_eq!(traced, Some(vec![1]));
    }

    #[test]
    fn definition_pair_trace_reports_unknown_if_all_pairs_are_uncertified() {
        let start = axis_plane_definition(&p(0, 0, 0));
        let end = axis_plane_definition(&p(1, 0, 0));

        let err = definition_pair_trace_backtracking_unknown(
            std::slice::from_ref(&start),
            std::slice::from_ref(&end),
            |_start_definition, _end_definition| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn definition_pair_trace_search_skips_duplicate_definition_pairs() {
        let start_a = axis_plane_definition(&p(0, 0, 0));
        let start_b = axis_plane_definition(&p(1, 0, 0));
        let end = axis_plane_definition(&p(2, 0, 0));
        let mut trace_calls = 0;

        let traced = definition_pair_trace_backtracking_unknown(
            &[start_a.clone(), start_a.clone(), start_b.clone()],
            &[end.clone(), end.clone()],
            |start_definition, end_definition| {
                trace_calls += 1;
                if start_definition == &start_a && end_definition == &end {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(vec![1])
                }
            },
        )
        .unwrap();

        assert_eq!(traced, Some(vec![1]));
        assert_eq!(trace_calls, 2);
    }

    #[test]
    fn definition_pair_trace_search_skips_permuted_definition_pairs() {
        let start_a = axis_plane_definition(&p(0, 0, 0));
        let start_a_permuted = [start_a[1].clone(), start_a[2].clone(), start_a[0].clone()];
        let start_b = axis_plane_definition(&p(1, 0, 0));
        let end = axis_plane_definition(&p(2, 0, 0));
        let end_permuted = [end[2].clone(), end[0].clone(), end[1].clone()];
        let mut trace_calls = 0;

        let traced = definition_pair_trace_backtracking_unknown(
            &[start_a.clone(), start_a_permuted, start_b.clone()],
            &[end.clone(), end_permuted],
            |start_definition, end_definition| {
                trace_calls += 1;
                if definition_planes_match_as_sets(start_definition, &start_a)
                    && definition_planes_match_as_sets(end_definition, &end)
                {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(vec![1])
                }
            },
        )
        .unwrap();

        assert_eq!(traced, Some(vec![1]));
        assert_eq!(trace_calls, 2);
    }

    #[test]
    fn detour_legs_retry_direct_paths_when_axis_order_fails() {
        let blockers = vec![
            make_triangle(&p(1, 0, 0), &p(2, 0, 0), &p(1, 1, 0), 0, 0),
            make_triangle(&p(0, 1, 0), &p(1, 1, 0), &p(0, 2, 0), 0, 1),
            make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 0, 2),
        ];

        assert_eq!(
            trace_axis_ordered_paths(&p(0, 0, 0), &p(1, 1, 1), &[0], &blockers),
            Err(HypermeshError::UnknownClassification)
        );
        assert_eq!(
            trace_direct_segment(&p(0, 0, 0), &p(1, 1, 1), &[0], &blockers)
                .unwrap()
                .winding,
            vec![0]
        );

        let traced = trace_segment_via_detours_with_definitions_budget(
            &p(0, 0, 0),
            &p(2, 2, 2),
            &[0],
            &blockers,
            &[DetourTarget {
                point: p(1, 1, 1),
                definitions: vec![axis_plane_definition(&p(1, 1, 1))],
                uncertified_definition_fallback: false,
            }],
            &[axis_plane_definition(&p(0, 0, 0))],
            &[axis_plane_definition(&p(2, 2, 2))],
            1,
            &mut |start, end, winding, start_definitions, end_definitions| {
                trace_segment_with_definitions_no_detours(
                    start,
                    end,
                    winding,
                    &blockers,
                    start_definitions,
                    end_definitions,
                )
            },
            &mut |_start, _end| Ok(Vec::new()),
        )
        .unwrap();

        assert_eq!(traced, Some(vec![0]));
    }

    #[test]
    fn detour_legs_retry_plane_replacement_from_detour_definitions() {
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];
        let detour = DetourTarget {
            point: p(2, 1, 0),
            definitions: vec![[
                Plane::axis_aligned(0, r(2)),
                Plane::axis_aligned(1, r(1)),
                Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
            ]],
            uncertified_definition_fallback: false,
        };

        assert_eq!(
            trace_segment_without_detours(&p(0, 0, 0), &detour.point, &[0], &[wall.clone()])
                .unwrap_err(),
            HypermeshError::UnknownClassification
        );

        let traced = trace_segment_via_detours_with_definitions_budget(
            &p(0, 0, 0),
            &p(2, 2, 0),
            &[0],
            &[wall.clone()],
            &[detour],
            &[axis_plane_definition(&p(0, 0, 0))],
            &[axis_plane_definition(&p(2, 2, 0))],
            1,
            &mut |start, end, winding, start_definitions, end_definitions| {
                trace_segment_with_definitions_no_detours(
                    start,
                    end,
                    winding,
                    &[wall.clone()],
                    start_definitions,
                    end_definitions,
                )
            },
            &mut |_start, _end| Ok(Vec::new()),
        )
        .unwrap();

        assert_eq!(traced, Some(vec![0]));
    }

    #[test]
    fn detour_legs_can_use_retained_start_definitions() {
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];
        let start = p(0, 0, 0);
        let end = p(2, 2, 0);
        let start_definitions = [[
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::from_coefficients(r(1), r(1), r(1), r(0)),
        ]];
        let detour = DetourTarget {
            point: p(2, 1, 0),
            definitions: vec![axis_plane_definition(&p(2, 1, 0))],
            uncertified_definition_fallback: false,
        };

        let without_retained_start = trace_segment_via_detours_with_definitions_budget(
            &start,
            &end,
            &[0],
            &[wall.clone()],
            std::slice::from_ref(&detour),
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut |start, end, winding, start_definitions, end_definitions| {
                trace_segment_with_definitions_no_detours(
                    start,
                    end,
                    winding,
                    &[wall.clone()],
                    start_definitions,
                    end_definitions,
                )
            },
            &mut |_start, _end| Ok(Vec::new()),
        );
        assert_eq!(
            without_retained_start.unwrap_err(),
            HypermeshError::UnknownClassification
        );

        let with_retained_start = trace_segment_via_detours_with_definitions_budget(
            &start,
            &end,
            &[0],
            &[wall.clone()],
            &[detour],
            &start_definitions,
            &[axis_plane_definition(&end)],
            1,
            &mut |start, end, winding, start_definitions, end_definitions| {
                trace_segment_with_definitions_no_detours(
                    start,
                    end,
                    winding,
                    &[wall.clone()],
                    start_definitions,
                    end_definitions,
                )
            },
            &mut |_start, _end| Ok(Vec::new()),
        )
        .unwrap();

        assert_eq!(with_retained_start, Some(vec![0]));
    }

    #[test]
    fn detour_search_continues_after_uncertified_no_detour_family() {
        let start = p(0, 0, 0);
        let detour_point = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour = DetourTarget {
            point: detour_point.clone(),
            definitions: vec![axis_plane_definition(&detour_point)],
            uncertified_definition_fallback: false,
        };

        let traced = trace_segment_from_definitions_with_budget_impl(
            &start,
            &end,
            &[0],
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut |from, to, winding, _start_definitions, _end_definitions| {
                if *from == start && *to == end {
                    Err(HypermeshError::UnknownClassification)
                } else if (*from == start && *to == detour_point)
                    || (*from == detour_point && *to == end)
                {
                    Ok(Some(winding.to_vec()))
                } else {
                    Ok(None)
                }
            },
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![detour.clone()])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert_eq!(traced, vec![0]);
    }

    #[test]
    fn detour_search_reports_unknown_if_all_detours_are_uncertified() {
        let start = p(0, 0, 0);
        let detour_point = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour = DetourTarget {
            point: detour_point,
            definitions: vec![axis_plane_definition(&p(1, 0, 0))],
            uncertified_definition_fallback: false,
        };

        let err = trace_segment_via_detours_with_definitions_budget(
            &start,
            &end,
            &[0],
            &[],
            &[detour],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut |_from, _to, _winding, _start_definitions, _end_definitions| {
                Err(HypermeshError::UnknownClassification)
            },
            &mut |_from, _to| Ok(Vec::new()),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn detour_trace_reports_unknown_when_fallback_surface_detour_is_skipped() {
        let start = p(0, 0, 0);
        let detour = p(1, 0, 0);
        let end = p(2, 0, 0);
        let fallback_detour = DetourTarget {
            point: detour.clone(),
            definitions: vec![axis_plane_definition(&detour)],
            uncertified_definition_fallback: true,
        };
        let polygons = vec![ConvexPolygon {
            support: Plane::axis_aligned(0, r(1)),
            edges: Vec::new(),
            mesh_index: 0,
            polygon_index: 0,
            delta_w: Vec::new(),
            approx_bounds: None,
        }];

        let err = trace_segment_via_detours_with_definitions_budget(
            &start,
            &end,
            &[0],
            &polygons,
            &[fallback_detour],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut |_from, _to, _winding, _start_definitions, _end_definitions| Ok(None),
            &mut |_from, _to| Ok(Vec::new()),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn detour_trace_reports_unknown_when_fallback_revisited_detour_is_skipped() {
        let start = p(0, 0, 0);
        let end = p(2, 0, 0);
        let fallback_detour = DetourTarget {
            point: end.clone(),
            definitions: vec![axis_plane_definition(&end)],
            uncertified_definition_fallback: true,
        };

        let err = trace_segment_from_definitions_with_cycle_guard_impl(
            &start,
            &end,
            &[0],
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &[start.clone(), end.clone()],
            &mut |_from, _to, _winding, _start_definitions, _end_definitions| Ok(None),
            &mut |_from, _to| Ok(vec![fallback_detour.clone()]),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn detour_trace_cycle_guard_tries_later_detour_after_uncertified_surface_query() {
        let start = p(0, 0, 0);
        let first_detour = p(1, 0, 0);
        let second_detour = p(2, 0, 0);
        let end = p(3, 0, 0);
        let mut surface_cache = Vec::new();

        let winding = trace_segment_via_detours_with_cycle_guard_with_surface_query(
            &start,
            &end,
            &[0],
            &[],
            &[
                DetourTarget {
                    point: first_detour.clone(),
                    definitions: vec![axis_plane_definition(&first_detour)],
                    uncertified_definition_fallback: false,
                },
                DetourTarget {
                    point: second_detour.clone(),
                    definitions: vec![axis_plane_definition(&second_detour)],
                    uncertified_definition_fallback: false,
                },
            ],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &[start.clone(), end.clone()],
            &mut surface_cache,
            &mut |point| {
                if *point == first_detour {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(false)
                }
            },
            &mut |_from, _to, winding, _start_definitions, _end_definitions| {
                Ok(Some(winding.to_vec()))
            },
            &mut |_from, _to| Ok(Vec::new()),
        )
        .unwrap();

        assert_eq!(winding, Some(vec![0]));
    }

    #[test]
    fn detour_trace_cycle_guard_reports_unknown_when_surface_query_is_uncertified_and_later_detours_fail()
     {
        let start = p(0, 0, 0);
        let first_detour = p(1, 0, 0);
        let second_detour = p(2, 0, 0);
        let end = p(3, 0, 0);
        let mut surface_cache = Vec::new();

        let err = trace_segment_via_detours_with_cycle_guard_with_surface_query(
            &start,
            &end,
            &[0],
            &[],
            &[
                DetourTarget {
                    point: first_detour.clone(),
                    definitions: vec![axis_plane_definition(&first_detour)],
                    uncertified_definition_fallback: false,
                },
                DetourTarget {
                    point: second_detour.clone(),
                    definitions: vec![axis_plane_definition(&second_detour)],
                    uncertified_definition_fallback: false,
                },
            ],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &[start.clone(), end.clone()],
            &mut surface_cache,
            &mut |point| {
                if *point == first_detour {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(false)
                }
            },
            &mut |_from, _to, _winding, _start_definitions, _end_definitions| Ok(None),
            &mut |_from, _to| Ok(Vec::new()),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn axis_defined_probes_retry_plane_replacement_from_reference_definitions() {
        let ref_point = p(0, 0, 0);
        let ref_definitions = [[
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::from_coefficients(r(1), r(1), r(1), r(0)),
        ]];
        let probe = ProbePoint {
            point: p(2, 1, 0),
            side: Classification::Positive,
            planes: Vec::new(),
            uncertified_definition_fallback: false,
        };
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];

        assert_eq!(
            trace_segment(&ref_point, &probe.point, &[0], &[wall.clone()]),
            Err(HypermeshError::UnknownClassification)
        );

        let winding =
            trace_probe_winding(&ref_point, &ref_definitions, &probe, &[0], &[wall]).unwrap();

        assert_eq!(winding, vec![0]);
    }

    #[test]
    fn probe_reachability_retries_plane_replacement_from_retained_definitions() {
        let host_support = Plane::axis_aligned(2, r(0));
        let blocker = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let interior = InteriorLeafPoint {
            point: p(0, 0, 0),
            planes: vec![[
                Plane::axis_aligned(0, r(0)),
                Plane::axis_aligned(1, r(0)),
                Plane::from_coefficients(r(1), r(1), r(1), r(0)),
            ]],
            uncertified_definition_fallback: false,
        };
        let probe = ProbePoint {
            point: p(2, 1, 1),
            side: Classification::Positive,
            planes: vec![[
                Plane::axis_aligned(0, r(2)),
                Plane::axis_aligned(1, r(1)),
                Plane::from_coefficients(r(1), r(1), r(1), r(-4)),
            ]],
            uncertified_definition_fallback: false,
        };

        assert_eq!(
            probe_reaches_adjacent_cell(
                &interior.point,
                &probe.point,
                &host_support,
                std::slice::from_ref(&blocker),
            ),
            Err(HypermeshError::UnknownClassification)
        );
        assert!(probe_reaches_adjacent_cell_from_interior(
            &interior,
            &probe,
            &host_support,
            &[blocker],
        )
        .unwrap());
    }

    #[test]
    fn probe_reaches_adjacent_cell_reports_unknown_for_boundary_crossing() {
        let host_support = Plane::axis_aligned(2, r(0));
        let blocker = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);

        assert_eq!(
            probe_reaches_adjacent_cell(&p(0, 0, 0), &p(2, 0, 0), &host_support, &[blocker]),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn probe_reachability_definition_search_continues_after_uncertified_direct_check() {
        let start = p(0, 0, 0);
        let end = p(1, 1, 1);

        assert!(
            probe_reaches_adjacent_cell_with_definition_search(
                &start,
                &end,
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                || Err(HypermeshError::UnknownClassification),
                |_start_definition, _end_definition| Ok(true),
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_definition_search_continues_after_boundary_direct_check() {
        let host_support = Plane::axis_aligned(2, r(0));
        let blocker = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let start = p(0, 0, 0);
        let end = p(2, 0, 0);

        assert!(
            probe_reaches_adjacent_cell_with_definition_search(
                &start,
                &end,
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                || probe_reaches_adjacent_cell(&start, &end, &host_support, &[blocker.clone()]),
                |_start_definition, _end_definition| Ok(true),
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_definition_search_reports_unknown_when_direct_check_is_uncertified_and_replacements_fail()
     {
        let start = p(0, 0, 0);
        let end = p(1, 1, 1);

        let err = probe_reaches_adjacent_cell_with_definition_search(
            &start,
            &end,
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            || Err(HypermeshError::UnknownClassification),
            |_start_definition, _end_definition| Ok(false),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn probe_step_detour_helper_retries_lower_definition_trace() {
        let host_support = Plane::axis_aligned(2, r(0));
        let blocker = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let interior = InteriorLeafPoint {
            point: p(0, 0, 0),
            planes: vec![[
                Plane::axis_aligned(0, r(0)),
                Plane::axis_aligned(1, r(0)),
                Plane::from_coefficients(r(1), r(1), r(1), r(0)),
            ]],
            uncertified_definition_fallback: false,
        };
        let probe = ProbePoint {
            point: p(2, 1, 1),
            side: Classification::Positive,
            planes: vec![[
                Plane::axis_aligned(0, r(2)),
                Plane::axis_aligned(1, r(1)),
                Plane::from_coefficients(r(1), r(1), r(1), r(-4)),
            ]],
            uncertified_definition_fallback: false,
        };

        assert!(
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions(
                &interior.point,
                &probe.point,
                &host_support,
                &[blocker],
                &interior.planes,
                &probe.planes,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_step_detour_runtime_allows_three_nested_detours() {
        let start = p(0, 0, 0);
        let detour_a = p(1, 0, 0);
        let detour_b = p(2, 0, 0);
        let detour_c = p(3, 0, 0);
        let end = p(4, 0, 0);
        let start_definitions = [axis_plane_definition(&start)];
        let end_definitions = [axis_plane_definition(&end)];
        let detour_a_definitions = [axis_plane_definition(&detour_a)];
        let detour_b_definitions = [axis_plane_definition(&detour_b)];
        let detour_c_definitions = [axis_plane_definition(&detour_c)];
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             start_definitions_arg: &[[Plane; 3]],
             end_definitions_arg: &[[Plane; 3]]| {
                Ok((*from == start
                    && *to == detour_a
                    && start_definitions_arg == start_definitions
                    && end_definitions_arg == detour_a_definitions)
                    || (*from == detour_a
                        && *to == detour_b
                        && start_definitions_arg == detour_a_definitions
                        && end_definitions_arg == detour_b_definitions)
                    || (*from == detour_b
                        && *to == detour_c
                        && start_definitions_arg == detour_b_definitions
                        && end_definitions_arg == detour_c_definitions)
                    || (*from == detour_c
                        && *to == end
                        && start_definitions_arg == detour_c_definitions
                        && end_definitions_arg == end_definitions))
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![DetourTarget {
                    point: detour_c.clone(),
                    definitions: vec![axis_plane_definition(&detour_c)],
                    uncertified_definition_fallback: false,
                }])
            } else if *from == start && *to == detour_c {
                Ok(vec![DetourTarget {
                    point: detour_b.clone(),
                    definitions: vec![axis_plane_definition(&detour_b)],
                    uncertified_definition_fallback: false,
                }])
            } else if *from == start && *to == detour_b {
                Ok(vec![DetourTarget {
                    point: detour_a.clone(),
                    definitions: vec![axis_plane_definition(&detour_a)],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        };

        assert!(
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
                &start,
                &end,
                &[],
                &[start.clone(), end.clone()],
                &start_definitions,
                &end_definitions,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_uses_geometry_seeded_arrangement_detour_replacement_leg() {
        let host_support = Plane::axis_aligned(2, r(0));
        let start = p(0, 0, 0);
        let end = p(4, 4, 4);
        let mut blockers = vec![
            make_triangle(&p(4, 0, 0), &p(5, 0, 0), &p(4, 1, 0), 0, 0),
            make_triangle(&p(0, 4, 0), &p(1, 4, 0), &p(0, 5, 0), 0, 1),
            make_triangle(&p(0, 0, 4), &p(1, 0, 4), &p(0, 1, 4), 0, 2),
        ];

        for (index, x) in [q(4, 3), r(2), q(8, 3)].into_iter().enumerate() {
            blockers.push(make_triangle(
                &px(x.clone(), -1, -1),
                &px(x.clone(), 5, -1),
                &px(x, 2, 5),
                0,
                3 + index as isize,
            ));
        }

        assert!(!probe_reaches_adjacent_cell(&start, &end, &host_support, &blockers).unwrap());
        assert!(
            probe_reaches_adjacent_cell_via_detours(
                &start,
                &end,
                &host_support,
                &blockers,
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
            )
            .unwrap()
        );
    }

    #[test]
    fn recursive_probe_reachability_budget_retries_detour_legs() {
        let start = p(0, 0, 0);
        let inner = p(1, 0, 0);
        let outer = p(2, 0, 0);
        let end = p(3, 0, 0);
        let outer_target = DetourTarget {
            point: outer.clone(),
            definitions: vec![axis_plane_definition(&outer)],
            uncertified_definition_fallback: false,
        };
        let inner_target = DetourTarget {
            point: inner.clone(),
            definitions: vec![axis_plane_definition(&inner)],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                Ok((*from == start && *to == inner)
                    || (*from == inner && *to == outer)
                    || (*from == outer && *to == end))
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![outer_target.clone()])
            } else if *from == start && *to == outer {
                Ok(vec![inner_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert!(
            !probe_reaches_adjacent_cell_with_definitions_budget_impl(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );

        assert!(
            probe_reaches_adjacent_cell_with_definitions_budget_impl(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                2,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_runtime_allows_three_nested_detours() {
        let start = p(0, 0, 0);
        let detour_a = p(1, 0, 0);
        let detour_b = p(2, 0, 0);
        let detour_c = p(3, 0, 0);
        let end = p(4, 0, 0);
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                Ok((*from == start && *to == detour_a)
                    || (*from == detour_a && *to == detour_b)
                    || (*from == detour_b && *to == detour_c)
                    || (*from == detour_c && *to == end))
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![DetourTarget {
                    point: detour_c.clone(),
                    definitions: vec![axis_plane_definition(&detour_c)],
                    uncertified_definition_fallback: false,
                }])
            } else if *from == start && *to == detour_c {
                Ok(vec![DetourTarget {
                    point: detour_b.clone(),
                    definitions: vec![axis_plane_definition(&detour_b)],
                    uncertified_definition_fallback: false,
                }])
            } else if *from == start && *to == detour_b {
                Ok(vec![DetourTarget {
                    point: detour_a.clone(),
                    definitions: vec![axis_plane_definition(&detour_a)],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        };

        assert!(
            probe_reaches_adjacent_cell_with_cycle_guard_impl(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &[start.clone(), end.clone()],
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_step_detour_cycle_guard_reports_unknown_when_fallback_detour_has_no_path() {
        let start = p(0, 0, 0);
        let detour = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour.clone(),
            definitions: vec![axis_plane_definition(&detour)],
            uncertified_definition_fallback: true,
        };
        let mut trace_without_detours =
            |_from: &Point3,
             _to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| Ok(false);
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        let err =
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
                &start,
                &end,
                &[],
                &[start.clone(), end.clone()],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn probe_step_detour_cycle_guard_reports_unknown_when_fallback_surface_detour_is_skipped() {
        let start = p(0, 0, 0);
        let detour = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour.clone(),
            definitions: vec![axis_plane_definition(&detour)],
            uncertified_definition_fallback: true,
        };
        let polygons = vec![ConvexPolygon {
            support: Plane::axis_aligned(0, r(1)),
            edges: Vec::new(),
            mesh_index: 0,
            polygon_index: 0,
            delta_w: Vec::new(),
            approx_bounds: None,
        }];
        let mut trace_without_detours =
            |_from: &Point3,
             _to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| Ok(false);
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        let err =
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
                &start,
                &end,
                &polygons,
                &[start.clone(), end.clone()],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn probe_step_detour_cycle_guard_reports_unknown_when_fallback_revisited_detour_is_skipped() {
        let start = p(0, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: end.clone(),
            definitions: vec![axis_plane_definition(&end)],
            uncertified_definition_fallback: true,
        };
        let mut trace_without_detours =
            |_from: &Point3,
             _to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| Ok(false);
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        let err =
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
                &start,
                &end,
                &[],
                &[start.clone(), end.clone()],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn probe_step_detour_cycle_guard_tries_later_detour_after_uncertified_surface_query() {
        let start = p(0, 0, 0);
        let first_detour = p(1, 0, 0);
        let second_detour = p(2, 0, 0);
        let end = p(3, 0, 0);
        let mut surface_cache = Vec::new();

        assert!(
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                &[start.clone(), end.clone()],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &mut surface_cache,
                &mut |point| {
                    if *point == first_detour {
                        Err(HypermeshError::UnknownClassification)
                    } else {
                        Ok(false)
                    }
                },
                &mut |_from, _to, _start_definitions, _end_definitions| Ok(true),
                &mut |from, to| {
                    if *from == start && *to == end {
                        Ok(vec![
                            DetourTarget {
                                point: first_detour.clone(),
                                definitions: vec![axis_plane_definition(&first_detour)],
                                uncertified_definition_fallback: false,
                            },
                            DetourTarget {
                                point: second_detour.clone(),
                                definitions: vec![axis_plane_definition(&second_detour)],
                                uncertified_definition_fallback: false,
                            },
                        ])
                    } else {
                        Ok(Vec::new())
                    }
                },
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_step_detour_cycle_guard_reuses_surface_queries_across_failed_branches() {
        let start = p(0, 0, 0);
        let shared = p(1, 0, 0);
        let outer_b = p(2, 0, 0);
        let outer_a = p(3, 0, 0);
        let end = p(4, 0, 0);
        let outer_targets = vec![
            DetourTarget {
                point: outer_a.clone(),
                definitions: vec![axis_plane_definition(&outer_a)],
                uncertified_definition_fallback: false,
            },
            DetourTarget {
                point: outer_b.clone(),
                definitions: vec![axis_plane_definition(&outer_b)],
                uncertified_definition_fallback: false,
            },
        ];
        let shared_target = DetourTarget {
            point: shared.clone(),
            definitions: vec![axis_plane_definition(&shared)],
            uncertified_definition_fallback: false,
        };
        let mut surface_cache = Vec::new();
        let mut query_calls = 0;

        assert!(
            !probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                &[start.clone(), end.clone()],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &mut surface_cache,
                &mut |_point| {
                    query_calls += 1;
                    Ok(false)
                },
                &mut |_from, _to, _start_definitions, _end_definitions| Ok(false),
                &mut |from, to| {
                    if *from == start && *to == end {
                        Ok(outer_targets.clone())
                    } else if *from == start && (*to == outer_a || *to == outer_b) {
                        Ok(vec![shared_target.clone()])
                    } else {
                        Ok(Vec::new())
                    }
                },
            )
            .unwrap()
        );
        assert_eq!(query_calls, 3);
    }

    #[test]
    fn probe_step_detour_entry_reuses_no_detour_and_detour_family_queries() {
        let start = p(0, 0, 0);
        let shared = p(1, 0, 0);
        let outer_b = p(2, 0, 0);
        let outer_a = p(3, 0, 0);
        let end = p(4, 0, 0);
        let outer_targets = vec![
            DetourTarget {
                point: outer_a.clone(),
                definitions: vec![axis_plane_definition(&outer_a)],
                uncertified_definition_fallback: false,
            },
            DetourTarget {
                point: outer_b.clone(),
                definitions: vec![axis_plane_definition(&outer_b)],
                uncertified_definition_fallback: false,
            },
        ];
        let shared_target = DetourTarget {
            point: shared.clone(),
            definitions: vec![axis_plane_definition(&shared)],
            uncertified_definition_fallback: false,
        };
        let mut no_detour_cache = Vec::new();
        let mut detour_target_cache = Vec::new();
        let mut shared_no_detour_calls = 0;
        let mut shared_detour_family_calls = 0;

        assert!(
            !probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &mut no_detour_cache,
                &mut detour_target_cache,
                |from, to, _start_definitions, _end_definitions| {
                    if *from == start && *to == shared {
                        shared_no_detour_calls += 1;
                    }
                    Ok(false)
                },
                |from, to| {
                    if *from == start && *to == end {
                        Ok(outer_targets.clone())
                    } else if *from == start && (*to == outer_a || *to == outer_b) {
                        Ok(vec![shared_target.clone()])
                    } else if *from == start && *to == shared {
                        shared_detour_family_calls += 1;
                        Ok(Vec::new())
                    } else {
                        Ok(Vec::new())
                    }
                },
            )
            .unwrap()
        );
        assert_eq!(shared_no_detour_calls, 1);
        assert_eq!(shared_detour_family_calls, 1);
    }

    #[test]
    fn probe_reachability_from_definitions_shared_query_caches_reuse_equivalent_calls() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definitions = [axis_plane_definition(&start)];
        let end_definitions = [axis_plane_definition(&end)];
        let mut no_detour_cache = Vec::new();
        let mut detour_target_cache = Vec::new();

        let first = probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
            &start,
            &end,
            &[],
            &start_definitions,
            &end_definitions,
            &mut no_detour_cache,
            &mut detour_target_cache,
            |_from, _to, _start_definitions, _end_definitions| Ok(true),
            |_from, _to| Ok(Vec::new()),
        )
        .unwrap();
        let no_detour_len = no_detour_cache.len();
        let detour_len = detour_target_cache.len();
        let second = probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
            &start,
            &end,
            &[],
            &start_definitions,
            &end_definitions,
            &mut no_detour_cache,
            &mut detour_target_cache,
            |_from, _to, _start_definitions, _end_definitions| Ok(true),
            |_from, _to| Ok(Vec::new()),
        )
        .unwrap();

        assert!(first);
        assert!(second);
        assert_eq!(no_detour_cache.len(), no_detour_len);
        assert_eq!(detour_target_cache.len(), detour_len);
    }

    #[test]
    fn probe_reachability_backtracks_after_uncertified_detour_leg() {
        let start = p(0, 0, 0);
        let blocked = p(1, 0, 0);
        let good = p(2, 0, 0);
        let end = p(3, 0, 0);
        let blocked_target = DetourTarget {
            point: blocked.clone(),
            definitions: vec![axis_plane_definition(&blocked)],
            uncertified_definition_fallback: false,
        };
        let good_target = DetourTarget {
            point: good.clone(),
            definitions: vec![axis_plane_definition(&good)],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                if *from == start && *to == blocked {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok((*from == start && *to == good) || (*from == good && *to == end))
                }
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![blocked_target.clone(), good_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert!(
            probe_reaches_adjacent_cell_via_detours_with_budget(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_continues_after_uncertified_no_detour_family() {
        let start = p(0, 0, 0);
        let detour = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour.clone(),
            definitions: vec![axis_plane_definition(&detour)],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                if *from == start && *to == end {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok((*from == start && *to == detour) || (*from == detour && *to == end))
                }
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert!(
            probe_reaches_adjacent_cell_with_definitions_budget_impl(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_reports_unknown_when_no_detour_family_is_uncertified_and_detours_fail() {
        let start = p(0, 0, 0);
        let detour = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour.clone(),
            definitions: vec![axis_plane_definition(&detour)],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                if *from == start && *to == end {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(false)
                }
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        let err = probe_reaches_adjacent_cell_with_definitions_budget_impl(
            &start,
            &end,
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn probe_reachability_reports_unknown_if_all_detours_are_uncertified() {
        let start = p(0, 0, 0);
        let detour = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour.clone(),
            definitions: vec![axis_plane_definition(&detour)],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours =
            |_from: &Point3,
             _to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                Err(HypermeshError::UnknownClassification)
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert_eq!(
            probe_reaches_adjacent_cell_via_detours_with_budget(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            ),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn probe_reachability_reports_unknown_when_fallback_detour_has_no_path() {
        let start = p(0, 0, 0);
        let detour = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour.clone(),
            definitions: vec![axis_plane_definition(&detour)],
            uncertified_definition_fallback: true,
        };
        let mut trace_without_detours =
            |_from: &Point3,
             _to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| Ok(false);
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        let err = probe_reaches_adjacent_cell_via_detours_with_budget(
            &start,
            &end,
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn probe_reachability_reports_unknown_when_fallback_surface_detour_is_skipped() {
        let start = p(0, 0, 0);
        let detour = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour.clone(),
            definitions: vec![axis_plane_definition(&detour)],
            uncertified_definition_fallback: true,
        };
        let polygons = vec![ConvexPolygon {
            support: Plane::axis_aligned(0, r(1)),
            edges: Vec::new(),
            mesh_index: 0,
            polygon_index: 0,
            delta_w: Vec::new(),
            approx_bounds: None,
        }];
        let mut trace_without_detours =
            |_from: &Point3,
             _to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| Ok(false);
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        let err = probe_reaches_adjacent_cell_via_detours_with_budget(
            &start,
            &end,
            &polygons,
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn probe_reachability_reports_unknown_when_fallback_revisited_detour_is_skipped() {
        let start = p(0, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: end.clone(),
            definitions: vec![axis_plane_definition(&end)],
            uncertified_definition_fallback: true,
        };
        let mut trace_without_detours =
            |_from: &Point3,
             _to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| Ok(false);
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        let err = probe_reaches_adjacent_cell_via_detours_with_cycle_guard(
            &start,
            &end,
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &[start.clone(), end.clone()],
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn probe_reachability_cycle_guard_tries_detours_after_uncertified_direct_check() {
        let start = p(0, 0, 0);
        let detour = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour.clone(),
            definitions: vec![axis_plane_definition(&detour)],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                if *from == start && *to == end {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(true)
                }
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert!(
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                &[start.clone(), end.clone()],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &mut Vec::new(),
                &mut |_point| Ok(false),
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_cycle_guard_reports_unknown_when_direct_check_is_uncertified_and_no_detour_succeeds()
     {
        let start = p(0, 0, 0);
        let end = p(2, 0, 0);
        let mut trace_without_detours =
            |_from: &Point3,
             _to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                Err(HypermeshError::UnknownClassification)
            };
        let mut detours_for = |_from: &Point3, _to: &Point3| Ok(Vec::new());

        let err =
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                &[start.clone(), end.clone()],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &mut Vec::new(),
                &mut |_point| Ok(false),
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn probe_reachability_cycle_guard_tries_later_detour_after_uncertified_surface_query() {
        let start = p(0, 0, 0);
        let first_detour = p(1, 0, 0);
        let second_detour = p(2, 0, 0);
        let end = p(3, 0, 0);
        let mut surface_cache = Vec::new();

        assert!(
            probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &[start.clone(), end.clone()],
                &mut surface_cache,
                &mut |point| {
                    if *point == first_detour {
                        Err(HypermeshError::UnknownClassification)
                    } else {
                        Ok(false)
                    }
                },
                &mut |_from, _to, _start_definitions, _end_definitions| Ok(true),
                &mut |from, to| {
                    if *from == start && *to == end {
                        Ok(vec![
                            DetourTarget {
                                point: first_detour.clone(),
                                definitions: vec![axis_plane_definition(&first_detour)],
                                uncertified_definition_fallback: false,
                            },
                            DetourTarget {
                                point: second_detour.clone(),
                                definitions: vec![axis_plane_definition(&second_detour)],
                                uncertified_definition_fallback: false,
                            },
                        ])
                    } else {
                        Ok(Vec::new())
                    }
                },
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_cycle_guard_reuses_surface_queries_across_failed_branches() {
        let start = p(0, 0, 0);
        let shared = p(1, 0, 0);
        let outer_b = p(2, 0, 0);
        let outer_a = p(3, 0, 0);
        let end = p(4, 0, 0);
        let outer_targets = vec![
            DetourTarget {
                point: outer_a.clone(),
                definitions: vec![axis_plane_definition(&outer_a)],
                uncertified_definition_fallback: false,
            },
            DetourTarget {
                point: outer_b.clone(),
                definitions: vec![axis_plane_definition(&outer_b)],
                uncertified_definition_fallback: false,
            },
        ];
        let shared_target = DetourTarget {
            point: shared.clone(),
            definitions: vec![axis_plane_definition(&shared)],
            uncertified_definition_fallback: false,
        };
        let mut surface_cache = Vec::new();
        let mut query_calls = 0;

        assert!(
            !probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &[start.clone(), end.clone()],
                &mut surface_cache,
                &mut |_point| {
                    query_calls += 1;
                    Ok(false)
                },
                &mut |_from, _to, _start_definitions, _end_definitions| Ok(false),
                &mut |from, to| {
                    if *from == start && *to == end {
                        Ok(outer_targets.clone())
                    } else if *from == start && (*to == outer_a || *to == outer_b) {
                        Ok(vec![shared_target.clone()])
                    } else {
                        Ok(Vec::new())
                    }
                },
            )
            .unwrap()
        );
        assert_eq!(query_calls, 3);
    }

    #[test]
    fn probe_plane_replacement_step_detour_budget_uses_single_detour() {
        let start = p(0, 0, 0);
        let detour_point = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour_point.clone(),
            definitions: vec![axis_plane_definition(&detour_point)],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours = |from: &Point3, to: &Point3| {
            Ok((*from == start && *to == detour_point) || (*from == detour_point && *to == end))
        };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert!(
            !probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
                &start,
                &end,
                &[],
                0,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );

        assert!(
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
                &start,
                &end,
                &[],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_winding_plane_replacement_step_detour_budget_uses_single_detour() {
        let start = p(0, 0, 0);
        let detour_point = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour_point.clone(),
            definitions: vec![axis_plane_definition(&detour_point)],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours = |from: &Point3, to: &Point3, winding: &[i32]| {
            if (*from == start && *to == detour_point) || (*from == detour_point && *to == end) {
                Ok(Some(winding.to_vec()))
            } else {
                Ok(None)
            }
        };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert_eq!(
            trace_segment_with_detours_without_plane_replacement_impl(
                &start,
                &end,
                &[0],
                &[],
                0,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap(),
            None
        );

        assert_eq!(
            trace_segment_with_detours_without_plane_replacement_impl(
                &start,
                &end,
                &[0],
                &[],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap(),
            Some(vec![0])
        );
    }

    #[test]
    fn no_detour_segment_search_backtracks_after_uncertified_direct_family() {
        let start = p(0, 0, 0);
        let detour_point = p(1, 0, 0);
        let end = p(2, 0, 0);
        let detour_target = DetourTarget {
            point: detour_point.clone(),
            definitions: vec![axis_plane_definition(&detour_point)],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours = |from: &Point3, to: &Point3, winding: &[i32]| {
            if *from == start && *to == end {
                Err(HypermeshError::UnknownClassification)
            } else if (*from == start && *to == detour_point)
                || (*from == detour_point && *to == end)
            {
                Ok(Some(winding.to_vec()))
            } else {
                Ok(None)
            }
        };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert_eq!(
            trace_segment_with_detours_without_plane_replacement_impl(
                &start,
                &end,
                &[0],
                &[],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap(),
            Some(vec![0])
        );
    }

    #[test]
    fn probe_plane_replacement_step_detours_preserve_intermediate_definitions() {
        let start = p(0, 0, 0);
        let detour_point = p(1, 0, 0);
        let end = p(2, 0, 0);
        let start_definition = axis_plane_definition(&start);
        let end_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-2)),
            Plane::axis_aligned(1, r(0)),
            Plane::axis_aligned(2, r(0)),
        ];
        let detour_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-1)),
            Plane::axis_aligned(1, r(0)),
            Plane::axis_aligned(2, r(0)),
        ];
        let detour_target = DetourTarget {
            point: detour_point.clone(),
            definitions: vec![detour_definition.clone()],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             start_definitions: &[[Plane; 3]],
             end_definitions: &[[Plane; 3]]| {
                Ok((*from == start
                    && *to == detour_point
                    && start_definitions == [start_definition.clone()]
                    && end_definitions == [detour_definition.clone()])
                    || (*from == detour_point
                        && *to == end
                        && start_definitions == [detour_definition.clone()]
                        && end_definitions == [end_definition.clone()]))
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![detour_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };
        let mut affine_cache = Vec::new();
        let mut step_cache = Vec::new();

        assert!(
            plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
                &start_definition,
                &end_definition,
                &mut affine_cache,
                &mut step_cache,
                |from, to, start_definitions, end_definitions| {
                    probe_reaches_adjacent_cell_with_definitions_budget_impl(
                        from,
                        to,
                        &[],
                        start_definitions,
                        end_definitions,
                        1,
                        &mut trace_without_detours,
                        &mut detours_for,
                    )
                },
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_plane_replacement_reachability_surfaces_uncertified_intermediate_orderings() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = [
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(2, r(0)),
        ];
        let mut affine_cache = Vec::new();
        let mut step_cache = Vec::new();

        let err = plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
            &start_definition,
            &end_definition,
            &mut affine_cache,
            &mut step_cache,
            |_from, _to, _start_definitions, _end_definitions| Ok(false),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn plane_replacement_reachability_step_reuses_equivalent_steps_across_orderings() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = axis_plane_definition(&p(1, 0, 0));
        let mut affine_cache = Vec::new();
        let mut step_cache = Vec::new();
        let mut step_calls = 0;

        assert!(
            !plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
                &start_definition,
                &end_definition,
                &mut affine_cache,
                &mut step_cache,
                |_from, _to, _start_definitions, _end_definitions| {
                    step_calls += 1;
                    Ok(false)
                },
            )
            .unwrap()
        );

        assert_eq!(step_calls, 1);
    }

    #[test]
    fn plane_replacement_reachability_tries_later_ordering_after_uncertified_step() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = axis_plane_definition(&p(1, 1, 0));
        let mut affine_cache = Vec::new();
        let mut step_cache = Vec::new();

        assert!(
            plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
                &start_definition,
                &end_definition,
                &mut affine_cache,
                &mut step_cache,
                |from, to, _start_definitions, _end_definitions| {
                    if *from == p(0, 0, 0) && *to == p(1, 0, 0) {
                        Err(HypermeshError::UnknownClassification)
                    } else {
                        Ok(true)
                    }
                },
            )
            .unwrap()
        );
    }

    #[test]
    fn plane_replacement_reachability_step_reuses_permuted_plane_sets() {
        let current_planes = axis_plane_definition(&p(0, 0, 0));
        let next_planes = axis_plane_definition(&p(1, 0, 0));
        let permuted_current = [
            current_planes[1].clone(),
            current_planes[2].clone(),
            current_planes[0].clone(),
        ];
        let permuted_next = [
            next_planes[1].clone(),
            next_planes[2].clone(),
            next_planes[0].clone(),
        ];
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_plane_replacement_reachability_step_with(
            &mut cache,
            &p(0, 0, 0),
            &p(1, 0, 0),
            &current_planes,
            &next_planes,
            || {
                calls += 1;
                Ok(true)
            },
        )
        .unwrap();
        let second = cached_plane_replacement_reachability_step_with(
            &mut cache,
            &p(0, 0, 0),
            &p(1, 0, 0),
            &permuted_current,
            &permuted_next,
            || {
                calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert!(first);
        assert!(second);
    }

    #[test]
    fn plane_replacement_reachability_shared_caches_reuse_equivalent_path_across_calls() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = axis_plane_definition(&p(1, 0, 0));
        let mut affine_cache = Vec::new();
        let mut step_cache = Vec::new();
        let mut step_calls = 0;

        let first = plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
            &start_definition,
            &end_definition,
            &mut affine_cache,
            &mut step_cache,
            |_from, _to, _start_definitions, _end_definitions| {
                step_calls += 1;
                Ok(true)
            },
        )
        .unwrap();
        let second = plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
            &start_definition,
            &end_definition,
            &mut affine_cache,
            &mut step_cache,
            |_from, _to, _start_definitions, _end_definitions| {
                step_calls += 1;
                Ok(true)
            },
        )
        .unwrap();

        assert!(first);
        assert!(second);
        assert_eq!(step_calls, 1);
    }

    #[test]
    fn definition_pair_reachability_backtracks_after_uncertified_pair() {
        let start_unknown = axis_plane_definition(&p(0, 0, 0));
        let start_ok = axis_plane_definition(&p(1, 0, 0));
        let end = axis_plane_definition(&p(2, 0, 0));

        assert!(
            definition_pair_reachability_backtracking_unknown(
                &[start_unknown.clone(), start_ok.clone()],
                std::slice::from_ref(&end),
                |start_definition, end_definition| {
                    if start_definition == &start_unknown && end_definition == &end {
                        Err(HypermeshError::UnknownClassification)
                    } else {
                        Ok(start_definition == &start_ok && end_definition == &end)
                    }
                },
            )
            .unwrap()
        );
    }

    #[test]
    fn definition_pair_reachability_reports_unknown_if_all_pairs_are_uncertified() {
        let start = axis_plane_definition(&p(0, 0, 0));
        let end = axis_plane_definition(&p(1, 0, 0));

        let err = definition_pair_reachability_backtracking_unknown(
            std::slice::from_ref(&start),
            std::slice::from_ref(&end),
            |_start_definition, _end_definition| Err(HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn definition_pair_reachability_search_skips_duplicate_definition_pairs() {
        let start_a = axis_plane_definition(&p(0, 0, 0));
        let start_b = axis_plane_definition(&p(1, 0, 0));
        let end = axis_plane_definition(&p(2, 0, 0));
        let mut reachability_calls = 0;

        let reaches = definition_pair_reachability_backtracking_unknown(
            &[start_a.clone(), start_a.clone(), start_b.clone()],
            &[end.clone(), end.clone()],
            |start_definition, end_definition| {
                reachability_calls += 1;
                if start_definition == &start_a && end_definition == &end {
                    Ok(false)
                } else {
                    Ok(start_definition == &start_b && end_definition == &end)
                }
            },
        )
        .unwrap();

        assert!(reaches);
        assert_eq!(reachability_calls, 2);
    }

    #[test]
    fn definition_pair_reachability_search_skips_permuted_definition_pairs() {
        let start_a = axis_plane_definition(&p(0, 0, 0));
        let start_a_permuted = [start_a[2].clone(), start_a[0].clone(), start_a[1].clone()];
        let start_b = axis_plane_definition(&p(1, 0, 0));
        let end = axis_plane_definition(&p(2, 0, 0));
        let end_permuted = [end[1].clone(), end[2].clone(), end[0].clone()];
        let mut reachability_calls = 0;

        let reaches = definition_pair_reachability_backtracking_unknown(
            &[start_a.clone(), start_a_permuted, start_b.clone()],
            &[end.clone(), end_permuted],
            |start_definition, end_definition| {
                reachability_calls += 1;
                if definition_planes_match_as_sets(start_definition, &start_a)
                    && definition_planes_match_as_sets(end_definition, &end)
                {
                    Ok(false)
                } else {
                    Ok(definition_planes_match_as_sets(start_definition, &start_b)
                        && definition_planes_match_as_sets(end_definition, &end))
                }
            },
        )
        .unwrap();

        assert!(reaches);
        assert_eq!(reachability_calls, 2);
    }

    #[test]
    fn plane_replacement_step_detours_preserve_intermediate_definitions() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = [
            Plane::from_coefficients(r(1), r(1), r(1), r(-1)),
            Plane::axis_aligned(1, r(0)),
            Plane::axis_aligned(2, r(0)),
        ];
        let expected_start_definitions = vec![start_definition.clone()];
        let expected_end_definitions = vec![end_definition.clone()];
        let expected_start = p(0, 0, 0);
        let expected_end = p(1, 0, 0);
        let mut affine_cache = Vec::new();
        let mut step_cache = Vec::new();

        let winding = trace_plane_replacement_path_with_step_detours_impl(
            &start_definition,
            &end_definition,
            &[7],
            &[],
            &mut affine_cache,
            &mut step_cache,
            |current, next, attempt, _polygons, current_definitions, next_definitions| {
                if *current == expected_start
                    && *next == expected_end
                    && current_definitions == expected_start_definitions.as_slice()
                    && next_definitions == expected_end_definitions.as_slice()
                {
                    Ok(Some(attempt.to_vec()))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert_eq!(winding, vec![7]);
    }

    #[test]
    fn plane_replacement_tracer_shared_caches_reuse_equivalent_path_across_calls() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = axis_plane_definition(&p(1, 0, 0));
        let mut affine_cache = Vec::new();
        let mut step_cache = Vec::new();
        let mut step_calls = 0;

        let first = trace_plane_replacement_path_with_tracer_and_caches(
            &start_definition,
            &end_definition,
            &[7],
            &[],
            &mut affine_cache,
            &mut step_cache,
            |_current, _next, _current_planes, _next_planes, attempt, _polygons| {
                step_calls += 1;
                Ok(Some(attempt.to_vec()))
            },
        )
        .unwrap();
        let second = trace_plane_replacement_path_with_tracer_and_caches(
            &start_definition,
            &end_definition,
            &[7],
            &[],
            &mut affine_cache,
            &mut step_cache,
            |_current, _next, _current_planes, _next_planes, attempt, _polygons| {
                step_calls += 1;
                Ok(Some(attempt.to_vec()))
            },
        )
        .unwrap();

        assert_eq!(first, vec![7]);
        assert_eq!(second, vec![7]);
        assert_eq!(step_calls, 1);
    }

    #[test]
    fn plane_replacement_no_detour_shared_caches_reuse_equivalent_path_across_calls() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = axis_plane_definition(&p(1, 0, 0));
        let mut affine_cache = Vec::new();
        let mut step_cache = Vec::new();

        let first = trace_plane_replacement_path_without_detours_with_caches(
            &start_definition,
            &end_definition,
            &[7],
            &[],
            &mut affine_cache,
            &mut step_cache,
        )
        .unwrap();
        let affine_len = affine_cache.len();
        let step_len = step_cache.len();
        let second = trace_plane_replacement_path_without_detours_with_caches(
            &start_definition,
            &end_definition,
            &[7],
            &[],
            &mut affine_cache,
            &mut step_cache,
        )
        .unwrap();

        assert_eq!(first, vec![7]);
        assert_eq!(second, vec![7]);
        assert_eq!(affine_cache.len(), affine_len);
        assert_eq!(step_cache.len(), step_len);
    }

    #[test]
    fn plane_replacement_step_tracer_backtracks_after_uncertified_step() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = axis_plane_definition(&p(1, 1, 0));
        let mut affine_cache = Vec::new();
        let mut step_cache = Vec::new();
        let mut first_call = true;

        let winding = trace_plane_replacement_path_with_tracer(
            &start_definition,
            &end_definition,
            &[7],
            &[],
            |_current, _next, _current_planes, _next_planes, attempt, _polygons| {
                if first_call {
                    first_call = false;
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(attempt.to_vec()))
                }
            },
            &mut affine_cache,
            &mut step_cache,
        )
        .unwrap();

        assert_eq!(winding, vec![7]);
    }

    #[test]
    fn plane_replacement_step_tracer_reuses_equivalent_steps_across_orderings() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = axis_plane_definition(&p(1, 0, 0));
        let mut affine_cache = Vec::new();
        let mut step_cache = Vec::new();
        let mut step_calls = 0;

        let err = trace_plane_replacement_path_with_tracer(
            &start_definition,
            &end_definition,
            &[7],
            &[],
            |_current, _next, _current_planes, _next_planes, _attempt, _polygons| {
                step_calls += 1;
                Ok(None)
            },
            &mut affine_cache,
            &mut step_cache,
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
        assert_eq!(step_calls, 1);
    }

    #[test]
    fn plane_replacement_step_tracer_reuses_permuted_plane_sets() {
        let current_planes = axis_plane_definition(&p(0, 0, 0));
        let next_planes = axis_plane_definition(&p(1, 0, 0));
        let permuted_current = [
            current_planes[1].clone(),
            current_planes[2].clone(),
            current_planes[0].clone(),
        ];
        let permuted_next = [
            next_planes[1].clone(),
            next_planes[2].clone(),
            next_planes[0].clone(),
        ];
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_plane_replacement_step_with(
            &mut cache,
            &p(0, 0, 0),
            &p(1, 0, 0),
            &current_planes,
            &next_planes,
            &[7],
            || {
                calls += 1;
                Ok(Some(vec![7]))
            },
        )
        .unwrap();
        let second = cached_plane_replacement_step_with(
            &mut cache,
            &p(0, 0, 0),
            &p(1, 0, 0),
            &permuted_current,
            &permuted_next,
            &[7],
            || {
                calls += 1;
                Ok(Some(vec![9]))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, Some(vec![7]));
        assert_eq!(second, Some(vec![7]));
    }

    #[test]
    fn cached_affine_from_planes_reuses_identical_plane_set() {
        let planes = axis_plane_definition(&p(1, 2, 3));
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_affine_from_planes_with(&mut cache, &planes, || {
            calls += 1;
            affine_from_planes(&planes)
        })
        .unwrap();
        let second = cached_affine_from_planes_with(&mut cache, &planes, || {
            calls += 1;
            affine_from_planes(&planes)
        })
        .unwrap();

        assert_eq!(first, p(1, 2, 3));
        assert_eq!(second, first);
        assert_eq!(calls, 1);
    }

    #[test]
    fn cached_affine_from_planes_reuses_permuted_plane_set() {
        let planes = axis_plane_definition(&p(1, 2, 3));
        let permuted = [planes[1].clone(), planes[2].clone(), planes[0].clone()];
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_affine_from_planes_with(&mut cache, &planes, || {
            calls += 1;
            affine_from_planes(&planes)
        })
        .unwrap();
        let second = cached_affine_from_planes_with(&mut cache, &permuted, || {
            calls += 1;
            affine_from_planes(&permuted)
        })
        .unwrap();

        assert_eq!(first, p(1, 2, 3));
        assert_eq!(second, first);
        assert_eq!(calls, 1);
    }

    #[test]
    fn recursive_detour_budget_retries_detour_legs() {
        let start = p(0, 0, 0);
        let inner = p(1, 0, 0);
        let outer = p(2, 0, 0);
        let end = p(3, 0, 0);
        let outer_target = DetourTarget {
            point: outer.clone(),
            definitions: vec![axis_plane_definition(&outer)],
            uncertified_definition_fallback: false,
        };
        let inner_target = DetourTarget {
            point: inner.clone(),
            definitions: vec![axis_plane_definition(&inner)],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             winding: &[i32],
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                if (*from == start && *to == inner)
                    || (*from == inner && *to == outer)
                    || (*from == outer && *to == end)
                {
                    Ok(Some(winding.to_vec()))
                } else {
                    Ok(None)
                }
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![outer_target.clone()])
            } else if *from == start && *to == outer {
                Ok(vec![inner_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert_eq!(
            trace_segment_from_definitions_with_budget_impl(
                &start,
                &end,
                &[0],
                &[],
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                1,
                &mut trace_without_detours,
                &mut detours_for,
            ),
            Err(HypermeshError::UnknownClassification)
        );

        let with_nested = trace_segment_from_definitions_with_budget_impl(
            &start,
            &end,
            &[0],
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            2,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap();
        assert_eq!(with_nested, vec![0]);
    }

    #[test]
    fn trace_segment_from_definitions_runtime_allows_three_nested_detours() {
        let start = p(0, 0, 0);
        let detour_a = p(1, 0, 0);
        let detour_b = p(2, 0, 0);
        let detour_c = p(3, 0, 0);
        let end = p(4, 0, 0);
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             winding: &[i32],
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                if (*from == start && *to == detour_a)
                    || (*from == detour_a && *to == detour_b)
                    || (*from == detour_b && *to == detour_c)
                    || (*from == detour_c && *to == end)
                {
                    Ok(Some(winding.to_vec()))
                } else {
                    Ok(None)
                }
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![DetourTarget {
                    point: detour_c.clone(),
                    definitions: vec![axis_plane_definition(&detour_c)],
                    uncertified_definition_fallback: false,
                }])
            } else if *from == start && *to == detour_c {
                Ok(vec![DetourTarget {
                    point: detour_b.clone(),
                    definitions: vec![axis_plane_definition(&detour_b)],
                    uncertified_definition_fallback: false,
                }])
            } else if *from == start && *to == detour_b {
                Ok(vec![DetourTarget {
                    point: detour_a.clone(),
                    definitions: vec![axis_plane_definition(&detour_a)],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        };

        let traced = trace_segment_from_definitions_with_cycle_guard_impl(
            &start,
            &end,
            &[0],
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &[start.clone(), end.clone()],
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap();

        assert_eq!(traced, vec![0]);
    }

    #[test]
    fn trace_segment_from_definitions_cycle_guard_skips_revisited_path_points() {
        let start = p(0, 0, 0);
        let detour = p(1, 0, 0);
        let end = p(2, 0, 0);
        let mut trace_without_detours =
            |_from: &Point3,
             _to: &Point3,
             _winding: &[i32],
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| { Ok(None) };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![DetourTarget {
                    point: detour.clone(),
                    definitions: vec![axis_plane_definition(&detour)],
                    uncertified_definition_fallback: false,
                }])
            } else if *from == detour && *to == end {
                Ok(vec![DetourTarget {
                    point: start.clone(),
                    definitions: vec![axis_plane_definition(&start)],
                    uncertified_definition_fallback: false,
                }])
            } else if *from == detour && *to == start {
                Ok(vec![DetourTarget {
                    point: end.clone(),
                    definitions: vec![axis_plane_definition(&end)],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        };

        let err = trace_segment_from_definitions_with_cycle_guard_impl(
            &start,
            &end,
            &[0],
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &[start.clone(), end.clone()],
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn detour_recursion_limit_scales_with_local_polygon_count() {
        let polygons = vec![
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 0, 1),
            make_triangle(&p(0, 0, 2), &p(1, 0, 2), &p(0, 1, 2), 0, 2),
        ];

        assert_eq!(detour_recursion_limit(&[]), 2);
        assert_eq!(detour_recursion_limit(&polygons), 3);
    }

    #[test]
    fn plane_replacement_step_detour_limit_scales_with_local_polygon_count() {
        let polygons = vec![
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 0, 1),
        ];

        assert_eq!(plane_replacement_step_detour_limit(&[]), 1);
        assert_eq!(plane_replacement_step_detour_limit(&polygons), 2);
    }

    #[test]
    fn polygon_scaled_detour_budget_allows_two_nested_detours() {
        let start = p(0, 0, 0);
        let inner = p(1, 0, 0);
        let outer = p(2, 0, 0);
        let end = p(3, 0, 0);
        let polygons = vec![
            make_triangle(&p(0, 10, 0), &p(1, 10, 0), &p(0, 11, 0), 0, 0),
            make_triangle(&p(0, 10, 1), &p(1, 10, 1), &p(0, 11, 1), 0, 1),
            make_triangle(&p(0, 10, 2), &p(1, 10, 2), &p(0, 11, 2), 0, 2),
        ];
        let outer_target = DetourTarget {
            point: outer.clone(),
            definitions: vec![axis_plane_definition(&outer)],
            uncertified_definition_fallback: false,
        };
        let inner_target = DetourTarget {
            point: inner.clone(),
            definitions: vec![axis_plane_definition(&inner)],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             winding: &[i32],
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                if (*from == start && *to == inner)
                    || (*from == inner && *to == outer)
                    || (*from == outer && *to == end)
                {
                    Ok(Some(winding.to_vec()))
                } else {
                    Ok(None)
                }
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![outer_target.clone()])
            } else if *from == start && *to == outer {
                Ok(vec![inner_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        let traced = trace_segment_from_definitions_with_budget_impl(
            &start,
            &end,
            &[0],
            &polygons,
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            detour_recursion_limit(&polygons),
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap();

        assert_eq!(traced, vec![0]);
    }

    #[test]
    fn polygon_scaled_probe_step_detour_budget_allows_two_nested_detours() {
        let start = p(0, 0, 0);
        let inner = p(1, 0, 0);
        let outer = p(2, 0, 0);
        let end = p(3, 0, 0);
        let polygons = vec![
            make_triangle(&p(0, 10, 0), &p(1, 10, 0), &p(0, 11, 0), 0, 0),
            make_triangle(&p(0, 10, 1), &p(1, 10, 1), &p(0, 11, 1), 0, 1),
        ];
        let outer_target = DetourTarget {
            point: outer.clone(),
            definitions: vec![axis_plane_definition(&outer)],
            uncertified_definition_fallback: false,
        };
        let inner_target = DetourTarget {
            point: inner.clone(),
            definitions: vec![axis_plane_definition(&inner)],
            uncertified_definition_fallback: false,
        };
        let mut trace_without_detours =
            |from: &Point3,
             to: &Point3,
             _start_definitions: &[[Plane; 3]],
             _end_definitions: &[[Plane; 3]]| {
                Ok((*from == start && *to == inner)
                    || (*from == inner && *to == outer)
                    || (*from == outer && *to == end))
            };
        let mut detours_for = |from: &Point3, to: &Point3| {
            if *from == start && *to == end {
                Ok(vec![outer_target.clone()])
            } else if *from == start && *to == outer {
                Ok(vec![inner_target.clone()])
            } else {
                Ok(Vec::new())
            }
        };

        assert!(
            probe_reaches_adjacent_cell_with_definitions_budget_impl(
                &start,
                &end,
                &polygons,
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                plane_replacement_step_detour_limit(&polygons),
                &mut trace_without_detours,
                &mut detours_for,
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_fallback_retries_axis_start_after_retained_definitions_fail() {
        let ref_point = p(0, 0, 0);
        let invalid_ref_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(0, r(2)),
        ];
        let valid_probe_definition = [
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(1, r(1)),
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
        ];
        let probe = ProbePoint {
            point: p(2, 1, 0),
            side: Classification::Positive,
            planes: vec![valid_probe_definition],
            uncertified_definition_fallback: false,
        };
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];

        assert_eq!(
            trace_segment(&ref_point, &probe.point, &[0], &[wall.clone()]),
            Err(HypermeshError::UnknownClassification)
        );

        let winding =
            trace_probe_winding(&ref_point, &[invalid_ref_definition], &probe, &[0], &[wall])
                .unwrap();

        assert_eq!(winding, vec![0]);
    }

    #[test]
    fn probe_winding_reports_unknown_if_all_definition_paths_are_uncertified() {
        let ref_point = p(0, 0, 0);
        let ref_definitions = [[
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(0, r(2)),
        ]];
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];
        let probe = ProbePoint {
            point: p(2, 1, 0),
            side: Classification::Positive,
            planes: Vec::new(),
            uncertified_definition_fallback: false,
        };

        assert_eq!(
            trace_segment(&ref_point, &probe.point, &[0], &[wall.clone()]),
            Err(HypermeshError::UnknownClassification)
        );

        let err =
            trace_probe_winding(&ref_point, &ref_definitions, &probe, &[0], &[wall]).unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }
}
