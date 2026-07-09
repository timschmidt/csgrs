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
struct VisitedDefinitionPoint {
    point: Point3,
    definitions: Vec<[Plane; 3]>,
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
struct DirectProbeReachabilityCacheEntry {
    start: Point3,
    end: Point3,
    host_support: Plane,
    polygons: Vec<ConvexPolygon>,
    reachable: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
#[cfg(test)]
struct ProbePointFamilyCacheEntry {
    interior_point: Point3,
    interior_planes: Vec<[Plane; 3]>,
    support: Plane,
    bounds: Aabb,
    positive_side: bool,
    probes: HypermeshResult<Vec<ProbePoint>>,
}

#[derive(Clone, Debug, PartialEq)]
struct HalfspaceSeedFamilyCacheEntry {
    bounds: Aabb,
    halfspaces: Vec<LimitPlane3>,
    saw_unknown: bool,
    result: HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)>,
}

#[derive(Clone, Debug, PartialEq)]
struct HalfspaceReportCacheEntry {
    halfspaces: Vec<LimitPlane3>,
    saw_unknown: bool,
    report: Option<hyperlimit::HalfspaceFeasibilityReport>,
}

#[derive(Clone, Debug, PartialEq)]
struct NormalProbeStopCacheEntry {
    interior_point: Point3,
    direction: Point3,
    support: Plane,
    bounds: Aabb,
    saw_unknown: bool,
    stop_values: HypermeshResult<Vec<Real>>,
}

#[derive(Clone, Debug, PartialEq)]
struct AxisProbeStopCacheEntry {
    interior_point: Point3,
    bounds: Aabb,
    axis: usize,
    direction_positive: bool,
    saw_unknown: bool,
    stop_values: HypermeshResult<Vec<Real>>,
}

#[cfg(test)]
#[derive(Clone, Debug, PartialEq)]
struct NormalProbeFamilyCacheEntry {
    interior_point: Point3,
    interior_planes: Vec<[Plane; 3]>,
    support: Plane,
    bounds: Aabb,
    positive_side: bool,
    probes: HypermeshResult<Vec<ProbePoint>>,
}

#[cfg(test)]
#[derive(Clone, Debug, PartialEq)]
struct AxisProbeFamilyCacheEntry {
    interior_point: Point3,
    interior_planes: Vec<[Plane; 3]>,
    support: Plane,
    bounds: Aabb,
    axis: usize,
    direction_positive: bool,
    probes: HypermeshResult<Vec<ProbePoint>>,
}

#[derive(Default)]
pub(crate) struct LeafProbeQueryCaches {
    #[cfg(test)]
    #[cfg_attr(test, allow(dead_code))]
    normal_probe_families: Vec<NormalProbeFamilyCacheEntry>,
    #[cfg(test)]
    #[cfg_attr(test, allow(dead_code))]
    axis_probe_families: Vec<AxisProbeFamilyCacheEntry>,
    #[cfg(test)]
    #[cfg_attr(test, allow(dead_code))]
    probe_families: Vec<ProbePointFamilyCacheEntry>,
    normal_probe_stop_values: Vec<NormalProbeStopCacheEntry>,
    axis_probe_stop_values: Vec<AxisProbeStopCacheEntry>,
    halfspace_reports: Vec<HalfspaceReportCacheEntry>,
    halfspace_seed_families: Vec<HalfspaceSeedFamilyCacheEntry>,
    probe_winding: Vec<ProbeWindingCacheEntry>,
    probe_surface: Vec<SurfaceCacheEntry>,
    probe_reachability: Vec<ProbeReachabilityCacheEntry>,
    direct_probe_reachability: Vec<DirectProbeReachabilityCacheEntry>,
    axis_ordered_segment_traces: Vec<AxisOrderedSegmentTraceCacheEntry>,
    plane_replacement_affine: Vec<PlaneReplacementAffineCacheEntry>,
    plane_replacement_trace_steps: Vec<PlaneReplacementStepCacheEntry>,
    plane_replacement_reachability_paths: Vec<PlaneReplacementReachabilityPathCacheEntry>,
    plane_replacement_reachability_steps: Vec<PlaneReplacementReachabilityStepCacheEntry>,
    plane_replacement_no_nested_ordering_warmups:
        Vec<PlaneReplacementNoNestedOrderingWarmupCacheEntry>,
    definition_cycle_guard_reachability: Vec<DefinitionCycleGuardReachabilityCacheEntry>,
    definition_no_step_detour_reachability: Vec<DefinitionNoDetourReachabilityCacheEntry>,
    definition_no_plane_replacement_cycle_guard:
        Vec<DefinitionNoPlaneReplacementCycleGuardCacheEntry>,
    definition_no_plane_replacement_reachability:
        Vec<DefinitionNoPlaneReplacementReachabilityCacheEntry>,
    no_step_detour_target_families: Vec<DetourTargetFamilyCacheEntry>,
    definition_no_detour_trace: Vec<DefinitionNoDetourTraceCacheEntry>,
    definition_no_detour_reachability: Vec<DefinitionNoDetourReachabilityCacheEntry>,
    detour_target_families: Vec<DetourTargetFamilyCacheEntry>,
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
    mode: PlaneReplacementReachabilityStepMode,
    current_point: Point3,
    next_point: Point3,
    current_planes: [Plane; 3],
    next_planes: [Plane; 3],
    result: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
struct PlaneReplacementReachabilityPathCacheEntry {
    mode: PlaneReplacementReachabilityStepMode,
    start_planes: [Plane; 3],
    end_planes: [Plane; 3],
    result: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
struct PlaneReplacementNoNestedOrderingWarmupCacheEntry {
    start_planes: [Plane; 3],
    end_planes: [Plane; 3],
    ordered: HypermeshResult<Vec<[usize; 3]>>,
    affine_cache: Vec<PlaneReplacementAffineCacheEntry>,
    path_cache: Vec<PlaneReplacementReachabilityPathCacheEntry>,
    step_cache: Vec<PlaneReplacementReachabilityStepCacheEntry>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PlaneReplacementReachabilityStepMode {
    WithoutNestedPlaneReplacement,
    WithoutStepDetours,
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
struct DefinitionCycleGuardReachabilityCacheEntry {
    start: Point3,
    end: Point3,
    start_definitions: Vec<[Plane; 3]>,
    end_definitions: Vec<[Plane; 3]>,
    visited_points: Vec<VisitedDefinitionPoint>,
    result: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
struct DefinitionNoPlaneReplacementCycleGuardCacheEntry {
    start: Point3,
    end: Point3,
    start_definitions: Vec<[Plane; 3]>,
    end_definitions: Vec<[Plane; 3]>,
    visited_points: Vec<VisitedDefinitionPoint>,
    result: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
struct DefinitionNoPlaneReplacementReachabilityCacheEntry {
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
    let mut surface_cache = Vec::new();
    let mut axis_ordered_segment_traces = Vec::new();
    let mut plane_replacement_affine = Vec::new();
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
    plane_replacement_affine: &mut Vec<PlaneReplacementAffineCacheEntry>,
    plane_replacement_trace_steps: &mut Vec<PlaneReplacementStepCacheEntry>,
    no_detour_cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
    let mut no_detour_surface_cache = Vec::new();
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
                    )
                },
            )
        };
    let mut detours_for = |start: &Point3, end: &Point3| {
        cached_detour_target_family_with(&mut *detour_target_cache, start, end, || {
            interior_box_detour_targets(start, end, polygons)
        })
    };
    trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
        start,
        end,
        winding,
        polygons,
        start_definitions,
        end_definitions,
        &initial_visited_definition_points(start, start_definitions, end, end_definitions),
        surface_cache,
        &mut |point| point_lies_on_traced_surface(point, polygons),
        &mut trace_without_detours,
        &mut detours_for,
    )
}

#[cfg(test)]
#[allow(dead_code)]
fn trace_segment_from_definitions_with_cycle_guard_impl(
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

fn trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
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
        if detour.uncertified_definition_fallback {
            saw_unknown = true;
            continue;
        }
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

fn cached_detour_target_family_with(
    cache: &mut Vec<DetourTargetFamilyCacheEntry>,
    start: &Point3,
    end: &Point3,
    build: impl FnOnce() -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<Vec<DetourTarget>> {
    if let Some(existing) = cached_detour_target_family(cache, start, end) {
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

fn cached_detour_target_family<'a>(
    cache: &'a [DetourTargetFamilyCacheEntry],
    start: &Point3,
    end: &Point3,
) -> Option<&'a DetourTargetFamilyCacheEntry> {
    cache.iter().rev().find(|existing| {
        (existing.start == *start && existing.end == *end)
            || (existing.start == *end && existing.end == *start)
    })
}

#[cfg(test)]
fn trace_segment_without_detours(
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
        if detour.uncertified_definition_fallback {
            saw_unknown = true;
            continue;
        }
        return Ok(Some(second_leg));
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

#[cfg(test)]
fn trace_segment_with_definitions_no_detours(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut surface_cache = Vec::new();
    let mut axis_ordered_segment_traces = Vec::new();
    let mut affine_cache = Vec::new();
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
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
) -> HypermeshResult<Option<WindingNumberVector>> {
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

    let mut start_definitions = start_definitions.to_vec();
    append_definition_if_missing(&mut start_definitions, axis_plane_definition(start));
    let mut end_definitions = end_definitions.to_vec();
    append_definition_if_missing(&mut end_definitions, axis_plane_definition(end));

    definition_pair_trace_backtracking_unknown(
        &start_definitions,
        &end_definitions,
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

#[cfg(test)]
fn trace_plane_replacement_path_without_detours_with_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
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
    )
}

fn trace_plane_replacement_path_without_detours_with_shared_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    axis_ordered_segment_traces: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
    trace_plane_replacement_path_with_tracer(
        start_planes,
        end_planes,
        winding,
        polygons,
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

        for plane_index in ordering.iter().copied() {
            let mut next_planes = current_planes.clone();
            next_planes[plane_index] = end_planes[plane_index].clone();
            if next_planes == current_planes {
                continue;
            }
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
        .rev()
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

#[cfg(test)]
fn trace_axis_ordered_paths(
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

fn cached_axis_ordered_segment_trace_with(
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
    let unresolved_fallback = targets
        .iter()
        .any(|target| target.uncertified_definition_fallback);
    let has_certified_target = targets
        .iter()
        .any(|target| !target.uncertified_definition_fallback);
    if targets.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_target && (saw_unknown || unresolved_fallback) {
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
    strict_aabb_targets_with_seed_families(bounds, |bounds, halfspaces, report, saw_unknown| {
        halfspace_cell_seed_families_from_optional_report(bounds, halfspaces, report, saw_unknown)
    })
}

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

#[cfg(test)]
fn search_strict_aabb_targets_progressively_with_seed_families(
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
    let halfspaces = aabb_core_halfspaces(bounds)?;
    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(false);
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
            certified_direct_target_points.push(target.point.clone());
        }
        direct_targets.push(target);
    }

    let mut ranked_direct_targets = Vec::with_capacity(direct_targets.len());
    for (index, target) in direct_targets.into_iter().enumerate() {
        let (rank_missing, rank) = match rank_direct(&target) {
            Ok(rank) => (0u8, Some(rank)),
            Err(HypermeshError::UnknownClassification) => (1u8, None),
            Err(err) => return Err(err),
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
                if target.uncertified_definition_fallback {
                    saw_unknown = true;
                } else {
                    return Ok(true);
                }
            }
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

    for witness in &shifted_witnesses {
        let target = match build_detour_target_from_shifted_witness(witness) {
            Ok(target) => target,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        match evaluate(target.clone()) {
            Ok(true) => {
                if target.uncertified_definition_fallback {
                    saw_unknown = true;
                } else {
                    return Ok(true);
                }
            }
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

fn detour_shifted_seed_families(
    report_witness: Option<&Point3>,
    certified_direct_target_points: &[Point3],
    seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
) -> (Vec<Point3>, Vec<Point3>, Vec<Point3>) {
    if !certified_direct_target_points.is_empty() {
        let _ = report_witness;
        return (Vec::new(), Vec::new(), Vec::new());
    }

    shifted_halfspace_seed_families_with_report_seed(
        report_witness,
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    )
}

fn strict_aabb_targets_with_seed_families(
    bounds: &Aabb,
    mut seed_families_for: impl FnMut(
        &Aabb,
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
        &mut bool,
    ) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)>,
) -> HypermeshResult<Vec<DetourTarget>> {
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
        seed_families_for(bounds, &halfspaces, report.as_ref(), &mut saw_unknown)?;
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    extend_detour_target_builds_backtracking_unknown(&mut targets, seeds.iter(), |seed| {
        let active_planes = active_planes_from_optional_report(report.as_ref(), seed);
        build_detour_target(seed, &halfspaces, active_planes, false)
    })?;

    let certified_direct_target_points = targets
        .iter()
        .filter(|target| !target.uncertified_definition_fallback)
        .map(|target| target.point.clone())
        .collect::<Vec<_>>();
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
    extend_detour_target_builds_backtracking_unknown(
        &mut targets,
        shifted_witnesses.iter(),
        build_detour_target_from_shifted_witness,
    )?;
    let unresolved_fallback = targets
        .iter()
        .any(|target| target.uncertified_definition_fallback);
    let has_certified_target = targets
        .iter()
        .any(|target| !target.uncertified_definition_fallback);
    if targets.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_target && (saw_unknown || unresolved_fallback) {
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

fn push_unique_detour_target(targets: &mut Vec<DetourTarget>, target: DetourTarget) -> bool {
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
    let unresolved_fallback = targets
        .iter()
        .any(|target| target.uncertified_definition_fallback);
    let has_certified_target = targets
        .iter()
        .any(|target| !target.uncertified_definition_fallback);
    if targets.is_empty() && (saw_hard_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_target && (saw_hard_unknown || unresolved_fallback) {
            mark_all_detour_targets_uncertified(targets);
        }
        Ok(())
    }
}

#[cfg(test)]
fn extend_detour_target_families_backtracking_unknown(
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
    let unresolved_fallback = targets
        .iter()
        .any(|target| target.uncertified_definition_fallback);
    let has_certified_target = targets
        .iter()
        .any(|target| !target.uncertified_definition_fallback);
    if targets.is_empty() && (saw_hard_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_target && (saw_hard_unknown || unresolved_fallback) {
            mark_all_detour_targets_uncertified(targets);
        }
        Ok(())
    }
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
    let unresolved_fallback = targets
        .iter()
        .any(|target| target.uncertified_definition_fallback);
    let has_certified_target = targets
        .iter()
        .any(|target| !target.uncertified_definition_fallback);
    if targets.is_empty() && (saw_hard_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_target && (saw_hard_unknown || unresolved_fallback) {
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
        let mut on_edge = false;
        for edge in &polygon.edges {
            match classify_point(point, edge)? {
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
    let mut probe_query_caches = LeafProbeQueryCaches::default();
    classify_leaf_polygon_with_probe_query_caches(
        support,
        leaf_edges,
        ref_point,
        ref_definitions,
        ref_wnv,
        polygons,
        bounds,
        host_delta_w,
        &mut probe_query_caches,
    )
}

pub(crate) fn classify_leaf_polygon_with_probe_query_caches(
    support: &Plane,
    leaf_edges: &[Plane],
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
) -> HypermeshResult<WindingNumberVector> {
    let interior_points = certified_leaf_interior_points(support, leaf_edges)?;
    classify_leaf_polygon_from_interior_points_with_probe_query_caches(
        &interior_points,
        support,
        ref_point,
        ref_definitions,
        ref_wnv,
        polygons,
        bounds,
        host_delta_w,
        probe_query_caches,
    )
}

#[cfg(test)]
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
    let mut probe_query_caches = LeafProbeQueryCaches::default();
    classify_leaf_polygon_from_interior_points_with_probe_query_caches(
        interior_points,
        support,
        ref_point,
        ref_definitions,
        ref_wnv,
        polygons,
        bounds,
        host_delta_w,
        &mut probe_query_caches,
    )
}

pub(crate) fn classify_leaf_polygon_from_interior_points_with_probe_query_caches(
    interior_points: &[InteriorLeafPoint],
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
) -> HypermeshResult<WindingNumberVector> {
    let mut saw_unknown = false;

    for point in ordered_interior_points_for_probe_search_with_support(interior_points, support)? {
        if let Some(winding) = classify_leaf_polygon_interior_point_with_probe_query_caches(
            point,
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            bounds,
            host_delta_w,
            probe_query_caches,
            &mut saw_unknown,
        )? {
            return Ok(winding);
        }
    }

    let _ = saw_unknown;
    Err(HypermeshError::UnknownClassification)
}

#[cfg(test)]
pub(crate) fn ordered_interior_points_for_probe_search(
    interior_points: &[InteriorLeafPoint],
) -> Vec<&InteriorLeafPoint> {
    let mut ordered = interior_points.iter().enumerate().collect::<Vec<_>>();
    ordered.sort_by_key(|(index, point)| {
        (
            std::cmp::Reverse(max_axis_aligned_planes_in_definition_family(point)),
            *index,
        )
    });
    ordered.into_iter().map(|(_, point)| point).collect()
}

fn max_axis_aligned_planes_in_definition_family(point: &InteriorLeafPoint) -> usize {
    point
        .planes
        .iter()
        .map(|definition| {
            definition
                .iter()
                .filter(|plane| plane.axis_split_value().is_some())
                .count()
        })
        .max()
        .unwrap_or(0)
}

pub(crate) fn classify_leaf_polygon_interior_point_with_probe_query_caches(
    point: &InteriorLeafPoint,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
) -> HypermeshResult<Option<WindingNumberVector>> {
    for positive_side in [true, false] {
        if let Some(winding) = search_adjacent_normal_probe_winding_with_queries(
            point,
            positive_side,
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            bounds,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
        )? {
            return Ok(Some(winding));
        }

        for axis in probe_axes(support)? {
            let normal_sign = crate::geometry::classify_real(axis_ref(&support.normal, axis))?;
            if normal_sign == Classification::On {
                continue;
            }

            let direction_positive = (normal_sign == Classification::Positive) == positive_side;
            let axis_value = axis_ref(&point.point, axis);
            let room = if direction_positive {
                axis_ref(&bounds.max, axis) - axis_value
            } else {
                axis_value - axis_ref(&bounds.min, axis)
            };
            if !compare_real(&room, &Real::zero())?.is_gt() {
                continue;
            }

            if let Some(winding) = search_adjacent_axis_probe_winding_with_queries(
                point,
                positive_side,
                axis,
                direction_positive,
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                probe_query_caches,
                saw_unknown,
                bounds,
                host_delta_w,
            )? {
                return Ok(Some(winding));
            }
        }
    }

    Ok(None)
}

pub(crate) fn ordered_interior_points_for_probe_search_with_support<'a>(
    interior_points: &'a [InteriorLeafPoint],
    support: &Plane,
) -> HypermeshResult<Vec<&'a InteriorLeafPoint>> {
    let mut scored = interior_points
        .iter()
        .enumerate()
        .map(|(index, point)| {
            Ok((
                index,
                point,
                unique_normal_probe_search_definitions(&point.planes, support)?.len(),
            ))
        })
        .collect::<HypermeshResult<Vec<_>>>()?;
    scored.sort_by_key(|(index, point, retained_definition_count)| {
        (
            std::cmp::Reverse(*retained_definition_count),
            std::cmp::Reverse(max_axis_aligned_planes_in_definition_family(point)),
            *index,
        )
    });
    Ok(scored.into_iter().map(|(_, point, _)| point).collect())
}

fn search_adjacent_normal_probe_winding_with_queries(
    point: &InteriorLeafPoint,
    positive_side: bool,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let retained_definitions = unique_normal_probe_search_definitions(&point.planes, support)?;
    let direction = if positive_side {
        support.normal.clone()
    } else {
        Point3::new(
            -support.normal.x.clone(),
            -support.normal.y.clone(),
            -support.normal.z.clone(),
        )
    };
    let (stop_values, local_unknown) = cached_adjacent_normal_probe_stop_values_with(
        &mut probe_query_caches.normal_probe_stop_values,
        &point.point,
        &direction,
        support,
        bounds,
        || {
            adjacent_normal_probe_stop_values_with_queries(
                &point.point,
                &direction,
                support,
                bounds,
                polygons,
                &mut |_interior, direction, polygon| {
                    Ok(dot_direction(&polygon.support.normal, direction))
                },
                &mut |candidate, polygon| classify_point_in_polygon(candidate, polygon),
            )
        },
    )?;
    *saw_unknown |= local_unknown;

    for stop_t in stop_values {
        if !compare_real(&stop_t, &Real::zero())?.is_gt() {
            continue;
        }
        let stop_point = offset_point(&point.point, &direction, &stop_t);
        let corridor = bounds_between_points(&point.point, &stop_point)?;

        for definition in &retained_definitions {
            if let Some(winding) = try_strict_normal_probe_report_witness_winding_with_queries(
                point,
                positive_side,
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
                &corridor,
                Some(definition),
                &stop_point,
            )? {
                return Ok(Some(winding));
            }
            if let Some(winding) = try_strict_normal_seed_winding_with_queries(
                point,
                positive_side,
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
                &corridor,
                Some(definition),
                &stop_point,
            )? {
                return Ok(Some(winding));
            }
            let probes = strict_normal_probe_targets_with_query_caches(
                point,
                support,
                &corridor,
                Some(definition),
                &stop_point,
                positive_side,
                probe_query_caches,
            );
            if let Some(winding) = try_leaf_probe_family_with_queries(
                point,
                positive_side,
                probes,
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
            )? {
                return Ok(Some(winding));
            }
        }

        let unrestricted_report = try_strict_normal_probe_report_witness_winding_with_queries(
            point,
            positive_side,
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
            &corridor,
            None,
            &stop_point,
        )?;
        if let Some(winding) = unrestricted_report {
            return Ok(Some(winding));
        }
        let unrestricted_progressive =
            try_strict_normal_probe_targets_progressively_with_query_caches(
                point,
                positive_side,
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
                &corridor,
                None,
                &stop_point,
            )?;
        if let Some(winding) = unrestricted_progressive {
            return Ok(Some(winding));
        }
        let probes = strict_normal_probe_targets_with_query_caches(
            point,
            support,
            &corridor,
            None,
            &stop_point,
            positive_side,
            probe_query_caches,
        );
        if let Some(winding) = try_leaf_probe_family_with_queries(
            point,
            positive_side,
            probes,
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
        )? {
            return Ok(Some(winding));
        }
    }

    Ok(None)
}

fn try_strict_normal_probe_targets_progressively_with_query_caches(
    point: &InteriorLeafPoint,
    positive_side: bool,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    halfspaces.push(normal_stop_halfspace(support, stop_point, positive_side));

    let mut local_unknown = false;
    let report = cached_optional_halfspace_feasibility_report_with(
        &mut probe_query_caches.halfspace_reports,
        &halfspaces,
        &mut local_unknown,
    )?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        *saw_unknown |= local_unknown;
        return Ok(None);
    }

    let extra_planes = normal_probe_extra_planes(point, definition);
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        cached_halfspace_cell_seed_families_from_optional_report_with(
            &mut probe_query_caches.halfspace_seed_families,
            corridor,
            &halfspaces,
            report.as_ref(),
            &mut local_unknown,
        )?;
    let mut seen_direct_seeds = Vec::new();
    let mut seeds = take_new_halfspace_seed_family(seeds, &mut seen_direct_seeds);
    let mut certified_probe_points = Vec::new();
    let mut prioritized_probes: Vec<ProbePoint> = Vec::new();
    let mut deferred_probes: Vec<ProbePoint> = Vec::new();
    let mut saw_any_probe = false;

    let mut queue_probe = |probe: ProbePoint,
                           local_unknown: &mut bool|
     -> HypermeshResult<Option<WindingNumberVector>> {
        saw_any_probe = true;
        let probe_fallback = probe.uncertified_definition_fallback;
        let LeafProbeQueryCaches {
            probe_winding,
            probe_surface,
            probe_reachability,
            axis_ordered_segment_traces,
            plane_replacement_affine,
            plane_replacement_trace_steps,
            plane_replacement_reachability_paths,
            plane_replacement_reachability_steps,
            plane_replacement_no_nested_ordering_warmups,
            definition_cycle_guard_reachability,
            definition_no_step_detour_reachability,
            definition_no_plane_replacement_cycle_guard,
            definition_no_plane_replacement_reachability,
            no_step_detour_target_families,
            definition_no_detour_trace,
            definition_no_detour_reachability,
            direct_probe_reachability,
            detour_target_families,
            ..
        } = probe_query_caches;

        if cached_surface_query_with(probe_surface, &probe.point, || {
            point_lies_on_traced_surface(&probe.point, polygons)
        })? {
            if point.uncertified_definition_fallback || probe_fallback {
                *local_unknown = true;
            }
            return Ok(None);
        }

        match probe_reaches_adjacent_cell_from_interior_without_step_detours_with_caches(
            point,
            &probe,
            support,
            polygons,
            plane_replacement_affine,
            plane_replacement_reachability_paths,
            plane_replacement_reachability_steps,
            definition_no_step_detour_reachability,
            direct_probe_reachability,
        ) {
            Ok(true) => {
                for deferred in deferred_probes.drain(..) {
                    if let Some(winding) = evaluate_leaf_probe_with_query_caches(
                        point,
                        positive_side,
                        deferred,
                        support,
                        ref_point,
                        ref_definitions,
                        ref_wnv,
                        polygons,
                        host_delta_w,
                        probe_surface,
                        probe_reachability,
                        probe_winding,
                        axis_ordered_segment_traces,
                        plane_replacement_affine,
                        plane_replacement_trace_steps,
                        plane_replacement_reachability_paths,
                        plane_replacement_reachability_steps,
                        plane_replacement_no_nested_ordering_warmups,
                        definition_cycle_guard_reachability,
                        definition_no_step_detour_reachability,
                        definition_no_plane_replacement_cycle_guard,
                        definition_no_plane_replacement_reachability,
                        no_step_detour_target_families,
                        definition_no_detour_trace,
                        definition_no_detour_reachability,
                        direct_probe_reachability,
                        detour_target_families,
                        saw_unknown,
                    )? {
                        return Ok(Some(winding));
                    }
                }
                if prioritized_probes.is_empty() {
                    if let Some(winding) = evaluate_leaf_probe_with_query_caches(
                        point,
                        positive_side,
                        probe,
                        support,
                        ref_point,
                        ref_definitions,
                        ref_wnv,
                        polygons,
                        host_delta_w,
                        probe_surface,
                        probe_reachability,
                        probe_winding,
                        axis_ordered_segment_traces,
                        plane_replacement_affine,
                        plane_replacement_trace_steps,
                        plane_replacement_reachability_paths,
                        plane_replacement_reachability_steps,
                        plane_replacement_no_nested_ordering_warmups,
                        definition_cycle_guard_reachability,
                        definition_no_step_detour_reachability,
                        definition_no_plane_replacement_cycle_guard,
                        definition_no_plane_replacement_reachability,
                        no_step_detour_target_families,
                        definition_no_detour_trace,
                        definition_no_detour_reachability,
                        direct_probe_reachability,
                        detour_target_families,
                        saw_unknown,
                    )? {
                        return Ok(Some(winding));
                    }
                } else {
                    prioritized_probes.push(probe);
                }
            }
            Ok(false) => deferred_probes.push(probe),
            Err(HypermeshError::UnknownClassification) => {
                *local_unknown = true;
                deferred_probes.push(probe);
            }
            Err(err) => return Err(err),
        }
        Ok(None)
    };

    for witness in &seeds {
        let probe = match build_probe_point(
            witness,
            corridor,
            support,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            &extra_planes,
            false,
        ) {
            Ok(Some(probe)) => probe,
            Ok(None) => continue,
            Err(HypermeshError::UnknownClassification) => {
                local_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if !probe.uncertified_definition_fallback
            && !certified_probe_points
                .iter()
                .any(|existing| *existing == probe.point)
        {
            certified_probe_points.push(probe.point.clone());
        }
        if let Some(winding) = queue_probe(probe, &mut local_unknown)? {
            return Ok(Some(winding));
        }
    }

    if definition.is_some() || certified_probe_points.is_empty() {
        let mut geometry_strict_seeds = Vec::new();
        local_unknown |= extend_strict_halfspace_seed_families_collect_unknown(
            &mut geometry_strict_seeds,
            [collect_strict_halfspace_seed_family(
                Ok(shifted_geometry_seeds.clone()),
                |candidate| {
                    point_strictly_inside_halfspace_cell_or_unknown(
                        candidate,
                        corridor,
                        &halfspaces,
                    )
                },
            )],
        )?;
        let mut seen_all_direct_seeds = seeds.clone();
        let geometry_strict_seeds =
            take_new_halfspace_seed_family(geometry_strict_seeds, &mut seen_all_direct_seeds);
        for witness in &geometry_strict_seeds {
            let probe = match build_probe_point(
                witness,
                corridor,
                support,
                &halfspaces,
                active_planes_from_optional_report(report.as_ref(), witness),
                &extra_planes,
                false,
            ) {
                Ok(Some(probe)) => probe,
                Ok(None) => continue,
                Err(HypermeshError::UnknownClassification) => {
                    local_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
            if !probe.uncertified_definition_fallback
                && !certified_probe_points
                    .iter()
                    .any(|existing| *existing == probe.point)
            {
                certified_probe_points.push(probe.point.clone());
            }
            if let Some(winding) = queue_probe(probe, &mut local_unknown)? {
                return Ok(Some(winding));
            }
        }
        seeds.extend(geometry_strict_seeds);
    }

    let shifted_geometry_seeds = if definition.is_some() || certified_probe_points.is_empty() {
        shifted_geometry_seeds
    } else {
        Vec::new()
    };
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);
    if seed_family_search_failed_without_any_seed(
        &seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        local_unknown,
    ) {
        *saw_unknown |= local_unknown;
        return Ok(None);
    }
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        normal_probe_shifted_seed_families(
            definition,
            report_witness,
            &certified_probe_points,
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );

    let mut seen_shifted_roots = Vec::new();
    for family in [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds] {
        let fresh = take_new_halfspace_seed_family(family, &mut seen_shifted_roots);
        for seed in fresh {
            let shifted_witnesses =
                match shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, &seed) {
                    Ok(witnesses) => witnesses,
                    Err(HypermeshError::UnknownClassification) => {
                        local_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
            for shifted in &shifted_witnesses {
                let duplicate_certified_direct_probe = certified_probe_points
                    .iter()
                    .any(|point| *point == shifted.point);
                let probe = match build_probe_point_from_shifted_witness(
                    shifted,
                    corridor,
                    support,
                    &extra_planes,
                ) {
                    Ok(Some(probe)) => probe,
                    Ok(None) => continue,
                    Err(HypermeshError::UnknownClassification) => {
                        local_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
                if duplicate_certified_direct_probe && probe.uncertified_definition_fallback {
                    local_unknown = true;
                    continue;
                }
                if let Some(winding) = queue_probe(probe, &mut local_unknown)? {
                    return Ok(Some(winding));
                }
            }
        }
    }

    *saw_unknown |= local_unknown;
    if !saw_any_probe {
        return Ok(None);
    }
    let mut probes = prioritized_probes;
    probes.extend(deferred_probes);
    try_leaf_probe_family_with_queries(
        point,
        positive_side,
        Ok(probes),
        support,
        ref_point,
        ref_definitions,
        ref_wnv,
        polygons,
        host_delta_w,
        probe_query_caches,
        saw_unknown,
    )
}
fn try_strict_normal_probe_report_witness_winding_with_queries(
    point: &InteriorLeafPoint,
    positive_side: bool,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    halfspaces.push(normal_stop_halfspace(support, stop_point, positive_side));

    let report = cached_optional_halfspace_feasibility_report_with(
        &mut probe_query_caches.halfspace_reports,
        &halfspaces,
        saw_unknown,
    )?;
    let Some(report) = report.as_ref() else {
        return Ok(None);
    };
    if report.status != HalfspaceFeasibility::Feasible {
        return Ok(None);
    }
    let Some(witness) = report.witness.as_ref() else {
        return Ok(None);
    };
    let extra_planes = normal_probe_extra_planes(point, definition);
    let probe_result = match build_probe_point(
        witness,
        corridor,
        support,
        &halfspaces,
        active_planes_from_optional_report(Some(report), witness),
        &extra_planes,
        false,
    ) {
        Ok(Some(probe)) => Ok(vec![probe]),
        Ok(None) => Ok(Vec::new()),
        Err(err) => Err(err),
    };
    let winding = try_leaf_probe_family_with_queries(
        point,
        positive_side,
        probe_result,
        support,
        ref_point,
        ref_definitions,
        ref_wnv,
        polygons,
        host_delta_w,
        probe_query_caches,
        saw_unknown,
    );
    winding
}

fn try_strict_normal_seed_winding_with_queries(
    point: &InteriorLeafPoint,
    positive_side: bool,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    halfspaces.push(normal_stop_halfspace(support, stop_point, positive_side));

    let report = cached_optional_halfspace_feasibility_report_with(
        &mut probe_query_caches.halfspace_reports,
        &halfspaces,
        saw_unknown,
    )?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(None);
    }

    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let extra_planes = normal_probe_extra_planes(point, definition);
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        cached_halfspace_cell_seed_families_from_optional_report_with(
            &mut probe_query_caches.halfspace_seed_families,
            corridor,
            &halfspaces,
            report.as_ref(),
            saw_unknown,
        )?;
    let mut seen = Vec::new();
    let mut strict_seeds = take_new_halfspace_seed_family(strict_seeds, &mut seen);
    if let Some(report_witness) = report_witness {
        strict_seeds.retain(|seed| *seed != *report_witness);
    }
    let mut certified_probe_points = Vec::new();

    for witness in &strict_seeds {
        let probe = match build_probe_point(
            witness,
            corridor,
            support,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            &extra_planes,
            false,
        ) {
            Ok(Some(probe)) => probe,
            Ok(None) => continue,
            Err(HypermeshError::UnknownClassification) => {
                *saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if !probe.uncertified_definition_fallback
            && !certified_probe_points
                .iter()
                .any(|existing| *existing == probe.point)
        {
            certified_probe_points.push(probe.point.clone());
        }
        if let Some(winding) = try_leaf_probe_family_with_queries(
            point,
            positive_side,
            Ok(vec![probe]),
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
        )? {
            return Ok(Some(winding));
        }
    }

    let allow_geometry_seed_expansion = definition.is_some();
    if allow_geometry_seed_expansion && certified_probe_points.is_empty() {
        let mut geometry_strict_seeds = Vec::new();
        *saw_unknown |= extend_strict_halfspace_seed_families_collect_unknown(
            &mut geometry_strict_seeds,
            [collect_strict_halfspace_seed_family(
                Ok(shifted_geometry_seeds.clone()),
                |candidate| {
                    point_strictly_inside_halfspace_cell_or_unknown(
                        candidate,
                        corridor,
                        &halfspaces,
                    )
                },
            )],
        )?;
        let mut seen_all_direct_seeds = strict_seeds.clone();
        let geometry_strict_seeds =
            take_new_halfspace_seed_family(geometry_strict_seeds, &mut seen_all_direct_seeds);
        for witness in &geometry_strict_seeds {
            let probe = match build_probe_point(
                witness,
                corridor,
                support,
                &halfspaces,
                active_planes_from_optional_report(report.as_ref(), witness),
                &extra_planes,
                false,
            ) {
                Ok(Some(probe)) => probe,
                Ok(None) => continue,
                Err(HypermeshError::UnknownClassification) => {
                    *saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            };
            if !probe.uncertified_definition_fallback
                && !certified_probe_points
                    .iter()
                    .any(|existing| *existing == probe.point)
            {
                certified_probe_points.push(probe.point.clone());
            }
            if let Some(winding) = try_leaf_probe_family_with_queries(
                point,
                positive_side,
                Ok(vec![probe]),
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
            )? {
                return Ok(Some(winding));
            }
        }
        strict_seeds.extend(geometry_strict_seeds);
    }

    let shifted_geometry_seeds = if allow_geometry_seed_expansion {
        shifted_geometry_seeds
    } else {
        Vec::new()
    };
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );
    if seed_family_search_failed_without_any_seed(
        &strict_seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        *saw_unknown,
    ) {
        return Ok(None);
    }
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        normal_probe_shifted_seed_families(
            definition,
            report_witness,
            &certified_probe_points,
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );
    let mut seen_shifted_roots = Vec::new();
    for family in [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds] {
        let fresh = take_new_halfspace_seed_family(family, &mut seen_shifted_roots);
        for seed in fresh {
            let shifted_witnesses =
                match shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, &seed) {
                    Ok(witnesses) => witnesses,
                    Err(HypermeshError::UnknownClassification) => {
                        *saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
            for shifted in &shifted_witnesses {
                let duplicate_certified_direct_probe = certified_probe_points
                    .iter()
                    .any(|point| *point == shifted.point);
                let probe = match build_probe_point_from_shifted_witness(
                    shifted,
                    corridor,
                    support,
                    &extra_planes,
                ) {
                    Ok(Some(probe)) => probe,
                    Ok(None) => continue,
                    Err(HypermeshError::UnknownClassification) => {
                        *saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
                if duplicate_certified_direct_probe && probe.uncertified_definition_fallback {
                    *saw_unknown = true;
                    continue;
                }
                if let Some(winding) = try_leaf_probe_family_with_queries(
                    point,
                    positive_side,
                    Ok(vec![probe]),
                    support,
                    ref_point,
                    ref_definitions,
                    ref_wnv,
                    polygons,
                    host_delta_w,
                    probe_query_caches,
                    saw_unknown,
                )? {
                    return Ok(Some(winding));
                }
            }
        }
    }

    Ok(None)
}

fn search_adjacent_axis_probe_winding_with_queries(
    point: &InteriorLeafPoint,
    positive_side: bool,
    axis: usize,
    direction_positive: bool,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
    bounds: &Aabb,
    host_delta_w: &[i32],
) -> HypermeshResult<Option<WindingNumberVector>> {
    let (stop_values, local_unknown) = cached_adjacent_axis_probe_stop_values_with(
        &mut probe_query_caches.axis_probe_stop_values,
        &point.point,
        bounds,
        axis,
        direction_positive,
        || {
            adjacent_axis_probe_stop_values_with_queries(
                &point.point,
                bounds,
                polygons,
                axis,
                direction_positive,
                &mut |interior, endpoint, polygon, _axis| {
                    let start_class = classify_point(interior, &polygon.support)?;
                    let endpoint_class = classify_point(endpoint, &polygon.support)?;
                    if start_class == Classification::On {
                        return Ok(Some(interior.clone()));
                    }
                    if endpoint_class == Classification::On {
                        return Ok(Some(endpoint.clone()));
                    }
                    segment_plane_crossing(interior, endpoint, &polygon.support)
                },
                &mut |crossing, polygon| classify_point_in_polygon(crossing, polygon),
            )
        },
    )?;
    *saw_unknown |= local_unknown;

    let definitions = unique_definition_family(&point.planes);
    let start_value = axis_ref(&point.point, axis);
    for stop_value in stop_values {
        if !axis_value_after_start(start_value, &stop_value, direction_positive)? {
            continue;
        }
        let corridor = axis_probe_bounds(&point.point, axis, &stop_value)?;

        for definition in &definitions {
            if !axis_probe_definition_preserves_axis_direction(definition, axis)? {
                continue;
            }
            if let Some(winding) = try_strict_axis_seed_winding_with_queries(
                point,
                positive_side,
                axis,
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
                &corridor,
                Some(definition),
            )? {
                return Ok(Some(winding));
            }
            if let Some(winding) = try_leaf_probe_family_with_queries(
                point,
                positive_side,
                strict_axis_probe_targets(
                    point,
                    support,
                    &corridor,
                    axis,
                    direction_positive,
                    Some(definition),
                ),
                support,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                host_delta_w,
                probe_query_caches,
                saw_unknown,
            )? {
                return Ok(Some(winding));
            }
        }

        if let Some(winding) = try_strict_axis_seed_winding_with_queries(
            point,
            positive_side,
            axis,
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
            &corridor,
            None,
        )? {
            return Ok(Some(winding));
        }
        if let Some(winding) = try_leaf_probe_family_with_queries(
            point,
            positive_side,
            strict_axis_probe_targets(point, support, &corridor, axis, direction_positive, None),
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
        )? {
            return Ok(Some(winding));
        }
    }

    Ok(None)
}

fn try_strict_axis_seed_winding_with_queries(
    point: &InteriorLeafPoint,
    positive_side: bool,
    axis: usize,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));

    let report = cached_optional_halfspace_feasibility_report_with(
        &mut probe_query_caches.halfspace_reports,
        &halfspaces,
        saw_unknown,
    )?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(None);
    }

    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        cached_halfspace_cell_seed_families_from_optional_report_with(
            &mut probe_query_caches.halfspace_seed_families,
            corridor,
            &halfspaces,
            report.as_ref(),
            saw_unknown,
        )?;
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    let mut certified_probe_points = Vec::new();
    for witness in &seeds {
        let probe = match build_axis_probe_point(
            witness,
            point,
            corridor,
            support,
            axis,
            definition,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            false,
        ) {
            Ok(Some(probe)) => probe,
            Ok(None) => continue,
            Err(HypermeshError::UnknownClassification) => {
                *saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if !probe.uncertified_definition_fallback
            && !certified_probe_points
                .iter()
                .any(|existing| *existing == probe.point)
        {
            certified_probe_points.push(probe.point.clone());
        }
        if let Some(winding) = try_leaf_probe_family_with_queries(
            point,
            positive_side,
            Ok(vec![probe]),
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_query_caches,
            saw_unknown,
        )? {
            return Ok(Some(winding));
        }
    }

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            report_witness,
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );

    let mut seen_shifted_roots = Vec::new();
    for family in [strict_shift_seeds, shifted_vertices, shifted_geometry_seeds] {
        let fresh = take_new_halfspace_seed_family(family, &mut seen_shifted_roots);
        for seed in fresh {
            let shifted_witnesses =
                match shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, &seed) {
                    Ok(witnesses) => witnesses,
                    Err(HypermeshError::UnknownClassification) => {
                        *saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
            for shifted in &shifted_witnesses {
                let duplicate_certified_direct_probe = certified_probe_points
                    .iter()
                    .any(|point| *point == shifted.point);
                let probe = match build_axis_probe_point_from_shifted_witness(
                    shifted, point, corridor, support, axis, definition,
                ) {
                    Ok(Some(probe)) => probe,
                    Ok(None) => continue,
                    Err(HypermeshError::UnknownClassification) => {
                        *saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
                if duplicate_certified_direct_probe && probe.uncertified_definition_fallback {
                    *saw_unknown = true;
                    continue;
                }
                if let Some(winding) = try_leaf_probe_family_with_queries(
                    point,
                    positive_side,
                    Ok(vec![probe]),
                    support,
                    ref_point,
                    ref_definitions,
                    ref_wnv,
                    polygons,
                    host_delta_w,
                    probe_query_caches,
                    saw_unknown,
                )? {
                    return Ok(Some(winding));
                }
            }
        }
    }

    Ok(None)
}

fn try_leaf_probe_family_with_queries(
    point: &InteriorLeafPoint,
    positive_side: bool,
    probes: HypermeshResult<Vec<ProbePoint>>,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_query_caches: &mut LeafProbeQueryCaches,
    saw_unknown: &mut bool,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let LeafProbeQueryCaches {
        probe_winding,
        probe_surface,
        probe_reachability,
        axis_ordered_segment_traces,
        plane_replacement_affine,
        plane_replacement_trace_steps,
        plane_replacement_reachability_paths,
        plane_replacement_reachability_steps,
        plane_replacement_no_nested_ordering_warmups,
        definition_cycle_guard_reachability,
        definition_no_step_detour_reachability,
        definition_no_plane_replacement_cycle_guard,
        definition_no_plane_replacement_reachability,
        no_step_detour_target_families,
        definition_no_detour_trace,
        definition_no_detour_reachability,
        direct_probe_reachability,
        detour_target_families,
        ..
    } = probe_query_caches;
    let probes = match probes {
        Ok(probes) => probes,
        Err(HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            return Ok(None);
        }
        Err(err) => return Err(err),
    };
    if probes.is_empty() && point.uncertified_definition_fallback {
        *saw_unknown = true;
    }
    let mut prioritized_probes: Vec<ProbePoint> = Vec::new();
    let mut deferred_probes: Vec<ProbePoint> = Vec::new();
    for probe in probes {
        let probe_fallback = probe.uncertified_definition_fallback;
        if cached_surface_query_with(probe_surface, &probe.point, || {
            point_lies_on_traced_surface(&probe.point, polygons)
        })? {
            if point.uncertified_definition_fallback || probe_fallback {
                *saw_unknown = true;
            }
            continue;
        }

        match probe_reaches_adjacent_cell_from_interior_without_step_detours_with_caches(
            point,
            &probe,
            support,
            polygons,
            plane_replacement_affine,
            plane_replacement_reachability_paths,
            plane_replacement_reachability_steps,
            definition_no_step_detour_reachability,
            direct_probe_reachability,
        ) {
            Ok(true) => {
                for deferred in deferred_probes.drain(..) {
                    if let Some(winding) = evaluate_leaf_probe_with_query_caches(
                        point,
                        positive_side,
                        deferred,
                        support,
                        ref_point,
                        ref_definitions,
                        ref_wnv,
                        polygons,
                        host_delta_w,
                        probe_surface,
                        probe_reachability,
                        probe_winding,
                        axis_ordered_segment_traces,
                        plane_replacement_affine,
                        plane_replacement_trace_steps,
                        plane_replacement_reachability_paths,
                        plane_replacement_reachability_steps,
                        plane_replacement_no_nested_ordering_warmups,
                        definition_cycle_guard_reachability,
                        definition_no_step_detour_reachability,
                        definition_no_plane_replacement_cycle_guard,
                        definition_no_plane_replacement_reachability,
                        no_step_detour_target_families,
                        definition_no_detour_trace,
                        definition_no_detour_reachability,
                        direct_probe_reachability,
                        detour_target_families,
                        saw_unknown,
                    )? {
                        return Ok(Some(winding));
                    }
                }
                prioritized_probes.push(probe);
            }
            Ok(false) => deferred_probes.push(probe),
            Err(HypermeshError::UnknownClassification) => {
                *saw_unknown = true;
                deferred_probes.push(probe);
            }
            Err(err) => return Err(err),
        }
    }

    for probe in prioritized_probes.into_iter().chain(deferred_probes) {
        if let Some(winding) = evaluate_leaf_probe_with_query_caches(
            point,
            positive_side,
            probe,
            support,
            ref_point,
            ref_definitions,
            ref_wnv,
            polygons,
            host_delta_w,
            probe_surface,
            probe_reachability,
            probe_winding,
            axis_ordered_segment_traces,
            plane_replacement_affine,
            plane_replacement_trace_steps,
            plane_replacement_reachability_paths,
            plane_replacement_reachability_steps,
            plane_replacement_no_nested_ordering_warmups,
            definition_cycle_guard_reachability,
            definition_no_step_detour_reachability,
            definition_no_plane_replacement_cycle_guard,
            definition_no_plane_replacement_reachability,
            no_step_detour_target_families,
            definition_no_detour_trace,
            definition_no_detour_reachability,
            direct_probe_reachability,
            detour_target_families,
            saw_unknown,
        )? {
            return Ok(Some(winding));
        }
    }

    Ok(None)
}

fn evaluate_leaf_probe_with_query_caches(
    point: &InteriorLeafPoint,
    positive_side: bool,
    probe: ProbePoint,
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    host_delta_w: &[i32],
    probe_surface: &mut Vec<SurfaceCacheEntry>,
    probe_reachability: &mut Vec<ProbeReachabilityCacheEntry>,
    probe_winding: &mut Vec<ProbeWindingCacheEntry>,
    axis_ordered_segment_traces: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
    plane_replacement_affine: &mut Vec<PlaneReplacementAffineCacheEntry>,
    plane_replacement_trace_steps: &mut Vec<PlaneReplacementStepCacheEntry>,
    plane_replacement_reachability_paths: &mut Vec<PlaneReplacementReachabilityPathCacheEntry>,
    plane_replacement_reachability_steps: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    plane_replacement_no_nested_ordering_warmups: &mut Vec<
        PlaneReplacementNoNestedOrderingWarmupCacheEntry,
    >,
    definition_cycle_guard_reachability: &mut Vec<DefinitionCycleGuardReachabilityCacheEntry>,
    definition_no_step_detour_reachability: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    definition_no_plane_replacement_cycle_guard: &mut Vec<
        DefinitionNoPlaneReplacementCycleGuardCacheEntry,
    >,
    definition_no_plane_replacement_reachability: &mut Vec<
        DefinitionNoPlaneReplacementReachabilityCacheEntry,
    >,
    no_step_detour_target_families: &mut Vec<DetourTargetFamilyCacheEntry>,
    definition_no_detour_trace: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    definition_no_detour_reachability: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    direct_probe_reachability: &mut Vec<DirectProbeReachabilityCacheEntry>,
    detour_target_families: &mut Vec<DetourTargetFamilyCacheEntry>,
    saw_unknown: &mut bool,
) -> HypermeshResult<Option<WindingNumberVector>> {
    let probe_fallback = probe.uncertified_definition_fallback;
    let winding = if cached_surface_query_with(probe_surface, &probe.point, || {
        point_lies_on_traced_surface(&probe.point, polygons)
    })? {
        None
    } else {
        let reaches = cached_probe_reachability_with(probe_reachability, point, &probe, || {
            probe_reaches_adjacent_cell_from_interior_with_caches(
                point,
                &probe,
                support,
                polygons,
                probe_surface,
                plane_replacement_affine,
                plane_replacement_reachability_paths,
                plane_replacement_reachability_steps,
                plane_replacement_no_nested_ordering_warmups,
                definition_cycle_guard_reachability,
                definition_no_step_detour_reachability,
                definition_no_plane_replacement_cycle_guard,
                definition_no_plane_replacement_reachability,
                no_step_detour_target_families,
                definition_no_detour_reachability,
                direct_probe_reachability,
                detour_target_families,
            )
        })?;
        if !reaches {
            None
        } else {
            let mut winding = cached_probe_winding_with(probe_winding, &probe, || {
                trace_probe_winding_with_caches(
                    ref_point,
                    ref_definitions,
                    &probe,
                    ref_wnv,
                    polygons,
                    probe_surface,
                    axis_ordered_segment_traces,
                    plane_replacement_affine,
                    plane_replacement_trace_steps,
                    definition_no_detour_trace,
                    detour_target_families,
                )
            })?;
            if probe.side == Classification::Negative {
                apply_winding_transition_in_place(&mut winding, -1, host_delta_w)?;
            }
            Some(winding)
        }
    };

    match winding {
        Some(winding) => {
            if point.uncertified_definition_fallback || probe_fallback {
                *saw_unknown = true;
                Ok(None)
            } else {
                let _ = positive_side;
                Ok(Some(winding))
            }
        }
        None => {
            if point.uncertified_definition_fallback || probe_fallback {
                *saw_unknown = true;
            }
            Ok(None)
        }
    }
}

fn probe_reaches_adjacent_cell_from_interior_without_step_detours_with_caches(
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    plane_replacement_affine: &mut Vec<PlaneReplacementAffineCacheEntry>,
    plane_replacement_reachability_paths: &mut Vec<PlaneReplacementReachabilityPathCacheEntry>,
    plane_replacement_reachability_steps: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    no_step_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    direct_probe_reachability: &mut Vec<DirectProbeReachabilityCacheEntry>,
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

    cached_definition_no_detour_reachability_with(
        no_step_cache,
        &interior.point,
        &probe.point,
        &start_definitions,
        &end_definitions,
        || {
            probe_reaches_adjacent_cell_with_definition_search(
                &interior.point,
                &probe.point,
                &start_definitions,
                &end_definitions,
                || {
                    cached_direct_probe_reachability_with(
                        direct_probe_reachability,
                        &interior.point,
                        &probe.point,
                        host_support,
                        polygons,
                        || {
                            probe_reaches_adjacent_cell(
                                &interior.point,
                                &probe.point,
                                host_support,
                                polygons,
                            )
                        },
                    )
                },
                |start_definition, end_definition| {
                    plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches(
                        start_definition,
                        end_definition,
                        host_support,
                        polygons,
                        plane_replacement_affine,
                        plane_replacement_reachability_paths,
                        plane_replacement_reachability_steps,
                    )
                },
            )
        },
    )
}

#[cfg(test)]
fn cached_adjacent_normal_probes_with(
    cache: &mut Vec<NormalProbeFamilyCacheEntry>,
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    positive_side: bool,
    query: impl FnOnce() -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.interior_point == interior.point
            && definition_families_match_as_sets(&existing.interior_planes, &interior.planes)
            && existing.support == *support
            && existing.bounds == *bounds
            && existing.positive_side == positive_side
    }) {
        return existing.probes.clone();
    }

    let probes = query();
    cache.push(NormalProbeFamilyCacheEntry {
        interior_point: interior.point.clone(),
        interior_planes: interior.planes.clone(),
        support: support.clone(),
        bounds: bounds.clone(),
        positive_side,
        probes: probes.clone(),
    });
    probes
}

#[cfg(test)]
fn cached_adjacent_axis_probes_with(
    cache: &mut Vec<AxisProbeFamilyCacheEntry>,
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    axis: usize,
    direction_positive: bool,
    query: impl FnOnce() -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.interior_point == interior.point
            && definition_families_match_as_sets(&existing.interior_planes, &interior.planes)
            && existing.support == *support
            && existing.bounds == *bounds
            && existing.axis == axis
            && existing.direction_positive == direction_positive
    }) {
        return existing.probes.clone();
    }

    let probes = query();
    cache.push(AxisProbeFamilyCacheEntry {
        interior_point: interior.point.clone(),
        interior_planes: interior.planes.clone(),
        support: support.clone(),
        bounds: bounds.clone(),
        axis,
        direction_positive,
        probes: probes.clone(),
    });
    probes
}

fn cached_adjacent_normal_probe_stop_values_with(
    cache: &mut Vec<NormalProbeStopCacheEntry>,
    interior: &Point3,
    direction: &Point3,
    support: &Plane,
    bounds: &Aabb,
    query: impl FnOnce() -> HypermeshResult<(Vec<Real>, bool)>,
) -> HypermeshResult<(Vec<Real>, bool)> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        existing.interior_point == *interior
            && existing.direction == *direction
            && existing.support == *support
            && existing.bounds == *bounds
    }) {
        return Ok((existing.stop_values.clone()?, existing.saw_unknown));
    }

    let (stop_values, saw_unknown) = query()?;
    cache.push(NormalProbeStopCacheEntry {
        interior_point: interior.clone(),
        direction: direction.clone(),
        support: support.clone(),
        bounds: bounds.clone(),
        saw_unknown,
        stop_values: Ok(stop_values.clone()),
    });
    Ok((stop_values, saw_unknown))
}

fn cached_adjacent_axis_probe_stop_values_with(
    cache: &mut Vec<AxisProbeStopCacheEntry>,
    interior: &Point3,
    bounds: &Aabb,
    axis: usize,
    direction_positive: bool,
    query: impl FnOnce() -> HypermeshResult<(Vec<Real>, bool)>,
) -> HypermeshResult<(Vec<Real>, bool)> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        existing.interior_point == *interior
            && existing.bounds == *bounds
            && existing.axis == axis
            && existing.direction_positive == direction_positive
    }) {
        return Ok((existing.stop_values.clone()?, existing.saw_unknown));
    }

    let (stop_values, saw_unknown) = query()?;
    cache.push(AxisProbeStopCacheEntry {
        interior_point: interior.clone(),
        bounds: bounds.clone(),
        axis,
        direction_positive,
        saw_unknown,
        stop_values: Ok(stop_values.clone()),
    });
    Ok((stop_values, saw_unknown))
}

fn cached_optional_halfspace_feasibility_report_with(
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

fn cached_halfspace_cell_seed_families_from_optional_report_with(
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

#[cfg(test)]
fn cached_bounded_probes_from_interior_with(
    cache: &mut Vec<ProbePointFamilyCacheEntry>,
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    positive_side: bool,
    query: impl FnOnce() -> HypermeshResult<Vec<ProbePoint>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.interior_point == interior.point
            && definition_families_match_as_sets(&existing.interior_planes, &interior.planes)
            && existing.support == *support
            && existing.bounds == *bounds
            && existing.positive_side == positive_side
    }) {
        return existing.probes.clone();
    }

    let probes = query();
    cache.push(ProbePointFamilyCacheEntry {
        interior_point: interior.point.clone(),
        interior_planes: interior.planes.clone(),
        support: support.clone(),
        bounds: bounds.clone(),
        positive_side,
        probes: probes.clone(),
    });
    probes
}

fn cached_probe_winding_with(
    cache: &mut Vec<ProbeWindingCacheEntry>,
    probe: &ProbePoint,
    trace: impl FnOnce() -> HypermeshResult<WindingNumberVector>,
) -> HypermeshResult<WindingNumberVector> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
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

fn probe_reachability_cache_entry_matches(
    existing: &ProbeReachabilityCacheEntry,
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
) -> bool {
    existing.interior_point == interior.point
        && definition_families_match_as_sets(&existing.interior_planes, &interior.planes)
        && existing.probe_point == probe.point
        && definition_families_match_as_sets(&existing.probe_planes, &probe.planes)
}

fn begin_probe_reachability_result(
    cache: &mut Vec<ProbeReachabilityCacheEntry>,
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
) -> usize {
    cache.push(ProbeReachabilityCacheEntry {
        interior_point: interior.point.clone(),
        interior_planes: interior.planes.clone(),
        probe_point: probe.point.clone(),
        probe_planes: probe.planes.clone(),
        reachable: Err(HypermeshError::UnknownClassification),
    });
    cache.len() - 1
}

fn cached_probe_reachability_with(
    cache: &mut Vec<ProbeReachabilityCacheEntry>,
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
    query: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache
        .iter()
        .rev()
        .find(|existing| probe_reachability_cache_entry_matches(existing, interior, probe))
    {
        return existing.reachable.clone();
    }

    let cache_index = begin_probe_reachability_result(cache, interior, probe);
    let reachable = query();
    cache[cache_index].reachable = reachable.clone();
    reachable
}

fn cached_direct_probe_reachability_with(
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

#[cfg(test)]
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
                    Ok(Some(winding)) => {
                        if point.uncertified_definition_fallback || probe_fallback {
                            saw_unknown = true;
                            continue;
                        }
                        return Ok(Some(winding));
                    }
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

#[cfg(test)]
fn trace_probe_winding(
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    probe: &ProbePoint,
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<WindingNumberVector> {
    let mut surface_cache = Vec::new();
    let mut axis_ordered_segment_traces = Vec::new();
    let mut plane_replacement_affine = Vec::new();
    let mut plane_replacement_trace_steps = Vec::new();
    let mut definition_no_detour_trace = Vec::new();
    let mut detour_target_families = Vec::new();
    trace_probe_winding_with_caches(
        ref_point,
        ref_definitions,
        probe,
        ref_wnv,
        polygons,
        &mut surface_cache,
        &mut axis_ordered_segment_traces,
        &mut plane_replacement_affine,
        &mut plane_replacement_trace_steps,
        &mut definition_no_detour_trace,
        &mut detour_target_families,
    )
}

fn trace_probe_winding_with_caches(
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    probe: &ProbePoint,
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    axis_ordered_segment_traces: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
    plane_replacement_affine: &mut Vec<PlaneReplacementAffineCacheEntry>,
    plane_replacement_trace_steps: &mut Vec<PlaneReplacementStepCacheEntry>,
    definition_no_detour_trace: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_families: &mut Vec<DetourTargetFamilyCacheEntry>,
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

    trace_segment_from_definitions_with_step_detoured_plane_replacement_with_caches(
        ref_point,
        &probe.point,
        ref_wnv,
        polygons,
        ref_definitions,
        &probe_definitions,
        surface_cache,
        axis_ordered_segment_traces,
        plane_replacement_affine,
        plane_replacement_trace_steps,
        definition_no_detour_trace,
        detour_target_families,
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
    let mut surface_cache = Vec::new();
    let mut axis_ordered_segment_traces = Vec::new();
    let mut plane_replacement_affine = Vec::new();
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
    )
}

fn trace_segment_from_definitions_with_step_detoured_plane_replacement_with_caches(
    start: &Point3,
    end: &Point3,
    winding: &[i32],
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    axis_ordered_segment_traces: &mut Vec<AxisOrderedSegmentTraceCacheEntry>,
    plane_replacement_affine: &mut Vec<PlaneReplacementAffineCacheEntry>,
    plane_replacement_trace_steps: &mut Vec<PlaneReplacementStepCacheEntry>,
    no_detour_cache: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
) -> HypermeshResult<WindingNumberVector> {
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

fn initial_visited_definition_points(
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

fn visited_definition_family_contains(
    points: &[VisitedDefinitionPoint],
    candidate: &Point3,
    definitions: &[[Plane; 3]],
) -> bool {
    points.iter().any(|point| {
        point.point == *candidate
            && definition_families_match_as_sets(&point.definitions, definitions)
    })
}

fn visited_definition_points_match_as_sets(
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

fn visited_definition_points_subset_of(
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

fn normalized_cycle_guard_visited_points(
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
                && definition_families_match_as_sets(&visited.definitions, start_definitions))
                && !(visited.point == *end
                    && definition_families_match_as_sets(&visited.definitions, end_definitions))
        })
        .cloned()
        .collect()
}

fn cached_definition_cycle_guard_result(
    cache: &[DefinitionCycleGuardReachabilityCacheEntry],
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
) -> Option<HypermeshResult<bool>> {
    let normalized_visited_points = normalized_cycle_guard_visited_points(
        start,
        end,
        start_definitions,
        end_definitions,
        visited_points,
    );
    if let Some(existing) = cache.iter().rev().find(|existing| {
        visited_definition_points_match_as_sets(
            &existing.visited_points,
            &normalized_visited_points,
        ) && ((existing.start == *start
            && existing.end == *end
            && definition_families_match_as_sets(&existing.start_definitions, start_definitions)
            && definition_families_match_as_sets(&existing.end_definitions, end_definitions))
            || (existing.start == *end
                && existing.end == *start
                && definition_families_match_as_sets(&existing.start_definitions, end_definitions)
                && definition_families_match_as_sets(&existing.end_definitions, start_definitions)))
    }) {
        return Some(existing.result.clone());
    }

    cache.iter().rev().find_map(|existing| {
        let same_direction = existing.start == *start
            && existing.end == *end
            && definition_families_match_as_sets(&existing.start_definitions, start_definitions)
            && definition_families_match_as_sets(&existing.end_definitions, end_definitions);
        let reversed_direction = existing.start == *end
            && existing.end == *start
            && definition_families_match_as_sets(&existing.start_definitions, end_definitions)
            && definition_families_match_as_sets(&existing.end_definitions, start_definitions);
        if !same_direction && !reversed_direction {
            return None;
        }
        match &existing.result {
            Ok(false)
                if visited_definition_points_subset_of(
                    &existing.visited_points,
                    &normalized_visited_points,
                ) =>
            {
                Some(existing.result.clone())
            }
            Ok(true)
                if visited_definition_points_subset_of(
                    &normalized_visited_points,
                    &existing.visited_points,
                ) =>
            {
                Some(existing.result.clone())
            }
            _ => None,
        }
    })
}

fn begin_definition_cycle_guard_result(
    cache: &mut Vec<DefinitionCycleGuardReachabilityCacheEntry>,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
) -> usize {
    let normalized_visited_points = normalized_cycle_guard_visited_points(
        start,
        end,
        start_definitions,
        end_definitions,
        visited_points,
    );
    cache.push(DefinitionCycleGuardReachabilityCacheEntry {
        start: start.clone(),
        end: end.clone(),
        start_definitions: start_definitions.to_vec(),
        end_definitions: end_definitions.to_vec(),
        visited_points: normalized_visited_points,
        result: Err(HypermeshError::UnknownClassification),
    });
    cache.len() - 1
}

fn cached_definition_no_plane_replacement_cycle_guard_result(
    cache: &[DefinitionNoPlaneReplacementCycleGuardCacheEntry],
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
) -> Option<HypermeshResult<bool>> {
    let normalized_visited_points = normalized_cycle_guard_visited_points(
        start,
        end,
        start_definitions,
        end_definitions,
        visited_points,
    );
    if let Some(existing) = cache.iter().rev().find(|existing| {
        visited_definition_points_match_as_sets(
            &existing.visited_points,
            &normalized_visited_points,
        ) && ((existing.start == *start
            && existing.end == *end
            && definition_families_match_as_sets(&existing.start_definitions, start_definitions)
            && definition_families_match_as_sets(&existing.end_definitions, end_definitions))
            || (existing.start == *end
                && existing.end == *start
                && definition_families_match_as_sets(&existing.start_definitions, end_definitions)
                && definition_families_match_as_sets(&existing.end_definitions, start_definitions)))
    }) {
        return Some(existing.result.clone());
    }

    cache.iter().rev().find_map(|existing| {
        let same_direction = existing.start == *start
            && existing.end == *end
            && definition_families_match_as_sets(&existing.start_definitions, start_definitions)
            && definition_families_match_as_sets(&existing.end_definitions, end_definitions);
        let reversed_direction = existing.start == *end
            && existing.end == *start
            && definition_families_match_as_sets(&existing.start_definitions, end_definitions)
            && definition_families_match_as_sets(&existing.end_definitions, start_definitions);
        if !same_direction && !reversed_direction {
            return None;
        }

        match &existing.result {
            Ok(false)
                if visited_definition_points_subset_of(
                    &existing.visited_points,
                    &normalized_visited_points,
                ) =>
            {
                Some(Ok(false))
            }
            Ok(true)
                if visited_definition_points_subset_of(
                    &normalized_visited_points,
                    &existing.visited_points,
                ) =>
            {
                Some(Ok(true))
            }
            _ => None,
        }
    })
}

fn begin_definition_no_plane_replacement_cycle_guard_result(
    cache: &mut Vec<DefinitionNoPlaneReplacementCycleGuardCacheEntry>,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    visited_points: &[VisitedDefinitionPoint],
) -> usize {
    let normalized_visited_points = normalized_cycle_guard_visited_points(
        start,
        end,
        start_definitions,
        end_definitions,
        visited_points,
    );
    cache.push(DefinitionNoPlaneReplacementCycleGuardCacheEntry {
        start: start.clone(),
        end: end.clone(),
        start_definitions: start_definitions.to_vec(),
        end_definitions: end_definitions.to_vec(),
        visited_points: normalized_visited_points,
        result: Err(HypermeshError::UnknownClassification),
    });
    cache.len() - 1
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

#[cfg(test)]
fn probe_reaches_adjacent_cell_from_interior(
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let mut surface_cache = Vec::new();
    let mut plane_replacement_affine = Vec::new();
    let mut plane_replacement_reachability_paths = Vec::new();
    let mut plane_replacement_reachability_steps = Vec::new();
    let mut plane_replacement_no_nested_ordering_warmups = Vec::new();
    let mut definition_cycle_guard_reachability = Vec::new();
    let mut definition_no_step_detour_reachability = Vec::new();
    let mut definition_no_plane_replacement_cycle_guard = Vec::new();
    let mut definition_no_plane_replacement_reachability = Vec::new();
    let mut no_step_detour_target_families = Vec::new();
    let mut definition_no_detour_reachability = Vec::new();
    let mut direct_probe_reachability = Vec::new();
    let mut detour_target_families = Vec::new();
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
        &mut definition_cycle_guard_reachability,
        &mut definition_no_step_detour_reachability,
        &mut definition_no_plane_replacement_cycle_guard,
        &mut definition_no_plane_replacement_reachability,
        &mut no_step_detour_target_families,
        &mut definition_no_detour_reachability,
        &mut direct_probe_reachability,
        &mut detour_target_families,
    )
}

fn probe_reaches_adjacent_cell_from_interior_with_caches(
    interior: &InteriorLeafPoint,
    probe: &ProbePoint,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    plane_replacement_affine: &mut Vec<PlaneReplacementAffineCacheEntry>,
    plane_replacement_reachability_paths: &mut Vec<PlaneReplacementReachabilityPathCacheEntry>,
    plane_replacement_reachability_steps: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    plane_replacement_no_nested_ordering_warmups: &mut Vec<
        PlaneReplacementNoNestedOrderingWarmupCacheEntry,
    >,
    definition_cycle_guard_reachability: &mut Vec<DefinitionCycleGuardReachabilityCacheEntry>,
    definition_no_step_detour_reachability: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    definition_no_plane_replacement_cycle_guard: &mut Vec<
        DefinitionNoPlaneReplacementCycleGuardCacheEntry,
    >,
    definition_no_plane_replacement_reachability: &mut Vec<
        DefinitionNoPlaneReplacementReachabilityCacheEntry,
    >,
    no_step_detour_target_families: &mut Vec<DetourTargetFamilyCacheEntry>,
    definition_no_detour_reachability: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    direct_probe_reachability: &mut Vec<DirectProbeReachabilityCacheEntry>,
    detour_target_families: &mut Vec<DetourTargetFamilyCacheEntry>,
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

    probe_reaches_adjacent_cell_with_cycle_guard_with_caches(
        &interior.point,
        &probe.point,
        host_support,
        polygons,
        &start_definitions,
        &end_definitions,
        surface_cache,
        plane_replacement_affine,
        plane_replacement_reachability_paths,
        plane_replacement_reachability_steps,
        plane_replacement_no_nested_ordering_warmups,
        definition_cycle_guard_reachability,
        definition_no_step_detour_reachability,
        definition_no_plane_replacement_cycle_guard,
        definition_no_plane_replacement_reachability,
        no_step_detour_target_families,
        definition_no_detour_reachability,
        direct_probe_reachability,
        detour_target_families,
    )
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
    let mut plane_replacement_affine = Vec::new();
    let mut plane_replacement_reachability_paths = Vec::new();
    let mut plane_replacement_reachability_steps = Vec::new();
    let mut plane_replacement_no_nested_ordering_warmups = Vec::new();
    let mut definition_cycle_guard_reachability = Vec::new();
    let mut no_step_cache = Vec::new();
    let mut no_plane_replacement_cycle_guard_cache = Vec::new();
    let mut no_plane_replacement_cache = Vec::new();
    let mut no_step_detour_target_cache = Vec::new();
    let mut no_detour_cache = Vec::new();
    let mut direct_probe_reachability_cache = Vec::new();
    let mut detour_target_cache = Vec::new();
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
        &mut definition_cycle_guard_reachability,
        &mut no_step_cache,
        &mut no_plane_replacement_cycle_guard_cache,
        &mut no_plane_replacement_cache,
        &mut no_step_detour_target_cache,
        &mut no_detour_cache,
        &mut direct_probe_reachability_cache,
        &mut detour_target_cache,
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
    plane_replacement_affine: &mut Vec<PlaneReplacementAffineCacheEntry>,
    plane_replacement_reachability_paths: &mut Vec<PlaneReplacementReachabilityPathCacheEntry>,
    plane_replacement_reachability_steps: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    plane_replacement_no_nested_ordering_warmups: &mut Vec<
        PlaneReplacementNoNestedOrderingWarmupCacheEntry,
    >,
    definition_cycle_guard_reachability: &mut Vec<DefinitionCycleGuardReachabilityCacheEntry>,
    no_step_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    no_plane_replacement_cycle_guard_cache: &mut Vec<
        DefinitionNoPlaneReplacementCycleGuardCacheEntry,
    >,
    no_plane_replacement_cache: &mut Vec<DefinitionNoPlaneReplacementReachabilityCacheEntry>,
    no_step_detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
    no_detour_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    direct_probe_reachability_cache: &mut Vec<DirectProbeReachabilityCacheEntry>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
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
                no_step_cache,
                no_plane_replacement_cycle_guard_cache,
                no_plane_replacement_cache,
                no_step_detour_target_cache,
                no_detour_cache,
                direct_probe_reachability_cache,
            )
        };
    let has_top_level_detour_cache_hit =
        cached_detour_target_family(&*detour_target_cache, start, end).is_some();
    let mut detours_for = |start: &Point3, end: &Point3| {
        cached_detour_target_family_with(detour_target_cache, start, end, || {
            interior_box_detour_targets(start, end, polygons)
        })
    };
    let direct_result = trace_without_detours(start, end, start_definitions, end_definitions);
    let no_detour_unknown = match direct_result {
        Ok(true) => {
            definition_cycle_guard_reachability[cache_index].result = Ok(true);
            return Ok(true);
        }
        Ok(false) => false,
        Err(HypermeshError::UnknownClassification) => true,
        Err(err) => {
            definition_cycle_guard_reachability[cache_index].result = Err(err.clone());
            return Err(err);
        }
    };

    let detour_result = if has_top_level_detour_cache_hit {
        probe_reaches_adjacent_cell_via_detours_with_cycle_guard_with_surface_query(
            start,
            end,
            polygons,
            start_definitions,
            end_definitions,
            &visited_points,
            surface_cache,
            &mut |point| point_lies_on_traced_surface(point, polygons),
            &mut trace_without_detours,
            &mut detours_for,
        )?
    } else {
        probe_reaches_adjacent_cell_via_interior_box_detours_progressively_with_surface_query(
            start,
            end,
            polygons,
            start_definitions,
            end_definitions,
            &visited_points,
            surface_cache,
            &mut |point| point_lies_on_traced_surface(point, polygons),
            &mut trace_without_detours,
            &mut detours_for,
        )?
    };

    if detour_result {
        definition_cycle_guard_reachability[cache_index].result = Ok(true);
        Ok(true)
    } else if no_detour_unknown {
        definition_cycle_guard_reachability[cache_index].result =
            Err(HypermeshError::UnknownClassification);
        Err(HypermeshError::UnknownClassification)
    } else {
        definition_cycle_guard_reachability[cache_index].result = Ok(false);
        Ok(false)
    }
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
    if let Some(existing) = cache.iter().rev().find(|existing| {
        (existing.start == *start
            && existing.end == *end
            && definition_families_match_as_sets(&existing.start_definitions, start_definitions)
            && definition_families_match_as_sets(&existing.end_definitions, end_definitions))
            || (existing.start == *end
                && existing.end == *start
                && definition_families_match_as_sets(&existing.start_definitions, end_definitions)
                && definition_families_match_as_sets(&existing.end_definitions, start_definitions))
    }) {
        return existing.result.clone();
    }

    cache.push(DefinitionNoDetourReachabilityCacheEntry {
        start: start.clone(),
        end: end.clone(),
        start_definitions: start_definitions.to_vec(),
        end_definitions: end_definitions.to_vec(),
        result: Err(HypermeshError::UnknownClassification),
    });
    let cache_index = cache.len() - 1;
    let result = trace();
    cache[cache_index].result = result.clone();
    result
}

fn cached_definition_no_plane_replacement_reachability_with(
    cache: &mut Vec<DefinitionNoPlaneReplacementReachabilityCacheEntry>,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    trace: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        (existing.start == *start
            && existing.end == *end
            && definition_families_match_as_sets(&existing.start_definitions, start_definitions)
            && definition_families_match_as_sets(&existing.end_definitions, end_definitions))
            || (existing.start == *end
                && existing.end == *start
                && definition_families_match_as_sets(&existing.start_definitions, end_definitions)
                && definition_families_match_as_sets(&existing.end_definitions, start_definitions))
    }) {
        return existing.result.clone();
    }

    cache.push(DefinitionNoPlaneReplacementReachabilityCacheEntry {
        start: start.clone(),
        end: end.clone(),
        start_definitions: start_definitions.to_vec(),
        end_definitions: end_definitions.to_vec(),
        result: Err(HypermeshError::UnknownClassification),
    });
    let cache_index = cache.len() - 1;
    let result = trace();
    cache[cache_index].result = result.clone();
    result
}

#[cfg(test)]
#[allow(dead_code)]
fn probe_reaches_adjacent_cell_with_cycle_guard_impl(
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

fn probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
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

fn probe_reaches_adjacent_cell_via_interior_box_detours_progressively_with_surface_query(
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
    let (intervals, mut saw_unknown) = interior_box_axis_intervals_with_surface_queries(
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
        Ok(true) => {
            if detour.uncertified_definition_fallback {
                *saw_unknown = true;
                Ok(false)
            } else {
                Ok(true)
            }
        }
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
        &initial_visited_definition_points(start, start_definitions, end, end_definitions),
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
            Ok(true) => {
                if detour.uncertified_definition_fallback {
                    saw_unknown = true;
                    continue;
                }
                return Ok(true);
            }
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
    let mut affine_cache = Vec::new();
    let mut path_cache = Vec::new();
    let mut step_cache = Vec::new();
    let mut no_nested_ordering_warmup_cache = Vec::new();
    let mut no_step_cache = Vec::new();
    let mut no_plane_replacement_cycle_guard_cache = Vec::new();
    let mut no_plane_replacement_cache = Vec::new();
    let mut detour_target_cache = Vec::new();
    let mut no_detour_cache = Vec::new();
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
        &mut no_step_cache,
        &mut no_plane_replacement_cycle_guard_cache,
        &mut no_plane_replacement_cache,
        &mut detour_target_cache,
        &mut no_detour_cache,
        &mut direct_probe_reachability_cache,
    )
}

fn probe_reaches_adjacent_cell_with_definitions_no_detours_with_caches(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    path_cache: &mut Vec<PlaneReplacementReachabilityPathCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    no_nested_ordering_warmup_cache: &mut Vec<PlaneReplacementNoNestedOrderingWarmupCacheEntry>,
    no_step_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    no_plane_replacement_cycle_guard_cache: &mut Vec<
        DefinitionNoPlaneReplacementCycleGuardCacheEntry,
    >,
    no_plane_replacement_cache: &mut Vec<DefinitionNoPlaneReplacementReachabilityCacheEntry>,
    no_step_detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
    no_detour_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    direct_probe_reachability_cache: &mut Vec<DirectProbeReachabilityCacheEntry>,
) -> HypermeshResult<bool> {
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
            )
        },
    )? {
        DefinitionSearchPrecheckOutcome::Reaches => Ok(true),
        DefinitionSearchPrecheckOutcome::Search(plan) => {
            let mut saw_unknown = plan.unknown_if_no_match;
            for (start_index, end_index) in plan.ordered_pairs {
                match plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement_with_caches(
                    &plan.start_definitions[start_index],
                    &plan.end_definitions[end_index],
                    host_support,
                    polygons,
                    affine_cache,
                    path_cache,
                    step_cache,
                    no_nested_ordering_warmup_cache,
                    no_step_cache,
                    no_detour_cache,
                    no_plane_replacement_cycle_guard_cache,
                    no_plane_replacement_cache,
                    no_step_detour_target_cache,
                    direct_probe_reachability_cache,
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
    let mut affine_cache = Vec::new();
    let mut path_cache = Vec::new();
    let mut step_cache = Vec::new();
    let mut no_step_cache = Vec::new();
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
    )
}

fn probe_reaches_adjacent_cell_with_definitions_no_step_detours_with_caches(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    path_cache: &mut Vec<PlaneReplacementReachabilityPathCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    no_step_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    direct_probe_reachability_cache: &mut Vec<DirectProbeReachabilityCacheEntry>,
) -> HypermeshResult<bool> {
    let mut start_definitions = start_definitions.to_vec();
    append_definition_if_missing(&mut start_definitions, axis_plane_definition(start));
    let mut end_definitions = end_definitions.to_vec();
    append_definition_if_missing(&mut end_definitions, axis_plane_definition(end));
    let start_definitions = unique_definition_family(&start_definitions);
    let end_definitions = unique_definition_family(&end_definitions);
    cached_definition_no_detour_reachability_with(
        no_step_cache,
        start,
        end,
        &start_definitions,
        &end_definitions,
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
                &start_definitions,
                &end_definitions,
                host_support,
                polygons,
                affine_cache,
                direct_probe_reachability_cache,
            )?;

            let mut saw_unknown = direct_unknown;
            for (start_index, end_index) in ordered_pairs {
                match plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches(
                    &start_definitions[start_index],
                    &end_definitions[end_index],
                    host_support,
                    polygons,
                    affine_cache,
                    path_cache,
                    step_cache,
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
    )
}

fn ordered_definition_pairs_by_no_step_precheck_with(
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
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
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
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
    let mut start_definitions = start_definitions.to_vec();
    append_definition_if_missing(&mut start_definitions, axis_plane_definition(start));
    let mut end_definitions = end_definitions.to_vec();
    append_definition_if_missing(&mut end_definitions, axis_plane_definition(end));
    let start_definitions = unique_definition_family(&start_definitions);
    let end_definitions = unique_definition_family(&end_definitions);

    let mut ordered_pairs = Vec::new();
    let mut saw_unknown = direct_unknown;

    for (start_index, start_definition) in start_definitions.iter().enumerate() {
        for (end_index, end_definition) in end_definitions.iter().enumerate() {
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
            start_definitions,
            end_definitions,
            ordered_pairs: ordered_pairs
                .into_iter()
                .map(|(_, start_index, end_index)| (start_index, end_index))
                .collect(),
            unknown_if_no_match: saw_unknown,
        },
    ))
}

#[cfg(test)]
fn probe_reaches_adjacent_cell_with_definition_search_preferring_precheck(
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
fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions(
    start: &Point3,
    end: &Point3,
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> HypermeshResult<bool> {
    let mut no_detour_cache = Vec::new();
    let mut no_plane_replacement_cycle_guard_cache = Vec::new();
    let mut no_plane_replacement_cache = Vec::new();
    let mut detour_target_cache = Vec::new();
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
        start,
        end,
        polygons,
        start_definitions,
        end_definitions,
        &mut no_detour_cache,
        &mut no_plane_replacement_cycle_guard_cache,
        &mut no_plane_replacement_cache,
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

#[cfg(test)]
fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    no_detour_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    no_plane_replacement_cycle_guard_cache: &mut Vec<
        DefinitionNoPlaneReplacementCycleGuardCacheEntry,
    >,
    no_plane_replacement_cache: &mut Vec<DefinitionNoPlaneReplacementReachabilityCacheEntry>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
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
        detour_target_cache,
        false,
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
    no_detour_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    no_plane_replacement_cycle_guard_cache: &mut Vec<
        DefinitionNoPlaneReplacementCycleGuardCacheEntry,
    >,
    no_plane_replacement_cache: &mut Vec<DefinitionNoPlaneReplacementReachabilityCacheEntry>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
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
        detour_target_cache,
        true,
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
    no_detour_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    no_plane_replacement_cycle_guard_cache: &mut Vec<
        DefinitionNoPlaneReplacementCycleGuardCacheEntry,
    >,
    no_plane_replacement_cache: &mut Vec<DefinitionNoPlaneReplacementReachabilityCacheEntry>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
    progressive_interior_box_detours: bool,
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
    cached_definition_no_plane_replacement_reachability_with(
        no_plane_replacement_cache,
        start,
        end,
        start_definitions,
        end_definitions,
        || {
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_mode(
                start,
                end,
                polygons,
                &initial_visited_definition_points(start, start_definitions, end, end_definitions),
                start_definitions,
                end_definitions,
                progressive_interior_box_detours,
                no_plane_replacement_cycle_guard_cache,
                &mut trace_without_detours,
                detour_target_cache,
                &mut detours_for_query,
            )
        },
    )
}

#[cfg(test)]
fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[VisitedDefinitionPoint],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detours_for_query: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut no_plane_replacement_cycle_guard_cache = Vec::new();
    let mut detour_target_cache = Vec::new();
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_mode(
        start,
        end,
        polygons,
        visited_points,
        start_definitions,
        end_definitions,
        false,
        &mut no_plane_replacement_cycle_guard_cache,
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
    no_plane_replacement_cycle_guard_cache: &mut Vec<
        DefinitionNoPlaneReplacementCycleGuardCacheEntry,
    >,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
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
        &mut surface_cache,
        &mut |point| point_lies_on_traced_surface(point, polygons),
        trace_without_detours,
        detour_target_cache,
        detours_for_query,
    )
}

#[cfg(test)]
fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[VisitedDefinitionPoint],
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
    detours_for_query: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let mut no_plane_replacement_cycle_guard_cache = Vec::new();
    let mut detour_target_cache = Vec::new();
    probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query_mode(
        start,
        end,
        polygons,
        visited_points,
        start_definitions,
        end_definitions,
        false,
        &mut no_plane_replacement_cycle_guard_cache,
        surface_cache,
        surface_query,
        trace_without_detours,
        &mut detour_target_cache,
        detours_for_query,
    )
}

fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query_mode(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[VisitedDefinitionPoint],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    progressive_interior_box_detours: bool,
    no_plane_replacement_cycle_guard_cache: &mut Vec<
        DefinitionNoPlaneReplacementCycleGuardCacheEntry,
    >,
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
    detours_for_query: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
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
                && cached_detour_target_family(&*detour_target_cache, start, end).is_none()
            {
                match probe_reaches_adjacent_cell_via_interior_box_detours_without_plane_replacement_progressively_with_surface_query(
                    start,
                    end,
                    polygons,
                    visited_points,
                    start_definitions,
                    end_definitions,
                    progressive_interior_box_detours,
                    no_plane_replacement_cycle_guard_cache,
                    surface_cache,
                    surface_query,
                    trace_without_detours,
                    detour_target_cache,
                    detours_for_query,
                ) {
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
                    cached_detour_target_family_with(detour_target_cache, start, end, || {
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
    no_plane_replacement_cycle_guard_cache[cache_index].result = result.clone();
    result
}

fn evaluate_probe_detour_target_without_plane_replacement_with_surface_query(
    detour: &DetourTarget,
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[VisitedDefinitionPoint],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    progressive_interior_box_detours: bool,
    no_plane_replacement_cycle_guard_cache: &mut Vec<
        DefinitionNoPlaneReplacementCycleGuardCacheEntry,
    >,
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
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
                            definition_transition: bool|
     -> HypermeshResult<Option<bool>> {
        match if definition_transition {
            trace_without_detours(
                leg_start,
                leg_end,
                leg_start_definitions,
                leg_end_definitions,
            )
        } else {
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query_mode(
                leg_start,
                leg_end,
                polygons,
                &next_visited_points,
                leg_start_definitions,
                leg_end_definitions,
                progressive_interior_box_detours,
                no_plane_replacement_cycle_guard_cache,
                surface_cache,
                surface_query,
                trace_without_detours,
                detour_target_cache,
                detours_for_query,
            )
        } {
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
        )?
    } else {
        evaluate_leg(
            start,
            &detour.point,
            start_definitions,
            &detour.definitions,
            start_definition_transition,
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
        )?
    } else {
        evaluate_leg(
            &detour.point,
            end,
            &detour.definitions,
            end_definitions,
            end_definition_transition,
        )?
    };
    match second_result {
        Some(true) => {
            if detour.uncertified_definition_fallback {
                *saw_unknown = true;
                Ok(false)
            } else {
                Ok(true)
            }
        }
        Some(false) | None => {
            if detour.uncertified_definition_fallback {
                *saw_unknown = true;
            }
            Ok(false)
        }
    }
}

fn probe_reaches_adjacent_cell_via_interior_box_detours_without_plane_replacement_progressively_with_surface_query(
    start: &Point3,
    end: &Point3,
    polygons: &[ConvexPolygon],
    visited_points: &[VisitedDefinitionPoint],
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    progressive_interior_box_detours: bool,
    no_plane_replacement_cycle_guard_cache: &mut Vec<
        DefinitionNoPlaneReplacementCycleGuardCacheEntry,
    >,
    surface_cache: &mut Vec<SurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
    detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
    detours_for_query: &mut impl FnMut(&Point3, &Point3) -> HypermeshResult<Vec<DetourTarget>>,
) -> HypermeshResult<bool> {
    let (intervals, mut saw_unknown) = interior_box_axis_intervals_with_surface_queries(
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

    for x in &intervals[0] {
        for y in &intervals[1] {
            for z in &intervals[2] {
                let bounds = aabb_from_axis_intervals([x, y, z])?;
                let trace_without_detours_cell =
                    std::cell::RefCell::new(&mut *trace_without_detours);
                let result = search_strict_aabb_targets_progressively_with_direct_ranking(
                    &bounds,
                    &mut |detour| {
                        detour_target_no_plane_direct_precheck_key(
                            detour,
                            start,
                            end,
                            start_definitions,
                            end_definitions,
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
                            surface_cache,
                            surface_query,
                            &mut **trace_without_detours_cell.borrow_mut(),
                            detour_target_cache,
                            detours_for_query,
                            &mut saw_unknown,
                        )
                    },
                );
                match result {
                    Ok(true) => return Ok(true),
                    Ok(false) => {}
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
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

fn detour_target_no_plane_direct_precheck_key(
    detour: &DetourTarget,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    trace_without_detours: &mut impl FnMut(
        &Point3,
        &Point3,
        &[[Plane; 3]],
        &[[Plane; 3]],
    ) -> HypermeshResult<bool>,
) -> HypermeshResult<[u8; 2]> {
    Ok([
        direct_precheck_rank(trace_without_detours(
            start,
            &detour.point,
            start_definitions,
            &detour.definitions,
        ))?,
        direct_precheck_rank(trace_without_detours(
            &detour.point,
            end,
            &detour.definitions,
            end_definitions,
        ))?,
    ])
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
#[allow(dead_code)]
fn plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let mut affine_cache = Vec::new();
    let mut path_cache = Vec::new();
    let mut step_cache = Vec::new();
    let mut no_nested_ordering_warmup_cache = Vec::new();
    let mut no_step_cache = Vec::new();
    let mut no_detour_cache = Vec::new();
    let mut no_plane_replacement_cycle_guard_cache = Vec::new();
    let mut no_plane_replacement_cache = Vec::new();
    let mut no_step_detour_target_cache = Vec::new();
    let mut direct_probe_reachability_cache = Vec::new();
    plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement_with_caches(
        start_planes,
        end_planes,
        host_support,
        polygons,
        &mut affine_cache,
        &mut path_cache,
        &mut step_cache,
        &mut no_nested_ordering_warmup_cache,
        &mut no_step_cache,
        &mut no_detour_cache,
        &mut no_plane_replacement_cycle_guard_cache,
        &mut no_plane_replacement_cache,
        &mut no_step_detour_target_cache,
        &mut direct_probe_reachability_cache,
    )
}

fn plane_replacement_path_reaches_adjacent_cell_without_nested_plane_replacement_with_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    path_cache: &mut Vec<PlaneReplacementReachabilityPathCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    no_nested_ordering_warmup_cache: &mut Vec<PlaneReplacementNoNestedOrderingWarmupCacheEntry>,
    no_step_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    no_detour_cache: &mut Vec<DefinitionNoDetourReachabilityCacheEntry>,
    no_plane_replacement_cycle_guard_cache: &mut Vec<
        DefinitionNoPlaneReplacementCycleGuardCacheEntry,
    >,
    no_plane_replacement_cache: &mut Vec<DefinitionNoPlaneReplacementReachabilityCacheEntry>,
    no_step_detour_target_cache: &mut Vec<DetourTargetFamilyCacheEntry>,
    direct_probe_reachability_cache: &mut Vec<DirectProbeReachabilityCacheEntry>,
) -> HypermeshResult<bool> {
    cached_plane_replacement_reachability_path_with(
        path_cache,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        start_planes,
        end_planes,
        || {
            let mut ordering_affine_cache = Vec::new();
            let mut no_step_affine_cache = Vec::new();
            let mut no_step_path_cache = Vec::new();
            let mut no_step_step_cache = Vec::new();
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
                        &mut ordering_affine_cache,
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
                        no_step_detour_target_cache,
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
fn plane_replacement_path_reaches_adjacent_cell_without_step_detours(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let mut affine_cache = Vec::new();
    let mut path_cache = Vec::new();
    let mut step_cache = Vec::new();
    plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches(
        start_planes,
        end_planes,
        host_support,
        polygons,
        &mut affine_cache,
        &mut path_cache,
        &mut step_cache,
    )
}

fn plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    host_support: &Plane,
    polygons: &[ConvexPolygon],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    path_cache: &mut Vec<PlaneReplacementReachabilityPathCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
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
            plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
                &ordered,
                start_planes,
                end_planes,
                PlaneReplacementReachabilityStepMode::WithoutStepDetours,
                affine_cache,
                step_cache,
                |current, next, _current_definitions, _next_definitions| {
                    probe_reaches_adjacent_cell(current, next, host_support, polygons)
                },
            )
        },
    )
}

#[cfg(test)]
fn plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    mode: PlaneReplacementReachabilityStepMode,
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    trace_step: impl FnMut(&Point3, &Point3, &[[Plane; 3]], &[[Plane; 3]]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
        &AXIS_ORDERINGS,
        start_planes,
        end_planes,
        mode,
        affine_cache,
        step_cache,
        trace_step,
    )
}

fn plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
    orderings: &[[usize; 3]],
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    mode: PlaneReplacementReachabilityStepMode,
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    mut trace_step: impl FnMut(&Point3, &Point3, &[[Plane; 3]], &[[Plane; 3]]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for ordering in orderings {
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
                    Ok(point) => point,
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

fn ordered_axis_orderings_by_no_step_precheck_with(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
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
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
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

fn cached_plane_replacement_reachability_step_with(
    cache: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    mode: PlaneReplacementReachabilityStepMode,
    current_point: &Point3,
    next_point: &Point3,
    current_planes: &[Plane; 3],
    next_planes: &[Plane; 3],
    trace: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        existing.mode == mode
            && ((existing.current_point == *current_point
                && existing.next_point == *next_point
                && definition_planes_match_as_sets(&existing.current_planes, current_planes)
                && definition_planes_match_as_sets(&existing.next_planes, next_planes))
                || (existing.current_point == *next_point
                    && existing.next_point == *current_point
                    && definition_planes_match_as_sets(&existing.current_planes, next_planes)
                    && definition_planes_match_as_sets(&existing.next_planes, current_planes)))
    }) {
        return existing.result.clone();
    }

    cache.push(PlaneReplacementReachabilityStepCacheEntry {
        mode,
        current_point: current_point.clone(),
        next_point: next_point.clone(),
        current_planes: current_planes.clone(),
        next_planes: next_planes.clone(),
        result: Err(HypermeshError::UnknownClassification),
    });
    let cache_index = cache.len() - 1;
    let result = trace();
    cache[cache_index].result = result.clone();
    result
}

fn cached_plane_replacement_reachability_path_with(
    cache: &mut Vec<PlaneReplacementReachabilityPathCacheEntry>,
    mode: PlaneReplacementReachabilityStepMode,
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    trace: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        existing.mode == mode
            && ((definition_planes_match_as_sets(&existing.start_planes, start_planes)
                && definition_planes_match_as_sets(&existing.end_planes, end_planes))
                || (definition_planes_match_as_sets(&existing.start_planes, end_planes)
                    && definition_planes_match_as_sets(&existing.end_planes, start_planes)))
    }) {
        return existing.result.clone();
    }

    cache.push(PlaneReplacementReachabilityPathCacheEntry {
        mode,
        start_planes: start_planes.clone(),
        end_planes: end_planes.clone(),
        result: Err(HypermeshError::UnknownClassification),
    });
    let cache_index = cache.len() - 1;
    let result = trace();
    cache[cache_index].result = result.clone();
    result
}

fn cached_plane_replacement_no_nested_ordering_warmup_with(
    cache: &mut Vec<PlaneReplacementNoNestedOrderingWarmupCacheEntry>,
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    affine_cache: &mut Vec<PlaneReplacementAffineCacheEntry>,
    path_cache: &mut Vec<PlaneReplacementReachabilityPathCacheEntry>,
    step_cache: &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    warm: impl FnOnce(
        &mut Vec<PlaneReplacementAffineCacheEntry>,
        &mut Vec<PlaneReplacementReachabilityPathCacheEntry>,
        &mut Vec<PlaneReplacementReachabilityStepCacheEntry>,
    ) -> HypermeshResult<Vec<[usize; 3]>>,
) -> HypermeshResult<Vec<[usize; 3]>> {
    if let Some(existing) = cache.iter().rev().find(|existing| {
        definition_planes_match_as_sets(&existing.start_planes, start_planes)
            && definition_planes_match_as_sets(&existing.end_planes, end_planes)
    }) {
        *affine_cache = existing.affine_cache.clone();
        *path_cache = existing.path_cache.clone();
        *step_cache = existing.step_cache.clone();
        return existing.ordered.clone();
    }

    let ordered = warm(affine_cache, path_cache, step_cache);
    cache.push(PlaneReplacementNoNestedOrderingWarmupCacheEntry {
        start_planes: start_planes.clone(),
        end_planes: end_planes.clone(),
        ordered: ordered.clone(),
        affine_cache: affine_cache.clone(),
        path_cache: path_cache.clone(),
        step_cache: step_cache.clone(),
    });
    ordered
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

#[cfg(test)]
fn halfspace_cell_geometry_seed_candidates(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Vec<Point3>> {
    let vertices = feasible_halfspace_cell_vertices(halfspaces)?;
    halfspace_cell_geometry_seed_candidates_from_vertices(&vertices)
}

#[cfg(test)]
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

    if let Some(center) = centroid(&vertices)? {
        match point_strictly_inside_leaf_or_unknown(&center, leaf) {
            Ok(true) => {
                let mut points = vec![InteriorLeafPoint {
                    point: center.clone(),
                    planes: Vec::new(),
                    uncertified_definition_fallback: false,
                }];
                extend_interior_leaf_points_backtracking_unknown(
                    &mut points,
                    std::iter::once(&center),
                    |witness| shifted_edge_interior_points(leaf, witness),
                )?;
                if points.iter().any(|point| !point.planes.is_empty()) {
                    points.retain(|point| !point.planes.is_empty());
                }
                if !points.is_empty() {
                    return Ok(points);
                }
            }
            Ok(false) => {}
            Err(HypermeshError::UnknownClassification) => {}
            Err(err) => return Err(err),
        }
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
    strict_leaf_witness_points_with_seed_families_and_stricter_replay(
        leaf,
        vertices,
        &mut seed_families_for,
        |leaf, witness| strict_leaf_cell_points(leaf, witness),
    )
}

fn strict_leaf_witness_points_with_seed_families_and_stricter_replay(
    leaf: &ConvexPolygon,
    vertices: &[Point3],
    seed_families_for: &mut impl FnMut(
        &ConvexPolygon,
        &[Point3],
        &Aabb,
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
    ) -> HypermeshResult<LeafWitnessSeedFamilies>,
    mut stricter_points_for: impl FnMut(
        &ConvexPolygon,
        &Point3,
    ) -> HypermeshResult<Vec<InteriorLeafPoint>>,
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
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    extend_leaf_point_builds_backtracking_unknown(&mut points, seeds.iter(), |seed| {
        let active_planes = active_planes_from_optional_report(report.as_ref(), seed);
        build_strict_leaf_point(leaf, seed, &halfspaces, active_planes, false)
    })?;

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            report_witness,
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
    let certified_direct_witness_points = points
        .iter()
        .filter(|point| !point.uncertified_definition_fallback)
        .map(|point| point.point.clone())
        .collect::<Vec<_>>();
    let direct_witnesses = points
        .iter()
        .map(|point| point.point.clone())
        .collect::<Vec<_>>();
    let mut stricter_points = Vec::new();
    match extend_interior_leaf_points_backtracking_unknown(
        &mut stricter_points,
        direct_witnesses.iter(),
        |witness| stricter_points_for(leaf, witness),
    ) {
        Ok(()) => {}
        Err(HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }
    for point in stricter_points {
        let duplicate_certified_direct_point = certified_direct_witness_points
            .iter()
            .any(|existing| *existing == point.point);
        if duplicate_certified_direct_point && point.uncertified_definition_fallback {
            saw_unknown = true;
            continue;
        }
        push_unique_interior_point(&mut points, point);
    }

    let unresolved_fallback = points
        .iter()
        .any(|point| point.uncertified_definition_fallback);
    let has_certified_point = points
        .iter()
        .any(|point| !point.uncertified_definition_fallback);
    if points.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_point && (saw_unknown || unresolved_fallback) {
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
            |candidate| point_strictly_inside_leaf_or_unknown(candidate, leaf),
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

        if point_strictly_inside_leaf_or_unknown(&candidate, leaf)? {
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

fn push_unique_interior_point(
    points: &mut Vec<InteriorLeafPoint>,
    point: InteriorLeafPoint,
) -> bool {
    if let Some(existing) = points
        .iter_mut()
        .find(|existing| existing.point == point.point)
    {
        let incoming_planes = point.planes;
        let incoming_is_fallback = point.uncertified_definition_fallback;
        let existing_covered_by_incoming = existing.planes.iter().all(|existing_planes| {
            incoming_planes.iter().any(|incoming_plane_set| {
                definition_planes_match_as_sets(existing_planes, incoming_plane_set)
            })
        });
        let mut introduced_new_definition = false;
        for planes in incoming_planes {
            if !existing
                .planes
                .iter()
                .any(|candidate| definition_planes_match_as_sets(candidate, &planes))
            {
                existing.planes.push(planes);
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
        let introduced_uncertified_state = point.uncertified_definition_fallback;
        points.push(point);
        introduced_uncertified_state
    }
}

fn extend_interior_leaf_points_backtracking_unknown<'a, T: 'a>(
    points: &mut Vec<InteriorLeafPoint>,
    candidates: impl IntoIterator<Item = &'a T>,
    mut build: impl FnMut(&'a T) -> HypermeshResult<Vec<InteriorLeafPoint>>,
) -> HypermeshResult<()> {
    let mut saw_hard_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(found) => {
                for point in found {
                    push_unique_interior_point(points, point);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let unresolved_fallback = points
        .iter()
        .any(|point| point.uncertified_definition_fallback);
    let has_certified_point = points
        .iter()
        .any(|point| !point.uncertified_definition_fallback);
    if points.is_empty() && (saw_hard_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_point && (saw_hard_unknown || unresolved_fallback) {
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
    let mut saw_hard_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(Some(point)) => {
                push_unique_interior_point(points, point);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let unresolved_fallback = points
        .iter()
        .any(|point| point.uncertified_definition_fallback);
    let has_certified_point = points
        .iter()
        .any(|point| !point.uncertified_definition_fallback);
    if points.is_empty() && (saw_hard_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_point && (saw_hard_unknown || unresolved_fallback) {
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
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);
    extend_leaf_point_builds_backtracking_unknown(&mut points, seeds.iter(), |witness| {
        let active_planes = active_planes_from_optional_report(report.as_ref(), witness);
        build_strict_leaf_point(leaf, witness, &halfspaces, active_planes, false)
    })?;

    let certified_direct_points = points
        .iter()
        .filter(|point| !point.uncertified_definition_fallback)
        .map(|point| point.point.clone())
        .collect::<Vec<_>>();

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            report_witness,
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
                |seed| shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    for shifted in &shifted_witnesses {
        let duplicate_certified_direct_point = certified_direct_points
            .iter()
            .any(|point| *point == shifted.point);
        match build_strict_leaf_point_from_shifted_witness(leaf, shifted) {
            Ok(Some(point)) => {
                if duplicate_certified_direct_point && point.uncertified_definition_fallback {
                    saw_unknown = true;
                    continue;
                }
                push_unique_interior_point(&mut points, point);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let unresolved_fallback = points
        .iter()
        .any(|point| point.uncertified_definition_fallback);
    let has_certified_point = points
        .iter()
        .any(|point| !point.uncertified_definition_fallback);
    if points.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_point && (saw_unknown || unresolved_fallback) {
            mark_all_interior_points_uncertified(&mut points);
        }
        Ok(points)
    }
}

#[cfg(test)]
fn strict_leaf_cell_points_from_seed_families_with_tracking_unknown(
    leaf: &ConvexPolygon,
    strict_interior: &Point3,
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    mut build_shifted_witnesses: impl FnMut(&Point3) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
) -> HypermeshResult<Vec<InteriorLeafPoint>> {
    let vertices = leaf.vertices()?;
    let _bounds = leaf_bounds(&vertices)?;
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

    let mut points = Vec::new();
    let mut saw_unknown = false;
    let report_witness = report.and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);
    extend_leaf_point_builds_backtracking_unknown(&mut points, seeds.iter(), |witness| {
        let active_planes = active_planes_from_optional_report(report, witness);
        build_strict_leaf_point(leaf, witness, &halfspaces, active_planes, false)
    })?;

    let certified_direct_points = points
        .iter()
        .filter(|point| !point.uncertified_definition_fallback)
        .map(|point| point.point.clone())
        .collect::<Vec<_>>();

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            report_witness,
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
                |seed| build_shifted_witnesses(seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    for shifted in &shifted_witnesses {
        let duplicate_certified_direct_point = certified_direct_points
            .iter()
            .any(|point| *point == shifted.point);
        match build_strict_leaf_point_from_shifted_witness(leaf, shifted) {
            Ok(Some(point)) => {
                if duplicate_certified_direct_point && point.uncertified_definition_fallback {
                    saw_unknown = true;
                    continue;
                }
                push_unique_interior_point(&mut points, point);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let unresolved_fallback = points
        .iter()
        .any(|point| point.uncertified_definition_fallback);
    let has_certified_point = points
        .iter()
        .any(|point| !point.uncertified_definition_fallback);
    if points.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_point && (saw_unknown || unresolved_fallback) {
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
    match classify_point_in_polygon(witness, leaf)? {
        PolygonPointLocation::Outside => return Ok(None),
        PolygonPointLocation::Boundary => {
            return Err(HypermeshError::UnknownClassification);
        }
        PolygonPointLocation::Interior => {}
    }

    let (planes, uncertified_definition_fallback) =
        match leaf_interior_definitions_from_active_halfspaces(
            witness,
            &leaf.support,
            halfspaces,
            active_planes,
        ) {
            Ok(found) => (found.definitions, false),
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
    match classify_point_in_polygon(&witness.point, leaf)? {
        PolygonPointLocation::Outside => return Ok(None),
        PolygonPointLocation::Boundary => {
            return Err(HypermeshError::UnknownClassification);
        }
        PolygonPointLocation::Interior => {}
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

    let used_axis_fallback = planes.is_empty() && saw_unknown;
    if planes.is_empty() {
        if used_axis_fallback {
            planes.push(axis_plane_definition(&witness.point));
        } else {
            return Ok(None);
        }
    }

    Ok(Some(InteriorLeafPoint {
        point: witness.point.clone(),
        planes,
        uncertified_definition_fallback: witness.uncertified_definition_fallback
            || used_axis_fallback,
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

#[cfg(test)]
fn point_strictly_inside_leaf(point: &Point3, leaf: &ConvexPolygon) -> HypermeshResult<bool> {
    let homogeneous = HomogeneousPoint3::new(
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
        Real::one(),
    );
    leaf.contains_point_strictly(&homogeneous)
}

fn point_strictly_inside_leaf_or_unknown(
    point: &Point3,
    leaf: &ConvexPolygon,
) -> HypermeshResult<bool> {
    match classify_point_in_polygon(point, leaf)? {
        PolygonPointLocation::Outside => Ok(false),
        PolygonPointLocation::Boundary => Err(HypermeshError::UnknownClassification),
        PolygonPointLocation::Interior => Ok(true),
    }
}

#[cfg(test)]
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

    let unresolved_fallback = probes
        .iter()
        .any(|probe| probe.uncertified_definition_fallback);
    let has_certified_probe = probes
        .iter()
        .any(|probe| !probe.uncertified_definition_fallback);
    if probes.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_probe && (saw_unknown || unresolved_fallback) {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

#[cfg_attr(not(test), allow(dead_code))]
fn extend_probe_families_backtracking_unknown(
    probes: &mut Vec<ProbePoint>,
    family: HypermeshResult<Vec<ProbePoint>>,
    saw_unknown: &mut bool,
) -> HypermeshResult<()> {
    match family {
        Ok(found) => {
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
        Ok(found) => Ok((found.definitions, false)),
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
    if definition_has_coplanar_pair(&definition)? {
        return Ok(());
    }
    let homogeneous = intersect_three_planes(&definition[0], &definition[1], &definition[2]);
    let w_class = match classify_real(&homogeneous.w) {
        Ok(classification) => classification,
        Err(HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            return Ok(());
        }
        Err(err) => return Err(err),
    };
    if w_class == Classification::On {
        return Ok(());
    }
    match homogeneous.to_affine_point() {
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

fn definition_has_coplanar_pair(definition: &[Plane; 3]) -> HypermeshResult<bool> {
    for first in 0..3 {
        for second in (first + 1)..3 {
            if planes_are_coplanar(&definition[first], &definition[second])? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

#[cfg(test)]
fn adjacent_normal_probes(
    interior: &InteriorLeafPoint,
    support: &Plane,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    positive_side: bool,
) -> HypermeshResult<Vec<ProbePoint>> {
    let retained_definitions = unique_normal_probe_search_definitions(&interior.planes, support)?;
    adjacent_normal_probes_with_queries(
        interior,
        support,
        bounds,
        polygons,
        positive_side,
        |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        |point, polygon| classify_point_in_polygon(point, polygon),
        |corridor, stop_point| {
            collect_normal_probe_targets(&retained_definitions, |definition| {
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

#[cfg(test)]
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

    let unresolved_fallback = probes
        .iter()
        .any(|probe| probe.uncertified_definition_fallback);
    let has_certified_probe = probes
        .iter()
        .any(|probe| !probe.uncertified_definition_fallback);
    if probes.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_probe && (saw_unknown || unresolved_fallback) {
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
    let (bound_stop, mut saw_unknown) = normal_probe_bounds_stop(interior, direction, bounds)?;
    let Some(bound_stop) = bound_stop else {
        return Ok((Vec::new(), saw_unknown));
    };

    let mut stop_values = vec![bound_stop.clone()];

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
        let crossing = offset_point(interior, direction, &crossing_t);
        if !positive_real_strictly_before(&crossing_t, &bound_stop)? {
            if compare_real(&crossing_t, &bound_stop)?.is_eq() {
                let point_location = match classify_point_on_polygon(&crossing, polygon) {
                    Ok(point_location) => point_location,
                    Err(HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
                if matches!(
                    point_location,
                    PolygonPointLocation::Boundary | PolygonPointLocation::Interior
                ) {
                    saw_unknown = true;
                }
            }
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

fn push_unique_probe_point(probes: &mut Vec<ProbePoint>, probe: ProbePoint) -> bool {
    if let Some(existing) = probes
        .iter_mut()
        .find(|existing| existing.point == probe.point && existing.side == probe.side)
    {
        let incoming_planes = probe.planes;
        let incoming_is_fallback = probe.uncertified_definition_fallback;
        let existing_covered_by_incoming = existing.planes.iter().all(|existing_plane_set| {
            incoming_planes.iter().any(|incoming_plane_set| {
                definition_planes_match_as_sets(existing_plane_set, incoming_plane_set)
            })
        });
        let mut introduced_new_definition = false;
        for definition in incoming_planes {
            if !existing
                .planes
                .iter()
                .any(|candidate| definition_planes_match_as_sets(candidate, &definition))
            {
                existing.planes.push(definition);
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
        let introduced_uncertified_state = probe.uncertified_definition_fallback;
        probes.push(probe);
        introduced_uncertified_state
    }
}

#[cfg(test)]
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
    let unresolved_fallback = probes
        .iter()
        .any(|probe| probe.uncertified_definition_fallback);
    let has_certified_probe = probes
        .iter()
        .any(|probe| !probe.uncertified_definition_fallback);
    if probes.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_probe && (saw_unknown || unresolved_fallback) {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

fn normal_probe_extra_planes(
    _interior: &InteriorLeafPoint,
    definition: Option<&[Plane; 3]>,
) -> Vec<Plane> {
    let mut extra_planes = Vec::new();
    if let Some(definition) = definition {
        for plane in &definition[1..] {
            if !extra_planes.iter().any(|existing| existing == plane) {
                extra_planes.push(plane.clone());
            }
        }
    }
    extra_planes
}

fn normal_probe_shifted_seed_families(
    definition: Option<&[Plane; 3]>,
    report_witness: Option<&Point3>,
    certified_probe_points: &[Point3],
    seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
) -> (Vec<Point3>, Vec<Point3>, Vec<Point3>) {
    if definition.is_none() {
        if !certified_probe_points.is_empty() {
            let mut shifted_roots = Vec::new();
            if let Some(report_witness) = report_witness {
                shifted_roots.push(report_witness.clone());
            } else if let Some(first_probe_point) = certified_probe_points.first() {
                shifted_roots.push(first_probe_point.clone());
            }
            return dedupe_shifted_halfspace_seed_families(shifted_roots, Vec::new(), Vec::new());
        }
        if !seeds.is_empty() {
            return shifted_halfspace_seed_families_with_report_seed(
                report_witness,
                seeds,
                Vec::new(),
                Vec::new(),
            );
        }
    }
    shifted_halfspace_seed_families_with_report_seed(
        report_witness,
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    )
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

fn retained_plane_pairs_match_as_sets(left: &[Plane; 3], right: &[Plane; 3]) -> bool {
    (left[1] == right[1] && left[2] == right[2]) || (left[1] == right[2] && left[2] == right[1])
}

fn unique_normal_probe_search_definitions(
    definitions: &[[Plane; 3]],
    support: &Plane,
) -> HypermeshResult<Vec<[Plane; 3]>> {
    let mut unique = Vec::new();
    for definition in unique_definition_family(definitions) {
        if !normal_probe_definition_preserves_support_direction(&definition, support)? {
            continue;
        }
        if unique
            .iter()
            .all(|existing| !retained_plane_pairs_match_as_sets(existing, &definition))
        {
            unique.push(definition);
        }
    }
    Ok(unique)
}

#[cfg(test)]
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
    let extra_planes = normal_probe_extra_planes(interior, definition);
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let shifted_vertex_family = feasible_halfspace_cell_vertex_family(&halfspaces)?;
    saw_unknown |= shifted_vertex_family.saw_unknown;
    let shifted_vertices = shifted_vertex_family.seeds;
    let mut shifted_geometry_seeds = Vec::new();
    let mut seeds = Vec::new();

    saw_unknown |= extend_strict_halfspace_seed_families_collect_unknown(
        &mut seeds,
        [
            if report
                .as_ref()
                .is_some_and(|report| report.status == HalfspaceFeasibility::Feasible)
                && let Some(witness) = report_witness
            {
                collect_strict_halfspace_seed_family(Ok(vec![witness.clone()]), |candidate| {
                    point_strictly_inside_halfspace_cell_or_unknown(
                        candidate,
                        corridor,
                        &halfspaces,
                    )
                })
            } else {
                Ok(HalfspaceSeedFamilyState {
                    seeds: Vec::new(),
                    saw_unknown: false,
                })
            },
            collect_strict_halfspace_seed_family(Ok(shifted_vertices.clone()), |candidate| {
                point_strictly_inside_halfspace_cell_or_unknown(candidate, corridor, &halfspaces)
            }),
        ],
    )?;

    let mut seen_direct_seeds = Vec::new();
    let mut seeds = take_new_halfspace_seed_family(seeds, &mut seen_direct_seeds);

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_probe_point(
            witness,
            corridor,
            support,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            &extra_planes,
            false,
        )
    })?;

    let mut certified_probe_points = probes
        .iter()
        .filter(|probe| !probe.uncertified_definition_fallback)
        .map(|probe| probe.point.clone())
        .collect::<Vec<_>>();

    if definition.is_some() || certified_probe_points.is_empty() {
        let shifted_geometry_seed_family =
            halfspace_centroid_subset_seed_family_from_vertices(&shifted_vertices)?;
        saw_unknown |= shifted_geometry_seed_family.saw_unknown;
        shifted_geometry_seeds = shifted_geometry_seed_family.seeds;

        let mut geometry_strict_seeds = Vec::new();
        saw_unknown |= extend_strict_halfspace_seed_families_collect_unknown(
            &mut geometry_strict_seeds,
            [collect_strict_halfspace_seed_family(
                Ok(shifted_geometry_seeds.clone()),
                |candidate| {
                    point_strictly_inside_halfspace_cell_or_unknown(
                        candidate,
                        corridor,
                        &halfspaces,
                    )
                },
            )],
        )?;
        let mut seen_all_direct_seeds = seeds.clone();
        let geometry_strict_seeds =
            take_new_halfspace_seed_family(geometry_strict_seeds, &mut seen_all_direct_seeds);
        extend_probe_point_builds_backtracking_unknown(
            &mut probes,
            geometry_strict_seeds.iter(),
            |witness| {
                build_probe_point(
                    witness,
                    corridor,
                    support,
                    &halfspaces,
                    active_planes_from_optional_report(report.as_ref(), witness),
                    &extra_planes,
                    false,
                )
            },
        )?;
        seeds.extend(geometry_strict_seeds);
        certified_probe_points = probes
            .iter()
            .filter(|probe| !probe.uncertified_definition_fallback)
            .map(|probe| probe.point.clone())
            .collect::<Vec<_>>();
    }

    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);
    if seed_family_search_failed_without_any_seed(
        &seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        saw_unknown,
    ) {
        return Err(HypermeshError::UnknownClassification);
    }
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        normal_probe_shifted_seed_families(
            definition,
            report_witness,
            &certified_probe_points,
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
                |seed| shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    for shifted in &shifted_witnesses {
        let duplicate_certified_direct_probe = certified_probe_points
            .iter()
            .any(|point| *point == shifted.point);
        match build_probe_point_from_shifted_witness(shifted, corridor, support, &extra_planes) {
            Ok(Some(probe)) => {
                if duplicate_certified_direct_probe && probe.uncertified_definition_fallback {
                    saw_unknown = true;
                    continue;
                }
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let unresolved_fallback = probes
        .iter()
        .any(|probe| probe.uncertified_definition_fallback);
    let has_certified_probe = probes
        .iter()
        .any(|probe| !probe.uncertified_definition_fallback);
    if probes.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_probe && (saw_unknown || unresolved_fallback) {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

fn strict_normal_probe_targets_with_query_caches(
    interior: &InteriorLeafPoint,
    support: &Plane,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
    positive_side: bool,
    probe_query_caches: &mut LeafProbeQueryCaches,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    halfspaces.push(normal_stop_halfspace(support, stop_point, positive_side));

    let mut local_unknown = false;
    let report = cached_optional_halfspace_feasibility_report_with(
        &mut probe_query_caches.halfspace_reports,
        &halfspaces,
        &mut local_unknown,
    )?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }
    let mut probes = Vec::new();
    let extra_planes = normal_probe_extra_planes(interior, definition);
    let report_witness = report.as_ref().and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        cached_halfspace_cell_seed_families_from_optional_report_with(
            &mut probe_query_caches.halfspace_seed_families,
            corridor,
            &halfspaces,
            report.as_ref(),
            &mut local_unknown,
        )?;
    let mut seen_direct_seeds = Vec::new();
    let mut seeds = take_new_halfspace_seed_family(seeds, &mut seen_direct_seeds);

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_probe_point(
            witness,
            corridor,
            support,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            &extra_planes,
            false,
        )
    })?;

    let mut certified_probe_points = probes
        .iter()
        .filter(|probe| !probe.uncertified_definition_fallback)
        .map(|probe| probe.point.clone())
        .collect::<Vec<_>>();

    if definition.is_some() || certified_probe_points.is_empty() {
        let mut geometry_strict_seeds = Vec::new();
        local_unknown |= extend_strict_halfspace_seed_families_collect_unknown(
            &mut geometry_strict_seeds,
            [collect_strict_halfspace_seed_family(
                Ok(shifted_geometry_seeds.clone()),
                |candidate| {
                    point_strictly_inside_halfspace_cell_or_unknown(
                        candidate,
                        corridor,
                        &halfspaces,
                    )
                },
            )],
        )?;
        let mut seen_all_direct_seeds = seeds.clone();
        let geometry_strict_seeds =
            take_new_halfspace_seed_family(geometry_strict_seeds, &mut seen_all_direct_seeds);
        extend_probe_point_builds_backtracking_unknown(
            &mut probes,
            geometry_strict_seeds.iter(),
            |witness| {
                build_probe_point(
                    witness,
                    corridor,
                    support,
                    &halfspaces,
                    active_planes_from_optional_report(report.as_ref(), witness),
                    &extra_planes,
                    false,
                )
            },
        )?;
        seeds.extend(geometry_strict_seeds);
        certified_probe_points = probes
            .iter()
            .filter(|probe| !probe.uncertified_definition_fallback)
            .map(|probe| probe.point.clone())
            .collect::<Vec<_>>();
    }

    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);
    if seed_family_search_failed_without_any_seed(
        &seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        local_unknown,
    ) {
        return Err(HypermeshError::UnknownClassification);
    }
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        normal_probe_shifted_seed_families(
            definition,
            report_witness,
            &certified_probe_points,
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
                |seed| shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut local_unknown,
    )?;
    for shifted in &shifted_witnesses {
        let duplicate_certified_direct_probe = certified_probe_points
            .iter()
            .any(|point| *point == shifted.point);
        match build_probe_point_from_shifted_witness(shifted, corridor, support, &extra_planes) {
            Ok(Some(probe)) => {
                if duplicate_certified_direct_probe && probe.uncertified_definition_fallback {
                    local_unknown = true;
                    continue;
                }
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                local_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let unresolved_fallback = probes
        .iter()
        .any(|probe| probe.uncertified_definition_fallback);
    let has_certified_probe = probes
        .iter()
        .any(|probe| !probe.uncertified_definition_fallback);
    if probes.is_empty() && (local_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_probe && (local_unknown || unresolved_fallback) {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

#[cfg(test)]
fn strict_normal_probe_targets_from_seed_families_with_tracking_unknown(
    interior: &InteriorLeafPoint,
    support: &Plane,
    corridor: &Aabb,
    definition: Option<&[Plane; 3]>,
    stop_point: &Point3,
    positive_side: bool,
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    mut build_shifted_witnesses: impl FnMut(&Point3) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));
    halfspaces.push(normal_stop_halfspace(support, stop_point, positive_side));

    let mut probes = Vec::new();
    let mut saw_unknown = false;
    let extra_planes = normal_probe_extra_planes(interior, definition);
    let report_witness = report.and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_probe_point(
            witness,
            corridor,
            support,
            &halfspaces,
            active_planes_from_optional_report(report, witness),
            &extra_planes,
            false,
        )
    })?;

    let certified_probe_points = probes
        .iter()
        .filter(|probe| !probe.uncertified_definition_fallback)
        .map(|probe| probe.point.clone())
        .collect::<Vec<_>>();
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        normal_probe_shifted_seed_families(
            definition,
            report_witness,
            &certified_probe_points,
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
                |seed| build_shifted_witnesses(seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    for shifted in &shifted_witnesses {
        let duplicate_certified_direct_probe = certified_probe_points
            .iter()
            .any(|point| *point == shifted.point);
        match build_probe_point_from_shifted_witness(shifted, corridor, support, &extra_planes) {
            Ok(Some(probe)) => {
                if duplicate_certified_direct_probe && probe.uncertified_definition_fallback {
                    saw_unknown = true;
                    continue;
                }
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let unresolved_fallback = probes
        .iter()
        .any(|probe| probe.uncertified_definition_fallback);
    let has_certified_probe = probes
        .iter()
        .any(|probe| !probe.uncertified_definition_fallback);
    if probes.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_probe && (saw_unknown || unresolved_fallback) {
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
) -> HypermeshResult<(Option<Real>, bool)> {
    let mut stop_t: Option<Real> = None;
    let mut saw_unknown = false;
    for axis in 0..3 {
        let component = axis_ref(direction, axis);
        match classify_real(component)? {
            Classification::Positive => {
                let room = axis_ref(&bounds.max, axis) - axis_ref(interior, axis);
                let room_order = compare_real(&room, &Real::zero())?;
                if !room_order.is_gt() {
                    saw_unknown = room_order.is_eq();
                    return Ok((None, saw_unknown));
                }
                update_positive_stop(
                    &mut stop_t,
                    (room / component.clone())
                        .map_err(|_| HypermeshError::UnknownClassification)?,
                )?;
            }
            Classification::Negative => {
                let room = axis_ref(interior, axis) - axis_ref(&bounds.min, axis);
                let room_order = compare_real(&room, &Real::zero())?;
                if !room_order.is_gt() {
                    saw_unknown = room_order.is_eq();
                    return Ok((None, saw_unknown));
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
    Ok((stop_t, saw_unknown))
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

#[cfg_attr(not(test), allow(dead_code))]
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
            let start_class = classify_point(interior, &polygon.support)?;
            let endpoint_class = classify_point(endpoint, &polygon.support)?;
            if start_class == Classification::On {
                return Ok(Some(interior.clone()));
            }
            if endpoint_class == Classification::On {
                return Ok(Some(endpoint.clone()));
            }
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

#[cfg_attr(not(test), allow(dead_code))]
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

    let unresolved_fallback = probes
        .iter()
        .any(|probe| probe.uncertified_definition_fallback);
    let has_certified_probe = probes
        .iter()
        .any(|probe| !probe.uncertified_definition_fallback);
    if probes.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_probe && (saw_unknown || unresolved_fallback) {
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
        return Ok((Vec::new(), compare_real(start_value, bound_value)?.is_eq()));
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
        let point_location = match classify_point_on_polygon(&crossing, polygon) {
            Ok(point_location) => point_location,
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if !point_strictly_between_axis(&crossing, interior, &endpoint, axis)? {
            if crossing == *interior
                && matches!(
                    point_location,
                    PolygonPointLocation::Boundary | PolygonPointLocation::Interior
                )
            {
                saw_unknown = true;
            }
            if crossing == endpoint
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

#[cfg_attr(not(test), allow(dead_code))]
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
    let unresolved_fallback = probes
        .iter()
        .any(|probe| probe.uncertified_definition_fallback);
    let has_certified_probe = probes
        .iter()
        .any(|probe| !probe.uncertified_definition_fallback);
    if probes.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_probe && (saw_unknown || unresolved_fallback) {
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
    let mut saw_hard_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(Some(probe)) => {
                push_unique_probe_point(probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let unresolved_fallback = probes
        .iter()
        .any(|probe| probe.uncertified_definition_fallback);
    let has_certified_probe = probes
        .iter()
        .any(|probe| !probe.uncertified_definition_fallback);
    if probes.is_empty() && (saw_hard_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_probe && (saw_hard_unknown || unresolved_fallback) {
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
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_axis_probe_point(
            witness,
            interior,
            corridor,
            support,
            axis,
            definition,
            &halfspaces,
            active_planes_from_optional_report(report.as_ref(), witness),
            false,
        )
    })?;

    let certified_probe_points = probes
        .iter()
        .filter(|probe| !probe.uncertified_definition_fallback)
        .map(|probe| probe.point.clone())
        .collect::<Vec<_>>();

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            report_witness,
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
                |seed| shifted_halfspace_cell_witnesses_from_seed(corridor, &halfspaces, seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    for shifted in &shifted_witnesses {
        let duplicate_certified_direct_probe = certified_probe_points
            .iter()
            .any(|point| *point == shifted.point);
        match build_axis_probe_point_from_shifted_witness(
            shifted, interior, corridor, support, axis, definition,
        ) {
            Ok(Some(probe)) => {
                if duplicate_certified_direct_probe && probe.uncertified_definition_fallback {
                    saw_unknown = true;
                    continue;
                }
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let unresolved_fallback = probes
        .iter()
        .any(|probe| probe.uncertified_definition_fallback);
    let has_certified_probe = probes
        .iter()
        .any(|probe| !probe.uncertified_definition_fallback);
    if probes.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_probe && (saw_unknown || unresolved_fallback) {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

#[cfg(test)]
fn strict_axis_probe_targets_from_seed_families_with_tracking_unknown(
    interior: &InteriorLeafPoint,
    support: &Plane,
    corridor: &Aabb,
    axis: usize,
    positive_side: bool,
    definition: Option<&[Plane; 3]>,
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    mut build_shifted_witnesses: impl FnMut(&Point3) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
) -> HypermeshResult<Vec<ProbePoint>> {
    let mut halfspaces = aabb_core_halfspaces(corridor)?;
    if let Some(definition) = definition {
        push_plane_equality_halfspaces(&mut halfspaces, &definition[1]);
        push_plane_equality_halfspaces(&mut halfspaces, &definition[2]);
    }
    halfspaces.push(support_side_halfspace(support, positive_side));

    let mut probes = Vec::new();
    let mut saw_unknown = false;
    let report_witness = report.and_then(|report| report.witness.as_ref());
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_halfspace_seed_families(seeds, shifted_vertices, shifted_geometry_seeds);

    extend_probe_point_builds_backtracking_unknown(&mut probes, seeds.iter(), |witness| {
        build_axis_probe_point(
            witness,
            interior,
            corridor,
            support,
            axis,
            definition,
            &halfspaces,
            active_planes_from_optional_report(report, witness),
            false,
        )
    })?;

    let certified_probe_points = probes
        .iter()
        .filter(|probe| !probe.uncertified_definition_fallback)
        .map(|probe| probe.point.clone())
        .collect::<Vec<_>>();

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            report_witness,
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
                |seed| build_shifted_witnesses(seed),
            )?;
            Ok(shifted_witnesses)
        },
        &mut saw_unknown,
    )?;
    for shifted in &shifted_witnesses {
        let duplicate_certified_direct_probe = certified_probe_points
            .iter()
            .any(|point| *point == shifted.point);
        match build_axis_probe_point_from_shifted_witness(
            shifted, interior, corridor, support, axis, definition,
        ) {
            Ok(Some(probe)) => {
                if duplicate_certified_direct_probe && probe.uncertified_definition_fallback {
                    saw_unknown = true;
                    continue;
                }
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let unresolved_fallback = probes
        .iter()
        .any(|probe| probe.uncertified_definition_fallback);
    let has_certified_probe = probes
        .iter()
        .any(|probe| !probe.uncertified_definition_fallback);
    if probes.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_probe && (saw_unknown || unresolved_fallback) {
            mark_all_probe_points_uncertified(&mut probes);
        }
        Ok(probes)
    }
}

fn build_probe_point(
    witness: &Point3,
    corridor: &Aabb,
    support: &Plane,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    extra_planes: &[Plane],
    inherited_uncertified_definition_fallback: bool,
) -> HypermeshResult<Option<ProbePoint>> {
    if !point_strictly_inside_halfspace_cell_or_unknown(witness, corridor, halfspaces)? {
        return Ok(None);
    }
    let side = classify_point(witness, support)?;
    if side == Classification::On {
        return Err(HypermeshError::UnknownClassification);
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
    corridor: &Aabb,
    support: &Plane,
    extra_planes: &[Plane],
) -> HypermeshResult<Option<ProbePoint>> {
    let side = classify_point(&witness.point, support)?;
    if side == Classification::On {
        return Err(HypermeshError::UnknownClassification);
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
    for family in &witness.families {
        match point_strictly_inside_halfspace_cell_or_unknown(
            &witness.point,
            corridor,
            &family.halfspaces,
        )? {
            true => {}
            false => continue,
        }
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

    let used_axis_fallback = planes.is_empty() && saw_unknown;
    if planes.is_empty() {
        if used_axis_fallback {
            planes.push(axis_plane_definition(&witness.point));
        } else {
            return Ok(None);
        }
    }

    Ok(Some(ProbePoint {
        point: witness.point.clone(),
        side,
        planes,
        uncertified_definition_fallback: witness.uncertified_definition_fallback
            || used_axis_fallback,
    }))
}

fn build_axis_probe_point(
    witness: &Point3,
    interior: &InteriorLeafPoint,
    corridor: &Aabb,
    support: &Plane,
    axis: usize,
    definition: Option<&[Plane; 3]>,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    inherited_uncertified_definition_fallback: bool,
) -> HypermeshResult<Option<ProbePoint>> {
    if !point_strictly_inside_halfspace_cell_or_unknown(witness, corridor, halfspaces)? {
        return Ok(None);
    }
    let side = classify_point(witness, support)?;
    if side == Classification::On {
        return Err(HypermeshError::UnknownClassification);
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
    corridor: &Aabb,
    support: &Plane,
    axis: usize,
    definition: Option<&[Plane; 3]>,
) -> HypermeshResult<Option<ProbePoint>> {
    let side = classify_point(&witness.point, support)?;
    if side == Classification::On {
        return Err(HypermeshError::UnknownClassification);
    }

    let mut planes = Vec::new();
    let mut saw_unknown = false;
    for family in &witness.families {
        match point_strictly_inside_halfspace_cell_or_unknown(
            &witness.point,
            corridor,
            &family.halfspaces,
        )? {
            true => {}
            false => continue,
        }
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

    let used_axis_fallback = planes.is_empty() && saw_unknown;
    if planes.is_empty() {
        if used_axis_fallback {
            planes.push(axis_plane_definition(&witness.point));
        } else {
            return Ok(None);
        }
    }

    Ok(Some(ProbePoint {
        point: witness.point.clone(),
        side,
        planes,
        uncertified_definition_fallback: witness.uncertified_definition_fallback
            || used_axis_fallback,
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

#[cfg(test)]
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
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            report_witness,
            Vec::new(),
            shifted_vertices,
            shifted_geometry_seeds,
        );
    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        strict_shift_seeds,
        |witness| {
            if !point_strictly_inside_halfspace_cell_or_unknown(&witness, bounds, halfspaces)? {
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
        shifted_vertices,
        |witness| {
            if !point_strictly_inside_halfspace_cell_or_unknown(&witness, bounds, halfspaces)? {
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
            if !point_strictly_inside_halfspace_cell_or_unknown(&witness, bounds, halfspaces)? {
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

    let unresolved_fallback = witnesses
        .iter()
        .any(|witness| witness.uncertified_definition_fallback);
    let has_certified_witness = witnesses
        .iter()
        .any(|witness| !witness.uncertified_definition_fallback);
    if witnesses.is_empty() && (saw_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_witness && (saw_unknown || unresolved_fallback) {
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
                    point_strictly_inside_halfspace_cell_or_unknown(candidate, bounds, halfspaces)
                })
            } else {
                Ok(HalfspaceSeedFamilyState {
                    seeds: Vec::new(),
                    saw_unknown: false,
                })
            },
            collect_strict_halfspace_seed_family(Ok(shifted_vertices.clone()), |candidate| {
                point_strictly_inside_halfspace_cell_or_unknown(candidate, bounds, halfspaces)
            }),
            collect_strict_halfspace_seed_family(Ok(shifted_geometry_seeds.clone()), |candidate| {
                point_strictly_inside_halfspace_cell_or_unknown(candidate, bounds, halfspaces)
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
) -> bool {
    if let Some(existing) = witnesses
        .iter_mut()
        .find(|existing| existing.point == witness.point)
    {
        let incoming_families = witness.families;
        let incoming_is_fallback = witness.uncertified_definition_fallback;
        let existing_covered_by_incoming = existing.families.iter().all(|existing_family| {
            incoming_families.iter().any(|incoming_family| {
                shifted_halfspace_witness_families_match(existing_family, incoming_family)
            })
        });
        let mut introduced_new_family = false;
        for family in incoming_families {
            if !existing
                .families
                .iter()
                .any(|candidate| shifted_halfspace_witness_families_match(candidate, &family))
            {
                existing.families.push(family);
                introduced_new_family = true;
            }
        }
        if incoming_is_fallback {
            if introduced_new_family {
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
        let introduced_uncertified_state = witness.uncertified_definition_fallback;
        witnesses.push(witness);
        introduced_uncertified_state
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
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
) -> (Vec<Point3>, Vec<Point3>, Vec<Point3>) {
    let mut seen = Vec::new();
    let strict_seeds = take_new_halfspace_seed_family(strict_seeds, &mut seen);
    let shifted_vertices = take_new_halfspace_seed_family(shifted_vertices, &mut seen);
    let shifted_geometry_seeds = take_new_halfspace_seed_family(shifted_geometry_seeds, &mut seen);
    (strict_seeds, shifted_vertices, shifted_geometry_seeds)
}

fn shifted_halfspace_seed_families_with_report_seed(
    report_witness: Option<&Point3>,
    mut strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
) -> (Vec<Point3>, Vec<Point3>, Vec<Point3>) {
    if let Some(report_witness) = report_witness
        && !strict_seeds
            .iter()
            .any(|existing| existing == report_witness)
    {
        strict_seeds.push(report_witness.clone());
    }
    dedupe_shifted_halfspace_seed_families(strict_seeds, shifted_vertices, shifted_geometry_seeds)
}

fn extend_shifted_halfspace_seed_families_backtracking_unknown(
    witnesses: &mut Vec<ShiftedHalfspaceWitness>,
    families: impl IntoIterator<Item = Vec<Point3>>,
    mut build: impl FnMut(&Point3) -> HypermeshResult<Vec<ShiftedHalfspaceWitness>>,
) -> HypermeshResult<()> {
    let mut saw_hard_unknown = false;
    let mut seen = Vec::new();
    for family in families {
        let fresh = take_new_halfspace_seed_family(family, &mut seen);
        let mut local = Vec::new();
        match extend_shifted_halfspace_witnesses_backtracking_unknown(&mut local, fresh, |seed| {
            build(seed)
        }) {
            Ok(()) => {
                for witness in local {
                    push_unique_shifted_halfspace_witness(witnesses, witness);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let unresolved_fallback = witnesses
        .iter()
        .any(|witness| witness.uncertified_definition_fallback);
    let has_certified_witness = witnesses
        .iter()
        .any(|witness| !witness.uncertified_definition_fallback);
    if witnesses.is_empty() && (saw_hard_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_witness && (saw_hard_unknown || unresolved_fallback) {
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
    let mut saw_hard_unknown = false;
    for seed in seeds {
        match build(&seed) {
            Ok(found) => {
                for witness in found {
                    push_unique_shifted_halfspace_witness(witnesses, witness);
                }
            }
            Err(HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let unresolved_fallback = witnesses
        .iter()
        .any(|witness| witness.uncertified_definition_fallback);
    let has_certified_witness = witnesses
        .iter()
        .any(|witness| !witness.uncertified_definition_fallback);
    if witnesses.is_empty() && (saw_hard_unknown || unresolved_fallback) {
        Err(HypermeshError::UnknownClassification)
    } else {
        if !has_certified_witness && (saw_hard_unknown || unresolved_fallback) {
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

#[cfg(test)]
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

#[cfg(test)]
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

#[cfg(test)]
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

fn point_strictly_inside_halfspace_cell_or_unknown(
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    if !point_strictly_inside_probe_bounds(point, bounds)? {
        for axis in 0..3 {
            let min = axis_ref(&bounds.min, axis);
            let max = axis_ref(&bounds.max, axis);
            if compare_real(min, max)?.is_eq() {
                continue;
            }
            let point_value = axis_ref(point, axis);
            if compare_real(point_value, min)?.is_eq() || compare_real(point_value, max)?.is_eq() {
                return Err(HypermeshError::UnknownClassification);
            }
        }
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
            return Err(HypermeshError::UnknownClassification);
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
    fn trace_axis_segment_reports_unknown_for_endpoint_surface_contact() {
        let wall = make_triangle(&p(1, 0, 0), &p(1, -1, 1), &p(1, 1, 1), 0, 0);

        assert_eq!(
            trace_axis_segment(&p(1, 0, 0), &p(2, 0, 0), 0, &[0], &[wall]),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn trace_axis_segment_reports_unknown_for_zero_length_surface_contact() {
        let wall = make_triangle(&p(1, 0, 0), &p(1, -1, 1), &p(1, 1, 1), 0, 0);

        assert_eq!(
            trace_axis_segment(&p(1, 0, 0), &p(1, 0, 0), 0, &[0], &[wall]),
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
    fn trace_direct_segment_reports_unknown_for_endpoint_surface_contact() {
        let wall = make_triangle(&p(1, 0, 0), &p(1, -1, 1), &p(1, 1, 1), 0, 0);

        assert_eq!(
            trace_direct_segment(&p(1, 0, 0), &p(2, 0, 0), &[0], &[wall]),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn trace_direct_segment_reports_unknown_for_zero_length_surface_contact() {
        let wall = make_triangle(&p(1, 0, 0), &p(1, -1, 1), &p(1, 1, 1), 0, 0);

        assert_eq!(
            trace_direct_segment(&p(1, 0, 0), &p(1, 0, 0), &[0], &[wall]),
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
    fn strict_aabb_targets_try_shifted_search_from_report_witness_seed() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        let targets = strict_aabb_targets_with_seed_families(
            &bounds,
            |_bounds, _halfspaces, _report, _saw_unknown| Ok((Vec::new(), Vec::new(), Vec::new())),
        )
        .unwrap();

        assert!(!targets.is_empty());
        assert!(targets.iter().all(|target| !target.definitions.is_empty()));
    }

    #[test]
    fn search_strict_aabb_targets_progressively_stops_after_first_certified_direct_target() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let mut evaluated = 0usize;

        let found = search_strict_aabb_targets_progressively_with_seed_families(
            &bounds,
            |_bounds, _halfspaces, _report, _saw_unknown| {
                Ok((vec![p(1, 1, 1)], vec![p(2, 2, 2)], vec![p(3, 3, 3)]))
            },
            &mut |_target| {
                evaluated += 1;
                Ok(true)
            },
        )
        .unwrap();

        assert!(found);
        assert_eq!(evaluated, 1);
    }

    #[test]
    fn search_strict_aabb_targets_progressively_ranks_direct_targets_before_evaluation() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let mut evaluated = Vec::new();

        let found = search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking(
            &bounds,
            |_bounds, _halfspaces, _report, _saw_unknown| {
                Ok((vec![p(1, 1, 1), p(2, 2, 2)], Vec::new(), Vec::new()))
            },
            &mut |target| {
                if target.point == p(2, 2, 2) {
                    Ok([0u8, 0u8])
                } else {
                    Ok([1u8, 1u8])
                }
            },
            &mut |target| {
                evaluated.push(target.point.clone());
                Ok(true)
            },
        )
        .unwrap();

        assert!(found);
        assert_eq!(evaluated, vec![p(2, 2, 2)]);
    }

    #[test]
    fn no_plane_detour_target_evaluation_prefers_lower_ranked_leg_first() {
        let point = p(1, 1, 1);
        let start_definitions = axis_plane_definition(&p(1, 1, 1));
        let detour_definitions = axis_plane_definition(&p(2, 2, 2));
        let end_definitions = axis_plane_definition(&p(3, 3, 3));
        let detour = DetourTarget {
            point: point.clone(),
            definitions: vec![detour_definitions.clone()],
            uncertified_definition_fallback: false,
        };
        let mut trace_calls = Vec::new();
        let mut saw_unknown = false;

        let result = evaluate_probe_detour_target_without_plane_replacement_with_surface_query(
            &detour,
            &point,
            &point,
            &[],
            &[],
            &[start_definitions.clone()],
            &[end_definitions.clone()],
            true,
            &mut Vec::new(),
            &mut Vec::new(),
            &mut |_point| Ok(false),
            &mut |_from, _to, from_definitions, to_definitions| {
                let call = if from_definitions == [start_definitions.clone()]
                    && to_definitions == [detour_definitions.clone()]
                {
                    "start_to_detour"
                } else if from_definitions == [detour_definitions.clone()]
                    && to_definitions == [end_definitions.clone()]
                {
                    "detour_to_end"
                } else {
                    panic!("unexpected trace leg")
                };
                trace_calls.push(call);
                if call == "start_to_detour" {
                    Ok(false)
                } else {
                    Ok(true)
                }
            },
            &mut Vec::new(),
            &mut |_from, _to| Ok(Vec::new()),
            &mut saw_unknown,
        )
        .unwrap();

        assert!(!result);
        assert!(!saw_unknown);
        assert_eq!(
            trace_calls,
            vec![
                "start_to_detour",
                "detour_to_end",
                "detour_to_end",
                "start_to_detour",
            ]
        );
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
        assert!(!targets[0].uncertified_definition_fallback);
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
    fn detour_target_build_collection_keeps_later_targets_certified_after_uncertain_candidate_result()
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
                .any(|target| !target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn detour_target_build_collection_keeps_certified_duplicate_state_certified() {
        let point = p(1, 2, 3);
        let definition = axis_plane_definition(&point);
        let mut targets = Vec::new();

        extend_detour_target_builds_backtracking_unknown(
            &mut targets,
            [0, 1].iter(),
            |candidate| {
                Ok(DetourTarget {
                    point: point.clone(),
                    definitions: vec![definition.clone()],
                    uncertified_definition_fallback: *candidate == 0,
                })
            },
        )
        .unwrap();

        assert_eq!(targets.len(), 1);
        assert!(!targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn detour_target_from_shifted_witness_stays_certified_when_one_family_is_singular() {
        let witness = ShiftedHalfspaceWitness {
            point: p(1, 1, 1),
            families: vec![
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(2, false, r(2))],
                    active_planes: [Some(9), None, None],
                },
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(0, false, r(2))],
                    active_planes: [Some(0), None, None],
                },
            ],
            uncertified_definition_fallback: false,
        };

        let target = build_detour_target_from_shifted_witness(&witness).unwrap();

        assert_eq!(target.point, witness.point);
        assert!(!target.uncertified_definition_fallback);
        assert!(!target.definitions.is_empty());
    }

    #[test]
    fn detour_target_family_collection_keeps_later_targets_certified_after_uncertified_family() {
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
        assert!(!targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn detour_target_family_collection_keeps_later_targets_certified_after_uncertain_family_result()
    {
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
                .any(|target| !target.uncertified_definition_fallback)
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
        assert!(!targets[0].uncertified_definition_fallback);
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
    fn interior_box_detour_target_collection_keeps_surviving_targets_certified_after_uncertified_surface_cut()
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
        assert!(!targets[0].uncertified_definition_fallback);
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
        assert_eq!(intervals[0], vec![(r(0), r(2)), (r(2), r(3))]);
    }

    #[test]
    fn axis_box_surface_cut_collection_treats_endpoint_boundary_contact_as_unknown_and_keeps_later_cut()
     {
        let start = p(0, 0, 0);
        let end = p(3, 0, 0);
        let first = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let (intervals, saw_unknown) = interior_box_axis_intervals_with_surface_queries(
            &start,
            &end,
            &[first, second],
            &mut |_edge_start, edge_end, polygon, _axis| {
                let x = polygon.vertices().unwrap()[0].x.clone();
                if x == r(3) {
                    Ok(Some(edge_end.clone()))
                } else {
                    Ok(Some(Point3::new(x, r(0), r(0))))
                }
            },
            &mut |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(3) {
                    Ok(PolygonPointLocation::Boundary)
                } else {
                    Ok(PolygonPointLocation::Interior)
                }
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(intervals[0], vec![(r(0), r(2)), (r(2), r(3))]);
    }

    #[test]
    fn axis_box_surface_cut_collection_treats_start_boundary_contact_as_unknown_and_keeps_later_cut()
     {
        let start = p(0, 0, 0);
        let end = p(3, 0, 0);
        let first = make_triangle(&p(0, 0, 0), &p(0, 1, 0), &p(0, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let (intervals, saw_unknown) = interior_box_axis_intervals_with_surface_queries(
            &start,
            &end,
            &[first, second],
            &mut |edge_start, _edge_end, polygon, _axis| {
                let x = polygon.vertices().unwrap()[0].x.clone();
                if x == r(0) {
                    Ok(Some(edge_start.clone()))
                } else {
                    Ok(Some(Point3::new(x, r(0), r(0))))
                }
            },
            &mut |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(0) {
                    Ok(PolygonPointLocation::Boundary)
                } else {
                    Ok(PolygonPointLocation::Interior)
                }
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(intervals[0], vec![(r(0), r(2)), (r(2), r(3))]);
    }

    #[test]
    fn interior_box_detour_target_collection_keeps_surviving_targets_certified_after_boundary_surface_cut()
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
        assert!(!targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn interior_box_detour_target_collection_keeps_surviving_targets_certified_after_endpoint_boundary_surface_cut()
     {
        let start = p(0, 0, 0);
        let end = p(3, 0, 0);
        let first = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let targets = interior_box_detour_targets_with_queries(
            &start,
            &end,
            &[first, second],
            |_edge_start, edge_end, polygon, _axis| {
                let x = polygon.vertices().unwrap()[0].x.clone();
                if x == r(3) {
                    Ok(Some(edge_end.clone()))
                } else {
                    Ok(Some(Point3::new(x, r(0), r(0))))
                }
            },
            |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(3) {
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
        assert!(!targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn interior_box_detour_target_collection_keeps_surviving_targets_certified_after_start_boundary_surface_cut()
     {
        let start = p(0, 0, 0);
        let end = p(3, 0, 0);
        let first = make_triangle(&p(0, 0, 0), &p(0, 1, 0), &p(0, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let targets = interior_box_detour_targets_with_queries(
            &start,
            &end,
            &[first, second],
            |edge_start, _edge_end, polygon, _axis| {
                let x = polygon.vertices().unwrap()[0].x.clone();
                if x == r(0) {
                    Ok(Some(edge_start.clone()))
                } else {
                    Ok(Some(Point3::new(x, r(0), r(0))))
                }
            },
            |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(0) {
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
        assert!(!targets[0].uncertified_definition_fallback);
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
    fn duplicate_detour_targets_prefer_certified_duplicate_definitions() {
        let point = p(1, 1, 1);
        let definition = axis_plane_definition(&point);
        let mut targets = vec![DetourTarget {
            point: point.clone(),
            definitions: vec![definition.clone()],
            uncertified_definition_fallback: true,
        }];

        push_unique_detour_target(
            &mut targets,
            DetourTarget {
                point,
                definitions: vec![definition],
                uncertified_definition_fallback: false,
            },
        );

        assert_eq!(targets.len(), 1);
        assert!(!targets[0].uncertified_definition_fallback);
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
        assert!(!witnesses[0].uncertified_definition_fallback);
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
        assert!(!witnesses[0].uncertified_definition_fallback);
    }

    #[test]
    fn shifted_halfspace_witness_collection_keeps_later_witnesses_certified_after_uncertain_candidate_result()
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
                .any(|witness| !witness.uncertified_definition_fallback)
        );
    }

    #[test]
    fn shifted_halfspace_witness_collection_keeps_certified_duplicate_state_certified() {
        let first_seed = p(1, 1, 1);
        let second_seed = p(2, 2, 2);
        let witness_point = p(3, 3, 3);
        let mut witnesses = Vec::new();

        extend_shifted_halfspace_witnesses_backtracking_unknown(
            &mut witnesses,
            [first_seed, second_seed],
            |seed| {
                Ok(vec![ShiftedHalfspaceWitness::with_family(
                    witness_point.clone(),
                    Vec::new(),
                    [None, None, None],
                    *seed == p(1, 1, 1),
                )])
            },
        )
        .unwrap();

        assert_eq!(witnesses.len(), 1);
        assert_eq!(witnesses[0].point, witness_point);
        assert!(!witnesses[0].uncertified_definition_fallback);
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
    fn duplicate_shifted_halfspace_witnesses_prefer_certified_duplicate_families() {
        let point = p(1, 1, 1);
        let mut witnesses = vec![ShiftedHalfspaceWitness::with_family(
            point.clone(),
            Vec::new(),
            [None, None, None],
            true,
        )];

        push_unique_shifted_halfspace_witness(
            &mut witnesses,
            ShiftedHalfspaceWitness::with_family(point, Vec::new(), [None, None, None], false),
        );

        assert_eq!(witnesses.len(), 1);
        assert!(!witnesses[0].uncertified_definition_fallback);
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
    fn collect_strict_halfspace_seed_family_tracks_unknown_after_halfspace_boundary_candidate() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let family =
            collect_strict_halfspace_seed_family(Ok(vec![p(0, 2, 2), p(1, 1, 1)]), |candidate| {
                point_strictly_inside_halfspace_cell_or_unknown(candidate, &bounds, &halfspaces)
            })
            .unwrap();

        assert_eq!(family.seeds, vec![p(1, 1, 1)]);
        assert!(family.saw_unknown);
    }

    #[test]
    fn collect_strict_halfspace_seed_family_tracks_unknown_after_leaf_boundary_candidate() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);

        let family =
            collect_strict_halfspace_seed_family(Ok(vec![p(3, 0, 0), p(1, 1, 1)]), |candidate| {
                point_strictly_inside_leaf_or_unknown(candidate, &leaf)
            })
            .unwrap();

        assert_eq!(family.seeds, vec![p(1, 1, 1)]);
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
    fn shifted_halfspace_seed_families_with_report_seed_promote_report_witness_to_shifted_root() {
        let witness = p(1, 1, 1);
        let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
            shifted_halfspace_seed_families_with_report_seed(
                Some(&witness),
                vec![p(2, 1, 1)],
                vec![p(2, 1, 1), witness.clone(), p(3, 1, 1)],
                vec![p(3, 1, 1), witness.clone(), p(4, 1, 1)],
            );

        assert_eq!(strict_seeds, vec![p(2, 1, 1), witness]);
        assert_eq!(shifted_vertices, vec![p(3, 1, 1)]);
        assert_eq!(shifted_geometry_seeds, vec![p(4, 1, 1)]);
    }

    #[test]
    fn shifted_halfspace_seed_families_with_report_seed_skip_later_duplicates() {
        let witness = p(1, 1, 1);
        let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
            shifted_halfspace_seed_families_with_report_seed(
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
    fn shifted_halfspace_witness_seed_family_search_keeps_existing_witnesses_certified_after_later_unknown()
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
        assert!(!witnesses[0].uncertified_definition_fallback);
    }

    #[test]
    fn shifted_halfspace_witness_seed_family_search_keeps_later_witnesses_certified_after_uncertain_family_result()
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
                .any(|witness| !witness.uncertified_definition_fallback)
        );
    }

    #[test]
    fn shifted_halfspace_witness_seed_family_search_keeps_certified_duplicate_state_certified() {
        let witness_point = p(3, 3, 3);
        let mut witnesses = Vec::new();

        extend_shifted_halfspace_seed_families_backtracking_unknown(
            &mut witnesses,
            [vec![p(1, 1, 1)], vec![p(2, 2, 2)]],
            |seed| {
                Ok(vec![ShiftedHalfspaceWitness::with_family(
                    witness_point.clone(),
                    Vec::new(),
                    [None, None, None],
                    *seed == p(1, 1, 1),
                )])
            },
        )
        .unwrap();

        assert_eq!(witnesses.len(), 1);
        assert_eq!(witnesses[0].point, witness_point);
        assert!(!witnesses[0].uncertified_definition_fallback);
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
    fn halfspace_cell_seed_families_track_unknown_after_boundary_vertex_candidate() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut saw_unknown = false;

        let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
            halfspace_cell_seed_families_from_optional_report(
                &bounds,
                &halfspaces,
                None,
                &mut saw_unknown,
            )
            .unwrap();

        assert!(saw_unknown);
        assert!(!strict_seeds.is_empty());
        assert!(!shifted_vertices.is_empty());
        assert!(!shifted_geometry_seeds.is_empty());
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
    fn shifted_halfspace_witnesses_keep_certified_survivors_after_boundary_seed_candidate() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let witnesses =
            shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, &p(1, 1, 1)).unwrap();

        assert!(!witnesses.is_empty());
        assert!(
            witnesses
                .iter()
                .any(|witness| !witness.uncertified_definition_fallback)
        );
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
    fn bounded_probes_find_positive_probe_for_core_leaf_wall_case() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
        let interior_points = certified_leaf_interior_points(&wall.support, &wall.edges).unwrap();

        assert!(!interior_points.is_empty());
        assert!(interior_points.iter().any(|point| !point.planes.is_empty()));

        let probes = bounded_probes_from_interior(
            &interior_points[0],
            &wall.support,
            &bounds,
            true,
            &[wall.clone()],
        )
        .unwrap();

        assert!(
            probes
                .iter()
                .any(|probe| probe.side == Classification::Positive)
        );
    }

    #[test]
    fn bounded_probes_keep_positive_probe_before_intervening_surface() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let mut blocker = make_triangle(&p(2, -10, -10), &p(2, 10, -10), &p(2, 0, 10), 1, 0);
        blocker.delta_w = vec![1];
        let bounds = Aabb::new(p(1, -2, -2), p(5, 2, 2));
        let interior_points = certified_leaf_interior_points(&wall.support, &wall.edges).unwrap();

        let probes = bounded_probes_from_interior(
            &interior_points[0],
            &wall.support,
            &bounds,
            true,
            &[wall.clone(), blocker],
        )
        .unwrap();

        assert!(
            probes
                .iter()
                .any(|probe| probe.side == Classification::Positive)
        );
    }

    #[test]
    fn positive_probe_traces_for_core_leaf_wall_case() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
        let ref_point = p(0, 0, 0);
        let ref_definitions = vec![axis_plane_definition(&ref_point)];
        let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("leaf should have a replayable interior witness");
        assert!(!interior.uncertified_definition_fallback);
        let probe =
            bounded_probes_from_interior(&interior, &wall.support, &bounds, true, &[wall.clone()])
                .unwrap()
                .into_iter()
                .find(|probe| probe.side == Classification::Positive)
                .expect("leaf should have a positive-side probe");
        assert!(!probe.uncertified_definition_fallback);

        assert!(!point_lies_on_traced_surface(&probe.point, &[wall.clone()]).unwrap());
        assert!(
            probe_reaches_adjacent_cell_from_interior(
                &interior,
                &probe,
                &wall.support,
                &[wall.clone()],
            )
            .unwrap()
        );

        let winding =
            trace_probe_winding(&ref_point, &ref_definitions, &probe, &[0], &[wall.clone()])
                .unwrap();

        assert_eq!(winding.len(), 1);
    }

    #[test]
    fn trace_probe_winding_with_query_caches_reuses_lower_trace_state_for_core_leaf_wall_case() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let ref_point = p(0, 0, 0);
        let ref_definitions = vec![axis_plane_definition(&ref_point)];
        let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("leaf should have a replayable interior witness");
        let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
        let probe =
            bounded_probes_from_interior(&interior, &wall.support, &bounds, true, &[wall.clone()])
                .unwrap()
                .into_iter()
                .find(|probe| probe.side == Classification::Positive)
                .expect("leaf should have a positive-side probe");
        let mut query_caches = LeafProbeQueryCaches::default();
        let LeafProbeQueryCaches {
            probe_surface,
            axis_ordered_segment_traces,
            plane_replacement_affine,
            plane_replacement_trace_steps,
            definition_no_detour_trace,
            detour_target_families,
            ..
        } = &mut query_caches;

        let first = trace_probe_winding_with_caches(
            &ref_point,
            &ref_definitions,
            &probe,
            &[0],
            &[wall.clone()],
            probe_surface,
            axis_ordered_segment_traces,
            plane_replacement_affine,
            plane_replacement_trace_steps,
            definition_no_detour_trace,
            detour_target_families,
        )
        .unwrap();
        let after_first = (
            probe_surface.len(),
            axis_ordered_segment_traces.len(),
            plane_replacement_affine.len(),
            plane_replacement_trace_steps.len(),
            definition_no_detour_trace.len(),
            detour_target_families.len(),
        );

        let second = trace_probe_winding_with_caches(
            &ref_point,
            &ref_definitions,
            &probe,
            &[0],
            &[wall],
            probe_surface,
            axis_ordered_segment_traces,
            plane_replacement_affine,
            plane_replacement_trace_steps,
            definition_no_detour_trace,
            detour_target_families,
        )
        .unwrap();
        let after_second = (
            probe_surface.len(),
            axis_ordered_segment_traces.len(),
            plane_replacement_affine.len(),
            plane_replacement_trace_steps.len(),
            definition_no_detour_trace.len(),
            detour_target_families.len(),
        );

        assert_eq!(first, second);
        assert_eq!(after_first, after_second);
    }

    #[test]
    fn probe_reachability_with_query_caches_reuses_lower_trace_state_for_core_leaf_wall_case() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
        let support = wall.support.clone();
        let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("leaf should have a replayable interior witness");
        let probe =
            bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
                .unwrap()
                .into_iter()
                .find(|probe| probe.side == Classification::Positive)
                .expect("leaf should have a positive-side probe");
        let mut query_caches = LeafProbeQueryCaches::default();
        let LeafProbeQueryCaches {
            probe_surface,
            plane_replacement_affine,
            plane_replacement_reachability_paths,
            plane_replacement_reachability_steps,
            plane_replacement_no_nested_ordering_warmups,
            definition_cycle_guard_reachability,
            definition_no_step_detour_reachability,
            definition_no_plane_replacement_cycle_guard,
            definition_no_plane_replacement_reachability,
            no_step_detour_target_families,
            definition_no_detour_reachability,
            direct_probe_reachability,
            detour_target_families,
            ..
        } = &mut query_caches;

        let first = probe_reaches_adjacent_cell_from_interior_with_caches(
            &interior,
            &probe,
            &support,
            &[wall.clone()],
            probe_surface,
            plane_replacement_affine,
            plane_replacement_reachability_paths,
            plane_replacement_reachability_steps,
            plane_replacement_no_nested_ordering_warmups,
            definition_cycle_guard_reachability,
            definition_no_step_detour_reachability,
            definition_no_plane_replacement_cycle_guard,
            definition_no_plane_replacement_reachability,
            no_step_detour_target_families,
            definition_no_detour_reachability,
            direct_probe_reachability,
            detour_target_families,
        )
        .unwrap();
        let after_first = (
            probe_surface.len(),
            plane_replacement_affine.len(),
            plane_replacement_reachability_paths.len(),
            plane_replacement_reachability_steps.len(),
            plane_replacement_no_nested_ordering_warmups.len(),
            definition_no_step_detour_reachability.len(),
            definition_no_plane_replacement_reachability.len(),
            no_step_detour_target_families.len(),
            definition_no_detour_reachability.len(),
            direct_probe_reachability.len(),
            detour_target_families.len(),
        );

        let second = probe_reaches_adjacent_cell_from_interior_with_caches(
            &interior,
            &probe,
            &support,
            &[wall],
            probe_surface,
            plane_replacement_affine,
            plane_replacement_reachability_paths,
            plane_replacement_reachability_steps,
            plane_replacement_no_nested_ordering_warmups,
            definition_cycle_guard_reachability,
            definition_no_step_detour_reachability,
            definition_no_plane_replacement_cycle_guard,
            definition_no_plane_replacement_reachability,
            no_step_detour_target_families,
            definition_no_detour_reachability,
            direct_probe_reachability,
            detour_target_families,
        )
        .unwrap();
        let after_second = (
            probe_surface.len(),
            plane_replacement_affine.len(),
            plane_replacement_reachability_paths.len(),
            plane_replacement_reachability_steps.len(),
            plane_replacement_no_nested_ordering_warmups.len(),
            definition_no_step_detour_reachability.len(),
            definition_no_plane_replacement_reachability.len(),
            no_step_detour_target_families.len(),
            definition_no_detour_reachability.len(),
            direct_probe_reachability.len(),
            detour_target_families.len(),
        );

        assert_eq!(first, second);
        assert_eq!(after_first, after_second);
    }

    #[test]
    fn no_step_definition_search_caches_whole_query_for_core_leaf_wall_case() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
        let support = wall.support.clone();
        let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("leaf should have a replayable interior witness");
        let probe =
            bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
                .unwrap()
                .into_iter()
                .find(|probe| probe.side == Classification::Positive)
                .expect("leaf should have a positive-side probe");
        let mut affine_cache = Vec::new();
        let mut path_cache = Vec::new();
        let mut step_cache = Vec::new();
        let mut no_step_cache = Vec::new();
        let mut direct_probe_reachability_cache = Vec::new();

        let first = probe_reaches_adjacent_cell_with_definitions_no_step_detours_with_caches(
            &interior.point,
            &probe.point,
            &support,
            &[wall.clone()],
            &interior.planes,
            &probe.planes,
            &mut affine_cache,
            &mut path_cache,
            &mut step_cache,
            &mut no_step_cache,
            &mut direct_probe_reachability_cache,
        )
        .unwrap();
        let after_first = (
            affine_cache.len(),
            path_cache.len(),
            step_cache.len(),
            no_step_cache.len(),
            direct_probe_reachability_cache.len(),
        );

        let second = probe_reaches_adjacent_cell_with_definitions_no_step_detours_with_caches(
            &interior.point,
            &probe.point,
            &support,
            &[wall],
            &interior.planes,
            &probe.planes,
            &mut affine_cache,
            &mut path_cache,
            &mut step_cache,
            &mut no_step_cache,
            &mut direct_probe_reachability_cache,
        )
        .unwrap();
        let after_second = (
            affine_cache.len(),
            path_cache.len(),
            step_cache.len(),
            no_step_cache.len(),
            direct_probe_reachability_cache.len(),
        );

        assert_eq!(first, second);
        assert_eq!(no_step_cache.len(), 1);
        assert_eq!(after_first, after_second);
    }

    #[test]
    fn adjacent_normal_probes_stay_certified_for_core_leaf_wall_case() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
        let support = wall.support.clone();
        let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("leaf should have a replayable interior witness");

        let probes = adjacent_normal_probes(&interior, &support, &bounds, &[wall], true).unwrap();

        assert!(
            probes
                .iter()
                .any(|probe| !probe.uncertified_definition_fallback)
        );
    }

    #[test]
    fn strict_normal_probe_targets_stay_certified_for_core_leaf_wall_case() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
        let support = wall.support.clone();
        let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("leaf should have a replayable interior witness");
        let existing_probe =
            bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
                .unwrap()
                .into_iter()
                .find(|probe| probe.side == Classification::Positive)
                .expect("leaf should have a positive-side probe");
        let corridor = bounds_between_points(&interior.point, &existing_probe.point).unwrap();

        let probes = strict_normal_probe_targets(
            &interior,
            &support,
            &corridor,
            Some(&interior.planes[0]),
            &existing_probe.point,
            true,
        )
        .unwrap();

        assert!(
            probes
                .iter()
                .any(|probe| !probe.uncertified_definition_fallback)
        );
    }

    #[test]
    fn strict_normal_probe_direct_seed_phase_stays_certified_for_core_leaf_wall_case() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
        let support = wall.support.clone();
        let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("leaf should have a replayable interior witness");
        let existing_probe =
            bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
                .unwrap()
                .into_iter()
                .find(|probe| probe.side == Classification::Positive)
                .expect("leaf should have a positive-side probe");
        let corridor = bounds_between_points(&interior.point, &existing_probe.point).unwrap();

        let mut halfspaces = aabb_core_halfspaces(&corridor).unwrap();
        halfspaces.push(support_side_halfspace(&support, true));
        halfspaces.push(normal_stop_halfspace(&support, &existing_probe.point, true));
        let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces).unwrap();
        assert!(!saw_unknown);
        let (seeds, shifted_vertices, shifted_geometry_seeds) =
            halfspace_cell_seed_families_from_optional_report(
                &corridor,
                &halfspaces,
                report.as_ref(),
                &mut saw_unknown,
            )
            .unwrap();

        let probes = strict_normal_probe_targets_from_seed_families_with_tracking_unknown(
            &interior,
            &support,
            &corridor,
            Some(&interior.planes[0]),
            &existing_probe.point,
            true,
            report.as_ref(),
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
            |_seed| Ok(Vec::new()),
        )
        .unwrap();

        assert!(
            probes
                .iter()
                .any(|probe| !probe.uncertified_definition_fallback)
        );
    }

    #[test]
    fn direct_normal_probe_seed_build_stays_certified_for_core_leaf_wall_case() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
        let support = wall.support.clone();
        let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("leaf should have a replayable interior witness");
        let existing_probe =
            bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
                .unwrap()
                .into_iter()
                .find(|probe| probe.side == Classification::Positive)
                .expect("leaf should have a positive-side probe");
        let corridor = bounds_between_points(&interior.point, &existing_probe.point).unwrap();

        let mut halfspaces = aabb_core_halfspaces(&corridor).unwrap();
        halfspaces.push(support_side_halfspace(&support, true));
        halfspaces.push(normal_stop_halfspace(&support, &existing_probe.point, true));
        let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces).unwrap();
        assert!(!saw_unknown);
        let (seeds, _shifted_vertices, _shifted_geometry_seeds) =
            halfspace_cell_seed_families_from_optional_report(
                &corridor,
                &halfspaces,
                report.as_ref(),
                &mut saw_unknown,
            )
            .unwrap();

        let mut extra_planes = Vec::new();
        for definition in &interior.planes {
            for plane in &definition[1..] {
                if !extra_planes.iter().any(|existing| existing == plane) {
                    extra_planes.push(plane.clone());
                }
            }
        }

        let built = seeds
            .iter()
            .filter_map(|seed| {
                build_probe_point(
                    seed,
                    &corridor,
                    &support,
                    &halfspaces,
                    active_planes_from_optional_report(report.as_ref(), seed),
                    &extra_planes,
                    false,
                )
                .unwrap()
            })
            .collect::<Vec<_>>();

        assert!(!built.is_empty());
        assert!(
            built
                .iter()
                .any(|probe| !probe.uncertified_definition_fallback)
        );
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

        let merged_unknown = saw_unknown
            || probes
                .iter()
                .any(|probe| probe.uncertified_definition_fallback);
        assert!(merged_unknown);
        assert_eq!(probes.len(), 2);
    }

    #[test]
    fn bounded_probe_family_collection_keeps_certified_duplicate_state_certified() {
        let point = p(1, 1, 1);
        let mut probes = Vec::new();
        let mut saw_unknown = false;

        extend_probe_families_backtracking_unknown(
            &mut probes,
            Ok(vec![ProbePoint {
                point: point.clone(),
                side: Classification::Positive,
                planes: vec![axis_plane_definition(&point)],
                uncertified_definition_fallback: true,
            }]),
            &mut saw_unknown,
        )
        .unwrap();
        extend_probe_families_backtracking_unknown(
            &mut probes,
            Ok(vec![ProbePoint {
                point: point.clone(),
                side: Classification::Positive,
                planes: vec![axis_plane_definition(&point)],
                uncertified_definition_fallback: false,
            }]),
            &mut saw_unknown,
        )
        .unwrap();

        let merged_unknown = saw_unknown
            || probes
                .iter()
                .any(|probe| probe.uncertified_definition_fallback);
        assert!(!merged_unknown);
        assert_eq!(probes.len(), 1);
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
    fn leaf_probe_family_search_skips_fallback_probe_even_when_winding_succeeds() {
        let point = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let fallback_probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: true,
        };
        let certified_probe = ProbePoint {
            point: p(3, 3, 3),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(3, 3, 3))],
            uncertified_definition_fallback: false,
        };

        let winding = search_leaf_probe_families(
            &[point],
            |_point, positive_side| {
                if positive_side {
                    Ok(vec![fallback_probe.clone(), certified_probe.clone()])
                } else {
                    Ok(Vec::new())
                }
            },
            |_point, _positive_side, probe| {
                if probe.point == fallback_probe.point {
                    Ok(Some(vec![11]))
                } else {
                    Ok(Some(vec![13]))
                }
            },
        )
        .unwrap();

        assert_eq!(winding, Some(vec![13]));
    }

    #[test]
    fn leaf_probe_family_search_reports_unknown_when_only_fallback_probe_succeeds() {
        let point = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let fallback_probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: true,
        };

        let err = search_leaf_probe_families(
            &[point],
            |_point, _positive_side| Ok(vec![fallback_probe.clone()]),
            |_point, _positive_side, _probe| Ok(Some(vec![11])),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn leaf_probe_family_search_skips_fallback_interior_even_when_winding_succeeds() {
        let fallback_point = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: true,
        };
        let certified_point = InteriorLeafPoint {
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
            &[fallback_point.clone(), certified_point.clone()],
            |_point, _positive_side| Ok(vec![probe.clone()]),
            |point, _positive_side, _probe| {
                if point.point == fallback_point.point {
                    Ok(Some(vec![17]))
                } else {
                    Ok(Some(vec![19]))
                }
            },
        )
        .unwrap();

        assert_eq!(winding, Some(vec![19]));
    }

    #[test]
    fn leaf_probe_family_search_reports_unknown_when_only_fallback_interior_succeeds() {
        let fallback_point = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: true,
        };
        let probe = ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: false,
        };

        let err = search_leaf_probe_families(
            &[fallback_point],
            |_point, _positive_side| Ok(vec![probe.clone()]),
            |_point, _positive_side, _probe| Ok(Some(vec![17])),
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
    fn cached_bounded_probes_from_interior_reuse_equivalent_queries() {
        let interior = InteriorLeafPoint {
            point: p(0, 0, 0),
            planes: vec![axis_plane_definition(&p(0, 0, 0))],
            uncertified_definition_fallback: false,
        };
        let support = Plane::axis_aligned(0, r(0));
        let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
        let probe = ProbePoint {
            point: p(1, 0, 0),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(1, 0, 0))],
            uncertified_definition_fallback: false,
        };
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_bounded_probes_from_interior_with(
            &mut cache,
            &interior,
            &support,
            &bounds,
            true,
            || {
                calls += 1;
                Ok(vec![probe.clone()])
            },
        )
        .unwrap();
        let second = cached_bounded_probes_from_interior_with(
            &mut cache,
            &interior,
            &support,
            &bounds,
            true,
            || {
                calls += 1;
                Ok(Vec::new())
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, vec![probe.clone()]);
        assert_eq!(second, vec![probe]);
    }

    #[test]
    fn cached_adjacent_normal_probes_reuse_equivalent_queries() {
        let interior = InteriorLeafPoint {
            point: p(0, 0, 0),
            planes: vec![axis_plane_definition(&p(0, 0, 0))],
            uncertified_definition_fallback: false,
        };
        let support = Plane::axis_aligned(0, r(0));
        let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
        let probe = ProbePoint {
            point: p(1, 0, 0),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(1, 0, 0))],
            uncertified_definition_fallback: false,
        };
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_adjacent_normal_probes_with(
            &mut cache,
            &interior,
            &support,
            &bounds,
            true,
            || {
                calls += 1;
                Ok(vec![probe.clone()])
            },
        )
        .unwrap();
        let second = cached_adjacent_normal_probes_with(
            &mut cache,
            &interior,
            &support,
            &bounds,
            true,
            || {
                calls += 1;
                Ok(Vec::new())
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, vec![probe.clone()]);
        assert_eq!(second, vec![probe]);
    }

    #[test]
    fn cached_adjacent_axis_probes_reuse_equivalent_queries() {
        let interior = InteriorLeafPoint {
            point: p(0, 0, 0),
            planes: vec![axis_plane_definition(&p(0, 0, 0))],
            uncertified_definition_fallback: false,
        };
        let support = Plane::axis_aligned(0, r(0));
        let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
        let probe = ProbePoint {
            point: p(0, 1, 0),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(0, 1, 0))],
            uncertified_definition_fallback: false,
        };
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_adjacent_axis_probes_with(
            &mut cache,
            &interior,
            &support,
            &bounds,
            1,
            true,
            || {
                calls += 1;
                Ok(vec![probe.clone()])
            },
        )
        .unwrap();
        let second = cached_adjacent_axis_probes_with(
            &mut cache,
            &interior,
            &support,
            &bounds,
            1,
            true,
            || {
                calls += 1;
                Ok(Vec::new())
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, vec![probe.clone()]);
        assert_eq!(second, vec![probe]);
    }

    #[test]
    fn cached_halfspace_cell_seed_families_reuse_permuted_halfspaces() {
        let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let (report, mut first_unknown) =
            optional_halfspace_feasibility_report(&halfspaces).unwrap();
        let mut cache = Vec::new();

        let first = cached_halfspace_cell_seed_families_from_optional_report_with(
            &mut cache,
            &bounds,
            &halfspaces,
            report.as_ref(),
            &mut first_unknown,
        )
        .unwrap();

        let mut permuted_halfspaces = halfspaces.clone();
        permuted_halfspaces.rotate_left(2);
        let (permuted_report, mut second_unknown) =
            optional_halfspace_feasibility_report(&permuted_halfspaces).unwrap();
        let second = cached_halfspace_cell_seed_families_from_optional_report_with(
            &mut cache,
            &bounds,
            &permuted_halfspaces,
            permuted_report.as_ref(),
            &mut second_unknown,
        )
        .unwrap();

        assert_eq!(first, second);
        assert_eq!(cache.len(), 1);
        assert_eq!(first_unknown, second_unknown);
    }

    #[test]
    fn cached_optional_halfspace_feasibility_report_reuses_permuted_halfspaces() {
        let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut cache = Vec::new();
        let mut first_unknown = false;

        let first = cached_optional_halfspace_feasibility_report_with(
            &mut cache,
            &halfspaces,
            &mut first_unknown,
        )
        .unwrap();

        let mut permuted_halfspaces = halfspaces.clone();
        permuted_halfspaces.rotate_left(2);
        let mut second_unknown = false;
        let second = cached_optional_halfspace_feasibility_report_with(
            &mut cache,
            &permuted_halfspaces,
            &mut second_unknown,
        )
        .unwrap();

        assert_eq!(first, second);
        assert_eq!(cache.len(), 1);
        assert_eq!(first_unknown, second_unknown);
    }

    #[test]
    fn cached_adjacent_normal_probe_stop_values_reuse_equivalent_query() {
        let mut cache = Vec::new();
        let interior = p(0, 0, 0);
        let direction = p(0, 0, 1);
        let support = Plane::new(p(0, 0, 1), Real::from(0));
        let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
        let mut calls = 0;

        let first = cached_adjacent_normal_probe_stop_values_with(
            &mut cache,
            &interior,
            &direction,
            &support,
            &bounds,
            || {
                calls += 1;
                Ok((vec![r(1), r(2)], true))
            },
        )
        .unwrap();
        let second = cached_adjacent_normal_probe_stop_values_with(
            &mut cache,
            &interior,
            &direction,
            &support,
            &bounds,
            || {
                calls += 1;
                Ok((vec![r(3)], false))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, (vec![r(1), r(2)], true));
        assert_eq!(second, first);
        assert_eq!(cache.len(), 1);
    }

    #[test]
    fn cached_adjacent_axis_probe_stop_values_reuse_equivalent_query() {
        let mut cache = Vec::new();
        let interior = p(0, 0, 0);
        let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
        let mut calls = 0;

        let first = cached_adjacent_axis_probe_stop_values_with(
            &mut cache,
            &interior,
            &bounds,
            2,
            true,
            || {
                calls += 1;
                Ok((vec![r(1), r(2)], true))
            },
        )
        .unwrap();
        let second = cached_adjacent_axis_probe_stop_values_with(
            &mut cache,
            &interior,
            &bounds,
            2,
            true,
            || {
                calls += 1;
                Ok((vec![r(3)], false))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, (vec![r(1), r(2)], true));
        assert_eq!(second, first);
        assert_eq!(cache.len(), 1);
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
    fn cached_probe_reachability_reuses_in_progress_exact_state() {
        let interior = InteriorLeafPoint {
            point: p(0, 0, 0),
            planes: vec![axis_plane_definition(&p(0, 0, 0))],
            uncertified_definition_fallback: false,
        };
        let probe = ProbePoint {
            point: p(1, 0, 0),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(1, 0, 0))],
            uncertified_definition_fallback: false,
        };
        let mut cache = vec![ProbeReachabilityCacheEntry {
            interior_point: interior.point.clone(),
            interior_planes: interior.planes.clone(),
            probe_point: probe.point.clone(),
            probe_planes: probe.planes.clone(),
            reachable: Err(HypermeshError::UnknownClassification),
        }];

        let result = cached_probe_reachability_with(&mut cache, &interior, &probe, || Ok(true));

        assert_eq!(result, Err(HypermeshError::UnknownClassification));
        assert_eq!(cache.len(), 1);
        assert_eq!(
            cache[0].reachable,
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn cached_direct_probe_reachability_reuses_identical_query() {
        let mut cache = Vec::new();
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let host_support = Plane::axis_aligned(2, r(0));
        let polygons = vec![make_triangle(
            &p(2, -1, -1),
            &p(2, 1, -1),
            &p(2, 0, 1),
            0,
            0,
        )];
        let mut calls = 0;

        let first = cached_direct_probe_reachability_with(
            &mut cache,
            &start,
            &end,
            &host_support,
            &polygons,
            || {
                calls += 1;
                Ok(true)
            },
        )
        .unwrap();
        let second = cached_direct_probe_reachability_with(
            &mut cache,
            &start,
            &end,
            &host_support,
            &polygons,
            || {
                calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert!(first);
        assert!(second);
        assert_eq!(cache.len(), 1);
    }

    #[test]
    fn cached_direct_probe_reachability_reuses_reversed_query() {
        let mut cache = Vec::new();
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let host_support = Plane::axis_aligned(2, r(0));
        let polygons = vec![make_triangle(
            &p(2, -1, -1),
            &p(2, 1, -1),
            &p(2, 0, 1),
            0,
            0,
        )];
        let mut calls = 0;

        let first = cached_direct_probe_reachability_with(
            &mut cache,
            &start,
            &end,
            &host_support,
            &polygons,
            || {
                calls += 1;
                Ok(true)
            },
        )
        .unwrap();
        let second = cached_direct_probe_reachability_with(
            &mut cache,
            &end,
            &start,
            &host_support,
            &polygons,
            || {
                calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert!(first);
        assert!(second);
        assert_eq!(calls, 1);
        assert_eq!(cache.len(), 1);
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
    fn trace_axis_ordered_paths_try_later_ordering_after_boundary_surface_query() {
        let start = p(0, 0, 0);
        let end = p(1, 1, 0);
        let polygon = make_triangle(&p(1, 0, 0), &p(2, 0, 0), &p(1, 1, 0), 0, 0);

        let winding = trace_axis_ordered_paths_with_surface_query(
            &start,
            &end,
            &[7],
            std::slice::from_ref(&polygon),
            |point| point_lies_on_traced_surface(point, std::slice::from_ref(&polygon)),
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
    fn trace_axis_ordered_paths_reports_unknown_for_zero_length_surface_contact() {
        let start = p(0, 0, 0);

        let err = trace_axis_ordered_paths_with_queries(
            &start,
            &start,
            &[7],
            &[],
            |_point| Ok(true),
            |_current, _next, _axis, _attempt, _polygons| {
                panic!("zero-length trace should not issue a segment step")
            },
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn trace_axis_ordered_paths_try_later_ordering_after_endpoint_surface_contact() {
        let start = p(0, 0, 0);
        let end = p(1, 1, 0);
        let polygon = make_triangle(&p(1, 0, 0), &p(1, -1, 1), &p(1, 1, 1), 0, 0);

        let winding = trace_axis_ordered_paths_with_queries(
            &start,
            &end,
            &[7],
            std::slice::from_ref(&polygon),
            |_point| Ok(false),
            |current, next, axis, attempt, polygons| {
                trace_axis_segment(current, next, axis, attempt, polygons)
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
    fn cached_definition_no_detour_reachability_reuses_reversed_query() {
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
            &end,
            &start,
            &end_definitions,
            &start_definitions,
            || {
                trace_calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert!(first);
        assert!(second);
        assert_eq!(trace_calls, 1);
    }

    #[test]
    fn cached_definition_no_detour_reachability_reuses_in_progress_exact_state() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let mut cache = vec![DefinitionNoDetourReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            result: Err(HypermeshError::UnknownClassification),
        }];

        let result = cached_definition_no_detour_reachability_with(
            &mut cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            || Ok(true),
        );

        assert_eq!(result, Err(HypermeshError::UnknownClassification));
        assert_eq!(cache.len(), 1);
        assert_eq!(cache[0].result, Err(HypermeshError::UnknownClassification));
    }

    #[test]
    fn cached_definition_no_plane_replacement_reachability_reuses_identical_query() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let mut cache = Vec::new();
        let mut trace_calls = 0;

        let first = cached_definition_no_plane_replacement_reachability_with(
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
        let second = cached_definition_no_plane_replacement_reachability_with(
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
    fn cached_definition_no_plane_replacement_reachability_reuses_permuted_definition_families() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_a = axis_plane_definition(&start);
        let start_b = axis_plane_definition(&p(0, 1, 0));
        let end_a = axis_plane_definition(&end);
        let end_b = axis_plane_definition(&p(1, 1, 0));
        let mut cache = Vec::new();
        let mut trace_calls = 0;

        let first = cached_definition_no_plane_replacement_reachability_with(
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
        let second = cached_definition_no_plane_replacement_reachability_with(
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
    fn cached_definition_no_plane_replacement_reachability_reuses_reversed_query() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let mut cache = Vec::new();
        let mut trace_calls = 0;

        let first = cached_definition_no_plane_replacement_reachability_with(
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
        let second = cached_definition_no_plane_replacement_reachability_with(
            &mut cache,
            &end,
            &start,
            &end_definitions,
            &start_definitions,
            || {
                trace_calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert!(first);
        assert!(second);
        assert_eq!(trace_calls, 1);
    }

    #[test]
    fn cached_definition_no_plane_replacement_reachability_reuses_in_progress_exact_state() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let mut cache = vec![DefinitionNoPlaneReplacementReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            result: Err(HypermeshError::UnknownClassification),
        }];

        let result = cached_definition_no_plane_replacement_reachability_with(
            &mut cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            || Ok(true),
        );

        assert_eq!(result, Err(HypermeshError::UnknownClassification));
        assert_eq!(cache.len(), 1);
        assert_eq!(cache[0].result, Err(HypermeshError::UnknownClassification));
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
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
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
    fn adjacent_normal_probe_keeps_later_corridor_certified_after_uncertified_crossing() {
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
        assert!(!probes[0].uncertified_definition_fallback);
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
    fn adjacent_normal_probe_stop_values_treat_endpoint_boundary_contact_as_unknown_and_keep_later_corridor()
     {
        let support = Plane::axis_aligned(0, r(0));
        let interior = p(1, 1, 1);
        let direction = support.normal.clone();
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(4, 0, 0), &p(4, 1, 0), &p(4, 0, 1), 0, 0);
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
                if polygon.vertices().unwrap()[0].x == r(4) {
                    Ok(if *point == p(4, 1, 1) {
                        PolygonPointLocation::Boundary
                    } else {
                        PolygonPointLocation::Outside
                    })
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
    fn adjacent_normal_probe_stop_values_treat_bound_start_contact_as_unknown() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = p(4, 1, 1);
        let direction = support.normal.clone();
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
            &interior,
            &direction,
            &support,
            &bounds,
            &[],
            &mut |_interior, direction, polygon| {
                Ok(dot_direction(&polygon.support.normal, direction))
            },
            &mut |_point, _polygon| Ok(PolygonPointLocation::Outside),
        )
        .unwrap();

        assert!(saw_unknown);
        assert!(stop_values.is_empty());
    }

    #[test]
    fn adjacent_normal_probe_reports_unknown_for_bound_start_contact() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(4, 1, 1),
            planes: vec![axis_plane_definition(&p(4, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        let err = adjacent_normal_probes_with_queries(
            &interior,
            &support,
            &bounds,
            &[],
            true,
            |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
            |_point, _polygon| Ok(PolygonPointLocation::Outside),
            |_corridor, _stop_point| Ok(Vec::new()),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn adjacent_normal_probe_keeps_later_corridor_certified_after_boundary_start_contact() {
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
        assert!(!probes[0].uncertified_definition_fallback);
    }

    #[test]
    fn adjacent_normal_probe_keeps_later_corridor_certified_after_endpoint_boundary_contact() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(4, 0, 0), &p(4, 1, 0), &p(4, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let probes = adjacent_normal_probes_with_queries(
            &interior,
            &support,
            &bounds,
            &[first, second],
            true,
            |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
            |point, polygon| {
                if polygon.vertices().unwrap()[0].x == r(4) {
                    Ok(if *point == p(4, 1, 1) {
                        PolygonPointLocation::Boundary
                    } else {
                        PolygonPointLocation::Outside
                    })
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
        assert!(!probes[0].uncertified_definition_fallback);
    }

    #[test]
    fn strict_normal_probe_targets_try_shifted_search_from_report_witness_seed() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
        let stop_point = p(3, 1, 1);
        let witness = p(1, 2, 2);
        let visited = std::cell::RefCell::new(Vec::new());

        let probes = strict_normal_probe_targets_from_seed_families_with_tracking_unknown(
            &interior,
            &support,
            &corridor,
            None,
            &stop_point,
            true,
            Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
                witness.clone(),
                [None, None, None],
            )),
            Vec::new(),
            vec![witness.clone()],
            Vec::new(),
            |seed| {
                visited.borrow_mut().push(seed.clone());
                Ok(vec![ShiftedHalfspaceWitness {
                    point: p(2, 1, 1),
                    families: vec![ShiftedHalfspaceWitnessFamily {
                        halfspaces: vec![axis_halfspace(0, false, r(3))],
                        active_planes: [Some(0), None, None],
                    }],
                    uncertified_definition_fallback: false,
                }])
            },
        )
        .unwrap();

        assert_eq!(visited.into_inner(), vec![witness]);
        assert!(probes.iter().any(|probe| probe.point == p(2, 1, 1)));
    }

    #[test]
    fn strict_normal_probe_targets_merge_same_point_certified_shifted_replay_definitions() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![[
                support.clone(),
                Plane::axis_aligned(2, r(1)),
                Plane::axis_aligned(2, r(1)),
            ]],
            uncertified_definition_fallback: false,
        };
        let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
        let stop_point = p(3, 1, 1);
        let witness = p(2, 1, 1);
        let visited = std::cell::RefCell::new(Vec::new());

        let probes = strict_normal_probe_targets_from_seed_families_with_tracking_unknown(
            &interior,
            &support,
            &corridor,
            None,
            &stop_point,
            true,
            Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
                witness.clone(),
                [None, None, None],
            )),
            vec![witness.clone()],
            Vec::new(),
            Vec::new(),
            |seed| {
                visited.borrow_mut().push(seed.clone());
                Ok(vec![ShiftedHalfspaceWitness {
                    point: seed.clone(),
                    families: vec![ShiftedHalfspaceWitnessFamily {
                        halfspaces: vec![axis_halfspace(1, false, r(2))],
                        active_planes: [Some(0), None, None],
                    }],
                    uncertified_definition_fallback: false,
                }])
            },
        )
        .unwrap();

        assert_eq!(visited.into_inner(), vec![witness.clone()]);
        let probe = probes
            .iter()
            .find(|probe| probe.point == witness && probe.side == Classification::Positive)
            .expect("same-point shifted replay should keep the direct probe and enrich it");
        assert!(!probe.uncertified_definition_fallback);
        assert!(probe.planes.iter().any(|definition| {
            definition
                .iter()
                .any(|plane| plane.normal == p(0, 1, 0) && plane.offset == r(-1))
        }));
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
    fn unique_normal_probe_search_definitions_skip_duplicate_retained_pairs() {
        let support = Plane::axis_aligned(0, r(0));
        let axis_definition = axis_plane_definition(&p(1, 2, 3));
        let duplicate_first = [
            Plane::axis_aligned(0, r(7)),
            axis_definition[1].clone(),
            axis_definition[2].clone(),
        ];
        let swapped_pair = [
            Plane::axis_aligned(0, r(9)),
            axis_definition[2].clone(),
            axis_definition[1].clone(),
        ];

        let unique = unique_normal_probe_search_definitions(
            &[axis_definition.clone(), duplicate_first, swapped_pair],
            &support,
        )
        .unwrap();

        assert_eq!(unique.len(), 1);
        assert!(retained_plane_pairs_match_as_sets(
            &unique[0],
            &axis_definition
        ));
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
        assert!(!probes[0].uncertified_definition_fallback);
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
    fn collect_normal_probe_targets_keep_later_probes_certified_after_uncertain_family_result() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];

        let probes = collect_normal_probe_targets(&[definition], |candidate| match candidate {
            Some(_) => Ok(vec![ProbePoint {
                point: p(2, 2, 2),
                side: Classification::Positive,
                planes: vec![axis_plane_definition(&p(2, 2, 2))],
                uncertified_definition_fallback: true,
            }]),
            None => Ok(vec![ProbePoint {
                point: p(3, 3, 3),
                side: Classification::Positive,
                planes: vec![axis_plane_definition(&p(3, 3, 3))],
                uncertified_definition_fallback: false,
            }]),
        })
        .unwrap();

        assert_eq!(probes.len(), 2);
        assert!(
            probes
                .iter()
                .any(|probe| !probe.uncertified_definition_fallback)
        );
    }

    #[test]
    fn collect_normal_probe_targets_keeps_certified_duplicate_state_certified() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let point = p(2, 2, 2);
        let planes = vec![axis_plane_definition(&point)];

        let probes = collect_normal_probe_targets(&[definition], |candidate| match candidate {
            Some(_) => Ok(vec![ProbePoint {
                point: point.clone(),
                side: Classification::Positive,
                planes: planes.clone(),
                uncertified_definition_fallback: true,
            }]),
            None => Ok(vec![ProbePoint {
                point: point.clone(),
                side: Classification::Positive,
                planes: planes.clone(),
                uncertified_definition_fallback: false,
            }]),
        })
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert!(!probes[0].uncertified_definition_fallback);
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
    fn probe_point_build_collection_keeps_existing_probes_certified_after_later_unknown() {
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
        assert!(!probes[0].uncertified_definition_fallback);
    }

    #[test]
    fn probe_point_build_collection_keeps_later_probes_certified_after_uncertain_candidate_result()
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
                .any(|probe| !probe.uncertified_definition_fallback)
        );
    }

    #[test]
    fn probe_point_build_collection_keeps_certified_duplicate_state_certified() {
        let mut probes = Vec::new();
        let point = p(1, 1, 1);
        let definition = axis_plane_definition(&point);

        extend_probe_point_builds_backtracking_unknown(&mut probes, [0, 1].iter(), |candidate| {
            Ok(Some(ProbePoint {
                point: point.clone(),
                side: Classification::Positive,
                planes: vec![definition.clone()],
                uncertified_definition_fallback: *candidate == 0,
            }))
        })
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert!(!probes[0].uncertified_definition_fallback);
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
    fn adjacent_axis_probe_keeps_later_corridor_certified_after_uncertified_crossing() {
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
        assert!(!probes[0].uncertified_definition_fallback);
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
    fn adjacent_axis_probe_stop_values_treat_endpoint_boundary_contact_as_unknown_and_keep_later_corridor()
     {
        let interior = p(1, 1, 1);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(4, 0, 0), &p(4, 1, 0), &p(4, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let (stop_values, saw_unknown) = adjacent_axis_probe_stop_values_with_queries(
            &interior,
            &bounds,
            &[first, second],
            0,
            true,
            &mut |_interior, endpoint, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(4) {
                    Ok(Some(endpoint.clone()))
                } else {
                    Ok(Some(p(3, 1, 1)))
                }
            },
            &mut |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(4) {
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
    fn adjacent_axis_probe_stop_values_treat_start_boundary_contact_as_unknown_and_keep_later_corridor()
     {
        let interior = p(1, 1, 1);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let (stop_values, saw_unknown) = adjacent_axis_probe_stop_values_with_queries(
            &interior,
            &bounds,
            &[first, second],
            0,
            true,
            &mut |interior, _endpoint, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(1) {
                    Ok(Some(interior.clone()))
                } else {
                    Ok(Some(p(3, 1, 1)))
                }
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
        assert_eq!(stop_values, vec![r(3), r(4)]);
    }

    #[test]
    fn adjacent_axis_probe_stop_values_treat_bound_start_contact_as_unknown() {
        let interior = p(4, 1, 1);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        let (stop_values, saw_unknown) = adjacent_axis_probe_stop_values_with_queries(
            &interior,
            &bounds,
            &[],
            0,
            true,
            &mut |_interior, _endpoint, _polygon, _axis| Ok(None),
            &mut |_crossing, _polygon| Ok(PolygonPointLocation::Outside),
        )
        .unwrap();

        assert!(saw_unknown);
        assert!(stop_values.is_empty());
    }

    #[test]
    fn adjacent_axis_probe_reports_unknown_for_bound_start_contact() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(4, 1, 1),
            planes: vec![axis_plane_definition(&p(4, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        let err = adjacent_axis_probes_with_queries(
            &interior,
            &support,
            &bounds,
            &[],
            0,
            true,
            |_interior, _endpoint, _polygon, _axis| Ok(None),
            |_crossing, _polygon| Ok(PolygonPointLocation::Outside),
            |_corridor| Ok(Vec::new()),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn adjacent_axis_probe_keeps_later_corridor_certified_after_boundary_crossing() {
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
        assert!(!probes[0].uncertified_definition_fallback);
    }

    #[test]
    fn adjacent_axis_probe_keeps_later_corridor_certified_after_endpoint_boundary_contact() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(4, 0, 0), &p(4, 1, 0), &p(4, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let probes = adjacent_axis_probes_with_queries(
            &interior,
            &support,
            &bounds,
            &[first, second],
            0,
            true,
            |_interior, endpoint, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(4) {
                    Ok(Some(endpoint.clone()))
                } else {
                    Ok(Some(p(3, 1, 1)))
                }
            },
            |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(4) {
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
        assert!(!probes[0].uncertified_definition_fallback);
    }

    #[test]
    fn adjacent_axis_probe_keeps_later_corridor_certified_after_boundary_start_contact() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

        let probes = adjacent_axis_probes_with_queries(
            &interior,
            &support,
            &bounds,
            &[first, second],
            0,
            true,
            |interior, _endpoint, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(1) {
                    Ok(Some(interior.clone()))
                } else {
                    Ok(Some(p(3, 1, 1)))
                }
            },
            |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(1) {
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
        assert!(!probes[0].uncertified_definition_fallback);
    }

    #[test]
    fn strict_axis_probe_targets_try_shifted_search_from_report_witness_seed() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        };
        let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
        let witness = p(1, 2, 2);
        let visited = std::cell::RefCell::new(Vec::new());

        let probes = strict_axis_probe_targets_from_seed_families_with_tracking_unknown(
            &interior,
            &support,
            &corridor,
            0,
            true,
            None,
            Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
                witness.clone(),
                [None, None, None],
            )),
            Vec::new(),
            vec![witness.clone()],
            Vec::new(),
            |seed| {
                visited.borrow_mut().push(seed.clone());
                Ok(vec![ShiftedHalfspaceWitness {
                    point: p(2, 1, 1),
                    families: vec![ShiftedHalfspaceWitnessFamily {
                        halfspaces: vec![axis_halfspace(0, false, r(3))],
                        active_planes: [Some(0), None, None],
                    }],
                    uncertified_definition_fallback: false,
                }])
            },
        )
        .unwrap();

        assert_eq!(visited.into_inner(), vec![witness]);
        assert!(probes.iter().any(|probe| probe.point == p(2, 1, 1)));
    }

    #[test]
    fn strict_axis_probe_targets_merge_same_point_certified_shifted_replay_definitions() {
        let support = Plane::axis_aligned(0, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 1),
            planes: vec![[
                support.clone(),
                Plane::axis_aligned(2, r(1)),
                Plane::axis_aligned(2, r(1)),
            ]],
            uncertified_definition_fallback: false,
        };
        let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
        let witness = p(2, 1, 1);
        let visited = std::cell::RefCell::new(Vec::new());

        let probes = strict_axis_probe_targets_from_seed_families_with_tracking_unknown(
            &interior,
            &support,
            &corridor,
            0,
            true,
            None,
            Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
                witness.clone(),
                [None, None, None],
            )),
            vec![witness.clone()],
            Vec::new(),
            Vec::new(),
            |seed| {
                visited.borrow_mut().push(seed.clone());
                Ok(vec![ShiftedHalfspaceWitness {
                    point: seed.clone(),
                    families: vec![ShiftedHalfspaceWitnessFamily {
                        halfspaces: vec![axis_halfspace(1, false, r(2))],
                        active_planes: [Some(0), None, None],
                    }],
                    uncertified_definition_fallback: false,
                }])
            },
        )
        .unwrap();

        assert_eq!(visited.into_inner(), vec![witness.clone()]);
        let probe = probes
            .iter()
            .find(|probe| probe.point == witness && probe.side == Classification::Positive)
            .expect("same-point shifted replay should keep the direct axis probe and enrich it");
        assert!(!probe.uncertified_definition_fallback);
        assert!(probe.planes.iter().any(|definition| {
            definition
                .iter()
                .any(|plane| plane.normal == p(0, 1, 0) && plane.offset == r(-1))
        }));
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
        assert!(!probes[0].uncertified_definition_fallback);
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
    fn collect_axis_probe_targets_keep_later_probes_certified_after_uncertain_family_result() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];

        let probes = collect_axis_probe_targets(&[definition], |candidate| match candidate {
            Some(_) => Ok(vec![ProbePoint {
                point: p(2, 2, 2),
                side: Classification::Positive,
                planes: vec![axis_plane_definition(&p(2, 2, 2))],
                uncertified_definition_fallback: true,
            }]),
            None => Ok(vec![ProbePoint {
                point: p(3, 3, 3),
                side: Classification::Positive,
                planes: vec![axis_plane_definition(&p(3, 3, 3))],
                uncertified_definition_fallback: false,
            }]),
        })
        .unwrap();

        assert_eq!(probes.len(), 2);
        assert!(
            probes
                .iter()
                .any(|probe| !probe.uncertified_definition_fallback)
        );
    }

    #[test]
    fn collect_axis_probe_targets_keeps_certified_duplicate_state_certified() {
        let definition = [
            Plane::axis_aligned(2, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let point = p(2, 2, 2);
        let planes = vec![axis_plane_definition(&point)];

        let probes = collect_axis_probe_targets(&[definition], |candidate| match candidate {
            Some(_) => Ok(vec![ProbePoint {
                point: point.clone(),
                side: Classification::Positive,
                planes: planes.clone(),
                uncertified_definition_fallback: true,
            }]),
            None => Ok(vec![ProbePoint {
                point: point.clone(),
                side: Classification::Positive,
                planes: planes.clone(),
                uncertified_definition_fallback: false,
            }]),
        })
        .unwrap();

        assert_eq!(probes.len(), 1);
        assert!(!probes[0].uncertified_definition_fallback);
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
    fn leaf_classification_keeps_certified_direct_leaf_witness_after_invalid_active_replay() {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let ref_point = p(0, 0, 0);
        let ref_definitions = [axis_plane_definition(&ref_point)];
        let interior = build_strict_leaf_point(
            &leaf,
            &p(1, 1, 1),
            &[
                limit_plane_from_plane(&leaf.support),
                axis_halfspace(0, false, r(1)),
            ],
            [Some(9), None, None],
            false,
        )
        .unwrap()
        .expect("direct leaf witness should still certify");

        assert!(!interior.uncertified_definition_fallback);

        let winding = classify_leaf_polygon_from_interior_points(
            std::slice::from_ref(&interior),
            &leaf.support,
            &ref_point,
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
    fn positive_probe_traces_for_slanted_leaf_case() {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let ref_point = p(0, 0, 0);
        let ref_definitions = [axis_plane_definition(&ref_point)];
        let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("slanted leaf should have a replayable interior witness");
        let probe =
            bounded_probes_from_interior(&interior, &leaf.support, &bounds, true, &[leaf.clone()])
                .unwrap()
                .into_iter()
                .find(|probe| {
                    probe.side == Classification::Positive && !probe.uncertified_definition_fallback
                })
                .expect("slanted leaf should have a certified positive-side probe");

        assert!(
            probe_reaches_adjacent_cell_from_interior(
                &interior,
                &probe,
                &leaf.support,
                &[leaf.clone()],
            )
            .unwrap()
        );

        let winding =
            trace_probe_winding(&ref_point, &ref_definitions, &probe, &[0], &[leaf.clone()])
                .unwrap();

        assert_eq!(winding, vec![-1]);
    }

    #[test]
    fn certified_leaf_interior_points_exist_for_slanted_leaf_case() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);

        let interior_points = certified_leaf_interior_points(&leaf.support, &leaf.edges).unwrap();

        assert!(!interior_points.is_empty());
        assert!(interior_points.iter().any(|point| !point.planes.is_empty()));
    }

    #[test]
    fn bounded_probes_find_positive_probe_for_slanted_leaf_case() {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("slanted leaf should have a replayable interior witness");

        let probes =
            bounded_probes_from_interior(&interior, &leaf.support, &bounds, true, &[leaf.clone()])
                .unwrap();

        assert!(
            probes
                .iter()
                .any(|probe| probe.side == Classification::Positive)
        );
    }

    #[test]
    fn bounded_probes_keep_certified_positive_probe_for_slanted_leaf_case() {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("slanted leaf should have a replayable interior witness");

        let probes =
            bounded_probes_from_interior(&interior, &leaf.support, &bounds, true, &[leaf.clone()])
                .unwrap();

        assert!(probes.iter().any(|probe| {
            probe.side == Classification::Positive && !probe.uncertified_definition_fallback
        }));
    }

    #[test]
    fn adjacent_normal_probe_stop_values_exist_for_slanted_leaf_case() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("slanted leaf should have a replayable interior witness");

        let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
            &interior.point,
            &leaf.support.normal,
            &leaf.support,
            &bounds,
            &[leaf.clone()],
            &mut |_interior, direction, polygon| {
                Ok(dot_direction(&polygon.support.normal, direction))
            },
            &mut |point, polygon| classify_point_in_polygon(point, polygon),
        )
        .unwrap();

        assert!(!saw_unknown);
        assert!(!stop_values.is_empty());
        assert!(
            stop_values
                .iter()
                .all(|stop| { compare_real(stop, &Real::zero()).unwrap().is_gt() })
        );
    }

    #[test]
    fn strict_normal_probe_targets_find_positive_probe_for_slanted_leaf_case() {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("slanted leaf should have a replayable interior witness");
        let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
            &interior.point,
            &leaf.support.normal,
            &leaf.support,
            &bounds,
            &[leaf.clone()],
            &mut |_interior, direction, polygon| {
                Ok(dot_direction(&polygon.support.normal, direction))
            },
            &mut |point, polygon| classify_point_in_polygon(point, polygon),
        )
        .unwrap();

        assert!(!saw_unknown);
        let stop_t = stop_values[0].clone();
        let stop_point = offset_point(&interior.point, &leaf.support.normal, &stop_t);
        let corridor = bounds_between_points(&interior.point, &stop_point).unwrap();

        let probes = strict_normal_probe_targets(
            &interior,
            &leaf.support,
            &corridor,
            Some(&interior.planes[0]),
            &stop_point,
            true,
        )
        .unwrap();

        assert!(
            probes
                .iter()
                .any(|probe| probe.side == Classification::Positive)
        );
    }

    #[test]
    fn strict_normal_probe_targets_keep_certified_probe_for_slanted_leaf_case() {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("slanted leaf should have a replayable interior witness");
        let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
            &interior.point,
            &leaf.support.normal,
            &leaf.support,
            &bounds,
            &[leaf.clone()],
            &mut |_interior, direction, polygon| {
                Ok(dot_direction(&polygon.support.normal, direction))
            },
            &mut |point, polygon| classify_point_in_polygon(point, polygon),
        )
        .unwrap();

        assert!(!saw_unknown);
        let stop_t = stop_values[0].clone();
        let stop_point = offset_point(&interior.point, &leaf.support.normal, &stop_t);
        let corridor = bounds_between_points(&interior.point, &stop_point).unwrap();

        let probes = strict_normal_probe_targets(
            &interior,
            &leaf.support,
            &corridor,
            Some(&interior.planes[0]),
            &stop_point,
            true,
        )
        .unwrap();

        assert!(probes.iter().any(|probe| {
            probe.side == Classification::Positive && !probe.uncertified_definition_fallback
        }));
    }

    #[test]
    fn adjacent_normal_probes_find_positive_probe_for_slanted_leaf_case() {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("slanted leaf should have a replayable interior witness");

        let probes =
            adjacent_normal_probes(&interior, &leaf.support, &bounds, &[leaf.clone()], true)
                .unwrap();

        assert!(
            probes
                .iter()
                .any(|probe| probe.side == Classification::Positive)
        );
    }

    #[test]
    fn adjacent_normal_probes_keep_certified_positive_probe_for_slanted_leaf_case() {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("slanted leaf should have a replayable interior witness");

        let probes =
            adjacent_normal_probes(&interior, &leaf.support, &bounds, &[leaf.clone()], true)
                .unwrap();

        assert!(probes.iter().any(|probe| {
            probe.side == Classification::Positive && !probe.uncertified_definition_fallback
        }));
    }

    #[test]
    fn strict_normal_probe_targets_find_positive_probe_for_slanted_leaf_case_unrestricted() {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("slanted leaf should have a replayable interior witness");
        let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
            &interior.point,
            &leaf.support.normal,
            &leaf.support,
            &bounds,
            &[leaf.clone()],
            &mut |_interior, direction, polygon| {
                Ok(dot_direction(&polygon.support.normal, direction))
            },
            &mut |point, polygon| classify_point_in_polygon(point, polygon),
        )
        .unwrap();

        assert!(!saw_unknown);
        let stop_t = stop_values[0].clone();
        let stop_point = offset_point(&interior.point, &leaf.support.normal, &stop_t);
        let corridor = bounds_between_points(&interior.point, &stop_point).unwrap();

        let probes = strict_normal_probe_targets(
            &interior,
            &leaf.support,
            &corridor,
            None,
            &stop_point,
            true,
        )
        .unwrap();

        assert!(
            probes
                .iter()
                .any(|probe| probe.side == Classification::Positive)
        );
    }

    #[test]
    fn strict_normal_probe_direct_seed_phase_finds_positive_probe_for_slanted_leaf_case_unrestricted()
     {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("slanted leaf should have a replayable interior witness");
        let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
            &interior.point,
            &leaf.support.normal,
            &leaf.support,
            &bounds,
            &[leaf.clone()],
            &mut |_interior, direction, polygon| {
                Ok(dot_direction(&polygon.support.normal, direction))
            },
            &mut |point, polygon| classify_point_in_polygon(point, polygon),
        )
        .unwrap();

        assert!(!saw_unknown);
        let stop_t = stop_values[0].clone();
        let stop_point = offset_point(&interior.point, &leaf.support.normal, &stop_t);
        let corridor = bounds_between_points(&interior.point, &stop_point).unwrap();

        let mut halfspaces = aabb_core_halfspaces(&corridor).unwrap();
        halfspaces.push(support_side_halfspace(&leaf.support, true));
        halfspaces.push(normal_stop_halfspace(&leaf.support, &stop_point, true));
        let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces).unwrap();
        assert!(!saw_unknown);
        let (seeds, shifted_vertices, shifted_geometry_seeds) =
            halfspace_cell_seed_families_from_optional_report(
                &corridor,
                &halfspaces,
                report.as_ref(),
                &mut saw_unknown,
            )
            .unwrap();

        let probes = strict_normal_probe_targets_from_seed_families_with_tracking_unknown(
            &interior,
            &leaf.support,
            &corridor,
            None,
            &stop_point,
            true,
            report.as_ref(),
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
            |_seed| Ok(Vec::new()),
        )
        .unwrap();

        assert!(
            probes
                .iter()
                .any(|probe| probe.side == Classification::Positive)
        );
    }

    #[test]
    fn strict_normal_probe_direct_seed_phase_keeps_certified_probe_for_slanted_leaf_case_unrestricted()
     {
        let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        leaf.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
            .unwrap()
            .into_iter()
            .find(|point| !point.planes.is_empty())
            .expect("slanted leaf should have a replayable interior witness");
        let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
            &interior.point,
            &leaf.support.normal,
            &leaf.support,
            &bounds,
            &[leaf.clone()],
            &mut |_interior, direction, polygon| {
                Ok(dot_direction(&polygon.support.normal, direction))
            },
            &mut |point, polygon| classify_point_in_polygon(point, polygon),
        )
        .unwrap();

        assert!(!saw_unknown);
        let stop_t = stop_values[0].clone();
        let stop_point = offset_point(&interior.point, &leaf.support.normal, &stop_t);
        let corridor = bounds_between_points(&interior.point, &stop_point).unwrap();

        let mut halfspaces = aabb_core_halfspaces(&corridor).unwrap();
        halfspaces.push(support_side_halfspace(&leaf.support, true));
        halfspaces.push(normal_stop_halfspace(&leaf.support, &stop_point, true));
        let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces).unwrap();
        assert!(!saw_unknown);
        let (seeds, shifted_vertices, shifted_geometry_seeds) =
            halfspace_cell_seed_families_from_optional_report(
                &corridor,
                &halfspaces,
                report.as_ref(),
                &mut saw_unknown,
            )
            .unwrap();

        let probes = strict_normal_probe_targets_from_seed_families_with_tracking_unknown(
            &interior,
            &leaf.support,
            &corridor,
            None,
            &stop_point,
            true,
            report.as_ref(),
            seeds,
            shifted_vertices,
            shifted_geometry_seeds,
            |_seed| Ok(Vec::new()),
        )
        .unwrap();

        assert!(probes.iter().any(|probe| {
            probe.side == Classification::Positive && !probe.uncertified_definition_fallback
        }));
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
    fn strict_leaf_cell_points_merge_same_point_certified_shifted_replay_definitions() {
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
        let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();
        let witness = seeds[0].clone();
        let extra_definition = [
            leaf.support.clone(),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let visited = std::cell::RefCell::new(Vec::new());

        let interiors = strict_leaf_cell_points_from_seed_families_with_tracking_unknown(
            &leaf,
            &center,
            Some(&report),
            vec![witness.clone()],
            Vec::new(),
            Vec::new(),
            |seed| {
                visited.borrow_mut().push(seed.clone());
                Ok(vec![ShiftedHalfspaceWitness {
                    point: seed.clone(),
                    families: vec![ShiftedHalfspaceWitnessFamily {
                        halfspaces: vec![axis_halfspace(1, false, r(1))],
                        active_planes: [Some(0), None, None],
                    }],
                    uncertified_definition_fallback: false,
                }])
            },
        )
        .unwrap();

        assert_eq!(visited.into_inner(), vec![witness.clone()]);
        let interior = interiors
            .iter()
            .find(|point| point.point == witness)
            .expect(
                "same-point shifted replay should keep the direct strict leaf point and enrich it",
            );
        assert!(!interior.uncertified_definition_fallback);
        assert!(
            interior.planes.iter().any(|definition| {
                definition_planes_match_as_sets(definition, &extra_definition)
            })
        );
    }

    #[test]
    fn normal_probe_extra_planes_only_keep_selected_definition_planes() {
        let support = Plane::axis_aligned(2, r(0));
        let first = [
            support.clone(),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];
        let second = [
            support.clone(),
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(1, r(2)),
        ];
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![first.clone(), second.clone()],
            uncertified_definition_fallback: false,
        };

        let extra_planes = normal_probe_extra_planes(&interior, Some(&first));

        assert_eq!(extra_planes.len(), 2);
        assert!(extra_planes.iter().any(|plane| plane == &first[1]));
        assert!(extra_planes.iter().any(|plane| plane == &first[2]));
        assert!(
            extra_planes
                .iter()
                .all(|plane| plane != &second[1] && plane != &second[2])
        );
    }

    #[test]
    fn normal_probe_extra_planes_leave_unrestricted_family_empty() {
        let support = Plane::axis_aligned(2, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![[
                support,
                Plane::axis_aligned(0, r(1)),
                Plane::axis_aligned(1, r(1)),
            ]],
            uncertified_definition_fallback: false,
        };

        assert!(normal_probe_extra_planes(&interior, None).is_empty());
    }

    #[test]
    fn normal_probe_shifted_seed_families_keep_only_report_root_after_certified_direct_probe() {
        let report_witness = p(9, 9, 9);
        let direct_probe_point = p(1, 1, 1);
        let shifted_vertex = p(2, 2, 2);
        let shifted_geometry = p(3, 3, 3);

        let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
            normal_probe_shifted_seed_families(
                None,
                Some(&report_witness),
                std::slice::from_ref(&direct_probe_point),
                vec![direct_probe_point.clone()],
                vec![shifted_vertex],
                vec![shifted_geometry],
            );

        assert_eq!(strict_shift_seeds, vec![report_witness]);
        assert!(shifted_vertices.is_empty());
        assert!(shifted_geometry_seeds.is_empty());
    }

    #[test]
    fn normal_probe_shifted_seed_families_fall_back_to_first_certified_probe_without_report() {
        let direct_probe_point = p(1, 1, 1);
        let shifted_vertex = p(2, 2, 2);
        let shifted_geometry = p(3, 3, 3);

        let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
            normal_probe_shifted_seed_families(
                None,
                None,
                std::slice::from_ref(&direct_probe_point),
                vec![direct_probe_point.clone()],
                vec![shifted_vertex],
                vec![shifted_geometry],
            );

        assert_eq!(strict_shift_seeds, vec![direct_probe_point]);
        assert!(shifted_vertices.is_empty());
        assert!(shifted_geometry_seeds.is_empty());
    }

    #[test]
    fn normal_probe_shifted_seed_families_keep_raw_roots_without_certified_direct_probe() {
        let shifted_vertex = p(2, 2, 2);
        let shifted_geometry = p(3, 3, 3);

        let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
            normal_probe_shifted_seed_families(
                None,
                None,
                &[],
                Vec::new(),
                vec![shifted_vertex.clone()],
                vec![shifted_geometry.clone()],
            );

        assert!(strict_shift_seeds.is_empty());
        assert_eq!(shifted_vertices, vec![shifted_vertex]);
        assert_eq!(shifted_geometry_seeds, vec![shifted_geometry]);
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
    fn strict_leaf_witness_points_merge_same_point_certified_stricter_replay_definitions() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let vertices = leaf.vertices().unwrap();
        let witness = p(1, 1, 1);
        let extra_definition = [
            leaf.support.clone(),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ];

        let interiors = strict_leaf_witness_points_with_seed_families_and_stricter_replay(
            &leaf,
            &vertices,
            &mut |_leaf, _vertices, _bounds, _halfspaces, _report| {
                Ok(LeafWitnessSeedFamilies {
                    seeds: vec![witness.clone()],
                    shifted_vertices: Vec::new(),
                    shifted_geometry_seeds: Vec::new(),
                    saw_unknown: false,
                })
            },
            |_leaf, point| {
                Ok(vec![InteriorLeafPoint {
                    point: point.clone(),
                    planes: vec![axis_plane_definition(point), extra_definition.clone()],
                    uncertified_definition_fallback: false,
                }])
            },
        )
        .unwrap();
        let merged = interiors
            .iter()
            .find(|point| point.point == witness)
            .expect("same-point stricter replay should survive witness aggregation");

        assert!(
            merged
                .planes
                .iter()
                .any(|candidate| { definition_planes_match_as_sets(candidate, &extra_definition) })
        );
        assert!(!merged.uncertified_definition_fallback);
    }

    #[test]
    fn strict_leaf_witness_points_try_shifted_search_from_report_witness_seed() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let vertices = leaf.vertices().unwrap();

        let interiors = strict_leaf_witness_points_with_seed_families(
            &leaf,
            &vertices,
            |_leaf, _vertices, _bounds, _halfspaces, _report| {
                Ok(LeafWitnessSeedFamilies {
                    seeds: Vec::new(),
                    shifted_vertices: Vec::new(),
                    shifted_geometry_seeds: Vec::new(),
                    saw_unknown: false,
                })
            },
        )
        .unwrap();

        assert!(!interiors.is_empty());
        assert!(
            interiors
                .iter()
                .any(|point| point.point == Point3::new(q(1, 2), q(1, 2), r(2)))
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
    fn interior_leaf_point_collection_keeps_existing_points_certified_after_later_unknown() {
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
        assert!(!points[0].uncertified_definition_fallback);
    }

    #[test]
    fn interior_leaf_point_collection_keeps_later_points_certified_after_uncertain_candidate_result()
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
                .any(|point| !point.uncertified_definition_fallback)
        );
    }

    #[test]
    fn interior_leaf_point_collection_keeps_certified_duplicate_state_certified() {
        let mut points = Vec::new();
        let point = p(1, 1, 1);
        let definition = axis_plane_definition(&point);

        extend_interior_leaf_points_backtracking_unknown(&mut points, [0, 1].iter(), |candidate| {
            Ok(vec![InteriorLeafPoint {
                point: point.clone(),
                planes: vec![definition.clone()],
                uncertified_definition_fallback: *candidate == 0,
            }])
        })
        .unwrap();

        assert_eq!(points.len(), 1);
        assert!(!points[0].uncertified_definition_fallback);
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
    fn leaf_point_build_collection_keeps_existing_points_certified_after_later_unknown() {
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
        assert!(!points[0].uncertified_definition_fallback);
    }

    #[test]
    fn leaf_point_build_collection_keeps_later_points_certified_after_uncertain_candidate_result() {
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
                .any(|point| !point.uncertified_definition_fallback)
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
    fn strict_leaf_witness_reports_unknown_for_leaf_boundary_contact() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let witness = p(3, 0, 0);
        let halfspaces = vec![limit_plane_from_plane(&leaf.support)];

        assert_eq!(
            build_strict_leaf_point(&leaf, &witness, &halfspaces, [None, None, None], false),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn strict_leaf_witness_points_keep_surviving_points_certified_after_seed_family_unknown() {
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
                .any(|point| !point.uncertified_definition_fallback)
        );
    }

    #[test]
    fn strict_leaf_witness_points_keep_surviving_points_certified_after_boundary_seed_candidate() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let vertices = leaf.vertices().unwrap();

        let points = strict_leaf_witness_points_with_seed_families(
            &leaf,
            &vertices,
            |leaf, _vertices, _bounds, _halfspaces, _report| {
                let boundary_family = collect_strict_halfspace_seed_family(
                    Ok(vec![p(3, 0, 0), p(1, 1, 1)]),
                    |candidate| point_strictly_inside_leaf_or_unknown(candidate, leaf),
                )?;
                Ok(LeafWitnessSeedFamilies {
                    seeds: boundary_family.seeds,
                    shifted_vertices: Vec::new(),
                    shifted_geometry_seeds: Vec::new(),
                    saw_unknown: boundary_family.saw_unknown,
                })
            },
        )
        .unwrap();

        assert!(points.iter().any(|point| point.point == p(1, 1, 1)));
        assert!(
            points
                .iter()
                .any(|point| !point.uncertified_definition_fallback)
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
    fn strict_leaf_witness_from_shifted_witness_reports_unknown_for_leaf_boundary_contact() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let witness = ShiftedHalfspaceWitness {
            point: p(3, 0, 0),
            families: vec![ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(0, false, r(3))],
                active_planes: [Some(0), None, None],
            }],
            uncertified_definition_fallback: false,
        };

        assert_eq!(
            build_strict_leaf_point_from_shifted_witness(&leaf, &witness),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn strict_leaf_witness_from_shifted_witness_stays_certified_when_one_family_is_singular() {
        let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
        let witness = ShiftedHalfspaceWitness {
            point: p(1, 1, 1),
            families: vec![
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![
                        limit_plane_from_plane(&leaf.support),
                        axis_halfspace(0, false, r(1)),
                    ],
                    active_planes: [Some(9), None, None],
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

        assert_eq!(point.point, witness.point);
        assert!(!point.uncertified_definition_fallback);
        assert!(!point.planes.is_empty());
    }

    #[test]
    fn strict_leaf_witness_keeps_certified_replay_after_invalid_active_index() {
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
        assert!(!point.uncertified_definition_fallback);
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

        assert!(!definitions.saw_unknown);
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
    fn strict_probe_witness_stays_certified_when_active_replay_is_singular() {
        let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let witness = p(1, 1, 1);
        let halfspaces = vec![axis_halfspace(2, false, r(2))];

        let probe = build_probe_point(
            &witness,
            &corridor,
            &support,
            &halfspaces,
            [Some(9), None, None],
            &[],
            false,
        )
        .unwrap()
        .expect("strict probe witness should still be retained");

        assert_eq!(probe.point, witness);
        assert!(!probe.uncertified_definition_fallback);
        assert!(probe.planes.iter().any(|definition| {
            definition_planes_match_as_sets(definition, &axis_plane_definition(&probe.point))
        }));
    }

    #[test]
    fn strict_probe_witness_preserves_inherited_uncertified_definition_fallback() {
        let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let witness = p(1, 1, 1);
        let halfspaces = vec![axis_halfspace(2, false, r(2))];

        let probe = build_probe_point(
            &witness,
            &corridor,
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
    fn strict_probe_witness_reports_unknown_for_support_boundary_contact() {
        let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let witness = p(1, 1, 0);
        let halfspaces = vec![axis_halfspace(0, false, r(2))];

        assert_eq!(
            build_probe_point(
                &witness,
                &corridor,
                &support,
                &halfspaces,
                [None, None, None],
                &[],
                false
            ),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn strict_probe_witness_from_shifted_witness_merges_definition_families() {
        let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let witness = ShiftedHalfspaceWitness {
            point: p(1, 1, 1),
            families: vec![
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(0, false, r(2))],
                    active_planes: [Some(0), None, None],
                },
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(1, false, r(2))],
                    active_planes: [Some(0), None, None],
                },
            ],
            uncertified_definition_fallback: false,
        };

        let probe = build_probe_point_from_shifted_witness(&witness, &corridor, &support, &[])
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
    fn strict_probe_witness_from_shifted_witness_reports_unknown_for_support_boundary_contact() {
        let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let witness = ShiftedHalfspaceWitness {
            point: p(1, 1, 0),
            families: vec![ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(0, false, r(2))],
                active_planes: [Some(0), None, None],
            }],
            uncertified_definition_fallback: false,
        };

        assert_eq!(
            build_probe_point_from_shifted_witness(&witness, &corridor, &support, &[]),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn strict_probe_witness_from_shifted_witness_stays_certified_when_one_family_is_singular() {
        let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let witness = ShiftedHalfspaceWitness {
            point: p(1, 1, 1),
            families: vec![
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(2, false, r(2))],
                    active_planes: [Some(9), None, None],
                },
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(0, false, r(2))],
                    active_planes: [Some(0), None, None],
                },
            ],
            uncertified_definition_fallback: false,
        };

        let probe = build_probe_point_from_shifted_witness(&witness, &corridor, &support, &[])
            .unwrap()
            .expect("shifted witness should still certify a strict probe");

        assert_eq!(probe.point, witness.point);
        assert!(!probe.uncertified_definition_fallback);
        assert!(!probe.planes.is_empty());
    }

    #[test]
    fn strict_probe_witness_reports_unknown_for_halfspace_boundary_contact() {
        let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let witness = p(1, 1, 1);
        let halfspaces = vec![axis_halfspace(2, false, r(1))];

        assert_eq!(
            build_probe_point(
                &witness,
                &corridor,
                &support,
                &halfspaces,
                [None, None, None],
                &[],
                false,
            ),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn strict_axis_probe_witness_stays_certified_when_active_replay_is_singular() {
        let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![axis_plane_definition(&p(1, 1, 0))],
            uncertified_definition_fallback: false,
        };
        let witness = p(2, 1, 1);
        let halfspaces = vec![axis_halfspace(0, false, r(3))];

        let probe = build_axis_probe_point(
            &witness,
            &interior,
            &corridor,
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
        assert!(!probe.uncertified_definition_fallback);
        assert!(probe.planes.iter().any(|definition| {
            definition_planes_match_as_sets(definition, &axis_plane_definition(&probe.point))
        }));
    }

    #[test]
    fn strict_axis_probe_witness_preserves_inherited_uncertified_definition_fallback() {
        let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![axis_plane_definition(&p(1, 1, 0))],
            uncertified_definition_fallback: false,
        };
        let witness = p(2, 1, 1);
        let halfspaces = vec![axis_halfspace(0, false, r(3))];

        let probe = build_axis_probe_point(
            &witness,
            &interior,
            &corridor,
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
    fn strict_axis_probe_witness_reports_unknown_for_support_boundary_contact() {
        let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![axis_plane_definition(&p(1, 1, 0))],
            uncertified_definition_fallback: false,
        };
        let witness = p(2, 1, 0);
        let halfspaces = vec![axis_halfspace(0, false, r(3))];

        assert_eq!(
            build_axis_probe_point(
                &witness,
                &interior,
                &corridor,
                &support,
                0,
                None,
                &halfspaces,
                [None, None, None],
                false,
            ),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn strict_axis_probe_witness_from_shifted_witness_reports_unknown_for_support_boundary_contact()
    {
        let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![axis_plane_definition(&p(1, 1, 0))],
            uncertified_definition_fallback: false,
        };
        let witness = ShiftedHalfspaceWitness {
            point: p(2, 1, 0),
            families: vec![ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(0, false, r(3))],
                active_planes: [Some(0), None, None],
            }],
            uncertified_definition_fallback: false,
        };

        assert_eq!(
            build_axis_probe_point_from_shifted_witness(
                &witness, &interior, &corridor, &support, 0, None
            ),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn strict_axis_probe_witness_from_shifted_witness_stays_certified_when_one_family_is_singular()
    {
        let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![axis_plane_definition(&p(1, 1, 0))],
            uncertified_definition_fallback: false,
        };
        let witness = ShiftedHalfspaceWitness {
            point: p(2, 1, 1),
            families: vec![
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(0, false, r(3))],
                    active_planes: [Some(9), None, None],
                },
                ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(1, false, r(2))],
                    active_planes: [Some(0), None, None],
                },
            ],
            uncertified_definition_fallback: false,
        };

        let probe = build_axis_probe_point_from_shifted_witness(
            &witness, &interior, &corridor, &support, 0, None,
        )
        .unwrap()
        .expect("shifted witness should still certify a strict axis probe");

        assert_eq!(probe.point, witness.point);
        assert!(!probe.uncertified_definition_fallback);
        assert!(!probe.planes.is_empty());
    }

    #[test]
    fn strict_probe_witness_from_shifted_witness_reports_unknown_for_halfspace_boundary_contact() {
        let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let witness = ShiftedHalfspaceWitness {
            point: p(1, 1, 1),
            families: vec![ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(0, false, r(1))],
                active_planes: [Some(0), None, None],
            }],
            uncertified_definition_fallback: false,
        };

        assert_eq!(
            build_probe_point_from_shifted_witness(&witness, &corridor, &support, &[]),
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn strict_axis_probe_witness_reports_unknown_for_halfspace_boundary_contact() {
        let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
        let support = Plane::axis_aligned(2, r(0));
        let interior = InteriorLeafPoint {
            point: p(1, 1, 0),
            planes: vec![axis_plane_definition(&p(1, 1, 0))],
            uncertified_definition_fallback: false,
        };
        let witness = p(2, 1, 1);
        let halfspaces = vec![axis_halfspace(0, false, r(2))];

        assert_eq!(
            build_axis_probe_point(
                &witness,
                &interior,
                &corridor,
                &support,
                0,
                None,
                &halfspaces,
                [None, None, None],
                false,
            ),
            Err(HypermeshError::UnknownClassification)
        );
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
    fn duplicate_probe_points_prefer_certified_duplicate_definitions() {
        let point = p(1, 1, 1);
        let definition = axis_plane_definition(&point);
        let mut probes = vec![ProbePoint {
            point: point.clone(),
            side: Classification::Positive,
            planes: vec![definition.clone()],
            uncertified_definition_fallback: true,
        }];

        push_unique_probe_point(
            &mut probes,
            ProbePoint {
                point,
                side: Classification::Positive,
                planes: vec![definition],
                uncertified_definition_fallback: false,
            },
        );

        assert_eq!(probes.len(), 1);
        assert!(!probes[0].uncertified_definition_fallback);
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
    fn duplicate_interior_points_prefer_certified_duplicate_definitions() {
        let point = p(1, 1, 1);
        let definition = axis_plane_definition(&point);
        let mut points = vec![InteriorLeafPoint {
            point: point.clone(),
            planes: vec![definition.clone()],
            uncertified_definition_fallback: true,
        }];

        push_unique_interior_point(
            &mut points,
            InteriorLeafPoint {
                point,
                planes: vec![definition],
                uncertified_definition_fallback: false,
            },
        );

        assert_eq!(points.len(), 1);
        assert!(!points[0].uncertified_definition_fallback);
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
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
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
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
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
    fn detour_trace_cycle_guard_allows_same_point_definition_transition_at_start() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definition = axis_plane_definition(&start);
        let end_definition = axis_plane_definition(&end);
        let lifted_start_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::new(Point3::new(r(1), r(1), r(1)), r(0)),
        ];
        let winding = trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[5],
            &[],
            std::slice::from_ref(&start_definition),
            std::slice::from_ref(&end_definition),
            &initial_visited_definition_points(
                &start,
                std::slice::from_ref(&start_definition),
                &end,
                std::slice::from_ref(&end_definition),
            ),
            &mut Vec::new(),
            &mut |_point| Ok(false),
            &mut |from, to, winding, start_definitions, end_definitions| {
                if *from == start
                    && *to == end
                    && start_definitions == std::slice::from_ref(&start_definition)
                    && end_definitions == std::slice::from_ref(&end_definition)
                {
                    Ok(None)
                } else if *from == start
                    && *to == start
                    && start_definitions == std::slice::from_ref(&start_definition)
                    && end_definitions == std::slice::from_ref(&lifted_start_definition)
                {
                    Ok(Some(winding.to_vec()))
                } else if *from == start
                    && *to == end
                    && start_definitions == std::slice::from_ref(&lifted_start_definition)
                    && end_definitions == std::slice::from_ref(&end_definition)
                {
                    Ok(Some(vec![7]))
                } else {
                    Ok(None)
                }
            },
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![DetourTarget {
                        point: start.clone(),
                        definitions: vec![lifted_start_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert_eq!(winding, vec![7]);
    }

    #[test]
    fn detour_trace_cycle_guard_allows_same_point_definition_transition_on_surface() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definition = axis_plane_definition(&start);
        let end_definition = axis_plane_definition(&end);
        let lifted_start_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::new(Point3::new(r(1), r(1), r(1)), r(0)),
        ];

        let winding = trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[5],
            &[],
            std::slice::from_ref(&start_definition),
            std::slice::from_ref(&end_definition),
            &initial_visited_definition_points(
                &start,
                std::slice::from_ref(&start_definition),
                &end,
                std::slice::from_ref(&end_definition),
            ),
            &mut Vec::new(),
            &mut |point| Ok(*point == start),
            &mut |from, to, winding, start_definitions, end_definitions| {
                if *from == start
                    && *to == end
                    && start_definitions == std::slice::from_ref(&start_definition)
                    && end_definitions == std::slice::from_ref(&end_definition)
                {
                    Ok(None)
                } else if *from == start
                    && *to == start
                    && start_definitions == std::slice::from_ref(&start_definition)
                    && end_definitions == std::slice::from_ref(&lifted_start_definition)
                {
                    Ok(Some(winding.to_vec()))
                } else if *from == start
                    && *to == end
                    && start_definitions == std::slice::from_ref(&lifted_start_definition)
                    && end_definitions == std::slice::from_ref(&end_definition)
                {
                    Ok(Some(vec![7]))
                } else {
                    Ok(None)
                }
            },
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![DetourTarget {
                        point: start.clone(),
                        definitions: vec![lifted_start_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert_eq!(winding, vec![7]);
    }

    #[test]
    fn detour_trace_cycle_guard_allows_revisiting_point_with_new_definitions() {
        let start = p(0, 0, 0);
        let shared = p(1, 0, 0);
        let mid = p(2, 0, 0);
        let end = p(3, 0, 0);
        let start_definition = axis_plane_definition(&start);
        let shared_definition = axis_plane_definition(&shared);
        let mid_definition = axis_plane_definition(&mid);
        let end_definition = axis_plane_definition(&end);
        let lifted_shared_definition = [
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(0)),
            Plane::new(Point3::new(r(1), r(1), r(1)), r(-1)),
        ];

        let winding = trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[5],
            &[],
            std::slice::from_ref(&start_definition),
            std::slice::from_ref(&end_definition),
            &initial_visited_definition_points(
                &start,
                std::slice::from_ref(&start_definition),
                &end,
                std::slice::from_ref(&end_definition),
            ),
            &mut Vec::new(),
            &mut |_point| Ok(false),
            &mut |from, to, winding, start_definitions, end_definitions| {
                if *from == start
                    && *to == end
                    && start_definitions == std::slice::from_ref(&start_definition)
                    && end_definitions == std::slice::from_ref(&end_definition)
                {
                    Ok(None)
                } else if *from == start
                    && *to == shared
                    && start_definitions == std::slice::from_ref(&start_definition)
                    && end_definitions == std::slice::from_ref(&shared_definition)
                {
                    Ok(Some(winding.to_vec()))
                } else if *from == shared
                    && *to == end
                    && start_definitions == std::slice::from_ref(&shared_definition)
                    && end_definitions == std::slice::from_ref(&end_definition)
                {
                    Ok(None)
                } else if *from == shared
                    && *to == mid
                    && start_definitions == std::slice::from_ref(&shared_definition)
                    && end_definitions == std::slice::from_ref(&mid_definition)
                {
                    Ok(Some(winding.to_vec()))
                } else if *from == mid
                    && *to == end
                    && start_definitions == std::slice::from_ref(&mid_definition)
                    && end_definitions == std::slice::from_ref(&end_definition)
                {
                    Ok(None)
                } else if *from == mid
                    && *to == shared
                    && start_definitions == std::slice::from_ref(&mid_definition)
                    && end_definitions == std::slice::from_ref(&lifted_shared_definition)
                {
                    Ok(Some(winding.to_vec()))
                } else if *from == shared
                    && *to == end
                    && start_definitions == std::slice::from_ref(&lifted_shared_definition)
                    && end_definitions == std::slice::from_ref(&end_definition)
                {
                    Ok(Some(vec![7]))
                } else {
                    Ok(None)
                }
            },
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![DetourTarget {
                        point: shared.clone(),
                        definitions: vec![shared_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else if *from == shared && *to == end {
                    Ok(vec![DetourTarget {
                        point: mid.clone(),
                        definitions: vec![mid_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else if *from == mid && *to == end {
                    Ok(vec![DetourTarget {
                        point: shared.clone(),
                        definitions: vec![lifted_shared_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert_eq!(winding, vec![7]);
    }

    #[test]
    fn detour_trace_cycle_guard_skips_fallback_detour_even_when_legs_succeed() {
        let start = p(0, 0, 0);
        let fallback_detour = p(1, 0, 0);
        let certified_detour = p(2, 0, 0);
        let end = p(3, 0, 0);
        let mut surface_cache = Vec::new();

        let winding = trace_segment_via_detours_with_cycle_guard_with_surface_query(
            &start,
            &end,
            &[0],
            &[],
            &[
                DetourTarget {
                    point: fallback_detour.clone(),
                    definitions: vec![axis_plane_definition(&fallback_detour)],
                    uncertified_definition_fallback: true,
                },
                DetourTarget {
                    point: certified_detour.clone(),
                    definitions: vec![axis_plane_definition(&certified_detour)],
                    uncertified_definition_fallback: false,
                },
            ],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
            &mut surface_cache,
            &mut |_point| Ok(false),
            &mut |_from, to, winding, _start_definitions, _end_definitions| {
                Ok(Some(vec![if *to == fallback_detour {
                    winding[0] + 1
                } else {
                    winding[0] + 2
                }]))
            },
            &mut |_from, _to| Ok(Vec::new()),
        )
        .unwrap();

        assert_eq!(winding, Some(vec![4]));
    }

    #[test]
    fn detour_trace_cycle_guard_reports_unknown_when_only_fallback_detour_succeeds() {
        let start = p(0, 0, 0);
        let fallback_detour = p(1, 0, 0);
        let end = p(2, 0, 0);
        let mut surface_cache = Vec::new();

        let err = trace_segment_via_detours_with_cycle_guard_with_surface_query(
            &start,
            &end,
            &[0],
            &[],
            &[DetourTarget {
                point: fallback_detour.clone(),
                definitions: vec![axis_plane_definition(&fallback_detour)],
                uncertified_definition_fallback: true,
            }],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
            &mut surface_cache,
            &mut |_point| Ok(false),
            &mut |_from, _to, winding, _start_definitions, _end_definitions| {
                Ok(Some(winding.to_vec()))
            },
            &mut |_from, _to| Ok(Vec::new()),
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
    }

    #[test]
    fn detour_trace_cycle_guard_tries_later_detour_after_boundary_surface_query() {
        let start = p(0, 0, 0);
        let first_detour = p(1, 0, 0);
        let second_detour = p(2, 0, 1);
        let end = p(3, 0, 0);
        let polygon = make_triangle(&p(1, 0, 0), &p(2, 0, 0), &p(1, 1, 0), 0, 0);
        let mut surface_cache = Vec::new();

        let winding = trace_segment_via_detours_with_cycle_guard_with_surface_query(
            &start,
            &end,
            &[0],
            std::slice::from_ref(&polygon),
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
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
            &mut surface_cache,
            &mut |point| point_lies_on_traced_surface(point, std::slice::from_ref(&polygon)),
            &mut |_from, _to, winding, _start_definitions, _end_definitions| {
                Ok(Some(winding.to_vec()))
            },
            &mut |_from, _to| Ok(Vec::new()),
        )
        .unwrap();

        assert_eq!(winding, Some(vec![0]));
    }

    #[test]
    fn point_lies_on_traced_surface_reports_unknown_for_boundary_contact() {
        let polygon = make_triangle(&p(1, 0, 0), &p(2, 0, 0), &p(1, 1, 0), 0, 0);

        assert_eq!(
            point_lies_on_traced_surface(&p(1, 0, 0), std::slice::from_ref(&polygon)),
            Err(HypermeshError::UnknownClassification)
        );
        assert!(!point_lies_on_traced_surface(&p(3, 3, 0), &[polygon]).unwrap());
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
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
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
    fn probe_reaches_adjacent_cell_accepts_zero_length_clear_point() {
        let host_support = Plane::axis_aligned(2, r(0));

        assert!(probe_reaches_adjacent_cell(&p(1, 1, 1), &p(1, 1, 1), &host_support, &[]).unwrap());
    }

    #[test]
    fn probe_reaches_adjacent_cell_reports_unknown_for_zero_length_surface_contact() {
        let host_support = Plane::axis_aligned(2, r(0));
        let blocker = make_triangle(&p(1, 0, 0), &p(1, -1, 1), &p(1, 1, 1), 0, 0);

        assert_eq!(
            probe_reaches_adjacent_cell(&p(1, 0, 0), &p(1, 0, 0), &host_support, &[blocker]),
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
    fn probe_reachability_definition_search_preferring_precheck_short_circuits_true_pair() {
        let start = p(0, 0, 0);
        let end = p(1, 1, 1);
        let start_defs = [
            axis_plane_definition(&start),
            [
                Plane::axis_aligned(0, r(0)),
                Plane::axis_aligned(1, r(0)),
                Plane::from_coefficients(r(1), r(1), r(1), r(0)),
            ],
        ];
        let end_defs = [
            axis_plane_definition(&end),
            [
                Plane::axis_aligned(0, r(1)),
                Plane::axis_aligned(1, r(1)),
                Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
            ],
        ];
        let mut replacement_calls = 0;

        let reaches = probe_reaches_adjacent_cell_with_definition_search_preferring_precheck(
            &start,
            &end,
            &start_defs,
            &end_defs,
            || Ok(false),
            |start_definition, end_definition| {
                if definition_planes_match_as_sets(start_definition, &start_defs[1])
                    && definition_planes_match_as_sets(end_definition, &end_defs[1])
                {
                    Ok(true)
                } else {
                    Ok(false)
                }
            },
            |_start_definition, _end_definition| {
                replacement_calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert!(reaches);
        assert_eq!(replacement_calls, 0);
    }

    #[test]
    fn probe_reachability_definition_search_preferring_precheck_prioritizes_unknown_pairs() {
        let start = p(0, 0, 0);
        let end = p(1, 1, 1);
        let start_defs = [
            axis_plane_definition(&start),
            [
                Plane::axis_aligned(0, r(0)),
                Plane::axis_aligned(1, r(0)),
                Plane::from_coefficients(r(1), r(1), r(1), r(0)),
            ],
        ];
        let end_defs = [
            axis_plane_definition(&end),
            [
                Plane::axis_aligned(0, r(1)),
                Plane::axis_aligned(1, r(1)),
                Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
            ],
        ];
        let mut seen_pairs = Vec::new();

        let reaches = probe_reaches_adjacent_cell_with_definition_search_preferring_precheck(
            &start,
            &end,
            &start_defs,
            &end_defs,
            || Ok(false),
            |start_definition, end_definition| {
                if definition_planes_match_as_sets(start_definition, &start_defs[1])
                    && definition_planes_match_as_sets(end_definition, &end_defs[0])
                {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(false)
                }
            },
            |start_definition, end_definition| {
                let start_index =
                    if definition_planes_match_as_sets(start_definition, &start_defs[0]) {
                        0
                    } else {
                        1
                    };
                let end_index = if definition_planes_match_as_sets(end_definition, &end_defs[0]) {
                    0
                } else {
                    1
                };
                seen_pairs.push((start_index, end_index));
                Ok(start_index == 1 && end_index == 0)
            },
        )
        .unwrap();

        assert!(reaches);
        assert_eq!(seen_pairs.first().copied(), Some((1, 0)));
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
                &initial_visited_definition_points(
                    &start,
                    &start_definitions,
                    &end,
                    &end_definitions,
                ),
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
                &initial_visited_definition_points(
                    &start,
                    &[axis_plane_definition(&start)],
                    &end,
                    &[axis_plane_definition(&end)],
                ),
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
                &initial_visited_definition_points(
                    &start,
                    &[axis_plane_definition(&start)],
                    &end,
                    &[axis_plane_definition(&end)],
                ),
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
                &initial_visited_definition_points(
                    &start,
                    &[axis_plane_definition(&start)],
                    &end,
                    &[axis_plane_definition(&end)],
                ),
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
                &initial_visited_definition_points(
                    &start,
                    &[axis_plane_definition(&start)],
                    &end,
                    &[axis_plane_definition(&end)],
                ),
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
                &initial_visited_definition_points(
                    &start,
                    &[axis_plane_definition(&start)],
                    &end,
                    &[axis_plane_definition(&end)],
                ),
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
    fn probe_step_detour_cycle_guard_allows_same_point_definition_transition_at_start() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definition = axis_plane_definition(&start);
        let end_definition = axis_plane_definition(&end);
        let lifted_start_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::new(Point3::new(r(1), r(1), r(1)), r(0)),
        ];

        assert!(
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                &initial_visited_definition_points(
                    &start,
                    std::slice::from_ref(&start_definition),
                    &end,
                    std::slice::from_ref(&end_definition),
                ),
                std::slice::from_ref(&start_definition),
                std::slice::from_ref(&end_definition),
                &mut Vec::new(),
                &mut |_point| Ok(false),
                &mut |from, to, start_definitions, end_definitions| {
                    Ok(
                        (*from == start
                            && *to == start
                            && start_definitions == std::slice::from_ref(&start_definition)
                            && end_definitions == std::slice::from_ref(&lifted_start_definition))
                            || (*from == start
                                && *to == end
                                && start_definitions
                                    == std::slice::from_ref(&lifted_start_definition)
                                && end_definitions == std::slice::from_ref(&end_definition)),
                    )
                },
                &mut |from, to| {
                    if *from == start && *to == end {
                        Ok(vec![DetourTarget {
                            point: start.clone(),
                            definitions: vec![lifted_start_definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
                    } else {
                        Ok(Vec::new())
                    }
                },
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_step_detour_cycle_guard_allows_same_point_definition_transition_on_surface() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definition = axis_plane_definition(&start);
        let end_definition = axis_plane_definition(&end);
        let lifted_start_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::new(Point3::new(r(1), r(1), r(1)), r(0)),
        ];

        assert!(
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                &initial_visited_definition_points(
                    &start,
                    std::slice::from_ref(&start_definition),
                    &end,
                    std::slice::from_ref(&end_definition),
                ),
                std::slice::from_ref(&start_definition),
                std::slice::from_ref(&end_definition),
                &mut Vec::new(),
                &mut |point| Ok(*point == start),
                &mut |from, to, start_definitions, end_definitions| {
                    Ok(
                        (*from == start
                            && *to == start
                            && start_definitions == std::slice::from_ref(&start_definition)
                            && end_definitions == std::slice::from_ref(&lifted_start_definition))
                            || (*from == start
                                && *to == end
                                && start_definitions
                                    == std::slice::from_ref(&lifted_start_definition)
                                && end_definitions == std::slice::from_ref(&end_definition)),
                    )
                },
                &mut |from, to| {
                    if *from == start && *to == end {
                        Ok(vec![DetourTarget {
                            point: start.clone(),
                            definitions: vec![lifted_start_definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
                    } else {
                        Ok(Vec::new())
                    }
                },
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_step_detour_cycle_guard_allows_revisiting_point_with_new_definitions() {
        let start = p(0, 0, 0);
        let shared = p(1, 0, 0);
        let mid = p(2, 0, 0);
        let end = p(3, 0, 0);
        let start_definition = axis_plane_definition(&start);
        let shared_definition = axis_plane_definition(&shared);
        let mid_definition = axis_plane_definition(&mid);
        let end_definition = axis_plane_definition(&end);
        let lifted_shared_definition = [
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(0)),
            Plane::new(Point3::new(r(1), r(1), r(1)), r(-1)),
        ];

        assert!(
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                &initial_visited_definition_points(
                    &start,
                    std::slice::from_ref(&start_definition),
                    &end,
                    std::slice::from_ref(&end_definition),
                ),
                std::slice::from_ref(&start_definition),
                std::slice::from_ref(&end_definition),
                &mut Vec::new(),
                &mut |_point| Ok(false),
                &mut |from, to, start_definitions, end_definitions| {
                    Ok(
                        (*from == start
                            && *to == shared
                            && start_definitions == std::slice::from_ref(&start_definition)
                            && end_definitions == std::slice::from_ref(&shared_definition))
                            || (*from == shared
                                && *to == mid
                                && start_definitions
                                    == std::slice::from_ref(&shared_definition)
                                && end_definitions == std::slice::from_ref(&mid_definition))
                            || (*from == mid
                                && *to == shared
                                && start_definitions == std::slice::from_ref(&mid_definition)
                                && end_definitions
                                    == std::slice::from_ref(&lifted_shared_definition))
                            || (*from == shared
                                && *to == end
                                && start_definitions
                                    == std::slice::from_ref(&lifted_shared_definition)
                                && end_definitions == std::slice::from_ref(&end_definition)),
                    )
                },
                &mut |from, to| {
                    if *from == start && *to == end {
                        Ok(vec![DetourTarget {
                            point: shared.clone(),
                            definitions: vec![shared_definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
                    } else if *from == shared && *to == end {
                        Ok(vec![DetourTarget {
                            point: mid.clone(),
                            definitions: vec![mid_definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
                    } else if *from == mid && *to == end {
                        Ok(vec![DetourTarget {
                            point: shared.clone(),
                            definitions: vec![lifted_shared_definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
                    } else {
                        Ok(Vec::new())
                    }
                },
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_step_detour_cycle_guard_skips_fallback_detour_even_when_path_succeeds() {
        let start = p(0, 0, 0);
        let fallback_detour = p(1, 0, 0);
        let certified_detour = p(2, 0, 0);
        let end = p(3, 0, 0);

        assert!(
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                &initial_visited_definition_points(
                    &start,
                    &[axis_plane_definition(&start)],
                    &end,
                    &[axis_plane_definition(&end)],
                ),
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &mut Vec::new(),
                &mut |_point| Ok(false),
                &mut |from, to, _start_definitions, _end_definitions| {
                    if *from == start && *to == end {
                        Ok(false)
                    } else {
                        Ok(true)
                    }
                },
                &mut |from, to| {
                    if *from == start && *to == end {
                        Ok(vec![
                            DetourTarget {
                                point: fallback_detour.clone(),
                                definitions: vec![axis_plane_definition(&fallback_detour)],
                                uncertified_definition_fallback: true,
                            },
                            DetourTarget {
                                point: certified_detour.clone(),
                                definitions: vec![axis_plane_definition(&certified_detour)],
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
    fn probe_step_detour_cycle_guard_reports_unknown_when_only_fallback_detour_succeeds() {
        let start = p(0, 0, 0);
        let fallback_detour = p(1, 0, 0);
        let end = p(2, 0, 0);

        let err =
            probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                &initial_visited_definition_points(
                    &start,
                    &[axis_plane_definition(&start)],
                    &end,
                    &[axis_plane_definition(&end)],
                ),
                &[axis_plane_definition(&start)],
                &[axis_plane_definition(&end)],
                &mut Vec::new(),
                &mut |_point| Ok(false),
                &mut |from, to, _start_definitions, _end_definitions| {
                    if *from == start && *to == end {
                        Ok(false)
                    } else {
                        Ok(true)
                    }
                },
                &mut |from, to| {
                    if *from == start && *to == end {
                        Ok(vec![DetourTarget {
                            point: fallback_detour.clone(),
                            definitions: vec![axis_plane_definition(&fallback_detour)],
                            uncertified_definition_fallback: true,
                        }])
                    } else {
                        Ok(Vec::new())
                    }
                },
            )
            .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
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
                &initial_visited_definition_points(
                    &start,
                    &[axis_plane_definition(&start)],
                    &end,
                    &[axis_plane_definition(&end)],
                ),
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
        let mut no_plane_replacement_cycle_guard_cache = Vec::new();
        let mut no_plane_replacement_cache = Vec::new();
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
                &mut no_plane_replacement_cycle_guard_cache,
                &mut no_plane_replacement_cache,
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
        let mut no_plane_replacement_cycle_guard_cache = Vec::new();
        let mut no_plane_replacement_cache = Vec::new();
        let mut detour_target_cache = Vec::new();

        let first = probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
            &start,
            &end,
            &[],
            &start_definitions,
            &end_definitions,
            &mut no_detour_cache,
            &mut no_plane_replacement_cycle_guard_cache,
            &mut no_plane_replacement_cache,
            &mut detour_target_cache,
            |_from, _to, _start_definitions, _end_definitions| Ok(true),
            |_from, _to| Ok(Vec::new()),
        )
        .unwrap();
        let no_detour_len = no_detour_cache.len();
        let no_plane_replacement_cycle_guard_len = no_plane_replacement_cycle_guard_cache.len();
        let no_plane_replacement_len = no_plane_replacement_cache.len();
        let detour_len = detour_target_cache.len();
        let second = probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
            &start,
            &end,
            &[],
            &start_definitions,
            &end_definitions,
            &mut no_detour_cache,
            &mut no_plane_replacement_cycle_guard_cache,
            &mut no_plane_replacement_cache,
            &mut detour_target_cache,
            |_from, _to, _start_definitions, _end_definitions| Ok(true),
            |_from, _to| Ok(Vec::new()),
        )
        .unwrap();

        assert!(first);
        assert!(second);
        assert_eq!(no_detour_cache.len(), no_detour_len);
        assert_eq!(
            no_plane_replacement_cycle_guard_cache.len(),
            no_plane_replacement_cycle_guard_len
        );
        assert_eq!(no_plane_replacement_cache.len(), no_plane_replacement_len);
        assert_eq!(detour_target_cache.len(), detour_len);
    }

    #[test]
    fn definition_no_plane_replacement_cycle_guard_cache_reuses_false_for_superset_visited_points()
    {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let shared = p(0, 1, 0);
        let extra = p(0, 2, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let shared_definitions = vec![axis_plane_definition(&shared)];
        let extra_definitions = vec![axis_plane_definition(&extra)];
        let cached_visited = vec![VisitedDefinitionPoint {
            point: shared.clone(),
            definitions: shared_definitions.clone(),
        }];
        let current_visited = vec![
            cached_visited[0].clone(),
            VisitedDefinitionPoint {
                point: extra,
                definitions: extra_definitions,
            },
        ];
        let cache = vec![DefinitionNoPlaneReplacementCycleGuardCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: cached_visited,
            result: Ok(false),
        }];

        let reused = cached_definition_no_plane_replacement_cycle_guard_result(
            &cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            &current_visited,
        );

        assert_eq!(reused, Some(Ok(false)));
    }

    #[test]
    fn definition_no_plane_replacement_cycle_guard_cache_reuses_true_for_subset_visited_points() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let shared = p(0, 1, 0);
        let extra = p(0, 2, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let shared_definitions = vec![axis_plane_definition(&shared)];
        let extra_definitions = vec![axis_plane_definition(&extra)];
        let current_visited = vec![VisitedDefinitionPoint {
            point: shared.clone(),
            definitions: shared_definitions.clone(),
        }];
        let cached_visited = vec![
            current_visited[0].clone(),
            VisitedDefinitionPoint {
                point: extra,
                definitions: extra_definitions,
            },
        ];
        let cache = vec![DefinitionNoPlaneReplacementCycleGuardCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: cached_visited,
            result: Ok(true),
        }];

        let reused = cached_definition_no_plane_replacement_cycle_guard_result(
            &cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            &current_visited,
        );

        assert_eq!(reused, Some(Ok(true)));
    }

    #[test]
    fn definition_no_plane_replacement_cycle_guard_cache_reuses_reversed_query() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let visited_points = vec![
            VisitedDefinitionPoint {
                point: start.clone(),
                definitions: start_definitions.clone(),
            },
            VisitedDefinitionPoint {
                point: end.clone(),
                definitions: end_definitions.clone(),
            },
        ];
        let cache = vec![DefinitionNoPlaneReplacementCycleGuardCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: visited_points.clone(),
            result: Ok(true),
        }];

        let reused = cached_definition_no_plane_replacement_cycle_guard_result(
            &cache,
            &end,
            &start,
            &end_definitions,
            &start_definitions,
            &visited_points,
        );

        assert_eq!(reused, Some(Ok(true)));
    }

    #[test]
    fn definition_no_plane_replacement_cycle_guard_cache_ignores_redundant_current_endpoints() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let shared = p(0, 1, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let shared_definitions = vec![axis_plane_definition(&shared)];
        let cache = vec![DefinitionNoPlaneReplacementCycleGuardCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: vec![VisitedDefinitionPoint {
                point: shared.clone(),
                definitions: shared_definitions.clone(),
            }],
            result: Ok(true),
        }];
        let current_visited = vec![
            VisitedDefinitionPoint {
                point: start.clone(),
                definitions: start_definitions.clone(),
            },
            VisitedDefinitionPoint {
                point: end.clone(),
                definitions: end_definitions.clone(),
            },
            VisitedDefinitionPoint {
                point: shared,
                definitions: shared_definitions,
            },
        ];

        let reused = cached_definition_no_plane_replacement_cycle_guard_result(
            &cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            &current_visited,
        );

        assert_eq!(reused, Some(Ok(true)));
    }

    #[test]
    fn definition_no_plane_replacement_cycle_guard_cache_reuses_in_progress_exact_state() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let shared = p(0, 1, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let shared_definitions = vec![axis_plane_definition(&shared)];
        let visited_points = vec![VisitedDefinitionPoint {
            point: shared,
            definitions: shared_definitions,
        }];
        let mut cache = Vec::new();
        let index = begin_definition_no_plane_replacement_cycle_guard_result(
            &mut cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            &visited_points,
        );

        let reused = cached_definition_no_plane_replacement_cycle_guard_result(
            &cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            &visited_points,
        );

        assert_eq!(reused, Some(Err(HypermeshError::UnknownClassification)));
        assert_eq!(
            cache[index].result,
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn definition_cycle_guard_cache_reuses_identical_query() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let shared = p(0, 1, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let shared_definitions = vec![axis_plane_definition(&shared)];
        let visited_points = vec![VisitedDefinitionPoint {
            point: shared.clone(),
            definitions: shared_definitions,
        }];
        let cache = vec![DefinitionCycleGuardReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: visited_points.clone(),
            result: Ok(true),
        }];

        let reused = cached_definition_cycle_guard_result(
            &cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            &visited_points,
        );

        assert_eq!(reused, Some(Ok(true)));
    }

    #[test]
    fn definition_cycle_guard_cache_reuses_reversed_query() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let shared = p(0, 1, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let visited_points = vec![VisitedDefinitionPoint {
            point: shared.clone(),
            definitions: vec![axis_plane_definition(&shared)],
        }];
        let cache = vec![DefinitionCycleGuardReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: visited_points.clone(),
            result: Ok(true),
        }];

        let reused = cached_definition_cycle_guard_result(
            &cache,
            &end,
            &start,
            &end_definitions,
            &start_definitions,
            &visited_points,
        );

        assert_eq!(reused, Some(Ok(true)));
    }

    #[test]
    fn definition_cycle_guard_cache_ignores_redundant_current_endpoints() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let shared = p(0, 1, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let shared_definitions = vec![axis_plane_definition(&shared)];
        let cache = vec![DefinitionCycleGuardReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: vec![VisitedDefinitionPoint {
                point: shared.clone(),
                definitions: shared_definitions.clone(),
            }],
            result: Err(HypermeshError::UnknownClassification),
        }];
        let current_visited = vec![
            VisitedDefinitionPoint {
                point: start.clone(),
                definitions: start_definitions.clone(),
            },
            VisitedDefinitionPoint {
                point: end.clone(),
                definitions: end_definitions.clone(),
            },
            VisitedDefinitionPoint {
                point: shared,
                definitions: shared_definitions,
            },
        ];

        let reused = cached_definition_cycle_guard_result(
            &cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            &current_visited,
        );

        assert_eq!(reused, Some(Err(HypermeshError::UnknownClassification)));
    }

    #[test]
    fn definition_cycle_guard_cache_reuses_in_progress_exact_state() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let shared = p(0, 1, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let shared_definitions = vec![axis_plane_definition(&shared)];
        let visited_points = vec![VisitedDefinitionPoint {
            point: shared,
            definitions: shared_definitions,
        }];
        let mut cache = Vec::new();
        let index = begin_definition_cycle_guard_result(
            &mut cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            &visited_points,
        );

        let reused = cached_definition_cycle_guard_result(
            &cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            &visited_points,
        );

        assert_eq!(reused, Some(Err(HypermeshError::UnknownClassification)));
        assert_eq!(
            cache[index].result,
            Err(HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn definition_cycle_guard_cache_reuses_false_for_superset_visited_points() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let shared = p(0, 1, 0);
        let extra = p(0, 2, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let shared_definitions = vec![axis_plane_definition(&shared)];
        let extra_definitions = vec![axis_plane_definition(&extra)];
        let cached_visited = vec![VisitedDefinitionPoint {
            point: shared.clone(),
            definitions: shared_definitions.clone(),
        }];
        let current_visited = vec![
            cached_visited[0].clone(),
            VisitedDefinitionPoint {
                point: extra,
                definitions: extra_definitions,
            },
        ];
        let cache = vec![DefinitionCycleGuardReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: cached_visited,
            result: Ok(false),
        }];

        let reused = cached_definition_cycle_guard_result(
            &cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            &current_visited,
        );

        assert_eq!(reused, Some(Ok(false)));
    }

    #[test]
    fn definition_cycle_guard_cache_reuses_true_for_subset_visited_points() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let shared = p(0, 1, 0);
        let extra = p(0, 2, 0);
        let start_definitions = vec![axis_plane_definition(&start)];
        let end_definitions = vec![axis_plane_definition(&end)];
        let shared_definitions = vec![axis_plane_definition(&shared)];
        let extra_definitions = vec![axis_plane_definition(&extra)];
        let current_visited = vec![VisitedDefinitionPoint {
            point: shared.clone(),
            definitions: shared_definitions.clone(),
        }];
        let cached_visited = vec![
            current_visited[0].clone(),
            VisitedDefinitionPoint {
                point: extra,
                definitions: extra_definitions,
            },
        ];
        let cache = vec![DefinitionCycleGuardReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: cached_visited,
            result: Ok(true),
        }];

        let reused = cached_definition_cycle_guard_result(
            &cache,
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            &current_visited,
        );

        assert_eq!(reused, Some(Ok(true)));
    }

    #[test]
    fn ordered_interior_points_for_probe_search_prefers_axis_aligned_definition_planes() {
        let slanted = Plane {
            normal: Point3::new(r(2), r(3), r(5)),
            offset: r(7),
        };
        let more_slanted = Plane {
            normal: Point3::new(r(7), r(11), r(13)),
            offset: r(17),
        };
        let most_axis_aligned = InteriorLeafPoint {
            point: p(3, 0, 0),
            planes: vec![axis_plane_definition(&p(3, 0, 0))],
            uncertified_definition_fallback: false,
        };
        let partly_axis_aligned = InteriorLeafPoint {
            point: p(2, 0, 0),
            planes: vec![[
                Plane::axis_aligned(2, r(1)),
                slanted.clone(),
                more_slanted.clone(),
            ]],
            uncertified_definition_fallback: false,
        };
        let non_axis_aligned = InteriorLeafPoint {
            point: p(1, 0, 0),
            planes: vec![[slanted, more_slanted.clone(), more_slanted]],
            uncertified_definition_fallback: false,
        };

        let points = [
            non_axis_aligned.clone(),
            partly_axis_aligned.clone(),
            most_axis_aligned.clone(),
        ];
        let ordered = ordered_interior_points_for_probe_search(&points);

        assert_eq!(ordered[0].point, most_axis_aligned.point);
        assert_eq!(ordered[1].point, partly_axis_aligned.point);
        assert_eq!(ordered[2].point, non_axis_aligned.point);
    }

    #[test]
    fn ordered_interior_points_for_probe_search_with_support_prefers_retained_definition_points_in_root_host_fixture()
     {
        use crate::mesh::prepare_input;
        use crate::polygon::ConvexPolygon;

        fn tetra_from_face_and_apex(
            a: Point3,
            b: Point3,
            c: Point3,
            apex: Point3,
        ) -> crate::InputMesh {
            crate::InputMesh::new(
                vec![a, b, c, apex],
                vec![
                    crate::Triangle::new(0, 2, 1),
                    crate::Triangle::new(0, 1, 3),
                    crate::Triangle::new(0, 3, 2),
                    crate::Triangle::new(1, 2, 3),
                ],
            )
        }

        fn face_at(
            polygons: &[ConvexPolygon],
            mesh_index: isize,
            polygon_index: isize,
        ) -> ConvexPolygon {
            polygons
                .iter()
                .find(|polygon| {
                    polygon.mesh_index == mesh_index && polygon.polygon_index == polygon_index
                })
                .unwrap()
                .clone()
        }

        let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
        let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
        let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
        let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
        let polygons = soup.polygons.clone();
        let host = face_at(&polygons, 0, 1);
        let intersections = polygons
            .iter()
            .enumerate()
            .filter_map(|(index, polygon)| {
                if polygon.mesh_index == host.mesh_index
                    && polygon.polygon_index == host.polygon_index
                {
                    return None;
                }
                let intersection =
                    crate::intersection::intersect_polygons(&host, polygon, index).ok()?;
                Some(intersection)
            })
            .collect::<Vec<_>>();
        let bsp_leaves =
            crate::subdivision::build_host_bsp_leaves(&host, &polygons, &intersections).unwrap();

        let mut checked = 0;
        for (leaf_index, leaf) in bsp_leaves.iter().enumerate() {
            if leaf.edges.len() < 3 {
                continue;
            }
            let Ok((interior_points, _)) =
                crate::subdivision::certify_bsp_leaf_and_delta_w(&host, &leaf.edges, &polygons)
            else {
                continue;
            };
            let ordered = ordered_interior_points_for_probe_search_with_support(
                &interior_points,
                &host.support,
            )
            .unwrap();
            let ordered_indices = ordered
                .iter()
                .map(|ordered_point| {
                    interior_points
                        .iter()
                        .position(|point| point.point == ordered_point.point)
                        .unwrap()
                })
                .collect::<Vec<_>>();

            match leaf_index {
                1 => {
                    assert_eq!(ordered_indices[0], 2);
                    checked += 1;
                }
                2 => {
                    assert_eq!(ordered_indices[0], 0);
                    checked += 1;
                }
                _ => {}
            }
        }

        assert_eq!(checked, 2);
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
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
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
                &initial_visited_definition_points(
                    &start,
                    &[axis_plane_definition(&start)],
                    &end,
                    &[axis_plane_definition(&end)],
                ),
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
                &initial_visited_definition_points(
                    &start,
                    &[axis_plane_definition(&start)],
                    &end,
                    &[axis_plane_definition(&end)],
                ),
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
                &initial_visited_definition_points(
                    &start,
                    &[axis_plane_definition(&start)],
                    &end,
                    &[axis_plane_definition(&end)],
                ),
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
    fn probe_reachability_cycle_guard_allows_same_point_definition_transition_at_start() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definition = axis_plane_definition(&start);
        let end_definition = axis_plane_definition(&end);
        let lifted_start_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::new(Point3::new(r(1), r(1), r(1)), r(0)),
        ];

        assert!(
            probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                std::slice::from_ref(&start_definition),
                std::slice::from_ref(&end_definition),
                &initial_visited_definition_points(
                    &start,
                    std::slice::from_ref(&start_definition),
                    &end,
                    std::slice::from_ref(&end_definition),
                ),
                &mut Vec::new(),
                &mut |_point| Ok(false),
                &mut |from, to, start_definitions, end_definitions| {
                    Ok((*from == start
                        && *to == start
                        && start_definitions == std::slice::from_ref(&start_definition)
                        && end_definitions == std::slice::from_ref(&lifted_start_definition))
                        || (*from == start
                            && *to == end
                            && start_definitions == std::slice::from_ref(&lifted_start_definition)
                            && end_definitions == std::slice::from_ref(&end_definition)))
                },
                &mut |from, to| {
                    if *from == start && *to == end {
                        Ok(vec![DetourTarget {
                            point: start.clone(),
                            definitions: vec![lifted_start_definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
                    } else {
                        Ok(Vec::new())
                    }
                },
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_cycle_guard_allows_same_point_definition_transition_on_surface() {
        let start = p(0, 0, 0);
        let end = p(1, 0, 0);
        let start_definition = axis_plane_definition(&start);
        let end_definition = axis_plane_definition(&end);
        let lifted_start_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::new(Point3::new(r(1), r(1), r(1)), r(0)),
        ];

        assert!(
            probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                std::slice::from_ref(&start_definition),
                std::slice::from_ref(&end_definition),
                &initial_visited_definition_points(
                    &start,
                    std::slice::from_ref(&start_definition),
                    &end,
                    std::slice::from_ref(&end_definition),
                ),
                &mut Vec::new(),
                &mut |point| Ok(*point == start),
                &mut |from, to, start_definitions, end_definitions| {
                    Ok((*from == start
                        && *to == start
                        && start_definitions == std::slice::from_ref(&start_definition)
                        && end_definitions == std::slice::from_ref(&lifted_start_definition))
                        || (*from == start
                            && *to == end
                            && start_definitions == std::slice::from_ref(&lifted_start_definition)
                            && end_definitions == std::slice::from_ref(&end_definition)))
                },
                &mut |from, to| {
                    if *from == start && *to == end {
                        Ok(vec![DetourTarget {
                            point: start.clone(),
                            definitions: vec![lifted_start_definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
                    } else {
                        Ok(Vec::new())
                    }
                },
            )
            .unwrap()
        );
    }

    #[test]
    fn probe_reachability_cycle_guard_allows_revisiting_point_with_new_definitions() {
        let start = p(0, 0, 0);
        let shared = p(1, 0, 0);
        let mid = p(2, 0, 0);
        let end = p(3, 0, 0);
        let start_definition = axis_plane_definition(&start);
        let shared_definition = axis_plane_definition(&shared);
        let mid_definition = axis_plane_definition(&mid);
        let end_definition = axis_plane_definition(&end);
        let lifted_shared_definition = [
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(0)),
            Plane::new(Point3::new(r(1), r(1), r(1)), r(-1)),
        ];

        assert!(
            probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
                &start,
                &end,
                &[],
                std::slice::from_ref(&start_definition),
                std::slice::from_ref(&end_definition),
                &initial_visited_definition_points(
                    &start,
                    std::slice::from_ref(&start_definition),
                    &end,
                    std::slice::from_ref(&end_definition),
                ),
                &mut Vec::new(),
                &mut |_point| Ok(false),
                &mut |from, to, start_definitions, end_definitions| {
                    Ok((*from == start
                        && *to == shared
                        && start_definitions == std::slice::from_ref(&start_definition)
                        && end_definitions == std::slice::from_ref(&shared_definition))
                        || (*from == shared
                            && *to == mid
                            && start_definitions == std::slice::from_ref(&shared_definition)
                            && end_definitions == std::slice::from_ref(&mid_definition))
                        || (*from == mid
                            && *to == shared
                            && start_definitions == std::slice::from_ref(&mid_definition)
                            && end_definitions == std::slice::from_ref(&lifted_shared_definition))
                        || (*from == shared
                            && *to == end
                            && start_definitions
                                == std::slice::from_ref(&lifted_shared_definition)
                            && end_definitions == std::slice::from_ref(&end_definition)))
                },
                &mut |from, to| {
                    if *from == start && *to == end {
                        Ok(vec![DetourTarget {
                            point: shared.clone(),
                            definitions: vec![shared_definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
                    } else if *from == shared && *to == end {
                        Ok(vec![DetourTarget {
                            point: mid.clone(),
                            definitions: vec![mid_definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
                    } else if *from == mid && *to == end {
                        Ok(vec![DetourTarget {
                            point: shared.clone(),
                            definitions: vec![lifted_shared_definition.clone()],
                            uncertified_definition_fallback: false,
                        }])
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
                &initial_visited_definition_points(
                    &start,
                    &[axis_plane_definition(&start)],
                    &end,
                    &[axis_plane_definition(&end)],
                ),
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
                PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
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
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
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
                PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
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
                PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
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
    fn ordered_axis_orderings_by_no_step_precheck_prefers_more_direct_prefixes() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = axis_plane_definition(&p(1, 1, 1));
        let mut affine_cache = Vec::new();

        let ordered = ordered_axis_orderings_by_no_step_precheck_with(
            &start_definition,
            &end_definition,
            &mut affine_cache,
            |_current, _next, current_planes, next_planes| {
                let changed_axis = (0..3)
                    .find(|axis| current_planes[*axis] != next_planes[*axis])
                    .unwrap();
                match changed_axis {
                    2 => Ok(true),
                    1 => Err(HypermeshError::UnknownClassification),
                    0 => Ok(false),
                    _ => unreachable!(),
                }
            },
        )
        .unwrap();

        assert_eq!(ordered[0], [2, 1, 0]);
        assert_eq!(ordered[1], [2, 0, 1]);
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
            PlaneReplacementReachabilityStepMode::WithoutStepDetours,
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
            PlaneReplacementReachabilityStepMode::WithoutStepDetours,
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
    fn plane_replacement_reachability_step_cache_distinguishes_modes() {
        let current_planes = axis_plane_definition(&p(0, 0, 0));
        let next_planes = axis_plane_definition(&p(1, 0, 0));
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_plane_replacement_reachability_step_with(
            &mut cache,
            PlaneReplacementReachabilityStepMode::WithoutStepDetours,
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
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            &p(0, 0, 0),
            &p(1, 0, 0),
            &current_planes,
            &next_planes,
            || {
                calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert!(first);
        assert!(!second);
        assert_eq!(calls, 2);
    }

    #[test]
    fn plane_replacement_reachability_step_reuses_reversed_query() {
        let current_planes = axis_plane_definition(&p(0, 0, 0));
        let next_planes = axis_plane_definition(&p(1, 0, 0));
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_plane_replacement_reachability_step_with(
            &mut cache,
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
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
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            &p(1, 0, 0),
            &p(0, 0, 0),
            &next_planes,
            &current_planes,
            || {
                calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert!(first);
        assert!(second);
        assert_eq!(calls, 1);
    }

    #[test]
    fn plane_replacement_reachability_step_reuses_in_progress_exact_state() {
        let current_planes = axis_plane_definition(&p(0, 0, 0));
        let next_planes = axis_plane_definition(&p(1, 0, 0));
        let mut cache = vec![PlaneReplacementReachabilityStepCacheEntry {
            mode: PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            current_point: p(0, 0, 0),
            next_point: p(1, 0, 0),
            current_planes,
            next_planes,
            result: Err(HypermeshError::UnknownClassification),
        }];

        let result = cached_plane_replacement_reachability_step_with(
            &mut cache,
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            &p(0, 0, 0),
            &p(1, 0, 0),
            &axis_plane_definition(&p(0, 0, 0)),
            &axis_plane_definition(&p(1, 0, 0)),
            || Ok(true),
        );

        assert_eq!(result, Err(HypermeshError::UnknownClassification));
        assert_eq!(cache.len(), 1);
        assert_eq!(cache[0].result, Err(HypermeshError::UnknownClassification));
    }

    #[test]
    fn plane_replacement_reachability_path_reuses_permuted_plane_sets() {
        let start_planes = axis_plane_definition(&p(0, 0, 0));
        let end_planes = axis_plane_definition(&p(1, 0, 0));
        let permuted_start = [
            start_planes[1].clone(),
            start_planes[2].clone(),
            start_planes[0].clone(),
        ];
        let permuted_end = [
            end_planes[1].clone(),
            end_planes[2].clone(),
            end_planes[0].clone(),
        ];
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_plane_replacement_reachability_path_with(
            &mut cache,
            PlaneReplacementReachabilityStepMode::WithoutStepDetours,
            &start_planes,
            &end_planes,
            || {
                calls += 1;
                Ok(true)
            },
        )
        .unwrap();
        let second = cached_plane_replacement_reachability_path_with(
            &mut cache,
            PlaneReplacementReachabilityStepMode::WithoutStepDetours,
            &permuted_start,
            &permuted_end,
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
    fn plane_replacement_reachability_path_cache_distinguishes_modes() {
        let start_planes = axis_plane_definition(&p(0, 0, 0));
        let end_planes = axis_plane_definition(&p(1, 0, 0));
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_plane_replacement_reachability_path_with(
            &mut cache,
            PlaneReplacementReachabilityStepMode::WithoutStepDetours,
            &start_planes,
            &end_planes,
            || {
                calls += 1;
                Ok(true)
            },
        )
        .unwrap();
        let second = cached_plane_replacement_reachability_path_with(
            &mut cache,
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            &start_planes,
            &end_planes,
            || {
                calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert!(first);
        assert!(!second);
        assert_eq!(calls, 2);
    }

    #[test]
    fn plane_replacement_reachability_path_reuses_reversed_query() {
        let start_planes = axis_plane_definition(&p(0, 0, 0));
        let end_planes = axis_plane_definition(&p(1, 0, 0));
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_plane_replacement_reachability_path_with(
            &mut cache,
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            &start_planes,
            &end_planes,
            || {
                calls += 1;
                Ok(true)
            },
        )
        .unwrap();
        let second = cached_plane_replacement_reachability_path_with(
            &mut cache,
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            &end_planes,
            &start_planes,
            || {
                calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert!(first);
        assert!(second);
        assert_eq!(calls, 1);
    }

    #[test]
    fn plane_replacement_reachability_path_reuses_in_progress_exact_state() {
        let start_planes = axis_plane_definition(&p(0, 0, 0));
        let end_planes = axis_plane_definition(&p(1, 0, 0));
        let mut cache = vec![PlaneReplacementReachabilityPathCacheEntry {
            mode: PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            start_planes,
            end_planes,
            result: Err(HypermeshError::UnknownClassification),
        }];

        let result = cached_plane_replacement_reachability_path_with(
            &mut cache,
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            &axis_plane_definition(&p(0, 0, 0)),
            &axis_plane_definition(&p(1, 0, 0)),
            || Ok(true),
        );

        assert_eq!(result, Err(HypermeshError::UnknownClassification));
        assert_eq!(cache.len(), 1);
        assert_eq!(cache[0].result, Err(HypermeshError::UnknownClassification));
    }

    #[test]
    fn plane_replacement_no_nested_ordering_warmup_reuses_cached_local_warm_state() {
        let start_planes = axis_plane_definition(&p(0, 0, 0));
        let end_planes = axis_plane_definition(&p(1, 0, 0));
        let affine_entry = PlaneReplacementAffineCacheEntry {
            planes: start_planes.clone(),
            point: Ok(p(0, 0, 0)),
        };
        let path_entry = PlaneReplacementReachabilityPathCacheEntry {
            mode: PlaneReplacementReachabilityStepMode::WithoutStepDetours,
            start_planes: start_planes.clone(),
            end_planes: end_planes.clone(),
            result: Ok(true),
        };
        let step_entry = PlaneReplacementReachabilityStepCacheEntry {
            mode: PlaneReplacementReachabilityStepMode::WithoutStepDetours,
            current_point: p(0, 0, 0),
            next_point: p(1, 0, 0),
            current_planes: start_planes.clone(),
            next_planes: end_planes.clone(),
            result: Ok(true),
        };
        let mut cache = Vec::new();
        let mut first_affine = Vec::new();
        let mut first_path = Vec::new();
        let mut first_step = Vec::new();
        let mut calls = 0;

        let first = cached_plane_replacement_no_nested_ordering_warmup_with(
            &mut cache,
            &start_planes,
            &end_planes,
            &mut first_affine,
            &mut first_path,
            &mut first_step,
            |affine, path, step| {
                calls += 1;
                affine.push(affine_entry.clone());
                path.push(path_entry.clone());
                step.push(step_entry.clone());
                Ok(vec![[0, 1, 2]])
            },
        )
        .unwrap();

        let mut second_affine = Vec::new();
        let mut second_path = Vec::new();
        let mut second_step = Vec::new();
        let second = cached_plane_replacement_no_nested_ordering_warmup_with(
            &mut cache,
            &start_planes,
            &end_planes,
            &mut second_affine,
            &mut second_path,
            &mut second_step,
            |_, _, _| {
                calls += 1;
                Ok(vec![[2, 1, 0]])
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
        assert_eq!(first_affine, second_affine);
        assert_eq!(first_path, second_path);
        assert_eq!(first_step, second_step);
    }

    #[test]
    fn probe_hot_leaf_probe_family_breakdown() {
        use crate::mesh::prepare_input;
        use crate::polygon::ConvexPolygon;

        fn tetra_from_face_and_apex(
            a: Point3,
            b: Point3,
            c: Point3,
            apex: Point3,
        ) -> crate::InputMesh {
            crate::InputMesh::new(
                vec![a, b, c, apex],
                vec![
                    crate::Triangle::new(0, 2, 1),
                    crate::Triangle::new(0, 1, 3),
                    crate::Triangle::new(0, 3, 2),
                    crate::Triangle::new(1, 2, 3),
                ],
            )
        }

        fn face_at(
            polygons: &[ConvexPolygon],
            mesh_index: isize,
            polygon_index: isize,
        ) -> ConvexPolygon {
            polygons
                .iter()
                .find(|polygon| {
                    polygon.mesh_index == mesh_index && polygon.polygon_index == polygon_index
                })
                .unwrap()
                .clone()
        }

        let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
        let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
        let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
        let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
        let polygons = vec![
            face_at(&soup.polygons, 1, 4),
            face_at(&soup.polygons, 1, 5),
            face_at(&soup.polygons, 1, 7),
            face_at(&soup.polygons, 2, 8),
            face_at(&soup.polygons, 2, 11),
        ];
        let bounds = Aabb::new(p(1, 1, 1), p(9, 9, 9));
        let ref_point = p(0, 5, 5);
        let ref_definitions = vec![axis_plane_definition(&ref_point)];
        let ref_wnv = vec![0; soup.num_meshes];

        let host = &polygons[0];
        let intersections = polygons
            .iter()
            .enumerate()
            .filter_map(|(index, polygon)| {
                if index == 0 {
                    return None;
                }
                let intersection =
                    crate::intersection::intersect_polygons(host, polygon, index).ok()?;
                Some(intersection)
            })
            .collect::<Vec<_>>();
        let bsp_leaves =
            crate::subdivision::build_host_bsp_leaves(host, &polygons, &intersections).unwrap();
        let (leaf, interior_points, effective_delta_w) = bsp_leaves
            .iter()
            .filter_map(|leaf| {
                crate::subdivision::certify_bsp_leaf_and_delta_w(host, &leaf.edges, &polygons)
                    .ok()
                    .map(|(interior_points, effective_delta_w)| {
                        (leaf, interior_points, effective_delta_w)
                    })
            })
            .max_by_key(|(leaf, _, _)| leaf.edges.len())
            .unwrap();
        let interior = interior_points[0].clone();

        let normal_probes =
            adjacent_normal_probes(&interior, &host.support, &bounds, &polygons, true).unwrap();

        let mut axis_probe_counts = Vec::new();
        for axis in probe_axes(&host.support).unwrap() {
            let normal_sign =
                crate::geometry::classify_real(axis_ref(&host.support.normal, axis)).unwrap();
            if normal_sign == Classification::On {
                continue;
            }
            let direction_positive = normal_sign == Classification::Positive;
            let probes = adjacent_axis_probes(
                &interior,
                &host.support,
                &bounds,
                &polygons,
                axis,
                direction_positive,
            )
            .unwrap();
            axis_probe_counts.push((axis, probes.len()));
        }

        let winding = classify_leaf_polygon_from_interior_points(
            &interior_points,
            &host.support,
            &ref_point,
            &ref_definitions,
            &ref_wnv,
            &polygons,
            &bounds,
            &effective_delta_w,
        )
        .unwrap();
        assert_eq!(leaf.edges.len(), 4);
        assert!(!normal_probes.is_empty());
        assert!(!axis_probe_counts.is_empty());
        assert!(axis_probe_counts.iter().all(|(_, count)| *count > 0));
        assert_eq!(winding, vec![0, 0, 0]);
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
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
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
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
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
    fn no_step_ordering_precheck_warms_shared_affine_cache_for_step_trace() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = axis_plane_definition(&p(1, 2, 3));
        let mut affine_cache = Vec::new();
        let ordered = ordered_axis_orderings_by_no_step_precheck_with(
            &start_definition,
            &end_definition,
            &mut affine_cache,
            |_from, _to, _start_definitions, _end_definitions| Ok(true),
        )
        .unwrap();
        let affine_len_after_precheck = affine_cache.len();
        let mut step_cache = Vec::new();

        let reaches =
            plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
                &ordered,
                &start_definition,
                &end_definition,
                PlaneReplacementReachabilityStepMode::WithoutStepDetours,
                &mut affine_cache,
                &mut step_cache,
                |_from, _to, _start_definitions, _end_definitions| Ok(true),
            )
            .unwrap();

        assert!(reaches);
        assert_eq!(affine_cache.len(), affine_len_after_precheck);
    }

    #[test]
    fn plane_replacement_reachability_reports_unknown_for_same_point_uncertified_step() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(2, r(0)),
            Plane::from_coefficients(r(1), r(1), r(0), r(0)),
        ];
        let mut affine_cache = Vec::new();
        let mut step_cache = Vec::new();

        let err = plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
            &start_definition,
            &end_definition,
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            &mut affine_cache,
            &mut step_cache,
            |from, to, start_definitions, end_definitions| {
                if *from == p(0, 0, 0)
                    && *to == p(0, 0, 0)
                    && start_definitions == std::slice::from_ref(&start_definition)
                    && end_definitions == std::slice::from_ref(&end_definition)
                {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(true)
                }
            },
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
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
    fn plane_replacement_tracer_reports_unknown_for_same_point_uncertified_step() {
        let start_definition = axis_plane_definition(&p(0, 0, 0));
        let end_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(2, r(0)),
            Plane::from_coefficients(r(1), r(1), r(0), r(0)),
        ];
        let mut affine_cache = Vec::new();
        let mut step_cache = Vec::new();

        let err = trace_plane_replacement_path_with_tracer_and_caches(
            &start_definition,
            &end_definition,
            &[7],
            &[],
            &mut affine_cache,
            &mut step_cache,
            |current, next, current_planes, next_planes, _attempt, _polygons| {
                if *current == p(0, 0, 0)
                    && *next == p(0, 0, 0)
                    && current_planes == &start_definition
                    && next_planes == &end_definition
                {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(vec![7]))
                }
            },
        )
        .unwrap_err();

        assert_eq!(err, HypermeshError::UnknownClassification);
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
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
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
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
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
