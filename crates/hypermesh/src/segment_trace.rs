//! Exact segment tracing for winding-number propagation.

mod leaf_probe;
mod path;
mod probe_cache;
mod probe_reachability;

pub use leaf_probe::classify_leaf_polygon;
#[cfg(test)]
use leaf_probe::{
    cached_adjacent_axis_probe_stop_values_with, cached_adjacent_axis_probes_with,
    cached_adjacent_normal_probe_stop_values_with, cached_adjacent_normal_probes_with,
    cached_bounded_probes_from_interior_with, cached_probe_reachability_with,
    cached_probe_winding_with, search_leaf_probe_families, trace_probe_winding,
    trace_probe_winding_with_caches,
};
#[cfg(test)]
pub(crate) use leaf_probe::{
    classify_leaf_polygon_from_interior_points,
    classify_leaf_polygon_from_interior_points_with_probe_query_caches,
    ordered_interior_points_for_probe_search,
};
pub(crate) use leaf_probe::{
    classify_leaf_polygon_interior_point_with_probe_query_caches,
    classify_leaf_polygon_with_probe_query_caches,
    ordered_interior_points_for_probe_search_with_support,
};
#[cfg(test)]
use path::*;
use path::{
    AXIS_ORDERINGS, DetourArrangementCellState, InteriorBoxDetourTargetBatchCache,
    aabb_from_axis_intervals, adapt_plane_replacement_vertex_to_trace_bounds,
    apply_winding_transition_in_place, axis_plane_defined_point, cached_affine_from_planes_with,
    cached_detour_target_family, cached_detour_target_family_with,
    cached_interior_box_axis_intervals_with_surface_queries, cached_strict_aabb_target_families,
    definition_families_match_as_sets, definition_planes_match_as_sets, detour_arrangement_cell,
    detour_arrangement_cell_state_is_dominated, detour_arrangement_planes,
    detour_target_family_result_from_targets, endpoint_definition_family,
    evaluate_strict_aabb_target_families_with_direct_ranking, extend_unique_definition_families,
    finalize_interior_point_family, finalize_probe_point_family,
    finalize_shifted_halfspace_witness_family, first_changed_axis,
    initial_visited_definition_points, interior_box_axis_intervals_with_surface_queries,
    interior_box_detour_targets, normalized_cycle_guard_visited_points, other_axes,
    point_is_inside_optional_trace_bounds, point_lies_on_traced_surface,
    push_detour_target_family_bucket_entry, push_strict_aabb_target_family_bucket_entry,
    push_unique_detour_target, record_detour_arrangement_cell_state,
    search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking_outcome,
    trace_bounds_including_point,
    trace_segment_from_definitions_with_step_detoured_plane_replacement_with_caches,
    unique_definition_family, visited_definition_family_contains,
    visited_definition_points_match_as_sets, visited_definition_points_subset_of,
};
pub(crate) use path::{
    affine_from_planes, axis_plane_definition,
    trace_segment_from_definitions_with_step_detoured_plane_replacement_in_bounds,
};
pub use path::{trace_axis_segment, trace_segment};
#[cfg(test)]
pub(crate) use path::{
    trace_plane_replacement_path,
    trace_segment_from_definitions_with_step_detoured_plane_replacement,
    trace_segment_without_detours,
};
use probe_cache::{
    DirectProbeReachabilityCacheEntry, HalfspaceReportCacheEntry, HalfspaceSeedFamilyCacheEntry,
    SurfaceCacheEntry, cached_halfspace_cell_seed_families_from_optional_report_with,
    cached_optional_halfspace_feasibility_report_with,
};
#[cfg(test)]
use probe_cache::{cached_direct_probe_reachability_with, cached_surface_query_with};
#[cfg(test)]
use probe_reachability::*;
use probe_reachability::{
    cached_definition_no_detour_reachability_with,
    plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches,
    probe_reaches_adjacent_cell, probe_reaches_adjacent_cell_from_interior_with_caches,
    probe_reaches_adjacent_cell_with_definition_search,
};

use hyperlattice::{HomogeneousPoint3, Point3, Real, intersect_three_planes};
use hyperlimit::{
    HalfspaceFeasibility, Plane3 as LimitPlane3, PredicateOutcome, classify_halfspace_feasibility3,
};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{
    Aabb, Classification, Plane, axis_mut, axis_ref, classify_point, classify_real, compare_real,
};
use crate::halfspace::{
    aabb_core_halfspaces, halfspace_has_opposite_pair, halfspace_is_degenerate_bound,
    limit_plane_families_match_as_sets, negated_halfspace, point_satisfies_halfspaces,
    support_side_halfspace,
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
    boundary_edge_count: usize,
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
struct ProbeReachabilityCacheEntry {
    interior_point: Point3,
    interior_planes: Vec<[Plane; 3]>,
    probe_point: Point3,
    probe_planes: Vec<[Plane; 3]>,
    reachable: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
struct InteriorBoxAxisIntervalsCacheEntry {
    start: Point3,
    end: Point3,
    intervals: Vec<Vec<(Real, Real)>>,
    saw_unknown: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct InteriorBoxAxisIntervalsBucket {
    start: Point3,
    end: Point3,
    indices: Vec<usize>,
}

#[derive(Default)]
struct InteriorBoxAxisIntervalsCache {
    entries: Vec<InteriorBoxAxisIntervalsCacheEntry>,
    buckets: Vec<InteriorBoxAxisIntervalsBucket>,
}

impl InteriorBoxAxisIntervalsCache {
    #[cfg(test)]
    fn len(&self) -> usize {
        self.entries.len()
    }
}

const DIRECT_TARGET_RANK_REFINEMENT_LIMIT: usize = 4;

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
    trace_bounds: Option<Aabb>,
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
    interior_box_axis_intervals: InteriorBoxAxisIntervalsCache,
    axis_ordered_segment_traces: Vec<AxisOrderedSegmentTraceCacheEntry>,
    plane_replacement_affine: PlaneReplacementAffineCache,
    plane_replacement_trace_steps: Vec<PlaneReplacementStepCacheEntry>,
    plane_replacement_reachability_paths: PlaneReplacementReachabilityPathCache,
    plane_replacement_reachability_steps: PlaneReplacementReachabilityStepCache,
    plane_replacement_no_nested_ordering_warmups: PlaneReplacementNoNestedOrderingWarmupCache,
    definition_cycle_guard_reachability: DefinitionCycleGuardReachabilityCache,
    definition_no_step_detour_reachability: DefinitionNoDetourReachabilityCache,
    definition_no_plane_replacement_cycle_guard: DefinitionNoPlaneReplacementCycleGuardCache,
    definition_no_plane_replacement_reachability: DefinitionNoPlaneReplacementReachabilityCache,
    no_step_detour_target_families: DetourTargetFamilyCache,
    definition_full_no_detour_reachability: DefinitionNoDetourReachabilityCache,
    definition_no_detour_trace: Vec<DefinitionNoDetourTraceCacheEntry>,
    definition_no_detour_reachability: DefinitionNoDetourReachabilityCache,
    detour_target_families: DetourTargetFamilyCache,
}

impl LeafProbeQueryCaches {
    fn prepare_for_trace_bounds(&mut self, bounds: &Aabb) {
        if self
            .trace_bounds
            .as_ref()
            .is_some_and(|existing| existing != bounds)
        {
            *self = Self::default();
        }
        self.trace_bounds = Some(bounds.clone());
    }
}

#[derive(Clone, Debug, PartialEq)]
struct StrictAabbTargetFamilies {
    direct_targets: Vec<DetourTarget>,
    shifted_targets: Vec<DetourTarget>,
    saw_unknown: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct StrictAabbTargetFamilyCacheEntry {
    bounds: Aabb,
    families: HypermeshResult<StrictAabbTargetFamilies>,
}

#[derive(Clone, Debug, PartialEq)]
struct StrictAabbTargetFamilyBucket {
    bounds: Aabb,
    indices: Vec<usize>,
}

#[derive(Default)]
struct StrictAabbTargetFamilyCache {
    entries: Vec<StrictAabbTargetFamilyCacheEntry>,
    buckets: Vec<StrictAabbTargetFamilyBucket>,
}

impl StrictAabbTargetFamilyCache {
    #[cfg(test)]
    fn len(&self) -> usize {
        self.entries.len()
    }
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
struct PlaneReplacementAffineBucket {
    planes: [Plane; 3],
    indices: Vec<usize>,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct PlaneReplacementAffineCache {
    entries: Vec<PlaneReplacementAffineCacheEntry>,
    buckets: Vec<PlaneReplacementAffineBucket>,
}

impl PlaneReplacementAffineCache {
    #[cfg(test)]
    fn len(&self) -> usize {
        self.entries.len()
    }
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
struct PlaneReplacementReachabilityStepBucket {
    mode: PlaneReplacementReachabilityStepMode,
    current_point: Point3,
    next_point: Point3,
    current_planes: [Plane; 3],
    next_planes: [Plane; 3],
    indices: Vec<usize>,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct PlaneReplacementReachabilityStepCache {
    entries: Vec<PlaneReplacementReachabilityStepCacheEntry>,
    buckets: Vec<PlaneReplacementReachabilityStepBucket>,
}

impl PlaneReplacementReachabilityStepCache {
    #[cfg(test)]
    fn len(&self) -> usize {
        self.entries.len()
    }
}

#[derive(Clone, Debug, PartialEq)]
struct PlaneReplacementReachabilityPathCacheEntry {
    mode: PlaneReplacementReachabilityStepMode,
    start_planes: [Plane; 3],
    end_planes: [Plane; 3],
    result: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
struct PlaneReplacementReachabilityPathBucket {
    mode: PlaneReplacementReachabilityStepMode,
    start_planes: [Plane; 3],
    end_planes: [Plane; 3],
    indices: Vec<usize>,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct PlaneReplacementReachabilityPathCache {
    entries: Vec<PlaneReplacementReachabilityPathCacheEntry>,
    buckets: Vec<PlaneReplacementReachabilityPathBucket>,
}

impl PlaneReplacementReachabilityPathCache {
    #[cfg(test)]
    fn len(&self) -> usize {
        self.entries.len()
    }
}

#[derive(Clone, Debug, PartialEq)]
struct PlaneReplacementNoNestedOrderingWarmupCacheEntry {
    start_planes: [Plane; 3],
    end_planes: [Plane; 3],
    ordered: HypermeshResult<Vec<[usize; 3]>>,
    affine_cache: PlaneReplacementAffineCache,
    path_cache: PlaneReplacementReachabilityPathCache,
    step_cache: PlaneReplacementReachabilityStepCache,
}

#[derive(Clone, Debug, PartialEq)]
struct PlaneReplacementNoNestedOrderingWarmupBucket {
    start_planes: [Plane; 3],
    end_planes: [Plane; 3],
    indices: Vec<usize>,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct PlaneReplacementNoNestedOrderingWarmupCache {
    entries: Vec<PlaneReplacementNoNestedOrderingWarmupCacheEntry>,
    buckets: Vec<PlaneReplacementNoNestedOrderingWarmupBucket>,
}

impl PlaneReplacementNoNestedOrderingWarmupCache {
    #[cfg(test)]
    fn len(&self) -> usize {
        self.entries.len()
    }
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

#[derive(Default)]
struct DefinitionNoDetourReachabilityCache {
    entries: Vec<DefinitionNoDetourReachabilityCacheEntry>,
    buckets: Vec<DefinitionReachabilityBucket>,
}

impl DefinitionNoDetourReachabilityCache {
    #[cfg(test)]
    fn len(&self) -> usize {
        self.entries.len()
    }
}

#[cfg(test)]
impl From<Vec<DefinitionNoDetourReachabilityCacheEntry>> for DefinitionNoDetourReachabilityCache {
    fn from(entries: Vec<DefinitionNoDetourReachabilityCacheEntry>) -> Self {
        let mut cache = Self::default();
        for entry in entries {
            let index = cache.entries.len();
            push_definition_reachability_bucket_entry(
                &mut cache.buckets,
                &entry.start,
                &entry.end,
                &entry.start_definitions,
                &entry.end_definitions,
                index,
            );
            cache.entries.push(entry);
        }
        cache
    }
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
struct DefinitionReachabilityBucket {
    start: Point3,
    end: Point3,
    start_definitions: Vec<[Plane; 3]>,
    end_definitions: Vec<[Plane; 3]>,
    entry_indices: Vec<usize>,
}

#[derive(Default)]
struct DefinitionCycleGuardReachabilityCache {
    entries: Vec<DefinitionCycleGuardReachabilityCacheEntry>,
    buckets: Vec<DefinitionReachabilityBucket>,
}

#[cfg(test)]
impl From<Vec<DefinitionCycleGuardReachabilityCacheEntry>>
    for DefinitionCycleGuardReachabilityCache
{
    fn from(entries: Vec<DefinitionCycleGuardReachabilityCacheEntry>) -> Self {
        let mut cache = Self::default();
        for entry in entries {
            let index = cache.entries.len();
            push_definition_reachability_bucket_entry(
                &mut cache.buckets,
                &entry.start,
                &entry.end,
                &entry.start_definitions,
                &entry.end_definitions,
                index,
            );
            cache.entries.push(entry);
        }
        cache
    }
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

#[derive(Default)]
struct DefinitionNoPlaneReplacementCycleGuardCache {
    entries: Vec<DefinitionNoPlaneReplacementCycleGuardCacheEntry>,
    buckets: Vec<DefinitionReachabilityBucket>,
}

impl DefinitionNoPlaneReplacementCycleGuardCache {
    #[cfg(test)]
    fn len(&self) -> usize {
        self.entries.len()
    }
}

#[cfg(test)]
impl From<Vec<DefinitionNoPlaneReplacementCycleGuardCacheEntry>>
    for DefinitionNoPlaneReplacementCycleGuardCache
{
    fn from(entries: Vec<DefinitionNoPlaneReplacementCycleGuardCacheEntry>) -> Self {
        let mut cache = Self::default();
        for entry in entries {
            let index = cache.entries.len();
            push_definition_reachability_bucket_entry(
                &mut cache.buckets,
                &entry.start,
                &entry.end,
                &entry.start_definitions,
                &entry.end_definitions,
                index,
            );
            cache.entries.push(entry);
        }
        cache
    }
}

#[derive(Clone, Debug, PartialEq)]
struct DefinitionNoPlaneReplacementReachabilityCacheEntry {
    start: Point3,
    end: Point3,
    start_definitions: Vec<[Plane; 3]>,
    end_definitions: Vec<[Plane; 3]>,
    result: HypermeshResult<bool>,
}

#[derive(Default)]
struct DefinitionNoPlaneReplacementReachabilityCache {
    entries: Vec<DefinitionNoPlaneReplacementReachabilityCacheEntry>,
    buckets: Vec<DefinitionReachabilityBucket>,
}

impl DefinitionNoPlaneReplacementReachabilityCache {
    #[cfg(test)]
    fn len(&self) -> usize {
        self.entries.len()
    }
}

#[cfg(test)]
impl From<Vec<DefinitionNoPlaneReplacementReachabilityCacheEntry>>
    for DefinitionNoPlaneReplacementReachabilityCache
{
    fn from(entries: Vec<DefinitionNoPlaneReplacementReachabilityCacheEntry>) -> Self {
        let mut cache = Self::default();
        for entry in entries {
            let index = cache.entries.len();
            push_definition_reachability_bucket_entry(
                &mut cache.buckets,
                &entry.start,
                &entry.end,
                &entry.start_definitions,
                &entry.end_definitions,
                index,
            );
            cache.entries.push(entry);
        }
        cache
    }
}

#[derive(Clone, Debug, PartialEq)]
struct DetourTargetFamilyCacheEntry {
    start: Point3,
    end: Point3,
    trace_bounds: Option<Aabb>,
    targets: HypermeshResult<Vec<DetourTarget>>,
}

#[derive(Clone, Debug, PartialEq)]
struct DetourTargetFamilyBucket {
    start: Point3,
    end: Point3,
    indices: Vec<usize>,
}

#[derive(Default)]
struct DetourTargetFamilyCache {
    entries: Vec<DetourTargetFamilyCacheEntry>,
    buckets: Vec<DetourTargetFamilyBucket>,
}

impl DetourTargetFamilyCache {
    #[cfg(test)]
    fn len(&self) -> usize {
        self.entries.len()
    }
}

#[derive(Clone, Debug, PartialEq)]
struct DetourTarget {
    point: Point3,
    definitions: Vec<[Plane; 3]>,
    uncertified_definition_fallback: bool,
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
        push_unique_interior_point(&mut points, point);
    }

    finalize_interior_point_family(&mut points, saw_unknown)?;
    Ok(points)
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
    finalize_interior_point_family(points, saw_hard_unknown)
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
    finalize_interior_point_family(points, saw_hard_unknown)
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
        match build_strict_leaf_point_from_shifted_witness(leaf, shifted) {
            Ok(Some(point)) => {
                push_unique_interior_point(&mut points, point);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_interior_point_family(&mut points, saw_unknown)?;
    Ok(points)
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
        match build_strict_leaf_point_from_shifted_witness(leaf, shifted) {
            Ok(Some(point)) => {
                push_unique_interior_point(&mut points, point);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_interior_point_family(&mut points, saw_unknown)?;
    Ok(points)
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

    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
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

    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
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
    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
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
        match build_probe_point_from_shifted_witness(shifted, corridor, support, &extra_planes) {
            Ok(Some(probe)) => {
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
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
        match build_probe_point_from_shifted_witness(shifted, corridor, support, &extra_planes) {
            Ok(Some(probe)) => {
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                local_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_probe_point_family(&mut probes, local_unknown)?;
    Ok(probes)
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
        match build_probe_point_from_shifted_witness(shifted, corridor, support, &extra_planes) {
            Ok(Some(probe)) => {
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
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

    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
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
    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
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
    finalize_probe_point_family(probes, saw_hard_unknown)
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
        match build_axis_probe_point_from_shifted_witness(
            shifted, interior, corridor, support, axis, definition,
        ) {
            Ok(Some(probe)) => {
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
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
        match build_axis_probe_point_from_shifted_witness(
            shifted, interior, corridor, support, axis, definition,
        ) {
            Ok(Some(probe)) => {
                push_unique_probe_point(&mut probes, probe);
            }
            Ok(None) => {}
            Err(HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    finalize_probe_point_family(&mut probes, saw_unknown)?;
    Ok(probes)
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

fn plane_halfspace(plane: &Plane) -> LimitPlane3 {
    LimitPlane3::new(plane.normal.clone(), plane.offset.clone())
}

fn push_plane_equality_halfspaces(halfspaces: &mut Vec<LimitPlane3>, plane: &Plane) {
    let halfspace = plane_halfspace(plane);
    halfspaces.push(halfspace.clone());
    halfspaces.push(negated_halfspace(&halfspace));
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

    finalize_shifted_halfspace_witness_family(&mut witnesses, saw_unknown)?;
    Ok(witnesses)
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
    finalize_shifted_halfspace_witness_family(witnesses, saw_hard_unknown)
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
    finalize_shifted_halfspace_witness_family(witnesses, saw_hard_unknown)
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
mod tests;
