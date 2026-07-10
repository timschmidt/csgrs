//! Exact segment tracing for winding-number propagation.

mod leaf_probe;
mod path;
mod probe_cache;
mod probe_reachability;
mod witness;

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
    evaluate_strict_aabb_target_families_with_direct_ranking, first_changed_axis,
    initial_visited_definition_points, interior_box_axis_intervals_with_surface_queries,
    interior_box_detour_targets, normalized_cycle_guard_visited_points,
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
    SurfaceCacheEntry,
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
#[cfg(test)]
pub(crate) use witness::certified_leaf_test_point;
#[cfg(test)]
use witness::*;
use witness::{
    PolygonPointLocation, ShiftedHalfspaceWitness, active_planes_from_optional_report,
    adjacent_axis_probe_stop_values_with_queries, adjacent_normal_probe_stop_values_with_queries,
    axis_probe_bounds, axis_probe_definition_preserves_axis_direction, axis_value_after_start,
    bounds_between_points, build_axis_probe_point, build_axis_probe_point_from_shifted_witness,
    build_probe_point, build_probe_point_from_shifted_witness, classify_point_in_polygon,
    collect_strict_halfspace_seed_family, dedupe_shifted_halfspace_seed_families,
    dominant_normal_axis, dot_direction,
    extend_shifted_halfspace_seed_families_backtracking_unknown,
    extend_strict_halfspace_seed_families_collect_unknown,
    halfspace_cell_seed_families_from_optional_report, interior_leaf_points,
    normal_probe_extra_planes, normal_probe_shifted_seed_families, normal_stop_halfspace,
    offset_point, optional_halfspace_feasibility_report, planes_are_coplanar,
    point_strictly_between_axis, point_strictly_inside_halfspace_cell_or_unknown, probe_axes,
    probe_definitions_from_active_halfspaces, probe_definitions_or_axis,
    push_plane_equality_halfspaces, seed_family_search_failed_without_any_seed,
    segment_plane_crossing, shifted_halfspace_cell_witnesses_from_seed,
    shifted_halfspace_seed_families_with_report_seed, shifted_halfspace_witness_family_or_empty,
    sort_crossing_events, strict_axis_probe_targets, strict_normal_probe_targets_with_query_caches,
    take_new_halfspace_seed_family, unique_normal_probe_search_definitions,
};
pub(crate) use witness::{certified_leaf_interior_points, certified_leaf_test_points};

use hyperlattice::{Point3, Real};

use crate::error::HypermeshResult;
use crate::geometry::{Aabb, Classification, Plane};
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

#[cfg(test)]
mod tests;
