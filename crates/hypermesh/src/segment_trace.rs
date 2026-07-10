//! Exact segment tracing for winding-number propagation.

use hyperlattice::{HomogeneousPoint3, Point3, Real, intersect_three_planes};
use hyperlimit::{
    HalfspaceFeasibility, Plane3 as LimitPlane3, PredicateOutcome, Sign,
    classify_halfspace_feasibility3, classify_plane_aabb3_report,
};

use crate::clip::clip_polygon_to_aabb;
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

fn detour_arrangement_planes(polygons: &[ConvexPolygon]) -> Vec<Plane> {
    let mut planes = Vec::new();
    for polygon in polygons {
        if !planes.iter().any(|existing| existing == &polygon.support) {
            planes.push(polygon.support.clone());
        }
    }
    planes
}

fn detour_arrangement_cell(
    point: &Point3,
    arrangement_planes: &[Plane],
) -> HypermeshResult<Vec<Classification>> {
    arrangement_planes
        .iter()
        .map(|plane| classify_point(point, plane))
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

fn strict_aabb_arrangement_cell(
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
struct DetourArrangementCellState {
    cell: Vec<Classification>,
    uncertified_definition_fallback: bool,
}

fn detour_arrangement_cell_state_is_dominated(
    seen: &[DetourArrangementCellState],
    cell: &[Classification],
    uncertified_definition_fallback: bool,
) -> bool {
    seen.iter().any(|existing| {
        existing.cell == cell
            && (!existing.uncertified_definition_fallback || uncertified_definition_fallback)
    })
}

fn record_detour_arrangement_cell_state(
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

fn finalize_interior_point_family(
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

fn finalize_probe_point_family(
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

fn finalize_shifted_halfspace_witness_family(
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

fn trace_segment_from_definitions_with_caches(
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

fn point_is_inside_optional_trace_bounds(
    point: &Point3,
    trace_bounds: Option<&Aabb>,
) -> HypermeshResult<bool> {
    trace_bounds.map_or(Ok(true), |bounds| bounds.contains_point(point))
}

fn trace_bounds_including_point(bounds: &Aabb, point: &Point3) -> HypermeshResult<Aabb> {
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

fn adapt_plane_replacement_vertex_to_trace_bounds(
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

fn points_share_open_arrangement_cell(
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

#[cfg(test)]
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

#[cfg(test)]
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
        return Ok(Some(second_leg));
    }

    if saw_unknown {
        Err(HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

fn trace_segment_with_detour_batches_breadth_first_with_surface_query(
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
            if seen_paths.iter().any(|seen| *seen == next_path) {
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

fn push_detour_target_family_bucket_entry(
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

fn cached_detour_target_family_with(
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

fn cached_detour_target_family<'a>(
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
fn trace_plane_replacement_path_without_detours_with_caches(
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

fn trace_plane_replacement_path_with_tracer_and_caches(
    start_planes: &[Plane; 3],
    end_planes: &[Plane; 3],
    winding: &[i32],
    polygons: &[ConvexPolygon],
    mut affine_cache: &mut PlaneReplacementAffineCache,
    mut step_cache: &mut Vec<PlaneReplacementStepCacheEntry>,
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
            match cached_affine_from_planes_with(&mut affine_cache, &current_planes, || {
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
                match cached_affine_from_planes_with(&mut affine_cache, &next_planes, || {
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
                &mut step_cache,
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

fn cached_affine_from_planes_with(
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
    let targets = collect_detour_targets_from_axis_intervals(&intervals, |bounds| build(bounds))?;
    detour_target_family_result_from_targets(targets, saw_unknown)
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

fn cached_interior_box_axis_intervals_with_surface_queries(
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

struct ProgressiveStrictAabbSearchOutcome {
    result: HypermeshResult<bool>,
    exhausted_families: Option<StrictAabbTargetFamilies>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StrictAabbTargetCursorStage {
    FrontDirect,
    Shifted,
    DeferredDirect,
    Done,
}

struct StrictAabbTargetCursor {
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
    saw_unknown: bool,
    stage: StrictAabbTargetCursorStage,
}

impl StrictAabbTargetCursor {
    fn new(bounds: &Aabb) -> HypermeshResult<Self> {
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

    fn next_batch(&mut self) -> HypermeshResult<Option<Vec<DetourTarget>>> {
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
                && !self
                    .certified_direct_target_points
                    .iter()
                    .any(|existing| *existing == target.point)
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

struct InteriorBoxDetourTargetCursor {
    bounds: Vec<Aabb>,
    next_bounds: usize,
    current: Option<StrictAabbTargetCursor>,
    emitted_targets: Vec<DetourTarget>,
    saw_unknown: bool,
}

impl InteriorBoxDetourTargetCursor {
    fn new(
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
                if !bounds.iter().any(|existing| *existing == candidate) {
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

    fn next_batch(&mut self) -> HypermeshResult<Option<Vec<DetourTarget>>> {
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

struct InteriorBoxDetourTargetBatchCacheEntry {
    start: Point3,
    end: Point3,
    trace_bounds: Option<Aabb>,
    cursor: InteriorBoxDetourTargetCursor,
    batches: Vec<Vec<DetourTarget>>,
    exhausted: bool,
}

#[derive(Default)]
struct InteriorBoxDetourTargetBatchCache {
    entries: Vec<InteriorBoxDetourTargetBatchCacheEntry>,
}

impl InteriorBoxDetourTargetBatchCache {
    fn batch_for(
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
    search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking_outcome(
        bounds,
        &mut seed_families_for,
        rank_direct,
        evaluate,
    )
    .result
}

fn search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking_outcome<
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
    for (_index, seed) in seeds.iter().take(refinement_len).enumerate() {
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
        if !target.uncertified_definition_fallback {
            if !certified_direct_target_points
                .iter()
                .any(|existing| *existing == target.point)
            {
                certified_direct_target_points.push(target.point.clone());
            }
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

fn evaluate_strict_aabb_target_families_with_direct_ranking<K: Ord>(
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
fn cached_strict_aabb_target_families_with_seed_families(
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

fn push_strict_aabb_target_family_bucket_entry(
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

fn cached_strict_aabb_target_families(
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

fn detour_shifted_seed_families(
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
fn strict_aabb_targets_with_seed_families(
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

fn detour_target_family_result_from_targets(
    mut targets: Vec<DetourTarget>,
    saw_unknown: bool,
) -> HypermeshResult<Vec<DetourTarget>> {
    finalize_detour_target_family(&mut targets, saw_unknown)?;
    Ok(targets)
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

#[cfg(test)]
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
    finalize_detour_target_family(targets, saw_hard_unknown)
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
    let leaf = ConvexPolygon {
        support: support.clone(),
        edges: leaf_edges.to_vec(),
        mesh_index: -1,
        polygon_index: -1,
        delta_w: WindingNumberTransitionVector::new(),
        approx_bounds: None,
    };
    let clipped_leaf = clip_polygon_to_aabb(&leaf, bounds)?;
    let interior_points = interior_leaf_points(&clipped_leaf)?;
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
    probe_query_caches.prepare_for_trace_bounds(bounds);
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
    probe_query_caches.prepare_for_trace_bounds(bounds);
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
        let half =
            (Real::one() / Real::from(2)).map_err(|_| HypermeshError::UnknownClassification)?;
        let direct_point = offset_point(&point.point, &direction, &(stop_t.clone() * half));
        let direct_side = classify_point(&direct_point, support)?;
        if direct_side != Classification::On {
            let direct_probe = ProbePoint {
                planes: vec![axis_plane_definition(&direct_point)],
                point: direct_point,
                side: direct_side,
                uncertified_definition_fallback: false,
            };
            if let Some(winding) = try_leaf_probe_family_with_queries(
                point,
                positive_side,
                Ok(vec![direct_probe]),
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
            trace_bounds,
            probe_winding,
            probe_surface,
            probe_reachability,
            axis_ordered_segment_traces,
            plane_replacement_affine,
            plane_replacement_trace_steps,
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
            definition_no_detour_trace,
            definition_no_detour_reachability,
            direct_probe_reachability,
            detour_target_families,
            ..
        } = probe_query_caches;
        let trace_bounds = trace_bounds
            .as_ref()
            .ok_or(HypermeshError::UnknownClassification)?;

        if cached_surface_query_with(probe_surface, &probe.point, || {
            point_lies_on_traced_surface(&probe.point, polygons)
        })? {
            if point.uncertified_definition_fallback || probe_fallback {
                *local_unknown = true;
            }
            return Ok(None);
        }

        let no_step_result =
            probe_reaches_adjacent_cell_from_interior_without_step_detours_with_caches(
                point,
                &probe,
                support,
                polygons,
                plane_replacement_affine,
                plane_replacement_reachability_paths,
                plane_replacement_reachability_steps,
                definition_no_step_detour_reachability,
                direct_probe_reachability,
                trace_bounds,
            );
        match no_step_result {
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
                        interior_box_axis_intervals,
                        definition_cycle_guard_reachability,
                        definition_no_step_detour_reachability,
                        definition_no_plane_replacement_cycle_guard,
                        definition_no_plane_replacement_reachability,
                        halfspace_reports,
                        halfspace_seed_families,
                        no_step_detour_target_families,
                        definition_full_no_detour_reachability,
                        definition_no_detour_trace,
                        definition_no_detour_reachability,
                        direct_probe_reachability,
                        detour_target_families,
                        trace_bounds,
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
                        interior_box_axis_intervals,
                        definition_cycle_guard_reachability,
                        definition_no_step_detour_reachability,
                        definition_no_plane_replacement_cycle_guard,
                        definition_no_plane_replacement_reachability,
                        halfspace_reports,
                        halfspace_seed_families,
                        no_step_detour_target_families,
                        definition_full_no_detour_reachability,
                        definition_no_detour_trace,
                        definition_no_detour_reachability,
                        direct_probe_reachability,
                        detour_target_families,
                        trace_bounds,
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
        trace_bounds,
        probe_winding,
        probe_surface,
        probe_reachability,
        axis_ordered_segment_traces,
        plane_replacement_affine,
        plane_replacement_trace_steps,
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
        definition_no_detour_trace,
        definition_no_detour_reachability,
        direct_probe_reachability,
        detour_target_families,
        ..
    } = probe_query_caches;
    let trace_bounds = trace_bounds
        .as_ref()
        .ok_or(HypermeshError::UnknownClassification)?;
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
            trace_bounds,
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
                        interior_box_axis_intervals,
                        definition_cycle_guard_reachability,
                        definition_no_step_detour_reachability,
                        definition_no_plane_replacement_cycle_guard,
                        definition_no_plane_replacement_reachability,
                        halfspace_reports,
                        halfspace_seed_families,
                        no_step_detour_target_families,
                        definition_full_no_detour_reachability,
                        definition_no_detour_trace,
                        definition_no_detour_reachability,
                        direct_probe_reachability,
                        detour_target_families,
                        trace_bounds,
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
            interior_box_axis_intervals,
            definition_cycle_guard_reachability,
            definition_no_step_detour_reachability,
            definition_no_plane_replacement_cycle_guard,
            definition_no_plane_replacement_reachability,
            halfspace_reports,
            halfspace_seed_families,
            no_step_detour_target_families,
            definition_full_no_detour_reachability,
            definition_no_detour_trace,
            definition_no_detour_reachability,
            direct_probe_reachability,
            detour_target_families,
            trace_bounds,
            saw_unknown,
        )? {
            return Ok(Some(winding));
        }
    }

    Ok(None)
}

fn evaluate_leaf_probe_with_query_caches(
    point: &InteriorLeafPoint,
    _positive_side: bool,
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
    plane_replacement_affine: &mut PlaneReplacementAffineCache,
    plane_replacement_trace_steps: &mut Vec<PlaneReplacementStepCacheEntry>,
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
    definition_no_detour_trace: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    definition_no_detour_reachability: &mut DefinitionNoDetourReachabilityCache,
    direct_probe_reachability: &mut Vec<DirectProbeReachabilityCacheEntry>,
    detour_target_families: &mut DetourTargetFamilyCache,
    trace_bounds: &Aabb,
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
                Some(trace_bounds),
            )
        })?;
        if !reaches {
            None
        } else {
            let winding_trace_bounds = trace_bounds_including_point(trace_bounds, ref_point)?;
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
                    Some(&winding_trace_bounds),
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
            // Strict leaf membership, adjacent-cell reachability, and the
            // reference-to-probe winding trace have all certified this pair.
            Ok(Some(winding))
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
    plane_replacement_affine: &mut PlaneReplacementAffineCache,
    plane_replacement_reachability_paths: &mut PlaneReplacementReachabilityPathCache,
    plane_replacement_reachability_steps: &mut PlaneReplacementReachabilityStepCache,
    no_step_cache: &mut DefinitionNoDetourReachabilityCache,
    direct_probe_reachability: &mut Vec<DirectProbeReachabilityCacheEntry>,
    trace_bounds: &Aabb,
) -> HypermeshResult<bool> {
    if !trace_bounds.contains_point(&interior.point)?
        || !trace_bounds.contains_point(&probe.point)?
    {
        return Err(HypermeshError::UnknownClassification);
    }
    let start_family = endpoint_definition_family(&interior.point, &interior.planes)?;
    let end_family = endpoint_definition_family(&probe.point, &probe.planes)?;
    let saw_unknown = start_family.saw_unknown || end_family.saw_unknown;

    let result = cached_definition_no_detour_reachability_with(
        no_step_cache,
        &interior.point,
        &probe.point,
        &start_family.definitions,
        &end_family.definitions,
        || {
            probe_reaches_adjacent_cell_with_definition_search(
                &interior.point,
                &probe.point,
                &start_family.definitions,
                &end_family.definitions,
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
                        Some(trace_bounds),
                    )
                },
            )
        },
    );
    match result {
        Ok(false) if saw_unknown => Err(HypermeshError::UnknownClassification),
        result => result,
    }
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
    let mut plane_replacement_affine = PlaneReplacementAffineCache::default();
    let mut plane_replacement_trace_steps = Vec::new();
    let mut definition_no_detour_trace = Vec::new();
    let mut detour_target_families = DetourTargetFamilyCache::default();
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
        None,
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
    plane_replacement_affine: &mut PlaneReplacementAffineCache,
    plane_replacement_trace_steps: &mut Vec<PlaneReplacementStepCacheEntry>,
    definition_no_detour_trace: &mut Vec<DefinitionNoDetourTraceCacheEntry>,
    detour_target_families: &mut DetourTargetFamilyCache,
    trace_bounds: Option<&Aabb>,
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
        trace_bounds,
    )
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

fn trace_segment_from_definitions_with_step_detoured_plane_replacement_with_caches(
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
fn trace_probe_from_reference_definitions(
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

fn trace_from_definition_sets_with_step_detoured_plane_replacement(
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

fn trace_plane_replacement_path_with_step_detours_impl(
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

struct EndpointDefinitionFamilyState {
    definitions: Vec<[Plane; 3]>,
    saw_unknown: bool,
}

fn endpoint_definition_family(
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

fn definition_reachability_bucket_matches(
    bucket: &DefinitionReachabilityBucket,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> bool {
    bucket.start == *start
        && bucket.end == *end
        && definition_families_match_as_sets(&bucket.start_definitions, start_definitions)
        && definition_families_match_as_sets(&bucket.end_definitions, end_definitions)
}

fn matching_definition_reachability_bucket_entry_indices<'a>(
    buckets: &'a [DefinitionReachabilityBucket],
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> (Option<&'a [usize]>, Option<&'a [usize]>) {
    let same_direction = buckets
        .iter()
        .rev()
        .find(|bucket| {
            definition_reachability_bucket_matches(
                bucket,
                start,
                end,
                start_definitions,
                end_definitions,
            )
        })
        .map(|bucket| bucket.entry_indices.as_slice());
    let reversed_direction =
        if start == end && definition_families_match_as_sets(start_definitions, end_definitions) {
            None
        } else {
            buckets
                .iter()
                .rev()
                .find(|bucket| {
                    definition_reachability_bucket_matches(
                        bucket,
                        end,
                        start,
                        end_definitions,
                        start_definitions,
                    )
                })
                .map(|bucket| bucket.entry_indices.as_slice())
        };
    (same_direction, reversed_direction)
}

fn newest_matching_bucket_entry_index(
    same_direction: Option<&[usize]>,
    reversed_direction: Option<&[usize]>,
    mut predicate: impl FnMut(usize) -> bool,
) -> Option<usize> {
    let mut same_index = same_direction.and_then(|indices| indices.len().checked_sub(1));
    let mut reversed_index = reversed_direction.and_then(|indices| indices.len().checked_sub(1));

    loop {
        let next_same = same_index.and_then(|index| same_direction.map(|indices| indices[index]));
        let next_reversed =
            reversed_index.and_then(|index| reversed_direction.map(|indices| indices[index]));
        let next = match (next_same, next_reversed) {
            (Some(same_entry), Some(reversed_entry)) => {
                if same_entry >= reversed_entry {
                    same_index = same_index.and_then(|index| index.checked_sub(1));
                    same_entry
                } else {
                    reversed_index = reversed_index.and_then(|index| index.checked_sub(1));
                    reversed_entry
                }
            }
            (Some(same_entry), None) => {
                same_index = same_index.and_then(|index| index.checked_sub(1));
                same_entry
            }
            (None, Some(reversed_entry)) => {
                reversed_index = reversed_index.and_then(|index| index.checked_sub(1));
                reversed_entry
            }
            (None, None) => return None,
        };
        if predicate(next) {
            return Some(next);
        }
    }
}

fn push_definition_reachability_bucket_entry(
    buckets: &mut Vec<DefinitionReachabilityBucket>,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
    entry_index: usize,
) {
    if let Some(bucket) = buckets.iter_mut().find(|bucket| {
        definition_reachability_bucket_matches(
            bucket,
            start,
            end,
            start_definitions,
            end_definitions,
        )
    }) {
        bucket.entry_indices.push(entry_index);
    } else {
        buckets.push(DefinitionReachabilityBucket {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.to_vec(),
            end_definitions: end_definitions.to_vec(),
            entry_indices: vec![entry_index],
        });
    }
}

fn cached_definition_cycle_guard_result(
    cache: &DefinitionCycleGuardReachabilityCache,
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
    let (same_direction, reversed_direction) =
        matching_definition_reachability_bucket_entry_indices(
            &cache.buckets,
            start,
            end,
            start_definitions,
            end_definitions,
        );
    if let Some(index) =
        newest_matching_bucket_entry_index(same_direction, reversed_direction, |index| {
            visited_definition_points_match_as_sets(
                &cache.entries[index].visited_points,
                &normalized_visited_points,
            )
        })
    {
        return Some(cache.entries[index].result.clone());
    }

    newest_matching_bucket_entry_index(same_direction, reversed_direction, |index| {
        match &cache.entries[index].result {
            Ok(false)
                if visited_definition_points_subset_of(
                    &cache.entries[index].visited_points,
                    &normalized_visited_points,
                ) =>
            {
                true
            }
            Ok(true)
                if visited_definition_points_subset_of(
                    &normalized_visited_points,
                    &cache.entries[index].visited_points,
                ) =>
            {
                true
            }
            _ => false,
        }
    })
    .map(|index| cache.entries[index].result.clone())
}

fn begin_definition_cycle_guard_result(
    cache: &mut DefinitionCycleGuardReachabilityCache,
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
    cache
        .entries
        .push(DefinitionCycleGuardReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.to_vec(),
            end_definitions: end_definitions.to_vec(),
            visited_points: normalized_visited_points,
            result: Err(HypermeshError::UnknownClassification),
        });
    let index = cache.entries.len() - 1;
    push_definition_reachability_bucket_entry(
        &mut cache.buckets,
        start,
        end,
        start_definitions,
        end_definitions,
        index,
    );
    index
}

fn cached_definition_no_plane_replacement_cycle_guard_result(
    cache: &DefinitionNoPlaneReplacementCycleGuardCache,
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
    let (same_direction, reversed_direction) =
        matching_definition_reachability_bucket_entry_indices(
            &cache.buckets,
            start,
            end,
            start_definitions,
            end_definitions,
        );
    if let Some(index) =
        newest_matching_bucket_entry_index(same_direction, reversed_direction, |index| {
            visited_definition_points_match_as_sets(
                &cache.entries[index].visited_points,
                &normalized_visited_points,
            )
        })
    {
        return Some(cache.entries[index].result.clone());
    }

    newest_matching_bucket_entry_index(same_direction, reversed_direction, |index| {
        match &cache.entries[index].result {
            Ok(false)
                if visited_definition_points_subset_of(
                    &cache.entries[index].visited_points,
                    &normalized_visited_points,
                ) =>
            {
                true
            }
            Ok(true)
                if visited_definition_points_subset_of(
                    &normalized_visited_points,
                    &cache.entries[index].visited_points,
                ) =>
            {
                true
            }
            _ => false,
        }
    })
    .map(|index| cache.entries[index].result.clone())
}

fn begin_definition_no_plane_replacement_cycle_guard_result(
    cache: &mut DefinitionNoPlaneReplacementCycleGuardCache,
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
    cache
        .entries
        .push(DefinitionNoPlaneReplacementCycleGuardCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.to_vec(),
            end_definitions: end_definitions.to_vec(),
            visited_points: normalized_visited_points,
            result: Err(HypermeshError::UnknownClassification),
        });
    let index = cache.entries.len() - 1;
    push_definition_reachability_bucket_entry(
        &mut cache.buckets,
        start,
        end,
        start_definitions,
        end_definitions,
        index,
    );
    index
}

fn cached_definition_no_detour_reachability_result(
    cache: &DefinitionNoDetourReachabilityCache,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> Option<HypermeshResult<bool>> {
    let (same_direction, reversed_direction) =
        matching_definition_reachability_bucket_entry_indices(
            &cache.buckets,
            start,
            end,
            start_definitions,
            end_definitions,
        );
    newest_matching_bucket_entry_index(same_direction, reversed_direction, |_| true)
        .map(|index| cache.entries[index].result.clone())
}

fn begin_definition_no_detour_reachability_result(
    cache: &mut DefinitionNoDetourReachabilityCache,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> usize {
    cache
        .entries
        .push(DefinitionNoDetourReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.to_vec(),
            end_definitions: end_definitions.to_vec(),
            result: Err(HypermeshError::UnknownClassification),
        });
    let index = cache.entries.len() - 1;
    push_definition_reachability_bucket_entry(
        &mut cache.buckets,
        start,
        end,
        start_definitions,
        end_definitions,
        index,
    );
    index
}

fn cached_definition_no_plane_replacement_reachability_result(
    cache: &DefinitionNoPlaneReplacementReachabilityCache,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> Option<HypermeshResult<bool>> {
    let (same_direction, reversed_direction) =
        matching_definition_reachability_bucket_entry_indices(
            &cache.buckets,
            start,
            end,
            start_definitions,
            end_definitions,
        );
    newest_matching_bucket_entry_index(same_direction, reversed_direction, |_| true)
        .map(|index| cache.entries[index].result.clone())
}

fn begin_definition_no_plane_replacement_reachability_result(
    cache: &mut DefinitionNoPlaneReplacementReachabilityCache,
    start: &Point3,
    end: &Point3,
    start_definitions: &[[Plane; 3]],
    end_definitions: &[[Plane; 3]],
) -> usize {
    cache
        .entries
        .push(DefinitionNoPlaneReplacementReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.to_vec(),
            end_definitions: end_definitions.to_vec(),
            result: Err(HypermeshError::UnknownClassification),
        });
    let index = cache.entries.len() - 1;
    push_definition_reachability_bucket_entry(
        &mut cache.buckets,
        start,
        end,
        start_definitions,
        end_definitions,
        index,
    );
    index
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

fn probe_polyline_reaches_adjacent_cell(
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
fn probe_reaches_adjacent_cell_from_interior(
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

fn probe_reaches_adjacent_cell_from_interior_with_caches(
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

fn cached_definition_no_detour_reachability_with(
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
fn cached_definition_no_plane_replacement_reachability_with(
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

#[cfg(test)]
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
fn probe_reaches_adjacent_cell_via_progressive_detours(
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

fn probe_reaches_adjacent_cell_with_definitions_no_detours_with_caches(
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

fn probe_reaches_adjacent_cell_with_definitions_no_step_detours_with_caches(
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
fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
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
fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
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
fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
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

fn probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query_mode(
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

fn evaluate_probe_detour_target_without_plane_replacement_with_surface_query(
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
fn probe_reaches_adjacent_cell_with_detours_breadth_first_with_surface_query(
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

fn probe_reaches_adjacent_cell_with_detour_batches_breadth_first_with_surface_query(
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
fn plane_replacement_path_reaches_adjacent_cell_without_step_detours(
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

fn plane_replacement_path_reaches_adjacent_cell_without_step_detours_with_caches(
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
fn plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
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

fn plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
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

fn ordered_axis_orderings_by_no_step_precheck_with(
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

fn cached_plane_replacement_reachability_step_with(
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

fn cached_plane_replacement_reachability_path_with(
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

fn cached_plane_replacement_no_nested_ordering_warmup_with(
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

fn push_plane_replacement_reachability_step_bucket_entry(
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

fn push_plane_replacement_reachability_path_bucket_entry(
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
