//! Leaf processing for the subdivision pipeline.

mod split;

use split::*;

use crate::bvh::ExactBvh;
use crate::clip::{ClipSide, clip_polygon};
use crate::error::HypermeshResult;
use crate::geometry::{
    Aabb, Classification, Plane, axis_mut, axis_ref, classify_real, compare_real,
};
use crate::halfspace::{
    aabb_core_halfspaces, axis_halfspace, halfspace_has_opposite_pair,
    halfspace_is_degenerate_bound, limit_plane_families_match_as_sets, point_satisfies_halfspaces,
    support_side_halfspace,
};
use crate::intersection::{
    IntersectionSegment, PairwiseIntersection, PairwiseIntersectionType, intersect_polygons,
};
use crate::local_bsp::{BspLeaf, LocalBsp};
use crate::mesh::classify_edge_balance;
use crate::output::{
    ClassifiedPolygon, ClassifiedPolygonBucketState, merge_unique_classified_polygons,
    merge_unique_classified_polygons_with_bucket_state,
    push_unique_classified_polygon_with_bucket_state,
};
use crate::polygon::ConvexPolygon;
use crate::segment_trace::{
    LeafProbeQueryCaches, affine_from_planes, axis_plane_definition,
    certified_leaf_interior_points, classify_leaf_polygon_interior_point_with_probe_query_caches,
    classify_leaf_polygon_with_probe_query_caches,
    ordered_interior_points_for_probe_search_with_support,
    trace_segment_from_definitions_with_step_detoured_plane_replacement_in_bounds,
};
use crate::winding::{
    BooleanOp, Indicator, WindingNumberVector, WindingPair,
    can_boolean_op_be_inside_with_component_ranges,
    can_boolean_op_be_inside_with_transition_reachability, classify_polygon_output, propagate_wnv,
};
use hyperlattice::{HomogeneousPoint3, Point3, Real, intersect_three_planes};
use std::cell::RefCell;
use std::sync::Arc;

use hyperlimit::{
    HalfspaceFeasibility, Plane3 as LimitPlane3, PredicateOutcome, classify_halfspace_feasibility3,
};

/// Default subdivision depth budget.
///
/// `usize::MAX` disables the caller-selected depth budget. Subdivision still
/// terminates because every branch can consume only the finite root split
/// basis constructed for the top-level task.
pub const DEFAULT_MAX_DEPTH: usize = usize::MAX;

/// Configuration for recursive subdivision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SubdivisionConfig {
    /// Maximum recursive depth, or `usize::MAX` for no caller-selected limit.
    ///
    /// Reaching this bound is an explicit failure mode when the current task
    /// has not certified as a complete leaf and an exact root-basis arrangement
    /// split remains available.
    pub max_depth: usize,
}

impl Default for SubdivisionConfig {
    fn default() -> Self {
        Self {
            max_depth: DEFAULT_MAX_DEPTH,
        }
    }
}

/// A subproblem in the subdivision tree.
#[derive(Clone, Debug, PartialEq)]
pub struct SubdivisionTask {
    /// Polygons clipped to this task.
    pub polygons: Vec<ConvexPolygon>,
    /// Task bounds.
    pub bounds: Aabb,
    /// Reference point with known winding.
    ///
    /// References are normally off the polygon arrangement. If a root caller
    /// supplies a point on a boundary-free closed polygon family contained in
    /// the task bounds, the implementation may normalize it only when a trace
    /// from a certified exterior point proves which adjacent open arrangement
    /// cell has `ref_wnv`. This includes face, edge, vertex, and non-coplanar
    /// multi-surface contacts; clipped-open and missing-mesh families remain
    /// uncertified.
    pub ref_point: Point3,
    /// Plane triples that certify constructions of `ref_point`.
    ///
    /// Subdivision normalizes this family before use: triples whose affine
    /// reconstruction is singular or differs from `ref_point` are removed,
    /// plane-set duplicates are collapsed, and an exact axis triple is used
    /// when no retained certificate survives.
    pub ref_definitions: Vec<[crate::geometry::Plane; 3]>,
    /// Winding number at `ref_point`, or the winding of one independently
    /// certified adjacent open cell under the on-surface rule above.
    pub ref_wnv: Vec<i32>,
    /// Recursive depth.
    pub depth: usize,
}

impl SubdivisionTask {
    /// Constructs a root subdivision task.
    pub fn new(
        polygons: Vec<ConvexPolygon>,
        bounds: Aabb,
        ref_point: Point3,
        ref_wnv: Vec<i32>,
    ) -> Self {
        Self {
            ref_definitions: vec![axis_plane_definition(&ref_point)],
            polygons,
            bounds,
            ref_point,
            ref_wnv,
            depth: 0,
        }
    }
}

/// Basic counters from leaf processing.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LeafProcessingStats {
    /// Number of polygons inspected in this leaf.
    pub polygon_count: usize,
    /// Number of non-empty pairwise intersections.
    pub intersection_count: usize,
    /// Number of polygons emitted through direct leaf classification.
    pub direct_polygon_count: usize,
    /// Number of enabled face-local BSP leaves classified.
    pub bsp_leaf_count: usize,
    /// Number of BSP fragments emitted.
    pub bsp_fragment_count: usize,
    /// Whether every emitted or discarded output decision in this leaf was
    /// certified after exact local BSP isolation checks and exact classifier
    /// traces.
    pub certified_complete: bool,
}

type PolygonFamilyProfile = Vec<(isize, isize, usize, Vec<i32>)>;

#[derive(Clone, Debug, PartialEq)]
struct LeafClassificationCacheEntry {
    context: Option<Arc<LeafClassificationCacheContextKey>>,
    support: Plane,
    edges: Vec<Plane>,
    delta_w: Vec<i32>,
    winding: HypermeshResult<WindingNumberVector>,
}

#[derive(Clone, Debug, PartialEq)]
struct LeafPointClassificationState {
    winding: Option<WindingNumberVector>,
    saw_unknown: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct LeafPointClassificationCacheEntry {
    context: Option<Arc<LeafClassificationCacheContextKey>>,
    support: Plane,
    point: crate::segment_trace::InteriorLeafPoint,
    delta_w: Vec<i32>,
    state: HypermeshResult<LeafPointClassificationState>,
}

#[derive(Clone, Debug, PartialEq)]
struct LeafClassificationCacheContextKey {
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    bounds: Aabb,
    ref_point: Point3,
    ref_definitions: Vec<[Plane; 3]>,
    ref_wnv: Vec<i32>,
}

#[derive(Clone, Debug, PartialEq)]
struct ChildReferenceCacheEntry {
    source_polygon_profile: PolygonFamilyProfile,
    source_polygons: Vec<ConvexPolygon>,
    bounds: Aabb,
    old_ref: Point3,
    old_ref_definitions: Vec<[Plane; 3]>,
    old_wnv: Vec<i32>,
    result: HypermeshResult<(Point3, Vec<[Plane; 3]>, Vec<i32>)>,
}

#[derive(Clone, Debug, PartialEq)]
struct ChildSubdivisionCacheEntry {
    polygon_profile: PolygonFamilyProfile,
    task: SubdivisionTask,
    result: HypermeshResult<Vec<ClassifiedPolygon>>,
}

#[derive(Clone, Debug, PartialEq)]
struct WindingReachabilityCacheEntry {
    op: BooleanOp,
    ref_wnv: Vec<i32>,
    transition_profile: Vec<Vec<i32>>,
    result: HypermeshResult<bool>,
}

#[derive(Clone, Debug, PartialEq)]
struct SplitAttemptChild {
    polygons: Vec<ConvexPolygon>,
    bounds: Aabb,
    unchanged_from_parent: bool,
    original_order: usize,
}

#[derive(Clone, Debug, PartialEq)]
struct PairwiseIntersectionsCacheEntry {
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    result: HypermeshResult<Vec<Vec<PairwiseIntersection>>>,
}

#[derive(Clone, Debug, PartialEq)]
struct HostBspLeavesCacheEntry {
    host: ConvexPolygon,
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    leaves: HypermeshResult<Vec<BspLeaf>>,
}

#[derive(Clone, Debug, PartialEq)]
struct BspLeafCertificationCacheEntry {
    host: ConvexPolygon,
    leaf_edges: Vec<Plane>,
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    result: HypermeshResult<(Vec<crate::segment_trace::InteriorLeafPoint>, Vec<i32>)>,
}

struct SubdivisionRuntimeCaches {
    polygon_family_bounds: RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    polygon_axis_values: RefCell<Vec<PolygonAxisValuesCacheEntry>>,
    split_candidates: RefCell<SplitCandidatesCache>,
    split_child_fanout_counts: RefCell<Vec<SplitAttemptChildFanoutCacheEntry>>,
    split_child_partitions: RefCell<Vec<SplitChildPartitionCacheEntry>>,
    pairwise_intersections: RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    host_bsp_leaves: RefCell<Vec<HostBspLeavesCacheEntry>>,
    bsp_leaf_certification: RefCell<Vec<BspLeafCertificationCacheEntry>>,
    leaf_classification: RefCell<Vec<LeafClassificationCacheEntry>>,
    leaf_point_classification: RefCell<Vec<LeafPointClassificationCacheEntry>>,
    support_reference_query: RefCell<SupportReferenceQueryCaches>,
    child_reference: RefCell<Vec<ChildReferenceCacheEntry>>,
    child_subdivision: RefCell<Vec<ChildSubdivisionCacheEntry>>,
    winding_reachability: RefCell<Vec<WindingReachabilityCacheEntry>>,
}

impl Default for SubdivisionRuntimeCaches {
    fn default() -> Self {
        Self {
            polygon_family_bounds: RefCell::new(Vec::new()),
            polygon_axis_values: RefCell::new(Vec::new()),
            split_candidates: RefCell::new(SplitCandidatesCache::default()),
            split_child_fanout_counts: RefCell::new(Vec::new()),
            split_child_partitions: RefCell::new(Vec::new()),
            pairwise_intersections: RefCell::new(Vec::new()),
            host_bsp_leaves: RefCell::new(Vec::new()),
            bsp_leaf_certification: RefCell::new(Vec::new()),
            leaf_classification: RefCell::new(Vec::new()),
            leaf_point_classification: RefCell::new(Vec::new()),
            support_reference_query: RefCell::new(SupportReferenceQueryCaches::default()),
            child_reference: RefCell::new(Vec::new()),
            child_subdivision: RefCell::new(Vec::new()),
            winding_reachability: RefCell::new(Vec::new()),
        }
    }
}

/// Processes one leaf and returns classified output polygons.
pub fn process_leaf(
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    indicator: &Indicator,
) -> HypermeshResult<Vec<ClassifiedPolygon>> {
    let mut output = Vec::new();
    process_leaf_into(
        polygons,
        bounds,
        ref_point,
        ref_definitions,
        ref_wnv,
        indicator,
        &mut output,
    )?;
    Ok(output)
}

/// Processes one leaf into an existing output buffer.
pub fn process_leaf_into(
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    indicator: &Indicator,
    output: &mut Vec<ClassifiedPolygon>,
) -> HypermeshResult<LeafProcessingStats> {
    let mut certified_output = Vec::new();
    let stats = process_leaf_into_inner(
        polygons,
        bounds,
        ref_point,
        ref_definitions,
        ref_wnv,
        indicator,
        &mut certified_output,
    )?;
    merge_unique_classified_polygons(output, certified_output);
    Ok(stats)
}

fn process_leaf_into_inner(
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    indicator: &Indicator,
    output: &mut Vec<ClassifiedPolygon>,
) -> HypermeshResult<LeafProcessingStats> {
    let leaf_classification_cache = RefCell::new(Vec::new());
    let leaf_point_classification_cache = RefCell::new(Vec::new());
    process_leaf_into_inner_with_pairwise_cache(
        polygons,
        bounds,
        ref_point,
        ref_definitions,
        ref_wnv,
        indicator,
        output,
        &leaf_classification_cache,
        &leaf_point_classification_cache,
        pairwise_intersections_by_polygon,
        build_host_bsp_leaves,
        |polygon, leaf_edges, polygons, intersections| {
            certify_bsp_leaf_and_delta_w_with_host_intersections(
                polygon,
                leaf_edges,
                polygons,
                Some(intersections),
            )
        },
    )
}

fn process_leaf_into_inner_with_pairwise_cache(
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    indicator: &Indicator,
    output: &mut Vec<ClassifiedPolygon>,
    leaf_classification_cache: &RefCell<Vec<LeafClassificationCacheEntry>>,
    leaf_point_classification_cache: &RefCell<Vec<LeafPointClassificationCacheEntry>>,
    pairwise_query: impl FnOnce(&[ConvexPolygon]) -> HypermeshResult<Vec<Vec<PairwiseIntersection>>>,
    bsp_leaves_query: impl Fn(
        &ConvexPolygon,
        &[ConvexPolygon],
        &[PairwiseIntersection],
    ) -> HypermeshResult<Vec<BspLeaf>>,
    certify_bsp_leaf: impl Fn(
        &ConvexPolygon,
        &[crate::geometry::Plane],
        &[ConvexPolygon],
        &[PairwiseIntersection],
    ) -> HypermeshResult<(
        Vec<crate::segment_trace::InteriorLeafPoint>,
        Vec<i32>,
    )>,
) -> HypermeshResult<LeafProcessingStats> {
    let mut stats = LeafProcessingStats {
        polygon_count: polygons.len(),
        ..LeafProcessingStats::default()
    };
    let mut output_buckets = ClassifiedPolygonBucketState::from_classified(output);
    if polygons.is_empty() {
        stats.certified_complete = true;
        return Ok(stats);
    }
    let leaf_cache_context = Arc::new(LeafClassificationCacheContextKey {
        polygon_profile: polygon_family_profile(polygons),
        polygons: polygons.to_vec(),
        bounds: bounds.clone(),
        ref_point: ref_point.clone(),
        ref_definitions: ref_definitions.to_vec(),
        ref_wnv: ref_wnv.to_vec(),
    });

    let intersections = pairwise_query(polygons)?;
    stats.intersection_count = intersections.iter().map(Vec::len).sum();

    for index in ordered_leaf_polygon_indices_by_intersections(&intersections) {
        let polygon = &polygons[index];
        if intersections[index].is_empty() {
            let mut leaf_probe_query_caches = LeafProbeQueryCaches::default();
            let emitted = emit_one_direct(
                polygon,
                bounds,
                ref_point,
                ref_definitions,
                ref_wnv,
                polygons,
                indicator,
                leaf_classification_cache,
                Some(&leaf_cache_context),
                &mut leaf_probe_query_caches,
                output,
                &mut output_buckets,
            )?;
            stats.direct_polygon_count += usize::from(emitted);
            continue;
        }

        let bsp_leaves = bsp_leaves_query(polygon, polygons, &intersections[index])?;
        let mut seen_bsp_leaf_edges = Vec::new();
        for leaf_index in ordered_bsp_leaf_indices_by_complexity(&bsp_leaves) {
            let leaf = &bsp_leaves[leaf_index];
            if leaf.edges.len() < 3 {
                continue;
            }
            if !take_new_bsp_leaf_edge_cycle(&mut seen_bsp_leaf_edges, &leaf.edges) {
                continue;
            }
            let (interior_points, effective_delta_w) =
                certify_bsp_leaf(polygon, &leaf.edges, polygons, &intersections[index])?;
            stats.bsp_leaf_count += 1;
            let w_front = cached_leaf_classification_with(
                &mut leaf_classification_cache.borrow_mut(),
                Some(&leaf_cache_context),
                &polygon.support,
                &leaf.edges,
                &effective_delta_w,
                || {
                    classify_leaf_polygon_from_interior_points_with_point_cache(
                        &interior_points,
                        &polygon.support,
                        ref_point,
                        ref_definitions,
                        ref_wnv,
                        polygons,
                        bounds,
                        &effective_delta_w,
                        &mut leaf_point_classification_cache.borrow_mut(),
                        Some(&leaf_cache_context),
                    )
                },
            )?;
            let w_back = propagate_wnv(&w_front, 1, &effective_delta_w)?;
            let classification = classify_polygon_output(&w_front, &w_back, indicator);
            if classification != 0 {
                let mut fragment = polygon.clone();
                fragment.edges = leaf.edges.clone();
                fragment.delta_w = effective_delta_w;
                let mut classified = ClassifiedPolygon::new(fragment, classification);
                classified.winding = Some(WindingPair { w_front, w_back });
                classified.is_bsp_fragment = true;
                push_unique_classified_polygon_with_bucket_state(
                    output,
                    &mut output_buckets,
                    classified,
                );
                stats.bsp_fragment_count += 1;
            }
        }
    }

    stats.certified_complete = true;
    Ok(stats)
}

fn ordered_leaf_polygon_indices_by_intersections(
    intersections: &[Vec<PairwiseIntersection>],
) -> Vec<usize> {
    let mut indices = (0..intersections.len()).collect::<Vec<_>>();
    indices.sort_by_key(|&index| {
        (
            intersections[index].is_empty(),
            std::cmp::Reverse(intersections[index].len()),
            index,
        )
    });
    indices
}

fn ordered_bsp_leaf_indices_by_complexity(leaves: &[BspLeaf]) -> Vec<usize> {
    let mut indices = (0..leaves.len()).collect::<Vec<_>>();
    indices.sort_by_key(|&index| (std::cmp::Reverse(leaves[index].edges.len()), index));
    indices
}

/// Recursively subdivides a task and returns classified output polygons.
pub fn subdivide(
    task: SubdivisionTask,
    indicator: &Indicator,
    config: SubdivisionConfig,
) -> HypermeshResult<Vec<ClassifiedPolygon>> {
    let mut output = Vec::new();
    let caches = SubdivisionRuntimeCaches::default();
    let pairwise_cache = &caches.pairwise_intersections;
    let host_bsp_cache = &caches.host_bsp_leaves;
    let bsp_leaf_cache = &caches.bsp_leaf_certification;
    let leaf_classification_cache = &caches.leaf_classification;
    let leaf_point_classification_cache = &caches.leaf_point_classification;
    let mut process_leaf = move |task: &SubdivisionTask,
                                 indicator: &Indicator,
                                 output: &mut Vec<ClassifiedPolygon>| {
        process_leaf_task_into_with_caches(
            task,
            indicator,
            output,
            pairwise_cache,
            host_bsp_cache,
            bsp_leaf_cache,
            leaf_classification_cache,
            leaf_point_classification_cache,
        )
    };
    subdivide_into_inner_with(
        task,
        indicator,
        config,
        None,
        &mut output,
        &mut process_leaf,
        &caches,
        &caches.winding_reachability,
    )?;
    Ok(output)
}

pub(crate) fn subdivide_for_operation(
    task: SubdivisionTask,
    indicator: &Indicator,
    config: SubdivisionConfig,
    op: BooleanOp,
) -> HypermeshResult<Vec<ClassifiedPolygon>> {
    let mut output = Vec::new();
    let caches = SubdivisionRuntimeCaches::default();
    let pairwise_cache = &caches.pairwise_intersections;
    let host_bsp_cache = &caches.host_bsp_leaves;
    let bsp_leaf_cache = &caches.bsp_leaf_certification;
    let leaf_classification_cache = &caches.leaf_classification;
    let leaf_point_classification_cache = &caches.leaf_point_classification;
    let mut process_leaf = move |task: &SubdivisionTask,
                                 indicator: &Indicator,
                                 output: &mut Vec<ClassifiedPolygon>| {
        process_leaf_task_into_with_caches(
            task,
            indicator,
            output,
            pairwise_cache,
            host_bsp_cache,
            bsp_leaf_cache,
            leaf_classification_cache,
            leaf_point_classification_cache,
        )
    };
    subdivide_into_inner_with(
        task,
        indicator,
        config,
        Some(op),
        &mut output,
        &mut process_leaf,
        &caches,
        &caches.winding_reachability,
    )?;
    Ok(output)
}

/// Recursively subdivides a task into an existing output buffer.
///
/// The caller-visible buffer is extended only after the whole task certifies.
/// If subdivision or leaf classification returns an error, no partial output
/// from that task is retained.
pub fn subdivide_into(
    task: SubdivisionTask,
    indicator: &Indicator,
    config: SubdivisionConfig,
    output: &mut Vec<ClassifiedPolygon>,
) -> HypermeshResult<()> {
    let mut certified_output = Vec::new();
    let caches = SubdivisionRuntimeCaches::default();
    let pairwise_cache = &caches.pairwise_intersections;
    let host_bsp_cache = &caches.host_bsp_leaves;
    let bsp_leaf_cache = &caches.bsp_leaf_certification;
    let leaf_classification_cache = &caches.leaf_classification;
    let leaf_point_classification_cache = &caches.leaf_point_classification;
    let mut process_leaf = move |task: &SubdivisionTask,
                                 indicator: &Indicator,
                                 output: &mut Vec<ClassifiedPolygon>| {
        process_leaf_task_into_with_caches(
            task,
            indicator,
            output,
            pairwise_cache,
            host_bsp_cache,
            bsp_leaf_cache,
            leaf_classification_cache,
            leaf_point_classification_cache,
        )
    };
    subdivide_into_inner_with(
        task,
        indicator,
        config,
        None,
        &mut certified_output,
        &mut process_leaf,
        &caches,
        &caches.winding_reachability,
    )?;
    merge_unique_classified_polygons(output, certified_output);
    Ok(())
}

fn subdivide_into_inner_with(
    mut task: SubdivisionTask,
    indicator: &Indicator,
    config: SubdivisionConfig,
    reachability_op: Option<BooleanOp>,
    output: &mut Vec<ClassifiedPolygon>,
    process_leaf: &mut impl FnMut(
        &SubdivisionTask,
        &Indicator,
        &mut Vec<ClassifiedPolygon>,
    ) -> HypermeshResult<LeafProcessingStats>,
    caches: &SubdivisionRuntimeCaches,
    winding_reachability_cache: &RefCell<Vec<WindingReachabilityCacheEntry>>,
) -> HypermeshResult<()> {
    if task.polygons.is_empty() {
        return Ok(());
    }
    task.ref_definitions = certified_reference_definitions(&task.ref_point, &task.ref_definitions);

    cached_root_split_basis_with(
        &caches.split_candidates,
        &caches.polygon_axis_values,
        &caches.pairwise_intersections,
        &task.bounds,
        &task.polygons,
    )?;

    if let Some(contracted_task) = contract_task_to_polygon_family_bounds_if_tighter(&task, caches)?
    {
        let contracted_output = if let Some(reused) = {
            let mut query_caches = caches.support_reference_query.borrow_mut();
            if let Some(reused) = reusable_child_subdivision_if_certified(
                &caches.child_subdivision,
                &contracted_task,
                &mut query_caches,
            )? {
                Some(reused)
            } else {
                reusable_child_subdivision_from_cached_trace_if_certified(
                    &caches.child_subdivision,
                    &contracted_task,
                    &mut query_caches,
                )?
            }
        } {
            reused
        } else {
            cached_child_subdivision_with(&caches.child_subdivision, &contracted_task, || {
                let mut contracted_output = Vec::new();
                subdivide_into_inner_with(
                    contracted_task.clone(),
                    indicator,
                    config,
                    reachability_op,
                    &mut contracted_output,
                    process_leaf,
                    caches,
                    winding_reachability_cache,
                )?;
                Ok(contracted_output)
            })?
        };
        merge_unique_classified_polygons(output, contracted_output);
        return Ok(());
    }

    let mut output_buckets = ClassifiedPolygonBucketState::from_classified(output);

    if let Some(op) = reachability_op
        && cached_winding_reachability_with(
            winding_reachability_cache,
            op,
            &task.ref_wnv,
            &task.polygons,
            || can_discard_by_winding_reachability(op, &task.ref_wnv, &task.polygons),
        )?
    {
        return Ok(());
    }

    let can_split = can_split_bounds(&task.bounds)?;

    if !can_split {
        if let Some(certified_output) =
            certified_leaf_output_if_complete_with(&task, indicator, |task, indicator, output| {
                process_leaf(task, indicator, output)
            })?
        {
            merge_unique_classified_polygons_with_bucket_state(
                output,
                &mut output_buckets,
                certified_output,
            );
            return Ok(());
        }
        return Err(crate::error::HypermeshError::UnknownClassification);
    }

    if let Some(certified_output) =
        certified_leaf_output_if_complete_with(&task, indicator, |task, indicator, output| {
            process_leaf(task, indicator, output)
        })?
    {
        merge_unique_classified_polygons_with_bucket_state(
            output,
            &mut output_buckets,
            certified_output,
        );
        return Ok(());
    }

    let split_candidates = cached_ordered_subdivision_splits_with(
        &caches.polygon_axis_values,
        &caches.split_candidates,
        &caches.split_child_fanout_counts,
        &caches.split_child_partitions,
        &caches.polygon_family_bounds,
        &caches.pairwise_intersections,
        &task.bounds,
        &task.polygons,
    )?;

    if subdivision_depth_budget_reached(task.depth, config.max_depth) {
        if split_candidates.is_empty() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        return Err(crate::error::HypermeshError::SubdivisionDepthLimit {
            depth: task.depth,
            polygon_count: task.polygons.len(),
        });
    }

    let (preferred_split, deferred_splits) =
        partition_preferred_subdivision_split(split_candidates, task.polygons.len());
    let mut best_failure = None;

    if let Some(candidate_output) = try_ranked_subdivision_attempts(
        &task,
        preferred_split,
        indicator,
        config,
        reachability_op,
        process_leaf,
        caches,
        winding_reachability_cache,
        &mut best_failure,
    )? {
        merge_unique_classified_polygons_with_bucket_state(
            output,
            &mut output_buckets,
            candidate_output,
        );
        return Ok(());
    }

    if let Some(candidate_output) = try_ranked_subdivision_attempts(
        &task,
        deferred_splits,
        indicator,
        config,
        reachability_op,
        process_leaf,
        caches,
        winding_reachability_cache,
        &mut best_failure,
    )? {
        merge_unique_classified_polygons_with_bucket_state(
            output,
            &mut output_buckets,
            candidate_output,
        );
        return Ok(());
    }

    Err(best_failure.unwrap_or(crate::error::HypermeshError::UnknownClassification))
}

fn split_attempt_strictly_reduces_polygon_family(
    attempt: &RankedSplitAttempt,
    parent_polygon_count: usize,
) -> bool {
    attempt.counts.0 < parent_polygon_count
}

fn partition_preferred_subdivision_split(
    split_candidates: Vec<RankedSplitAttempt>,
    parent_polygon_count: usize,
) -> (Option<RankedSplitAttempt>, Vec<RankedSplitAttempt>) {
    let mut deferred_splits = split_candidates;
    let preferred_split = deferred_splits
        .iter()
        .position(|attempt| {
            split_attempt_strictly_reduces_polygon_family(attempt, parent_polygon_count)
        })
        .map(|index| deferred_splits.remove(index));
    (preferred_split, deferred_splits)
}

fn try_ranked_subdivision_attempts(
    task: &SubdivisionTask,
    split_attempts: impl IntoIterator<Item = RankedSplitAttempt>,
    indicator: &Indicator,
    config: SubdivisionConfig,
    reachability_op: Option<BooleanOp>,
    process_leaf: &mut impl FnMut(
        &SubdivisionTask,
        &Indicator,
        &mut Vec<ClassifiedPolygon>,
    ) -> HypermeshResult<LeafProcessingStats>,
    caches: &SubdivisionRuntimeCaches,
    winding_reachability_cache: &RefCell<Vec<WindingReachabilityCacheEntry>>,
    best_failure: &mut Option<crate::error::HypermeshError>,
) -> HypermeshResult<Option<Vec<ClassifiedPolygon>>> {
    for split_attempt in split_attempts {
        let split_children = ordered_split_attempt_children(
            &task.polygons,
            split_attempt.left_polys,
            split_attempt.left_bounds,
            split_attempt.right_polys,
            split_attempt.right_bounds,
        );
        let mut candidate_output = Vec::new();
        let mut candidate_buckets = ClassifiedPolygonBucketState::new();
        let attempt = (|| -> HypermeshResult<()> {
            for split_child in split_children {
                process_split_attempt_child(
                    task,
                    split_child.polygons,
                    split_child.bounds,
                    indicator,
                    config,
                    reachability_op,
                    &mut candidate_output,
                    &mut candidate_buckets,
                    process_leaf,
                    caches,
                    winding_reachability_cache,
                )?;
            }
            Ok(())
        })();

        match attempt {
            Ok(()) => return Ok(Some(candidate_output)),
            Err(err) if is_backtrackable_split_error(&err) => {
                record_split_failure(best_failure, err);
            }
            Err(err) => return Err(err),
        }
    }

    Ok(None)
}

fn contract_task_to_polygon_family_bounds_if_tighter(
    task: &SubdivisionTask,
    caches: &SubdivisionRuntimeCaches,
) -> HypermeshResult<Option<SubdivisionTask>> {
    let contracted_bounds = cached_polygon_family_bounds_with(
        &caches.polygon_family_bounds,
        &task.polygons,
        polygon_family_bounds,
    )?;
    if contracted_bounds == task.bounds {
        return Ok(None);
    }
    if !bounds_contains_bounds(&task.bounds, &contracted_bounds)? {
        return Ok(None);
    }

    let (ref_point, ref_definitions, ref_wnv) = {
        let mut query_caches = caches.support_reference_query.borrow_mut();
        if let Some(reused) =
            reusable_contracted_task_reference_from_cached_subdivision_if_certified(
                &caches.child_subdivision,
                task,
                &contracted_bounds,
                &mut query_caches,
            )?
        {
            reused
        } else {
            drop(query_caches);
            propagate_child_reference(task, &task.polygons, &contracted_bounds, caches)?
        }
    };
    let contracted_task = SubdivisionTask {
        polygons: task.polygons.clone(),
        bounds: contracted_bounds,
        ref_point,
        ref_definitions,
        ref_wnv,
        depth: task.depth,
    };
    if subdivision_task_state_matches_for_cache(&contracted_task, task) {
        return Ok(None);
    }
    Ok(Some(contracted_task))
}

fn ordered_split_attempt_children(
    parent_polygons: &[ConvexPolygon],
    left_polygons: Vec<ConvexPolygon>,
    left_bounds: Option<Aabb>,
    right_polygons: Vec<ConvexPolygon>,
    right_bounds: Option<Aabb>,
) -> Vec<SplitAttemptChild> {
    let mut children = Vec::with_capacity(2);
    if let Some(bounds) = left_bounds {
        children.push(SplitAttemptChild {
            unchanged_from_parent: polygon_families_match_as_multisets(
                &left_polygons,
                parent_polygons,
            ),
            polygons: left_polygons,
            bounds,
            original_order: 0,
        });
    }
    if let Some(bounds) = right_bounds {
        children.push(SplitAttemptChild {
            unchanged_from_parent: polygon_families_match_as_multisets(
                &right_polygons,
                parent_polygons,
            ),
            polygons: right_polygons,
            bounds,
            original_order: 1,
        });
    }
    children.sort_by_key(|child| {
        (
            child.unchanged_from_parent,
            child.polygons.len(),
            child.original_order,
        )
    });
    children
}

fn split_child_matches_parent_geometry(
    parent_polygons: &[ConvexPolygon],
    parent_bounds: &Aabb,
    child_polygons: &[ConvexPolygon],
    child_bounds: &Aabb,
) -> bool {
    child_bounds == parent_bounds
        && polygon_families_match_as_multisets(child_polygons, parent_polygons)
}

fn process_split_attempt_child(
    task: &SubdivisionTask,
    child_polygons: Vec<ConvexPolygon>,
    child_bounds: Aabb,
    indicator: &Indicator,
    config: SubdivisionConfig,
    reachability_op: Option<BooleanOp>,
    candidate_output: &mut Vec<ClassifiedPolygon>,
    candidate_buckets: &mut ClassifiedPolygonBucketState,
    process_leaf: &mut impl FnMut(
        &SubdivisionTask,
        &Indicator,
        &mut Vec<ClassifiedPolygon>,
    ) -> HypermeshResult<LeafProcessingStats>,
    caches: &SubdivisionRuntimeCaches,
    winding_reachability_cache: &RefCell<Vec<WindingReachabilityCacheEntry>>,
) -> HypermeshResult<()> {
    let (child_ref, child_ref_definitions, child_wnv) =
        propagate_child_reference(task, &child_polygons, &child_bounds, caches)?;
    let child_task = SubdivisionTask {
        polygons: child_polygons,
        bounds: child_bounds,
        ref_point: child_ref,
        ref_definitions: child_ref_definitions,
        ref_wnv: child_wnv,
        depth: task
            .depth
            .checked_add(1)
            .ok_or(crate::error::HypermeshError::UnknownClassification)?,
    };
    if subdivision_task_state_matches_for_cache(&child_task, task) {
        return Err(crate::error::HypermeshError::ReferencePropagationFailed);
    }
    let child_output = if let Some(reused) = {
        let mut query_caches = caches.support_reference_query.borrow_mut();
        if let Some(reused) = reusable_child_subdivision_if_certified(
            &caches.child_subdivision,
            &child_task,
            &mut query_caches,
        )? {
            Some(reused)
        } else {
            reusable_child_subdivision_from_cached_trace_if_certified(
                &caches.child_subdivision,
                &child_task,
                &mut query_caches,
            )?
        }
    } {
        reused
    } else {
        cached_child_subdivision_with(&caches.child_subdivision, &child_task, || {
            let mut child_output = Vec::new();
            subdivide_into_inner_with(
                child_task.clone(),
                indicator,
                config,
                reachability_op,
                &mut child_output,
                process_leaf,
                caches,
                winding_reachability_cache,
            )?;
            Ok(child_output)
        })?
    };
    merge_unique_classified_polygons_with_bucket_state(
        candidate_output,
        candidate_buckets,
        child_output,
    );
    Ok(())
}

fn subdivision_depth_budget_reached(depth: usize, max_depth: usize) -> bool {
    max_depth != usize::MAX && depth >= max_depth
}

fn propagate_child_reference(
    task: &SubdivisionTask,
    child_polygons: &[ConvexPolygon],
    child_bounds: &Aabb,
    caches: &SubdivisionRuntimeCaches,
) -> HypermeshResult<(Point3, Vec<[Plane; 3]>, Vec<i32>)> {
    let source_polygons = ordered_reference_search_polygons(&task.polygons, child_bounds);
    caches
        .support_reference_query
        .borrow_mut()
        .reset_per_reference_call_caches();
    let reused_reference = {
        let mut query_caches = caches.support_reference_query.borrow_mut();
        reusable_child_reference_if_certified(
            &caches.child_reference,
            task,
            child_polygons,
            child_bounds,
            &mut query_caches,
        )?
    };
    if let Some(reused) = reused_reference {
        return Ok(certified_reference_result(reused));
    }

    let direct_result = compute_new_reference_with_query_caches(
        &task.ref_point,
        &task.ref_definitions,
        &task.ref_wnv,
        child_bounds,
        &source_polygons,
        &mut caches.support_reference_query.borrow_mut(),
    );
    match direct_result {
        Ok(result) => {
            let result = certified_reference_result(result);
            cache_child_reference_result(
                &caches.child_reference,
                &task.ref_point,
                &task.ref_definitions,
                &task.ref_wnv,
                &source_polygons,
                child_bounds,
                &result,
            );
            Ok(result)
        }
        Err(crate::error::HypermeshError::UnknownClassification) => cached_child_reference_with(
            &caches.child_reference,
            &task.ref_point,
            &task.ref_definitions,
            &task.ref_wnv,
            &source_polygons,
            child_bounds,
            || Err(crate::error::HypermeshError::UnknownClassification),
        ),
        Err(err) => Err(err),
    }
}

fn ordered_reference_search_polygons(
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
) -> Vec<ConvexPolygon> {
    let bounds_approx = crate::polygon::ApproxBounds::new(bounds.min.clone(), bounds.max.clone());
    let mut indexed = polygons
        .iter()
        .cloned()
        .enumerate()
        .map(|(index, polygon)| {
            let overlaps_bounds = polygon.approx_bounds.as_ref().is_none_or(|polygon_bounds| {
                crate::bvh::bounds_overlap(polygon_bounds, &bounds_approx).unwrap_or(true)
            });
            (index, overlaps_bounds, polygon)
        })
        .collect::<Vec<_>>();
    indexed.sort_by_key(|(index, overlaps_bounds, polygon)| {
        (
            !*overlaps_bounds,
            polygon.mesh_index,
            polygon.polygon_index,
            *index,
        )
    });
    indexed.into_iter().map(|(_, _, polygon)| polygon).collect()
}

fn cached_child_reference_with(
    cache: &RefCell<Vec<ChildReferenceCacheEntry>>,
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    source_polygons: &[ConvexPolygon],
    bounds: &Aabb,
    query: impl FnOnce() -> HypermeshResult<(Point3, Vec<[Plane; 3]>, Vec<i32>)>,
) -> HypermeshResult<(Point3, Vec<[Plane; 3]>, Vec<i32>)> {
    if let Some(existing) = cache
        .borrow()
        .iter()
        .rev()
        .find(|existing| {
            child_reference_cache_entry_matches_exact_state(
                existing,
                old_ref,
                old_ref_definitions,
                old_wnv,
                source_polygons,
                bounds,
            )
        })
        .cloned()
    {
        return existing.result.map(certified_reference_result);
    }

    let source_polygon_profile = polygon_family_profile(source_polygons);
    let existing = cache
        .borrow()
        .iter()
        .rev()
        .find(|existing| {
            existing.old_ref == *old_ref
                && reference_definition_families_match_as_sets(
                    &existing.old_ref_definitions,
                    old_ref_definitions,
                )
                && existing.old_wnv == old_wnv
                && existing.source_polygon_profile == source_polygon_profile
                && polygon_families_match_as_multisets(&existing.source_polygons, source_polygons)
                && existing.bounds == *bounds
        })
        .cloned();
    if let Some(existing) = existing {
        if let Ok(result) = &existing.result
            && !child_reference_cache_entry_matches_exact_state(
                &existing,
                old_ref,
                old_ref_definitions,
                old_wnv,
                source_polygons,
                bounds,
            )
        {
            cache_child_reference_result(
                cache,
                old_ref,
                old_ref_definitions,
                old_wnv,
                source_polygons,
                bounds,
                result,
            );
        }
        return existing.result.map(certified_reference_result);
    }

    let result = query().map(certified_reference_result);
    cache.borrow_mut().push(ChildReferenceCacheEntry {
        source_polygon_profile,
        old_ref: old_ref.clone(),
        old_ref_definitions: old_ref_definitions.to_vec(),
        old_wnv: old_wnv.to_vec(),
        source_polygons: source_polygons.to_vec(),
        bounds: bounds.clone(),
        result: result.clone(),
    });
    result
}

fn cache_child_reference_result(
    cache: &RefCell<Vec<ChildReferenceCacheEntry>>,
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    source_polygons: &[ConvexPolygon],
    bounds: &Aabb,
    result: &(Point3, Vec<[Plane; 3]>, Vec<i32>),
) {
    if cache.borrow().iter().any(|existing| {
        child_reference_cache_entry_matches_exact_state(
            existing,
            old_ref,
            old_ref_definitions,
            old_wnv,
            source_polygons,
            bounds,
        )
    }) {
        return;
    }

    cache.borrow_mut().push(ChildReferenceCacheEntry {
        source_polygon_profile: polygon_family_profile(source_polygons),
        source_polygons: source_polygons.to_vec(),
        bounds: bounds.clone(),
        old_ref: old_ref.clone(),
        old_ref_definitions: old_ref_definitions.to_vec(),
        old_wnv: old_wnv.to_vec(),
        result: Ok(certified_reference_result(result.clone())),
    });
}

fn child_reference_cache_entry_matches_exact_state(
    existing: &ChildReferenceCacheEntry,
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    source_polygons: &[ConvexPolygon],
    bounds: &Aabb,
) -> bool {
    existing.old_ref == *old_ref
        && existing.old_ref_definitions == old_ref_definitions
        && existing.old_wnv == old_wnv
        && existing.source_polygons == source_polygons
        && existing.bounds == *bounds
}

fn reusable_child_reference_if_certified(
    cache: &RefCell<Vec<ChildReferenceCacheEntry>>,
    task: &SubdivisionTask,
    child_polygons: &[ConvexPolygon],
    child_bounds: &Aabb,
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<Option<(Point3, Vec<[Plane; 3]>, Vec<i32>)>> {
    let context = support_reference_cache_context_key(
        &task.ref_point,
        &task.ref_definitions,
        &task.ref_wnv,
        child_polygons,
    );
    match cached_reference_bounds_validity_with_context(
        &mut query_caches.validity_cache,
        Some(&context),
        child_bounds,
        &task.ref_point,
        |point| is_certified_valid_reference_for_bounds(point, child_bounds, child_polygons),
    ) {
        Ok(true) => {
            let reused = certified_reference_result((
                task.ref_point.clone(),
                task.ref_definitions.clone(),
                task.ref_wnv.clone(),
            ));
            cache_child_reference_result(
                cache,
                &task.ref_point,
                &task.ref_definitions,
                &task.ref_wnv,
                child_polygons,
                child_bounds,
                &reused,
            );
            Ok(Some(reused))
        }
        Ok(false) | Err(crate::error::HypermeshError::UnknownClassification) => Ok(None),
        Err(err) => Err(err),
    }
}

#[cfg(test)]
fn reusable_child_reference_from_cached_trace_if_certified(
    cache: &RefCell<Vec<ChildReferenceCacheEntry>>,
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    source_polygons: &[ConvexPolygon],
    bounds: &Aabb,
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<Option<(Point3, Vec<[Plane; 3]>, Vec<i32>)>> {
    let source_polygon_profile = polygon_family_profile(source_polygons);
    let context =
        support_reference_cache_context_key(old_ref, old_ref_definitions, old_wnv, source_polygons);
    let mut reused = None;
    {
        let cache_ref = cache.borrow();
        for existing in cache_ref.iter().rev() {
            if existing.source_polygon_profile != source_polygon_profile
                || !polygon_families_match_as_multisets(&existing.source_polygons, source_polygons)
            {
                continue;
            }
            let Ok((point, definitions, _)) = &existing.result else {
                continue;
            };
            let valid_for_bounds = cached_reference_bounds_validity_with_context(
                &mut query_caches.validity_cache,
                Some(&context),
                bounds,
                point,
                |point| is_certified_valid_reference_for_bounds(point, bounds, source_polygons),
            )?;
            if !valid_for_bounds {
                continue;
            }
            let target = ReferenceTarget::with_definitions(point.clone(), definitions.clone());
            if let Some(winding) = cached_reference_target_trace_with_context(
                &mut query_caches.trace_cache,
                Some(&context),
                &target,
                |target| {
                    trace_reference_target_from_validated_bounds(
                        old_ref,
                        old_ref_definitions,
                        old_wnv,
                        bounds,
                        source_polygons,
                        target,
                    )
                },
            )? {
                reused = Some((point.clone(), definitions.clone(), winding));
                break;
            }
        }
    }
    if let Some(reused) = reused {
        cache_child_reference_result(
            cache,
            old_ref,
            old_ref_definitions,
            old_wnv,
            source_polygons,
            bounds,
            &reused,
        );
        return Ok(Some(reused));
    }
    Ok(None)
}

#[cfg(test)]
fn reusable_child_reference_from_cached_result_if_certified(
    cache: &RefCell<Vec<ChildReferenceCacheEntry>>,
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    source_polygons: &[ConvexPolygon],
    bounds: &Aabb,
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<Option<(Point3, Vec<[Plane; 3]>, Vec<i32>)>> {
    let source_polygon_profile = polygon_family_profile(source_polygons);
    let context =
        support_reference_cache_context_key(old_ref, old_ref_definitions, old_wnv, source_polygons);
    let mut reused = None;
    {
        let cache_ref = cache.borrow();
        for existing in cache_ref.iter().rev() {
            if existing.source_polygon_profile != source_polygon_profile
                || existing.old_wnv != old_wnv
                || !polygon_families_match_as_multisets(&existing.source_polygons, source_polygons)
            {
                continue;
            }
            let Ok((point, definitions, winding)) = &existing.result else {
                continue;
            };
            let valid_for_bounds = cached_reference_bounds_validity_with_context(
                &mut query_caches.validity_cache,
                Some(&context),
                bounds,
                point,
                |point| is_certified_valid_reference_for_bounds(point, bounds, source_polygons),
            )?;
            if !valid_for_bounds {
                continue;
            }
            reused = Some((point.clone(), definitions.clone(), winding.clone()));
            break;
        }
    }
    if let Some(reused) = reused {
        cache_child_reference_result(
            cache,
            old_ref,
            old_ref_definitions,
            old_wnv,
            source_polygons,
            bounds,
            &reused,
        );
        return Ok(Some(reused));
    }
    Ok(None)
}

fn child_task_reference_is_certified_valid(
    task: &SubdivisionTask,
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<bool> {
    reference_is_certified_valid_for_task_bounds(
        &task.ref_point,
        &task.ref_definitions,
        &task.ref_wnv,
        &task.bounds,
        &task.polygons,
        query_caches,
    )
}

fn reference_is_certified_valid_for_task_bounds(
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<bool> {
    let context =
        support_reference_cache_context_key(ref_point, ref_definitions, ref_wnv, polygons);
    cached_reference_bounds_validity_with_context(
        &mut query_caches.validity_cache,
        Some(&context),
        bounds,
        ref_point,
        |point| is_certified_valid_reference_for_bounds(point, bounds, polygons),
    )
}

fn reusable_child_subdivision_if_certified(
    cache: &RefCell<Vec<ChildSubdivisionCacheEntry>>,
    task: &SubdivisionTask,
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<Option<Vec<ClassifiedPolygon>>> {
    if !child_task_reference_is_certified_valid(task, query_caches)? {
        return Ok(None);
    }

    let polygon_profile = polygon_family_profile(&task.polygons);
    let mut reused = None;
    {
        let cache_ref = cache.borrow();
        for existing in cache_ref.iter().rev() {
            if existing.polygon_profile != polygon_profile
                || existing.task.ref_wnv != task.ref_wnv
                || !polygon_families_match_as_multisets(&existing.task.polygons, &task.polygons)
                || existing.result.is_err()
                || existing.task.depth < task.depth
            {
                continue;
            }

            if !bounds_contains_bounds(&existing.task.bounds, &task.bounds)? {
                continue;
            }

            if reference_is_certified_valid_for_task_bounds(
                &existing.task.ref_point,
                &existing.task.ref_definitions,
                &existing.task.ref_wnv,
                &task.bounds,
                &task.polygons,
                query_caches,
            )? && let Ok(result) = &existing.result
            {
                reused = Some(result.clone());
                break;
            }
        }
    }
    if let Some(reused) = reused {
        cache_child_subdivision_result(cache, task, &Ok(reused.clone()));
        return Ok(Some(reused));
    }
    Ok(None)
}

fn reusable_child_subdivision_from_cached_trace_if_certified(
    cache: &RefCell<Vec<ChildSubdivisionCacheEntry>>,
    task: &SubdivisionTask,
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<Option<Vec<ClassifiedPolygon>>> {
    if !child_task_reference_is_certified_valid(task, query_caches)? {
        return Ok(None);
    }

    let polygon_profile = polygon_family_profile(&task.polygons);
    let context = support_reference_cache_context_key(
        &task.ref_point,
        &task.ref_definitions,
        &task.ref_wnv,
        &task.polygons,
    );
    let mut reused = None;
    {
        let cache_ref = cache.borrow();
        for existing in cache_ref.iter().rev() {
            if existing.polygon_profile != polygon_profile
                || !polygon_families_match_as_multisets(&existing.task.polygons, &task.polygons)
                || existing.result.is_err()
                || existing.task.depth < task.depth
            {
                continue;
            }

            if !bounds_contains_bounds(&existing.task.bounds, &task.bounds)? {
                continue;
            }

            if !reference_is_certified_valid_for_task_bounds(
                &existing.task.ref_point,
                &existing.task.ref_definitions,
                &existing.task.ref_wnv,
                &task.bounds,
                &task.polygons,
                query_caches,
            )? {
                continue;
            }

            let target = ReferenceTarget::with_definitions(
                existing.task.ref_point.clone(),
                existing.task.ref_definitions.clone(),
            );
            let Some(winding) = cached_reference_target_trace_with_context(
                &mut query_caches.trace_cache,
                Some(&context),
                &target,
                |target| {
                    trace_reference_target_from_validated_bounds(
                        &task.ref_point,
                        &task.ref_definitions,
                        &task.ref_wnv,
                        &task.bounds,
                        &task.polygons,
                        target,
                    )
                },
            )?
            else {
                continue;
            };
            if winding != existing.task.ref_wnv {
                continue;
            }

            if let Ok(result) = &existing.result {
                reused = Some(result.clone());
                break;
            }
        }
    }
    if let Some(reused) = reused {
        cache_child_subdivision_result(cache, task, &Ok(reused.clone()));
        return Ok(Some(reused));
    }
    Ok(None)
}

fn reusable_contracted_task_reference_from_cached_subdivision_if_certified(
    cache: &RefCell<Vec<ChildSubdivisionCacheEntry>>,
    task: &SubdivisionTask,
    contracted_bounds: &Aabb,
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<Option<(Point3, Vec<[Plane; 3]>, Vec<i32>)>> {
    let polygon_profile = polygon_family_profile(&task.polygons);
    let mut reused = None;
    {
        let cache_ref = cache.borrow();
        for existing in cache_ref.iter().rev() {
            if existing.polygon_profile != polygon_profile
                || existing.task.ref_wnv != task.ref_wnv
                || !polygon_families_match_as_multisets(&existing.task.polygons, &task.polygons)
                || existing.result.is_err()
            {
                continue;
            }

            if !bounds_contains_bounds(&existing.task.bounds, contracted_bounds)? {
                continue;
            }

            if reference_is_certified_valid_for_task_bounds(
                &existing.task.ref_point,
                &existing.task.ref_definitions,
                &existing.task.ref_wnv,
                contracted_bounds,
                &task.polygons,
                query_caches,
            )? {
                reused = Some(certified_reference_result((
                    existing.task.ref_point.clone(),
                    existing.task.ref_definitions.clone(),
                    existing.task.ref_wnv.clone(),
                )));
                break;
            }
        }
    }
    Ok(reused)
}

fn bounds_contains_bounds(outer: &Aabb, inner: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        if compare_real(axis_ref(&outer.min, axis), axis_ref(&inner.min, axis))?.is_gt()
            || compare_real(axis_ref(&outer.max, axis), axis_ref(&inner.max, axis))?.is_lt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn cache_child_subdivision_result(
    cache: &RefCell<Vec<ChildSubdivisionCacheEntry>>,
    task: &SubdivisionTask,
    result: &HypermeshResult<Vec<ClassifiedPolygon>>,
) {
    if cache.borrow().iter().any(|existing| existing.task == *task) {
        return;
    }

    cache.borrow_mut().push(ChildSubdivisionCacheEntry {
        polygon_profile: polygon_family_profile(&task.polygons),
        task: task.clone(),
        result: result.clone(),
    });
}

fn cached_child_subdivision_with(
    cache: &RefCell<Vec<ChildSubdivisionCacheEntry>>,
    task: &SubdivisionTask,
    query: impl FnOnce() -> HypermeshResult<Vec<ClassifiedPolygon>>,
) -> HypermeshResult<Vec<ClassifiedPolygon>> {
    if let Some(existing) = cache
        .borrow()
        .iter()
        .rev()
        .find(|existing| existing.task == *task)
        .cloned()
    {
        return existing.result;
    }

    let polygon_profile = polygon_family_profile(&task.polygons);
    let existing = cache
        .borrow()
        .iter()
        .rev()
        .find(|existing| {
            existing.polygon_profile == polygon_profile
                && subdivision_task_state_matches_for_cache(&existing.task, task)
                && (existing.task.depth == task.depth
                    || (existing.task.depth > task.depth && existing.result.is_ok()))
        })
        .cloned();
    if let Some(existing) = existing {
        if existing.task != *task {
            cache_child_subdivision_result(cache, task, &existing.result);
        }
        return existing.result;
    }

    let result = query();
    cache.borrow_mut().push(ChildSubdivisionCacheEntry {
        polygon_profile,
        task: task.clone(),
        result: result.clone(),
    });
    result
}

fn cached_winding_reachability_with(
    cache: &RefCell<Vec<WindingReachabilityCacheEntry>>,
    op: BooleanOp,
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    query: impl FnOnce() -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    let transition_profile = transition_family_profile(polygons);
    if let Some(existing) = cache.borrow().iter().find(|existing| {
        existing.op == op
            && existing.ref_wnv == ref_wnv
            && existing.transition_profile == transition_profile
    }) {
        return existing.result.clone();
    }

    let result = query();
    cache.borrow_mut().push(WindingReachabilityCacheEntry {
        op,
        ref_wnv: ref_wnv.to_vec(),
        transition_profile,
        result: result.clone(),
    });
    result
}

fn subdivision_task_state_matches_for_cache(
    left: &SubdivisionTask,
    right: &SubdivisionTask,
) -> bool {
    polygon_families_match_as_multisets(&left.polygons, &right.polygons)
        && left.bounds == right.bounds
        && left.ref_point == right.ref_point
        && reference_definition_families_match_as_sets(
            &left.ref_definitions,
            &right.ref_definitions,
        )
        && left.ref_wnv == right.ref_wnv
}

fn polygon_families_match_as_multisets(left: &[ConvexPolygon], right: &[ConvexPolygon]) -> bool {
    if left.len() != right.len() {
        return false;
    }

    let mut matched = vec![false; right.len()];
    for left_polygon in left {
        let Some((index, _)) = right
            .iter()
            .enumerate()
            .find(|(index, right_polygon)| !matched[*index] && *right_polygon == left_polygon)
        else {
            return false;
        };
        matched[index] = true;
    }

    true
}

fn transition_family_profile(polygons: &[ConvexPolygon]) -> Vec<Vec<i32>> {
    let mut profile = polygons
        .iter()
        .map(|polygon| polygon.delta_w.clone())
        .collect::<Vec<_>>();
    profile.sort_unstable();
    profile
}

fn polygon_family_profile(polygons: &[ConvexPolygon]) -> PolygonFamilyProfile {
    let mut profile = polygons
        .iter()
        .map(|polygon| {
            (
                polygon.mesh_index,
                polygon.polygon_index,
                polygon.edges.len(),
                polygon.delta_w.clone(),
            )
        })
        .collect::<Vec<_>>();
    profile.sort_unstable();
    profile
}

fn leaf_classification_cache_context_matches(
    left: Option<&Arc<LeafClassificationCacheContextKey>>,
    right: Option<&Arc<LeafClassificationCacheContextKey>>,
) -> bool {
    match (left, right) {
        (None, None) => true,
        (Some(left), Some(right)) if Arc::ptr_eq(left, right) => true,
        (Some(left), Some(right)) => {
            left.polygon_profile == right.polygon_profile
                && polygon_families_match_as_multisets(&left.polygons, &right.polygons)
                && left.bounds == right.bounds
                && left.ref_point == right.ref_point
                && reference_definition_families_match_as_sets(
                    &left.ref_definitions,
                    &right.ref_definitions,
                )
                && left.ref_wnv == right.ref_wnv
        }
        _ => false,
    }
}

fn process_leaf_task_into_with_caches(
    task: &SubdivisionTask,
    indicator: &Indicator,
    output: &mut Vec<ClassifiedPolygon>,
    pairwise_cache: &RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    host_bsp_cache: &RefCell<Vec<HostBspLeavesCacheEntry>>,
    bsp_leaf_cache: &RefCell<Vec<BspLeafCertificationCacheEntry>>,
    leaf_classification_cache: &RefCell<Vec<LeafClassificationCacheEntry>>,
    leaf_point_classification_cache: &RefCell<Vec<LeafPointClassificationCacheEntry>>,
) -> HypermeshResult<LeafProcessingStats> {
    let pairwise_query = |polygons: &[ConvexPolygon]| {
        cached_pairwise_intersections_by_polygon_with(pairwise_cache, polygons)
    };
    let bsp_leaves_query = |polygon: &ConvexPolygon,
                            polygons: &[ConvexPolygon],
                            intersections: &[PairwiseIntersection]| {
        cached_host_bsp_leaves_with(host_bsp_cache, polygon, polygons, intersections)
    };
    let bsp_leaf_query = |polygon: &ConvexPolygon,
                          leaf_edges: &[crate::geometry::Plane],
                          polygons: &[ConvexPolygon],
                          intersections: &[PairwiseIntersection]| {
        cached_bsp_leaf_certification_with(
            bsp_leaf_cache,
            polygon,
            leaf_edges,
            polygons,
            intersections,
        )
    };
    process_leaf_into_inner_with_pairwise_cache(
        &task.polygons,
        &task.bounds,
        &task.ref_point,
        &task.ref_definitions,
        &task.ref_wnv,
        indicator,
        output,
        leaf_classification_cache,
        leaf_point_classification_cache,
        pairwise_query,
        bsp_leaves_query,
        bsp_leaf_query,
    )
}

fn certified_leaf_output_if_complete_with(
    task: &SubdivisionTask,
    indicator: &Indicator,
    mut process_leaf: impl FnMut(
        &SubdivisionTask,
        &Indicator,
        &mut Vec<ClassifiedPolygon>,
    ) -> HypermeshResult<LeafProcessingStats>,
) -> HypermeshResult<Option<Vec<ClassifiedPolygon>>> {
    let mut certified_output = Vec::new();
    let stats = match process_leaf(task, indicator, &mut certified_output) {
        Ok(stats) => stats,
        Err(crate::error::HypermeshError::UnknownClassification) => return Ok(None),
        Err(err) => return Err(err),
    };
    if stats.certified_complete {
        Ok(Some(certified_output))
    } else {
        Ok(None)
    }
}

fn can_discard_by_winding_reachability(
    op: BooleanOp,
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    let mut lower = ref_wnv.to_vec();
    let mut upper = ref_wnv.to_vec();
    for polygon in polygons {
        if polygon.delta_w.len() != ref_wnv.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        for ((lower, upper), delta) in lower.iter_mut().zip(&mut upper).zip(&polygon.delta_w) {
            let span = delta
                .checked_abs()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            *lower = lower
                .checked_sub(span)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            *upper = upper
                .checked_add(span)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
        }
    }

    if !can_boolean_op_be_inside_with_component_ranges(op, &lower, &upper)? {
        return Ok(true);
    }

    let transitions = polygons
        .iter()
        .map(|polygon| polygon.delta_w.clone())
        .collect::<Vec<_>>();
    Ok(!can_boolean_op_be_inside_with_transition_reachability(
        op,
        ref_wnv,
        &transitions,
    )?)
}

fn emit_one_direct(
    polygon: &ConvexPolygon,
    bounds: &Aabb,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    class_polygons: &[ConvexPolygon],
    indicator: &Indicator,
    cache: &RefCell<Vec<LeafClassificationCacheEntry>>,
    context: Option<&Arc<LeafClassificationCacheContextKey>>,
    probe_query_caches: &mut LeafProbeQueryCaches,
    output: &mut Vec<ClassifiedPolygon>,
    output_buckets: &mut ClassifiedPolygonBucketState,
) -> HypermeshResult<bool> {
    let w_front = cached_leaf_classification_with(
        &mut cache.borrow_mut(),
        context,
        &polygon.support,
        &polygon.edges,
        &polygon.delta_w,
        || {
            classify_leaf_polygon_with_probe_query_caches(
                &polygon.support,
                &polygon.edges,
                ref_point,
                ref_definitions,
                ref_wnv,
                class_polygons,
                bounds,
                &polygon.delta_w,
                probe_query_caches,
            )
        },
    )?;
    let w_back = propagate_wnv(&w_front, 1, &polygon.delta_w)?;
    let classification = classify_polygon_output(&w_front, &w_back, indicator);
    if classification != 0 {
        let mut classified = ClassifiedPolygon::new(polygon.clone(), classification);
        classified.winding = Some(WindingPair { w_front, w_back });
        push_unique_classified_polygon_with_bucket_state(output, output_buckets, classified);
        return Ok(true);
    }
    Ok(false)
}

fn cached_leaf_classification_with(
    cache: &mut Vec<LeafClassificationCacheEntry>,
    context: Option<&Arc<LeafClassificationCacheContextKey>>,
    support: &Plane,
    edges: &[Plane],
    delta_w: &[i32],
    classify: impl FnOnce() -> HypermeshResult<WindingNumberVector>,
) -> HypermeshResult<WindingNumberVector> {
    if let Some(existing) = cache.iter().find(|existing| {
        leaf_classification_cache_context_matches(existing.context.as_ref(), context)
            && existing.support == *support
            && existing.delta_w == delta_w
            && edge_cycles_match_up_to_rotation(&existing.edges, edges)
    }) {
        return existing.winding.clone();
    }

    let winding = classify();
    cache.push(LeafClassificationCacheEntry {
        context: context.cloned(),
        support: support.clone(),
        edges: edges.to_vec(),
        delta_w: delta_w.to_vec(),
        winding: winding.clone(),
    });
    winding
}

fn cached_leaf_point_classification_with(
    cache: &mut Vec<LeafPointClassificationCacheEntry>,
    context: Option<&Arc<LeafClassificationCacheContextKey>>,
    support: &Plane,
    point: &crate::segment_trace::InteriorLeafPoint,
    delta_w: &[i32],
    classify: impl FnOnce() -> HypermeshResult<LeafPointClassificationState>,
) -> HypermeshResult<LeafPointClassificationState> {
    if let Some(existing) = cache.iter().find(|existing| {
        leaf_classification_cache_context_matches(existing.context.as_ref(), context)
            && existing.support == *support
            && existing.point == *point
            && existing.delta_w == delta_w
    }) {
        return existing.state.clone();
    }

    let state = classify();
    cache.push(LeafPointClassificationCacheEntry {
        context: context.cloned(),
        support: support.clone(),
        point: point.clone(),
        delta_w: delta_w.to_vec(),
        state: state.clone(),
    });
    state
}

fn classify_leaf_polygon_from_interior_points_with_point_cache(
    interior_points: &[crate::segment_trace::InteriorLeafPoint],
    support: &Plane,
    ref_point: &Point3,
    ref_definitions: &[[Plane; 3]],
    ref_wnv: &[i32],
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    host_delta_w: &[i32],
    point_cache: &mut Vec<LeafPointClassificationCacheEntry>,
    context: Option<&Arc<LeafClassificationCacheContextKey>>,
) -> HypermeshResult<WindingNumberVector> {
    let mut saw_unknown = false;

    for point in ordered_interior_points_for_probe_search_with_support(interior_points, support)? {
        let state = cached_leaf_point_classification_with(
            point_cache,
            context,
            support,
            point,
            host_delta_w,
            || {
                let mut probe_query_caches = LeafProbeQueryCaches::default();
                let mut local_unknown = false;
                let winding = classify_leaf_polygon_interior_point_with_probe_query_caches(
                    point,
                    support,
                    ref_point,
                    ref_definitions,
                    ref_wnv,
                    polygons,
                    bounds,
                    host_delta_w,
                    &mut probe_query_caches,
                    &mut local_unknown,
                )?;
                Ok(LeafPointClassificationState {
                    winding,
                    saw_unknown: local_unknown,
                })
            },
        )?;
        saw_unknown |= state.saw_unknown;
        if let Some(winding) = state.winding {
            return Ok(winding);
        }
    }

    let _ = saw_unknown;
    Err(crate::error::HypermeshError::UnknownClassification)
}

pub(crate) fn build_host_bsp_leaves(
    polygon: &ConvexPolygon,
    polygons: &[ConvexPolygon],
    intersections: &[PairwiseIntersection],
) -> HypermeshResult<Vec<BspLeaf>> {
    let mut bsp = LocalBsp::new(polygon);
    bsp.add_overlap_edges(&unique_overlap_edge_planes(intersections))?;
    for intersection in intersections {
        match intersection.kind {
            PairwiseIntersectionType::Segment => {
                if let Some(segment) = &intersection.segment {
                    bsp.add_segment(segment)?;
                }
            }
            PairwiseIntersectionType::Overlap => {
                if let Some(overlap) = &intersection.overlap {
                    bsp.mark_overlap(&polygons[overlap.other_polygon_idx])?;
                }
            }
            PairwiseIntersectionType::None | PairwiseIntersectionType::Point => {}
        }
    }
    Ok(bsp.collect_leaves().into_iter().cloned().collect())
}

fn cached_host_bsp_leaves_with(
    cache: &RefCell<Vec<HostBspLeavesCacheEntry>>,
    polygon: &ConvexPolygon,
    polygons: &[ConvexPolygon],
    intersections: &[PairwiseIntersection],
) -> HypermeshResult<Vec<BspLeaf>> {
    let polygon_profile = polygon_family_profile(polygons);
    if let Some(existing) = cache.borrow().iter().find(|existing| {
        existing.host == *polygon
            && existing.polygon_profile == polygon_profile
            && polygon_families_match_as_multisets(&existing.polygons, polygons)
    }) {
        return existing.leaves.clone();
    }

    let leaves = build_host_bsp_leaves(polygon, polygons, intersections);
    cache.borrow_mut().push(HostBspLeavesCacheEntry {
        host: polygon.clone(),
        polygon_profile,
        polygons: polygons.to_vec(),
        leaves: leaves.clone(),
    });
    leaves
}

fn cached_bsp_leaf_certification_with(
    cache: &RefCell<Vec<BspLeafCertificationCacheEntry>>,
    host: &ConvexPolygon,
    leaf_edges: &[crate::geometry::Plane],
    polygons: &[ConvexPolygon],
    intersections: &[PairwiseIntersection],
) -> HypermeshResult<(Vec<crate::segment_trace::InteriorLeafPoint>, Vec<i32>)> {
    let polygon_profile = polygon_family_profile(polygons);
    if let Some(existing) = cache.borrow().iter().find(|existing| {
        existing.host == *host
            && edge_cycles_match_up_to_rotation(&existing.leaf_edges, leaf_edges)
            && existing.polygon_profile == polygon_profile
            && polygon_families_match_as_multisets(&existing.polygons, polygons)
    }) {
        return existing.result.clone();
    }

    let result = certify_bsp_leaf_and_delta_w_with_host_intersections(
        host,
        leaf_edges,
        polygons,
        Some(intersections),
    );
    cache.borrow_mut().push(BspLeafCertificationCacheEntry {
        host: host.clone(),
        leaf_edges: leaf_edges.to_vec(),
        polygon_profile,
        polygons: polygons.to_vec(),
        result: result.clone(),
    });
    result
}

fn take_new_bsp_leaf_edge_cycle(seen: &mut Vec<Vec<Plane>>, candidate: &[Plane]) -> bool {
    if seen
        .iter()
        .any(|existing| edge_cycles_match_up_to_rotation(existing, candidate))
    {
        return false;
    }
    seen.push(candidate.to_vec());
    true
}

fn edge_cycles_match_up_to_rotation(left: &[Plane], right: &[Plane]) -> bool {
    if left.len() != right.len() {
        return false;
    }
    if left.is_empty() {
        return true;
    }

    for offset in 0..left.len() {
        let mut all_match = true;
        for index in 0..left.len() {
            if left[index] != right[(index + offset) % right.len()] {
                all_match = false;
                break;
            }
        }
        if all_match {
            return true;
        }
    }

    false
}

#[cfg(test)]
pub(crate) fn certify_bsp_leaf_and_delta_w(
    polygon: &ConvexPolygon,
    leaf_edges: &[crate::geometry::Plane],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<(Vec<crate::segment_trace::InteriorLeafPoint>, Vec<i32>)> {
    certify_bsp_leaf_and_delta_w_with_host_intersections(polygon, leaf_edges, polygons, None)
}

fn certify_bsp_leaf_and_delta_w_with_host_intersections(
    polygon: &ConvexPolygon,
    leaf_edges: &[crate::geometry::Plane],
    polygons: &[ConvexPolygon],
    host_intersections: Option<&[PairwiseIntersection]>,
) -> HypermeshResult<(Vec<crate::segment_trace::InteriorLeafPoint>, Vec<i32>)> {
    let leaf_polygon = ConvexPolygon {
        support: polygon.support.clone(),
        edges: leaf_edges.to_vec(),
        mesh_index: polygon.mesh_index,
        polygon_index: polygon.polygon_index,
        delta_w: polygon.delta_w.clone(),
        approx_bounds: None,
    };
    let interior_points =
        certified_leaf_interior_points(&leaf_polygon.support, &leaf_polygon.edges)?;
    if interior_points.is_empty() {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    let leaf_test_points = interior_points
        .iter()
        .map(|point| {
            HomogeneousPoint3::new(
                point.point.x.clone(),
                point.point.y.clone(),
                point.point.z.clone(),
                Real::one(),
            )
        })
        .collect::<Vec<_>>();
    let mut delta_w = polygon.delta_w.clone();

    for other_index in
        bsp_leaf_certification_candidate_indices(polygon, polygons, host_intersections)?
    {
        let other = &polygons[other_index];
        if delta_w.len() != other.delta_w.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let relation = classify_leaf_test_relation(&leaf_test_points, other)?;
        let intersection = intersect_polygons(&leaf_polygon, other, 0)?;
        match intersection.kind {
            PairwiseIntersectionType::None | PairwiseIntersectionType::Point => {}
            PairwiseIntersectionType::Segment => {
                let Some(segment) = intersection.segment else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                if segment_has_strict_interior_point_in_both(
                    &segment.v0,
                    &segment.v1,
                    &leaf_polygon,
                    other,
                )? {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
            }
            PairwiseIntersectionType::Overlap => {
                let Some(strictly_inside) = relation else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                if leaf_polygon_key(polygon) > leaf_polygon_key(other) && strictly_inside {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
            }
        }

        let Some(strictly_inside) = relation else {
            return Err(crate::error::HypermeshError::UnknownClassification);
        };
        if strictly_inside {
            let sign = if supports_have_same_direction(&polygon.support, &other.support)? {
                1
            } else {
                -1
            };
            for (value, delta) in delta_w.iter_mut().zip(&other.delta_w) {
                *value += sign * *delta;
            }
        }
    }

    Ok((interior_points, delta_w))
}

fn bsp_leaf_certification_candidate_indices(
    polygon: &ConvexPolygon,
    polygons: &[ConvexPolygon],
    host_intersections: Option<&[PairwiseIntersection]>,
) -> HypermeshResult<Vec<usize>> {
    if let Some(host_intersections) = host_intersections {
        let mut indices = Vec::new();
        for intersection in host_intersections {
            let other_index = pairwise_intersection_other_polygon_idx(intersection)?;
            if polygons.get(other_index).is_some_and(|other| {
                other.mesh_index == polygon.mesh_index
                    && other.polygon_index == polygon.polygon_index
            }) {
                continue;
            }
            if !indices.contains(&other_index) {
                indices.push(other_index);
            }
        }
        return Ok(indices);
    }

    Ok(polygons
        .iter()
        .enumerate()
        .filter_map(|(index, other)| {
            ((other.mesh_index != polygon.mesh_index)
                || (other.polygon_index != polygon.polygon_index))
                .then_some(index)
        })
        .collect())
}

fn pairwise_intersection_other_polygon_idx(
    intersection: &PairwiseIntersection,
) -> HypermeshResult<usize> {
    match intersection.kind {
        PairwiseIntersectionType::Segment => intersection
            .segment
            .as_ref()
            .map(|segment| segment.other_polygon_idx)
            .ok_or(crate::error::HypermeshError::UnknownClassification),
        PairwiseIntersectionType::Overlap => intersection
            .overlap
            .as_ref()
            .map(|overlap| overlap.other_polygon_idx)
            .ok_or(crate::error::HypermeshError::UnknownClassification),
        PairwiseIntersectionType::None | PairwiseIntersectionType::Point => {
            Err(crate::error::HypermeshError::UnknownClassification)
        }
    }
}

fn classify_leaf_test_relation(
    test_points: &[HomogeneousPoint3],
    polygon: &ConvexPolygon,
) -> HypermeshResult<Option<bool>> {
    let mut any_inside = false;
    let mut any_outside = false;

    for test_point in test_points {
        let inside_or_on = polygon.contains_point(test_point)?;
        let strictly_inside = polygon.contains_point_strictly(test_point)?;
        if strictly_inside {
            any_inside = true;
        } else if !inside_or_on {
            any_outside = true;
        }
    }

    if any_inside && any_outside {
        Ok(None)
    } else if any_inside {
        Ok(Some(true))
    } else if any_outside {
        Ok(Some(false))
    } else {
        Ok(None)
    }
}

fn supports_have_same_direction(
    left: &crate::geometry::Plane,
    right: &crate::geometry::Plane,
) -> HypermeshResult<bool> {
    let dot = (&left.normal.x * &right.normal.x)
        + (&left.normal.y * &right.normal.y)
        + (&left.normal.z * &right.normal.z);
    Ok(crate::geometry::classify_real(&dot)? != Classification::Negative)
}

#[cfg(test)]
fn certify_bsp_leaf_has_no_interior_intersections(
    host: &ConvexPolygon,
    leaf_edges: &[crate::geometry::Plane],
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    match certify_bsp_leaf_and_delta_w(host, leaf_edges, polygons) {
        Ok((_interior_points, _delta_w)) => Ok(true),
        Err(crate::error::HypermeshError::UnknownClassification) => Ok(false),
        Err(err) => Err(err),
    }
}

fn segment_has_strict_interior_point_in_both(
    a: &Point3,
    b: &Point3,
    left: &ConvexPolygon,
    right: &ConvexPolygon,
) -> HypermeshResult<bool> {
    let mut lower = Real::zero();
    let mut upper = Real::one();
    Ok(
        constrain_open_segment_interval_to_polygon(a, b, left, &mut lower, &mut upper)?
            && constrain_open_segment_interval_to_polygon(a, b, right, &mut lower, &mut upper)?
            && compare_real(&lower, &upper)?.is_lt(),
    )
}

fn constrain_open_segment_interval_to_polygon(
    a: &Point3,
    b: &Point3,
    polygon: &ConvexPolygon,
    lower: &mut Real,
    upper: &mut Real,
) -> HypermeshResult<bool> {
    for edge in &polygon.edges {
        if !constrain_open_segment_interval_to_plane_negative(a, b, edge, lower, upper)? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn constrain_open_segment_interval_to_plane_negative(
    a: &Point3,
    b: &Point3,
    plane: &Plane,
    lower: &mut Real,
    upper: &mut Real,
) -> HypermeshResult<bool> {
    let start = plane.expression_at_point(a);
    let end = plane.expression_at_point(b);
    let start_class = classify_real(&start)?;
    let end_class = classify_real(&end)?;

    match (start_class, end_class) {
        (Classification::Negative, Classification::Negative) => Ok(true),
        (Classification::Negative, Classification::On) => Ok(true),
        (Classification::On, Classification::Negative) => Ok(true),
        (Classification::Positive, Classification::Negative) => {
            let cut = (start.clone() / (&start - &end))
                .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
            update_open_segment_lower(lower, &cut)
        }
        (Classification::Negative, Classification::Positive) => {
            let cut = (start.clone() / (&start - &end))
                .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
            update_open_segment_upper(upper, &cut)
        }
        (Classification::On, Classification::On)
        | (Classification::Positive, Classification::Positive)
        | (Classification::Positive, Classification::On)
        | (Classification::On, Classification::Positive) => Ok(false),
    }
}

fn update_open_segment_lower(lower: &mut Real, candidate: &Real) -> HypermeshResult<bool> {
    if compare_real(candidate, lower)?.is_gt() {
        *lower = candidate.clone();
    }
    Ok(compare_real(lower, &Real::one())?.is_lt())
}

fn update_open_segment_upper(upper: &mut Real, candidate: &Real) -> HypermeshResult<bool> {
    if compare_real(candidate, upper)?.is_lt() {
        *upper = candidate.clone();
    }
    Ok(compare_real(&Real::zero(), upper)?.is_lt())
}

fn leaf_polygon_key(polygon: &ConvexPolygon) -> (isize, isize) {
    (polygon.mesh_index, polygon.polygon_index)
}

fn push_unique_overlap_edge_plane(edges: &mut Vec<Plane>, candidate: &Plane) {
    if edges
        .iter()
        .any(|existing| existing == candidate || existing == &candidate.inverted())
    {
        return;
    }
    edges.push(candidate.clone());
}

fn unique_overlap_edge_planes(intersections: &[PairwiseIntersection]) -> Vec<Plane> {
    let mut edges = Vec::new();
    for intersection in intersections {
        if let Some(overlap) = &intersection.overlap {
            for edge in &overlap.other_edges {
                push_unique_overlap_edge_plane(&mut edges, edge);
            }
        }
    }
    edges
}

fn pairwise_intersections_by_polygon(
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<Vec<PairwiseIntersection>>> {
    let mut by_polygon = vec![Vec::new(); polygons.len()];
    let bvh = ExactBvh::build(polygons)?;
    let mut candidate_pairs = Vec::new();
    bvh.intersect_pairs(&bvh, |left, right| {
        if left < right {
            candidate_pairs.push((left, right));
        }
    })?;

    for (global_i, global_j) in candidate_pairs {
        let intersection = intersect_polygons(&polygons[global_i], &polygons[global_j], global_j)?;
        if matches!(
            intersection.kind,
            PairwiseIntersectionType::Segment | PairwiseIntersectionType::Overlap
        ) {
            by_polygon[global_i].push(intersection);
        }

        let intersection = intersect_polygons(&polygons[global_j], &polygons[global_i], global_i)?;
        if matches!(
            intersection.kind,
            PairwiseIntersectionType::Segment | PairwiseIntersectionType::Overlap
        ) {
            by_polygon[global_j].push(intersection);
        }
    }

    Ok(by_polygon)
}

fn cached_pairwise_intersections_by_polygon_with(
    cache: &RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<Vec<PairwiseIntersection>>> {
    if let Some(existing) = cache
        .borrow()
        .iter()
        .rev()
        .find(|existing| existing.polygons == polygons)
        .cloned()
    {
        return existing.result;
    }

    let polygon_profile = polygon_family_profile(polygons);
    let existing = cache
        .borrow()
        .iter()
        .rev()
        .find(|existing| {
            existing.polygon_profile == polygon_profile
                && (existing.polygons == polygons
                    || polygon_family_order_mapping(polygons, &existing.polygons).is_some())
        })
        .cloned();
    if let Some(existing) = existing {
        if existing.polygons == polygons {
            return existing.result;
        }
        if let Some(query_to_cached) = polygon_family_order_mapping(polygons, &existing.polygons) {
            let remapped =
                remap_pairwise_intersections_for_polygon_order(existing.result, &query_to_cached);
            if remapped.is_ok() {
                cache_pairwise_intersections_result(cache, polygons, &remapped);
            }
            return remapped;
        }
    }

    let result = pairwise_intersections_by_polygon(polygons);
    cache.borrow_mut().push(PairwiseIntersectionsCacheEntry {
        polygon_profile,
        polygons: polygons.to_vec(),
        result: result.clone(),
    });
    result
}

fn cache_pairwise_intersections_result(
    cache: &RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    polygons: &[ConvexPolygon],
    result: &HypermeshResult<Vec<Vec<PairwiseIntersection>>>,
) {
    if cache
        .borrow()
        .iter()
        .any(|existing| existing.polygons == polygons)
    {
        return;
    }

    cache.borrow_mut().push(PairwiseIntersectionsCacheEntry {
        polygon_profile: polygon_family_profile(polygons),
        polygons: polygons.to_vec(),
        result: result.clone(),
    });
}

fn polygon_family_order_mapping(
    query_polygons: &[ConvexPolygon],
    cached_polygons: &[ConvexPolygon],
) -> Option<Vec<usize>> {
    if query_polygons.len() != cached_polygons.len() {
        return None;
    }

    let mut cached_used = vec![false; cached_polygons.len()];
    let mut query_to_cached = Vec::with_capacity(query_polygons.len());
    for query_polygon in query_polygons {
        let (cached_index, _) =
            cached_polygons
                .iter()
                .enumerate()
                .find(|(cached_index, cached_polygon)| {
                    !cached_used[*cached_index] && *cached_polygon == query_polygon
                })?;
        cached_used[cached_index] = true;
        query_to_cached.push(cached_index);
    }

    Some(query_to_cached)
}

fn remap_pairwise_intersections_for_polygon_order(
    intersections: HypermeshResult<Vec<Vec<PairwiseIntersection>>>,
    query_to_cached: &[usize],
) -> HypermeshResult<Vec<Vec<PairwiseIntersection>>> {
    let intersections = intersections?;
    if intersections.len() != query_to_cached.len() {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }

    let mut cached_to_query = vec![usize::MAX; query_to_cached.len()];
    for (query_index, &cached_index) in query_to_cached.iter().enumerate() {
        if cached_index >= cached_to_query.len() || cached_to_query[cached_index] != usize::MAX {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        cached_to_query[cached_index] = query_index;
    }

    let mut remapped = Vec::with_capacity(query_to_cached.len());
    for &cached_index in query_to_cached {
        let mut query_intersections = Vec::with_capacity(intersections[cached_index].len());
        for intersection in &intersections[cached_index] {
            let mut remapped_intersection = intersection.clone();
            if let Some(segment) = &mut remapped_intersection.segment {
                if segment.other_polygon_idx >= cached_to_query.len() {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
                segment.other_polygon_idx = cached_to_query[segment.other_polygon_idx];
            }
            if let Some(overlap) = &mut remapped_intersection.overlap {
                if overlap.other_polygon_idx >= cached_to_query.len() {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
                overlap.other_polygon_idx = cached_to_query[overlap.other_polygon_idx];
            }
            query_intersections.push(remapped_intersection);
        }
        remapped.push(query_intersections);
    }

    Ok(remapped)
}

fn normalize_surface_reference(
    old_ref: &Point3,
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<(Point3, Vec<[Plane; 3]>, Vec<i32>)>> {
    if !bounds.contains_point(old_ref)? {
        return Ok(None);
    }

    let mut incident = Vec::new();
    let mut supports_through_reference = Vec::new();
    let mut has_boundary_contact = false;
    for polygon in polygons {
        if crate::geometry::classify_point(old_ref, &polygon.support)? == Classification::On {
            supports_through_reference.push(polygon);
        }
        match classify_point_in_local_polygon(old_ref, polygon)? {
            LocalPolygonPointLocation::Outside => {}
            LocalPolygonPointLocation::Boundary => {
                has_boundary_contact = true;
                incident.push(polygon);
            }
            LocalPolygonPointLocation::Interior => incident.push(polygon),
        }
    }
    if incident.is_empty() && !has_boundary_contact {
        return Ok(None);
    }
    if !polygon_family_is_closed_within_bounds(polygons, bounds, old_wnv.len())? {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }

    if !has_boundary_contact {
        let base = incident[0];
        let mut parallel = true;
        for polygon in incident.iter().skip(1) {
            match supports_have_parallel_normals(&base.support, &polygon.support) {
                Ok(true) => {}
                Ok(false) | Err(crate::error::HypermeshError::UnknownClassification) => {
                    parallel = false;
                    break;
                }
                Err(err) => return Err(err),
            }
        }

        if parallel {
            let exterior = exterior_reference_point(bounds)?;
            let exterior_definitions = vec![axis_plane_definition(&exterior)];
            let exterior_winding = vec![0; old_wnv.len()];
            let trace_bounds = reference_trace_bounds(&exterior, bounds)?;
            for positive_side in [true, false] {
                let direction = if positive_side {
                    base.support.normal.clone()
                } else {
                    Point3::new(
                        -base.support.normal.x.clone(),
                        -base.support.normal.y.clone(),
                        -base.support.normal.z.clone(),
                    )
                };
                let Some(point) =
                    surface_reference_departure_point(old_ref, &direction, bounds, polygons)?
                else {
                    continue;
                };
                match is_certified_valid_reference_for_bounds(&point, bounds, polygons) {
                    Ok(true) => {}
                    Ok(false) | Err(crate::error::HypermeshError::UnknownClassification) => {
                        continue;
                    }
                    Err(err) => return Err(err),
                }

                let definitions = vec![axis_plane_definition(&point)];
                let winding = match trace_segment_from_definitions_with_step_detoured_plane_replacement_in_bounds(
                        &exterior,
                        &point,
                        &exterior_winding,
                        polygons,
                        &exterior_definitions,
                        &definitions,
                        &trace_bounds,
                    ) {
                        Ok(winding) => winding,
                        Err(crate::error::HypermeshError::UnknownClassification) => continue,
                        Err(err) => return Err(err),
                    };
                if winding == old_wnv {
                    return Ok(Some((point, definitions, winding)));
                }
            }
        }
    }

    match closed_family_adjacent_reference_with_winding(
        old_ref,
        old_wnv,
        bounds,
        polygons,
        &supports_through_reference,
    )? {
        Some(reference) => Ok(Some(reference)),
        None => Err(crate::error::HypermeshError::ReferencePropagationFailed),
    }
}

fn polygon_family_is_closed_within_bounds(
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    expected_mesh_count: usize,
) -> HypermeshResult<bool> {
    let mut mesh_edges = vec![Vec::new(); expected_mesh_count];
    for polygon in polygons {
        let Ok(mesh_index) = usize::try_from(polygon.mesh_index) else {
            return Ok(false);
        };
        let Some(edges) = mesh_edges.get_mut(mesh_index) else {
            return Ok(false);
        };
        let vertices = polygon.vertices()?;
        if vertices.len() < 3 {
            return Ok(false);
        }
        for vertex in &vertices {
            if !bounds.contains_point(vertex)? {
                return Ok(false);
            }
        }
        for index in 0..vertices.len() {
            edges.push([
                vertices[index].clone(),
                vertices[(index + 1) % vertices.len()].clone(),
            ]);
        }
    }
    if mesh_edges.iter().any(Vec::is_empty) {
        return Ok(false);
    }

    for edges in &mesh_edges {
        let balance = classify_edge_balance(edges);
        if balance.boundary_edges != 0 || balance.unbalanced_edges != 0 {
            return Ok(false);
        }
    }
    Ok(true)
}

fn closed_family_adjacent_reference_with_winding(
    surface_point: &Point3,
    required_winding: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    supports_through_reference: &[&ConvexPolygon],
) -> HypermeshResult<Option<(Point3, Vec<[Plane; 3]>, Vec<i32>)>> {
    let exterior = exterior_reference_point(bounds)?;
    let exterior_definitions = vec![axis_plane_definition(&exterior)];
    let exterior_winding = vec![0; required_winding.len()];
    let trace_bounds = reference_trace_bounds(&exterior, bounds)?;
    let direction_bounds = Aabb::new(
        Point3::new(-Real::one(), -Real::one(), -Real::one()),
        Point3::new(Real::one(), Real::one(), Real::one()),
    );
    let direction_supports = supports_through_reference
        .iter()
        .map(|polygon| {
            let mut direction_support = (*polygon).clone();
            direction_support.support = Plane::new(polygon.support.normal.clone(), Real::zero());
            direction_support
        })
        .collect::<Vec<_>>();
    let mut halfspaces = aabb_core_halfspaces(&direction_bounds)?;
    let search_cache = std::cell::RefCell::new(Vec::new());

    let mut accept = |cell_halfspaces: &[LimitPlane3],
                      report: Option<hyperlimit::HalfspaceFeasibilityReport>|
     -> HypermeshResult<Option<(Point3, Vec<[Plane; 3]>, Vec<i32>)>> {
        if advance_fixed_support_search_index(&direction_supports, 0, cell_halfspaces)
            < direction_supports.len()
        {
            return Ok(None);
        }
        let Some(report) = report else {
            return Ok(None);
        };
        if report.status != HalfspaceFeasibility::Feasible {
            return Ok(None);
        }

        let mut directions = Vec::new();
        let mut saw_unknown = false;
        if let Some(witness) = report.witness {
            match point_strictly_inside_support_cell(&witness, &direction_bounds, cell_halfspaces) {
                Ok(true) => push_unique_point3(&mut directions, witness),
                Ok(false) => {}
                Err(crate::error::HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                }
                Err(err) => return Err(err),
            }
        }
        // The all-vertex centroid is strict for a full-dimensional bounded
        // convex cell; replay below certifies that condition before use.
        let vertex_family = match feasible_support_cell_vertex_family(cell_halfspaces) {
            Ok(family) => family,
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                Point3FamilyState {
                    points: Vec::new(),
                    saw_unknown: true,
                }
            }
            Err(err) => return Err(err),
        };
        saw_unknown |= vertex_family.saw_unknown;
        match point3_centroid(&vertex_family.points) {
            Ok(Some(center)) => {
                match point_strictly_inside_support_cell(
                    &center,
                    &direction_bounds,
                    cell_halfspaces,
                ) {
                    Ok(true) => push_unique_point3(&mut directions, center),
                    Ok(false) => {}
                    Err(crate::error::HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                    }
                    Err(err) => return Err(err),
                }
            }
            Ok(None) => {}
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }

        let mut certified_cell = false;
        for direction in directions {
            let Some(point) =
                surface_reference_departure_point(surface_point, &direction, bounds, polygons)?
            else {
                continue;
            };
            match is_certified_valid_reference_for_bounds(&point, bounds, polygons) {
                Ok(true) => {}
                Ok(false) => continue,
                Err(crate::error::HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    continue;
                }
                Err(err) => return Err(err),
            }
            let definitions = vec![axis_plane_definition(&point)];
            let winding =
                match trace_segment_from_definitions_with_step_detoured_plane_replacement_in_bounds(
                    &exterior,
                    &point,
                    &exterior_winding,
                    polygons,
                    &exterior_definitions,
                    &definitions,
                    &trace_bounds,
                ) {
                    Ok(winding) => winding,
                    Err(crate::error::HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                        continue;
                    }
                    Err(err) => return Err(err),
                };
            certified_cell = true;
            if winding == required_winding {
                return Ok(Some((point, definitions, winding)));
            }
        }
        if certified_cell {
            Ok(None)
        } else if saw_unknown {
            Err(crate::error::HypermeshError::UnknownClassification)
        } else {
            Ok(None)
        }
    };

    support_plane_cell_search_with_queries_cached(
        None,
        None,
        &direction_bounds,
        &direction_supports,
        0,
        &mut halfspaces,
        &mut |halfspaces| halfspace_system_report(halfspaces),
        &mut |halfspaces| halfspace_system_is_feasible(halfspaces),
        &mut accept,
        &search_cache,
    )
}

fn exterior_reference_point(bounds: &Aabb) -> HypermeshResult<Point3> {
    let mut point = bounds.max.clone();
    for axis in 0..3 {
        let span = axis_ref(&bounds.max, axis) - axis_ref(&bounds.min, axis);
        let margin = if compare_real(&span, &Real::zero())?.is_gt() {
            span
        } else {
            Real::one()
        };
        *axis_mut(&mut point, axis) += margin;
    }
    Ok(point)
}

fn supports_have_parallel_normals(left: &Plane, right: &Plane) -> HypermeshResult<bool> {
    let cross = [
        (&left.normal.y * &right.normal.z) - (&left.normal.z * &right.normal.y),
        (&left.normal.z * &right.normal.x) - (&left.normal.x * &right.normal.z),
        (&left.normal.x * &right.normal.y) - (&left.normal.y * &right.normal.x),
    ];
    for component in &cross {
        if classify_real(component)? != Classification::On {
            return Ok(false);
        }
    }
    Ok(true)
}

fn surface_reference_departure_point(
    start: &Point3,
    direction: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<Point3>> {
    let Some(mut stop) = ray_bounds_stop(start, direction, bounds)? else {
        return Ok(None);
    };

    for polygon in polygons {
        let start_value = polygon.support.expression_at_point(start);
        let denom = (&polygon.support.normal.x * &direction.x)
            + (&polygon.support.normal.y * &direction.y)
            + (&polygon.support.normal.z * &direction.z);
        if classify_real(&denom)? == Classification::On {
            continue;
        }
        let crossing_t = ((-start_value) / denom)
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
        if !compare_real(&crossing_t, &Real::zero())?.is_gt()
            || !compare_real(&crossing_t, &stop)?.is_lt()
        {
            continue;
        }
        let crossing = offset_reference_point(start, direction, &crossing_t);
        match classify_point_in_local_polygon(&crossing, polygon)? {
            LocalPolygonPointLocation::Outside => {}
            LocalPolygonPointLocation::Boundary | LocalPolygonPointLocation::Interior => {
                stop = crossing_t;
            }
        }
    }

    let half = (Real::one() / Real::from(2))
        .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
    Ok(Some(offset_reference_point(
        start,
        direction,
        &(stop * half),
    )))
}

fn ray_bounds_stop(
    start: &Point3,
    direction: &Point3,
    bounds: &Aabb,
) -> HypermeshResult<Option<Real>> {
    let mut stop: Option<Real> = None;
    for axis in 0..3 {
        let direction_value = axis_ref(direction, axis);
        let boundary = match classify_real(direction_value)? {
            Classification::Positive => axis_ref(&bounds.max, axis),
            Classification::Negative => axis_ref(&bounds.min, axis),
            Classification::On => continue,
        };
        let candidate = ((boundary - axis_ref(start, axis)) / direction_value)
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
        if !compare_real(&candidate, &Real::zero())?.is_gt() {
            return Ok(None);
        }
        let replace = match &stop {
            Some(current) => compare_real(&candidate, current)?.is_lt(),
            None => true,
        };
        if replace {
            stop = Some(candidate);
        }
    }
    Ok(stop)
}

fn offset_reference_point(point: &Point3, direction: &Point3, amount: &Real) -> Point3 {
    Point3::new(
        &point.x + &(amount * &direction.x),
        &point.y + &(amount * &direction.y),
        &point.z + &(amount * &direction.z),
    )
}

#[cfg(test)]
fn compute_new_reference(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<(Point3, Vec<[Plane; 3]>, Vec<i32>)> {
    let mut query_caches = SupportReferenceQueryCaches::default();
    compute_new_reference_with_query_caches(
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        &mut query_caches,
    )
}

fn compute_new_reference_with_query_caches(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<(Point3, Vec<[Plane; 3]>, Vec<i32>)> {
    query_caches.reset_per_reference_call_caches();
    let query_caches = std::cell::RefCell::new(query_caches);

    let old_ref_unknown = match is_certified_valid_reference_for_bounds(old_ref, bounds, polygons) {
        Ok(true) => {
            return Ok(certified_reference_result((
                old_ref.clone(),
                old_ref_definitions.to_vec(),
                old_wnv.to_vec(),
            )));
        }
        Ok(false) => false,
        Err(crate::error::HypermeshError::UnknownClassification) => true,
        Err(err) => return Err(err),
    };

    let surface_departure_unknown =
        match normalize_surface_reference(old_ref, old_wnv, bounds, polygons) {
            Ok(Some(reference)) => return Ok(reference),
            Ok(None) => false,
            Err(crate::error::HypermeshError::UnknownClassification) => true,
            Err(err) => return Err(err),
        };

    let support_unknown = match support_plane_cell_reference_with_query_caches(
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        &mut query_caches.borrow_mut(),
    ) {
        Ok(Some((target, winding))) => {
            return Ok(certified_reference_result((
                target.point,
                target.definitions,
                winding,
            )));
        }
        Ok(None) => false,
        Err(crate::error::HypermeshError::UnknownClassification) => true,
        Err(err) => return Err(err),
    };

    let projected_halfspaces = projected_reference_halfspaces(old_ref, bounds)?;
    let projected_root = {
        let mut query_caches = query_caches.borrow_mut();
        cached_projected_root_reference_families_with(
            bounds,
            &projected_halfspaces,
            &mut query_caches,
        )?
    };
    {
        let mut query_caches = query_caches.borrow_mut();
        prime_support_reference_query_caches_with_known_halfspace_report(
            &mut query_caches,
            &projected_halfspaces,
            projected_root.report.as_ref(),
            projected_root.saw_unknown,
        );
    }
    let mut projected_unknown = projected_root.saw_unknown || old_ref_unknown;
    let cache_context =
        support_reference_cache_context_key(old_ref, old_ref_definitions, old_wnv, polygons);

    let projected = projected_reference_search_or_none_tracking_unknown(
        if let Some(reused) = {
            let mut query_caches = query_caches.borrow_mut();
            let SupportReferenceQueryCaches {
                projected_reference_result_cache,
                validity_cache,
                trace_cache,
                ..
            } = &mut *query_caches;
            if let Some(reused) = reusable_projected_reference_result_if_certified(
                projected_reference_result_cache,
                old_ref,
                old_ref_definitions,
                old_wnv,
                bounds,
                polygons,
                &projected_halfspaces,
                validity_cache,
            )? {
                Some(reused)
            } else {
                reusable_projected_reference_result_from_cached_trace_if_certified(
                    projected_reference_result_cache,
                    old_ref,
                    old_ref_definitions,
                    old_wnv,
                    bounds,
                    polygons,
                    &projected_halfspaces,
                    validity_cache,
                    trace_cache,
                )?
            }
        } {
            Ok(Some(reused))
        } else {
            let mut projected_reference_result_cache = {
                let mut query_caches = query_caches.borrow_mut();
                std::mem::take(&mut query_caches.projected_reference_result_cache)
            };
            let result = cached_projected_reference_result_with(
                &mut projected_reference_result_cache,
                &cache_context,
                bounds,
                &projected_halfspaces,
                || {
                    search_projected_reference_families_lazy_escape(
                        &projected_root.projected_targets,
                        || {
                            projected_support_plane_cell_reference_with_query_caches(
                                old_ref,
                                old_ref_definitions,
                                old_wnv,
                                bounds,
                                polygons,
                                projected_halfspaces.clone(),
                                &mut query_caches.borrow_mut(),
                            )
                        },
                        |projected_target| {
                            let mut query_caches = query_caches.borrow_mut();
                            let query_caches = &mut **query_caches;
                            let SupportReferenceQueryCaches {
                                validity_cache,
                                trace_cache,
                                ..
                            } = query_caches;
                            trace_projected_reference_target_with_queries(
                                validity_cache,
                                trace_cache,
                                Some(&cache_context),
                                bounds,
                                projected_target,
                                |point| {
                                    is_certified_valid_reference_for_bounds(point, bounds, polygons)
                                },
                                |target| {
                                    trace_reference_target_from_validated_bounds(
                                        old_ref,
                                        old_ref_definitions,
                                        old_wnv,
                                        bounds,
                                        polygons,
                                        target,
                                    )
                                },
                            )
                        },
                        || {
                            let mut query_caches = query_caches.borrow_mut();
                            let query_caches = &mut **query_caches;
                            let SupportReferenceQueryCaches {
                                shifted_projected_family_cache,
                                reference_witness_cache,
                                pure_halfspace_contains_cache,
                                ..
                            } = query_caches;
                            projected_reference_escape_targets_from_seed_family_state_with_tracking_unknown_and_witness_cache(
                                bounds,
                                &projected_halfspaces,
                                &projected_root.projected_targets,
                                projected_root.report.as_ref(),
                                &projected_root.projected_escape_seed_families,
                                &mut projected_unknown,
                                shifted_projected_family_cache,
                                reference_witness_cache,
                                pure_halfspace_contains_cache,
                            )
                        },
                        |projected_target| {
                            let axis_options = {
                                let query_caches = query_caches.borrow_mut();
                                cached_projection_escape_axis_options_state_with(
                                    &mut query_caches
                                        .projection_escape_axis_options_cache
                                        .borrow_mut(),
                                    &projected_target.point,
                                    bounds,
                                    polygons,
                                    || {
                                        projection_escape_axis_options_family_tracking_unknown(
                                            &projected_target.point,
                                            bounds,
                                            polygons,
                                        )
                                    },
                                )?
                            };
                            projection_axis_escape_reference_with_axis_options_tracking_unknown(
                                &projected_target.point,
                                &axis_options.axis_options,
                                axis_options.saw_unknown,
                                |corridor| {
                                    let mut query_caches = query_caches.borrow_mut();
                                    cached_reference_escape_search_in_query_caches(
                                        &mut query_caches,
                                        &cache_context,
                                        corridor,
                                        |corridor, query_caches| {
                                            support_plane_cell_reference_with_query_caches(
                                                old_ref,
                                                old_ref_definitions,
                                                old_wnv,
                                                corridor,
                                                polygons,
                                                query_caches,
                                            )
                                        },
                                    )
                                },
                            )
                        },
                        |projected_target| {
                            let axis_options = {
                                let query_caches = query_caches.borrow_mut();
                                cached_projection_escape_axis_options_state_with(
                                    &mut query_caches
                                        .projection_escape_axis_options_cache
                                        .borrow_mut(),
                                    &projected_target.point,
                                    bounds,
                                    polygons,
                                    || {
                                        projection_escape_axis_options_family_tracking_unknown(
                                            &projected_target.point,
                                            bounds,
                                            polygons,
                                        )
                                    },
                                )?
                            };
                            projection_escape_reference_with_axis_options_tracking_unknown(
                                &axis_options.axis_options,
                                bounds,
                                axis_options.saw_unknown,
                                |escape_bounds| {
                                    let mut query_caches = query_caches.borrow_mut();
                                    cached_reference_escape_search_in_query_caches(
                                        &mut query_caches,
                                        &cache_context,
                                        escape_bounds,
                                        |escape_bounds, query_caches| {
                                            support_plane_cell_reference_with_query_caches(
                                                old_ref,
                                                old_ref_definitions,
                                                old_wnv,
                                                escape_bounds,
                                                polygons,
                                                query_caches,
                                            )
                                        },
                                    )
                                },
                            )
                        },
                    )
                },
            );
            query_caches.borrow_mut().projected_reference_result_cache =
                projected_reference_result_cache;
            result
        },
        &mut projected_unknown,
    )?;
    reference_result_or_error(
        projected,
        None,
        projected_unknown || support_unknown || surface_departure_unknown,
    )
}

#[derive(Clone)]
struct ProjectedRootReferenceFamilies {
    report: Option<hyperlimit::HalfspaceFeasibilityReport>,
    projected_targets: Vec<ReferenceTarget>,
    projected_escape_seed_families: ProjectedEscapeSeedFamilies,
    saw_unknown: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct ProjectedEscapeSeedFamilies {
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
}

#[derive(Clone)]
struct ProjectedRootReferenceFamilyCacheEntry {
    bounds: Aabb,
    halfspaces: Vec<LimitPlane3>,
    result: HypermeshResult<ProjectedRootReferenceFamilies>,
}

#[cfg(test)]
fn projected_root_reference_families(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed_geometry_cache: &mut Vec<SupportCellSeedGeometryCacheEntry>,
) -> HypermeshResult<ProjectedRootReferenceFamilies> {
    let mut centroid_subset_seed_cache = Vec::new();
    let reference_witness_cache = std::cell::RefCell::new(Vec::new());
    let strict_contains_cache = std::cell::RefCell::new(Vec::new());
    let pure_halfspace_contains_cache = std::cell::RefCell::new(Vec::new());
    let mut shifted_projected_family_cache = Vec::new();
    projected_root_reference_families_with_witness_cache(
        bounds,
        halfspaces,
        seed_geometry_cache,
        &mut centroid_subset_seed_cache,
        &mut shifted_projected_family_cache,
        &reference_witness_cache,
        &strict_contains_cache,
        &pure_halfspace_contains_cache,
    )
}

fn projected_root_reference_families_with_witness_cache(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed_geometry_cache: &mut Vec<SupportCellSeedGeometryCacheEntry>,
    centroid_subset_seed_cache: &mut Vec<Point3CentroidSubsetFamilyCacheEntry>,
    shifted_projected_family_cache: &mut Vec<ShiftedProjectedCellFamilyCacheEntry>,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    strict_contains_cache: &std::cell::RefCell<Vec<ReferenceHalfspaceContainmentCacheEntry>>,
    _pure_halfspace_contains_cache: &std::cell::RefCell<
        Vec<ReferencePureHalfspaceContainmentCacheEntry>,
    >,
) -> HypermeshResult<ProjectedRootReferenceFamilies> {
    let (report, saw_report_unknown) = optional_halfspace_system_report(halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(ProjectedRootReferenceFamilies {
            report,
            projected_targets: Vec::new(),
            projected_escape_seed_families: ProjectedEscapeSeedFamilies {
                strict_seeds: Vec::new(),
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
            },
            saw_unknown: saw_report_unknown,
        });
    }

    let mut saw_unknown = saw_report_unknown;
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        projected_cell_seed_families_from_optional_report_with_seed_geometry_cache(
            bounds,
            halfspaces,
            report.as_ref(),
            &mut saw_unknown,
            seed_geometry_cache,
            centroid_subset_seed_cache,
        )?;
    let projected_targets =
        strict_projected_cell_targets_from_seed_families_with_tracking_unknown_and_witness_cache(
            bounds,
            halfspaces,
            report.as_ref(),
            strict_seeds.clone(),
            shifted_vertices.clone(),
            shifted_geometry_seeds.clone(),
            &mut saw_unknown,
            reference_witness_cache,
            strict_contains_cache,
            |seed| {
                shifted_projected_cell_targets_from_seed_with_cache(
                    bounds,
                    halfspaces,
                    seed,
                    shifted_projected_family_cache,
                    reference_witness_cache,
                    strict_contains_cache,
                )
            },
        )?;
    Ok(ProjectedRootReferenceFamilies {
        report,
        projected_targets,
        projected_escape_seed_families: ProjectedEscapeSeedFamilies {
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        },
        saw_unknown,
    })
}

fn cached_projected_root_reference_families_with(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<ProjectedRootReferenceFamilies> {
    if let Some(existing) = query_caches.projected_root_cache.iter().find(|existing| {
        existing.bounds == *bounds && same_limit_halfspace_state(&existing.halfspaces, halfspaces)
    }) {
        return existing.result.clone();
    }

    let result = projected_root_reference_families_with_witness_cache(
        bounds,
        halfspaces,
        &mut query_caches.seed_geometry_cache,
        &mut query_caches.centroid_subset_seed_cache,
        &mut query_caches.shifted_projected_family_cache,
        &query_caches.reference_witness_cache,
        &query_caches.strict_contains_cache,
        &query_caches.pure_halfspace_contains_cache,
    );
    query_caches
        .projected_root_cache
        .push(ProjectedRootReferenceFamilyCacheEntry {
            bounds: bounds.clone(),
            halfspaces: halfspaces.to_vec(),
            result: result.clone(),
        });
    result
}

fn same_limit_halfspace_state(left: &[LimitPlane3], right: &[LimitPlane3]) -> bool {
    if left.len() != right.len() {
        return false;
    }

    let mut right_used = vec![false; right.len()];
    for left_halfspace in left {
        let Some((matched_index, _)) = right.iter().enumerate().find(|(index, right_halfspace)| {
            !right_used[*index] && *right_halfspace == left_halfspace
        }) else {
            return false;
        };
        right_used[matched_index] = true;
    }

    true
}

#[cfg(test)]
fn reference_target_family_or_empty(
    result: HypermeshResult<Vec<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    match result {
        Ok(targets) => Ok(targets),
        Err(crate::error::HypermeshError::UnknownClassification) => Ok(Vec::new()),
        Err(err) => Err(err),
    }
}

#[cfg(test)]
fn reference_target_family_or_empty_tracking_unknown(
    result: HypermeshResult<Vec<ReferenceTarget>>,
    saw_unknown: &mut bool,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    match result {
        Ok(targets) => Ok(targets),
        Err(crate::error::HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            Ok(Vec::new())
        }
        Err(err) => Err(err),
    }
}

#[cfg(test)]
fn projected_reference_search_or_none(
    result: HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    match result {
        Ok(found) => Ok(found),
        Err(crate::error::HypermeshError::UnknownClassification) => Ok(None),
        Err(err) => Err(err),
    }
}

fn projected_reference_search_or_none_tracking_unknown(
    result: HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
    saw_unknown: &mut bool,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    match result {
        Ok(found) => Ok(found),
        Err(crate::error::HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            Ok(None)
        }
        Err(err) => Err(err),
    }
}

fn reference_result_or_error(
    projected: Option<(ReferenceTarget, Vec<i32>)>,
    support: Option<(ReferenceTarget, Vec<i32>)>,
    projected_unknown: bool,
) -> HypermeshResult<(Point3, Vec<[Plane; 3]>, Vec<i32>)> {
    if let Some((target, winding)) = projected {
        return Ok(certified_reference_result((
            target.point,
            target.definitions,
            winding,
        )));
    }
    if let Some((target, winding)) = support {
        return Ok(certified_reference_result((
            target.point,
            target.definitions,
            winding,
        )));
    }
    if projected_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Err(crate::error::HypermeshError::ReferencePropagationFailed)
    }
}

#[cfg(test)]
fn reference_result_with_support_fallback(
    projected: Option<(ReferenceTarget, Vec<i32>)>,
    projected_unknown: bool,
    support_search: impl FnOnce() -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<(Point3, Vec<[Plane; 3]>, Vec<i32>)> {
    if let Some((target, winding)) = projected {
        return Ok(certified_reference_result((
            target.point,
            target.definitions,
            winding,
        )));
    }

    let support = support_search()?;
    reference_result_or_error(None, support, projected_unknown)
}

fn search_projected_reference_families_lazy_escape(
    projected_targets: &[ReferenceTarget],
    mut projected_support_search: impl FnMut() -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
    mut trace_projected_target: impl FnMut(&ReferenceTarget) -> HypermeshResult<Option<Vec<i32>>>,
    load_projected_escape_targets: impl FnOnce() -> HypermeshResult<Vec<ReferenceTarget>>,
    mut axis_escape_search: impl FnMut(
        &ReferenceTarget,
    ) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
    mut tight_escape_search: impl FnMut(
        &ReferenceTarget,
    ) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut saw_unknown = false;
    let mut traced_direct_targets = Vec::new();

    for projected_target in projected_targets {
        traced_direct_targets.push(projected_target.clone());
        match trace_projected_target(projected_target) {
            Ok(Some(winding)) => {
                return Ok(Some((
                    certify_reference_target_after_trace(projected_target.clone()),
                    winding,
                )));
            }
            Ok(None) => {
                if projected_target.uncertified_definition_fallback {
                    saw_unknown = true;
                }
            }
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    match projected_support_search() {
        Ok(Some((target, winding))) => {
            return Ok(Some((
                certify_reference_target_after_trace(target),
                winding,
            )));
        }
        Ok(None) => {}
        Err(crate::error::HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }

    let projected_escape_targets = match load_projected_escape_targets() {
        Ok(targets) => targets,
        Err(crate::error::HypermeshError::UnknownClassification) => {
            saw_unknown = true;
            Vec::new()
        }
        Err(err) => return Err(err),
    };

    for projected_target in &projected_escape_targets {
        if !traced_direct_targets
            .iter()
            .any(|candidate| reference_targets_match_for_trace_cache(candidate, projected_target))
        {
            match trace_projected_target(projected_target) {
                Ok(Some(winding)) => {
                    return Ok(Some((
                        certify_reference_target_after_trace(projected_target.clone()),
                        winding,
                    )));
                }
                Ok(None) => {
                    if projected_target.uncertified_definition_fallback {
                        saw_unknown = true;
                    }
                }
                Err(crate::error::HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                }
                Err(err) => return Err(err),
            }
        }

        match axis_escape_search(projected_target) {
            Ok(Some((target, winding))) => {
                return Ok(Some((
                    certify_reference_target_after_trace(target),
                    winding,
                )));
            }
            Ok(None) => {
                if projected_target.uncertified_definition_fallback {
                    saw_unknown = true;
                }
            }
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }

        match tight_escape_search(projected_target) {
            Ok(Some((target, winding))) => {
                return Ok(Some((
                    certify_reference_target_after_trace(target),
                    winding,
                )));
            }
            Ok(None) => {
                if projected_target.uncertified_definition_fallback {
                    saw_unknown = true;
                }
            }
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    if saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

#[cfg(test)]
fn search_projected_reference_families(
    projected_targets: &[ReferenceTarget],
    projected_escape_targets: &[ReferenceTarget],
    projected_support_search: impl FnMut() -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
    trace_projected_target: impl FnMut(&ReferenceTarget) -> HypermeshResult<Option<Vec<i32>>>,
    axis_escape_search: impl FnMut(
        &ReferenceTarget,
    ) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
    tight_escape_search: impl FnMut(
        &ReferenceTarget,
    ) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    search_projected_reference_families_lazy_escape(
        projected_targets,
        projected_support_search,
        trace_projected_target,
        || Ok(projected_escape_targets.to_vec()),
        axis_escape_search,
        tight_escape_search,
    )
}

#[cfg(test)]
fn projected_reference_escape_targets(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    projected_targets: &[ReferenceTarget],
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let (report, saw_unknown) = optional_halfspace_system_report(halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(projected_targets.to_vec());
    }
    let targets = projected_reference_escape_targets_from_optional_report(
        bounds,
        halfspaces,
        projected_targets,
        report.as_ref(),
    )?;
    if targets.len() == projected_targets.len() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(targets)
    }
}

#[cfg(test)]
fn projected_reference_escape_targets_from_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    projected_targets: &[ReferenceTarget],
    report: &hyperlimit::HalfspaceFeasibilityReport,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    projected_reference_escape_targets_from_optional_report(
        bounds,
        halfspaces,
        projected_targets,
        Some(report),
    )
}

#[cfg(test)]
fn projected_reference_escape_targets_from_optional_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    projected_targets: &[ReferenceTarget],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut saw_unknown = false;
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        projected_cell_seed_families_from_optional_report(
            bounds,
            halfspaces,
            report,
            &mut saw_unknown,
        )?;
    let targets = projected_reference_escape_targets_from_seed_families(
        bounds,
        halfspaces,
        projected_targets,
        report,
        strict_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    )?;

    if targets.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(targets)
    }
}

#[cfg(test)]
fn projected_reference_escape_targets_from_seed_families(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    projected_targets: &[ReferenceTarget],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    strict_shift_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    projected_reference_escape_targets_from_seed_families_with(
        bounds,
        halfspaces,
        projected_targets,
        report,
        strict_shift_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
        |seed| projected_escape_targets_from_seed(bounds, halfspaces, seed),
    )
}

#[cfg(test)]
fn projected_reference_escape_targets_from_seed_families_with(
    _bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    projected_targets: &[ReferenceTarget],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    strict_shift_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    build_escape_targets: impl FnMut(&Point3) -> HypermeshResult<Vec<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut saw_unknown = false;
    let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
        halfspaces,
        projected_targets,
        report,
        strict_shift_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
        &mut saw_unknown,
        build_escape_targets,
    )?;
    if targets.len() == projected_targets.len() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        let mut targets = targets;
        if saw_unknown {
            mark_all_reference_targets_uncertified(&mut targets);
        }
        Ok(targets)
    }
}

#[cfg(test)]
fn projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
    halfspaces: &[LimitPlane3],
    projected_targets: &[ReferenceTarget],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    strict_shift_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    saw_unknown: &mut bool,
    build_escape_targets: impl FnMut(&Point3) -> HypermeshResult<Vec<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let reference_witness_cache = std::cell::RefCell::new(Vec::new());
    let pure_halfspace_contains_cache = std::cell::RefCell::new(Vec::new());
    projected_reference_escape_targets_from_seed_families_with_tracking_unknown_and_witness_cache(
        halfspaces,
        projected_targets,
        report,
        strict_shift_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
        saw_unknown,
        &reference_witness_cache,
        &pure_halfspace_contains_cache,
        build_escape_targets,
    )
}

fn projected_reference_escape_targets_from_seed_families_with_tracking_unknown_and_witness_cache(
    halfspaces: &[LimitPlane3],
    projected_targets: &[ReferenceTarget],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    strict_shift_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    saw_unknown: &mut bool,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    pure_halfspace_contains_cache: &std::cell::RefCell<
        Vec<ReferencePureHalfspaceContainmentCacheEntry>,
    >,
    mut build_escape_targets: impl FnMut(&Point3) -> HypermeshResult<Vec<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut targets = projected_targets.to_vec();
    let report_witness = report.and_then(|report| report.witness.clone());
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_target_seed_families_with_report_seed(
            report_witness.as_ref(),
            strict_shift_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );
    *saw_unknown |= extend_reference_target_families_collect_hard_unknown(
        &mut targets,
        [
            reference_target_family_from_witness(
                report.and_then(|report| report.witness.as_ref()),
                |witness| {
                    cached_point_strictly_inside_halfspaces_or_unknown_with(
                        &mut pure_halfspace_contains_cache.borrow_mut(),
                        witness,
                        halfspaces,
                    )
                },
                |witness| {
                    cached_reference_target_from_halfspace_witness_with(
                        &mut reference_witness_cache.borrow_mut(),
                        witness,
                        halfspaces,
                        active_planes_from_optional_halfspace_report(report, witness),
                        || {
                            reference_target_from_halfspace_witness(
                                witness,
                                halfspaces,
                                active_planes_from_optional_halfspace_report(report, witness),
                            )
                        },
                    )
                },
            ),
            deferred_projected_escape_direct_targets_with_contains_and_build(
                &strict_shift_seeds,
                report_witness.as_ref(),
                halfspaces,
                |seed, halfspaces| {
                    cached_point_strictly_inside_halfspaces_or_unknown_with(
                        &mut pure_halfspace_contains_cache.borrow_mut(),
                        seed,
                        halfspaces,
                    )
                },
                |seed| {
                    cached_reference_target_from_halfspace_witness_with(
                        &mut reference_witness_cache.borrow_mut(),
                        seed,
                        halfspaces,
                        [None, None, None],
                        || {
                            reference_target_from_halfspace_witness(
                                seed,
                                halfspaces,
                                [None, None, None],
                            )
                        },
                    )
                },
            ),
            collect_reference_target_family(strict_shift_seeds, |seed| build_escape_targets(&seed)),
            collect_reference_target_family(shifted_vertices, |vertex| {
                build_escape_targets(&vertex)
            }),
            collect_reference_target_family(shifted_geometry_seeds, |seed| {
                build_escape_targets(&seed)
            }),
        ],
    )?;
    *saw_unknown |= targets
        .iter()
        .any(|target| target.uncertified_definition_fallback);
    if *saw_unknown {
        mark_all_reference_targets_uncertified(&mut targets);
    }
    Ok(targets)
}

fn projected_reference_escape_targets_from_seed_family_state_with_tracking_unknown_and_witness_cache(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    projected_targets: &[ReferenceTarget],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    seed_families: &ProjectedEscapeSeedFamilies,
    saw_unknown: &mut bool,
    shifted_projected_family_cache: &mut Vec<ShiftedProjectedCellFamilyCacheEntry>,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    pure_halfspace_contains_cache: &std::cell::RefCell<
        Vec<ReferencePureHalfspaceContainmentCacheEntry>,
    >,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    projected_reference_escape_targets_from_seed_families_with_tracking_unknown_and_witness_cache(
        halfspaces,
        projected_targets,
        report,
        seed_families.strict_seeds.clone(),
        seed_families.shifted_vertices.clone(),
        seed_families.shifted_geometry_seeds.clone(),
        saw_unknown,
        reference_witness_cache,
        pure_halfspace_contains_cache,
        |seed| {
            projected_escape_targets_from_seed_with_cache(
                bounds,
                halfspaces,
                seed,
                shifted_projected_family_cache,
                reference_witness_cache,
                pure_halfspace_contains_cache,
            )
        },
    )
}

#[cfg(test)]
#[allow(dead_code)]
fn projected_support_plane_cell_reference(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    projected_halfspaces: Vec<LimitPlane3>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    support_plane_cell_reference_with_halfspaces(
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        projected_halfspaces,
    )
}

fn projected_support_plane_cell_reference_with_query_caches(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    projected_halfspaces: Vec<LimitPlane3>,
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    support_plane_cell_reference_with_halfspaces_and_query_caches(
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        projected_halfspaces,
        query_caches,
    )
}

#[cfg(test)]
#[allow(dead_code)]
fn projection_escape_reference(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let axis_options = projection_escape_axis_options_family(projected, bounds, polygons)?;
    projection_escape_reference_with_axis_options(&axis_options, bounds, |escape_bounds| {
        support_plane_cell_reference(
            old_ref,
            old_ref_definitions,
            old_wnv,
            escape_bounds,
            polygons,
        )
    })
}

#[cfg(test)]
#[allow(dead_code)]
fn projection_escape_reference_with_axis_options(
    axis_options: &ProjectionEscapeAxisOptions,
    bounds: &Aabb,
    search: impl FnMut(&Aabb) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    projection_escape_reference_with_axis_options_tracking_unknown(
        axis_options,
        bounds,
        false,
        search,
    )
}

fn projection_escape_reference_with_axis_options_tracking_unknown(
    axis_options: &ProjectionEscapeAxisOptions,
    bounds: &Aabb,
    saw_unknown: bool,
    search: impl FnMut(&Aabb) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    projection_escape_reference_with_search_and_axis_options_tracking_unknown(
        axis_options,
        bounds,
        saw_unknown,
        search,
    )
}

#[cfg(test)]
fn projection_escape_reference_with_search(
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    mut search: impl FnMut(&Aabb) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let axis_options = projection_escape_axis_options_family(projected, bounds, polygons)?;
    projection_escape_reference_with_search_and_axis_options(&axis_options, bounds, &mut search)
}

#[cfg(test)]
fn projection_escape_reference_with_search_and_axis_options(
    axis_options: &ProjectionEscapeAxisOptions,
    bounds: &Aabb,
    mut search: impl FnMut(&Aabb) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    projection_escape_reference_with_search_and_axis_options_tracking_unknown(
        axis_options,
        bounds,
        false,
        &mut search,
    )
}

fn projection_escape_reference_with_search_and_axis_options_tracking_unknown(
    axis_options: &ProjectionEscapeAxisOptions,
    bounds: &Aabb,
    saw_unknown: bool,
    mut search: impl FnMut(&Aabb) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    projection_escape_reference_with_search_and_axis_options_and_bounds_family(
        axis_options,
        bounds,
        saw_unknown,
        &mut search,
        |axis_options, saw_unknown| {
            projection_escape_bounds_family_from_axis_options_tracking_unknown(
                axis_options,
                saw_unknown,
            )
        },
    )
}

fn projection_escape_reference_with_search_and_axis_options_and_bounds_family(
    axis_options: &ProjectionEscapeAxisOptions,
    bounds: &Aabb,
    initial_saw_unknown: bool,
    mut search: impl FnMut(&Aabb) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
    mut escape_bounds_family: impl FnMut(
        &ProjectionEscapeAxisOptions,
        &mut bool,
    ) -> HypermeshResult<Vec<Aabb>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut saw_unknown = initial_saw_unknown;

    for escape_bounds in escape_bounds_family(axis_options, &mut saw_unknown)? {
        if escape_bounds == *bounds {
            continue;
        }
        match search(&escape_bounds) {
            Ok(Some((target, winding))) => {
                return Ok(Some((
                    certify_reference_target_after_trace(target),
                    winding,
                )));
            }
            Ok(None) => {}
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    if saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

#[cfg(test)]
fn projection_escape_bounds(
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<Aabb>> {
    Ok(
        projection_escape_bounds_family(projected, bounds, polygons)?
            .into_iter()
            .next(),
    )
}

#[cfg(test)]
fn projection_escape_bounds_family(
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<Aabb>> {
    let axis_options = projection_escape_axis_options_family(projected, bounds, polygons)?;
    projection_escape_bounds_family_from_axis_options(&axis_options)
}

#[cfg(test)]
fn projection_escape_bounds_family_from_axis_options(
    axis_options: &ProjectionEscapeAxisOptions,
) -> HypermeshResult<Vec<Aabb>> {
    let mut saw_unknown = false;
    let family = projection_escape_bounds_family_from_axis_options_tracking_unknown(
        axis_options,
        &mut saw_unknown,
    )?;
    if family.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(family)
    }
}

fn projection_escape_bounds_family_from_axis_options_tracking_unknown(
    axis_options: &ProjectionEscapeAxisOptions,
    saw_unknown: &mut bool,
) -> HypermeshResult<Vec<Aabb>> {
    let (family, family_unknown) =
        projection_escape_bounds_family_from_axis_options_with_extents(axis_options, |bounds| {
            aabb_has_positive_or_zero_extents(bounds)
        })?;
    *saw_unknown |= family_unknown;
    Ok(family)
}

fn projection_escape_bounds_family_from_axis_options_with_extents(
    axis_options: &ProjectionEscapeAxisOptions,
    mut extents_ok: impl FnMut(&Aabb) -> HypermeshResult<bool>,
) -> HypermeshResult<(Vec<Aabb>, bool)> {
    if axis_options.len() != 3 {
        return Ok((Vec::new(), false));
    }
    let mut keyed_boxes = Vec::new();
    let mut saw_unknown = false;
    for lower_x in 0..axis_options[0].0.len() {
        for upper_x in 0..axis_options[0].1.len() {
            for lower_y in 0..axis_options[1].0.len() {
                for upper_y in 0..axis_options[1].1.len() {
                    for lower_z in 0..axis_options[2].0.len() {
                        for upper_z in 0..axis_options[2].1.len() {
                            let min = Point3::new(
                                axis_options[0].0[lower_x].clone(),
                                axis_options[1].0[lower_y].clone(),
                                axis_options[2].0[lower_z].clone(),
                            );
                            let max = Point3::new(
                                axis_options[0].1[upper_x].clone(),
                                axis_options[1].1[upper_y].clone(),
                                axis_options[2].1[upper_z].clone(),
                            );
                            let escape_bounds = Aabb::new(min, max);
                            match extents_ok(&escape_bounds) {
                                Ok(true) => {}
                                Ok(false) => continue,
                                Err(crate::error::HypermeshError::UnknownClassification) => {
                                    saw_unknown = true;
                                    continue;
                                }
                                Err(err) => return Err(err),
                            }
                            keyed_boxes.push((
                                (
                                    lower_x + upper_x + lower_y + upper_y + lower_z + upper_z,
                                    lower_x,
                                    upper_x,
                                    lower_y,
                                    upper_y,
                                    lower_z,
                                    upper_z,
                                ),
                                escape_bounds,
                            ));
                        }
                    }
                }
            }
        }
    }

    keyed_boxes.sort_by_key(|left| left.0);

    let mut family = Vec::new();
    for (_, escape_bounds) in keyed_boxes {
        if !family.iter().any(|existing| existing == &escape_bounds) {
            family.push(escape_bounds);
        }
    }

    Ok((family, saw_unknown))
}

#[cfg(test)]
fn projection_escape_axis_options_family(
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<ProjectionEscapeAxisOptions> {
    Ok(
        projection_escape_axis_options_family_tracking_unknown(projected, bounds, polygons)?
            .axis_options,
    )
}

fn projection_escape_axis_options_family_tracking_unknown(
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<ProjectionEscapeAxisOptionsState> {
    let mut saw_unknown = false;
    let axis_options = (0..3)
        .map(|axis| {
            projection_escape_axis_options_tracking_unknown(
                projected,
                bounds,
                polygons,
                axis,
                &mut saw_unknown,
            )
        })
        .collect::<HypermeshResult<_>>()?;
    Ok(ProjectionEscapeAxisOptionsState {
        axis_options,
        saw_unknown,
    })
}

fn projection_escape_axis_options_tracking_unknown(
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
    saw_unknown: &mut bool,
) -> HypermeshResult<(Vec<Real>, Vec<Real>)> {
    let bound_min = axis_ref(&bounds.min, axis);
    let bound_max = axis_ref(&bounds.max, axis);
    if compare_real(bound_min, bound_max)?.is_eq() {
        return Ok((vec![bound_min.clone()], vec![bound_max.clone()]));
    }

    let lower = escaped_reference_axis_stop_values_tracking_unknown(
        projected,
        bounds,
        polygons,
        axis,
        false,
        saw_unknown,
    )?;
    let upper = escaped_reference_axis_stop_values_tracking_unknown(
        projected,
        bounds,
        polygons,
        axis,
        true,
        saw_unknown,
    )?;
    if lower.is_empty() || upper.is_empty() {
        return Ok((Vec::new(), Vec::new()));
    }

    Ok((lower, upper))
}

fn aabb_has_positive_or_zero_extents(bounds: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        if compare_real(axis_ref(&bounds.min, axis), axis_ref(&bounds.max, axis))?.is_gt() {
            return Ok(false);
        }
    }
    Ok(true)
}

#[cfg(test)]
fn escaped_reference_axis_stop_values(
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
    direction_positive: bool,
) -> HypermeshResult<Vec<Real>> {
    let mut saw_unknown = false;
    let stop_values = escaped_reference_axis_stop_values_tracking_unknown(
        projected,
        bounds,
        polygons,
        axis,
        direction_positive,
        &mut saw_unknown,
    )?;
    if stop_values.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(stop_values)
    }
}

fn escaped_reference_axis_stop_values_tracking_unknown(
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
    direction_positive: bool,
    saw_unknown: &mut bool,
) -> HypermeshResult<Vec<Real>> {
    let (stop_values, family_unknown) = escaped_reference_axis_stop_values_with_queries(
        projected,
        bounds,
        polygons,
        axis,
        direction_positive,
        |projected, endpoint, polygon, axis| {
            reference_axis_surface_crossing(projected, endpoint, polygon, axis)
        },
        classify_point_in_local_polygon,
    )?;
    *saw_unknown |= family_unknown;
    Ok(stop_values)
}

fn escaped_reference_axis_stop_values_with_queries(
    projected: &Point3,
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
    ) -> HypermeshResult<LocalPolygonPointLocation>,
) -> HypermeshResult<(Vec<Real>, bool)> {
    let start_value = axis_ref(projected, axis);
    let bound_value = if direction_positive {
        axis_ref(&bounds.max, axis)
    } else {
        axis_ref(&bounds.min, axis)
    };
    let room = if direction_positive {
        bound_value - start_value
    } else {
        start_value - bound_value
    };
    let room_order = compare_real(&room, &Real::zero())?;
    if !room_order.is_gt() {
        return Ok((Vec::new(), room_order.is_eq()));
    }

    let mut endpoint = projected.clone();
    *axis_mut(&mut endpoint, axis) = bound_value.clone();
    let mut stop_values = vec![bound_value.clone()];
    let mut saw_unknown = false;

    for polygon in polygons {
        let Some(crossing) = (match crossing_for(projected, &endpoint, polygon, axis) {
            Ok(crossing) => crossing,
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        }) else {
            continue;
        };
        let point_location = match classify_point_on_polygon(&crossing, polygon) {
            Ok(point_location) => point_location,
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        match point_location {
            LocalPolygonPointLocation::Outside => continue,
            LocalPolygonPointLocation::Boundary => {
                saw_unknown = true;
                continue;
            }
            LocalPolygonPointLocation::Interior => {}
        }

        let crossing_value = axis_ref(&crossing, axis);
        let from_start = compare_real(crossing_value, start_value)?;
        if (direction_positive && !from_start.is_gt())
            || (!direction_positive && !from_start.is_lt())
        {
            if from_start.is_eq()
                && matches!(
                    point_location,
                    LocalPolygonPointLocation::Boundary | LocalPolygonPointLocation::Interior
                )
            {
                saw_unknown = true;
            }
            if compare_real(crossing_value, bound_value)?.is_eq()
                && matches!(
                    point_location,
                    LocalPolygonPointLocation::Boundary | LocalPolygonPointLocation::Interior
                )
            {
                saw_unknown = true;
            }
            continue;
        }

        let mut insert_at = stop_values.len();
        let mut duplicate = false;
        for (index, existing) in stop_values.iter().enumerate() {
            let order = compare_real(crossing_value, existing)?;
            if order.is_eq() {
                duplicate = true;
                break;
            }
            if (direction_positive && order.is_lt()) || (!direction_positive && order.is_gt()) {
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

fn push_unique_reference_target(
    targets: &mut Vec<ReferenceTarget>,
    target: ReferenceTarget,
) -> bool {
    if let Some(existing) = targets
        .iter_mut()
        .find(|existing| existing.point == target.point)
    {
        let incoming_definitions = target.definitions;
        let incoming_is_fallback = target.uncertified_definition_fallback;
        let existing_covered_by_incoming = existing.definitions.iter().all(|existing_definition| {
            incoming_definitions.iter().any(|incoming_definition| {
                reference_definition_planes_match_as_sets(existing_definition, incoming_definition)
            })
        });
        let mut introduced_new_definition = false;
        for definition in incoming_definitions {
            if !existing
                .definitions
                .iter()
                .any(|candidate| reference_definition_planes_match_as_sets(candidate, &definition))
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

fn reference_definition_planes_match_as_sets(left: &[Plane; 3], right: &[Plane; 3]) -> bool {
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

fn reference_definition_families_match_as_sets(left: &[[Plane; 3]], right: &[[Plane; 3]]) -> bool {
    if left.len() != right.len() {
        return false;
    }

    let mut matched = vec![false; right.len()];
    for left_definition in left {
        let Some((index, _)) = right.iter().enumerate().find(|(index, right_definition)| {
            !matched[*index]
                && reference_definition_planes_match_as_sets(left_definition, right_definition)
        }) else {
            return false;
        };
        matched[index] = true;
    }

    true
}

fn reference_targets_match_for_trace_cache(
    left: &ReferenceTarget,
    right: &ReferenceTarget,
) -> bool {
    left.point == right.point
        && reference_definition_families_match_as_sets(&left.definitions, &right.definitions)
}

fn extend_reference_targets_backtracking_unknown<T>(
    targets: &mut Vec<ReferenceTarget>,
    candidates: impl IntoIterator<Item = T>,
    mut build: impl FnMut(T) -> HypermeshResult<Vec<ReferenceTarget>>,
) -> HypermeshResult<()> {
    let mut saw_hard_unknown = false;
    for candidate in candidates {
        match build(candidate) {
            Ok(found) => {
                for target in found {
                    push_unique_reference_target(targets, target);
                }
            }
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    let saw_unknown = saw_hard_unknown
        || targets
            .iter()
            .any(|target| target.uncertified_definition_fallback);
    if targets.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_reference_targets_uncertified(targets);
        }
        Ok(())
    }
}

fn collect_reference_target_family<T>(
    candidates: impl IntoIterator<Item = T>,
    build: impl FnMut(T) -> HypermeshResult<Vec<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut targets = Vec::new();
    extend_reference_targets_backtracking_unknown(&mut targets, candidates, build)?;
    Ok(targets)
}

fn reference_target_family_from_witness(
    witness: Option<&Point3>,
    mut include: impl FnMut(&Point3) -> HypermeshResult<bool>,
    mut build: impl FnMut(&Point3) -> HypermeshResult<Option<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let Some(witness) = witness else {
        return Ok(Vec::new());
    };
    collect_reference_target_family(std::iter::once(witness.clone()), |candidate| {
        if !include(&candidate)? {
            return Ok(Vec::new());
        }
        Ok(build(&candidate)?.into_iter().collect())
    })
}

#[cfg(test)]
fn deferred_direct_reference_targets_from_strict_seeds(
    strict_seeds: &[Point3],
    report_witness: Option<&Point3>,
    halfspaces: &[LimitPlane3],
    saw_unknown: &mut bool,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    deferred_direct_reference_targets_from_strict_seeds_with(
        strict_seeds,
        report_witness,
        saw_unknown,
        |seed| reference_target_from_halfspace_witness(seed, halfspaces, [None, None, None]),
    )
}

fn deferred_direct_reference_targets_from_strict_seeds_with(
    strict_seeds: &[Point3],
    _report_witness: Option<&Point3>,
    saw_unknown: &mut bool,
    mut build: impl FnMut(&Point3) -> HypermeshResult<Option<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut targets = Vec::new();
    let saw_hard_unknown = extend_reference_target_families_collect_hard_unknown(
        &mut targets,
        strict_seeds
            .iter()
            .map(|seed| Ok(build(seed)?.into_iter().collect())),
    )?;
    if saw_hard_unknown {
        *saw_unknown = true;
        mark_all_reference_targets_uncertified(&mut targets);
    }
    Ok(targets)
}

#[cfg(test)]
fn deferred_projected_escape_direct_targets(
    strict_seeds: &[Point3],
    report_witness: Option<&Point3>,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Vec<ReferenceTarget>> {
    deferred_projected_escape_direct_targets_with_contains_and_build(
        strict_seeds,
        report_witness,
        halfspaces,
        |seed, halfspaces| point_strictly_inside_halfspaces_or_unknown(seed, halfspaces),
        |seed| reference_target_from_halfspace_witness(seed, halfspaces, [None, None, None]),
    )
}

#[cfg(test)]
fn deferred_projected_escape_direct_targets_with_contains(
    strict_seeds: &[Point3],
    _report_witness: Option<&Point3>,
    halfspaces: &[LimitPlane3],
    mut contains: impl FnMut(&Point3, &[LimitPlane3]) -> HypermeshResult<bool>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    deferred_projected_escape_direct_targets_with_contains_and_build(
        strict_seeds,
        None,
        halfspaces,
        |seed, halfspaces| contains(seed, halfspaces),
        |seed| reference_target_from_halfspace_witness(seed, halfspaces, [None, None, None]),
    )
}

fn deferred_projected_escape_direct_targets_with_contains_and_build(
    strict_seeds: &[Point3],
    _report_witness: Option<&Point3>,
    halfspaces: &[LimitPlane3],
    mut contains: impl FnMut(&Point3, &[LimitPlane3]) -> HypermeshResult<bool>,
    mut build: impl FnMut(&Point3) -> HypermeshResult<Option<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut seen = Vec::new();
    let strict_seeds = take_new_point_family(strict_seeds.to_vec(), &mut seen);
    collect_reference_target_family(strict_seeds, |seed| {
        if !contains(&seed, halfspaces)? {
            return Ok(Vec::new());
        }
        Ok(build(&seed)?.into_iter().collect())
    })
}

fn extend_reference_target_families_backtracking_unknown(
    targets: &mut Vec<ReferenceTarget>,
    families: impl IntoIterator<Item = HypermeshResult<Vec<ReferenceTarget>>>,
) -> HypermeshResult<()> {
    let saw_unknown = extend_reference_target_families_collect_unknown(targets, families)?;
    if targets.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_reference_targets_uncertified(targets);
        }
        Ok(())
    }
}

fn extend_reference_target_families_collect_unknown(
    targets: &mut Vec<ReferenceTarget>,
    families: impl IntoIterator<Item = HypermeshResult<Vec<ReferenceTarget>>>,
) -> HypermeshResult<bool> {
    let mut saw_hard_unknown = false;
    for family in families {
        match family {
            Ok(found) => {
                for target in found {
                    push_unique_reference_target(targets, target);
                }
            }
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    Ok(saw_hard_unknown
        || targets
            .iter()
            .any(|target| target.uncertified_definition_fallback))
}

fn extend_reference_target_families_collect_hard_unknown(
    targets: &mut Vec<ReferenceTarget>,
    families: impl IntoIterator<Item = HypermeshResult<Vec<ReferenceTarget>>>,
) -> HypermeshResult<bool> {
    let mut saw_hard_unknown = false;
    for family in families {
        match family {
            Ok(found) => {
                for target in found {
                    push_unique_reference_target(targets, target);
                }
            }
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_hard_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    Ok(saw_hard_unknown)
}

#[derive(Clone)]
struct HalfspaceReportCacheEntry {
    halfspaces: Vec<LimitPlane3>,
    report: HypermeshResult<Option<hyperlimit::HalfspaceFeasibilityReport>>,
}

fn cached_halfspace_report_with(
    cache: &mut Vec<HalfspaceReportCacheEntry>,
    halfspaces: &[LimitPlane3],
    query: impl FnOnce(
        &[LimitPlane3],
    ) -> HypermeshResult<Option<hyperlimit::HalfspaceFeasibilityReport>>,
) -> HypermeshResult<Option<hyperlimit::HalfspaceFeasibilityReport>> {
    if let Some(existing) = cache
        .iter()
        .find(|existing| limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces))
    {
        return existing.report.clone();
    }

    let report = query(halfspaces);
    cache.push(HalfspaceReportCacheEntry {
        halfspaces: halfspaces.to_vec(),
        report: report.clone(),
    });
    report
}

#[derive(Clone)]
struct HalfspaceFeasibilityCacheEntry {
    halfspaces: Vec<LimitPlane3>,
    feasible: HypermeshResult<bool>,
}

#[cfg(test)]
fn cached_halfspace_feasibility_with(
    cache: &mut Vec<HalfspaceFeasibilityCacheEntry>,
    halfspaces: &[LimitPlane3],
    query: impl FnOnce(&[LimitPlane3]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache
        .iter()
        .find(|existing| limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces))
    {
        return existing.feasible.clone();
    }

    let feasible = query(halfspaces);
    cache.push(HalfspaceFeasibilityCacheEntry {
        halfspaces: halfspaces.to_vec(),
        feasible: feasible.clone(),
    });
    feasible
}

fn cached_halfspace_feasibility_with_report_cache(
    report_cache: &mut Vec<HalfspaceReportCacheEntry>,
    feasible_cache: &mut Vec<HalfspaceFeasibilityCacheEntry>,
    halfspaces: &[LimitPlane3],
    report_query: impl FnOnce(
        &[LimitPlane3],
    ) -> HypermeshResult<Option<hyperlimit::HalfspaceFeasibilityReport>>,
    feasible_query: impl FnOnce(&[LimitPlane3]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = feasible_cache
        .iter()
        .find(|existing| limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces))
    {
        return existing.feasible.clone();
    }

    let feasible = if let Some(existing) = report_cache
        .iter()
        .find(|existing| limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces))
    {
        match &existing.report {
            Ok(Some(report)) => Ok(report.status == HalfspaceFeasibility::Feasible),
            Ok(None) => feasible_query(halfspaces),
            Err(err) => Err(err.clone()),
        }
    } else {
        match cached_halfspace_report_with(report_cache, halfspaces, report_query) {
            Ok(Some(report)) => Ok(report.status == HalfspaceFeasibility::Feasible),
            Ok(None) => feasible_query(halfspaces),
            Err(err) => Err(err),
        }
    };

    feasible_cache.push(HalfspaceFeasibilityCacheEntry {
        halfspaces: halfspaces.to_vec(),
        feasible: feasible.clone(),
    });
    feasible
}

#[derive(Clone)]
struct ReferenceTargetTraceCacheEntry {
    context: Option<SupportReferenceCacheContextKey>,
    target: ReferenceTarget,
    winding: HypermeshResult<Option<Vec<i32>>>,
}

#[derive(Clone)]
struct ReferenceBoundsValidityCacheEntry {
    context: Option<SupportReferencePolygonContextKey>,
    bounds: Aabb,
    point: Point3,
    is_valid: HypermeshResult<bool>,
}

#[derive(Clone)]
struct ReferenceHalfspaceContainmentCacheEntry {
    bounds: Aabb,
    point: Point3,
    halfspaces: Vec<LimitPlane3>,
    contains: HypermeshResult<bool>,
}

#[derive(Clone)]
struct ReferencePureHalfspaceContainmentCacheEntry {
    point: Point3,
    halfspaces: Vec<LimitPlane3>,
    contains: HypermeshResult<bool>,
}

#[derive(Clone)]
struct SupportSurfaceCacheEntry {
    context: Option<SupportReferencePolygonContextKey>,
    point: Point3,
    on_support_surface: HypermeshResult<bool>,
}

#[derive(Clone)]
struct ReferenceWitnessTargetCacheEntry {
    point: Point3,
    halfspaces: Vec<LimitPlane3>,
    active_planes: [Option<usize>; 3],
    target: HypermeshResult<Option<ReferenceTarget>>,
}

#[derive(Default)]
struct SupportReferenceQueryCaches {
    report_cache: Vec<HalfspaceReportCacheEntry>,
    feasible_cache: Vec<HalfspaceFeasibilityCacheEntry>,
    seed_geometry_cache: Vec<SupportCellSeedGeometryCacheEntry>,
    centroid_subset_seed_cache: Vec<Point3CentroidSubsetFamilyCacheEntry>,
    support_seed_family_cache: Vec<SupportCellSeedFamiliesCacheEntry>,
    support_direct_target_cache: Vec<SupportDirectReferenceTargetsCacheEntry>,
    projected_root_cache: Vec<ProjectedRootReferenceFamilyCacheEntry>,
    projection_escape_axis_options_cache:
        std::cell::RefCell<Vec<ProjectionEscapeAxisOptionsCacheEntry>>,
    projection_escape_search_cache: std::cell::RefCell<Vec<ProjectionEscapeSearchCacheEntry>>,
    shifted_projected_family_cache: Vec<ShiftedProjectedCellFamilyCacheEntry>,
    shifted_support_family_cache: Vec<ShiftedSupportCellFamilyCacheEntry>,
    reference_witness_cache: std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    strict_contains_cache: std::cell::RefCell<Vec<ReferenceHalfspaceContainmentCacheEntry>>,
    pure_halfspace_contains_cache:
        std::cell::RefCell<Vec<ReferencePureHalfspaceContainmentCacheEntry>>,
    trace_cache: Vec<ReferenceTargetTraceCacheEntry>,
    validity_cache: Vec<ReferenceBoundsValidityCacheEntry>,
    support_surface_cache: Vec<SupportSurfaceCacheEntry>,
    target_cache: std::cell::RefCell<Vec<SupportTargetFamilyCacheEntry>>,
    accept_cache: std::cell::RefCell<Vec<SupportReferenceAcceptCacheEntry>>,
    projected_reference_result_cache: Vec<ProjectedReferenceResultCacheEntry>,
    support_reference_result_cache: Vec<SupportReferenceResultCacheEntry>,
    search_cache:
        std::cell::RefCell<Vec<SupportPlaneCellSearchCacheEntry<(ReferenceTarget, Vec<i32>)>>>,
}

impl SupportReferenceQueryCaches {
    fn reset_per_reference_call_caches(&mut self) {
        self.support_seed_family_cache.clear();
        self.support_direct_target_cache.clear();
        self.projected_root_cache.clear();
        self.projection_escape_axis_options_cache.get_mut().clear();
        self.projection_escape_search_cache.get_mut().clear();
        self.shifted_projected_family_cache.clear();
        self.shifted_support_family_cache.clear();
        self.strict_contains_cache.get_mut().clear();
        self.pure_halfspace_contains_cache.get_mut().clear();
        self.trace_cache.clear();
        self.validity_cache.clear();
        self.support_surface_cache.clear();
        self.target_cache.get_mut().clear();
        self.accept_cache.get_mut().clear();
        self.projected_reference_result_cache.clear();
        self.support_reference_result_cache.clear();
        self.search_cache.get_mut().clear();
    }
}

#[derive(Clone, Debug, PartialEq)]
struct SupportCellSeedGeometryState {
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    saw_unknown: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct SupportCellSeedFamiliesState {
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    saw_unknown: bool,
}

#[derive(Clone)]
struct SupportCellSeedGeometryCacheEntry {
    halfspaces: Vec<LimitPlane3>,
    geometry: HypermeshResult<SupportCellSeedGeometryState>,
}

#[derive(Clone)]
struct Point3CentroidSubsetFamilyCacheEntry {
    vertices: Vec<Point3>,
    family: HypermeshResult<Point3FamilyState>,
}

#[derive(Clone)]
struct SupportCellSeedFamiliesCacheEntry {
    bounds: Aabb,
    halfspaces: Vec<LimitPlane3>,
    report: Option<hyperlimit::HalfspaceFeasibilityReport>,
    families: HypermeshResult<SupportCellSeedFamiliesState>,
}

#[derive(Clone)]
struct SupportDirectReferenceTargetsCacheEntry {
    bounds: Aabb,
    halfspaces: Vec<LimitPlane3>,
    report: Option<hyperlimit::HalfspaceFeasibilityReport>,
    targets: HypermeshResult<(Vec<ReferenceTarget>, bool)>,
}

fn cached_support_cell_seed_geometry_with(
    cache: &mut Vec<SupportCellSeedGeometryCacheEntry>,
    halfspaces: &[LimitPlane3],
    build: impl FnOnce() -> HypermeshResult<SupportCellSeedGeometryState>,
) -> HypermeshResult<SupportCellSeedGeometryState> {
    if let Some(existing) = cache
        .iter()
        .find(|existing| limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces))
    {
        return existing.geometry.clone();
    }

    let geometry = build();
    cache.push(SupportCellSeedGeometryCacheEntry {
        halfspaces: halfspaces.to_vec(),
        geometry: geometry.clone(),
    });
    geometry
}

fn cached_point3_centroid_subset_family_from_vertices_with(
    cache: &mut Vec<Point3CentroidSubsetFamilyCacheEntry>,
    vertices: &[Point3],
    build: impl FnOnce() -> HypermeshResult<Point3FamilyState>,
) -> HypermeshResult<Point3FamilyState> {
    if let Some(existing) = cache
        .iter()
        .find(|existing| point_families_match_as_sets(&existing.vertices, vertices))
    {
        return existing.family.clone();
    }

    let family = build();
    cache.push(Point3CentroidSubsetFamilyCacheEntry {
        vertices: vertices.to_vec(),
        family: family.clone(),
    });
    family
}

fn cached_support_cell_seed_families_with(
    cache: &mut Vec<SupportCellSeedFamiliesCacheEntry>,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    build: impl FnOnce() -> HypermeshResult<SupportCellSeedFamiliesState>,
) -> HypermeshResult<SupportCellSeedFamiliesState> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.bounds == *bounds
            && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            && support_seed_optional_halfspace_reports_match_for_cache(
                &existing.halfspaces,
                existing.report.as_ref(),
                halfspaces,
                report,
            )
    }) {
        return existing.families.clone();
    }

    let families = build();
    cache.push(SupportCellSeedFamiliesCacheEntry {
        bounds: bounds.clone(),
        halfspaces: halfspaces.to_vec(),
        report: report.cloned(),
        families: families.clone(),
    });
    families
}

fn cached_support_direct_reference_targets_with(
    cache: &mut Vec<SupportDirectReferenceTargetsCacheEntry>,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    build: impl FnOnce() -> HypermeshResult<(Vec<ReferenceTarget>, bool)>,
) -> HypermeshResult<(Vec<ReferenceTarget>, bool)> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.bounds == *bounds
            && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            && support_seed_optional_halfspace_reports_match_for_cache(
                &existing.halfspaces,
                existing.report.as_ref(),
                halfspaces,
                report,
            )
    }) {
        return existing.targets.clone();
    }

    let targets = build();
    cache.push(SupportDirectReferenceTargetsCacheEntry {
        bounds: bounds.clone(),
        halfspaces: halfspaces.to_vec(),
        report: report.cloned(),
        targets: targets.clone(),
    });
    targets
}

fn prime_support_reference_query_caches_with_known_halfspace_report(
    query_caches: &mut SupportReferenceQueryCaches,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    saw_unknown: bool,
) {
    let cached_report = if saw_unknown && report.is_none() {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(report.cloned())
    };
    if !query_caches
        .report_cache
        .iter()
        .any(|existing| limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces))
    {
        query_caches.report_cache.push(HalfspaceReportCacheEntry {
            halfspaces: halfspaces.to_vec(),
            report: cached_report.clone(),
        });
    }

    let cached_feasible = match &cached_report {
        Ok(Some(report)) => Some(Ok(report.status == HalfspaceFeasibility::Feasible)),
        Ok(None) => None,
        Err(err) => Some(Err(err.clone())),
    };
    if let Some(cached_feasible) = cached_feasible
        && !query_caches
            .feasible_cache
            .iter()
            .any(|existing| limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces))
    {
        query_caches
            .feasible_cache
            .push(HalfspaceFeasibilityCacheEntry {
                halfspaces: halfspaces.to_vec(),
                feasible: cached_feasible,
            });
    }
}

#[cfg(test)]
fn cached_reference_target_trace_with(
    cache: &mut Vec<ReferenceTargetTraceCacheEntry>,
    target: &ReferenceTarget,
    trace: impl FnOnce(&ReferenceTarget) -> HypermeshResult<Option<Vec<i32>>>,
) -> HypermeshResult<Option<Vec<i32>>> {
    cached_reference_target_trace_with_context(cache, None, target, trace)
}

fn cached_reference_target_trace_with_context(
    cache: &mut Vec<ReferenceTargetTraceCacheEntry>,
    context: Option<&SupportReferenceCacheContextKey>,
    target: &ReferenceTarget,
    trace: impl FnOnce(&ReferenceTarget) -> HypermeshResult<Option<Vec<i32>>>,
) -> HypermeshResult<Option<Vec<i32>>> {
    if let Some(existing) = cache.iter().find(|existing| {
        support_reference_cache_context_matches(existing.context.as_ref(), context)
            && reference_targets_match_for_trace_cache(&existing.target, target)
    }) {
        return existing.winding.clone();
    }

    let winding = trace(target);
    cache.push(ReferenceTargetTraceCacheEntry {
        context: context.cloned(),
        target: target.clone(),
        winding: winding.clone(),
    });
    winding
}

#[cfg(test)]
fn cached_reference_bounds_validity_with(
    cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
    bounds: &Aabb,
    point: &Point3,
    query: impl FnOnce(&Point3) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    cached_reference_bounds_validity_with_context(cache, None, bounds, point, query)
}

fn cached_reference_bounds_validity_with_context(
    cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
    context: Option<&SupportReferenceCacheContextKey>,
    bounds: &Aabb,
    point: &Point3,
    query: impl FnOnce(&Point3) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().find(|existing| {
        support_reference_polygon_only_context_matches(existing.context.as_ref(), context)
            && existing.bounds == *bounds
            && existing.point == *point
    }) {
        return existing.is_valid.clone();
    }

    let is_valid = query(point);
    cache.push(ReferenceBoundsValidityCacheEntry {
        context: context.map(support_reference_polygon_context_key_from_support_context),
        bounds: bounds.clone(),
        point: point.clone(),
        is_valid: is_valid.clone(),
    });
    is_valid
}

fn cached_reference_halfspace_containment_with(
    cache: &mut Vec<ReferenceHalfspaceContainmentCacheEntry>,
    bounds: &Aabb,
    point: &Point3,
    halfspaces: &[LimitPlane3],
    query: impl FnOnce(&Point3, &Aabb, &[LimitPlane3]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.bounds == *bounds
            && existing.point == *point
            && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
    }) {
        return existing.contains.clone();
    }

    let contains = query(point, bounds, halfspaces);
    cache.push(ReferenceHalfspaceContainmentCacheEntry {
        bounds: bounds.clone(),
        point: point.clone(),
        halfspaces: halfspaces.to_vec(),
        contains: contains.clone(),
    });
    contains
}

fn cached_pure_halfspace_containment_with(
    cache: &mut Vec<ReferencePureHalfspaceContainmentCacheEntry>,
    point: &Point3,
    halfspaces: &[LimitPlane3],
    query: impl FnOnce(&Point3, &[LimitPlane3]) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.point == *point
            && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
    }) {
        return existing.contains.clone();
    }

    let contains = query(point, halfspaces);
    cache.push(ReferencePureHalfspaceContainmentCacheEntry {
        point: point.clone(),
        halfspaces: halfspaces.to_vec(),
        contains: contains.clone(),
    });
    contains
}

fn cached_support_surface_query_with_context(
    cache: &mut Vec<SupportSurfaceCacheEntry>,
    context: Option<&SupportReferenceCacheContextKey>,
    point: &Point3,
    query: impl FnOnce(&Point3) -> HypermeshResult<bool>,
) -> HypermeshResult<bool> {
    if let Some(existing) = cache.iter().find(|existing| {
        support_reference_polygon_only_context_matches(existing.context.as_ref(), context)
            && existing.point == *point
    }) {
        return existing.on_support_surface.clone();
    }

    let on_support_surface = query(point);
    cache.push(SupportSurfaceCacheEntry {
        context: context.map(support_reference_polygon_context_key_from_support_context),
        point: point.clone(),
        on_support_surface: on_support_surface.clone(),
    });
    on_support_surface
}

fn cached_reference_target_from_halfspace_witness_with(
    cache: &mut Vec<ReferenceWitnessTargetCacheEntry>,
    point: &Point3,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
    build: impl FnOnce() -> HypermeshResult<Option<ReferenceTarget>>,
) -> HypermeshResult<Option<ReferenceTarget>> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.point == *point
            && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            && active_halfspace_reports_match_as_sets(
                &existing.halfspaces,
                existing.active_planes,
                halfspaces,
                active_planes,
            )
    }) {
        return existing.target.clone();
    }

    let target = build();
    cache.push(ReferenceWitnessTargetCacheEntry {
        point: point.clone(),
        halfspaces: halfspaces.to_vec(),
        active_planes,
        target: target.clone(),
    });
    target
}

#[derive(Clone)]
struct SupportTargetFamilyCacheEntry {
    bounds: Aabb,
    halfspaces: Vec<LimitPlane3>,
    report: Option<hyperlimit::HalfspaceFeasibilityReport>,
    targets: HypermeshResult<Vec<ReferenceTarget>>,
}

fn cached_support_target_family_with(
    cache: &mut Vec<SupportTargetFamilyCacheEntry>,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    build: impl FnOnce(
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
    ) -> HypermeshResult<Vec<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.bounds == *bounds
            && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            && support_optional_halfspace_reports_match_for_cache(
                &existing.halfspaces,
                existing.report.as_ref(),
                halfspaces,
                report,
            )
    }) {
        return existing.targets.clone();
    }

    let targets = build(halfspaces, report);
    cache.push(SupportTargetFamilyCacheEntry {
        bounds: bounds.clone(),
        halfspaces: halfspaces.to_vec(),
        report: report.cloned(),
        targets: targets.clone(),
    });
    targets
}

#[derive(Clone)]
struct SupportReferenceAcceptCacheEntry {
    context: Option<SupportReferenceCacheContextKey>,
    bounds: Aabb,
    halfspaces: Vec<LimitPlane3>,
    report: Option<hyperlimit::HalfspaceFeasibilityReport>,
    accepted: HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
}

#[derive(Clone)]
struct SupportReferenceResultCacheEntry {
    context: SupportReferenceCacheContextKey,
    bounds: Aabb,
    halfspaces: Vec<LimitPlane3>,
    result: HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
}

#[derive(Clone)]
struct ProjectedReferenceResultCacheEntry {
    context: SupportReferenceCacheContextKey,
    bounds: Aabb,
    halfspaces: Vec<LimitPlane3>,
    result: HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
}

#[derive(Clone, Debug, PartialEq)]
struct SupportReferenceCacheContextKey {
    old_ref: Point3,
    old_ref_definitions: Vec<[Plane; 3]>,
    old_wnv: Vec<i32>,
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
}

#[derive(Clone, Debug, PartialEq)]
struct SupportReferencePolygonContextKey {
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
}

fn support_reference_cache_context_key(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    polygons: &[ConvexPolygon],
) -> SupportReferenceCacheContextKey {
    SupportReferenceCacheContextKey {
        old_ref: old_ref.clone(),
        old_ref_definitions: old_ref_definitions.to_vec(),
        old_wnv: old_wnv.to_vec(),
        polygon_profile: polygon_family_profile(polygons),
        polygons: polygons.to_vec(),
    }
}

fn support_reference_polygon_context_key_from_support_context(
    context: &SupportReferenceCacheContextKey,
) -> SupportReferencePolygonContextKey {
    SupportReferencePolygonContextKey {
        polygon_profile: context.polygon_profile.clone(),
        polygons: context.polygons.clone(),
    }
}

fn support_reference_cache_context_matches(
    existing: Option<&SupportReferenceCacheContextKey>,
    context: Option<&SupportReferenceCacheContextKey>,
) -> bool {
    match (existing, context) {
        (None, None) => true,
        (Some(existing), Some(context)) => {
            existing.old_ref == context.old_ref
                && reference_definition_families_match_as_sets(
                    &existing.old_ref_definitions,
                    &context.old_ref_definitions,
                )
                && existing.old_wnv == context.old_wnv
                && existing.polygon_profile == context.polygon_profile
                && polygon_families_match_as_multisets(&existing.polygons, &context.polygons)
        }
        _ => false,
    }
}

fn support_reference_cache_context_matches_exact_state(
    existing: Option<&SupportReferenceCacheContextKey>,
    context: Option<&SupportReferenceCacheContextKey>,
) -> bool {
    match (existing, context) {
        (None, None) => true,
        (Some(existing), Some(context)) => existing == context,
        _ => false,
    }
}

fn support_reference_polygon_context_matches(
    existing: Option<&SupportReferenceCacheContextKey>,
    context: Option<&SupportReferenceCacheContextKey>,
) -> bool {
    match (existing, context) {
        (None, None) => true,
        (Some(existing), Some(context)) => {
            existing.polygon_profile == context.polygon_profile
                && polygon_families_match_as_multisets(&existing.polygons, &context.polygons)
        }
        _ => false,
    }
}

fn support_reference_polygon_only_context_matches(
    existing: Option<&SupportReferencePolygonContextKey>,
    context: Option<&SupportReferenceCacheContextKey>,
) -> bool {
    match (existing, context) {
        (None, None) => true,
        (Some(existing), Some(context)) => {
            existing.polygon_profile == context.polygon_profile
                && polygon_families_match_as_multisets(&existing.polygons, &context.polygons)
        }
        _ => false,
    }
}

fn cached_support_reference_accept_with(
    cache: &mut Vec<SupportReferenceAcceptCacheEntry>,
    context: Option<&SupportReferenceCacheContextKey>,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    accept: impl FnOnce(
        &[LimitPlane3],
        Option<&hyperlimit::HalfspaceFeasibilityReport>,
    ) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    if let Some(existing) = cache
        .iter()
        .rev()
        .find(|existing| {
            support_reference_cache_context_matches_exact_state(existing.context.as_ref(), context)
                && existing.bounds == *bounds
                && existing.halfspaces == halfspaces
                && existing.report.as_ref() == report
        })
        .cloned()
    {
        return existing.accepted;
    }

    if let Some(existing) = cache
        .iter()
        .rev()
        .find(|existing| {
            support_reference_cache_context_matches(existing.context.as_ref(), context)
                && existing.bounds == *bounds
                && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
                && support_optional_halfspace_reports_match_for_cache(
                    &existing.halfspaces,
                    existing.report.as_ref(),
                    halfspaces,
                    report,
                )
        })
        .cloned()
    {
        if !cache.iter().any(|current| {
            support_reference_cache_context_matches_exact_state(current.context.as_ref(), context)
                && current.bounds == *bounds
                && current.halfspaces == halfspaces
                && current.report.as_ref() == report
        }) {
            cache.push(SupportReferenceAcceptCacheEntry {
                context: context.cloned(),
                bounds: bounds.clone(),
                halfspaces: halfspaces.to_vec(),
                report: report.cloned(),
                accepted: existing.accepted.clone(),
            });
        }
        return existing.accepted.clone();
    }

    let accepted = accept(halfspaces, report);
    cache.push(SupportReferenceAcceptCacheEntry {
        context: context.cloned(),
        bounds: bounds.clone(),
        halfspaces: halfspaces.to_vec(),
        report: report.cloned(),
        accepted: accepted.clone(),
    });
    accepted
}

fn reusable_support_reference_accept_if_certified(
    cache: &mut Vec<SupportReferenceAcceptCacheEntry>,
    context: &SupportReferenceCacheContextKey,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut reused = None;
    for existing in cache.iter().rev() {
        if !limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            || !support_reference_polygon_context_matches(existing.context.as_ref(), Some(context))
            || existing
                .context
                .as_ref()
                .is_none_or(|existing| existing.old_wnv != context.old_wnv)
            || !support_optional_halfspace_reports_match_for_cache(
                &existing.halfspaces,
                existing.report.as_ref(),
                halfspaces,
                report,
            )
        {
            continue;
        }
        let Ok(Some((target, winding))) = &existing.accepted else {
            continue;
        };
        let valid_for_bounds = cached_reference_bounds_validity_with_context(
            validity_cache,
            Some(context),
            bounds,
            &target.point,
            |point| is_certified_valid_reference_for_bounds(point, bounds, &context.polygons),
        )?;
        if !valid_for_bounds {
            continue;
        }
        reused = Some((target.clone(), winding.clone()));
        break;
    }
    if let Some((target, winding)) = reused {
        let reused = Some((target, winding));
        if !cache.iter().any(|existing| {
            support_reference_cache_context_matches(existing.context.as_ref(), Some(context))
                && existing.bounds == *bounds
                && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
                && support_optional_halfspace_reports_match_for_cache(
                    &existing.halfspaces,
                    existing.report.as_ref(),
                    halfspaces,
                    report,
                )
        }) {
            cache.push(SupportReferenceAcceptCacheEntry {
                context: Some(context.clone()),
                bounds: bounds.clone(),
                halfspaces: halfspaces.to_vec(),
                report: report.cloned(),
                accepted: Ok(reused.clone()),
            });
        }
        return Ok(reused);
    }
    Ok(None)
}

fn reusable_support_reference_accept_from_cached_trace_if_certified(
    cache: &mut Vec<SupportReferenceAcceptCacheEntry>,
    context: &SupportReferenceCacheContextKey,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
    trace_cache: &mut Vec<ReferenceTargetTraceCacheEntry>,
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut reused = None;
    for existing in cache.iter().rev() {
        if !limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            || !support_reference_polygon_context_matches(existing.context.as_ref(), Some(context))
            || !support_optional_halfspace_reports_match_for_cache(
                &existing.halfspaces,
                existing.report.as_ref(),
                halfspaces,
                report,
            )
        {
            continue;
        }
        let Ok(Some((target, _))) = &existing.accepted else {
            continue;
        };
        let valid_for_bounds = cached_reference_bounds_validity_with_context(
            validity_cache,
            Some(context),
            bounds,
            &target.point,
            |point| is_certified_valid_reference_for_bounds(point, bounds, &context.polygons),
        )?;
        if !valid_for_bounds {
            continue;
        }

        if let Some(winding) = cached_reference_target_trace_with_context(
            trace_cache,
            Some(context),
            target,
            |target| {
                trace_reference_target_from_validated_bounds(
                    old_ref,
                    old_ref_definitions,
                    old_wnv,
                    bounds,
                    &context.polygons,
                    target,
                )
            },
        )? {
            reused = Some((target.clone(), winding));
            break;
        }
    }
    if let Some((target, winding)) = reused {
        let reused = Some((target, winding));
        if !cache.iter().any(|existing| {
            support_reference_cache_context_matches(existing.context.as_ref(), Some(context))
                && existing.bounds == *bounds
                && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
                && support_optional_halfspace_reports_match_for_cache(
                    &existing.halfspaces,
                    existing.report.as_ref(),
                    halfspaces,
                    report,
                )
        }) {
            cache.push(SupportReferenceAcceptCacheEntry {
                context: Some(context.clone()),
                bounds: bounds.clone(),
                halfspaces: halfspaces.to_vec(),
                report: report.cloned(),
                accepted: Ok(reused.clone()),
            });
        }
        return Ok(reused);
    }
    Ok(None)
}

fn cached_support_reference_result_with(
    cache: &mut Vec<SupportReferenceResultCacheEntry>,
    context: &SupportReferenceCacheContextKey,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    build: impl FnOnce() -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    if let Some(existing) = cache
        .iter()
        .rev()
        .find(|existing| {
            existing.context == *context
                && existing.bounds == *bounds
                && existing.halfspaces == halfspaces
        })
        .cloned()
    {
        return existing.result;
    }

    if let Some(existing) = cache
        .iter()
        .rev()
        .find(|existing| {
            existing.bounds == *bounds
                && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
                && support_reference_cache_context_matches(Some(&existing.context), Some(context))
        })
        .cloned()
    {
        if !cache.iter().any(|current| {
            current.context == *context
                && current.bounds == *bounds
                && current.halfspaces == halfspaces
        }) {
            cache.push(SupportReferenceResultCacheEntry {
                context: context.clone(),
                bounds: bounds.clone(),
                halfspaces: halfspaces.to_vec(),
                result: existing.result.clone(),
            });
        }
        return existing.result.clone();
    }

    let result = build();
    cache.push(SupportReferenceResultCacheEntry {
        context: context.clone(),
        bounds: bounds.clone(),
        halfspaces: halfspaces.to_vec(),
        result: result.clone(),
    });
    result
}

fn reusable_support_reference_result_if_certified(
    cache: &mut Vec<SupportReferenceResultCacheEntry>,
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    halfspaces: &[LimitPlane3],
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let context =
        support_reference_cache_context_key(old_ref, old_ref_definitions, old_wnv, polygons);
    let polygon_context = Some(&context);
    let mut reused = None;
    for existing in cache.iter().rev() {
        if !limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            || existing.context.old_wnv != old_wnv
            || !support_reference_polygon_context_matches(Some(&existing.context), polygon_context)
        {
            continue;
        }
        let Ok(Some((target, winding))) = &existing.result else {
            continue;
        };
        let valid_for_bounds = cached_reference_bounds_validity_with_context(
            validity_cache,
            Some(&context),
            bounds,
            &target.point,
            |point| is_certified_valid_reference_for_bounds(point, bounds, polygons),
        )?;
        if !valid_for_bounds {
            continue;
        }
        reused = Some((target.clone(), winding.clone()));
        break;
    }
    if let Some((target, winding)) = reused {
        let reused = Some((target, winding));
        if !cache.iter().any(|existing| {
            existing.bounds == *bounds
                && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
                && support_reference_cache_context_matches(Some(&existing.context), Some(&context))
        }) {
            cache.push(SupportReferenceResultCacheEntry {
                context,
                bounds: bounds.clone(),
                halfspaces: halfspaces.to_vec(),
                result: Ok(reused.clone()),
            });
        }
        return Ok(reused);
    }
    Ok(None)
}

fn reusable_support_reference_result_from_cached_trace_if_certified(
    cache: &mut Vec<SupportReferenceResultCacheEntry>,
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    halfspaces: &[LimitPlane3],
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
    trace_cache: &mut Vec<ReferenceTargetTraceCacheEntry>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let context =
        support_reference_cache_context_key(old_ref, old_ref_definitions, old_wnv, polygons);
    let polygon_context = Some(&context);
    let mut reused = None;
    for existing in cache.iter().rev() {
        if !limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            || !support_reference_polygon_context_matches(Some(&existing.context), polygon_context)
        {
            continue;
        }
        let Ok(Some((target, _))) = &existing.result else {
            continue;
        };
        let valid_for_bounds = cached_reference_bounds_validity_with_context(
            validity_cache,
            Some(&context),
            bounds,
            &target.point,
            |point| is_certified_valid_reference_for_bounds(point, bounds, polygons),
        )?;
        if !valid_for_bounds {
            continue;
        }

        if let Some(winding) = cached_reference_target_trace_with_context(
            trace_cache,
            Some(&context),
            target,
            |target| {
                trace_reference_target_from_validated_bounds(
                    old_ref,
                    old_ref_definitions,
                    old_wnv,
                    bounds,
                    polygons,
                    target,
                )
            },
        )? {
            reused = Some((target.clone(), winding));
            break;
        }
    }
    if let Some((target, winding)) = reused {
        let reused = Some((target, winding));
        if !cache.iter().any(|existing| {
            existing.bounds == *bounds
                && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
                && support_reference_cache_context_matches(Some(&existing.context), Some(&context))
        }) {
            cache.push(SupportReferenceResultCacheEntry {
                context,
                bounds: bounds.clone(),
                halfspaces: halfspaces.to_vec(),
                result: Ok(reused.clone()),
            });
        }
        return Ok(reused);
    }
    Ok(None)
}

fn cached_projected_reference_result_with(
    cache: &mut Vec<ProjectedReferenceResultCacheEntry>,
    context: &SupportReferenceCacheContextKey,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    build: impl FnOnce() -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    if let Some(existing) = cache
        .iter()
        .rev()
        .find(|existing| {
            existing.context == *context
                && existing.bounds == *bounds
                && existing.halfspaces == halfspaces
        })
        .cloned()
    {
        return existing.result;
    }

    if let Some(existing) = cache
        .iter()
        .rev()
        .find(|existing| {
            existing.bounds == *bounds
                && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
                && support_reference_cache_context_matches(Some(&existing.context), Some(context))
        })
        .cloned()
    {
        if !cache.iter().any(|current| {
            current.context == *context
                && current.bounds == *bounds
                && current.halfspaces == halfspaces
        }) {
            cache.push(ProjectedReferenceResultCacheEntry {
                context: context.clone(),
                bounds: bounds.clone(),
                halfspaces: halfspaces.to_vec(),
                result: existing.result.clone(),
            });
        }
        return existing.result.clone();
    }

    let result = build();
    cache.push(ProjectedReferenceResultCacheEntry {
        context: context.clone(),
        bounds: bounds.clone(),
        halfspaces: halfspaces.to_vec(),
        result: result.clone(),
    });
    result
}

fn reusable_projected_reference_result_if_certified(
    cache: &mut Vec<ProjectedReferenceResultCacheEntry>,
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    halfspaces: &[LimitPlane3],
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let context =
        support_reference_cache_context_key(old_ref, old_ref_definitions, old_wnv, polygons);
    let polygon_context = Some(&context);
    let mut reused = None;
    for existing in cache.iter().rev() {
        if !limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            || existing.context.old_wnv != old_wnv
            || !support_reference_polygon_context_matches(Some(&existing.context), polygon_context)
        {
            continue;
        }
        let Ok(Some((target, winding))) = &existing.result else {
            continue;
        };
        let valid_for_bounds = cached_reference_bounds_validity_with_context(
            validity_cache,
            Some(&context),
            bounds,
            &target.point,
            |point| is_certified_valid_reference_for_bounds(point, bounds, polygons),
        )?;
        if !valid_for_bounds {
            continue;
        }
        reused = Some((target.clone(), winding.clone()));
        break;
    }
    if let Some((target, winding)) = reused {
        let reused = Some((target, winding));
        if !cache.iter().any(|existing| {
            existing.bounds == *bounds
                && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
                && support_reference_cache_context_matches(Some(&existing.context), Some(&context))
        }) {
            cache.push(ProjectedReferenceResultCacheEntry {
                context,
                bounds: bounds.clone(),
                halfspaces: halfspaces.to_vec(),
                result: Ok(reused.clone()),
            });
        }
        return Ok(reused);
    }
    Ok(None)
}

fn reusable_projected_reference_result_from_cached_trace_if_certified(
    cache: &mut Vec<ProjectedReferenceResultCacheEntry>,
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    halfspaces: &[LimitPlane3],
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
    trace_cache: &mut Vec<ReferenceTargetTraceCacheEntry>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let context =
        support_reference_cache_context_key(old_ref, old_ref_definitions, old_wnv, polygons);
    let polygon_context = Some(&context);
    let mut reused = None;
    for existing in cache.iter().rev() {
        if !limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            || !support_reference_polygon_context_matches(Some(&existing.context), polygon_context)
        {
            continue;
        }
        let Ok(Some((target, _))) = &existing.result else {
            continue;
        };
        let valid_for_bounds = cached_reference_bounds_validity_with_context(
            validity_cache,
            Some(&context),
            bounds,
            &target.point,
            |point| is_certified_valid_reference_for_bounds(point, bounds, polygons),
        )?;
        if !valid_for_bounds {
            continue;
        }

        if let Some(winding) = cached_reference_target_trace_with_context(
            trace_cache,
            Some(&context),
            target,
            |target| {
                trace_reference_target_from_validated_bounds(
                    old_ref,
                    old_ref_definitions,
                    old_wnv,
                    bounds,
                    polygons,
                    target,
                )
            },
        )? {
            reused = Some((target.clone(), winding));
            break;
        }
    }
    if let Some((target, winding)) = reused {
        let reused = Some((target, winding));
        if !cache.iter().any(|existing| {
            existing.bounds == *bounds
                && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
                && support_reference_cache_context_matches(Some(&existing.context), Some(&context))
        }) {
            cache.push(ProjectedReferenceResultCacheEntry {
                context,
                bounds: bounds.clone(),
                halfspaces: halfspaces.to_vec(),
                result: Ok(reused.clone()),
            });
        }
        return Ok(reused);
    }
    Ok(None)
}

#[derive(Clone)]
struct SupportPlaneCellSearchCacheEntry<T: Clone> {
    context: Option<SupportReferenceCacheContextKey>,
    bounds: Aabb,
    polygon_index: usize,
    halfspaces: Vec<LimitPlane3>,
    result: HypermeshResult<Option<T>>,
}

fn cached_support_plane_cell_search_with<T: Clone>(
    cache: &std::cell::RefCell<Vec<SupportPlaneCellSearchCacheEntry<T>>>,
    context: Option<&SupportReferenceCacheContextKey>,
    _preferred_order: [bool; 2],
    bounds: &Aabb,
    polygon_index: usize,
    halfspaces: Vec<LimitPlane3>,
    search: impl FnOnce() -> HypermeshResult<Option<T>>,
) -> HypermeshResult<Option<T>> {
    let exact_existing = {
        let cache_ref = cache.borrow();
        cache_ref
            .iter()
            .rev()
            .find(|existing| {
                support_reference_cache_context_matches_exact_state(
                    existing.context.as_ref(),
                    context,
                ) && existing.bounds == *bounds
                    && existing.polygon_index == polygon_index
                    && existing.halfspaces == halfspaces
            })
            .cloned()
    };
    if let Some(existing) = exact_existing {
        return existing.result;
    }

    let existing = {
        let cache_ref = cache.borrow();
        cache_ref
            .iter()
            .rev()
            .find(|existing| {
                support_reference_cache_context_matches(existing.context.as_ref(), context)
                    && existing.bounds == *bounds
                    && existing.polygon_index == polygon_index
                    && limit_plane_families_match_as_sets(&existing.halfspaces, &halfspaces)
            })
            .cloned()
    };
    if let Some(existing) = existing {
        if !cache.borrow().iter().any(|current| {
            support_reference_cache_context_matches_exact_state(current.context.as_ref(), context)
                && current.bounds == *bounds
                && current.polygon_index == polygon_index
                && current.halfspaces == halfspaces
        }) {
            cache.borrow_mut().push(SupportPlaneCellSearchCacheEntry {
                context: context.cloned(),
                bounds: bounds.clone(),
                polygon_index,
                halfspaces: halfspaces.clone(),
                result: existing.result.clone(),
            });
        }
        return existing.result.clone();
    }

    let result = search();
    cache.borrow_mut().push(SupportPlaneCellSearchCacheEntry {
        context: context.cloned(),
        bounds: bounds.clone(),
        polygon_index,
        halfspaces,
        result: result.clone(),
    });
    result
}

fn reusable_support_plane_cell_search_result_if_certified(
    cache: &std::cell::RefCell<Vec<SupportPlaneCellSearchCacheEntry<(ReferenceTarget, Vec<i32>)>>>,
    context: &SupportReferenceCacheContextKey,
    bounds: &Aabb,
    polygon_index: usize,
    halfspaces: &[LimitPlane3],
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut reused = None;
    {
        let cache_ref = cache.borrow();
        for existing in cache_ref.iter().rev() {
            if !support_reference_cache_context_matches(existing.context.as_ref(), Some(context))
                || existing.polygon_index != polygon_index
                || !limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            {
                continue;
            }
            let Ok(Some((target, winding))) = &existing.result else {
                continue;
            };
            if !bounds_contains_bounds(&existing.bounds, bounds)? {
                continue;
            }

            let valid_for_bounds = cached_reference_bounds_validity_with_context(
                validity_cache,
                Some(context),
                bounds,
                &target.point,
                |point| is_certified_valid_reference_for_bounds(point, bounds, &context.polygons),
            )?;
            if !valid_for_bounds {
                continue;
            }

            reused = Some((target.clone(), winding.clone()));
            break;
        }
    }
    if let Some((target, winding)) = reused {
        let reused = Some((target, winding));
        if !cache.borrow().iter().any(|existing| {
            support_reference_cache_context_matches(existing.context.as_ref(), Some(context))
                && existing.bounds == *bounds
                && existing.polygon_index == polygon_index
                && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
        }) {
            cache.borrow_mut().push(SupportPlaneCellSearchCacheEntry {
                context: Some(context.clone()),
                bounds: bounds.clone(),
                polygon_index,
                halfspaces: halfspaces.to_vec(),
                result: Ok(reused.clone()),
            });
        }
        return Ok(reused);
    }
    Ok(None)
}

fn reusable_support_plane_cell_search_result_from_cached_trace_if_certified(
    cache: &std::cell::RefCell<Vec<SupportPlaneCellSearchCacheEntry<(ReferenceTarget, Vec<i32>)>>>,
    context: &SupportReferenceCacheContextKey,
    bounds: &Aabb,
    polygon_index: usize,
    halfspaces: &[LimitPlane3],
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
    trace_cache: &mut Vec<ReferenceTargetTraceCacheEntry>,
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut reused = None;
    {
        let cache_ref = cache.borrow();
        for existing in cache_ref.iter().rev() {
            if !support_reference_polygon_context_matches(existing.context.as_ref(), Some(context))
                || existing.polygon_index != polygon_index
                || !limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            {
                continue;
            }
            let Ok(Some((target, _))) = &existing.result else {
                continue;
            };
            if !bounds_contains_bounds(&existing.bounds, bounds)? {
                continue;
            }

            let valid_for_bounds = cached_reference_bounds_validity_with_context(
                validity_cache,
                Some(context),
                bounds,
                &target.point,
                |point| is_certified_valid_reference_for_bounds(point, bounds, &context.polygons),
            )?;
            if !valid_for_bounds {
                continue;
            }

            if let Some(winding) = cached_reference_target_trace_with_context(
                trace_cache,
                Some(context),
                target,
                |target| {
                    trace_reference_target_from_validated_bounds(
                        old_ref,
                        old_ref_definitions,
                        old_wnv,
                        bounds,
                        &context.polygons,
                        target,
                    )
                },
            )? {
                reused = Some((target.clone(), winding));
                break;
            }
        }
    }
    if let Some((target, winding)) = reused {
        let reused = Some((target, winding));
        if !cache.borrow().iter().any(|existing| {
            support_reference_cache_context_matches(existing.context.as_ref(), Some(context))
                && existing.bounds == *bounds
                && existing.polygon_index == polygon_index
                && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
        }) {
            cache.borrow_mut().push(SupportPlaneCellSearchCacheEntry {
                context: Some(context.clone()),
                bounds: bounds.clone(),
                polygon_index,
                halfspaces: halfspaces.to_vec(),
                result: Ok(reused.clone()),
            });
        }
        return Ok(reused);
    }
    Ok(None)
}

fn optional_halfspace_reports_match_for_cache(
    left_halfspaces: &[LimitPlane3],
    left: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    right_halfspaces: &[LimitPlane3],
    right: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> bool {
    match (left, right) {
        (None, None) => true,
        (Some(left), Some(right)) => {
            left.status == right.status
                && left.witness == right.witness
                && optional_halfspace_certificates_match_for_cache(
                    left_halfspaces,
                    left.infeasibility_certificate.as_ref(),
                    right_halfspaces,
                    right.infeasibility_certificate.as_ref(),
                )
                && active_halfspace_reports_match_as_sets(
                    left_halfspaces,
                    left.active_planes,
                    right_halfspaces,
                    right.active_planes,
                )
        }
        _ => false,
    }
}

fn support_optional_halfspace_reports_match_for_cache(
    left_halfspaces: &[LimitPlane3],
    left: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    right_halfspaces: &[LimitPlane3],
    right: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> bool {
    if support_report_witness_for_cache(left).is_none()
        && support_report_witness_for_cache(right).is_none()
    {
        return true;
    }

    optional_halfspace_reports_match_for_cache(left_halfspaces, left, right_halfspaces, right)
}

fn support_seed_optional_halfspace_reports_match_for_cache(
    left_halfspaces: &[LimitPlane3],
    left: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    right_halfspaces: &[LimitPlane3],
    right: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> bool {
    match (
        support_report_witness_for_cache(left),
        support_report_witness_for_cache(right),
    ) {
        (None, None) => true,
        (Some(left_witness), Some(right_witness)) => left_witness == right_witness,
        _ => optional_halfspace_reports_match_for_cache(
            left_halfspaces,
            left,
            right_halfspaces,
            right,
        ),
    }
}

fn support_report_witness_for_cache(
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> Option<&Point3> {
    report
        .filter(|report| report.status == HalfspaceFeasibility::Feasible)
        .and_then(|report| report.witness.as_ref())
}

fn optional_halfspace_certificates_match_for_cache(
    left_halfspaces: &[LimitPlane3],
    left: Option<&hyperlimit::HalfspaceInfeasibilityCertificate>,
    right_halfspaces: &[LimitPlane3],
    right: Option<&hyperlimit::HalfspaceInfeasibilityCertificate>,
) -> bool {
    match (left, right) {
        (None, None) => true,
        (Some(left), Some(right)) => {
            left.offset_sum == right.offset_sum
                && active_halfspace_certificates_match_as_sets(
                    left_halfspaces,
                    left.active_planes,
                    &left.multipliers,
                    right_halfspaces,
                    right.active_planes,
                    &right.multipliers,
                )
        }
        _ => false,
    }
}

fn active_halfspace_reports_match_as_sets(
    left_halfspaces: &[LimitPlane3],
    left_active_planes: [Option<usize>; 3],
    right_halfspaces: &[LimitPlane3],
    right_active_planes: [Option<usize>; 3],
) -> bool {
    let left_planes = mapped_active_halfspace_planes(left_halfspaces, left_active_planes);
    let right_planes = mapped_active_halfspace_planes(right_halfspaces, right_active_planes);
    limit_plane_families_match_as_sets(&left_planes, &right_planes)
}

fn active_halfspace_certificates_match_as_sets(
    left_halfspaces: &[LimitPlane3],
    left_active_planes: [Option<usize>; 4],
    left_multipliers: &[Real; 4],
    right_halfspaces: &[LimitPlane3],
    right_active_planes: [Option<usize>; 4],
    right_multipliers: &[Real; 4],
) -> bool {
    let left_pairs = mapped_active_halfspace_certificate_pairs(
        left_halfspaces,
        left_active_planes,
        left_multipliers,
    );
    let right_pairs = mapped_active_halfspace_certificate_pairs(
        right_halfspaces,
        right_active_planes,
        right_multipliers,
    );
    limit_plane_multiplier_pairs_match_as_sets(&left_pairs, &right_pairs)
}

fn mapped_active_halfspace_planes(
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
) -> Vec<LimitPlane3> {
    active_planes
        .into_iter()
        .flatten()
        .filter_map(|index| halfspaces.get(index).cloned())
        .collect()
}

fn mapped_active_halfspace_certificate_pairs(
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 4],
    multipliers: &[Real; 4],
) -> Vec<(LimitPlane3, Real)> {
    active_planes
        .into_iter()
        .zip(multipliers.iter().cloned())
        .filter_map(|(index, multiplier)| {
            index.and_then(|index| {
                halfspaces
                    .get(index)
                    .cloned()
                    .map(|plane| (plane, multiplier))
            })
        })
        .collect()
}

fn limit_plane_multiplier_pairs_match_as_sets(
    left: &[(LimitPlane3, Real)],
    right: &[(LimitPlane3, Real)],
) -> bool {
    if left.len() != right.len() {
        return false;
    }

    let mut matched = vec![false; right.len()];
    for (left_plane, left_multiplier) in left {
        let Some((index, _)) =
            right
                .iter()
                .enumerate()
                .find(|(index, (right_plane, right_multiplier))| {
                    !matched[*index]
                        && *right_plane == *left_plane
                        && *right_multiplier == *left_multiplier
                })
        else {
            return false;
        };
        matched[index] = true;
    }

    true
}

#[derive(Clone, Debug, PartialEq)]
struct ShiftedProjectedCellFamilies {
    shifted: Vec<LimitPlane3>,
    report: Option<hyperlimit::HalfspaceFeasibilityReport>,
    saw_unknown: bool,
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
}

#[derive(Clone)]
struct ShiftedProjectedCellFamilyCacheEntry {
    bounds: Aabb,
    halfspaces: Vec<LimitPlane3>,
    seed: Point3,
    families: HypermeshResult<Option<ShiftedProjectedCellFamilies>>,
}

fn cached_shifted_projected_cell_families_with(
    cache: &mut Vec<ShiftedProjectedCellFamilyCacheEntry>,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
    build: impl FnOnce() -> HypermeshResult<Option<ShiftedProjectedCellFamilies>>,
) -> HypermeshResult<Option<ShiftedProjectedCellFamilies>> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.bounds == *bounds
            && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            && existing.seed == *seed
    }) {
        return existing.families.clone();
    }

    let families = build();
    cache.push(ShiftedProjectedCellFamilyCacheEntry {
        bounds: bounds.clone(),
        halfspaces: halfspaces.to_vec(),
        seed: seed.clone(),
        families: families.clone(),
    });
    families
}

#[derive(Clone, Debug, PartialEq)]
struct ShiftedSupportCellFamilies {
    shifted: Vec<LimitPlane3>,
    report: Option<hyperlimit::HalfspaceFeasibilityReport>,
    saw_unknown: bool,
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
}

#[derive(Clone)]
struct ShiftedSupportCellFamilyCacheEntry {
    bounds: Aabb,
    halfspaces: Vec<LimitPlane3>,
    seed: Point3,
    families: HypermeshResult<Option<ShiftedSupportCellFamilies>>,
}

fn cached_shifted_support_cell_families_with(
    cache: &mut Vec<ShiftedSupportCellFamilyCacheEntry>,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
    build: impl FnOnce() -> HypermeshResult<Option<ShiftedSupportCellFamilies>>,
) -> HypermeshResult<Option<ShiftedSupportCellFamilies>> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.bounds == *bounds
            && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            && existing.seed == *seed
    }) {
        return existing.families.clone();
    }

    let families = build();
    cache.push(ShiftedSupportCellFamilyCacheEntry {
        bounds: bounds.clone(),
        halfspaces: halfspaces.to_vec(),
        seed: seed.clone(),
        families: families.clone(),
    });
    families
}

type ProjectionEscapeAxisOptions = Vec<(Vec<Real>, Vec<Real>)>;

#[derive(Clone, Debug, PartialEq)]
struct ProjectionEscapeAxisOptionsState {
    axis_options: ProjectionEscapeAxisOptions,
    saw_unknown: bool,
}

#[derive(Clone)]
struct ProjectionEscapeAxisOptionsCacheEntry {
    point: Point3,
    bounds: Aabb,
    polygons: Vec<ConvexPolygon>,
    state: ProjectionEscapeAxisOptionsState,
}

#[cfg(test)]
fn cached_projection_escape_axis_options_with(
    cache: &mut Vec<ProjectionEscapeAxisOptionsCacheEntry>,
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    build: impl FnOnce() -> HypermeshResult<ProjectionEscapeAxisOptions>,
) -> HypermeshResult<ProjectionEscapeAxisOptions> {
    Ok(cached_projection_escape_axis_options_state_with(
        cache,
        projected,
        bounds,
        polygons,
        || {
            Ok(ProjectionEscapeAxisOptionsState {
                axis_options: build()?,
                saw_unknown: false,
            })
        },
    )?
    .axis_options)
}

fn cached_projection_escape_axis_options_state_with(
    cache: &mut Vec<ProjectionEscapeAxisOptionsCacheEntry>,
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    build: impl FnOnce() -> HypermeshResult<ProjectionEscapeAxisOptionsState>,
) -> HypermeshResult<ProjectionEscapeAxisOptionsState> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.point == *projected
            && existing.bounds == *bounds
            && polygon_families_match_as_multisets(&existing.polygons, polygons)
    }) {
        return Ok(existing.state.clone());
    }

    let state = build()?;
    cache.push(ProjectionEscapeAxisOptionsCacheEntry {
        point: projected.clone(),
        bounds: bounds.clone(),
        polygons: polygons.to_vec(),
        state: state.clone(),
    });
    Ok(state)
}

#[derive(Clone)]
struct ProjectionEscapeSearchCacheEntry {
    context: SupportReferenceCacheContextKey,
    bounds: Aabb,
    result: HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
}

#[cfg(test)]
fn cached_reference_escape_search_with(
    cache: &mut Vec<ProjectionEscapeSearchCacheEntry>,
    context: &SupportReferenceCacheContextKey,
    bounds: &Aabb,
    search: impl FnOnce(&Aabb) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    if let Some(existing) = cache
        .iter()
        .rev()
        .find(|existing| existing.context == *context && existing.bounds == *bounds)
        .cloned()
    {
        return existing.result;
    }

    if let Some(existing) = cache
        .iter()
        .rev()
        .find(|existing| {
            existing.bounds == *bounds
                && support_reference_cache_context_matches(Some(&existing.context), Some(context))
        })
        .cloned()
    {
        if !cache
            .iter()
            .any(|current| current.context == *context && current.bounds == *bounds)
        {
            cache.push(ProjectionEscapeSearchCacheEntry {
                context: context.clone(),
                bounds: bounds.clone(),
                result: existing.result.clone(),
            });
        }
        return existing.result.clone();
    }

    let result = search(bounds);
    cache.push(ProjectionEscapeSearchCacheEntry {
        context: context.clone(),
        bounds: bounds.clone(),
        result: result.clone(),
    });
    result
}

fn cached_reference_escape_search_in_query_caches(
    query_caches: &mut SupportReferenceQueryCaches,
    context: &SupportReferenceCacheContextKey,
    bounds: &Aabb,
    search: impl FnOnce(
        &Aabb,
        &mut SupportReferenceQueryCaches,
    ) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    if let Some(existing) = query_caches
        .projection_escape_search_cache
        .borrow()
        .iter()
        .rev()
        .find(|existing| existing.context == *context && existing.bounds == *bounds)
        .cloned()
    {
        return existing.result;
    }

    let mut retraced = None;
    {
        let cache_ref = query_caches.projection_escape_search_cache.borrow();
        for existing in cache_ref.iter().rev() {
            if support_reference_cache_context_matches(Some(&existing.context), Some(context))
                || !support_reference_polygon_context_matches(
                    Some(&existing.context),
                    Some(context),
                )
            {
                continue;
            }
            let Ok(Some((target, _))) = &existing.result else {
                continue;
            };
            if !bounds_contains_bounds(&existing.bounds, bounds)? {
                continue;
            }
            let valid_for_bounds = cached_reference_bounds_validity_with_context(
                &mut query_caches.validity_cache,
                Some(context),
                bounds,
                &target.point,
                |point| is_certified_valid_reference_for_bounds(point, bounds, &context.polygons),
            )?;
            if !valid_for_bounds {
                continue;
            }
            if let Some(winding) = cached_reference_target_trace_with_context(
                &mut query_caches.trace_cache,
                Some(context),
                target,
                |target| {
                    trace_reference_target_from_validated_bounds(
                        &context.old_ref,
                        &context.old_ref_definitions,
                        &context.old_wnv,
                        bounds,
                        &context.polygons,
                        target,
                    )
                },
            )? {
                retraced = Some((target.clone(), winding));
                break;
            }
        }
    }
    if let Some((target, winding)) = retraced {
        let reused = Some((target, winding));
        if !query_caches
            .projection_escape_search_cache
            .borrow()
            .iter()
            .any(|existing| {
                existing.bounds == *bounds
                    && support_reference_cache_context_matches(
                        Some(&existing.context),
                        Some(context),
                    )
            })
        {
            query_caches
                .projection_escape_search_cache
                .borrow_mut()
                .push(ProjectionEscapeSearchCacheEntry {
                    context: context.clone(),
                    bounds: bounds.clone(),
                    result: Ok(reused.clone()),
                });
        }
        return Ok(reused);
    }

    let mut reused = None;
    {
        let cache_ref = query_caches.projection_escape_search_cache.borrow();
        for existing in cache_ref.iter().rev() {
            if !support_reference_cache_context_matches(Some(&existing.context), Some(context)) {
                continue;
            }
            let Ok(Some((target, winding))) = &existing.result else {
                continue;
            };
            if !bounds_contains_bounds(&existing.bounds, bounds)? {
                continue;
            }
            let valid_for_bounds = cached_reference_bounds_validity_with_context(
                &mut query_caches.validity_cache,
                Some(context),
                bounds,
                &target.point,
                |point| is_certified_valid_reference_for_bounds(point, bounds, &context.polygons),
            )?;
            if !valid_for_bounds {
                continue;
            }
            reused = Some((target.clone(), winding.clone()));
            break;
        }
    }
    if let Some((target, winding)) = reused {
        let reused = Some((target, winding));
        if !query_caches
            .projection_escape_search_cache
            .borrow()
            .iter()
            .any(|existing| {
                existing.bounds == *bounds
                    && support_reference_cache_context_matches(
                        Some(&existing.context),
                        Some(context),
                    )
            })
        {
            query_caches
                .projection_escape_search_cache
                .borrow_mut()
                .push(ProjectionEscapeSearchCacheEntry {
                    context: context.clone(),
                    bounds: bounds.clone(),
                    result: Ok(reused.clone()),
                });
        }
        return Ok(reused);
    }

    if let Some(existing) = query_caches
        .projection_escape_search_cache
        .borrow()
        .iter()
        .rev()
        .find(|existing| {
            existing.bounds == *bounds
                && support_reference_cache_context_matches(Some(&existing.context), Some(context))
        })
        .cloned()
    {
        if !query_caches
            .projection_escape_search_cache
            .borrow()
            .iter()
            .any(|current| current.context == *context && current.bounds == *bounds)
        {
            query_caches
                .projection_escape_search_cache
                .borrow_mut()
                .push(ProjectionEscapeSearchCacheEntry {
                    context: context.clone(),
                    bounds: bounds.clone(),
                    result: existing.result.clone(),
                });
        }
        return existing.result;
    }

    let result = search(bounds, query_caches);
    query_caches
        .projection_escape_search_cache
        .borrow_mut()
        .push(ProjectionEscapeSearchCacheEntry {
            context: context.clone(),
            bounds: bounds.clone(),
            result: result.clone(),
        });
    result
}

fn reference_target_from_halfspace_witness(
    point: &Point3,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
) -> HypermeshResult<Option<ReferenceTarget>> {
    match reference_definitions_from_active_halfspaces(point, halfspaces, active_planes) {
        Ok(found) => {
            let mut target = ReferenceTarget::with_definitions(point.clone(), found.definitions);
            if found.saw_unknown {
                target.uncertified_definition_fallback = true;
            }
            Ok(Some(target))
        }
        Err(crate::error::HypermeshError::UnknownClassification) => {
            Ok(Some(ReferenceTarget::axis_defined_fallback(point.clone())))
        }
        Err(err) => Err(err),
    }
}

#[cfg(test)]
fn projection_axis_escape_reference(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let axis_options = projection_escape_axis_options_family(projected, bounds, polygons)?;
    projection_axis_escape_reference_with_axis_options(projected, &axis_options, |corridor| {
        support_plane_cell_reference(old_ref, old_ref_definitions, old_wnv, corridor, polygons)
    })
}

#[cfg(test)]
fn projection_axis_escape_reference_with_axis_options(
    projected: &Point3,
    axis_options: &ProjectionEscapeAxisOptions,
    search: impl FnMut(&Aabb) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    projection_axis_escape_reference_with_axis_options_tracking_unknown(
        projected,
        axis_options,
        false,
        search,
    )
}

fn projection_axis_escape_reference_with_axis_options_tracking_unknown(
    projected: &Point3,
    axis_options: &ProjectionEscapeAxisOptions,
    saw_unknown: bool,
    search: impl FnMut(&Aabb) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    projection_axis_escape_reference_with_search_and_axis_options_tracking_unknown(
        projected,
        axis_options,
        saw_unknown,
        search,
    )
}

#[cfg(test)]
fn projection_axis_escape_reference_with_search(
    projected: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    mut search: impl FnMut(&Aabb) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let axis_options = projection_escape_axis_options_family(projected, bounds, polygons)?;
    projection_axis_escape_reference_with_search_and_axis_options(
        projected,
        &axis_options,
        &mut search,
    )
}

#[cfg(test)]
fn projection_axis_escape_reference_with_search_and_axis_options(
    projected: &Point3,
    axis_options: &ProjectionEscapeAxisOptions,
    mut search: impl FnMut(&Aabb) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    projection_axis_escape_reference_with_search_and_axis_options_tracking_unknown(
        projected,
        axis_options,
        false,
        &mut search,
    )
}

fn projection_axis_escape_reference_with_search_and_axis_options_tracking_unknown(
    projected: &Point3,
    axis_options: &ProjectionEscapeAxisOptions,
    initial_saw_unknown: bool,
    mut search: impl FnMut(&Aabb) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut saw_unknown = initial_saw_unknown;

    for (axis, (lower, upper)) in axis_options.iter().enumerate() {
        for stop_values in [upper, lower] {
            for stop_value in stop_values {
                let corridor = axis_escape_bounds(projected, axis, stop_value.clone())?;
                match search(&corridor) {
                    Ok(Some((target, winding))) => {
                        return Ok(Some((
                            certify_reference_target_after_trace(target),
                            winding,
                        )));
                    }
                    Ok(None) => {}
                    Err(crate::error::HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                    }
                    Err(err) => return Err(err),
                }
            }
        }
    }

    if saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

fn axis_escape_bounds(projected: &Point3, axis: usize, stop_value: Real) -> HypermeshResult<Aabb> {
    let mut min = projected.clone();
    let mut max = projected.clone();
    let start_value = axis_ref(projected, axis);
    if compare_real(start_value, &stop_value)?.is_lt() {
        *axis_mut(&mut max, axis) = stop_value;
    } else {
        *axis_mut(&mut min, axis) = stop_value;
    }
    Ok(Aabb::new(min, max))
}
#[derive(Clone, Debug, PartialEq)]
struct ReferenceTarget {
    point: Point3,
    definitions: Vec<[Plane; 3]>,
    uncertified_definition_fallback: bool,
}

impl ReferenceTarget {
    #[cfg(test)]
    fn axis_defined(point: Point3) -> Self {
        Self {
            definitions: vec![axis_plane_definition(&point)],
            point,
            uncertified_definition_fallback: false,
        }
    }

    fn axis_defined_fallback(point: Point3) -> Self {
        Self {
            definitions: vec![axis_plane_definition(&point)],
            point,
            uncertified_definition_fallback: true,
        }
    }

    fn with_definitions(point: Point3, definitions: Vec<[Plane; 3]>) -> Self {
        Self {
            point,
            definitions,
            uncertified_definition_fallback: false,
        }
    }
}

fn mark_all_reference_targets_uncertified(targets: &mut Vec<ReferenceTarget>) {
    for target in targets {
        target.uncertified_definition_fallback = true;
    }
}

#[cfg(test)]
fn trace_reference_target(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    target: &ReferenceTarget,
) -> HypermeshResult<Option<Vec<i32>>> {
    if !is_certified_valid_reference_for_bounds(&target.point, bounds, polygons)? {
        return Ok(None);
    }

    trace_reference_target_from_validated_bounds(
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        target,
    )
}

fn trace_projected_reference_target_with_queries(
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
    trace_cache: &mut Vec<ReferenceTargetTraceCacheEntry>,
    context: Option<&SupportReferenceCacheContextKey>,
    bounds: &Aabb,
    target: &ReferenceTarget,
    valid_for: impl FnOnce(&Point3) -> HypermeshResult<bool>,
    trace: impl FnOnce(&ReferenceTarget) -> HypermeshResult<Option<Vec<i32>>>,
) -> HypermeshResult<Option<Vec<i32>>> {
    if !cached_reference_bounds_validity_with_context(
        validity_cache,
        context,
        bounds,
        &target.point,
        valid_for,
    )? {
        return Ok(None);
    }

    cached_reference_target_trace_with_context(trace_cache, context, target, trace)
}

fn trace_reference_target_from_validated_bounds(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    target: &ReferenceTarget,
) -> HypermeshResult<Option<Vec<i32>>> {
    let trace_bounds = reference_trace_bounds(old_ref, bounds)?;
    match trace_segment_from_definitions_with_step_detoured_plane_replacement_in_bounds(
        old_ref,
        &target.point,
        old_wnv,
        polygons,
        old_ref_definitions,
        &target.definitions,
        &trace_bounds,
    ) {
        Ok(winding) => Ok(Some(winding)),
        Err(crate::error::HypermeshError::UnknownClassification) => {
            Err(crate::error::HypermeshError::UnknownClassification)
        }
        Err(err) => Err(err),
    }
}

fn reference_trace_bounds(start: &Point3, bounds: &Aabb) -> HypermeshResult<Aabb> {
    let mut min = bounds.min.clone();
    let mut max = bounds.max.clone();
    for axis in 0..3 {
        let start_value = axis_ref(start, axis);
        if compare_real(start_value, axis_ref(&min, axis))?.is_lt() {
            *axis_mut(&mut min, axis) = start_value.clone();
        }
        if compare_real(start_value, axis_ref(&max, axis))?.is_gt() {
            *axis_mut(&mut max, axis) = start_value.clone();
        }
    }
    Ok(Aabb::new(min, max))
}

#[cfg(test)]
fn is_valid_reference_for_bounds(
    point: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    Ok(point_strictly_inside_bounds(point, bounds)?
        && !point_lies_on_local_surface(point, polygons)?)
}

fn is_certified_valid_reference_for_bounds(
    point: &Point3,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    if !point_strictly_inside_bounds(point, bounds)? {
        return Ok(false);
    }
    for polygon in polygons {
        match classify_point_in_local_polygon(point, polygon)? {
            LocalPolygonPointLocation::Outside => {}
            LocalPolygonPointLocation::Interior => return Ok(false),
            LocalPolygonPointLocation::Boundary => {
                return Err(crate::error::HypermeshError::UnknownClassification);
            }
        }
    }
    Ok(true)
}

#[cfg(test)]
fn support_plane_cell_reference(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut query_caches = SupportReferenceQueryCaches::default();
    support_plane_cell_reference_with_query_caches(
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        &mut query_caches,
    )
}

fn support_plane_cell_reference_with_query_caches(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    support_plane_cell_reference_with_halfspaces_and_query_caches(
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        aabb_core_halfspaces(bounds)?,
        query_caches,
    )
}

#[cfg(test)]
fn support_plane_cell_reference_with_halfspaces(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    halfspaces: Vec<LimitPlane3>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut query_caches = SupportReferenceQueryCaches::default();
    support_plane_cell_reference_with_halfspaces_and_query_caches(
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        halfspaces,
        &mut query_caches,
    )
}

fn support_plane_cell_reference_with_halfspaces_and_query_caches(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    mut halfspaces: Vec<LimitPlane3>,
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    if let Some(reused) = reusable_support_reference_result_if_certified(
        &mut query_caches.support_reference_result_cache,
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        &halfspaces,
        &mut query_caches.validity_cache,
    )? {
        return Ok(Some(reused));
    }
    if let Some(reused) = reusable_support_reference_result_from_cached_trace_if_certified(
        &mut query_caches.support_reference_result_cache,
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        &halfspaces,
        &mut query_caches.validity_cache,
        &mut query_caches.trace_cache,
    )? {
        return Ok(Some(reused));
    }

    let cache_context =
        support_reference_cache_context_key(old_ref, old_ref_definitions, old_wnv, polygons);
    let cache_halfspaces = halfspaces.clone();
    cached_support_reference_result_with(
        &mut query_caches.support_reference_result_cache,
        &cache_context,
        bounds,
        &cache_halfspaces,
        || {
            let report_cache = &mut query_caches.report_cache;
            let feasible_cache = &mut query_caches.feasible_cache;
            let seed_geometry_cache = &mut query_caches.seed_geometry_cache;
            let centroid_subset_seed_cache = &mut query_caches.centroid_subset_seed_cache;
            let support_seed_family_cache = &mut query_caches.support_seed_family_cache;
            let support_direct_target_cache = &mut query_caches.support_direct_target_cache;
            let shifted_support_family_cache = &mut query_caches.shifted_support_family_cache;
            let reference_witness_cache = &mut query_caches.reference_witness_cache;
            let strict_contains_cache = &query_caches.strict_contains_cache;
            let trace_cache = &mut query_caches.trace_cache;
            let validity_cache = &mut query_caches.validity_cache;
            let support_surface_cache = &mut query_caches.support_surface_cache;
            let target_cache = &query_caches.target_cache;
            let accept_cache = &query_caches.accept_cache;
            let search_cache = &query_caches.search_cache;
            let shared_halfspace_caches = std::cell::RefCell::new((report_cache, feasible_cache));
            support_plane_cell_reference_with_queries_and_trace_surface_caches(
                old_ref,
                old_ref_definitions,
                old_wnv,
                bounds,
                polygons,
                &mut halfspaces,
                &mut |halfspaces| {
                    let mut caches = shared_halfspace_caches.borrow_mut();
                    cached_halfspace_report_with(caches.0, halfspaces, |halfspaces| {
                        halfspace_system_report(halfspaces)
                    })
                },
                &mut |halfspaces| {
                    let mut caches = shared_halfspace_caches.borrow_mut();
                    let (report_cache, feasible_cache) = &mut *caches;
                    cached_halfspace_feasibility_with_report_cache(
                        report_cache,
                        feasible_cache,
                        halfspaces,
                        halfspace_system_report,
                        halfspace_system_is_feasible,
                    )
                },
                trace_cache,
                validity_cache,
                support_surface_cache,
                seed_geometry_cache,
                centroid_subset_seed_cache,
                support_seed_family_cache,
                support_direct_target_cache,
                shifted_support_family_cache,
                reference_witness_cache,
                strict_contains_cache,
                target_cache,
                accept_cache,
                search_cache,
            )
        },
    )
}

#[cfg(test)]
fn support_plane_cell_reference_with_queries(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    halfspaces: &mut Vec<LimitPlane3>,
    report_for: &mut impl FnMut(
        &[LimitPlane3],
    ) -> HypermeshResult<Option<hyperlimit::HalfspaceFeasibilityReport>>,
    feasible_for: &mut impl FnMut(&[LimitPlane3]) -> HypermeshResult<bool>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut trace_cache = Vec::new();
    let mut validity_cache = Vec::new();
    let mut support_surface_cache = Vec::new();
    let mut query_caches = SupportReferenceQueryCaches::default();
    support_plane_cell_reference_with_queries_and_trace_surface_caches(
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        halfspaces,
        report_for,
        feasible_for,
        &mut trace_cache,
        &mut validity_cache,
        &mut support_surface_cache,
        &mut query_caches.seed_geometry_cache,
        &mut query_caches.centroid_subset_seed_cache,
        &mut query_caches.support_seed_family_cache,
        &mut query_caches.support_direct_target_cache,
        &mut query_caches.shifted_support_family_cache,
        &query_caches.reference_witness_cache,
        &query_caches.strict_contains_cache,
        &query_caches.target_cache,
        &query_caches.accept_cache,
        &query_caches.search_cache,
    )
}

fn support_plane_cell_reference_with_queries_and_trace_surface_caches(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    halfspaces: &mut Vec<LimitPlane3>,
    report_for: &mut impl FnMut(
        &[LimitPlane3],
    ) -> HypermeshResult<Option<hyperlimit::HalfspaceFeasibilityReport>>,
    feasible_for: &mut impl FnMut(&[LimitPlane3]) -> HypermeshResult<bool>,
    trace_cache: &mut Vec<ReferenceTargetTraceCacheEntry>,
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
    support_surface_cache: &mut Vec<SupportSurfaceCacheEntry>,
    seed_geometry_cache: &mut Vec<SupportCellSeedGeometryCacheEntry>,
    centroid_subset_seed_cache: &mut Vec<Point3CentroidSubsetFamilyCacheEntry>,
    support_seed_family_cache: &mut Vec<SupportCellSeedFamiliesCacheEntry>,
    support_direct_target_cache: &mut Vec<SupportDirectReferenceTargetsCacheEntry>,
    shifted_support_family_cache: &mut Vec<ShiftedSupportCellFamilyCacheEntry>,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    strict_contains_cache: &std::cell::RefCell<Vec<ReferenceHalfspaceContainmentCacheEntry>>,
    target_cache: &std::cell::RefCell<Vec<SupportTargetFamilyCacheEntry>>,
    accept_cache: &std::cell::RefCell<Vec<SupportReferenceAcceptCacheEntry>>,
    search_cache: &std::cell::RefCell<
        Vec<SupportPlaneCellSearchCacheEntry<(ReferenceTarget, Vec<i32>)>>,
    >,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    if halfspaces.is_empty() {
        return Ok(None);
    }

    let initial_feasible_unknown = match feasible_for(halfspaces) {
        Ok(true) => false,
        Ok(false) => return Ok(None),
        Err(crate::error::HypermeshError::UnknownClassification) => true,
        Err(err) => return Err(err),
    };
    let cache_context =
        support_reference_cache_context_key(old_ref, old_ref_definitions, old_wnv, polygons);
    let normalized_polygon_index =
        advance_fixed_support_search_index(polygons, 0, halfspaces.as_slice());
    if let Some(reused) = reusable_support_plane_cell_search_result_if_certified(
        search_cache,
        &cache_context,
        bounds,
        normalized_polygon_index,
        halfspaces,
        validity_cache,
    )? {
        return Ok(Some(reused));
    }
    if let Some(reused) = reusable_support_plane_cell_search_result_from_cached_trace_if_certified(
        search_cache,
        &cache_context,
        bounds,
        normalized_polygon_index,
        halfspaces,
        validity_cache,
        trace_cache,
        old_ref,
        old_ref_definitions,
        old_wnv,
    )? {
        return Ok(Some(reused));
    }

    let mut accept = |halfspaces: &[LimitPlane3],
                      report: Option<hyperlimit::HalfspaceFeasibilityReport>|
     -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
        if let Some(reused) = reusable_support_reference_accept_if_certified(
            &mut accept_cache.borrow_mut(),
            &cache_context,
            bounds,
            halfspaces,
            report.as_ref(),
            validity_cache,
        )? {
            return Ok(Some(reused));
        }
        if let Some(reused) = reusable_support_reference_accept_from_cached_trace_if_certified(
            &mut accept_cache.borrow_mut(),
            &cache_context,
            bounds,
            halfspaces,
            report.as_ref(),
            validity_cache,
            trace_cache,
            old_ref,
            old_ref_definitions,
            old_wnv,
        )? {
            return Ok(Some(reused));
        }
        cached_support_reference_accept_with(
            &mut accept_cache.borrow_mut(),
            Some(&cache_context),
            bounds,
            halfspaces,
            report.as_ref(),
            |halfspaces, report| {
                let direct_targets = cached_support_direct_reference_targets_with(
                    support_direct_target_cache,
                    bounds,
                    halfspaces,
                    report,
                    || {
                        let families = cached_support_cell_seed_families_with(
                            support_seed_family_cache,
                            bounds,
                            halfspaces,
                            report,
                            || {
                                support_cell_seed_family_state_from_optional_report_with_seed_geometry_cache(
                                    bounds,
                                    halfspaces,
                                    report,
                                    seed_geometry_cache,
                                    centroid_subset_seed_cache,
                                )
                            },
                        )?;
                        let report_witness = report.and_then(|report| report.witness.clone());
                        let mut strict_direct_seed_search_order = Vec::new();
                        let strict_direct_seeds = take_new_point_family(
                            families.strict_seeds,
                            &mut strict_direct_seed_search_order,
                        );
                        let mut direct_unknown = families.saw_unknown;
                        let direct_targets =
                            deferred_direct_reference_targets_from_strict_seeds_with(
                                &strict_direct_seeds,
                                report_witness.as_ref(),
                                &mut direct_unknown,
                                |seed| {
                                    cached_reference_target_from_halfspace_witness_with(
                                        &mut reference_witness_cache.borrow_mut(),
                                        seed,
                                        halfspaces,
                                        [None, None, None],
                                        || {
                                            reference_target_from_halfspace_witness(
                                                seed,
                                                halfspaces,
                                                [None, None, None],
                                            )
                                        },
                                    )
                                },
                            )?;
                        Ok((direct_targets, direct_unknown))
                    },
                );
                trace_support_reference_targets_with_report_shortcut(
                    bounds,
                    halfspaces,
                    report,
                    reference_witness_cache,
                    strict_contains_cache,
                    support_surface_cache,
                    validity_cache,
                    Some(&cache_context),
                    &mut |point| point_lies_on_any_support_plane(point, polygons),
                    &mut |point| is_certified_valid_reference_for_bounds(point, bounds, polygons),
                    || direct_targets.clone(),
                    || {
                        cached_support_target_family_with(
                            &mut target_cache.borrow_mut(),
                            bounds,
                            halfspaces,
                            report,
                            |halfspaces, report| {
                                strict_support_cell_targets_from_optional_report_with_seed_geometry_cache(
                                    bounds,
                                    halfspaces,
                                    report,
                                    seed_geometry_cache,
                                    centroid_subset_seed_cache,
                                    support_seed_family_cache,
                                    support_direct_target_cache,
                                    shifted_support_family_cache,
                                    reference_witness_cache,
                                    strict_contains_cache,
                                )
                            },
                        )
                    },
                    |target| {
                        cached_reference_target_trace_with_context(
                            trace_cache,
                            Some(&cache_context),
                            target,
                            |target| {
                                trace_reference_target_from_validated_bounds(
                                    old_ref,
                                    old_ref_definitions,
                                    old_wnv,
                                    bounds,
                                    polygons,
                                    target,
                                )
                            },
                        )
                    },
                )
            },
        )
    };

    match support_plane_cell_search_with_queries_cached(
        Some(&cache_context),
        Some(old_ref),
        bounds,
        polygons,
        0,
        halfspaces,
        report_for,
        feasible_for,
        &mut accept,
        search_cache,
    ) {
        Ok(Some(found)) => Ok(Some(found)),
        Ok(None) if initial_feasible_unknown => {
            Err(crate::error::HypermeshError::UnknownClassification)
        }
        Ok(None) => Ok(None),
        Err(err) => Err(err),
    }
}

#[cfg(test)]
fn trace_reference_targets_backtracking_unknown(
    targets: Vec<ReferenceTarget>,
    polygons: &[ConvexPolygon],
    trace: impl FnMut(&ReferenceTarget) -> HypermeshResult<Option<Vec<i32>>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut surface_cache = Vec::new();
    trace_reference_targets_backtracking_unknown_with_surface_cache(
        targets,
        &mut surface_cache,
        &mut |point| point_lies_on_any_support_plane(point, polygons),
        trace,
    )
}

#[cfg(test)]
fn trace_reference_targets_backtracking_unknown_with_surface_cache(
    targets: Vec<ReferenceTarget>,
    surface_cache: &mut Vec<SupportSurfaceCacheEntry>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace: impl FnMut(&ReferenceTarget) -> HypermeshResult<Option<Vec<i32>>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut validity_cache = Vec::new();
    let zero = Real::zero();
    let dummy_bounds = Aabb::new(
        Point3::new(zero.clone(), zero.clone(), zero.clone()),
        Point3::new(zero.clone(), zero.clone(), zero),
    );
    trace_reference_targets_backtracking_unknown_with_query_caches(
        targets,
        surface_cache,
        &mut validity_cache,
        None,
        &dummy_bounds,
        surface_query,
        &mut |_point| Ok(true),
        trace,
    )
}

fn trace_reference_targets_backtracking_unknown_with_query_caches(
    targets: Vec<ReferenceTarget>,
    surface_cache: &mut Vec<SupportSurfaceCacheEntry>,
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
    context: Option<&SupportReferenceCacheContextKey>,
    bounds: &Aabb,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    validity_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    mut trace: impl FnMut(&ReferenceTarget) -> HypermeshResult<Option<Vec<i32>>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut saw_unknown = false;

    for target in targets {
        let on_support_surface = match cached_support_surface_query_with_context(
            surface_cache,
            context,
            &target.point,
            |point| surface_query(point),
        ) {
            Ok(on_support_surface) => on_support_surface,
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if on_support_surface {
            if target.uncertified_definition_fallback {
                saw_unknown = true;
            }
            continue;
        }
        let valid_for_bounds = match cached_reference_bounds_validity_with_context(
            validity_cache,
            context,
            bounds,
            &target.point,
            |point| validity_query(point),
        ) {
            Ok(valid_for_bounds) => valid_for_bounds,
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
                continue;
            }
            Err(err) => return Err(err),
        };
        if !valid_for_bounds {
            if target.uncertified_definition_fallback {
                saw_unknown = true;
            }
            continue;
        }
        match trace(&target) {
            Ok(Some(winding)) => {
                return Ok(Some((
                    certify_reference_target_after_trace(target),
                    winding,
                )));
            }
            Ok(None) => {
                if target.uncertified_definition_fallback {
                    saw_unknown = true;
                }
            }
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }

    if saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(None)
    }
}

fn trace_support_report_witness_target_with_query_caches(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    strict_contains_cache: &std::cell::RefCell<Vec<ReferenceHalfspaceContainmentCacheEntry>>,
    surface_cache: &mut Vec<SupportSurfaceCacheEntry>,
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
    context: Option<&SupportReferenceCacheContextKey>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    validity_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    trace: impl FnMut(&ReferenceTarget) -> HypermeshResult<Option<Vec<i32>>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let Some(witness) = report.and_then(|report| report.witness.as_ref()) else {
        return Ok(None);
    };
    if !cached_point_strictly_inside_support_cell_or_unknown_with(
        &mut strict_contains_cache.borrow_mut(),
        witness,
        bounds,
        halfspaces,
    )? {
        return Ok(None);
    }
    let Some(target) = cached_reference_target_from_halfspace_witness_with(
        &mut reference_witness_cache.borrow_mut(),
        witness,
        halfspaces,
        active_planes_from_optional_halfspace_report(report, witness),
        || {
            reference_target_from_halfspace_witness(
                witness,
                halfspaces,
                active_planes_from_optional_halfspace_report(report, witness),
            )
        },
    )?
    else {
        return Ok(None);
    };

    trace_reference_targets_backtracking_unknown_with_query_caches(
        vec![target],
        surface_cache,
        validity_cache,
        context,
        bounds,
        surface_query,
        validity_query,
        trace,
    )
}

fn trace_support_reference_targets_with_report_shortcut(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    strict_contains_cache: &std::cell::RefCell<Vec<ReferenceHalfspaceContainmentCacheEntry>>,
    surface_cache: &mut Vec<SupportSurfaceCacheEntry>,
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
    context: Option<&SupportReferenceCacheContextKey>,
    surface_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    validity_query: &mut impl FnMut(&Point3) -> HypermeshResult<bool>,
    direct_targets: impl FnOnce() -> HypermeshResult<(Vec<ReferenceTarget>, bool)>,
    build_targets: impl FnOnce() -> HypermeshResult<Vec<ReferenceTarget>>,
    mut trace: impl FnMut(&ReferenceTarget) -> HypermeshResult<Option<Vec<i32>>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    let mut saw_unknown = false;
    match trace_support_report_witness_target_with_query_caches(
        bounds,
        halfspaces,
        report,
        reference_witness_cache,
        strict_contains_cache,
        surface_cache,
        validity_cache,
        context,
        surface_query,
        validity_query,
        |target| trace(target),
    ) {
        Ok(Some(found)) => return Ok(Some(found)),
        Ok(None) => {}
        Err(crate::error::HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }

    match direct_targets() {
        Ok((targets, direct_unknown)) => {
            if !targets.is_empty() {
                match trace_reference_targets_backtracking_unknown_with_query_caches(
                    targets,
                    surface_cache,
                    validity_cache,
                    context,
                    bounds,
                    surface_query,
                    validity_query,
                    |target| trace(target),
                ) {
                    Ok(Some(found)) => return Ok(Some(found)),
                    Ok(None) => {}
                    Err(crate::error::HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                    }
                    Err(err) => return Err(err),
                }
            }
            if direct_unknown {
                saw_unknown = true;
            }
        }
        Err(crate::error::HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }

    match trace_reference_targets_backtracking_unknown_with_query_caches(
        build_targets()?,
        surface_cache,
        validity_cache,
        context,
        bounds,
        surface_query,
        validity_query,
        trace,
    ) {
        Ok(Some(found)) => Ok(Some(found)),
        Ok(None) if saw_unknown => Err(crate::error::HypermeshError::UnknownClassification),
        other => other,
    }
}

#[cfg(test)]
fn support_plane_cell_target(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Option<ReferenceTarget>> {
    let mut halfspaces = aabb_core_halfspaces(bounds)?;
    if halfspaces.is_empty() {
        return Ok(None);
    }
    if !halfspace_system_is_feasible(&halfspaces)? {
        return Ok(None);
    }

    support_plane_cell_target_from(bounds, polygons, 0, &mut halfspaces)
}

#[cfg(test)]
fn support_plane_cell_target_from(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    polygon_index: usize,
    halfspaces: &mut Vec<LimitPlane3>,
) -> HypermeshResult<Option<ReferenceTarget>> {
    let required_halfspace_count = halfspaces.len() + polygons.len().saturating_sub(polygon_index);
    let mut accept = |halfspaces: &[LimitPlane3],
                      report: Option<hyperlimit::HalfspaceFeasibilityReport>|
     -> HypermeshResult<Option<ReferenceTarget>> {
        if halfspaces.len() < required_halfspace_count {
            return Ok(None);
        }
        let strict_seeds =
            strict_support_cell_seeds_from_optional_report(bounds, halfspaces, report.as_ref())?;
        let mut saw_unknown = false;
        let targets = deferred_direct_reference_targets_from_strict_seeds(
            &strict_seeds,
            report.as_ref().and_then(|report| report.witness.as_ref()),
            halfspaces,
            &mut saw_unknown,
        )?;
        for target in targets {
            if !point_lies_on_any_support_plane(&target.point, polygons)? {
                return Ok(Some(target));
            }
        }
        if saw_unknown {
            Err(crate::error::HypermeshError::UnknownClassification)
        } else {
            Ok(None)
        }
    };
    support_plane_cell_search_from(bounds, polygons, polygon_index, halfspaces, &mut accept)
}

#[cfg(test)]
fn support_plane_cell_search_from<T>(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    polygon_index: usize,
    halfspaces: &mut Vec<LimitPlane3>,
    accept: &mut impl FnMut(
        &[LimitPlane3],
        Option<hyperlimit::HalfspaceFeasibilityReport>,
    ) -> HypermeshResult<Option<T>>,
) -> HypermeshResult<Option<T>>
where
    T: Clone,
{
    support_plane_cell_search_with_queries(
        None,
        bounds,
        polygons,
        polygon_index,
        halfspaces,
        &mut |halfspaces| halfspace_system_report(halfspaces),
        &mut |halfspaces| halfspace_system_is_feasible(halfspaces),
        accept,
    )
}

#[cfg(test)]
fn support_plane_cell_search_with_queries<T>(
    preferred_point: Option<&Point3>,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    polygon_index: usize,
    halfspaces: &mut Vec<LimitPlane3>,
    report_for: &mut impl FnMut(
        &[LimitPlane3],
    ) -> HypermeshResult<Option<hyperlimit::HalfspaceFeasibilityReport>>,
    feasible_for: &mut impl FnMut(&[LimitPlane3]) -> HypermeshResult<bool>,
    accept: &mut impl FnMut(
        &[LimitPlane3],
        Option<hyperlimit::HalfspaceFeasibilityReport>,
    ) -> HypermeshResult<Option<T>>,
) -> HypermeshResult<Option<T>>
where
    T: Clone,
{
    let cache = std::cell::RefCell::new(Vec::new());
    support_plane_cell_search_with_queries_cached(
        None,
        preferred_point,
        bounds,
        polygons,
        polygon_index,
        halfspaces,
        report_for,
        feasible_for,
        accept,
        &cache,
    )
}

fn support_plane_cell_search_with_queries_cached<T>(
    context: Option<&SupportReferenceCacheContextKey>,
    preferred_point: Option<&Point3>,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    polygon_index: usize,
    halfspaces: &mut Vec<LimitPlane3>,
    report_for: &mut impl FnMut(
        &[LimitPlane3],
    ) -> HypermeshResult<Option<hyperlimit::HalfspaceFeasibilityReport>>,
    feasible_for: &mut impl FnMut(&[LimitPlane3]) -> HypermeshResult<bool>,
    accept: &mut impl FnMut(
        &[LimitPlane3],
        Option<hyperlimit::HalfspaceFeasibilityReport>,
    ) -> HypermeshResult<Option<T>>,
    cache: &std::cell::RefCell<Vec<SupportPlaneCellSearchCacheEntry<T>>>,
) -> HypermeshResult<Option<T>>
where
    T: Clone,
{
    let polygon_index = advance_fixed_support_search_index(polygons, polygon_index, halfspaces);
    let preferred_order = if polygon_index < polygons.len() {
        support_side_search_order(preferred_point, &polygons[polygon_index].support)
    } else {
        [false, true]
    };
    cached_support_plane_cell_search_with(
        cache,
        context,
        preferred_order,
        bounds,
        polygon_index,
        halfspaces.to_vec(),
        || {
            if halfspaces_force_support_plane_contact(halfspaces, polygons) {
                return Ok(None);
            }

            let mut saw_unknown = false;

            match accept(halfspaces, None) {
                Ok(Some(target)) => return Ok(Some(target)),
                Ok(None) => {}
                Err(crate::error::HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                }
                Err(err) => return Err(err),
            }

            let current_report = match report_for(halfspaces) {
                Ok(report) => report,
                Err(crate::error::HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    None
                }
                Err(err) => return Err(err),
            };
            if current_report.is_some() {
                match accept(halfspaces, current_report) {
                    Ok(Some(target)) => return Ok(Some(target)),
                    Ok(None) => {}
                    Err(crate::error::HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                    }
                    Err(err) => return Err(err),
                }
            }

            if polygon_index < polygons.len() {
                let mut tried_unchanged_branch = false;
                for positive in
                    support_side_search_order(preferred_point, &polygons[polygon_index].support)
                {
                    let branch_halfspace =
                        support_side_halfspace(&polygons[polygon_index].support, positive);
                    if halfspaces
                        .iter()
                        .any(|halfspace| halfspace == &branch_halfspace)
                    {
                        if tried_unchanged_branch {
                            continue;
                        }
                        tried_unchanged_branch = true;
                        match support_plane_cell_search_with_queries_cached(
                            context,
                            preferred_point,
                            bounds,
                            polygons,
                            polygon_index + 1,
                            halfspaces,
                            report_for,
                            feasible_for,
                            accept,
                            cache,
                        ) {
                            Ok(Some(target)) => return Ok(Some(target)),
                            Ok(None) => {}
                            Err(crate::error::HypermeshError::UnknownClassification) => {
                                saw_unknown = true;
                            }
                            Err(err) => return Err(err),
                        }
                        continue;
                    }
                    if halfspace_has_opposite_pair(&branch_halfspace, halfspaces) {
                        continue;
                    }

                    halfspaces.push(branch_halfspace);
                    let mut feasibility_unknown = false;
                    let feasible = match feasible_for(halfspaces) {
                        Ok(feasible) => feasible,
                        Err(crate::error::HypermeshError::UnknownClassification) => {
                            saw_unknown = true;
                            feasibility_unknown = true;
                            true
                        }
                        Err(err) => {
                            halfspaces.pop();
                            return Err(err);
                        }
                    };
                    if feasible || feasibility_unknown {
                        match support_plane_cell_search_with_queries_cached(
                            context,
                            preferred_point,
                            bounds,
                            polygons,
                            polygon_index + 1,
                            halfspaces,
                            report_for,
                            feasible_for,
                            accept,
                            cache,
                        ) {
                            Ok(Some(target)) => {
                                halfspaces.pop();
                                return Ok(Some(target));
                            }
                            Ok(None) => {}
                            Err(crate::error::HypermeshError::UnknownClassification) => {
                                saw_unknown = true;
                            }
                            Err(err) => {
                                halfspaces.pop();
                                return Err(err);
                            }
                        }
                    }
                    halfspaces.pop();
                }
                return if saw_unknown {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(None)
                };
            }

            if saw_unknown {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(None)
            }
        },
    )
}

fn advance_fixed_support_search_index(
    polygons: &[ConvexPolygon],
    mut polygon_index: usize,
    halfspaces: &[LimitPlane3],
) -> usize {
    while polygon_index < polygons.len() {
        let negative = support_side_halfspace(&polygons[polygon_index].support, false);
        let positive = support_side_halfspace(&polygons[polygon_index].support, true);
        let has_negative = halfspaces.iter().any(|halfspace| halfspace == &negative);
        let has_positive = halfspaces.iter().any(|halfspace| halfspace == &positive);
        if has_negative == has_positive {
            break;
        }
        polygon_index += 1;
    }
    polygon_index
}

fn halfspaces_force_support_plane_contact(
    halfspaces: &[LimitPlane3],
    polygons: &[ConvexPolygon],
) -> bool {
    polygons.iter().any(|polygon| {
        let negative = support_side_halfspace(&polygon.support, false);
        let positive = support_side_halfspace(&polygon.support, true);
        halfspaces.iter().any(|halfspace| halfspace == &negative)
            && halfspaces.iter().any(|halfspace| halfspace == &positive)
    })
}

fn support_side_search_order(
    preferred_point: Option<&Point3>,
    plane: &crate::geometry::Plane,
) -> [bool; 2] {
    let Some(point) = preferred_point else {
        return [false, true];
    };
    match classify_real(&plane.expression_at_point(point)) {
        Ok(Classification::Negative) => [false, true],
        Ok(Classification::Positive) => [true, false],
        Ok(Classification::On) | Err(_) => [false, true],
    }
}

fn halfspace_system_is_feasible(halfspaces: &[LimitPlane3]) -> HypermeshResult<bool> {
    Ok(matches!(
        halfspace_system_report(halfspaces)?,
        Some(report) if report.status == HalfspaceFeasibility::Feasible
    ))
}

fn halfspace_system_report(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Option<hyperlimit::HalfspaceFeasibilityReport>> {
    match classify_halfspace_feasibility3(halfspaces) {
        PredicateOutcome::Decided { value, .. } => Ok(Some(value)),
        PredicateOutcome::Unknown { .. } => {
            Err(crate::error::HypermeshError::UnknownClassification)
        }
    }
}

fn optional_halfspace_system_report(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<(Option<hyperlimit::HalfspaceFeasibilityReport>, bool)> {
    match classify_halfspace_feasibility3(halfspaces) {
        PredicateOutcome::Decided { value, .. } => Ok((Some(value), false)),
        PredicateOutcome::Unknown { .. } => Ok((None, true)),
    }
}

fn active_planes_from_optional_halfspace_report(
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    witness: &Point3,
) -> [Option<usize>; 3] {
    report.map_or([None, None, None], |report| {
        if report.witness.as_ref() == Some(witness) {
            report.active_planes
        } else {
            [None, None, None]
        }
    })
}

fn reference_definitions_from_active_halfspaces(
    witness: &Point3,
    halfspaces: &[LimitPlane3],
    active_planes: [Option<usize>; 3],
) -> HypermeshResult<ReferenceDefinitionFamilyState> {
    let axis_definition = axis_plane_definition(witness);
    let mut definitions = Vec::new();
    let mut saw_unknown = false;
    let mut active = Vec::new();
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
                push_verified_definition(
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
            for axis_plane in &axis_definition {
                push_verified_definition(
                    &mut definitions,
                    [
                        active[first].clone(),
                        active[second].clone(),
                        axis_plane.clone(),
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
                push_verified_definition(
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

    push_verified_definition(&mut definitions, axis_definition, witness, &mut saw_unknown)?;
    Ok(ReferenceDefinitionFamilyState {
        definitions,
        saw_unknown,
    })
}

#[cfg(test)]
fn projected_reference_targets(
    old_ref: &Point3,
    bounds: &Aabb,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let halfspaces = projected_reference_halfspaces(old_ref, bounds)?;
    let (report, saw_unknown) = optional_halfspace_system_report(&halfspaces)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(Vec::new());
    }
    let mut seed_unknown = saw_unknown;
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        projected_cell_seed_families_from_optional_report(
            bounds,
            &halfspaces,
            report.as_ref(),
            &mut seed_unknown,
        )?;
    let targets = strict_projected_cell_targets_from_seed_families(
        bounds,
        &halfspaces,
        report.as_ref(),
        strict_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    )?;
    if targets.is_empty() && seed_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(targets)
    }
}

fn projected_reference_halfspaces(
    old_ref: &Point3,
    bounds: &Aabb,
) -> HypermeshResult<Vec<LimitPlane3>> {
    let mut halfspaces = aabb_core_halfspaces(bounds)?;
    for axis in 0..3 {
        let value = axis_ref(old_ref, axis);
        let min = axis_ref(&bounds.min, axis);
        let max = axis_ref(&bounds.max, axis);
        if compare_real(value, min)?.is_gt() && compare_real(value, max)?.is_lt() {
            halfspaces.push(axis_halfspace(axis, true, value.clone()));
            halfspaces.push(axis_halfspace(axis, false, value.clone()));
        }
    }
    Ok(halfspaces)
}

#[cfg(test)]
fn strict_projected_cell_targets(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: &hyperlimit::HalfspaceFeasibilityReport,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    strict_projected_cell_targets_from_optional_report(bounds, halfspaces, Some(report))
}

#[cfg(test)]
fn strict_projected_cell_targets_from_optional_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut saw_unknown = false;
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        projected_cell_seed_families_from_optional_report(
            bounds,
            halfspaces,
            report,
            &mut saw_unknown,
        )?;
    let targets = strict_projected_cell_targets_from_seed_families(
        bounds,
        halfspaces,
        report,
        strict_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    )?;

    if targets.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(targets)
    }
}

#[cfg(test)]
fn strict_projected_cell_targets_from_seed_families(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    strict_projected_cell_targets_from_seed_families_with(
        bounds,
        halfspaces,
        report,
        strict_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
        |seed| shifted_projected_cell_targets_from_seed(bounds, halfspaces, seed),
    )
}

#[cfg(test)]
fn strict_projected_cell_targets_from_seed_families_with(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    build_shifted_targets: impl FnMut(&Point3) -> HypermeshResult<Vec<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut saw_unknown = false;
    let targets = strict_projected_cell_targets_from_seed_families_with_tracking_unknown(
        bounds,
        halfspaces,
        report,
        strict_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
        &mut saw_unknown,
        build_shifted_targets,
    )?;
    if targets.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        let mut targets = targets;
        if saw_unknown {
            mark_all_reference_targets_uncertified(&mut targets);
        }
        Ok(targets)
    }
}

#[cfg(test)]
fn strict_projected_cell_targets_from_seed_families_with_tracking_unknown(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    saw_unknown: &mut bool,
    build_shifted_targets: impl FnMut(&Point3) -> HypermeshResult<Vec<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let reference_witness_cache = std::cell::RefCell::new(Vec::new());
    let strict_contains_cache = std::cell::RefCell::new(Vec::new());
    strict_projected_cell_targets_from_seed_families_with_tracking_unknown_and_witness_cache(
        bounds,
        halfspaces,
        report,
        strict_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
        saw_unknown,
        &reference_witness_cache,
        &strict_contains_cache,
        build_shifted_targets,
    )
}

fn strict_projected_cell_targets_from_seed_families_with_tracking_unknown_and_witness_cache(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    saw_unknown: &mut bool,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    strict_contains_cache: &std::cell::RefCell<Vec<ReferenceHalfspaceContainmentCacheEntry>>,
    mut build_shifted_targets: impl FnMut(&Point3) -> HypermeshResult<Vec<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut targets = Vec::new();
    let report_witness = report.and_then(|report| report.witness.clone());
    let mut strict_direct_seed_search_order = Vec::new();
    let strict_direct_seeds =
        take_new_point_family(strict_seeds.clone(), &mut strict_direct_seed_search_order);
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_target_seed_families_with_report_seed(
            report_witness.as_ref(),
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );
    let deferred_direct_targets = deferred_direct_reference_targets_from_strict_seeds_with(
        &strict_direct_seeds,
        report_witness.as_ref(),
        saw_unknown,
        |seed| {
            cached_reference_target_from_halfspace_witness_with(
                &mut reference_witness_cache.borrow_mut(),
                seed,
                halfspaces,
                [None, None, None],
                || reference_target_from_halfspace_witness(seed, halfspaces, [None, None, None]),
            )
        },
    )?;
    *saw_unknown |= extend_reference_target_families_collect_hard_unknown(
        &mut targets,
        [
            reference_target_family_from_witness(
                report.and_then(|report| report.witness.as_ref()),
                |witness| {
                    cached_point_strictly_inside_projected_cell_or_unknown_with(
                        &mut strict_contains_cache.borrow_mut(),
                        witness,
                        bounds,
                        halfspaces,
                    )
                },
                |witness| {
                    cached_reference_target_from_halfspace_witness_with(
                        &mut reference_witness_cache.borrow_mut(),
                        witness,
                        halfspaces,
                        active_planes_from_optional_halfspace_report(report, witness),
                        || {
                            reference_target_from_halfspace_witness(
                                witness,
                                halfspaces,
                                active_planes_from_optional_halfspace_report(report, witness),
                            )
                        },
                    )
                },
            ),
            collect_reference_target_family(strict_shift_seeds, |seed| {
                build_shifted_targets(&seed)
            }),
            collect_reference_target_family(shifted_vertices, |vertex| {
                build_shifted_targets(&vertex)
            }),
            collect_reference_target_family(shifted_geometry_seeds, |seed| {
                build_shifted_targets(&seed)
            }),
        ],
    )?;
    for target in deferred_direct_targets {
        push_unique_reference_target(&mut targets, target);
    }
    *saw_unknown |= targets
        .iter()
        .any(|target| target.uncertified_definition_fallback);
    if *saw_unknown {
        mark_all_reference_targets_uncertified(&mut targets);
    }

    Ok(targets)
}

#[cfg(test)]
fn strict_projected_cell_seeds_from_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: &hyperlimit::HalfspaceFeasibilityReport,
) -> HypermeshResult<Vec<Point3>> {
    strict_projected_cell_seeds_from_optional_report(bounds, halfspaces, Some(report))
}

#[cfg(test)]
fn strict_projected_cell_seeds_from_optional_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> HypermeshResult<Vec<Point3>> {
    let mut saw_unknown = false;
    projected_cell_seed_families_from_optional_report(bounds, halfspaces, report, &mut saw_unknown)
        .map(|(strict_seeds, _shifted_vertices, _shifted_geometry_seeds)| strict_seeds)
}

#[cfg(test)]
fn shifted_projected_cell_targets_from_seed(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    match shifted_projected_cell_families_from_seed(bounds, halfspaces, seed)? {
        Some(families) => {
            shifted_projected_cell_targets_from_families(bounds, halfspaces, &families)
        }
        None => Ok(Vec::new()),
    }
}

fn shifted_projected_cell_targets_from_seed_with_cache(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
    cache: &mut Vec<ShiftedProjectedCellFamilyCacheEntry>,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    strict_contains_cache: &std::cell::RefCell<Vec<ReferenceHalfspaceContainmentCacheEntry>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    match cached_shifted_projected_cell_families_with(cache, bounds, halfspaces, seed, || {
        shifted_projected_cell_families_from_seed(bounds, halfspaces, seed)
    })? {
        Some(families) => shifted_projected_cell_targets_from_families_with_witness_cache(
            bounds,
            halfspaces,
            &families,
            reference_witness_cache,
            strict_contains_cache,
        ),
        None => Ok(Vec::new()),
    }
}

#[cfg(test)]
fn shifted_projected_cell_targets_from_families(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    families: &ShiftedProjectedCellFamilies,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let reference_witness_cache = std::cell::RefCell::new(Vec::new());
    let strict_contains_cache = std::cell::RefCell::new(Vec::new());
    shifted_projected_cell_targets_from_families_with_witness_cache(
        bounds,
        halfspaces,
        families,
        &reference_witness_cache,
        &strict_contains_cache,
    )
}

fn shifted_projected_cell_targets_from_families_with_witness_cache(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    families: &ShiftedProjectedCellFamilies,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    strict_contains_cache: &std::cell::RefCell<Vec<ReferenceHalfspaceContainmentCacheEntry>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let shifted = &families.shifted;
    let report = families.report.as_ref();
    let mut targets = Vec::new();
    let report_witness = report.and_then(|report| report.witness.clone());
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_target_seed_families_with_report_seed(
            report_witness.as_ref(),
            families.strict_seeds.clone(),
            families.shifted_vertices.clone(),
            families.shifted_geometry_seeds.clone(),
        );
    extend_reference_target_families_backtracking_unknown(
        &mut targets,
        [
            reference_target_family_from_witness(
                report_witness.as_ref(),
                |witness| {
                    cached_point_strictly_inside_projected_cell_or_unknown_with(
                        &mut strict_contains_cache.borrow_mut(),
                        witness,
                        bounds,
                        halfspaces,
                    )
                },
                |witness| {
                    cached_reference_target_from_halfspace_witness_with(
                        &mut reference_witness_cache.borrow_mut(),
                        witness,
                        shifted,
                        active_planes_from_optional_halfspace_report(report, witness),
                        || {
                            reference_target_from_halfspace_witness(
                                witness,
                                shifted,
                                active_planes_from_optional_halfspace_report(report, witness),
                            )
                        },
                    )
                },
            ),
            collect_reference_target_family(strict_shift_seeds, |witness| {
                if !cached_point_strictly_inside_projected_cell_or_unknown_with(
                    &mut strict_contains_cache.borrow_mut(),
                    &witness,
                    bounds,
                    halfspaces,
                )? {
                    return Ok(Vec::new());
                }
                Ok(cached_reference_target_from_halfspace_witness_with(
                    &mut reference_witness_cache.borrow_mut(),
                    &witness,
                    shifted,
                    [None, None, None],
                    || {
                        reference_target_from_halfspace_witness(
                            &witness,
                            shifted,
                            [None, None, None],
                        )
                    },
                )?
                .into_iter()
                .collect())
            }),
            collect_reference_target_family(shifted_vertices, |witness| {
                if !cached_point_strictly_inside_projected_cell_or_unknown_with(
                    &mut strict_contains_cache.borrow_mut(),
                    &witness,
                    bounds,
                    halfspaces,
                )? {
                    return Ok(Vec::new());
                }
                Ok(cached_reference_target_from_halfspace_witness_with(
                    &mut reference_witness_cache.borrow_mut(),
                    &witness,
                    shifted,
                    [None, None, None],
                    || {
                        reference_target_from_halfspace_witness(
                            &witness,
                            shifted,
                            [None, None, None],
                        )
                    },
                )?
                .into_iter()
                .collect())
            }),
            collect_reference_target_family(shifted_geometry_seeds, |witness| {
                if !cached_point_strictly_inside_projected_cell_or_unknown_with(
                    &mut strict_contains_cache.borrow_mut(),
                    &witness,
                    bounds,
                    halfspaces,
                )? {
                    return Ok(Vec::new());
                }
                Ok(cached_reference_target_from_halfspace_witness_with(
                    &mut reference_witness_cache.borrow_mut(),
                    &witness,
                    shifted,
                    [None, None, None],
                    || {
                        reference_target_from_halfspace_witness(
                            &witness,
                            shifted,
                            [None, None, None],
                        )
                    },
                )?
                .into_iter()
                .collect())
            }),
        ],
    )?;

    if targets.is_empty() && families.saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        if families.saw_unknown {
            mark_all_reference_targets_uncertified(&mut targets);
        }
        Ok(targets)
    }
}

fn shifted_projected_cell_families_from_seed(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
) -> HypermeshResult<Option<ShiftedProjectedCellFamilies>> {
    let shifted = shifted_support_cell_halfspaces(bounds, halfspaces, seed)?;
    let (report, saw_report_unknown) = optional_halfspace_system_report(&shifted)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(None);
    }

    let mut saw_unknown = saw_report_unknown;

    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        projected_cell_seed_families_from_optional_report(
            bounds,
            &shifted,
            report.as_ref(),
            &mut saw_unknown,
        )?;
    Ok(Some(ShiftedProjectedCellFamilies {
        shifted,
        report,
        saw_unknown,
        strict_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    }))
}

#[cfg(test)]
fn projected_escape_targets_from_seed(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    match shifted_projected_cell_families_from_seed(bounds, halfspaces, seed)? {
        Some(families) => projected_escape_targets_from_families(halfspaces, &families),
        None => Ok(Vec::new()),
    }
}

fn projected_escape_targets_from_seed_with_cache(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
    cache: &mut Vec<ShiftedProjectedCellFamilyCacheEntry>,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    pure_halfspace_contains_cache: &std::cell::RefCell<
        Vec<ReferencePureHalfspaceContainmentCacheEntry>,
    >,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    match cached_shifted_projected_cell_families_with(cache, bounds, halfspaces, seed, || {
        shifted_projected_cell_families_from_seed(bounds, halfspaces, seed)
    })? {
        Some(families) => projected_escape_targets_from_families_with_witness_cache(
            halfspaces,
            &families,
            reference_witness_cache,
            pure_halfspace_contains_cache,
        ),
        None => Ok(Vec::new()),
    }
}

#[cfg(test)]
fn projected_escape_targets_from_families(
    halfspaces: &[LimitPlane3],
    families: &ShiftedProjectedCellFamilies,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let reference_witness_cache = std::cell::RefCell::new(Vec::new());
    let pure_halfspace_contains_cache = std::cell::RefCell::new(Vec::new());
    projected_escape_targets_from_families_with_witness_cache(
        halfspaces,
        families,
        &reference_witness_cache,
        &pure_halfspace_contains_cache,
    )
}

fn projected_escape_targets_from_families_with_witness_cache(
    halfspaces: &[LimitPlane3],
    families: &ShiftedProjectedCellFamilies,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    pure_halfspace_contains_cache: &std::cell::RefCell<
        Vec<ReferencePureHalfspaceContainmentCacheEntry>,
    >,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let shifted = &families.shifted;
    let report = families.report.as_ref();
    let mut targets = Vec::new();
    let report_witness = report.and_then(|report| report.witness.clone());
    collect_shifted_projected_escape_target_families(
        &mut targets,
        report_witness.as_ref(),
        families.strict_seeds.clone(),
        families.shifted_vertices.clone(),
        families.shifted_geometry_seeds.clone(),
        |witness| {
            cached_point_strictly_inside_halfspaces_or_unknown_with(
                &mut pure_halfspace_contains_cache.borrow_mut(),
                witness,
                halfspaces,
            )
        },
        |witness| {
            cached_reference_target_from_halfspace_witness_with(
                &mut reference_witness_cache.borrow_mut(),
                witness,
                shifted,
                active_planes_from_optional_halfspace_report(report, witness),
                || {
                    reference_target_from_halfspace_witness(
                        witness,
                        shifted,
                        active_planes_from_optional_halfspace_report(report, witness),
                    )
                },
            )
        },
        |witness| {
            cached_reference_target_from_halfspace_witness_with(
                &mut reference_witness_cache.borrow_mut(),
                witness,
                shifted,
                [None, None, None],
                || reference_target_from_halfspace_witness(witness, shifted, [None, None, None]),
            )
        },
    )?;

    if targets.is_empty() && families.saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        if families.saw_unknown {
            mark_all_reference_targets_uncertified(&mut targets);
        }
        Ok(targets)
    }
}

fn projected_cell_seed_families_from_optional_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    saw_unknown: &mut bool,
) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)> {
    let mut seed_geometry_cache = Vec::new();
    let mut centroid_subset_seed_cache = Vec::new();
    projected_cell_seed_families_from_optional_report_with_seed_geometry_cache(
        bounds,
        halfspaces,
        report,
        saw_unknown,
        &mut seed_geometry_cache,
        &mut centroid_subset_seed_cache,
    )
}

fn projected_cell_seed_families_from_optional_report_with_seed_geometry_cache(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    saw_unknown: &mut bool,
    seed_geometry_cache: &mut Vec<SupportCellSeedGeometryCacheEntry>,
    centroid_subset_seed_cache: &mut Vec<Point3CentroidSubsetFamilyCacheEntry>,
) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)> {
    let seed_geometry =
        cached_support_cell_seed_geometry_with(seed_geometry_cache, halfspaces, || {
            support_cell_seed_geometry_state(halfspaces, centroid_subset_seed_cache)
        })?;
    *saw_unknown |= seed_geometry.saw_unknown;
    let shifted_vertices = seed_geometry.shifted_vertices;
    let shifted_geometry_seeds = seed_geometry.shifted_geometry_seeds;
    let mut strict_seeds = Vec::new();

    *saw_unknown |= extend_point3_families_collect_unknown(
        &mut strict_seeds,
        [
            if report.is_some_and(|report| report.status == HalfspaceFeasibility::Feasible)
                && let Some(witness) = report.and_then(|report| report.witness.as_ref())
            {
                collect_point3_family(Ok(vec![witness.clone()]), |candidate| {
                    point_strictly_inside_projected_cell_or_unknown(candidate, bounds, halfspaces)
                })
            } else {
                Ok(Point3FamilyState {
                    points: Vec::new(),
                    saw_unknown: false,
                })
            },
            collect_point3_family(Ok(shifted_vertices.clone()), |candidate| {
                point_strictly_inside_projected_cell_or_unknown(candidate, bounds, halfspaces)
            }),
            collect_point3_family(Ok(shifted_geometry_seeds.clone()), |candidate| {
                point_strictly_inside_projected_cell_or_unknown(candidate, bounds, halfspaces)
            }),
        ],
    )?;

    if point_seed_family_search_failed_without_any_seed(
        &strict_seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        *saw_unknown,
    ) {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok((strict_seeds, shifted_vertices, shifted_geometry_seeds))
    }
}

fn collect_shifted_projected_escape_target_families(
    targets: &mut Vec<ReferenceTarget>,
    report_witness: Option<&Point3>,
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    mut include: impl FnMut(&Point3) -> HypermeshResult<bool>,
    mut build_report_target: impl FnMut(&Point3) -> HypermeshResult<Option<ReferenceTarget>>,
    mut build_shifted_target: impl FnMut(&Point3) -> HypermeshResult<Option<ReferenceTarget>>,
) -> HypermeshResult<()> {
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_target_seed_families_with_report_seed(
            report_witness,
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );
    extend_reference_target_families_backtracking_unknown(
        targets,
        [
            reference_target_family_from_witness(
                report_witness,
                |witness| include(witness),
                |witness| build_report_target(witness),
            ),
            collect_reference_target_family(strict_seeds, |witness| {
                if !include(&witness)? {
                    return Ok(Vec::new());
                }
                Ok(build_shifted_target(&witness)?.into_iter().collect())
            }),
            collect_reference_target_family(shifted_vertices, |witness| {
                if !include(&witness)? {
                    return Ok(Vec::new());
                }
                Ok(build_shifted_target(&witness)?.into_iter().collect())
            }),
            collect_reference_target_family(shifted_geometry_seeds, |witness| {
                if !include(&witness)? {
                    return Ok(Vec::new());
                }
                Ok(build_shifted_target(&witness)?.into_iter().collect())
            }),
        ],
    )
}

fn dedupe_shifted_target_seed_families(
    report_witness: Option<&Point3>,
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
) -> (Vec<Point3>, Vec<Point3>, Vec<Point3>) {
    let mut shifted_seed_search_order = Vec::new();
    let strict_seeds = take_new_point_family(strict_seeds, &mut shifted_seed_search_order);
    if let Some(report_witness) = report_witness
        && !shifted_seed_search_order
            .iter()
            .any(|existing| existing == report_witness)
    {
        shifted_seed_search_order.push(report_witness.clone());
    }
    let shifted_vertices = take_new_point_family(shifted_vertices, &mut shifted_seed_search_order);
    let shifted_geometry_seeds =
        take_new_point_family(shifted_geometry_seeds, &mut shifted_seed_search_order);
    (strict_seeds, shifted_vertices, shifted_geometry_seeds)
}

fn shifted_target_seed_families_with_report_seed(
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
    dedupe_shifted_target_seed_families(
        report_witness,
        strict_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    )
}

fn support_shifted_target_seed_families(
    report_witness: Option<&Point3>,
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    direct_targets: &[ReferenceTarget],
) -> (Vec<Point3>, Vec<Point3>, Vec<Point3>) {
    if direct_targets
        .iter()
        .any(|target| !target.uncertified_definition_fallback)
    {
        let mut strict_shift_seeds = Vec::new();
        if let Some(report_witness) = report_witness {
            strict_shift_seeds.push(report_witness.clone());
        } else if let Some(first_certified_target) = direct_targets
            .iter()
            .find(|target| !target.uncertified_definition_fallback)
        {
            strict_shift_seeds.push(first_certified_target.point.clone());
        }
        return dedupe_shifted_target_seed_families(
            report_witness,
            strict_shift_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );
    }

    shifted_target_seed_families_with_report_seed(
        report_witness,
        strict_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    )
}

fn push_verified_definition(
    definitions: &mut Vec<[Plane; 3]>,
    definition: [Plane; 3],
    witness: &Point3,
    saw_unknown: &mut bool,
) -> HypermeshResult<()> {
    match affine_from_planes(&definition) {
        Ok(point) if point == *witness => {
            if !definitions
                .iter()
                .any(|existing| reference_definition_planes_match_as_sets(existing, &definition))
            {
                definitions.push(definition);
            }
        }
        Ok(_) => {}
        Err(crate::error::HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
        }
        Err(err) => return Err(err),
    }
    Ok(())
}

fn certified_reference_definitions(point: &Point3, definitions: &[[Plane; 3]]) -> Vec<[Plane; 3]> {
    let mut certified = Vec::new();
    for definition in definitions {
        let Ok(defined_point) = affine_from_planes(definition) else {
            continue;
        };
        if defined_point == *point
            && !certified
                .iter()
                .any(|existing| reference_definition_planes_match_as_sets(existing, definition))
        {
            certified.push(definition.clone());
        }
    }
    if certified.is_empty() {
        certified.push(axis_plane_definition(point));
    }
    certified
}

fn certify_reference_target_after_trace(mut target: ReferenceTarget) -> ReferenceTarget {
    // Callers have already certified strict target validity and its winding trace.
    target.definitions = certified_reference_definitions(&target.point, &target.definitions);
    target.uncertified_definition_fallback = false;
    target
}

fn certified_reference_result(
    (point, definitions, winding): (Point3, Vec<[Plane; 3]>, Vec<i32>),
) -> (Point3, Vec<[Plane; 3]>, Vec<i32>) {
    let definitions = certified_reference_definitions(&point, &definitions);
    (point, definitions, winding)
}

fn strict_support_cell_targets_from_optional_report_with_seed_geometry_cache(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    seed_geometry_cache: &mut Vec<SupportCellSeedGeometryCacheEntry>,
    centroid_subset_seed_cache: &mut Vec<Point3CentroidSubsetFamilyCacheEntry>,
    support_seed_family_cache: &mut Vec<SupportCellSeedFamiliesCacheEntry>,
    support_direct_target_cache: &mut Vec<SupportDirectReferenceTargetsCacheEntry>,
    shifted_support_family_cache: &mut Vec<ShiftedSupportCellFamilyCacheEntry>,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    strict_contains_cache: &std::cell::RefCell<Vec<ReferenceHalfspaceContainmentCacheEntry>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut targets = Vec::new();
    let families = cached_support_cell_seed_families_with(
        support_seed_family_cache,
        bounds,
        halfspaces,
        report,
        || {
            support_cell_seed_family_state_from_optional_report_with_seed_geometry_cache(
                bounds,
                halfspaces,
                report,
                seed_geometry_cache,
                centroid_subset_seed_cache,
            )
        },
    )?;
    let family_saw_unknown = families.saw_unknown;
    let mut saw_unknown = family_saw_unknown;
    let strict_seeds = families.strict_seeds;
    let shifted_vertices = families.shifted_vertices;
    let shifted_geometry_seeds = families.shifted_geometry_seeds;
    let report_witness = report.and_then(|report| report.witness.clone());
    let mut strict_direct_seed_search_order = Vec::new();
    let strict_direct_seeds =
        take_new_point_family(strict_seeds.clone(), &mut strict_direct_seed_search_order);
    let (deferred_direct_targets, direct_unknown) = cached_support_direct_reference_targets_with(
        support_direct_target_cache,
        bounds,
        halfspaces,
        report,
        || {
            let mut direct_unknown = family_saw_unknown;
            let direct_targets = deferred_direct_reference_targets_from_strict_seeds_with(
                &strict_direct_seeds,
                report_witness.as_ref(),
                &mut direct_unknown,
                |seed| {
                    cached_reference_target_from_halfspace_witness_with(
                        &mut reference_witness_cache.borrow_mut(),
                        seed,
                        halfspaces,
                        [None, None, None],
                        || {
                            reference_target_from_halfspace_witness(
                                seed,
                                halfspaces,
                                [None, None, None],
                            )
                        },
                    )
                },
            )?;
            Ok((direct_targets, direct_unknown))
        },
    )?;
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        support_shifted_target_seed_families(
            report_witness.as_ref(),
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
            &deferred_direct_targets,
        );
    saw_unknown |= direct_unknown;
    saw_unknown |= extend_reference_target_families_collect_hard_unknown(
        &mut targets,
        [
            reference_target_family_from_witness(
                report.and_then(|report| report.witness.as_ref()),
                |witness| {
                    cached_point_strictly_inside_support_cell_or_unknown_with(
                        &mut strict_contains_cache.borrow_mut(),
                        witness,
                        bounds,
                        halfspaces,
                    )
                },
                |witness| {
                    cached_reference_target_from_halfspace_witness_with(
                        &mut reference_witness_cache.borrow_mut(),
                        witness,
                        halfspaces,
                        active_planes_from_optional_halfspace_report(report, witness),
                        || {
                            reference_target_from_halfspace_witness(
                                witness,
                                halfspaces,
                                active_planes_from_optional_halfspace_report(report, witness),
                            )
                        },
                    )
                },
            ),
            collect_reference_target_family(strict_shift_seeds, |seed| {
                shifted_support_cell_targets_from_seed_with_caches(
                    bounds,
                    halfspaces,
                    &seed,
                    seed_geometry_cache,
                    centroid_subset_seed_cache,
                    shifted_support_family_cache,
                    reference_witness_cache,
                    strict_contains_cache,
                )
            }),
            collect_reference_target_family(shifted_vertices, |vertex| {
                shifted_support_cell_targets_from_seed_with_caches(
                    bounds,
                    halfspaces,
                    &vertex,
                    seed_geometry_cache,
                    centroid_subset_seed_cache,
                    shifted_support_family_cache,
                    reference_witness_cache,
                    strict_contains_cache,
                )
            }),
            collect_reference_target_family(shifted_geometry_seeds, |seed| {
                shifted_support_cell_targets_from_seed_with_caches(
                    bounds,
                    halfspaces,
                    &seed,
                    seed_geometry_cache,
                    centroid_subset_seed_cache,
                    shifted_support_family_cache,
                    reference_witness_cache,
                    strict_contains_cache,
                )
            }),
        ],
    )?;
    for target in deferred_direct_targets {
        push_unique_reference_target(&mut targets, target);
    }
    saw_unknown |= targets
        .iter()
        .any(|target| target.uncertified_definition_fallback);

    if targets.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        if saw_unknown {
            mark_all_reference_targets_uncertified(&mut targets);
        }
        Ok(targets)
    }
}

#[cfg(test)]
fn strict_support_cell_targets_from_seed_families_with_tracking_unknown(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    strict_seeds: Vec<Point3>,
    shifted_vertices: Vec<Point3>,
    shifted_geometry_seeds: Vec<Point3>,
    saw_unknown: &mut bool,
    mut build_shifted_targets: impl FnMut(&Point3) -> HypermeshResult<Vec<ReferenceTarget>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut targets = Vec::new();
    let report_witness = report.and_then(|report| report.witness.clone());
    let mut strict_direct_seed_search_order = Vec::new();
    let strict_direct_seeds =
        take_new_point_family(strict_seeds.clone(), &mut strict_direct_seed_search_order);
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_target_seed_families_with_report_seed(
            report_witness.as_ref(),
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
        );
    let deferred_direct_targets = deferred_direct_reference_targets_from_strict_seeds(
        &strict_direct_seeds,
        report_witness.as_ref(),
        halfspaces,
        saw_unknown,
    )?;
    *saw_unknown |= extend_reference_target_families_collect_hard_unknown(
        &mut targets,
        [
            reference_target_family_from_witness(
                report.and_then(|report| report.witness.as_ref()),
                |witness| {
                    point_strictly_inside_support_cell_or_unknown(witness, bounds, halfspaces)
                },
                |witness| {
                    reference_target_from_halfspace_witness(
                        witness,
                        halfspaces,
                        active_planes_from_optional_halfspace_report(report, witness),
                    )
                },
            ),
            collect_reference_target_family(strict_shift_seeds, |seed| {
                build_shifted_targets(&seed)
            }),
            collect_reference_target_family(shifted_vertices, |vertex| {
                build_shifted_targets(&vertex)
            }),
            collect_reference_target_family(shifted_geometry_seeds, |seed| {
                build_shifted_targets(&seed)
            }),
        ],
    )?;
    for target in deferred_direct_targets {
        push_unique_reference_target(&mut targets, target);
    }
    *saw_unknown |= targets
        .iter()
        .any(|target| target.uncertified_definition_fallback);
    if *saw_unknown {
        mark_all_reference_targets_uncertified(&mut targets);
    }
    Ok(targets)
}

#[cfg(test)]
fn strict_support_cell_seeds_from_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: &hyperlimit::HalfspaceFeasibilityReport,
) -> HypermeshResult<Vec<Point3>> {
    strict_support_cell_seeds_from_optional_report(bounds, halfspaces, Some(report))
}

#[cfg(test)]
fn strict_support_cell_seeds_from_optional_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> HypermeshResult<Vec<Point3>> {
    let mut saw_unknown = false;
    support_cell_seed_families_from_optional_report(bounds, halfspaces, report, &mut saw_unknown)
        .map(|(strict_seeds, _shifted_vertices, _shifted_geometry_seeds)| strict_seeds)
}

#[cfg(test)]
fn support_cell_geometry_seed_candidates(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Vec<Point3>> {
    let vertices = feasible_support_cell_vertices(halfspaces)?;
    support_cell_geometry_seed_candidates_from_vertices(&vertices)
}

#[cfg(test)]
fn support_cell_geometry_seed_candidates_from_vertices(
    vertices: &[Point3],
) -> HypermeshResult<Vec<Point3>> {
    Ok(point3_centroid_subset_family_from_vertices(vertices)?.points)
}

fn point3_centroid_subset_family_from_vertices(
    vertices: &[Point3],
) -> HypermeshResult<Point3FamilyState> {
    point3_centroid_subset_family_from_vertices_with(vertices, point3_centroid)
}

fn point3_centroid_subset_family_from_vertices_with(
    vertices: &[Point3],
    mut center_of: impl FnMut(&[Point3]) -> HypermeshResult<Option<Point3>>,
) -> HypermeshResult<Point3FamilyState> {
    let mut candidates = Vec::new();
    let mut subset = Vec::new();
    let mut saw_unknown = false;
    collect_point3_centroid_subset_candidates(
        &mut candidates,
        vertices,
        0,
        &mut subset,
        &mut saw_unknown,
        &mut center_of,
    )?;
    if candidates.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(Point3FamilyState {
            points: candidates,
            saw_unknown,
        })
    }
}

fn collect_point3_centroid_subset_candidates(
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
                Ok(Some(center)) => push_unique_point3(candidates, center),
                Ok(None) => {}
                Err(crate::error::HypermeshError::UnknownClassification) => {
                    *saw_unknown = true;
                }
                Err(err) => return Err(err),
            }
        }
        collect_point3_centroid_subset_candidates(
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

fn push_unique_point3(points: &mut Vec<Point3>, point: Point3) {
    if !points.iter().any(|existing| existing == &point) {
        points.push(point);
    }
}

#[derive(Clone, Debug, PartialEq)]
struct Point3FamilyState {
    points: Vec<Point3>,
    saw_unknown: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct ReferenceDefinitionFamilyState {
    definitions: Vec<[Plane; 3]>,
    saw_unknown: bool,
}

fn take_new_point_family(points: Vec<Point3>, seen: &mut Vec<Point3>) -> Vec<Point3> {
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

fn point_families_match_as_sets(left: &[Point3], right: &[Point3]) -> bool {
    if left.len() != right.len() {
        return false;
    }

    let mut matched = vec![false; right.len()];
    for left_point in left {
        let Some((index, _)) = right
            .iter()
            .enumerate()
            .find(|(index, right_point)| !matched[*index] && *right_point == left_point)
        else {
            return false;
        };
        matched[index] = true;
    }

    true
}

fn point3_centroid(points: &[Point3]) -> HypermeshResult<Option<Point3>> {
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
        (sum.x / denom.clone()).map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        (sum.y / denom.clone()).map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
        (sum.z / denom).map_err(|_| crate::error::HypermeshError::UnknownClassification)?,
    )))
}

#[cfg(test)]
fn extend_point3_backtracking_unknown(
    points: &mut Vec<Point3>,
    candidates: impl IntoIterator<Item = Point3>,
    mut keep: impl FnMut(&Point3) -> HypermeshResult<bool>,
) -> HypermeshResult<()> {
    let mut saw_unknown = false;
    for candidate in candidates {
        match keep(&candidate) {
            Ok(true) => push_unique_point3(points, candidate),
            Ok(false) => {}
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if points.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(())
    }
}

fn collect_point3_family(
    candidates: HypermeshResult<Vec<Point3>>,
    mut keep: impl FnMut(&Point3) -> HypermeshResult<bool>,
) -> HypermeshResult<Point3FamilyState> {
    let mut points = Vec::new();
    let mut saw_unknown = false;
    for candidate in candidates? {
        match keep(&candidate) {
            Ok(true) => push_unique_point3(&mut points, candidate),
            Ok(false) => {}
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    if points.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(Point3FamilyState {
            points,
            saw_unknown,
        })
    }
}

#[cfg(test)]
fn extend_point3_families_backtracking_unknown(
    points: &mut Vec<Point3>,
    families: impl IntoIterator<Item = HypermeshResult<Point3FamilyState>>,
) -> HypermeshResult<()> {
    let saw_unknown = extend_point3_families_collect_unknown(points, families)?;
    if points.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(())
    }
}

fn extend_point3_families_collect_unknown(
    points: &mut Vec<Point3>,
    families: impl IntoIterator<Item = HypermeshResult<Point3FamilyState>>,
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for family in families {
        match family {
            Ok(found) => {
                saw_unknown |= found.saw_unknown;
                for point in found.points {
                    push_unique_point3(points, point);
                }
            }
            Err(crate::error::HypermeshError::UnknownClassification) => {
                saw_unknown = true;
            }
            Err(err) => return Err(err),
        }
    }
    Ok(saw_unknown)
}

#[cfg(test)]
fn shifted_support_cell_targets_from_seed(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut seed_geometry_cache = Vec::new();
    let mut centroid_subset_seed_cache = Vec::new();
    let mut shifted_support_family_cache = Vec::new();
    let reference_witness_cache = std::cell::RefCell::new(Vec::new());
    let strict_contains_cache = std::cell::RefCell::new(Vec::new());
    shifted_support_cell_targets_from_seed_with_caches(
        bounds,
        halfspaces,
        seed,
        &mut seed_geometry_cache,
        &mut centroid_subset_seed_cache,
        &mut shifted_support_family_cache,
        &reference_witness_cache,
        &strict_contains_cache,
    )
}

fn shifted_support_cell_targets_from_seed_with_caches(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
    seed_geometry_cache: &mut Vec<SupportCellSeedGeometryCacheEntry>,
    centroid_subset_seed_cache: &mut Vec<Point3CentroidSubsetFamilyCacheEntry>,
    shifted_support_family_cache: &mut Vec<ShiftedSupportCellFamilyCacheEntry>,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    strict_contains_cache: &std::cell::RefCell<Vec<ReferenceHalfspaceContainmentCacheEntry>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    match cached_shifted_support_cell_families_with(
        shifted_support_family_cache,
        bounds,
        halfspaces,
        seed,
        || {
            shifted_support_cell_families_from_seed(
                bounds,
                halfspaces,
                seed,
                seed_geometry_cache,
                centroid_subset_seed_cache,
            )
        },
    )? {
        Some(families) => shifted_support_cell_targets_from_families(
            bounds,
            halfspaces,
            &families,
            reference_witness_cache,
            strict_contains_cache,
        ),
        None => Ok(Vec::new()),
    }
}

#[cfg(test)]
fn support_cell_seed_families_from_optional_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    saw_unknown: &mut bool,
) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)> {
    let mut seed_geometry_cache = Vec::new();
    let mut centroid_subset_seed_cache = Vec::new();
    support_cell_seed_families_from_optional_report_with_seed_geometry_cache(
        bounds,
        halfspaces,
        report,
        saw_unknown,
        &mut seed_geometry_cache,
        &mut centroid_subset_seed_cache,
    )
}

fn support_cell_seed_families_from_optional_report_with_seed_geometry_cache(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    saw_unknown: &mut bool,
    seed_geometry_cache: &mut Vec<SupportCellSeedGeometryCacheEntry>,
    centroid_subset_seed_cache: &mut Vec<Point3CentroidSubsetFamilyCacheEntry>,
) -> HypermeshResult<(Vec<Point3>, Vec<Point3>, Vec<Point3>)> {
    let state = support_cell_seed_family_state_from_optional_report_with_seed_geometry_cache(
        bounds,
        halfspaces,
        report,
        seed_geometry_cache,
        centroid_subset_seed_cache,
    )?;
    *saw_unknown |= state.saw_unknown;
    Ok((
        state.strict_seeds,
        state.shifted_vertices,
        state.shifted_geometry_seeds,
    ))
}

fn support_cell_seed_family_state_from_optional_report_with_seed_geometry_cache(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
    seed_geometry_cache: &mut Vec<SupportCellSeedGeometryCacheEntry>,
    centroid_subset_seed_cache: &mut Vec<Point3CentroidSubsetFamilyCacheEntry>,
) -> HypermeshResult<SupportCellSeedFamiliesState> {
    let seed_geometry =
        cached_support_cell_seed_geometry_with(seed_geometry_cache, halfspaces, || {
            support_cell_seed_geometry_state(halfspaces, centroid_subset_seed_cache)
        })?;
    let mut saw_unknown = seed_geometry.saw_unknown;
    let shifted_vertices = seed_geometry.shifted_vertices;
    let shifted_geometry_seeds = seed_geometry.shifted_geometry_seeds;
    let mut strict_seeds = Vec::new();

    saw_unknown |= extend_point3_families_collect_unknown(
        &mut strict_seeds,
        [
            if report.is_some_and(|report| report.status == HalfspaceFeasibility::Feasible)
                && let Some(witness) = report.and_then(|report| report.witness.as_ref())
            {
                collect_point3_family(Ok(vec![witness.clone()]), |candidate| {
                    point_strictly_inside_support_cell_or_unknown(candidate, bounds, halfspaces)
                })
            } else {
                Ok(Point3FamilyState {
                    points: Vec::new(),
                    saw_unknown: false,
                })
            },
            collect_point3_family(Ok(shifted_vertices.clone()), |candidate| {
                point_strictly_inside_support_cell_or_unknown(candidate, bounds, halfspaces)
            }),
            collect_point3_family(Ok(shifted_geometry_seeds.clone()), |candidate| {
                point_strictly_inside_support_cell_or_unknown(candidate, bounds, halfspaces)
            }),
        ],
    )?;

    if point_seed_family_search_failed_without_any_seed(
        &strict_seeds,
        &shifted_vertices,
        &shifted_geometry_seeds,
        saw_unknown,
    ) {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(SupportCellSeedFamiliesState {
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
            saw_unknown,
        })
    }
}

fn point_seed_family_search_failed_without_any_seed(
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

fn shifted_support_cell_families_from_seed(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    seed: &Point3,
    seed_geometry_cache: &mut Vec<SupportCellSeedGeometryCacheEntry>,
    centroid_subset_seed_cache: &mut Vec<Point3CentroidSubsetFamilyCacheEntry>,
) -> HypermeshResult<Option<ShiftedSupportCellFamilies>> {
    let shifted = shifted_support_cell_halfspaces(bounds, halfspaces, seed)?;
    let (report, saw_report_unknown) = optional_halfspace_system_report(&shifted)?;
    if report
        .as_ref()
        .is_some_and(|report| report.status != HalfspaceFeasibility::Feasible)
    {
        return Ok(None);
    }

    let mut saw_unknown = saw_report_unknown;
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        support_cell_seed_families_from_optional_report_with_seed_geometry_cache(
            bounds,
            &shifted,
            report.as_ref(),
            &mut saw_unknown,
            seed_geometry_cache,
            centroid_subset_seed_cache,
        )?;
    Ok(Some(ShiftedSupportCellFamilies {
        shifted,
        report,
        saw_unknown,
        strict_seeds,
        shifted_vertices,
        shifted_geometry_seeds,
    }))
}

fn shifted_support_cell_targets_from_families(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    families: &ShiftedSupportCellFamilies,
    reference_witness_cache: &std::cell::RefCell<Vec<ReferenceWitnessTargetCacheEntry>>,
    strict_contains_cache: &std::cell::RefCell<Vec<ReferenceHalfspaceContainmentCacheEntry>>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let shifted = &families.shifted;
    let report = families.report.as_ref();
    let mut targets = Vec::new();
    let report_witness = report.and_then(|report| report.witness.clone());
    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_target_seed_families_with_report_seed(
            report_witness.as_ref(),
            families.strict_seeds.clone(),
            families.shifted_vertices.clone(),
            families.shifted_geometry_seeds.clone(),
        );
    extend_reference_target_families_backtracking_unknown(
        &mut targets,
        [
            reference_target_family_from_witness(
                report_witness.as_ref(),
                |witness| {
                    cached_point_strictly_inside_support_cell_or_unknown_with(
                        &mut strict_contains_cache.borrow_mut(),
                        witness,
                        bounds,
                        halfspaces,
                    )
                },
                |witness| {
                    cached_reference_target_from_halfspace_witness_with(
                        &mut reference_witness_cache.borrow_mut(),
                        witness,
                        shifted,
                        active_planes_from_optional_halfspace_report(report, witness),
                        || {
                            reference_target_from_halfspace_witness(
                                witness,
                                shifted,
                                active_planes_from_optional_halfspace_report(report, witness),
                            )
                        },
                    )
                },
            ),
            collect_reference_target_family(strict_shift_seeds, |witness| {
                if !cached_point_strictly_inside_support_cell_or_unknown_with(
                    &mut strict_contains_cache.borrow_mut(),
                    &witness,
                    bounds,
                    halfspaces,
                )? {
                    return Ok(Vec::new());
                }
                Ok(cached_reference_target_from_halfspace_witness_with(
                    &mut reference_witness_cache.borrow_mut(),
                    &witness,
                    shifted,
                    [None, None, None],
                    || {
                        reference_target_from_halfspace_witness(
                            &witness,
                            shifted,
                            [None, None, None],
                        )
                    },
                )?
                .into_iter()
                .collect())
            }),
            collect_reference_target_family(shifted_vertices, |witness| {
                if !cached_point_strictly_inside_support_cell_or_unknown_with(
                    &mut strict_contains_cache.borrow_mut(),
                    &witness,
                    bounds,
                    halfspaces,
                )? {
                    return Ok(Vec::new());
                }
                Ok(cached_reference_target_from_halfspace_witness_with(
                    &mut reference_witness_cache.borrow_mut(),
                    &witness,
                    shifted,
                    [None, None, None],
                    || {
                        reference_target_from_halfspace_witness(
                            &witness,
                            shifted,
                            [None, None, None],
                        )
                    },
                )?
                .into_iter()
                .collect())
            }),
            collect_reference_target_family(shifted_geometry_seeds, |witness| {
                if !cached_point_strictly_inside_support_cell_or_unknown_with(
                    &mut strict_contains_cache.borrow_mut(),
                    &witness,
                    bounds,
                    halfspaces,
                )? {
                    return Ok(Vec::new());
                }
                Ok(cached_reference_target_from_halfspace_witness_with(
                    &mut reference_witness_cache.borrow_mut(),
                    &witness,
                    shifted,
                    [None, None, None],
                    || {
                        reference_target_from_halfspace_witness(
                            &witness,
                            shifted,
                            [None, None, None],
                        )
                    },
                )?
                .into_iter()
                .collect())
            }),
        ],
    )?;

    if targets.is_empty() && families.saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        if families.saw_unknown {
            mark_all_reference_targets_uncertified(&mut targets);
        }
        Ok(targets)
    }
}

fn support_cell_seed_geometry_state(
    halfspaces: &[LimitPlane3],
    centroid_subset_seed_cache: &mut Vec<Point3CentroidSubsetFamilyCacheEntry>,
) -> HypermeshResult<SupportCellSeedGeometryState> {
    let shifted_vertex_family = feasible_support_cell_vertex_family(halfspaces)?;
    let mut saw_unknown = shifted_vertex_family.saw_unknown;
    let shifted_vertices = shifted_vertex_family.points;
    let subset_seed_family = cached_point3_centroid_subset_family_from_vertices_with(
        centroid_subset_seed_cache,
        &shifted_vertices,
        || point3_centroid_subset_family_from_vertices(&shifted_vertices),
    )?;
    saw_unknown |= subset_seed_family.saw_unknown;

    // A bounded full-dimensional convex cell contains the centroid of all of
    // its vertices strictly. Keep that canonical witness first; the remaining
    // subset centroids only provide alternate replay definitions and paths.
    let mut shifted_geometry_seeds = Vec::new();
    match point3_centroid(&shifted_vertices) {
        Ok(Some(center)) => push_unique_point3(&mut shifted_geometry_seeds, center),
        Ok(None) => {}
        Err(crate::error::HypermeshError::UnknownClassification) => saw_unknown = true,
        Err(err) => return Err(err),
    }
    for seed in subset_seed_family.points {
        push_unique_point3(&mut shifted_geometry_seeds, seed);
    }
    Ok(SupportCellSeedGeometryState {
        shifted_vertices,
        shifted_geometry_seeds,
        saw_unknown,
    })
}

#[cfg(test)]
fn feasible_support_cell_vertices(halfspaces: &[LimitPlane3]) -> HypermeshResult<Vec<Point3>> {
    Ok(feasible_support_cell_vertex_family(halfspaces)?.points)
}

fn feasible_support_cell_vertex_family(
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<Point3FamilyState> {
    feasible_support_cell_vertex_family_with_contains(halfspaces, |point, halfspaces| {
        point_satisfies_halfspaces(point, halfspaces)
    })
}

fn feasible_support_cell_vertex_family_with_contains(
    halfspaces: &[LimitPlane3],
    mut contains: impl FnMut(&Point3, &[LimitPlane3]) -> HypermeshResult<bool>,
) -> HypermeshResult<Point3FamilyState> {
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
                    Err(crate::error::HypermeshError::UnknownClassification) => {
                        saw_unknown = true;
                    }
                    Err(err) => return Err(err),
                }
            }
        }
    }
    if vertices.is_empty() && saw_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Ok(Point3FamilyState {
            points: vertices,
            saw_unknown,
        })
    }
}

#[cfg(test)]
fn feasible_support_cell_vertices_with_contains(
    halfspaces: &[LimitPlane3],
    contains: impl FnMut(&Point3, &[LimitPlane3]) -> HypermeshResult<bool>,
) -> HypermeshResult<Vec<Point3>> {
    Ok(feasible_support_cell_vertex_family_with_contains(halfspaces, contains)?.points)
}

fn point_strictly_inside_halfspaces_or_unknown(
    point: &Point3,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        match crate::geometry::classify_point(point, &plane)? {
            Classification::Positive => return Ok(false),
            Classification::On => {
                if !halfspace_has_opposite_pair(halfspace, halfspaces) {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
            }
            Classification::Negative => {}
        }
    }
    Ok(true)
}

#[cfg(test)]
fn point_strictly_inside_projected_cell(
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    point_strictly_inside_reference_halfspace_cell(point, bounds, halfspaces)
}

fn point_strictly_inside_projected_cell_or_unknown(
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    point_strictly_inside_reference_halfspace_cell_or_unknown(point, bounds, halfspaces)
}

fn cached_point_strictly_inside_projected_cell_or_unknown_with(
    cache: &mut Vec<ReferenceHalfspaceContainmentCacheEntry>,
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    cached_reference_halfspace_containment_with(
        cache,
        bounds,
        point,
        halfspaces,
        |point, bounds, halfspaces| {
            point_strictly_inside_projected_cell_or_unknown(point, bounds, halfspaces)
        },
    )
}

fn point_strictly_inside_support_cell(
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    point_strictly_inside_reference_halfspace_cell(point, bounds, halfspaces)
}

fn point_strictly_inside_support_cell_or_unknown(
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    point_strictly_inside_reference_halfspace_cell_or_unknown(point, bounds, halfspaces)
}

fn cached_point_strictly_inside_support_cell_or_unknown_with(
    cache: &mut Vec<ReferenceHalfspaceContainmentCacheEntry>,
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    cached_reference_halfspace_containment_with(
        cache,
        bounds,
        point,
        halfspaces,
        |point, bounds, halfspaces| {
            point_strictly_inside_support_cell_or_unknown(point, bounds, halfspaces)
        },
    )
}

fn cached_point_strictly_inside_halfspaces_or_unknown_with(
    cache: &mut Vec<ReferencePureHalfspaceContainmentCacheEntry>,
    point: &Point3,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    cached_pure_halfspace_containment_with(cache, point, halfspaces, |point, halfspaces| {
        point_strictly_inside_halfspaces_or_unknown(point, halfspaces)
    })
}

fn point_strictly_inside_reference_halfspace_cell(
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    if !point_strictly_inside_bounds(point, bounds)? {
        return Ok(false);
    }
    for halfspace in halfspaces {
        if halfspace_is_degenerate_bound(halfspace, bounds)? {
            continue;
        }
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        let value = plane.expression_at_point(point);
        if halfspace_has_opposite_pair(halfspace, halfspaces) {
            if compare_real(&value, &Real::zero())?.is_ne() {
                return Ok(false);
            }
        } else if compare_real(&value, &Real::zero())?.is_eq() {
            return Ok(false);
        }
    }
    Ok(true)
}

fn point_strictly_inside_reference_halfspace_cell_or_unknown(
    point: &Point3,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
) -> HypermeshResult<bool> {
    if !point_strictly_inside_bounds(point, bounds)? {
        for axis in 0..3 {
            let min = axis_ref(&bounds.min, axis);
            let max = axis_ref(&bounds.max, axis);
            if compare_real(min, max)?.is_eq() {
                continue;
            }
            let point_value = axis_ref(point, axis);
            if compare_real(point_value, min)?.is_eq() || compare_real(point_value, max)?.is_eq() {
                return Err(crate::error::HypermeshError::UnknownClassification);
            }
        }
        return Ok(false);
    }
    for halfspace in halfspaces {
        if halfspace_is_degenerate_bound(halfspace, bounds)? {
            continue;
        }
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        let value = plane.expression_at_point(point);
        if halfspace_has_opposite_pair(halfspace, halfspaces) {
            if compare_real(&value, &Real::zero())?.is_ne() {
                return Ok(false);
            }
        } else if compare_real(&value, &Real::zero())?.is_eq() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
    }
    Ok(true)
}

fn shifted_support_cell_halfspaces(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    strict_interior: &Point3,
) -> HypermeshResult<Vec<LimitPlane3>> {
    let half = (Real::one() / Real::from(2))
        .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
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

fn point_lies_on_any_support_plane(
    point: &Point3,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    for polygon in polygons {
        match classify_point_in_local_polygon(point, polygon)? {
            LocalPolygonPointLocation::Outside => {}
            LocalPolygonPointLocation::Interior => return Ok(true),
            LocalPolygonPointLocation::Boundary => {
                return Err(crate::error::HypermeshError::UnknownClassification);
            }
        }
    }
    Ok(false)
}

fn reference_axis_surface_crossing(
    start: &Point3,
    endpoint: &Point3,
    polygon: &ConvexPolygon,
    axis: usize,
) -> HypermeshResult<Option<Point3>> {
    let start_class = crate::geometry::classify_point(start, &polygon.support)?;
    let endpoint_class = crate::geometry::classify_point(endpoint, &polygon.support)?;
    if start_class == crate::geometry::Classification::On {
        return Ok(Some(start.clone()));
    }
    if endpoint_class == crate::geometry::Classification::On {
        return Ok(Some(endpoint.clone()));
    }
    if start_class == endpoint_class {
        return Ok(None);
    }

    let start_value = polygon.support.expression_at_point(start);
    let endpoint_value = polygon.support.expression_at_point(endpoint);
    let denom = &start_value - &endpoint_value;
    let t =
        (start_value / denom).map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
    let axis_value =
        axis_ref(start, axis) + &(t * (axis_ref(endpoint, axis) - axis_ref(start, axis)));
    let mut crossing = start.clone();
    *axis_mut(&mut crossing, axis) = axis_value;
    Ok(Some(crossing))
}

fn point_strictly_inside_bounds(point: &Point3, bounds: &Aabb) -> HypermeshResult<bool> {
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

#[cfg(test)]
fn point_lies_on_local_surface(
    point: &Point3,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<bool> {
    for polygon in polygons {
        if point_lies_on_local_polygon(point, polygon)? {
            return Ok(true);
        }
    }
    Ok(false)
}

#[cfg(test)]
fn point_lies_on_local_polygon(point: &Point3, polygon: &ConvexPolygon) -> HypermeshResult<bool> {
    if crate::geometry::classify_point(point, &polygon.support)?
        != crate::geometry::Classification::On
    {
        return Ok(false);
    }
    for edge in &polygon.edges {
        if crate::geometry::classify_point(point, edge)?.is_positive() {
            return Ok(false);
        }
    }
    Ok(true)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LocalPolygonPointLocation {
    Outside,
    Boundary,
    Interior,
}

fn classify_point_in_local_polygon(
    point: &Point3,
    polygon: &ConvexPolygon,
) -> HypermeshResult<LocalPolygonPointLocation> {
    if crate::geometry::classify_point(point, &polygon.support)?
        != crate::geometry::Classification::On
    {
        return Ok(LocalPolygonPointLocation::Outside);
    }
    let mut on_edge = false;
    for edge in &polygon.edges {
        match crate::geometry::classify_point(point, edge)? {
            crate::geometry::Classification::Positive => {
                return Ok(LocalPolygonPointLocation::Outside);
            }
            crate::geometry::Classification::On => on_edge = true,
            crate::geometry::Classification::Negative => {}
        }
    }
    if on_edge {
        Ok(LocalPolygonPointLocation::Boundary)
    } else {
        Ok(LocalPolygonPointLocation::Interior)
    }
}

#[cfg(test)]
mod tests;
