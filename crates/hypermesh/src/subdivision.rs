//! Leaf processing for the subdivision pipeline.

use crate::bvh::ExactBvh;
use crate::clip::{ClipSide, clip_polygon};
use crate::error::HypermeshResult;
use crate::geometry::{
    Aabb, Classification, Plane, axis_mut, axis_ref, classify_real, compare_real,
};
use crate::intersection::{
    IntersectionSegment, PairwiseIntersection, PairwiseIntersectionType, intersect_polygons,
};
use crate::local_bsp::{BspLeaf, LocalBsp};
use crate::output::{
    ClassifiedPolygon, ClassifiedPolygonBucketState, merge_unique_classified_polygons,
    merge_unique_classified_polygons_with_bucket_state,
    push_unique_classified_polygon_with_bucket_state,
};
use crate::polygon::ConvexPolygon;
use crate::segment_trace::{
    affine_from_planes, axis_plane_definition, certified_leaf_interior_points,
    classify_leaf_polygon, classify_leaf_polygon_from_interior_points,
    trace_segment_from_definitions_with_step_detoured_plane_replacement,
};
use crate::winding::{
    BooleanOp, Indicator, WindingNumberVector, WindingPair,
    can_boolean_op_be_inside_with_component_ranges,
    can_boolean_op_be_inside_with_transition_reachability, classify_polygon_output, propagate_wnv,
};
use hyperlattice::{HomogeneousPoint3, Point3, Real, intersect_three_planes};
use std::cell::RefCell;

use hyperlimit::{
    HalfspaceFeasibility, Plane3 as LimitPlane3, PredicateOutcome, classify_halfspace_feasibility3,
};

/// Default maximum subdivision depth.
pub const DEFAULT_MAX_DEPTH: usize = 40;

/// Configuration for recursive subdivision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SubdivisionConfig {
    /// Maximum recursive depth.
    ///
    /// Reaching this bound is an explicit failure mode unless the current task
    /// has already certified as a complete leaf.
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
    pub ref_point: Point3,
    /// Plane triples that certify constructions of `ref_point`.
    pub ref_definitions: Vec<[crate::geometry::Plane; 3]>,
    /// Winding number at `ref_point`.
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
    context: Option<LeafClassificationCacheContextKey>,
    support: Plane,
    edges: Vec<Plane>,
    delta_w: Vec<i32>,
    winding: HypermeshResult<WindingNumberVector>,
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
struct SubdivisionChildPartition {
    left_polygon_profile: PolygonFamilyProfile,
    left_polygons: Vec<ConvexPolygon>,
    left_bounds: Option<Aabb>,
    right_polygon_profile: PolygonFamilyProfile,
    right_polygons: Vec<ConvexPolygon>,
    right_bounds: Option<Aabb>,
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
struct PolygonFamilyBoundsCacheEntry {
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    bounds: HypermeshResult<Aabb>,
}

#[derive(Clone, Debug, PartialEq)]
struct SplitCandidatesCacheEntry {
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    bounds: Aabb,
    candidates: HypermeshResult<Vec<RankedSplitAttempt>>,
}

#[derive(Clone, Debug, PartialEq)]
struct SplitChildPartition {
    left_polys: Vec<ConvexPolygon>,
    right_polys: Vec<ConvexPolygon>,
    both_count: usize,
}

#[derive(Clone, Debug, PartialEq)]
struct SplitChildPartitionCacheEntry {
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    axis: usize,
    value: Real,
    result: HypermeshResult<SplitChildPartition>,
}

#[derive(Clone, Debug, PartialEq)]
struct RankedSplitAttempt {
    axis: usize,
    value: Real,
    left_polys: Vec<ConvexPolygon>,
    left_bounds: Option<Aabb>,
    right_polys: Vec<ConvexPolygon>,
    right_bounds: Option<Aabb>,
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

#[derive(Clone, Debug, PartialEq)]
struct PolygonAxisValuesCacheEntry {
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    result: HypermeshResult<[Vec<Real>; 3]>,
}

struct SubdivisionRuntimeCaches {
    polygon_family_bounds: RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    polygon_axis_values: RefCell<Vec<PolygonAxisValuesCacheEntry>>,
    split_candidates: RefCell<Vec<SplitCandidatesCacheEntry>>,
    split_child_partitions: RefCell<Vec<SplitChildPartitionCacheEntry>>,
    pairwise_intersections: RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    host_bsp_leaves: RefCell<Vec<HostBspLeavesCacheEntry>>,
    bsp_leaf_certification: RefCell<Vec<BspLeafCertificationCacheEntry>>,
    leaf_classification: RefCell<Vec<LeafClassificationCacheEntry>>,
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
            split_candidates: RefCell::new(Vec::new()),
            split_child_partitions: RefCell::new(Vec::new()),
            pairwise_intersections: RefCell::new(Vec::new()),
            host_bsp_leaves: RefCell::new(Vec::new()),
            bsp_leaf_certification: RefCell::new(Vec::new()),
            leaf_classification: RefCell::new(Vec::new()),
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
    process_leaf_into_inner_with_pairwise_cache(
        polygons,
        bounds,
        ref_point,
        ref_definitions,
        ref_wnv,
        indicator,
        output,
        &leaf_classification_cache,
        pairwise_intersections_by_polygon,
        build_host_bsp_leaves,
        certify_bsp_leaf_and_delta_w,
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
    let leaf_cache_context = LeafClassificationCacheContextKey {
        polygon_profile: polygon_family_profile(polygons),
        polygons: polygons.to_vec(),
        bounds: bounds.clone(),
        ref_point: ref_point.clone(),
        ref_definitions: ref_definitions.to_vec(),
        ref_wnv: ref_wnv.to_vec(),
    };

    let intersections = pairwise_query(polygons)?;
    stats.intersection_count = intersections.iter().map(Vec::len).sum();

    for (index, polygon) in polygons.iter().enumerate() {
        if intersections[index].is_empty() {
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
                output,
                &mut output_buckets,
            )?;
            stats.direct_polygon_count += usize::from(emitted);
            continue;
        }

        let mut seen_bsp_leaf_edges = Vec::new();
        let bsp_leaves = bsp_leaves_query(polygon, polygons, &intersections[index])?;
        for leaf in &bsp_leaves {
            if leaf.edges.len() < 3 {
                continue;
            }
            if !take_new_bsp_leaf_edge_cycle(&mut seen_bsp_leaf_edges, &leaf.edges) {
                continue;
            }
            let (interior_points, effective_delta_w) =
                certify_bsp_leaf(polygon, &leaf.edges, polygons)?;
            stats.bsp_leaf_count += 1;
            let w_front = cached_leaf_classification_with(
                &mut leaf_classification_cache.borrow_mut(),
                Some(&leaf_cache_context),
                &polygon.support,
                &leaf.edges,
                &effective_delta_w,
                || {
                    classify_leaf_polygon_from_interior_points(
                        &interior_points,
                        &polygon.support,
                        ref_point,
                        ref_definitions,
                        ref_wnv,
                        polygons,
                        bounds,
                        &effective_delta_w,
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
    task: SubdivisionTask,
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

    let mut output_buckets = ClassifiedPolygonBucketState::from_classified(output);

    if let Some(op) = reachability_op {
        if cached_winding_reachability_with(
            winding_reachability_cache,
            op,
            &task.ref_wnv,
            &task.polygons,
            || can_discard_by_winding_reachability(op, &task.ref_wnv, &task.polygons),
        )? {
            return Ok(());
        }
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

    if task.depth >= config.max_depth {
        return Err(crate::error::HypermeshError::SubdivisionDepthLimit {
            depth: task.depth,
            polygon_count: task.polygons.len(),
        });
    }

    let split_candidates = cached_ordered_subdivision_splits_with(
        &caches.polygon_axis_values,
        &caches.split_candidates,
        &caches.split_child_partitions,
        &caches.polygon_family_bounds,
        &caches.pairwise_intersections,
        &task.bounds,
        &task.polygons,
    )?;
    let mut best_failure = None;

    for split_attempt in split_candidates {
        let left_polys = split_attempt.left_polys;
        let left_bounds = split_attempt.left_bounds;
        let right_polys = split_attempt.right_polys;
        let right_bounds = split_attempt.right_bounds;
        let mut candidate_output = Vec::new();
        let mut candidate_buckets = ClassifiedPolygonBucketState::new();
        let attempt = (|| -> HypermeshResult<()> {
            if let Some(left_bounds) = left_bounds {
                let reused_left_reference = {
                    let mut query_caches = caches.support_reference_query.borrow_mut();
                    if let Some(reused) = reusable_child_reference_if_certified(
                        &task,
                        &left_polys,
                        &left_bounds,
                        &mut query_caches,
                    )? {
                        Some(reused)
                    } else {
                        reusable_child_reference_from_cached_trace_if_certified(
                            &caches.child_reference,
                            &task.ref_point,
                            &task.ref_definitions,
                            &task.ref_wnv,
                            &task.polygons,
                            &left_bounds,
                            &mut query_caches,
                        )?
                    }
                };
                let (left_ref, left_ref_definitions, left_wnv) =
                    if let Some(reused) = reused_left_reference {
                        reused
                    } else {
                        cached_child_reference_with(
                            &caches.child_reference,
                            &task.ref_point,
                            &task.ref_definitions,
                            &task.ref_wnv,
                            &task.polygons,
                            &left_bounds,
                            || {
                                compute_new_reference_with_query_caches(
                                    &task.ref_point,
                                    &task.ref_definitions,
                                    &task.ref_wnv,
                                    &left_bounds,
                                    &task.polygons,
                                    &mut caches.support_reference_query.borrow_mut(),
                                )
                            },
                        )?
                    };
                let left_task = SubdivisionTask {
                    polygons: left_polys,
                    bounds: left_bounds,
                    ref_point: left_ref,
                    ref_definitions: left_ref_definitions,
                    ref_wnv: left_wnv,
                    depth: task.depth + 1,
                };
                let child_output = if let Some(reused) = {
                    let mut query_caches = caches.support_reference_query.borrow_mut();
                    reusable_child_subdivision_if_certified(
                        &caches.child_subdivision,
                        &left_task,
                        &mut query_caches,
                    )?
                } {
                    reused
                } else {
                    cached_child_subdivision_with(&caches.child_subdivision, &left_task, || {
                        let mut child_output = Vec::new();
                        subdivide_into_inner_with(
                            left_task.clone(),
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
                    &mut candidate_output,
                    &mut candidate_buckets,
                    child_output,
                );
            }

            if let Some(right_bounds) = right_bounds {
                let reused_right_reference = {
                    let mut query_caches = caches.support_reference_query.borrow_mut();
                    if let Some(reused) = reusable_child_reference_if_certified(
                        &task,
                        &right_polys,
                        &right_bounds,
                        &mut query_caches,
                    )? {
                        Some(reused)
                    } else {
                        reusable_child_reference_from_cached_trace_if_certified(
                            &caches.child_reference,
                            &task.ref_point,
                            &task.ref_definitions,
                            &task.ref_wnv,
                            &task.polygons,
                            &right_bounds,
                            &mut query_caches,
                        )?
                    }
                };
                let (right_ref, right_ref_definitions, right_wnv) =
                    if let Some(reused) = reused_right_reference {
                        reused
                    } else {
                        cached_child_reference_with(
                            &caches.child_reference,
                            &task.ref_point,
                            &task.ref_definitions,
                            &task.ref_wnv,
                            &task.polygons,
                            &right_bounds,
                            || {
                                compute_new_reference_with_query_caches(
                                    &task.ref_point,
                                    &task.ref_definitions,
                                    &task.ref_wnv,
                                    &right_bounds,
                                    &task.polygons,
                                    &mut caches.support_reference_query.borrow_mut(),
                                )
                            },
                        )?
                    };
                let right_task = SubdivisionTask {
                    polygons: right_polys,
                    bounds: right_bounds,
                    ref_point: right_ref,
                    ref_definitions: right_ref_definitions,
                    ref_wnv: right_wnv,
                    depth: task.depth + 1,
                };
                let child_output = if let Some(reused) = {
                    let mut query_caches = caches.support_reference_query.borrow_mut();
                    reusable_child_subdivision_if_certified(
                        &caches.child_subdivision,
                        &right_task,
                        &mut query_caches,
                    )?
                } {
                    reused
                } else {
                    cached_child_subdivision_with(&caches.child_subdivision, &right_task, || {
                        let mut child_output = Vec::new();
                        subdivide_into_inner_with(
                            right_task.clone(),
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
                    &mut candidate_output,
                    &mut candidate_buckets,
                    child_output,
                );
            }

            Ok(())
        })();

        match attempt {
            Ok(()) => {
                merge_unique_classified_polygons_with_bucket_state(
                    output,
                    &mut output_buckets,
                    candidate_output,
                );
                return Ok(());
            }
            Err(err) if is_backtrackable_split_error(&err) => {
                record_split_failure(&mut best_failure, err);
            }
            Err(err) => return Err(err),
        }
    }

    Err(best_failure.unwrap_or(crate::error::HypermeshError::UnknownClassification))
}

#[cfg(test)]
fn recursive_child_bounds(
    parent_polygons: &[ConvexPolygon],
    child_polygons: &[ConvexPolygon],
    child_bounds: &Aabb,
) -> HypermeshResult<Aabb> {
    if polygon_families_match_as_multisets(child_polygons, parent_polygons) {
        return polygon_family_bounds(child_polygons);
    }
    Ok(child_bounds.clone())
}

fn cached_polygon_family_bounds_with(
    cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    polygons: &[ConvexPolygon],
    query: impl FnOnce(&[ConvexPolygon]) -> HypermeshResult<Aabb>,
) -> HypermeshResult<Aabb> {
    let polygon_profile = polygon_family_profile(polygons);
    if let Some(existing) = cache.borrow().iter().find(|existing| {
        existing.polygon_profile == polygon_profile
            && polygon_families_match_as_multisets(&existing.polygons, polygons)
    }) {
        return existing.bounds.clone();
    }

    let bounds = query(polygons);
    cache.borrow_mut().push(PolygonFamilyBoundsCacheEntry {
        polygon_profile,
        polygons: polygons.to_vec(),
        bounds: bounds.clone(),
    });
    bounds
}

fn cached_recursive_child_bounds_with(
    cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    parent_polygons: &[ConvexPolygon],
    child_polygons: &[ConvexPolygon],
    child_bounds: &Aabb,
) -> HypermeshResult<Aabb> {
    if polygon_families_match_as_multisets(child_polygons, parent_polygons) {
        return cached_polygon_family_bounds_with(cache, child_polygons, polygon_family_bounds);
    }
    Ok(child_bounds.clone())
}

fn polygon_axis_values(polygons: &[ConvexPolygon]) -> HypermeshResult<[Vec<Real>; 3]> {
    let mut values = [Vec::new(), Vec::new(), Vec::new()];
    for polygon in polygons {
        for vertex in polygon.vertices()? {
            for axis in 0..3 {
                push_unique_ordered_axis_value(&mut values[axis], axis_ref(&vertex, axis).clone())?;
            }
        }
    }
    Ok(values)
}

fn cached_polygon_axis_values_with(
    cache: &RefCell<Vec<PolygonAxisValuesCacheEntry>>,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<[Vec<Real>; 3]> {
    let polygon_profile = polygon_family_profile(polygons);
    if let Some(existing) = cache.borrow().iter().find(|existing| {
        existing.polygon_profile == polygon_profile
            && polygon_families_match_as_multisets(&existing.polygons, polygons)
    }) {
        return existing.result.clone();
    }

    let result = polygon_axis_values(polygons);
    cache.borrow_mut().push(PolygonAxisValuesCacheEntry {
        polygon_profile,
        polygons: polygons.to_vec(),
        result: result.clone(),
    });
    result
}

fn cached_ordered_subdivision_splits_with(
    axis_values_cache: &RefCell<Vec<PolygonAxisValuesCacheEntry>>,
    cache: &RefCell<Vec<SplitCandidatesCacheEntry>>,
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygon_bounds_cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    pairwise_cache: &RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<RankedSplitAttempt>> {
    let polygon_profile = polygon_family_profile(polygons);
    if let Some(existing) = cache.borrow().iter().find(|existing| {
        existing.bounds == *bounds
            && existing.polygon_profile == polygon_profile
            && polygon_families_match_as_multisets(&existing.polygons, polygons)
    }) {
        return existing.candidates.clone();
    }

    let candidates = ordered_subdivision_splits_with_partition_cache(
        bounds,
        polygons,
        axis_values_cache,
        partition_cache,
        polygon_bounds_cache,
        pairwise_cache,
    );
    cache.borrow_mut().push(SplitCandidatesCacheEntry {
        polygon_profile,
        polygons: polygons.to_vec(),
        bounds: bounds.clone(),
        candidates: candidates.clone(),
    });
    candidates
}

fn cached_split_child_partition_with(
    cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygons: &[ConvexPolygon],
    axis: usize,
    value: &Real,
) -> HypermeshResult<SplitChildPartition> {
    let polygon_profile = polygon_family_profile(polygons);
    for existing in cache.borrow().iter() {
        if existing.axis == axis
            && existing.polygon_profile == polygon_profile
            && polygon_families_match_as_multisets(&existing.polygons, polygons)
            && compare_real(&existing.value, value)?.is_eq()
        {
            return existing.result.clone();
        }
    }

    let result = split_child_partition(polygons, axis, value);
    cache.borrow_mut().push(SplitChildPartitionCacheEntry {
        polygon_profile,
        polygons: polygons.to_vec(),
        axis,
        value: value.clone(),
        result: result.clone(),
    });
    result
}

fn take_new_subdivision_child_partition(
    seen: &mut Vec<SubdivisionChildPartition>,
    left_polygons: &[ConvexPolygon],
    left_bounds: Option<&Aabb>,
    right_polygons: &[ConvexPolygon],
    right_bounds: Option<&Aabb>,
) -> bool {
    let left_polygon_profile = polygon_family_profile(left_polygons);
    let right_polygon_profile = polygon_family_profile(right_polygons);
    for existing in seen.iter() {
        let direct_match = existing.left_polygon_profile == left_polygon_profile
            && existing.left_bounds.as_ref() == left_bounds
            && existing.right_polygon_profile == right_polygon_profile
            && existing.right_bounds.as_ref() == right_bounds
            && polygon_families_match_as_multisets(&existing.left_polygons, left_polygons)
            && polygon_families_match_as_multisets(&existing.right_polygons, right_polygons);
        let swapped_match = existing.left_polygon_profile == right_polygon_profile
            && existing.left_bounds.as_ref() == right_bounds
            && existing.right_polygon_profile == left_polygon_profile
            && existing.right_bounds.as_ref() == left_bounds
            && polygon_families_match_as_multisets(&existing.left_polygons, right_polygons)
            && polygon_families_match_as_multisets(&existing.right_polygons, left_polygons);
        if direct_match || swapped_match {
            return false;
        }
    }

    seen.push(SubdivisionChildPartition {
        left_polygon_profile,
        left_polygons: left_polygons.to_vec(),
        left_bounds: left_bounds.cloned(),
        right_polygon_profile,
        right_polygons: right_polygons.to_vec(),
        right_bounds: right_bounds.cloned(),
    });
    true
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
    let source_polygon_profile = polygon_family_profile(source_polygons);
    if let Some(existing) = cache.borrow().iter().find(|existing| {
        existing.old_ref == *old_ref
            && reference_definition_families_match_as_sets(
                &existing.old_ref_definitions,
                old_ref_definitions,
            )
            && existing.old_wnv == old_wnv
            && existing.source_polygon_profile == source_polygon_profile
            && polygon_families_match_as_multisets(&existing.source_polygons, source_polygons)
            && existing.bounds == *bounds
    }) {
        return existing.result.clone();
    }

    let result = query();
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

fn reusable_child_reference_if_certified(
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
        Ok(true) => Ok(Some((
            task.ref_point.clone(),
            task.ref_definitions.clone(),
            task.ref_wnv.clone(),
        ))),
        Ok(false) | Err(crate::error::HypermeshError::UnknownClassification) => Ok(None),
        Err(err) => Err(err),
    }
}

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
    let candidates = cache
        .borrow()
        .iter()
        .filter(|existing| {
            existing.source_polygon_profile == source_polygon_profile
                && existing.bounds == *bounds
                && existing.old_wnv == old_wnv
                && polygon_families_match_as_multisets(&existing.source_polygons, source_polygons)
        })
        .filter_map(|existing| match &existing.result {
            Ok((point, definitions, _)) => Some((point.clone(), definitions.clone())),
            Err(_) => None,
        })
        .collect::<Vec<_>>();
    let context =
        support_reference_cache_context_key(old_ref, old_ref_definitions, old_wnv, source_polygons);

    for (point, definitions) in candidates {
        let valid_for_bounds = cached_reference_bounds_validity_with_context(
            &mut query_caches.validity_cache,
            Some(&context),
            bounds,
            &point,
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
                    source_polygons,
                    target,
                )
            },
        )? {
            return Ok(Some((point, definitions, winding)));
        }
    }

    Ok(None)
}

fn child_task_reference_is_certified_valid(
    task: &SubdivisionTask,
    query_caches: &mut SupportReferenceQueryCaches,
) -> HypermeshResult<bool> {
    let context = support_reference_cache_context_key(
        &task.ref_point,
        &task.ref_definitions,
        &task.ref_wnv,
        &task.polygons,
    );
    cached_reference_bounds_validity_with_context(
        &mut query_caches.validity_cache,
        Some(&context),
        &task.bounds,
        &task.ref_point,
        |point| is_certified_valid_reference_for_bounds(point, &task.bounds, &task.polygons),
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
    for existing in cache.borrow().iter() {
        if existing.polygon_profile != polygon_profile
            || existing.task.bounds != task.bounds
            || existing.task.ref_wnv != task.ref_wnv
            || !polygon_families_match_as_multisets(&existing.task.polygons, &task.polygons)
            || !matches!(existing.result, Ok(_))
            || (existing.task.depth != task.depth && existing.task.depth <= task.depth)
        {
            continue;
        }

        if child_task_reference_is_certified_valid(&existing.task, query_caches)? {
            if let Ok(result) = &existing.result {
                return Ok(Some(result.clone()));
            }
        }
    }

    Ok(None)
}

fn cached_child_subdivision_with(
    cache: &RefCell<Vec<ChildSubdivisionCacheEntry>>,
    task: &SubdivisionTask,
    query: impl FnOnce() -> HypermeshResult<Vec<ClassifiedPolygon>>,
) -> HypermeshResult<Vec<ClassifiedPolygon>> {
    let polygon_profile = polygon_family_profile(&task.polygons);
    if let Some(existing) = cache.borrow().iter().find(|existing| {
        existing.polygon_profile == polygon_profile
            && subdivision_task_state_matches_for_cache(&existing.task, task)
            && (existing.task.depth == task.depth
                || (existing.task.depth > task.depth && existing.result.is_ok()))
    }) {
        return existing.result.clone();
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
    left: Option<&LeafClassificationCacheContextKey>,
    right: Option<&LeafClassificationCacheContextKey>,
) -> bool {
    match (left, right) {
        (None, None) => true,
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
                          polygons: &[ConvexPolygon]| {
        cached_bsp_leaf_certification_with(bsp_leaf_cache, polygon, leaf_edges, polygons)
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
    context: Option<&LeafClassificationCacheContextKey>,
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
            classify_leaf_polygon(
                &polygon.support,
                &polygon.edges,
                ref_point,
                ref_definitions,
                ref_wnv,
                class_polygons,
                bounds,
                &polygon.delta_w,
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
    context: Option<&LeafClassificationCacheContextKey>,
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

fn build_host_bsp_leaves(
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

    let result = certify_bsp_leaf_and_delta_w(host, leaf_edges, polygons);
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

fn certify_bsp_leaf_and_delta_w(
    polygon: &ConvexPolygon,
    leaf_edges: &[crate::geometry::Plane],
    polygons: &[ConvexPolygon],
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

    for other in polygons {
        if other.polygon_index == polygon.polygon_index && other.mesh_index == polygon.mesh_index {
            continue;
        }
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
    let polygon_profile = polygon_family_profile(polygons);
    for existing in cache.borrow().iter() {
        if existing.polygon_profile != polygon_profile {
            continue;
        }
        if existing.polygons == polygons {
            return existing.result.clone();
        }
        if let Some(query_to_cached) = polygon_family_order_mapping(polygons, &existing.polygons) {
            return remap_pairwise_intersections_for_polygon_order(
                existing.result.clone(),
                &query_to_cached,
            );
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

fn can_split_bounds(bounds: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        if compare_real(&bounds.extent(axis), &Real::zero())?.is_gt() {
            return Ok(true);
        }
    }
    Ok(false)
}

fn polygon_family_bounds(polygons: &[ConvexPolygon]) -> HypermeshResult<Aabb> {
    let mut vertices = Vec::new();
    for polygon in polygons {
        vertices.extend(polygon.vertices()?);
    }
    let first = vertices
        .pop()
        .ok_or(crate::error::HypermeshError::UnknownClassification)?;
    let mut min = first.clone();
    let mut max = first;

    for vertex in vertices {
        for axis in 0..3 {
            if compare_real(axis_ref(&vertex, axis), axis_ref(&min, axis))?.is_lt() {
                *axis_mut(&mut min, axis) = axis_ref(&vertex, axis).clone();
            }
            if compare_real(axis_ref(&vertex, axis), axis_ref(&max, axis))?.is_gt() {
                *axis_mut(&mut max, axis) = axis_ref(&vertex, axis).clone();
            }
        }
    }

    Ok(Aabb::new(min, max))
}

#[cfg(test)]
fn select_subdivision_split(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<(usize, Real)> {
    ordered_subdivision_splits(bounds, polygons)?
        .into_iter()
        .next()
        .ok_or(crate::error::HypermeshError::UnknownClassification)
}

type SplitCounts = (usize, usize, usize, usize);

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
enum SplitSource {
    Intersection,
    Arrangement,
    Midpoint,
}

#[derive(Clone, Debug, PartialEq)]
struct SplitCandidate {
    axis: usize,
    value: Real,
    counts: SplitCounts,
    source: SplitSource,
}

#[cfg(test)]
fn ordered_subdivision_splits(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<(usize, Real)>> {
    let mut candidates = Vec::new();

    for axis in 0..3 {
        if compare_real(&bounds.extent(axis), &Real::zero())?.is_le() {
            continue;
        }
        push_split_candidate(
            &mut candidates,
            polygons,
            axis,
            bounds.midpoint(axis),
            SplitSource::Midpoint,
        )?;
    }

    for axis in 0..3 {
        if compare_real(&bounds.extent(axis), &Real::zero())?.is_le() {
            continue;
        }
        for (_gap, value) in arrangement_split_candidates(bounds, polygons, axis)? {
            push_split_candidate(
                &mut candidates,
                polygons,
                axis,
                value,
                SplitSource::Arrangement,
            )?;
        }
    }

    for axis in 0..3 {
        if compare_real(&bounds.extent(axis), &Real::zero())?.is_le() {
            continue;
        }
        for value in intersection_split_candidates(bounds, polygons, axis)? {
            push_split_candidate(
                &mut candidates,
                polygons,
                axis,
                value,
                SplitSource::Intersection,
            )?;
        }
    }

    candidates.sort_by(|left, right| {
        left.counts
            .cmp(&right.counts)
            .then_with(|| left.source.cmp(&right.source))
    });
    Ok(candidates
        .into_iter()
        .map(|candidate| (candidate.axis, candidate.value))
        .collect())
}

fn ordered_subdivision_splits_with_partition_cache(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis_values_cache: &RefCell<Vec<PolygonAxisValuesCacheEntry>>,
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygon_bounds_cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    pairwise_cache: &RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
) -> HypermeshResult<Vec<RankedSplitAttempt>> {
    let mut candidates = Vec::new();
    let axis_values = cached_polygon_axis_values_with(axis_values_cache, polygons)?;
    let intersection_segments =
        split_intersection_segments_with_pairwise_cache(pairwise_cache, polygons)?;

    for axis in 0..3 {
        if compare_real(&bounds.extent(axis), &Real::zero())?.is_le() {
            continue;
        }
        push_split_candidate_with_partition_cache(
            &mut candidates,
            polygons,
            axis,
            bounds.midpoint(axis),
            SplitSource::Midpoint,
            partition_cache,
        )?;
    }

    for axis in 0..3 {
        if compare_real(&bounds.extent(axis), &Real::zero())?.is_le() {
            continue;
        }
        for (_gap, value) in
            arrangement_split_candidates_from_axis_values(bounds, &axis_values[axis], axis)?
        {
            push_split_candidate_with_partition_cache(
                &mut candidates,
                polygons,
                axis,
                value,
                SplitSource::Arrangement,
                partition_cache,
            )?;
        }
    }

    for axis in 0..3 {
        if compare_real(&bounds.extent(axis), &Real::zero())?.is_le() {
            continue;
        }
        for value in
            intersection_split_candidates_from_segments(bounds, &intersection_segments, axis)?
        {
            push_split_candidate_with_partition_cache(
                &mut candidates,
                polygons,
                axis,
                value,
                SplitSource::Intersection,
                partition_cache,
            )?;
        }
    }

    candidates.sort_by(|left, right| {
        left.counts
            .cmp(&right.counts)
            .then_with(|| left.source.cmp(&right.source))
    });
    let mut unique = Vec::new();
    let mut seen_partitions = Vec::new();
    for candidate in candidates {
        let unclipped_left_bounds = bounds.left_half(candidate.axis, candidate.value.clone());
        let unclipped_right_bounds = bounds.right_half(candidate.axis, candidate.value.clone());
        let split_partition = cached_split_child_partition_with(
            partition_cache,
            polygons,
            candidate.axis,
            &candidate.value,
        )?;
        let left_bounds = if split_partition.left_polys.is_empty() {
            None
        } else {
            Some(cached_recursive_child_bounds_with(
                polygon_bounds_cache,
                polygons,
                &split_partition.left_polys,
                &unclipped_left_bounds,
            )?)
        };
        let right_bounds = if split_partition.right_polys.is_empty() {
            None
        } else {
            Some(cached_recursive_child_bounds_with(
                polygon_bounds_cache,
                polygons,
                &split_partition.right_polys,
                &unclipped_right_bounds,
            )?)
        };
        if take_new_subdivision_child_partition(
            &mut seen_partitions,
            &split_partition.left_polys,
            left_bounds.as_ref(),
            &split_partition.right_polys,
            right_bounds.as_ref(),
        ) {
            unique.push(RankedSplitAttempt {
                axis: candidate.axis,
                value: candidate.value,
                left_polys: split_partition.left_polys,
                left_bounds,
                right_polys: split_partition.right_polys,
                right_bounds,
            });
        }
    }
    Ok(unique)
}

#[cfg(test)]
fn split_intersection_segments(
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<IntersectionSegment>> {
    let bvh = ExactBvh::build(polygons)?;
    let mut candidate_pairs = Vec::new();
    bvh.intersect_pairs(&bvh, |left, right| {
        if left < right {
            candidate_pairs.push((left, right));
        }
    })?;

    let mut segments = Vec::new();
    for (left, right) in candidate_pairs {
        let intersection = intersect_polygons(&polygons[left], &polygons[right], right)?;
        if intersection.kind != PairwiseIntersectionType::Segment {
            continue;
        }
        let Some(segment) = intersection.segment else {
            continue;
        };
        segments.push(segment);
    }
    Ok(segments)
}

fn split_intersection_segments_with_pairwise_cache(
    pairwise_cache: &RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<IntersectionSegment>> {
    let by_polygon = cached_pairwise_intersections_by_polygon_with(pairwise_cache, polygons)?;
    let mut segments = Vec::new();
    for (polygon_idx, intersections) in by_polygon.iter().enumerate() {
        for intersection in intersections {
            if intersection.kind != PairwiseIntersectionType::Segment {
                continue;
            }
            let Some(segment) = &intersection.segment else {
                continue;
            };
            if polygon_idx < segment.other_polygon_idx {
                segments.push(segment.clone());
            }
        }
    }
    Ok(segments)
}

#[cfg(test)]
fn push_split_candidate(
    candidates: &mut Vec<SplitCandidate>,
    polygons: &[ConvexPolygon],
    axis: usize,
    value: Real,
    source: SplitSource,
) -> HypermeshResult<()> {
    for existing in candidates.iter_mut() {
        if existing.axis == axis && compare_real(&existing.value, &value)?.is_eq() {
            if source < existing.source {
                existing.source = source;
            }
            return Ok(());
        }
    }

    candidates.push(SplitCandidate {
        axis,
        counts: split_child_counts(polygons, axis, &value)?,
        source,
        value,
    });
    Ok(())
}

fn push_split_candidate_with_partition_cache(
    candidates: &mut Vec<SplitCandidate>,
    polygons: &[ConvexPolygon],
    axis: usize,
    value: Real,
    source: SplitSource,
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
) -> HypermeshResult<()> {
    for existing in candidates.iter_mut() {
        if existing.axis == axis && compare_real(&existing.value, &value)?.is_eq() {
            if source < existing.source {
                existing.source = source;
            }
            return Ok(());
        }
    }

    let partition = cached_split_child_partition_with(partition_cache, polygons, axis, &value)?;
    let left_count = partition.left_polys.len();
    let right_count = partition.right_polys.len();
    candidates.push(SplitCandidate {
        axis,
        counts: (
            left_count.max(right_count),
            usize::from(left_count == 0 || right_count == 0),
            partition.both_count,
            left_count.abs_diff(right_count),
        ),
        source,
        value,
    });
    Ok(())
}

#[cfg(test)]
fn try_ordered_subdivision_splits<T>(
    split_candidates: &[(usize, Real)],
    mut attempt: impl FnMut(usize, &Real) -> HypermeshResult<T>,
) -> HypermeshResult<T> {
    let mut best_failure = None;

    for (axis, value) in split_candidates {
        match attempt(*axis, value) {
            Ok(result) => return Ok(result),
            Err(err) if is_backtrackable_split_error(&err) => {
                record_split_failure(&mut best_failure, err);
            }
            Err(err) => return Err(err),
        }
    }

    Err(best_failure.unwrap_or(crate::error::HypermeshError::UnknownClassification))
}

fn is_backtrackable_split_error(err: &crate::error::HypermeshError) -> bool {
    matches!(
        err,
        crate::error::HypermeshError::UnknownClassification
            | crate::error::HypermeshError::ReferencePropagationFailed
            | crate::error::HypermeshError::SubdivisionDepthLimit { .. }
    )
}

fn record_split_failure(
    best_failure: &mut Option<crate::error::HypermeshError>,
    candidate: crate::error::HypermeshError,
) {
    let candidate_priority = split_failure_priority(&candidate);
    if best_failure
        .as_ref()
        .is_none_or(|existing| candidate_priority > split_failure_priority(existing))
    {
        *best_failure = Some(candidate);
    }
}

fn split_failure_priority(err: &crate::error::HypermeshError) -> u8 {
    match err {
        crate::error::HypermeshError::SubdivisionDepthLimit { .. } => 3,
        crate::error::HypermeshError::ReferencePropagationFailed => 2,
        crate::error::HypermeshError::UnknownClassification => 1,
        _ => 0,
    }
}

#[cfg(test)]
fn consider_split_candidates(
    best_axis: &mut usize,
    best_value: &mut Real,
    best_counts: &mut SplitCounts,
    axis: usize,
    candidates: impl IntoIterator<Item = Real>,
    mut split_counts: impl FnMut(&Real) -> HypermeshResult<SplitCounts>,
) -> HypermeshResult<()> {
    for value in candidates {
        let counts = split_counts(&value)?;
        if split_counts_strictly_better(counts, *best_counts) {
            *best_axis = axis;
            *best_value = value;
            *best_counts = counts;
        }
    }
    Ok(())
}

#[cfg(test)]
fn split_counts_strictly_better(candidate: SplitCounts, baseline: SplitCounts) -> bool {
    candidate.0 < baseline.0
        || (candidate.0 == baseline.0
            && (candidate.1 < baseline.1
                || (candidate.1 == baseline.1
                    && (candidate.2 < baseline.2
                        || (candidate.2 == baseline.2 && candidate.3 < baseline.3)))))
}

#[cfg(test)]
fn arrangement_split_candidates(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
) -> HypermeshResult<Vec<(Real, Real)>> {
    let axis_values = polygon_axis_values(polygons)?;
    arrangement_split_candidates_from_axis_values(bounds, &axis_values[axis], axis)
}

fn arrangement_split_candidates_from_axis_values(
    bounds: &Aabb,
    axis_values: &[Real],
    axis: usize,
) -> HypermeshResult<Vec<(Real, Real)>> {
    let min = axis_ref(&bounds.min, axis);
    let max = axis_ref(&bounds.max, axis);
    if !compare_real(min, max)?.is_lt() {
        return Ok(Vec::new());
    }

    let mut values = Vec::new();
    for value in axis_values {
        if compare_real(value, min)?.is_gt() && compare_real(value, max)?.is_lt() {
            values.push(value.clone());
        }
    }
    if values.is_empty() {
        return Ok(Vec::new());
    }

    let mut candidates = axis_gap_candidates_between_values(&values)?;
    if !candidates.is_empty() {
        return Ok(candidates);
    }

    update_axis_gap_candidates(&mut candidates, min, &values[0])?;
    update_axis_gap_candidates(&mut candidates, values.last().expect("non-empty"), max)?;
    Ok(candidates)
}

#[cfg(test)]
fn intersection_split_candidates(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
) -> HypermeshResult<Vec<Real>> {
    let segments = split_intersection_segments(polygons)?;
    intersection_split_candidates_from_segments(bounds, &segments, axis)
}

fn intersection_split_candidates_from_segments(
    bounds: &Aabb,
    segments: &[IntersectionSegment],
    axis: usize,
) -> HypermeshResult<Vec<Real>> {
    let min = axis_ref(&bounds.min, axis);
    let max = axis_ref(&bounds.max, axis);
    if !compare_real(min, max)?.is_lt() {
        return Ok(Vec::new());
    }

    let mut values = Vec::new();
    for segment in segments {
        for point in [&segment.v0, &segment.v1] {
            let value = axis_ref(point, axis);
            if compare_real(value, min)?.is_gt() && compare_real(value, max)?.is_lt() {
                push_unique_ordered_axis_value(&mut values, value.clone())?;
            }
        }
    }

    Ok(values)
}

fn axis_gap_candidates_between_values(values: &[Real]) -> HypermeshResult<Vec<(Real, Real)>> {
    let mut candidates = Vec::new();
    for pair in values.windows(2) {
        update_axis_gap_candidates(&mut candidates, &pair[0], &pair[1])?;
    }
    Ok(candidates)
}

fn update_axis_gap_candidates(
    candidates: &mut Vec<(Real, Real)>,
    start: &Real,
    end: &Real,
) -> HypermeshResult<()> {
    if !compare_real(start, end)?.is_lt() {
        return Ok(());
    }
    let gap = end - start;
    let midpoint = ((start + end) / Real::from(2))
        .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
    candidates.push((gap, midpoint));
    Ok(())
}

fn push_unique_ordered_axis_value(values: &mut Vec<Real>, value: Real) -> HypermeshResult<()> {
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

#[cfg(test)]
fn split_child_counts(
    polygons: &[ConvexPolygon],
    axis: usize,
    value: &Real,
) -> HypermeshResult<SplitCounts> {
    let partition = split_child_partition(polygons, axis, value)?;
    let left_count = partition.left_polys.len();
    let right_count = partition.right_polys.len();

    Ok((
        left_count.max(right_count),
        usize::from(left_count == 0 || right_count == 0),
        partition.both_count,
        left_count.abs_diff(right_count),
    ))
}

fn split_child_partition(
    polygons: &[ConvexPolygon],
    axis: usize,
    value: &Real,
) -> HypermeshResult<SplitChildPartition> {
    let split_plane = Plane::axis_aligned(axis, value.clone());
    let mut left_polys = Vec::with_capacity(polygons.len());
    let mut right_polys = Vec::with_capacity(polygons.len());
    let mut both_count = 0;

    for polygon in polygons {
        let clipped = clip_polygon(polygon, &split_plane)?;
        match clipped.side {
            ClipSide::Left => left_polys.push(polygon.clone()),
            ClipSide::Right => right_polys.push(polygon.clone()),
            ClipSide::Both => {
                left_polys.push(clipped.left);
                right_polys.push(clipped.right);
                both_count += 1;
            }
        }
    }

    Ok(SplitChildPartition {
        left_polys,
        right_polys,
        both_count,
    })
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
            return Ok((
                old_ref.clone(),
                old_ref_definitions.to_vec(),
                old_wnv.to_vec(),
            ));
        }
        Ok(false) => false,
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
        search_projected_reference_families(
            &projected_root.projected_targets,
            &projected_root.projected_escape_targets,
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
                    bounds,
                    projected_target,
                    |point| is_certified_valid_reference_for_bounds(point, bounds, polygons),
                    |target| {
                        trace_reference_target_from_validated_bounds(
                            old_ref,
                            old_ref_definitions,
                            old_wnv,
                            polygons,
                            target,
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
        ),
        &mut projected_unknown,
    )?;
    let support = support_plane_cell_reference_with_query_caches(
        old_ref,
        old_ref_definitions,
        old_wnv,
        bounds,
        polygons,
        &mut query_caches.borrow_mut(),
    )?;

    reference_result_or_error(projected, support, projected_unknown)
}

#[derive(Clone)]
struct ProjectedRootReferenceFamilies {
    report: Option<hyperlimit::HalfspaceFeasibilityReport>,
    projected_targets: Vec<ReferenceTarget>,
    projected_escape_targets: Vec<ReferenceTarget>,
    saw_unknown: bool,
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
    pure_halfspace_contains_cache: &std::cell::RefCell<
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
            projected_escape_targets: Vec::new(),
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
    let projected_escape_targets =
        projected_reference_escape_targets_from_seed_families_with_tracking_unknown_and_witness_cache(
            halfspaces,
            &projected_targets,
            report.as_ref(),
            strict_seeds,
            shifted_vertices,
            shifted_geometry_seeds,
            &mut saw_unknown,
            reference_witness_cache,
            &pure_halfspace_contains_cache,
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
        )?;

    Ok(ProjectedRootReferenceFamilies {
        report,
        projected_targets,
        projected_escape_targets,
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
        return Ok((target.point, target.definitions, winding));
    }
    if let Some((target, winding)) = support {
        return Ok((target.point, target.definitions, winding));
    }
    if projected_unknown {
        Err(crate::error::HypermeshError::UnknownClassification)
    } else {
        Err(crate::error::HypermeshError::ReferencePropagationFailed)
    }
}

fn search_projected_reference_families(
    projected_targets: &[ReferenceTarget],
    projected_escape_targets: &[ReferenceTarget],
    mut projected_support_search: impl FnMut() -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
    mut trace_projected_target: impl FnMut(&ReferenceTarget) -> HypermeshResult<Option<Vec<i32>>>,
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
                if projected_target.uncertified_definition_fallback {
                    saw_unknown = true;
                } else {
                    return Ok(Some((projected_target.clone(), winding)));
                }
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
        Ok(Some(found)) => {
            if found.0.uncertified_definition_fallback {
                saw_unknown = true;
            } else {
                return Ok(Some(found));
            }
        }
        Ok(None) => {}
        Err(crate::error::HypermeshError::UnknownClassification) => {
            saw_unknown = true;
        }
        Err(err) => return Err(err),
    }

    for projected_target in projected_escape_targets {
        if !traced_direct_targets
            .iter()
            .any(|candidate| reference_targets_match_for_trace_cache(candidate, projected_target))
        {
            match trace_projected_target(projected_target) {
                Ok(Some(winding)) => {
                    if projected_target.uncertified_definition_fallback {
                        saw_unknown = true;
                    } else {
                        return Ok(Some((projected_target.clone(), winding)));
                    }
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
            Ok(Some(found)) => {
                if found.0.uncertified_definition_fallback {
                    saw_unknown = true;
                } else {
                    return Ok(Some(found));
                }
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
            Ok(Some(found)) => {
                if found.0.uncertified_definition_fallback {
                    saw_unknown = true;
                } else {
                    return Ok(Some(found));
                }
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

#[cfg(test)]
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
            Ok(Some(found)) => {
                if found.0.uncertified_definition_fallback {
                    saw_unknown = true;
                } else {
                    return Ok(Some(found));
                }
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

    keyed_boxes.sort_by(|left, right| left.0.cmp(&right.0));

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
        |crossing, polygon| classify_point_in_local_polygon(crossing, polygon),
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
    match collect_reference_target_family(strict_seeds.iter().cloned(), |seed| {
        Ok(build(&seed)?.into_iter().collect())
    }) {
        Ok(targets) => Ok(targets),
        Err(crate::error::HypermeshError::UnknownClassification) => {
            *saw_unknown = true;
            Ok(Vec::new())
        }
        Err(err) => Err(err),
    }
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
    context: Option<SupportReferenceCacheContextKey>,
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
    context: Option<SupportReferenceCacheContextKey>,
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
    support_reference_result_cache: Vec<SupportReferenceResultCacheEntry>,
    search_cache:
        std::cell::RefCell<Vec<SupportPlaneCellSearchCacheEntry<(ReferenceTarget, Vec<i32>)>>>,
}

impl SupportReferenceQueryCaches {
    fn reset_per_reference_call_caches(&mut self) {}
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
    if let Some(cached_feasible) = cached_feasible {
        if !query_caches
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
}

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
        support_reference_polygon_context_matches(existing.context.as_ref(), context)
            && existing.bounds == *bounds
            && existing.point == *point
    }) {
        return existing.is_valid.clone();
    }

    let is_valid = query(point);
    cache.push(ReferenceBoundsValidityCacheEntry {
        context: context.cloned(),
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
        support_reference_polygon_context_matches(existing.context.as_ref(), context)
            && existing.point == *point
    }) {
        return existing.on_support_surface.clone();
    }

    let on_support_surface = query(point);
    cache.push(SupportSurfaceCacheEntry {
        context: context.cloned(),
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

#[derive(Clone, Debug, PartialEq)]
struct SupportReferenceCacheContextKey {
    old_ref: Point3,
    old_ref_definitions: Vec<[Plane; 3]>,
    old_wnv: Vec<i32>,
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
    if let Some(existing) = cache.iter().find(|existing| {
        support_reference_cache_context_matches(existing.context.as_ref(), context)
            && existing.bounds == *bounds
            && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            && support_optional_halfspace_reports_match_for_cache(
                &existing.halfspaces,
                existing.report.as_ref(),
                halfspaces,
                report,
            )
    }) {
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

fn cached_support_reference_result_with(
    cache: &mut Vec<SupportReferenceResultCacheEntry>,
    context: &SupportReferenceCacheContextKey,
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    build: impl FnOnce() -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>>,
) -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
    if let Some(existing) = cache.iter().find(|existing| {
        existing.bounds == *bounds
            && limit_plane_families_match_as_sets(&existing.halfspaces, halfspaces)
            && support_reference_cache_context_matches(Some(&existing.context), Some(context))
    }) {
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

#[derive(Clone)]
struct SupportPlaneCellSearchCacheEntry<T: Clone> {
    context: Option<SupportReferenceCacheContextKey>,
    preferred_order: [bool; 2],
    bounds: Aabb,
    polygon_index: usize,
    halfspaces: Vec<LimitPlane3>,
    result: HypermeshResult<Option<T>>,
}

fn cached_support_plane_cell_search_with<T: Clone>(
    cache: &std::cell::RefCell<Vec<SupportPlaneCellSearchCacheEntry<T>>>,
    context: Option<&SupportReferenceCacheContextKey>,
    preferred_order: [bool; 2],
    bounds: &Aabb,
    polygon_index: usize,
    halfspaces: Vec<LimitPlane3>,
    search: impl FnOnce() -> HypermeshResult<Option<T>>,
) -> HypermeshResult<Option<T>> {
    if let Some(existing) = cache.borrow().iter().find(|existing| {
        support_reference_cache_context_matches(existing.context.as_ref(), context)
            && existing.preferred_order == preferred_order
            && existing.bounds == *bounds
            && existing.polygon_index == polygon_index
            && limit_plane_families_match_as_sets(&existing.halfspaces, &halfspaces)
    }) {
        return existing.result.clone();
    }

    let result = search();
    cache.borrow_mut().push(SupportPlaneCellSearchCacheEntry {
        context: context.cloned(),
        preferred_order,
        bounds: bounds.clone(),
        polygon_index,
        halfspaces,
        result: result.clone(),
    });
    result
}

fn limit_plane_families_match_as_sets(left: &[LimitPlane3], right: &[LimitPlane3]) -> bool {
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
    if let Some(existing) = cache.iter().find(|existing| {
        existing.bounds == *bounds
            && support_reference_cache_context_matches(Some(&existing.context), Some(context))
    }) {
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
        .find(|existing| {
            existing.bounds == *bounds
                && support_reference_cache_context_matches(Some(&existing.context), Some(context))
        })
    {
        return existing.result.clone();
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
                    Ok(Some(found)) => {
                        if found.0.uncertified_definition_fallback {
                            saw_unknown = true;
                        } else {
                            return Ok(Some(found));
                        }
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
        polygons,
        target,
    )
}

fn trace_projected_reference_target_with_queries(
    validity_cache: &mut Vec<ReferenceBoundsValidityCacheEntry>,
    trace_cache: &mut Vec<ReferenceTargetTraceCacheEntry>,
    bounds: &Aabb,
    target: &ReferenceTarget,
    valid_for: impl FnOnce(&Point3) -> HypermeshResult<bool>,
    trace: impl FnOnce(&ReferenceTarget) -> HypermeshResult<Option<Vec<i32>>>,
) -> HypermeshResult<Option<Vec<i32>>> {
    if !cached_reference_bounds_validity_with(validity_cache, bounds, &target.point, valid_for)? {
        return Ok(None);
    }

    cached_reference_target_trace_with(trace_cache, target, trace)
}

fn trace_reference_target_from_validated_bounds(
    old_ref: &Point3,
    old_ref_definitions: &[[Plane; 3]],
    old_wnv: &[i32],
    polygons: &[ConvexPolygon],
    target: &ReferenceTarget,
) -> HypermeshResult<Option<Vec<i32>>> {
    match trace_segment_from_definitions_with_step_detoured_plane_replacement(
        old_ref,
        &target.point,
        old_wnv,
        polygons,
        old_ref_definitions,
        &target.definitions,
    ) {
        Ok(winding) => return Ok(Some(winding)),
        Err(crate::error::HypermeshError::UnknownClassification) => {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        Err(err) => return Err(err),
    }
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
                        |halfspaces| halfspace_system_report(halfspaces),
                        |halfspaces| halfspace_system_is_feasible(halfspaces),
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

    let mut accept = |halfspaces: &[LimitPlane3],
                      report: Option<hyperlimit::HalfspaceFeasibilityReport>|
     -> HypermeshResult<Option<(ReferenceTarget, Vec<i32>)>> {
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
                if target.uncertified_definition_fallback {
                    saw_unknown = true;
                    continue;
                }
                return Ok(Some((target, winding)));
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
        let targets =
            strict_support_cell_targets_from_optional_report(bounds, halfspaces, report.as_ref())?;
        let Some(target) = targets.into_iter().next() else {
            return Ok(None);
        };
        if point_lies_on_any_support_plane(&target.point, polygons)? {
            return Ok(None);
        }
        Ok(Some(target))
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

            let current_report = match report_for(halfspaces) {
                Ok(report) => report,
                Err(crate::error::HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                    None
                }
                Err(err) => return Err(err),
            };

            match accept(halfspaces, current_report) {
                Ok(Some(target)) => return Ok(Some(target)),
                Ok(None) => {}
                Err(crate::error::HypermeshError::UnknownClassification) => {
                    saw_unknown = true;
                }
                Err(err) => return Err(err),
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

fn support_side_halfspace(plane: &crate::geometry::Plane, positive: bool) -> LimitPlane3 {
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

fn halfspace_has_opposite_pair(target: &LimitPlane3, halfspaces: &[LimitPlane3]) -> bool {
    let opposite = negated_halfspace(target);
    halfspaces.iter().any(|halfspace| halfspace == &opposite)
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
            for axis in 0..3 {
                push_verified_definition(
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

#[cfg(test)]
fn strict_support_cell_targets(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: &hyperlimit::HalfspaceFeasibilityReport,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    strict_support_cell_targets_from_optional_report(bounds, halfspaces, Some(report))
}

#[cfg(test)]
fn strict_support_cell_targets_from_optional_report(
    bounds: &Aabb,
    halfspaces: &[LimitPlane3],
    report: Option<&hyperlimit::HalfspaceFeasibilityReport>,
) -> HypermeshResult<Vec<ReferenceTarget>> {
    let mut seed_geometry_cache = Vec::new();
    let mut centroid_subset_seed_cache = Vec::new();
    let mut support_seed_family_cache = Vec::new();
    let mut support_direct_target_cache = Vec::new();
    let mut shifted_support_family_cache = Vec::new();
    let reference_witness_cache = std::cell::RefCell::new(Vec::new());
    let strict_contains_cache = std::cell::RefCell::new(Vec::new());
    strict_support_cell_targets_from_optional_report_with_seed_geometry_cache(
        bounds,
        halfspaces,
        report,
        &mut seed_geometry_cache,
        &mut centroid_subset_seed_cache,
        &mut support_seed_family_cache,
        &mut support_direct_target_cache,
        &mut shifted_support_family_cache,
        &reference_witness_cache,
        &strict_contains_cache,
    )
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
    let shifted_geometry_seed_family = cached_point3_centroid_subset_family_from_vertices_with(
        centroid_subset_seed_cache,
        &shifted_vertices,
        || point3_centroid_subset_family_from_vertices(&shifted_vertices),
    )?;
    saw_unknown |= shifted_geometry_seed_family.saw_unknown;
    let shifted_geometry_seeds = shifted_geometry_seed_family.points;
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

fn point_satisfies_halfspaces(point: &Point3, halfspaces: &[LimitPlane3]) -> HypermeshResult<bool> {
    for halfspace in halfspaces {
        let plane = Plane::new(halfspace.normal.clone(), halfspace.offset.clone());
        if crate::geometry::classify_point(point, &plane)? == Classification::Positive {
            return Ok(false);
        }
    }
    Ok(true)
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

#[cfg(test)]
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

#[cfg(test)]
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
mod tests {
    use super::*;
    use crate::geometry::Plane;
    use crate::intersection::OverlapInfo;
    use crate::mesh::{OutputVertex, PolygonSoup, prepare_input};
    use crate::operations::{EmberConfig, boolean_operation};
    use crate::output::{BooleanResult, TriangleSoup, triangulate_and_resolve_certified};
    use crate::polygon::make_triangle;
    use crate::winding::{BooleanOp, make_indicator};
    use crate::{InputMesh, Triangle};

    fn r(value: i32) -> Real {
        value.into()
    }

    fn q(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    fn quadrilateral_reference_cell_fixture() -> (Aabb, Vec<LimitPlane3>, Point3) {
        let bounds = Aabb::new(p(0, 0, 0), p(5, 4, 0));
        let support = Plane::axis_aligned(2, r(0));
        let interior = Point3::new(q(9, 4), r(2), r(0));
        let vertices = [p(0, 0, 0), p(4, 0, 0), p(5, 4, 0), p(0, 4, 0)];
        let mut halfspaces = vec![
            LimitPlane3::new(support.normal.clone(), support.offset.clone()),
            LimitPlane3::new(
                support.inverted().normal.clone(),
                support.inverted().offset.clone(),
            ),
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
            halfspaces.push(LimitPlane3::new(
                edge_plane.normal.clone(),
                edge_plane.offset.clone(),
            ));
        }

        (bounds, halfspaces, Point3::new(q(5, 2), r(2), r(0)))
    }

    fn px(x: Real, y: i32, z: i32) -> Point3 {
        Point3::new(x, r(y), r(z))
    }

    fn axis_defs(point: &Point3) -> Vec<[Plane; 3]> {
        vec![axis_plane_definition(point)]
    }

    fn tetra_from_face_and_apex(a: Point3, b: Point3, c: Point3, apex: Point3) -> InputMesh {
        InputMesh::new(
            vec![a, b, c, apex],
            vec![
                Triangle::new(0, 2, 1),
                Triangle::new(0, 1, 3),
                Triangle::new(0, 3, 2),
                Triangle::new(1, 2, 3),
            ],
        )
    }

    fn axis_face_polygon(polygons: &[ConvexPolygon], axis: usize, value: i32) -> ConvexPolygon {
        polygons
            .iter()
            .find(|polygon| {
                compare_real(axis_ref(&polygon.support.normal, axis), &Real::zero())
                    .unwrap()
                    .is_gt()
                    && polygon
                        .vertices()
                        .unwrap()
                        .iter()
                        .all(|vertex| axis_ref(vertex, axis) == &r(value))
            })
            .cloned()
            .expect("expected axis-aligned support face in prepared mesh soup")
    }

    #[test]
    fn cached_leaf_classification_reuses_rotated_edge_cycles() {
        let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let mut rotated_edges = polygon.edges[1..].to_vec();
        rotated_edges.push(polygon.edges[0].clone());
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_leaf_classification_with(
            &mut cache,
            None,
            &polygon.support,
            &polygon.edges,
            &polygon.delta_w,
            || {
                calls += 1;
                Ok(vec![7])
            },
        )
        .unwrap();
        let second = cached_leaf_classification_with(
            &mut cache,
            None,
            &polygon.support,
            &rotated_edges,
            &polygon.delta_w,
            || {
                calls += 1;
                Ok(vec![9])
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, vec![7]);
        assert_eq!(second, vec![7]);
    }

    #[test]
    fn cached_leaf_classification_distinguishes_leaf_context() {
        let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let left_context = LeafClassificationCacheContextKey {
            polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
            polygons: vec![polygon.clone()],
            bounds: Aabb::new(p(0, 0, 0), p(2, 2, 0)),
            ref_point: p(0, 0, -1),
            ref_definitions: vec![axis_plane_definition(&p(0, 0, -1))],
            ref_wnv: vec![0],
        };
        let right_context = LeafClassificationCacheContextKey {
            polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
            polygons: vec![polygon.clone()],
            bounds: Aabb::new(p(0, 0, 0), p(2, 2, 0)),
            ref_point: p(0, 0, 1),
            ref_definitions: vec![axis_plane_definition(&p(0, 0, 1))],
            ref_wnv: vec![0],
        };
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_leaf_classification_with(
            &mut cache,
            Some(&left_context),
            &polygon.support,
            &polygon.edges,
            &polygon.delta_w,
            || {
                calls += 1;
                Ok(vec![7])
            },
        )
        .unwrap();
        let second = cached_leaf_classification_with(
            &mut cache,
            Some(&right_context),
            &polygon.support,
            &polygon.edges,
            &polygon.delta_w,
            || {
                calls += 1;
                Ok(vec![9])
            },
        )
        .unwrap();

        assert_eq!(calls, 2);
        assert_eq!(first, vec![7]);
        assert_eq!(second, vec![9]);
    }

    #[test]
    fn cached_bsp_leaf_certification_reuses_permuted_polygon_families() {
        let mut host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        host.delta_w = vec![1, 0];
        let mut cutter = make_triangle(&p(2, 0, 0), &p(0, 0, 0), &p(2, -1, 0), 1, 0);
        cutter.delta_w = vec![0, 1];
        let cache = RefCell::new(Vec::new());

        let first = cached_bsp_leaf_certification_with(
            &cache,
            &host,
            &host.edges,
            &[host.clone(), cutter.clone()],
        )
        .unwrap();
        let second =
            cached_bsp_leaf_certification_with(&cache, &host, &host.edges, &[cutter, host.clone()])
                .unwrap();

        assert_eq!(first, second);
        assert_eq!(cache.borrow().len(), 1);
    }

    #[test]
    fn cached_host_bsp_leaves_reuse_permuted_polygon_families() {
        let mut host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        host.delta_w = vec![1, 0];
        let mut cutter = make_triangle(&p(2, 0, 0), &p(0, 0, 0), &p(2, -1, 0), 1, 0);
        cutter.delta_w = vec![0, 1];
        let cache = RefCell::new(Vec::new());

        let first_polygons = vec![host.clone(), cutter.clone()];
        let first_intersections = pairwise_intersections_by_polygon(&first_polygons).unwrap();
        let first =
            cached_host_bsp_leaves_with(&cache, &host, &first_polygons, &first_intersections[0])
                .unwrap();

        let second_polygons = vec![cutter, host.clone()];
        let second_intersections = pairwise_intersections_by_polygon(&second_polygons).unwrap();
        let second =
            cached_host_bsp_leaves_with(&cache, &host, &second_polygons, &second_intersections[1])
                .unwrap();

        assert_eq!(first, second);
        assert_eq!(cache.borrow().len(), 1);
    }

    #[test]
    fn bsp_leaf_edge_cycle_dedupe_skips_rotated_duplicates() {
        let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let mut rotated_edges = polygon.edges[1..].to_vec();
        rotated_edges.push(polygon.edges[0].clone());
        let mut seen = Vec::new();

        assert!(take_new_bsp_leaf_edge_cycle(&mut seen, &polygon.edges));
        assert!(!take_new_bsp_leaf_edge_cycle(&mut seen, &rotated_edges));
        assert_eq!(seen, vec![polygon.edges.clone()]);
    }

    fn vertex_key(vertex: &OutputVertex) -> [String; 3] {
        [
            vertex.x.to_string(),
            vertex.y.to_string(),
            vertex.z.to_string(),
        ]
    }

    fn sorted_triangle_key(soup: &TriangleSoup, triangle: [usize; 3]) -> [[String; 3]; 3] {
        let mut keys = [
            vertex_key(&soup.vertices[triangle[0]]),
            vertex_key(&soup.vertices[triangle[1]]),
            vertex_key(&soup.vertices[triangle[2]]),
        ];
        keys.sort();
        keys
    }

    fn assert_same_shape(left: &TriangleSoup, right: &TriangleSoup) {
        let left_faces = left
            .triangles
            .iter()
            .map(|triangle| sorted_triangle_key(left, *triangle))
            .collect::<std::collections::BTreeSet<_>>();
        let right_faces = right
            .triangles
            .iter()
            .map(|triangle| sorted_triangle_key(right, *triangle))
            .collect::<std::collections::BTreeSet<_>>();
        assert_eq!(left_faces, right_faces);
    }

    fn definition_uses_non_axis_plane(definition: &[Plane; 3]) -> bool {
        definition.iter().any(|plane| {
            plane.normal != p(1, 0, 0) && plane.normal != p(0, 1, 0) && plane.normal != p(0, 0, 1)
        })
    }

    #[test]
    fn can_split_any_certified_positive_extent() {
        let bounds = Aabb::new(p(0, 0, 0), p(1, 0, 0));

        assert!(can_split_bounds(&bounds).unwrap());
    }

    #[test]
    fn select_subdivision_split_prefers_interior_arrangement_gap() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
        let polygons = vec![
            make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0),
            make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0),
        ];

        let (axis, value) = select_subdivision_split(&bounds, &polygons).unwrap();

        assert_eq!(axis, 0);
        assert_eq!(value, q(3, 2));
    }

    #[test]
    fn select_subdivision_split_avoids_empty_child_midpoint_when_nonempty_midpoint_exists() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
        let polygons = vec![make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 0, 0)];

        let (axis, value) = select_subdivision_split(&bounds, &polygons).unwrap();

        assert_eq!(axis, 1);
        assert_eq!(value, r(1));
    }

    #[test]
    fn select_subdivision_split_can_use_intersection_segment_coordinates() {
        let bounds = Aabb::new(p(-3, 0, -1), p(3, 4, 1));
        let horizontal =
            crate::polygon::make_quad(&p(-3, 0, 0), &p(3, 0, 0), &p(3, 4, 0), &p(-3, 4, 0), 0, 0);
        let vertical = make_triangle(&p(-2, 2, -1), &p(2, 2, -1), &p(1, 2, 1), 1, 0);

        let candidates =
            intersection_split_candidates(&bounds, &[horizontal.clone(), vertical.clone()], 0)
                .unwrap();

        assert_eq!(candidates, vec![q(-1, 2), q(3, 2)]);
        let vertex_candidates =
            arrangement_split_candidates(&bounds, &[horizontal, vertical], 0).unwrap();
        assert!(!vertex_candidates.iter().any(|(_, value)| *value == q(1, 2)));
    }

    #[test]
    fn arrangement_split_candidates_from_axis_values_matches_direct_query() {
        let horizontal = make_triangle(&p(2, 1, 0), &p(8, 1, 0), &p(5, 1, 4), 0, 0);
        let vertical = make_triangle(&p(5, 0, 1), &p(5, 4, 1), &p(5, 2, 4), 1, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
        let polygons = vec![horizontal, vertical];

        let direct = arrangement_split_candidates(&bounds, &polygons, 0).unwrap();
        let axis_values = polygon_axis_values(&polygons).unwrap();
        let cached =
            arrangement_split_candidates_from_axis_values(&bounds, &axis_values[0], 0).unwrap();

        assert_eq!(direct, cached);
    }

    #[test]
    fn cached_polygon_axis_values_reuse_permuted_polygon_families() {
        let polygon_a = make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0);
        let polygon_b = make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0);
        let cache = RefCell::new(Vec::new());

        let first =
            cached_polygon_axis_values_with(&cache, &[polygon_a.clone(), polygon_b.clone()])
                .unwrap();
        let second = cached_polygon_axis_values_with(&cache, &[polygon_b, polygon_a]).unwrap();

        assert_eq!(first, second);
        assert_eq!(cache.borrow().len(), 1);
    }

    #[test]
    fn select_subdivision_split_uses_best_midpoint_across_axes() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
        let polygons = vec![
            make_triangle(&p(0, 0, 0), &p(10, 0, 0), &p(0, 0, 4), 0, 0),
            make_triangle(&p(0, 4, 0), &p(10, 4, 0), &p(0, 4, 4), 1, 0),
        ];

        let (axis, value) = select_subdivision_split(&bounds, &polygons).unwrap();

        assert_eq!(axis, 1);
        assert_eq!(value, r(2));
    }

    #[test]
    fn intersection_split_candidates_can_beat_arrangement_improvement() {
        let mut best_axis = 0;
        let mut best_value = r(5);
        let mut best_counts = (6, 0, 3, 2);

        consider_split_candidates(
            &mut best_axis,
            &mut best_value,
            &mut best_counts,
            0,
            [r(4)],
            |_value| Ok((5, 0, 2, 1)),
        )
        .unwrap();

        assert_eq!(best_axis, 0);
        assert_eq!(best_value, r(4));
        assert_eq!(best_counts, (5, 0, 2, 1));

        consider_split_candidates(
            &mut best_axis,
            &mut best_value,
            &mut best_counts,
            1,
            [r(2)],
            |_value| Ok((4, 0, 0, 0)),
        )
        .unwrap();

        assert_eq!(best_axis, 1);
        assert_eq!(best_value, r(2));
        assert_eq!(best_counts, (4, 0, 0, 0));
    }

    #[test]
    fn exact_split_sources_win_midpoint_ties() {
        let mut candidates = vec![
            SplitCandidate {
                axis: 0,
                value: r(5),
                counts: (4, 0, 1, 0),
                source: SplitSource::Midpoint,
            },
            SplitCandidate {
                axis: 1,
                value: r(2),
                counts: (4, 0, 1, 0),
                source: SplitSource::Arrangement,
            },
            SplitCandidate {
                axis: 2,
                value: r(1),
                counts: (4, 0, 1, 0),
                source: SplitSource::Intersection,
            },
        ];

        candidates.sort_by(|left, right| {
            left.counts
                .cmp(&right.counts)
                .then_with(|| left.source.cmp(&right.source))
        });

        assert_eq!(
            candidates
                .into_iter()
                .map(|candidate| (candidate.axis, candidate.value, candidate.source))
                .collect::<Vec<_>>(),
            vec![
                (2, r(1), SplitSource::Intersection),
                (1, r(2), SplitSource::Arrangement),
                (0, r(5), SplitSource::Midpoint),
            ]
        );
    }

    #[test]
    fn duplicate_split_candidate_promotes_to_exact_source() {
        let polygons = vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)];
        let mut candidates = vec![SplitCandidate {
            axis: 0,
            value: r(5),
            counts: (1, 0, 0, 0),
            source: SplitSource::Midpoint,
        }];

        push_split_candidate(
            &mut candidates,
            &polygons,
            0,
            r(5),
            SplitSource::Arrangement,
        )
        .unwrap();
        assert_eq!(candidates.len(), 1);
        assert_eq!(candidates[0].source, SplitSource::Arrangement);

        push_split_candidate(
            &mut candidates,
            &polygons,
            0,
            r(5),
            SplitSource::Intersection,
        )
        .unwrap();
        assert_eq!(candidates.len(), 1);
        assert_eq!(candidates[0].source, SplitSource::Intersection);
    }

    #[test]
    fn split_ranking_penalizes_empty_child_splits() {
        let mut best_axis = 0;
        let mut best_value = r(5);
        let mut best_counts = (4, 0, 2, 0);

        consider_split_candidates(
            &mut best_axis,
            &mut best_value,
            &mut best_counts,
            1,
            [r(1)],
            |_value| Ok((4, 1, 0, 4)),
        )
        .unwrap();

        assert_eq!(best_axis, 0);
        assert_eq!(best_value, r(5));
        assert_eq!(best_counts, (4, 0, 2, 0));
    }

    #[test]
    fn split_ranking_prefers_lower_child_imbalance_on_count_tie() {
        let mut best_axis = 0;
        let mut best_value = r(5);
        let mut best_counts = (4, 0, 2, 5);

        consider_split_candidates(
            &mut best_axis,
            &mut best_value,
            &mut best_counts,
            1,
            [r(2)],
            |_value| Ok((4, 0, 2, 1)),
        )
        .unwrap();

        assert_eq!(best_axis, 1);
        assert_eq!(best_value, r(2));
        assert_eq!(best_counts, (4, 0, 2, 1));
    }

    #[test]
    fn ordered_subdivision_splits_rank_best_candidate_first() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
        let polygons = vec![
            make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0),
            make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0),
        ];

        let ordered = ordered_subdivision_splits(&bounds, &polygons).unwrap();

        assert!(!ordered.is_empty());
        assert_eq!(ordered[0], (0, q(3, 2)));
    }

    #[test]
    fn intersection_split_candidates_from_segments_matches_direct_query() {
        let horizontal = make_triangle(&p(2, 1, 0), &p(8, 1, 0), &p(5, 1, 4), 0, 0);
        let vertical = make_triangle(&p(5, 0, 1), &p(5, 4, 1), &p(5, 2, 4), 1, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
        let polygons = vec![horizontal, vertical];

        let direct = intersection_split_candidates(&bounds, &polygons, 0).unwrap();
        let segments = split_intersection_segments(&polygons).unwrap();
        let cached = intersection_split_candidates_from_segments(&bounds, &segments, 0).unwrap();

        assert_eq!(direct, cached);
    }

    #[test]
    fn split_intersection_segments_with_pairwise_cache_matches_direct_query() {
        let horizontal = make_triangle(&p(2, 1, 0), &p(8, 1, 0), &p(5, 1, 4), 0, 0);
        let vertical = make_triangle(&p(5, 0, 1), &p(5, 4, 1), &p(5, 2, 4), 1, 0);
        let polygons = vec![horizontal, vertical];
        let cache = RefCell::new(Vec::<PairwiseIntersectionsCacheEntry>::new());

        let direct = split_intersection_segments(&polygons).unwrap();
        let cached = split_intersection_segments_with_pairwise_cache(&cache, &polygons).unwrap();

        assert_eq!(direct, cached);
    }

    #[test]
    fn cached_ordered_subdivision_splits_reuse_permuted_polygon_families() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
        let polygon_a = make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0);
        let polygon_b = make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0);
        let axis_value_cache = RefCell::new(Vec::new());
        let cache = RefCell::new(Vec::new());
        let partition_cache = RefCell::new(Vec::new());
        let pairwise_cache = RefCell::new(Vec::new());

        let first = cached_ordered_subdivision_splits_with(
            &axis_value_cache,
            &cache,
            &partition_cache,
            &RefCell::new(Vec::new()),
            &pairwise_cache,
            &bounds,
            &[polygon_a.clone(), polygon_b.clone()],
        )
        .unwrap();
        let second = cached_ordered_subdivision_splits_with(
            &axis_value_cache,
            &cache,
            &partition_cache,
            &RefCell::new(Vec::new()),
            &pairwise_cache,
            &bounds,
            &[polygon_b, polygon_a],
        )
        .unwrap();

        assert_eq!(
            first
                .iter()
                .map(|candidate| (candidate.axis, candidate.value.clone()))
                .collect::<Vec<_>>(),
            second
                .iter()
                .map(|candidate| (candidate.axis, candidate.value.clone()))
                .collect::<Vec<_>>()
        );
        assert_eq!(cache.borrow().len(), 1);
    }

    #[test]
    fn cached_ordered_subdivision_splits_cache_distinguishes_bounds_even_when_results_match() {
        let polygon = make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0);
        let axis_value_cache = RefCell::new(Vec::new());
        let cache = RefCell::new(Vec::new());
        let partition_cache = RefCell::new(Vec::new());
        let pairwise_cache = RefCell::new(Vec::new());
        let first_bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
        let second_bounds = Aabb::new(p(0, 0, 0), p(8, 4, 4));

        let first = cached_ordered_subdivision_splits_with(
            &axis_value_cache,
            &cache,
            &partition_cache,
            &RefCell::new(Vec::new()),
            &pairwise_cache,
            &first_bounds,
            std::slice::from_ref(&polygon),
        )
        .unwrap();
        let second = cached_ordered_subdivision_splits_with(
            &axis_value_cache,
            &cache,
            &partition_cache,
            &RefCell::new(Vec::new()),
            &pairwise_cache,
            &second_bounds,
            &[polygon],
        )
        .unwrap();

        assert_eq!(
            first
                .iter()
                .map(|candidate| (candidate.axis, candidate.value.clone()))
                .collect::<Vec<_>>(),
            second
                .iter()
                .map(|candidate| (candidate.axis, candidate.value.clone()))
                .collect::<Vec<_>>()
        );
        assert_eq!(cache.borrow().len(), 2);
    }

    #[test]
    fn cached_ordered_subdivision_splits_populate_partition_cache() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
        let polygons = vec![
            make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0),
            make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0),
        ];
        let axis_value_cache = RefCell::new(Vec::new());
        let split_cache = RefCell::new(Vec::new());
        let partition_cache = RefCell::new(Vec::new());
        let polygon_bounds_cache = RefCell::new(Vec::new());
        let pairwise_cache = RefCell::new(Vec::new());

        let ordered = cached_ordered_subdivision_splits_with(
            &axis_value_cache,
            &split_cache,
            &partition_cache,
            &polygon_bounds_cache,
            &pairwise_cache,
            &bounds,
            &polygons,
        )
        .unwrap();
        let cached_partition_count = partition_cache.borrow().len();

        assert!(!ordered.is_empty());
        assert!(cached_partition_count > 0);

        let axis = ordered[0].axis;
        let value = &ordered[0].value;
        let _partition =
            cached_split_child_partition_with(&partition_cache, &polygons, axis, value).unwrap();

        assert_eq!(partition_cache.borrow().len(), cached_partition_count);
    }

    #[test]
    fn cached_ordered_subdivision_splits_dedupes_equivalent_child_partitions() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygon = make_triangle(&p(1, 1, 1), &p(2, 1, 1), &p(1, 2, 1), 0, 0);
        let polygons = vec![polygon];
        let axis_value_cache = RefCell::new(Vec::new());
        let split_cache = RefCell::new(Vec::new());
        let partition_cache = RefCell::new(Vec::new());
        let polygon_bounds_cache = RefCell::new(Vec::new());
        let pairwise_cache = RefCell::new(Vec::new());

        let raw = ordered_subdivision_splits(&bounds, &polygons).unwrap();
        let deduped = cached_ordered_subdivision_splits_with(
            &axis_value_cache,
            &split_cache,
            &partition_cache,
            &polygon_bounds_cache,
            &pairwise_cache,
            &bounds,
            &polygons,
        )
        .unwrap();

        assert!(deduped.len() < raw.len());
    }

    #[test]
    fn cached_split_child_partition_reuses_permuted_polygon_families() {
        let polygon_a = make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0);
        let polygon_b = make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0);
        let cache = RefCell::new(Vec::new());

        let first = cached_split_child_partition_with(
            &cache,
            &[polygon_a.clone(), polygon_b.clone()],
            0,
            &r(3),
        )
        .unwrap();
        let second =
            cached_split_child_partition_with(&cache, &[polygon_b, polygon_a], 0, &r(3)).unwrap();

        assert_eq!(first, second);
        assert_eq!(cache.borrow().len(), 1);
    }

    #[test]
    fn ordered_subdivision_split_search_backtracks_after_unknown_candidate() {
        let candidates = vec![(0, r(1)), (1, r(2))];
        let mut visited = Vec::new();

        let found = try_ordered_subdivision_splits(&candidates, |axis, value| {
            visited.push((axis, value.clone()));
            if axis == 0 {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok((axis, value.clone()))
            }
        })
        .unwrap();

        assert_eq!(visited, candidates);
        assert_eq!(found, (1, r(2)));
    }

    #[test]
    fn ordered_subdivision_split_search_keeps_strongest_failure() {
        let candidates = vec![(0, r(1)), (1, r(2)), (2, r(3))];

        let err = try_ordered_subdivision_splits(&candidates, |axis, _value| match axis {
            0 => Err::<(usize, Real), crate::error::HypermeshError>(
                crate::error::HypermeshError::UnknownClassification,
            ),
            1 => Err::<(usize, Real), crate::error::HypermeshError>(
                crate::error::HypermeshError::ReferencePropagationFailed,
            ),
            _ => Err::<(usize, Real), crate::error::HypermeshError>(
                crate::error::HypermeshError::SubdivisionDepthLimit {
                    depth: 7,
                    polygon_count: 11,
                },
            ),
        })
        .unwrap_err();

        assert_eq!(
            err,
            crate::error::HypermeshError::SubdivisionDepthLimit {
                depth: 7,
                polygon_count: 11,
            }
        );
    }

    #[test]
    fn cannot_split_zero_extent_bounds() {
        let bounds = Aabb::new(p(0, 0, 0), p(0, 0, 0));

        assert!(!can_split_bounds(&bounds).unwrap());
    }

    #[test]
    fn point_strictly_inside_bounds_rejects_positive_extent_boundary() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        assert!(!point_strictly_inside_bounds(&p(0, 2, 2), &bounds).unwrap());
        assert!(point_strictly_inside_bounds(&p(2, 2, 2), &bounds).unwrap());
    }

    #[test]
    fn point_strictly_inside_bounds_accepts_zero_extent_axis_on_plane() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 0, 4));

        assert!(point_strictly_inside_bounds(&p(2, 0, 2), &bounds).unwrap());
        assert!(!point_strictly_inside_bounds(&p(2, 1, 2), &bounds).unwrap());
    }

    #[test]
    fn projected_reference_targets_preserve_strict_inherited_axes() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let targets = projected_reference_targets(&p(0, 2, 5), &bounds).unwrap();

        assert!(!targets.is_empty());
        for target in &targets {
            assert!(point_strictly_inside_bounds(&target.point, &bounds).unwrap());
            assert_eq!(target.point.y, r(2));
        }
    }

    #[test]
    fn compute_new_reference_uses_projected_target_family() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        let (point, definitions, winding) =
            compute_new_reference(&p(0, 2, 5), &axis_defs(&p(0, 2, 5)), &[0], &bounds, &[])
                .unwrap();

        assert!(point_strictly_inside_bounds(&point, &bounds).unwrap());
        assert_eq!(point.y, r(2));
        assert!(!definitions.is_empty());
        assert_eq!(winding, vec![0]);
    }

    #[test]
    fn compute_new_reference_falls_through_to_support_cell_search() {
        let old_ref = p(0, 5, 5);
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let old_defs = axis_defs(&old_ref);
        let old_wnv = vec![0];
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(5))),
            support_only_polygon(Plane::axis_aligned(1, r(5))),
            support_only_polygon(Plane::axis_aligned(2, r(5))),
        ];

        let projected_targets = projected_reference_targets(&old_ref, &bounds).unwrap();
        let projected_halfspaces = projected_reference_halfspaces(&old_ref, &bounds).unwrap();
        let projected_escape_targets =
            projected_reference_escape_targets(&bounds, &projected_halfspaces, &projected_targets)
                .unwrap();

        let projected = projected_reference_search_or_none(search_projected_reference_families(
            &projected_targets,
            &projected_escape_targets,
            || {
                projected_support_plane_cell_reference(
                    &old_ref,
                    &old_defs,
                    &old_wnv,
                    &bounds,
                    &polygons,
                    projected_halfspaces.clone(),
                )
            },
            |projected_target| {
                trace_reference_target(
                    &old_ref,
                    &old_defs,
                    &old_wnv,
                    &bounds,
                    &polygons,
                    projected_target,
                )
            },
            |projected_target| {
                projection_axis_escape_reference(
                    &old_ref,
                    &old_defs,
                    &old_wnv,
                    &projected_target.point,
                    &bounds,
                    &polygons,
                )
            },
            |projected_target| {
                projection_escape_reference(
                    &old_ref,
                    &old_defs,
                    &old_wnv,
                    &projected_target.point,
                    &bounds,
                    &polygons,
                )
            },
        ))
        .unwrap();

        let support =
            support_plane_cell_reference(&old_ref, &old_defs, &old_wnv, &bounds, &polygons)
                .unwrap();

        let (point, definitions, winding) =
            compute_new_reference(&old_ref, &old_defs, &old_wnv, &bounds, &polygons).unwrap();

        assert_eq!(projected, None);
        let support = support.expect("support-cell fallback should find a witness");
        assert_eq!(point, support.0.point);
        assert_eq!(definitions, support.0.definitions);
        assert_eq!(winding, support.1);
    }

    #[test]
    fn support_cell_reference_fallback_uses_closed_mesh_polygons() {
        let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
        let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
        let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
        let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();

        let polygons = vec![
            axis_face_polygon(&soup.polygons, 0, 5),
            axis_face_polygon(&soup.polygons, 1, 5),
            axis_face_polygon(&soup.polygons, 2, 5),
        ];
        let old_ref = p(0, 5, 5);
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let old_defs = axis_defs(&old_ref);
        let old_wnv = vec![0; soup.num_meshes];

        let projected_targets = projected_reference_targets(&old_ref, &bounds).unwrap();
        let projected_halfspaces = projected_reference_halfspaces(&old_ref, &bounds).unwrap();
        let projected_escape_targets =
            projected_reference_escape_targets(&bounds, &projected_halfspaces, &projected_targets)
                .unwrap();

        let projected = projected_reference_search_or_none(search_projected_reference_families(
            &projected_targets,
            &projected_escape_targets,
            || {
                projected_support_plane_cell_reference(
                    &old_ref,
                    &old_defs,
                    &old_wnv,
                    &bounds,
                    &polygons,
                    projected_halfspaces.clone(),
                )
            },
            |projected_target| {
                trace_reference_target(
                    &old_ref,
                    &old_defs,
                    &old_wnv,
                    &bounds,
                    &polygons,
                    projected_target,
                )
            },
            |projected_target| {
                projection_axis_escape_reference(
                    &old_ref,
                    &old_defs,
                    &old_wnv,
                    &projected_target.point,
                    &bounds,
                    &polygons,
                )
            },
            |projected_target| {
                projection_escape_reference(
                    &old_ref,
                    &old_defs,
                    &old_wnv,
                    &projected_target.point,
                    &bounds,
                    &polygons,
                )
            },
        ))
        .unwrap();

        let support =
            support_plane_cell_reference(&old_ref, &old_defs, &old_wnv, &bounds, &polygons)
                .unwrap();
        let (point, definitions, winding) =
            compute_new_reference(&old_ref, &old_defs, &old_wnv, &bounds, &polygons).unwrap();

        assert_eq!(projected, None);
        let support = support.expect("support-cell fallback should find a witness");
        assert_eq!(point, support.0.point);
        assert_eq!(definitions, support.0.definitions);
        assert_eq!(winding, support.1);
    }

    #[test]
    fn alternate_support_reference_matches_general_boolean_results() {
        let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
        let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
        let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
        let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
        let refs = [x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()];
        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::SymmetricDifference,
        ] {
            let indicator = make_indicator(op, soup.num_meshes);
            let classified = subdivide(
                SubdivisionTask::new(
                    soup.polygons.clone(),
                    Aabb::new(p(0, 0, 0), p(10, 10, 10)),
                    p(0, 5, 5),
                    vec![0; soup.num_meshes],
                ),
                &indicator,
                SubdivisionConfig { max_depth: 4 },
            )
            .unwrap_or_else(|err| panic!("alternate {op:?} failed: {err:?}"));

            let alternate_result = BooleanResult::from_classified(
                PolygonSoup {
                    polygons: Vec::new(),
                    bounds: soup.bounds.clone(),
                    num_meshes: soup.num_meshes,
                },
                classified,
            );
            let alternate_soup = triangulate_and_resolve_certified(&alternate_result)
                .unwrap_or_else(|err| panic!("alternate triangulation {op:?} failed: {err:?}"));

            let general_result = boolean_operation(&refs, op, EmberConfig { max_depth: 4 })
                .unwrap_or_else(|err| panic!("general {op:?} failed: {err:?}"));
            let general_soup = triangulate_and_resolve_certified(&general_result)
                .unwrap_or_else(|err| panic!("general triangulation {op:?} failed: {err:?}"));

            assert_same_shape(&alternate_soup, &general_soup);
        }
    }

    #[test]
    fn projected_support_plane_cell_reference_preserves_inherited_axes() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        let found = support_plane_cell_reference_with_halfspaces(
            &p(0, 2, 5),
            &axis_defs(&p(0, 2, 5)),
            &[0],
            &bounds,
            &[],
            projected_reference_halfspaces(&p(0, 2, 5), &bounds).unwrap(),
        )
        .unwrap()
        .expect("projected support-cell search should find a strict witness");

        assert_eq!(found.1, vec![0]);
        assert_eq!(found.0.point.y, r(2));
        assert!(
            point_strictly_inside_reference_halfspace_cell(
                &found.0.point,
                &bounds,
                &projected_reference_halfspaces(&p(0, 2, 5), &bounds).unwrap(),
            )
            .unwrap()
        );
        assert!(!found.0.definitions.is_empty());
    }

    #[test]
    fn projected_reference_search_tries_projected_support_before_escape() {
        use std::cell::RefCell;

        let projected = ReferenceTarget::axis_defined(p(1, 2, 3));
        let support_target = ReferenceTarget::axis_defined(p(2, 2, 3));
        let calls = RefCell::new(Vec::new());

        let found = search_projected_reference_families(
            std::slice::from_ref(&projected),
            std::slice::from_ref(&projected),
            || {
                calls.borrow_mut().push("projected_support");
                Ok(Some((support_target.clone(), vec![7])))
            },
            |target| {
                calls.borrow_mut().push("direct");
                assert_eq!(target, &projected);
                Ok(None)
            },
            |_target| {
                calls.borrow_mut().push("axis_escape");
                Ok(None)
            },
            |_target| {
                calls.borrow_mut().push("tight_escape");
                Ok(None)
            },
        )
        .unwrap();

        assert_eq!(found, Some((support_target, vec![7])));
        assert_eq!(*calls.borrow(), vec!["direct", "projected_support"]);
    }

    #[test]
    fn projected_reference_search_backtracks_after_uncertified_projected_support() {
        use std::cell::RefCell;

        let projected = ReferenceTarget::axis_defined(p(1, 2, 3));
        let axis_target = ReferenceTarget::axis_defined(p(3, 2, 3));
        let calls = RefCell::new(Vec::new());

        let found = search_projected_reference_families(
            std::slice::from_ref(&projected),
            std::slice::from_ref(&projected),
            || {
                calls.borrow_mut().push("projected_support");
                Err(crate::error::HypermeshError::UnknownClassification)
            },
            |target| {
                calls.borrow_mut().push("direct");
                assert_eq!(target, &projected);
                Ok(None)
            },
            |target| {
                calls.borrow_mut().push("axis_escape");
                assert_eq!(target, &projected);
                Ok(Some((axis_target.clone(), vec![11])))
            },
            |_target| {
                calls.borrow_mut().push("tight_escape");
                Ok(None)
            },
        )
        .unwrap();

        assert_eq!(found, Some((axis_target, vec![11])));
        assert_eq!(
            *calls.borrow(),
            vec!["direct", "projected_support", "axis_escape"]
        );
    }

    #[test]
    fn projected_reference_search_skips_duplicate_escape_direct_trace() {
        use std::cell::RefCell;

        let projected = ReferenceTarget::axis_defined(p(1, 2, 3));
        let calls = RefCell::new(Vec::new());

        let found = search_projected_reference_families(
            std::slice::from_ref(&projected),
            std::slice::from_ref(&projected),
            || {
                calls.borrow_mut().push("projected_support");
                Ok(None)
            },
            |_target| {
                calls.borrow_mut().push("direct");
                Ok(None)
            },
            |_target| {
                calls.borrow_mut().push("axis_escape");
                Ok(None)
            },
            |target| {
                calls.borrow_mut().push("tight_escape");
                Ok(Some((target.clone(), vec![31])))
            },
        )
        .unwrap();

        assert_eq!(found, Some((projected, vec![31])));
        assert_eq!(
            *calls.borrow(),
            vec!["direct", "projected_support", "axis_escape", "tight_escape"]
        );
    }

    #[test]
    fn projected_reference_search_skips_duplicate_escape_direct_trace_for_permuted_definitions() {
        use std::cell::RefCell;

        let point = p(1, 2, 3);
        let definition = axis_defs(&point)[0].clone();
        let permuted = [
            definition[1].clone(),
            definition[2].clone(),
            definition[0].clone(),
        ];
        let projected = ReferenceTarget::with_definitions(point.clone(), vec![definition]);
        let escape_target = ReferenceTarget::with_definitions(point, vec![permuted]);
        let axis_target = ReferenceTarget::axis_defined(p(2, 2, 4));
        let calls = RefCell::new(Vec::new());

        let found = search_projected_reference_families(
            std::slice::from_ref(&projected),
            std::slice::from_ref(&escape_target),
            || {
                calls.borrow_mut().push("projected_support");
                Ok(None)
            },
            |_target| {
                calls.borrow_mut().push("direct");
                Ok(None)
            },
            |target| {
                calls.borrow_mut().push("axis_escape");
                assert!(reference_targets_match_for_trace_cache(
                    target,
                    &escape_target
                ));
                Ok(Some((axis_target.clone(), vec![37])))
            },
            |_target| {
                calls.borrow_mut().push("tight_escape");
                Ok(None)
            },
        )
        .unwrap();

        assert_eq!(found, Some((axis_target, vec![37])));
        assert_eq!(
            *calls.borrow(),
            vec!["direct", "projected_support", "axis_escape"]
        );
    }

    #[test]
    fn projected_reference_search_skips_duplicate_escape_direct_trace_for_fallback_duplicate() {
        use std::cell::RefCell;

        let point = p(1, 2, 3);
        let projected = ReferenceTarget::axis_defined(point.clone());
        let escape_target = ReferenceTarget::axis_defined_fallback(point);
        let axis_target = ReferenceTarget::axis_defined(p(2, 2, 4));
        let calls = RefCell::new(Vec::new());

        let found = search_projected_reference_families(
            std::slice::from_ref(&projected),
            std::slice::from_ref(&escape_target),
            || {
                calls.borrow_mut().push("projected_support");
                Ok(None)
            },
            |_target| {
                calls.borrow_mut().push("direct");
                Ok(None)
            },
            |target| {
                calls.borrow_mut().push("axis_escape");
                assert!(reference_targets_match_for_trace_cache(
                    target,
                    &escape_target
                ));
                Ok(Some((axis_target.clone(), vec![41])))
            },
            |_target| {
                calls.borrow_mut().push("tight_escape");
                Ok(None)
            },
        )
        .unwrap();

        assert_eq!(found, Some((axis_target, vec![41])));
        assert_eq!(
            *calls.borrow(),
            vec!["direct", "projected_support", "axis_escape"]
        );
    }

    #[test]
    fn projected_reference_search_still_tries_projected_support_without_targets() {
        use std::cell::RefCell;

        let support_target = ReferenceTarget::axis_defined(p(2, 2, 3));
        let calls = RefCell::new(Vec::new());

        let found = search_projected_reference_families(
            &[],
            &[],
            || {
                calls.borrow_mut().push("projected_support");
                Ok(Some((support_target.clone(), vec![13])))
            },
            |_target| {
                calls.borrow_mut().push("direct");
                Ok(None)
            },
            |_target| {
                calls.borrow_mut().push("axis_escape");
                Ok(None)
            },
            |_target| {
                calls.borrow_mut().push("tight_escape");
                Ok(None)
            },
        )
        .unwrap();

        assert_eq!(found, Some((support_target, vec![13])));
        assert_eq!(*calls.borrow(), vec!["projected_support"]);
    }

    #[test]
    fn projected_reference_search_uses_escape_targets_without_direct_targets() {
        use std::cell::RefCell;

        let escape_target = ReferenceTarget::axis_defined(p(2, 2, 2));
        let axis_target = ReferenceTarget::axis_defined(p(1, 2, 4));
        let calls = RefCell::new(Vec::new());

        let found = search_projected_reference_families(
            &[],
            std::slice::from_ref(&escape_target),
            || {
                calls.borrow_mut().push("projected_support");
                Ok(None)
            },
            |_target| {
                calls.borrow_mut().push("direct");
                Ok(None)
            },
            |target| {
                calls.borrow_mut().push("axis_escape");
                assert_eq!(target, &escape_target);
                Ok(Some((axis_target.clone(), vec![17])))
            },
            |_target| {
                calls.borrow_mut().push("tight_escape");
                Ok(None)
            },
        )
        .unwrap();

        assert_eq!(found, Some((axis_target, vec![17])));
        assert_eq!(
            *calls.borrow(),
            vec!["projected_support", "direct", "axis_escape"]
        );
    }

    #[test]
    fn projected_reference_search_tries_direct_escape_targets_before_axis_escape() {
        use std::cell::RefCell;

        let escape_target = ReferenceTarget::axis_defined(p(2, 2, 2));
        let calls = RefCell::new(Vec::new());

        let found = search_projected_reference_families(
            &[],
            std::slice::from_ref(&escape_target),
            || {
                calls.borrow_mut().push("projected_support");
                Ok(None)
            },
            |target| {
                calls.borrow_mut().push("direct");
                assert_eq!(target, &escape_target);
                Ok(Some(vec![23]))
            },
            |_target| {
                calls.borrow_mut().push("axis_escape");
                Ok(None)
            },
            |_target| {
                calls.borrow_mut().push("tight_escape");
                Ok(None)
            },
        )
        .unwrap();

        assert_eq!(found, Some((escape_target, vec![23])));
        assert_eq!(*calls.borrow(), vec!["projected_support", "direct"]);
    }

    #[test]
    fn projected_reference_search_reports_unknown_if_all_families_are_uncertified() {
        let projected = ReferenceTarget::axis_defined(p(1, 2, 3));
        let err = search_projected_reference_families(
            std::slice::from_ref(&projected),
            std::slice::from_ref(&projected),
            || Err(crate::error::HypermeshError::UnknownClassification),
            |_target| Err(crate::error::HypermeshError::UnknownClassification),
            |_target| Err(crate::error::HypermeshError::UnknownClassification),
            |_target| Err(crate::error::HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projected_reference_search_reports_unknown_when_fallback_target_cannot_trace() {
        let projected = ReferenceTarget::axis_defined_fallback(p(1, 2, 3));
        let err = search_projected_reference_families(
            std::slice::from_ref(&projected),
            &[],
            || Ok(None),
            |_target| Ok(None),
            |_target| Ok(None),
            |_target| Ok(None),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projected_reference_search_skips_fallback_projected_target_even_when_trace_succeeds() {
        let fallback = ReferenceTarget::axis_defined_fallback(p(1, 2, 3));
        let certified = ReferenceTarget::axis_defined(p(2, 2, 3));

        let found = search_projected_reference_families(
            &[fallback.clone(), certified.clone()],
            &[],
            || Ok(None),
            |target| {
                if target == &fallback {
                    Ok(Some(vec![41]))
                } else {
                    Ok(Some(vec![43]))
                }
            },
            |_target| Ok(None),
            |_target| Ok(None),
        )
        .unwrap();

        assert_eq!(found, Some((certified, vec![43])));
    }

    #[test]
    fn projected_reference_search_reports_unknown_when_only_fallback_projected_target_traces() {
        let fallback = ReferenceTarget::axis_defined_fallback(p(1, 2, 3));

        let err = search_projected_reference_families(
            std::slice::from_ref(&fallback),
            &[],
            || Ok(None),
            |_target| Ok(Some(vec![41])),
            |_target| Ok(None),
            |_target| Ok(None),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projected_reference_search_skips_fallback_projected_support_success() {
        let escape_target = ReferenceTarget::axis_defined(p(4, 2, 3));
        let found = search_projected_reference_families(
            &[],
            std::slice::from_ref(&escape_target),
            || {
                Ok(Some((
                    ReferenceTarget::axis_defined_fallback(p(1, 2, 3)),
                    vec![41],
                )))
            },
            |_target| Ok(None),
            |_target| Ok(None),
            |_target| Ok(Some((ReferenceTarget::axis_defined(p(5, 2, 3)), vec![43]))),
        )
        .unwrap();

        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(5, 2, 3)), vec![43]))
        );
    }

    #[test]
    fn projected_reference_search_reports_unknown_when_only_fallback_projected_support_succeeds() {
        let err = search_projected_reference_families(
            &[],
            &[],
            || {
                Ok(Some((
                    ReferenceTarget::axis_defined_fallback(p(1, 2, 3)),
                    vec![41],
                )))
            },
            |_target| Ok(None),
            |_target| Ok(None),
            |_target| Ok(None),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projected_reference_search_reports_unknown_when_fallback_escape_target_has_no_escape_path() {
        let escape_target = ReferenceTarget::axis_defined_fallback(p(1, 2, 3));
        let err = search_projected_reference_families(
            &[],
            std::slice::from_ref(&escape_target),
            || Ok(None),
            |_target| Ok(None),
            |_target| Ok(None),
            |_target| Ok(None),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projected_reference_search_skips_fallback_axis_escape_success() {
        let escape_target = ReferenceTarget::axis_defined(p(1, 2, 3));
        let found = search_projected_reference_families(
            &[],
            std::slice::from_ref(&escape_target),
            || Ok(None),
            |_target| Ok(None),
            |_target| {
                Ok(Some((
                    ReferenceTarget::axis_defined_fallback(p(2, 2, 3)),
                    vec![41],
                )))
            },
            |_target| Ok(Some((ReferenceTarget::axis_defined(p(3, 2, 3)), vec![43]))),
        )
        .unwrap();

        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(3, 2, 3)), vec![43]))
        );
    }

    #[test]
    fn projected_reference_search_reports_unknown_when_only_fallback_axis_escape_succeeds() {
        let escape_target = ReferenceTarget::axis_defined(p(1, 2, 3));
        let err = search_projected_reference_families(
            &[],
            std::slice::from_ref(&escape_target),
            || Ok(None),
            |_target| Ok(None),
            |_target| {
                Ok(Some((
                    ReferenceTarget::axis_defined_fallback(p(2, 2, 3)),
                    vec![41],
                )))
            },
            |_target| Ok(None),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projected_reference_search_accepts_later_tight_escape_after_fallback_escape_axis_failure() {
        let escape_target = ReferenceTarget::axis_defined_fallback(p(1, 2, 3));
        let found = search_projected_reference_families(
            &[],
            std::slice::from_ref(&escape_target),
            || Ok(None),
            |_target| Ok(None),
            |_target| Ok(None),
            |_target| Ok(Some((ReferenceTarget::axis_defined(p(2, 2, 3)), vec![41]))),
        )
        .unwrap();

        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(2, 2, 3)), vec![41]))
        );
    }

    #[test]
    fn projected_reference_search_reports_unknown_when_only_fallback_tight_escape_succeeds() {
        let escape_target = ReferenceTarget::axis_defined(p(1, 2, 3));
        let err = search_projected_reference_families(
            &[],
            std::slice::from_ref(&escape_target),
            || Ok(None),
            |_target| Ok(None),
            |_target| Ok(None),
            |_target| {
                Ok(Some((
                    ReferenceTarget::axis_defined_fallback(p(2, 2, 3)),
                    vec![41],
                )))
            },
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projected_reference_search_skips_fallback_tight_escape_success_for_later_certified_escape() {
        let first_escape = ReferenceTarget::axis_defined(p(1, 2, 3));
        let second_escape = ReferenceTarget::axis_defined(p(4, 2, 3));
        let found = search_projected_reference_families(
            &[],
            &[first_escape.clone(), second_escape.clone()],
            || Ok(None),
            |_target| Ok(None),
            |_target| Ok(None),
            |target| {
                if target == &first_escape {
                    Ok(Some((
                        ReferenceTarget::axis_defined_fallback(p(2, 2, 3)),
                        vec![41],
                    )))
                } else {
                    Ok(Some((ReferenceTarget::axis_defined(p(5, 2, 3)), vec![43])))
                }
            },
        )
        .unwrap();

        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(5, 2, 3)), vec![43]))
        );
    }

    #[test]
    fn projected_reference_search_or_none_skips_uncertified_local_search() {
        let target = ReferenceTarget::axis_defined(p(1, 2, 3));
        assert_eq!(
            projected_reference_search_or_none(Err(
                crate::error::HypermeshError::UnknownClassification
            ))
            .unwrap(),
            None
        );
        assert_eq!(
            projected_reference_search_or_none(Ok(Some((target.clone(), vec![29])))).unwrap(),
            Some((target, vec![29]))
        );
        assert_eq!(
            projected_reference_search_or_none(Err(
                crate::error::HypermeshError::ReferencePropagationFailed
            )),
            Err(crate::error::HypermeshError::ReferencePropagationFailed)
        );
    }

    #[test]
    fn projected_reference_search_or_none_tracking_sets_unknown_flag() {
        let target = ReferenceTarget::axis_defined(p(1, 2, 3));
        let mut saw_unknown = false;

        assert_eq!(
            projected_reference_search_or_none_tracking_unknown(
                Err(crate::error::HypermeshError::UnknownClassification),
                &mut saw_unknown,
            )
            .unwrap(),
            None
        );
        assert!(saw_unknown);

        saw_unknown = false;
        assert_eq!(
            projected_reference_search_or_none_tracking_unknown(
                Ok(Some((target.clone(), vec![29]))),
                &mut saw_unknown,
            )
            .unwrap(),
            Some((target, vec![29]))
        );
        assert!(!saw_unknown);
    }

    #[test]
    fn projected_reference_escape_targets_use_certified_projected_cell_family() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = projected_reference_halfspaces(&p(-2, 2, 7), &bounds).unwrap();

        let targets = projected_reference_escape_targets(&bounds, &halfspaces, &[]).unwrap();

        assert!(targets.len() > 1);
        assert!(targets.iter().any(|target| target.point == p(2, 2, 2)));
        assert!(
            targets
                .iter()
                .find(|target| target.point == p(2, 2, 2))
                .is_some_and(|target| target.definitions != axis_defs(&target.point))
        );
        for target in &targets {
            assert_eq!(axis_ref(&target.point, 1), &r(2));
            assert!(point_satisfies_halfspaces(&target.point, &halfspaces).unwrap());
        }
    }

    #[test]
    fn projected_reference_escape_targets_extend_direct_projected_targets() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = projected_reference_halfspaces(&p(-2, 2, 7), &bounds).unwrap();
        let direct = ReferenceTarget::axis_defined(p(2, 2, 2));

        let targets =
            projected_reference_escape_targets(&bounds, &halfspaces, std::slice::from_ref(&direct))
                .unwrap();

        assert!(targets.iter().any(|target| target.point == direct.point));
        assert!(targets.len() > 1);
    }

    #[test]
    fn projected_reference_escape_targets_include_direct_strict_seed_targets() {
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

        let targets =
            projected_reference_escape_targets_from_report(&bounds, &halfspaces, &[], &report)
                .unwrap();

        assert!(
            targets
                .iter()
                .any(|target| target.point == Point3::new(r(1), r(2), r(3)))
        );
        assert!(
            targets
                .iter()
                .find(|target| target.point == Point3::new(r(1), r(2), r(3)))
                .is_some_and(|target| !target.definitions.is_empty())
        );
    }

    #[test]
    fn reference_target_collection_backtracks_after_uncertified_candidate() {
        let mut targets = Vec::new();

        extend_reference_targets_backtracking_unknown(&mut targets, [0, 1], |candidate| {
            if candidate == 0 {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(vec![ReferenceTarget::axis_defined(p(1, 2, 3))])
            }
        })
        .unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, p(1, 2, 3));
        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn reference_target_collection_marks_later_targets_uncertain_after_uncertain_candidate_result()
    {
        let mut targets = Vec::new();

        extend_reference_targets_backtracking_unknown(&mut targets, [0, 1], |candidate| {
            if candidate == 0 {
                Ok(vec![ReferenceTarget {
                    point: p(1, 2, 3),
                    definitions: vec![axis_plane_definition(&p(1, 2, 3))],
                    uncertified_definition_fallback: true,
                }])
            } else {
                Ok(vec![ReferenceTarget::axis_defined(p(2, 3, 4))])
            }
        })
        .unwrap();

        assert_eq!(targets.len(), 2);
        assert!(
            targets
                .iter()
                .all(|target| target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn reference_target_collection_reports_unknown_if_all_candidates_are_uncertified() {
        let mut targets = Vec::new();

        let err =
            extend_reference_targets_backtracking_unknown(&mut targets, [0, 1], |_candidate| {
                Err(crate::error::HypermeshError::UnknownClassification)
            })
            .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn reference_target_collection_keeps_certified_duplicate_state_certified() {
        let mut targets = Vec::new();
        let point = p(1, 2, 3);
        let definition = axis_plane_definition(&point);

        extend_reference_targets_backtracking_unknown(&mut targets, [0, 1], |candidate| {
            if candidate == 0 {
                Ok(vec![ReferenceTarget {
                    point: point.clone(),
                    definitions: vec![definition.clone()],
                    uncertified_definition_fallback: true,
                }])
            } else {
                Ok(vec![ReferenceTarget::with_definitions(
                    point.clone(),
                    vec![definition.clone()],
                )])
            }
        })
        .unwrap();

        assert_eq!(targets.len(), 1);
        assert!(!targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn reference_target_family_search_backtracks_after_uncertified_earlier_family() {
        let mut targets = Vec::new();

        extend_reference_target_families_backtracking_unknown(
            &mut targets,
            [
                Err(crate::error::HypermeshError::UnknownClassification),
                Ok(vec![ReferenceTarget::axis_defined(p(1, 2, 3))]),
            ],
        )
        .unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, p(1, 2, 3));
        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn reference_target_family_search_tracks_unknown_after_later_certified_family() {
        let mut targets = Vec::new();

        let saw_unknown = extend_reference_target_families_collect_unknown(
            &mut targets,
            [
                Err(crate::error::HypermeshError::UnknownClassification),
                Ok(vec![ReferenceTarget::axis_defined(p(1, 2, 3))]),
            ],
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(targets, vec![ReferenceTarget::axis_defined(p(1, 2, 3))]);
    }

    #[test]
    fn reference_target_family_search_tracks_unknown_after_uncertain_family_result() {
        let mut targets = Vec::new();

        let saw_unknown = extend_reference_target_families_collect_unknown(
            &mut targets,
            [
                Ok(vec![ReferenceTarget {
                    point: p(1, 2, 3),
                    definitions: vec![axis_plane_definition(&p(1, 2, 3))],
                    uncertified_definition_fallback: true,
                }]),
                Ok(vec![ReferenceTarget::axis_defined(p(2, 3, 4))]),
            ],
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(targets.len(), 2);
        assert!(targets[0].uncertified_definition_fallback);
        assert!(!targets[1].uncertified_definition_fallback);
    }

    #[test]
    fn reference_target_family_search_ignores_redundant_fallback_duplicate() {
        let mut targets = Vec::new();
        let point = p(1, 2, 3);
        let definition = axis_plane_definition(&point);

        let saw_unknown = extend_reference_target_families_collect_unknown(
            &mut targets,
            [
                Ok(vec![ReferenceTarget {
                    point: point.clone(),
                    definitions: vec![definition.clone()],
                    uncertified_definition_fallback: true,
                }]),
                Ok(vec![ReferenceTarget::with_definitions(
                    point.clone(),
                    vec![definition.clone()],
                )]),
            ],
        )
        .unwrap();

        assert!(!saw_unknown);
        assert_eq!(targets.len(), 1);
        assert!(!targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn reference_target_family_search_marks_later_targets_uncertain_after_uncertain_family_result()
    {
        let mut targets = Vec::new();

        extend_reference_target_families_backtracking_unknown(
            &mut targets,
            [
                Ok(vec![ReferenceTarget {
                    point: p(1, 2, 3),
                    definitions: vec![axis_plane_definition(&p(1, 2, 3))],
                    uncertified_definition_fallback: true,
                }]),
                Ok(vec![ReferenceTarget::axis_defined(p(2, 3, 4))]),
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
    fn reference_target_family_search_reports_unknown_if_all_families_are_uncertified() {
        let mut targets = Vec::new();

        let err = extend_reference_target_families_backtracking_unknown(
            &mut targets,
            [
                Err(crate::error::HypermeshError::UnknownClassification),
                Err(crate::error::HypermeshError::UnknownClassification),
            ],
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn reference_target_family_from_witness_reports_unknown_for_uncertified_witness() {
        let err = reference_target_family_from_witness(
            Some(&p(1, 2, 3)),
            |_candidate| Err(crate::error::HypermeshError::UnknownClassification),
            |_candidate| Ok(Some(ReferenceTarget::axis_defined(p(9, 9, 9)))),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn reference_target_family_from_witness_returns_direct_target_when_certified() {
        let targets = reference_target_family_from_witness(
            Some(&p(1, 2, 3)),
            |_candidate| Ok(true),
            |candidate| Ok(Some(ReferenceTarget::axis_defined(candidate.clone()))),
        )
        .unwrap();

        assert_eq!(targets, vec![ReferenceTarget::axis_defined(p(1, 2, 3))]);
    }

    #[test]
    fn reference_target_family_from_witness_reports_unknown_for_boundary_reference_witness() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let err = reference_target_family_from_witness(
            Some(&p(0, 2, 2)),
            |candidate| {
                point_strictly_inside_reference_halfspace_cell_or_unknown(
                    candidate,
                    &bounds,
                    &halfspaces,
                )
            },
            |candidate| Ok(Some(ReferenceTarget::axis_defined(candidate.clone()))),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn strict_projected_target_family_tracking_preserves_empty_unknown_result() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut saw_unknown = false;

        let targets = strict_projected_cell_targets_from_seed_families_with_tracking_unknown(
            &bounds,
            &halfspaces,
            None,
            Vec::new(),
            vec![p(1, 1, 1)],
            Vec::new(),
            &mut saw_unknown,
            |_seed| Err(crate::error::HypermeshError::UnknownClassification),
        )
        .unwrap();

        assert!(targets.is_empty());
        assert!(saw_unknown);
    }

    #[test]
    fn projected_escape_target_family_tracking_preserves_unknown_with_existing_targets() {
        let projected_targets = vec![ReferenceTarget::axis_defined(p(0, 0, 0))];
        let mut saw_unknown = false;

        let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
            &[],
            &projected_targets,
            None,
            Vec::new(),
            vec![p(1, 1, 1)],
            Vec::new(),
            &mut saw_unknown,
            |_seed| Err(crate::error::HypermeshError::UnknownClassification),
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(targets.len(), projected_targets.len());
        assert_eq!(targets[0].point, projected_targets[0].point);
        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn projected_escape_target_family_tracking_marks_surviving_targets_uncertain_after_boundary_report_witness()
     {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(p(0, 2, 2), [None, None, None]);
        let mut saw_unknown = false;

        let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
            &halfspaces,
            &[],
            Some(&report),
            vec![p(1, 1, 1)],
            Vec::new(),
            Vec::new(),
            &mut saw_unknown,
            |seed| Ok(vec![ReferenceTarget::axis_defined(seed.clone())]),
        )
        .unwrap();

        assert!(saw_unknown);
        assert!(targets.iter().any(|target| target.point == p(1, 1, 1)));
        assert!(
            targets
                .iter()
                .all(|target| target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn projected_escape_target_family_tracking_marks_surviving_targets_uncertain_after_fallback_family()
     {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut saw_unknown = false;

        let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
            &[],
            &[],
            None,
            Vec::new(),
            vec![first.clone(), second.clone()],
            Vec::new(),
            &mut saw_unknown,
            |seed| {
                Ok(vec![if *seed == first {
                    ReferenceTarget::axis_defined_fallback(seed.clone())
                } else {
                    ReferenceTarget::axis_defined(seed.clone())
                }])
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(targets.len(), 2);
        assert!(targets.iter().any(|target| target.point == first));
        assert!(targets.iter().any(|target| target.point == second));
        assert!(
            targets
                .iter()
                .all(|target| target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn projected_escape_target_family_tracking_ignores_redundant_fallback_duplicate() {
        let point = p(1, 2, 3);
        let mut saw_unknown = false;

        let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
            &[],
            &[ReferenceTarget::axis_defined(point.clone())],
            None,
            Vec::new(),
            vec![point.clone()],
            Vec::new(),
            &mut saw_unknown,
            |seed| Ok(vec![ReferenceTarget::axis_defined_fallback(seed.clone())]),
        )
        .unwrap();

        assert!(!saw_unknown);
        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, point);
        assert!(!targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn projected_escape_target_family_tries_shifted_search_from_report_witness_seed() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 2, 3), [None, None, None]);
        let visited = std::cell::RefCell::new(Vec::new());
        let mut saw_unknown = false;

        let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
            &halfspaces,
            &[],
            Some(&report),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            &mut saw_unknown,
            |seed| {
                visited.borrow_mut().push(seed.clone());
                Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
            },
        )
        .unwrap();

        assert_eq!(visited.into_inner(), vec![p(1, 2, 3)]);
        assert!(targets.iter().any(|target| target.point == p(9, 9, 9)));
        assert!(!saw_unknown);
    }

    #[test]
    fn strict_projected_target_family_tracking_marks_surviving_targets_uncertain_after_unknown() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut saw_unknown = false;

        let targets = strict_projected_cell_targets_from_seed_families_with_tracking_unknown(
            &bounds,
            &halfspaces,
            None,
            vec![first.clone(), second.clone()],
            Vec::new(),
            Vec::new(),
            &mut saw_unknown,
            |seed| {
                if *seed == second {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert!(!targets.is_empty());
        assert!(targets.iter().any(|target| target.point == first));
        assert!(
            targets
                .iter()
                .all(|target| target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn strict_projected_target_family_marks_surviving_targets_uncertain_after_unknown() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);

        let targets = strict_projected_cell_targets_from_seed_families_with(
            &bounds,
            &halfspaces,
            None,
            vec![first.clone(), second.clone()],
            Vec::new(),
            Vec::new(),
            |seed| {
                if *seed == second {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

        assert!(!targets.is_empty());
        assert!(targets.iter().any(|target| target.point == first));
        assert!(
            targets
                .iter()
                .all(|target| target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn shifted_projected_target_family_marks_surviving_targets_uncertain_after_boundary_report_witness()
     {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let families = ShiftedProjectedCellFamilies {
            shifted: halfspaces.clone(),
            report: Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                p(0, 2, 2),
                [None, None, None],
            )),
            saw_unknown: false,
            strict_seeds: vec![p(1, 1, 1)],
            shifted_vertices: Vec::new(),
            shifted_geometry_seeds: Vec::new(),
        };

        let targets =
            shifted_projected_cell_targets_from_families(&bounds, &halfspaces, &families).unwrap();

        assert!(!targets.is_empty());
        assert!(
            targets
                .iter()
                .all(|target| target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn shifted_projected_target_family_prefers_certified_report_witness_duplicate() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let witness = p(1, 2, 3);
        let families = ShiftedProjectedCellFamilies {
            shifted: halfspaces.clone(),
            report: Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                witness.clone(),
                [Some(9), None, None],
            )),
            saw_unknown: false,
            strict_seeds: Vec::new(),
            shifted_vertices: Vec::new(),
            shifted_geometry_seeds: Vec::new(),
        };

        let targets =
            shifted_projected_cell_targets_from_families(&bounds, &halfspaces, &families).unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, witness);
        assert!(!targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn shifted_projected_target_family_marks_surviving_targets_uncertain_after_unknown() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let families = ShiftedProjectedCellFamilies {
            shifted: halfspaces.clone(),
            report: None,
            saw_unknown: true,
            strict_seeds: vec![p(1, 1, 1)],
            shifted_vertices: Vec::new(),
            shifted_geometry_seeds: Vec::new(),
        };

        let targets =
            shifted_projected_cell_targets_from_families(&bounds, &halfspaces, &families).unwrap();

        assert!(!targets.is_empty());
        assert!(
            targets
                .iter()
                .all(|target| target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn projected_escape_family_marks_surviving_targets_uncertain_after_unknown() {
        let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
        let families = ShiftedProjectedCellFamilies {
            shifted: halfspaces.clone(),
            report: None,
            saw_unknown: true,
            strict_seeds: vec![p(1, 1, 1)],
            shifted_vertices: Vec::new(),
            shifted_geometry_seeds: Vec::new(),
        };

        let targets = projected_escape_targets_from_families(&halfspaces, &families).unwrap();

        assert!(!targets.is_empty());
        assert!(
            targets
                .iter()
                .all(|target| target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn projected_escape_family_prefers_certified_report_witness_duplicate() {
        let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
        let witness = p(1, 2, 3);
        let families = ShiftedProjectedCellFamilies {
            shifted: halfspaces.clone(),
            report: Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                witness.clone(),
                [Some(9), None, None],
            )),
            saw_unknown: false,
            strict_seeds: Vec::new(),
            shifted_vertices: Vec::new(),
            shifted_geometry_seeds: Vec::new(),
        };

        let targets = projected_escape_targets_from_families(&halfspaces, &families).unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, witness);
        assert!(!targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn projected_escape_family_marks_surviving_targets_uncertain_after_boundary_report_witness() {
        let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
        let families = ShiftedProjectedCellFamilies {
            shifted: halfspaces.clone(),
            report: Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                p(0, 2, 2),
                [None, None, None],
            )),
            saw_unknown: false,
            strict_seeds: vec![p(1, 1, 1)],
            shifted_vertices: Vec::new(),
            shifted_geometry_seeds: Vec::new(),
        };

        let targets = projected_escape_targets_from_families(&halfspaces, &families).unwrap();

        assert!(!targets.is_empty());
        assert!(
            targets
                .iter()
                .all(|target| target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn deferred_projected_escape_direct_targets_backtrack_after_uncertified_seed() {
        let strict_seeds = vec![p(1, 2, 3), p(1, 2, 4)];
        let halfspaces = vec![
            axis_halfspace(0, true, r(1)),
            axis_halfspace(0, false, r(1)),
            axis_halfspace(1, true, r(2)),
            axis_halfspace(1, false, r(2)),
            axis_halfspace(2, true, r(4)),
            axis_halfspace(2, false, r(4)),
        ];

        let targets = deferred_projected_escape_direct_targets_with_contains(
            &strict_seeds,
            None,
            &halfspaces,
            |seed, _halfspaces| {
                if seed == &p(1, 2, 3) {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(true)
                }
            },
        )
        .unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, p(1, 2, 4));
        assert!(!targets[0].definitions.is_empty());
    }

    #[test]
    fn deferred_projected_escape_direct_targets_mark_later_target_uncertain_after_boundary_seed() {
        let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();

        let targets =
            deferred_projected_escape_direct_targets(&[p(0, 2, 2), p(1, 2, 2)], None, &halfspaces)
                .unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, p(1, 2, 2));
        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn deferred_projected_escape_direct_targets_report_unknown_for_boundary_seed() {
        let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();

        let err =
            deferred_projected_escape_direct_targets(&[p(0, 2, 2)], None, &halfspaces).unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn deferred_projected_escape_direct_targets_report_unknown_if_all_seeds_are_uncertified() {
        let strict_seeds = vec![p(1, 2, 3), p(1, 2, 4)];
        let halfspaces = vec![
            axis_halfspace(0, true, r(1)),
            axis_halfspace(0, false, r(1)),
            axis_halfspace(1, true, r(2)),
            axis_halfspace(1, false, r(2)),
            axis_halfspace(2, true, r(4)),
            axis_halfspace(2, false, r(4)),
        ];

        let err = deferred_projected_escape_direct_targets_with_contains(
            &strict_seeds,
            None,
            &halfspaces,
            |_seed, _halfspaces| Err(crate::error::HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn deferred_direct_reference_targets_backtrack_after_uncertified_seed() {
        let first = p(1, 2, 3);
        let second = p(1, 2, 4);
        let mut saw_unknown = false;

        let targets = deferred_direct_reference_targets_from_strict_seeds_with(
            &[first.clone(), second.clone()],
            None,
            &mut saw_unknown,
            |seed| {
                if *seed == first {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(ReferenceTarget::axis_defined(seed.clone())))
                }
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, second);
        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn deferred_direct_reference_targets_track_unknown_if_all_seeds_are_uncertified() {
        let first = p(1, 2, 3);
        let second = p(1, 2, 4);
        let mut saw_unknown = false;

        let targets = deferred_direct_reference_targets_from_strict_seeds_with(
            &[first, second],
            None,
            &mut saw_unknown,
            |_seed| Err(crate::error::HypermeshError::UnknownClassification),
        )
        .unwrap();

        assert!(targets.is_empty());
        assert!(saw_unknown);
    }

    #[test]
    fn deferred_direct_reference_targets_do_not_mark_unknown_for_fallback_results() {
        let mut saw_unknown = false;

        let targets = deferred_direct_reference_targets_from_strict_seeds_with(
            &[p(1, 2, 3)],
            None,
            &mut saw_unknown,
            |seed| Ok(Some(ReferenceTarget::axis_defined_fallback(seed.clone()))),
        )
        .unwrap();

        assert!(!saw_unknown);
        assert_eq!(targets.len(), 1);
        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn strict_projected_target_family_tries_shifted_search_from_report_witness_seed() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let witness = p(1, 2, 3);
        let visited = std::cell::RefCell::new(Vec::new());
        let mut saw_unknown = false;

        let targets = strict_projected_cell_targets_from_seed_families_with_tracking_unknown(
            &bounds,
            &halfspaces,
            Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
                witness.clone(),
                [None, None, None],
            )),
            Vec::new(),
            vec![witness.clone()],
            Vec::new(),
            &mut saw_unknown,
            |seed| {
                visited.borrow_mut().push(seed.clone());
                Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
            },
        )
        .unwrap();

        assert_eq!(visited.into_inner(), vec![witness]);
        assert!(targets.iter().any(|target| target.point == p(9, 9, 9)));
    }

    #[test]
    fn strict_support_target_family_tries_shifted_search_from_report_witness_seed() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let witness = p(1, 2, 3);
        let visited = std::cell::RefCell::new(Vec::new());
        let mut saw_unknown = false;

        let targets = strict_support_cell_targets_from_seed_families_with_tracking_unknown(
            &bounds,
            &halfspaces,
            Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
                witness.clone(),
                [None, None, None],
            )),
            Vec::new(),
            vec![witness.clone()],
            Vec::new(),
            &mut saw_unknown,
            |seed| {
                visited.borrow_mut().push(seed.clone());
                Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
            },
        )
        .unwrap();

        assert_eq!(visited.into_inner(), vec![witness]);
        assert!(targets.iter().any(|target| target.point == p(9, 9, 9)));
    }

    #[test]
    fn point3_family_search_backtracks_after_uncertified_earlier_family() {
        let mut points = Vec::new();

        extend_point3_families_backtracking_unknown(
            &mut points,
            [
                Err(crate::error::HypermeshError::UnknownClassification),
                Ok(Point3FamilyState {
                    points: vec![p(1, 2, 3)],
                    saw_unknown: false,
                }),
            ],
        )
        .unwrap();

        assert_eq!(points, vec![p(1, 2, 3)]);
    }

    #[test]
    fn point3_family_search_tracks_unknown_after_uncertain_family_result() {
        let mut points = Vec::new();

        let saw_unknown = extend_point3_families_collect_unknown(
            &mut points,
            [
                Ok(Point3FamilyState {
                    points: vec![p(1, 2, 3)],
                    saw_unknown: true,
                }),
                Ok(Point3FamilyState {
                    points: vec![p(2, 3, 4)],
                    saw_unknown: false,
                }),
            ],
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(points, vec![p(1, 2, 3), p(2, 3, 4)]);
    }

    #[test]
    fn point3_family_search_reports_unknown_if_all_families_are_uncertified() {
        let mut points = Vec::new();

        let err = extend_point3_families_backtracking_unknown(
            &mut points,
            [
                Err(crate::error::HypermeshError::UnknownClassification),
                Err(crate::error::HypermeshError::UnknownClassification),
            ],
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn collect_point3_family_tracks_unknown_after_later_strict_point() {
        let family = collect_point3_family(Ok(vec![p(1, 2, 3), p(2, 3, 4)]), |candidate| {
            if *candidate == p(1, 2, 3) {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(true)
            }
        })
        .unwrap();

        assert_eq!(family.points, vec![p(2, 3, 4)]);
        assert!(family.saw_unknown);
    }

    #[test]
    fn collect_point3_family_tracks_unknown_after_reference_boundary_candidate() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let family = collect_point3_family(Ok(vec![p(0, 2, 2), p(1, 1, 1)]), |candidate| {
            point_strictly_inside_reference_halfspace_cell_or_unknown(
                candidate,
                &bounds,
                &halfspaces,
            )
        })
        .unwrap();

        assert_eq!(family.points, vec![p(1, 1, 1)]);
        assert!(family.saw_unknown);
    }

    #[test]
    fn reference_target_family_or_empty_skips_uncertified_family() {
        let target = ReferenceTarget::axis_defined(p(1, 2, 3));

        assert_eq!(
            reference_target_family_or_empty(Err(
                crate::error::HypermeshError::UnknownClassification
            ))
            .unwrap(),
            Vec::<ReferenceTarget>::new()
        );
        assert_eq!(
            reference_target_family_or_empty(Ok(vec![target.clone()])).unwrap(),
            vec![target]
        );
        assert_eq!(
            reference_target_family_or_empty(Err(
                crate::error::HypermeshError::ReferencePropagationFailed
            )),
            Err(crate::error::HypermeshError::ReferencePropagationFailed)
        );
    }

    #[test]
    fn reference_target_family_or_empty_tracking_sets_unknown_flag() {
        let target = ReferenceTarget::axis_defined(p(1, 2, 3));
        let mut saw_unknown = false;

        assert_eq!(
            reference_target_family_or_empty_tracking_unknown(
                Err(crate::error::HypermeshError::UnknownClassification),
                &mut saw_unknown,
            )
            .unwrap(),
            Vec::<ReferenceTarget>::new()
        );
        assert!(saw_unknown);

        saw_unknown = false;
        assert_eq!(
            reference_target_family_or_empty_tracking_unknown(
                Ok(vec![target.clone()]),
                &mut saw_unknown
            )
            .unwrap(),
            vec![target]
        );
        assert!(!saw_unknown);
    }

    #[test]
    fn reference_result_or_error_prefers_support_after_uncertified_projected_search() {
        let projected_unknown = true;
        let support_target = ReferenceTarget::axis_defined(p(4, 5, 6));

        let (point, definitions, winding) = reference_result_or_error(
            None,
            Some((support_target.clone(), vec![11])),
            projected_unknown,
        )
        .unwrap();

        assert_eq!(point, support_target.point);
        assert_eq!(definitions, support_target.definitions);
        assert_eq!(winding, vec![11]);
    }

    #[test]
    fn reference_result_or_error_reports_unknown_after_uncertified_projected_search() {
        let err = reference_result_or_error(None, None, true).unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn reference_result_or_error_reports_reference_failure_when_all_families_are_certified_absent()
    {
        let err = reference_result_or_error(None, None, false).unwrap_err();

        assert_eq!(
            err,
            crate::error::HypermeshError::ReferencePropagationFailed
        );
    }

    #[test]
    fn certified_leaf_output_helper_runs_leaf_attempt_once() {
        let task = SubdivisionTask::new(
            vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
            Aabb::new(p(0, 0, 0), p(1, 1, 1)),
            p(0, 0, 0),
            vec![0],
        );
        let indicator = crate::winding::make_indicator(BooleanOp::Union, 1);
        let mut attempts = 0;

        let output = certified_leaf_output_if_complete_with(
            &task,
            &indicator,
            |_task, _indicator, _output| {
                attempts += 1;
                Err(crate::error::HypermeshError::UnknownClassification)
            },
        )
        .unwrap();

        assert_eq!(attempts, 1);
        assert_eq!(output, None);
    }

    #[test]
    fn unsplittable_subdivision_runs_leaf_processor_once() {
        let task = SubdivisionTask::new(
            vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
            Aabb::new(p(0, 0, 0), p(0, 0, 0)),
            p(0, 0, 0),
            vec![0],
        );
        let indicator = crate::winding::make_indicator(BooleanOp::Union, 1);
        let mut attempts = 0;
        let mut output = Vec::new();
        let caches = SubdivisionRuntimeCaches::default();

        let err = subdivide_into_inner_with(
            task,
            &indicator,
            SubdivisionConfig { max_depth: 0 },
            None,
            &mut output,
            &mut |_task, _indicator, _output| {
                attempts += 1;
                Err(crate::error::HypermeshError::UnknownClassification)
            },
            &caches,
            &caches.winding_reachability,
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
        assert_eq!(attempts, 1);
        assert!(output.is_empty());
    }

    #[test]
    fn recursive_child_bounds_contract_unchanged_polygon_family() {
        let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let parent_bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let child_bounds = parent_bounds.left_half(0, r(5));

        let tightened = recursive_child_bounds(
            std::slice::from_ref(&polygon),
            std::slice::from_ref(&polygon),
            &child_bounds,
        )
        .unwrap();

        assert_eq!(tightened, Aabb::new(p(0, 0, 0), p(1, 1, 0)));
    }

    #[test]
    fn recursive_child_bounds_contracts_permuted_unchanged_polygon_family() {
        let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
        let parent_bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let child_bounds = parent_bounds.left_half(0, r(5));

        let tightened = recursive_child_bounds(
            &[polygon_a.clone(), polygon_b.clone()],
            &[polygon_b, polygon_a],
            &child_bounds,
        )
        .unwrap();

        assert_eq!(tightened, Aabb::new(p(0, 0, 0), p(1, 1, 1)));
    }

    #[test]
    fn cached_polygon_family_bounds_reuses_permuted_polygon_families() {
        let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);

        let first = cached_polygon_family_bounds_with(
            &cache,
            &[polygon_a.clone(), polygon_b.clone()],
            |_polygons| {
                calls.set(calls.get() + 1);
                Ok(Aabb::new(p(0, 0, 0), p(1, 1, 1)))
            },
        )
        .unwrap();
        let second =
            cached_polygon_family_bounds_with(&cache, &[polygon_b, polygon_a], |_polygons| {
                calls.set(calls.get() + 1);
                Ok(Aabb::new(p(0, 0, 0), p(9, 9, 9)))
            })
            .unwrap();

        assert_eq!(calls.get(), 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_pairwise_intersections_reuse_identical_polygon_sequence() {
        let horizontal = make_triangle(&p(2, 1, 0), &p(8, 1, 0), &p(5, 1, 4), 0, 0);
        let vertical = make_triangle(&p(5, 0, 1), &p(5, 4, 1), &p(5, 2, 4), 1, 0);
        let polygons = vec![horizontal, vertical];
        let cache = RefCell::new(Vec::new());

        let first = cached_pairwise_intersections_by_polygon_with(&cache, &polygons).unwrap();
        let second = cached_pairwise_intersections_by_polygon_with(&cache, &polygons).unwrap();

        assert_eq!(first, second);
        assert_eq!(cache.borrow().len(), 1);
    }

    #[test]
    fn cached_pairwise_intersections_reuse_permuted_polygon_sequence() {
        let horizontal = make_triangle(&p(2, 1, 0), &p(8, 1, 0), &p(5, 1, 4), 0, 0);
        let vertical = make_triangle(&p(5, 0, 1), &p(5, 4, 1), &p(5, 2, 4), 1, 0);
        let first_polygons = vec![horizontal.clone(), vertical.clone()];
        let second_polygons = vec![vertical, horizontal];
        let cache = RefCell::new(Vec::new());

        let first = cached_pairwise_intersections_by_polygon_with(&cache, &first_polygons).unwrap();
        let second =
            cached_pairwise_intersections_by_polygon_with(&cache, &second_polygons).unwrap();
        let direct = pairwise_intersections_by_polygon(&second_polygons).unwrap();

        assert_eq!(first.len(), 2);
        assert_eq!(second, direct);
        assert_eq!(cache.borrow().len(), 1);
    }

    #[test]
    fn cached_reference_halfspace_containment_reuses_permuted_halfspaces() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let point = p(2, 2, 2);
        let left = vec![
            axis_halfspace(0, false, r(0)),
            axis_halfspace(1, false, r(0)),
        ];
        let right = vec![
            axis_halfspace(1, false, r(0)),
            axis_halfspace(0, false, r(0)),
        ];
        let cache = RefCell::new(Vec::<ReferenceHalfspaceContainmentCacheEntry>::new());
        let calls = std::cell::Cell::new(0);

        let first = cached_reference_halfspace_containment_with(
            &mut cache.borrow_mut(),
            &bounds,
            &point,
            &left,
            |_point, _bounds, _halfspaces| {
                calls.set(calls.get() + 1);
                Ok(true)
            },
        )
        .unwrap();
        let second = cached_reference_halfspace_containment_with(
            &mut cache.borrow_mut(),
            &bounds,
            &point,
            &right,
            |_point, _bounds, _halfspaces| {
                calls.set(calls.get() + 1);
                Ok(false)
            },
        )
        .unwrap();

        assert!(first);
        assert!(second);
        assert_eq!(calls.get(), 1);
    }

    #[test]
    fn cached_pure_halfspace_containment_reuses_permuted_halfspaces() {
        let point = p(2, 2, 2);
        let left = vec![
            axis_halfspace(0, false, r(0)),
            axis_halfspace(1, false, r(0)),
        ];
        let right = vec![
            axis_halfspace(1, false, r(0)),
            axis_halfspace(0, false, r(0)),
        ];
        let cache = RefCell::new(Vec::<ReferencePureHalfspaceContainmentCacheEntry>::new());
        let calls = std::cell::Cell::new(0);

        let first = cached_pure_halfspace_containment_with(
            &mut cache.borrow_mut(),
            &point,
            &left,
            |_point, _halfspaces| {
                calls.set(calls.get() + 1);
                Ok(true)
            },
        )
        .unwrap();
        let second = cached_pure_halfspace_containment_with(
            &mut cache.borrow_mut(),
            &point,
            &right,
            |_point, _halfspaces| {
                calls.set(calls.get() + 1);
                Ok(false)
            },
        )
        .unwrap();

        assert!(first);
        assert!(second);
        assert_eq!(calls.get(), 1);
    }

    #[test]
    fn subdivision_child_partition_dedupe_skips_duplicate_contracted_unchanged_branch() {
        let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let parent_bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let left_x = recursive_child_bounds(
            std::slice::from_ref(&polygon),
            std::slice::from_ref(&polygon),
            &parent_bounds.left_half(0, r(5)),
        )
        .unwrap();
        let left_y = recursive_child_bounds(
            std::slice::from_ref(&polygon),
            std::slice::from_ref(&polygon),
            &parent_bounds.left_half(1, r(5)),
        )
        .unwrap();
        let mut seen = Vec::new();

        assert!(take_new_subdivision_child_partition(
            &mut seen,
            std::slice::from_ref(&polygon),
            Some(&left_x),
            &[],
            None,
        ));
        assert!(!take_new_subdivision_child_partition(
            &mut seen,
            std::slice::from_ref(&polygon),
            Some(&left_y),
            &[],
            None,
        ));
    }

    #[test]
    fn subdivision_child_partition_dedupe_keeps_distinct_nonempty_bounds() {
        let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let mut seen = Vec::new();
        let left_a = Aabb::new(p(0, 0, 0), p(1, 1, 0));
        let left_b = Aabb::new(p(0, 0, 0), p(2, 1, 0));

        assert!(take_new_subdivision_child_partition(
            &mut seen,
            std::slice::from_ref(&polygon),
            Some(&left_a),
            &[],
            None,
        ));
        assert!(take_new_subdivision_child_partition(
            &mut seen,
            std::slice::from_ref(&polygon),
            Some(&left_b),
            &[],
            None,
        ));
    }

    #[test]
    fn cached_child_reference_reuses_identical_child_state() {
        let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 0));
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);
        let old_ref = p(0, 0, 0);
        let old_ref_definitions = axis_defs(&old_ref);
        let old_wnv = vec![0];

        let first = cached_child_reference_with(
            &cache,
            &old_ref,
            &old_ref_definitions,
            &old_wnv,
            std::slice::from_ref(&polygon),
            &bounds,
            || {
                calls.set(calls.get() + 1);
                Ok((p(1, 2, 3), axis_defs(&p(1, 2, 3)), vec![7]))
            },
        )
        .unwrap();
        let second = cached_child_reference_with(
            &cache,
            &old_ref,
            &old_ref_definitions,
            &old_wnv,
            std::slice::from_ref(&polygon),
            &bounds,
            || {
                calls.set(calls.get() + 1);
                Ok((p(9, 9, 9), axis_defs(&p(9, 9, 9)), vec![99]))
            },
        )
        .unwrap();

        assert_eq!(calls.get(), 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_child_reference_reuses_permuted_parent_definition_families() {
        let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 0));
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);
        let old_ref = p(1, 2, 3);
        let definition = axis_defs(&old_ref)[0].clone();
        let permuted = [
            definition[1].clone(),
            definition[2].clone(),
            definition[0].clone(),
        ];
        let old_wnv = vec![0];

        let first = cached_child_reference_with(
            &cache,
            &old_ref,
            std::slice::from_ref(&definition),
            &old_wnv,
            std::slice::from_ref(&polygon),
            &bounds,
            || {
                calls.set(calls.get() + 1);
                Ok((p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![9]))
            },
        )
        .unwrap();
        let second = cached_child_reference_with(
            &cache,
            &old_ref,
            std::slice::from_ref(&permuted),
            &old_wnv,
            std::slice::from_ref(&polygon),
            &bounds,
            || {
                calls.set(calls.get() + 1);
                Ok((p(7, 8, 9), axis_defs(&p(7, 8, 9)), vec![11]))
            },
        )
        .unwrap();

        assert_eq!(calls.get(), 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_child_reference_keeps_distinct_child_bounds_separate() {
        let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let bounds_a = Aabb::new(p(0, 0, 0), p(1, 1, 0));
        let bounds_b = Aabb::new(p(0, 0, 0), p(2, 1, 0));
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);
        let old_ref = p(0, 0, 0);
        let old_ref_definitions = axis_defs(&old_ref);
        let old_wnv = vec![0];

        cached_child_reference_with(
            &cache,
            &old_ref,
            &old_ref_definitions,
            &old_wnv,
            std::slice::from_ref(&polygon),
            &bounds_a,
            || {
                calls.set(calls.get() + 1);
                Ok((p(1, 2, 3), axis_defs(&p(1, 2, 3)), vec![7]))
            },
        )
        .unwrap();
        cached_child_reference_with(
            &cache,
            &old_ref,
            &old_ref_definitions,
            &old_wnv,
            std::slice::from_ref(&polygon),
            &bounds_b,
            || {
                calls.set(calls.get() + 1);
                Ok((p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![8]))
            },
        )
        .unwrap();

        assert_eq!(calls.get(), 2);
    }

    #[test]
    fn reusable_child_reference_if_certified_reuses_parent_reference() {
        let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let task = SubdivisionTask::new(
            vec![polygon.clone()],
            Aabb::new(p(0, 0, 0), p(4, 4, 4)),
            p(1, 1, 1),
            vec![0],
        );
        let child_bounds = Aabb::new(p(0, 0, 0), p(2, 2, 2));
        let mut query_caches = SupportReferenceQueryCaches::default();

        let reused = reusable_child_reference_if_certified(
            &task,
            std::slice::from_ref(&polygon),
            &child_bounds,
            &mut query_caches,
        )
        .unwrap();

        assert_eq!(
            reused,
            Some((
                task.ref_point.clone(),
                task.ref_definitions.clone(),
                task.ref_wnv.clone(),
            ))
        );
        assert_eq!(query_caches.validity_cache.len(), 1);
    }

    #[test]
    fn reusable_child_reference_if_certified_reuses_changed_child_family_when_point_stays_valid() {
        let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let other = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 1, 0);
        let task = SubdivisionTask::new(
            vec![polygon],
            Aabb::new(p(0, 0, 0), p(4, 4, 4)),
            p(1, 1, 1),
            vec![0],
        );
        let child_bounds = Aabb::new(p(0, 0, 0), p(2, 2, 2));
        let mut query_caches = SupportReferenceQueryCaches::default();

        let reused = reusable_child_reference_if_certified(
            &task,
            std::slice::from_ref(&other),
            &child_bounds,
            &mut query_caches,
        )
        .unwrap();

        assert_eq!(
            reused,
            Some((
                task.ref_point.clone(),
                task.ref_definitions.clone(),
                task.ref_wnv.clone(),
            ))
        );
        assert_eq!(query_caches.validity_cache.len(), 1);
    }

    #[test]
    fn reusable_child_reference_if_certified_skips_invalid_point() {
        let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let task = SubdivisionTask::new(
            vec![polygon.clone()],
            Aabb::new(p(0, 0, 0), p(4, 4, 4)),
            p(0, 0, 0),
            vec![0],
        );
        let child_bounds = Aabb::new(p(0, 0, 0), p(2, 2, 2));
        let mut query_caches = SupportReferenceQueryCaches::default();

        let reused = reusable_child_reference_if_certified(
            &task,
            std::slice::from_ref(&polygon),
            &child_bounds,
            &mut query_caches,
        )
        .unwrap();

        assert_eq!(reused, None);
        assert_eq!(query_caches.validity_cache.len(), 1);
    }

    #[test]
    fn reusable_child_reference_from_cached_trace_if_certified_reuses_cached_target() {
        let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let cached_point = p(2, 1, 1);
        let cache = RefCell::new(vec![ChildReferenceCacheEntry {
            source_polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
            source_polygons: vec![polygon.clone()],
            bounds: bounds.clone(),
            old_ref: p(1, 1, 1),
            old_ref_definitions: axis_defs(&p(1, 1, 1)),
            old_wnv: vec![0],
            result: Ok((cached_point.clone(), axis_defs(&cached_point), vec![0])),
        }]);
        let mut query_caches = SupportReferenceQueryCaches::default();

        let reused = reusable_child_reference_from_cached_trace_if_certified(
            &cache,
            &cached_point,
            &axis_defs(&cached_point),
            &[0],
            std::slice::from_ref(&polygon),
            &bounds,
            &mut query_caches,
        )
        .unwrap();

        assert_eq!(
            reused,
            Some((cached_point.clone(), axis_defs(&cached_point), vec![0]))
        );
    }

    #[test]
    fn reusable_child_reference_from_cached_trace_if_certified_skips_invalid_cached_target() {
        let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let cached_point = p(0, 0, 0);
        let query_point = p(1, 1, 1);
        let cache = RefCell::new(vec![ChildReferenceCacheEntry {
            source_polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
            source_polygons: vec![polygon.clone()],
            bounds: bounds.clone(),
            old_ref: p(2, 1, 1),
            old_ref_definitions: axis_defs(&p(2, 1, 1)),
            old_wnv: vec![0],
            result: Ok((cached_point.clone(), axis_defs(&cached_point), vec![0])),
        }]);
        let mut query_caches = SupportReferenceQueryCaches::default();

        let reused = reusable_child_reference_from_cached_trace_if_certified(
            &cache,
            &query_point,
            &axis_defs(&query_point),
            &[0],
            std::slice::from_ref(&polygon),
            &bounds,
            &mut query_caches,
        )
        .unwrap();

        assert_eq!(reused, None);
    }

    #[test]
    fn subdivision_child_partition_dedupe_skips_permuted_polygon_order() {
        let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
        let left_bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));
        let mut seen = Vec::new();

        assert!(take_new_subdivision_child_partition(
            &mut seen,
            &[polygon_a.clone(), polygon_b.clone()],
            Some(&left_bounds),
            &[],
            None,
        ));
        assert!(!take_new_subdivision_child_partition(
            &mut seen,
            &[polygon_b, polygon_a],
            Some(&left_bounds),
            &[],
            None,
        ));
    }

    #[test]
    fn subdivision_child_partition_dedupe_skips_swapped_equivalent_children() {
        let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
        let left_bounds = Aabb::new(p(0, 0, 0), p(1, 1, 0));
        let right_bounds = Aabb::new(p(0, 0, 1), p(1, 1, 1));
        let mut seen = Vec::new();

        assert!(take_new_subdivision_child_partition(
            &mut seen,
            std::slice::from_ref(&polygon_a),
            Some(&left_bounds),
            std::slice::from_ref(&polygon_b),
            Some(&right_bounds),
        ));
        assert!(!take_new_subdivision_child_partition(
            &mut seen,
            std::slice::from_ref(&polygon_b),
            Some(&right_bounds),
            std::slice::from_ref(&polygon_a),
            Some(&left_bounds),
        ));
    }

    #[test]
    fn cached_child_reference_keeps_distinct_parent_reference_states_separate() {
        let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 0));
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);
        let old_ref_a = p(0, 0, 0);
        let old_ref_b = p(9, 9, 9);
        let old_ref_definitions_a = axis_defs(&old_ref_a);
        let old_ref_definitions_b = axis_defs(&old_ref_b);
        let old_wnv_a = vec![0];
        let old_wnv_b = vec![1];

        cached_child_reference_with(
            &cache,
            &old_ref_a,
            &old_ref_definitions_a,
            &old_wnv_a,
            std::slice::from_ref(&polygon),
            &bounds,
            || {
                calls.set(calls.get() + 1);
                Ok((p(1, 2, 3), axis_defs(&p(1, 2, 3)), vec![7]))
            },
        )
        .unwrap();
        cached_child_reference_with(
            &cache,
            &old_ref_b,
            &old_ref_definitions_b,
            &old_wnv_b,
            std::slice::from_ref(&polygon),
            &bounds,
            || {
                calls.set(calls.get() + 1);
                Ok((p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![8]))
            },
        )
        .unwrap();

        assert_eq!(calls.get(), 2);
    }

    #[test]
    fn cached_child_reference_keeps_distinct_source_polygon_families_separate() {
        let source_polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let source_polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 0));
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);
        let old_ref = p(0, 0, 0);
        let old_ref_definitions = axis_defs(&old_ref);
        let old_wnv = vec![0];

        cached_child_reference_with(
            &cache,
            &old_ref,
            &old_ref_definitions,
            &old_wnv,
            std::slice::from_ref(&source_polygon_a),
            &bounds,
            || {
                calls.set(calls.get() + 1);
                Ok((p(1, 2, 3), axis_defs(&p(1, 2, 3)), vec![7]))
            },
        )
        .unwrap();
        cached_child_reference_with(
            &cache,
            &old_ref,
            &old_ref_definitions,
            &old_wnv,
            std::slice::from_ref(&source_polygon_b),
            &bounds,
            || {
                calls.set(calls.get() + 1);
                Ok((p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![8]))
            },
        )
        .unwrap();

        assert_eq!(calls.get(), 2);
    }

    #[test]
    fn cached_child_reference_reuses_permuted_source_polygon_families() {
        let source_polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let source_polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);
        let old_ref = p(0, 0, 0);
        let old_ref_definitions = axis_defs(&old_ref);
        let old_wnv = vec![0];

        let first = cached_child_reference_with(
            &cache,
            &old_ref,
            &old_ref_definitions,
            &old_wnv,
            &[source_polygon_a.clone(), source_polygon_b.clone()],
            &bounds,
            || {
                calls.set(calls.get() + 1);
                Ok((p(1, 2, 3), axis_defs(&p(1, 2, 3)), vec![7]))
            },
        )
        .unwrap();
        let second = cached_child_reference_with(
            &cache,
            &old_ref,
            &old_ref_definitions,
            &old_wnv,
            &[source_polygon_b, source_polygon_a],
            &bounds,
            || {
                calls.set(calls.get() + 1);
                Ok((p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![8]))
            },
        )
        .unwrap();

        assert_eq!(calls.get(), 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_child_subdivision_reuses_identical_child_task() {
        let task = SubdivisionTask::new(
            vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
            Aabb::new(p(0, 0, 0), p(1, 1, 0)),
            p(0, 0, 0),
            vec![0],
        );
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);

        let first = cached_child_subdivision_with(&cache, &task, || {
            calls.set(calls.get() + 1);
            Ok(vec![ClassifiedPolygon::new(
                make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
                1,
            )])
        })
        .unwrap();
        let second = cached_child_subdivision_with(&cache, &task, || {
            calls.set(calls.get() + 1);
            Ok(vec![ClassifiedPolygon::new(
                make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1),
                1,
            )])
        })
        .unwrap();

        assert_eq!(calls.get(), 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_child_subdivision_keeps_distinct_child_tasks_separate() {
        let task_a = SubdivisionTask::new(
            vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
            Aabb::new(p(0, 0, 0), p(1, 1, 0)),
            p(0, 0, 0),
            vec![0],
        );
        let task_b = SubdivisionTask::new(
            vec![make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0)],
            Aabb::new(p(0, 0, 0), p(2, 2, 0)),
            p(0, 0, 0),
            vec![0],
        );
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);

        cached_child_subdivision_with(&cache, &task_a, || {
            calls.set(calls.get() + 1);
            Ok(Vec::new())
        })
        .unwrap();
        cached_child_subdivision_with(&cache, &task_b, || {
            calls.set(calls.get() + 1);
            Ok(Vec::new())
        })
        .unwrap();

        assert_eq!(calls.get(), 2);
    }

    #[test]
    fn cached_child_subdivision_reuses_permuted_parent_definition_families() {
        let mut task = SubdivisionTask::new(
            vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
            Aabb::new(p(0, 0, 0), p(1, 1, 0)),
            p(1, 2, 3),
            vec![0],
        );
        let definition = axis_defs(&task.ref_point)[0].clone();
        task.ref_definitions = vec![definition.clone()];
        let mut permuted_task = task.clone();
        permuted_task.ref_definitions = vec![[
            definition[1].clone(),
            definition[2].clone(),
            definition[0].clone(),
        ]];
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);

        let first = cached_child_subdivision_with(&cache, &task, || {
            calls.set(calls.get() + 1);
            Ok(vec![ClassifiedPolygon::new(
                make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
                1,
            )])
        })
        .unwrap();
        let second = cached_child_subdivision_with(&cache, &permuted_task, || {
            calls.set(calls.get() + 1);
            Ok(vec![ClassifiedPolygon::new(
                make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1),
                1,
            )])
        })
        .unwrap();

        assert_eq!(calls.get(), 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_child_subdivision_reuses_permuted_polygon_families() {
        let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
        let task = SubdivisionTask::new(
            vec![polygon_a.clone(), polygon_b.clone()],
            Aabb::new(p(0, 0, 0), p(1, 1, 1)),
            p(0, 0, 0),
            vec![0],
        );
        let mut permuted_task = task.clone();
        permuted_task.polygons = vec![polygon_b, polygon_a];
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);

        let first = cached_child_subdivision_with(&cache, &task, || {
            calls.set(calls.get() + 1);
            Ok(vec![ClassifiedPolygon::new(
                make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
                1,
            )])
        })
        .unwrap();
        let second = cached_child_subdivision_with(&cache, &permuted_task, || {
            calls.set(calls.get() + 1);
            Ok(vec![ClassifiedPolygon::new(
                make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1),
                1,
            )])
        })
        .unwrap();

        assert_eq!(calls.get(), 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_child_subdivision_reuses_deeper_success_for_shallower_equivalent_task() {
        let mut deeper_task = SubdivisionTask::new(
            vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
            Aabb::new(p(0, 0, 0), p(1, 1, 0)),
            p(0, 0, 0),
            vec![0],
        );
        deeper_task.depth = 3;
        let mut shallower_task = deeper_task.clone();
        shallower_task.depth = 1;
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);

        let first = cached_child_subdivision_with(&cache, &deeper_task, || {
            calls.set(calls.get() + 1);
            Ok(vec![ClassifiedPolygon::new(
                make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
                1,
            )])
        })
        .unwrap();
        let second = cached_child_subdivision_with(&cache, &shallower_task, || {
            calls.set(calls.get() + 1);
            Ok(vec![ClassifiedPolygon::new(
                make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1),
                1,
            )])
        })
        .unwrap();

        assert_eq!(calls.get(), 1);
        assert_eq!(first, second);
    }

    #[test]
    fn reusable_child_subdivision_if_certified_reuses_changed_reference_state() {
        let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let existing_task =
            SubdivisionTask::new(vec![polygon.clone()], bounds.clone(), p(1, 1, 1), vec![0]);
        let query_task = SubdivisionTask::new(vec![polygon], bounds, p(2, 1, 1), vec![0]);
        let cache = RefCell::new(vec![ChildSubdivisionCacheEntry {
            polygon_profile: polygon_family_profile(&existing_task.polygons),
            task: existing_task,
            result: Ok(vec![]),
        }]);
        let mut query_caches = SupportReferenceQueryCaches::default();

        let reused =
            reusable_child_subdivision_if_certified(&cache, &query_task, &mut query_caches)
                .unwrap();

        assert_eq!(reused, Some(vec![]));
    }

    #[test]
    fn reusable_child_subdivision_if_certified_skips_invalid_reference_state() {
        let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let existing_task =
            SubdivisionTask::new(vec![polygon.clone()], bounds.clone(), p(1, 1, 1), vec![0]);
        let query_task = SubdivisionTask::new(vec![polygon], bounds, p(0, 0, 0), vec![0]);
        let cache = RefCell::new(vec![ChildSubdivisionCacheEntry {
            polygon_profile: polygon_family_profile(&existing_task.polygons),
            task: existing_task,
            result: Ok(vec![]),
        }]);
        let mut query_caches = SupportReferenceQueryCaches::default();

        let reused =
            reusable_child_subdivision_if_certified(&cache, &query_task, &mut query_caches)
                .unwrap();

        assert_eq!(reused, None);
    }

    #[test]
    fn cached_child_subdivision_keeps_shallower_and_deeper_successes_separate() {
        let mut shallower_task = SubdivisionTask::new(
            vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
            Aabb::new(p(0, 0, 0), p(1, 1, 0)),
            p(0, 0, 0),
            vec![0],
        );
        shallower_task.depth = 1;
        let mut deeper_task = shallower_task.clone();
        deeper_task.depth = 3;
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);

        let first = cached_child_subdivision_with(&cache, &shallower_task, || {
            calls.set(calls.get() + 1);
            Ok(vec![ClassifiedPolygon::new(
                make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
                1,
            )])
        })
        .unwrap();
        let second = cached_child_subdivision_with(&cache, &deeper_task, || {
            calls.set(calls.get() + 1);
            Ok(vec![ClassifiedPolygon::new(
                make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1),
                1,
            )])
        })
        .unwrap();

        assert_eq!(calls.get(), 2);
        assert_ne!(first, second);
    }

    #[test]
    fn cached_child_subdivision_allows_nested_shared_cache_queries() {
        let task_a = SubdivisionTask::new(
            vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
            Aabb::new(p(0, 0, 0), p(1, 1, 0)),
            p(0, 0, 0),
            vec![0],
        );
        let task_b = SubdivisionTask::new(
            vec![make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0)],
            Aabb::new(p(0, 0, 0), p(2, 2, 0)),
            p(0, 0, 0),
            vec![0],
        );
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);

        cached_child_subdivision_with(&cache, &task_a, || {
            calls.set(calls.get() + 1);
            cached_child_subdivision_with(&cache, &task_b, || {
                calls.set(calls.get() + 1);
                Ok(Vec::new())
            })?;
            Ok(Vec::new())
        })
        .unwrap();

        cached_child_subdivision_with(&cache, &task_b, || {
            calls.set(calls.get() + 100);
            Ok(Vec::new())
        })
        .unwrap();

        assert_eq!(calls.get(), 2);
    }

    #[test]
    fn support_target_collection_backtracks_after_uncertified_candidate() {
        let mut targets = Vec::new();

        extend_reference_targets_backtracking_unknown(
            &mut targets,
            [p(0, 0, 0), p(1, 2, 3)],
            |candidate| {
                if candidate == p(0, 0, 0) {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(vec![ReferenceTarget::axis_defined(candidate)])
                }
            },
        )
        .unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, p(1, 2, 3));
        assert!(targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn shifted_support_target_family_marks_surviving_targets_uncertain_after_boundary_report_witness()
     {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let families = ShiftedSupportCellFamilies {
            shifted: halfspaces.clone(),
            report: Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                p(0, 2, 2),
                [None, None, None],
            )),
            saw_unknown: false,
            strict_seeds: vec![p(1, 1, 1)],
            shifted_vertices: Vec::new(),
            shifted_geometry_seeds: Vec::new(),
        };

        let cache = std::cell::RefCell::new(Vec::<ReferenceWitnessTargetCacheEntry>::new());
        let strict_contains_cache =
            std::cell::RefCell::new(Vec::<ReferenceHalfspaceContainmentCacheEntry>::new());
        let targets = shifted_support_cell_targets_from_families(
            &bounds,
            &halfspaces,
            &families,
            &cache,
            &strict_contains_cache,
        )
        .unwrap();

        assert!(!targets.is_empty());
        assert!(
            targets
                .iter()
                .all(|target| target.uncertified_definition_fallback)
        );
    }

    #[test]
    fn shifted_support_target_family_prefers_certified_report_witness_duplicate() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let witness = p(1, 2, 3);
        let families = ShiftedSupportCellFamilies {
            shifted: halfspaces.clone(),
            report: Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                witness.clone(),
                [Some(9), None, None],
            )),
            saw_unknown: false,
            strict_seeds: Vec::new(),
            shifted_vertices: Vec::new(),
            shifted_geometry_seeds: Vec::new(),
        };

        let cache = std::cell::RefCell::new(Vec::<ReferenceWitnessTargetCacheEntry>::new());
        let strict_contains_cache =
            std::cell::RefCell::new(Vec::<ReferenceHalfspaceContainmentCacheEntry>::new());
        let targets = shifted_support_cell_targets_from_families(
            &bounds,
            &halfspaces,
            &families,
            &cache,
            &strict_contains_cache,
        )
        .unwrap();

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].point, witness);
        assert!(!targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn reference_target_from_halfspace_witness_retains_axis_definition_when_active_definitions_fail()
     {
        let halfspaces = vec![axis_halfspace(0, false, r(1))];

        let target = reference_target_from_halfspace_witness(
            &p(1, 2, 3),
            &halfspaces,
            [Some(9), None, None],
        )
        .unwrap();

        let target = target.expect("witness target should still be retained");
        assert_eq!(target.point, p(1, 2, 3));
        assert!(target.uncertified_definition_fallback);
        assert!(
            target
                .definitions
                .iter()
                .any(|definition| definition == &axis_plane_definition(&p(1, 2, 3)))
        );
    }

    #[test]
    fn reference_target_from_halfspace_witness_salvages_coincident_halfspaces_after_invalid_active_index()
     {
        let witness = p(1, 2, 3);
        let halfspaces = vec![
            axis_halfspace(0, false, r(1)),
            LimitPlane3::new(p(1, 1, 1), r(-6)),
        ];

        let target =
            reference_target_from_halfspace_witness(&witness, &halfspaces, [Some(9), None, None])
                .unwrap()
                .expect("witness target should still be retained");

        assert_eq!(target.point, witness);
        assert!(target.uncertified_definition_fallback);
        assert!(target.definitions.iter().any(|definition| {
            definition
                .iter()
                .any(|plane| plane.normal == p(1, 1, 1) && plane.offset == r(-6))
        }));
    }

    #[test]
    fn cached_reference_target_from_halfspace_witness_reuses_permuted_active_state() {
        let witness = p(1, 2, 3);
        let halfspaces = vec![
            axis_halfspace(0, false, r(1)),
            axis_halfspace(1, false, r(2)),
            axis_halfspace(2, false, r(3)),
        ];
        let permuted = vec![
            halfspaces[2].clone(),
            halfspaces[0].clone(),
            halfspaces[1].clone(),
        ];
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_reference_target_from_halfspace_witness_with(
            &mut cache,
            &witness,
            &halfspaces,
            [Some(0), Some(1), None],
            || {
                calls += 1;
                Ok(Some(ReferenceTarget::axis_defined(witness.clone())))
            },
        )
        .unwrap();
        let second = cached_reference_target_from_halfspace_witness_with(
            &mut cache,
            &witness,
            &permuted,
            [Some(1), Some(2), None],
            || {
                calls += 1;
                Ok(Some(ReferenceTarget::axis_defined(p(9, 9, 9))))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_reference_target_from_halfspace_witness_distinguishes_active_state() {
        let witness = p(1, 2, 3);
        let halfspaces = vec![
            axis_halfspace(0, false, r(1)),
            axis_halfspace(1, false, r(2)),
            axis_halfspace(2, false, r(3)),
        ];
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_reference_target_from_halfspace_witness_with(
            &mut cache,
            &witness,
            &halfspaces,
            [Some(0), None, None],
            || {
                calls += 1;
                Ok(Some(ReferenceTarget::axis_defined(witness.clone())))
            },
        )
        .unwrap();
        let second = cached_reference_target_from_halfspace_witness_with(
            &mut cache,
            &witness,
            &halfspaces,
            [Some(1), None, None],
            || {
                calls += 1;
                Ok(Some(ReferenceTarget::axis_defined(p(9, 9, 9))))
            },
        )
        .unwrap();

        assert_eq!(calls, 2);
        assert_ne!(first, second);
    }

    #[test]
    fn valid_reference_rejects_local_surface_points() {
        let mut wall = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 2, 4), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        assert!(!is_valid_reference_for_bounds(&p(2, 2, 1), &bounds, &[wall]).unwrap());
    }

    #[test]
    fn certified_reference_validity_reports_unknown_for_local_surface_boundary_point() {
        let mut wall = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 2, 4), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        assert_eq!(
            is_certified_valid_reference_for_bounds(&p(2, 1, 2), &bounds, &[wall]),
            Err(crate::error::HypermeshError::UnknownClassification)
        );
    }

    #[test]
    fn compute_new_reference_reports_unknown_after_boundary_inherited_reference_if_search_exhausts()
    {
        let mut wall = make_triangle(&p(0, 0, 0), &p(0, 1, 0), &p(0, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let old_ref = p(0, 0, 0);
        let bounds = Aabb::new(old_ref.clone(), old_ref.clone());

        let err = compute_new_reference(&old_ref, &axis_defs(&old_ref), &[0], &bounds, &[wall])
            .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn trace_reference_target_rejects_invalid_targets() {
        let mut wall = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 2, 4), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

        assert_eq!(
            trace_reference_target(
                &p(-1, -1, -1),
                &axis_defs(&p(-1, -1, -1)),
                &[0],
                &bounds,
                &[wall.clone()],
                &ReferenceTarget::axis_defined(p(2, 2, 1))
            )
            .unwrap(),
            None
        );
        assert_eq!(
            trace_reference_target(
                &p(-1, -1, -1),
                &axis_defs(&p(-1, -1, -1)),
                &[0],
                &bounds,
                &[wall],
                &ReferenceTarget::axis_defined(p(5, 2, 2))
            )
            .unwrap(),
            None
        );
    }

    #[test]
    fn trace_reference_target_reports_unknown_for_uncertified_valid_target() {
        let ref_point = p(0, 0, 0);
        let target_point = p(2, 1, 0);
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, -1, -1), p(3, 2, 1));

        assert_eq!(
            crate::segment_trace::trace_segment(&ref_point, &target_point, &[0], &[wall.clone()]),
            Err(crate::error::HypermeshError::UnknownClassification)
        );

        let err = trace_reference_target(
            &ref_point,
            &axis_defs(&ref_point),
            &[0],
            &bounds,
            &[wall],
            &ReferenceTarget::axis_defined(target_point),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn trace_reference_target_retries_axis_plane_replacement_definitions() {
        let ref_point = p(0, 0, 0);
        let target_point = p(2, 1, 0);
        let ref_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::from_coefficients(r(1), r(1), r(1), r(0)),
        ];
        let invalid_definition = [
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(0, r(0)),
        ];
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, -1, -1), p(3, 2, 1));

        assert_eq!(
            crate::segment_trace::trace_segment(&ref_point, &target_point, &[0], &[wall.clone()]),
            Err(crate::error::HypermeshError::UnknownClassification)
        );

        let winding = trace_reference_target(
            &ref_point,
            &[ref_definition],
            &[0],
            &bounds,
            &[wall],
            &ReferenceTarget::with_definitions(target_point, vec![invalid_definition]),
        )
        .unwrap();

        assert_eq!(winding, Some(vec![0]));
    }

    #[test]
    fn trace_reference_target_retries_axis_start_after_retained_definitions_fail() {
        let ref_point = p(0, 0, 0);
        let target_point = p(2, 1, 0);
        let invalid_ref_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(0, r(2)),
        ];
        let valid_target_definition = [
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(1, r(1)),
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
        ];
        let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, -1, -1), p(3, 2, 1));

        assert_eq!(
            crate::segment_trace::trace_segment(&ref_point, &target_point, &[0], &[wall.clone()]),
            Err(crate::error::HypermeshError::UnknownClassification)
        );

        let winding = trace_reference_target(
            &ref_point,
            &[invalid_ref_definition],
            &[0],
            &bounds,
            &[wall],
            &ReferenceTarget::with_definitions(target_point, vec![valid_target_definition]),
        )
        .unwrap();

        assert_eq!(winding, Some(vec![0]));
    }

    #[test]
    fn trace_reference_target_uses_detour_on_plane_replacement_step() {
        let ref_point = p(0, 0, 0);
        let target_point = p(4, 0, 0);
        let ref_definition = [
            Plane::axis_aligned(0, r(0)),
            Plane::from_coefficients(r(-1), r(1), r(0), r(0)),
            Plane::from_coefficients(r(-1), r(0), r(1), r(0)),
        ];
        let target_definition = [
            Plane::from_coefficients(r(1), r(1), r(0), r(-4)),
            Plane::axis_aligned(1, r(0)),
            Plane::axis_aligned(2, r(0)),
        ];
        let mut blockers = vec![
            make_triangle(&p(2, 0, 0), &p(3, 0, 0), &p(2, 1, 0), 0, 0),
            make_triangle(&p(0, 2, 0), &p(1, 2, 0), &p(0, 3, 0), 0, 1),
            make_triangle(&p(0, 0, 2), &p(1, 0, 2), &p(0, 1, 2), 0, 2),
        ];
        for (index, x) in [q(2, 3), r(1), q(4, 3)].into_iter().enumerate() {
            blockers.push(make_triangle(
                &px(x.clone(), -1, -1),
                &px(x.clone(), 3, -1),
                &px(x, 1, 3),
                0,
                3 + index as isize,
            ));
        }
        let bounds = Aabb::new(p(0, -1, -1), p(5, 3, 5));

        assert_eq!(
            crate::segment_trace::trace_segment(&ref_point, &target_point, &[0], &blockers),
            Err(crate::error::HypermeshError::UnknownClassification)
        );

        let winding = trace_reference_target(
            &ref_point,
            &[ref_definition],
            &[0],
            &bounds,
            &blockers,
            &ReferenceTarget::with_definitions(target_point, vec![target_definition]),
        )
        .unwrap();

        assert_eq!(winding, Some(vec![0]));
    }

    #[test]
    fn projection_escape_bounds_stop_at_nearest_axis_surfaces() {
        let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
        left.delta_w = vec![1];
        let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
        right.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

        let escape = projection_escape_bounds(&p(1, 3, 3), &bounds, &[left, right])
            .unwrap()
            .expect("parallel walls should define a strict projection escape box");

        assert_eq!(escape.min.x, r(0));
        assert_eq!(escape.max.x, r(4));
        assert_eq!(escape.min.y, r(0));
        assert_eq!(escape.max.y, r(6));
        assert_eq!(escape.min.z, r(0));
        assert_eq!(escape.max.z, r(6));
    }

    #[test]
    fn projection_escape_bounds_family_includes_later_exact_boxes() {
        let mut x_wall = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 0);
        x_wall.delta_w = vec![1];
        let mut y_wall = make_triangle(&p(0, 5, 0), &p(6, 5, 0), &p(0, 5, 6), 0, 1);
        y_wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

        let family =
            projection_escape_bounds_family(&p(1, 3, 3), &bounds, &[x_wall, y_wall]).unwrap();

        assert!(family.len() >= 4);
        assert_eq!(family[0], Aabb::new(p(0, 0, 0), p(4, 5, 6)));
        assert!(
            family
                .iter()
                .any(|bounds| *bounds == Aabb::new(p(0, 0, 0), p(6, 5, 6)))
        );
        assert!(
            family
                .iter()
                .any(|bounds| *bounds == Aabb::new(p(0, 0, 0), p(4, 6, 6)))
        );
    }

    #[test]
    fn projection_escape_bounds_family_backtracks_after_uncertified_candidate_box() {
        let axis_options = vec![
            (vec![r(0)], vec![r(1), r(2)]),
            (vec![r(0)], vec![r(1)]),
            (vec![r(0)], vec![r(1)]),
        ];

        let (family, saw_unknown) = projection_escape_bounds_family_from_axis_options_with_extents(
            &axis_options,
            |bounds| {
                if *bounds == Aabb::new(p(0, 0, 0), p(1, 1, 1)) {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(true)
                }
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(family, vec![Aabb::new(p(0, 0, 0), p(2, 1, 1))]);
    }

    #[test]
    fn escaped_reference_axis_stop_values_backtrack_after_uncertified_crossing() {
        let projected = p(0, 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(3, 1, 1));
        let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let (stop_values, saw_unknown) = escaped_reference_axis_stop_values_with_queries(
            &projected,
            &bounds,
            &[first, second],
            0,
            true,
            |_projected, _endpoint, polygon, _axis| {
                if polygon.vertices().unwrap()[0].x == r(1) {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(p(2, 0, 0)))
                }
            },
            |_crossing, _polygon| Ok(LocalPolygonPointLocation::Interior),
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(stop_values, vec![r(2), r(3)]);
    }

    #[test]
    fn escaped_reference_axis_stop_values_treat_boundary_crossing_as_unknown_and_keep_later_corridor()
     {
        let projected = p(0, 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(3, 1, 1));
        let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let (stop_values, saw_unknown) = escaped_reference_axis_stop_values_with_queries(
            &projected,
            &bounds,
            &[first, second],
            0,
            true,
            |_projected, _endpoint, polygon, _axis| {
                let x = polygon.vertices().unwrap()[0].x.clone();
                Ok(Some(Point3::new(x, r(0), r(0))))
            },
            |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(1) {
                    Ok(LocalPolygonPointLocation::Boundary)
                } else {
                    Ok(LocalPolygonPointLocation::Interior)
                }
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(stop_values, vec![r(2), r(3)]);
    }

    #[test]
    fn escaped_reference_axis_stop_values_treat_endpoint_boundary_contact_as_unknown_and_keep_later_corridor()
     {
        let projected = p(0, 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(3, 1, 1));
        let first = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let (stop_values, saw_unknown) = escaped_reference_axis_stop_values_with_queries(
            &projected,
            &bounds,
            &[first, second],
            0,
            true,
            |_projected, endpoint, polygon, _axis| {
                let x = polygon.vertices().unwrap()[0].x.clone();
                if x == r(3) {
                    Ok(Some(endpoint.clone()))
                } else {
                    Ok(Some(Point3::new(x, r(0), r(0))))
                }
            },
            |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(3) {
                    Ok(LocalPolygonPointLocation::Boundary)
                } else {
                    Ok(LocalPolygonPointLocation::Interior)
                }
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(stop_values, vec![r(2), r(3)]);
    }

    #[test]
    fn escaped_reference_axis_stop_values_treat_start_boundary_contact_as_unknown_and_keep_later_corridor()
     {
        let projected = p(0, 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(3, 1, 1));
        let first = make_triangle(&p(0, 0, 0), &p(0, 1, 0), &p(0, 0, 1), 0, 0);
        let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

        let (stop_values, saw_unknown) = escaped_reference_axis_stop_values_with_queries(
            &projected,
            &bounds,
            &[first, second],
            0,
            true,
            |projected, _endpoint, polygon, _axis| {
                let x = polygon.vertices().unwrap()[0].x.clone();
                if x == r(0) {
                    Ok(Some(projected.clone()))
                } else {
                    Ok(Some(Point3::new(x, r(0), r(0))))
                }
            },
            |_crossing, polygon| {
                if polygon.vertices().unwrap()[0].x == r(0) {
                    Ok(LocalPolygonPointLocation::Boundary)
                } else {
                    Ok(LocalPolygonPointLocation::Interior)
                }
            },
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(stop_values, vec![r(2), r(3)]);
    }

    #[test]
    fn escaped_reference_axis_stop_values_treat_bound_start_contact_as_unknown() {
        let projected = p(3, 0, 0);
        let bounds = Aabb::new(p(0, 0, 0), p(3, 1, 1));

        let (stop_values, saw_unknown) = escaped_reference_axis_stop_values_with_queries(
            &projected,
            &bounds,
            &[],
            0,
            true,
            |_projected, _endpoint, _polygon, _axis| Ok(None),
            |_crossing, _polygon| Ok(LocalPolygonPointLocation::Outside),
        )
        .unwrap();

        assert!(saw_unknown);
        assert!(stop_values.is_empty());
    }

    #[test]
    fn projection_axis_escape_reference_reports_unknown_when_only_fallback_corridor_witness_exists()
    {
        let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
        left.delta_w = vec![1];
        let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
        right.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

        let err = projection_axis_escape_reference(
            &p(-1, 3, 3),
            &axis_defs(&p(-1, 3, 3)),
            &[0],
            &p(1, 3, 3),
            &bounds,
            &[left, right],
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn cached_projection_escape_axis_options_reuses_projected_target_point() {
        let projected = p(1, 3, 3);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_projection_escape_axis_options_with(
            &mut cache,
            &projected,
            &bounds,
            &polygons,
            || {
                calls += 1;
                Ok(vec![(vec![r(0)], vec![r(4)]); 3])
            },
        )
        .unwrap();
        let second = cached_projection_escape_axis_options_with(
            &mut cache,
            &projected,
            &bounds,
            &polygons,
            || {
                calls += 1;
                Ok(vec![(vec![r(0)], vec![r(6)]); 3])
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn support_reference_query_caches_reuse_identical_halfspace_queries() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut query_caches = SupportReferenceQueryCaches::default();
        let mut report_calls = 0;
        let mut feasible_calls = 0;

        let first_report = cached_halfspace_report_with(
            &mut query_caches.report_cache,
            &halfspaces,
            |_halfspaces| {
                report_calls += 1;
                Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                    p(1, 1, 1),
                    [None, None, None],
                )))
            },
        )
        .unwrap();
        let first_feasible = cached_halfspace_feasibility_with(
            &mut query_caches.feasible_cache,
            &halfspaces,
            |_halfspaces| {
                feasible_calls += 1;
                Ok(true)
            },
        )
        .unwrap();
        let second_report = cached_halfspace_report_with(
            &mut query_caches.report_cache,
            &halfspaces,
            |_halfspaces| {
                report_calls += 1;
                Ok(None)
            },
        )
        .unwrap();
        let second_feasible = cached_halfspace_feasibility_with(
            &mut query_caches.feasible_cache,
            &halfspaces,
            |_halfspaces| {
                feasible_calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert_eq!(report_calls, 1);
        assert_eq!(feasible_calls, 1);
        assert_eq!(first_report, second_report);
        assert_eq!(first_feasible, second_feasible);
    }

    #[test]
    fn support_reference_query_caches_reuse_report_for_feasibility() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut query_caches = SupportReferenceQueryCaches::default();
        let mut report_calls = 0;
        let mut feasible_calls = 0;

        let report = cached_halfspace_report_with(
            &mut query_caches.report_cache,
            &halfspaces,
            |_halfspaces| {
                report_calls += 1;
                Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                    p(1, 1, 1),
                    [None, None, None],
                )))
            },
        )
        .unwrap();
        let feasible = cached_halfspace_feasibility_with_report_cache(
            &mut query_caches.report_cache,
            &mut query_caches.feasible_cache,
            &halfspaces,
            |_halfspaces| {
                report_calls += 1;
                Ok(None)
            },
            |_halfspaces| {
                feasible_calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert!(report.is_some());
        assert!(feasible);
        assert_eq!(report_calls, 1);
        assert_eq!(feasible_calls, 0);
    }

    #[test]
    fn support_reference_query_caches_prime_report_from_feasibility() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut query_caches = SupportReferenceQueryCaches::default();
        let mut report_calls = 0;
        let mut feasible_calls = 0;

        let feasible = cached_halfspace_feasibility_with_report_cache(
            &mut query_caches.report_cache,
            &mut query_caches.feasible_cache,
            &halfspaces,
            |_halfspaces| {
                report_calls += 1;
                Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                    p(1, 1, 1),
                    [None, None, None],
                )))
            },
            |_halfspaces| {
                feasible_calls += 1;
                Ok(false)
            },
        )
        .unwrap();
        let report = cached_halfspace_report_with(
            &mut query_caches.report_cache,
            &halfspaces,
            |_halfspaces| {
                report_calls += 1;
                Ok(None)
            },
        )
        .unwrap();

        assert!(feasible);
        assert!(report.is_some());
        assert_eq!(report_calls, 1);
        assert_eq!(feasible_calls, 0);
    }

    #[test]
    fn support_reference_query_caches_prime_projected_root_report_for_later_support_queries() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let old_ref = p(-1, 2, 2);
        let halfspaces = projected_reference_halfspaces(&old_ref, &bounds).unwrap();
        let projected_root =
            projected_root_reference_families(&bounds, &halfspaces, &mut Vec::new()).unwrap();
        let mut query_caches = SupportReferenceQueryCaches::default();
        let mut report_calls = 0;
        let mut feasible_calls = 0;

        prime_support_reference_query_caches_with_known_halfspace_report(
            &mut query_caches,
            &halfspaces,
            projected_root.report.as_ref(),
            projected_root.saw_unknown,
        );

        let report = cached_halfspace_report_with(
            &mut query_caches.report_cache,
            &halfspaces,
            |_halfspaces| {
                report_calls += 1;
                Ok(None)
            },
        )
        .unwrap();
        let feasible = cached_halfspace_feasibility_with_report_cache(
            &mut query_caches.report_cache,
            &mut query_caches.feasible_cache,
            &halfspaces,
            |_halfspaces| {
                report_calls += 1;
                Ok(None)
            },
            |_halfspaces| {
                feasible_calls += 1;
                Ok(false)
            },
        )
        .unwrap();

        assert_eq!(report, projected_root.report);
        assert!(feasible);
        assert_eq!(report_calls, 0);
        assert_eq!(feasible_calls, 0);
    }

    #[test]
    fn cached_projected_root_reference_families_reuse_permuted_halfspace_state() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let old_ref = p(-1, 2, 2);
        let halfspaces = projected_reference_halfspaces(&old_ref, &bounds).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(2);
        let mut caches = SupportReferenceQueryCaches::default();

        let first =
            cached_projected_root_reference_families_with(&bounds, &halfspaces, &mut caches)
                .unwrap();
        let second =
            cached_projected_root_reference_families_with(&bounds, &permuted, &mut caches).unwrap();

        assert_eq!(first.report, second.report);
        assert_eq!(first.projected_targets, second.projected_targets);
        assert_eq!(
            first.projected_escape_targets,
            second.projected_escape_targets
        );
        assert_eq!(first.saw_unknown, second.saw_unknown);
        assert_eq!(caches.projected_root_cache.len(), 1);
    }

    #[test]
    fn cached_support_cell_seed_geometry_reuses_identical_halfspaces() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = projected_reference_halfspaces(&p(-1, 2, 2), &bounds).unwrap();
        let mut cache = Vec::new();
        let mut centroid_subset_seed_cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_cell_seed_geometry_with(&mut cache, &halfspaces, || {
            calls += 1;
            support_cell_seed_geometry_state(&halfspaces, &mut centroid_subset_seed_cache)
        })
        .unwrap();
        let second = cached_support_cell_seed_geometry_with(&mut cache, &halfspaces, || {
            calls += 1;
            Ok(SupportCellSeedGeometryState {
                shifted_vertices: vec![p(9, 9, 9)],
                shifted_geometry_seeds: vec![p(8, 8, 8)],
                saw_unknown: false,
            })
        })
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_cell_seed_geometry_reuses_permuted_halfspaces() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = projected_reference_halfspaces(&p(-1, 2, 2), &bounds).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(1);
        let mut cache = Vec::new();
        let mut centroid_subset_seed_cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_cell_seed_geometry_with(&mut cache, &halfspaces, || {
            calls += 1;
            support_cell_seed_geometry_state(&halfspaces, &mut centroid_subset_seed_cache)
        })
        .unwrap();
        let second = cached_support_cell_seed_geometry_with(&mut cache, &permuted, || {
            calls += 1;
            Ok(SupportCellSeedGeometryState {
                shifted_vertices: vec![p(9, 9, 9)],
                shifted_geometry_seeds: vec![p(8, 8, 8)],
                saw_unknown: false,
            })
        })
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_point3_centroid_subset_family_reuses_permuted_vertices() {
        let first_vertices = vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0)];
        let second_vertices = vec![p(0, 2, 0), p(0, 0, 0), p(2, 0, 0)];
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_point3_centroid_subset_family_from_vertices_with(
            &mut cache,
            &first_vertices,
            || {
                calls += 1;
                point3_centroid_subset_family_from_vertices(&first_vertices)
            },
        )
        .unwrap();
        let second = cached_point3_centroid_subset_family_from_vertices_with(
            &mut cache,
            &second_vertices,
            || {
                calls += 1;
                Ok(Point3FamilyState {
                    points: vec![p(9, 9, 9)],
                    saw_unknown: true,
                })
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn support_reference_query_caches_prime_unknown_report_for_later_support_queries() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut query_caches = SupportReferenceQueryCaches::default();
        let mut report_calls = 0;
        let mut feasible_calls = 0;

        prime_support_reference_query_caches_with_known_halfspace_report(
            &mut query_caches,
            &halfspaces,
            None,
            true,
        );

        let report_err = cached_halfspace_report_with(
            &mut query_caches.report_cache,
            &halfspaces,
            |_halfspaces| {
                report_calls += 1;
                Ok(None)
            },
        )
        .unwrap_err();
        let feasible_err = cached_halfspace_feasibility_with_report_cache(
            &mut query_caches.report_cache,
            &mut query_caches.feasible_cache,
            &halfspaces,
            |_halfspaces| {
                report_calls += 1;
                Ok(None)
            },
            |_halfspaces| {
                feasible_calls += 1;
                Ok(false)
            },
        )
        .unwrap_err();

        assert_eq!(
            report_err,
            crate::error::HypermeshError::UnknownClassification
        );
        assert_eq!(
            feasible_err,
            crate::error::HypermeshError::UnknownClassification
        );
        assert_eq!(report_calls, 0);
        assert_eq!(feasible_calls, 0);
    }

    #[test]
    fn support_reference_query_caches_reset_preserves_shareable_caches() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let point = p(1, 1, 1);
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let context = support_reference_cache_context_key(
            &point,
            &[axis_plane_definition(&point)],
            &[0],
            &[support_only_polygon(Plane::axis_aligned(0, r(2)))],
        );
        let mut query_caches = SupportReferenceQueryCaches::default();

        query_caches.report_cache.push(HalfspaceReportCacheEntry {
            halfspaces: halfspaces.clone(),
            report: Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                point.clone(),
                [None, None, None],
            ))),
        });
        query_caches
            .seed_geometry_cache
            .push(SupportCellSeedGeometryCacheEntry {
                halfspaces: halfspaces.clone(),
                geometry: Ok(SupportCellSeedGeometryState {
                    shifted_vertices: vec![point.clone()],
                    shifted_geometry_seeds: Vec::new(),
                    saw_unknown: false,
                }),
            });
        query_caches
            .centroid_subset_seed_cache
            .push(Point3CentroidSubsetFamilyCacheEntry {
                vertices: vec![point.clone()],
                family: Ok(Point3FamilyState {
                    points: Vec::new(),
                    saw_unknown: false,
                }),
            });
        query_caches
            .reference_witness_cache
            .get_mut()
            .push(ReferenceWitnessTargetCacheEntry {
                point: point.clone(),
                halfspaces: halfspaces.clone(),
                active_planes: [None, None, None],
                target: Ok(Some(ReferenceTarget::axis_defined(point.clone()))),
            });
        query_caches
            .trace_cache
            .push(ReferenceTargetTraceCacheEntry {
                context: Some(context.clone()),
                target: ReferenceTarget::axis_defined(point.clone()),
                winding: Ok(Some(vec![0])),
            });
        query_caches
            .validity_cache
            .push(ReferenceBoundsValidityCacheEntry {
                context: Some(context.clone()),
                bounds: bounds.clone(),
                point: point.clone(),
                is_valid: Ok(true),
            });
        query_caches
            .support_surface_cache
            .push(SupportSurfaceCacheEntry {
                context: Some(context.clone()),
                point: point.clone(),
                on_support_surface: Ok(false),
            });
        query_caches
            .accept_cache
            .get_mut()
            .push(SupportReferenceAcceptCacheEntry {
                context: Some(context.clone()),
                bounds: bounds.clone(),
                halfspaces: halfspaces.clone(),
                report: None,
                accepted: Ok(None),
            });
        query_caches
            .search_cache
            .get_mut()
            .push(SupportPlaneCellSearchCacheEntry {
                context: Some(context),
                preferred_order: [false, true],
                bounds: bounds.clone(),
                polygon_index: 0,
                halfspaces: halfspaces.clone(),
                result: Ok(None::<(ReferenceTarget, Vec<i32>)>),
            });

        query_caches.reset_per_reference_call_caches();

        assert_eq!(query_caches.report_cache.len(), 1);
        assert_eq!(query_caches.seed_geometry_cache.len(), 1);
        assert_eq!(query_caches.centroid_subset_seed_cache.len(), 1);
        assert_eq!(query_caches.reference_witness_cache.get_mut().len(), 1);
        assert_eq!(query_caches.trace_cache.len(), 1);
        assert_eq!(query_caches.validity_cache.len(), 1);
        assert_eq!(query_caches.support_surface_cache.len(), 1);
        assert_eq!(query_caches.accept_cache.get_mut().len(), 1);
        assert_eq!(query_caches.search_cache.get_mut().len(), 1);
    }

    #[test]
    fn cached_projection_escape_axis_options_state_reuses_projected_target_point() {
        let projected = p(1, 3, 3);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_projection_escape_axis_options_state_with(
            &mut cache,
            &projected,
            &bounds,
            &polygons,
            || {
                calls += 1;
                Ok(ProjectionEscapeAxisOptionsState {
                    axis_options: vec![(vec![r(0)], vec![r(4)]); 3],
                    saw_unknown: true,
                })
            },
        )
        .unwrap();
        let second = cached_projection_escape_axis_options_state_with(
            &mut cache,
            &projected,
            &bounds,
            &polygons,
            || {
                calls += 1;
                Ok(ProjectionEscapeAxisOptionsState {
                    axis_options: vec![(vec![r(0)], vec![r(6)]); 3],
                    saw_unknown: false,
                })
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_halfspace_report_reuses_identical_state() {
        let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_halfspace_report_with(&mut cache, &halfspaces, |_halfspaces| {
            calls += 1;
            Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                p(1, 1, 1),
                [None, None, None],
            )))
        })
        .unwrap();
        let second = cached_halfspace_report_with(&mut cache, &halfspaces, |_halfspaces| {
            calls += 1;
            Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                p(2, 2, 2),
                [Some(0), None, None],
            )))
        })
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_halfspace_report_reuses_permuted_state() {
        let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(1);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_halfspace_report_with(&mut cache, &halfspaces, |_halfspaces| {
            calls += 1;
            Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                p(1, 1, 1),
                [None, None, None],
            )))
        })
        .unwrap();
        let second = cached_halfspace_report_with(&mut cache, &permuted, |_halfspaces| {
            calls += 1;
            Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                p(2, 2, 2),
                [Some(0), None, None],
            )))
        })
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_halfspace_feasibility_reuses_identical_state() {
        let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_halfspace_feasibility_with(&mut cache, &halfspaces, |_halfspaces| {
            calls += 1;
            Ok(true)
        })
        .unwrap();
        let second = cached_halfspace_feasibility_with(&mut cache, &halfspaces, |_halfspaces| {
            calls += 1;
            Ok(false)
        })
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_halfspace_feasibility_reuses_permuted_state() {
        let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(1);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_halfspace_feasibility_with(&mut cache, &halfspaces, |_halfspaces| {
            calls += 1;
            Ok(true)
        })
        .unwrap();
        let second = cached_halfspace_feasibility_with(&mut cache, &permuted, |_halfspaces| {
            calls += 1;
            Ok(false)
        })
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_reference_target_trace_reuses_identical_target() {
        let target = ReferenceTarget::axis_defined(p(1, 2, 3));
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_reference_target_trace_with(&mut cache, &target, |_target| {
            calls += 1;
            Ok(Some(vec![17]))
        })
        .unwrap();
        let second = cached_reference_target_trace_with(&mut cache, &target, |_target| {
            calls += 1;
            Ok(Some(vec![99]))
        })
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_reference_target_trace_distinguishes_reference_context() {
        let point = p(1, 2, 3);
        let target = ReferenceTarget::axis_defined(point.clone());
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let left_old_ref = p(0, 0, 0);
        let left_context = support_reference_cache_context_key(
            &left_old_ref,
            &[axis_plane_definition(&left_old_ref)],
            &[0],
            &polygons,
        );
        let right_old_ref = p(1, 0, 0);
        let right_context = support_reference_cache_context_key(
            &right_old_ref,
            &[axis_plane_definition(&right_old_ref)],
            &[0],
            &polygons,
        );
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_reference_target_trace_with_context(
            &mut cache,
            Some(&left_context),
            &target,
            |_target| {
                calls += 1;
                Ok(Some(vec![17]))
            },
        )
        .unwrap();
        let second = cached_reference_target_trace_with_context(
            &mut cache,
            Some(&right_context),
            &target,
            |_target| {
                calls += 1;
                Ok(Some(vec![23]))
            },
        )
        .unwrap();

        assert_eq!(calls, 2);
        assert_eq!(first, Some(vec![17]));
        assert_eq!(second, Some(vec![23]));
    }

    #[test]
    fn cached_reference_bounds_validity_reuses_identical_point() {
        let point = p(1, 2, 3);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_reference_bounds_validity_with(&mut cache, &bounds, &point, |_point| {
            calls += 1;
            Ok(true)
        })
        .unwrap();
        let second = cached_reference_bounds_validity_with(&mut cache, &bounds, &point, |_point| {
            calls += 1;
            Ok(false)
        })
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_reference_bounds_validity_keeps_distinct_bounds_separate() {
        let point = p(1, 2, 3);
        let bounds_a = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let bounds_b = Aabb::new(p(0, 0, 0), p(5, 4, 4));
        let mut cache = Vec::new();
        let mut calls = 0;

        cached_reference_bounds_validity_with(&mut cache, &bounds_a, &point, |_point| {
            calls += 1;
            Ok(true)
        })
        .unwrap();
        cached_reference_bounds_validity_with(&mut cache, &bounds_b, &point, |_point| {
            calls += 1;
            Ok(true)
        })
        .unwrap();

        assert_eq!(calls, 2);
    }

    #[test]
    fn cached_reference_bounds_validity_reuses_same_polygon_context() {
        let point = p(1, 2, 3);
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let left_old_ref = p(0, 0, 0);
        let left_context = support_reference_cache_context_key(
            &left_old_ref,
            &[axis_plane_definition(&left_old_ref)],
            &[0],
            &polygons,
        );
        let right_old_ref = p(1, 0, 0);
        let right_context = support_reference_cache_context_key(
            &right_old_ref,
            &[axis_plane_definition(&right_old_ref)],
            &[0],
            &polygons,
        );
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_reference_bounds_validity_with_context(
            &mut cache,
            Some(&left_context),
            &bounds,
            &point,
            |_point| {
                calls += 1;
                Ok(true)
            },
        )
        .unwrap();
        let second = cached_reference_bounds_validity_with_context(
            &mut cache,
            Some(&right_context),
            &bounds,
            &point,
            |_point| {
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
    fn cached_support_surface_query_reuses_same_polygon_context() {
        let point = p(2, 1, 1);
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let left_old_ref = p(0, 0, 0);
        let left_context = support_reference_cache_context_key(
            &left_old_ref,
            &[axis_plane_definition(&left_old_ref)],
            &[0],
            &polygons,
        );
        let right_old_ref = p(1, 0, 0);
        let right_context = support_reference_cache_context_key(
            &right_old_ref,
            &[axis_plane_definition(&right_old_ref)],
            &[0],
            &polygons,
        );
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_surface_query_with_context(
            &mut cache,
            Some(&left_context),
            &point,
            |_point| {
                calls += 1;
                Ok(true)
            },
        )
        .unwrap();
        let second = cached_support_surface_query_with_context(
            &mut cache,
            Some(&right_context),
            &point,
            |_point| {
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
    fn projected_reference_trace_helper_reuses_point_validity_and_full_target_trace() {
        use std::cell::Cell;

        let first = ReferenceTarget::axis_defined(p(1, 2, 3));
        let second = ReferenceTarget::axis_defined_fallback(p(1, 2, 3));
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let mut validity_cache = Vec::new();
        let mut trace_cache = Vec::new();
        let validity_calls = Cell::new(0);
        let trace_calls = Cell::new(0);

        let first_result = trace_projected_reference_target_with_queries(
            &mut validity_cache,
            &mut trace_cache,
            &bounds,
            &first,
            |_point| {
                validity_calls.set(validity_calls.get() + 1);
                Ok(true)
            },
            |target| {
                trace_calls.set(trace_calls.get() + 1);
                Ok(Some(vec![if target.uncertified_definition_fallback {
                    2
                } else {
                    1
                }]))
            },
        )
        .unwrap();
        let second_result = trace_projected_reference_target_with_queries(
            &mut validity_cache,
            &mut trace_cache,
            &bounds,
            &second,
            |_point| {
                validity_calls.set(validity_calls.get() + 1);
                Ok(true)
            },
            |target| {
                trace_calls.set(trace_calls.get() + 1);
                Ok(Some(vec![if target.uncertified_definition_fallback {
                    2
                } else {
                    1
                }]))
            },
        )
        .unwrap();
        let third_result = trace_projected_reference_target_with_queries(
            &mut validity_cache,
            &mut trace_cache,
            &bounds,
            &first,
            |_point| {
                validity_calls.set(validity_calls.get() + 1);
                Ok(false)
            },
            |_target| {
                trace_calls.set(trace_calls.get() + 1);
                Ok(Some(vec![99]))
            },
        )
        .unwrap();

        assert_eq!(validity_calls.get(), 1);
        assert_eq!(trace_calls.get(), 1);
        assert_eq!(first_result, Some(vec![1]));
        assert_eq!(second_result, Some(vec![1]));
        assert_eq!(third_result, Some(vec![1]));
    }

    #[test]
    fn cached_reference_target_trace_reuses_certified_and_fallback_duplicates() {
        use std::cell::Cell;

        let point = p(1, 2, 3);
        let target = ReferenceTarget::axis_defined(point.clone());
        let fallback = ReferenceTarget::axis_defined_fallback(point);
        let mut trace_cache = Vec::new();
        let calls = Cell::new(0);

        let first = cached_reference_target_trace_with(&mut trace_cache, &fallback, |_target| {
            calls.set(calls.get() + 1);
            Ok(Some(vec![7]))
        })
        .unwrap();
        let second = cached_reference_target_trace_with(&mut trace_cache, &target, |_target| {
            calls.set(calls.get() + 1);
            Ok(Some(vec![9]))
        })
        .unwrap();

        assert_eq!(first, Some(vec![7]));
        assert_eq!(second, Some(vec![7]));
        assert_eq!(calls.get(), 1);
    }

    #[test]
    fn cached_reference_target_trace_reuses_permuted_definition_families() {
        use std::cell::Cell;

        let point = p(1, 2, 3);
        let definition = axis_defs(&point)[0].clone();
        let permuted = [
            definition[1].clone(),
            definition[2].clone(),
            definition[0].clone(),
        ];
        let first = ReferenceTarget::with_definitions(point.clone(), vec![definition.clone()]);
        let second = ReferenceTarget::with_definitions(point, vec![permuted]);
        let mut trace_cache = Vec::new();
        let calls = Cell::new(0);

        let first_result =
            cached_reference_target_trace_with(&mut trace_cache, &first, |_target| {
                calls.set(calls.get() + 1);
                Ok(Some(vec![7]))
            })
            .unwrap();
        let second_result =
            cached_reference_target_trace_with(&mut trace_cache, &second, |_target| {
                calls.set(calls.get() + 1);
                Ok(Some(vec![9]))
            })
            .unwrap();

        assert_eq!(first_result, Some(vec![7]));
        assert_eq!(second_result, Some(vec![7]));
        assert_eq!(calls.get(), 1);
    }

    #[test]
    fn push_unique_reference_target_merges_permuted_definitions() {
        let point = p(1, 2, 3);
        let definition = axis_defs(&point)[0].clone();
        let permuted = [
            definition[1].clone(),
            definition[2].clone(),
            definition[0].clone(),
        ];
        let mut targets = vec![ReferenceTarget::with_definitions(
            point.clone(),
            vec![definition.clone()],
        )];

        push_unique_reference_target(
            &mut targets,
            ReferenceTarget::with_definitions(point, vec![permuted.clone()]),
        );

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].definitions.len(), 1);
        assert!(reference_definition_planes_match_as_sets(
            &targets[0].definitions[0],
            &permuted
        ));
    }

    #[test]
    fn push_unique_reference_target_prefers_certified_duplicate_definitions() {
        let point = p(1, 2, 3);
        let definition = axis_plane_definition(&point);
        let mut targets = vec![ReferenceTarget {
            point: point.clone(),
            definitions: vec![definition.clone()],
            uncertified_definition_fallback: true,
        }];

        push_unique_reference_target(
            &mut targets,
            ReferenceTarget::with_definitions(point, vec![definition]),
        );

        assert_eq!(targets.len(), 1);
        assert!(!targets[0].uncertified_definition_fallback);
    }

    #[test]
    fn push_verified_definition_merges_permuted_definitions() {
        let witness = p(1, 2, 3);
        let definition = axis_defs(&witness)[0].clone();
        let permuted = [
            definition[1].clone(),
            definition[2].clone(),
            definition[0].clone(),
        ];
        let mut definitions = vec![definition.clone()];

        let mut saw_unknown = false;
        push_verified_definition(
            &mut definitions,
            permuted.clone(),
            &witness,
            &mut saw_unknown,
        )
        .unwrap();

        assert_eq!(definitions.len(), 1);
        assert!(!saw_unknown);
        assert!(reference_definition_planes_match_as_sets(
            &definitions[0],
            &permuted
        ));
    }

    #[test]
    fn projected_and_support_reference_traces_share_validity_and_trace_caches() {
        use std::cell::Cell;

        let target = ReferenceTarget::axis_defined(p(1, 2, 3));
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let validity_calls = Cell::new(0);
        let trace_calls = Cell::new(0);
        let mut query_caches = SupportReferenceQueryCaches::default();
        let mut surface_cache = Vec::new();

        let projected = {
            let SupportReferenceQueryCaches {
                validity_cache,
                trace_cache,
                ..
            } = &mut query_caches;
            trace_projected_reference_target_with_queries(
                validity_cache,
                trace_cache,
                &bounds,
                &target,
                |_point| {
                    validity_calls.set(validity_calls.get() + 1);
                    Ok(true)
                },
                |_target| {
                    trace_calls.set(trace_calls.get() + 1);
                    Ok(Some(vec![7]))
                },
            )
            .unwrap()
        };

        let support = {
            let SupportReferenceQueryCaches {
                validity_cache,
                trace_cache,
                ..
            } = &mut query_caches;
            trace_reference_targets_backtracking_unknown_with_query_caches(
                vec![target],
                &mut surface_cache,
                validity_cache,
                None,
                &bounds,
                &mut |_point| Ok(false),
                &mut |_point| {
                    validity_calls.set(validity_calls.get() + 1);
                    Ok(true)
                },
                |target| {
                    cached_reference_target_trace_with(trace_cache, target, |_target| {
                        trace_calls.set(trace_calls.get() + 1);
                        Ok(Some(vec![99]))
                    })
                },
            )
            .unwrap()
        };

        assert_eq!(projected, Some(vec![7]));
        assert_eq!(
            support,
            Some((ReferenceTarget::axis_defined(p(1, 2, 3)), vec![7]))
        );
        assert_eq!(validity_calls.get(), 1);
        assert_eq!(trace_calls.get(), 1);
    }

    #[test]
    fn support_reference_target_trace_shortcut_skips_full_target_build_after_certified_report_witness()
     {
        use std::cell::Cell;

        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let witness = p(1, 1, 1);
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(witness.clone(), [None, None, None]);
        let reference_witness_cache = std::cell::RefCell::new(Vec::new());
        let strict_contains_cache = std::cell::RefCell::new(Vec::new());
        let mut surface_cache = Vec::new();
        let mut validity_cache = Vec::new();
        let build_calls = Cell::new(0);

        let found = trace_support_reference_targets_with_report_shortcut(
            &bounds,
            &halfspaces,
            Some(&report),
            &reference_witness_cache,
            &strict_contains_cache,
            &mut surface_cache,
            &mut validity_cache,
            None,
            &mut |_point| Ok(false),
            &mut |_point| Ok(true),
            || Ok((Vec::new(), false)),
            || {
                build_calls.set(build_calls.get() + 1);
                Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
            },
            |_target| Ok(Some(vec![7])),
        )
        .unwrap()
        .expect("certified report witness should short-circuit support target search");

        assert_eq!(build_calls.get(), 0);
        assert_eq!(found.0.point, witness);
        assert_eq!(found.1, vec![7]);
    }

    #[test]
    fn support_reference_target_trace_shortcut_falls_through_after_uncertified_report_witness() {
        use std::cell::Cell;

        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let witness = p(1, 1, 1);
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(witness.clone(), [None, None, None]);
        let later_target = ReferenceTarget::axis_defined(p(2, 2, 2));
        let reference_witness_cache = std::cell::RefCell::new(Vec::new());
        let strict_contains_cache = std::cell::RefCell::new(Vec::new());
        let mut surface_cache = Vec::new();
        let mut validity_cache = Vec::new();
        let build_calls = Cell::new(0);

        let found = trace_support_reference_targets_with_report_shortcut(
            &bounds,
            &halfspaces,
            Some(&report),
            &reference_witness_cache,
            &strict_contains_cache,
            &mut surface_cache,
            &mut validity_cache,
            None,
            &mut |_point| Ok(false),
            &mut |_point| Ok(true),
            || Ok((Vec::new(), false)),
            || {
                build_calls.set(build_calls.get() + 1);
                Ok(vec![later_target.clone()])
            },
            |target| {
                if target.point == witness {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(vec![5]))
                }
            },
        )
        .unwrap()
        .expect("later certified support target should survive uncertified report witness");

        assert_eq!(build_calls.get(), 1);
        assert_eq!(found, (later_target, vec![5]));
    }

    #[test]
    fn support_reference_target_trace_shortcut_skips_full_target_build_after_certified_direct_target()
     {
        use std::cell::Cell;

        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let direct_target = ReferenceTarget::axis_defined(p(2, 2, 2));
        let reference_witness_cache = std::cell::RefCell::new(Vec::new());
        let strict_contains_cache = std::cell::RefCell::new(Vec::new());
        let mut surface_cache = Vec::new();
        let mut validity_cache = Vec::new();
        let build_calls = Cell::new(0);

        let found = trace_support_reference_targets_with_report_shortcut(
            &bounds,
            &halfspaces,
            None,
            &reference_witness_cache,
            &strict_contains_cache,
            &mut surface_cache,
            &mut validity_cache,
            None,
            &mut |_point| Ok(false),
            &mut |_point| Ok(true),
            || Ok((vec![direct_target.clone()], false)),
            || {
                build_calls.set(build_calls.get() + 1);
                Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
            },
            |_target| Ok(Some(vec![11])),
        )
        .unwrap()
        .expect("certified direct support target should short-circuit full target build");

        assert_eq!(build_calls.get(), 0);
        assert_eq!(found, (direct_target, vec![11]));
    }

    #[test]
    fn cached_support_target_family_reuses_identical_state_and_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_target_family_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(vec![ReferenceTarget::axis_defined(p(1, 2, 3))])
            },
        )
        .unwrap();
        let second = cached_support_target_family_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_cell_seed_families_reuse_identical_state_and_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
        let mut cache = Vec::new();
        let mut calls = 0;
        let expected = SupportCellSeedFamiliesState {
            strict_seeds: vec![p(1, 1, 1)],
            shifted_vertices: vec![p(1, 1, 1)],
            shifted_geometry_seeds: Vec::new(),
            saw_unknown: false,
        };

        let first = cached_support_cell_seed_families_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&report),
            || {
                calls += 1;
                Ok(expected.clone())
            },
        )
        .unwrap();
        let second = cached_support_cell_seed_families_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&report),
            || {
                calls += 1;
                Ok(SupportCellSeedFamiliesState {
                    strict_seeds: vec![p(9, 9, 9)],
                    shifted_vertices: Vec::new(),
                    shifted_geometry_seeds: Vec::new(),
                    saw_unknown: true,
                })
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, expected);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_direct_reference_targets_reuse_identical_state_and_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
        let mut cache = Vec::new();
        let mut calls = 0;
        let expected = (vec![ReferenceTarget::axis_defined(p(1, 2, 3))], false);

        let first = cached_support_direct_reference_targets_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&report),
            || {
                calls += 1;
                Ok(expected.clone())
            },
        )
        .unwrap();
        let second = cached_support_direct_reference_targets_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&report),
            || {
                calls += 1;
                Ok((vec![ReferenceTarget::axis_defined(p(9, 9, 9))], true))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, expected);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_cell_seed_families_reuse_none_and_infeasible_reports() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let infeasible = hyperlimit::HalfspaceFeasibilityReport::infeasible(Some(
            hyperlimit::HalfspaceInfeasibilityCertificate {
                active_planes: [Some(0), Some(1), None, None],
                multipliers: [r(1), r(2), r(0), r(0)],
                offset_sum: r(3),
            },
        ));
        let mut cache = Vec::new();
        let mut calls = 0;
        let expected = SupportCellSeedFamiliesState {
            strict_seeds: vec![p(1, 1, 1)],
            shifted_vertices: vec![p(2, 2, 2)],
            shifted_geometry_seeds: vec![p(3, 3, 3)],
            saw_unknown: false,
        };

        let first =
            cached_support_cell_seed_families_with(&mut cache, &bounds, &halfspaces, None, || {
                calls += 1;
                Ok(expected.clone())
            })
            .unwrap();
        let second = cached_support_cell_seed_families_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&infeasible),
            || {
                calls += 1;
                Ok(SupportCellSeedFamiliesState {
                    strict_seeds: vec![p(9, 9, 9)],
                    shifted_vertices: Vec::new(),
                    shifted_geometry_seeds: Vec::new(),
                    saw_unknown: true,
                })
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, expected);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_cell_seed_families_reuse_same_witness_different_active_planes() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let witness = p(1, 1, 1);
        let left = hyperlimit::HalfspaceFeasibilityReport::feasible(
            witness.clone(),
            [Some(0), None, None],
        );
        let right =
            hyperlimit::HalfspaceFeasibilityReport::feasible(witness, [Some(1), None, None]);
        let mut cache = Vec::new();
        let mut calls = 0;
        let expected = SupportCellSeedFamiliesState {
            strict_seeds: vec![p(1, 1, 1)],
            shifted_vertices: vec![p(2, 2, 2)],
            shifted_geometry_seeds: vec![p(3, 3, 3)],
            saw_unknown: false,
        };

        let first = cached_support_cell_seed_families_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&left),
            || {
                calls += 1;
                Ok(expected.clone())
            },
        )
        .unwrap();
        let second = cached_support_cell_seed_families_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&right),
            || {
                calls += 1;
                Ok(SupportCellSeedFamiliesState {
                    strict_seeds: vec![p(9, 9, 9)],
                    shifted_vertices: Vec::new(),
                    shifted_geometry_seeds: Vec::new(),
                    saw_unknown: true,
                })
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, expected);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_direct_reference_targets_reuse_none_and_infeasible_reports() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let infeasible = hyperlimit::HalfspaceFeasibilityReport::infeasible(None);
        let mut cache = Vec::new();
        let mut calls = 0;
        let expected = (vec![ReferenceTarget::axis_defined(p(1, 2, 3))], false);

        let first = cached_support_direct_reference_targets_with(
            &mut cache,
            &bounds,
            &halfspaces,
            None,
            || {
                calls += 1;
                Ok(expected.clone())
            },
        )
        .unwrap();
        let second = cached_support_direct_reference_targets_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&infeasible),
            || {
                calls += 1;
                Ok((vec![ReferenceTarget::axis_defined(p(9, 9, 9))], true))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, expected);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_direct_reference_targets_reuse_same_witness_different_active_planes() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let witness = p(1, 1, 1);
        let left = hyperlimit::HalfspaceFeasibilityReport::feasible(
            witness.clone(),
            [Some(0), None, None],
        );
        let right =
            hyperlimit::HalfspaceFeasibilityReport::feasible(witness, [Some(1), None, None]);
        let mut cache = Vec::new();
        let mut calls = 0;
        let expected = (vec![ReferenceTarget::axis_defined(p(1, 2, 3))], false);

        let first = cached_support_direct_reference_targets_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&left),
            || {
                calls += 1;
                Ok(expected.clone())
            },
        )
        .unwrap();
        let second = cached_support_direct_reference_targets_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&right),
            || {
                calls += 1;
                Ok((vec![ReferenceTarget::axis_defined(p(9, 9, 9))], true))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, expected);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_target_family_reuses_permuted_state_and_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(1);
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_target_family_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(vec![ReferenceTarget::axis_defined(p(1, 2, 3))])
            },
        )
        .unwrap();
        let second = cached_support_target_family_with(
            &mut cache,
            &bounds,
            &permuted,
            Some(&report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_target_family_reuses_permuted_state_and_permuted_report_indices() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(1);
        let witness = p(1, 1, 1);
        let left_active = [Some(0), Some(1), Some(2)];
        let right_active = left_active.map(|index| {
            index.map(|index| {
                permuted
                    .iter()
                    .position(|plane| plane == &halfspaces[index])
                    .unwrap()
            })
        });
        let left_report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(witness.clone(), left_active);
        let right_report = hyperlimit::HalfspaceFeasibilityReport::feasible(witness, right_active);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_target_family_with(
            &mut cache,
            &bounds,
            &halfspaces,
            Some(&left_report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(vec![ReferenceTarget::axis_defined(p(1, 2, 3))])
            },
        )
        .unwrap();
        let second = cached_support_target_family_with(
            &mut cache,
            &bounds,
            &permuted,
            Some(&right_report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_reference_accept_reuses_identical_state_and_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_reference_accept_with(
            &mut cache,
            None,
            &bounds,
            &halfspaces,
            Some(&report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(Some((
                    ReferenceTarget::axis_defined(bounds.min.clone()),
                    vec![23],
                )))
            },
        )
        .unwrap();
        let second = cached_support_reference_accept_with(
            &mut cache,
            None,
            &bounds,
            &halfspaces,
            Some(&report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_reference_accept_reuses_permuted_state_and_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(1);
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_reference_accept_with(
            &mut cache,
            None,
            &bounds,
            &halfspaces,
            Some(&report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(Some((
                    ReferenceTarget::axis_defined(bounds.min.clone()),
                    vec![23],
                )))
            },
        )
        .unwrap();
        let second = cached_support_reference_accept_with(
            &mut cache,
            None,
            &bounds,
            &permuted,
            Some(&report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_reference_accept_reuses_permuted_state_and_permuted_report_indices() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(1);
        let witness = p(1, 1, 1);
        let left_active = [Some(0), Some(1), Some(2)];
        let right_active = left_active.map(|index| {
            index.map(|index| {
                permuted
                    .iter()
                    .position(|plane| plane == &halfspaces[index])
                    .unwrap()
            })
        });
        let left_report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(witness.clone(), left_active);
        let right_report = hyperlimit::HalfspaceFeasibilityReport::feasible(witness, right_active);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_reference_accept_with(
            &mut cache,
            None,
            &bounds,
            &halfspaces,
            Some(&left_report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(Some((
                    ReferenceTarget::axis_defined(bounds.min.clone()),
                    vec![23],
                )))
            },
        )
        .unwrap();
        let second = cached_support_reference_accept_with(
            &mut cache,
            None,
            &bounds,
            &permuted,
            Some(&right_report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_reference_accept_distinguishes_reference_context() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let left_old_ref = p(0, 0, 0);
        let left_context = support_reference_cache_context_key(
            &left_old_ref,
            &[axis_plane_definition(&left_old_ref)],
            &[0],
            &polygons,
        );
        let right_old_ref = p(1, 0, 0);
        let right_context = support_reference_cache_context_key(
            &right_old_ref,
            &[axis_plane_definition(&right_old_ref)],
            &[0],
            &polygons,
        );
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_reference_accept_with(
            &mut cache,
            Some(&left_context),
            &bounds,
            &halfspaces,
            Some(&report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(Some((ReferenceTarget::axis_defined(p(0, 0, 0)), vec![23])))
            },
        )
        .unwrap();
        let second = cached_support_reference_accept_with(
            &mut cache,
            Some(&right_context),
            &bounds,
            &halfspaces,
            Some(&report),
            |_halfspaces, _report| {
                calls += 1;
                Ok(Some((ReferenceTarget::axis_defined(p(1, 0, 0)), vec![24])))
            },
        )
        .unwrap();

        assert_eq!(calls, 2);
        assert_ne!(first, second);
    }

    #[test]
    fn cached_support_reference_result_reuses_identical_state() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let old_ref = p(0, 0, 0);
        let context = support_reference_cache_context_key(
            &old_ref,
            &[axis_plane_definition(&old_ref)],
            &[0],
            &polygons,
        );
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_reference_result_with(
            &mut cache,
            &context,
            &bounds,
            &halfspaces,
            || {
                calls += 1;
                Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23])))
            },
        )
        .unwrap();
        let second = cached_support_reference_result_with(
            &mut cache,
            &context,
            &bounds,
            &halfspaces,
            || {
                calls += 1;
                Ok(Some((ReferenceTarget::axis_defined(p(3, 3, 3)), vec![24])))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_reference_result_reuses_permuted_halfspaces() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(1);
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let old_ref = p(0, 0, 0);
        let context = support_reference_cache_context_key(
            &old_ref,
            &[axis_plane_definition(&old_ref)],
            &[0],
            &polygons,
        );
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_reference_result_with(
            &mut cache,
            &context,
            &bounds,
            &halfspaces,
            || {
                calls += 1;
                Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23])))
            },
        )
        .unwrap();
        let second =
            cached_support_reference_result_with(&mut cache, &context, &bounds, &permuted, || {
                calls += 1;
                Ok(Some((ReferenceTarget::axis_defined(p(3, 3, 3)), vec![24])))
            })
            .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_reference_result_distinguishes_reference_context() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let left_old_ref = p(0, 0, 0);
        let left_context = support_reference_cache_context_key(
            &left_old_ref,
            &[axis_plane_definition(&left_old_ref)],
            &[0],
            &polygons,
        );
        let right_old_ref = p(1, 0, 0);
        let right_context = support_reference_cache_context_key(
            &right_old_ref,
            &[axis_plane_definition(&right_old_ref)],
            &[0],
            &polygons,
        );
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_support_reference_result_with(
            &mut cache,
            &left_context,
            &bounds,
            &halfspaces,
            || {
                calls += 1;
                Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23])))
            },
        )
        .unwrap();
        let second = cached_support_reference_result_with(
            &mut cache,
            &right_context,
            &bounds,
            &halfspaces,
            || {
                calls += 1;
                Ok(Some((ReferenceTarget::axis_defined(p(3, 3, 3)), vec![24])))
            },
        )
        .unwrap();

        assert_eq!(calls, 2);
        assert_ne!(first, second);
    }

    #[test]
    fn cached_support_plane_cell_search_reuses_identical_state_and_index() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let cache = std::cell::RefCell::new(Vec::new());
        let mut calls = 0;

        let first = cached_support_plane_cell_search_with(
            &cache,
            None,
            [false, true],
            &bounds,
            3,
            halfspaces.clone(),
            || {
                calls += 1;
                Ok(Some(17))
            },
        )
        .unwrap();
        let second = cached_support_plane_cell_search_with(
            &cache,
            None,
            [false, true],
            &bounds,
            3,
            halfspaces,
            || {
                calls += 1;
                Ok(Some(99))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_plane_cell_search_reuses_same_preferred_order() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let cache = std::cell::RefCell::new(Vec::new());
        let mut calls = 0;
        let support = Plane::axis_aligned(0, r(2));
        let first_order = support_side_search_order(Some(&p(1, 1, 1)), &support);
        let second_order = support_side_search_order(Some(&p(1, 3, 3)), &support);

        assert_eq!(first_order, [false, true]);
        assert_eq!(first_order, second_order);

        let first = cached_support_plane_cell_search_with(
            &cache,
            None,
            first_order,
            &bounds,
            3,
            halfspaces.clone(),
            || {
                calls += 1;
                Ok(Some(17))
            },
        )
        .unwrap();
        let second = cached_support_plane_cell_search_with(
            &cache,
            None,
            second_order,
            &bounds,
            3,
            halfspaces,
            || {
                calls += 1;
                Ok(Some(99))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_support_plane_cell_search_distinguishes_preferred_order() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let cache = std::cell::RefCell::new(Vec::new());
        let mut calls = 0;

        let first = cached_support_plane_cell_search_with(
            &cache,
            None,
            [false, true],
            &bounds,
            3,
            halfspaces.clone(),
            || {
                calls += 1;
                Ok(Some(17))
            },
        )
        .unwrap();
        let second = cached_support_plane_cell_search_with(
            &cache,
            None,
            [true, false],
            &bounds,
            3,
            halfspaces,
            || {
                calls += 1;
                Ok(Some(99))
            },
        )
        .unwrap();

        assert_eq!(calls, 2);
        assert_eq!(first, Some(17));
        assert_eq!(second, Some(99));
    }

    #[test]
    fn optional_halfspace_reports_match_permuted_infeasibility_certificates() {
        let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(1);
        let left_active = [Some(0), Some(1), None, None];
        let right_active = left_active.map(|index| {
            index.map(|index| {
                permuted
                    .iter()
                    .position(|plane| plane == &halfspaces[index])
                    .unwrap()
            })
        });
        let left = hyperlimit::HalfspaceFeasibilityReport::infeasible(Some(
            hyperlimit::HalfspaceInfeasibilityCertificate {
                active_planes: left_active,
                multipliers: [r(1), r(2), r(0), r(0)],
                offset_sum: r(3),
            },
        ));
        let right = hyperlimit::HalfspaceFeasibilityReport::infeasible(Some(
            hyperlimit::HalfspaceInfeasibilityCertificate {
                active_planes: right_active,
                multipliers: [r(1), r(2), r(0), r(0)],
                offset_sum: r(3),
            },
        ));

        assert!(optional_halfspace_reports_match_for_cache(
            &halfspaces,
            Some(&left),
            &permuted,
            Some(&right),
        ));
    }

    #[test]
    fn cached_support_plane_cell_search_reuses_permuted_state_and_index() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(1);
        let cache = std::cell::RefCell::new(Vec::new());
        let mut calls = 0;

        let first = cached_support_plane_cell_search_with(
            &cache,
            None,
            [false, true],
            &bounds,
            3,
            halfspaces,
            || {
                calls += 1;
                Ok(Some(17))
            },
        )
        .unwrap();
        let second = cached_support_plane_cell_search_with(
            &cache,
            None,
            [false, true],
            &bounds,
            3,
            permuted,
            || {
                calls += 1;
                Ok(Some(99))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn support_plane_cell_search_cache_reuses_same_normalized_polygon_index() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygon = support_only_polygon(Plane::axis_aligned(0, r(2)));
        let polygons = vec![polygon.clone()];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        halfspaces.push(support_side_halfspace(&polygon.support, false));
        let cache = std::cell::RefCell::new(
            Vec::<SupportPlaneCellSearchCacheEntry<ReferenceTarget>>::new(),
        );
        let mut report_calls = 0;
        let mut accept_calls = 0;

        let first = support_plane_cell_search_with_queries_cached(
            None,
            Some(&p(0, 0, 0)),
            &bounds,
            &polygons,
            0,
            &mut halfspaces,
            &mut |_halfspaces| {
                report_calls += 1;
                Ok(None)
            },
            &mut |_halfspaces| Ok(true),
            &mut |_halfspaces, _report| {
                accept_calls += 1;
                Ok(None)
            },
            &cache,
        )
        .unwrap();
        let second = support_plane_cell_search_with_queries_cached(
            None,
            Some(&p(9, 9, 9)),
            &bounds,
            &polygons,
            1,
            &mut halfspaces,
            &mut |_halfspaces| {
                report_calls += 1;
                Ok(None)
            },
            &mut |_halfspaces| Ok(true),
            &mut |_halfspaces, _report| {
                accept_calls += 1;
                Ok(None)
            },
            &cache,
        )
        .unwrap();

        assert_eq!(first, None);
        assert_eq!(second, None);
        assert_eq!(report_calls, 1);
        assert_eq!(accept_calls, 1);
    }

    #[test]
    fn cached_support_plane_cell_search_distinguishes_reference_context() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let left_old_ref = p(0, 0, 0);
        let left_context = support_reference_cache_context_key(
            &left_old_ref,
            &[axis_plane_definition(&left_old_ref)],
            &[0],
            &polygons,
        );
        let right_old_ref = p(1, 0, 0);
        let right_context = support_reference_cache_context_key(
            &right_old_ref,
            &[axis_plane_definition(&right_old_ref)],
            &[0],
            &polygons,
        );
        let cache = std::cell::RefCell::new(
            Vec::<SupportPlaneCellSearchCacheEntry<ReferenceTarget>>::new(),
        );
        let mut calls = 0;

        let first = cached_support_plane_cell_search_with(
            &cache,
            Some(&left_context),
            [false, true],
            &bounds,
            0,
            halfspaces.clone(),
            || {
                calls += 1;
                Ok(Some(ReferenceTarget::axis_defined(p(0, 0, 0))))
            },
        )
        .unwrap();
        let second = cached_support_plane_cell_search_with(
            &cache,
            Some(&right_context),
            [false, true],
            &bounds,
            0,
            halfspaces,
            || {
                calls += 1;
                Ok(Some(ReferenceTarget::axis_defined(p(1, 0, 0))))
            },
        )
        .unwrap();

        assert_eq!(calls, 2);
        assert_ne!(first, second);
    }

    #[test]
    fn cached_shifted_projected_cell_families_reuse_identical_seed() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let seed = p(1, 2, 3);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_shifted_projected_cell_families_with(
            &mut cache,
            &bounds,
            &halfspaces,
            &seed,
            || {
                calls += 1;
                Ok(None)
            },
        )
        .unwrap();
        let second = cached_shifted_projected_cell_families_with(
            &mut cache,
            &bounds,
            &halfspaces,
            &seed,
            || {
                calls += 1;
                Ok(Some(ShiftedProjectedCellFamilies {
                    shifted: Vec::new(),
                    report: None,
                    saw_unknown: false,
                    strict_seeds: vec![p(9, 9, 9)],
                    shifted_vertices: Vec::new(),
                    shifted_geometry_seeds: Vec::new(),
                }))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_shifted_projected_cell_families_reuse_permuted_halfspace_state() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(1);
        let seed = p(1, 2, 3);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_shifted_projected_cell_families_with(
            &mut cache,
            &bounds,
            &halfspaces,
            &seed,
            || {
                calls += 1;
                Ok(None)
            },
        )
        .unwrap();
        let second = cached_shifted_projected_cell_families_with(
            &mut cache,
            &bounds,
            &permuted,
            &seed,
            || {
                calls += 1;
                Ok(Some(ShiftedProjectedCellFamilies {
                    shifted: Vec::new(),
                    report: None,
                    saw_unknown: false,
                    strict_seeds: vec![p(9, 9, 9)],
                    shifted_vertices: Vec::new(),
                    shifted_geometry_seeds: Vec::new(),
                }))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_shifted_projected_cell_families_distinguish_halfspace_state() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let alternate_halfspaces = {
            let mut alternate = halfspaces.clone();
            alternate.push(support_side_halfspace(&Plane::axis_aligned(0, r(2)), false));
            alternate
        };
        let seed = p(1, 2, 3);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_shifted_projected_cell_families_with(
            &mut cache,
            &bounds,
            &halfspaces,
            &seed,
            || {
                calls += 1;
                Ok(None)
            },
        )
        .unwrap();
        let second = cached_shifted_projected_cell_families_with(
            &mut cache,
            &bounds,
            &alternate_halfspaces,
            &seed,
            || {
                calls += 1;
                Ok(Some(ShiftedProjectedCellFamilies {
                    shifted: Vec::new(),
                    report: None,
                    saw_unknown: false,
                    strict_seeds: vec![p(9, 9, 9)],
                    shifted_vertices: Vec::new(),
                    shifted_geometry_seeds: Vec::new(),
                }))
            },
        )
        .unwrap();

        assert_eq!(calls, 2);
        assert_eq!(first, None);
        assert!(second.is_some());
    }

    #[test]
    fn cached_shifted_support_cell_families_reuse_identical_seed_and_state() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let seed = p(1, 2, 3);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_shifted_support_cell_families_with(
            &mut cache,
            &bounds,
            &halfspaces,
            &seed,
            || {
                calls += 1;
                Ok(None)
            },
        )
        .unwrap();
        let second = cached_shifted_support_cell_families_with(
            &mut cache,
            &bounds,
            &halfspaces,
            &seed,
            || {
                calls += 1;
                Ok(Some(ShiftedSupportCellFamilies {
                    shifted: Vec::new(),
                    report: None,
                    saw_unknown: false,
                    strict_seeds: vec![p(9, 9, 9)],
                    shifted_vertices: Vec::new(),
                    shifted_geometry_seeds: Vec::new(),
                }))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_shifted_support_cell_families_reuse_permuted_halfspace_state() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut permuted = halfspaces.clone();
        permuted.rotate_left(1);
        let seed = p(1, 2, 3);
        let mut cache = Vec::new();
        let mut calls = 0;

        let first = cached_shifted_support_cell_families_with(
            &mut cache,
            &bounds,
            &halfspaces,
            &seed,
            || {
                calls += 1;
                Ok(None)
            },
        )
        .unwrap();
        let second = cached_shifted_support_cell_families_with(
            &mut cache,
            &bounds,
            &permuted,
            &seed,
            || {
                calls += 1;
                Ok(Some(ShiftedSupportCellFamilies {
                    shifted: Vec::new(),
                    report: None,
                    saw_unknown: false,
                    strict_seeds: vec![p(9, 9, 9)],
                    shifted_vertices: Vec::new(),
                    shifted_geometry_seeds: Vec::new(),
                }))
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn cached_reference_escape_search_reuses_identical_escape_bounds() {
        let bounds = Aabb::new(p(1, 2, 3), p(4, 5, 6));
        let old_ref = p(0, 0, 0);
        let context = support_reference_cache_context_key(
            &old_ref,
            &[axis_plane_definition(&old_ref)],
            &[0],
            &[support_only_polygon(Plane::axis_aligned(0, r(2)))],
        );
        let mut cache = Vec::new();
        let mut calls = 0;

        let first =
            cached_reference_escape_search_with(&mut cache, &context, &bounds, |escape_bounds| {
                calls += 1;
                Ok(Some((
                    ReferenceTarget::axis_defined(escape_bounds.min.clone()),
                    vec![11],
                )))
            })
            .unwrap();
        let second =
            cached_reference_escape_search_with(&mut cache, &context, &bounds, |_escape_bounds| {
                calls += 1;
                Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
            })
            .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(first, second);
    }

    #[test]
    fn projection_axis_escape_stop_values_include_later_bound_corridor() {
        let mut wall = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

        let stops =
            escaped_reference_axis_stop_values(&p(1, 3, 3), &bounds, &[wall], 0, true).unwrap();

        assert_eq!(stops, vec![r(4), r(6)]);
    }

    #[test]
    fn projection_axis_escape_stop_values_report_unknown_for_bound_start_contact() {
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

        let err =
            escaped_reference_axis_stop_values(&p(6, 3, 3), &bounds, &[], 0, true).unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projection_axis_escape_reference_backtracks_after_empty_nearer_corridor() {
        let mut wall = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));
        let mut searched_corridors = Vec::new();

        let found = projection_axis_escape_reference_with_search(
            &p(1, 3, 3),
            &bounds,
            &[wall],
            |corridor| {
                searched_corridors.push(corridor.clone());
                if corridor.max.x == r(4) {
                    Ok(None)
                } else if corridor.max.x == r(6) {
                    Ok(Some((ReferenceTarget::axis_defined(p(5, 3, 3)), vec![9])))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert!(
            searched_corridors
                .iter()
                .any(|corridor| corridor.max.x == r(4) && corridor.min.x == r(1))
        );
        assert!(
            searched_corridors
                .iter()
                .any(|corridor| corridor.max.x == r(6) && corridor.min.x == r(1))
        );
        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(5, 3, 3)), vec![9]))
        );
    }

    #[test]
    fn projection_axis_escape_reference_backtracks_after_uncertified_corridor() {
        let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
        left.delta_w = vec![1];
        let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
        right.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));
        let mut attempts = 0;

        let found = projection_axis_escape_reference_with_search(
            &p(1, 3, 3),
            &bounds,
            &[left, right],
            |_corridor| {
                attempts += 1;
                if attempts == 1 {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![7])))
                }
            },
        )
        .unwrap();

        assert!(attempts >= 2);
        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![7]))
        );
    }

    #[test]
    fn projection_axis_escape_reference_accepts_later_corridor_after_endpoint_boundary_contact() {
        let mut boundary = make_triangle(&p(6, 3, 3), &p(6, 5, 3), &p(6, 3, 5), 0, 0);
        boundary.delta_w = vec![1];
        let mut interior = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
        interior.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));
        let mut searched_corridors = Vec::new();

        let found = projection_axis_escape_reference_with_search(
            &p(1, 3, 3),
            &bounds,
            &[boundary, interior],
            |corridor| {
                searched_corridors.push(corridor.clone());
                if corridor.max.x == r(4) {
                    Ok(Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![31])))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert!(
            searched_corridors
                .iter()
                .any(|corridor| corridor.max.x == r(4) && corridor.min.x == r(1))
        );
        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![31]))
        );
    }

    #[test]
    fn projection_axis_escape_reference_accepts_later_corridor_after_boundary_start_contact() {
        let mut boundary = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
        boundary.delta_w = vec![1];
        let mut interior = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
        interior.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));
        let mut searched_corridors = Vec::new();

        let found = projection_axis_escape_reference_with_search(
            &p(1, 3, 3),
            &bounds,
            &[boundary, interior],
            |corridor| {
                searched_corridors.push(corridor.clone());
                if corridor.max.x == r(4) {
                    Ok(Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![41])))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert!(
            searched_corridors
                .iter()
                .any(|corridor| corridor.max.x == r(4) && corridor.min.x == r(1))
        );
        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![41]))
        );
    }

    #[test]
    fn projection_axis_escape_reference_reports_unknown_when_corridor_family_is_partially_uncertified_and_search_fails()
     {
        let projected = p(1, 3, 3);
        let axis_options = vec![
            (vec![r(0)], vec![r(1), r(2)]),
            (vec![r(0)], vec![r(6)]),
            (vec![r(0)], vec![r(6)]),
        ];

        let err = projection_axis_escape_reference_with_search_and_axis_options_tracking_unknown(
            &projected,
            &axis_options,
            true,
            |_corridor| Ok(None),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projection_axis_escape_reference_accepts_later_corridor_after_uncertified_family_candidate()
    {
        let projected = p(1, 3, 3);
        let axis_options = vec![
            (vec![r(0)], vec![r(1), r(2)]),
            (vec![r(0)], vec![r(6)]),
            (vec![r(0)], vec![r(6)]),
        ];

        let found = projection_axis_escape_reference_with_search_and_axis_options_tracking_unknown(
            &projected,
            &axis_options,
            true,
            |corridor| {
                if corridor.max.x == r(2) {
                    Ok(Some((ReferenceTarget::axis_defined(p(2, 3, 3)), vec![13])))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(2, 3, 3)), vec![13]))
        );
    }

    #[test]
    fn projection_axis_escape_reference_skips_fallback_corridor_success() {
        let projected = p(1, 3, 3);
        let axis_options = vec![
            (vec![r(0)], vec![r(1), r(2)]),
            (vec![r(0)], vec![r(6)]),
            (vec![r(0)], vec![r(6)]),
        ];

        let found = projection_axis_escape_reference_with_search_and_axis_options_tracking_unknown(
            &projected,
            &axis_options,
            false,
            |corridor| {
                if corridor.max.x == r(1) {
                    Ok(Some((
                        ReferenceTarget::axis_defined_fallback(p(1, 3, 3)),
                        vec![11],
                    )))
                } else if corridor.max.x == r(2) {
                    Ok(Some((ReferenceTarget::axis_defined(p(2, 3, 3)), vec![13])))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(2, 3, 3)), vec![13]))
        );
    }

    #[test]
    fn projection_axis_escape_reference_reports_unknown_when_only_fallback_corridor_succeeds() {
        let projected = p(1, 3, 3);
        let axis_options = vec![
            (vec![r(0)], vec![r(1)]),
            (vec![r(0)], vec![r(6)]),
            (vec![r(0)], vec![r(6)]),
        ];

        let err = projection_axis_escape_reference_with_search_and_axis_options_tracking_unknown(
            &projected,
            &axis_options,
            false,
            |corridor| {
                if corridor.max.x == r(1) {
                    Ok(Some((
                        ReferenceTarget::axis_defined_fallback(p(1, 3, 3)),
                        vec![11],
                    )))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projection_axis_escape_reference_reports_unknown_if_all_corridors_are_uncertified() {
        let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
        left.delta_w = vec![1];
        let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
        right.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

        let err = projection_axis_escape_reference_with_search(
            &p(1, 3, 3),
            &bounds,
            &[left, right],
            |_corridor| Err(crate::error::HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projection_escape_reference_backtracks_after_uncertified_tight_box() {
        let mut x_wall = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 0);
        x_wall.delta_w = vec![1];
        let mut y_wall = make_triangle(&p(0, 5, 0), &p(6, 5, 0), &p(0, 5, 6), 0, 1);
        y_wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

        let found = projection_escape_reference_with_search(
            &p(1, 3, 3),
            &bounds,
            &[x_wall, y_wall],
            |escape_bounds| {
                if *escape_bounds == Aabb::new(p(0, 0, 0), p(4, 6, 6)) {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else if *escape_bounds == Aabb::new(p(0, 0, 0), p(6, 5, 6)) {
                    Ok(Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![5])))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![5]))
        );
    }

    #[test]
    fn projection_escape_reference_reports_unknown_when_box_family_is_partially_uncertified_and_search_fails()
     {
        let axis_options = vec![
            (vec![r(0)], vec![r(1), r(2)]),
            (vec![r(0)], vec![r(1)]),
            (vec![r(0)], vec![r(1)]),
        ];
        let bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));

        let err = projection_escape_reference_with_search_and_axis_options_and_bounds_family(
            &axis_options,
            &bounds,
            false,
            |_escape_bounds| Ok(None),
            |axis_options, saw_unknown| {
                let (family, family_unknown) =
                    projection_escape_bounds_family_from_axis_options_with_extents(
                        axis_options,
                        |escape_bounds| {
                            if *escape_bounds == Aabb::new(p(0, 0, 0), p(1, 1, 1)) {
                                Err(crate::error::HypermeshError::UnknownClassification)
                            } else {
                                Ok(true)
                            }
                        },
                    )?;
                *saw_unknown |= family_unknown;
                Ok(family)
            },
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projection_escape_reference_accepts_later_box_after_uncertified_family_candidate() {
        let axis_options = vec![
            (vec![r(0)], vec![r(1), r(2)]),
            (vec![r(0)], vec![r(1)]),
            (vec![r(0)], vec![r(1)]),
        ];
        let bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));

        let found = projection_escape_reference_with_search_and_axis_options_and_bounds_family(
            &axis_options,
            &bounds,
            false,
            |escape_bounds| {
                if *escape_bounds == Aabb::new(p(0, 0, 0), p(2, 1, 1)) {
                    Ok(Some((ReferenceTarget::axis_defined(p(2, 0, 0)), vec![3])))
                } else {
                    Ok(None)
                }
            },
            |axis_options, saw_unknown| {
                let (family, family_unknown) =
                    projection_escape_bounds_family_from_axis_options_with_extents(
                        axis_options,
                        |escape_bounds| {
                            if *escape_bounds == Aabb::new(p(0, 0, 0), p(1, 1, 1)) {
                                Err(crate::error::HypermeshError::UnknownClassification)
                            } else {
                                Ok(true)
                            }
                        },
                    )?;
                *saw_unknown |= family_unknown;
                Ok(family)
            },
        )
        .unwrap();

        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(2, 0, 0)), vec![3]))
        );
    }

    #[test]
    fn projection_escape_reference_skips_fallback_box_success() {
        let axis_options = vec![
            (vec![r(0)], vec![r(1), r(2)]),
            (vec![r(0)], vec![r(1)]),
            (vec![r(0)], vec![r(1)]),
        ];
        let bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));

        let found = projection_escape_reference_with_search_and_axis_options_and_bounds_family(
            &axis_options,
            &bounds,
            false,
            |escape_bounds| {
                if *escape_bounds == Aabb::new(p(0, 0, 0), p(1, 1, 1)) {
                    Ok(Some((
                        ReferenceTarget::axis_defined_fallback(p(1, 1, 1)),
                        vec![7],
                    )))
                } else if *escape_bounds == Aabb::new(p(0, 0, 0), p(2, 1, 1)) {
                    Ok(Some((ReferenceTarget::axis_defined(p(2, 0, 0)), vec![9])))
                } else {
                    Ok(None)
                }
            },
            |axis_options, saw_unknown| {
                let (family, family_unknown) =
                    projection_escape_bounds_family_from_axis_options_with_extents(
                        axis_options,
                        |_escape_bounds| Ok(true),
                    )?;
                *saw_unknown |= family_unknown;
                Ok(family)
            },
        )
        .unwrap();

        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(2, 0, 0)), vec![9]))
        );
    }

    #[test]
    fn projection_escape_reference_reports_unknown_when_only_fallback_box_succeeds() {
        let axis_options = vec![
            (vec![r(0)], vec![r(1)]),
            (vec![r(0)], vec![r(1)]),
            (vec![r(0)], vec![r(1)]),
        ];
        let bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));

        let err = projection_escape_reference_with_search_and_axis_options_and_bounds_family(
            &axis_options,
            &bounds,
            false,
            |_escape_bounds| {
                Ok(Some((
                    ReferenceTarget::axis_defined_fallback(p(1, 1, 1)),
                    vec![7],
                )))
            },
            |axis_options, saw_unknown| {
                let (family, family_unknown) =
                    projection_escape_bounds_family_from_axis_options_with_extents(
                        axis_options,
                        |_escape_bounds| Ok(true),
                    )?;
                *saw_unknown |= family_unknown;
                Ok(family)
            },
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projection_escape_reference_reports_unknown_when_axis_option_family_is_partially_uncertified_and_box_search_fails()
     {
        let axis_options = vec![
            (vec![r(0)], vec![r(1), r(2)]),
            (vec![r(0)], vec![r(1)]),
            (vec![r(0)], vec![r(1)]),
        ];
        let bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));

        let err = projection_escape_reference_with_search_and_axis_options_tracking_unknown(
            &axis_options,
            &bounds,
            true,
            |_escape_bounds| Ok(None),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projection_escape_reference_accepts_later_box_after_uncertified_axis_option_family_candidate()
     {
        let axis_options = vec![
            (vec![r(0)], vec![r(1), r(2)]),
            (vec![r(0)], vec![r(1)]),
            (vec![r(0)], vec![r(1)]),
        ];
        let bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));

        let found = projection_escape_reference_with_search_and_axis_options_tracking_unknown(
            &axis_options,
            &bounds,
            true,
            |escape_bounds| {
                if *escape_bounds == Aabb::new(p(0, 0, 0), p(2, 1, 1)) {
                    Ok(Some((ReferenceTarget::axis_defined(p(2, 0, 0)), vec![19])))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(2, 0, 0)), vec![19]))
        );
    }

    #[test]
    fn projection_escape_reference_reports_unknown_if_all_boxes_are_uncertified() {
        let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
        left.delta_w = vec![1];
        let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
        right.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

        let err = projection_escape_reference_with_search(
            &p(1, 3, 3),
            &bounds,
            &[left, right],
            |_escape_bounds| Err(crate::error::HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn projection_escape_reference_backtracks_after_empty_tighter_box() {
        let mut x_wall = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 0);
        x_wall.delta_w = vec![1];
        let mut y_wall = make_triangle(&p(0, 5, 0), &p(6, 5, 0), &p(0, 5, 6), 0, 1);
        y_wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));
        let mut searched_boxes = Vec::new();

        let found = projection_escape_reference_with_search(
            &p(1, 3, 3),
            &bounds,
            &[x_wall, y_wall],
            |escape_bounds| {
                searched_boxes.push(escape_bounds.clone());
                if *escape_bounds == Aabb::new(p(0, 0, 0), p(4, 5, 6)) {
                    Ok(None)
                } else if *escape_bounds == Aabb::new(p(0, 0, 0), p(6, 5, 6)) {
                    Ok(Some((ReferenceTarget::axis_defined(p(5, 4, 3)), vec![11])))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert!(searched_boxes.contains(&Aabb::new(p(0, 0, 0), p(4, 5, 6))));
        assert!(searched_boxes.contains(&Aabb::new(p(0, 0, 0), p(6, 5, 6))));
        assert_eq!(
            found,
            Some((ReferenceTarget::axis_defined(p(5, 4, 3)), vec![11]))
        );
    }

    #[test]
    fn support_plane_cell_finds_target_when_midpoint_is_blocked() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(5))),
            support_only_polygon(Plane::axis_aligned(1, r(5))),
            support_only_polygon(Plane::axis_aligned(2, r(5))),
            support_only_polygon(Plane::from_coefficients(r(1), r(1), r(1), r(-15))),
        ];

        assert!(point_lies_on_any_support_plane(&p(5, 5, 5), &polygons).unwrap());

        let target = support_plane_cell_target(&bounds, &polygons)
            .unwrap()
            .expect("strict support cell should have a feasible witness");

        assert!(point_strictly_inside_bounds(&target.point, &bounds).unwrap());
        assert!(!point_lies_on_any_support_plane(&target.point, &polygons).unwrap());
        assert!(
            target
                .definitions
                .iter()
                .any(|definition| affine_from_planes(definition).unwrap() == target.point)
        );
    }

    #[test]
    fn point_lies_on_any_support_plane_reports_unknown_for_boundary_contact() {
        let polygon = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0);

        let err = point_lies_on_any_support_plane(&p(2, 0, 0), &[polygon]).unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn point_lies_on_any_support_plane_ignores_coplanar_points_outside_polygon() {
        let polygon = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0);

        assert!(!point_lies_on_any_support_plane(&p(5, 5, 0), &[polygon]).unwrap());
    }

    #[test]
    fn support_plane_cell_search_accepts_current_cell_before_full_side_assignment() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(2))),
            support_only_polygon(Plane::axis_aligned(1, r(2))),
        ];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let root_halfspace_count = halfspaces.len();
        let mut calls = 0;

        let found = support_plane_cell_search_from(
            &bounds,
            &polygons,
            0,
            &mut halfspaces,
            &mut |halfspaces, _report| {
                calls += 1;
                if calls == 1 {
                    assert_eq!(halfspaces.len(), root_halfspace_count);
                    Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))))
                } else {
                    panic!(
                        "search should have accepted the current feasible support cell before \
                         exhausting later polygon branches"
                    );
                }
            },
        )
        .unwrap();

        assert_eq!(calls, 1);
        assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
    }

    #[test]
    fn support_plane_cell_search_backtracks_after_uncertified_current_cell() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(2))),
            support_only_polygon(Plane::axis_aligned(1, r(2))),
        ];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let root_halfspace_count = halfspaces.len();
        let mut call_halfspace_counts = Vec::new();
        let mut calls = 0;

        let found = support_plane_cell_search_from(
            &bounds,
            &polygons,
            0,
            &mut halfspaces,
            &mut |halfspaces, _report| {
                calls += 1;
                call_halfspace_counts.push(halfspaces.len());
                if calls == 1 {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))))
                }
            },
        )
        .unwrap();

        assert!(calls >= 2);
        assert_eq!(call_halfspace_counts[0], root_halfspace_count);
        assert!(
            call_halfspace_counts[1..]
                .iter()
                .any(|count| *count > root_halfspace_count)
        );
        assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
    }

    #[test]
    fn support_plane_cell_search_backtracks_after_uncertified_current_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(2))),
            support_only_polygon(Plane::axis_aligned(1, r(2))),
        ];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let root_halfspace_count = halfspaces.len();
        let mut accept_counts = Vec::new();

        let found = support_plane_cell_search_with_queries(
            None,
            &bounds,
            &polygons,
            0,
            &mut halfspaces,
            &mut |halfspaces| {
                if halfspaces.len() == root_halfspace_count {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    halfspace_system_report(halfspaces)
                }
            },
            &mut |halfspaces| halfspace_system_is_feasible(halfspaces),
            &mut |halfspaces, report| {
                assert!(report.is_none());
                accept_counts.push(halfspaces.len());
                Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))))
            },
        )
        .unwrap();

        assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
        assert!(accept_counts.contains(&root_halfspace_count));
    }

    #[test]
    fn support_plane_cell_search_backtracks_after_uncertified_branch_feasibility() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let root_halfspace_count = halfspaces.len();
        let mut accepted_counts = Vec::new();

        let found = support_plane_cell_search_with_queries(
            None,
            &bounds,
            &polygons,
            0,
            &mut halfspaces,
            &mut |halfspaces| halfspace_system_report(halfspaces),
            &mut |halfspaces| {
                if halfspaces.len() == root_halfspace_count + 1 {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    halfspace_system_is_feasible(halfspaces)
                }
            },
            &mut |halfspaces, _report| {
                accepted_counts.push(halfspaces.len());
                if halfspaces.len() == root_halfspace_count + 1 {
                    Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
        assert!(accepted_counts.contains(&(root_halfspace_count + 1)));
    }

    #[test]
    fn support_plane_cell_search_accepts_current_cell_without_certified_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(2))),
            support_only_polygon(Plane::axis_aligned(1, r(2))),
        ];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let root_halfspace_count = halfspaces.len();
        let mut accepted_counts = Vec::new();

        let found = support_plane_cell_search_with_queries(
            None,
            &bounds,
            &polygons,
            0,
            &mut halfspaces,
            &mut |halfspaces| {
                if halfspaces.len() == root_halfspace_count {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    halfspace_system_report(halfspaces)
                }
            },
            &mut |halfspaces| halfspace_system_is_feasible(halfspaces),
            &mut |halfspaces, report| {
                accepted_counts.push((halfspaces.len(), report.is_some()));
                if halfspaces.len() == root_halfspace_count {
                    Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))))
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap();

        assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
        assert!(
            accepted_counts
                .iter()
                .any(|(count, had_report)| *count == root_halfspace_count && !had_report)
        );
    }

    #[test]
    fn support_plane_cell_search_reports_unknown_if_current_report_and_branches_are_uncertified() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let err = support_plane_cell_search_with_queries(
            None,
            &bounds,
            &polygons,
            0,
            &mut halfspaces,
            &mut |_halfspaces| Err(crate::error::HypermeshError::UnknownClassification),
            &mut |_halfspaces| Err(crate::error::HypermeshError::UnknownClassification),
            &mut |_halfspaces, _report| {
                Err::<Option<ReferenceTarget>, _>(
                    crate::error::HypermeshError::UnknownClassification,
                )
            },
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn support_plane_cell_search_prefers_reference_side_first() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let root_halfspace_count = halfspaces.len();
        let mut accepted_branch = None;

        let found = support_plane_cell_search_with_queries(
            Some(&p(1, 1, 1)),
            &bounds,
            &polygons,
            0,
            &mut halfspaces,
            &mut |halfspaces| halfspace_system_report(halfspaces),
            &mut |halfspaces| halfspace_system_is_feasible(halfspaces),
            &mut |halfspaces, _report| {
                if halfspaces.len() == root_halfspace_count + 1 {
                    accepted_branch = Some(
                        halfspaces.last().unwrap()
                            == &support_side_halfspace(&polygons[0].support, false),
                    );
                    return Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
                }
                Ok(None)
            },
        )
        .unwrap();

        assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
        assert_eq!(accepted_branch, Some(true));
    }

    #[test]
    fn support_plane_cell_search_skips_duplicate_support_halfspace_branches() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(2))),
            support_only_polygon(Plane::axis_aligned(0, r(2))),
        ];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let repeated_branch = support_side_halfspace(&polygons[0].support, false);
        let mut duplicate_branch_count_seen = false;

        let found = support_plane_cell_search_with_queries(
            Some(&p(1, 1, 1)),
            &bounds,
            &polygons,
            0,
            &mut halfspaces,
            &mut |halfspaces| halfspace_system_report(halfspaces),
            &mut |halfspaces| {
                let repeated_count = halfspaces
                    .iter()
                    .filter(|halfspace| *halfspace == &repeated_branch)
                    .count();
                if repeated_count > 1 {
                    duplicate_branch_count_seen = true;
                }
                halfspace_system_is_feasible(halfspaces)
            },
            &mut |_halfspaces, _report| Ok(None::<ReferenceTarget>),
        )
        .unwrap();

        assert_eq!(found, None);
        assert!(!duplicate_branch_count_seen);
    }

    #[test]
    fn support_plane_cell_search_skips_already_fixed_support_plane_states() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(2))),
            support_only_polygon(Plane::axis_aligned(0, r(2))),
        ];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        halfspaces.push(support_side_halfspace(&polygons[0].support, false));
        let mut report_calls = 0;
        let mut accept_calls = 0;

        let found = support_plane_cell_search_with_queries(
            Some(&p(1, 1, 1)),
            &bounds,
            &polygons,
            0,
            &mut halfspaces,
            &mut |_halfspaces| {
                report_calls += 1;
                Ok(None)
            },
            &mut |_halfspaces| Ok(true),
            &mut |_halfspaces, _report| {
                accept_calls += 1;
                Ok(None::<ReferenceTarget>)
            },
        )
        .unwrap();

        assert_eq!(found, None);
        assert_eq!(report_calls, 1);
        assert_eq!(accept_calls, 1);
    }

    #[test]
    fn support_plane_cell_search_skips_opposite_support_halfspace_branches() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygon = support_only_polygon(Plane::axis_aligned(0, r(2)));
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        halfspaces.push(support_side_halfspace(&polygon.support, false));
        let opposite_branch = support_side_halfspace(&polygon.support, true);
        let mut opposite_branch_count_seen = false;

        let found = support_plane_cell_search_with_queries(
            Some(&p(1, 1, 1)),
            &bounds,
            &[polygon],
            0,
            &mut halfspaces,
            &mut |halfspaces| halfspace_system_report(halfspaces),
            &mut |halfspaces| {
                let opposite_count = halfspaces
                    .iter()
                    .filter(|halfspace| *halfspace == &opposite_branch)
                    .count();
                if opposite_count > 0 {
                    opposite_branch_count_seen = true;
                }
                halfspace_system_is_feasible(halfspaces)
            },
            &mut |_halfspaces, _report| Ok(None::<ReferenceTarget>),
        )
        .unwrap();

        assert_eq!(found, None);
        assert!(!opposite_branch_count_seen);
    }

    #[test]
    fn support_plane_cell_search_skips_surface_forcing_halfspace_states() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygon = support_only_polygon(Plane::axis_aligned(0, r(2)));
        let polygons = vec![polygon.clone()];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        halfspaces.push(support_side_halfspace(&polygon.support, false));
        halfspaces.push(support_side_halfspace(&polygon.support, true));
        let mut report_calls = 0;
        let mut accept_calls = 0;

        let found = support_plane_cell_search_with_queries(
            Some(&p(1, 1, 1)),
            &bounds,
            &polygons,
            0,
            &mut halfspaces,
            &mut |_halfspaces| {
                report_calls += 1;
                Ok(None)
            },
            &mut |_halfspaces| Ok(true),
            &mut |_halfspaces, _report| {
                accept_calls += 1;
                Ok(None::<ReferenceTarget>)
            },
        )
        .unwrap();

        assert_eq!(found, None);
        assert_eq!(report_calls, 0);
        assert_eq!(accept_calls, 0);
    }

    #[test]
    fn support_plane_cell_reference_accepts_current_cell_without_certified_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let old_ref = p(-1, 1, 1);
        let old_defs = axis_defs(&old_ref);
        let old_wnv = vec![0];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let root_halfspace_count = halfspaces.len();

        let found = support_plane_cell_reference_with_queries(
            &old_ref,
            &old_defs,
            &old_wnv,
            &bounds,
            &polygons,
            &mut halfspaces,
            &mut |halfspaces| {
                if halfspaces.len() == root_halfspace_count {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    halfspace_system_report(halfspaces)
                }
            },
            &mut |halfspaces| halfspace_system_is_feasible(halfspaces),
        )
        .unwrap()
        .expect("current support cell should be usable without a certified report");

        assert!(point_strictly_inside_bounds(&found.0.point, &bounds).unwrap());
        assert!(!point_lies_on_any_support_plane(&found.0.point, &polygons).unwrap());
    }

    #[test]
    fn support_plane_cell_reference_backtracks_after_uncertified_initial_feasibility_check() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
        let old_ref = p(-1, 1, 1);
        let old_defs = axis_defs(&old_ref);
        let old_wnv = vec![0];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let root_halfspace_count = halfspaces.len();

        let found = support_plane_cell_reference_with_queries(
            &old_ref,
            &old_defs,
            &old_wnv,
            &bounds,
            &polygons,
            &mut halfspaces,
            &mut |halfspaces| {
                if halfspaces.len() == root_halfspace_count {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    halfspace_system_report(halfspaces)
                }
            },
            &mut |halfspaces| {
                if halfspaces.len() == root_halfspace_count {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    halfspace_system_is_feasible(halfspaces)
                }
            },
        )
        .unwrap();

        assert!(found.is_some());
    }

    #[test]
    fn support_plane_cell_reference_reports_unknown_if_initial_feasibility_and_search_fail() {
        let bounds = Aabb::new(p(0, 0, 0), p(0, 0, 0));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(0)))];
        let old_ref = p(-1, 0, 0);
        let old_defs = axis_defs(&old_ref);
        let old_wnv = vec![0];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let err = support_plane_cell_reference_with_queries(
            &old_ref,
            &old_defs,
            &old_wnv,
            &bounds,
            &polygons,
            &mut halfspaces,
            &mut |_halfspaces| Ok(None),
            &mut |_halfspaces| Err(crate::error::HypermeshError::UnknownClassification),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn duplicate_reference_targets_merge_definitions() {
        let point = p(1, 2, 3);
        let mut targets = vec![ReferenceTarget::axis_defined(point.clone())];
        let slanted_definition = [
            Plane::from_coefficients(r(1), r(1), r(0), r(-3)),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(2, r(3)),
        ];

        push_unique_reference_target(
            &mut targets,
            ReferenceTarget::with_definitions(point, vec![slanted_definition.clone()]),
        );
        push_unique_reference_target(
            &mut targets,
            ReferenceTarget::with_definitions(p(1, 2, 3), vec![slanted_definition]),
        );

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].definitions.len(), 2);
        assert!(
            targets[0]
                .definitions
                .iter()
                .any(definition_uses_non_axis_plane)
        );
    }

    #[test]
    fn take_new_point_family_preserves_first_occurrence_order() {
        let mut seen = vec![p(0, 0, 0)];
        let fresh = take_new_point_family(
            vec![p(1, 0, 0), p(0, 0, 0), p(2, 0, 0), p(1, 0, 0)],
            &mut seen,
        );

        assert_eq!(fresh, vec![p(1, 0, 0), p(2, 0, 0)]);
        assert_eq!(seen, vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)]);
    }

    #[test]
    fn shifted_target_seed_families_preserve_direct_report_witness_and_skip_later_duplicates() {
        let witness = p(1, 1, 1);
        let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
            dedupe_shifted_target_seed_families(
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
    fn shifted_target_seed_families_with_report_seed_promote_report_witness_to_shifted_root() {
        let witness = p(1, 1, 1);
        let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
            shifted_target_seed_families_with_report_seed(
                Some(&witness),
                Vec::new(),
                vec![witness.clone(), p(2, 1, 1)],
                vec![witness.clone(), p(3, 1, 1)],
            );

        assert_eq!(strict_seeds, vec![witness]);
        assert_eq!(shifted_vertices, vec![p(2, 1, 1)]);
        assert_eq!(shifted_geometry_seeds, vec![p(3, 1, 1)]);
    }

    #[test]
    fn support_shifted_target_seed_families_keep_one_strict_root_after_certified_direct_target() {
        let witness = p(1, 1, 1);
        let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
            support_shifted_target_seed_families(
                Some(&witness),
                vec![witness.clone(), p(2, 1, 1)],
                vec![p(3, 1, 1)],
                vec![p(4, 1, 1)],
                &[ReferenceTarget::axis_defined(p(2, 1, 1))],
            );

        assert_eq!(strict_seeds, vec![witness]);
        assert_eq!(shifted_vertices, vec![p(3, 1, 1)]);
        assert_eq!(shifted_geometry_seeds, vec![p(4, 1, 1)]);
    }

    #[test]
    fn support_shifted_target_seed_families_fall_back_to_first_certified_direct_target() {
        let direct_target_point = p(2, 1, 1);
        let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
            support_shifted_target_seed_families(
                None,
                vec![direct_target_point.clone(), p(5, 1, 1)],
                vec![p(3, 1, 1)],
                vec![p(4, 1, 1)],
                &[ReferenceTarget::axis_defined(direct_target_point.clone())],
            );

        assert_eq!(strict_seeds, vec![direct_target_point]);
        assert_eq!(shifted_vertices, vec![p(3, 1, 1)]);
        assert_eq!(shifted_geometry_seeds, vec![p(4, 1, 1)]);
    }

    #[test]
    fn support_shifted_target_seed_families_keep_full_strict_family_without_certified_direct_target()
     {
        let strict_seed = p(2, 1, 1);
        let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
            support_shifted_target_seed_families(
                None,
                vec![strict_seed.clone()],
                vec![p(3, 1, 1)],
                vec![p(4, 1, 1)],
                &[ReferenceTarget::axis_defined_fallback(p(7, 7, 7))],
            );

        assert_eq!(strict_seeds, vec![strict_seed]);
        assert_eq!(shifted_vertices, vec![p(3, 1, 1)]);
        assert_eq!(shifted_geometry_seeds, vec![p(4, 1, 1)]);
    }

    #[test]
    fn point_seed_family_search_failure_allows_later_shifted_seeds_after_unknown_strict_family() {
        assert!(!point_seed_family_search_failed_without_any_seed(
            &[],
            &[p(1, 1, 1)],
            &[],
            true,
        ));
        assert!(!point_seed_family_search_failed_without_any_seed(
            &[],
            &[],
            &[p(2, 2, 2)],
            true,
        ));
    }

    #[test]
    fn point_seed_family_search_failure_reports_unknown_only_when_every_seed_family_is_empty() {
        assert!(point_seed_family_search_failed_without_any_seed(
            &[],
            &[],
            &[],
            true,
        ));
        assert!(!point_seed_family_search_failed_without_any_seed(
            &[p(3, 3, 3)],
            &[],
            &[],
            true,
        ));
    }

    #[test]
    fn projected_escape_target_family_keeps_same_point_report_witness_definitions() {
        let point = p(1, 2, 3);
        let halfspaces = vec![
            axis_halfspace(0, true, r(1)),
            axis_halfspace(0, false, r(1)),
            axis_halfspace(1, true, r(2)),
            axis_halfspace(1, false, r(2)),
            axis_halfspace(2, true, r(3)),
            axis_halfspace(2, false, r(3)),
            LimitPlane3::new(p(1, 1, 1), r(-6)),
            LimitPlane3::new(p(-1, -1, -1), r(6)),
        ];
        let mut saw_unknown = false;

        let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
            &halfspaces,
            &[ReferenceTarget::axis_defined(point.clone())],
            Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
                point.clone(),
                [None, None, None],
            )),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            &mut saw_unknown,
            |_seed| Ok(Vec::new()),
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(targets.len(), 1);
        assert!(targets[0].uncertified_definition_fallback);
        assert!(
            targets[0]
                .definitions
                .iter()
                .any(definition_uses_non_axis_plane)
        );
    }

    #[test]
    fn projected_escape_target_family_keeps_same_point_direct_seed_definitions() {
        let point = p(1, 2, 3);
        let halfspaces = vec![
            axis_halfspace(0, true, r(1)),
            axis_halfspace(0, false, r(1)),
            axis_halfspace(1, true, r(2)),
            axis_halfspace(1, false, r(2)),
            axis_halfspace(2, true, r(3)),
            axis_halfspace(2, false, r(3)),
            LimitPlane3::new(p(1, 1, 1), r(-6)),
            LimitPlane3::new(p(-1, -1, -1), r(6)),
        ];
        let mut saw_unknown = false;

        let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
            &halfspaces,
            &[ReferenceTarget::axis_defined(point.clone())],
            None,
            vec![point],
            Vec::new(),
            Vec::new(),
            &mut saw_unknown,
            |_seed| Ok(Vec::new()),
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(targets.len(), 1);
        assert!(targets[0].uncertified_definition_fallback);
        assert!(
            targets[0]
                .definitions
                .iter()
                .any(definition_uses_non_axis_plane)
        );
    }

    #[test]
    fn projected_escape_target_family_keeps_same_point_report_witness_direct_seed_definitions() {
        let point = p(1, 2, 3);
        let halfspaces = vec![
            axis_halfspace(0, true, r(1)),
            axis_halfspace(0, false, r(1)),
            axis_halfspace(1, true, r(2)),
            axis_halfspace(1, false, r(2)),
            axis_halfspace(2, true, r(3)),
            axis_halfspace(2, false, r(3)),
            LimitPlane3::new(p(1, 1, 1), r(-6)),
            LimitPlane3::new(p(-1, -1, -1), r(6)),
        ];
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(point.clone(), [None, None, None]);
        let mut saw_unknown = false;

        let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
            &halfspaces,
            &[ReferenceTarget::axis_defined(point.clone())],
            Some(&report),
            vec![point],
            Vec::new(),
            Vec::new(),
            &mut saw_unknown,
            |_seed| Ok(Vec::new()),
        )
        .unwrap();

        assert!(saw_unknown);
        assert_eq!(targets.len(), 1);
        assert!(targets[0].uncertified_definition_fallback);
        assert!(
            targets[0]
                .definitions
                .iter()
                .any(definition_uses_non_axis_plane)
        );
    }

    #[test]
    fn shifted_projected_escape_target_family_search_skips_duplicate_seed_sources() {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut targets = Vec::new();
        let visited = std::cell::RefCell::new(Vec::new());

        collect_shifted_projected_escape_target_families(
            &mut targets,
            None,
            vec![first.clone(), second.clone()],
            vec![second.clone(), first.clone()],
            Vec::new(),
            |_candidate| Ok(true),
            |_candidate| Ok(None),
            |candidate| {
                visited.borrow_mut().push(candidate.clone());
                Ok(Some(ReferenceTarget::axis_defined(candidate.clone())))
            },
        )
        .unwrap();

        assert_eq!(visited.into_inner(), vec![first.clone(), second.clone()]);
        assert_eq!(
            targets
                .into_iter()
                .map(|target| target.point)
                .collect::<Vec<_>>(),
            vec![first, second]
        );
    }

    #[test]
    fn shifted_projected_escape_target_family_search_promotes_report_witness_to_shifted_root() {
        let witness = p(1, 2, 3);
        let mut targets = Vec::new();
        let visited = std::cell::RefCell::new(Vec::new());

        collect_shifted_projected_escape_target_families(
            &mut targets,
            Some(&witness),
            Vec::new(),
            vec![witness.clone()],
            Vec::new(),
            |_candidate| Ok(true),
            |_candidate| Ok(None),
            |candidate| {
                visited.borrow_mut().push(candidate.clone());
                Ok(Some(ReferenceTarget::axis_defined(p(9, 9, 9))))
            },
        )
        .unwrap();

        assert_eq!(visited.into_inner(), vec![witness]);
        assert!(targets.iter().any(|target| target.point == p(9, 9, 9)));
    }

    #[test]
    fn shifted_projected_escape_target_family_search_backtracks_after_uncertified_earlier_family() {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut targets = Vec::new();

        collect_shifted_projected_escape_target_families(
            &mut targets,
            None,
            vec![first.clone()],
            vec![first, second.clone()],
            Vec::new(),
            |_candidate| Ok(true),
            |_candidate| Ok(None),
            |candidate| {
                if *candidate == p(2, 2, 2) {
                    Ok(Some(ReferenceTarget::axis_defined(candidate.clone())))
                } else {
                    Err(crate::error::HypermeshError::UnknownClassification)
                }
            },
        )
        .unwrap();

        assert_eq!(
            targets
                .into_iter()
                .map(|target| target.point)
                .collect::<Vec<_>>(),
            vec![second]
        );
    }

    #[test]
    fn winding_reachability_prunes_difference_when_other_mesh_cannot_reach_zero() {
        let mut first = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        first.delta_w = vec![0, 1];
        let mut second = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
        second.delta_w = vec![0, 1];

        assert!(
            can_discard_by_winding_reachability(BooleanOp::Difference, &[1, 3], &[first, second])
                .unwrap()
        );
    }

    #[test]
    fn winding_reachability_keeps_difference_when_other_mesh_can_reach_zero() {
        let mut first = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        first.delta_w = vec![0, 1];

        assert!(
            !can_discard_by_winding_reachability(BooleanOp::Difference, &[1, 1], &[first]).unwrap()
        );
    }

    #[test]
    fn winding_reachability_prunes_correlated_difference_when_zero_is_not_jointly_reachable() {
        let mut correlated = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        correlated.delta_w = vec![1, 1];

        assert!(
            can_discard_by_winding_reachability(BooleanOp::Difference, &[1, 1], &[correlated])
                .unwrap()
        );
    }

    #[test]
    fn cached_winding_reachability_reuses_transition_multiset_across_polygon_geometry() {
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);

        let mut first = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        first.delta_w = vec![1, 1];
        let mut second = make_triangle(&p(0, 0, 2), &p(1, 0, 2), &p(0, 1, 2), 1, 0);
        second.delta_w = vec![0, -1];
        let first_polygons = vec![first.clone(), second.clone()];

        let mut third = make_triangle(&p(3, 0, 0), &p(4, 0, 0), &p(3, 1, 0), 2, 0);
        third.delta_w = vec![0, -1];
        let mut fourth = make_triangle(&p(3, 0, 2), &p(4, 0, 2), &p(3, 1, 2), 3, 0);
        fourth.delta_w = vec![1, 1];
        let second_polygons = vec![third, fourth];

        let first_result = cached_winding_reachability_with(
            &cache,
            BooleanOp::Difference,
            &[1, 1],
            &first_polygons,
            || {
                calls.set(calls.get() + 1);
                can_discard_by_winding_reachability(BooleanOp::Difference, &[1, 1], &first_polygons)
            },
        )
        .unwrap();
        let second_result = cached_winding_reachability_with(
            &cache,
            BooleanOp::Difference,
            &[1, 1],
            &second_polygons,
            || {
                calls.set(calls.get() + 1);
                can_discard_by_winding_reachability(
                    BooleanOp::Difference,
                    &[1, 1],
                    &second_polygons,
                )
            },
        )
        .unwrap();

        assert_eq!(first_result, second_result);
        assert_eq!(calls.get(), 1);
    }

    #[test]
    fn cached_winding_reachability_distinguishes_reference_winding_context() {
        let cache = RefCell::new(Vec::new());
        let calls = std::cell::Cell::new(0);

        let mut first_polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        first_polygon.delta_w = vec![0, 1];
        let mut second_polygon = first_polygon.clone();
        second_polygon.mesh_index = 1;

        cached_winding_reachability_with(
            &cache,
            BooleanOp::Difference,
            &[1, 3],
            &[first_polygon.clone()],
            || {
                calls.set(calls.get() + 1);
                can_discard_by_winding_reachability(
                    BooleanOp::Difference,
                    &[1, 3],
                    &[first_polygon.clone()],
                )
            },
        )
        .unwrap();
        cached_winding_reachability_with(
            &cache,
            BooleanOp::Difference,
            &[1, 1],
            &[second_polygon.clone()],
            || {
                calls.set(calls.get() + 1);
                can_discard_by_winding_reachability(
                    BooleanOp::Difference,
                    &[1, 1],
                    &[second_polygon.clone()],
                )
            },
        )
        .unwrap();

        assert_eq!(calls.get(), 2);
    }

    #[test]
    fn support_plane_cell_target_finds_strict_point_in_closed_feasible_cell() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![
            support_only_polygon(Plane::from_coefficients(r(-1), r(0), r(0), q(7, 2))),
            support_only_polygon(Plane::from_coefficients(r(1), r(0), r(0), q(-13, 2))),
            support_only_polygon(Plane::axis_aligned(0, r(5))),
        ];

        let target = support_plane_cell_target(&bounds, &polygons)
            .unwrap()
            .expect("closed feasible support cell should produce a strict interior point");

        assert!(point_strictly_inside_bounds(&target.point, &bounds).unwrap());
        assert!(!point_lies_on_any_support_plane(&target.point, &polygons).unwrap());
        assert!(compare_real(&target.point.x, &q(7, 2)).unwrap().is_gt());
        assert!(compare_real(&target.point.x, &q(13, 2)).unwrap().is_lt());
        assert!(
            target
                .definitions
                .iter()
                .any(definition_uses_non_axis_plane)
        );
    }

    #[test]
    fn support_plane_cell_search_backtracks_after_leaf_rejection() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(5)))];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut rejected_first_leaf = false;
        let mut accept = |_halfspaces: &[LimitPlane3],
                          report: Option<hyperlimit::HalfspaceFeasibilityReport>|
         -> HypermeshResult<Option<Point3>> {
            let Some(report) = report else {
                return Ok(None);
            };
            let Some(witness) = report.witness else {
                return Ok(None);
            };
            if compare_real(&witness.x, &r(5))?.is_lt() {
                rejected_first_leaf = true;
                return Ok(None);
            }
            Ok(Some(witness))
        };

        let target =
            support_plane_cell_search_from(&bounds, &polygons, 0, &mut halfspaces, &mut accept)
                .unwrap()
                .expect("search should continue after the first accepted leaf rejects");

        assert!(rejected_first_leaf);
        assert!(compare_real(&target.x, &r(5)).unwrap().is_gt());
    }

    #[test]
    fn support_plane_cell_search_backtracks_after_uncertified_leaf() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(5)))];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut rejected_first_leaf = false;
        let mut accept = |_halfspaces: &[LimitPlane3],
                          report: Option<hyperlimit::HalfspaceFeasibilityReport>|
         -> HypermeshResult<Option<Point3>> {
            let Some(report) = report else {
                return Ok(None);
            };
            let Some(witness) = report.witness else {
                return Ok(None);
            };
            if compare_real(&witness.x, &r(5))?.is_lt() {
                rejected_first_leaf = true;
                return Err(crate::error::HypermeshError::UnknownClassification);
            }
            Ok(Some(witness))
        };

        let target =
            support_plane_cell_search_from(&bounds, &polygons, 0, &mut halfspaces, &mut accept)
                .unwrap()
                .expect("search should continue after an uncertified leaf branch");

        assert!(rejected_first_leaf);
        assert!(compare_real(&target.x, &r(5)).unwrap().is_gt());
    }

    #[test]
    fn support_plane_cell_search_reports_unknown_if_all_branches_are_uncertified() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(5)))];
        let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut accept = |_halfspaces: &[LimitPlane3],
                          _report: Option<hyperlimit::HalfspaceFeasibilityReport>|
         -> HypermeshResult<Option<Point3>> {
            Err(crate::error::HypermeshError::UnknownClassification)
        };

        let err =
            support_plane_cell_search_from(&bounds, &polygons, 0, &mut halfspaces, &mut accept)
                .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn support_plane_cell_reference_traces_certified_winding() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(5))),
            support_only_polygon(Plane::axis_aligned(1, r(5))),
            support_only_polygon(Plane::axis_aligned(2, r(5))),
        ];

        let (target, winding) = support_plane_cell_reference(
            &p(-1, -1, -1),
            &axis_defs(&p(-1, -1, -1)),
            &[7],
            &bounds,
            &polygons,
        )
        .unwrap()
        .expect("strict support cell target should trace from old reference");

        assert_eq!(winding, vec![7]);
        assert!(point_strictly_inside_bounds(&target.point, &bounds).unwrap());
        assert!(!point_lies_on_any_support_plane(&target.point, &polygons).unwrap());
        assert!(!target.definitions.is_empty());
    }

    #[test]
    fn support_plane_cell_reference_retains_active_plane_definitions() {
        let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
        let polygons = vec![
            support_only_polygon(Plane::axis_aligned(0, r(5))),
            support_only_polygon(Plane::axis_aligned(1, r(5))),
            support_only_polygon(Plane::axis_aligned(2, r(5))),
            support_only_polygon(Plane::from_coefficients(r(1), r(1), r(1), r(-15))),
        ];

        let (target, winding) = support_plane_cell_reference(
            &p(-1, -1, -1),
            &axis_defs(&p(-1, -1, -1)),
            &[3],
            &bounds,
            &polygons,
        )
        .unwrap()
        .expect("support-cell witness should be traceable");

        assert_eq!(winding, vec![3]);
        assert!(
            target
                .definitions
                .iter()
                .any(definition_uses_non_axis_plane)
        );
        for definition in &target.definitions {
            assert_eq!(affine_from_planes(definition).unwrap(), target.point);
        }
    }

    #[test]
    fn reference_target_trace_search_backtracks_after_uncertified_target() {
        let first = ReferenceTarget::axis_defined(p(1, 1, 1));
        let second = ReferenceTarget::axis_defined(p(2, 2, 2));

        let found = trace_reference_targets_backtracking_unknown(
            vec![first.clone(), second.clone()],
            &[],
            |target| {
                if target == &first {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(Some(vec![31]))
                }
            },
        )
        .unwrap();

        assert_eq!(found, Some((second, vec![31])));
    }

    #[test]
    fn reference_target_trace_search_reports_unknown_if_all_targets_are_uncertified() {
        let first = ReferenceTarget::axis_defined(p(1, 1, 1));
        let second = ReferenceTarget::axis_defined(p(2, 2, 2));

        let err =
            trace_reference_targets_backtracking_unknown(vec![first, second], &[], |_target| {
                Err(crate::error::HypermeshError::UnknownClassification)
            })
            .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn reference_target_trace_search_skips_fallback_target_even_when_trace_succeeds() {
        let fallback = ReferenceTarget::axis_defined_fallback(p(1, 1, 1));
        let certified = ReferenceTarget::axis_defined(p(2, 2, 2));

        let found = trace_reference_targets_backtracking_unknown(
            vec![fallback.clone(), certified.clone()],
            &[],
            |target| {
                if target == &fallback {
                    Ok(Some(vec![31]))
                } else {
                    Ok(Some(vec![37]))
                }
            },
        )
        .unwrap();

        assert_eq!(found, Some((certified, vec![37])));
    }

    #[test]
    fn reference_target_trace_search_reports_unknown_when_only_fallback_target_traces() {
        let fallback = ReferenceTarget::axis_defined_fallback(p(1, 1, 1));

        let err = trace_reference_targets_backtracking_unknown(vec![fallback], &[], |_target| {
            Ok(Some(vec![31]))
        })
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn reference_target_trace_search_skips_support_surface_targets_before_trace() {
        let polygon = support_only_polygon(Plane::axis_aligned(0, r(2)));
        let surface = ReferenceTarget::axis_defined(p(2, 1, 1));
        let interior = ReferenceTarget::axis_defined(p(1, 1, 1));
        let mut trace_calls = 0;

        let found = trace_reference_targets_backtracking_unknown(
            vec![surface, interior.clone()],
            &[polygon],
            |target| {
                trace_calls += 1;
                assert_eq!(target, &interior);
                Ok(Some(vec![13]))
            },
        )
        .unwrap();

        assert_eq!(trace_calls, 1);
        assert_eq!(found, Some((interior, vec![13])));
    }

    #[test]
    fn reference_target_trace_search_tries_later_target_after_boundary_support_surface_contact() {
        let polygon = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 0, 4), 0, 0);
        let boundary = ReferenceTarget::axis_defined(p(2, 0, 2));
        let interior = ReferenceTarget::axis_defined(p(1, 1, 1));
        let mut trace_calls = 0;

        let found = trace_reference_targets_backtracking_unknown(
            vec![boundary, interior.clone()],
            &[polygon],
            |target| {
                trace_calls += 1;
                assert_eq!(target, &interior);
                Ok(Some(vec![29]))
            },
        )
        .unwrap();

        assert_eq!(trace_calls, 1);
        assert_eq!(found, Some((interior, vec![29])));
    }

    #[test]
    fn reference_target_trace_search_reports_unknown_when_boundary_support_surface_contact_blocks_only_target()
     {
        let polygon = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 0, 4), 0, 0);
        let boundary = ReferenceTarget::axis_defined(p(2, 0, 2));

        let err =
            trace_reference_targets_backtracking_unknown(vec![boundary], &[polygon], |_target| {
                Ok(Some(vec![29]))
            })
            .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn reference_target_trace_search_reports_unknown_when_fallback_surface_target_is_skipped() {
        let polygon = support_only_polygon(Plane::axis_aligned(0, r(2)));
        let surface = ReferenceTarget::axis_defined_fallback(p(2, 1, 1));

        let err =
            trace_reference_targets_backtracking_unknown(vec![surface], &[polygon], |_target| {
                Ok(Some(vec![13]))
            })
            .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn reference_target_trace_search_reuses_equivalent_support_surface_queries() {
        let first = ReferenceTarget::with_definitions(
            p(2, 1, 1),
            vec![[
                Plane::axis_aligned(0, r(2)),
                Plane::axis_aligned(1, r(1)),
                Plane::axis_aligned(2, r(1)),
            ]],
        );
        let second = ReferenceTarget::axis_defined(p(2, 1, 1));
        let surface_calls = std::cell::Cell::new(0);
        let mut surface_cache = Vec::new();
        let mut validity_cache = Vec::new();

        let found = trace_reference_targets_backtracking_unknown_with_query_caches(
            vec![first, second],
            &mut surface_cache,
            &mut validity_cache,
            None,
            &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
            &mut |point| {
                surface_calls.set(surface_calls.get() + 1);
                Ok(*point == p(2, 1, 1))
            },
            &mut |_point| Ok(true),
            |_target| Ok(Some(vec![17])),
        )
        .unwrap();

        assert_eq!(found, None);
        assert_eq!(surface_calls.get(), 1);
    }

    #[test]
    fn reference_target_trace_search_reuses_reference_validity_queries_after_surface_passes() {
        let first = ReferenceTarget::with_definitions(
            p(1, 1, 1),
            vec![[
                Plane::axis_aligned(0, r(1)),
                Plane::axis_aligned(1, r(1)),
                Plane::axis_aligned(2, r(1)),
            ]],
        );
        let second = ReferenceTarget::axis_defined(p(1, 1, 1));
        let validity_calls = std::cell::Cell::new(0);
        let mut surface_cache = Vec::new();
        let mut validity_cache = Vec::new();

        let found = trace_reference_targets_backtracking_unknown_with_query_caches(
            vec![first, second],
            &mut surface_cache,
            &mut validity_cache,
            None,
            &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
            &mut |_point| Ok(false),
            &mut |point| {
                validity_calls.set(validity_calls.get() + 1);
                Ok(*point == p(1, 1, 1))
            },
            |_target| Ok(None),
        )
        .unwrap();

        assert_eq!(found, None);
        assert_eq!(validity_calls.get(), 1);
    }

    #[test]
    fn reference_target_trace_search_reuses_reference_validity_queries_across_calls() {
        let target = ReferenceTarget::axis_defined(p(1, 1, 1));
        let validity_calls = std::cell::Cell::new(0);
        let mut surface_cache = Vec::new();
        let mut validity_cache = Vec::new();

        let first = trace_reference_targets_backtracking_unknown_with_query_caches(
            vec![target.clone()],
            &mut surface_cache,
            &mut validity_cache,
            None,
            &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
            &mut |_point| Ok(false),
            &mut |point| {
                validity_calls.set(validity_calls.get() + 1);
                Ok(*point == p(1, 1, 1))
            },
            |_target| Ok(None),
        )
        .unwrap();

        let second = trace_reference_targets_backtracking_unknown_with_query_caches(
            vec![target],
            &mut surface_cache,
            &mut validity_cache,
            None,
            &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
            &mut |_point| Ok(false),
            &mut |point| {
                validity_calls.set(validity_calls.get() + 1);
                Ok(*point == p(1, 1, 1))
            },
            |_target| Ok(None),
        )
        .unwrap();

        assert_eq!(first, None);
        assert_eq!(second, None);
        assert_eq!(validity_calls.get(), 1);
    }

    #[test]
    fn reference_target_trace_search_keeps_distinct_reference_validity_bounds_separate() {
        let target = ReferenceTarget::axis_defined(p(1, 1, 1));
        let validity_calls = std::cell::Cell::new(0);
        let mut surface_cache = Vec::new();
        let mut validity_cache = Vec::new();

        trace_reference_targets_backtracking_unknown_with_query_caches(
            vec![target.clone()],
            &mut surface_cache,
            &mut validity_cache,
            None,
            &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
            &mut |_point| Ok(false),
            &mut |point| {
                validity_calls.set(validity_calls.get() + 1);
                Ok(*point == p(1, 1, 1))
            },
            |_target| Ok(None),
        )
        .unwrap();

        trace_reference_targets_backtracking_unknown_with_query_caches(
            vec![target],
            &mut surface_cache,
            &mut validity_cache,
            None,
            &Aabb::new(p(0, 0, 0), p(5, 4, 4)),
            &mut |_point| Ok(false),
            &mut |point| {
                validity_calls.set(validity_calls.get() + 1);
                Ok(*point == p(1, 1, 1))
            },
            |_target| Ok(None),
        )
        .unwrap();

        assert_eq!(validity_calls.get(), 2);
    }

    #[test]
    fn reference_target_trace_search_reuses_support_surface_queries_across_calls() {
        let target = ReferenceTarget::axis_defined(p(2, 1, 1));
        let surface_calls = std::cell::Cell::new(0);
        let mut surface_cache = Vec::new();

        let first = trace_reference_targets_backtracking_unknown_with_surface_cache(
            vec![target.clone()],
            &mut surface_cache,
            &mut |point| {
                surface_calls.set(surface_calls.get() + 1);
                Ok(*point == p(2, 1, 1))
            },
            |_target| Ok(Some(vec![17])),
        )
        .unwrap();

        let second = trace_reference_targets_backtracking_unknown_with_surface_cache(
            vec![target],
            &mut surface_cache,
            &mut |point| {
                surface_calls.set(surface_calls.get() + 1);
                Ok(*point == p(2, 1, 1))
            },
            |_target| Ok(Some(vec![17])),
        )
        .unwrap();

        assert_eq!(first, None);
        assert_eq!(second, None);
        assert_eq!(surface_calls.get(), 1);
    }

    #[test]
    fn reference_target_trace_search_tries_later_target_after_uncertified_surface_query() {
        let first = ReferenceTarget::axis_defined(p(1, 1, 1));
        let second = ReferenceTarget::axis_defined(p(2, 2, 2));
        let mut surface_cache = Vec::new();
        let mut validity_cache = Vec::new();

        let found = trace_reference_targets_backtracking_unknown_with_query_caches(
            vec![first.clone(), second.clone()],
            &mut surface_cache,
            &mut validity_cache,
            None,
            &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
            &mut |point| {
                if *point == first.point {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(false)
                }
            },
            &mut |_point| Ok(true),
            |target| {
                assert_eq!(target, &second);
                Ok(Some(vec![23]))
            },
        )
        .unwrap();

        assert_eq!(found, Some((second, vec![23])));
    }

    #[test]
    fn reference_target_trace_search_reports_unknown_when_surface_query_is_uncertified_and_later_targets_fail()
     {
        let first = ReferenceTarget::axis_defined(p(1, 1, 1));
        let second = ReferenceTarget::axis_defined(p(2, 2, 2));
        let mut surface_cache = Vec::new();
        let mut validity_cache = Vec::new();

        let err = trace_reference_targets_backtracking_unknown_with_query_caches(
            vec![first.clone(), second],
            &mut surface_cache,
            &mut validity_cache,
            None,
            &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
            &mut |point| {
                if *point == first.point {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(false)
                }
            },
            &mut |_point| Ok(true),
            |_target| Ok(None),
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn reference_target_trace_search_tries_later_target_after_uncertified_reference_validity_query()
    {
        let first = ReferenceTarget::axis_defined(p(1, 1, 1));
        let second = ReferenceTarget::axis_defined(p(2, 2, 2));
        let mut surface_cache = Vec::new();
        let mut validity_cache = Vec::new();

        let found = trace_reference_targets_backtracking_unknown_with_query_caches(
            vec![first.clone(), second.clone()],
            &mut surface_cache,
            &mut validity_cache,
            None,
            &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
            &mut |_point| Ok(false),
            &mut |point| {
                if *point == first.point {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(true)
                }
            },
            |target| {
                assert_eq!(target, &second);
                Ok(Some(vec![29]))
            },
        )
        .unwrap();

        assert_eq!(found, Some((second, vec![29])));
    }

    #[test]
    fn reference_target_trace_search_tries_later_target_after_boundary_local_surface_validity_query()
     {
        let first = ReferenceTarget::axis_defined(p(2, 1, 2));
        let second = ReferenceTarget::axis_defined(p(1, 1, 1));
        let mut wall = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 2, 4), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let mut surface_cache = Vec::new();
        let mut validity_cache = Vec::new();

        let found = trace_reference_targets_backtracking_unknown_with_query_caches(
            vec![first, second.clone()],
            &mut surface_cache,
            &mut validity_cache,
            None,
            &bounds,
            &mut |_point| Ok(false),
            &mut |point| is_certified_valid_reference_for_bounds(point, &bounds, &[wall.clone()]),
            |target| {
                assert_eq!(target, &second);
                Ok(Some(vec![31]))
            },
        )
        .unwrap();

        assert_eq!(found, Some((second, vec![31])));
    }

    #[test]
    fn unique_overlap_edge_planes_preserve_first_occurrence_and_skip_inverted_duplicates() {
        let x0 = Plane::axis_aligned(0, r(0));
        let y0 = Plane::axis_aligned(1, r(0));
        let y1 = Plane::axis_aligned(1, r(1));
        let support = Plane::axis_aligned(2, r(0));
        let intersections = vec![
            PairwiseIntersection {
                kind: PairwiseIntersectionType::Overlap,
                segment: None,
                overlap: Some(OverlapInfo {
                    other_polygon_idx: 0,
                    other_edges: vec![x0.clone(), y0.clone()],
                    other_support: support.clone(),
                }),
            },
            PairwiseIntersection {
                kind: PairwiseIntersectionType::Overlap,
                segment: None,
                overlap: Some(OverlapInfo {
                    other_polygon_idx: 1,
                    other_edges: vec![x0.inverted(), y1.clone()],
                    other_support: support,
                }),
            },
        ];

        assert_eq!(unique_overlap_edge_planes(&intersections), vec![x0, y0, y1]);
    }

    #[test]
    fn support_cell_targets_include_direct_strict_feasibility_witness() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let direct = p(2, 1, 3);
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(direct.clone(), [None, None, None]);

        let targets = strict_support_cell_targets(&bounds, &halfspaces, &report).unwrap();

        assert!(targets.iter().any(|target| target.point == direct));
        assert!(
            targets
                .iter()
                .find(|target| target.point == direct)
                .is_some_and(|target| !target.definitions.is_empty())
        );
    }

    #[test]
    fn strict_projected_cell_seeds_include_strict_feasible_vertices() {
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

        let seeds = strict_projected_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

        assert_eq!(seeds, vec![Point3::new(r(1), r(2), r(3))]);
    }

    #[test]
    fn strict_projected_cell_seeds_include_strict_geometry_seeds() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(Point3::origin(), [None, None, None]);
        let triangle_center = Point3::new(q(4, 3), q(4, 3), q(8, 3));
        let tetra_center = p(1, 1, 1);

        let seeds = strict_projected_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

        assert!(
            point_strictly_inside_projected_cell(&triangle_center, &bounds, &halfspaces).unwrap()
        );
        assert!(point_strictly_inside_projected_cell(&tetra_center, &bounds, &halfspaces).unwrap());
        assert!(seeds.iter().any(|seed| seed == &triangle_center));
        assert!(seeds.iter().any(|seed| seed == &tetra_center));
    }

    #[test]
    fn projected_cell_seed_families_track_unknown_after_boundary_vertex_candidate() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut saw_unknown = false;

        let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
            projected_cell_seed_families_from_optional_report(
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
    fn strict_projected_cell_seeds_include_strict_geometry_seeds_without_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let triangle_center = Point3::new(q(4, 3), q(4, 3), q(8, 3));
        let tetra_center = p(1, 1, 1);

        let seeds =
            strict_projected_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

        assert!(
            point_strictly_inside_projected_cell(&triangle_center, &bounds, &halfspaces).unwrap()
        );
        assert!(point_strictly_inside_projected_cell(&tetra_center, &bounds, &halfspaces).unwrap());
        assert!(seeds.iter().any(|seed| seed == &triangle_center));
        assert!(seeds.iter().any(|seed| seed == &tetra_center));
    }

    #[test]
    fn strict_projected_cell_seeds_include_strict_edge_midpoints() {
        let (bounds, halfspaces, midpoint) = quadrilateral_reference_cell_fixture();

        let seeds =
            strict_projected_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

        assert!(point_strictly_inside_projected_cell(&midpoint, &bounds, &halfspaces).unwrap());
        assert!(seeds.iter().any(|seed| seed == &midpoint));
    }

    #[test]
    fn strict_projected_cell_seeds_include_strict_five_vertex_centroids() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let five_vertex_center = Point3::new(q(8, 5), q(8, 5), q(8, 5));

        let seeds =
            strict_projected_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

        assert!(
            point_strictly_inside_projected_cell(&five_vertex_center, &bounds, &halfspaces)
                .unwrap()
        );
        assert!(seeds.iter().any(|seed| seed == &five_vertex_center));
    }

    #[test]
    fn point3_seed_collection_backtracks_after_uncertified_candidate() {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut points = Vec::new();

        extend_point3_backtracking_unknown(
            &mut points,
            vec![first.clone(), second.clone()],
            |candidate| {
                if candidate == &first {
                    Err(crate::error::HypermeshError::UnknownClassification)
                } else {
                    Ok(candidate == &second)
                }
            },
        )
        .unwrap();

        assert_eq!(points, vec![second]);
    }

    #[test]
    fn point3_seed_collection_reports_unknown_if_all_candidates_are_uncertified() {
        let first = p(1, 1, 1);
        let second = p(2, 2, 2);
        let mut points = Vec::new();

        let err =
            extend_point3_backtracking_unknown(&mut points, vec![first, second], |_candidate| {
                Err(crate::error::HypermeshError::UnknownClassification)
            })
            .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn feasible_support_cell_vertices_backtrack_after_uncertified_candidate() {
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

        let vertices = feasible_support_cell_vertices_with_contains(&halfspaces, |point, _| {
            if point == &first {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(point == &second)
            }
        })
        .unwrap();

        assert_eq!(vertices, vec![second]);
    }

    #[test]
    fn feasible_support_cell_vertex_family_tracks_unknown_after_later_vertex() {
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

        let family = feasible_support_cell_vertex_family_with_contains(&halfspaces, |point, _| {
            if point == &first {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(point == &second)
            }
        })
        .unwrap();

        assert_eq!(family.points, vec![second]);
        assert!(family.saw_unknown);
    }

    #[test]
    fn feasible_support_cell_vertices_report_unknown_if_all_candidates_are_uncertified() {
        let halfspaces = vec![
            axis_halfspace(0, true, r(0)),
            axis_halfspace(0, false, r(0)),
            axis_halfspace(1, true, r(0)),
            axis_halfspace(1, false, r(0)),
            axis_halfspace(2, true, r(0)),
            axis_halfspace(2, false, r(1)),
        ];

        let err = feasible_support_cell_vertices_with_contains(&halfspaces, |_point, _| {
            Err(crate::error::HypermeshError::UnknownClassification)
        })
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    }

    #[test]
    fn support_cell_geometry_seed_candidates_from_vertices_matches_direct_query() {
        let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
        let vertices = feasible_support_cell_vertices(&halfspaces).unwrap();

        let from_vertices = support_cell_geometry_seed_candidates_from_vertices(&vertices).unwrap();
        let direct = support_cell_geometry_seed_candidates(&halfspaces).unwrap();

        assert_eq!(from_vertices, direct);
    }

    #[test]
    fn point3_centroid_subset_family_tracks_unknown_after_later_centroid() {
        let vertices = vec![p(0, 0, 0), p(2, 0, 0), p(4, 0, 0)];
        let blocked_subset = vec![vertices[0].clone(), vertices[1].clone()];

        let family = point3_centroid_subset_family_from_vertices_with(&vertices, |subset| {
            if subset == blocked_subset.as_slice() {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                point3_centroid(subset)
            }
        })
        .unwrap();

        assert!(family.saw_unknown);
        assert!(!family.points.is_empty());
    }

    #[test]
    fn strict_projected_cell_targets_include_direct_strict_seed_targets() {
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

        let targets = strict_projected_cell_targets(&bounds, &halfspaces, &report).unwrap();

        assert!(
            targets
                .iter()
                .any(|target| target.point == Point3::new(r(1), r(2), r(3)))
        );
        assert!(
            targets
                .iter()
                .find(|target| target.point == Point3::new(r(1), r(2), r(3)))
                .is_some_and(|target| !target.definitions.is_empty())
        );
    }

    #[test]
    fn strict_projected_cell_targets_include_direct_strict_seed_targets_without_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = vec![
            axis_halfspace(0, true, r(1)),
            axis_halfspace(0, false, r(1)),
            axis_halfspace(1, true, r(2)),
            axis_halfspace(1, false, r(2)),
            axis_halfspace(2, true, r(3)),
            axis_halfspace(2, false, r(3)),
        ];

        let targets =
            strict_projected_cell_targets_from_optional_report(&bounds, &halfspaces, None).unwrap();

        assert!(
            targets
                .iter()
                .any(|target| target.point == Point3::new(r(1), r(2), r(3)))
        );
        assert!(
            targets
                .iter()
                .find(|target| target.point == Point3::new(r(1), r(2), r(3)))
                .is_some_and(|target| !target.definitions.is_empty())
        );
    }

    #[test]
    fn strict_support_cell_seeds_include_strict_feasible_vertices() {
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

        let seeds = strict_support_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

        assert_eq!(seeds, vec![Point3::new(r(1), r(2), r(3))]);
    }

    #[test]
    fn strict_support_cell_seeds_include_strict_geometry_seeds() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(Point3::origin(), [None, None, None]);
        let triangle_center = Point3::new(q(4, 3), q(4, 3), q(8, 3));
        let tetra_center = p(1, 1, 1);

        let seeds = strict_support_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

        assert!(
            point_strictly_inside_support_cell(&triangle_center, &bounds, &halfspaces).unwrap()
        );
        assert!(point_strictly_inside_support_cell(&tetra_center, &bounds, &halfspaces).unwrap());
        assert!(seeds.iter().any(|seed| seed == &triangle_center));
        assert!(seeds.iter().any(|seed| seed == &tetra_center));
    }

    #[test]
    fn support_cell_seed_families_track_unknown_after_boundary_vertex_candidate() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let mut saw_unknown = false;

        let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
            support_cell_seed_families_from_optional_report(
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
    fn strict_support_cell_seeds_include_strict_geometry_seeds_without_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let triangle_center = Point3::new(q(4, 3), q(4, 3), q(8, 3));
        let tetra_center = p(1, 1, 1);

        let seeds =
            strict_support_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

        assert!(
            point_strictly_inside_support_cell(&triangle_center, &bounds, &halfspaces).unwrap()
        );
        assert!(point_strictly_inside_support_cell(&tetra_center, &bounds, &halfspaces).unwrap());
        assert!(seeds.iter().any(|seed| seed == &triangle_center));
        assert!(seeds.iter().any(|seed| seed == &tetra_center));
    }

    #[test]
    fn strict_support_cell_seeds_include_strict_edge_midpoints() {
        let (bounds, halfspaces, midpoint) = quadrilateral_reference_cell_fixture();

        let seeds =
            strict_support_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

        assert!(point_strictly_inside_support_cell(&midpoint, &bounds, &halfspaces).unwrap());
        assert!(seeds.iter().any(|seed| seed == &midpoint));
    }

    #[test]
    fn strict_support_cell_seeds_include_strict_five_vertex_centroids() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let five_vertex_center = Point3::new(q(8, 5), q(8, 5), q(8, 5));

        let seeds =
            strict_support_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

        assert!(
            point_strictly_inside_support_cell(&five_vertex_center, &bounds, &halfspaces).unwrap()
        );
        assert!(seeds.iter().any(|seed| seed == &five_vertex_center));
    }

    #[test]
    fn support_cell_targets_try_shifted_targets_from_all_strict_seeds() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let direct = p(2, 1, 3);
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(direct.clone(), [None, None, None]);

        let seeds = strict_support_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();
        assert!(seeds.iter().any(|seed| seed == &direct));
        assert!(seeds.len() > 1);

        let targets = strict_support_cell_targets(&bounds, &halfspaces, &report).unwrap();

        assert!(targets.iter().any(|target| target.point == direct));
        assert!(
            targets
                .iter()
                .any(|target| { target.point == Point3::new(r(1), q(1, 2), q(3, 2)) })
        );
        assert!(targets.iter().any(|target| target.point != direct));
    }

    #[test]
    fn support_cell_targets_include_direct_strict_feasibility_witness_without_report() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = vec![
            axis_halfspace(0, true, r(1)),
            axis_halfspace(0, false, r(1)),
            axis_halfspace(1, true, r(2)),
            axis_halfspace(1, false, r(2)),
            axis_halfspace(2, true, r(3)),
            axis_halfspace(2, false, r(3)),
        ];

        let targets =
            strict_support_cell_targets_from_optional_report(&bounds, &halfspaces, None).unwrap();

        assert!(
            targets
                .iter()
                .any(|target| target.point == Point3::new(r(1), r(2), r(3)))
        );
        assert!(
            targets
                .iter()
                .find(|target| target.point == Point3::new(r(1), r(2), r(3)))
                .is_some_and(|target| !target.definitions.is_empty())
        );
    }

    #[test]
    fn shifted_support_cell_targets_try_all_shifted_strict_seeds() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let targets =
            shifted_support_cell_targets_from_seed(&bounds, &halfspaces, &p(2, 1, 3)).unwrap();

        assert!(
            targets
                .iter()
                .any(|target| { target.point == Point3::new(r(1), q(1, 2), q(3, 2)) })
        );
        assert!(targets.iter().all(|target| !target.definitions.is_empty()));
    }

    #[test]
    fn shifted_projected_cell_targets_from_geometry_seed_return_targets() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let targets =
            shifted_projected_cell_targets_from_seed(&bounds, &halfspaces, &p(1, 1, 1)).unwrap();

        assert!(!targets.is_empty());
        assert!(targets.iter().all(|target| {
            point_strictly_inside_projected_cell(&target.point, &bounds, &halfspaces).unwrap()
        }));
    }

    #[test]
    fn shifted_support_cell_targets_from_geometry_seed_return_targets() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

        let targets =
            shifted_support_cell_targets_from_seed(&bounds, &halfspaces, &p(1, 1, 1)).unwrap();

        assert!(!targets.is_empty());
        assert!(targets.iter().all(|target| {
            point_strictly_inside_support_cell(&target.point, &bounds, &halfspaces).unwrap()
        }));
    }

    #[test]
    fn support_cell_targets_include_shifted_targets_without_centroid_seed() {
        let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
        let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
        let direct = p(2, 1, 3);
        let report =
            hyperlimit::HalfspaceFeasibilityReport::feasible(direct.clone(), [None, None, None]);

        let targets = strict_support_cell_targets(&bounds, &halfspaces, &report).unwrap();

        assert!(
            targets
                .iter()
                .filter(|target| target.point != direct)
                .any(|target| !target.definitions.is_empty())
        );
    }

    #[test]
    fn support_reference_definitions_include_non_basis_active_halfspaces() {
        let witness = p(1, 1, 1);
        let halfspaces = vec![
            axis_halfspace(0, false, r(1)),
            axis_halfspace(1, false, r(1)),
            axis_halfspace(2, false, r(1)),
            LimitPlane3::new(p(1, 1, 1), r(-3)),
        ];

        let definitions = reference_definitions_from_active_halfspaces(
            &witness,
            &halfspaces,
            [Some(0), Some(1), Some(2)],
        )
        .unwrap();

        assert!(
            definitions
                .definitions
                .iter()
                .any(definition_uses_non_axis_plane)
        );
        for definition in &definitions.definitions {
            assert_eq!(affine_from_planes(definition).unwrap(), witness);
        }
    }

    #[test]
    fn reference_propagation_reports_exhausted_construction() {
        let bounds = Aabb::new(p(0, 0, 0), p(0, 0, 0));
        let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(0)))];

        let err = compute_new_reference(
            &p(-1, -1, -1),
            &axis_defs(&p(-1, -1, -1)),
            &[0],
            &bounds,
            &polygons,
        )
        .unwrap_err();

        assert_eq!(
            err,
            crate::error::HypermeshError::ReferencePropagationFailed
        );
    }

    #[test]
    fn subdivide_into_keeps_output_unchanged_on_uncertified_failure() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
        let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
        let sentinel = ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 99),
            1,
        );
        let mut output = vec![sentinel.clone()];

        let err = subdivide_into(
            SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0]),
            &indicator,
            SubdivisionConfig { max_depth: 0 },
            &mut output,
        )
        .unwrap_err();

        assert_eq!(
            err,
            crate::error::HypermeshError::SubdivisionDepthLimit {
                depth: 0,
                polygon_count: 1
            }
        );
        assert_eq!(output, vec![sentinel]);
    }

    #[test]
    fn unsplittable_task_requires_certified_leaf_completion() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(1, 0, 0), p(1, 0, 0));
        let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
        let mut output = Vec::new();
        let emitted = ClassifiedPolygon::new(wall.clone(), 1);
        let caches = SubdivisionRuntimeCaches::default();

        let err = subdivide_into_inner_with(
            SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0]),
            &indicator,
            SubdivisionConfig { max_depth: 4 },
            None,
            &mut output,
            &mut |_task, _indicator, out| {
                out.push(emitted.clone());
                Ok(LeafProcessingStats {
                    polygon_count: 1,
                    certified_complete: false,
                    ..LeafProcessingStats::default()
                })
            },
            &caches,
            &caches.winding_reachability,
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
        assert!(output.is_empty());
    }

    #[test]
    fn unsplittable_task_accepts_certified_leaf_completion() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(1, 0, 0), p(1, 0, 0));
        let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
        let mut output = Vec::new();
        let emitted = ClassifiedPolygon::new(wall.clone(), 1);
        let caches = SubdivisionRuntimeCaches::default();

        subdivide_into_inner_with(
            SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0]),
            &indicator,
            SubdivisionConfig { max_depth: 4 },
            None,
            &mut output,
            &mut |_task, _indicator, out| {
                out.push(emitted.clone());
                Ok(LeafProcessingStats {
                    polygon_count: 1,
                    certified_complete: true,
                    ..LeafProcessingStats::default()
                })
            },
            &caches,
            &caches.winding_reachability,
        )
        .unwrap();

        assert_eq!(output, vec![emitted]);
    }

    #[test]
    fn subdivision_keeps_splitting_after_uncertified_leaf_failure() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
        let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
        let sentinel = ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 99),
            1,
        );
        let mut output = vec![sentinel.clone()];

        let err = subdivide_into(
            SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0]),
            &indicator,
            SubdivisionConfig { max_depth: 0 },
            &mut output,
        )
        .unwrap_err();

        assert_eq!(
            err,
            crate::error::HypermeshError::SubdivisionDepthLimit {
                depth: 0,
                polygon_count: 1
            }
        );
        assert_eq!(output, vec![sentinel]);
    }

    #[test]
    fn operation_subdivision_discards_fixed_difference_outside_region() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 1, 0);
        wall.delta_w = vec![0, 1];
        let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
        let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Difference, 2);

        let output = subdivide_for_operation(
            SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0, 0]),
            &indicator,
            SubdivisionConfig { max_depth: 0 },
            BooleanOp::Difference,
        )
        .unwrap();

        assert!(output.is_empty());
    }

    #[test]
    fn operation_subdivision_keeps_potential_difference_region() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1, 0];
        let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
        let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Difference, 2);

        let err = subdivide_for_operation(
            SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0, 0]),
            &indicator,
            SubdivisionConfig { max_depth: 0 },
            BooleanOp::Difference,
        )
        .unwrap_err();

        assert_eq!(
            err,
            crate::error::HypermeshError::SubdivisionDepthLimit {
                depth: 0,
                polygon_count: 1
            }
        );
    }

    #[test]
    fn process_leaf_into_keeps_output_unchanged_on_uncertified_failure() {
        let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
        wall.delta_w = vec![1];
        let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
        let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
        let sentinel = ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 99),
            1,
        );
        let mut output = vec![sentinel.clone()];

        let err = process_leaf_into(
            &[wall],
            &bounds,
            &p(0, 0, 0),
            &axis_defs(&p(0, 0, 0)),
            &[0],
            &indicator,
            &mut output,
        )
        .unwrap_err();

        assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
        assert_eq!(output, vec![sentinel]);
    }

    #[test]
    fn bsp_leaf_certification_rejects_unsplit_interior_segment() {
        let mut host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        host.delta_w = vec![1, 0];
        let mut cutter = make_triangle(&p(1, 0, -1), &p(1, 0, 1), &p(1, 2, 0), 1, 0);
        cutter.delta_w = vec![0, 1];
        let polygons = vec![host.clone(), cutter];

        assert!(
            !certify_bsp_leaf_has_no_interior_intersections(&host, &host.edges, &polygons).unwrap()
        );
    }

    #[test]
    fn bsp_leaf_certification_rejects_boundary_ambiguous_overlap() {
        let mut host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 1, 0);
        host.delta_w = vec![0, 1];
        let mut overlap = make_triangle(
            &p(0, 0, 0),
            &Point3::new(q(4, 3), r(0), r(0)),
            &Point3::new(r(0), q(4, 3), r(0)),
            0,
            0,
        );
        overlap.delta_w = vec![1, 0];
        let polygons = vec![host.clone(), overlap];

        assert!(
            !certify_bsp_leaf_has_no_interior_intersections(&host, &host.edges, &polygons).unwrap()
        );
    }

    #[test]
    fn segment_interval_witness_finds_strict_overlap_when_midpoint_is_on_boundary() {
        let left = make_triangle(&p(1, -1, 0), &p(3, -1, 0), &p(1, 1, 0), 0, 0);
        let right = make_triangle(&p(0, -2, 0), &p(4, -2, 0), &p(0, 2, 0), 1, 0);

        assert!(
            segment_has_strict_interior_point_in_both(&p(0, 0, 0), &p(2, 0, 0), &left, &right)
                .unwrap()
        );
    }

    fn support_only_polygon(support: Plane) -> ConvexPolygon {
        ConvexPolygon {
            support,
            edges: Vec::new(),
            mesh_index: 0,
            polygon_index: 0,
            delta_w: Vec::new(),
            approx_bounds: None,
        }
    }
}
