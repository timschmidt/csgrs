//! Exact split-basis construction, ranking, and child partitioning.

#[cfg(test)]
use super::ExactBvh;
use super::{
    Aabb, ClipSide, ConvexPolygon, HypermeshResult, IntersectionSegment, PairwiseIntersectionType,
    PairwiseIntersectionsCacheEntry, Plane, PolygonFamilyProfile, Real, axis_mut, axis_ref,
    cached_pairwise_intersections_by_polygon_with_certified_embedded_inputs,
    polygon_families_match_as_multisets, polygon_family_profile,
    split_child_matches_parent_geometry,
};
use crate::clip::clip_polygon_decision;
use crate::context::DecisionContext;
use crate::geometry::compare_real_decision;
#[cfg(test)]
use crate::intersection::intersect_polygons_with_vertices;
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Clone, Debug, PartialEq)]
pub(super) struct SubdivisionChildPartition {
    left_polygon_profile: PolygonFamilyProfile,
    left_polygons: Vec<ConvexPolygon>,
    left_bounds: Option<Aabb>,
    right_polygon_profile: PolygonFamilyProfile,
    right_polygons: Vec<ConvexPolygon>,
    right_bounds: Option<Aabb>,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct PolygonFamilyBoundsCacheEntry {
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    bounds: HypermeshResult<Aabb>,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub(super) enum SplitSource {
    Intersection,
    Arrangement,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct RootSplitPlane {
    pub(super) axis: usize,
    pub(super) value: Real,
    pub(super) source: SplitSource,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct SplitCandidatesCacheEntry {
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    bounds: Aabb,
    candidates: HypermeshResult<Vec<RankedSplitAttempt>>,
}

#[derive(Default)]
pub(super) struct SplitCandidatesCache {
    // Initialized from the top-level task before contraction or clipping.
    pub(super) root_basis: Option<HypermeshResult<Rc<Vec<RootSplitPlane>>>>,
    pub(super) entries: Vec<SplitCandidatesCacheEntry>,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct PolygonAxisValuesCacheEntry {
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    result: HypermeshResult<[Vec<Real>; 3]>,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct SplitAttemptChildFanoutCacheEntry {
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    bounds: Aabb,
    count: HypermeshResult<usize>,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct SplitChildPartition {
    pub(super) left_polys: Vec<ConvexPolygon>,
    pub(super) right_polys: Vec<ConvexPolygon>,
    pub(super) both_count: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct SplitChildPartitionCacheEntry {
    polygon_profile: PolygonFamilyProfile,
    polygons: Vec<ConvexPolygon>,
    axis: usize,
    value: Real,
    result: HypermeshResult<SplitChildPartition>,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct RankedSplitAttempt {
    pub(super) axis: usize,
    pub(super) value: Real,
    pub(super) counts: SplitCounts,
    pub(super) source: SplitSource,
    pub(super) left_polys: Vec<ConvexPolygon>,
    pub(super) left_bounds: Option<Aabb>,
    pub(super) right_polys: Vec<ConvexPolygon>,
    pub(super) right_bounds: Option<Aabb>,
}

#[cfg(test)]
pub(super) fn recursive_child_bounds(
    decisions: &DecisionContext,
    _parent_polygons: &[ConvexPolygon],
    child_polygons: &[ConvexPolygon],
    _child_bounds: &Aabb,
) -> HypermeshResult<Aabb> {
    polygon_family_bounds(decisions, child_polygons)
}

pub(super) fn cached_polygon_family_bounds_with(
    cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    polygons: &[ConvexPolygon],
    query: impl FnOnce(&[ConvexPolygon]) -> HypermeshResult<Aabb>,
) -> HypermeshResult<Aabb> {
    if let Some(existing) = cache
        .borrow()
        .iter()
        .rev()
        .find(|existing| existing.polygons == polygons)
        .cloned()
    {
        return existing.bounds;
    }

    let polygon_profile = polygon_family_profile(polygons);
    let existing = cache
        .borrow()
        .iter()
        .rev()
        .find(|existing| {
            existing.polygon_profile == polygon_profile
                && polygon_families_match_as_multisets(&existing.polygons, polygons)
        })
        .cloned();
    if let Some(existing) = existing {
        if existing.polygons != polygons {
            cache_polygon_family_bounds_result(cache, polygons, &existing.bounds);
        }
        return existing.bounds;
    }

    let bounds = query(polygons);
    cache.borrow_mut().push(PolygonFamilyBoundsCacheEntry {
        polygon_profile,
        polygons: polygons.to_vec(),
        bounds: bounds.clone(),
    });
    bounds
}

fn cache_polygon_family_bounds_result(
    cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    polygons: &[ConvexPolygon],
    bounds: &HypermeshResult<Aabb>,
) {
    if cache
        .borrow()
        .iter()
        .any(|existing| existing.polygons == polygons)
    {
        return;
    }

    cache.borrow_mut().push(PolygonFamilyBoundsCacheEntry {
        polygon_profile: polygon_family_profile(polygons),
        polygons: polygons.to_vec(),
        bounds: bounds.clone(),
    });
}

fn cached_recursive_child_bounds_with(
    decisions: &DecisionContext,
    cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    _parent_polygons: &[ConvexPolygon],
    child_polygons: &[ConvexPolygon],
    _child_bounds: &Aabb,
) -> HypermeshResult<Aabb> {
    cached_polygon_family_bounds_with(cache, child_polygons, |polygons| {
        polygon_family_bounds(decisions, polygons)
    })
}

pub(super) fn polygon_axis_values(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<[Vec<Real>; 3]> {
    let mut values = [Vec::new(), Vec::new(), Vec::new()];
    for polygon in polygons {
        for vertex in polygon.vertices()? {
            for (axis, axis_values) in values.iter_mut().enumerate() {
                push_unique_ordered_axis_value(
                    decisions,
                    axis_values,
                    axis_ref(&vertex, axis).clone(),
                )?;
            }
        }
    }
    Ok(values)
}

pub(super) fn cached_polygon_axis_values_with(
    decisions: &DecisionContext,
    cache: &RefCell<Vec<PolygonAxisValuesCacheEntry>>,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<[Vec<Real>; 3]> {
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
        .find(|existing| {
            existing.polygon_profile == polygon_profile
                && polygon_families_match_as_multisets(&existing.polygons, polygons)
        })
        .cloned();
    if let Some(existing) = existing {
        if existing.polygons != polygons {
            cache_polygon_axis_values_result(cache, polygons, &existing.result);
        }
        return existing.result;
    }

    let result = polygon_axis_values(decisions, polygons);
    cache.borrow_mut().push(PolygonAxisValuesCacheEntry {
        polygon_profile,
        polygons: polygons.to_vec(),
        result: result.clone(),
    });
    result
}

fn cache_polygon_axis_values_result(
    cache: &RefCell<Vec<PolygonAxisValuesCacheEntry>>,
    polygons: &[ConvexPolygon],
    result: &HypermeshResult<[Vec<Real>; 3]>,
) {
    if cache
        .borrow()
        .iter()
        .any(|existing| existing.polygons == polygons)
    {
        return;
    }

    cache.borrow_mut().push(PolygonAxisValuesCacheEntry {
        polygon_profile: polygon_family_profile(polygons),
        polygons: polygons.to_vec(),
        result: result.clone(),
    });
}

#[cfg(test)]
pub(super) fn cached_root_split_basis_with(
    decisions: &DecisionContext,
    cache: &RefCell<SplitCandidatesCache>,
    axis_values_cache: &RefCell<Vec<PolygonAxisValuesCacheEntry>>,
    pairwise_cache: &RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Rc<Vec<RootSplitPlane>>> {
    cached_root_split_basis_with_certified_embedded_inputs(
        decisions,
        cache,
        axis_values_cache,
        pairwise_cache,
        &[],
        bounds,
        polygons,
    )
}

pub(super) fn cached_root_split_basis_with_certified_embedded_inputs(
    decisions: &DecisionContext,
    cache: &RefCell<SplitCandidatesCache>,
    axis_values_cache: &RefCell<Vec<PolygonAxisValuesCacheEntry>>,
    pairwise_cache: &RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    certified_embedded_inputs: &[bool],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Rc<Vec<RootSplitPlane>>> {
    if let Some(existing) = cache.borrow().root_basis.clone() {
        return existing;
    }

    let result = (|| {
        crate::trace_dispatch!("root-split-basis", "axis-values");
        let axis_values = cached_polygon_axis_values_with(decisions, axis_values_cache, polygons)
            .map_err(|error| {
            crate::trace_dispatch!("root-split-basis", "axis-values-failed");
            if cfg!(debug_assertions) {
                eprintln!("[DEBUG] root axis values failed: {error}");
            }
            error
        })?;
        crate::trace_dispatch!("root-split-basis", "intersection-segments");
        let intersection_segments =
            split_intersection_segments_with_pairwise_cache_and_certified_embedded_inputs(
                decisions,
                pairwise_cache,
                polygons,
                certified_embedded_inputs,
            )
            .map_err(|error| {
                crate::trace_dispatch!("root-split-basis", "intersection-segments-failed");
                if cfg!(debug_assertions) {
                    eprintln!("[DEBUG] root intersection segments failed: {error}");
                }
                error
            })?;
        crate::trace_dispatch!("root-split-basis", "event-basis");
        root_split_basis_from_events(decisions, bounds, &axis_values, &intersection_segments)
            .map(Rc::new)
            .map_err(|error| {
                crate::trace_dispatch!("root-split-basis", "event-basis-failed");
                if cfg!(debug_assertions) {
                    eprintln!("[DEBUG] root event basis failed: {error}");
                }
                error
            })
    })();
    cache.borrow_mut().root_basis = Some(result.clone());
    result
}

#[cfg(test)]
pub(super) fn cached_ordered_subdivision_splits_with(
    decisions: &DecisionContext,
    axis_values_cache: &RefCell<Vec<PolygonAxisValuesCacheEntry>>,
    cache: &RefCell<SplitCandidatesCache>,
    fanout_count_cache: &RefCell<Vec<SplitAttemptChildFanoutCacheEntry>>,
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygon_bounds_cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    pairwise_cache: &RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<RankedSplitAttempt>> {
    cached_ordered_subdivision_splits_with_certified_embedded_inputs(
        decisions,
        axis_values_cache,
        cache,
        fanout_count_cache,
        partition_cache,
        polygon_bounds_cache,
        pairwise_cache,
        &[],
        bounds,
        polygons,
    )
}

pub(super) fn cached_ordered_subdivision_splits_with_certified_embedded_inputs(
    decisions: &DecisionContext,
    axis_values_cache: &RefCell<Vec<PolygonAxisValuesCacheEntry>>,
    cache: &RefCell<SplitCandidatesCache>,
    fanout_count_cache: &RefCell<Vec<SplitAttemptChildFanoutCacheEntry>>,
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygon_bounds_cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    pairwise_cache: &RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    certified_embedded_inputs: &[bool],
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<RankedSplitAttempt>> {
    let root_basis = cached_root_split_basis_with_certified_embedded_inputs(
        decisions,
        cache,
        axis_values_cache,
        pairwise_cache,
        certified_embedded_inputs,
        bounds,
        polygons,
    )?;
    if let Some(existing) = cache
        .borrow()
        .entries
        .iter()
        .rev()
        .find(|existing| existing.bounds == *bounds && existing.polygons == polygons)
        .cloned()
    {
        return existing.candidates;
    }

    let polygon_profile = polygon_family_profile(polygons);
    let existing = cache
        .borrow()
        .entries
        .iter()
        .rev()
        .find(|existing| {
            existing.bounds == *bounds
                && existing.polygon_profile == polygon_profile
                && polygon_families_match_as_multisets(&existing.polygons, polygons)
        })
        .cloned();
    if let Some(existing) = existing {
        if existing.polygons != polygons {
            cache_split_candidates_result(cache, polygons, bounds, &existing.candidates);
        }
        return existing.candidates;
    }

    let candidates = ordered_subdivision_splits_with_partition_cache(
        decisions,
        bounds,
        polygons,
        fanout_count_cache,
        partition_cache,
        polygon_bounds_cache,
        root_basis.as_ref(),
    );
    cache.borrow_mut().entries.push(SplitCandidatesCacheEntry {
        polygon_profile,
        polygons: polygons.to_vec(),
        bounds: bounds.clone(),
        candidates: candidates.clone(),
    });
    candidates
}

fn cache_split_candidates_result(
    cache: &RefCell<SplitCandidatesCache>,
    polygons: &[ConvexPolygon],
    bounds: &Aabb,
    candidates: &HypermeshResult<Vec<RankedSplitAttempt>>,
) {
    if cache
        .borrow()
        .entries
        .iter()
        .any(|existing| existing.bounds == *bounds && existing.polygons == polygons)
    {
        return;
    }

    cache.borrow_mut().entries.push(SplitCandidatesCacheEntry {
        polygon_profile: polygon_family_profile(polygons),
        polygons: polygons.to_vec(),
        bounds: bounds.clone(),
        candidates: candidates.clone(),
    });
}

pub(super) fn cached_split_child_partition_with(
    decisions: &DecisionContext,
    cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygons: &[ConvexPolygon],
    axis: usize,
    value: &Real,
) -> HypermeshResult<SplitChildPartition> {
    {
        let cache_ref = cache.borrow();
        for existing in cache_ref.iter().rev() {
            if existing.polygons == polygons
                && existing.axis == axis
                && compare_real_decision(decisions, &existing.value, value)?.is_eq()
            {
                return existing.result.clone();
            }
        }
    }

    let polygon_profile = polygon_family_profile(polygons);
    let existing = {
        let cache_ref = cache.borrow();
        let mut found = None;
        for existing in cache_ref.iter().rev() {
            if existing.axis == axis
                && existing.polygon_profile == polygon_profile
                && polygon_families_match_as_multisets(&existing.polygons, polygons)
                && compare_real_decision(decisions, &existing.value, value)?.is_eq()
            {
                found = Some(existing.clone());
                break;
            }
        }
        found
    };
    if let Some(existing) = existing {
        if !split_child_partition_cache_entry_matches_exact_state(
            decisions, &existing, polygons, axis, value,
        )? {
            cache_split_child_partition_result(
                decisions,
                cache,
                polygons,
                axis,
                value,
                &existing.result,
            )?;
        }
        return existing.result;
    }

    let result = split_child_partition(decisions, polygons, axis, value);
    cache.borrow_mut().push(SplitChildPartitionCacheEntry {
        polygon_profile,
        polygons: polygons.to_vec(),
        axis,
        value: value.clone(),
        result: result.clone(),
    });
    result
}

fn cache_split_child_partition_result(
    decisions: &DecisionContext,
    cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygons: &[ConvexPolygon],
    axis: usize,
    value: &Real,
    result: &HypermeshResult<SplitChildPartition>,
) -> HypermeshResult<()> {
    {
        let cache_ref = cache.borrow();
        for existing in cache_ref.iter() {
            if split_child_partition_cache_entry_matches_exact_state(
                decisions, existing, polygons, axis, value,
            )? {
                return Ok(());
            }
        }
    }

    cache.borrow_mut().push(SplitChildPartitionCacheEntry {
        polygon_profile: polygon_family_profile(polygons),
        polygons: polygons.to_vec(),
        axis,
        value: value.clone(),
        result: result.clone(),
    });
    Ok(())
}

fn split_child_partition_cache_entry_matches_exact_state(
    decisions: &DecisionContext,
    existing: &SplitChildPartitionCacheEntry,
    polygons: &[ConvexPolygon],
    axis: usize,
    value: &Real,
) -> HypermeshResult<bool> {
    Ok(existing.axis == axis
        && existing.polygons == polygons
        && compare_real_decision(decisions, &existing.value, value)?.is_eq())
}

pub(super) fn take_new_subdivision_child_partition(
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

pub(super) fn can_split_bounds(
    decisions: &DecisionContext,
    bounds: &Aabb,
) -> HypermeshResult<bool> {
    for axis in 0..3 {
        if compare_real_decision(decisions, &bounds.extent(axis), &Real::zero())?.is_gt() {
            return Ok(true);
        }
    }
    Ok(false)
}

pub(super) fn polygon_family_bounds(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Aabb> {
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
            if compare_real_decision(decisions, axis_ref(&vertex, axis), axis_ref(&min, axis))?
                .is_lt()
            {
                *axis_mut(&mut min, axis) = axis_ref(&vertex, axis).clone();
            }
            if compare_real_decision(decisions, axis_ref(&vertex, axis), axis_ref(&max, axis))?
                .is_gt()
            {
                *axis_mut(&mut max, axis) = axis_ref(&vertex, axis).clone();
            }
        }
    }

    Ok(Aabb::new(min, max))
}

#[cfg(test)]
pub(super) fn select_subdivision_split(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<(usize, Real)> {
    ordered_subdivision_splits(
        &crate::test_support::approximate_decisions(),
        bounds,
        polygons,
    )?
    .into_iter()
    .next()
    .ok_or(crate::error::HypermeshError::UnknownClassification)
}

pub(super) type SplitCounts = (usize, usize, usize, usize, usize, usize);
type RecursiveRoomKey = (usize, usize, usize);
type SplitAttemptOrderKey = (RecursiveRoomKey, SplitCounts, SplitSource);
type SplitAttemptFanoutOrderKey = (RecursiveRoomKey, SplitCounts, RecursiveRoomKey, SplitSource);

#[derive(Clone, Debug, PartialEq)]
pub(super) struct SplitCandidate {
    pub(super) axis: usize,
    pub(super) value: Real,
    pub(super) counts: SplitCounts,
    pub(super) source: SplitSource,
}

#[cfg(test)]
pub(super) fn ordered_subdivision_splits(
    decisions: &DecisionContext,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<(usize, Real)>> {
    let mut candidates = Vec::new();

    for axis in 0..3 {
        if compare_real_decision(decisions, &bounds.extent(axis), &Real::zero())?.is_le() {
            continue;
        }
        for (_gap, value) in arrangement_split_candidates(decisions, bounds, polygons, axis)? {
            push_split_candidate(
                decisions,
                &mut candidates,
                polygons,
                axis,
                value,
                SplitSource::Arrangement,
            )?;
        }
    }

    for axis in 0..3 {
        if compare_real_decision(decisions, &bounds.extent(axis), &Real::zero())?.is_le() {
            continue;
        }
        for value in intersection_split_candidates(decisions, bounds, polygons, axis)? {
            push_split_candidate(
                decisions,
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

pub(super) fn ordered_subdivision_splits_with_partition_cache(
    decisions: &DecisionContext,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    fanout_count_cache: &RefCell<Vec<SplitAttemptChildFanoutCacheEntry>>,
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygon_bounds_cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    root_basis: &[RootSplitPlane],
) -> HypermeshResult<Vec<RankedSplitAttempt>> {
    let unique = unique_subdivision_split_attempts_with_partition_cache(
        decisions,
        bounds,
        polygons,
        partition_cache,
        polygon_bounds_cache,
        root_basis,
    )
    .map_err(|error| {
        if cfg!(debug_assertions) {
            eprintln!("[DEBUG] unique split attempts failed: {error}");
        }
        error
    })?;
    let mut ranked_attempts = unique
        .into_iter()
        .map(|attempt| {
            let key = split_attempt_cheap_order_key(decisions, &attempt)?;
            Ok((attempt, key))
        })
        .collect::<HypermeshResult<Vec<_>>>()?;
    ranked_attempts.sort_by_key(|(_, key)| *key);
    let mut ranked_attempts = ranked_attempts
        .into_iter()
        .map(|(attempt, _)| attempt)
        .collect::<Vec<_>>();
    let fanout_refinement_len = ranked_attempts.len().min(4);
    let mut fanout_cache = fanout_count_cache.borrow_mut();
    let mut fanout_ranked_attempts = Vec::with_capacity(fanout_refinement_len);
    for attempt in ranked_attempts.drain(..fanout_refinement_len) {
        let fanout_key = split_attempt_child_fanout_key(
            decisions,
            &attempt,
            partition_cache,
            polygon_bounds_cache,
            root_basis,
            &mut fanout_cache,
        )
        .map_err(|error| {
            if cfg!(debug_assertions) {
                eprintln!("[DEBUG] split fanout key failed: {error}");
            }
            error
        })?;
        let order_key = split_attempt_fanout_order_key(decisions, &attempt, fanout_key)?;
        fanout_ranked_attempts.push((attempt, order_key));
    }
    fanout_ranked_attempts.sort_by_key(|(_, order_key)| *order_key);
    let mut ordered = fanout_ranked_attempts
        .into_iter()
        .map(|(attempt, _)| attempt)
        .collect::<Vec<_>>();
    ordered.extend(ranked_attempts);
    Ok(ordered)
}

fn unique_subdivision_split_attempts_with_partition_cache(
    decisions: &DecisionContext,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygon_bounds_cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    root_basis: &[RootSplitPlane],
) -> HypermeshResult<Vec<RankedSplitAttempt>> {
    let mut candidates = Vec::new();
    for split in root_basis {
        match split_value_is_strictly_inside_bounds(decisions, bounds, split.axis, &split.value) {
            Ok(true) => {}
            Ok(false)
            | Err(
                crate::error::HypermeshError::PredicateUndecided { .. }
                | crate::error::HypermeshError::UnknownClassification,
            ) => continue,
            Err(error) => return Err(error),
        }
        match push_split_candidate_with_partition_cache(
            decisions,
            &mut candidates,
            polygons,
            split.axis,
            split.value.clone(),
            split.source,
            partition_cache,
        ) {
            Ok(()) => {}
            Err(error) if is_backtrackable_split_error(&error) => {
                crate::trace_dispatch!("ordered-splits", "candidate-skipped-uncertified");
            }
            Err(error) => return Err(error),
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
        let attempt = (|| {
            let unclipped_left_bounds = bounds.left_half(candidate.axis, candidate.value.clone());
            let unclipped_right_bounds = bounds.right_half(candidate.axis, candidate.value.clone());
            let split_partition = cached_split_child_partition_with(
                decisions,
                partition_cache,
                polygons,
                candidate.axis,
                &candidate.value,
            )?;
            let left_bounds = if split_partition.left_polys.is_empty() {
                None
            } else {
                Some(cached_recursive_child_bounds_with(
                    decisions,
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
                    decisions,
                    polygon_bounds_cache,
                    polygons,
                    &split_partition.right_polys,
                    &unclipped_right_bounds,
                )?)
            };
            if left_bounds.as_ref().is_some_and(|child_bounds| {
                split_child_matches_parent_geometry(
                    polygons,
                    bounds,
                    &split_partition.left_polys,
                    child_bounds,
                )
            }) || right_bounds.as_ref().is_some_and(|child_bounds| {
                split_child_matches_parent_geometry(
                    polygons,
                    bounds,
                    &split_partition.right_polys,
                    child_bounds,
                )
            }) {
                return Ok(None);
            }
            Ok(Some((split_partition, left_bounds, right_bounds)))
        })();
        let (split_partition, left_bounds, right_bounds) = match attempt {
            Ok(Some(attempt)) => attempt,
            Ok(None) => continue,
            Err(error) if is_backtrackable_split_error(&error) => {
                crate::trace_dispatch!("ordered-splits", "rank-candidate-skipped-uncertified");
                continue;
            }
            Err(error) => return Err(error),
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
                counts: candidate.counts,
                source: candidate.source,
                left_polys: split_partition.left_polys,
                left_bounds,
                right_polys: split_partition.right_polys,
                right_bounds,
            });
        }
    }
    Ok(unique)
}

pub(super) fn split_attempt_recursive_room_key(
    decisions: &DecisionContext,
    attempt: &RankedSplitAttempt,
) -> HypermeshResult<RecursiveRoomKey> {
    let left_axes = match attempt.left_bounds.as_ref() {
        Some(bounds) => positive_extent_axis_count(decisions, bounds)?,
        None => 0,
    };
    let right_axes = match attempt.right_bounds.as_ref() {
        Some(bounds) => positive_extent_axis_count(decisions, bounds)?,
        None => 0,
    };
    Ok((
        left_axes.max(right_axes),
        left_axes + right_axes,
        left_axes.abs_diff(right_axes),
    ))
}

fn split_attempt_cheap_order_key(
    decisions: &DecisionContext,
    attempt: &RankedSplitAttempt,
) -> HypermeshResult<SplitAttemptOrderKey> {
    Ok((
        split_attempt_recursive_room_key(decisions, attempt)?,
        attempt.counts,
        attempt.source,
    ))
}

pub(super) fn split_attempt_fanout_order_key(
    decisions: &DecisionContext,
    attempt: &RankedSplitAttempt,
    fanout: RecursiveRoomKey,
) -> HypermeshResult<SplitAttemptFanoutOrderKey> {
    Ok((
        split_attempt_recursive_room_key(decisions, attempt)?,
        attempt.counts,
        fanout,
        attempt.source,
    ))
}

fn split_attempt_child_fanout_key(
    decisions: &DecisionContext,
    attempt: &RankedSplitAttempt,
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygon_bounds_cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    root_basis: &[RootSplitPlane],
    cache: &mut Vec<SplitAttemptChildFanoutCacheEntry>,
) -> HypermeshResult<(usize, usize, usize)> {
    let left_count = if let Some(bounds) = attempt.left_bounds.as_ref() {
        cached_unique_subdivision_split_attempt_count_with(
            decisions,
            cache,
            bounds,
            &attempt.left_polys,
            partition_cache,
            polygon_bounds_cache,
            root_basis,
        )?
    } else {
        0
    };
    let right_count = if let Some(bounds) = attempt.right_bounds.as_ref() {
        cached_unique_subdivision_split_attempt_count_with(
            decisions,
            cache,
            bounds,
            &attempt.right_polys,
            partition_cache,
            polygon_bounds_cache,
            root_basis,
        )?
    } else {
        0
    };
    Ok((
        left_count.max(right_count),
        left_count + right_count,
        left_count.abs_diff(right_count),
    ))
}

fn cached_unique_subdivision_split_attempt_count_with(
    decisions: &DecisionContext,
    cache: &mut Vec<SplitAttemptChildFanoutCacheEntry>,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygon_bounds_cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    root_basis: &[RootSplitPlane],
) -> HypermeshResult<usize> {
    cached_unique_subdivision_split_attempt_count_with_query(cache, bounds, polygons, || {
        unique_subdivision_split_attempts_with_partition_cache(
            decisions,
            bounds,
            polygons,
            partition_cache,
            polygon_bounds_cache,
            root_basis,
        )
        .map(|attempts| attempts.len())
    })
}

pub(super) fn cached_unique_subdivision_split_attempt_count_with_query(
    cache: &mut Vec<SplitAttemptChildFanoutCacheEntry>,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    query: impl FnOnce() -> HypermeshResult<usize>,
) -> HypermeshResult<usize> {
    if let Some(existing) = cache
        .iter()
        .rev()
        .find(|existing| existing.bounds == *bounds && existing.polygons == polygons)
        .cloned()
    {
        return existing.count;
    }

    let polygon_profile = polygon_family_profile(polygons);
    let existing = cache
        .iter()
        .rev()
        .find(|existing| {
            existing.bounds == *bounds
                && existing.polygon_profile == polygon_profile
                && polygon_families_match_as_multisets(&existing.polygons, polygons)
        })
        .cloned();
    if let Some(existing) = existing {
        if existing.polygons != polygons {
            cache.push(SplitAttemptChildFanoutCacheEntry {
                polygon_profile,
                polygons: polygons.to_vec(),
                bounds: bounds.clone(),
                count: existing.count.clone(),
            });
        }
        return existing.count;
    }

    let count = query();
    cache.push(SplitAttemptChildFanoutCacheEntry {
        polygon_profile,
        polygons: polygons.to_vec(),
        bounds: bounds.clone(),
        count: count.clone(),
    });
    count
}

fn positive_extent_axis_count(
    decisions: &DecisionContext,
    bounds: &Aabb,
) -> HypermeshResult<usize> {
    let mut count = 0;
    for axis in 0..3 {
        if compare_real_decision(decisions, &bounds.extent(axis), &Real::zero())?.is_gt() {
            count += 1;
        }
    }
    Ok(count)
}

#[cfg(test)]
pub(super) fn split_intersection_segments(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<IntersectionSegment>> {
    let bvh = ExactBvh::build_decision(decisions, polygons)?;
    let mut candidate_pairs = Vec::new();
    bvh.intersect_pairs_decision(decisions, &bvh, |left, right| {
        if left < right {
            candidate_pairs.push((left, right));
        }
    })?;

    let mut segments = Vec::new();
    for (left, right) in candidate_pairs {
        let left_vertices = polygons[left].vertices()?;
        let right_vertices = polygons[right].vertices()?;
        let intersection = intersect_polygons_with_vertices(
            decisions,
            &polygons[left],
            &left_vertices,
            &polygons[right],
            &right_vertices,
            right,
        )?;
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

#[cfg(test)]
pub(super) fn split_intersection_segments_with_pairwise_cache(
    decisions: &DecisionContext,
    pairwise_cache: &RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<IntersectionSegment>> {
    split_intersection_segments_with_pairwise_cache_and_certified_embedded_inputs(
        decisions,
        pairwise_cache,
        polygons,
        &[],
    )
}

fn split_intersection_segments_with_pairwise_cache_and_certified_embedded_inputs(
    decisions: &DecisionContext,
    pairwise_cache: &RefCell<Vec<PairwiseIntersectionsCacheEntry>>,
    polygons: &[ConvexPolygon],
    certified_embedded_inputs: &[bool],
) -> HypermeshResult<Vec<IntersectionSegment>> {
    let by_polygon = cached_pairwise_intersections_by_polygon_with_certified_embedded_inputs(
        decisions,
        pairwise_cache,
        polygons,
        certified_embedded_inputs,
    )?;
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
pub(super) fn push_split_candidate(
    decisions: &DecisionContext,
    candidates: &mut Vec<SplitCandidate>,
    polygons: &[ConvexPolygon],
    axis: usize,
    value: Real,
    source: SplitSource,
) -> HypermeshResult<()> {
    for existing in candidates.iter_mut() {
        if existing.axis == axis
            && compare_real_decision(decisions, &existing.value, &value)?.is_eq()
        {
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
    decisions: &DecisionContext,
    candidates: &mut Vec<SplitCandidate>,
    polygons: &[ConvexPolygon],
    axis: usize,
    value: Real,
    source: SplitSource,
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
) -> HypermeshResult<()> {
    for existing in candidates.iter_mut() {
        if existing.axis == axis
            && compare_real_decision(decisions, &existing.value, &value)?.is_eq()
        {
            if source < existing.source {
                existing.source = source;
            }
            return Ok(());
        }
    }

    let partition =
        cached_split_child_partition_with(decisions, partition_cache, polygons, axis, &value)?;
    candidates.push(SplitCandidate {
        axis,
        counts: split_counts_from_partition(polygons, &partition),
        source,
        value,
    });
    Ok(())
}

#[cfg(test)]
pub(super) fn try_ordered_subdivision_splits<T>(
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

pub(super) fn is_backtrackable_split_error(err: &crate::error::HypermeshError) -> bool {
    matches!(
        err,
        crate::error::HypermeshError::PredicateUndecided { .. }
            | crate::error::HypermeshError::UnknownClassification
            | crate::error::HypermeshError::ReferencePropagationFailed
            | crate::error::HypermeshError::SubdivisionDepthLimit { .. }
    )
}

pub(super) fn record_split_failure(
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
        crate::error::HypermeshError::PredicateUndecided { .. }
        | crate::error::HypermeshError::UnknownClassification => 1,
        _ => 0,
    }
}

#[cfg(test)]
pub(super) fn consider_split_candidates(
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
    candidate < baseline
}

pub(super) fn root_split_basis_from_events(
    decisions: &DecisionContext,
    bounds: &Aabb,
    axis_values: &[Vec<Real>; 3],
    intersection_segments: &[IntersectionSegment],
) -> HypermeshResult<Vec<RootSplitPlane>> {
    let mut basis = Vec::new();
    for (axis, axis_values) in axis_values.iter().enumerate() {
        for (_gap, value) in
            arrangement_split_candidates_from_axis_values(decisions, bounds, axis_values, axis)?
        {
            push_root_split_plane(&mut basis, axis, value, SplitSource::Arrangement)?;
        }
        for value in intersection_split_candidates_from_segments(
            decisions,
            bounds,
            intersection_segments,
            axis,
        )? {
            push_root_split_plane(&mut basis, axis, value, SplitSource::Intersection)?;
        }
    }
    Ok(basis)
}

fn push_root_split_plane(
    basis: &mut Vec<RootSplitPlane>,
    axis: usize,
    value: Real,
    source: SplitSource,
) -> HypermeshResult<()> {
    for existing in basis.iter_mut() {
        if existing.axis == axis && existing.value == value {
            if source < existing.source {
                existing.source = source;
            }
            return Ok(());
        }
    }
    basis.push(RootSplitPlane {
        axis,
        value,
        source,
    });
    Ok(())
}

pub(super) fn split_value_is_strictly_inside_bounds(
    decisions: &DecisionContext,
    bounds: &Aabb,
    axis: usize,
    value: &Real,
) -> HypermeshResult<bool> {
    Ok(
        compare_real_decision(decisions, value, axis_ref(&bounds.min, axis))?.is_gt()
            && compare_real_decision(decisions, value, axis_ref(&bounds.max, axis))?.is_lt(),
    )
}

#[cfg(test)]
pub(super) fn arrangement_split_candidates(
    decisions: &DecisionContext,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
) -> HypermeshResult<Vec<(Real, Real)>> {
    let axis_values = polygon_axis_values(decisions, polygons)?;
    arrangement_split_candidates_from_axis_values(decisions, bounds, &axis_values[axis], axis)
}

pub(super) fn arrangement_split_candidates_from_axis_values(
    decisions: &DecisionContext,
    bounds: &Aabb,
    axis_values: &[Real],
    axis: usize,
) -> HypermeshResult<Vec<(Real, Real)>> {
    let min = axis_ref(&bounds.min, axis);
    let max = axis_ref(&bounds.max, axis);
    if !compare_real_decision(decisions, min, max)?.is_lt() {
        return Ok(Vec::new());
    }

    let mut values = Vec::new();
    for value in axis_values {
        if compare_real_decision(decisions, value, min)?.is_gt()
            && compare_real_decision(decisions, value, max)?.is_lt()
        {
            values.push(value.clone());
        }
    }
    if values.is_empty() {
        return Ok(Vec::new());
    }

    let mut candidates = axis_gap_candidates_between_values(decisions, &values)?;
    if !candidates.is_empty() {
        return Ok(candidates);
    }

    update_axis_gap_candidates(decisions, &mut candidates, min, &values[0])?;
    update_axis_gap_candidates(
        decisions,
        &mut candidates,
        values.last().expect("non-empty"),
        max,
    )?;
    Ok(candidates)
}

#[cfg(test)]
pub(super) fn intersection_split_candidates(
    decisions: &DecisionContext,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
) -> HypermeshResult<Vec<Real>> {
    let segments = split_intersection_segments(decisions, polygons)?;
    intersection_split_candidates_from_segments(decisions, bounds, &segments, axis)
}

pub(super) fn intersection_split_candidates_from_segments(
    decisions: &DecisionContext,
    bounds: &Aabb,
    segments: &[IntersectionSegment],
    axis: usize,
) -> HypermeshResult<Vec<Real>> {
    let min = axis_ref(&bounds.min, axis);
    let max = axis_ref(&bounds.max, axis);
    if !compare_real_decision(decisions, min, max)?.is_lt() {
        return Ok(Vec::new());
    }

    let mut values = Vec::new();
    for segment in segments {
        for point in [&segment.v0, &segment.v1] {
            let value = axis_ref(point, axis);
            if compare_real_decision(decisions, value, min)?.is_gt()
                && compare_real_decision(decisions, value, max)?.is_lt()
            {
                match push_unique_ordered_axis_value(decisions, &mut values, value.clone()) {
                    Ok(()) => {}
                    Err(
                        crate::error::HypermeshError::PredicateUndecided { .. }
                        | crate::error::HypermeshError::UnknownClassification,
                    ) => {
                        // Split-candidate order is only a search heuristic. Keep an
                        // undecidable symbolic coordinate in encounter order rather
                        // than turning an otherwise certified subdivision into an
                        // exact-classification failure.
                        if !values.iter().any(|existing| existing == value) {
                            values.push(value.clone());
                        }
                    }
                    Err(error) => return Err(error),
                }
            }
        }
    }

    Ok(values)
}

fn axis_gap_candidates_between_values(
    decisions: &DecisionContext,
    values: &[Real],
) -> HypermeshResult<Vec<(Real, Real)>> {
    let mut candidates = Vec::new();
    for pair in values.windows(2) {
        update_axis_gap_candidates(decisions, &mut candidates, &pair[0], &pair[1])?;
    }
    Ok(candidates)
}

fn update_axis_gap_candidates(
    decisions: &DecisionContext,
    candidates: &mut Vec<(Real, Real)>,
    start: &Real,
    end: &Real,
) -> HypermeshResult<()> {
    if !compare_real_decision(decisions, start, end)?.is_lt() {
        return Ok(());
    }
    let gap = end - start;
    let midpoint = ((start + end) / Real::from(2))
        .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
    candidates.push((gap, midpoint));
    Ok(())
}

pub(super) fn push_unique_ordered_axis_value(
    decisions: &DecisionContext,
    values: &mut Vec<Real>,
    value: Real,
) -> HypermeshResult<()> {
    for (index, existing) in values.iter().enumerate() {
        match compare_real_decision(decisions, &value, existing)? {
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
    let partition = split_child_partition(
        &crate::test_support::approximate_decisions(),
        polygons,
        axis,
        value,
    )?;
    Ok(split_counts_from_partition(polygons, &partition))
}

fn split_counts_from_partition(
    polygons: &[ConvexPolygon],
    partition: &SplitChildPartition,
) -> SplitCounts {
    let left_count = partition.left_polys.len();
    let right_count = partition.right_polys.len();
    let unchanged_children = usize::from(polygon_families_match_as_multisets(
        &partition.left_polys,
        polygons,
    )) + usize::from(polygon_families_match_as_multisets(
        &partition.right_polys,
        polygons,
    ));

    (
        left_count.max(right_count),
        usize::from(left_count == 0 || right_count == 0),
        left_count + right_count,
        unchanged_children,
        partition.both_count,
        left_count.abs_diff(right_count),
    )
}

pub(super) fn split_child_partition(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    axis: usize,
    value: &Real,
) -> HypermeshResult<SplitChildPartition> {
    let split_plane = Plane::axis_aligned(axis, value.clone());
    let mut left_polys = Vec::with_capacity(polygons.len());
    let mut right_polys = Vec::with_capacity(polygons.len());
    let mut both_count = 0;

    for polygon in polygons {
        let clipped = clip_polygon_decision(decisions, polygon, &split_plane)?;
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
