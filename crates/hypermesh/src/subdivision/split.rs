//! Exact split-basis construction, ranking, and child partitioning.

use super::{
    Aabb, ClipSide, ConvexPolygon, HypermeshResult, IntersectionSegment, PairwiseIntersectionType,
    PairwiseIntersectionsCacheEntry, Plane, PolygonFamilyBoundsCacheEntry, RankedSplitAttempt,
    Real, RootSplitPlane, SplitAttemptChildFanoutCacheEntry, SplitChildPartition,
    SplitChildPartitionCacheEntry, SplitSource, axis_mut, axis_ref,
    cached_pairwise_intersections_by_polygon_with, cached_recursive_child_bounds_with,
    cached_split_child_partition_with, clip_polygon, compare_real,
    polygon_families_match_as_multisets, polygon_family_profile,
    split_child_matches_parent_geometry, take_new_subdivision_child_partition,
};
#[cfg(test)]
use super::{ExactBvh, intersect_polygons, polygon_axis_values};
use std::cell::RefCell;

pub(super) fn can_split_bounds(bounds: &Aabb) -> HypermeshResult<bool> {
    for axis in 0..3 {
        if compare_real(&bounds.extent(axis), &Real::zero())?.is_gt() {
            return Ok(true);
        }
    }
    Ok(false)
}

pub(super) fn polygon_family_bounds(polygons: &[ConvexPolygon]) -> HypermeshResult<Aabb> {
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
pub(super) fn select_subdivision_split(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<(usize, Real)> {
    ordered_subdivision_splits(bounds, polygons)?
        .into_iter()
        .next()
        .ok_or(crate::error::HypermeshError::UnknownClassification)
}

pub(super) type SplitCounts = (usize, usize, usize, usize, usize, usize);

#[derive(Clone, Debug, PartialEq)]
pub(super) struct SplitCandidate {
    pub(super) axis: usize,
    pub(super) value: Real,
    pub(super) counts: SplitCounts,
    pub(super) source: SplitSource,
}

#[cfg(test)]
pub(super) fn ordered_subdivision_splits(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<(usize, Real)>> {
    let mut candidates = Vec::new();

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

pub(super) fn ordered_subdivision_splits_with_partition_cache(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    fanout_count_cache: &RefCell<Vec<SplitAttemptChildFanoutCacheEntry>>,
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygon_bounds_cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    root_basis: &[RootSplitPlane],
) -> HypermeshResult<Vec<RankedSplitAttempt>> {
    let unique = unique_subdivision_split_attempts_with_partition_cache(
        bounds,
        polygons,
        partition_cache,
        polygon_bounds_cache,
        root_basis,
    )?;
    let mut ranked_attempts = unique;
    ranked_attempts.sort_by(|left, right| {
        split_attempt_cheap_order_key(left).cmp(&split_attempt_cheap_order_key(right))
    });
    let fanout_refinement_len = ranked_attempts.len().min(4);
    let mut fanout_cache = fanout_count_cache.borrow_mut();
    let mut fanout_ranked_attempts = Vec::with_capacity(fanout_refinement_len);
    for attempt in ranked_attempts.drain(..fanout_refinement_len) {
        let fanout_key = split_attempt_child_fanout_key(
            &attempt,
            partition_cache,
            polygon_bounds_cache,
            root_basis,
            &mut fanout_cache,
        )?;
        fanout_ranked_attempts.push((attempt, fanout_key));
    }
    fanout_ranked_attempts
        .sort_by_key(|(attempt, fanout)| split_attempt_fanout_order_key(attempt, *fanout));
    let mut ordered = fanout_ranked_attempts
        .into_iter()
        .map(|(attempt, _)| attempt)
        .collect::<Vec<_>>();
    ordered.extend(ranked_attempts);
    Ok(ordered)
}

fn unique_subdivision_split_attempts_with_partition_cache(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygon_bounds_cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    root_basis: &[RootSplitPlane],
) -> HypermeshResult<Vec<RankedSplitAttempt>> {
    let mut candidates = Vec::new();
    for split in root_basis {
        if !split_value_is_strictly_inside_bounds(bounds, split.axis, &split.value)? {
            continue;
        }
        push_split_candidate_with_partition_cache(
            &mut candidates,
            polygons,
            split.axis,
            split.value.clone(),
            split.source,
            partition_cache,
        )?;
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
            continue;
        }
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
    attempt: &RankedSplitAttempt,
) -> (usize, usize, usize) {
    let left_axes = attempt
        .left_bounds
        .as_ref()
        .map_or(0, positive_extent_axis_count);
    let right_axes = attempt
        .right_bounds
        .as_ref()
        .map_or(0, positive_extent_axis_count);
    (
        left_axes.max(right_axes),
        left_axes + right_axes,
        left_axes.abs_diff(right_axes),
    )
}

fn split_attempt_cheap_order_key(
    attempt: &RankedSplitAttempt,
) -> ((usize, usize, usize), SplitCounts, SplitSource) {
    (
        split_attempt_recursive_room_key(attempt),
        attempt.counts,
        attempt.source,
    )
}

pub(super) fn split_attempt_fanout_order_key(
    attempt: &RankedSplitAttempt,
    fanout: (usize, usize, usize),
) -> (
    (usize, usize, usize),
    SplitCounts,
    (usize, usize, usize),
    SplitSource,
) {
    (
        split_attempt_recursive_room_key(attempt),
        attempt.counts,
        fanout,
        attempt.source,
    )
}

fn split_attempt_child_fanout_key(
    attempt: &RankedSplitAttempt,
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygon_bounds_cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    root_basis: &[RootSplitPlane],
    cache: &mut Vec<SplitAttemptChildFanoutCacheEntry>,
) -> HypermeshResult<(usize, usize, usize)> {
    let left_count = if let Some(bounds) = attempt.left_bounds.as_ref() {
        cached_unique_subdivision_split_attempt_count_with(
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
    cache: &mut Vec<SplitAttemptChildFanoutCacheEntry>,
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    partition_cache: &RefCell<Vec<SplitChildPartitionCacheEntry>>,
    polygon_bounds_cache: &RefCell<Vec<PolygonFamilyBoundsCacheEntry>>,
    root_basis: &[RootSplitPlane],
) -> HypermeshResult<usize> {
    cached_unique_subdivision_split_attempt_count_with_query(cache, bounds, polygons, || {
        unique_subdivision_split_attempts_with_partition_cache(
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

fn positive_extent_axis_count(bounds: &Aabb) -> usize {
    (0..3)
        .filter(|&axis| {
            compare_real(&bounds.extent(axis), &Real::zero()).is_ok_and(|order| order.is_gt())
        })
        .count()
}

#[cfg(test)]
pub(super) fn split_intersection_segments(
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

pub(super) fn split_intersection_segments_with_pairwise_cache(
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
pub(super) fn push_split_candidate(
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
        crate::error::HypermeshError::UnknownClassification
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
        crate::error::HypermeshError::UnknownClassification => 1,
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
    bounds: &Aabb,
    axis_values: &[Vec<Real>; 3],
    intersection_segments: &[IntersectionSegment],
) -> HypermeshResult<Vec<RootSplitPlane>> {
    let mut basis = Vec::new();
    for axis in 0..3 {
        for (_gap, value) in
            arrangement_split_candidates_from_axis_values(bounds, &axis_values[axis], axis)?
        {
            push_root_split_plane(&mut basis, axis, value, SplitSource::Arrangement)?;
        }
        for value in
            intersection_split_candidates_from_segments(bounds, intersection_segments, axis)?
        {
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
        if existing.axis == axis && compare_real(&existing.value, &value)?.is_eq() {
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
    bounds: &Aabb,
    axis: usize,
    value: &Real,
) -> HypermeshResult<bool> {
    Ok(compare_real(value, axis_ref(&bounds.min, axis))?.is_gt()
        && compare_real(value, axis_ref(&bounds.max, axis))?.is_lt())
}

#[cfg(test)]
pub(super) fn arrangement_split_candidates(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
) -> HypermeshResult<Vec<(Real, Real)>> {
    let axis_values = polygon_axis_values(polygons)?;
    arrangement_split_candidates_from_axis_values(bounds, &axis_values[axis], axis)
}

pub(super) fn arrangement_split_candidates_from_axis_values(
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
pub(super) fn intersection_split_candidates(
    bounds: &Aabb,
    polygons: &[ConvexPolygon],
    axis: usize,
) -> HypermeshResult<Vec<Real>> {
    let segments = split_intersection_segments(polygons)?;
    intersection_split_candidates_from_segments(bounds, &segments, axis)
}

pub(super) fn intersection_split_candidates_from_segments(
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

pub(super) fn push_unique_ordered_axis_value(
    values: &mut Vec<Real>,
    value: Real,
) -> HypermeshResult<()> {
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
