//! Ear clipping triangulation port foundation.
//!
//! This module starts the `earcutr` port with a predicate-backed polygon
//! triangulator. It deliberately keeps the public return shape compatible with
//! earcut-style flat triangle indices while replacing numeric decisions with
//! the crate kernel.

use std::cmp::Ordering;

use crate::error::{Error, Result};
use crate::kernel::{ExactKernel, Kernel};
use crate::polygon::{RingRange, open_ring_indices, rings_from_hole_indices};
use crate::predicates;
use crate::types::Sign;
use crate::types::{ExactPoint, Point2, TriangleIndices};

/// Non-certifying diagnostics for the exact earcut hot loop.
///
/// These counters make candidate pressure visible before introducing optional
/// z-order pruning, unsafe indexing, or additional 2D-specialized kernels.
/// They are scheduling metadata only: exact predicates still certify topology.
/// This mirrors Yap's object-layer discipline of measuring and carrying
/// geometric structure before changing the arithmetic package; see Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997). The ear candidate loop itself follows the two-ears theorem; see
/// Meisters, "Polygons Have Ears," *The American Mathematical Monthly* 82.6
/// (1975).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct EarcutDiagnostics {
    /// Number of vertices tested as ear candidates.
    pub ear_tests: usize,
    /// Number of non-triangle vertices considered for candidate-ear containment.
    pub containment_candidates: usize,
    /// Number of containment candidates rejected because their local vertex is strictly convex.
    pub containment_convex_rejects: usize,
    /// Number of containment candidates classified through prepared local reflex facts.
    pub containment_prepared_reflex_lookups: usize,
    /// Number of full-ring prepared reflex/convex fact rebuilds.
    pub prepared_reflex_rebuilds: usize,
    /// Number of local prepared reflex/convex fact updates after clipping.
    pub prepared_reflex_updates: usize,
    /// Number of containment candidates rejected by an exact triangle AABB.
    pub containment_bbox_rejects: usize,
    /// Number of non-triangle vertices tested with exact triangle containment.
    pub containment_tests: usize,
    /// Number of triangles emitted by clipping, curing, or split fallback.
    pub emitted_triangles: usize,
    /// Number of accepted local-intersection cures.
    pub local_intersection_cures: usize,
    /// Number of split fallback attempts entered.
    pub split_fallbacks: usize,
}

/// Triangle indices paired with hot-loop diagnostics.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EarcutReport {
    /// Flat earcut-compatible triangle index buffer.
    pub triangles: TriangleIndices,
    /// Non-certifying hot-loop counters.
    pub diagnostics: EarcutDiagnostics,
}

/// Triangulate an exact polygon.
pub fn triangulate(vertices: &[ExactPoint], hole_indices: &[usize]) -> Result<TriangleIndices> {
    triangulate_with_kernel::<ExactKernel>(vertices, hole_indices)
}

/// Triangulate an exact polygon and return hot-loop diagnostics.
pub fn triangulate_report(vertices: &[ExactPoint], hole_indices: &[usize]) -> Result<EarcutReport> {
    triangulate_report_with_kernel::<ExactKernel>(vertices, hole_indices)
}

/// Triangulate a polygon with the provided numeric kernel.
pub fn triangulate_with_kernel<K>(
    vertices: &[Point2],
    hole_indices: &[usize],
) -> Result<TriangleIndices>
where
    K: Kernel,
{
    Ok(triangulate_report_with_kernel::<K>(vertices, hole_indices)?.triangles)
}

/// Triangulate a polygon with the provided numeric kernel and return hot-loop
/// diagnostics.
pub fn triangulate_report_with_kernel<K>(
    vertices: &[Point2],
    hole_indices: &[usize],
) -> Result<EarcutReport>
where
    K: Kernel,
{
    let rings = rings_from_hole_indices(vertices, hole_indices)?;
    let mut diagnostics = EarcutDiagnostics::default();
    // Structural-dispatch note: normalization discovers several facts that are
    // currently consumed locally and then discarded. Preserving "simple",
    // "convex", "monotone", exact-rational coordinate kind, and the count of
    // removed duplicate/collinear vertices would let the public runtime choose
    // fan, monotone partition, earcut, or CDT without re-scanning coordinates.
    let mut ring = normalized_ring::<K>(vertices, rings.exterior())?;
    if ring.len() < 3 {
        return Ok(EarcutReport {
            triangles: Vec::new(),
            diagnostics,
        });
    }

    let winding = ring_area_sign(vertices, &ring)?;
    if winding == Sign::Zero {
        return Ok(EarcutReport {
            triangles: Vec::new(),
            diagnostics,
        });
    }

    let holes = normalized_holes::<K>(vertices, rings.holes(), winding)?;
    if !holes.is_empty() {
        ring = bridge_holes::<K>(vertices, ring, &holes)?;
        ring = filter_ring::<K>(vertices, ring)?;
        if ring.len() < 3 {
            return Ok(EarcutReport {
                triangles: Vec::new(),
                diagnostics,
            });
        }
    }

    let triangles = clip_ring::<K>(vertices, ring, winding, &mut diagnostics)?;
    Ok(EarcutReport {
        triangles,
        diagnostics,
    })
}

fn normalized_ring<K>(vertices: &[Point2], range: RingRange) -> Result<Vec<usize>>
where
    K: Kernel,
{
    filter_ring::<K>(vertices, open_ring_indices(vertices, range))
}

fn normalized_holes<K>(
    vertices: &[Point2],
    ranges: &[RingRange],
    exterior_winding: Sign,
) -> Result<Vec<Vec<usize>>>
where
    K: Kernel,
{
    let mut holes = Vec::with_capacity(ranges.len());
    for &range in ranges {
        let mut hole = normalized_ring::<K>(vertices, range)?;
        if hole.len() < 3 {
            return Err(Error::InvalidInput {
                reason: "hole ring is degenerate",
            });
        }

        let hole_winding = ring_area_sign(vertices, &hole)?;
        if hole_winding == Sign::Zero {
            return Err(Error::InvalidInput {
                reason: "hole ring is degenerate",
            });
        }

        if hole_winding == exterior_winding {
            hole.reverse();
        }

        holes.push(hole);
    }

    Ok(holes)
}

fn bridge_holes<K>(
    vertices: &[Point2],
    mut boundary: Vec<usize>,
    holes: &[Vec<usize>],
) -> Result<Vec<usize>>
where
    K: Kernel,
{
    for hole in holes {
        // A visible diagonal from an exterior boundary to a hole converts a
        // polygon-with-holes into one simple boundary walk without changing the
        // represented region. This is the standard reduction described in
        // de Berg et al., Computational Geometry: Algorithms and Applications.
        let bridge = find_visible_bridge::<K>(vertices, &boundary, hole, holes)?;
        boundary = splice_hole(boundary, hole, bridge.boundary_pos, bridge.hole_pos);
    }

    Ok(boundary)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Bridge {
    boundary_pos: usize,
    hole_pos: usize,
}

fn find_visible_bridge<K>(
    vertices: &[Point2],
    boundary: &[usize],
    hole: &[usize],
    holes: &[Vec<usize>],
) -> Result<Bridge>
where
    K: Kernel,
{
    let hole_positions = positions_by_xy(vertices, hole)?;

    for hole_pos in hole_positions {
        let boundary_positions = positions_by_distance_then_xy(vertices, boundary, hole[hole_pos])?;
        for &boundary_pos in &boundary_positions {
            if bridge_is_visible::<K>(vertices, boundary, hole, holes, boundary_pos, hole_pos)? {
                return Ok(Bridge {
                    boundary_pos,
                    hole_pos,
                });
            }
        }
    }

    Err(Error::InvalidInput {
        reason: "hole cannot be connected by a visible bridge",
    })
}

fn bridge_is_visible<K>(
    vertices: &[Point2],
    boundary: &[usize],
    hole: &[usize],
    holes: &[Vec<usize>],
    boundary_pos: usize,
    hole_pos: usize,
) -> Result<bool>
where
    K: Kernel,
{
    let boundary_index = boundary[boundary_pos];
    let hole_index = hole[hole_pos];
    if points_equal(&vertices[boundary_index], &vertices[hole_index]) {
        return Ok(false);
    }

    let midpoint = K::midpoint(&vertices[boundary_index], &vertices[hole_index])?;
    if !predicates::point_in_ring_even_odd(vertices, boundary, &midpoint)? {
        return Ok(false);
    }

    for other_hole in holes {
        if predicates::point_in_ring_even_odd(vertices, other_hole, &midpoint)? {
            return Ok(false);
        }
    }

    if !segment_is_clear_of_ring(vertices, boundary_index, hole_index, boundary)? {
        return Ok(false);
    }

    for other_hole in holes {
        if !segment_is_clear_of_ring(vertices, boundary_index, hole_index, other_hole)? {
            return Ok(false);
        }
    }

    Ok(true)
}

fn segment_is_clear_of_ring(
    vertices: &[Point2],
    from: usize,
    to: usize,
    ring: &[usize],
) -> Result<bool> {
    for i in 0..ring.len() {
        let edge_from = ring[i];
        let edge_to = ring[(i + 1) % ring.len()];
        if points_equal(&vertices[edge_from], &vertices[edge_to]) {
            continue;
        }

        let intersection = predicates::segment_intersection(
            &vertices[from],
            &vertices[to],
            &vertices[edge_from],
            &vertices[edge_to],
        )?;
        if intersection.is_disjoint() {
            continue;
        }
        if intersection.is_endpoint_touch() {
            let touches_allowed_endpoint = same_point(vertices, edge_from, from)
                || same_point(vertices, edge_to, from)
                || same_point(vertices, edge_from, to)
                || same_point(vertices, edge_to, to);
            if touches_allowed_endpoint {
                continue;
            }
        }
        return Ok(false);
    }

    Ok(true)
}

fn splice_hole(
    boundary: Vec<usize>,
    hole: &[usize],
    boundary_pos: usize,
    hole_pos: usize,
) -> Vec<usize> {
    let boundary_index = boundary[boundary_pos];
    let hole_index = hole[hole_pos];
    let mut merged = Vec::with_capacity(boundary.len() + hole.len() + 2);

    merged.extend_from_slice(&boundary[..=boundary_pos]);
    for offset in 0..hole.len() {
        merged.push(hole[(hole_pos + offset) % hole.len()]);
    }
    merged.push(hole_index);
    merged.push(boundary_index);
    merged.extend_from_slice(&boundary[boundary_pos + 1..]);

    merged
}

fn positions_by_xy(vertices: &[Point2], ring: &[usize]) -> Result<Vec<usize>> {
    // Structural-dispatch note: this insertion sort is intentionally simple for
    // the first exact port. If polygon preprocessing retains axis-aligned
    // bounding boxes and exact dyadic/integer coordinate tags, hole-bridge
    // candidate ordering can switch to bucketed or radix-like keys while still
    // using exact comparisons to break ties.
    let mut positions = Vec::with_capacity(ring.len());
    for position in 0..ring.len() {
        let mut insert_at = positions.len();
        for (candidate_at, &candidate) in positions.iter().enumerate() {
            if compare_ring_positions(vertices, ring, position, candidate)? == Ordering::Less {
                insert_at = candidate_at;
                break;
            }
        }
        positions.insert(insert_at, position);
    }

    Ok(positions)
}

fn positions_by_distance_then_xy(
    vertices: &[Point2],
    ring: &[usize],
    from: usize,
) -> Result<Vec<usize>> {
    let mut positions = Vec::with_capacity(ring.len());
    for position in 0..ring.len() {
        let mut insert_at = positions.len();
        for (candidate_at, &candidate) in positions.iter().enumerate() {
            if compare_distance_then_xy(vertices, from, ring, position, candidate)?
                == Ordering::Less
            {
                insert_at = candidate_at;
                break;
            }
        }
        positions.insert(insert_at, position);
    }

    Ok(positions)
}

fn compare_distance_then_xy(
    vertices: &[Point2],
    from: usize,
    ring: &[usize],
    left_pos: usize,
    right_pos: usize,
) -> Result<Ordering> {
    match decide_hyperlimit_ordering(
        hyperlimit::compare_point2_distance_squared(
            &predicate_point(&vertices[from]),
            &predicate_point(&vertices[ring[left_pos]]),
            &predicate_point(&vertices[ring[right_pos]]),
        ),
        "compare_point2_distance_squared",
    )? {
        Ordering::Equal => compare_ring_positions(vertices, ring, left_pos, right_pos),
        ordering => Ok(ordering),
    }
}

fn compare_ring_positions(
    vertices: &[Point2],
    ring: &[usize],
    left_pos: usize,
    right_pos: usize,
) -> Result<Ordering> {
    compare_point_indices(vertices, ring[left_pos], ring[right_pos])
}

fn compare_point_indices(vertices: &[Point2], left: usize, right: usize) -> Result<Ordering> {
    decide_hyperlimit_ordering(
        hyperlimit::compare_point2_lexicographic(
            &predicate_point(&vertices[left]),
            &predicate_point(&vertices[right]),
        ),
        "compare_point2_lexicographic",
    )
}

fn filter_ring<K>(vertices: &[Point2], mut ring: Vec<usize>) -> Result<Vec<usize>>
where
    K: Kernel,
{
    if ring.len() < 3 {
        return Ok(ring);
    }

    let mut changed = true;
    while changed && ring.len() >= 3 {
        changed = false;
        let mut i = 0;
        while i < ring.len() {
            let len = ring.len();
            let prev = ring[(i + len - 1) % len];
            let curr = ring[i];
            let next = ring[(i + 1) % len];

            let duplicate = points_equal(&vertices[curr], &vertices[next])
                || points_equal(&vertices[prev], &vertices[curr]);
            let collinear =
                predicates::orient2d::<K>(&vertices[prev], &vertices[curr], &vertices[next])?
                    == Sign::Zero;

            if duplicate || (collinear && ring.len() > 3) {
                ring.remove(i);
                changed = true;
            } else {
                i += 1;
            }
        }
    }

    if ring.len() == 3
        && predicates::orient2d::<K>(&vertices[ring[0]], &vertices[ring[1]], &vertices[ring[2]])?
            == Sign::Zero
    {
        ring.clear();
    }

    Ok(ring)
}

fn clip_ring<K>(
    vertices: &[Point2],
    ring: Vec<usize>,
    winding: Sign,
    diagnostics: &mut EarcutDiagnostics,
) -> Result<TriangleIndices>
where
    K: Kernel,
{
    clip_ring_with_splits::<K>(vertices, ring, winding, 0, diagnostics)
}

fn clip_ring_with_splits<K>(
    vertices: &[Point2],
    mut ring: Vec<usize>,
    winding: Sign,
    split_depth: usize,
    diagnostics: &mut EarcutDiagnostics,
) -> Result<TriangleIndices>
where
    K: Kernel,
{
    let mut triangles = Vec::with_capacity((ring.len().saturating_sub(2)) * 3);
    let mut cursor = 0;
    let mut misses = 0;
    let mut guard = ring.len() * ring.len() * 4 + 1;
    let mut prepared_convex = prepare_local_convexity::<K>(vertices, &ring, winding, diagnostics)?;

    while ring.len() > 3 {
        if guard == 0 {
            let (cured_ring, mut cured_triangles, cured) =
                cure_local_intersections::<K>(vertices, ring, winding)?;
            if cured {
                triangles.append(&mut cured_triangles);
                triangles.append(&mut clip_ring_with_splits::<K>(
                    vertices,
                    cured_ring,
                    winding,
                    split_depth,
                    diagnostics,
                )?);
                return Ok(triangles);
            }
            return split_or_fail::<K>(vertices, cured_ring, winding, split_depth, diagnostics);
        }
        guard -= 1;

        if is_ear::<K>(vertices, &ring, &prepared_convex, cursor, diagnostics)? {
            let len = ring.len();
            let prev = ring[(cursor + len - 1) % len];
            let curr = ring[cursor];
            let next = ring[(cursor + 1) % len];
            push_triangle(&mut triangles, prev, curr, next, winding);
            diagnostics.emitted_triangles += 1;
            ring.remove(cursor);
            prepared_convex.remove(cursor);
            if cursor == ring.len() {
                cursor = 0;
            }
            update_prepared_convexity_after_clip::<K>(
                vertices,
                &ring,
                &mut prepared_convex,
                cursor,
                winding,
                diagnostics,
            )?;
            misses = 0;
            continue;
        }

        cursor = (cursor + 1) % ring.len();
        misses += 1;

        if misses > ring.len() {
            let previous_len = ring.len();
            ring = filter_ring::<K>(vertices, ring)?;
            if ring.len() < 3 {
                return Ok(triangles);
            }
            prepared_convex = prepare_local_convexity::<K>(vertices, &ring, winding, diagnostics)?;
            if ring.len() == previous_len {
                let (cured_ring, mut cured_triangles, cured) =
                    cure_local_intersections::<K>(vertices, ring, winding)?;
                triangles.append(&mut cured_triangles);
                if cured {
                    diagnostics.local_intersection_cures += 1;
                    diagnostics.emitted_triangles += cured_triangles.len() / 3;
                    ring = cured_ring;
                    prepared_convex =
                        prepare_local_convexity::<K>(vertices, &ring, winding, diagnostics)?;
                    cursor = 0;
                    misses = 0;
                    guard = ring.len() * ring.len() * 4 + 1;
                    continue;
                }

                let mut split_triangles =
                    split_or_fail::<K>(vertices, cured_ring, winding, split_depth, diagnostics)?;
                triangles.append(&mut split_triangles);
                return Ok(triangles);
            }
            cursor %= ring.len();
            misses = 0;
        }
    }

    let sign = K::orient2d(&vertices[ring[0]], &vertices[ring[1]], &vertices[ring[2]])?;
    if sign != Sign::Zero {
        push_triangle(&mut triangles, ring[0], ring[1], ring[2], sign);
        diagnostics.emitted_triangles += 1;
    }

    Ok(triangles)
}

fn cure_local_intersections<K>(
    vertices: &[Point2],
    ring: Vec<usize>,
    _winding: Sign,
) -> Result<(Vec<usize>, TriangleIndices, bool)>
where
    K: Kernel,
{
    if ring.len() < 4 {
        return Ok((ring, TriangleIndices::new(), false));
    }

    for cursor in 0..ring.len() {
        let len = ring.len();
        let a_pos = (cursor + len - 1) % len;
        let p_pos = cursor;
        let q_pos = (cursor + 1) % len;
        let b_pos = (cursor + 2) % len;

        if local_intersection_is_curable::<K>(vertices, &ring, a_pos, p_pos, q_pos, b_pos)? {
            let a = ring[a_pos];
            let p = ring[p_pos];
            let b = ring[b_pos];
            let mut cured_ring = ring.clone();

            // This is earcut's local-intersection cure: when two nearby
            // boundary edges cross, emit the triangle that bridges around the
            // twist and remove the two offending vertices. The exact segment
            // and orientation tests keep the fallback in Yap/Shewchuk-style
            // predicate territory instead of inheriting earcutr's float tests.
            remove_adjacent_positions(&mut cured_ring, p_pos, q_pos);

            let sign = predicates::orient2d::<K>(&vertices[a], &vertices[p], &vertices[b])?;
            if sign == Sign::Zero {
                return Ok((ring, TriangleIndices::new(), false));
            }

            let mut triangles = TriangleIndices::new();
            push_triangle(&mut triangles, a, p, b, sign);
            return Ok((cured_ring, triangles, true));
        }
    }

    Ok((ring, TriangleIndices::new(), false))
}

fn local_intersection_is_curable<K>(
    vertices: &[Point2],
    ring: &[usize],
    a_pos: usize,
    p_pos: usize,
    q_pos: usize,
    b_pos: usize,
) -> Result<bool>
where
    K: Kernel,
{
    let a = ring[a_pos];
    let p = ring[p_pos];
    let q = ring[q_pos];
    let b = ring[b_pos];

    if same_point(vertices, a, b) || same_point(vertices, p, q) {
        return Ok(false);
    }

    if !predicates::segment_intersection(&vertices[a], &vertices[p], &vertices[q], &vertices[b])?
        .is_proper_crossing()
    {
        return Ok(false);
    }

    if predicates::orient2d::<K>(&vertices[a], &vertices[p], &vertices[b])? == Sign::Zero {
        return Ok(false);
    }

    diagonal_clear_after_removing_local_pair(vertices, ring, a, p, q, b)
}

fn diagonal_clear_after_removing_local_pair(
    vertices: &[Point2],
    ring: &[usize],
    from: usize,
    removed_first: usize,
    removed_second: usize,
    to: usize,
) -> Result<bool> {
    for i in 0..ring.len() {
        let edge_from = ring[i];
        let edge_to = ring[(i + 1) % ring.len()];
        if edge_from == removed_first
            || edge_from == removed_second
            || edge_to == removed_first
            || edge_to == removed_second
            || edge_from == from
            || edge_to == from
            || edge_from == to
            || edge_to == to
        {
            continue;
        }

        if predicates::segment_intersection(
            &vertices[from],
            &vertices[to],
            &vertices[edge_from],
            &vertices[edge_to],
        )?
        .intersects()
        {
            return Ok(false);
        }
    }

    Ok(true)
}

fn remove_adjacent_positions(ring: &mut Vec<usize>, first: usize, second: usize) {
    debug_assert!((first + 1) % ring.len() == second);
    if second > first {
        ring.remove(second);
        ring.remove(first);
    } else {
        ring.remove(first);
        ring.remove(second);
    }
}

fn split_or_fail<K>(
    vertices: &[Point2],
    ring: Vec<usize>,
    winding: Sign,
    split_depth: usize,
    diagnostics: &mut EarcutDiagnostics,
) -> Result<TriangleIndices>
where
    K: Kernel,
{
    if split_depth > ring.len() {
        return Err(Error::NoEarFound);
    }
    diagnostics.split_fallbacks += 1;

    let Some((first, second)) = find_split_diagonal::<K>(vertices, &ring)? else {
        return Err(Error::NoEarFound);
    };

    // Earcut's final fallback splits a difficult polygon by a valid internal
    // diagonal and resumes ear clipping on each side. The validity tests here
    // are exact: the diagonal must not cross the boundary, and its midpoint
    // must lie inside the represented region. See de Berg et al.,
    // Computational Geometry: Algorithms and Applications, for the diagonal
    // decomposition model.
    let (mut left, mut right) = split_ring(&ring, first, second);
    left = filter_ring::<K>(vertices, left)?;
    right = filter_ring::<K>(vertices, right)?;

    let mut triangles = TriangleIndices::new();
    if left.len() >= 3 {
        triangles.append(&mut clip_ring_with_splits::<K>(
            vertices,
            left,
            winding,
            split_depth + 1,
            diagnostics,
        )?);
    }
    if right.len() >= 3 {
        triangles.append(&mut clip_ring_with_splits::<K>(
            vertices,
            right,
            winding,
            split_depth + 1,
            diagnostics,
        )?);
    }

    Ok(triangles)
}

fn find_split_diagonal<K>(vertices: &[Point2], ring: &[usize]) -> Result<Option<(usize, usize)>>
where
    K: Kernel,
{
    for gap in 2..ring.len().saturating_sub(1) {
        for first in 0..ring.len() {
            let second = (first + gap) % ring.len();
            if positions_are_adjacent(ring.len(), first, second) {
                continue;
            }
            if diagonal_is_valid::<K>(vertices, ring, first, second)? {
                return Ok(Some(ordered_positions(first, second)));
            }
        }
    }

    Ok(None)
}

fn diagonal_is_valid<K>(
    vertices: &[Point2],
    ring: &[usize],
    first_pos: usize,
    second_pos: usize,
) -> Result<bool>
where
    K: Kernel,
{
    let first = ring[first_pos];
    let second = ring[second_pos];
    if same_point(vertices, first, second) {
        return Ok(false);
    }

    if !segment_is_clear_of_ring(vertices, first, second, ring)? {
        return Ok(false);
    }

    let midpoint = K::midpoint(&vertices[first], &vertices[second])?;
    predicates::point_in_ring_even_odd(vertices, ring, &midpoint)
}

fn split_ring(ring: &[usize], first: usize, second: usize) -> (Vec<usize>, Vec<usize>) {
    debug_assert!(first < second);
    let mut left = Vec::with_capacity(second - first + 1);
    left.extend_from_slice(&ring[first..=second]);

    let mut right = Vec::with_capacity(ring.len() - (second - first) + 1);
    right.extend_from_slice(&ring[second..]);
    right.extend_from_slice(&ring[..=first]);

    (left, right)
}

fn positions_are_adjacent(len: usize, first: usize, second: usize) -> bool {
    first == second || (first + 1) % len == second || (second + 1) % len == first
}

fn ordered_positions(first: usize, second: usize) -> (usize, usize) {
    if first < second {
        (first, second)
    } else {
        (second, first)
    }
}

fn is_ear<K>(
    vertices: &[Point2],
    ring: &[usize],
    prepared_convex: &[bool],
    cursor: usize,
    diagnostics: &mut EarcutDiagnostics,
) -> Result<bool>
where
    K: Kernel,
{
    diagnostics.ear_tests += 1;
    let len = ring.len();
    let prev = ring[(cursor + len - 1) % len];
    let curr = ring[cursor];
    let next = ring[(cursor + 1) % len];

    // Meisters proved every simple polygon with more than three vertices has
    // at least two ears. The exact predicate here is the convexity gate for
    // that theorem; containment below rejects ears that would cover another
    // vertex. See Meisters, "Polygons Have Ears" (1975).
    //
    // Structural-dispatch note: the loop uses two exact object facts before
    // the full containment predicate: the local reflex/convex turn and the
    // candidate triangle AABB. Keeping these as facts, rather than as
    // floating-point heuristics, mirrors Yap's EGC advice to preserve useful
    // geometric structure before changing arithmetic packages; see Yap,
    // "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    // (1997).
    debug_assert_eq!(ring.len(), prepared_convex.len());
    if !prepared_convex[cursor] {
        return Ok(false);
    }

    for (candidate_cursor, &candidate) in ring.iter().enumerate() {
        if candidate == prev || candidate == curr || candidate == next {
            continue;
        }

        diagnostics.containment_candidates += 1;
        diagnostics.containment_prepared_reflex_lookups += 1;
        if prepared_convex[candidate_cursor] {
            diagnostics.containment_convex_rejects += 1;
            continue;
        }

        if !point_in_triangle_bbox(
            &vertices[prev],
            &vertices[curr],
            &vertices[next],
            &vertices[candidate],
        )? {
            diagnostics.containment_bbox_rejects += 1;
            continue;
        }

        diagnostics.containment_tests += 1;
        if predicates::point_in_or_on_triangle::<K>(
            &vertices[prev],
            &vertices[curr],
            &vertices[next],
            &vertices[candidate],
        )? {
            return Ok(false);
        }
    }

    Ok(true)
}

fn prepare_local_convexity<K>(
    vertices: &[Point2],
    ring: &[usize],
    winding: Sign,
    diagnostics: &mut EarcutDiagnostics,
) -> Result<Vec<bool>>
where
    K: Kernel,
{
    diagnostics.prepared_reflex_rebuilds += 1;
    (0..ring.len())
        .map(|cursor| local_vertex_is_strictly_convex::<K>(vertices, ring, cursor, winding))
        .collect()
}

fn update_prepared_convexity_after_clip<K>(
    vertices: &[Point2],
    ring: &[usize],
    prepared_convex: &mut [bool],
    cursor: usize,
    winding: Sign,
    diagnostics: &mut EarcutDiagnostics,
) -> Result<()>
where
    K: Kernel,
{
    if ring.len() < 3 {
        return Ok(());
    }
    let next = cursor % ring.len();
    let prev = (next + ring.len() - 1) % ring.len();
    prepared_convex[prev] = local_vertex_is_strictly_convex::<K>(vertices, ring, prev, winding)?;
    diagnostics.prepared_reflex_updates += 1;
    if next != prev {
        prepared_convex[next] =
            local_vertex_is_strictly_convex::<K>(vertices, ring, next, winding)?;
        diagnostics.prepared_reflex_updates += 1;
    }
    Ok(())
}

fn local_vertex_is_strictly_convex<K>(
    vertices: &[Point2],
    ring: &[usize],
    cursor: usize,
    winding: Sign,
) -> Result<bool>
where
    K: Kernel,
{
    // Standard ear-clipping implementations only need to test reflex vertices
    // for containment in a candidate ear. Hypertri prepares that local
    // convex/reflex bit once per active ring state, then updates only the two
    // vertices whose neighborhoods changed after clipping. This is the
    // object-fact reuse pattern Yap recommends in "Towards Exact Geometric
    // Computation," *Computational Geometry* 7.1-2 (1997), applied to the
    // exact orientation predicate used by Meisters' two-ears theorem; see
    // Meisters, "Polygons Have Ears," *The American Mathematical Monthly*
    // 82.6 (1975). Collinear vertices remain non-convex here so degeneracies
    // still flow to AABB and containment predicates.
    let len = ring.len();
    let prev = ring[(cursor + len - 1) % len];
    let curr = ring[cursor];
    let next = ring[(cursor + 1) % len];
    Ok(predicates::orient2d::<K>(&vertices[prev], &vertices[curr], &vertices[next])? == winding)
}

fn point_in_triangle_bbox(a: &Point2, b: &Point2, c: &Point2, point: &Point2) -> Result<bool> {
    // This is only a rejection filter. Exact coordinate comparisons prove that
    // a point outside the triangle's axis-aligned bounding box cannot be inside
    // the triangle, while points inside the box still go through the exact
    // orientation-based containment predicate. The filter is the
    // axis-aligned-box stage described in broad-phase geometry texts such as
    // Ericson, *Real-Time Collision Detection* (2005), but it is evaluated with
    // the crate's exact kernel rather than primitive floats.
    decide_hyperlimit_bool(
        hyperlimit::point_in_triangle2_aabb(
            &predicate_point(a),
            &predicate_point(b),
            &predicate_point(c),
            &predicate_point(point),
        ),
        "point_in_triangle2_aabb",
    )
}

fn ring_area_sign(vertices: &[Point2], ring: &[usize]) -> Result<Sign> {
    let predicate_vertices: Vec<_> = vertices.iter().map(predicate_point).collect();
    decide_hyperlimit_sign(
        hyperlimit::indexed_ring_area_sign(&predicate_vertices, ring),
        "indexed_ring_area_sign",
    )
}

fn push_triangle(triangles: &mut TriangleIndices, a: usize, b: usize, c: usize, winding: Sign) {
    match winding {
        Sign::Positive | Sign::Zero => triangles.extend_from_slice(&[a, b, c]),
        Sign::Negative => triangles.extend_from_slice(&[c, b, a]),
    }
}

fn points_equal(left: &ExactPoint, right: &ExactPoint) -> bool {
    left.x == right.x && left.y == right.y
}

fn same_point(vertices: &[ExactPoint], left: usize, right: usize) -> bool {
    points_equal(&vertices[left], &vertices[right])
}

fn predicate_point(point: &Point2) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(point.x.clone(), point.y.clone())
}

fn decide_hyperlimit_bool(
    outcome: hyperlimit::PredicateOutcome<bool>,
    predicate: &'static str,
) -> Result<bool> {
    match outcome {
        hyperlimit::PredicateOutcome::Decided { value, .. } => Ok(value),
        hyperlimit::PredicateOutcome::Unknown { .. } => {
            Err(Error::PredicateUndecided { predicate })
        }
    }
}

fn decide_hyperlimit_ordering(
    outcome: hyperlimit::PredicateOutcome<Ordering>,
    predicate: &'static str,
) -> Result<Ordering> {
    match outcome {
        hyperlimit::PredicateOutcome::Decided { value, .. } => Ok(value),
        hyperlimit::PredicateOutcome::Unknown { .. } => {
            Err(Error::PredicateUndecided { predicate })
        }
    }
}

fn decide_hyperlimit_sign(
    outcome: hyperlimit::PredicateOutcome<hyperlimit::Sign>,
    predicate: &'static str,
) -> Result<Sign> {
    match outcome {
        hyperlimit::PredicateOutcome::Decided { value, .. } => Ok(map_hyperlimit_sign(value)),
        hyperlimit::PredicateOutcome::Unknown { .. } => {
            Err(Error::PredicateUndecided { predicate })
        }
    }
}

fn map_hyperlimit_sign(sign: hyperlimit::Sign) -> Sign {
    match sign {
        hyperlimit::Sign::Negative => Sign::Negative,
        hyperlimit::Sign::Zero => Sign::Zero,
        hyperlimit::Sign::Positive => Sign::Positive,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::ExactKernel;
    use crate::types::Real;

    fn exact_point(x: i32, y: i32) -> ExactPoint {
        Point2::new(Real::from(x), Real::from(y))
    }

    fn exact_fraction_point(xn: i64, xd: u64, yn: i64, yd: u64) -> ExactPoint {
        Point2::new(
            Real::from(crate::types::Rational::fraction(xn, xd).unwrap()),
            Real::from(crate::types::Rational::fraction(yn, yd).unwrap()),
        )
    }

    #[test]
    fn triangulates_exact_square() {
        let vertices = vec![
            exact_point(0, 0),
            exact_point(1, 0),
            exact_point(1, 1),
            exact_point(0, 1),
        ];

        let triangles = triangulate(&vertices, &[]).unwrap();

        assert_eq!(triangles.len(), 6);
    }

    #[test]
    fn triangulates_exact_concave_polygon() {
        let vertices = vec![
            exact_point(0, 0),
            exact_point(2, 0),
            exact_point(2, 2),
            exact_point(1, 1),
            exact_point(0, 2),
        ];

        let triangles = triangulate_with_kernel::<ExactKernel>(&vertices, &[]).unwrap();

        assert_eq!(triangles.len(), 9);
    }

    #[test]
    fn triangulates_square_with_square_hole() {
        let vertices = vec![
            exact_point(0, 0),
            exact_point(4, 0),
            exact_point(4, 4),
            exact_point(0, 4),
            exact_point(1, 1),
            exact_point(3, 1),
            exact_point(3, 3),
            exact_point(1, 3),
        ];

        let triangles = triangulate(&vertices, &[4]).unwrap();

        assert_eq!(triangles.len(), 24);
        assert!(triangles.iter().all(|&index| index < vertices.len()));
    }

    #[test]
    fn triangulates_multiple_holes_with_mixed_winding() {
        let vertices = vec![
            exact_point(0, 0),
            exact_point(12, 0),
            exact_point(12, 8),
            exact_point(0, 8),
            exact_point(1, 1),
            exact_point(3, 1),
            exact_point(3, 3),
            exact_point(1, 3),
            exact_point(8, 5),
            exact_point(10, 5),
            exact_point(10, 7),
            exact_point(8, 7),
        ];

        let triangles = triangulate(&vertices, &[4, 8]).unwrap();

        assert_eq!(triangles.len(), 42);
        assert!(triangles.iter().all(|&index| index < vertices.len()));
    }

    #[test]
    fn triangulates_hole_with_duplicate_closing_vertex() {
        let vertices = vec![
            exact_point(0, 0),
            exact_point(5, 0),
            exact_point(5, 5),
            exact_point(0, 5),
            exact_point(1, 1),
            exact_point(3, 1),
            exact_point(3, 3),
            exact_point(1, 3),
            exact_point(1, 1),
        ];

        let triangles = triangulate(&vertices, &[4]).unwrap();

        assert_eq!(triangles.len(), 24);
        assert!(triangles.iter().all(|&index| index < 8));
    }

    #[test]
    fn earcut_report_matches_plain_triangulation_and_counts_hot_loop() {
        let vertices = vec![
            exact_point(0, 0),
            exact_point(4, 0),
            exact_point(4, 3),
            exact_point(2, 1),
            exact_point(0, 3),
        ];

        let plain = triangulate(&vertices, &[]).unwrap();
        let report = triangulate_report(&vertices, &[]).unwrap();

        assert_eq!(report.triangles, plain);
        assert_eq!(report.diagnostics.emitted_triangles * 3, plain.len());
        assert!(report.diagnostics.ear_tests > 0);
        assert!(report.diagnostics.containment_candidates > 0);
        assert!(
            report.diagnostics.prepared_reflex_rebuilds > 0,
            "earcut should prepare exact local convexity facts before scanning ears"
        );
        assert!(
            report.diagnostics.prepared_reflex_updates > 0,
            "ear clipping should update the prepared local convexity facts after removing vertices"
        );
        assert_eq!(
            report.diagnostics.containment_prepared_reflex_lookups,
            report.diagnostics.containment_candidates,
            "every containment candidate should route through the prepared reflex/convex fact table"
        );
        assert!(
            report.diagnostics.containment_convex_rejects > 0,
            "exact reflex filter should reject at least one strictly convex candidate"
        );
        assert!(
            report.diagnostics.containment_convex_rejects
                + report.diagnostics.containment_bbox_rejects
                > 0,
            "at least one exact containment prefilter should reject a candidate"
        );
        assert!(
            report.diagnostics.containment_tests
                <= report.diagnostics.containment_candidates
                    - report.diagnostics.containment_convex_rejects
                    - report.diagnostics.containment_bbox_rejects
        );
    }

    #[test]
    fn split_fallback_triangulates_forced_valid_diagonal() {
        let vertices = vec![
            exact_point(0, 0),
            exact_point(2, 0),
            exact_point(3, 1),
            exact_point(2, 2),
            exact_point(0, 2),
            exact_point(-1, 1),
        ];
        let ring = vec![0, 1, 2, 3, 4, 5];

        let mut diagnostics = EarcutDiagnostics::default();
        let triangles =
            split_or_fail::<ExactKernel>(&vertices, ring, Sign::Positive, 0, &mut diagnostics)
                .unwrap();

        assert_eq!(triangles.len(), 12);
        assert_eq!(diagnostics.split_fallbacks, 1);
        assert!(triangles.iter().all(|&index| index < vertices.len()));
    }

    #[test]
    fn local_intersection_cure_removes_twisted_pair() {
        let vertices = vec![
            exact_point(0, 0),
            exact_point(3, 3),
            exact_point(0, 3),
            exact_point(3, 0),
            exact_point(4, 4),
            exact_point(-1, 4),
        ];
        let ring = vec![0, 1, 2, 3, 4, 5];

        let (cured_ring, triangles, cured) =
            cure_local_intersections::<ExactKernel>(&vertices, ring, Sign::Positive).unwrap();

        assert!(cured);
        assert_eq!(cured_ring, vec![0, 3, 4, 5]);
        assert_eq!(triangles.len(), 3);
        assert!(triangles.iter().all(|&index| index < vertices.len()));
    }

    #[test]
    fn drops_duplicate_closing_vertex() {
        let vertices = vec![
            exact_point(0, 0),
            exact_point(1, 0),
            exact_point(1, 1),
            exact_point(0, 1),
            exact_point(0, 0),
        ];

        let triangles = triangulate(&vertices, &[]).unwrap();

        assert_eq!(triangles.len(), 6);
        assert!(triangles.iter().all(|&index| index < 4));
    }

    #[test]
    fn filters_collinear_chain_without_losing_area() {
        let vertices = vec![
            exact_point(0, 0),
            exact_point(1, 0),
            exact_point(2, 0),
            exact_point(2, 1),
            exact_point(0, 1),
        ];

        let triangles = triangulate(&vertices, &[]).unwrap();

        assert_eq!(triangles.len(), 6);
    }

    #[test]
    fn handles_near_collinear_exact_rational_not_epsilon_collapsed() {
        let vertices = vec![
            exact_point(0, 0),
            exact_fraction_point(1, 1_000_000_000, 1, 1_000_000_000_000),
            exact_point(2, 0),
            exact_point(2, 1),
            exact_point(0, 1),
        ];

        let triangles = triangulate(&vertices, &[]).unwrap();

        assert_eq!(triangles.len(), 9);
    }

    #[test]
    fn rejects_unsorted_hole_indices_before_algorithm_dispatch() {
        let vertices = vec![
            exact_point(0, 0),
            exact_point(4, 0),
            exact_point(4, 4),
            exact_point(0, 4),
            exact_point(1, 1),
            exact_point(3, 1),
            exact_point(3, 3),
            exact_point(1, 3),
        ];

        let error = triangulate(&vertices, &[5, 4]).unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "hole indices must be strictly increasing interior starts"
            }
        );
    }
}
