//! Convex polygon clipping.

use crate::error::HypermeshResult;
use crate::geometry::{Aabb, Classification, Plane, classify_projective_point};
use crate::polygon::ConvexPolygon;

/// Result side from clipping a polygon against a plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClipSide {
    /// Entirely on the negative side.
    Left,
    /// Entirely on the positive side.
    Right,
    /// Straddles the plane.
    Both,
}

/// Polygon clipping result.
#[derive(Clone, Debug, PartialEq)]
pub struct ClipResult {
    /// Negative-side polygon.
    pub left: ConvexPolygon,
    /// Positive-side polygon.
    pub right: ConvexPolygon,
    /// Which side was produced.
    pub side: ClipSide,
}

/// Clips a convex polygon against a plane.
pub fn clip_polygon(poly: &ConvexPolygon, split_plane: &Plane) -> HypermeshResult<ClipResult> {
    let n = poly.vertex_count();
    if n < 3 {
        return Ok(ClipResult {
            left: poly.clone(),
            right: ConvexPolygon::empty(),
            side: ClipSide::Left,
        });
    }

    let mut classifications = Vec::with_capacity(n);
    let mut has_pos = false;
    let mut has_neg = false;
    for index in 0..n {
        let classification = classify_projective_point(&poly.vertex(index), split_plane)?;
        has_pos |= classification == Classification::Positive;
        has_neg |= classification == Classification::Negative;
        classifications.push(classification);
    }

    if !has_pos {
        return Ok(ClipResult {
            left: poly.clone(),
            right: ConvexPolygon::empty(),
            side: ClipSide::Left,
        });
    }
    if !has_neg {
        return Ok(ClipResult {
            left: ConvexPolygon::empty(),
            right: poly.clone(),
            side: ClipSide::Right,
        });
    }

    let q_inv = split_plane.inverted();
    let mut left_edges = Vec::with_capacity(n + 2);
    let mut right_edges = Vec::with_capacity(n + 2);

    for index in 0..n {
        let next = (index + 1) % n;
        let seg_edge = poly.edges[next].clone();
        match (
            classifications[index].is_non_positive(),
            classifications[next].is_non_positive(),
        ) {
            (true, true) => left_edges.push(seg_edge),
            (true, false) => {
                left_edges.push(seg_edge.clone());
                left_edges.push(split_plane.clone());
                right_edges.push(seg_edge);
            }
            (false, true) => {
                right_edges.push(seg_edge.clone());
                right_edges.push(q_inv.clone());
                left_edges.push(seg_edge);
            }
            (false, false) => right_edges.push(seg_edge),
        }
    }

    let mut left = poly.clone();
    left.edges = left_edges;
    let mut right = poly.clone();
    right.edges = right_edges;

    Ok(ClipResult {
        left,
        right,
        side: ClipSide::Both,
    })
}

/// Clips a polygon to an AABB, returning an empty polygon if outside.
pub fn clip_polygon_to_aabb(poly: &ConvexPolygon, aabb: &Aabb) -> HypermeshResult<ConvexPolygon> {
    let mut current = poly.clone();

    for axis in 0..3 {
        if current.edges.is_empty() {
            break;
        }

        let min_plane =
            Plane::axis_aligned(axis, crate::geometry::axis_ref(&aabb.min, axis).clone());
        let min_clip = clip_polygon(&current, &min_plane)?;
        current = match min_clip.side {
            ClipSide::Left => {
                let mut empty = current;
                empty.edges.clear();
                empty
            }
            ClipSide::Right => current,
            ClipSide::Both => min_clip.right,
        };

        if current.edges.is_empty() {
            break;
        }

        let max_plane =
            Plane::axis_aligned(axis, crate::geometry::axis_ref(&aabb.max, axis).clone());
        let max_clip = clip_polygon(&current, &max_plane)?;
        current = match max_clip.side {
            ClipSide::Right => {
                let mut empty = current;
                empty.edges.clear();
                empty
            }
            ClipSide::Left => current,
            ClipSide::Both => max_clip.left,
        };
    }

    Ok(current)
}
