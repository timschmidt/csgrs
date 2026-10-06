//! Convex polygon clipping.

use crate::context::{DecisionContext, MeshContext, MeshOutcome};
use crate::error::HypermeshResult;
use crate::geometry::{Aabb, Classification, Plane};
use crate::polygon::ConvexPolygon;
use crate::predicate::classify_projective_point_decision;

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
pub fn clip_polygon(
    context: &MeshContext,
    poly: &ConvexPolygon,
    split_plane: &Plane,
) -> HypermeshResult<MeshOutcome<ClipResult>> {
    let decisions = DecisionContext::new(context);
    let result = clip_polygon_decision(&decisions, poly, split_plane)?;
    Ok(decisions.finish(result))
}

pub(crate) fn clip_polygon_decision(
    decisions: &DecisionContext,
    poly: &ConvexPolygon,
    split_plane: &Plane,
) -> HypermeshResult<ClipResult> {
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
        let classification =
            classify_projective_point_decision(decisions, &poly.vertex(index), split_plane)?;
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
        let seg_edge = poly.edge_planes()[next].clone();
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
    left.replace_edge_planes(left_edges);
    left.clear_known_vertices();
    left.known_identities = None;
    let mut right = poly.clone();
    right.replace_edge_planes(right_edges);
    right.clear_known_vertices();
    right.known_identities = None;

    Ok(ClipResult {
        left,
        right,
        side: ClipSide::Both,
    })
}

/// Clips a polygon to an AABB, returning an empty polygon if outside.
pub fn clip_polygon_to_aabb(
    context: &MeshContext,
    poly: &ConvexPolygon,
    aabb: &Aabb,
) -> HypermeshResult<MeshOutcome<ConvexPolygon>> {
    let decisions = DecisionContext::new(context);
    let polygon = clip_polygon_to_aabb_decision(&decisions, poly, aabb)?;
    Ok(decisions.finish(polygon))
}

pub(crate) fn clip_polygon_to_aabb_decision(
    decisions: &DecisionContext,
    poly: &ConvexPolygon,
    aabb: &Aabb,
) -> HypermeshResult<ConvexPolygon> {
    let mut current = poly.clone();

    for axis in 0..3 {
        if current.edge_planes().is_empty() {
            break;
        }

        let min_plane =
            Plane::axis_aligned(axis, crate::geometry::axis_ref(&aabb.min, axis).clone());
        if !polygon_lies_on_plane(decisions, &current, &min_plane)? {
            let min_clip = clip_polygon_decision(decisions, &current, &min_plane)?;
            current = match min_clip.side {
                ClipSide::Left => {
                    let mut empty = current;
                    empty.clear_edge_planes();
                    empty.clear_known_vertices();
                    empty.known_identities = None;
                    empty
                }
                ClipSide::Right => current,
                ClipSide::Both => min_clip.right,
            };
        }

        if current.edge_planes().is_empty() {
            break;
        }

        let max_plane =
            Plane::axis_aligned(axis, crate::geometry::axis_ref(&aabb.max, axis).clone());
        if !polygon_lies_on_plane(decisions, &current, &max_plane)? {
            let max_clip = clip_polygon_decision(decisions, &current, &max_plane)?;
            current = match max_clip.side {
                ClipSide::Right => {
                    let mut empty = current;
                    empty.clear_edge_planes();
                    empty.clear_known_vertices();
                    empty.known_identities = None;
                    empty
                }
                ClipSide::Left => current,
                ClipSide::Both => max_clip.left,
            };
        }
    }

    Ok(current)
}

fn polygon_lies_on_plane(
    decisions: &DecisionContext,
    poly: &ConvexPolygon,
    plane: &Plane,
) -> HypermeshResult<bool> {
    for index in 0..poly.vertex_count() {
        if classify_projective_point_decision(decisions, &poly.vertex(index), plane)?
            != Classification::On
        {
            return Ok(false);
        }
    }
    Ok(true)
}

#[cfg(test)]
mod tests {
    use hyperlattice::{Point3, Real};

    use super::clip_polygon_to_aabb_decision;
    use crate::geometry::Aabb;
    use crate::test_support::{approximate_convex_triangle, approximate_decisions};

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn clip_polygon_to_aabb_preserves_closed_boundary_faces() {
        let bounds = Aabb::new(p(0, -2, -2), p(2, 2, 2));
        for x in [0, 2] {
            let polygon =
                approximate_convex_triangle(&p(x, -1, -1), &p(x, 1, -1), &p(x, 0, 1), 0, 0);

            let clipped =
                clip_polygon_to_aabb_decision(&approximate_decisions(), &polygon, &bounds).unwrap();

            assert_eq!(clipped, polygon);
        }
    }
}
