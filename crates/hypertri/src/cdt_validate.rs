//! Exact validation helpers for Delaunay and constrained-Delaunay topology.
//!
//! These routines intentionally stay separate from insertion code. They are
//! useful in tests and downstream debug checks, and they document the invariants
//! maintained by the current port: protected PSLG edges are present in the
//! output triangulation, and every unprotected interior edge can be checked with
//! the same empty-circle predicate used by Delaunay insertion.

use crate::error::{Error, Result};
use crate::kernel::ExactKernel;
use crate::predicates;
use crate::types::Sign;
use crate::types::{Constraint, ExactPoint, Triangle};

/// Validate unconstrained exact Delaunay topology and local edge legality.
pub(crate) fn validate_delaunay(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    triangles: &[Triangle],
) -> Result<()> {
    let _ = validate_triangles(kernel, points, triangles)?;
    let edge_uses = sorted_edge_uses(triangles);
    validate_edge_adjacency(&edge_uses)?;
    if !triangulates_convex_hull_with_edge_uses(kernel, points, triangles, &edge_uses)? {
        return Err(Error::InvalidInput {
            reason: "triangulation does not cover the convex hull",
        });
    }
    validate_local_delaunay(kernel, points, &edge_uses, &[])
}

/// Return whether `triangles` form one complete triangulation of the convex
/// hull of every input point.
///
/// This catches finite-supertriangle artifacts that local empty-circle checks
/// cannot see: missing wedges appear as a concave boundary even though every
/// retained interior edge is locally legal.
pub(crate) fn triangulates_convex_hull(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    triangles: &[Triangle],
) -> Result<bool> {
    let edge_uses = sorted_edge_uses(triangles);
    triangulates_convex_hull_with_edge_uses(kernel, points, triangles, &edge_uses)
}

fn triangulates_convex_hull_with_edge_uses(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    triangles: &[Triangle],
    edge_uses: &[EdgeUse],
) -> Result<bool> {
    if triangles.is_empty() {
        return points_are_collinear(kernel, points);
    }

    let mut used = vec![false; points.len()];
    for triangle in triangles {
        if triangle.iter().any(|&vertex| vertex >= points.len()) {
            return Ok(false);
        }
        for &vertex in triangle {
            used[vertex] = true;
        }
    }
    if used.iter().any(|&is_used| !is_used) {
        return Ok(false);
    }

    let mut boundary = Vec::new();
    let mut parents = (0..triangles.len()).collect::<Vec<_>>();
    let mut edge_count = 0_usize;
    let mut start = 0;
    while start < edge_uses.len() {
        let mut end = start + 1;
        while end < edge_uses.len() && edge_uses[end].edge == edge_uses[start].edge {
            end += 1;
        }
        edge_count += 1;
        match end - start {
            1 => boundary.push(edge_uses[start].edge),
            2 => union(
                &mut parents,
                edge_uses[start].triangle,
                edge_uses[start + 1].triangle,
            ),
            _ => return Ok(false),
        }
        start = end;
    }

    let expected_root = root(&mut parents, 0);
    if (1..triangles.len()).any(|triangle| root(&mut parents, triangle) != expected_root)
        || points.len() as i128 - edge_count as i128 + triangles.len() as i128 != 1
        || boundary.len() < 3
    {
        return Ok(false);
    }

    let missing = usize::MAX;
    let mut neighbors = vec![[missing; 2]; points.len()];
    for edge in &boundary {
        for (vertex, neighbor) in [(edge.from, edge.to), (edge.to, edge.from)] {
            if neighbors[vertex][0] == missing {
                neighbors[vertex][0] = neighbor;
            } else if neighbors[vertex][1] == missing {
                neighbors[vertex][1] = neighbor;
            } else {
                return Ok(false);
            }
        }
    }

    let first = boundary[0].from;
    let mut cycle = Vec::with_capacity(boundary.len());
    let mut seen = vec![false; points.len()];
    let mut previous = missing;
    let mut current = first;
    for _ in 0..boundary.len() {
        if seen[current] || neighbors[current][1] == missing {
            return Ok(false);
        }
        seen[current] = true;
        cycle.push(current);
        let next = if neighbors[current][0] != previous {
            neighbors[current][0]
        } else {
            neighbors[current][1]
        };
        previous = current;
        current = next;
    }
    if current != first || cycle.len() != boundary.len() {
        return Ok(false);
    }

    let mut turn = None;
    for index in 0..cycle.len() {
        let sign = predicates::orient2(
            kernel,
            &points[cycle[(index + cycle.len() - 1) % cycle.len()]],
            &points[cycle[index]],
            &points[cycle[(index + 1) % cycle.len()]],
        )?;
        if sign == Sign::Zero {
            continue;
        }
        if turn.is_some_and(|turn| turn != sign) {
            return Ok(false);
        }
        turn = Some(sign);
    }
    Ok(turn.is_some())
}

fn points_are_collinear(kernel: &ExactKernel, points: &[ExactPoint]) -> Result<bool> {
    let Some(first) = points.first() else {
        return Ok(true);
    };
    let mut second = None;
    for candidate in &points[1..] {
        if !predicates::points_equal(kernel, first, candidate)? {
            second = Some(candidate);
            break;
        }
    }
    let Some(second) = second else {
        return Ok(true);
    };
    for point in points {
        if predicates::orient2(kernel, first, second, point)? != Sign::Zero {
            return Ok(false);
        }
    }
    Ok(true)
}

fn root(parents: &mut [usize], mut item: usize) -> usize {
    while parents[item] != item {
        parents[item] = parents[parents[item]];
        item = parents[item];
    }
    item
}

fn union(parents: &mut [usize], first: usize, second: usize) {
    let first = root(parents, first);
    let second = root(parents, second);
    if first != second {
        parents[second] = first;
    }
}

/// Validate exact constrained triangulation topology without Delaunay legality.
pub(crate) fn validate_constrained_topology(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    constraints: &[Constraint],
    triangles: &[Triangle],
) -> Result<()> {
    validated_constrained_edge_uses(kernel, points, constraints, triangles).map(drop)
}

fn validated_constrained_edge_uses(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    constraints: &[Constraint],
    triangles: &[Triangle],
) -> Result<(Vec<EdgeUse>, Option<Sign>)> {
    validate_constraints(points.len(), constraints)?;
    let winding = validate_triangles(kernel, points, triangles)?;
    let edge_uses = sorted_edge_uses(triangles);
    validate_edge_adjacency(&edge_uses)?;
    for &constraint in constraints {
        if !triangulation_has_edge(&edge_uses, EdgeKey::from_constraint(constraint)) {
            return Err(Error::InvalidInput {
                reason: "constraint edge missing from triangulation",
            });
        }
    }
    Ok((edge_uses, winding))
}

/// Validate constrained topology and local Delaunay legality of unprotected
/// interior edges.
pub(crate) fn validate_constrained_delaunay(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    constraints: &[Constraint],
    triangles: &[Triangle],
) -> Result<()> {
    let (edge_uses, _) = validated_constrained_edge_uses(kernel, points, constraints, triangles)?;
    let constrained_edges = sorted_constraint_edges(constraints);
    validate_local_delaunay(kernel, points, &edge_uses, &constrained_edges)
}

/// Validate a constrained Delaunay triangulation whose domain is the complete
/// convex hull, rather than a boundary-preserving polygon subset.
pub(crate) fn validate_constrained_convex_hull_delaunay(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    constraints: &[Constraint],
    triangles: &[Triangle],
) -> Result<()> {
    let (edge_uses, _) = validated_constrained_edge_uses(kernel, points, constraints, triangles)?;
    let constrained_edges = sorted_constraint_edges(constraints);
    validate_local_delaunay(kernel, points, &edge_uses, &constrained_edges)?;
    if !triangulates_convex_hull_with_edge_uses(kernel, points, triangles, &edge_uses)? {
        return Err(Error::InvalidInput {
            reason: "constrained triangulation does not cover the convex hull",
        });
    }
    Ok(())
}

/// Validate positively oriented constrained topology covering the complete
/// convex hull without imposing Delaunay legality on unprotected interior
/// edges.
pub(crate) fn validate_constrained_convex_hull_topology(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    constraints: &[Constraint],
    triangles: &[Triangle],
) -> Result<()> {
    let (edge_uses, winding) =
        validated_constrained_edge_uses(kernel, points, constraints, triangles)?;
    if winding == Some(Sign::Negative) {
        return Err(Error::InvalidInput {
            reason: "triangle winding is not positive",
        });
    }
    if !triangulates_convex_hull_with_edge_uses(kernel, points, triangles, &edge_uses)? {
        return Err(Error::InvalidInput {
            reason: "constrained triangulation does not cover the convex hull",
        });
    }
    Ok(())
}

fn sorted_constraint_edges(constraints: &[Constraint]) -> Vec<EdgeKey> {
    let mut edges = constraints
        .iter()
        .copied()
        .map(EdgeKey::from_constraint)
        .collect::<Vec<_>>();
    edges.sort_unstable();
    edges.dedup();
    edges
}

fn validate_constraints(point_count: usize, constraints: &[Constraint]) -> Result<()> {
    for constraint in constraints {
        if constraint.from >= point_count || constraint.to >= point_count {
            return Err(Error::InvalidInput {
                reason: "constraint edge index out of bounds",
            });
        }
        if constraint.from == constraint.to {
            return Err(Error::InvalidInput {
                reason: "constraint edge endpoints must differ",
            });
        }
    }
    Ok(())
}

fn validate_triangles(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    triangles: &[Triangle],
) -> Result<Option<Sign>> {
    let mut seen = Vec::with_capacity(triangles.len());
    let mut winding = None;
    for &triangle in triangles {
        if triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[2] == triangle[0] {
            return Err(Error::InvalidInput {
                reason: "triangle has duplicate vertex indices",
            });
        }
        if triangle.iter().any(|&index| index >= points.len()) {
            return Err(Error::InvalidInput {
                reason: "triangle index out of bounds",
            });
        }
        let sign = predicates::orient2(
            kernel,
            &points[triangle[0]],
            &points[triangle[1]],
            &points[triangle[2]],
        )?;
        if sign == Sign::Zero {
            return Err(Error::InvalidInput {
                reason: "triangle is degenerate",
            });
        }
        if winding.is_some_and(|winding| winding != sign) {
            return Err(Error::InvalidInput {
                reason: "triangle winding is inconsistent",
            });
        }
        winding = Some(sign);

        seen.push(normalized_triangle(triangle));
    }
    seen.sort_unstable();
    if seen.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(Error::InvalidInput {
            reason: "duplicate triangle",
        });
    }
    Ok(winding)
}

fn validate_edge_adjacency(edge_uses: &[EdgeUse]) -> Result<()> {
    let mut start = 0;
    while start < edge_uses.len() {
        let mut end = start + 1;
        while end < edge_uses.len() && edge_uses[end].edge == edge_uses[start].edge {
            end += 1;
        }
        if end - start > 2 {
            return Err(Error::InvalidInput {
                reason: "triangulation edge has more than two adjacent triangles",
            });
        }
        start = end;
    }
    Ok(())
}

fn validate_local_delaunay(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    edge_uses: &[EdgeUse],
    constrained_edges: &[EdgeKey],
) -> Result<()> {
    let mut start = 0;
    while start < edge_uses.len() {
        let mut end = start + 1;
        while end < edge_uses.len() && edge_uses[end].edge == edge_uses[start].edge {
            end += 1;
        }
        let edge = edge_uses[start].edge;
        if constrained_edges.binary_search(&edge).is_ok() {
            start = end;
            continue;
        }
        if end - start != 2 {
            start = end;
            continue;
        }

        let first = edge_uses[start].opposite;
        let second = edge_uses[start + 1].opposite;
        if !opposite_sides_of_edge(kernel, points, edge, first, second)? {
            return Err(Error::InvalidInput {
                reason: "adjacent triangles are not on opposite sides of edge",
            });
        }

        if edge_is_illegal(kernel, points, edge, first, second)? {
            return Err(Error::InvalidInput {
                reason: "unconstrained interior edge violates Delaunay legality",
            });
        }
        start = end;
    }
    Ok(())
}

fn edge_is_illegal(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    edge: EdgeKey,
    first_opposite: usize,
    second_opposite: usize,
) -> Result<bool> {
    let orientation = predicates::orient2(
        kernel,
        &points[edge.from],
        &points[edge.to],
        &points[first_opposite],
    )?;
    if orientation == Sign::Zero {
        return Ok(false);
    }

    let sign = kernel.incircle2(
        &points[edge.from],
        &points[edge.to],
        &points[first_opposite],
        &points[second_opposite],
    )?;
    Ok(matches!(
        (orientation, sign),
        (Sign::Positive, Sign::Positive) | (Sign::Negative, Sign::Negative)
    ))
}

fn opposite_sides_of_edge(
    kernel: &ExactKernel,
    points: &[ExactPoint],
    edge: EdgeKey,
    first: usize,
    second: usize,
) -> Result<bool> {
    let first_side =
        predicates::orient2(kernel, &points[edge.from], &points[edge.to], &points[first])?;
    let second_side = predicates::orient2(
        kernel,
        &points[edge.from],
        &points[edge.to],
        &points[second],
    )?;
    Ok(signs_strictly_differ(first_side, second_side))
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct EdgeUse {
    edge: EdgeKey,
    triangle: usize,
    opposite: usize,
}

fn sorted_edge_uses(triangles: &[Triangle]) -> Vec<EdgeUse> {
    let mut edge_uses = Vec::with_capacity(triangles.len().saturating_mul(3));
    for (triangle_index, triangle) in triangles.iter().copied().enumerate() {
        edge_uses.extend([
            EdgeUse {
                edge: EdgeKey::new(triangle[0], triangle[1]),
                triangle: triangle_index,
                opposite: triangle[2],
            },
            EdgeUse {
                edge: EdgeKey::new(triangle[1], triangle[2]),
                triangle: triangle_index,
                opposite: triangle[0],
            },
            EdgeUse {
                edge: EdgeKey::new(triangle[2], triangle[0]),
                triangle: triangle_index,
                opposite: triangle[1],
            },
        ]);
    }
    edge_uses.sort_unstable();
    edge_uses
}

fn triangulation_has_edge(edge_uses: &[EdgeUse], edge: EdgeKey) -> bool {
    edge_uses
        .binary_search_by_key(&edge, |edge_use| edge_use.edge)
        .is_ok()
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct EdgeKey {
    from: usize,
    to: usize,
}

impl EdgeKey {
    fn from_constraint(constraint: Constraint) -> Self {
        Self::new(constraint.from, constraint.to)
    }

    fn new(first: usize, second: usize) -> Self {
        if first <= second {
            Self {
                from: first,
                to: second,
            }
        } else {
            Self {
                from: second,
                to: first,
            }
        }
    }
}

fn normalized_triangle(mut triangle: Triangle) -> Triangle {
    triangle.sort();
    triangle
}

fn signs_strictly_differ(first: Sign, second: Sign) -> bool {
    matches!(
        (first, second),
        (Sign::Negative, Sign::Positive) | (Sign::Positive, Sign::Negative)
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TriangulationContext;
    use hyperreal::Real;

    #[test]
    fn convex_hull_topology_contract_rejects_consistent_negative_winding() {
        let context = TriangulationContext::new(hyperlimit::PredicatePolicy::STRICT);
        let kernel = ExactKernel::new(&context);
        let points = [
            ExactPoint::new(Real::from(0), Real::from(0)),
            ExactPoint::new(Real::from(1), Real::from(0)),
            ExactPoint::new(Real::from(1), Real::from(1)),
            ExactPoint::new(Real::from(0), Real::from(1)),
        ];

        let error = validate_constrained_convex_hull_topology(
            &kernel,
            &points,
            &[],
            &[[0, 2, 1], [0, 3, 2]],
        )
        .unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "triangle winding is not positive"
            }
        );
    }
}
