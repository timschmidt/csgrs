//! Exact validation helpers for Delaunay and constrained-Delaunay topology.
//!
//! These routines intentionally stay separate from insertion code. They are
//! useful in tests and downstream debug checks, and they document the invariants
//! maintained by the current port: protected PSLG edges are present in the
//! output triangulation, and every unprotected interior edge can be checked with
//! the same empty-circle predicate used by Delaunay insertion.

use crate::error::{Error, Result};
use crate::kernel::{ExactKernel, Kernel};
use crate::predicates;
use crate::types::Sign;
use crate::types::{Constraint, ExactPoint, Triangle};

/// Validate unconstrained exact Delaunay topology and local edge legality.
pub(crate) fn validate_delaunay(points: &[ExactPoint], triangles: &[Triangle]) -> Result<()> {
    validate_triangles(points, triangles)?;
    validate_edge_adjacency(triangles)?;
    validate_local_delaunay(points, triangles, &[])
}

/// Validate exact constrained triangulation topology without Delaunay legality.
pub(crate) fn validate_constrained_topology(
    points: &[ExactPoint],
    constraints: &[Constraint],
    triangles: &[Triangle],
) -> Result<()> {
    validate_constraints(points.len(), constraints)?;
    validate_triangles(points, triangles)?;
    validate_edge_adjacency(triangles)?;
    for &constraint in constraints {
        if !triangulation_has_edge(triangles, EdgeKey::from_constraint(constraint)) {
            return Err(Error::InvalidInput {
                reason: "constraint edge missing from triangulation",
            });
        }
    }
    Ok(())
}

/// Validate constrained topology and local Delaunay legality of unprotected
/// interior edges.
pub(crate) fn validate_constrained_delaunay(
    points: &[ExactPoint],
    constraints: &[Constraint],
    triangles: &[Triangle],
) -> Result<()> {
    validate_constrained_topology(points, constraints, triangles)?;
    let constrained_edges = constraints
        .iter()
        .copied()
        .map(EdgeKey::from_constraint)
        .collect::<Vec<_>>();
    validate_local_delaunay(points, triangles, &constrained_edges)
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

fn validate_triangles(points: &[ExactPoint], triangles: &[Triangle]) -> Result<()> {
    let mut seen = Vec::new();
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
        if predicates::orient2::<ExactKernel>(
            &points[triangle[0]],
            &points[triangle[1]],
            &points[triangle[2]],
        )? == Sign::Zero
        {
            return Err(Error::InvalidInput {
                reason: "triangle is degenerate",
            });
        }

        let normalized = normalized_triangle(triangle);
        if seen.contains(&normalized) {
            return Err(Error::InvalidInput {
                reason: "duplicate triangle",
            });
        }
        seen.push(normalized);
    }
    Ok(())
}

fn validate_edge_adjacency(triangles: &[Triangle]) -> Result<()> {
    for edge in unique_edges(triangles) {
        if adjacent_triangles(triangles, edge)?.len() > 2 {
            return Err(Error::InvalidInput {
                reason: "triangulation edge has more than two adjacent triangles",
            });
        }
    }
    Ok(())
}

fn validate_local_delaunay(
    points: &[ExactPoint],
    triangles: &[Triangle],
    constrained_edges: &[EdgeKey],
) -> Result<()> {
    for edge in unique_edges(triangles) {
        if constrained_edges.contains(&edge) {
            continue;
        }

        let adjacent = adjacent_triangles(triangles, edge)?;
        if adjacent.len() != 2 {
            continue;
        }

        let first = adjacent[0].opposite;
        let second = adjacent[1].opposite;
        if !opposite_sides_of_edge(points, edge, first, second)? {
            return Err(Error::InvalidInput {
                reason: "adjacent triangles are not on opposite sides of edge",
            });
        }

        if edge_is_illegal(points, edge, first, second)? {
            return Err(Error::InvalidInput {
                reason: "unconstrained interior edge violates Delaunay legality",
            });
        }
    }
    Ok(())
}

fn edge_is_illegal(
    points: &[ExactPoint],
    edge: EdgeKey,
    first_opposite: usize,
    second_opposite: usize,
) -> Result<bool> {
    let orientation = predicates::orient2::<ExactKernel>(
        &points[edge.from],
        &points[edge.to],
        &points[first_opposite],
    )?;
    if orientation == Sign::Zero {
        return Ok(false);
    }

    let sign = ExactKernel::incircle2(
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
    points: &[ExactPoint],
    edge: EdgeKey,
    first: usize,
    second: usize,
) -> Result<bool> {
    let first_side =
        predicates::orient2::<ExactKernel>(&points[edge.from], &points[edge.to], &points[first])?;
    let second_side =
        predicates::orient2::<ExactKernel>(&points[edge.from], &points[edge.to], &points[second])?;
    Ok(signs_strictly_differ(first_side, second_side))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct AdjacentTriangle {
    opposite: usize,
}

fn adjacent_triangles(triangles: &[Triangle], edge: EdgeKey) -> Result<Vec<AdjacentTriangle>> {
    let mut adjacent = Vec::new();
    for &triangle in triangles {
        if triangle_contains_edge(triangle, edge) {
            let Some(opposite) = triangle
                .iter()
                .copied()
                .find(|&index| !edge.contains(index))
            else {
                return Err(Error::InvalidInput {
                    reason: "triangle edge has no opposite vertex",
                });
            };
            adjacent.push(AdjacentTriangle { opposite });
        }
    }
    Ok(adjacent)
}

fn triangulation_has_edge(triangles: &[Triangle], edge: EdgeKey) -> bool {
    triangles
        .iter()
        .any(|&triangle| triangle_contains_edge(triangle, edge))
}

fn triangle_contains_edge(triangle: Triangle, edge: EdgeKey) -> bool {
    triangle.contains(&edge.from) && triangle.contains(&edge.to)
}

fn unique_edges(triangles: &[Triangle]) -> Vec<EdgeKey> {
    let mut edges = Vec::new();
    for triangle in triangles {
        for edge in [
            EdgeKey::new(triangle[0], triangle[1]),
            EdgeKey::new(triangle[1], triangle[2]),
            EdgeKey::new(triangle[2], triangle[0]),
        ] {
            if !edges.contains(&edge) {
                edges.push(edge);
            }
        }
    }
    edges
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
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

    fn contains(self, index: usize) -> bool {
        self.from == index || self.to == index
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
