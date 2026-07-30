//! Constraint graph recognition for polygonal CDT inputs.
//!
//! This module is deliberately private to `hypertri`: it recognizes the
//! polygon-with-holes subset of constrained triangulation while the full DCEL
//! port is still in progress. Keeping this recognition outside `cdt.rs`
//! preserves the public CDT abstraction boundary and keeps exact ring semantics
//! close to polygon normalization.

use std::cmp::Ordering;

use crate::error::Result;
use crate::kernel::ExactKernel;
use crate::predicates;
#[cfg(feature = "earcut")]
use crate::types::ExactPoint;
use crate::types::{Constraint, Point2};

/// Closed polygonal constraint rings ordered as exterior first, then holes.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ConstraintPolygon {
    rings: Vec<Vec<usize>>,
}

impl ConstraintPolygon {
    /// Convert ordered source-index rings into earcut-compatible flat buffers.
    #[cfg(feature = "earcut")]
    pub(crate) fn to_flat_polygon(
        &self,
        points: &[ExactPoint],
    ) -> (Vec<ExactPoint>, Vec<usize>, Vec<usize>) {
        let total_len = self.rings.iter().map(Vec::len).sum();
        let mut flat_points = Vec::with_capacity(total_len);
        let mut hole_indices = Vec::with_capacity(self.rings.len().saturating_sub(1));
        let mut source_indices = Vec::with_capacity(total_len);

        for (ring_index, ring) in self.rings.iter().enumerate() {
            if ring_index > 0 {
                hole_indices.push(flat_points.len());
            }
            for &source_index in ring {
                flat_points.push(points[source_index].clone());
                source_indices.push(source_index);
            }
        }

        (flat_points, hole_indices, source_indices)
    }
}

/// Recognize a disjoint set of closed constraint rings as one polygon.
///
/// The accepted subset is one exterior cycle plus zero or more hole cycles.
/// Ring ordering is decided by exact even-odd containment rather than by input
/// order. This is the same containment model used by the other polygon
/// algorithms in this crate.
pub(crate) fn polygon_from_closed_constraints(
    kernel: &ExactKernel,
    points: &[Point2],
    constraints: &[Constraint],
) -> Result<Option<ConstraintPolygon>> {
    let rings = extract_closed_rings(points.len(), constraints);
    let Some(rings) = rings else {
        return Ok(None);
    };

    order_polygon_rings(kernel, points, rings)
}

fn extract_closed_rings(point_count: usize, constraints: &[Constraint]) -> Option<Vec<Vec<usize>>> {
    if constraints.is_empty() || point_count < 3 {
        return None;
    }

    let mut adjacency = vec![Vec::<(usize, usize)>::new(); point_count];
    for (edge_index, edge) in constraints.iter().enumerate() {
        adjacency[edge.from].push((edge_index, edge.to));
        adjacency[edge.to].push((edge_index, edge.from));
    }

    if adjacency
        .iter()
        .any(|neighbors| !neighbors.is_empty() && neighbors.len() != 2)
    {
        return None;
    }

    let mut used_edges = vec![false; constraints.len()];
    let mut rings = Vec::new();

    for (edge_index, edge) in constraints.iter().enumerate() {
        if used_edges[edge_index] {
            continue;
        }

        let mut ring = Vec::new();
        let start = edge.from;
        let mut current = edge.to;
        let mut previous = start;
        used_edges[edge_index] = true;
        ring.push(start);

        while current != start {
            if ring.contains(&current) {
                return None;
            }
            ring.push(current);

            let next = adjacency[current]
                .iter()
                .copied()
                .find(|(candidate_edge, candidate_next)| {
                    !used_edges[*candidate_edge] && *candidate_next != previous
                })
                .or_else(|| {
                    adjacency[current]
                        .iter()
                        .copied()
                        .find(|(candidate_edge, _)| !used_edges[*candidate_edge])
                })?;

            used_edges[next.0] = true;
            previous = current;
            current = next.1;
        }

        if ring.len() < 3 {
            return None;
        }
        rings.push(ring);
    }

    used_edges.iter().all(|used| *used).then_some(rings)
}

fn order_polygon_rings(
    kernel: &ExactKernel,
    points: &[Point2],
    rings: Vec<Vec<usize>>,
) -> Result<Option<ConstraintPolygon>> {
    if rings.len() == 1 {
        return Ok(Some(ConstraintPolygon { rings }));
    }

    let Some(exterior_index) = exterior_ring_index(kernel, points, &rings)? else {
        return Ok(None);
    };

    let mut ordered = Vec::with_capacity(rings.len());
    ordered.push(rings[exterior_index].clone());

    let mut holes = Vec::<Vec<usize>>::with_capacity(rings.len() - 1);
    for (ring_index, ring) in rings.iter().enumerate() {
        if ring_index == exterior_index {
            continue;
        }
        if ring_is_inside_any_other_hole(kernel, points, &rings, exterior_index, ring_index)? {
            return Ok(None);
        }
        insert_hole_sorted(kernel, points, &mut holes, ring.clone())?;
    }
    ordered.extend(holes);

    Ok(Some(ConstraintPolygon { rings: ordered }))
}

fn insert_hole_sorted(
    kernel: &ExactKernel,
    points: &[Point2],
    holes: &mut Vec<Vec<usize>>,
    hole: Vec<usize>,
) -> Result<()> {
    let mut insert_at = holes.len();
    for (candidate_at, candidate) in holes.iter().enumerate() {
        if compare_ring_representatives(kernel, points, &hole, candidate)? == Ordering::Less {
            insert_at = candidate_at;
            break;
        }
    }
    holes.insert(insert_at, hole);
    Ok(())
}

fn exterior_ring_index(
    kernel: &ExactKernel,
    points: &[Point2],
    rings: &[Vec<usize>],
) -> Result<Option<usize>> {
    let mut candidate = None;
    for (ring_index, ring) in rings.iter().enumerate() {
        let contains_all_other_rings = rings
            .iter()
            .enumerate()
            .filter(|(other_index, _)| *other_index != ring_index)
            .try_fold(true, |contains_all, (_, other)| {
                if !contains_all {
                    return Ok(false);
                }
                predicates::point_in_ring_even_odd(kernel, points, ring, &points[other[0]])
            })?;

        if contains_all_other_rings {
            if candidate.is_some() {
                return Ok(None);
            }
            candidate = Some(ring_index);
        }
    }

    Ok(candidate)
}

fn ring_is_inside_any_other_hole(
    kernel: &ExactKernel,
    points: &[Point2],
    rings: &[Vec<usize>],
    exterior_index: usize,
    ring_index: usize,
) -> Result<bool> {
    for (other_index, other) in rings.iter().enumerate() {
        if other_index == exterior_index || other_index == ring_index {
            continue;
        }
        if predicates::point_in_ring_even_odd(kernel, points, other, &points[rings[ring_index][0]])?
        {
            return Ok(true);
        }
    }

    Ok(false)
}

fn compare_ring_representatives(
    kernel: &ExactKernel,
    points: &[Point2],
    left: &[usize],
    right: &[usize],
) -> Result<Ordering> {
    let left_rep = leftmost_position(kernel, points, left)?;
    let right_rep = leftmost_position(kernel, points, right)?;
    compare_points(kernel, points, left[left_rep], right[right_rep])
}

fn leftmost_position(kernel: &ExactKernel, points: &[Point2], ring: &[usize]) -> Result<usize> {
    let mut best = 0;
    for position in 1..ring.len() {
        if compare_points(kernel, points, ring[position], ring[best])? == Ordering::Less {
            best = position;
        }
    }
    Ok(best)
}

fn compare_points(
    kernel: &ExactKernel,
    points: &[Point2],
    left: usize,
    right: usize,
) -> Result<Ordering> {
    // Ring ordering is not CDT topology; it is a reusable exact point-order
    // predicate. Keep it in hyperlimit so hypertri only chooses how ordered
    // rings are consumed.
    kernel.decide(
        hyperlimit::compare_point2_lexicographic(
            &predicate_point(&points[left]),
            &predicate_point(&points[right]),
            kernel.policy(),
        ),
        "compare_point2_lexicographic",
    )
}

fn predicate_point(point: &Point2) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(point.x.clone(), point.y.clone())
}
