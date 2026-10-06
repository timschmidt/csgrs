//! Constraint graph recognition for polygonal CDT inputs.
//!
//! This module is deliberately private to `hypertri`: it recognizes the
//! polygon-with-holes subset of constrained triangulation while the full DCEL
//! port is still in progress. Keeping this recognition outside `cdt.rs`
//! preserves the public CDT abstraction boundary and keeps exact ring semantics
//! close to polygon normalization.

use std::cmp::Ordering;

use crate::error::Result;
use crate::predicate_evaluator::PredicateEvaluator;
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
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    constraints: &[Constraint],
) -> Result<Option<ConstraintPolygon>> {
    let rings = extract_closed_rings(points.len(), constraints);
    let Some(rings) = rings else {
        return Ok(None);
    };

    order_polygon_rings(evaluator, points, rings)
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
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    rings: Vec<Vec<usize>>,
) -> Result<Option<ConstraintPolygon>> {
    if rings.len() == 1 {
        return Ok(Some(ConstraintPolygon { rings }));
    }

    let Some(exterior_index) = exterior_ring_index(evaluator, points, &rings)? else {
        return Ok(None);
    };

    let mut ordered = Vec::with_capacity(rings.len());
    ordered.push(rings[exterior_index].clone());

    let mut holes = Vec::<Vec<usize>>::with_capacity(rings.len() - 1);
    for (ring_index, ring) in rings.iter().enumerate() {
        if ring_index == exterior_index {
            continue;
        }
        if ring_is_inside_any_other_hole(evaluator, points, &rings, exterior_index, ring_index)? {
            return Ok(None);
        }
        insert_hole_sorted(evaluator, points, &mut holes, ring.clone())?;
    }
    ordered.extend(holes);

    Ok(Some(ConstraintPolygon { rings: ordered }))
}

fn insert_hole_sorted(
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    holes: &mut Vec<Vec<usize>>,
    hole: Vec<usize>,
) -> Result<()> {
    let mut insert_at = holes.len();
    for (candidate_at, candidate) in holes.iter().enumerate() {
        if compare_ring_representatives(evaluator, points, &hole, candidate)? == Ordering::Less {
            insert_at = candidate_at;
            break;
        }
    }
    holes.insert(insert_at, hole);
    Ok(())
}

fn exterior_ring_index(
    evaluator: &PredicateEvaluator,
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
                predicates::point_in_ring_even_odd(evaluator, points, ring, &points[other[0]])
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
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    rings: &[Vec<usize>],
    exterior_index: usize,
    ring_index: usize,
) -> Result<bool> {
    for (other_index, other) in rings.iter().enumerate() {
        if other_index == exterior_index || other_index == ring_index {
            continue;
        }
        if predicates::point_in_ring_even_odd(
            evaluator,
            points,
            other,
            &points[rings[ring_index][0]],
        )? {
            return Ok(true);
        }
    }

    Ok(false)
}

fn compare_ring_representatives(
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    left: &[usize],
    right: &[usize],
) -> Result<Ordering> {
    let left_rep = leftmost_position(evaluator, points, left)?;
    let right_rep = leftmost_position(evaluator, points, right)?;
    compare_points(evaluator, points, left[left_rep], right[right_rep])
}

fn leftmost_position(
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    ring: &[usize],
) -> Result<usize> {
    let mut best = 0;
    for position in 1..ring.len() {
        if compare_points(evaluator, points, ring[position], ring[best])? == Ordering::Less {
            best = position;
        }
    }
    Ok(best)
}

fn compare_points(
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    left: usize,
    right: usize,
) -> Result<Ordering> {
    // Ring ordering is not CDT topology; it is a reusable exact point-order
    // predicate. Keep it in hyperlimit so hypertri only chooses how ordered
    // rings are consumed.
    evaluator.decide(
        hyperlimit::compare_point2_lexicographic(&points[left], &points[right], evaluator.policy()),
        "compare_point2_lexicographic",
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::TriangulationContext;
    use crate::types::Real;

    const STRICT: TriangulationContext =
        TriangulationContext::new(hyperlimit::PredicatePolicy::STRICT);

    fn evaluator() -> PredicateEvaluator {
        PredicateEvaluator::new(&STRICT)
    }

    fn point(x: i64, y: i64) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    fn cycle(indices: &[usize]) -> Vec<Constraint> {
        (0..indices.len())
            .map(|index| Constraint::new(indices[index], indices[(index + 1) % indices.len()]))
            .collect()
    }

    #[test]
    fn closed_ring_extraction_rejects_noncycles_and_keeps_all_cycles() {
        assert_eq!(extract_closed_rings(0, &[]), None);
        assert_eq!(extract_closed_rings(2, &[Constraint::new(0, 1)]), None);
        assert_eq!(extract_closed_rings(3, &[Constraint::new(0, 1)]), None);
        assert_eq!(extract_closed_rings(3, &[Constraint::new(0, 0)]), None);
        assert_eq!(
            extract_closed_rings(3, &[Constraint::new(0, 1), Constraint::new(0, 1)]),
            None
        );

        let mut constraints = cycle(&[0, 1, 2]);
        constraints.extend(cycle(&[3, 4, 5, 6]));
        let rings = extract_closed_rings(7, &constraints).unwrap();
        assert_eq!(rings.len(), 2);
        assert_eq!(rings[0], vec![0, 1, 2]);
        assert_eq!(rings[1], vec![3, 4, 5, 6]);
    }

    #[test]
    fn ring_ordering_finds_exterior_and_sorts_holes_lexicographically() {
        let points = vec![
            point(0, 0),
            point(12, 0),
            point(12, 12),
            point(0, 12),
            point(8, 4),
            point(10, 4),
            point(10, 6),
            point(8, 6),
            point(4, 6),
            point(2, 6),
            point(2, 4),
            point(4, 4),
        ];
        // Put the exterior last and rotate both holes so representative search
        // and insertion-before-an-existing-hole are exercised.
        let rings = vec![vec![4, 5, 6, 7], vec![8, 9, 10, 11], vec![0, 1, 2, 3]];
        let polygon = order_polygon_rings(&evaluator(), &points, rings)
            .unwrap()
            .unwrap();

        assert_eq!(polygon.rings[0], vec![0, 1, 2, 3]);
        assert_eq!(polygon.rings[1], vec![8, 9, 10, 11]);
        assert_eq!(polygon.rings[2], vec![4, 5, 6, 7]);

        #[cfg(feature = "earcut")]
        {
            let (flat, hole_indices, source_indices) = polygon.to_flat_polygon(&points);
            assert_eq!(flat.len(), 12);
            assert_eq!(hole_indices, vec![4, 8]);
            assert_eq!(source_indices, vec![0, 1, 2, 3, 8, 9, 10, 11, 4, 5, 6, 7]);
        }
    }

    #[test]
    fn private_hole_helpers_cover_prepend_and_nested_detection() {
        let points = vec![
            point(0, 0),
            point(12, 0),
            point(12, 12),
            point(0, 12),
            point(6, 6),
            point(8, 4),
            point(10, 4),
            point(10, 8),
            point(8, 8),
            point(9, 5),
        ];
        let mut holes = vec![vec![5, 6, 7, 8]];
        insert_hole_sorted(&evaluator(), &points, &mut holes, vec![4]).unwrap();
        assert_eq!(holes, vec![vec![4], vec![5, 6, 7, 8]]);

        let rings = vec![vec![0, 1, 2, 3], vec![5, 6, 7, 8], vec![9]];
        assert!(ring_is_inside_any_other_hole(&evaluator(), &points, &rings, 0, 2).unwrap());
    }

    #[test]
    fn ring_ordering_rejects_disjoint_nested_and_ambiguous_exteriors() {
        let disjoint = vec![
            point(0, 0),
            point(2, 0),
            point(0, 2),
            point(5, 0),
            point(7, 0),
            point(5, 2),
        ];
        assert_eq!(
            order_polygon_rings(&evaluator(), &disjoint, vec![vec![0, 1, 2], vec![3, 4, 5]])
                .unwrap(),
            None
        );

        let nested = vec![
            point(0, 0),
            point(10, 0),
            point(10, 10),
            point(0, 10),
            point(2, 2),
            point(8, 2),
            point(8, 8),
            point(2, 8),
            point(3, 3),
            point(4, 3),
            point(4, 4),
            point(3, 4),
        ];
        assert_eq!(
            order_polygon_rings(
                &evaluator(),
                &nested,
                vec![vec![0, 1, 2, 3], vec![4, 5, 6, 7], vec![8, 9, 10, 11]],
            )
            .unwrap(),
            None
        );

        let duplicate = vec![
            point(0, 0),
            point(4, 0),
            point(0, 4),
            point(0, 0),
            point(4, 0),
            point(0, 4),
        ];
        assert_eq!(
            exterior_ring_index(&evaluator(), &duplicate, &[vec![0, 1, 2], vec![3, 4, 5]]).unwrap(),
            None
        );
    }

    #[test]
    fn public_recognizer_distinguishes_cycle_graphs_from_general_pslgs() {
        let points = vec![point(0, 0), point(4, 0), point(0, 4), point(2, 2)];
        let polygon =
            polygon_from_closed_constraints(&evaluator(), &points, &cycle(&[0, 1, 2])).unwrap();
        assert!(polygon.is_some());

        let open = [Constraint::new(0, 1), Constraint::new(1, 2)];
        assert_eq!(
            polygon_from_closed_constraints(&evaluator(), &points, &open).unwrap(),
            None
        );
    }
}
