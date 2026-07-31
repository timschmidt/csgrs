//! Constraint recovery and local legalization for exact CDT.
//!
//! The public [`crate::cdt`] module owns API shape and result records; this
//! module keeps the incremental segment-insertion machinery local. The
//! implementation starts from the exact Delaunay triangulation, planarizes
//! crossing constraints into exact Steiner vertices, recovers each protected
//! subsegment by flipping crossed unconstrained edges, then re-legalizes only
//! unconstrained edges. Correctness reduces to local Delaunay checks on
//! unprotected edges; exact predicate ownership stays in the kernel/predicate
//! layer.

use crate::error::{Error, Result};
use crate::kernel::ExactKernel;
use crate::predicates;
use crate::types::Sign;
use crate::types::{Constraint, Point2, Triangle};
use std::cmp::Ordering;

/// Exact planar straight-line graph produced from caller constraints.
pub(crate) struct PlanarConstraints {
    /// Original points plus exact intersection vertices inserted by planarizing
    /// crossing constraints.
    pub(crate) points: Vec<Point2>,
    /// Constraint subsegments over `points`.
    pub(crate) constraints: Vec<Constraint>,
}

/// Insert all constraints into an existing triangulation.
///
/// `triangles` must triangulate the convex hull of `points`. The function keeps
/// every requested segment as a triangulation edge and restores local Delaunay
/// legality for unconstrained interior edges where exact predicates can decide
/// the required flips.
pub(crate) fn insert_constraints(
    kernel: &ExactKernel,
    points: &[Point2],
    mut triangles: Vec<Triangle>,
    constraints: &[Constraint],
) -> Result<Vec<Triangle>> {
    // Structural-dispatch note: constraint recovery processes the planarized
    // subsegments in caller-derived order. The retained PSLG facts already
    // keep intersection vertices and split subsegments explicit; richer
    // prepared objects can add axis-alignment, endpoint order, and
    // exact-rational denominator classes without changing the topology
    // contract. Useful structure stays beside exact arithmetic rather than
    // leaking scalar internals into topology code.
    let approximate_points = crate::cdt::exact_points_f64(points);
    let mut constrained_edges = Vec::new();
    for &constraint in constraints {
        let edge = EdgeKey::new(constraint.from, constraint.to);
        if triangulation_has_edge(&triangles, edge) {
            push_unique_edge(&mut constrained_edges, edge);
            continue;
        }

        recover_constraint(
            kernel,
            points,
            &mut triangles,
            constraint,
            &constrained_edges,
            approximate_points.as_deref(),
        )?;
        push_unique_edge(&mut constrained_edges, edge);
    }

    legalize_unconstrained_edges(
        kernel,
        points,
        &mut triangles,
        &constrained_edges,
        approximate_points.as_deref(),
    )?;
    Ok(triangles)
}

/// Planarize caller constraints into exact PSLG subsegments.
///
/// Segment insertion algorithms normally work on a planar straight-line graph.
/// When constraints properly cross or pass through existing vertices, the
/// intersection becomes a graph vertex and each original segment is normalized
/// into subsegments. Constraint recovery then operates only on a valid planar
/// straight-line graph.
pub(crate) fn planarize_constraints(
    kernel: &ExactKernel,
    points: &[Point2],
    constraints: &[Constraint],
) -> Result<PlanarConstraints> {
    let mut planar_points = points.to_vec();
    let approximate_points = crate::cdt::exact_points_f64(points);

    // Rational-to-binary64 rounding is monotone. Disjoint rounded bounds
    // therefore certify disjoint exact bounds, while every survivor still
    // reaches the exact segment predicate below.
    for first in 0..constraints.len() {
        for second in first + 1..constraints.len() {
            let a = constraints[first];
            let b = constraints[second];
            if constraints_share_endpoint(a, b) {
                continue;
            }
            if approximate_points.as_ref().is_some_and(|points| {
                !crate::cdt::approximate_constraint_bounds_overlap(points, a, b)
            }) {
                continue;
            }

            if predicates::segment_intersection(
                kernel,
                &planar_points[a.from],
                &planar_points[a.to],
                &planar_points[b.from],
                &planar_points[b.to],
            )?
            .is_proper_crossing()
            {
                let point = segment_intersection_point(kernel, &planar_points, a, b)?;
                push_unique_point(kernel, &mut planar_points, point)?;
            }
        }
    }

    let approximate_planar_points = crate::cdt::exact_points_f64(&planar_points);
    let mut split = Vec::new();
    for constraint in constraints {
        let mut on_segment = Vec::new();
        for point_index in 0..planar_points.len() {
            if approximate_planar_points.as_ref().is_some_and(|points| {
                !crate::cdt::approximate_point_within_constraint_bounds(
                    points,
                    *constraint,
                    point_index,
                )
            }) {
                continue;
            }
            if predicates::point_on_segment(
                kernel,
                &planar_points[constraint.from],
                &planar_points[constraint.to],
                &planar_points[point_index],
            )? {
                on_segment.push(point_index);
            }
        }

        sort_indices_on_segment(kernel, &planar_points, constraint, &mut on_segment)?;
        for pair in on_segment.windows(2) {
            push_unique_constraint(&mut split, Constraint::new(pair[0], pair[1]));
        }
    }

    Ok(PlanarConstraints {
        points: planar_points,
        constraints: split,
    })
}

fn segment_intersection_point(
    kernel: &ExactKernel,
    points: &[Point2],
    first: Constraint,
    second: Constraint,
) -> Result<Point2> {
    let a = &points[first.from];
    let b = &points[first.to];
    let c = &points[second.from];
    let d = &points[second.to];

    match kernel.decide(
        hyperlimit::proper_segment_intersection_point(
            &predicate_point(a),
            &predicate_point(b),
            &predicate_point(c),
            &predicate_point(d),
            kernel.policy(),
        ),
        "proper_segment_intersection_point",
    )? {
        Some(point) => Ok(Point2::new(point.x, point.y)),
        None => Err(Error::InvalidInput {
            reason: "constraints do not properly cross",
        }),
    }
}

fn push_unique_point(
    kernel: &ExactKernel,
    points: &mut Vec<Point2>,
    point: Point2,
) -> Result<usize> {
    for (index, candidate) in points.iter().enumerate() {
        if predicates::points_equal(kernel, candidate, &point)? {
            return Ok(index);
        }
    }

    let index = points.len();
    points.push(point);
    Ok(index)
}

fn constraints_share_endpoint(first: Constraint, second: Constraint) -> bool {
    first.from == second.from
        || first.from == second.to
        || first.to == second.from
        || first.to == second.to
}

fn predicate_point(point: &Point2) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(point.x.clone(), point.y.clone())
}

fn sort_indices_on_segment(
    kernel: &ExactKernel,
    points: &[Point2],
    constraint: &Constraint,
    indices: &mut [usize],
) -> Result<()> {
    let use_x = compare_segment_axis_reals(
        kernel,
        &points[constraint.from].x,
        &points[constraint.to].x,
        "compare_constraint_endpoint_x",
    )? != Ordering::Equal;

    for index in 1..indices.len() {
        let mut cursor = index;
        while cursor > 0
            && compare_segment_indices(kernel, points, indices[cursor], indices[cursor - 1], use_x)?
                == Ordering::Less
        {
            indices.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }

    Ok(())
}

fn compare_segment_indices(
    kernel: &ExactKernel,
    points: &[Point2],
    left: usize,
    right: usize,
    use_x: bool,
) -> Result<Ordering> {
    if use_x {
        compare_segment_axis_reals(
            kernel,
            &points[left].x,
            &points[right].x,
            "compare_segment_x",
        )
    } else {
        compare_segment_axis_reals(
            kernel,
            &points[left].y,
            &points[right].y,
            "compare_segment_y",
        )
    }
}

fn compare_segment_axis_reals(
    kernel: &ExactKernel,
    left: &crate::types::Real,
    right: &crate::types::Real,
    predicate: &'static str,
) -> Result<Ordering> {
    // Points have already been certified to lie on this segment. The remaining
    // subsegment split order is therefore a scalar exact-ordering predicate,
    // which belongs in hyperlimit rather than CDT topology.
    kernel.decide(
        hyperlimit::compare_reals(left, right, kernel.policy()),
        predicate,
    )
}

fn push_unique_constraint(constraints: &mut Vec<Constraint>, constraint: Constraint) {
    let edge = EdgeKey::new(constraint.from, constraint.to);
    if !constraints
        .iter()
        .any(|existing| EdgeKey::new(existing.from, existing.to) == edge)
    {
        constraints.push(constraint);
    }
}

fn recover_constraint(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut [Triangle],
    constraint: Constraint,
    constrained_edges: &[EdgeKey],
    approximate_points: Option<&[[f64; 2]]>,
) -> Result<()> {
    let target = EdgeKey::new(constraint.from, constraint.to);
    let max_flips = flip_budget(points.len(), triangles.len());

    for _ in 0..max_flips {
        if triangulation_has_edge(triangles, target) {
            return Ok(());
        }

        let Some(crossing_edge) = first_flippable_edge_crossing_constraint(
            kernel,
            points,
            triangles,
            constraint,
            constrained_edges,
            approximate_points,
        )?
        else {
            return Err(Error::UnsupportedFeature {
                feature: "constraint recovery without a flippable crossing edge",
            });
        };

        if !flip_edge(kernel, points, triangles, crossing_edge)? {
            return Err(Error::UnsupportedFeature {
                feature: "constraint recovery across a non-convex edge cavity",
            });
        }
    }

    Err(Error::UnsupportedFeature {
        feature: "constraint edge recovery did not converge",
    })
}

fn first_flippable_edge_crossing_constraint(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &[Triangle],
    constraint: Constraint,
    constrained_edges: &[EdgeKey],
    approximate_points: Option<&[[f64; 2]]>,
) -> Result<Option<EdgeKey>> {
    let mut saw_crossing = false;
    for edge in unique_edges(triangles) {
        if edge.contains(constraint.from) || edge.contains(constraint.to) {
            continue;
        }
        if approximate_points.is_some_and(|points| {
            !crate::cdt::approximate_constraint_bounds_overlap(
                points,
                constraint,
                Constraint::new(edge.from, edge.to),
            )
        }) {
            continue;
        }
        let intersection = predicates::segment_intersection(
            kernel,
            &points[constraint.from],
            &points[constraint.to],
            &points[edge.from],
            &points[edge.to],
        )?;
        if intersection.is_proper_crossing() {
            saw_crossing = true;
            if constrained_edges.contains(&edge) {
                return Err(Error::InvalidInput {
                    reason: "constraint crosses an existing constrained edge",
                });
            }
            let Some([first, second]) = two_adjacent_triangles(triangles, edge)? else {
                continue;
            };
            let new_edge = EdgeKey::new(first.opposite, second.opposite);
            if !edge_is_flippable(kernel, points, edge, first.opposite, second.opposite)?
                || !flip_preserves_constraints(
                    kernel,
                    points,
                    new_edge,
                    constrained_edges,
                    approximate_points,
                )?
            {
                continue;
            }
            return Ok(Some(edge));
        }
    }

    if saw_crossing {
        Err(Error::UnsupportedFeature {
            feature: "constraint recovery across a non-convex edge cavity",
        })
    } else {
        Ok(None)
    }
}

fn legalize_unconstrained_edges(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut [Triangle],
    constrained_edges: &[EdgeKey],
    approximate_points: Option<&[[f64; 2]]>,
) -> Result<()> {
    let max_flips = flip_budget(points.len(), triangles.len()) * 4;

    for _ in 0..max_flips {
        let mut flipped = false;
        for edge in unique_edges(triangles) {
            if constrained_edges.contains(&edge) {
                continue;
            }

            let Some([first, second]) = two_adjacent_triangles(triangles, edge)? else {
                continue;
            };

            let new_edge = EdgeKey::new(first.opposite, second.opposite);
            if !edge_is_illegal(kernel, points, edge, first.opposite, second.opposite)? {
                continue;
            }
            if !flip_preserves_constraints(
                kernel,
                points,
                new_edge,
                constrained_edges,
                approximate_points,
            )? {
                continue;
            }
            if flip_edge(kernel, points, triangles, edge)? {
                flipped = true;
                break;
            }
        }

        if !flipped {
            return Ok(());
        }
    }

    Err(Error::UnsupportedFeature {
        feature: "constrained Delaunay legalization did not converge",
    })
}

fn edge_is_illegal(
    kernel: &ExactKernel,
    points: &[Point2],
    edge: EdgeKey,
    first_opposite: usize,
    second_opposite: usize,
) -> Result<bool> {
    if !edge_is_flippable(kernel, points, edge, first_opposite, second_opposite)? {
        return Ok(false);
    }

    let orientation = predicates::orient2(
        kernel,
        &points[edge.from],
        &points[edge.to],
        &points[first_opposite],
    )?;
    if orientation == Sign::Zero {
        return Ok(false);
    }

    let incircle = kernel.incircle2(
        &points[edge.from],
        &points[edge.to],
        &points[first_opposite],
        &points[second_opposite],
    )?;

    Ok(matches!(
        (orientation, incircle),
        (Sign::Positive, Sign::Positive) | (Sign::Negative, Sign::Negative)
    ))
}

fn flip_edge(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut [Triangle],
    edge: EdgeKey,
) -> Result<bool> {
    let Some([first, second]) = two_adjacent_triangles(triangles, edge)? else {
        return Ok(false);
    };

    if !edge_is_flippable(kernel, points, edge, first.opposite, second.opposite)? {
        return Ok(false);
    }

    let first_new = make_oriented(kernel, points, [first.opposite, second.opposite, edge.from])?;
    let second_new = make_oriented(kernel, points, [second.opposite, first.opposite, edge.to])?;
    triangles[first.triangle] = first_new;
    triangles[second.triangle] = second_new;
    Ok(true)
}

fn edge_is_flippable(
    kernel: &ExactKernel,
    points: &[Point2],
    edge: EdgeKey,
    first_opposite: usize,
    second_opposite: usize,
) -> Result<bool> {
    if first_opposite == second_opposite
        || edge.contains(first_opposite)
        || edge.contains(second_opposite)
    {
        return Ok(false);
    }

    let first_side = predicates::orient2(
        kernel,
        &points[edge.from],
        &points[edge.to],
        &points[first_opposite],
    )?;
    let second_side = predicates::orient2(
        kernel,
        &points[edge.from],
        &points[edge.to],
        &points[second_opposite],
    )?;
    let opposite_edge_side = predicates::orient2(
        kernel,
        &points[first_opposite],
        &points[second_opposite],
        &points[edge.from],
    )?;
    let opposite_other_side = predicates::orient2(
        kernel,
        &points[first_opposite],
        &points[second_opposite],
        &points[edge.to],
    )?;

    Ok(signs_strictly_differ(first_side, second_side)
        && signs_strictly_differ(opposite_edge_side, opposite_other_side))
}

fn flip_preserves_constraints(
    kernel: &ExactKernel,
    points: &[Point2],
    new_edge: EdgeKey,
    constrained_edges: &[EdgeKey],
    approximate_points: Option<&[[f64; 2]]>,
) -> Result<bool> {
    for point_index in 0..points.len() {
        if new_edge.contains(point_index) {
            continue;
        }
        if approximate_points.is_some_and(|points| {
            !crate::cdt::approximate_point_within_constraint_bounds(
                points,
                Constraint::new(new_edge.from, new_edge.to),
                point_index,
            )
        }) {
            continue;
        }
        if predicates::point_on_segment(
            kernel,
            &points[new_edge.from],
            &points[new_edge.to],
            &points[point_index],
        )? {
            return Ok(false);
        }
    }

    for &constraint in constrained_edges {
        if new_edge == constraint {
            continue;
        }
        if approximate_points.is_some_and(|points| {
            !crate::cdt::approximate_constraint_bounds_overlap(
                points,
                Constraint::new(new_edge.from, new_edge.to),
                Constraint::new(constraint.from, constraint.to),
            )
        }) {
            continue;
        }

        let intersection = predicates::segment_intersection(
            kernel,
            &points[new_edge.from],
            &points[new_edge.to],
            &points[constraint.from],
            &points[constraint.to],
        )?;
        if intersection.is_disjoint()
            || (intersection.is_endpoint_touch() && new_edge.shares_endpoint(constraint))
        {
            continue;
        }
        return Ok(false);
    }

    Ok(true)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct AdjacentTriangle {
    triangle: usize,
    opposite: usize,
}

fn two_adjacent_triangles(
    triangles: &[Triangle],
    edge: EdgeKey,
) -> Result<Option<[AdjacentTriangle; 2]>> {
    let mut adjacent = Vec::new();
    for (triangle_index, triangle) in triangles.iter().enumerate() {
        if triangle_contains_edge(*triangle, edge) {
            let Some(opposite) = triangle
                .iter()
                .copied()
                .find(|&index| !edge.contains(index))
            else {
                return Err(Error::InvalidInput {
                    reason: "triangle edge has no opposite vertex",
                });
            };
            adjacent.push(AdjacentTriangle {
                triangle: triangle_index,
                opposite,
            });
        }
    }

    match adjacent.len() {
        0 | 1 => Ok(None),
        2 => Ok(Some([adjacent[0], adjacent[1]])),
        _ => Err(Error::InvalidInput {
            reason: "triangulation edge has more than two adjacent triangles",
        }),
    }
}

fn make_oriented(
    kernel: &ExactKernel,
    points: &[Point2],
    mut triangle: Triangle,
) -> Result<Triangle> {
    let sign = predicates::orient2(
        kernel,
        &points[triangle[0]],
        &points[triangle[1]],
        &points[triangle[2]],
    )?;
    match sign {
        Sign::Positive => Ok(triangle),
        Sign::Negative => {
            triangle.swap(1, 2);
            Ok(triangle)
        }
        Sign::Zero => Err(Error::InvalidInput {
            reason: "degenerate triangle",
        }),
    }
}

fn triangulation_has_edge(triangles: &[Triangle], edge: EdgeKey) -> bool {
    triangles
        .iter()
        .any(|triangle| triangle_contains_edge(*triangle, edge))
}

fn triangle_contains_edge(triangle: Triangle, edge: EdgeKey) -> bool {
    triangle.contains(&edge.from) && triangle.contains(&edge.to)
}

fn unique_edges(triangles: &[Triangle]) -> Vec<EdgeKey> {
    let mut edges = Vec::new();
    for triangle in triangles {
        push_unique_edge(&mut edges, EdgeKey::new(triangle[0], triangle[1]));
        push_unique_edge(&mut edges, EdgeKey::new(triangle[1], triangle[2]));
        push_unique_edge(&mut edges, EdgeKey::new(triangle[2], triangle[0]));
    }
    edges
}

fn push_unique_edge(edges: &mut Vec<EdgeKey>, edge: EdgeKey) {
    if !edges.contains(&edge) {
        edges.push(edge);
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct EdgeKey {
    from: usize,
    to: usize,
}

impl EdgeKey {
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

    fn shares_endpoint(self, other: Self) -> bool {
        self.contains(other.from) || self.contains(other.to)
    }
}

fn signs_strictly_differ(first: Sign, second: Sign) -> bool {
    matches!(
        (first, second),
        (Sign::Negative, Sign::Positive) | (Sign::Positive, Sign::Negative)
    )
}

fn flip_budget(point_count: usize, triangle_count: usize) -> usize {
    point_count
        .saturating_add(triangle_count)
        .saturating_add(1)
        .pow(2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::TriangulationContext;
    use crate::types::Real;

    #[test]
    fn steiner_point_deduplication_uses_numeric_equality() {
        let left = Real::pi() + Real::e();
        let right = Real::e() + Real::pi();
        assert_ne!(left, right);

        let mut points = vec![Point2::new(left, Real::zero())];
        let context = TriangulationContext::new(hyperlimit::PredicatePolicy::APPROXIMATE_512);
        let kernel = ExactKernel::new(&context);
        assert_eq!(
            push_unique_point(&kernel, &mut points, Point2::new(right, Real::zero())),
            Ok(0)
        );
        assert_eq!(points.len(), 1);
    }
}
