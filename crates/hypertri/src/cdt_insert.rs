//! Exact constraint recovery with structural or Delaunay legalization.
//!
//! The public [`crate::cdt`] module owns API shape and result records; this
//! module keeps the incremental segment-insertion machinery local. The
//! implementation starts from the exact Delaunay triangulation, planarizes
//! crossing constraints into exact Steiner vertices, recovers each protected
//! subsegment by flipping crossed unconstrained edges or retriangulating its
//! exact cavity, then re-legalizes only unconstrained edges. Correctness
//! reduces to complete convex-hull coverage and local Delaunay checks on
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

struct RecoveredConstraints {
    triangles: Vec<Triangle>,
    constrained_edges: Vec<EdgeKey>,
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
    triangles: Vec<Triangle>,
    constraints: &[Constraint],
) -> Result<Vec<Triangle>> {
    let RecoveredConstraints {
        mut triangles,
        constrained_edges,
    } = recover_constraints(kernel, points, triangles, constraints)?;
    legalize_unconstrained_edges(kernel, points, &mut triangles, &constrained_edges)?;
    Ok(triangles)
}

/// Insert every constraint without imposing a triangle-quality policy on the
/// remaining edges.
///
/// This is the topology-only counterpart of [`insert_constraints`]. The same
/// exact crossing, flip, and cavity predicates recover every protected edge;
/// only the final empty-circle legalization sweep is omitted.
pub(crate) fn insert_constraints_topology(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: Vec<Triangle>,
    constraints: &[Constraint],
) -> Result<Vec<Triangle>> {
    let RecoveredConstraints {
        mut triangles,
        constrained_edges,
    } = recover_constraints(kernel, points, triangles, constraints)?;
    canonicalize_unconstrained_edges(kernel, points, &mut triangles, &constrained_edges)?;
    Ok(triangles)
}

fn recover_constraints(
    kernel: &ExactKernel,
    points: &[Point2],
    mut triangles: Vec<Triangle>,
    constraints: &[Constraint],
) -> Result<RecoveredConstraints> {
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

        recover_missing_constraint(
            kernel,
            points,
            &mut triangles,
            constraint,
            &constrained_edges,
            approximate_points.as_deref(),
        )?;
        push_unique_edge(&mut constrained_edges, edge);
    }

    Ok(RecoveredConstraints {
        triangles,
        constrained_edges,
    })
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
                let point = segment_intersection_point(&planar_points, a, b)?;
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
    points: &[Point2],
    first: Constraint,
    second: Constraint,
) -> Result<Point2> {
    let a = &points[first.from];
    let b = &points[first.to];
    let c = &points[second.from];
    let d = &points[second.to];

    match hyperlimit::construct_line_intersection_point(a, b, c, d) {
        Some(point) => Ok(Point2::new(point.x, point.y)),
        None => Err(Error::InvalidInput {
            reason: "properly crossing constraint lines have no constructible intersection",
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

fn recover_missing_constraint(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut [Triangle],
    constraint: Constraint,
    constrained_edges: &[EdgeKey],
    approximate_points: Option<&[[f64; 2]]>,
) -> Result<()> {
    // Recover one complete crossed-triangle corridor. Rebuilding and sorting
    // global edge incidence after each individually legal flip repeats work
    // proportional to the whole triangulation and can become superlinear in
    // the number of crossings. The exact cavity is the standard structural
    // unit of segment insertion; unconstrained Delaunay legality is restored
    // once after all protected segments have been recovered.
    recover_constraint_cavity(
        kernel,
        points,
        triangles,
        constraint,
        constrained_edges,
        approximate_points,
    )
}

fn recover_constraint_cavity(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut [Triangle],
    constraint: Constraint,
    constrained_edges: &[EdgeKey],
    approximate_points: Option<&[[f64; 2]]>,
) -> Result<()> {
    enum CrossedCorridor {
        Empty,
        One(EdgeKey, [AdjacentTriangle; 2]),
        Many(Vec<bool>),
    }

    let topology = TopologyEdges::new(triangles)?;
    let mut crossed = CrossedCorridor::Empty;
    for topology in topology.iter() {
        let (edge, owners) = topology?;
        if edge.contains(constraint.from) || edge.contains(constraint.to) {
            continue;
        }
        if !edge_properly_crosses_constraint(kernel, points, edge, constraint, approximate_points)?
        {
            continue;
        }
        if constrained_edges.binary_search(&edge).is_ok() {
            return Err(Error::InvalidInput {
                reason: "constraint crosses an existing constrained edge",
            });
        }
        let Some(owners) = owners else {
            return Err(Error::UnsupportedFeature {
                feature: "constraint cavity crosses a boundary edge",
            });
        };
        let adjacent = adjacent_triangles(triangles, edge, owners)?;
        crossed = match crossed {
            CrossedCorridor::Empty => CrossedCorridor::One(edge, adjacent),
            CrossedCorridor::One(_, first) => {
                let mut cavity = vec![false; triangles.len()];
                cavity[first[0].triangle] = true;
                cavity[first[1].triangle] = true;
                cavity[adjacent[0].triangle] = true;
                cavity[adjacent[1].triangle] = true;
                CrossedCorridor::Many(cavity)
            }
            CrossedCorridor::Many(mut cavity) => {
                cavity[adjacent[0].triangle] = true;
                cavity[adjacent[1].triangle] = true;
                CrossedCorridor::Many(cavity)
            }
        };
    }
    let mut cavity = match crossed {
        CrossedCorridor::Empty => {
            return Err(Error::UnsupportedFeature {
                feature: "constraint cavity contains no crossed triangles",
            });
        }
        CrossedCorridor::One(edge, adjacent) => {
            let replacement = EdgeKey::new(adjacent[0].opposite, adjacent[1].opposite);
            // A proper crossing between the shared edge and the
            // opposite-vertex diagonal is already the exact
            // convex-quadrilateral certificate. Reuse it instead of repeating
            // the same four side orientations.
            if replacement == EdgeKey::new(constraint.from, constraint.to) {
                return replace_adjacent_edge(kernel, points, triangles, edge, adjacent);
            }
            let mut cavity = vec![false; triangles.len()];
            cavity[adjacent[0].triangle] = true;
            cavity[adjacent[1].triangle] = true;
            cavity
        }
        CrossedCorridor::Many(cavity) => cavity,
    };
    close_constraint_cavity_holes(&topology, constrained_edges, &mut cavity)?;
    let cavity_indices = cavity
        .iter()
        .enumerate()
        .filter_map(|(index, &in_cavity)| in_cavity.then_some(index))
        .collect::<Vec<_>>();
    let mut cavity_vertices = cavity_indices
        .iter()
        .flat_map(|&triangle| triangles[triangle])
        .collect::<Vec<_>>();
    cavity_vertices.sort_unstable();
    cavity_vertices.dedup();

    let mut edge_uses = Vec::with_capacity(cavity_indices.len().saturating_mul(3));
    for &triangle_index in &cavity_indices {
        let triangle = triangles[triangle_index];
        edge_uses.extend([
            EdgeKey::new(triangle[0], triangle[1]),
            EdgeKey::new(triangle[1], triangle[2]),
            EdgeKey::new(triangle[2], triangle[0]),
        ]);
    }
    edge_uses.sort_unstable();
    let mut boundary_edges = Vec::new();
    let mut start = 0;
    while start < edge_uses.len() {
        let mut end = start + 1;
        while end < edge_uses.len() && edge_uses[end] == edge_uses[start] {
            end += 1;
        }
        match end - start {
            1 => boundary_edges.push(edge_uses[start]),
            2 if constrained_edges.binary_search(&edge_uses[start]).is_ok() => {
                return Err(Error::UnsupportedFeature {
                    feature: "constraint cavity contains an existing constrained edge",
                });
            }
            2 => {}
            _ => {
                return Err(Error::InvalidInput {
                    reason: "constraint cavity contains a non-manifold edge",
                });
            }
        }
        start = end;
    }

    let missing = usize::MAX;
    let mut adjacency = vec![[missing; 2]; points.len()];
    for edge in &boundary_edges {
        for (vertex, neighbor) in [(edge.from, edge.to), (edge.to, edge.from)] {
            if adjacency[vertex][0] == missing {
                adjacency[vertex][0] = neighbor;
            } else if adjacency[vertex][1] == missing {
                adjacency[vertex][1] = neighbor;
            } else {
                return Err(Error::UnsupportedFeature {
                    feature: "constraint cavity boundary is not a simple cycle",
                });
            }
        }
    }
    if adjacency[constraint.from][1] == missing || adjacency[constraint.to][1] == missing {
        return Err(Error::UnsupportedFeature {
            feature: "constraint cavity endpoints are not on one boundary cycle",
        });
    }
    let mut cycle = vec![constraint.from];
    let mut previous = usize::MAX;
    let mut current = constraint.from;
    for _ in 0..=boundary_edges.len() {
        let neighbors = &adjacency[current];
        if neighbors[1] == missing {
            return Err(Error::UnsupportedFeature {
                feature: "constraint cavity boundary is not a simple cycle",
            });
        }
        let next = if neighbors[0] != previous {
            neighbors[0]
        } else {
            neighbors[1]
        };
        if next == constraint.from {
            break;
        }
        cycle.push(next);
        previous = current;
        current = next;
    }
    if cycle.len() != boundary_edges.len() {
        return Err(Error::UnsupportedFeature {
            feature: "constraint cavity boundary traversal did not close",
        });
    }
    let Some(to_position) = cycle.iter().position(|&vertex| vertex == constraint.to) else {
        return Err(Error::UnsupportedFeature {
            feature: "constraint cavity boundary omits one endpoint",
        });
    };

    let first = cycle[..=to_position].to_vec();
    let mut second = Vec::with_capacity(cycle.len() - to_position + 1);
    second.push(constraint.from);
    second.extend(cycle[to_position..].iter().rev().copied());
    let mut replacement = Vec::with_capacity(cavity_indices.len());
    for side in [first, second] {
        if side.len() < 3 {
            continue;
        }
        replacement.extend(triangulate_cavity_side(kernel, points, side)?);
    }
    for vertex in cavity_vertices {
        if replacement
            .iter()
            .any(|triangle| triangle.contains(&vertex))
        {
            continue;
        }
        crate::cdt::insert_topology_point(kernel, points, &mut replacement, vertex)?;
    }
    if replacement.len() != cavity_indices.len() {
        return Err(Error::UnsupportedFeature {
            feature: "constraint cavity retriangulation changed triangle count",
        });
    }
    for (triangle_index, triangle) in cavity_indices.into_iter().zip(replacement) {
        triangles[triangle_index] = triangle;
    }
    if !triangulation_has_edge(triangles, EdgeKey::new(constraint.from, constraint.to)) {
        return Err(Error::UnsupportedFeature {
            feature: "constraint cavity retriangulation omitted the target edge",
        });
    }
    Ok(())
}

/// Close only holes created by the crossed-triangle corridor itself.
///
/// A valid segment can cross a fan of triangulation edges whose dual corridor
/// wraps around an unselected interior component. The resulting boundary is
/// weakly simple even though both the triangulation and PSLG are valid. A
/// component is safe to absorb exactly when it reaches neither the convex-hull
/// boundary nor an already protected edge. This turns the recovery cavity into
/// one disk without crossing a prior constraint or consuming exterior work.
fn close_constraint_cavity_holes(
    topology: &TopologyEdges,
    constrained_edges: &[EdgeKey],
    cavity: &mut [bool],
) -> Result<()> {
    let missing = usize::MAX;
    let mut neighbors = vec![[missing; 3]; cavity.len()];
    let mut open = vec![false; cavity.len()];
    let mut start = 0;
    while start < topology.uses.len() {
        let edge = topology.uses[start].edge;
        let mut end = start + 1;
        while end < topology.uses.len() && topology.uses[end].edge == edge {
            end += 1;
        }
        match end - start {
            1 => open[topology.uses[start].triangle] = true,
            2 => {
                let first = topology.uses[start].triangle;
                let second = topology.uses[start + 1].triangle;
                if first == second {
                    return Err(Error::InvalidInput {
                        reason: "triangle contains the same edge more than once",
                    });
                }
                if constrained_edges.binary_search(&edge).is_ok() {
                    open[first] = true;
                    open[second] = true;
                } else {
                    for (owner, neighbor) in [(first, second), (second, first)] {
                        let slot = neighbors[owner]
                            .iter_mut()
                            .find(|candidate| **candidate == missing)
                            .ok_or(Error::InvalidInput {
                                reason: "triangulation triangle has too many adjacent edges",
                            })?;
                        *slot = neighbor;
                    }
                }
            }
            _ => {
                return Err(Error::InvalidInput {
                    reason: "triangulation edge has more than two adjacent triangles",
                });
            }
        }
        start = end;
    }

    let mut visited = vec![false; cavity.len()];
    let mut stack = Vec::new();
    let mut component = Vec::new();
    for seed in 0..cavity.len() {
        if cavity[seed] || visited[seed] {
            continue;
        }
        visited[seed] = true;
        stack.clear();
        component.clear();
        stack.push(seed);
        let mut enclosed = true;
        while let Some(triangle) = stack.pop() {
            component.push(triangle);
            enclosed &= !open[triangle];
            for &neighbor in &neighbors[triangle] {
                if neighbor != missing && !cavity[neighbor] && !visited[neighbor] {
                    visited[neighbor] = true;
                    stack.push(neighbor);
                }
            }
        }
        if enclosed {
            for &triangle in &component {
                cavity[triangle] = true;
            }
        }
    }
    Ok(())
}

fn triangulate_cavity_side(
    kernel: &ExactKernel,
    points: &[Point2],
    mut ring: Vec<usize>,
) -> Result<Vec<Triangle>> {
    // Remove straight-chain vertices before emitting any ears. If a vertex is
    // discarded after an adjacent ear has already been committed, it remains
    // present in that ear while its newly exposed sliver is silently omitted.
    // The caller reinserts every omitted cavity vertex into the completed
    // topology, preserving both the boundary and Euler triangle count.
    loop {
        let mut collinear = None;
        for position in 1..ring.len().saturating_sub(1) {
            let previous = ring[position - 1];
            let current = ring[position];
            let next = ring[position + 1];
            if predicates::orient2(kernel, &points[previous], &points[current], &points[next])?
                == Sign::Zero
            {
                collinear = Some(position);
                break;
            }
        }
        let Some(position) = collinear else {
            break;
        };
        ring.remove(position);
    }

    let predicate_ring = ring
        .iter()
        .map(|&vertex| hyperlimit::Point2::new(points[vertex].x.clone(), points[vertex].y.clone()))
        .collect::<Vec<_>>();
    let winding = match kernel.decide(
        hyperlimit::ring_area_sign(&predicate_ring, kernel.policy()),
        "constraint_cavity_ring_area_sign",
    )? {
        hyperlimit::Sign::Negative => Sign::Negative,
        hyperlimit::Sign::Zero => {
            return Err(Error::InvalidInput {
                reason: "constraint cavity side is degenerate",
            });
        }
        hyperlimit::Sign::Positive => Sign::Positive,
    };

    let mut triangles = Vec::with_capacity(ring.len().saturating_sub(2));
    while ring.len() > 3 {
        let mut ear = None;
        for position in 0..ring.len() {
            let previous = ring[(position + ring.len() - 1) % ring.len()];
            let current = ring[position];
            let next = ring[(position + 1) % ring.len()];
            let turn =
                predicates::orient2(kernel, &points[previous], &points[current], &points[next])?;
            if turn == Sign::Zero {
                continue;
            }
            if turn != winding {
                continue;
            }
            let mut contains_vertex = false;
            for &candidate in &ring {
                if candidate == previous || candidate == current || candidate == next {
                    continue;
                }
                if predicates::point_in_or_on_triangle(
                    kernel,
                    &points[previous],
                    &points[current],
                    &points[next],
                    &points[candidate],
                )? {
                    contains_vertex = true;
                    break;
                }
            }
            if !contains_vertex {
                ear = Some((position, [previous, current, next]));
                break;
            }
        }
        let Some((position, triangle)) = ear else {
            return Err(Error::UnsupportedFeature {
                feature: "constraint cavity side has no exact ear",
            });
        };
        triangles.push(make_oriented(kernel, points, triangle)?);
        ring.remove(position);
    }
    if ring.len() == 3 {
        triangles.push(make_oriented(kernel, points, [ring[0], ring[1], ring[2]])?);
    }
    Ok(triangles)
}

fn edge_properly_crosses_constraint(
    kernel: &ExactKernel,
    points: &[Point2],
    edge: EdgeKey,
    constraint: Constraint,
    approximate_points: Option<&[[f64; 2]]>,
) -> Result<bool> {
    if edge.contains(constraint.from) || edge.contains(constraint.to) {
        return Ok(false);
    }
    if approximate_points.is_some_and(|points| {
        !crate::cdt::approximate_constraint_bounds_overlap(
            points,
            constraint,
            Constraint::new(edge.from, edge.to),
        )
    }) {
        return Ok(false);
    }
    Ok(predicates::segment_intersection(
        kernel,
        &points[constraint.from],
        &points[constraint.to],
        &points[edge.from],
        &points[edge.to],
    )?
    .is_proper_crossing())
}

fn legalize_unconstrained_edges(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut [Triangle],
    constrained_edges: &[EdgeKey],
) -> Result<()> {
    loop {
        let mut flipped = false;
        for topology in TopologyEdges::new(triangles)?.iter() {
            let (edge, owners) = topology?;
            if constrained_edges.binary_search(&edge).is_ok() {
                continue;
            }

            let Some(owners) = owners else {
                continue;
            };
            let [first, second] = adjacent_triangles(triangles, edge, owners)?;

            if !edge_is_illegal(kernel, points, edge, first.opposite, second.opposite)? {
                continue;
            }
            replace_adjacent_edge(kernel, points, triangles, edge, [first, second])?;
            flipped = true;
            break;
        }

        if !flipped {
            return Ok(());
        }
    }
}

/// Choose one structural triangulation without evaluating empty circles.
///
/// Every retained flip replaces an unconstrained diagonal by a strictly
/// smaller endpoint pair. The finite edge set therefore gives a direct
/// termination measure. Exact convexity keeps the replacement diagonal inside
/// the two adjacent triangles, so it cannot cross a protected triangulation
/// edge or pass through another represented vertex. The descending rule also
/// stabilizes common equivalent-cell cases that constraint recovery can reach
/// through different initial diagonals.
fn canonicalize_unconstrained_edges(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut [Triangle],
    constrained_edges: &[EdgeKey],
) -> Result<()> {
    loop {
        let mut flipped = false;
        for topology in TopologyEdges::new(triangles)?.iter() {
            let (edge, owners) = topology?;
            if constrained_edges.binary_search(&edge).is_ok() {
                continue;
            }
            let Some(owners) = owners else {
                continue;
            };
            let [first, second] = adjacent_triangles(triangles, edge, owners)?;
            let replacement = EdgeKey::new(first.opposite, second.opposite);
            if replacement >= edge
                || !edge_is_flippable(kernel, points, edge, first.opposite, second.opposite)?
            {
                continue;
            }
            replace_adjacent_edge(kernel, points, triangles, edge, [first, second])?;
            flipped = true;
            break;
        }
        if !flipped {
            return Ok(());
        }
    }
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

/// Replaces an edge after the caller has certified this exact adjacency and
/// the strict convex-quadrilateral predicate against the current triangles.
/// Keeping the proof beside the selected edge avoids rediscovering topology or
/// repeating the four exact side predicates immediately before the write.
fn replace_adjacent_edge(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut [Triangle],
    edge: EdgeKey,
    adjacent: [AdjacentTriangle; 2],
) -> Result<()> {
    let [first, second] = adjacent;
    let first_new = make_oriented(kernel, points, [first.opposite, second.opposite, edge.from])?;
    let second_new = make_oriented(kernel, points, [second.opposite, first.opposite, edge.to])?;
    triangles[first.triangle] = first_new;
    triangles[second.triangle] = second_new;
    Ok(())
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

    // These four strict sides certify a convex quadrilateral. Its replacement
    // diagonal stays inside the two adjacent triangles; the triangulation
    // invariant therefore proves it cannot cross an existing edge or contain
    // any other represented vertex.
    Ok(signs_strictly_differ(first_side, second_side)
        && signs_strictly_differ(opposite_edge_side, opposite_other_side))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct AdjacentTriangle {
    triangle: usize,
    opposite: usize,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct EdgeUse {
    edge: EdgeKey,
    triangle: usize,
}

struct TopologyEdges {
    uses: Vec<EdgeUse>,
}

impl TopologyEdges {
    /// Builds one deterministic edge-use stream for the current triangulation.
    /// Sorted owners make every adjacency lookup local and detect malformed or
    /// nonmanifold incidence without a per-edge scan of all triangles.
    fn new(triangles: &[Triangle]) -> Result<Self> {
        let capacity = triangles.len().checked_mul(3).ok_or(Error::InvalidInput {
            reason: "triangle topology capacity overflow",
        })?;
        let mut uses = Vec::with_capacity(capacity);
        for (triangle, indices) in triangles.iter().copied().enumerate() {
            if indices[0] == indices[1] || indices[1] == indices[2] || indices[2] == indices[0] {
                return Err(Error::InvalidInput {
                    reason: "triangle edge has no opposite vertex",
                });
            }
            uses.extend([
                EdgeUse {
                    edge: EdgeKey::new(indices[0], indices[1]),
                    triangle,
                },
                EdgeUse {
                    edge: EdgeKey::new(indices[1], indices[2]),
                    triangle,
                },
                EdgeUse {
                    edge: EdgeKey::new(indices[2], indices[0]),
                    triangle,
                },
            ]);
        }
        uses.sort_unstable();
        Ok(Self { uses })
    }

    fn iter(&self) -> TopologyEdgeIter<'_> {
        TopologyEdgeIter {
            uses: &self.uses,
            next: 0,
        }
    }
}

struct TopologyEdgeIter<'a> {
    uses: &'a [EdgeUse],
    next: usize,
}

impl Iterator for TopologyEdgeIter<'_> {
    type Item = Result<(EdgeKey, Option<[usize; 2]>)>;

    fn next(&mut self) -> Option<Self::Item> {
        let first = *self.uses.get(self.next)?;
        let mut end = self.next + 1;
        while self
            .uses
            .get(end)
            .is_some_and(|use_| use_.edge == first.edge)
        {
            end += 1;
        }
        let count = end - self.next;
        self.next = end;
        Some(match count {
            1 => Ok((first.edge, None)),
            2 => {
                let second = self.uses[end - 1];
                if first.triangle == second.triangle {
                    Err(Error::InvalidInput {
                        reason: "triangle contains the same edge more than once",
                    })
                } else {
                    Ok((first.edge, Some([first.triangle, second.triangle])))
                }
            }
            _ => Err(Error::InvalidInput {
                reason: "triangulation edge has more than two adjacent triangles",
            }),
        })
    }
}

fn adjacent_triangles(
    triangles: &[Triangle],
    edge: EdgeKey,
    owners: [usize; 2],
) -> Result<[AdjacentTriangle; 2]> {
    let adjacent = |triangle: usize| -> Result<AdjacentTriangle> {
        let triangle_vertices = triangles.get(triangle).ok_or(Error::InvalidInput {
            reason: "triangle topology references an absent triangle",
        })?;
        let opposite = triangle_vertices
            .iter()
            .copied()
            .find(|&vertex| !edge.contains(vertex))
            .ok_or(Error::InvalidInput {
                reason: "triangle edge has no opposite vertex",
            })?;
        Ok(AdjacentTriangle { triangle, opposite })
    };
    Ok([adjacent(owners[0])?, adjacent(owners[1])?])
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

fn push_unique_edge(edges: &mut Vec<EdgeKey>, edge: EdgeKey) {
    if let Err(position) = edges.binary_search(&edge) {
        edges.insert(position, edge);
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
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
    use crate::context::TriangulationContext;
    use crate::types::Real;

    fn p(x: i64, y: i64) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn sorted_topology_edges_retain_owners_and_reject_malformed_incidence() {
        let triangles = [[0, 1, 2], [1, 0, 3]];
        let edges = TopologyEdges::new(&triangles)
            .unwrap()
            .iter()
            .collect::<Result<Vec<_>>>()
            .unwrap();
        let (_, owners) = edges
            .iter()
            .find(|(edge, _)| *edge == EdgeKey::new(0, 1))
            .copied()
            .unwrap();
        assert_eq!(owners, Some([0, 1]));
        assert_eq!(
            adjacent_triangles(&triangles, EdgeKey::new(0, 1), owners.unwrap()).unwrap(),
            [
                AdjacentTriangle {
                    triangle: 0,
                    opposite: 2,
                },
                AdjacentTriangle {
                    triangle: 1,
                    opposite: 3,
                },
            ]
        );

        assert!(matches!(
            TopologyEdges::new(&[[0, 0, 1]]),
            Err(Error::InvalidInput {
                reason: "triangle edge has no opposite vertex"
            })
        ));
        let nonmanifold = TopologyEdges::new(&[[0, 1, 2], [1, 0, 3], [0, 1, 4]])
            .unwrap()
            .iter()
            .collect::<Result<Vec<_>>>();
        assert_eq!(
            nonmanifold,
            Err(Error::InvalidInput {
                reason: "triangulation edge has more than two adjacent triangles",
            })
        );
    }

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

    #[test]
    fn cavity_retriangulation_reinserts_collinear_boundary_vertex() {
        let points = vec![p(-2, 0), p(-2, 2), p(0, 2), p(2, 2), p(2, 0), p(0, -3)];
        let original = vec![[5, 0, 1], [5, 1, 2], [5, 2, 3], [5, 3, 4]];
        let constraint = Constraint::new(0, 4);
        let approximate = crate::cdt::exact_points_f64(&points);

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = TriangulationContext::new(policy);
            let kernel = ExactKernel::new(&context);
            let mut triangles = original.clone();

            recover_constraint_cavity(
                &kernel,
                &points,
                &mut triangles,
                constraint,
                &[],
                approximate.as_deref(),
            )
            .unwrap();

            assert_eq!(triangles.len(), original.len());
            assert!(triangulation_has_edge(
                &triangles,
                EdgeKey::new(constraint.from, constraint.to)
            ));
            assert!(triangles.iter().any(|triangle| triangle.contains(&2)));
            crate::cdt_validate::validate_constrained_topology(
                &kernel,
                &points,
                &[constraint],
                &triangles,
            )
            .unwrap();
            assert_eq!(
                kernel.finish(()).certainty,
                crate::TriangulationCertainty::Certified
            );
        }
    }

    #[test]
    fn cavity_side_prunes_straight_chains_before_emitting_ears() {
        // A narrow, exactly integral polygon whose long straight boundary
        // chain previously exposed another collinear vertex only after an ear
        // had already been emitted.
        let points = vec![
            p(131, 40),
            p(136, 48),
            p(142, 58),
            p(162, 92),
            p(182, 126),
            p(202, 160),
            p(222, 194),
            p(242, 228),
            p(262, 262),
            p(148, 68),
            p(134, 44),
            p(140, 54),
            p(132, 40),
        ];

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = TriangulationContext::new(policy);
            let kernel = ExactKernel::new(&context);
            let mut triangles =
                triangulate_cavity_side(&kernel, &points, (0..points.len()).collect()).unwrap();
            for vertex in 0..points.len() {
                if !triangles.iter().any(|triangle| triangle.contains(&vertex)) {
                    crate::cdt::insert_topology_point(&kernel, &points, &mut triangles, vertex)
                        .unwrap();
                }
            }

            assert_eq!(triangles.len(), points.len() - 2);
            assert!(triangulation_has_edge(
                &triangles,
                EdgeKey::new(0, points.len() - 1)
            ));
            assert!(
                (0..points.len())
                    .all(|vertex| triangles.iter().any(|triangle| triangle.contains(&vertex)))
            );
            assert_eq!(
                kernel.finish(()).certainty,
                crate::TriangulationCertainty::Certified
            );
        }
    }

    #[test]
    fn cavity_closure_absorbs_only_unprotected_enclosed_components() {
        const SIDE: usize = 5;
        let vertex = |x: usize, y: usize| y * (SIDE + 1) + x;
        let mut triangles = Vec::with_capacity(SIDE * SIDE * 2);
        for y in 0..SIDE {
            for x in 0..SIDE {
                let [a, b, c, d] = [
                    vertex(x, y),
                    vertex(x + 1, y),
                    vertex(x, y + 1),
                    vertex(x + 1, y + 1),
                ];
                triangles.extend([[a, b, d], [a, d, c]]);
            }
        }
        let topology = TopologyEdges::new(&triangles).unwrap();
        let center = |x: usize, y: usize| (y * SIDE + x) * 2;
        let mut corridor = vec![false; triangles.len()];
        for y in 1..=3 {
            for x in 1..=3 {
                if (x, y) == (2, 2) {
                    continue;
                }
                corridor[center(x, y)] = true;
                corridor[center(x, y) + 1] = true;
            }
        }

        let mut closed = corridor.clone();
        close_constraint_cavity_holes(&topology, &[], &mut closed).unwrap();
        assert!(closed[center(2, 2)] && closed[center(2, 2) + 1]);
        assert!(!closed[center(0, 0)] && !closed[center(4, 4)]);

        let central_diagonal = EdgeKey::new(vertex(2, 2), vertex(3, 3));
        close_constraint_cavity_holes(&topology, &[central_diagonal], &mut corridor).unwrap();
        assert!(!corridor[center(2, 2)] && !corridor[center(2, 2) + 1]);
    }

    #[test]
    fn lexicographic_topology_is_independent_of_the_initial_convex_fan() {
        let points = vec![p(0, 0), p(2, 0), p(3, 1), p(2, 2), p(0, 2), p(-1, 1)];
        let first = [[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 5]];
        let second = [[3, 4, 5], [3, 5, 0], [3, 0, 1], [3, 1, 2]];

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = TriangulationContext::new(policy);
            let kernel = ExactKernel::new(&context);
            let orient = |triangles: &[[usize; 3]]| {
                triangles
                    .iter()
                    .copied()
                    .map(|triangle| make_oriented(&kernel, &points, triangle))
                    .collect::<Result<Vec<_>>>()
            };
            let mut first = orient(&first).unwrap();
            let mut second = orient(&second).unwrap();

            canonicalize_unconstrained_edges(&kernel, &points, &mut first, &[]).unwrap();
            canonicalize_unconstrained_edges(&kernel, &points, &mut second, &[]).unwrap();

            let canonical = |mut triangles: Vec<Triangle>| {
                for triangle in &mut triangles {
                    triangle.sort_unstable();
                }
                triangles.sort_unstable();
                triangles
            };
            assert_eq!(canonical(first), canonical(second));
        }
    }
}
