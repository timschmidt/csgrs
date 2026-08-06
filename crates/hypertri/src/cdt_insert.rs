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
use std::cmp::{Ordering, Reverse};
use std::collections::BinaryHeap;

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
    topology: Option<TriangleTopology>,
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
        topology,
    } = recover_constraints(kernel, points, triangles, constraints, None)?;
    legalize_unconstrained_edges(kernel, points, &mut triangles, &constrained_edges, topology)?;
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
    initial_topology: Option<Box<TriangleTopology>>,
) -> Result<Vec<Triangle>> {
    let RecoveredConstraints {
        mut triangles,
        constrained_edges,
        topology,
    } = recover_constraints(
        kernel,
        points,
        triangles,
        constraints,
        initial_topology.map(|topology| *topology),
    )?;
    canonicalize_unconstrained_edges(kernel, points, &mut triangles, &constrained_edges, topology)?;
    Ok(triangles)
}

fn recover_constraints(
    kernel: &ExactKernel,
    points: &[Point2],
    mut triangles: Vec<Triangle>,
    constraints: &[Constraint],
    initial_topology: Option<TriangleTopology>,
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
    let mut topology = initial_topology;
    let mut cavity = Vec::new();
    let mut incident_triangles = Vec::new();
    for &constraint in constraints {
        let edge = EdgeKey::new(constraint.from, constraint.to);
        if topology.is_none() && triangulation_has_edge(&triangles, edge) {
            push_unique_edge(&mut constrained_edges, edge);
            continue;
        }
        let topology = match &mut topology {
            Some(topology) => topology,
            None => topology.insert(TriangleTopology::new(&triangles, points.len())?),
        };
        recover_constraint(
            ConstraintRecovery {
                kernel,
                points,
                triangles: &mut triangles,
                topology,
                constrained_edges: &constrained_edges,
                approximate_points: approximate_points.as_deref(),
                cavity: &mut cavity,
                incident_triangles: &mut incident_triangles,
            },
            constraint,
        )?;
        push_unique_edge(&mut constrained_edges, edge);
    }

    Ok(RecoveredConstraints {
        triangles,
        constrained_edges,
        topology,
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

#[derive(Clone, Copy)]
enum ConstraintLocation {
    ExistingEdge,
    FirstCrossing(ConstraintCrossing),
}

#[derive(Clone, Copy)]
struct ConstraintCrossing {
    edge: EdgeKey,
    before: usize,
    after: usize,
}

struct ConstraintRecovery<'a> {
    kernel: &'a ExactKernel,
    points: &'a [Point2],
    triangles: &'a mut Vec<Triangle>,
    topology: &'a mut TriangleTopology,
    constrained_edges: &'a [EdgeKey],
    approximate_points: Option<&'a [[f64; 2]]>,
    cavity: &'a mut Vec<bool>,
    incident_triangles: &'a mut Vec<usize>,
}

fn recover_constraint(recovery: ConstraintRecovery<'_>, constraint: Constraint) -> Result<()> {
    let location = locate_constraint_from_endpoint(
        recovery.kernel,
        recovery.points,
        recovery.triangles,
        recovery.topology,
        constraint,
        recovery.approximate_points,
        recovery.incident_triangles,
    )?;
    let ConstraintLocation::FirstCrossing(first) = location else {
        return Ok(());
    };

    // Recover one complete crossed-triangle corridor. Rebuilding and sorting
    // global edge incidence after each individually legal flip repeats work
    // proportional to the whole triangulation and can become superlinear in
    // the number of crossings. The exact cavity is the standard structural
    // unit of segment insertion; unconstrained Delaunay legality is restored
    // once after all protected segments have been recovered.
    recover_constraint_cavity(recovery, constraint, first)
}

fn locate_constraint_from_endpoint(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &[Triangle],
    topology: &TriangleTopology,
    constraint: Constraint,
    approximate_points: Option<&[[f64; 2]]>,
    incident_triangles: &mut Vec<usize>,
) -> Result<ConstraintLocation> {
    incident_triangles.clear();
    let Some(start) = topology.vertex_triangle(constraint.from) else {
        return Err(Error::InvalidInput {
            reason: "constraint endpoint is absent from triangulation topology",
        });
    };
    incident_triangles.push(start);
    let mut cursor = 0;
    let mut crossing = None;
    while cursor < incident_triangles.len() {
        let triangle_index = incident_triangles[cursor];
        cursor += 1;
        let triangle = triangles[triangle_index];
        if !triangle.contains(&constraint.from) {
            return Err(Error::InvalidInput {
                reason: "vertex topology references a nonincident triangle",
            });
        }
        if triangle.contains(&constraint.to) {
            return Ok(ConstraintLocation::ExistingEdge);
        }

        for edge in triangle_edges(triangle) {
            if edge.contains(constraint.from) {
                if let Some(neighbor) = topology.neighbor_across(triangles, triangle_index, edge)?
                    && !incident_triangles.contains(&neighbor)
                {
                    incident_triangles.push(neighbor);
                }
                continue;
            }
            if !edge_properly_crosses_constraint(
                kernel,
                points,
                edge,
                constraint,
                approximate_points,
            )? {
                continue;
            }
            if crossing.is_some() {
                return Err(Error::InvalidInput {
                    reason: "constraint leaves its endpoint through multiple triangle edges",
                });
            }
            let Some(after) = topology.neighbor_across(triangles, triangle_index, edge)? else {
                return Err(Error::UnsupportedFeature {
                    feature: "constraint cavity crosses a boundary edge",
                });
            };
            crossing = Some(ConstraintLocation::FirstCrossing(ConstraintCrossing {
                edge,
                before: triangle_index,
                after,
            }));
        }
    }
    crossing.ok_or(Error::UnsupportedFeature {
        feature: "constraint cavity contains no crossed triangles",
    })
}

fn recover_constraint_cavity(
    recovery: ConstraintRecovery<'_>,
    constraint: Constraint,
    first: ConstraintCrossing,
) -> Result<()> {
    let ConstraintRecovery {
        kernel,
        points,
        triangles,
        topology,
        constrained_edges,
        approximate_points,
        cavity,
        incident_triangles: _,
    } = recovery;
    let ConstraintCrossing {
        edge: first_edge,
        before: first_before,
        after: first_after,
    } = first;
    if constrained_edges.binary_search(&first_edge).is_ok() {
        return Err(Error::InvalidInput {
            reason: "constraint crosses an existing constrained edge",
        });
    }
    let first_adjacent = adjacent_triangles(triangles, first_edge, [first_before, first_after])?;
    let mut crossing_count = 1usize;
    let mut incoming = first_edge;
    let mut current = first_after;
    while !triangles[current].contains(&constraint.to) {
        let mut outgoing = None;
        for edge in triangle_edges(triangles[current]) {
            if edge == incoming
                || !edge_properly_crosses_constraint(
                    kernel,
                    points,
                    edge,
                    constraint,
                    approximate_points,
                )?
            {
                continue;
            }
            if outgoing.replace(edge).is_some() {
                return Err(Error::InvalidInput {
                    reason: "constraint crosses multiple outgoing edges of one triangle",
                });
            }
        }
        let Some(edge) = outgoing else {
            return Err(Error::UnsupportedFeature {
                feature: "constraint corridor ends before its target endpoint",
            });
        };
        if constrained_edges.binary_search(&edge).is_ok() {
            return Err(Error::InvalidInput {
                reason: "constraint crosses an existing constrained edge",
            });
        }
        let Some(next) = topology.neighbor_across(triangles, current, edge)? else {
            return Err(Error::UnsupportedFeature {
                feature: "constraint cavity crosses a boundary edge",
            });
        };
        crossing_count += 1;
        if crossing_count > triangles.len() {
            return Err(Error::InvalidInput {
                reason: "constraint corridor does not terminate",
            });
        }
        if crossing_count == 2 {
            cavity.resize(triangles.len(), false);
            cavity.fill(false);
            mark_cavity_triangles(cavity, first_adjacent);
        }
        mark_cavity_triangles(
            cavity,
            adjacent_triangles(triangles, edge, [current, next])?,
        );
        incoming = edge;
        current = next;
    }
    if crossing_count == 1 {
        let replacement = EdgeKey::new(first_adjacent[0].opposite, first_adjacent[1].opposite);
        // A proper crossing between the shared edge and the opposite-vertex
        // diagonal is already the exact convex-quadrilateral certificate.
        // Reuse it instead of repeating the same four side orientations.
        if replacement == EdgeKey::new(constraint.from, constraint.to) {
            return replace_adjacent_edge_in_topology(
                kernel,
                points,
                triangles,
                topology,
                first_edge,
                first_adjacent,
            );
        }
        cavity.resize(triangles.len(), false);
        cavity.fill(false);
        mark_cavity_triangles(cavity, first_adjacent);
    }
    let mut cavity_indices = Vec::new();
    let mut boundary_edges = Vec::new();
    collect_constraint_cavity_boundary(
        topology,
        triangles,
        constrained_edges,
        cavity,
        &mut cavity_indices,
        &mut boundary_edges,
    )?;
    let (cycle, to_position) =
        match constraint_cavity_cycle(points.len(), constraint, &boundary_edges) {
            Ok(cycle) => cycle,
            Err(_) => {
                // A proper crossed corridor normally has one simple boundary. Only
                // a weakly simple corridor can enclose an unselected component, so
                // defer the complete protected-component search until the local
                // boundary proves it is necessary.
                close_constraint_cavity_holes(topology, triangles, constrained_edges, cavity)?;
                collect_constraint_cavity_boundary(
                    topology,
                    triangles,
                    constrained_edges,
                    cavity,
                    &mut cavity_indices,
                    &mut boundary_edges,
                )?;
                constraint_cavity_cycle(points.len(), constraint, &boundary_edges)
                    .map_err(|feature| Error::UnsupportedFeature { feature })?
            }
        };
    let mut cavity_vertices = cavity_indices
        .iter()
        .flat_map(|&triangle| triangles[triangle])
        .collect::<Vec<_>>();
    cavity_vertices.sort_unstable();
    cavity_vertices.dedup();

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
    let mut replacement_topology = None;
    for vertex in cavity_vertices {
        if replacement
            .iter()
            .any(|triangle| triangle.contains(&vertex))
        {
            continue;
        }
        crate::cdt::insert_topology_point(
            kernel,
            points,
            &mut replacement,
            vertex,
            &mut replacement_topology,
        )?;
    }
    if replacement.len() != cavity_indices.len() {
        return Err(Error::UnsupportedFeature {
            feature: "constraint cavity retriangulation changed triangle count",
        });
    }
    if !replacement.iter().any(|triangle| {
        triangle_contains_edge(*triangle, EdgeKey::new(constraint.from, constraint.to))
    }) {
        return Err(Error::UnsupportedFeature {
            feature: "constraint cavity retriangulation omitted the target edge",
        });
    }
    topology.replace_region(triangles, &cavity_indices, &replacement, None)?;
    Ok(())
}

fn collect_constraint_cavity_boundary(
    topology: &TriangleTopology,
    triangles: &[Triangle],
    constrained_edges: &[EdgeKey],
    cavity: &[bool],
    cavity_indices: &mut Vec<usize>,
    boundary_edges: &mut Vec<EdgeKey>,
) -> Result<()> {
    cavity_indices.clear();
    boundary_edges.clear();
    cavity_indices.extend(
        cavity
            .iter()
            .enumerate()
            .filter_map(|(index, &in_cavity)| in_cavity.then_some(index)),
    );
    for &triangle_index in cavity_indices.iter() {
        let triangle = triangles[triangle_index];
        for edge in triangle_edges(triangle) {
            let neighbor = topology.neighbor_across(triangles, triangle_index, edge)?;
            match neighbor {
                None => boundary_edges.push(edge),
                Some(neighbor) if !cavity[neighbor] => boundary_edges.push(edge),
                Some(neighbor)
                    if triangle_index < neighbor
                        && constrained_edges.binary_search(&edge).is_ok() =>
                {
                    return Err(Error::UnsupportedFeature {
                        feature: "constraint cavity contains an existing constrained edge",
                    });
                }
                Some(_) => {}
            }
        }
    }
    boundary_edges.sort_unstable();
    Ok(())
}

fn constraint_cavity_cycle(
    point_count: usize,
    constraint: Constraint,
    boundary_edges: &[EdgeKey],
) -> std::result::Result<(Vec<usize>, usize), &'static str> {
    let missing = usize::MAX;
    let mut adjacency = vec![[missing; 2]; point_count];
    for edge in boundary_edges {
        for (vertex, neighbor) in [(edge.from, edge.to), (edge.to, edge.from)] {
            if adjacency[vertex][0] == missing {
                adjacency[vertex][0] = neighbor;
            } else if adjacency[vertex][1] == missing {
                adjacency[vertex][1] = neighbor;
            } else {
                return Err("constraint cavity boundary is not a simple cycle");
            }
        }
    }
    if adjacency[constraint.from][1] == missing || adjacency[constraint.to][1] == missing {
        return Err("constraint cavity endpoints are not on one boundary cycle");
    }
    let mut cycle = vec![constraint.from];
    let mut previous = usize::MAX;
    let mut current = constraint.from;
    for _ in 0..=boundary_edges.len() {
        let neighbors = &adjacency[current];
        if neighbors[1] == missing {
            return Err("constraint cavity boundary is not a simple cycle");
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
        return Err("constraint cavity boundary traversal did not close");
    }
    let Some(to_position) = cycle.iter().position(|&vertex| vertex == constraint.to) else {
        return Err("constraint cavity boundary omits one endpoint");
    };
    Ok((cycle, to_position))
}

fn mark_cavity_triangles(cavity: &mut [bool], adjacent: [AdjacentTriangle; 2]) {
    cavity[adjacent[0].triangle] = true;
    cavity[adjacent[1].triangle] = true;
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
    topology: &TriangleTopology,
    triangles: &[Triangle],
    constrained_edges: &[EdgeKey],
    cavity: &mut [bool],
) -> Result<()> {
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
            for edge in triangle_edges(triangles[triangle]) {
                if constrained_edges.binary_search(&edge).is_ok() {
                    enclosed = false;
                    continue;
                }
                let Some(neighbor) = topology.neighbor_across(triangles, triangle, edge)? else {
                    enclosed = false;
                    continue;
                };
                if !cavity[neighbor] && !visited[neighbor] {
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
    triangles: &mut Vec<Triangle>,
    constrained_edges: &[EdgeKey],
    topology: Option<TriangleTopology>,
) -> Result<()> {
    match topology {
        Some(topology) => restore_unconstrained_edges(
            kernel,
            points,
            triangles,
            constrained_edges,
            topology,
            EdgeSchedule::Delaunay,
        ),
        None => restore_unconstrained_edges_by_scan(
            kernel,
            points,
            triangles,
            constrained_edges,
            EdgeSchedule::Delaunay,
        ),
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
    triangles: &mut Vec<Triangle>,
    constrained_edges: &[EdgeKey],
    topology: Option<TriangleTopology>,
) -> Result<()> {
    match topology {
        Some(topology) => restore_unconstrained_edges(
            kernel,
            points,
            triangles,
            constrained_edges,
            topology,
            EdgeSchedule::Lexicographic,
        ),
        None => restore_unconstrained_edges_by_scan(
            kernel,
            points,
            triangles,
            constrained_edges,
            EdgeSchedule::Lexicographic,
        ),
    }
}

#[derive(Clone, Copy)]
enum EdgeSchedule {
    Delaunay,
    Lexicographic,
}

fn restore_unconstrained_edges(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut Vec<Triangle>,
    constrained_edges: &[EdgeKey],
    mut topology: TriangleTopology,
    schedule: EdgeSchedule,
) -> Result<()> {
    let mut pending = BinaryHeap::new();
    for (triangle, indices) in triangles.iter().copied().enumerate() {
        for (slot, edge) in triangle_edges(indices).into_iter().enumerate() {
            let neighbor = topology.neighbors[triangle][slot];
            if neighbor.is_some_and(|neighbor| triangle < neighbor)
                && constrained_edges.binary_search(&edge).is_err()
            {
                pending.push(Reverse((edge, triangle)));
            }
        }
    }
    while let Some(Reverse((edge, owner))) = pending.pop() {
        if constrained_edges.binary_search(&edge).is_ok() {
            continue;
        }
        if !triangles
            .get(owner)
            .is_some_and(|triangle| triangle_contains_edge(*triangle, edge))
        {
            continue;
        }
        let Some(neighbor) = topology.neighbor_across(triangles, owner, edge)? else {
            continue;
        };
        let adjacent = adjacent_triangles(triangles, edge, [owner, neighbor])?;
        let [first, second] = adjacent;
        let should_flip = match schedule {
            EdgeSchedule::Delaunay => {
                edge_is_illegal(kernel, points, edge, first.opposite, second.opposite)?
            }
            EdgeSchedule::Lexicographic => {
                EdgeKey::new(first.opposite, second.opposite) < edge
                    && edge_is_flippable(kernel, points, edge, first.opposite, second.opposite)?
            }
        };
        if !should_flip {
            continue;
        }
        let changed = [first.triangle, second.triangle];
        replace_adjacent_edge_in_topology(
            kernel,
            points,
            triangles,
            &mut topology,
            edge,
            adjacent,
        )?;
        for triangle in changed {
            enqueue_triangle_edges(
                triangles,
                &topology,
                constrained_edges,
                triangle,
                &mut pending,
            );
        }
    }
    Ok(())
}

fn restore_unconstrained_edges_by_scan(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut [Triangle],
    constrained_edges: &[EdgeKey],
    schedule: EdgeSchedule,
) -> Result<()> {
    loop {
        let mut replacement = None;
        for topology in TopologyEdges::new(triangles)?.iter() {
            let (edge, owners) = topology?;
            if constrained_edges.binary_search(&edge).is_ok() {
                continue;
            }
            let Some(owners) = owners else {
                continue;
            };
            let adjacent = adjacent_triangles(triangles, edge, owners)?;
            let [first, second] = adjacent;
            let should_flip = match schedule {
                EdgeSchedule::Delaunay => {
                    edge_is_illegal(kernel, points, edge, first.opposite, second.opposite)?
                }
                EdgeSchedule::Lexicographic => {
                    EdgeKey::new(first.opposite, second.opposite) < edge
                        && edge_is_flippable(kernel, points, edge, first.opposite, second.opposite)?
                }
            };
            if should_flip {
                replacement = Some((edge, adjacent));
                break;
            }
        }
        let Some((edge, adjacent)) = replacement else {
            return Ok(());
        };
        let replacement = adjacent_edge_replacement(kernel, points, edge, adjacent)?;
        for (owner, triangle) in adjacent.into_iter().zip(replacement) {
            triangles[owner.triangle] = triangle;
        }
    }
}

fn enqueue_triangle_edges(
    triangles: &[Triangle],
    topology: &TriangleTopology,
    constrained_edges: &[EdgeKey],
    triangle: usize,
    pending: &mut BinaryHeap<Reverse<(EdgeKey, usize)>>,
) {
    for (slot, edge) in triangle_edges(triangles[triangle]).into_iter().enumerate() {
        let neighbor = topology.neighbors[triangle][slot];
        if neighbor.is_some() && constrained_edges.binary_search(&edge).is_err() {
            pending.push(Reverse((edge, triangle)));
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
fn replace_adjacent_edge_in_topology(
    kernel: &ExactKernel,
    points: &[Point2],
    triangles: &mut Vec<Triangle>,
    topology: &mut TriangleTopology,
    edge: EdgeKey,
    mut adjacent: [AdjacentTriangle; 2],
) -> Result<()> {
    adjacent.sort_unstable_by_key(|owner| owner.triangle);
    let replacement = adjacent_edge_replacement(kernel, points, edge, adjacent)?;
    let indices = [adjacent[0].triangle, adjacent[1].triangle];
    topology.replace_region(triangles, &indices, &replacement, None)
}

fn adjacent_edge_replacement(
    kernel: &ExactKernel,
    points: &[Point2],
    edge: EdgeKey,
    adjacent: [AdjacentTriangle; 2],
) -> Result<[Triangle; 2]> {
    let [first, second] = adjacent;
    Ok([
        make_oriented(kernel, points, [first.opposite, second.opposite, edge.from])?,
        make_oriented(kernel, points, [second.opposite, first.opposite, edge.to])?,
    ])
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

/// Checked triangle adjacency retained from nontrivial point insertion through
/// a batch of constraint mutations.
///
/// Point insertion appends local slots; cavity replacement and edge flips
/// preserve the triangle count. Keeping one reciprocal neighbor row per slot
/// avoids rebuilding and sorting the complete edge-use stream while retaining
/// the same manifold checks at construction and every local patch boundary.
pub(crate) struct TriangleTopology {
    neighbors: Vec<[Option<usize>; 3]>,
    vertex_triangles: Vec<Option<usize>>,
    patch: TopologyPatchScratch,
}

#[derive(Default)]
struct TopologyPatchScratch {
    old_boundary: Vec<(EdgeKey, Option<usize>)>,
    edge_uses: Vec<EdgeUse>,
    new_neighbors: Vec<[Option<usize>; 3]>,
    outside_updates: Vec<(usize, usize, usize)>,
}

impl TriangleTopology {
    pub(crate) fn new(triangles: &[Triangle], point_count: usize) -> Result<Self> {
        let mut vertex_triangles = vec![None; point_count];
        for (triangle_index, triangle) in triangles.iter().copied().enumerate() {
            if triangle[0] == triangle[1]
                || triangle[1] == triangle[2]
                || triangle[2] == triangle[0]
            {
                return Err(Error::InvalidInput {
                    reason: "triangle edge has no opposite vertex",
                });
            }
            for vertex in triangle {
                let representative =
                    vertex_triangles
                        .get_mut(vertex)
                        .ok_or(Error::InvalidInput {
                            reason: "triangle vertex index is out of bounds",
                        })?;
                *representative = Some(triangle_index);
            }
        }
        Ok(Self {
            neighbors: crate::cdt::triangle_neighbors(triangles)?,
            vertex_triangles,
            patch: TopologyPatchScratch::default(),
        })
    }

    pub(crate) fn neighbors(&self) -> &[[Option<usize>; 3]] {
        &self.neighbors
    }

    fn vertex_triangle(&self, vertex: usize) -> Option<usize> {
        self.vertex_triangles.get(vertex).copied().flatten()
    }

    pub(crate) fn neighbor_across_vertices(
        &self,
        triangles: &[Triangle],
        triangle: usize,
        first: usize,
        second: usize,
    ) -> Result<Option<usize>> {
        self.neighbor_across(triangles, triangle, EdgeKey::new(first, second))
    }

    fn neighbor_across(
        &self,
        triangles: &[Triangle],
        triangle: usize,
        edge: EdgeKey,
    ) -> Result<Option<usize>> {
        let indices = *triangles.get(triangle).ok_or(Error::InvalidInput {
            reason: "triangle topology references an absent triangle",
        })?;
        let slot = triangle_edge_slot(indices, edge)?;
        let Some(neighbor) = self.neighbors[triangle][slot] else {
            return Ok(None);
        };
        let neighbor_indices = *triangles.get(neighbor).ok_or(Error::InvalidInput {
            reason: "triangle adjacency references an absent triangle",
        })?;
        let neighbor_slot = triangle_edge_slot(neighbor_indices, edge)?;
        if self.neighbors[neighbor][neighbor_slot] != Some(triangle) {
            return Err(Error::InvalidInput {
                reason: "triangle adjacency is not reciprocal",
            });
        }
        Ok(Some(neighbor))
    }

    pub(crate) fn replace_point_region(
        &mut self,
        triangles: &mut Vec<Triangle>,
        indices: &[usize],
        replacement: &[Triangle],
        boundary_split: Option<(usize, usize, usize)>,
    ) -> Result<()> {
        if indices.len() > 2 || replacement.len() > 4 {
            return Err(Error::InvalidInput {
                reason: "point insertion exceeds its local topology bound",
            });
        }
        self.replace_region(
            triangles,
            indices,
            replacement,
            boundary_split.map(|(first, second, point)| (EdgeKey::new(first, second), point)),
        )
    }

    /// Atomically replace triangle slots after proving that the new local
    /// triangulation has exactly the old region boundary. Point insertion may
    /// append slots and may refine one hull edge through its inserted vertex;
    /// constraint recovery and flips preserve both boundary and slot count.
    fn replace_region(
        &mut self,
        triangles: &mut Vec<Triangle>,
        indices: &[usize],
        replacement: &[Triangle],
        boundary_split: Option<(EdgeKey, usize)>,
    ) -> Result<()> {
        let mut patch = std::mem::take(&mut self.patch);
        let result = self.replace_region_with_scratch(
            triangles,
            indices,
            replacement,
            boundary_split,
            &mut patch,
        );
        self.patch = patch;
        result
    }

    fn replace_region_with_scratch(
        &mut self,
        triangles: &mut Vec<Triangle>,
        indices: &[usize],
        replacement: &[Triangle],
        boundary_split: Option<(EdgeKey, usize)>,
        patch: &mut TopologyPatchScratch,
    ) -> Result<()> {
        let TopologyPatchScratch {
            old_boundary,
            edge_uses,
            new_neighbors,
            outside_updates,
        } = patch;
        if indices.is_empty() || replacement.len() < indices.len() {
            return Err(Error::InvalidInput {
                reason: "topology replacement has mismatched triangle slots",
            });
        }
        for pair in indices.windows(2) {
            if pair[0] >= pair[1] {
                return Err(Error::InvalidInput {
                    reason: "topology replacement triangle slots are not unique and sorted",
                });
            }
        }
        for &triangle_index in indices {
            if triangle_index >= triangles.len() {
                return Err(Error::InvalidInput {
                    reason: "topology replacement references an absent triangle",
                });
            }
        }
        for triangle in replacement {
            if triangle[0] == triangle[1]
                || triangle[1] == triangle[2]
                || triangle[2] == triangle[0]
                || triangle
                    .iter()
                    .any(|&vertex| vertex >= self.vertex_triangles.len())
            {
                return Err(Error::InvalidInput {
                    reason: "topology replacement contains an invalid triangle",
                });
            }
        }

        let appended = replacement.len() - indices.len();
        triangles
            .len()
            .checked_add(appended)
            .ok_or(Error::InvalidInput {
                reason: "topology replacement triangle count overflow",
            })?;
        let original_len = triangles.len();

        old_boundary.clear();
        for &triangle_index in indices {
            for edge in triangle_edges(triangles[triangle_index]) {
                let neighbor = self.neighbor_across(triangles, triangle_index, edge)?;
                if neighbor.is_none_or(|neighbor| indices.binary_search(&neighbor).is_err()) {
                    old_boundary.push((edge, neighbor));
                }
            }
        }
        if let Some((split, point)) = boundary_split {
            if split.contains(point) {
                return Err(Error::InvalidInput {
                    reason: "topology replacement does not split an edge interior",
                });
            }
            let old_uses = indices
                .iter()
                .filter(|&&index| triangle_contains_edge(triangles[index], split))
                .count();
            match old_uses {
                1 => {
                    let position = old_boundary
                        .iter()
                        .position(|&(edge, _)| edge == split)
                        .ok_or(Error::InvalidInput {
                            reason: "split edge is absent from the topology boundary",
                        })?;
                    if old_boundary[position].1.is_some() {
                        return Err(Error::InvalidInput {
                            reason: "split edge omitted an adjacent triangle",
                        });
                    }
                    old_boundary.swap_remove(position);
                    old_boundary.extend([
                        (EdgeKey::new(split.from, point), None),
                        (EdgeKey::new(point, split.to), None),
                    ]);
                }
                2 => {
                    if old_boundary.iter().any(|&(edge, _)| edge == split) {
                        return Err(Error::InvalidInput {
                            reason: "interior split edge remained on the topology boundary",
                        });
                    }
                }
                _ => {
                    return Err(Error::InvalidInput {
                        reason: "split edge has invalid topology incidence",
                    });
                }
            }
        }
        old_boundary.sort_unstable_by_key(|&(edge, _)| edge);
        if old_boundary.windows(2).any(|pair| pair[0].0 == pair[1].0) {
            return Err(Error::InvalidInput {
                reason: "topology replacement boundary contains a duplicate edge",
            });
        }

        edge_uses.clear();
        edge_uses.reserve(replacement.len().saturating_mul(3));
        for (local, triangle) in replacement.iter().enumerate() {
            for edge in triangle_edges(*triangle) {
                edge_uses.push(EdgeUse {
                    edge,
                    triangle: replacement_triangle_index(indices, original_len, local),
                });
            }
        }
        edge_uses.sort_unstable();
        new_neighbors.clear();
        new_neighbors.resize(replacement.len(), [None; 3]);
        outside_updates.clear();
        let mut boundary_count = 0usize;
        let mut start = 0;
        while start < edge_uses.len() {
            let edge = edge_uses[start].edge;
            let mut end = start + 1;
            while end < edge_uses.len() && edge_uses[end].edge == edge {
                end += 1;
            }
            match end - start {
                1 => {
                    boundary_count += 1;
                    let boundary_position = old_boundary
                        .binary_search_by_key(&edge, |&(candidate, _)| candidate)
                        .map_err(|_| Error::InvalidInput {
                            reason: "topology replacement changes the region boundary",
                        })?;
                    let triangle = edge_uses[start].triangle;
                    let local = replacement_local_index(
                        indices,
                        original_len,
                        replacement.len(),
                        triangle,
                    )?;
                    let slot = triangle_edge_slot(replacement[local], edge)?;
                    let outside = old_boundary[boundary_position].1;
                    new_neighbors[local][slot] = outside;
                    if let Some(outside) = outside {
                        let outside_slot = triangle_edge_slot(triangles[outside], edge)?;
                        let Some(old_inside) = self.neighbors[outside][outside_slot] else {
                            return Err(Error::InvalidInput {
                                reason: "topology replacement boundary is not reciprocal",
                            });
                        };
                        if indices.binary_search(&old_inside).is_err() {
                            return Err(Error::InvalidInput {
                                reason: "topology replacement boundary is not reciprocal",
                            });
                        }
                        outside_updates.push((outside, outside_slot, triangle));
                    }
                }
                2 => {
                    let first = edge_uses[start].triangle;
                    let second = edge_uses[start + 1].triangle;
                    let first_local =
                        replacement_local_index(indices, original_len, replacement.len(), first)?;
                    let second_local =
                        replacement_local_index(indices, original_len, replacement.len(), second)?;
                    let first_slot = triangle_edge_slot(replacement[first_local], edge)?;
                    let second_slot = triangle_edge_slot(replacement[second_local], edge)?;
                    new_neighbors[first_local][first_slot] = Some(second);
                    new_neighbors[second_local][second_slot] = Some(first);
                }
                _ => {
                    return Err(Error::InvalidInput {
                        reason: "topology replacement contains a non-manifold edge",
                    });
                }
            }
            start = end;
        }
        if boundary_count != old_boundary.len() {
            return Err(Error::InvalidInput {
                reason: "topology replacement omits an existing boundary edge",
            });
        }

        triangles.reserve(appended);
        self.neighbors.reserve(appended);
        for (local, (triangle, neighbors)) in replacement
            .iter()
            .zip(new_neighbors.iter().copied())
            .enumerate()
        {
            let triangle_index = replacement_triangle_index(indices, original_len, local);
            if local < indices.len() {
                triangles[triangle_index] = *triangle;
                self.neighbors[triangle_index] = neighbors;
            } else {
                debug_assert_eq!(triangle_index, triangles.len());
                triangles.push(*triangle);
                self.neighbors.push(neighbors);
            }
            for &vertex in triangle {
                self.vertex_triangles[vertex] = Some(triangle_index);
            }
        }
        for (outside, slot, triangle) in outside_updates.iter().copied() {
            self.neighbors[outside][slot] = Some(triangle);
        }
        Ok(())
    }
}

fn replacement_triangle_index(indices: &[usize], original_len: usize, local: usize) -> usize {
    if local < indices.len() {
        indices[local]
    } else {
        original_len + local - indices.len()
    }
}

fn replacement_local_index(
    indices: &[usize],
    original_len: usize,
    replacement_len: usize,
    triangle: usize,
) -> Result<usize> {
    if triangle < original_len {
        return indices
            .binary_search(&triangle)
            .map_err(|_| Error::InvalidInput {
                reason: "topology replacement edge has no triangle slot",
            });
    }
    let local = indices.len() + triangle - original_len;
    if local < replacement_len {
        Ok(local)
    } else {
        Err(Error::InvalidInput {
            reason: "topology replacement edge has no appended slot",
        })
    }
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

fn triangle_edges(triangle: Triangle) -> [EdgeKey; 3] {
    [
        EdgeKey::new(triangle[0], triangle[1]),
        EdgeKey::new(triangle[1], triangle[2]),
        EdgeKey::new(triangle[2], triangle[0]),
    ]
}

fn triangle_edge_slot(triangle: Triangle, edge: EdgeKey) -> Result<usize> {
    triangle_edges(triangle)
        .iter()
        .position(|&candidate| candidate == edge)
        .ok_or(Error::InvalidInput {
            reason: "triangle topology references an absent edge",
        })
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
    fn retained_topology_updates_reciprocal_neighbors_after_a_flip() {
        let points = vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)];
        let mut triangles = vec![[0, 1, 2], [0, 2, 3]];
        let mut topology = TriangleTopology::new(&triangles, points.len()).unwrap();
        let context = TriangulationContext::new(hyperlimit::PredicatePolicy::STRICT);
        let kernel = ExactKernel::new(&context);
        let edge = EdgeKey::new(0, 2);
        let adjacent = adjacent_triangles(&triangles, edge, [0, 1]).unwrap();

        replace_adjacent_edge_in_topology(
            &kernel,
            &points,
            &mut triangles,
            &mut topology,
            edge,
            adjacent,
        )
        .unwrap();

        let replacement = EdgeKey::new(1, 3);
        assert!(!triangulation_has_edge(&triangles, edge));
        assert!(triangulation_has_edge(&triangles, replacement));
        assert_eq!(
            topology.neighbor_across(&triangles, 0, replacement),
            Ok(Some(1))
        );
        assert_eq!(
            topology.neighbor_across(&triangles, 1, replacement),
            Ok(Some(0))
        );
    }

    #[test]
    fn retained_topology_rejects_a_changed_boundary_atomically() {
        let original = vec![[0, 1, 2]];
        let mut triangles = original.clone();
        let mut topology = TriangleTopology::new(&triangles, 4).unwrap();
        let neighbors = topology.neighbors.clone();
        let vertex_triangles = topology.vertex_triangles.clone();

        assert_eq!(
            topology.replace_region(&mut triangles, &[0], &[[0, 1, 3]], None),
            Err(Error::InvalidInput {
                reason: "topology replacement changes the region boundary",
            })
        );
        assert_eq!(triangles, original);
        assert_eq!(topology.neighbors, neighbors);
        assert_eq!(topology.vertex_triangles, vertex_triangles);
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
            let mut topology = TriangleTopology::new(&triangles, points.len()).unwrap();
            let mut cavity = Vec::new();
            let mut incident_triangles = Vec::new();

            recover_constraint(
                ConstraintRecovery {
                    kernel: &kernel,
                    points: &points,
                    triangles: &mut triangles,
                    topology: &mut topology,
                    constrained_edges: &[],
                    approximate_points: approximate.as_deref(),
                    cavity: &mut cavity,
                    incident_triangles: &mut incident_triangles,
                },
                constraint,
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
                    crate::cdt::insert_topology_point(
                        &kernel,
                        &points,
                        &mut triangles,
                        vertex,
                        &mut None,
                    )
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
        let topology = TriangleTopology::new(&triangles, (SIDE + 1) * (SIDE + 1)).unwrap();
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

        let constraint = Constraint::new(vertex(1, 1), vertex(4, 4));
        let mut cavity_indices = Vec::new();
        let mut boundary_edges = Vec::new();
        collect_constraint_cavity_boundary(
            &topology,
            &triangles,
            &[],
            &corridor,
            &mut cavity_indices,
            &mut boundary_edges,
        )
        .unwrap();
        assert!(
            constraint_cavity_cycle((SIDE + 1) * (SIDE + 1), constraint, &boundary_edges).is_err()
        );

        let mut closed = corridor.clone();
        close_constraint_cavity_holes(&topology, &triangles, &[], &mut closed).unwrap();
        assert!(closed[center(2, 2)] && closed[center(2, 2) + 1]);
        assert!(!closed[center(0, 0)] && !closed[center(4, 4)]);
        collect_constraint_cavity_boundary(
            &topology,
            &triangles,
            &[],
            &closed,
            &mut cavity_indices,
            &mut boundary_edges,
        )
        .unwrap();
        assert!(
            constraint_cavity_cycle((SIDE + 1) * (SIDE + 1), constraint, &boundary_edges).is_ok()
        );

        let central_diagonal = EdgeKey::new(vertex(2, 2), vertex(3, 3));
        close_constraint_cavity_holes(&topology, &triangles, &[central_diagonal], &mut corridor)
            .unwrap();
        assert!(!corridor[center(2, 2)] && !corridor[center(2, 2) + 1]);
        collect_constraint_cavity_boundary(
            &topology,
            &triangles,
            &[central_diagonal],
            &corridor,
            &mut cavity_indices,
            &mut boundary_edges,
        )
        .unwrap();
        assert!(
            constraint_cavity_cycle((SIDE + 1) * (SIDE + 1), constraint, &boundary_edges).is_err()
        );
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
            let mut retained_first = first.clone();
            let mut retained_second = second.clone();
            let first_topology = TriangleTopology::new(&first, points.len()).unwrap();
            let second_topology = TriangleTopology::new(&second, points.len()).unwrap();

            canonicalize_unconstrained_edges(&kernel, &points, &mut first, &[], None).unwrap();
            canonicalize_unconstrained_edges(&kernel, &points, &mut second, &[], None).unwrap();
            canonicalize_unconstrained_edges(
                &kernel,
                &points,
                &mut retained_first,
                &[],
                Some(first_topology),
            )
            .unwrap();
            canonicalize_unconstrained_edges(
                &kernel,
                &points,
                &mut retained_second,
                &[],
                Some(second_topology),
            )
            .unwrap();

            let canonical = |mut triangles: Vec<Triangle>| {
                for triangle in &mut triangles {
                    triangle.sort_unstable();
                }
                triangles.sort_unstable();
                triangles
            };
            let first = canonical(first);
            assert_eq!(first, canonical(retained_first));
            assert_eq!(first, canonical(second));
            assert_eq!(first, canonical(retained_second));
        }
    }
}
