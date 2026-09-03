//! Exact constraint recovery with structural or Delaunay legalization.
//!
//! The public [`crate::cdt`] module owns API shape and result records; this
//! module keeps the incremental segment-insertion machinery local. The
//! implementation starts from the exact Delaunay triangulation, planarizes
//! crossing constraints into exact Steiner vertices, recovers each protected
//! subsegment by flipping crossed unconstrained edges or retriangulating its
//! exact cavity, then re-legalizes only unconstrained edges. Correctness
//! reduces to complete convex-hull coverage and local Delaunay checks on
//! unprotected edges; exact predicate ownership stays in the evaluator/predicate
//! layer.

use crate::error::{Error, Result};
use crate::predicate_evaluator::PredicateEvaluator;
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
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    triangles: Vec<Triangle>,
    constraints: &[Constraint],
) -> Result<Vec<Triangle>> {
    let RecoveredConstraints {
        mut triangles,
        constrained_edges,
        topology,
    } = recover_constraints(evaluator, points, triangles, constraints, None)?;
    legalize_unconstrained_edges(
        evaluator,
        points,
        &mut triangles,
        &constrained_edges,
        topology,
    )?;
    Ok(triangles)
}

/// Insert every constraint without imposing a triangle-quality policy on the
/// remaining edges.
///
/// This is the topology-only counterpart of [`insert_constraints`]. The same
/// exact crossing, flip, and cavity predicates recover every protected edge;
/// only the final empty-circle legalization sweep is omitted.
pub(crate) fn insert_constraints_topology(
    evaluator: &PredicateEvaluator,
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
        evaluator,
        points,
        triangles,
        constraints,
        initial_topology.map(|topology| *topology),
    )?;
    canonicalize_unconstrained_edges(
        evaluator,
        points,
        &mut triangles,
        &constrained_edges,
        topology,
    )?;
    Ok(triangles)
}

fn recover_constraints(
    evaluator: &PredicateEvaluator,
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
    let mut left_chain = Vec::new();
    let mut right_chain = Vec::new();
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
                evaluator,
                points,
                triangles: &mut triangles,
                topology,
                constrained_edges: &constrained_edges,
                approximate_points: approximate_points.as_deref(),
                cavity: &mut cavity,
                incident_triangles: &mut incident_triangles,
                left_chain: &mut left_chain,
                right_chain: &mut right_chain,
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
    evaluator: &PredicateEvaluator,
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
                evaluator,
                &planar_points[a.from],
                &planar_points[a.to],
                &planar_points[b.from],
                &planar_points[b.to],
            )?
            .is_proper_crossing()
            {
                let point = segment_intersection_point(evaluator, &planar_points, a, b)?;
                push_unique_point(evaluator, &mut planar_points, point)?;
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
                evaluator,
                &planar_points[constraint.from],
                &planar_points[constraint.to],
                &planar_points[point_index],
            )? {
                on_segment.push(point_index);
            }
        }

        sort_indices_on_segment(evaluator, &planar_points, constraint, &mut on_segment)?;
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
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    first: Constraint,
    second: Constraint,
) -> Result<Point2> {
    let a = &points[first.from];
    let b = &points[first.to];
    let c = &points[second.from];
    let d = &points[second.to];

    match hyperlimit::construct_line_intersection_point_with_policy(a, b, c, d, evaluator.policy())
    {
        Some(point) => Ok(Point2::new(point.x, point.y)),
        None => Err(Error::InvalidInput {
            reason: "properly crossing constraint lines have no constructible intersection",
        }),
    }
}

fn push_unique_point(
    evaluator: &PredicateEvaluator,
    points: &mut Vec<Point2>,
    point: Point2,
) -> Result<usize> {
    for (index, candidate) in points.iter().enumerate() {
        if predicates::points_equal(evaluator, candidate, &point)? {
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
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    constraint: &Constraint,
    indices: &mut [usize],
) -> Result<()> {
    let use_x = compare_segment_axis_reals(
        evaluator,
        &points[constraint.from].x,
        &points[constraint.to].x,
        "compare_constraint_endpoint_x",
    )? != Ordering::Equal;

    for index in 1..indices.len() {
        let mut cursor = index;
        while cursor > 0
            && compare_segment_indices(
                evaluator,
                points,
                indices[cursor],
                indices[cursor - 1],
                use_x,
            )? == Ordering::Less
        {
            indices.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }

    Ok(())
}

fn compare_segment_indices(
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    left: usize,
    right: usize,
    use_x: bool,
) -> Result<Ordering> {
    if use_x {
        compare_segment_axis_reals(
            evaluator,
            &points[left].x,
            &points[right].x,
            "compare_segment_x",
        )
    } else {
        compare_segment_axis_reals(
            evaluator,
            &points[left].y,
            &points[right].y,
            "compare_segment_y",
        )
    }
}

fn compare_segment_axis_reals(
    evaluator: &PredicateEvaluator,
    left: &crate::types::Real,
    right: &crate::types::Real,
    predicate: &'static str,
) -> Result<Ordering> {
    // Points have already been certified to lie on this segment. The remaining
    // subsegment split order is therefore a scalar exact-ordering predicate,
    // which belongs in hyperlimit rather than CDT topology.
    evaluator.decide(
        hyperlimit::compare_reals(left, right, evaluator.policy()),
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
    edge_sides: [Sign; 2],
    before: usize,
    after: usize,
}

struct ConstraintRecovery<'a> {
    evaluator: &'a PredicateEvaluator,
    points: &'a [Point2],
    triangles: &'a mut Vec<Triangle>,
    topology: &'a mut TriangleTopology,
    constrained_edges: &'a [EdgeKey],
    approximate_points: Option<&'a [[f64; 2]]>,
    cavity: &'a mut Vec<bool>,
    incident_triangles: &'a mut Vec<usize>,
    left_chain: &'a mut Vec<usize>,
    right_chain: &'a mut Vec<usize>,
}

fn recover_constraint(recovery: ConstraintRecovery<'_>, constraint: Constraint) -> Result<()> {
    let location = locate_constraint_from_endpoint(
        recovery.evaluator,
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
    evaluator: &PredicateEvaluator,
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
            let Some(edge_sides) = edge_proper_crossing_sides(
                evaluator,
                points,
                edge,
                constraint,
                approximate_points,
            )?
            else {
                continue;
            };
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
                edge_sides,
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
        evaluator,
        points,
        triangles,
        topology,
        constrained_edges,
        approximate_points,
        cavity,
        incident_triangles,
        left_chain,
        right_chain,
    } = recovery;
    let ConstraintCrossing {
        edge: first_edge,
        edge_sides: first_edge_sides,
        before: first_before,
        after: first_after,
    } = first;
    if constrained_edges.binary_search(&first_edge).is_ok() {
        return Err(Error::InvalidInput {
            reason: "constraint crosses an existing constrained edge",
        });
    }
    let first_adjacent = adjacent_triangles(triangles, first_edge, [first_before, first_after])?;
    // Build both half-hole boundaries while walking the exact line corridor.
    // A non-crossed edge shared by nonconsecutive corridor faces appears once
    // in each direction in its side chain. Retaining that weakly-simple spike
    // preserves prior constrained chords without absorbing the component that
    // they protect or searching the global cavity boundary afterward.
    left_chain.clear();
    right_chain.clear();
    left_chain.push(constraint.from);
    right_chain.push(constraint.from);
    for (vertex, side) in [first_edge.from, first_edge.to]
        .into_iter()
        .zip(first_edge_sides)
    {
        match side {
            Sign::Positive => left_chain.push(vertex),
            Sign::Negative => right_chain.push(vertex),
            Sign::Zero => {
                return Err(Error::InvalidInput {
                    reason: "constraint corridor contains an unsplit collinear vertex",
                });
            }
        }
    }
    if left_chain.len() != 2 || right_chain.len() != 2 {
        return Err(Error::InvalidInput {
            reason: "constraint corridor first edge does not straddle the target",
        });
    }
    let mut crossing_count = 1usize;
    let mut incoming = first_edge;
    let mut current = first_after;
    while !triangles[current].contains(&constraint.to) {
        let mut outgoing = None;
        for edge in triangle_edges(triangles[current]) {
            if edge == incoming {
                continue;
            }
            let Some(edge_sides) = edge_proper_crossing_sides(
                evaluator,
                points,
                edge,
                constraint,
                approximate_points,
            )?
            else {
                continue;
            };
            if outgoing.replace((edge, edge_sides)).is_some() {
                return Err(Error::InvalidInput {
                    reason: "constraint crosses multiple outgoing edges of one triangle",
                });
            }
        }
        let Some((edge, edge_sides)) = outgoing else {
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
        let next_vertex = triangles[current]
            .iter()
            .copied()
            .find(|&vertex| !incoming.contains(vertex))
            .ok_or(Error::InvalidInput {
                reason: "constraint corridor triangle has no advancing vertex",
            })?;
        let next_side = if next_vertex == edge.from {
            edge_sides[0]
        } else if next_vertex == edge.to {
            edge_sides[1]
        } else {
            return Err(Error::InvalidInput {
                reason: "constraint corridor advancing vertex is absent from its outgoing edge",
            });
        };
        match next_side {
            Sign::Positive => left_chain.push(next_vertex),
            Sign::Negative => right_chain.push(next_vertex),
            Sign::Zero => {
                return Err(Error::InvalidInput {
                    reason: "constraint corridor contains an unsplit collinear vertex",
                });
            }
        }
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
    left_chain.push(constraint.to);
    right_chain.push(constraint.to);
    if crossing_count == 1 {
        let replacement = EdgeKey::new(first_adjacent[0].opposite, first_adjacent[1].opposite);
        // A proper crossing between the shared edge and the opposite-vertex
        // diagonal is already the exact convex-quadrilateral certificate.
        // Reuse it instead of repeating the same four side orientations.
        if replacement == EdgeKey::new(constraint.from, constraint.to) {
            return replace_adjacent_edge_in_topology(
                evaluator,
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
    incident_triangles.clear();
    incident_triangles.extend(
        cavity
            .iter()
            .enumerate()
            .filter_map(|(index, &in_cavity)| in_cavity.then_some(index)),
    );
    let cavity_indices = incident_triangles;
    let target = EdgeKey::new(constraint.from, constraint.to);
    let mut replacement = match triangulate_cavity_region(
        evaluator,
        points,
        triangles,
        cavity_indices.as_slice(),
        [left_chain, right_chain],
    ) {
        Ok(replacement) => Some(replacement),
        Err(Error::NoEarFound) => None,
        Err(error) => return Err(error),
    };
    let direct_is_complete = match &replacement {
        Some(replacement) if replacement.len() == cavity_indices.len() => {
            triangulation_has_edge(replacement, target)
                && replacement_retains_chain_constraints(
                    [left_chain, right_chain],
                    constrained_edges,
                    replacement,
                )
        }
        _ => false,
    };
    if !direct_is_complete {
        // A line corridor can wrap around an unselected, unconstrained face
        // component and expose a weak half-hole. Absorb exactly those enclosed
        // components, then erase only the boundary detours that became
        // internal. Protected detours remain in the ordered half-hole.
        close_constraint_cavity_holes(topology, triangles, constrained_edges, cavity)?;
        cavity_indices.clear();
        cavity_indices.extend(
            cavity
                .iter()
                .enumerate()
                .filter_map(|(index, &in_cavity)| in_cavity.then_some(index)),
        );
        let mut boundary_edges = Vec::new();
        collect_cavity_boundary(
            topology,
            triangles,
            cavity,
            cavity_indices.as_slice(),
            &mut boundary_edges,
        )?;
        prune_absorbed_chain_detours(left_chain, constrained_edges, &boundary_edges);
        prune_absorbed_chain_detours(right_chain, constrained_edges, &boundary_edges);
        replacement = Some(triangulate_cavity_region(
            evaluator,
            points,
            triangles,
            cavity_indices.as_slice(),
            [left_chain, right_chain],
        )?);
    }
    let Some(replacement) = replacement else {
        return Err(Error::NoEarFound);
    };
    if replacement.len() != cavity_indices.len() {
        return Err(Error::UnsupportedFeature {
            feature: "constraint cavity retriangulation changed triangle count",
        });
    }
    if !triangulation_has_edge(&replacement, target) {
        return Err(Error::UnsupportedFeature {
            feature: "constraint cavity retriangulation omitted the target edge",
        });
    }
    topology.replace_region(triangles, cavity_indices.as_slice(), &replacement, None)?;
    Ok(())
}

fn mark_cavity_triangles(cavity: &mut [bool], adjacent: [AdjacentTriangle; 2]) {
    cavity[adjacent[0].triangle] = true;
    cavity[adjacent[1].triangle] = true;
}

fn triangulate_cavity_region(
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    triangles: &[Triangle],
    cavity_indices: &[usize],
    sides: [&[usize]; 2],
) -> Result<Vec<Triangle>> {
    let mut cavity_vertices = cavity_indices
        .iter()
        .flat_map(|&triangle| triangles[triangle])
        .collect::<Vec<_>>();
    cavity_vertices.sort_unstable();
    cavity_vertices.dedup();

    let mut replacement = Vec::with_capacity(cavity_indices.len());
    // The corridor walk orders both half-hole chains from the constraint's
    // start to its end and classifies every interior vertex against that
    // directed line. Closing the positive-side (left) chain along the
    // constraint is therefore clockwise; closing the negative-side (right)
    // chain is counterclockwise. Weakly-simple protected-edge spikes add zero
    // signed area and do not change either winding. Carry that exact topology
    // fact into ear selection instead of rebuilding the whole-ring area.
    for (side, winding) in sides.into_iter().zip([Sign::Negative, Sign::Positive]) {
        if side.len() >= 3 {
            replacement.extend(triangulate_cavity_side(
                evaluator,
                points,
                side.to_vec(),
                winding,
            )?);
        }
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
            evaluator,
            points,
            &mut replacement,
            vertex,
            &mut replacement_topology,
        )?;
    }
    Ok(replacement)
}

fn replacement_retains_chain_constraints(
    sides: [&[usize]; 2],
    constrained_edges: &[EdgeKey],
    replacement: &[Triangle],
) -> bool {
    for side in sides {
        for pair in side.windows(2) {
            let edge = EdgeKey::new(pair[0], pair[1]);
            if constrained_edges.binary_search(&edge).is_ok()
                && !triangulation_has_edge(replacement, edge)
            {
                return false;
            }
        }
    }
    true
}

fn collect_cavity_boundary(
    topology: &TriangleTopology,
    triangles: &[Triangle],
    cavity: &[bool],
    cavity_indices: &[usize],
    boundary_edges: &mut Vec<EdgeKey>,
) -> Result<()> {
    boundary_edges.clear();
    for &triangle_index in cavity_indices {
        for edge in triangle_edges(triangles[triangle_index]) {
            match topology.neighbor_across(triangles, triangle_index, edge)? {
                None => boundary_edges.push(edge),
                Some(neighbor) if !cavity[neighbor] => boundary_edges.push(edge),
                Some(_) => {}
            }
        }
    }
    boundary_edges.sort_unstable();
    Ok(())
}

fn prune_absorbed_chain_detours(
    chain: &mut Vec<usize>,
    constrained_edges: &[EdgeKey],
    boundary_edges: &[EdgeKey],
) {
    let mut first = 0;
    while first + 1 < chain.len() {
        let Some(second) = (first + 1..chain.len()).find(|&index| chain[index] == chain[first])
        else {
            first += 1;
            continue;
        };
        let absorbed = chain[first..=second].windows(2).all(|pair| {
            let edge = EdgeKey::new(pair[0], pair[1]);
            constrained_edges.binary_search(&edge).is_err()
                && boundary_edges.binary_search(&edge).is_err()
        });
        if absorbed {
            chain.drain(first + 1..=second);
            first = first.saturating_sub(1);
        } else {
            first += 1;
        }
    }
}

/// Add only unprotected face components enclosed by the walked corridor.
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
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    mut ring: Vec<usize>,
    winding: Sign,
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
            if predicates::orient2(
                evaluator,
                &points[previous],
                &points[current],
                &points[next],
            )? == Sign::Zero
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
    if ring.len() < 3 || winding == Sign::Zero {
        return Err(Error::NoEarFound);
    }

    let mut triangles = Vec::with_capacity(ring.len().saturating_sub(2));
    while ring.len() > 3 {
        let mut ear = None;
        for position in 0..ring.len() {
            let previous = ring[(position + ring.len() - 1) % ring.len()];
            let current = ring[position];
            let next = ring[(position + 1) % ring.len()];
            let turn = predicates::orient2(
                evaluator,
                &points[previous],
                &points[current],
                &points[next],
            )?;
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
                if predicates::point_in_or_on_triangle_with_orientation(
                    evaluator,
                    &points[previous],
                    &points[current],
                    &points[next],
                    &points[candidate],
                    turn,
                )? {
                    contains_vertex = true;
                    break;
                }
            }
            if !contains_vertex {
                ear = Some((position, [previous, current, next], turn));
                break;
            }
        }
        let Some((position, triangle, turn)) = ear else {
            return Err(Error::NoEarFound);
        };
        triangles.push(oriented_triangle(turn, triangle)?);
        ring.remove(position);
    }
    if ring.len() == 3 {
        triangles.push(make_oriented(
            evaluator,
            points,
            [ring[0], ring[1], ring[2]],
        )?);
    }
    Ok(triangles)
}

fn edge_proper_crossing_sides(
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    edge: EdgeKey,
    constraint: Constraint,
    approximate_points: Option<&[[f64; 2]]>,
) -> Result<Option<[Sign; 2]>> {
    if edge.contains(constraint.from) || edge.contains(constraint.to) {
        return Ok(None);
    }
    if approximate_points.is_some_and(|points| {
        !crate::cdt::approximate_constraint_bounds_overlap(
            points,
            constraint,
            Constraint::new(edge.from, edge.to),
        )
    }) {
        return Ok(None);
    }
    let edge_sides = [
        predicates::orient2(
            evaluator,
            &points[constraint.from],
            &points[constraint.to],
            &points[edge.from],
        )?,
        predicates::orient2(
            evaluator,
            &points[constraint.from],
            &points[constraint.to],
            &points[edge.to],
        )?,
    ];
    if !signs_strictly_differ(edge_sides[0], edge_sides[1]) {
        return Ok(None);
    }
    let constraint_sides = [
        predicates::orient2(
            evaluator,
            &points[edge.from],
            &points[edge.to],
            &points[constraint.from],
        )?,
        predicates::orient2(
            evaluator,
            &points[edge.from],
            &points[edge.to],
            &points[constraint.to],
        )?,
    ];
    Ok(signs_strictly_differ(constraint_sides[0], constraint_sides[1]).then_some(edge_sides))
}

fn legalize_unconstrained_edges(
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    triangles: &mut Vec<Triangle>,
    constrained_edges: &[EdgeKey],
    topology: Option<TriangleTopology>,
) -> Result<()> {
    match topology {
        Some(topology) => restore_unconstrained_edges(
            evaluator,
            points,
            triangles,
            constrained_edges,
            topology,
            EdgeSchedule::Delaunay,
        ),
        None => restore_unconstrained_edges_by_scan(
            evaluator,
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
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    triangles: &mut Vec<Triangle>,
    constrained_edges: &[EdgeKey],
    topology: Option<TriangleTopology>,
) -> Result<()> {
    match topology {
        Some(topology) => restore_unconstrained_edges(
            evaluator,
            points,
            triangles,
            constrained_edges,
            topology,
            EdgeSchedule::Lexicographic,
        ),
        None => restore_unconstrained_edges_by_scan(
            evaluator,
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
    evaluator: &PredicateEvaluator,
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
                edge_is_illegal(evaluator, points, edge, first.opposite, second.opposite)?
            }
            EdgeSchedule::Lexicographic => {
                EdgeKey::new(first.opposite, second.opposite) < edge
                    && edge_is_flippable(evaluator, points, edge, first.opposite, second.opposite)?
            }
        };
        if !should_flip {
            continue;
        }
        let changed = [first.triangle, second.triangle];
        replace_adjacent_edge_in_topology(
            evaluator,
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
    evaluator: &PredicateEvaluator,
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
                    edge_is_illegal(evaluator, points, edge, first.opposite, second.opposite)?
                }
                EdgeSchedule::Lexicographic => {
                    EdgeKey::new(first.opposite, second.opposite) < edge
                        && edge_is_flippable(
                            evaluator,
                            points,
                            edge,
                            first.opposite,
                            second.opposite,
                        )?
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
        let replacement = adjacent_edge_replacement(evaluator, points, edge, adjacent)?;
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
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    edge: EdgeKey,
    first_opposite: usize,
    second_opposite: usize,
) -> Result<bool> {
    if !edge_is_flippable(evaluator, points, edge, first_opposite, second_opposite)? {
        return Ok(false);
    }

    let orientation = predicates::orient2(
        evaluator,
        &points[edge.from],
        &points[edge.to],
        &points[first_opposite],
    )?;
    if orientation == Sign::Zero {
        return Ok(false);
    }

    let incircle = evaluator.incircle2(
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
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    triangles: &mut Vec<Triangle>,
    topology: &mut TriangleTopology,
    edge: EdgeKey,
    mut adjacent: [AdjacentTriangle; 2],
) -> Result<()> {
    adjacent.sort_unstable_by_key(|owner| owner.triangle);
    let replacement = adjacent_edge_replacement(evaluator, points, edge, adjacent)?;
    let indices = [adjacent[0].triangle, adjacent[1].triangle];
    topology.replace_region(triangles, &indices, &replacement, None)
}

fn adjacent_edge_replacement(
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    edge: EdgeKey,
    adjacent: [AdjacentTriangle; 2],
) -> Result<[Triangle; 2]> {
    let [first, second] = adjacent;
    Ok([
        make_oriented(
            evaluator,
            points,
            [first.opposite, second.opposite, edge.from],
        )?,
        make_oriented(
            evaluator,
            points,
            [second.opposite, first.opposite, edge.to],
        )?,
    ])
}

fn edge_is_flippable(
    evaluator: &PredicateEvaluator,
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
        evaluator,
        &points[edge.from],
        &points[edge.to],
        &points[first_opposite],
    )?;
    let second_side = predicates::orient2(
        evaluator,
        &points[edge.from],
        &points[edge.to],
        &points[second_opposite],
    )?;
    let opposite_edge_side = predicates::orient2(
        evaluator,
        &points[first_opposite],
        &points[second_opposite],
        &points[edge.from],
    )?;
    let opposite_other_side = predicates::orient2(
        evaluator,
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
    evaluator: &PredicateEvaluator,
    points: &[Point2],
    triangle: Triangle,
) -> Result<Triangle> {
    let sign = predicates::orient2(
        evaluator,
        &points[triangle[0]],
        &points[triangle[1]],
        &points[triangle[2]],
    )?;
    oriented_triangle(sign, triangle)
}

fn oriented_triangle(sign: Sign, mut triangle: Triangle) -> Result<Triangle> {
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

    fn exact_normal_positive() -> Real {
        let root_two = Real::from(2).sqrt().unwrap();
        let root_two_over_pi = (root_two.clone() / Real::pi()).unwrap();
        let half = (Real::from(1) / Real::from(2)).unwrap();
        let shared_offset = root_two.clone() * Real::from(3) + half;
        let contact = (((root_two.clone() * Real::from(4) - shared_offset.clone()) * Real::pi())
            * root_two_over_pi.clone()
            / Real::from(4))
        .unwrap();
        let domain = (((root_two * Real::from(2) - shared_offset) * Real::pi()) * root_two_over_pi
            / Real::from(4))
        .unwrap()
            + Real::from(1);
        contact - domain + Real::from(2).powi_i64(-3000).unwrap()
    }

    #[test]
    fn crossing_planarization_reuses_the_evaluator_nonzero_policy() {
        let context = TriangulationContext::new(hyperlimit::PredicatePolicy::STRICT);
        let evaluator = PredicateEvaluator::new(&context);
        let extent = exact_normal_positive();
        let half = (Real::from(1) / Real::from(2)).unwrap();
        let crossing_x = extent.clone() * &half;
        let points = vec![
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(extent, Real::zero()),
            Point2::new(crossing_x.clone(), Real::from(-1)),
            Point2::new(crossing_x.clone(), Real::from(1)),
        ];
        let constraints = [Constraint::new(0, 1), Constraint::new(2, 3)];

        let planar = planarize_constraints(&evaluator, &points, &constraints)
            .expect("strict policy should construct the certified crossing");
        assert_eq!(planar.points.len(), 5);
        assert_eq!(
            evaluator.cmp(&planar.points[4].x, &crossing_x),
            Ok(Ordering::Equal)
        );
        assert_eq!(planar.points[4].y, Real::zero());
        assert_eq!(planar.constraints.len(), 4);
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
        let evaluator = PredicateEvaluator::new(&context);
        let edge = EdgeKey::new(0, 2);
        let adjacent = adjacent_triangles(&triangles, edge, [0, 1]).unwrap();

        replace_adjacent_edge_in_topology(
            &evaluator,
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
    fn retained_topology_rejects_malformed_local_replacements_before_mutation() {
        fn replacement_error(
            original: &[Triangle],
            point_count: usize,
            indices: &[usize],
            replacement: &[Triangle],
            split: Option<(EdgeKey, usize)>,
        ) -> Error {
            let mut triangles = original.to_vec();
            let mut topology = TriangleTopology::new(&triangles, point_count).unwrap();
            let error = topology
                .replace_region(&mut triangles, indices, replacement, split)
                .unwrap_err();
            assert_eq!(triangles, original);
            error
        }

        let one = [[0, 1, 2]];
        assert_eq!(
            replacement_error(&one, 5, &[], &[[0, 1, 2]], None),
            Error::InvalidInput {
                reason: "topology replacement has mismatched triangle slots"
            }
        );
        assert_eq!(
            replacement_error(&one, 5, &[0], &[], None),
            Error::InvalidInput {
                reason: "topology replacement has mismatched triangle slots"
            }
        );
        assert_eq!(
            replacement_error(&one, 5, &[0, 0], &[[0, 1, 2], [0, 1, 2]], None),
            Error::InvalidInput {
                reason: "topology replacement triangle slots are not unique and sorted"
            }
        );
        assert_eq!(
            replacement_error(&one, 5, &[1], &[[0, 1, 2]], None),
            Error::InvalidInput {
                reason: "topology replacement references an absent triangle"
            }
        );
        assert_eq!(
            replacement_error(&one, 5, &[0], &[[0, 0, 2]], None),
            Error::InvalidInput {
                reason: "topology replacement contains an invalid triangle"
            }
        );
        assert_eq!(
            replacement_error(&one, 3, &[0], &[[0, 1, 3]], None),
            Error::InvalidInput {
                reason: "topology replacement contains an invalid triangle"
            }
        );
        assert_eq!(
            replacement_error(&one, 5, &[0], &[[0, 1, 2]], Some((EdgeKey::new(0, 1), 0)),),
            Error::InvalidInput {
                reason: "topology replacement does not split an edge interior"
            }
        );
        assert_eq!(
            replacement_error(&one, 5, &[0], &[[0, 1, 2]], Some((EdgeKey::new(3, 4), 2)),),
            Error::InvalidInput {
                reason: "split edge has invalid topology incidence"
            }
        );

        let mut triangles = one.to_vec();
        let mut topology = TriangleTopology::new(&triangles, 6).unwrap();
        assert_eq!(
            topology.replace_point_region(&mut triangles, &[0, 1, 2], &[[0, 1, 2]], None,),
            Err(Error::InvalidInput {
                reason: "point insertion exceeds its local topology bound"
            })
        );
        assert_eq!(
            topology.replace_point_region(
                &mut triangles,
                &[0],
                &[[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 5], [0, 5, 1],],
                None,
            ),
            Err(Error::InvalidInput {
                reason: "point insertion exceeds its local topology bound"
            })
        );
    }

    #[test]
    fn retained_topology_detects_corrupted_split_and_boundary_adjacency() {
        let mesh = [[0, 1, 2], [0, 2, 3]];
        let split = EdgeKey::new(0, 2);

        let mut triangles = mesh.to_vec();
        let mut topology = TriangleTopology::new(&triangles, 5).unwrap();
        assert_eq!(
            topology.replace_region(
                &mut triangles,
                &[0],
                &[[0, 1, 4], [1, 2, 4], [2, 0, 4]],
                Some((split, 4)),
            ),
            Err(Error::InvalidInput {
                reason: "split edge omitted an adjacent triangle"
            })
        );

        let mut triangles = mesh.to_vec();
        let mut topology = TriangleTopology::new(&triangles, 5).unwrap();
        topology.neighbors.fill([None; 3]);
        assert_eq!(
            topology.replace_region(
                &mut triangles,
                &[0, 1],
                &[[0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4]],
                Some((split, 4)),
            ),
            Err(Error::InvalidInput {
                reason: "interior split edge remained on the topology boundary"
            })
        );

        let duplicate_mesh = [[0, 1, 2], [0, 1, 2]];
        let mut triangles = duplicate_mesh.to_vec();
        let mut topology = TriangleTopology::new(&triangles, 3).unwrap();
        topology.neighbors.fill([None; 3]);
        assert_eq!(
            topology.replace_region(&mut triangles, &[0, 1], &[[0, 1, 2], [0, 1, 2]], None,),
            Err(Error::InvalidInput {
                reason: "topology replacement boundary contains a duplicate edge"
            })
        );

        for bad_backlink in [None, Some(1)] {
            let mut triangles = mesh.to_vec();
            let mut topology = TriangleTopology::new(&triangles, 4).unwrap();
            let outside_slot = triangle_edge_slot(mesh[1], split).unwrap();
            topology.neighbors[1][outside_slot] = bad_backlink;
            assert_eq!(
                topology.replace_region(&mut triangles, &[0], &[mesh[0]], None),
                Err(Error::InvalidInput {
                    reason: "triangle adjacency is not reciprocal"
                })
            );
        }
    }

    #[test]
    fn retained_topology_rejects_nonmanifold_replacement_and_bad_lookup_slots() {
        let mut triangles = vec![[0, 1, 2]];
        let mut topology = TriangleTopology::new(&triangles, 5).unwrap();
        assert_eq!(
            topology.replace_region(
                &mut triangles,
                &[0],
                &[[0, 1, 2], [0, 1, 3], [0, 1, 4]],
                None,
            ),
            Err(Error::InvalidInput {
                reason: "topology replacement contains a non-manifold edge"
            })
        );

        assert_eq!(
            replacement_local_index(&[1], 3, 1, 0),
            Err(Error::InvalidInput {
                reason: "topology replacement edge has no triangle slot"
            })
        );
        assert_eq!(
            replacement_local_index(&[0], 1, 1, 2),
            Err(Error::InvalidInput {
                reason: "topology replacement edge has no appended slot"
            })
        );

        let topology = TriangleTopology::new(&[[0, 1, 2]], 3).unwrap();
        assert!(
            topology
                .neighbor_across(&[[0, 1, 2]], 9, EdgeKey::new(0, 1))
                .is_err()
        );
        assert!(
            topology
                .neighbor_across(&[[0, 1, 2]], 0, EdgeKey::new(0, 9))
                .is_err()
        );

        assert!(TriangleTopology::new(&[[0, 0, 1]], 2).is_err());
        assert!(TriangleTopology::new(&[[0, 1, 3]], 3).is_err());
        assert!(adjacent_triangles(&[[0, 1, 0]], EdgeKey::new(0, 1), [0, 0]).is_err());
        assert_eq!(
            oriented_triangle(Sign::Zero, [0, 1, 2]),
            Err(Error::InvalidInput {
                reason: "degenerate triangle"
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
        let evaluator = PredicateEvaluator::new(&context);
        assert_eq!(
            push_unique_point(&evaluator, &mut points, Point2::new(right, Real::zero())),
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
            let evaluator = PredicateEvaluator::new(&context);
            let mut triangles = original.clone();
            let mut topology = TriangleTopology::new(&triangles, points.len()).unwrap();
            let mut cavity = Vec::new();
            let mut incident_triangles = Vec::new();
            let mut left_chain = Vec::new();
            let mut right_chain = Vec::new();

            recover_constraint(
                ConstraintRecovery {
                    evaluator: &evaluator,
                    points: &points,
                    triangles: &mut triangles,
                    topology: &mut topology,
                    constrained_edges: &[],
                    approximate_points: approximate.as_deref(),
                    cavity: &mut cavity,
                    incident_triangles: &mut incident_triangles,
                    left_chain: &mut left_chain,
                    right_chain: &mut right_chain,
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
                &evaluator,
                &points,
                &[constraint],
                &triangles,
            )
            .unwrap();
            assert_eq!(
                evaluator.finish(()).certainty,
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
            let evaluator = PredicateEvaluator::new(&context);
            let mut triangles = triangulate_cavity_side(
                &evaluator,
                &points,
                (0..points.len()).collect(),
                Sign::Negative,
            )
            .unwrap();
            for vertex in 0..points.len() {
                if !triangles.iter().any(|triangle| triangle.contains(&vertex)) {
                    crate::cdt::insert_topology_point(
                        &evaluator,
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
                evaluator.finish(()).certainty,
                crate::TriangulationCertainty::Certified
            );
        }
    }

    #[test]
    fn exact_corridor_preserves_a_non_crossed_protected_chord() {
        // The target walks every face in order, but the same-side boundary
        // folds back across protected edge 5--6. A union-boundary cavity sees
        // that chord as internal and used to reject this valid PSLG. Ordered
        // half-hole chains retain the backtracking edge and both incident
        // faces while recovering the target without a fallback algorithm.
        let points = vec![
            p(5209, 6308),
            p(-1052, -12323),
            p(651, 10036),
            p(5378, 6100),
            p(5445, 6017),
            p(7329, 3690),
            p(5502, 5229),
            p(2267, 7287),
            p(5167, 2265),
            p(304, -3637),
            p(-502, -7436),
            p(5228, 676),
            p(3872, -4221),
        ];
        let original = vec![
            [3, 0, 2],
            [4, 3, 2],
            [5, 4, 2],
            [5, 2, 6],
            [6, 2, 7],
            [5, 6, 7],
            [7, 8, 5],
            [7, 9, 8],
            [9, 5, 8],
            [10, 5, 9],
            [5, 10, 11],
            [10, 12, 11],
            [1, 12, 10],
        ];
        let protected = Constraint::new(5, 6);
        let target = Constraint::new(0, 1);
        let protected_edges = [EdgeKey::new(protected.from, protected.to)];
        let approximate = crate::cdt::exact_points_f64(&points);

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = TriangulationContext::new(policy);
            let evaluator = PredicateEvaluator::new(&context);
            let mut triangles = original.clone();
            let mut topology = TriangleTopology::new(&triangles, points.len()).unwrap();
            let mut cavity = Vec::new();
            let mut incident_triangles = Vec::new();
            let mut left_chain = Vec::new();
            let mut right_chain = Vec::new();

            recover_constraint(
                ConstraintRecovery {
                    evaluator: &evaluator,
                    points: &points,
                    triangles: &mut triangles,
                    topology: &mut topology,
                    constrained_edges: &protected_edges,
                    approximate_points: approximate.as_deref(),
                    cavity: &mut cavity,
                    incident_triangles: &mut incident_triangles,
                    left_chain: &mut left_chain,
                    right_chain: &mut right_chain,
                },
                target,
            )
            .unwrap();

            assert_eq!(triangles.len(), original.len());
            assert!(triangulation_has_edge(&triangles, protected_edges[0]));
            assert!(triangulation_has_edge(
                &triangles,
                EdgeKey::new(target.from, target.to)
            ));
            TriangleTopology::new(&triangles, points.len()).unwrap();
            crate::cdt_validate::validate_constrained_topology(
                &evaluator,
                &points,
                &[protected, target],
                &triangles,
            )
            .unwrap();
            assert_eq!(
                evaluator.finish(()).certainty,
                crate::TriangulationCertainty::Certified
            );
        }
    }

    #[test]
    fn exact_corridor_absorbs_an_unprotected_enclosed_face() {
        // The target's dual walk surrounds triangle 6--5--7 without crossing
        // it. The direct half-hole is therefore weak; the local closure path
        // absorbs that unconstrained face and reconstructs one simple cavity.
        let points = vec![
            p(35, 28),
            p(36, 28),
            p(70, 70),
            p(12, 0),
            p(42, 35),
            p(22, 12),
            p(32, 24),
            p(34, 26),
            p(42, 36),
            p(40, 34),
            p(50, 46),
            p(60, 58),
        ];
        let original = vec![
            [2, 3, 5],
            [0, 3, 9],
            [2, 5, 6],
            [5, 4, 7],
            [4, 6, 7],
            [1, 4, 5],
            [2, 6, 8],
            [6, 4, 8],
            [2, 11, 3],
            [10, 9, 3],
            [11, 10, 3],
            [6, 5, 7],
        ];
        let target = Constraint::new(0, 1);
        let approximate = crate::cdt::exact_points_f64(&points);

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = TriangulationContext::new(policy);
            let evaluator = PredicateEvaluator::new(&context);
            let mut triangles = original.clone();
            let mut topology = TriangleTopology::new(&triangles, points.len()).unwrap();
            let mut cavity = Vec::new();
            let mut incident_triangles = Vec::new();
            let mut left_chain = Vec::new();
            let mut right_chain = Vec::new();

            recover_constraint(
                ConstraintRecovery {
                    evaluator: &evaluator,
                    points: &points,
                    triangles: &mut triangles,
                    topology: &mut topology,
                    constrained_edges: &[],
                    approximate_points: approximate.as_deref(),
                    cavity: &mut cavity,
                    incident_triangles: &mut incident_triangles,
                    left_chain: &mut left_chain,
                    right_chain: &mut right_chain,
                },
                target,
            )
            .unwrap();

            assert_eq!(triangles.len(), original.len());
            assert!(triangulation_has_edge(
                &triangles,
                EdgeKey::new(target.from, target.to)
            ));
            TriangleTopology::new(&triangles, points.len()).unwrap();
            crate::cdt_validate::validate_constrained_topology(
                &evaluator,
                &points,
                &[target],
                &triangles,
            )
            .unwrap();
            assert_eq!(
                evaluator.finish(()).certainty,
                crate::TriangulationCertainty::Certified
            );
        }
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
            let evaluator = PredicateEvaluator::new(&context);
            let orient = |triangles: &[[usize; 3]]| {
                triangles
                    .iter()
                    .copied()
                    .map(|triangle| make_oriented(&evaluator, &points, triangle))
                    .collect::<Result<Vec<_>>>()
            };
            let mut first = orient(&first).unwrap();
            let mut second = orient(&second).unwrap();
            let mut retained_first = first.clone();
            let mut retained_second = second.clone();
            let first_topology = TriangleTopology::new(&first, points.len()).unwrap();
            let second_topology = TriangleTopology::new(&second, points.len()).unwrap();

            canonicalize_unconstrained_edges(&evaluator, &points, &mut first, &[], None).unwrap();
            canonicalize_unconstrained_edges(&evaluator, &points, &mut second, &[], None).unwrap();
            canonicalize_unconstrained_edges(
                &evaluator,
                &points,
                &mut retained_first,
                &[],
                Some(first_topology),
            )
            .unwrap();
            canonicalize_unconstrained_edges(
                &evaluator,
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
