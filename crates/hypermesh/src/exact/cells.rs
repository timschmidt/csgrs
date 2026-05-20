//! Exact planar cell triangulation for intersecting source faces.
//!
//! Split-edge loops are not enough for volumetric named booleans: when an
//! opposite face cuts through the interior of a source triangle, the source
//! face must be subdivided by the exact intersection segment before
//! inside/outside winding can decide which pieces survive. This module turns
//! the retained intersection graph into a planar straight-line graph per
//! source face and triangulates it with `hypertri`'s constrained Delaunay
//! implementation. The staging follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): graph events become
//! topology only after exact predicate and construction evidence is retained.
//!
//! The constrained triangulation call uses the constrained-Delaunay criterion
//! of Lee and Lin, "Generalized Delaunay triangulation for planar graphs,"
//! *Discrete & Computational Geometry* 1 (1986), as implemented by `hypertri`;
//! `hypermesh` still validates every emitted triangle against its exact 3D
//! source point before boolean assembly consumes it.

#[cfg(feature = "exact-triangulation")]
use std::{cmp::Ordering, collections::BTreeSet};

#[cfg(feature = "exact-triangulation")]
use hyperlimit::{Point3, TriangleLocation, classify_point_triangle, compare_reals};
#[cfg(feature = "exact-triangulation")]
use hypertri::Constraint;

#[cfg(feature = "exact-triangulation")]
use super::construction::SegmentPlaneRelation;
#[cfg(feature = "exact-triangulation")]
use super::coplanar::CoplanarProjection;
#[cfg(feature = "exact-triangulation")]
use super::graph::{
    ExactFaceRegionPlan, ExactIntersectionGraph, ExactSplitTopologyPlan, FacePairEvents,
    FaceRegionBoundary, FaceSplitBoundaryNode, IntersectionEvent, MeshSide,
};
#[cfg(feature = "exact-triangulation")]
use super::intersection::MeshFacePairRelation;
#[cfg(feature = "exact-triangulation")]
use super::mesh::ExactMesh;
#[cfg(feature = "exact-triangulation")]
use super::region::{
    FaceRegionTriangulation, boundary_node_point, choose_region_projection, project_for_hypertri,
    project_for_predicate,
};
#[cfg(feature = "exact-triangulation")]
use super::scalar::ExactReal;

/// Full-face cell plan used by winding-materialized booleans.
#[cfg(feature = "exact-triangulation")]
pub type ExactFaceCellTriangulationPlan = (ExactFaceRegionPlan, Vec<FaceRegionTriangulation>);

/// Triangulate every source face into exact constrained planar cells.
///
/// The returned region plan stores the exact 3D cell vertices retained for
/// each source face, while each [`FaceRegionTriangulation`] stores the
/// projected CDT triangles over those vertices. Non-coplanar intersection
/// segments are recovered from graph vertices that share a face pair; boundary
/// edges of the original source triangle are always constraints. If `hypertri`
/// introduces exact Steiner vertices while planarizing crossing constraints,
/// each new projected point is lifted back to the original source-face plane
/// and retained as a [`FaceSplitBoundaryNode::FaceInterior`] witness before
/// validation or assembly can consume it.
pub fn triangulate_all_face_cells_with_cdt(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<Option<ExactFaceCellTriangulationPlan>> {
    let topology = graph.split_topology_plan();
    if topology.unresolved_equalities != 0
        || topology.unresolved_vertex_lookups != 0
        || topology.unknown_orderings != 0
        || !topology.validate().is_valid()
    {
        return Ok(None);
    }

    let mut regions = Vec::with_capacity(left.triangles().len() + right.triangles().len());
    let mut triangulations = Vec::with_capacity(left.triangles().len() + right.triangles().len());
    for (side, mesh) in [(MeshSide::Left, left), (MeshSide::Right, right)] {
        for face in 0..mesh.triangles().len() {
            let Some((region, triangulation)) =
                triangulate_one_face_cell_graph(graph, &topology, side, face, mesh, left, right)?
            else {
                return Ok(None);
            };
            regions.push(region);
            triangulations.push(triangulation);
        }
    }

    Ok(Some((ExactFaceRegionPlan { regions }, triangulations)))
}

#[cfg(feature = "exact-triangulation")]
fn triangulate_one_face_cell_graph(
    graph: &ExactIntersectionGraph,
    topology: &ExactSplitTopologyPlan,
    side: MeshSide,
    face: usize,
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<Option<(FaceRegionBoundary, FaceRegionTriangulation)>> {
    let source_triangle = mesh.triangles()[face].0;
    let projection = choose_region_projection(mesh, face)?;
    let mut boundary = Vec::new();
    let mut interior_constraints = Vec::new();
    let mut unique_interior_constraints = BTreeSet::new();

    for &vertex in &source_triangle {
        push_cell_node(
            &mut boundary,
            FaceSplitBoundaryNode::OriginalVertex {
                vertex,
                point: mesh.vertices()[vertex].to_hyperlimit_point(),
            },
        )?;
    }

    for pair in graph
        .face_pairs
        .iter()
        .filter(|pair| pair_involves_face(pair, side, face))
    {
        if pair.relation != MeshFacePairRelation::Candidate || !pair_has_proper_crossing(pair) {
            continue;
        }
        let mut endpoints = Vec::new();
        for (graph_vertex, vertex) in topology.graph_vertices.iter().enumerate() {
            if !graph_vertex_in_face_pair(vertex, pair, side, face) {
                continue;
            }
            if !point_lies_in_face_pair_overlap(&vertex.point, pair, left, right)? {
                continue;
            }
            let node = FaceSplitBoundaryNode::GraphVertex {
                graph_vertex,
                point: vertex.point.clone(),
            };
            let index = push_cell_node(&mut boundary, node)?;
            if !endpoints.contains(&index) {
                endpoints.push(index);
            }
        }
        match endpoints.as_slice() {
            [a, b] if a != b => {
                push_constraint(
                    &mut interior_constraints,
                    &mut unique_interior_constraints,
                    *a,
                    *b,
                );
            }
            [] => {}
            [_] => {
                // A triangle/triangle candidate can contain proper
                // segment-plane constructions while the closed overlap of the
                // two finite triangles is only a point. That point is valid
                // graph evidence, but it does not cut a positive-area
                // source-face cell. Yap's exact-computation boundary lets us
                // discard it here only because the retained overlap incidence
                // was checked exactly; it remains available in the graph for
                // boundary-policy reports.
            }
            _ => {
                return Ok(None);
            }
        }
    }

    let mut vertices = boundary
        .iter()
        .map(|node| project_for_hypertri(boundary_node_point(node), projection))
        .collect::<Vec<_>>();
    let interior_constraints =
        positive_area_interior_constraints(&vertices, &interior_constraints)?;
    let mut constraints = Vec::new();
    let mut unique_constraints = BTreeSet::new();
    if !interior_constraints.is_empty() {
        append_subdivided_source_boundary_constraints(
            &vertices,
            &mut constraints,
            &mut unique_constraints,
        )?;
        for constraint in &interior_constraints {
            push_constraint(
                &mut constraints,
                &mut unique_constraints,
                constraint.from,
                constraint.to,
            );
        }
    }

    // Lee and Lin's constrained-Delaunay lemma is consumed through
    // `hypertri::cdt::constrained_delaunay`; every input point and constraint
    // above is exact graph/source evidence. `hypertri` may append exact
    // Steiner points when constraints cross; in Yap's object/predicate split
    // those points become usable topology only after we retain an exact 3D
    // witness and replay its source-face incidence.
    let cdt = hypertri::cdt::constrained_delaunay(&vertices, &constraints)?;
    if cdt.points().len() < vertices.len() {
        return Err(hypertri::Error::InvalidInput {
            reason: "face-cell CDT dropped an input vertex",
        });
    }
    if cdt.points().len() > vertices.len() {
        for point in &cdt.points()[vertices.len()..] {
            let lifted = lift_projected_face_cell_point(mesh, face, projection, point)?;
            boundary.push(FaceSplitBoundaryNode::FaceInterior { point: lifted });
        }
        vertices = cdt.points().to_vec();
    }
    if boundary.len() != vertices.len() {
        return Err(hypertri::Error::InvalidInput {
            reason: "face-cell CDT point and source witness counts differ",
        });
    }
    let planarized_interior_constraints =
        planarized_interior_constraints(cdt.constraint_edges(), &vertices, &interior_constraints)?;
    let mut triangles = cdt
        .triangles()
        .iter()
        .flat_map(|triangle| triangle.iter().copied())
        .collect::<Vec<_>>();
    append_closed_constraint_loop_triangles(
        &vertices,
        &planarized_interior_constraints,
        &mut triangles,
    )?;
    let triangulation = FaceRegionTriangulation {
        side,
        face,
        projection,
        boundary: boundary.clone(),
        vertices,
        triangles,
    };
    triangulation.validate()?;
    Ok(Some((
        FaceRegionBoundary {
            side,
            face,
            triangle: source_triangle,
            boundary,
        },
        triangulation,
    )))
}

#[cfg(feature = "exact-triangulation")]
fn append_closed_constraint_loop_triangles(
    vertices: &[hypertri::ExactPoint],
    constraints: &[Constraint],
    triangles: &mut Vec<usize>,
) -> hypertri::Result<()> {
    let mut adjacency = vec![Vec::<usize>::new(); vertices.len()];
    for constraint in constraints {
        adjacency[constraint.from].push(constraint.to);
        adjacency[constraint.to].push(constraint.from);
    }

    let mut seen = vec![false; vertices.len()];
    for start in 0..vertices.len() {
        if adjacency[start].is_empty() || seen[start] {
            continue;
        }
        let mut stack = vec![start];
        let mut component = Vec::new();
        seen[start] = true;
        while let Some(vertex) = stack.pop() {
            component.push(vertex);
            for &next in &adjacency[vertex] {
                if !seen[next] {
                    seen[next] = true;
                    stack.push(next);
                }
            }
        }
        if component.len() < 3 || component.iter().any(|&vertex| adjacency[vertex].len() != 2) {
            continue;
        }

        let ordered = order_simple_cycle(&adjacency, component[0])?;
        let loop_vertices = ordered
            .iter()
            .map(|&index| vertices[index].clone())
            .collect::<Vec<_>>();
        // Closed constraint loops are the exact planar cells that `hypertri`'s
        // polygon dispatch treats as holes when an exterior boundary ring is
        // present. Triangulating those loop interiors explicitly preserves both
        // sides of the arrangement for winding policy. Earcut is used only
        // after the loop graph has been certified as a simple degree-two cycle;
        // this follows Yap's rule that topology changes consume exact
        // combinatorial facts, and the loop triangulation itself is still
        // predicate-validated by `FaceRegionTriangulation::validate`.
        let loop_triangles = hypertri::earcut(&loop_vertices, &[])?;
        for triangle in loop_triangles.chunks_exact(3) {
            triangles.extend([
                ordered[triangle[0]],
                ordered[triangle[1]],
                ordered[triangle[2]],
            ]);
        }
    }
    Ok(())
}

#[cfg(feature = "exact-triangulation")]
fn lift_projected_face_cell_point(
    mesh: &ExactMesh,
    face: usize,
    projection: CoplanarProjection,
    point: &hypertri::ExactPoint,
) -> hypertri::Result<Point3> {
    let triangle = mesh.triangles()[face].0;
    let a = mesh.vertices()[triangle[0]].to_hyperlimit_point();
    let b = mesh.vertices()[triangle[1]].to_hyperlimit_point();
    let c = mesh.vertices()[triangle[2]].to_hyperlimit_point();
    let ab = point3_sub(&b, &a);
    let ac = point3_sub(&c, &a);
    let normal = cross(&ab, &ac);
    let plane_value = add_real(
        &add_real(&mul_real(&normal.x, &a.x), &mul_real(&normal.y, &a.y)),
        &mul_real(&normal.z, &a.z),
    );

    // The projection was selected by an exact nonzero projected area, so the
    // dropped normal component is the denominator that recovers the omitted
    // coordinate on the source plane. This is the same retained-construction
    // discipline as Yap's exact computation model: the planar Steiner vertex
    // is not accepted until the 3D lift reprojects exactly.
    let lifted = match projection {
        CoplanarProjection::Xy => {
            let x = point.x.clone();
            let y = point.y.clone();
            let numerator = sub_real(
                &sub_real(&plane_value, &mul_real(&normal.x, &x)),
                &mul_real(&normal.y, &y),
            );
            let z = div_real(
                numerator,
                &normal.z,
                "face-cell XY Steiner lift has zero normal denominator",
            )?;
            Point3::new(x, y, z)
        }
        CoplanarProjection::Xz => {
            let x = point.x.clone();
            let z = point.y.clone();
            let numerator = sub_real(
                &sub_real(&plane_value, &mul_real(&normal.x, &x)),
                &mul_real(&normal.z, &z),
            );
            let y = div_real(
                numerator,
                &normal.y,
                "face-cell XZ Steiner lift has zero normal denominator",
            )?;
            Point3::new(x, y, z)
        }
        CoplanarProjection::Yz => {
            let y = point.x.clone();
            let z = point.y.clone();
            let numerator = sub_real(
                &sub_real(&plane_value, &mul_real(&normal.y, &y)),
                &mul_real(&normal.z, &z),
            );
            let x = div_real(
                numerator,
                &normal.x,
                "face-cell YZ Steiner lift has zero normal denominator",
            )?;
            Point3::new(x, y, z)
        }
    };

    let replay = project_for_hypertri(&lifted, projection);
    match exact_points_equal(&replay, point)? {
        true => Ok(lifted),
        false => Err(hypertri::Error::InvalidInput {
            reason: "face-cell Steiner lift does not replay to the projected CDT point",
        }),
    }
}

#[cfg(feature = "exact-triangulation")]
fn positive_area_interior_constraints(
    vertices: &[hypertri::ExactPoint],
    constraints: &[Constraint],
) -> hypertri::Result<Vec<Constraint>> {
    let mut retained = Vec::new();
    let mut unique = BTreeSet::new();
    for &constraint in constraints {
        if constraint_lies_on_source_boundary(vertices, constraint)? {
            continue;
        }
        push_constraint(&mut retained, &mut unique, constraint.from, constraint.to);
    }
    Ok(retained)
}

#[cfg(feature = "exact-triangulation")]
fn constraint_lies_on_source_boundary(
    vertices: &[hypertri::ExactPoint],
    constraint: Constraint,
) -> hypertri::Result<bool> {
    for boundary in [
        Constraint::new(0, 1),
        Constraint::new(1, 2),
        Constraint::new(2, 0),
    ] {
        if edge_is_subsegment_of_constraint(constraint, vertices, boundary)? {
            return Ok(true);
        }
    }
    Ok(false)
}

#[cfg(feature = "exact-triangulation")]
fn planarized_interior_constraints(
    constraint_edges: &[Constraint],
    vertices: &[hypertri::ExactPoint],
    interior_constraints: &[Constraint],
) -> hypertri::Result<Vec<Constraint>> {
    let mut constraints = Vec::new();
    let mut unique = BTreeSet::new();
    for &edge in constraint_edges {
        if constraint_lies_on_any(edge, vertices, interior_constraints)? {
            push_constraint(&mut constraints, &mut unique, edge.from, edge.to);
        }
    }
    Ok(constraints)
}

#[cfg(feature = "exact-triangulation")]
fn constraint_lies_on_any(
    edge: Constraint,
    vertices: &[hypertri::ExactPoint],
    constraints: &[Constraint],
) -> hypertri::Result<bool> {
    for &constraint in constraints {
        if edge_is_subsegment_of_constraint(edge, vertices, constraint)? {
            return Ok(true);
        }
    }
    Ok(false)
}

#[cfg(feature = "exact-triangulation")]
fn edge_is_subsegment_of_constraint(
    edge: Constraint,
    vertices: &[hypertri::ExactPoint],
    constraint: Constraint,
) -> hypertri::Result<bool> {
    let start = vertices
        .get(constraint.from)
        .ok_or(hypertri::Error::InvalidInput {
            reason: "face-cell interior constraint start is out of range",
        })?;
    let end = vertices
        .get(constraint.to)
        .ok_or(hypertri::Error::InvalidInput {
            reason: "face-cell interior constraint end is out of range",
        })?;
    let edge_start = vertices
        .get(edge.from)
        .ok_or(hypertri::Error::InvalidInput {
            reason: "face-cell planarized constraint start is out of range",
        })?;
    let edge_end = vertices.get(edge.to).ok_or(hypertri::Error::InvalidInput {
        reason: "face-cell planarized constraint end is out of range",
    })?;
    Ok(point_on_closed_segment(edge_start, start, end)?
        && point_on_closed_segment(edge_end, start, end)?)
}

#[cfg(feature = "exact-triangulation")]
fn point_on_closed_segment(
    point: &hypertri::ExactPoint,
    start: &hypertri::ExactPoint,
    end: &hypertri::ExactPoint,
) -> hypertri::Result<bool> {
    if compare_ordering(
        &orient2d_value(start, end, point),
        &ExactReal::from(0),
        "face-cell constraint collinearity",
    )? != Ordering::Equal
    {
        return Ok(false);
    }
    Ok(
        real_between_closed(&point.x, &start.x, &end.x, "face-cell constraint x range")?
            && real_between_closed(&point.y, &start.y, &end.y, "face-cell constraint y range")?,
    )
}

#[cfg(feature = "exact-triangulation")]
fn real_between_closed(
    value: &ExactReal,
    a: &ExactReal,
    b: &ExactReal,
    predicate: &'static str,
) -> hypertri::Result<bool> {
    let value_vs_a = compare_ordering(value, a, predicate)?;
    let value_vs_b = compare_ordering(value, b, predicate)?;
    let a_vs_b = compare_ordering(a, b, predicate)?;
    Ok(match a_vs_b {
        Ordering::Less | Ordering::Equal => {
            value_vs_a != Ordering::Less && value_vs_b != Ordering::Greater
        }
        Ordering::Greater => value_vs_b != Ordering::Less && value_vs_a != Ordering::Greater,
    })
}

#[cfg(feature = "exact-triangulation")]
fn exact_points_equal(
    left: &hypertri::ExactPoint,
    right: &hypertri::ExactPoint,
) -> hypertri::Result<bool> {
    let x = compare_ordering(&left.x, &right.x, "face-cell projected x equality")?;
    let y = compare_ordering(&left.y, &right.y, "face-cell projected y equality")?;
    Ok(x == Ordering::Equal && y == Ordering::Equal)
}

#[cfg(feature = "exact-triangulation")]
fn compare_ordering(
    left: &ExactReal,
    right: &ExactReal,
    predicate: &'static str,
) -> hypertri::Result<Ordering> {
    compare_reals(left, right)
        .value()
        .ok_or(hypertri::Error::PredicateUndecided { predicate })
}

#[cfg(feature = "exact-triangulation")]
fn orient2d_value(
    a: &hypertri::ExactPoint,
    b: &hypertri::ExactPoint,
    c: &hypertri::ExactPoint,
) -> ExactReal {
    let bax = sub_real(&b.x, &a.x);
    let bay = sub_real(&b.y, &a.y);
    let cax = sub_real(&c.x, &a.x);
    let cay = sub_real(&c.y, &a.y);
    sub_real(&mul_real(&bax, &cay), &mul_real(&bay, &cax))
}

#[cfg(feature = "exact-triangulation")]
fn point3_sub(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        sub_real(&left.x, &right.x),
        sub_real(&left.y, &right.y),
        sub_real(&left.z, &right.z),
    )
}

#[cfg(feature = "exact-triangulation")]
fn cross(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        sub_real(&mul_real(&left.y, &right.z), &mul_real(&left.z, &right.y)),
        sub_real(&mul_real(&left.z, &right.x), &mul_real(&left.x, &right.z)),
        sub_real(&mul_real(&left.x, &right.y), &mul_real(&left.y, &right.x)),
    )
}

#[cfg(feature = "exact-triangulation")]
fn add_real(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() + right
}

#[cfg(feature = "exact-triangulation")]
fn sub_real(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() - right
}

#[cfg(feature = "exact-triangulation")]
fn mul_real(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() * right
}

#[cfg(feature = "exact-triangulation")]
fn div_real(
    numerator: ExactReal,
    denominator: &ExactReal,
    reason: &'static str,
) -> hypertri::Result<ExactReal> {
    (numerator / denominator).map_err(|_| hypertri::Error::InvalidInput { reason })
}

#[cfg(feature = "exact-triangulation")]
fn order_simple_cycle(adjacency: &[Vec<usize>], start: usize) -> hypertri::Result<Vec<usize>> {
    let mut ordered = vec![start];
    let mut previous = start;
    let mut current = adjacency[start][0];
    while current != start {
        if ordered.len() > adjacency.len() {
            return Err(hypertri::Error::InvalidInput {
                reason: "closed face-cell constraint loop did not terminate",
            });
        }
        ordered.push(current);
        let neighbors = &adjacency[current];
        if neighbors.len() != 2 {
            return Err(hypertri::Error::InvalidInput {
                reason: "closed face-cell constraint loop is not degree two",
            });
        }
        let next = if neighbors[0] == previous {
            neighbors[1]
        } else {
            neighbors[0]
        };
        previous = current;
        current = next;
    }
    Ok(ordered)
}

#[cfg(feature = "exact-triangulation")]
fn push_cell_node(
    nodes: &mut Vec<FaceSplitBoundaryNode>,
    node: FaceSplitBoundaryNode,
) -> hypertri::Result<usize> {
    for (index, existing) in nodes.iter().enumerate() {
        match points_equal(boundary_node_point(existing), boundary_node_point(&node)) {
            Some(true) => return Ok(index),
            Some(false) => {}
            None => {
                return Err(hypertri::Error::PredicateUndecided {
                    predicate: "face_cell_vertex_equality",
                });
            }
        }
    }
    nodes.push(node);
    Ok(nodes.len() - 1)
}

#[cfg(feature = "exact-triangulation")]
fn push_constraint(
    constraints: &mut Vec<Constraint>,
    unique: &mut BTreeSet<(usize, usize)>,
    a: usize,
    b: usize,
) {
    if a == b {
        return;
    }
    let key = if a < b { (a, b) } else { (b, a) };
    if unique.insert(key) {
        constraints.push(Constraint::new(a, b));
    }
}

#[cfg(feature = "exact-triangulation")]
/// Add exact source-triangle boundary constraints after retained split points
/// have been inserted.
///
/// A graph vertex may lie on an original source edge. Passing both the
/// unsplit edge and its retained subsegments to CDT makes the constraint graph
/// inconsistent, because the triangulation can only contain the subsegments
/// once the intermediate point is part of the object. The boundary is
/// therefore rebuilt by sorting all exact-on-edge nodes with certified
/// parameter comparisons. This is the same object/predicate separation Yap
/// describes in "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): the combinatorial constraint list is derived only
/// after exact incidence and ordering facts are available.
fn append_subdivided_source_boundary_constraints(
    vertices: &[hypertri::ExactPoint],
    constraints: &mut Vec<Constraint>,
    unique: &mut BTreeSet<(usize, usize)>,
) -> hypertri::Result<()> {
    for [start, end] in [[0, 1], [1, 2], [2, 0]] {
        let mut chain = Vec::new();
        for (index, point) in vertices.iter().enumerate() {
            if point_on_closed_segment(point, &vertices[start], &vertices[end])? {
                let parameter = segment_parameter(point, &vertices[start], &vertices[end])?;
                chain.push((index, parameter));
            }
        }
        sort_boundary_chain(&mut chain)?;
        for pair in chain.windows(2) {
            push_constraint(constraints, unique, pair[0].0, pair[1].0);
        }
    }
    Ok(())
}

#[cfg(feature = "exact-triangulation")]
fn sort_boundary_chain(chain: &mut Vec<(usize, ExactReal)>) -> hypertri::Result<()> {
    let mut ordered = Vec::<(usize, ExactReal)>::with_capacity(chain.len());
    for candidate in chain.drain(..) {
        let mut insert_at = ordered.len();
        for (index, (_, parameter)) in ordered.iter().enumerate() {
            match compare_ordering(
                &candidate.1,
                parameter,
                "face-cell boundary parameter ordering",
            )? {
                Ordering::Less => {
                    insert_at = index;
                    break;
                }
                Ordering::Equal | Ordering::Greater => {}
            }
        }
        ordered.insert(insert_at, candidate);
    }
    *chain = ordered;
    Ok(())
}

#[cfg(feature = "exact-triangulation")]
fn segment_parameter(
    point: &hypertri::ExactPoint,
    start: &hypertri::ExactPoint,
    end: &hypertri::ExactPoint,
) -> hypertri::Result<ExactReal> {
    let dx = sub_real(&end.x, &start.x);
    if compare_ordering(&dx, &ExactReal::from(0), "face-cell boundary dx")? != Ordering::Equal {
        return div_real(
            sub_real(&point.x, &start.x),
            &dx,
            "face-cell boundary x parameter denominator is zero",
        );
    }
    let dy = sub_real(&end.y, &start.y);
    if compare_ordering(&dy, &ExactReal::from(0), "face-cell boundary dy")? != Ordering::Equal {
        return div_real(
            sub_real(&point.y, &start.y),
            &dy,
            "face-cell boundary y parameter denominator is zero",
        );
    }
    Err(hypertri::Error::InvalidInput {
        reason: "face-cell source boundary has duplicate projected endpoints",
    })
}

#[cfg(feature = "exact-triangulation")]
fn pair_involves_face(pair: &FacePairEvents, side: MeshSide, face: usize) -> bool {
    match side {
        MeshSide::Left => pair.left_face == face,
        MeshSide::Right => pair.right_face == face,
    }
}

#[cfg(feature = "exact-triangulation")]
fn pair_has_proper_crossing(pair: &FacePairEvents) -> bool {
    // Contact-only candidate pairs can occur next to coplanar source-face
    // overlaps. They are valid graph evidence, but they do not cut a positive
    // area source-face cell. Following Yap, "Towards Exact Geometric
    // Computation," Comput. Geom. 7.1-2 (1997), only retained proper
    // segment/plane constructions become topology constraints here; endpoint
    // and coplanar contacts stay explicit graph facts for boundary policy.
    pair.events.iter().any(|event| {
        matches!(
            event,
            IntersectionEvent::SegmentPlane {
                relation: SegmentPlaneRelation::ProperCrossing,
                ..
            }
        )
    })
}

#[cfg(feature = "exact-triangulation")]
fn graph_vertex_in_face_pair(
    vertex: &super::graph::ExactGraphVertex,
    pair: &FacePairEvents,
    side: MeshSide,
    face: usize,
) -> bool {
    vertex.uses.iter().any(|source_use| {
        source_use.face_pair == [pair.left_face, pair.right_face]
            && match side {
                MeshSide::Left => source_use.face_pair[0] == face,
                MeshSide::Right => source_use.face_pair[1] == face,
            }
    })
}

#[cfg(feature = "exact-triangulation")]
fn point_lies_in_face_pair_overlap(
    point: &Point3,
    pair: &FacePairEvents,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<bool> {
    let left_location = classify_point_on_mesh_face(left, pair.left_face, point)?;
    let right_location = classify_point_on_mesh_face(right, pair.right_face, point)?;
    Ok(point_triangle_location_is_closed(left_location)
        && point_triangle_location_is_closed(right_location))
}

#[cfg(feature = "exact-triangulation")]
fn classify_point_on_mesh_face(
    mesh: &ExactMesh,
    face: usize,
    point: &Point3,
) -> hypertri::Result<TriangleLocation> {
    let projection = choose_region_projection(mesh, face)?;
    let triangle = mesh.triangles()[face].0;
    let vertices = [
        mesh.vertices()[triangle[0]].to_hyperlimit_point(),
        mesh.vertices()[triangle[1]].to_hyperlimit_point(),
        mesh.vertices()[triangle[2]].to_hyperlimit_point(),
    ];
    classify_point_triangle(
        &project_for_predicate(&vertices[0], projection),
        &project_for_predicate(&vertices[1], projection),
        &project_for_predicate(&vertices[2], projection),
        &project_for_predicate(point, projection),
    )
    .value()
    .ok_or(hypertri::Error::PredicateUndecided {
        predicate: "face_cell_pair_endpoint_containment",
    })
}

#[cfg(feature = "exact-triangulation")]
const fn point_triangle_location_is_closed(location: TriangleLocation) -> bool {
    matches!(
        location,
        TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
    )
}

#[cfg(feature = "exact-triangulation")]
fn points_equal(left: &Point3, right: &Point3) -> Option<bool> {
    Some(
        compare_reals(&left.x, &right.x).value()? == std::cmp::Ordering::Equal
            && compare_reals(&left.y, &right.y).value()? == std::cmp::Ordering::Equal
            && compare_reals(&left.z, &right.z).value()? == std::cmp::Ordering::Equal,
    )
}

#[cfg(all(test, feature = "exact-triangulation"))]
mod tests {
    use super::super::validation::ValidationPolicy;
    use super::*;

    fn open_triangle_mesh(pos: &[i64]) -> ExactMesh {
        ExactMesh::from_i64_triangles_with_policy(pos, &[0, 1, 2], ValidationPolicy::ALLOW_BOUNDARY)
            .unwrap()
    }

    fn ep(x: i64, y: i64) -> hypertri::ExactPoint {
        hypertri::ExactPoint::new(ExactReal::from(x), ExactReal::from(y))
    }

    fn assert_point(point: &Point3, x: i64, y: i64, z: i64) {
        assert_eq!(
            compare_reals(&point.x, &ExactReal::from(x)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(&point.y, &ExactReal::from(y)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(&point.z, &ExactReal::from(z)).value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn lifted_face_cell_steiner_points_replay_for_all_projections() {
        let xy = open_triangle_mesh(&[0, 0, 1, 4, 0, 1, 0, 4, 5]);
        let lifted_xy =
            lift_projected_face_cell_point(&xy, 0, CoplanarProjection::Xy, &ep(1, 1)).unwrap();
        assert_point(&lifted_xy, 1, 1, 2);

        let xz = open_triangle_mesh(&[0, 1, 0, 4, 1, 0, 0, 5, 4]);
        let lifted_xz =
            lift_projected_face_cell_point(&xz, 0, CoplanarProjection::Xz, &ep(1, 1)).unwrap();
        assert_point(&lifted_xz, 1, 2, 1);

        let yz = open_triangle_mesh(&[1, 0, 0, 1, 4, 0, 5, 0, 4]);
        let lifted_yz =
            lift_projected_face_cell_point(&yz, 0, CoplanarProjection::Yz, &ep(1, 1)).unwrap();
        assert_point(&lifted_yz, 2, 1, 1);
    }

    #[test]
    fn planarized_interior_constraints_keep_cdt_steiner_subsegments() {
        let vertices = vec![ep(0, 0), ep(2, 2), ep(0, 2), ep(2, 0)];
        let constraints = vec![Constraint::new(0, 1), Constraint::new(2, 3)];
        let cdt = hypertri::cdt::constrained_delaunay(&vertices, &constraints).unwrap();

        assert_eq!(cdt.points().len(), 5);
        assert_eq!(cdt.points()[4], ep(1, 1));
        assert_eq!(
            planarized_interior_constraints(cdt.constraint_edges(), cdt.points(), &constraints)
                .unwrap(),
            vec![
                Constraint::new(0, 4),
                Constraint::new(4, 1),
                Constraint::new(2, 4),
                Constraint::new(4, 3),
            ]
        );
    }
}
