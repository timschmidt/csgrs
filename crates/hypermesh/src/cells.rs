//! Exact planar cell triangulation for intersecting source faces.
//!
//! Split-edge loops are not enough for volumetric named booleans: when an
//! opposite face cuts through the interior of a source triangle, the source
//! face must be subdivided by the exact intersection segment before
//! inside/outside winding can decide which pieces survive. This module turns
//! the retained intersection graph into a planar straight-line graph per
//! source face and triangulates it with `hypertri`'s constrained Delaunay
//! topology only after exact predicate and construction evidence is retained.
//!
//! The constrained triangulation call uses the constrained-Delaunay criterion
//! of Lee and Lin, "Generalized Delaunay Triangulation for Planar Graphs,"
//! *Discrete & Computational Geometry*. `hypermesh` still validates every
//! emitted triangle against its exact 3D source point before boolean assembly
//! consumes it.

use std::{cmp::Ordering, collections::BTreeSet};

use hyperlimit::{
    Point2, Point3, SegmentIntersection, SegmentPlaneRelation, TriangleLocation,
    classify_point_triangle, classify_segment_intersection, compare_reals, point_on_segment,
    projected_segment_parameter3, proper_segment_intersection_point,
};
use hypertri::Constraint;

use super::graph::{
    CoplanarOverlapSplitPlan, ExactFaceRegionPlan, ExactIntersectionGraph, ExactSplitTopologyPlan,
    FacePairEvents, FaceRegionBoundary, FaceSplitBoundaryNode, IntersectionEvent, MeshSide,
};
use super::intersection::MeshFacePairRelation;
use super::mesh::ExactMesh;
use super::region::{
    FaceRegionTriangulation, boundary_node_point, choose_region_projection, project_for_hypertri,
    project_for_predicate,
};
use super::topology::triangle_edges;
use hyperlimit::CoplanarProjection;
use hyperreal::Real;

/// Candidate constraint edge from the opposite coplanar triangle boundary.
///
/// These records are local to one source face. They gather exact split points
/// and contained opposite vertices along an opposite-face edge before that edge
/// is emitted as CDT constraints.
#[derive(Clone, Debug)]
struct CoplanarCellEdge {
    edge: [usize; 2],
    points: Vec<CoplanarCellEdgePoint>,
}

/// Exact point on a [`CoplanarCellEdge`] with its edge parameter.
#[derive(Clone, Debug)]
struct CoplanarCellEdgePoint {
    parameter: Real,
    point: Point3,
}

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
) -> hypertri::Result<Option<(ExactFaceRegionPlan, Vec<FaceRegionTriangulation>)>> {
    let topology = graph.split_topology_plan();
    if topology.unresolved_equalities != 0
        || topology.unresolved_vertex_lookups != 0
        || topology.unknown_orderings != 0
        || !topology.validate().is_valid()
    {
        return Ok(None);
    }
    let coplanar_splits = graph
        .coplanar_overlap_split_plan(left, right)
        .map_err(|_| hypertri::Error::InvalidInput {
            reason: "face-cell coplanar overlap split construction failed",
        })?;

    let mut regions = Vec::with_capacity(left.triangles().len() + right.triangles().len());
    let mut triangulations = Vec::with_capacity(left.triangles().len() + right.triangles().len());
    for (side, mesh) in [(MeshSide::Left, left), (MeshSide::Right, right)] {
        for face in 0..mesh.triangles().len() {
            let Some((region, triangulation)) = triangulate_one_face_cell_graph(
                graph,
                &topology,
                &coplanar_splits,
                side,
                face,
                mesh,
                left,
                right,
            )?
            else {
                return Ok(None);
            };
            regions.push(region);
            triangulations.push(triangulation);
        }
    }

    Ok(Some((ExactFaceRegionPlan { regions }, triangulations)))
}

/// Validate a retained constrained face-cell triangulation by source replay.
///
/// [`ExactFaceRegionPlan::validate_against_sources`] replays the simpler
/// split-region boundary plan. CDT face cells intentionally add retained
/// interior constraints and possible exact Steiner vertices, so their
/// provenance must replay through [`triangulate_all_face_cells_with_cdt`]
/// instead of the basic boundary-loop path.
pub fn validate_face_cell_cdt_against_sources(
    regions: &ExactFaceRegionPlan,
    triangulations: &[FaceRegionTriangulation],
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<()> {
    if !regions.validate(left, right).is_valid() {
        return Err(hypertri::Error::InvalidInput {
            reason: "face-cell CDT region plan failed exact validation",
        });
    }
    if triangulations.len() != regions.regions.len() {
        return Err(hypertri::Error::InvalidInput {
            reason: "face-cell CDT triangulation count does not match region plan",
        });
    }
    for triangulation in triangulations {
        triangulation.validate()?;
    }

    let graph = super::graph::build_intersection_graph(left, right).map_err(|_| {
        hypertri::Error::InvalidInput {
            reason: "face-cell CDT source replay could not rebuild intersection graph",
        }
    })?;
    let replay = triangulate_all_face_cells_with_cdt(&graph, left, right)?.ok_or(
        hypertri::Error::InvalidInput {
            reason: "face-cell CDT source replay did not materialize",
        },
    )?;
    if replay.0 == *regions && replay.1 == triangulations {
        Ok(())
    } else {
        Err(hypertri::Error::InvalidInput {
            reason: "face-cell CDT source replay mismatch",
        })
    }
}

fn triangulate_one_face_cell_graph(
    graph: &ExactIntersectionGraph,
    topology: &ExactSplitTopologyPlan,
    coplanar_splits: &CoplanarOverlapSplitPlan,
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
                point: mesh.vertices()[vertex].clone(),
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
                // discard it here only because the retained overlap incidence
                // was checked exactly; it remains available in the graph for
                // boundary-policy reports.
            }
            _ => {
                return Ok(None);
            }
        }
    }
    append_coplanar_face_cell_constraints(
        coplanar_splits,
        side,
        face,
        left,
        right,
        &mut boundary,
        &mut interior_constraints,
        &mut unique_interior_constraints,
    )?;

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
    // Lee-Lin constrained-Delaunay triangulation is provided by
    // `hypertri::cdt::constrained_delaunay`; every input point and constraint
    // above is exact graph/source evidence. Any appended exact Steiner points
    // become usable topology only after we retain an exact 3D witness and
    // replay its source-face incidence.
    let (mut triangles, planarized_interior_constraints) =
        match hypertri::cdt::constrained_delaunay(&vertices, &constraints) {
            Ok(cdt) => {
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
                let planarized = planarized_interior_constraints(
                    cdt.constraint_edges(),
                    &vertices,
                    &interior_constraints,
                )?;
                let triangles = cdt
                    .triangles()
                    .iter()
                    .flat_map(|triangle| triangle.iter().copied())
                    .collect::<Vec<_>>();
                (triangles, planarized)
            }
            Err(error) => {
                if let Some(triangles) = triangulate_source_triangle_with_closed_constraint_loops(
                    &vertices,
                    &interior_constraints,
                )? {
                    (triangles, Vec::new())
                } else if let Some(triangles) =
                    triangulate_source_triangle_with_collinear_constraint_refinement(
                        &vertices,
                        &constraints,
                    )?
                {
                    (triangles, interior_constraints.clone())
                } else {
                    return Err(error);
                }
            }
        };
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

#[allow(clippy::too_many_arguments)]
/// Append exact constraints induced by coplanar source-face overlaps.
///
/// A non-coplanar face-cell graph only needs proper segment/plane crossings.
/// Coplanar volumetric overlaps also need the opposite coplanar triangle's
/// boundary clipped into the current source face; otherwise assembly can see
/// two unsplit copies of a partial shared patch. The input facts come from
/// [`ExactIntersectionGraph::coplanar_overlap_split_plan`], whose edge
/// crossings, collinear intervals, and vertex-containment facts follow the
/// topology only after exact parameters are sorted and replayed as local CDT
/// constraints on the source face.
fn append_coplanar_face_cell_constraints(
    split_plan: &CoplanarOverlapSplitPlan,
    side: MeshSide,
    face: usize,
    left: &ExactMesh,
    right: &ExactMesh,
    boundary: &mut Vec<FaceSplitBoundaryNode>,
    constraints: &mut Vec<Constraint>,
    unique_constraints: &mut BTreeSet<(usize, usize)>,
) -> hypertri::Result<()> {
    for graph in &split_plan.graphs {
        if !coplanar_split_graph_involves_face(graph.left_face, graph.right_face, side, face) {
            continue;
        }
        let mut edges =
            coplanar_opposite_edges(graph.left_face, graph.right_face, side, left, right)?;
        seed_contained_coplanar_opposite_edge_endpoints(side, face, left, right, &mut edges)?;
        seed_source_boundary_vertices_on_coplanar_opposite_edges(
            side, face, left, right, &mut edges,
        )?;
        seed_source_boundary_edge_crossings_on_coplanar_opposite_edges(
            side, face, left, right, &mut edges,
        )?;
        for split in &graph.edge_splits {
            match side {
                MeshSide::Left => {
                    let edge = split.overlap.right_edge;
                    for point in &split.points {
                        push_coplanar_cell_edge_point(
                            &mut edges,
                            edge,
                            point.right_parameter.clone(),
                            point.point.clone(),
                        )?;
                    }
                    if let Some(interval) = &split.interval {
                        for point in &interval.endpoints {
                            push_coplanar_cell_edge_point(
                                &mut edges,
                                edge,
                                point.right_parameter.clone(),
                                point.point.clone(),
                            )?;
                        }
                    }
                }
                MeshSide::Right => {
                    let edge = split.overlap.left_edge;
                    for point in &split.points {
                        push_coplanar_cell_edge_point(
                            &mut edges,
                            edge,
                            point.left_parameter.clone(),
                            point.point.clone(),
                        )?;
                    }
                    if let Some(interval) = &split.interval {
                        for point in &interval.endpoints {
                            push_coplanar_cell_edge_point(
                                &mut edges,
                                edge,
                                point.left_parameter.clone(),
                                point.point.clone(),
                            )?;
                        }
                    }
                }
            }
        }
        for vertex_overlap in &graph.vertex_overlaps {
            if vertex_overlap.triangle_side != side || vertex_overlap.triangle_face != face {
                continue;
            }
            let point = vertex_point_for_side(
                vertex_overlap.vertex_side,
                vertex_overlap.vertex,
                left,
                right,
            )?;
            for edge in coplanar_edges_incident_to_vertex(&edges, vertex_overlap.vertex) {
                let parameter = if edge[0] == vertex_overlap.vertex {
                    Real::from(0)
                } else {
                    Real::from(1)
                };
                push_coplanar_cell_edge_point(&mut edges, edge, parameter, point.clone())?;
            }
        }

        for mut edge in edges {
            sort_coplanar_cell_edge_points(&mut edge.points)?;
            dedup_coplanar_cell_edge_points(&mut edge.points)?;
            for pair in edge.points.windows(2) {
                if compare_ordering(
                    &pair[0].parameter,
                    &pair[1].parameter,
                    "face-cell coplanar edge parameter order",
                )? != Ordering::Less
                {
                    continue;
                }
                if points_equal(&pair[0].point, &pair[1].point) != Some(false) {
                    continue;
                }
                let from = push_cell_node(
                    boundary,
                    FaceSplitBoundaryNode::FaceInterior {
                        point: pair[0].point.clone(),
                    },
                )?;
                let to = push_cell_node(
                    boundary,
                    FaceSplitBoundaryNode::FaceInterior {
                        point: pair[1].point.clone(),
                    },
                )?;
                push_constraint(constraints, unique_constraints, from, to);
            }
        }
    }
    Ok(())
}

/// Seed opposite coplanar edge endpoints that lie in the current source face.
///
/// Split records retain edge/edge crossings and collinear intervals, and
/// vertex-overlap records retain the same facts when the source graph reports
/// them explicitly. A complete planar cell builder must also be able to replay
/// endpoint containment directly from source operands: an opposite coplanar
/// triangle can lie wholly inside the current source face without needing an
/// edge crossing construction. These endpoint facts are exact
/// point-in-triangle predicate decisions, so they are safe to promote to CDT
/// constraints with the same provenance as copied vertex-overlap records.
fn seed_contained_coplanar_opposite_edge_endpoints(
    side: MeshSide,
    face: usize,
    left: &ExactMesh,
    right: &ExactMesh,
    edges: &mut [CoplanarCellEdge],
) -> hypertri::Result<()> {
    let opposite_side = match side {
        MeshSide::Left => MeshSide::Right,
        MeshSide::Right => MeshSide::Left,
    };
    let source_mesh = match side {
        MeshSide::Left => left,
        MeshSide::Right => right,
    };
    let edge_keys = edges.iter().map(|edge| edge.edge).collect::<Vec<_>>();
    for edge in edge_keys {
        for (vertex, parameter) in [(edge[0], Real::from(0)), (edge[1], Real::from(1))] {
            let point = vertex_point_for_side(opposite_side, vertex, left, right)?;
            if point_lies_on_mesh_face_closed(source_mesh, face, &point)? {
                push_coplanar_cell_edge_point(edges, edge, parameter, point)?;
            }
        }
    }
    Ok(())
}

/// Seed source-boundary vertices that lie on opposite coplanar edges.
///
/// A coplanar opposite edge can enter the current source face exactly through a
/// source vertex and then continue through the face interior. If the retained
/// split graph has no explicit edge/edge split for that endpoint touch, the
/// contained opposite endpoint alone is insufficient to emit a positive cell
/// constraint. This replayed point-on-segment predicate plus exact projected
/// edge parameter keeps the boundary-crossing cell construction auditable
/// without adding a boolean-specific recovery branch.
fn seed_source_boundary_vertices_on_coplanar_opposite_edges(
    side: MeshSide,
    face: usize,
    left: &ExactMesh,
    right: &ExactMesh,
    edges: &mut [CoplanarCellEdge],
) -> hypertri::Result<()> {
    let source_mesh = match side {
        MeshSide::Left => left,
        MeshSide::Right => right,
    };
    let opposite_side = match side {
        MeshSide::Left => MeshSide::Right,
        MeshSide::Right => MeshSide::Left,
    };
    let source_triangle =
        source_mesh
            .triangles()
            .get(face)
            .ok_or(hypertri::Error::InvalidInput {
                reason: "face-cell coplanar split graph references a missing source face",
            })?;
    let projection = choose_region_projection(source_mesh, face)?;

    let edge_keys = edges.iter().map(|edge| edge.edge).collect::<Vec<_>>();
    for edge in edge_keys {
        let start = vertex_point_for_side(opposite_side, edge[0], left, right)?;
        let end = vertex_point_for_side(opposite_side, edge[1], left, right)?;
        let projected_start = project_for_predicate(&start, projection);
        let projected_end = project_for_predicate(&end, projection);
        for &vertex in &source_triangle.0 {
            let point = source_mesh
                .vertices()
                .get(vertex)
                .ok_or(hypertri::Error::InvalidInput {
                    reason: "face-cell source triangle references a missing vertex",
                })?
                .clone();
            let projected_point = project_for_predicate(&point, projection);
            match point_on_segment(&projected_start, &projected_end, &projected_point).value() {
                Some(true) => {
                    let parameter = projected_segment_parameter3(&point, &start, &end, projection)
                        .ok_or(hypertri::Error::InvalidInput {
                            reason: "face-cell coplanar source-boundary parameter is undefined",
                        })?;
                    push_coplanar_cell_edge_point(edges, edge, parameter, point)?;
                }
                Some(false) => {}
                None => {
                    return Err(hypertri::Error::PredicateUndecided {
                        predicate: "face-cell source vertex on coplanar opposite edge",
                    });
                }
            }
        }
    }
    Ok(())
}

/// Seed proper source-boundary/opposite-edge crossings.
///
/// The coplanar split plan normally carries these facts as edge split
/// constructions. Replaying them here makes the planar cell builder complete
/// against the exact source operands instead of depending on a previous graph
/// stage to have retained every boundary clipping point. Each inserted point
/// is first certified as a proper projected segment intersection, then lifted
/// back to the source carrier plane and parameterized on the opposite edge.
fn seed_source_boundary_edge_crossings_on_coplanar_opposite_edges(
    side: MeshSide,
    face: usize,
    left: &ExactMesh,
    right: &ExactMesh,
    edges: &mut [CoplanarCellEdge],
) -> hypertri::Result<()> {
    let source_mesh = match side {
        MeshSide::Left => left,
        MeshSide::Right => right,
    };
    let opposite_side = match side {
        MeshSide::Left => MeshSide::Right,
        MeshSide::Right => MeshSide::Left,
    };
    let source_triangle =
        source_mesh
            .triangles()
            .get(face)
            .ok_or(hypertri::Error::InvalidInput {
                reason: "face-cell coplanar split graph references a missing source face",
            })?;
    let projection = choose_region_projection(source_mesh, face)?;
    let source_edges = triangle_edges(source_triangle.0);

    let edge_keys = edges.iter().map(|edge| edge.edge).collect::<Vec<_>>();
    for edge in edge_keys {
        let opposite_start = vertex_point_for_side(opposite_side, edge[0], left, right)?;
        let opposite_end = vertex_point_for_side(opposite_side, edge[1], left, right)?;
        let projected_opposite_start = project_for_predicate(&opposite_start, projection);
        let projected_opposite_end = project_for_predicate(&opposite_end, projection);
        for source_edge in source_edges {
            let source_start = source_mesh.vertices().get(source_edge[0]).ok_or(
                hypertri::Error::InvalidInput {
                    reason: "face-cell source edge references a missing start vertex",
                },
            )?;
            let source_end = source_mesh.vertices().get(source_edge[1]).ok_or(
                hypertri::Error::InvalidInput {
                    reason: "face-cell source edge references a missing end vertex",
                },
            )?;
            let projected_source_start = project_for_predicate(source_start, projection);
            let projected_source_end = project_for_predicate(source_end, projection);
            match classify_segment_intersection(
                &projected_source_start,
                &projected_source_end,
                &projected_opposite_start,
                &projected_opposite_end,
            )
            .value()
            {
                Some(SegmentIntersection::Proper) => {
                    let projected_crossing = proper_segment_intersection_point(
                        &projected_source_start,
                        &projected_source_end,
                        &projected_opposite_start,
                        &projected_opposite_end,
                    )
                    .value()
                    .flatten()
                    .ok_or(hypertri::Error::PredicateUndecided {
                        predicate: "face-cell coplanar source-boundary crossing construction",
                    })?;
                    let lifted = lift_projected_face_cell_point(
                        source_mesh,
                        face,
                        projection,
                        &hypertri::ExactPoint::new(projected_crossing.x, projected_crossing.y),
                    )?;
                    let parameter =
                        projected_segment_parameter3(
                            &lifted,
                            &opposite_start,
                            &opposite_end,
                            projection,
                        )
                        .ok_or(hypertri::Error::InvalidInput {
                            reason: "face-cell coplanar source-boundary crossing parameter is undefined",
                        })?;
                    push_coplanar_cell_edge_point(edges, edge, parameter, lifted)?;
                }
                Some(
                    SegmentIntersection::Disjoint
                    | SegmentIntersection::EndpointTouch
                    | SegmentIntersection::CollinearOverlap
                    | SegmentIntersection::Identical,
                ) => {}
                None => {
                    return Err(hypertri::Error::PredicateUndecided {
                        predicate: "face-cell coplanar source-boundary segment relation",
                    });
                }
            }
        }
    }
    Ok(())
}

/// Return whether a coplanar split graph touches the requested source face.
fn coplanar_split_graph_involves_face(
    left_face: usize,
    right_face: usize,
    side: MeshSide,
    face: usize,
) -> bool {
    match side {
        MeshSide::Left => left_face == face,
        MeshSide::Right => right_face == face,
    }
}

/// Return the opposite triangle's directed edges for one source-face graph.
fn coplanar_opposite_edges(
    left_face: usize,
    right_face: usize,
    side: MeshSide,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<Vec<CoplanarCellEdge>> {
    let (mesh, face) = match side {
        MeshSide::Left => (right, right_face),
        MeshSide::Right => (left, left_face),
    };
    let triangle = mesh
        .triangles()
        .get(face)
        .ok_or(hypertri::Error::InvalidInput {
            reason: "face-cell coplanar split graph references a missing opposite face",
        })?;
    Ok(triangle_edges(triangle.0)
        .into_iter()
        .map(|edge| CoplanarCellEdge {
            edge,
            points: Vec::new(),
        })
        .collect())
}

/// Return opposite-face edges that use `vertex` as an endpoint.
fn coplanar_edges_incident_to_vertex(edges: &[CoplanarCellEdge], vertex: usize) -> Vec<[usize; 2]> {
    edges
        .iter()
        .map(|edge| edge.edge)
        .filter(|edge| edge[0] == vertex || edge[1] == vertex)
        .collect()
}

/// Fetch a retained exact vertex point from the requested source mesh.
fn vertex_point_for_side(
    side: MeshSide,
    vertex: usize,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<Point3> {
    let mesh = match side {
        MeshSide::Left => left,
        MeshSide::Right => right,
    };
    mesh.vertices()
        .get(vertex)
        .cloned()
        .ok_or(hypertri::Error::InvalidInput {
            reason: "face-cell coplanar vertex overlap references a missing vertex",
        })
}

/// Insert one exact point on one opposite-face edge, deduplicating by point.
fn push_coplanar_cell_edge_point(
    edges: &mut [CoplanarCellEdge],
    edge: [usize; 2],
    parameter: Real,
    point: Point3,
) -> hypertri::Result<()> {
    let Some(entry) = edges.iter_mut().find(|entry| entry.edge == edge) else {
        return Err(hypertri::Error::InvalidInput {
            reason: "face-cell coplanar edge constraint references a non-triangle edge",
        });
    };
    if entry
        .points
        .iter()
        .any(|seen| points_equal(&seen.point, &point) == Some(true))
    {
        return Ok(());
    }
    entry
        .points
        .push(CoplanarCellEdgePoint { parameter, point });
    Ok(())
}

/// Sort edge points by their exact edge parameter.
fn sort_coplanar_cell_edge_points(points: &mut Vec<CoplanarCellEdgePoint>) -> hypertri::Result<()> {
    let mut ordered = Vec::<CoplanarCellEdgePoint>::with_capacity(points.len());
    for point in points.drain(..) {
        let mut insert_at = ordered.len();
        for (index, existing) in ordered.iter().enumerate() {
            if compare_ordering(
                &point.parameter,
                &existing.parameter,
                "face-cell coplanar edge point ordering",
            )? == Ordering::Less
            {
                insert_at = index;
                break;
            }
        }
        ordered.insert(insert_at, point);
    }
    *points = ordered;
    Ok(())
}

/// Drop repeated exact points after sorting.
fn dedup_coplanar_cell_edge_points(
    points: &mut Vec<CoplanarCellEdgePoint>,
) -> hypertri::Result<()> {
    let mut deduped = Vec::<CoplanarCellEdgePoint>::with_capacity(points.len());
    for point in points.drain(..) {
        if deduped
            .iter()
            .any(|seen| points_equal(&seen.point, &point.point) == Some(true))
        {
            continue;
        }
        deduped.push(point);
    }
    *points = deduped;
    Ok(())
}

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
        // sides of the arrangement for winding policy. Earcut follows Held,
        // "FIST: Fast Industrial-Strength Triangulation of Polygons,"
        // *Algorithmica* 30 (2001), and is used only after the loop graph has
        // been certified as a simple degree-two cycle. The loop triangulation
        // is still predicate-validated by `FaceRegionTriangulation::validate`.
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

fn lift_projected_face_cell_point(
    mesh: &ExactMesh,
    face: usize,
    projection: CoplanarProjection,
    point: &hypertri::ExactPoint,
) -> hypertri::Result<Point3> {
    let triangle = mesh.triangles()[face].0;
    let a = mesh.vertices()[triangle[0]].clone();
    let b = mesh.vertices()[triangle[1]].clone();
    let c = mesh.vertices()[triangle[2]].clone();
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

fn point_on_closed_segment(
    point: &hypertri::ExactPoint,
    start: &hypertri::ExactPoint,
    end: &hypertri::ExactPoint,
) -> hypertri::Result<bool> {
    point_on_segment(
        &predicate_point2(start),
        &predicate_point2(end),
        &predicate_point2(point),
    )
    .value()
    .ok_or(hypertri::Error::PredicateUndecided {
        predicate: "face-cell constraint point-on-segment",
    })
}

fn exact_points_equal(
    left: &hypertri::ExactPoint,
    right: &hypertri::ExactPoint,
) -> hypertri::Result<bool> {
    let x = compare_ordering(&left.x, &right.x, "face-cell projected x equality")?;
    let y = compare_ordering(&left.y, &right.y, "face-cell projected y equality")?;
    Ok(x == Ordering::Equal && y == Ordering::Equal)
}

fn triangulate_source_triangle_with_closed_constraint_loops(
    vertices: &[hypertri::ExactPoint],
    interior_constraints: &[Constraint],
) -> hypertri::Result<Option<Vec<usize>>> {
    if interior_constraints.is_empty() {
        return Ok(None);
    }
    let mut adjacency = vec![Vec::<usize>::new(); vertices.len()];
    for constraint in interior_constraints {
        adjacency[constraint.from].push(constraint.to);
        adjacency[constraint.to].push(constraint.from);
    }

    let mut seen = vec![false; vertices.len()];
    let mut loops = Vec::<Vec<usize>>::new();
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
            return Ok(None);
        }
        loops.push(order_simple_cycle(&adjacency, component[0])?);
    }
    if loops.is_empty() {
        return Ok(None);
    }

    let mut projected = vec![
        vertices[0].clone(),
        vertices[1].clone(),
        vertices[2].clone(),
    ];
    let mut local_to_global = vec![0, 1, 2];
    let mut hole_indices = Vec::with_capacity(loops.len());
    for loop_vertices in &loops {
        hole_indices.push(projected.len());
        for &vertex in loop_vertices {
            projected.push(vertices[vertex].clone());
            local_to_global.push(vertex);
        }
    }

    let mut triangles = hypertri::earcut(&projected, &hole_indices)?
        .chunks_exact(3)
        .flat_map(|triangle| {
            [
                local_to_global[triangle[0]],
                local_to_global[triangle[1]],
                local_to_global[triangle[2]],
            ]
        })
        .collect::<Vec<_>>();

    for loop_vertices in &loops {
        let loop_points = loop_vertices
            .iter()
            .map(|&vertex| vertices[vertex].clone())
            .collect::<Vec<_>>();
        let loop_triangles = hypertri::earcut(&loop_points, &[])?;
        for triangle in loop_triangles.chunks_exact(3) {
            triangles.extend([
                loop_vertices[triangle[0]],
                loop_vertices[triangle[1]],
                loop_vertices[triangle[2]],
            ]);
        }
    }

    refine_missing_constraint_edges(vertices, &mut triangles, interior_constraints)?;
    let all_constraints_present = interior_constraints
        .iter()
        .all(|constraint| triangles_have_edge(&triangles, constraint.from, constraint.to));
    Ok(all_constraints_present.then_some(triangles))
}

fn triangulate_source_triangle_with_collinear_constraint_refinement(
    vertices: &[hypertri::ExactPoint],
    constraints: &[Constraint],
) -> hypertri::Result<Option<Vec<usize>>> {
    if vertices.len() < 3 {
        return Ok(None);
    }
    let mut triangles = vec![0, 1, 2];
    let max_passes = constraints.len().saturating_mul(vertices.len()).max(1);
    refine_missing_constraint_edges_with_pass_limit(
        vertices,
        &mut triangles,
        constraints,
        max_passes,
    )?;
    if constraints
        .iter()
        .all(|constraint| triangles_have_edge(&triangles, constraint.from, constraint.to))
    {
        Ok(Some(triangles))
    } else {
        Ok(None)
    }
}

fn refine_missing_constraint_edges(
    vertices: &[hypertri::ExactPoint],
    triangles: &mut Vec<usize>,
    constraints: &[Constraint],
) -> hypertri::Result<()> {
    let max_passes = constraints.len().saturating_mul(vertices.len()).max(1);
    refine_missing_constraint_edges_with_pass_limit(vertices, triangles, constraints, max_passes)
}

fn refine_missing_constraint_edges_with_pass_limit(
    vertices: &[hypertri::ExactPoint],
    triangles: &mut Vec<usize>,
    constraints: &[Constraint],
    max_passes: usize,
) -> hypertri::Result<()> {
    for _ in 0..max_passes {
        let Some(missing) = constraints
            .iter()
            .find(|constraint| !triangles_have_edge(triangles, constraint.from, constraint.to))
        else {
            return Ok(());
        };
        if !split_collinear_triangle_edge(vertices, triangles, missing.from, missing.to)? {
            return Ok(());
        }
    }
    Ok(())
}

fn triangles_have_edge(triangles: &[usize], from: usize, to: usize) -> bool {
    triangles.chunks_exact(3).any(|triangle| {
        (0..3).any(|edge| {
            let a = triangle[edge];
            let b = triangle[(edge + 1) % 3];
            (a == from && b == to) || (a == to && b == from)
        })
    })
}

fn split_collinear_triangle_edge(
    vertices: &[hypertri::ExactPoint],
    triangles: &mut Vec<usize>,
    from: usize,
    to: usize,
) -> hypertri::Result<bool> {
    let mut triangle_index = 0;
    while triangle_index + 2 < triangles.len() {
        let triangle = [
            triangles[triangle_index],
            triangles[triangle_index + 1],
            triangles[triangle_index + 2],
        ];
        for edge in 0..3 {
            let a = triangle[edge];
            let b = triangle[(edge + 1) % 3];
            let opposite = triangle[(edge + 2) % 3];
            for mid in [from, to] {
                if point_lies_strictly_between(vertices, mid, a, b)?
                    && split_triangle_edge(
                        vertices,
                        triangles,
                        triangle_index,
                        a,
                        mid,
                        b,
                        opposite,
                    )?
                {
                    return Ok(true);
                }
            }
        }
        triangle_index += 3;
    }
    Ok(false)
}

fn split_triangle_edge(
    vertices: &[hypertri::ExactPoint],
    triangles: &mut Vec<usize>,
    triangle_index: usize,
    a: usize,
    mid: usize,
    b: usize,
    opposite: usize,
) -> hypertri::Result<bool> {
    let first = [a, mid, opposite];
    let second = [mid, b, opposite];
    if !triangle_is_non_degenerate(vertices, first)?
        || !triangle_is_non_degenerate(vertices, second)?
    {
        return Ok(false);
    }
    triangles.splice(
        triangle_index..triangle_index + 3,
        first.into_iter().chain(second),
    );
    Ok(true)
}

fn triangle_is_non_degenerate(
    vertices: &[hypertri::ExactPoint],
    triangle: [usize; 3],
) -> hypertri::Result<bool> {
    Ok(compare_ordering(
        &triangle_area2_signed(vertices, triangle)?,
        &Real::from(0),
        "face-cell refined triangle area",
    )? != Ordering::Equal)
}

fn triangle_area2_signed(
    vertices: &[hypertri::ExactPoint],
    triangle: [usize; 3],
) -> hypertri::Result<Real> {
    let [a, b, c] = triangle;
    let a = vertices.get(a).ok_or(hypertri::Error::InvalidInput {
        reason: "face-cell refined triangle references missing vertex",
    })?;
    let b = vertices.get(b).ok_or(hypertri::Error::InvalidInput {
        reason: "face-cell refined triangle references missing vertex",
    })?;
    let c = vertices.get(c).ok_or(hypertri::Error::InvalidInput {
        reason: "face-cell refined triangle references missing vertex",
    })?;
    Ok(((a.x.clone() * &b.y) - &(a.y.clone() * &b.x))
        + &((b.x.clone() * &c.y) - &(b.y.clone() * &c.x))
        + &((c.x.clone() * &a.y) - &(c.y.clone() * &a.x)))
}

fn point_lies_strictly_between(
    vertices: &[hypertri::ExactPoint],
    point: usize,
    start: usize,
    end: usize,
) -> hypertri::Result<bool> {
    if point == start || point == end {
        return Ok(false);
    }
    let point_ref = vertices.get(point).ok_or(hypertri::Error::InvalidInput {
        reason: "face-cell refined split point is out of range",
    })?;
    let start_ref = vertices.get(start).ok_or(hypertri::Error::InvalidInput {
        reason: "face-cell refined edge start is out of range",
    })?;
    let end_ref = vertices.get(end).ok_or(hypertri::Error::InvalidInput {
        reason: "face-cell refined edge end is out of range",
    })?;
    if exact_points_equal(point_ref, start_ref)? || exact_points_equal(point_ref, end_ref)? {
        return Ok(false);
    }
    point_on_closed_segment(point_ref, start_ref, end_ref)
}

fn compare_ordering(
    left: &Real,
    right: &Real,
    predicate: &'static str,
) -> hypertri::Result<Ordering> {
    compare_reals(left, right)
        .value()
        .ok_or(hypertri::Error::PredicateUndecided { predicate })
}

fn predicate_point2(point: &hypertri::ExactPoint) -> Point2 {
    Point2::new(point.x.clone(), point.y.clone())
}

fn point3_sub(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        sub_real(&left.x, &right.x),
        sub_real(&left.y, &right.y),
        sub_real(&left.z, &right.z),
    )
}

fn cross(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        sub_real(&mul_real(&left.y, &right.z), &mul_real(&left.z, &right.y)),
        sub_real(&mul_real(&left.z, &right.x), &mul_real(&left.x, &right.z)),
        sub_real(&mul_real(&left.x, &right.y), &mul_real(&left.y, &right.x)),
    )
}

fn add_real(left: &Real, right: &Real) -> Real {
    left.clone() + right
}

fn sub_real(left: &Real, right: &Real) -> Real {
    left.clone() - right
}

fn mul_real(left: &Real, right: &Real) -> Real {
    left.clone() * right
}

fn div_real(numerator: Real, denominator: &Real, reason: &'static str) -> hypertri::Result<Real> {
    (numerator / denominator).map_err(|_| hypertri::Error::InvalidInput { reason })
}

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

/// Add exact source-triangle boundary constraints after retained split points
/// have been inserted.
///
/// A graph vertex may lie on an original source edge. Passing both the
/// unsplit edge and its retained subsegments to CDT makes the constraint graph
/// inconsistent, because the triangulation can only contain the subsegments
/// once the intermediate point is part of the object. The boundary is
/// therefore rebuilt by sorting all exact-on-edge nodes with certified
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

fn sort_boundary_chain(chain: &mut Vec<(usize, Real)>) -> hypertri::Result<()> {
    let mut ordered = Vec::<(usize, Real)>::with_capacity(chain.len());
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

fn segment_parameter(
    point: &hypertri::ExactPoint,
    start: &hypertri::ExactPoint,
    end: &hypertri::ExactPoint,
) -> hypertri::Result<Real> {
    let dx = sub_real(&end.x, &start.x);
    if compare_ordering(&dx, &Real::from(0), "face-cell boundary dx")? != Ordering::Equal {
        return div_real(
            sub_real(&point.x, &start.x),
            &dx,
            "face-cell boundary x parameter denominator is zero",
        );
    }
    let dy = sub_real(&end.y, &start.y);
    if compare_ordering(&dy, &Real::from(0), "face-cell boundary dy")? != Ordering::Equal {
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

fn pair_involves_face(pair: &FacePairEvents, side: MeshSide, face: usize) -> bool {
    match side {
        MeshSide::Left => pair.left_face == face,
        MeshSide::Right => pair.right_face == face,
    }
}

fn pair_has_proper_crossing(pair: &FacePairEvents) -> bool {
    // Contact-only candidate pairs can occur next to coplanar source-face
    // overlaps. They are valid graph evidence, but they do not cut a positive
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

fn classify_point_on_mesh_face(
    mesh: &ExactMesh,
    face: usize,
    point: &Point3,
) -> hypertri::Result<TriangleLocation> {
    let projection = choose_region_projection(mesh, face)?;
    let triangle = mesh.triangles()[face].0;
    let vertices = [
        mesh.vertices()[triangle[0]].clone(),
        mesh.vertices()[triangle[1]].clone(),
        mesh.vertices()[triangle[2]].clone(),
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

const fn point_triangle_location_is_closed(location: TriangleLocation) -> bool {
    matches!(
        location,
        TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
    )
}

fn point_lies_on_mesh_face_closed(
    mesh: &ExactMesh,
    face: usize,
    point: &Point3,
) -> hypertri::Result<bool> {
    classify_point_on_mesh_face(mesh, face, point).map(point_triangle_location_is_closed)
}

fn points_equal(left: &Point3, right: &Point3) -> Option<bool> {
    Some(
        compare_reals(&left.x, &right.x).value()? == std::cmp::Ordering::Equal
            && compare_reals(&left.y, &right.y).value()? == std::cmp::Ordering::Equal
            && compare_reals(&left.z, &right.z).value()? == std::cmp::Ordering::Equal,
    )
}

#[cfg(test)]
mod tests {
    use super::super::graph::CoplanarOverlapSplitGraph;
    use super::super::validation::ValidationPolicy;
    use super::*;

    fn open_triangle_mesh(pos: &[i64]) -> ExactMesh {
        ExactMesh::from_i64_triangles_with_policy(pos, &[0, 1, 2], ValidationPolicy::ALLOW_BOUNDARY)
            .unwrap()
    }

    fn ep(x: i64, y: i64) -> hypertri::ExactPoint {
        hypertri::ExactPoint::new(Real::from(x), Real::from(y))
    }

    fn p3(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn assert_point(point: &Point3, x: i64, y: i64, z: i64) {
        assert_eq!(
            compare_reals(&point.x, &Real::from(x)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(&point.y, &Real::from(y)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_reals(&point.z, &Real::from(z)).value(),
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

    #[test]
    fn coplanar_cell_constraints_replay_contained_opposite_edge_endpoints() {
        let left = open_triangle_mesh(&[0, 0, 0, 8, 0, 0, 0, 8, 0]);
        let right = open_triangle_mesh(&[2, 2, 0, 4, 2, 0, 2, 4, 0]);
        let split_plan = CoplanarOverlapSplitPlan {
            graphs: vec![CoplanarOverlapSplitGraph {
                left_face: 0,
                right_face: 0,
                projection: CoplanarProjection::Xy,
                edge_splits: Vec::new(),
                vertex_overlaps: Vec::new(),
            }],
        };
        let mut boundary = left.triangles()[0]
            .0
            .into_iter()
            .map(|vertex| FaceSplitBoundaryNode::OriginalVertex {
                vertex,
                point: left.vertices()[vertex].clone(),
            })
            .collect::<Vec<_>>();
        let mut constraints = Vec::new();
        let mut unique_constraints = BTreeSet::new();

        append_coplanar_face_cell_constraints(
            &split_plan,
            MeshSide::Left,
            0,
            &left,
            &right,
            &mut boundary,
            &mut constraints,
            &mut unique_constraints,
        )
        .unwrap();

        assert_eq!(boundary.len(), 6);
        assert_eq!(constraints.len(), 3);
        for [from, to] in [[3, 4], [4, 5], [5, 3]] {
            assert!(
                constraints
                    .iter()
                    .any(|constraint| constraint.from == from && constraint.to == to
                        || constraint.from == to && constraint.to == from),
                "missing contained opposite edge constraint {from}-{to}"
            );
        }
    }

    #[test]
    fn coplanar_cell_constraints_replay_source_vertex_on_opposite_edge() {
        let left = open_triangle_mesh(&[0, 0, 0, 4, 0, 0, 0, 4, 0]);
        let right = open_triangle_mesh(&[-1, -1, 0, 2, 2, 0, -1, 2, 0]);
        let split_plan = CoplanarOverlapSplitPlan {
            graphs: vec![CoplanarOverlapSplitGraph {
                left_face: 0,
                right_face: 0,
                projection: CoplanarProjection::Xy,
                edge_splits: Vec::new(),
                vertex_overlaps: Vec::new(),
            }],
        };
        let mut boundary = left.triangles()[0]
            .0
            .into_iter()
            .map(|vertex| FaceSplitBoundaryNode::OriginalVertex {
                vertex,
                point: left.vertices()[vertex].clone(),
            })
            .collect::<Vec<_>>();
        let mut constraints = Vec::new();
        let mut unique_constraints = BTreeSet::new();

        append_coplanar_face_cell_constraints(
            &split_plan,
            MeshSide::Left,
            0,
            &left,
            &right,
            &mut boundary,
            &mut constraints,
            &mut unique_constraints,
        )
        .unwrap();

        assert!(
            constraints
                .iter()
                .any(|constraint| constraint.from == 0 && constraint.to == 3
                    || constraint.from == 3 && constraint.to == 0),
            "missing source-vertex-to-contained-endpoint coplanar constraint: {constraints:?}"
        );
    }

    #[test]
    fn coplanar_cell_constraints_replay_source_boundary_crossings() {
        let left = open_triangle_mesh(&[0, 0, 0, 4, 0, 0, 0, 4, 0]);
        let right = open_triangle_mesh(&[-1, 2, 0, 2, -1, 0, -1, -1, 0]);
        let split_plan = CoplanarOverlapSplitPlan {
            graphs: vec![CoplanarOverlapSplitGraph {
                left_face: 0,
                right_face: 0,
                projection: CoplanarProjection::Xy,
                edge_splits: Vec::new(),
                vertex_overlaps: Vec::new(),
            }],
        };
        let mut boundary = left.triangles()[0]
            .0
            .into_iter()
            .map(|vertex| FaceSplitBoundaryNode::OriginalVertex {
                vertex,
                point: left.vertices()[vertex].clone(),
            })
            .collect::<Vec<_>>();
        let mut constraints = Vec::new();
        let mut unique_constraints = BTreeSet::new();

        append_coplanar_face_cell_constraints(
            &split_plan,
            MeshSide::Left,
            0,
            &left,
            &right,
            &mut boundary,
            &mut constraints,
            &mut unique_constraints,
        )
        .unwrap();

        assert_eq!(constraints.len(), 1);
        assert!(constraints.iter().any(|constraint| {
            points_equal(
                boundary_node_point(&boundary[constraint.from]),
                &p3(0, 1, 0),
            ) == Some(true)
                && points_equal(boundary_node_point(&boundary[constraint.to]), &p3(1, 0, 0))
                    == Some(true)
                || points_equal(
                    boundary_node_point(&boundary[constraint.from]),
                    &p3(1, 0, 0),
                ) == Some(true)
                    && points_equal(boundary_node_point(&boundary[constraint.to]), &p3(0, 1, 0))
                        == Some(true)
        }));
    }
}
