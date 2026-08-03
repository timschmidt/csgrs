//! Exact source-face corefinement prototype.
//!
//! This module is test-only until the new surface-arrangement orchestrator is
//! atomically substituted for EMBER. It deliberately exercises the production
//! scalar, predicate-policy, intersection-graph, and Hypertri paths while
//! adding no shipped second engine.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use hyperlattice::{Point3, Real};

use crate::context::{DecisionContext, MeshCertainty};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, compare_real_decision};
use crate::intersection::{
    PairwiseIntersectionEventIds, PairwiseIntersectionGraph, pairwise_support_identity,
};
use crate::point_interner::PointInterner;
use crate::polygon::{
    ConstructionEdgeIdentity, ConstructionPlaneIdentity, ConstructionVertexIdentity, ConvexPolygon,
};
use crate::predicate::{
    classify_point_decision, classify_projective_point_decision, classify_real,
};
use crate::storage_hash::StorageHashMap;

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
enum ArrangementPointIdentity {
    Construction(ConstructionVertexIdentity),
    CoplanarEdges([ConstructionEdgeIdentity; 2]),
}

#[derive(Clone)]
struct RawConstraint {
    endpoints: [u32; 2],
    line: ConstructionEdgeIdentity,
}

#[derive(Default)]
struct FaceWork {
    boundary: Vec<u32>,
    constraints: Vec<RawConstraint>,
    contacts: Vec<u32>,
    changed: bool,
}

struct ArrangementPointArena {
    points: Vec<Point3>,
    identities: Vec<ArrangementPointIdentity>,
    structural: StorageHashMap<ArrangementPointIdentity, u32>,
    numeric: PointInterner<()>,
}

impl ArrangementPointArena {
    fn with_capacity(capacity: usize) -> HypermeshResult<Self> {
        let mut points = Vec::new();
        points
            .try_reserve_exact(capacity)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface arrangement point arena",
            })?;
        let mut identities = Vec::new();
        identities
            .try_reserve_exact(capacity)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface arrangement point identity arena",
            })?;
        let mut structural = StorageHashMap::default();
        structural
            .try_reserve(capacity)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface arrangement point identity index",
            })?;
        Ok(Self {
            points,
            identities,
            structural,
            numeric: PointInterner::try_with_capacity(capacity, false, false)?,
        })
    }

    fn insert(
        &mut self,
        decisions: &DecisionContext,
        identity: ArrangementPointIdentity,
        point: Point3,
    ) -> HypermeshResult<u32> {
        if let Some(&existing) = self.structural.get(&identity) {
            if exact_rational_points_contradict(&self.points[existing as usize], &point) {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "one construction identity materialized at contradictory points",
                });
            }
            return Ok(existing);
        }

        self.structural
            .try_reserve(1)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface arrangement point identity index",
            })?;
        self.identities
            .try_reserve(1)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface arrangement point identity arena",
            })?;
        let old_len = self.points.len();
        let index = self
            .numeric
            .intern_owned(decisions, &mut self.points, point, None)?;
        let compact = u32::try_from(index).map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement point arena",
        })?;
        if index == old_len {
            self.identities.push(identity.clone());
        } else {
            let canonical =
                self.identities
                    .get_mut(index)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "numeric point alias has no construction identity",
                    })?;
            if identity < *canonical {
                *canonical = identity.clone();
            }
        }
        self.structural.insert(identity, compact);
        Ok(compact)
    }
}

#[derive(Debug)]
struct SurfaceCorefinement {
    points: Vec<Point3>,
    identities: Vec<ArrangementPointIdentity>,
    face_offsets: Box<[u32]>,
    triangles: Vec<[u32; 3]>,
    constraint_offsets: Box<[u32]>,
    constraints: Vec<[u32; 2]>,
    contact_offsets: Box<[u32]>,
    contacts: Vec<u32>,
}

impl SurfaceCorefinement {
    fn face_triangles(&self, face: usize) -> &[[u32; 3]] {
        let start = self.face_offsets[face] as usize;
        let end = self.face_offsets[face + 1] as usize;
        &self.triangles[start..end]
    }

    fn face_constraints(&self, face: usize) -> &[[u32; 2]] {
        let start = self.constraint_offsets[face] as usize;
        let end = self.constraint_offsets[face + 1] as usize;
        &self.constraints[start..end]
    }

    fn face_contacts(&self, face: usize) -> &[u32] {
        let start = self.contact_offsets[face] as usize;
        let end = self.contact_offsets[face + 1] as usize;
        &self.contacts[start..end]
    }
}

fn corefine_surface(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    intersections: &PairwiseIntersectionGraph,
) -> HypermeshResult<SurfaceCorefinement> {
    if intersections.len() != polygons.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "intersection graph and source-face counts differ",
        });
    }
    let initial_points = polygons.iter().try_fold(0usize, |total, polygon| {
        total.checked_add(polygon.vertex_count())
    });
    let mut arena = ArrangementPointArena::with_capacity(initial_points.ok_or(
        HypermeshError::CapacityOverflow {
            operation: "surface arrangement source vertices",
        },
    )?)?;
    let mut work = Vec::new();
    work.try_reserve_exact(polygons.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement face work",
        })?;
    work.resize_with(polygons.len(), FaceWork::default);

    for (face, polygon) in polygons.iter().enumerate() {
        add_source_boundary(decisions, face, polygon, &mut arena, &mut work[face])?;
    }
    append_intersection_constraints(decisions, polygons, intersections, &mut arena, &mut work)?;

    let offset_capacity =
        polygons
            .len()
            .checked_add(1)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "surface arrangement face offsets",
            })?;
    let mut face_offsets = Vec::new();
    let mut constraint_offsets = Vec::new();
    let mut contact_offsets = Vec::new();
    for offsets in [
        &mut face_offsets,
        &mut constraint_offsets,
        &mut contact_offsets,
    ] {
        offsets.try_reserve_exact(offset_capacity).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "surface arrangement face offsets",
            }
        })?;
        offsets.push(0_u32);
    }
    let mut triangles = Vec::new();
    let mut constraints = Vec::new();
    let mut contacts = Vec::new();
    for (face, (polygon, face_work)) in polygons.iter().zip(&work).enumerate() {
        let result = corefine_face(decisions, face, polygon, face_work, &mut arena)?;
        triangles.try_reserve(result.triangles.len()).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "surface arrangement triangles",
            }
        })?;
        constraints
            .try_reserve(result.constraints.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "surface arrangement constraints",
            })?;
        contacts.try_reserve(result.contacts.len()).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "surface arrangement contacts",
            }
        })?;
        triangles.extend(result.triangles);
        constraints.extend(result.constraints);
        contacts.extend(result.contacts);
        face_offsets.push(compact_len(
            triangles.len(),
            "surface arrangement triangle offsets",
        )?);
        constraint_offsets.push(compact_len(
            constraints.len(),
            "surface arrangement constraint offsets",
        )?);
        contact_offsets.push(compact_len(
            contacts.len(),
            "surface arrangement contact offsets",
        )?);
    }
    Ok(SurfaceCorefinement {
        points: arena.points,
        identities: arena.identities,
        face_offsets: face_offsets.into_boxed_slice(),
        triangles,
        constraint_offsets: constraint_offsets.into_boxed_slice(),
        constraints,
        contact_offsets: contact_offsets.into_boxed_slice(),
        contacts,
    })
}

fn compact_len(len: usize, operation: &'static str) -> HypermeshResult<u32> {
    u32::try_from(len).map_err(|_| HypermeshError::CapacityOverflow { operation })
}

fn add_source_boundary(
    decisions: &DecisionContext,
    _face: usize,
    polygon: &ConvexPolygon,
    arena: &mut ArrangementPointArena,
    work: &mut FaceWork,
) -> HypermeshResult<()> {
    let vertices =
        polygon
            .known_vertices
            .as_ref()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "source face has no retained vertex cycle",
            })?;
    let vertex_identities =
        polygon
            .known_vertex_identities()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "source face has no canonical vertex identities",
            })?;
    let edge_identities =
        polygon
            .known_edge_identities()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "source face has no canonical edge identities",
            })?;
    if vertices.len() < 3
        || vertices.len() != vertex_identities.len()
        || vertices.len() != edge_identities.len()
    {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "source face geometry and identity cycles are not aligned",
        });
    }
    work.boundary
        .try_reserve_exact(vertices.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement face boundary",
        })?;
    work.constraints
        .try_reserve(vertices.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement face constraints",
        })?;
    for (point, identity) in vertices.iter().zip(vertex_identities) {
        work.boundary.push(arena.insert(
            decisions,
            ArrangementPointIdentity::Construction(identity),
            point.clone(),
        )?);
    }
    for index in 0..work.boundary.len() {
        work.constraints.push(RawConstraint {
            endpoints: [
                work.boundary[index],
                work.boundary[(index + 1) % work.boundary.len()],
            ],
            line: edge_identities
                .get(index)
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "source edge identity cycle is incomplete",
                })?,
        });
    }
    Ok(())
}

fn append_intersection_constraints(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    intersections: &PairwiseIntersectionGraph,
    arena: &mut ArrangementPointArena,
    work: &mut [FaceWork],
) -> HypermeshResult<()> {
    for face in 0..polygons.len() {
        for event in intersections.event_ids(face)? {
            match event {
                PairwiseIntersectionEventIds::NonCoplanarPoint {
                    point,
                    other_polygon: _,
                }
                | PairwiseIntersectionEventIds::CoplanarPoint {
                    point,
                    other_polygon: _,
                } => {
                    let point = insert_graph_point(decisions, intersections, arena, point)?;
                    work[face].contacts.push(point);
                    work[face].changed = true;
                }
                PairwiseIntersectionEventIds::NonCoplanarSegment {
                    endpoints,
                    other_polygon,
                } => {
                    let endpoints = endpoints
                        .map(|point| insert_graph_point(decisions, intersections, arena, point));
                    let endpoints = [endpoints[0].clone()?, endpoints[1].clone()?];
                    work[face].constraints.push(RawConstraint {
                        endpoints,
                        line: pairwise_split_line(face, other_polygon as usize)?,
                    });
                    work[face].changed = true;
                }
                PairwiseIntersectionEventIds::CoplanarSegment {
                    endpoints,
                    other_polygon: _,
                } => {
                    let first = intersections.construction_point(endpoints[0])?.0;
                    let second = intersections.construction_point(endpoints[1])?.0;
                    let line =
                        boundary_line_for_segment(decisions, &polygons[face], first, second)?;
                    let endpoints = [
                        insert_graph_point(decisions, intersections, arena, endpoints[0])?,
                        insert_graph_point(decisions, intersections, arena, endpoints[1])?,
                    ];
                    work[face]
                        .constraints
                        .push(RawConstraint { endpoints, line });
                    work[face].changed = true;
                }
                PairwiseIntersectionEventIds::CoplanarOverlap { other_polygon } => {
                    let other = other_polygon as usize;
                    if face >= other {
                        continue;
                    }
                    let overlay = coplanar_overlay(
                        decisions,
                        face,
                        &polygons[face],
                        other,
                        polygons
                            .get(other)
                            .ok_or(HypermeshError::SurfaceArrangementFailed {
                                reason: "coplanar overlap references an absent face",
                            })?,
                    )?;
                    let mut point_ids = Vec::new();
                    point_ids.try_reserve_exact(overlay.len()).map_err(|_| {
                        HypermeshError::CapacityOverflow {
                            operation: "coplanar overlay point IDs",
                        }
                    })?;
                    for vertex in &overlay {
                        point_ids.push(arena.insert(
                            decisions,
                            vertex.identity.clone(),
                            vertex.point.clone(),
                        )?);
                    }
                    for index in 0..overlay.len() {
                        let constraint = RawConstraint {
                            endpoints: [point_ids[index], point_ids[(index + 1) % overlay.len()]],
                            line: overlay[index].outgoing.clone(),
                        };
                        work[face].constraints.push(constraint.clone());
                        work[other].constraints.push(constraint);
                    }
                    work[face].changed = true;
                    work[other].changed = true;
                }
            }
        }
    }
    Ok(())
}

fn insert_graph_point(
    decisions: &DecisionContext,
    graph: &PairwiseIntersectionGraph,
    arena: &mut ArrangementPointArena,
    point: u32,
) -> HypermeshResult<u32> {
    let (materialized, identity) = graph.construction_point(point)?;
    arena.insert(
        decisions,
        ArrangementPointIdentity::Construction(identity.clone()),
        materialized.clone(),
    )
}

fn pairwise_split_line(face: usize, other: usize) -> HypermeshResult<ConstructionEdgeIdentity> {
    let mut planes = [
        pairwise_support_identity(face)?.ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "source face has no operation-local support identity",
        })?,
        pairwise_support_identity(other)?.ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "intersecting face has no operation-local support identity",
        })?,
    ];
    planes.sort_unstable();
    if planes[0] == planes[1] {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "non-coplanar cut has one support-plane identity",
        });
    }
    Ok(ConstructionEdgeIdentity::Split { planes })
}

fn boundary_line_for_segment(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    first: &Point3,
    second: &Point3,
) -> HypermeshResult<ConstructionEdgeIdentity> {
    let identities =
        polygon
            .known_edge_identities()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "coplanar contact face has no edge identities",
            })?;
    let mut found = None;
    for edge in 0..polygon.edges.len() {
        let plane = &polygon.edges[(edge + 1) % polygon.edges.len()];
        if classify_point_decision(decisions, first, plane)? == Classification::On
            && classify_point_decision(decisions, second, plane)? == Classification::On
        {
            let identity =
                identities
                    .get(edge)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "coplanar contact edge identity is absent",
                    })?;
            if found.as_ref().is_none_or(|existing| identity < *existing) {
                found = Some(identity);
            }
        }
    }
    found.ok_or(HypermeshError::SurfaceArrangementFailed {
        reason: "coplanar contact segment is not on a source edge",
    })
}

#[derive(Clone)]
struct OverlayVertex {
    point: Point3,
    identity: ArrangementPointIdentity,
    outgoing: ConstructionEdgeIdentity,
}

struct IdentifiedPolygon {
    polygon: ConvexPolygon,
    plane_edges: Vec<ConstructionEdgeIdentity>,
}

impl IdentifiedPolygon {
    fn from_source(polygon: &ConvexPolygon) -> HypermeshResult<Self> {
        let outgoing =
            polygon
                .known_edge_identities()
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "coplanar overlap face has no edge identities",
                })?;
        if polygon.edges.len() != outgoing.len() || polygon.edges.len() < 3 {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "coplanar overlap edge planes and identities are not aligned",
            });
        }
        let mut plane_edges = Vec::new();
        plane_edges.try_reserve_exact(outgoing.len()).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "coplanar overlay edge identities",
            }
        })?;
        for plane in 0..outgoing.len() {
            // ConvexPolygon stores source edge i and its half-space plane at
            // the same index. Its projective vertex i is the intersection of
            // planes i and i + 1, so retaining that direct alignment is what
            // lets a clipped vertex recover the correct construction recipe.
            plane_edges.push(outgoing.get(plane).ok_or(
                HypermeshError::SurfaceArrangementFailed {
                    reason: "coplanar overlap edge identity is absent",
                },
            )?);
        }
        let mut polygon = polygon.clone();
        // Retained source vertices follow the source cycle, whereas the
        // projective representation names vertex i by edge planes i and i+1.
        // A clip clears the retained cycle; clear it up front as well so a
        // no-op (coincident/contained) overlay uses exactly the same indexing.
        polygon.known_vertices = None;
        polygon.known_identities = None;
        polygon.approx_bounds = None;
        Ok(Self {
            polygon,
            plane_edges,
        })
    }

    fn clip_negative(
        mut self,
        decisions: &DecisionContext,
        plane: &Plane,
        plane_edge: ConstructionEdgeIdentity,
    ) -> HypermeshResult<Option<Self>> {
        let count = self.polygon.vertex_count();
        let mut classifications = Vec::new();
        classifications
            .try_reserve_exact(count)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "coplanar overlay classifications",
            })?;
        let mut has_negative = false;
        let mut has_positive = false;
        for vertex in 0..count {
            let classification =
                classify_projective_point_decision(decisions, &self.polygon.vertex(vertex), plane)?;
            has_negative |= classification == Classification::Negative;
            has_positive |= classification == Classification::Positive;
            classifications.push(classification);
        }
        if !has_positive {
            return Ok(Some(self));
        }
        if !has_negative {
            return Ok(None);
        }

        let mut edges = Vec::new();
        let mut identities = Vec::new();
        edges
            .try_reserve(count + 1)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "coplanar overlay edge planes",
            })?;
        identities
            .try_reserve(count + 1)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "coplanar overlay edge identities",
            })?;
        for index in 0..count {
            let next = (index + 1) % count;
            let segment_plane = self.polygon.edges[next].clone();
            let segment_identity = self.plane_edges[next].clone();
            match (
                classifications[index].is_non_positive(),
                classifications[next].is_non_positive(),
            ) {
                (true, true) | (false, true) => {
                    edges.push(segment_plane);
                    identities.push(segment_identity);
                }
                (true, false) => {
                    edges.push(segment_plane);
                    identities.push(segment_identity);
                    edges.push(plane.clone());
                    identities.push(plane_edge.clone());
                }
                (false, false) => {}
            }
        }
        if edges.len() < 3 || edges.len() != identities.len() {
            return Ok(None);
        }
        self.polygon.edges = Arc::new(edges);
        self.polygon.known_vertices = None;
        self.polygon.known_identities = None;
        self.polygon.approx_bounds = None;
        self.plane_edges = identities;
        Ok(Some(self))
    }
}

fn coplanar_overlay(
    decisions: &DecisionContext,
    left_face: usize,
    left: &ConvexPolygon,
    _right_face: usize,
    right: &ConvexPolygon,
) -> HypermeshResult<Vec<OverlayVertex>> {
    let right_edges =
        right
            .known_edge_identities()
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "coplanar overlap face has no edge identities",
            })?;
    if right.edges.len() != right_edges.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "coplanar overlap clip planes and identities are not aligned",
        });
    }
    let mut overlap = IdentifiedPolygon::from_source(left)?;
    for edge in 0..right.edges.len() {
        overlap = overlap
            .clip_negative(
                decisions,
                &right.edges[edge],
                right_edges
                    .get(edge)
                    .ok_or(HypermeshError::SurfaceArrangementFailed {
                        reason: "coplanar overlap clip identity is absent",
                    })?,
            )?
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "positive-area overlap clipped to an empty region",
            })?;
    }
    let points = overlap.polygon.vertices_decision(decisions)?;
    if points.len() < 3 || points.len() != overlap.plane_edges.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "positive-area overlap has no planar boundary cycle",
        });
    }
    let support =
        pairwise_support_identity(left_face)?.ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "coplanar overlap face has no support identity",
        })?;
    let mut result = Vec::new();
    result
        .try_reserve_exact(points.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "coplanar overlay vertices",
        })?;
    for (index, point) in points.into_iter().enumerate() {
        let next = (index + 1) % overlap.plane_edges.len();
        result.push(OverlayVertex {
            point,
            identity: intersect_arrangement_lines(
                &overlap.plane_edges[index],
                &overlap.plane_edges[next],
                support,
            )?,
            outgoing: overlap.plane_edges[next].clone(),
        });
    }
    Ok(result)
}

fn intersect_arrangement_lines(
    first: &ConstructionEdgeIdentity,
    second: &ConstructionEdgeIdentity,
    support: ConstructionPlaneIdentity,
) -> HypermeshResult<ArrangementPointIdentity> {
    if first == second {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "adjacent arrangement edges have one construction identity",
        });
    }
    if let (
        ConstructionEdgeIdentity::Source {
            mesh: first_mesh,
            endpoints: first_endpoints,
        },
        ConstructionEdgeIdentity::Source {
            mesh: second_mesh,
            endpoints: second_endpoints,
        },
    ) = (first, second)
        && first_mesh == second_mesh
        && let Some(vertex) = first_endpoints
            .iter()
            .find(|vertex| second_endpoints.contains(vertex))
    {
        return Ok(ArrangementPointIdentity::Construction(
            ConstructionVertexIdentity::Source {
                mesh: *first_mesh,
                vertex: *vertex,
            },
        ));
    }
    let source_split = |source: &ConstructionEdgeIdentity,
                        split: &ConstructionEdgeIdentity|
     -> Option<ConstructionVertexIdentity> {
        let ConstructionEdgeIdentity::Split { planes } = split else {
            return None;
        };
        let other = if planes[0] == support {
            planes[1]
        } else if planes[1] == support {
            planes[0]
        } else {
            return None;
        };
        Some(source.intersection_identity(other))
    };
    if matches!(first, ConstructionEdgeIdentity::Source { .. })
        && let Some(identity) = source_split(first, second)
    {
        return Ok(ArrangementPointIdentity::Construction(identity));
    }
    if matches!(second, ConstructionEdgeIdentity::Source { .. })
        && let Some(identity) = source_split(second, first)
    {
        return Ok(ArrangementPointIdentity::Construction(identity));
    }
    if let (
        ConstructionEdgeIdentity::Split { planes: first },
        ConstructionEdgeIdentity::Split { planes: second },
    ) = (first, second)
        && first.contains(&support)
        && second.contains(&support)
    {
        let first_other = if first[0] == support {
            first[1]
        } else {
            first[0]
        };
        let second_other = if second[0] == support {
            second[1]
        } else {
            second[0]
        };
        if first_other != second_other {
            let mut planes = [support, first_other, second_other];
            planes.sort_unstable();
            return Ok(ArrangementPointIdentity::Construction(
                ConstructionVertexIdentity::PlaneTriple { planes },
            ));
        }
    }
    let mut edges = [first.clone(), second.clone()];
    edges.sort_unstable();
    Ok(ArrangementPointIdentity::CoplanarEdges(edges))
}

struct FaceResult {
    triangles: Vec<[u32; 3]>,
    constraints: Vec<[u32; 2]>,
    contacts: Vec<u32>,
}

fn corefine_face(
    decisions: &DecisionContext,
    face: usize,
    polygon: &ConvexPolygon,
    work: &FaceWork,
    arena: &mut ArrangementPointArena,
) -> HypermeshResult<FaceResult> {
    let mut constraint_lines = BTreeMap::<[u32; 2], ConstructionEdgeIdentity>::new();
    for constraint in &work.constraints {
        let endpoints = sorted_edge(constraint.endpoints);
        if endpoints[0] == endpoints[1] {
            continue;
        }
        constraint_lines
            .entry(endpoints)
            .and_modify(|line| {
                if constraint.line < *line {
                    *line = constraint.line.clone();
                }
            })
            .or_insert_with(|| constraint.line.clone());
    }
    let mut point_ids = BTreeSet::new();
    point_ids.extend(work.boundary.iter().copied());
    point_ids.extend(work.contacts.iter().copied());
    for edge in constraint_lines.keys() {
        point_ids.extend(edge);
    }
    let projection_axis = projection_axis(decisions, &polygon.support)?;
    let axes =
        projection_axes(projection_axis).ok_or(HypermeshError::SurfaceArrangementFailed {
            reason: "source face projection axis is invalid",
        })?;
    let mut projected = BTreeMap::new();
    for &point in &point_ids {
        let materialized =
            arena
                .points
                .get(point as usize)
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "face constraint references an absent point",
                })?;
        if classify_point_decision(decisions, materialized, &polygon.support)? != Classification::On
            || polygon.edges.iter().try_fold(false, |outside, edge| {
                Ok::<_, HypermeshError>(
                    outside
                        || classify_point_decision(decisions, materialized, edge)?
                            == Classification::Positive,
                )
            })?
        {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "face arrangement point lies outside its source face",
            });
        }
        projected.insert(point, project_point(materialized, axes));
    }

    let authored = constraint_lines
        .iter()
        .map(|(&endpoints, line)| RawConstraint {
            endpoints,
            line: line.clone(),
        })
        .collect::<Vec<_>>();
    for left_index in 0..authored.len() {
        let left = &authored[left_index];
        for right in &authored[(left_index + 1)..] {
            if left
                .endpoints
                .iter()
                .any(|point| right.endpoints.contains(point))
            {
                continue;
            }
            let left_points = left.endpoints.map(|point| {
                projected
                    .get(&point)
                    .expect("constraint endpoint is projected")
            });
            let right_points = right.endpoints.map(|point| {
                projected
                    .get(&point)
                    .expect("constraint endpoint is projected")
            });
            if !segments_properly_cross(decisions, left_points, right_points)? {
                continue;
            }
            let intersection = decisions
                .decide(
                    hyperlimit::proper_segment_intersection_point(
                        &limit_point(left_points[0]),
                        &limit_point(left_points[1]),
                        &limit_point(right_points[0]),
                        &limit_point(right_points[1]),
                        decisions.policy(),
                    ),
                    "surface arrangement proper segment intersection",
                )?
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "proper face constraints have no intersection point",
                })?;
            let planar = hypertri::ExactPoint::new(intersection.x, intersection.y);
            let point = lift_planar_point(&planar, &polygon.support, projection_axis, axes)?;
            if classify_point_decision(decisions, &point, &polygon.support)? != Classification::On {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "lifted face crossing does not lie on its source support",
                });
            }
            let support = pairwise_support_identity(face)?.ok_or(
                HypermeshError::SurfaceArrangementFailed {
                    reason: "source face has no operation-local support identity",
                },
            )?;
            let identity = intersect_arrangement_lines(&left.line, &right.line, support)?;
            let point_id = arena.insert(decisions, identity, point)?;
            projected
                .entry(point_id)
                .or_insert_with(|| project_point(&arena.points[point_id as usize], axes));
        }
    }

    let mut split_lines = BTreeMap::<[u32; 2], ConstructionEdgeIdentity>::new();
    for constraint in &authored {
        let mut on_segment = Vec::new();
        for (&point_id, point) in &projected {
            if planar_point_on_segment(
                decisions,
                [
                    projected
                        .get(&constraint.endpoints[0])
                        .expect("constraint endpoint is projected"),
                    projected
                        .get(&constraint.endpoints[1])
                        .expect("constraint endpoint is projected"),
                ],
                point,
            )? {
                on_segment.push(point_id);
            }
        }
        sort_point_ids_on_segment(decisions, &projected, constraint.endpoints, &mut on_segment)?;
        for pair in on_segment.windows(2) {
            let edge = sorted_edge([pair[0], pair[1]]);
            if edge[0] != edge[1] {
                split_lines
                    .entry(edge)
                    .and_modify(|line| {
                        if constraint.line < *line {
                            *line = constraint.line.clone();
                        }
                    })
                    .or_insert_with(|| constraint.line.clone());
            }
        }
    }

    let boundary_edges = work
        .boundary
        .iter()
        .copied()
        .zip(work.boundary.iter().copied().cycle().skip(1))
        .take(work.boundary.len())
        .map(|edge| sorted_edge([edge.0, edge.1]))
        .collect::<BTreeSet<_>>();
    let only_source_boundary = projected.len() == work.boundary.len()
        && split_lines.keys().copied().collect::<BTreeSet<_>>() == boundary_edges;
    let mut contacts = work.contacts.clone();
    contacts.sort_unstable();
    contacts.dedup();
    if !work.changed || only_source_boundary {
        return Ok(FaceResult {
            triangles: triangulate_convex_boundary(&work.boundary),
            constraints: boundary_edges.into_iter().collect(),
            contacts,
        });
    }

    let point_ids = projected.keys().copied().collect::<Vec<_>>();
    let local = point_ids
        .iter()
        .enumerate()
        .map(|(local, &global)| (global, local))
        .collect::<BTreeMap<_, _>>();
    let points = point_ids
        .iter()
        .map(|point| projected[point].clone())
        .collect::<Vec<_>>();
    let constraints = split_lines
        .keys()
        .map(|edge| hypertri::Constraint::new(local[&edge[0]], local[&edge[1]]))
        .collect::<Vec<_>>();
    let context = hypertri::TriangulationContext::new(decisions.policy());
    let outcome = hypertri::cdt::constrained_delaunay_convex_hull(&context, &points, &constraints)
        .map_err(map_triangulation_error)?;
    decisions.absorb(match outcome.certainty {
        hypertri::TriangulationCertainty::Certified => MeshCertainty::Certified,
        hypertri::TriangulationCertainty::Approximate512Consumed => {
            MeshCertainty::Approximate512Consumed
        }
    });
    if outcome.value.points().len() != points.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "preplanarized face constraints produced an unexpected Steiner point",
        });
    }
    let source_positive = source_projection_is_positive(decisions, &projected, &work.boundary)?;
    let mut triangles = Vec::new();
    triangles
        .try_reserve_exact(outcome.value.triangles().len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "surface arrangement face triangles",
        })?;
    for triangle in outcome.value.triangles() {
        let mut triangle = triangle.map(|vertex| point_ids[vertex]);
        let triangle_positive = match planar_orientation(
            decisions,
            &projected[&triangle[0]],
            &projected[&triangle[1]],
            &projected[&triangle[2]],
        )? {
            Classification::Positive => true,
            Classification::Negative => false,
            Classification::On => {
                return Err(HypermeshError::SurfaceArrangementFailed {
                    reason: "bounded face triangulation produced a degenerate triangle",
                });
            }
        };
        if triangle_positive != source_positive {
            triangle.swap(1, 2);
        }
        triangles.push(triangle);
    }
    let constraints = outcome
        .value
        .constraint_edges()
        .iter()
        .map(|constraint| sorted_edge([point_ids[constraint.from], point_ids[constraint.to]]))
        .collect::<BTreeSet<_>>();
    let expected_constraints = split_lines.keys().copied().collect::<BTreeSet<_>>();
    let triangulation_edges = triangles
        .iter()
        .flat_map(|triangle| {
            [
                sorted_edge([triangle[0], triangle[1]]),
                sorted_edge([triangle[1], triangle[2]]),
                sorted_edge([triangle[2], triangle[0]]),
            ]
        })
        .collect::<BTreeSet<_>>();
    if constraints != expected_constraints
        || !constraints
            .iter()
            .all(|constraint| triangulation_edges.contains(constraint))
    {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "bounded face triangulation did not preserve every exact constraint",
        });
    }
    Ok(FaceResult {
        triangles,
        constraints: constraints.into_iter().collect(),
        contacts,
    })
}

fn triangulate_convex_boundary(boundary: &[u32]) -> Vec<[u32; 3]> {
    (1..boundary.len().saturating_sub(1))
        .map(|index| [boundary[0], boundary[index], boundary[index + 1]])
        .collect()
}

fn source_projection_is_positive(
    decisions: &DecisionContext,
    projected: &BTreeMap<u32, hypertri::ExactPoint>,
    boundary: &[u32],
) -> HypermeshResult<bool> {
    for index in 1..boundary.len().saturating_sub(1) {
        match planar_orientation(
            decisions,
            &projected[&boundary[0]],
            &projected[&boundary[index]],
            &projected[&boundary[index + 1]],
        )? {
            Classification::Positive => return Ok(true),
            Classification::Negative => return Ok(false),
            Classification::On => {}
        }
    }
    Err(HypermeshError::SurfaceArrangementFailed {
        reason: "source face projection is degenerate",
    })
}

fn segments_properly_cross(
    decisions: &DecisionContext,
    left: [&hypertri::ExactPoint; 2],
    right: [&hypertri::ExactPoint; 2],
) -> HypermeshResult<bool> {
    let opposite = |first, second| {
        matches!(
            (first, second),
            (Classification::Negative, Classification::Positive)
                | (Classification::Positive, Classification::Negative)
        )
    };
    let right_sides = [
        planar_orientation(decisions, left[0], left[1], right[0])?,
        planar_orientation(decisions, left[0], left[1], right[1])?,
    ];
    if !opposite(right_sides[0], right_sides[1]) {
        return Ok(false);
    }
    let left_sides = [
        planar_orientation(decisions, right[0], right[1], left[0])?,
        planar_orientation(decisions, right[0], right[1], left[1])?,
    ];
    Ok(opposite(left_sides[0], left_sides[1]))
}

fn planar_point_on_segment(
    decisions: &DecisionContext,
    edge: [&hypertri::ExactPoint; 2],
    point: &hypertri::ExactPoint,
) -> HypermeshResult<bool> {
    decisions.decide(
        hyperlimit::point_on_segment(
            &limit_point(edge[0]),
            &limit_point(edge[1]),
            &limit_point(point),
            decisions.policy(),
        ),
        "surface arrangement point on segment",
    )
}

fn sort_point_ids_on_segment(
    decisions: &DecisionContext,
    points: &BTreeMap<u32, hypertri::ExactPoint>,
    edge: [u32; 2],
    indices: &mut [u32],
) -> HypermeshResult<()> {
    let use_x =
        !compare_real_decision(decisions, &points[&edge[0]].x, &points[&edge[1]].x)?.is_eq();
    for index in 1..indices.len() {
        let mut cursor = index;
        while cursor > 0 {
            let left = &points[&indices[cursor]];
            let right = &points[&indices[cursor - 1]];
            let ordering = if use_x {
                compare_real_decision(decisions, &left.x, &right.x)?
            } else {
                compare_real_decision(decisions, &left.y, &right.y)?
            };
            if !ordering.is_lt() {
                break;
            }
            indices.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    Ok(())
}

fn planar_orientation(
    decisions: &DecisionContext,
    a: &hypertri::ExactPoint,
    b: &hypertri::ExactPoint,
    c: &hypertri::ExactPoint,
) -> HypermeshResult<Classification> {
    let sign = decisions.decide(
        hyperlimit::orient2(
            &limit_point(a),
            &limit_point(b),
            &limit_point(c),
            decisions.policy(),
        ),
        "surface arrangement orientation",
    )?;
    Ok(match sign {
        hyperlimit::Sign::Negative => Classification::Negative,
        hyperlimit::Sign::Zero => Classification::On,
        hyperlimit::Sign::Positive => Classification::Positive,
    })
}

fn limit_point(point: &hypertri::ExactPoint) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(point.x.clone(), point.y.clone())
}

fn projection_axis(decisions: &DecisionContext, plane: &Plane) -> HypermeshResult<usize> {
    let normal = [&plane.normal.x, &plane.normal.y, &plane.normal.z];
    if let Some(axis) = normal.iter().position(|component| {
        component
            .exact_rational_ref()
            .is_some_and(|value| !value.is_zero())
    }) {
        return Ok(axis);
    }
    for (axis, component) in normal.into_iter().enumerate() {
        if classify_real(decisions, component)? != Classification::On {
            return Ok(axis);
        }
    }
    Err(HypermeshError::SurfaceArrangementFailed {
        reason: "source face has a zero support normal",
    })
}

fn projection_axes(dropped: usize) -> Option<[usize; 2]> {
    match dropped {
        0 => Some([1, 2]),
        1 => Some([0, 2]),
        2 => Some([0, 1]),
        _ => None,
    }
}

fn point_axis(point: &Point3, axis: usize) -> &Real {
    match axis {
        0 => &point.x,
        1 => &point.y,
        _ => &point.z,
    }
}

fn project_point(point: &Point3, [u, v]: [usize; 2]) -> hypertri::ExactPoint {
    hypertri::ExactPoint::new(point_axis(point, u).clone(), point_axis(point, v).clone())
}

fn lift_planar_point(
    point: &hypertri::ExactPoint,
    plane: &Plane,
    dropped: usize,
    [u, v]: [usize; 2],
) -> HypermeshResult<Point3> {
    let normal = [&plane.normal.x, &plane.normal.y, &plane.normal.z];
    let one = Real::one();
    let numerator = Real::signed_product_sum(
        [false; 3],
        [
            [normal[u], &point.x],
            [normal[v], &point.y],
            [&plane.offset, &one],
        ],
    );
    let coordinate =
        (numerator / normal[dropped]).map_err(|_| HypermeshError::SurfaceArrangementFailed {
            reason: "source face projection coefficient is zero",
        })?;
    Ok(match dropped {
        0 => Point3::new(coordinate, point.x.clone(), point.y.clone()),
        1 => Point3::new(point.x.clone(), coordinate, point.y.clone()),
        2 => Point3::new(point.x.clone(), point.y.clone(), coordinate),
        _ => {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "source face projection axis is invalid",
            });
        }
    })
}

fn map_triangulation_error(error: hypertri::Error) -> HypermeshError {
    match error {
        hypertri::Error::PredicateUndecided { predicate } => {
            HypermeshError::PredicateUndecided { predicate }
        }
        hypertri::Error::InvalidInput { reason } => {
            HypermeshError::SurfaceArrangementFailed { reason }
        }
        hypertri::Error::NoEarFound => HypermeshError::SurfaceArrangementFailed {
            reason: "bounded face triangulation found no ear",
        },
        hypertri::Error::UnsupportedFeature { feature } => {
            HypermeshError::SurfaceArrangementFailed { reason: feature }
        }
    }
}

fn exact_rational_points_contradict(left: &Point3, right: &Point3) -> bool {
    let left = [&left.x, &left.y, &left.z].map(Real::exact_rational_ref);
    let right = [&right.x, &right.y, &right.z].map(Real::exact_rational_ref);
    match (left, right) {
        ([Some(lx), Some(ly), Some(lz)], [Some(rx), Some(ry), Some(rz)]) => {
            lx != rx || ly != ry || lz != rz
        }
        _ => false,
    }
}

fn sorted_edge(mut edge: [u32; 2]) -> [u32; 2] {
    edge.sort_unstable();
    edge
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::MeshContext;
    use crate::intersection::pairwise_intersections_by_polygon;
    use crate::test_support::approximate_convex_triangle;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn triangle(
        points: [Point3; 3],
        mesh: usize,
        face: usize,
        vertices: [usize; 3],
    ) -> ConvexPolygon {
        let mut polygon = approximate_convex_triangle(
            &points[0],
            &points[1],
            &points[2],
            mesh as isize,
            face as isize,
        );
        polygon
            .set_source_triangle_edge_identities(mesh, vertices)
            .unwrap();
        polygon
    }

    fn triangle_edges(triangle: [u32; 3]) -> [[u32; 2]; 3] {
        [
            sorted_edge([triangle[0], triangle[1]]),
            sorted_edge([triangle[1], triangle[2]]),
            sorted_edge([triangle[2], triangle[0]]),
        ]
    }

    fn assert_constraints_are_edges(surface: &SurfaceCorefinement, face: usize) {
        let edges = surface
            .face_triangles(face)
            .iter()
            .flat_map(|triangle| triangle_edges(*triangle))
            .collect::<BTreeSet<_>>();
        assert!(
            surface
                .face_constraints(face)
                .iter()
                .all(|constraint| edges.contains(constraint))
        );
    }

    fn assert_face_result_constraints_are_edges(result: &FaceResult) {
        let edges = result
            .triangles
            .iter()
            .flat_map(|triangle| triangle_edges(*triangle))
            .collect::<BTreeSet<_>>();
        assert!(
            result
                .constraints
                .iter()
                .all(|constraint| edges.contains(constraint))
        );
    }

    fn test_split_line(face: usize, plane: u32) -> ConstructionEdgeIdentity {
        let mut planes = [
            pairwise_support_identity(face).unwrap().unwrap(),
            ConstructionPlaneIdentity {
                mesh: u32::MAX - 1,
                plane,
            },
        ];
        planes.sort_unstable();
        ConstructionEdgeIdentity::Split { planes }
    }

    fn insert_test_point(
        decisions: &DecisionContext,
        arena: &mut ArrangementPointArena,
        vertex: &mut u32,
        point: Point3,
    ) -> u32 {
        let identity = ArrangementPointIdentity::Construction(ConstructionVertexIdentity::Source {
            mesh: u32::MAX - 2,
            vertex: *vertex,
        });
        *vertex += 1;
        arena.insert(decisions, identity, point).unwrap()
    }

    #[test]
    fn bounded_face_pslg_keeps_disconnected_inner_cells() {
        let polygons = vec![
            triangle([p(0, 0, 0), p(20, 0, 0), p(0, 20, 0)], 0, 0, [0, 1, 2]),
            triangle([p(2, 2, 0), p(4, 2, 0), p(2, 4, 0)], 1, 1, [0, 1, 2]),
            triangle([p(8, 2, 0), p(10, 2, 0), p(8, 4, 0)], 2, 2, [0, 1, 2]),
        ];

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let graph = pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
            let surface = corefine_surface(&decisions, &polygons, &graph).unwrap();

            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
            assert_eq!(surface.face_triangles(0).len(), 13);
            assert_eq!(surface.face_constraints(0).len(), 9);
            assert!(surface.face_contacts(0).is_empty());
            assert_constraints_are_edges(&surface, 0);
        }
    }

    #[test]
    fn crossing_face_constraints_share_one_triple_point() {
        let polygons = vec![
            triangle([p(0, 0, 0), p(20, 0, 0), p(0, 20, 0)], 0, 0, [0, 1, 2]),
            triangle([p(5, -2, -2), p(5, 20, -2), p(5, -2, 2)], 1, 1, [0, 1, 2]),
            triangle([p(-2, 5, -2), p(20, 5, -2), p(-2, 5, 2)], 2, 2, [0, 1, 2]),
        ];
        let decisions = crate::test_support::approximate_decisions();
        let graph = pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
        let surface = corefine_surface(&decisions, &polygons, &graph).unwrap();

        let center = surface
            .points
            .iter()
            .position(|point| point == &p(5, 5, 0))
            .expect("the two cuts have one exact crossing");
        assert!(matches!(
            &surface.identities[center],
            ArrangementPointIdentity::Construction(ConstructionVertexIdentity::PlaneTriple { .. })
        ));
        assert!(surface.face_triangles(0).len() >= 8);
        assert_constraints_are_edges(&surface, 0);
    }

    #[test]
    fn opposite_winding_coincident_faces_reuse_one_point_set() {
        let first = triangle([p(0, 0, 0), p(6, 0, 0), p(0, 6, 0)], 0, 0, [0, 1, 2]);
        let second = triangle([p(0, 6, 0), p(6, 0, 0), p(0, 0, 0)], 1, 1, [0, 1, 2]);
        let polygons = [first, second];
        let decisions = crate::test_support::approximate_decisions();
        let graph = pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
        let surface = corefine_surface(&decisions, &polygons, &graph).unwrap();

        let canonical = |face: usize| {
            let mut triangles = surface.face_triangles(face).to_vec();
            for triangle in &mut triangles {
                triangle.sort_unstable();
            }
            triangles.sort_unstable();
            triangles
        };
        assert_eq!(surface.points.len(), 3);
        assert_eq!(canonical(0), canonical(1));
        assert_constraints_are_edges(&surface, 0);
        assert_constraints_are_edges(&surface, 1);
    }

    #[test]
    fn partial_coplanar_overlap_reuses_the_same_bounded_cell_triangulation() {
        let polygons = [
            triangle([p(0, 0, 0), p(6, 0, 0), p(0, 6, 0)], 0, 0, [0, 1, 2]),
            triangle([p(5, 5, 0), p(-1, 5, 0), p(5, -1, 0)], 1, 1, [0, 1, 2]),
        ];
        let overlap_points = [
            p(4, 0, 0),
            p(5, 0, 0),
            p(5, 1, 0),
            p(1, 5, 0),
            p(0, 5, 0),
            p(0, 4, 0),
        ];

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let graph = pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
            let surface = corefine_surface(&decisions, &polygons, &graph).unwrap();
            let overlap = surface
                .points
                .iter()
                .enumerate()
                .filter_map(|(index, point)| {
                    overlap_points
                        .contains(point)
                        .then_some(u32::try_from(index).unwrap())
                })
                .collect::<BTreeSet<_>>();
            assert_eq!(overlap.len(), overlap_points.len());

            let canonical_overlap = |face: usize| {
                let mut triangles = surface
                    .face_triangles(face)
                    .iter()
                    .copied()
                    .filter(|triangle| triangle.iter().all(|point| overlap.contains(point)))
                    .collect::<Vec<_>>();
                for triangle in &mut triangles {
                    triangle.sort_unstable();
                }
                triangles.sort_unstable();
                triangles
            };
            let left = canonical_overlap(0);
            let right = canonical_overlap(1);
            assert_eq!(left.len(), overlap_points.len() - 2);
            assert_eq!(left, right);
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
            assert_constraints_are_edges(&surface, 0);
            assert_constraints_are_edges(&surface, 1);
        }
    }

    #[test]
    fn collinear_overlaps_and_t_junctions_split_into_one_edge_set() {
        let polygon = triangle([p(0, 0, 0), p(24, 0, 0), p(0, 24, 0)], 0, 0, [0, 1, 2]);
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let mut arena = ArrangementPointArena::with_capacity(11).unwrap();
            let mut work = FaceWork::default();
            add_source_boundary(&decisions, 0, &polygon, &mut arena, &mut work).unwrap();
            let mut vertex = 0;
            for (line, endpoints) in [
                (0, [p(2, 5, 0), p(18, 5, 0)]),
                (1, [p(8, 5, 0), p(16, 5, 0)]),
                (2, [p(10, 2, 0), p(10, 5, 0)]),
                (3, [p(12, 5, 0), p(12, 8, 0)]),
            ] {
                let endpoints = endpoints
                    .map(|point| insert_test_point(&decisions, &mut arena, &mut vertex, point));
                work.constraints.push(RawConstraint {
                    endpoints,
                    line: test_split_line(0, line),
                });
            }
            work.changed = true;

            let result = corefine_face(&decisions, 0, &polygon, &work, &mut arena).unwrap();
            assert_eq!(arena.points.len(), 11);
            assert_eq!(result.constraints.len(), 10);
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
            assert_face_result_constraints_are_edges(&result);
        }
    }

    #[test]
    fn dense_crossing_discovery_exhausts_the_finite_grid_without_a_pass_limit() {
        const LINE_COUNT: u32 = 17;
        let polygon = triangle([p(0, 0, 0), p(40, 0, 0), p(0, 40, 0)], 0, 0, [0, 1, 2]);
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let mut arena = ArrangementPointArena::with_capacity(3).unwrap();
            let mut work = FaceWork::default();
            add_source_boundary(&decisions, 0, &polygon, &mut arena, &mut work).unwrap();
            work.constraints
                .try_reserve((LINE_COUNT * 2) as usize)
                .unwrap();
            let mut vertex = 0;
            for coordinate in 1..=LINE_COUNT {
                let endpoints = [p(coordinate.into(), 1, 0), p(coordinate.into(), 17, 0)]
                    .map(|point| insert_test_point(&decisions, &mut arena, &mut vertex, point));
                work.constraints.push(RawConstraint {
                    endpoints,
                    line: test_split_line(0, coordinate - 1),
                });
            }
            for coordinate in 1..=LINE_COUNT {
                let endpoints = [p(1, coordinate.into(), 0), p(17, coordinate.into(), 0)]
                    .map(|point| insert_test_point(&decisions, &mut arena, &mut vertex, point));
                work.constraints.push(RawConstraint {
                    endpoints,
                    line: test_split_line(0, LINE_COUNT + coordinate - 1),
                });
            }
            work.changed = true;

            let result = corefine_face(&decisions, 0, &polygon, &work, &mut arena).unwrap();
            assert_eq!(arena.points.len(), 3 + (LINE_COUNT * LINE_COUNT) as usize);
            assert_eq!(
                result.constraints.len(),
                3 + (2 * LINE_COUNT * (LINE_COUNT - 1)) as usize
            );
            assert_eq!(decisions.certainty(), MeshCertainty::Certified);
            assert_face_result_constraints_are_edges(&result);
        }
    }

    #[test]
    fn isolated_face_contact_is_retained_as_a_triangulation_vertex() {
        let polygon = triangle([p(0, 0, 0), p(8, 0, 0), p(0, 8, 0)], 0, 0, [0, 1, 2]);
        let decisions = crate::test_support::approximate_decisions();
        let mut arena = ArrangementPointArena::with_capacity(4).unwrap();
        let mut work = FaceWork::default();
        add_source_boundary(&decisions, 0, &polygon, &mut arena, &mut work).unwrap();
        let contact = arena
            .insert(
                &decisions,
                ArrangementPointIdentity::Construction(ConstructionVertexIdentity::PlaneTriple {
                    planes: [
                        ConstructionPlaneIdentity { mesh: 0, plane: 0 },
                        ConstructionPlaneIdentity { mesh: 1, plane: 0 },
                        ConstructionPlaneIdentity { mesh: 2, plane: 0 },
                    ],
                }),
                p(2, 2, 0),
            )
            .unwrap();
        work.contacts.push(contact);
        work.changed = true;

        let result = corefine_face(&decisions, 0, &polygon, &work, &mut arena).unwrap();
        assert_eq!(result.contacts, [contact]);
        assert_eq!(result.triangles.len(), 3);
        assert!(
            result
                .triangles
                .iter()
                .all(|triangle| triangle.contains(&contact))
        );
        assert_face_result_constraints_are_edges(&result);
    }

    #[test]
    fn point_aliasing_obeys_strict_and_approximate_512_terminal_policy() {
        let left = p(0, 0, 0);
        let mut symbolic_left = left.clone();
        symbolic_left.x = Real::pi() + Real::e();
        let mut symbolic_right = left;
        symbolic_right.x = Real::e() + Real::pi();
        let first_identity =
            ArrangementPointIdentity::Construction(ConstructionVertexIdentity::Source {
                mesh: 0,
                vertex: 0,
            });
        let second_identity =
            ArrangementPointIdentity::Construction(ConstructionVertexIdentity::Source {
                mesh: 1,
                vertex: 0,
            });

        let strict_context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let strict = DecisionContext::new(&strict_context);
        let mut strict_arena = ArrangementPointArena::with_capacity(2).unwrap();
        strict_arena
            .insert(&strict, first_identity.clone(), symbolic_left.clone())
            .unwrap();
        assert!(matches!(
            strict_arena.insert(&strict, second_identity.clone(), symbolic_right.clone()),
            Err(HypermeshError::PredicateUndecided { .. })
        ));

        let approximate_context = MeshContext::new(hyperlimit::PredicatePolicy::APPROXIMATE_512);
        let approximate = DecisionContext::new(&approximate_context);
        let mut approximate_arena = ArrangementPointArena::with_capacity(2).unwrap();
        let first = approximate_arena
            .insert(&approximate, first_identity, symbolic_left)
            .unwrap();
        let second = approximate_arena
            .insert(&approximate, second_identity, symbolic_right)
            .unwrap();
        assert_eq!(first, second);
        assert_eq!(
            approximate.certainty(),
            MeshCertainty::Approximate512Consumed
        );
    }
}
