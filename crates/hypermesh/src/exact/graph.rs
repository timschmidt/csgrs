//! Exact intersection graph event extraction.
//!
//! The graph here is intentionally an event graph, not yet a mutable boolean
//! topology. It converts certified face-pair classifications into stable
//! records for split points, coplanar edge contacts, containment facts, and
//! unresolved predicate outcomes. This is the next layer in Yap's exact
//! geometric computation split: predicates and constructions produce auditable
//! events first; mesh mutation consumes those events only after validation.
//! See Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7.1-2 (1997).
//!
//! The event categories follow the triangle/triangle decomposition used by
//! Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap Test Using
//! Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003): reject by
//! plane side, retain non-coplanar segment/plane crossings, and handle
//! coplanar overlap through projected segment and containment predicates.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{
    Point3, SegmentIntersection, Sign, TriangleLocation, compare_reals, orient3d_report,
};

use super::construction::{SegmentPlaneIntersection, SegmentPlaneRelation};
use super::coplanar::{CoplanarProjection, CoplanarTriangleClassification};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::intersection::{
    MeshFacePairClassification, MeshFacePairRelation, classify_mesh_face_pairs,
};
use super::mesh::ExactMesh;
use super::scalar::ExactReal;

/// Side of a two-mesh graph event.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeshSide {
    /// The first mesh passed to graph construction.
    Left,
    /// The second mesh passed to graph construction.
    Right,
}

/// Exact intersection event extracted from a retained face pair.
#[derive(Clone, Debug, PartialEq)]
pub enum IntersectionEvent {
    /// A triangle edge intersects the opposite triangle plane.
    SegmentPlane {
        /// Mesh owning the segment edge.
        segment_side: MeshSide,
        /// Edge endpoints in that mesh's vertex index space.
        edge: [usize; 2],
        /// Mesh owning the plane face.
        plane_side: MeshSide,
        /// Face index of the plane triangle.
        plane_face: usize,
        /// Coarse segment/plane relation.
        relation: SegmentPlaneRelation,
        /// Exact point for endpoint and proper-crossing events.
        point: Option<Point3>,
        /// Exact edge parameter when available.
        parameter: Option<ExactReal>,
    },
    /// A projected coplanar edge-pair relation.
    CoplanarEdge {
        /// Edge in the left mesh.
        left_edge: [usize; 2],
        /// Edge in the right mesh.
        right_edge: [usize; 2],
        /// Exact projected segment relation.
        relation: SegmentIntersection,
    },
    /// A projected coplanar vertex containment fact.
    CoplanarVertex {
        /// Mesh owning the tested vertex.
        vertex_side: MeshSide,
        /// Vertex index in that mesh.
        vertex: usize,
        /// Mesh owning the containing face.
        triangle_side: MeshSide,
        /// Face index in the containing mesh.
        triangle_face: usize,
        /// Exact projected point/triangle location.
        location: TriangleLocation,
    },
    /// A retained pair could not be completely decided.
    Unknown,
}

/// Event records for one retained face pair.
#[derive(Clone, Debug, PartialEq)]
pub struct FacePairEvents {
    /// Face index in the left mesh.
    pub left_face: usize,
    /// Face index in the right mesh.
    pub right_face: usize,
    /// Coarse relation that caused retention.
    pub relation: MeshFacePairRelation,
    /// Projection used by coplanar events, if any.
    pub projection: Option<CoplanarProjection>,
    /// Extracted exact events.
    pub events: Vec<IntersectionEvent>,
}

impl FacePairEvents {
    /// Return whether the pair contains at least one event that can drive graph
    /// construction.
    pub fn has_constructive_events(&self) -> bool {
        self.events.iter().any(|event| {
            !matches!(
                event,
                IntersectionEvent::CoplanarEdge {
                    relation: SegmentIntersection::Disjoint,
                    ..
                } | IntersectionEvent::CoplanarVertex {
                    location: TriangleLocation::Outside | TriangleLocation::Degenerate,
                    ..
                }
            )
        })
    }
}

/// Exact intersection event graph for two meshes.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactIntersectionGraph {
    /// Retained face-pair event records.
    pub face_pairs: Vec<FacePairEvents>,
}

impl ExactIntersectionGraph {
    /// Count all retained events.
    pub fn event_count(&self) -> usize {
        self.face_pairs.iter().map(|pair| pair.events.len()).sum()
    }

    /// Return whether any retained pair still needs a policy decision or
    /// additional refinement.
    pub fn has_unknowns(&self) -> bool {
        self.face_pairs.iter().any(|pair| {
            pair.relation == MeshFacePairRelation::Unknown
                || pair
                    .events
                    .iter()
                    .any(|event| matches!(event, IntersectionEvent::Unknown))
        })
    }

    /// Extract exact edge split parameters from segment/plane events.
    ///
    /// The plan keeps split points grouped by directed mesh edge. Parameters
    /// are sorted only through `hyperlimit::compare_reals`; unresolved
    /// comparisons are counted rather than replaced with a primitive-float
    /// fallback.
    pub fn edge_split_plan(&self) -> ExactEdgeSplitPlan {
        edge_split_plan(self)
    }

    /// Merge coincident exact split points into graph vertices.
    ///
    /// Equality is tested coordinate-by-coordinate through
    /// `hyperlimit::compare_reals`. Unknown comparisons do not merge points;
    /// they increment [`ExactGraphVertexPlan::unresolved_equalities`] so a
    /// caller can choose a refinement or unsupported-degeneracy policy.
    pub fn graph_vertex_plan(&self) -> ExactGraphVertexPlan {
        graph_vertex_plan(&self.edge_split_plan())
    }

    /// Build a non-mutating split-topology plan.
    ///
    /// The plan maps each split edge to an ordered chain from the original
    /// start vertex through merged exact graph vertices to the original end
    /// vertex. It is deliberately still a plan, not a halfedge mutation: exact
    /// geometric computation keeps certified event extraction separate from
    /// topological updates until all assumptions are validated. See Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997).
    pub fn split_topology_plan(&self) -> ExactSplitTopologyPlan {
        let edge_splits = self.edge_split_plan();
        let graph_vertices = graph_vertex_plan(&edge_splits);
        split_topology_plan(&edge_splits, &graph_vertices)
    }

    /// Build face-local split work items from the split topology plan.
    ///
    /// The result tells later triangulation which original face boundary edges
    /// gained graph vertices. It does not infer a polygonization or winding
    /// decision; those remain exact downstream steps.
    pub fn face_split_plan(&self) -> ExactFaceSplitPlan {
        face_split_plan(&self.split_topology_plan())
    }

    /// Build exact face-boundary geometry for later triangulation.
    ///
    /// This resolves face split work items into original and constructed
    /// boundary nodes with exact coordinates. It remains a pre-mutation handoff:
    /// no halfedges are created and no winding decision is inferred here. The
    /// separation mirrors Yap's exact-computation staging, where certified
    /// predicates and constructions are validated before combinatorial edits.
    pub fn face_split_geometry_plan(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<ExactFaceSplitGeometryPlan, MeshError> {
        let topology = self.split_topology_plan();
        let face_plan = face_split_plan(&topology);
        face_split_geometry_plan(left, right, &topology, &face_plan)
    }
}

/// Build an exact event graph from two exact meshes.
pub fn build_intersection_graph(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactIntersectionGraph, MeshError> {
    let classifications = classify_mesh_face_pairs(left, right)?;
    let face_pairs = classifications
        .iter()
        .map(|classification| events_for_face_pair(left, right, classification))
        .collect();
    Ok(ExactIntersectionGraph { face_pairs })
}

/// Exact split points for one directed mesh edge.
#[derive(Clone, Debug, PartialEq)]
pub struct EdgeSplit {
    /// Mesh side owning the edge.
    pub side: MeshSide,
    /// Directed edge endpoints in that mesh's vertex index space.
    pub edge: [usize; 2],
    /// Ordered split points when exact parameter comparisons were available.
    pub points: Vec<EdgeSplitPoint>,
}

/// One exact split point on an edge.
#[derive(Clone, Debug, PartialEq)]
pub struct EdgeSplitPoint {
    /// Face pair that produced the split.
    pub face_pair: [usize; 2],
    /// Opposite face whose plane produced the split.
    pub plane_face: usize,
    /// Exact parameter on the directed edge.
    pub parameter: ExactReal,
    /// Exact constructed point.
    pub point: Point3,
}

/// Edge split extraction result.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactEdgeSplitPlan {
    /// Per-edge split points.
    pub splits: Vec<EdgeSplit>,
    /// Number of parameter comparisons that could not be certified.
    pub unknown_orderings: usize,
}

impl ExactEdgeSplitPlan {
    /// Count split points across all edges.
    pub fn point_count(&self) -> usize {
        self.splits.iter().map(|split| split.points.len()).sum()
    }
}

/// One merged exact graph vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactGraphVertex {
    /// Representative exact point.
    pub point: Point3,
    /// Split-point uses that are exactly coincident with the representative.
    pub uses: Vec<ExactGraphVertexUse>,
}

/// One source use of a merged graph vertex.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactGraphVertexUse {
    /// Mesh side owning the split edge.
    pub side: MeshSide,
    /// Directed edge endpoints in that mesh's vertex index space.
    pub edge: [usize; 2],
    /// Face pair that produced the split.
    pub face_pair: [usize; 2],
    /// Opposite face whose plane produced the split.
    pub plane_face: usize,
}

/// Exact graph-vertex merge result.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactGraphVertexPlan {
    /// Merged graph vertices.
    pub vertices: Vec<ExactGraphVertex>,
    /// Equality checks that could not be certified.
    pub unresolved_equalities: usize,
}

/// One node in an ordered split-edge chain.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SplitEdgeNode {
    /// Original mesh vertex.
    OriginalVertex {
        /// Mesh side owning the original vertex.
        side: MeshSide,
        /// Vertex index in that mesh.
        vertex: usize,
    },
    /// Merged exact graph vertex.
    GraphVertex {
        /// Index in [`ExactSplitTopologyPlan::graph_vertices`].
        graph_vertex: usize,
    },
}

/// Ordered split chain for one original edge.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SplitEdgeChain {
    /// Mesh side owning the edge.
    pub side: MeshSide,
    /// Directed original edge.
    pub edge: [usize; 2],
    /// Chain from original start through split graph vertices to original end.
    pub nodes: Vec<SplitEdgeNode>,
}

/// Non-mutating exact split topology plan.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactSplitTopologyPlan {
    /// Merged exact graph vertices used by edge chains.
    pub graph_vertices: Vec<ExactGraphVertex>,
    /// Ordered edge chains to materialize.
    pub edge_chains: Vec<SplitEdgeChain>,
    /// Number of split points that could not be matched back to a graph vertex.
    pub unresolved_vertex_lookups: usize,
    /// Number of equality checks that could not be certified while merging.
    pub unresolved_equalities: usize,
    /// Number of edge parameter comparisons that could not be certified.
    pub unknown_orderings: usize,
}

impl ExactSplitTopologyPlan {
    /// Count new graph vertices referenced by all split edge chains.
    pub fn referenced_graph_vertices(&self) -> usize {
        self.edge_chains
            .iter()
            .flat_map(|chain| chain.nodes.iter())
            .filter(|node| matches!(node, SplitEdgeNode::GraphVertex { .. }))
            .count()
    }

    /// Validate the non-mutating split-topology contract.
    ///
    /// Yap's exact-geometric-computation model separates certified predicate
    /// events from combinatorial edits. This report is the handoff check: it
    /// rejects unresolved exact comparisons and malformed chain references
    /// before any future halfedge mutation can consume the plan. See Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997).
    pub fn validate(&self) -> SplitPlanValidationReport {
        validate_split_topology_plan(self)
    }
}

/// One split edge chain as used by an affected face.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FaceSplitEdge {
    /// Original face edge endpoints.
    pub edge: [usize; 2],
    /// Graph vertices on that edge in directed edge order.
    pub graph_vertices: Vec<usize>,
}

/// Face-local split work item.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FaceSplitPlan {
    /// Mesh side owning the face.
    pub side: MeshSide,
    /// Face index.
    pub face: usize,
    /// Split boundary edges for this face.
    pub edges: Vec<FaceSplitEdge>,
}

/// Non-mutating exact face split plan.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactFaceSplitPlan {
    /// Per-face split work items.
    pub faces: Vec<FaceSplitPlan>,
}

impl ExactFaceSplitPlan {
    /// Count graph-vertex references across all face work items.
    pub fn graph_vertex_references(&self) -> usize {
        self.faces
            .iter()
            .flat_map(|face| face.edges.iter())
            .map(|edge| edge.graph_vertices.len())
            .sum()
    }

    /// Validate face-local split work items against a split-topology plan.
    ///
    /// The face plan is still deliberately pre-triangulation: it only says
    /// which original face boundary edges were split by exact graph vertices.
    /// Validation keeps that narrow API honest by checking graph-vertex ranges,
    /// duplicate face-edge instructions, and that each referenced graph vertex
    /// has an exact source use on the requested face edge.
    pub fn validate_against_topology(
        &self,
        topology: &ExactSplitTopologyPlan,
    ) -> SplitPlanValidationReport {
        validate_face_split_plan(self, topology)
    }
}

/// Stable category for split-plan validation diagnostics.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SplitPlanDiagnosticKind {
    /// Exact parameter ordering could not be certified.
    UnknownOrdering,
    /// Exact split-point equality could not be certified.
    UnresolvedEquality,
    /// A split point could not be matched to a graph vertex.
    UnresolvedVertexLookup,
    /// A split chain has no usable endpoint-to-endpoint path.
    EmptyOrShortEdgeChain,
    /// A split chain does not begin at its directed edge start.
    WrongChainStart,
    /// A split chain does not end at its directed edge end.
    WrongChainEnd,
    /// An original vertex node appears on the wrong mesh side.
    ChainSideMismatch,
    /// A graph-vertex reference is out of range.
    GraphVertexOutOfRange,
    /// A merged graph vertex has no source uses.
    EmptyGraphVertexUses,
    /// A face split work item has no split edges.
    EmptyFaceSplit,
    /// A face split edge has no graph vertices.
    EmptyFaceSplitEdge,
    /// A face split plan repeats the same original edge for one face.
    DuplicateFaceSplitEdge,
    /// A face split edge references a graph vertex with no matching source use.
    MissingFaceSplitSourceUse,
    /// Boundary incidence against the original face plane could not be decided.
    UnknownBoundaryIncidence,
    /// A split boundary node is not on the original face plane.
    BoundaryNodeOffFacePlane,
    /// A split face region has fewer than three boundary nodes.
    EmptyOrShortRegionBoundary,
    /// A split face region contains consecutive duplicate boundary nodes.
    DuplicateConsecutiveRegionNode,
}

/// One split-plan validation diagnostic.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SplitPlanDiagnostic {
    /// Stable diagnostic category.
    pub kind: SplitPlanDiagnosticKind,
    /// Human-readable detail.
    pub message: String,
    /// Optional mesh side.
    pub side: Option<MeshSide>,
    /// Optional face index.
    pub face: Option<usize>,
    /// Optional directed edge.
    pub edge: Option<[usize; 2]>,
    /// Optional graph-vertex index.
    pub graph_vertex: Option<usize>,
}

impl SplitPlanDiagnostic {
    fn new(kind: SplitPlanDiagnosticKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
            side: None,
            face: None,
            edge: None,
            graph_vertex: None,
        }
    }

    const fn with_side(mut self, side: MeshSide) -> Self {
        self.side = Some(side);
        self
    }

    const fn with_face(mut self, face: usize) -> Self {
        self.face = Some(face);
        self
    }

    const fn with_edge(mut self, edge: [usize; 2]) -> Self {
        self.edge = Some(edge);
        self
    }

    const fn with_graph_vertex(mut self, graph_vertex: usize) -> Self {
        self.graph_vertex = Some(graph_vertex);
        self
    }
}

/// Validation report for exact split topology and face split plans.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SplitPlanValidationReport {
    /// Diagnostics collected during validation.
    pub diagnostics: Vec<SplitPlanDiagnostic>,
}

impl SplitPlanValidationReport {
    /// Return whether the checked split plan is ready for the next exact stage.
    pub fn is_valid(&self) -> bool {
        self.diagnostics.is_empty()
    }
}

/// Exact boundary node for a split face.
#[derive(Clone, Debug, PartialEq)]
pub enum FaceSplitBoundaryNode {
    /// Original mesh vertex with its exact point.
    OriginalVertex {
        /// Vertex index in the source mesh.
        vertex: usize,
        /// Exact point carried into the split boundary.
        point: Point3,
    },
    /// Constructed graph vertex with its exact point.
    GraphVertex {
        /// Index in [`ExactSplitTopologyPlan::graph_vertices`].
        graph_vertex: usize,
        /// Exact constructed point.
        point: Point3,
    },
}

/// Exact boundary chain for one split edge of an original face.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceSplitBoundaryChain {
    /// Original directed face edge.
    pub edge: [usize; 2],
    /// Boundary nodes in directed edge order.
    pub nodes: Vec<FaceSplitBoundaryNode>,
}

/// Exact geometry handoff for one split face.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceSplitGeometry {
    /// Mesh side owning the face.
    pub side: MeshSide,
    /// Face index in the owning mesh.
    pub face: usize,
    /// Original triangle vertices.
    pub triangle: [usize; 3],
    /// Boundary chains that contain exact graph vertices.
    pub boundary_chains: Vec<FaceSplitBoundaryChain>,
}

/// Non-mutating exact split-face geometry plan.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactFaceSplitGeometryPlan {
    /// Per-face exact boundary geometry.
    pub faces: Vec<FaceSplitGeometry>,
}

impl ExactFaceSplitGeometryPlan {
    /// Count exact graph vertices referenced by boundary geometry.
    pub fn graph_vertex_references(&self) -> usize {
        self.faces
            .iter()
            .flat_map(|face| face.boundary_chains.iter())
            .flat_map(|chain| chain.nodes.iter())
            .filter(|node| matches!(node, FaceSplitBoundaryNode::GraphVertex { .. }))
            .count()
    }

    /// Validate that every split boundary node lies on its original face plane.
    ///
    /// Segment/plane crossings create points that should be incident to the
    /// face whose boundary they are splitting. This check replays that
    /// incidence as exact `hyperlimit::orient3d_report` predicates rather than
    /// trusting construction history or approximate coordinates, following
    /// Yap's requirement that later topology stages consume certified facts.
    pub fn validate_boundary_incidence(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> SplitPlanValidationReport {
        validate_face_split_geometry_incidence(self, left, right)
    }

    /// Build full face-region boundary loops for downstream exact triangulation.
    ///
    /// The geometry handoff stores only split edge chains. This method expands
    /// each affected triangle into one boundary loop in original face-edge
    /// order, inserting exact graph vertices along the split edges. It still
    /// does not decide winding, ownership, or boolean output; those decisions
    /// remain certified downstream stages under Yap's exact-geometric-
    /// computation separation.
    pub fn region_plan(&self, left: &ExactMesh, right: &ExactMesh) -> ExactFaceRegionPlan {
        face_region_plan(self, left, right)
    }
}

/// One pre-triangulation boundary loop for an affected face.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceRegionBoundary {
    /// Mesh side owning the source face.
    pub side: MeshSide,
    /// Face index in the source mesh.
    pub face: usize,
    /// Original triangle vertices.
    pub triangle: [usize; 3],
    /// Boundary loop in source triangle order, with split graph vertices
    /// inserted along each affected edge.
    pub boundary: Vec<FaceSplitBoundaryNode>,
}

/// Exact pre-triangulation region plan for affected faces.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactFaceRegionPlan {
    /// One boundary loop per affected source face.
    pub regions: Vec<FaceRegionBoundary>,
}

impl ExactFaceRegionPlan {
    /// Count graph vertices referenced by all region loops.
    pub fn graph_vertex_references(&self) -> usize {
        self.regions
            .iter()
            .flat_map(|region| region.boundary.iter())
            .filter(|node| matches!(node, FaceSplitBoundaryNode::GraphVertex { .. }))
            .count()
    }

    /// Validate boundary-loop structure and original-face incidence.
    ///
    /// Region loops are the direct input expected by exact triangulation. This
    /// check rejects malformed loops and reuses exact plane-incidence
    /// predicates so downstream triangulation does not inherit unchecked
    /// construction assumptions.
    pub fn validate(&self, left: &ExactMesh, right: &ExactMesh) -> SplitPlanValidationReport {
        validate_face_region_plan(self, left, right)
    }
}

fn events_for_face_pair(
    left: &ExactMesh,
    right: &ExactMesh,
    classification: &MeshFacePairClassification,
) -> FacePairEvents {
    let left_tri = left.triangles()[classification.left_face].0;
    let right_tri = right.triangles()[classification.right_face].0;
    let left_edges = triangle_edges(left_tri);
    let right_edges = triangle_edges(right_tri);
    let mut events = Vec::new();
    let mut projection = None;

    if let Some(triangle) = &classification.triangle {
        append_segment_plane_events(
            &mut events,
            MeshSide::Right,
            &right_edges,
            MeshSide::Left,
            classification.left_face,
            &triangle.right_edge_events,
        );
        append_segment_plane_events(
            &mut events,
            MeshSide::Left,
            &left_edges,
            MeshSide::Right,
            classification.right_face,
            &triangle.left_edge_events,
        );

        if let Some(coplanar) = &triangle.coplanar {
            projection = coplanar.projection;
            append_coplanar_events(
                &mut events,
                classification.left_face,
                classification.right_face,
                left_tri,
                right_tri,
                &left_edges,
                &right_edges,
                coplanar,
            );
        }
    }

    if classification.relation == MeshFacePairRelation::Unknown {
        events.push(IntersectionEvent::Unknown);
    }

    FacePairEvents {
        left_face: classification.left_face,
        right_face: classification.right_face,
        relation: classification.relation,
        projection,
        events,
    }
}

fn edge_split_plan(graph: &ExactIntersectionGraph) -> ExactEdgeSplitPlan {
    let mut grouped = BTreeMap::<(u8, usize, usize), EdgeSplit>::new();
    for pair in &graph.face_pairs {
        for event in &pair.events {
            let IntersectionEvent::SegmentPlane {
                segment_side,
                edge,
                plane_face,
                point: Some(point),
                parameter: Some(parameter),
                ..
            } = event
            else {
                continue;
            };
            let key = (side_key(*segment_side), edge[0], edge[1]);
            grouped
                .entry(key)
                .or_insert_with(|| EdgeSplit {
                    side: *segment_side,
                    edge: *edge,
                    points: Vec::new(),
                })
                .points
                .push(EdgeSplitPoint {
                    face_pair: [pair.left_face, pair.right_face],
                    plane_face: *plane_face,
                    parameter: parameter.clone(),
                    point: point.clone(),
                });
        }
    }

    let mut unknown_orderings = 0;
    let mut splits = grouped.into_values().collect::<Vec<_>>();
    for split in &mut splits {
        unknown_orderings += sort_split_points(&mut split.points);
    }
    ExactEdgeSplitPlan {
        splits,
        unknown_orderings,
    }
}

fn graph_vertex_plan(split_plan: &ExactEdgeSplitPlan) -> ExactGraphVertexPlan {
    let mut vertices = Vec::<ExactGraphVertex>::new();
    let mut unresolved_equalities = 0;

    for split in &split_plan.splits {
        for point in &split.points {
            let vertex_use = ExactGraphVertexUse {
                side: split.side,
                edge: split.edge,
                face_pair: point.face_pair,
                plane_face: point.plane_face,
            };

            let mut matched = None;
            for (index, vertex) in vertices.iter().enumerate() {
                match points_equal(&point.point, &vertex.point) {
                    Some(true) => {
                        matched = Some(index);
                        break;
                    }
                    Some(false) => {}
                    None => unresolved_equalities += 1,
                }
            }

            if let Some(index) = matched {
                vertices[index].uses.push(vertex_use);
            } else {
                vertices.push(ExactGraphVertex {
                    point: point.point.clone(),
                    uses: vec![vertex_use],
                });
            }
        }
    }

    ExactGraphVertexPlan {
        vertices,
        unresolved_equalities,
    }
}

fn split_topology_plan(
    split_plan: &ExactEdgeSplitPlan,
    graph_vertices: &ExactGraphVertexPlan,
) -> ExactSplitTopologyPlan {
    let mut unresolved_vertex_lookups = 0;
    let mut edge_chains = Vec::new();
    for split in &split_plan.splits {
        let mut nodes = Vec::with_capacity(split.points.len() + 2);
        nodes.push(SplitEdgeNode::OriginalVertex {
            side: split.side,
            vertex: split.edge[0],
        });
        for point in &split.points {
            match find_graph_vertex(&point.point, graph_vertices) {
                Some(index) => nodes.push(SplitEdgeNode::GraphVertex {
                    graph_vertex: index,
                }),
                None => unresolved_vertex_lookups += 1,
            }
        }
        nodes.push(SplitEdgeNode::OriginalVertex {
            side: split.side,
            vertex: split.edge[1],
        });
        edge_chains.push(SplitEdgeChain {
            side: split.side,
            edge: split.edge,
            nodes,
        });
    }

    ExactSplitTopologyPlan {
        graph_vertices: graph_vertices.vertices.clone(),
        edge_chains,
        unresolved_vertex_lookups,
        unresolved_equalities: graph_vertices.unresolved_equalities,
        unknown_orderings: split_plan.unknown_orderings,
    }
}

fn face_split_plan(topology: &ExactSplitTopologyPlan) -> ExactFaceSplitPlan {
    let mut faces = BTreeMap::<(u8, usize), FaceSplitPlan>::new();
    for chain in &topology.edge_chains {
        let graph_vertices = chain
            .nodes
            .iter()
            .filter_map(|node| match node {
                SplitEdgeNode::GraphVertex { graph_vertex } => Some(*graph_vertex),
                SplitEdgeNode::OriginalVertex { .. } => None,
            })
            .collect::<Vec<_>>();
        if graph_vertices.is_empty() {
            continue;
        }
        let face_indices = graph_vertices
            .iter()
            .flat_map(|&index| topology.graph_vertices[index].uses.iter())
            .filter(|vertex_use| vertex_use.side == chain.side && vertex_use.edge == chain.edge)
            .map(|vertex_use| match chain.side {
                MeshSide::Left => vertex_use.face_pair[0],
                MeshSide::Right => vertex_use.face_pair[1],
            })
            .collect::<BTreeSet<_>>();
        for face in face_indices {
            faces
                .entry((side_key(chain.side), face))
                .or_insert_with(|| FaceSplitPlan {
                    side: chain.side,
                    face,
                    edges: Vec::new(),
                })
                .edges
                .push(FaceSplitEdge {
                    edge: chain.edge,
                    graph_vertices: graph_vertices.clone(),
                });
        }
    }
    ExactFaceSplitPlan {
        faces: faces.into_values().collect(),
    }
}

fn validate_split_topology_plan(topology: &ExactSplitTopologyPlan) -> SplitPlanValidationReport {
    let mut diagnostics = Vec::new();

    for _ in 0..topology.unknown_orderings {
        diagnostics.push(SplitPlanDiagnostic::new(
            SplitPlanDiagnosticKind::UnknownOrdering,
            "edge split parameters have an uncertified ordering",
        ));
    }
    for _ in 0..topology.unresolved_equalities {
        diagnostics.push(SplitPlanDiagnostic::new(
            SplitPlanDiagnosticKind::UnresolvedEquality,
            "graph-vertex equality could not be certified",
        ));
    }
    for _ in 0..topology.unresolved_vertex_lookups {
        diagnostics.push(SplitPlanDiagnostic::new(
            SplitPlanDiagnosticKind::UnresolvedVertexLookup,
            "split point could not be matched to a graph vertex",
        ));
    }

    for (index, vertex) in topology.graph_vertices.iter().enumerate() {
        if vertex.uses.is_empty() {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::EmptyGraphVertexUses,
                    "graph vertex has no exact source uses",
                )
                .with_graph_vertex(index),
            );
        }
    }

    for chain in &topology.edge_chains {
        if chain.nodes.len() < 2 {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::EmptyOrShortEdgeChain,
                    "split edge chain does not connect both original endpoints",
                )
                .with_side(chain.side)
                .with_edge(chain.edge),
            );
            continue;
        }

        if chain.nodes.first()
            != Some(&SplitEdgeNode::OriginalVertex {
                side: chain.side,
                vertex: chain.edge[0],
            })
        {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::WrongChainStart,
                    "split edge chain does not start at the directed edge start",
                )
                .with_side(chain.side)
                .with_edge(chain.edge),
            );
        }

        if chain.nodes.last()
            != Some(&SplitEdgeNode::OriginalVertex {
                side: chain.side,
                vertex: chain.edge[1],
            })
        {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::WrongChainEnd,
                    "split edge chain does not end at the directed edge end",
                )
                .with_side(chain.side)
                .with_edge(chain.edge),
            );
        }

        for node in &chain.nodes {
            match node {
                SplitEdgeNode::OriginalVertex { side, .. } if *side != chain.side => {
                    diagnostics.push(
                        SplitPlanDiagnostic::new(
                            SplitPlanDiagnosticKind::ChainSideMismatch,
                            "original vertex node is on a different mesh side from its chain",
                        )
                        .with_side(chain.side)
                        .with_edge(chain.edge),
                    );
                }
                SplitEdgeNode::GraphVertex { graph_vertex }
                    if *graph_vertex >= topology.graph_vertices.len() =>
                {
                    diagnostics.push(
                        SplitPlanDiagnostic::new(
                            SplitPlanDiagnosticKind::GraphVertexOutOfRange,
                            "split edge chain references a missing graph vertex",
                        )
                        .with_side(chain.side)
                        .with_edge(chain.edge)
                        .with_graph_vertex(*graph_vertex),
                    );
                }
                _ => {}
            }
        }
    }

    SplitPlanValidationReport { diagnostics }
}

fn validate_face_split_plan(
    face_plan: &ExactFaceSplitPlan,
    topology: &ExactSplitTopologyPlan,
) -> SplitPlanValidationReport {
    let mut diagnostics = Vec::new();

    for face in &face_plan.faces {
        if face.edges.is_empty() {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::EmptyFaceSplit,
                    "face split work item has no split edges",
                )
                .with_side(face.side)
                .with_face(face.face),
            );
        }

        let mut seen_edges = BTreeSet::new();
        for edge in &face.edges {
            if !seen_edges.insert(edge.edge) {
                diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::DuplicateFaceSplitEdge,
                        "face split work item repeats an original edge",
                    )
                    .with_side(face.side)
                    .with_face(face.face)
                    .with_edge(edge.edge),
                );
            }

            if edge.graph_vertices.is_empty() {
                diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::EmptyFaceSplitEdge,
                        "face split edge has no graph vertices",
                    )
                    .with_side(face.side)
                    .with_face(face.face)
                    .with_edge(edge.edge),
                );
            }

            for &graph_vertex in &edge.graph_vertices {
                let Some(vertex) = topology.graph_vertices.get(graph_vertex) else {
                    diagnostics.push(
                        SplitPlanDiagnostic::new(
                            SplitPlanDiagnosticKind::GraphVertexOutOfRange,
                            "face split edge references a missing graph vertex",
                        )
                        .with_side(face.side)
                        .with_face(face.face)
                        .with_edge(edge.edge)
                        .with_graph_vertex(graph_vertex),
                    );
                    continue;
                };

                if !vertex.uses.iter().any(|vertex_use| {
                    vertex_use.side == face.side
                        && vertex_use.edge == edge.edge
                        && match face.side {
                            MeshSide::Left => vertex_use.face_pair[0] == face.face,
                            MeshSide::Right => vertex_use.face_pair[1] == face.face,
                        }
                }) {
                    diagnostics.push(
                        SplitPlanDiagnostic::new(
                            SplitPlanDiagnosticKind::MissingFaceSplitSourceUse,
                            "face split edge graph vertex has no exact source use on this face edge",
                        )
                        .with_side(face.side)
                        .with_face(face.face)
                        .with_edge(edge.edge)
                        .with_graph_vertex(graph_vertex),
                    );
                }
            }
        }
    }

    SplitPlanValidationReport { diagnostics }
}

fn face_split_geometry_plan(
    left: &ExactMesh,
    right: &ExactMesh,
    topology: &ExactSplitTopologyPlan,
    face_plan: &ExactFaceSplitPlan,
) -> Result<ExactFaceSplitGeometryPlan, MeshError> {
    if let Some(diagnostic) = first_face_geometry_error(left, right, topology, face_plan) {
        return Err(MeshError::one(diagnostic));
    }

    let chains = topology
        .edge_chains
        .iter()
        .map(|chain| ((side_key(chain.side), chain.edge[0], chain.edge[1]), chain))
        .collect::<BTreeMap<_, _>>();

    let mut faces = Vec::with_capacity(face_plan.faces.len());
    for face in &face_plan.faces {
        let mesh = mesh_for_side(face.side, left, right);
        let triangle = mesh.triangles()[face.face].0;
        let mut boundary_chains = Vec::with_capacity(face.edges.len());
        for edge in &face.edges {
            let chain = chains[&(side_key(face.side), edge.edge[0], edge.edge[1])];
            boundary_chains.push(FaceSplitBoundaryChain {
                edge: edge.edge,
                nodes: chain
                    .nodes
                    .iter()
                    .map(|node| face_boundary_node(face.side, node, left, right, topology))
                    .collect::<Result<Vec<_>, _>>()?,
            });
        }
        faces.push(FaceSplitGeometry {
            side: face.side,
            face: face.face,
            triangle,
            boundary_chains,
        });
    }

    Ok(ExactFaceSplitGeometryPlan { faces })
}

fn first_face_geometry_error(
    left: &ExactMesh,
    right: &ExactMesh,
    topology: &ExactSplitTopologyPlan,
    face_plan: &ExactFaceSplitPlan,
) -> Option<MeshDiagnostic> {
    let chains = topology
        .edge_chains
        .iter()
        .map(|chain| ((side_key(chain.side), chain.edge[0], chain.edge[1]), chain))
        .collect::<BTreeMap<_, _>>();

    for face in &face_plan.faces {
        let mesh = mesh_for_side(face.side, left, right);
        if face.face >= mesh.triangles().len() {
            return Some(
                MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::IndexOutOfBounds,
                    "face split geometry references a missing face",
                )
                .with_face(face.face),
            );
        }
        for edge in &face.edges {
            if !chains.contains_key(&(side_key(face.side), edge.edge[0], edge.edge[1])) {
                return Some(
                    MeshDiagnostic::new(
                        Severity::Error,
                        DiagnosticKind::IndexOutOfBounds,
                        "face split geometry references a missing split edge chain",
                    )
                    .with_face(face.face)
                    .with_edge(edge.edge),
                );
            }
            for &graph_vertex in &edge.graph_vertices {
                if graph_vertex >= topology.graph_vertices.len() {
                    return Some(
                        MeshDiagnostic::new(
                            Severity::Error,
                            DiagnosticKind::IndexOutOfBounds,
                            "face split geometry references a missing graph vertex",
                        )
                        .with_face(face.face)
                        .with_edge(edge.edge),
                    );
                }
            }
        }
    }

    None
}

fn face_boundary_node(
    side: MeshSide,
    node: &SplitEdgeNode,
    left: &ExactMesh,
    right: &ExactMesh,
    topology: &ExactSplitTopologyPlan,
) -> Result<FaceSplitBoundaryNode, MeshError> {
    match node {
        SplitEdgeNode::OriginalVertex {
            side: vertex_side,
            vertex,
        } if *vertex_side == side => {
            let mesh = mesh_for_side(side, left, right);
            let point = mesh.vertices().get(*vertex).ok_or_else(|| {
                MeshError::one(
                    MeshDiagnostic::new(
                        Severity::Error,
                        DiagnosticKind::IndexOutOfBounds,
                        "split boundary references a missing original vertex",
                    )
                    .with_vertex(*vertex),
                )
            })?;
            Ok(FaceSplitBoundaryNode::OriginalVertex {
                vertex: *vertex,
                point: point.to_hyperlimit_point(),
            })
        }
        SplitEdgeNode::GraphVertex { graph_vertex } => {
            let vertex = topology.graph_vertices.get(*graph_vertex).ok_or_else(|| {
                MeshError::one(
                    MeshDiagnostic::new(
                        Severity::Error,
                        DiagnosticKind::IndexOutOfBounds,
                        "split boundary references a missing graph vertex",
                    )
                    .with_vertex(*graph_vertex),
                )
            })?;
            Ok(FaceSplitBoundaryNode::GraphVertex {
                graph_vertex: *graph_vertex,
                point: vertex.point.clone(),
            })
        }
        SplitEdgeNode::OriginalVertex { vertex, .. } => Err(MeshError::one(
            MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexOutOfBounds,
                "split boundary original vertex is on the wrong mesh side",
            )
            .with_vertex(*vertex),
        )),
    }
}

fn mesh_for_side<'a>(side: MeshSide, left: &'a ExactMesh, right: &'a ExactMesh) -> &'a ExactMesh {
    match side {
        MeshSide::Left => left,
        MeshSide::Right => right,
    }
}

fn validate_face_split_geometry_incidence(
    geometry: &ExactFaceSplitGeometryPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut diagnostics = Vec::new();

    for face in &geometry.faces {
        let mesh = mesh_for_side(face.side, left, right);
        if face.face >= mesh.triangles().len() {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::GraphVertexOutOfRange,
                    "split-face geometry references a missing source face",
                )
                .with_side(face.side)
                .with_face(face.face),
            );
            continue;
        }

        let triangle = mesh.triangles()[face.face].0;
        let a = mesh.vertices()[triangle[0]].to_hyperlimit_point();
        let b = mesh.vertices()[triangle[1]].to_hyperlimit_point();
        let c = mesh.vertices()[triangle[2]].to_hyperlimit_point();
        for chain in &face.boundary_chains {
            for node in &chain.nodes {
                let point = match node {
                    FaceSplitBoundaryNode::OriginalVertex { point, .. }
                    | FaceSplitBoundaryNode::GraphVertex { point, .. } => point,
                };
                match orient3d_report(&a, &b, &c, point).value() {
                    Some(Sign::Zero) => {}
                    Some(Sign::Negative | Sign::Positive) => diagnostics.push(
                        SplitPlanDiagnostic::new(
                            SplitPlanDiagnosticKind::BoundaryNodeOffFacePlane,
                            "split boundary node is not incident to its original face plane",
                        )
                        .with_side(face.side)
                        .with_face(face.face)
                        .with_edge(chain.edge),
                    ),
                    None => diagnostics.push(
                        SplitPlanDiagnostic::new(
                            SplitPlanDiagnosticKind::UnknownBoundaryIncidence,
                            "split boundary node incidence could not be certified",
                        )
                        .with_side(face.side)
                        .with_face(face.face)
                        .with_edge(chain.edge),
                    ),
                }
            }
        }
    }

    SplitPlanValidationReport { diagnostics }
}

fn face_region_plan(
    geometry: &ExactFaceSplitGeometryPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactFaceRegionPlan {
    let mut regions = Vec::with_capacity(geometry.faces.len());
    for face in &geometry.faces {
        let mesh = mesh_for_side(face.side, left, right);
        let triangle = face.triangle;
        let mut chains = face
            .boundary_chains
            .iter()
            .map(|chain| ((chain.edge[0], chain.edge[1]), chain))
            .collect::<BTreeMap<_, _>>();
        let mut boundary = Vec::new();

        for edge in triangle_edges(triangle) {
            let nodes = if let Some(chain) = chains.remove(&(edge[0], edge[1])) {
                chain.nodes.clone()
            } else {
                vec![
                    original_boundary_node(mesh, edge[0]),
                    original_boundary_node(mesh, edge[1]),
                ]
            };
            for node in nodes {
                push_boundary_node(&mut boundary, node);
            }
        }
        if boundary
            .first()
            .zip(boundary.last())
            .is_some_and(|(first, last)| boundary_nodes_equal(first, last) == Some(true))
        {
            boundary.pop();
        }

        regions.push(FaceRegionBoundary {
            side: face.side,
            face: face.face,
            triangle,
            boundary,
        });
    }

    ExactFaceRegionPlan { regions }
}

fn validate_face_region_plan(
    plan: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> SplitPlanValidationReport {
    let mut diagnostics = Vec::new();
    for region in &plan.regions {
        if region.boundary.len() < 3 {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::EmptyOrShortRegionBoundary,
                    "face region boundary has fewer than three nodes",
                )
                .with_side(region.side)
                .with_face(region.face),
            );
        }

        for window in region.boundary.windows(2) {
            if boundary_nodes_equal(&window[0], &window[1]) == Some(true) {
                diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::DuplicateConsecutiveRegionNode,
                        "face region boundary contains consecutive duplicate nodes",
                    )
                    .with_side(region.side)
                    .with_face(region.face),
                );
            }
        }

        let mesh = mesh_for_side(region.side, left, right);
        if region.face >= mesh.triangles().len() {
            diagnostics.push(
                SplitPlanDiagnostic::new(
                    SplitPlanDiagnosticKind::GraphVertexOutOfRange,
                    "face region references a missing source face",
                )
                .with_side(region.side)
                .with_face(region.face),
            );
            continue;
        }

        let triangle = mesh.triangles()[region.face].0;
        let a = mesh.vertices()[triangle[0]].to_hyperlimit_point();
        let b = mesh.vertices()[triangle[1]].to_hyperlimit_point();
        let c = mesh.vertices()[triangle[2]].to_hyperlimit_point();
        for node in &region.boundary {
            let point = boundary_node_point(node);
            match orient3d_report(&a, &b, &c, point).value() {
                Some(Sign::Zero) => {}
                Some(Sign::Negative | Sign::Positive) => diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::BoundaryNodeOffFacePlane,
                        "face region boundary node is not incident to its source face plane",
                    )
                    .with_side(region.side)
                    .with_face(region.face),
                ),
                None => diagnostics.push(
                    SplitPlanDiagnostic::new(
                        SplitPlanDiagnosticKind::UnknownBoundaryIncidence,
                        "face region boundary incidence could not be certified",
                    )
                    .with_side(region.side)
                    .with_face(region.face),
                ),
            }
        }
    }

    SplitPlanValidationReport { diagnostics }
}

fn original_boundary_node(mesh: &ExactMesh, vertex: usize) -> FaceSplitBoundaryNode {
    FaceSplitBoundaryNode::OriginalVertex {
        vertex,
        point: mesh.vertices()[vertex].to_hyperlimit_point(),
    }
}

fn push_boundary_node(boundary: &mut Vec<FaceSplitBoundaryNode>, node: FaceSplitBoundaryNode) {
    if boundary
        .last()
        .is_some_and(|last| boundary_nodes_equal(last, &node) == Some(true))
    {
        return;
    }
    boundary.push(node);
}

fn boundary_node_point(node: &FaceSplitBoundaryNode) -> &Point3 {
    match node {
        FaceSplitBoundaryNode::OriginalVertex { point, .. }
        | FaceSplitBoundaryNode::GraphVertex { point, .. } => point,
    }
}

fn boundary_nodes_equal(
    left: &FaceSplitBoundaryNode,
    right: &FaceSplitBoundaryNode,
) -> Option<bool> {
    points_equal(boundary_node_point(left), boundary_node_point(right))
}

fn find_graph_vertex(point: &Point3, graph_vertices: &ExactGraphVertexPlan) -> Option<usize> {
    graph_vertices
        .vertices
        .iter()
        .position(|vertex| points_equal(point, &vertex.point) == Some(true))
}

fn sort_split_points(points: &mut [EdgeSplitPoint]) -> usize {
    let mut unknown_orderings = 0;
    points.sort_by(
        |left, right| match compare_reals(&left.parameter, &right.parameter).value() {
            Some(ordering) => ordering,
            None => {
                unknown_orderings += 1;
                Ordering::Equal
            }
        },
    );
    unknown_orderings
}

fn append_segment_plane_events(
    events: &mut Vec<IntersectionEvent>,
    segment_side: MeshSide,
    edges: &[[usize; 2]; 3],
    plane_side: MeshSide,
    plane_face: usize,
    segment_events: &[SegmentPlaneIntersection],
) {
    for (edge, event) in edges.iter().zip(segment_events) {
        if matches!(event.relation, SegmentPlaneRelation::Disjoint) {
            continue;
        }
        events.push(IntersectionEvent::SegmentPlane {
            segment_side,
            edge: *edge,
            plane_side,
            plane_face,
            relation: event.relation,
            point: event.point.clone(),
            parameter: event.parameter.clone(),
        });
    }
}

#[allow(clippy::too_many_arguments)]
fn append_coplanar_events(
    events: &mut Vec<IntersectionEvent>,
    left_face: usize,
    right_face: usize,
    left_tri: [usize; 3],
    right_tri: [usize; 3],
    left_edges: &[[usize; 2]; 3],
    right_edges: &[[usize; 2]; 3],
    coplanar: &CoplanarTriangleClassification,
) {
    for (index, relation) in coplanar.edge_intersections.iter().copied().enumerate() {
        let left_edge = left_edges[index / 3];
        let right_edge = right_edges[index % 3];
        if relation != SegmentIntersection::Disjoint {
            events.push(IntersectionEvent::CoplanarEdge {
                left_edge,
                right_edge,
                relation,
            });
        }
    }

    for (vertex, location) in right_tri.into_iter().zip(coplanar.right_vertices_in_left) {
        append_vertex_event(
            events,
            MeshSide::Right,
            vertex,
            MeshSide::Left,
            left_face,
            location,
        );
    }
    for (vertex, location) in left_tri.into_iter().zip(coplanar.left_vertices_in_right) {
        append_vertex_event(
            events,
            MeshSide::Left,
            vertex,
            MeshSide::Right,
            right_face,
            location,
        );
    }
}

fn append_vertex_event(
    events: &mut Vec<IntersectionEvent>,
    vertex_side: MeshSide,
    vertex: usize,
    triangle_side: MeshSide,
    triangle_face: usize,
    location: Option<TriangleLocation>,
) {
    match location {
        Some(
            location @ (TriangleLocation::Inside
            | TriangleLocation::OnEdge
            | TriangleLocation::OnVertex),
        ) => {
            events.push(IntersectionEvent::CoplanarVertex {
                vertex_side,
                vertex,
                triangle_side,
                triangle_face,
                location,
            });
        }
        None => events.push(IntersectionEvent::Unknown),
        Some(TriangleLocation::Outside | TriangleLocation::Degenerate) => {}
    }
}

fn triangle_edges(tri: [usize; 3]) -> [[usize; 2]; 3] {
    [[tri[0], tri[1]], [tri[1], tri[2]], [tri[2], tri[0]]]
}

fn side_key(side: MeshSide) -> u8 {
    match side {
        MeshSide::Left => 0,
        MeshSide::Right => 1,
    }
}

fn points_equal(left: &Point3, right: &Point3) -> Option<bool> {
    let x = compare_reals(&left.x, &right.x).value()?;
    let y = compare_reals(&left.y, &right.y).value()?;
    let z = compare_reals(&left.z, &right.z).value()?;
    Some(x == Ordering::Equal && y == Ordering::Equal && z == Ordering::Equal)
}
