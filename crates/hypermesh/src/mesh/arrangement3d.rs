//! Exact 3D arrangement artifact for mesh pairs.
//!
//! This module is deliberately an arrangement/cell-complex handoff, not a
//! direct triangle-soup Boolean shortcut. It retains source vertices, exact
//! graph-intersection vertices, split edge chains, face-region boundary loops,
//! carrier-face provenance, and winding labels needed by later selection and
//! simplification stages.

pub(crate) mod arrangement2d;
pub(crate) mod cell_complex;
pub(crate) mod loop_triangulation;
pub(crate) mod regularization;

use self::arrangement2d::{
    ExactArrangement2d, ExactArrangement2dBlocker, ExactArrangement2dInputSegment,
    ExactArrangement2dOverlay, ExactArrangement2dRegion, ExactArrangement2dRegionRing,
    ExactArrangement2dSegmentSource, ExactArrangement2dSetOperation, build_exact_arrangement2d,
    build_exact_arrangement2d_overlay, exact_arrangement2d_face_witness,
};
use self::cell_complex::{
    ExactCellComplex, ExactLabeledCellComplex, ExactLabeledCellComplexFreshness,
    ExactRegionOwnershipReport, region_ownership_status,
};
use super::ExactMesh;
use super::boolean::solid::{
    ClosedMeshOrientation, ConvexSolidPointClassification, ConvexSolidPointRelation,
    classify_point_against_convex_solid_report, exact_mesh_orientation,
};
use super::boolean::winding::{
    ClosedMeshWindingRelation, PointMeshWindingReport,
    classify_point_against_closed_mesh_winding_report,
};
use super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
use super::graph::key::{
    ExactPoint3Key, ExactUndirectedPoint3EdgeKey, exact_point3_key,
    exact_undirected_point3_edge_key,
};
use super::graph::{
    CoplanarOverlapGraph, ExactFaceRegionPlan, ExactIntersectionGraph, ExactSplitTopologyPlan,
    FaceRegionBoundary, FaceSplitBoundaryNode, MeshSide, SplitEdgeNode, SplitPlanValidationReport,
    build_validated_intersection_graph,
};
use super::validation::ExactMeshValidationPolicy;
use core::cmp::Ordering;
use hyperlimit::CoplanarProjection;
use hyperlimit::SourceProvenance;
use loop_triangulation::{
    group_exact_coplanar_loops, projected_loop_interior_witness, triangulate_exact_loop_group,
};
use regularization::{
    ExactArrangementBlocker, ExactLowerDimensionalPolicy, ExactRegularizationPolicy,
    ExactUnresolvedPolicy,
};
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{
    Point2, Point3, SegmentIntersection, TriangleLocation, classify_point_triangle,
    classify_segment_intersection, compare_point2_lexicographic, compare_reals, point_on_segment,
    point_on_segment3, point2_equal, point3_equal, project_point3,
    proper_segment_intersection_point,
};
use hyperreal::Real;

/// Source of an arrangement vertex.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum ArrangementVertexProvenance {
    /// Original source mesh vertex.
    SourceVertex { side: MeshSide, vertex: usize },
    /// Constructed intersection graph vertex.
    GraphIntersection { graph_vertex: usize },
    /// Vertex from a retained carrier-plane overlay arrangement.
    CarrierPlaneVertex { overlay: usize, vertex: usize },
    /// Vertex from a retained per-source-face split arrangement.
    FacePlaneVertex { arrangement: usize, vertex: usize },
}

/// Exact arrangement vertex.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ArrangementVertex {
    /// Exact coordinate.
    pub(crate) point: Point3,
    /// Construction/source provenance.
    pub(crate) provenance: Vec<ArrangementVertexProvenance>,
}

/// Source of an arrangement edge.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum ArrangementEdgeProvenance {
    /// Split segment from one original mesh edge.
    Source { side: MeshSide, edge: [usize; 2] },
    /// Split edge from one retained carrier-plane overlay arrangement.
    CarrierPlane { overlay: usize, edge: usize },
    /// Split edge from one retained per-source-face arrangement.
    FacePlane { arrangement: usize, edge: usize },
}

/// Exact arrangement edge.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ArrangementEdge {
    /// Endpoint arrangement vertex indices.
    pub(crate) vertices: [usize; 2],
    /// Construction/source provenance.
    pub(crate) provenance: Vec<ArrangementEdgeProvenance>,
}

/// Boundary node reference for an arrangement face cell.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum ArrangementFaceCellNode {
    /// Original source vertex.
    Source { side: MeshSide, vertex: usize },
    /// Constructed graph vertex.
    Graph { graph_vertex: usize },
    /// Vertex from a retained carrier-plane 2D overlay.
    CarrierPlane { overlay: usize, vertex: usize },
    /// Vertex from a retained per-source-face 2D split arrangement.
    FacePlane { arrangement: usize, vertex: usize },
}

/// Cell owner/carrier information.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ArrangementFaceCarrier {
    /// Source mesh side owning the carrier triangle.
    pub(crate) side: MeshSide,
    /// Source face index.
    pub(crate) face: usize,
    /// Source triangle vertex indices.
    pub(crate) triangle: [usize; 3],
}

/// Exact classification of a face cell against the opposite mesh.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ArrangementOppositeClassification {
    /// Exact representative point used for the winding query.
    pub(crate) representative: Point3,
    /// Winding report against the opposite mesh.
    pub(crate) winding: PointMeshWindingReport,
    /// Exact convex-solid classification retained when it certifies a relation.
    pub(crate) convex_fallback: Option<ConvexSolidPointClassification>,
}

impl ArrangementOppositeClassification {
    pub(crate) fn convex_certified_relation(&self) -> Option<ConvexSolidPointRelation> {
        self.convex_fallback
            .as_ref()
            .and_then(|classification| certified_convex_point_relation(classification.relation()))
    }
}

/// Exact face cell in the retained arrangement.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ArrangementFaceCell {
    /// Source carrier face.
    pub(crate) carrier: ArrangementFaceCarrier,
    /// Boundary nodes in carrier-face order.
    pub(crate) boundary: Vec<ArrangementFaceCellNode>,
    /// Exact boundary coordinates in carrier-face order.
    pub(crate) boundary_points: Vec<Point3>,
    /// Classification against the opposite mesh, when the query was meaningful.
    pub(crate) opposite: Option<ArrangementOppositeClassification>,
}

pub(crate) fn validate_arrangement_face_cell(
    cell: &ArrangementFaceCell,
) -> Result<(), ExactArrangementBlocker> {
    if cell.boundary.len() != cell.boundary_points.len() || cell.boundary.len() < 3 {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    Ok(())
}

pub(crate) fn validate_arrangement_face_cells(
    face_cells: &[ArrangementFaceCell],
) -> Result<(), ExactArrangementBlocker> {
    for cell in face_cells {
        validate_arrangement_face_cell(cell)?;
    }
    Ok(())
}

pub(crate) fn arrangement_face_cell_boundary_counts(
    face_cells: &[ArrangementFaceCell],
) -> (usize, usize) {
    let boundary_nodes = face_cells.iter().map(|cell| cell.boundary.len()).sum();
    let boundary_points = face_cells
        .iter()
        .map(|cell| cell.boundary_points.len())
        .sum();
    (boundary_nodes, boundary_points)
}

/// Exact lower-dimensional contact retained by arrangement regularization.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum ArrangementLowerDimensionalArtifact {
    /// A certified point contact between source faces.
    PointContact {
        /// Left source face.
        left_face: usize,
        /// Right source face.
        right_face: usize,
        /// Exact contact point.
        point: Point3,
    },
    /// A certified positive-length edge/segment contact between source faces.
    EdgeContact {
        /// Left source face.
        left_face: usize,
        /// Right source face.
        right_face: usize,
        /// Exact interval endpoints.
        endpoints: [Point3; 2],
    },
}

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
enum LowerDimensionalArtifactExactKey {
    Point {
        left_face: usize,
        right_face: usize,
        point: ExactPoint3Key,
    },
    Edge {
        left_face: usize,
        right_face: usize,
        edge: ExactUndirectedPoint3EdgeKey,
    },
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
enum LowerDimensionalArtifactKind {
    Point,
    Edge,
}

type LowerDimensionalArtifactBucketKey = (LowerDimensionalArtifactKind, usize, usize);

#[derive(Default)]
struct LowerDimensionalArtifactBuildIndex {
    exact_keys: BTreeSet<LowerDimensionalArtifactExactKey>,
    keyed_by_bucket: BTreeMap<LowerDimensionalArtifactBucketKey, Vec<usize>>,
    unkeyed_by_bucket: BTreeMap<LowerDimensionalArtifactBucketKey, Vec<usize>>,
}

impl LowerDimensionalArtifactBuildIndex {
    fn push_unique(
        &mut self,
        artifacts: &mut Vec<ArrangementLowerDimensionalArtifact>,
        artifact: ArrangementLowerDimensionalArtifact,
    ) {
        let bucket = lower_dimensional_artifact_bucket_key(&artifact);
        let exact_key = lower_dimensional_artifact_exact_key(&artifact);
        if self.contains_equivalent(artifacts, &artifact, bucket, exact_key.as_ref()) {
            return;
        }
        let index = artifacts.len();
        if let Some(key) = exact_key {
            self.exact_keys.insert(key);
            self.keyed_by_bucket.entry(bucket).or_default().push(index);
        } else {
            self.unkeyed_by_bucket
                .entry(bucket)
                .or_default()
                .push(index);
        }
        artifacts.push(artifact);
    }

    fn contains_equivalent(
        &self,
        artifacts: &[ArrangementLowerDimensionalArtifact],
        artifact: &ArrangementLowerDimensionalArtifact,
        bucket: LowerDimensionalArtifactBucketKey,
        exact_key: Option<&LowerDimensionalArtifactExactKey>,
    ) -> bool {
        if let Some(key) = exact_key {
            if self.exact_keys.contains(key) {
                return true;
            }
            return self
                .unkeyed_by_bucket
                .get(&bucket)
                .is_some_and(|candidates| artifact_matches_any(artifact, artifacts, candidates));
        }
        self.keyed_by_bucket
            .get(&bucket)
            .is_some_and(|candidates| artifact_matches_any(artifact, artifacts, candidates))
            || self
                .unkeyed_by_bucket
                .get(&bucket)
                .is_some_and(|candidates| artifact_matches_any(artifact, artifacts, candidates))
    }
}

fn artifact_matches_any(
    artifact: &ArrangementLowerDimensionalArtifact,
    artifacts: &[ArrangementLowerDimensionalArtifact],
    candidates: &[usize],
) -> bool {
    candidates
        .iter()
        .any(|&candidate| artifact == &artifacts[candidate])
}

/// Validate retained lower-dimensional contact evidence.
pub(crate) fn validate_lower_dimensional_artifacts(
    artifacts: &[ArrangementLowerDimensionalArtifact],
) -> Result<(), ExactArrangementBlocker> {
    for artifact in artifacts {
        if let ArrangementLowerDimensionalArtifact::EdgeContact { endpoints, .. } = artifact {
            match point3_equal(&endpoints[0], &endpoints[1]).value() {
                Some(false) => {}
                Some(true) => return Err(ExactArrangementBlocker::NonManifoldCellComplex),
                None => return Err(ExactArrangementBlocker::UndecidableOrdering),
            }
        }
    }

    validate_lower_dimensional_artifacts_unique(artifacts)?;
    Ok(())
}

pub(crate) fn lower_dimensional_artifact_counts(
    artifacts: &[ArrangementLowerDimensionalArtifact],
) -> (usize, usize, usize) {
    let mut point_contacts = 0;
    let mut edge_contacts = 0;
    let mut edge_endpoints = 0;
    for artifact in artifacts {
        match artifact {
            ArrangementLowerDimensionalArtifact::PointContact { .. } => point_contacts += 1,
            ArrangementLowerDimensionalArtifact::EdgeContact { .. } => {
                edge_contacts += 1;
                edge_endpoints += 2;
            }
        }
    }
    (point_contacts, edge_contacts, edge_endpoints)
}

fn validate_lower_dimensional_artifacts_unique(
    artifacts: &[ArrangementLowerDimensionalArtifact],
) -> Result<(), ExactArrangementBlocker> {
    let mut exact_keys = BTreeSet::new();
    let mut keyed_by_bucket = BTreeMap::<LowerDimensionalArtifactBucketKey, Vec<usize>>::new();
    let mut unkeyed_by_bucket = BTreeMap::<LowerDimensionalArtifactBucketKey, Vec<usize>>::new();
    for (index, artifact) in artifacts.iter().enumerate() {
        let bucket = lower_dimensional_artifact_bucket_key(artifact);
        let exact_key = lower_dimensional_artifact_exact_key(artifact);
        if let Some(candidates) = unkeyed_by_bucket.get(&bucket) {
            for &candidate in candidates {
                if lower_dimensional_artifacts_duplicate(artifact, &artifacts[candidate])? {
                    return Err(ExactArrangementBlocker::NonManifoldCellComplex);
                }
            }
        }
        if exact_key.is_none()
            && let Some(candidates) = keyed_by_bucket.get(&bucket)
        {
            for &candidate in candidates {
                if lower_dimensional_artifacts_duplicate(artifact, &artifacts[candidate])? {
                    return Err(ExactArrangementBlocker::NonManifoldCellComplex);
                }
            }
        }
        if let Some(key) = exact_key {
            if !exact_keys.insert(key) {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
            keyed_by_bucket.entry(bucket).or_default().push(index);
        } else {
            unkeyed_by_bucket.entry(bucket).or_default().push(index);
        }
    }
    Ok(())
}

fn lower_dimensional_artifact_bucket_key(
    artifact: &ArrangementLowerDimensionalArtifact,
) -> LowerDimensionalArtifactBucketKey {
    match artifact {
        ArrangementLowerDimensionalArtifact::PointContact {
            left_face,
            right_face,
            ..
        } => (LowerDimensionalArtifactKind::Point, *left_face, *right_face),
        ArrangementLowerDimensionalArtifact::EdgeContact {
            left_face,
            right_face,
            ..
        } => (LowerDimensionalArtifactKind::Edge, *left_face, *right_face),
    }
}

fn lower_dimensional_artifact_exact_key(
    artifact: &ArrangementLowerDimensionalArtifact,
) -> Option<LowerDimensionalArtifactExactKey> {
    match artifact {
        ArrangementLowerDimensionalArtifact::PointContact {
            left_face,
            right_face,
            point,
        } => Some(LowerDimensionalArtifactExactKey::Point {
            left_face: *left_face,
            right_face: *right_face,
            point: exact_point3_key(point)?,
        }),
        ArrangementLowerDimensionalArtifact::EdgeContact {
            left_face,
            right_face,
            endpoints,
        } => Some(LowerDimensionalArtifactExactKey::Edge {
            left_face: *left_face,
            right_face: *right_face,
            edge: exact_undirected_point3_edge_key(endpoints)?,
        }),
    }
}

fn lower_dimensional_artifacts_duplicate(
    left: &ArrangementLowerDimensionalArtifact,
    right: &ArrangementLowerDimensionalArtifact,
) -> Result<bool, ExactArrangementBlocker> {
    match (left, right) {
        (
            ArrangementLowerDimensionalArtifact::PointContact {
                left_face: left_left_face,
                right_face: left_right_face,
                point: left_point,
            },
            ArrangementLowerDimensionalArtifact::PointContact {
                left_face: right_left_face,
                right_face: right_right_face,
                point: right_point,
            },
        ) if left_left_face == right_left_face && left_right_face == right_right_face => {
            match point3_equal(left_point, right_point).value() {
                Some(duplicate) => Ok(duplicate),
                None => Err(ExactArrangementBlocker::UndecidableOrdering),
            }
        }
        (
            ArrangementLowerDimensionalArtifact::EdgeContact {
                left_face: left_left_face,
                right_face: left_right_face,
                endpoints: left_endpoints,
            },
            ArrangementLowerDimensionalArtifact::EdgeContact {
                left_face: right_left_face,
                right_face: right_right_face,
                endpoints: right_endpoints,
            },
        ) if left_left_face == right_left_face && left_right_face == right_right_face => {
            let endpoints_equal = |reverse_right| {
                let right_first = if reverse_right {
                    &right_endpoints[1]
                } else {
                    &right_endpoints[0]
                };
                let right_second = if reverse_right {
                    &right_endpoints[0]
                } else {
                    &right_endpoints[1]
                };
                let first = point3_equal(&left_endpoints[0], right_first).value();
                let second = point3_equal(&left_endpoints[1], right_second).value();
                if first == Some(false) || second == Some(false) {
                    Some(false)
                } else if first == Some(true) && second == Some(true) {
                    Some(true)
                } else {
                    None
                }
            };
            let same = endpoints_equal(false);
            let reversed = endpoints_equal(true);
            match (same, reversed) {
                (Some(true), _) | (_, Some(true)) => Ok(true),
                (Some(false), Some(false)) => Ok(false),
                _ => Err(ExactArrangementBlocker::UndecidableOrdering),
            }
        }
        _ => Ok(false),
    }
}

/// Retained 2D arrangement for one coplanar carrier-plane face pair.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ArrangementCarrierPlaneOverlay {
    /// Left carrier face.
    pub(crate) left_face: usize,
    /// Right carrier face.
    pub(crate) right_face: usize,
    /// Projection used by the retained exact coplanar predicates.
    pub(crate) projection: CoplanarProjection,
    /// Exact 2D cell overlay of the projected source face boundaries.
    pub(crate) overlay: ExactArrangement2dOverlay,
}

/// Retained 2D arrangement for one source carrier face split by non-coplanar
/// intersection chords.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ArrangementFacePlaneArrangement {
    /// Source mesh side owning the carrier face.
    pub(crate) side: MeshSide,
    /// Source carrier face.
    pub(crate) face: usize,
    /// Projection used to run exact 2D subdivision on the carrier plane.
    pub(crate) projection: CoplanarProjection,
    /// Exact 2D arrangement over the source triangle boundary and retained
    /// graph-vertex intersection chords.
    pub(crate) arrangement: ExactArrangement2d,
    /// Arrangement vertex classification back to original source vertices or
    /// graph vertices. `None` means a local face-interior construction.
    pub(crate) vertex_provenance: Vec<Option<ArrangementFaceCellNode>>,
}

/// Connected arrangement face-cell region.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ArrangementRegion {
    /// Face-cell indices belonging to this connected component.
    pub(crate) face_cells: Vec<usize>,
    /// Undirected adjacency pairs between face-cells in this region.
    pub(crate) adjacent_face_cells: Vec<[usize; 2]>,
    /// Exact edge incidences internal or boundary to this shell component.
    pub(crate) edge_incidences: Vec<ArrangementRegionEdgeIncidence>,
    /// Oriented sides contributed by face cells in this shell component.
    pub(crate) oriented_sides: Vec<ArrangementRegionSide>,
    /// Number of exact boundary edges incident to only one face-cell.
    pub(crate) boundary_edges: usize,
    /// Number of exact boundary edges incident to more than two face-cells.
    pub(crate) non_manifold_edges: usize,
    /// Source sides represented by this connected shell component.
    pub(crate) source_sides: Vec<MeshSide>,
    /// Whether every retained boundary edge has exactly two incident cells.
    pub(crate) closed: bool,
    /// Whether no retained boundary edge has more than two incident cells.
    pub(crate) manifold: bool,
}

pub(crate) fn arrangement_region_topology_counts(
    regions: Option<&[ArrangementRegion]>,
) -> (usize, usize, usize, usize, usize, usize, usize) {
    let Some(regions) = regions else {
        return (0, 0, 0, 0, 0, 0, 0);
    };
    (
        regions.len(),
        regions.iter().map(|region| region.face_cells.len()).sum(),
        regions
            .iter()
            .map(|region| region.adjacent_face_cells.len())
            .sum(),
        regions
            .iter()
            .map(|region| region.edge_incidences.len())
            .sum(),
        regions
            .iter()
            .map(|region| region.oriented_sides.len())
            .sum(),
        regions.iter().map(|region| region.boundary_edges).sum(),
        regions.iter().map(|region| region.non_manifold_edges).sum(),
    )
}

pub(crate) fn validate_arrangement_regions(
    regions: &[ArrangementRegion],
    face_cells: &[ArrangementFaceCell],
) -> Result<(), ExactArrangementBlocker> {
    let face_cell_count = face_cells.len();
    let mut seen_faces = Vec::new();
    for region in regions {
        let Some(region_faces) = sorted_unique_usize_set(&region.face_cells) else {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        };
        if region_faces.iter().any(|&face| face >= face_cell_count) {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        seen_faces.extend(region_faces.iter().copied());
        if region.oriented_sides.len() != region.face_cells.len()
            || region.closed != (region.boundary_edges == 0)
            || region.manifold != (region.non_manifold_edges == 0)
            || region.boundary_edges + region.non_manifold_edges > region.edge_incidences.len()
        {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        let Some(region_membership) =
            ArrangementRegionComponentMembership::new(&region_faces, face_cell_count)
        else {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        };
        for pair in &region.adjacent_face_cells {
            if pair[0] == pair[1]
                || pair[0] >= face_cell_count
                || pair[1] >= face_cell_count
                || !region_membership.contains(pair[0])
                || !region_membership.contains(pair[1])
            {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
        }
        for incidence in &region.edge_incidences {
            let Some(incidence_faces) = sorted_unique_usize_set(&incidence.face_cells) else {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            };
            if incidence_faces
                .iter()
                .any(|&face| !region_membership.contains(face))
            {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
            let incident_count =
                regularized_incident_sheet_count(&incidence.face_cells, face_cells);
            if incidence.boundary != (incident_count == 1)
                || incidence.non_manifold != (incident_count > 2)
            {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
        }
        for side in &region.oriented_sides {
            if !region_membership.contains(side.face_cell) {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
        }
    }
    let Some(seen_faces) = sorted_unique_usize_set(&seen_faces) else {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    };
    if seen_faces.len() != face_cell_count {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    Ok(())
}

/// Exact edge incidence for one connected arrangement shell.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ArrangementRegionEdgeIncidence {
    /// Canonical exact boundary edge.
    pub(crate) edge: [ArrangementFaceCellNode; 2],
    /// Face-cells in the owning shell incident to this edge.
    pub(crate) face_cells: Vec<usize>,
    /// Whether this edge is used by exactly one retained face-cell.
    pub(crate) boundary: bool,
    /// Whether this edge has more than two retained incident face-cells.
    pub(crate) non_manifold: bool,
}

/// Oriented side evidence for a face cell in a shell component.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ArrangementRegionSide {
    /// Face-cell contributing this side.
    pub(crate) face_cell: usize,
    /// Source side whose boundary owns the face-cell carrier.
    pub(crate) source: MeshSide,
    /// Carrier face index in the source mesh.
    pub(crate) source_face: usize,
    /// Boundary node order as emitted by the carrier-face arrangement.
    pub(crate) boundary: Vec<ArrangementFaceCellNode>,
}

/// Exact volume-region node induced by closed manifold shell components.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ArrangementVolumeRegion {
    /// Volume-region index in the arrangement volume graph.
    pub(crate) index: usize,
    /// Whether this is the unbounded exterior region.
    pub(crate) exterior: bool,
    /// Shell components bounding this volume region.
    pub(crate) boundary_shells: Vec<usize>,
    /// Source sides whose closed shell interiors contribute this volume.
    pub(crate) source_sides: Vec<MeshSide>,
}

/// Adjacency between two volume regions across one closed shell component.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ArrangementVolumeAdjacency {
    /// Shell component crossed by this adjacency.
    pub(crate) shell_region: usize,
    /// Unbounded/outside volume-region index for the shell.
    pub(crate) exterior_volume: usize,
    /// Bounded/interior volume-region index for the shell.
    pub(crate) interior_volume: usize,
    /// Face-cells forming the separating shell.
    pub(crate) separating_face_cells: Vec<usize>,
    /// Oriented face-cell sides making this volume boundary explicit.
    pub(crate) oriented_face_sides: Vec<ArrangementVolumeFaceSide>,
}

/// Oriented face-cell side crossing between two volume regions.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ArrangementVolumeFaceSide {
    /// Face-cell contributing this volume side.
    pub(crate) face_cell: usize,
    /// Source side whose boundary owns the face-cell carrier.
    pub(crate) source: MeshSide,
    /// Carrier face index in the source mesh.
    pub(crate) source_face: usize,
    /// Boundary node order as emitted by the carrier-face arrangement.
    pub(crate) boundary: Vec<ArrangementFaceCellNode>,
    /// Volume on the outside of the owning shell.
    pub(crate) exterior_volume: usize,
    /// Volume on the inside of the owning shell.
    pub(crate) interior_volume: usize,
}

/// Exact 3D arrangement over two source meshes.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactArrangement3d {
    /// Exact vertices from source meshes and constructed intersections.
    pub(crate) vertices: Vec<ArrangementVertex>,
    /// Exact split edges from source edges and graph vertices.
    pub(crate) edges: Vec<ArrangementEdge>,
    /// Face cells retained in carrier-face coordinates.
    pub(crate) face_cells: Vec<ArrangementFaceCell>,
    /// Per-carrier-plane 2D arrangements for retained coplanar overlap graphs.
    pub(crate) carrier_plane_overlays: Vec<ArrangementCarrierPlaneOverlay>,
    /// Per-source-face 2D arrangements for non-coplanar intersection chords.
    pub(crate) face_plane_arrangements: Vec<ArrangementFacePlaneArrangement>,
    /// Lower-dimensional contacts retained by regularization policy.
    pub(crate) lower_dimensional_artifacts: Vec<ArrangementLowerDimensionalArtifact>,
    /// Connected face-cell regions/shell components retained by the arrangement.
    pub(crate) shells_or_regions: Option<Vec<ArrangementRegion>>,
    /// Explicit volume regions induced by closed manifold shell components.
    pub(crate) volume_regions: Option<Vec<ArrangementVolumeRegion>>,
    /// Volume adjacency through closed shell components.
    pub(crate) volume_adjacencies: Option<Vec<ArrangementVolumeAdjacency>>,
    /// Retained exact intersection graph.
    pub(crate) graph: ExactIntersectionGraph,
    /// Checked split topology, when exact ordering/equality completed.
    pub(crate) topology: Option<ExactSplitTopologyPlan>,
    /// Checked face-region loop plan, when available.
    pub(crate) region_plan: Option<ExactFaceRegionPlan>,
    /// Explicit blockers for incomplete exact arrangement construction.
    pub(crate) blockers: Vec<ExactArrangementBlocker>,
}

/// Public arrangement entry point for the exact Boolean pipeline.
pub(crate) type ExactArrangement = ExactArrangement3d;

/// Borrowed exact arrangement view.
#[derive(Clone, Copy, Debug)]
pub struct ArrangementView<'a> {
    arrangement: &'a ExactArrangement3d,
}

/// Borrowed arrangement vertex view.
#[derive(Clone, Copy, Debug)]
pub struct ArrangementVertexRef<'a> {
    arrangement: &'a ExactArrangement3d,
    index: usize,
}

/// Borrowed arrangement edge view.
#[derive(Clone, Copy, Debug)]
pub struct ArrangementEdgeRef<'a> {
    arrangement: &'a ExactArrangement3d,
    index: usize,
}

/// Borrowed arrangement face-cell view.
#[derive(Clone, Copy, Debug)]
pub struct ArrangementFaceCellRef<'a> {
    arrangement: &'a ExactArrangement3d,
    index: usize,
}

impl<'a> ArrangementView<'a> {
    /// Borrow a retained exact arrangement as a query view.
    pub(crate) const fn new(arrangement: &'a ExactArrangement3d) -> Self {
        Self { arrangement }
    }

    /// Return whether construction reached a blocker-free arrangement handoff.
    pub fn is_complete(self) -> bool {
        self.arrangement.is_complete()
    }

    /// Validate retained arrangement state without cloning arrangement storage.
    pub fn validate_retained_state(self) -> Result<(), ExactMeshError> {
        self.arrangement.validate().map_err(|blocker| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::ExactConstructionFailure,
                format!("retained arrangement validation failed: {blocker:?}"),
            ))
        })
    }

    /// Retained arrangement vertex count.
    pub fn vertex_count(self) -> usize {
        self.arrangement.vertices.len()
    }

    /// Retained arrangement edge count.
    pub fn edge_count(self) -> usize {
        self.arrangement.edges.len()
    }

    /// Retained arrangement face-cell count.
    pub fn face_cell_count(self) -> usize {
        self.arrangement.face_cells.len()
    }

    /// Retained connected face-cell region count.
    pub fn region_count(self) -> usize {
        self.arrangement
            .shells_or_regions
            .as_ref()
            .map_or(0, Vec::len)
    }

    /// Retained volume-region count.
    pub fn volume_region_count(self) -> usize {
        self.arrangement.volume_regions.as_ref().map_or(0, Vec::len)
    }

    /// Retained volume-adjacency count.
    pub fn volume_adjacency_count(self) -> usize {
        self.arrangement
            .volume_adjacencies
            .as_ref()
            .map_or(0, Vec::len)
    }

    /// Retained lower-dimensional artifact count.
    pub fn lower_dimensional_artifact_count(self) -> usize {
        self.arrangement.lower_dimensional_artifacts.len()
    }

    /// Retained blocker count.
    pub fn blocker_count(self) -> usize {
        self.arrangement.blockers.len()
    }

    /// Borrow one arrangement vertex by index.
    pub fn vertex(self, index: usize) -> Option<ArrangementVertexRef<'a>> {
        (index < self.arrangement.vertices.len()).then_some(ArrangementVertexRef {
            arrangement: self.arrangement,
            index,
        })
    }

    /// Borrow one arrangement vertex by index, returning a typed blocker when absent.
    pub fn require_vertex(self, index: usize) -> Result<ArrangementVertexRef<'a>, ExactMeshError> {
        self.vertex(index).ok_or_else(|| {
            ExactMeshError::one(
                ExactMeshBlocker::new(
                    ExactMeshBlockerKind::IndexOutOfBounds,
                    format!(
                        "arrangement vertex index {index} is out of bounds for {} retained vertices",
                        self.vertex_count()
                    ),
                )
                .with_vertex(index),
            )
        })
    }

    /// Borrow one arrangement edge by index.
    pub fn edge(self, index: usize) -> Option<ArrangementEdgeRef<'a>> {
        (index < self.arrangement.edges.len()).then_some(ArrangementEdgeRef {
            arrangement: self.arrangement,
            index,
        })
    }

    /// Borrow one arrangement edge by index, returning a typed blocker when absent.
    pub fn require_edge(self, index: usize) -> Result<ArrangementEdgeRef<'a>, ExactMeshError> {
        self.edge(index).ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::IndexOutOfBounds,
                format!(
                    "arrangement edge index {index} is out of bounds for {} retained edges",
                    self.edge_count()
                ),
            ))
        })
    }

    /// Borrow one arrangement face cell by index.
    pub fn face_cell(self, index: usize) -> Option<ArrangementFaceCellRef<'a>> {
        (index < self.arrangement.face_cells.len()).then_some(ArrangementFaceCellRef {
            arrangement: self.arrangement,
            index,
        })
    }

    /// Borrow one arrangement face cell by index, returning a typed blocker when absent.
    pub fn require_face_cell(
        self,
        index: usize,
    ) -> Result<ArrangementFaceCellRef<'a>, ExactMeshError> {
        self.face_cell(index).ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::IndexOutOfBounds,
                format!(
                    "arrangement face-cell index {index} is out of bounds for {} retained face cells",
                    self.face_cell_count()
                ),
            ))
        })
    }

    /// Iterate borrowed arrangement vertices.
    pub fn vertices(self) -> impl Iterator<Item = ArrangementVertexRef<'a>> + 'a {
        (0..self.arrangement.vertices.len()).map(move |index| ArrangementVertexRef {
            arrangement: self.arrangement,
            index,
        })
    }

    /// Iterate borrowed arrangement edges.
    pub fn edges(self) -> impl Iterator<Item = ArrangementEdgeRef<'a>> + 'a {
        (0..self.arrangement.edges.len()).map(move |index| ArrangementEdgeRef {
            arrangement: self.arrangement,
            index,
        })
    }

    /// Iterate borrowed arrangement face cells.
    pub fn face_cells(self) -> impl Iterator<Item = ArrangementFaceCellRef<'a>> + 'a {
        (0..self.arrangement.face_cells.len()).map(move |index| ArrangementFaceCellRef {
            arrangement: self.arrangement,
            index,
        })
    }
}

impl<'a> ArrangementVertexRef<'a> {
    /// Vertex index in the retained arrangement.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Exact retained vertex coordinate.
    pub fn point(self) -> Result<&'a Point3, ExactMeshError> {
        retained_arrangement_vertex(self.arrangement, self.index).map(|vertex| &vertex.point)
    }

    /// Number of retained source/construction provenance records.
    pub fn provenance_count(self) -> Result<usize, ExactMeshError> {
        retained_arrangement_vertex(self.arrangement, self.index)
            .map(|vertex| vertex.provenance.len())
    }
}

impl<'a> ArrangementEdgeRef<'a> {
    /// Edge index in the retained arrangement.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Endpoint arrangement vertex indices.
    pub fn vertices(self) -> Result<[usize; 2], ExactMeshError> {
        retained_arrangement_edge(self.arrangement, self.index).map(|edge| edge.vertices)
    }

    /// Number of retained source/construction provenance records.
    pub fn provenance_count(self) -> Result<usize, ExactMeshError> {
        retained_arrangement_edge(self.arrangement, self.index).map(|edge| edge.provenance.len())
    }
}

impl<'a> ArrangementFaceCellRef<'a> {
    /// Face-cell index in the retained arrangement.
    pub const fn index(self) -> usize {
        self.index
    }

    /// Source carrier face index.
    pub fn carrier_face(self) -> Result<usize, ExactMeshError> {
        retained_arrangement_face_cell(self.arrangement, self.index)
            .map(|face_cell| face_cell.carrier.face)
    }

    /// Boundary node count in carrier-face order.
    pub fn boundary_node_count(self) -> Result<usize, ExactMeshError> {
        retained_arrangement_face_cell(self.arrangement, self.index)
            .map(|face_cell| face_cell.boundary.len())
    }

    /// Boundary point count in carrier-face order.
    pub fn boundary_point_count(self) -> Result<usize, ExactMeshError> {
        retained_arrangement_face_cell(self.arrangement, self.index)
            .map(|face_cell| face_cell.boundary_points.len())
    }

    /// Iterate exact boundary coordinates in carrier-face order.
    pub fn boundary_points(self) -> Result<impl Iterator<Item = &'a Point3> + 'a, ExactMeshError> {
        retained_arrangement_face_cell(self.arrangement, self.index)
            .map(|face_cell| face_cell.boundary_points.iter())
    }

    /// Return whether this face-cell retained an opposite-mesh classification.
    pub fn has_opposite_classification(self) -> Result<bool, ExactMeshError> {
        retained_arrangement_face_cell(self.arrangement, self.index)
            .map(|face_cell| face_cell.opposite.is_some())
    }
}

fn retained_arrangement_vertex(
    arrangement: &ExactArrangement3d,
    vertex: usize,
) -> Result<&ArrangementVertex, ExactMeshError> {
    arrangement.vertices.get(vertex).ok_or_else(|| {
        ExactMeshError::one(
            ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!("retained arrangement vertex {vertex} has no vertex row"),
            )
            .with_vertex(vertex),
        )
    })
}

fn retained_arrangement_edge(
    arrangement: &ExactArrangement3d,
    edge: usize,
) -> Result<&ArrangementEdge, ExactMeshError> {
    arrangement.edges.get(edge).ok_or_else(|| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::StaleFactReplay,
            format!("retained arrangement edge {edge} has no edge row"),
        ))
    })
}

fn retained_arrangement_face_cell(
    arrangement: &ExactArrangement3d,
    face_cell: usize,
) -> Result<&ArrangementFaceCell, ExactMeshError> {
    arrangement.face_cells.get(face_cell).ok_or_else(|| {
        ExactMeshError::one(
            ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!("retained arrangement face-cell {face_cell} has no face-cell row"),
            )
            .with_face(face_cell),
        )
    })
}

/// Freshness status for a retained exact 3D arrangement.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactArrangementFreshness {
    /// The arrangement replays exactly from the current source operands.
    Current,
    /// Rebuilding the arrangement from the source operands is currently blocked.
    SourceReplayBlocked,
    /// The source operands rebuild, but the retained arrangement no longer matches.
    StaleArrangement,
}

/// Exact topology-assembly bridge status for a retained arrangement.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactTopologyAssemblyStatus {
    /// The arrangement, split topology, and face-region loops replay from the
    /// source operands with no arrangement blockers.
    Complete,
    /// The arrangement could not be rebuilt from the source operands.
    SourceReplayBlocked,
    /// Rebuilding from sources produced different retained arrangement state.
    StaleArrangement,
    /// Exact split topology was not retained.
    MissingSplitTopology,
    /// Exact face-region boundary loops were not retained.
    MissingRegionPlan,
    /// The topology bridge exists, but arrangement construction retained
    /// explicit blockers.
    ArrangementBlocked,
}

impl ExactTopologyAssemblyStatus {
    /// Return whether retained arrangement topology completed the exact
    /// graph/split/region bridge.
    pub(crate) const fn is_complete(self) -> bool {
        matches!(self, Self::Complete)
    }
}

/// Compact retained-topology report connecting graph/split plans to arrangement
/// topology.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactTopologyAssemblyReport {
    /// Overall topology-assembly status.
    pub(crate) status: ExactTopologyAssemblyStatus,
    /// Source replay freshness for the retained arrangement.
    pub(crate) freshness: ExactArrangementFreshness,
    /// Arrangement blockers retained by the topology bridge.
    pub(crate) blockers: Vec<ExactArrangementBlocker>,
    /// Retained face-pair records in the exact intersection graph.
    pub(crate) graph_face_pairs: usize,
    /// Retained exact intersection events in the graph.
    pub(crate) graph_events: usize,
    /// Merged graph vertices in the split topology plan.
    pub(crate) split_graph_vertices: usize,
    /// Ordered split-edge chains in the split topology plan.
    pub(crate) split_edge_chains: usize,
    /// Graph-vertex references across all split-edge chains.
    pub(crate) split_graph_vertex_references: usize,
    /// Split vertex lookups that could not be resolved.
    pub(crate) unresolved_vertex_lookups: usize,
    /// Equality checks that could not be certified while merging split points.
    pub(crate) unresolved_equalities: usize,
    /// Edge parameter comparisons that could not be certified.
    pub(crate) unknown_orderings: usize,
    /// Retained face-region boundary loops.
    pub(crate) region_boundaries: usize,
    /// Boundary nodes across retained face-region loops.
    pub(crate) region_boundary_nodes: usize,
    /// Arrangement vertices retained for topology consumers.
    pub(crate) arrangement_vertices: usize,
    /// Arrangement edges retained for topology consumers.
    pub(crate) arrangement_edges: usize,
    /// Arrangement face cells retained for topology consumers.
    pub(crate) arrangement_face_cells: usize,
    /// Boundary nodes across retained arrangement face cells.
    pub(crate) arrangement_face_cell_boundary_nodes: usize,
    /// Boundary coordinates across retained arrangement face cells.
    pub(crate) arrangement_face_cell_boundary_points: usize,
    /// Connected arrangement shell/region components retained for topology.
    pub(crate) arrangement_regions: usize,
    /// Face-cell memberships across retained arrangement regions.
    pub(crate) arrangement_region_face_cells: usize,
    /// Adjacency pairs across retained arrangement regions.
    pub(crate) arrangement_region_adjacencies: usize,
    /// Edge incidences across retained arrangement regions.
    pub(crate) arrangement_region_edge_incidences: usize,
    /// Oriented sides across retained arrangement regions.
    pub(crate) arrangement_region_oriented_sides: usize,
    /// Boundary edges across retained arrangement regions.
    pub(crate) arrangement_region_boundary_edges: usize,
    /// Non-manifold edges across retained arrangement regions.
    pub(crate) arrangement_region_non_manifold_edges: usize,
    /// Retained carrier-plane overlays.
    pub(crate) carrier_plane_overlays: usize,
    /// Retained per-source-face plane arrangements.
    pub(crate) face_plane_arrangements: usize,
    /// Lower-dimensional artifacts retained under regularization policy.
    pub(crate) lower_dimensional_artifacts: usize,
    /// Retained point-contact lower-dimensional artifacts.
    pub(crate) lower_dimensional_point_contacts: usize,
    /// Retained edge-contact lower-dimensional artifacts.
    pub(crate) lower_dimensional_edge_contacts: usize,
    /// Endpoints carried by retained edge-contact artifacts.
    pub(crate) lower_dimensional_edge_endpoints: usize,
    /// Explicit volume regions retained by the arrangement.
    pub(crate) volume_regions: usize,
    /// Explicit volume adjacencies retained by the arrangement.
    pub(crate) volume_adjacencies: usize,
    /// Oriented face-side witnesses carried by retained volume adjacencies.
    pub(crate) volume_adjacency_face_sides: usize,
    /// Separating face-cell references carried by retained volume adjacencies.
    pub(crate) volume_adjacency_separating_faces: usize,
}

impl ExactTopologyAssemblyReport {
    /// Return whether this report represents a complete topology bridge.
    pub(crate) fn is_complete(&self) -> bool {
        self.status.is_complete()
    }

    /// Validate local topology-assembly report shape without source replay.
    pub(crate) fn validate(&self) -> Result<(), ExactArrangementBlocker> {
        let has_non_ownership_blocker = self
            .blockers
            .iter()
            .any(|blocker| *blocker != ExactArrangementBlocker::UnresolvedRegionClassification);
        match self.status {
            ExactTopologyAssemblyStatus::Complete => {
                if self.freshness != ExactArrangementFreshness::Current || has_non_ownership_blocker
                {
                    return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
                }
            }
            ExactTopologyAssemblyStatus::SourceReplayBlocked => {
                if self.freshness != ExactArrangementFreshness::SourceReplayBlocked {
                    return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
                }
            }
            ExactTopologyAssemblyStatus::StaleArrangement => {
                if self.freshness != ExactArrangementFreshness::StaleArrangement {
                    return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
                }
            }
            ExactTopologyAssemblyStatus::MissingSplitTopology => {
                if self.freshness != ExactArrangementFreshness::Current
                    || self.split_graph_vertices != 0
                {
                    return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
                }
            }
            ExactTopologyAssemblyStatus::MissingRegionPlan => {
                if self.freshness != ExactArrangementFreshness::Current
                    || self.region_boundaries != 0
                {
                    return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
                }
            }
            ExactTopologyAssemblyStatus::ArrangementBlocked => {
                if self.freshness != ExactArrangementFreshness::Current
                    || !has_non_ownership_blocker
                {
                    return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
                }
            }
        }
        if (self.volume_adjacencies == 0
            && (self.volume_adjacency_face_sides != 0
                || self.volume_adjacency_separating_faces != 0))
            || (self.volume_adjacencies != 0
                && (self.volume_adjacency_face_sides == 0
                    || self.volume_adjacency_separating_faces == 0))
            || self.volume_adjacency_face_sides < self.volume_adjacencies
            || self.volume_adjacency_separating_faces < self.volume_adjacencies
        {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let has_split_edge_chains = self.split_edge_chains != 0;
        let has_split_graph_vertex_references = self.split_graph_vertex_references != 0;
        let split_vertices_missing_for_retained_splits = self.split_graph_vertices == 0
            && (has_split_edge_chains || has_split_graph_vertex_references);
        let split_references_without_chains =
            !has_split_edge_chains && has_split_graph_vertex_references;
        let split_chains_missing_references =
            has_split_edge_chains && self.split_graph_vertex_references < self.split_edge_chains;
        if split_vertices_missing_for_retained_splits
            || split_references_without_chains
            || split_chains_missing_references
        {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let Some(min_region_boundary_nodes) = self.region_boundaries.checked_mul(3) else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        if (self.region_boundaries == 0 && self.region_boundary_nodes != 0)
            || (self.region_boundaries != 0
                && self.region_boundary_nodes < min_region_boundary_nodes)
        {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let Some(expected_edge_endpoints) = self.lower_dimensional_edge_contacts.checked_mul(2)
        else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        let Some(expected_lower_dimensional_artifacts) = self
            .lower_dimensional_point_contacts
            .checked_add(self.lower_dimensional_edge_contacts)
        else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        if self.lower_dimensional_artifacts != expected_lower_dimensional_artifacts
            || self.lower_dimensional_edge_endpoints != expected_edge_endpoints
        {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let Some(min_face_cell_boundary_nodes) = self.arrangement_face_cells.checked_mul(3) else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        if self.arrangement_face_cell_boundary_nodes != self.arrangement_face_cell_boundary_points
            || (self.arrangement_face_cells == 0 && self.arrangement_face_cell_boundary_nodes != 0)
            || (self.arrangement_face_cells != 0
                && self.arrangement_face_cell_boundary_nodes < min_face_cell_boundary_nodes)
        {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let Some(arrangement_region_problem_edges) = self
            .arrangement_region_boundary_edges
            .checked_add(self.arrangement_region_non_manifold_edges)
        else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        if (self.arrangement_face_cells == 0
            && (self.arrangement_regions != 0
                || self.arrangement_region_face_cells != 0
                || self.arrangement_region_adjacencies != 0
                || self.arrangement_region_edge_incidences != 0
                || self.arrangement_region_oriented_sides != 0
                || self.arrangement_region_boundary_edges != 0
                || self.arrangement_region_non_manifold_edges != 0))
            || (self.arrangement_face_cells != 0 && self.arrangement_regions == 0)
            || self.arrangement_region_face_cells != self.arrangement_face_cells
            || self.arrangement_region_oriented_sides != self.arrangement_region_face_cells
            || arrangement_region_problem_edges > self.arrangement_region_edge_incidences
        {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        Ok(())
    }
}

impl ExactArrangement3d {
    /// Borrow this exact arrangement through the lightweight query view API.
    pub(crate) const fn view(&self) -> ArrangementView<'_> {
        ArrangementView::new(self)
    }

    /// Build a retained exact arrangement from two meshes with explicit policy.
    pub(crate) fn from_meshes_with_policy(
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<Self, ExactMeshError> {
        let graph = build_validated_intersection_graph(left, right)?;
        Self::from_intersection_graph_with_policy(graph, left, right, policy)
    }

    /// Build a retained exact arrangement from an already retained
    /// intersection graph.
    ///
    /// This validates the graph's source handles against `left` and `right`,
    /// then consumes it directly without replay-building the graph. Public
    /// exact-computation boundaries that require full source replay should
    /// perform that check before calling this constructor.
    pub(crate) fn from_intersection_graph_with_policy(
        graph: ExactIntersectionGraph,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<Self, ExactMeshError> {
        graph
            .validate_against_meshes(left, right)
            .map_err(|error| {
                ExactMeshError::one(ExactMeshBlocker::new(
                    ExactMeshBlockerKind::StaleFactReplay,
                    format!("retained exact intersection graph failed mesh handoff: {error:?}"),
                ))
            })?;
        Self::from_source_certified_intersection_graph_with_policy(graph, left, right, policy)
    }

    /// Build a retained exact arrangement from a source-certified intersection graph.
    ///
    /// Callers must only use this after the graph's source handles have already
    /// been certified against `left` and `right`; the arrangement builder then
    /// consumes the retained graph evidence without replaying that certificate.
    pub(crate) fn from_source_certified_intersection_graph_with_policy(
        graph: ExactIntersectionGraph,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<Self, ExactMeshError> {
        let mut blockers = match graph.validate() {
            Ok(()) => Vec::new(),
            Err(error) => vec![ExactArrangementBlocker::InvalidIntersectionGraph(
                error.into(),
            )],
        };
        if graph.has_unknowns() {
            blockers.push(ExactArrangementBlocker::UnresolvedIntersection);
        }

        let topology = match graph.checked_split_topology_plan() {
            Ok(topology) => Some(topology),
            Err(report) => {
                extend_split_plan_blockers(&mut blockers, &report);
                None
            }
        };

        let region_plan = if topology.is_some() {
            match graph.face_split_geometry_plan(left, right) {
                Ok(geometry) => {
                    let report = geometry.validate_boundary_incidence(left, right);
                    if !report.is_valid() {
                        extend_split_plan_blockers(&mut blockers, &report);
                        None
                    } else {
                        let regions = geometry.region_plan(left, right);
                        let region_report = regions.validate(left, right);
                        if !region_report.is_valid() {
                            extend_split_plan_blockers(&mut blockers, &region_report);
                            None
                        } else {
                            Some(regions)
                        }
                    }
                }
                Err(_) => {
                    blockers.push(ExactArrangementBlocker::UnresolvedIntersection);
                    None
                }
            }
        } else {
            None
        };

        let carrier_plane_overlays = carrier_plane_overlays(&graph, left, right, &mut blockers);
        let lower_dimensional_artifacts =
            lower_dimensional_artifacts(&graph, left, right, policy, &mut blockers);
        let face_plane_arrangements = face_plane_arrangements(
            topology.as_ref(),
            left,
            right,
            &carrier_plane_overlays,
            &mut blockers,
        );
        let vertices = arrangement_vertices(
            left,
            right,
            topology.as_ref(),
            &carrier_plane_overlays,
            &face_plane_arrangements,
            &mut blockers,
        );
        let edges = arrangement_edges(
            topology.as_ref(),
            &vertices,
            &carrier_plane_overlays,
            &face_plane_arrangements,
        );
        let face_cells = arrangement_face_cells(
            left,
            right,
            policy,
            region_plan.as_ref(),
            &carrier_plane_overlays,
            &face_plane_arrangements,
            &mut blockers,
        );
        let shells_or_regions = Some(arrangement_regions(&face_cells, &mut blockers));
        let (volume_regions, volume_adjacencies) = arrangement_volume_graph(
            shells_or_regions.as_ref().map_or(&[][..], Vec::as_slice),
            &face_cells,
            left,
            right,
            &mut blockers,
        );
        validate_arrangement_volume_graph(
            shells_or_regions.as_ref().map_or(&[][..], Vec::as_slice),
            &face_cells,
            volume_regions.as_deref(),
            volume_adjacencies.as_deref(),
            &mut blockers,
        );
        let has_mixed_source_open_sheet_complex =
            shells_or_regions.as_ref().is_some_and(|regions| {
                regions.iter().any(|region| {
                    region.boundary_edges > 0
                        && region.non_manifold_edges > 0
                        && region.source_sides.len() > 1
                })
            });
        let regularized_closed_solid_sheet_complex = has_mixed_source_open_sheet_complex
            && policy == ExactRegularizationPolicy::REGULARIZED_SOLID
            && left.facts().mesh.closed_manifold
            && right.facts().mesh.closed_manifold;
        let retained_sheet_artifact_complex = policy == ExactRegularizationPolicy::RETAIN_ARTIFACTS;
        if has_mixed_source_open_sheet_complex
            && !regularized_closed_solid_sheet_complex
            && !retained_sheet_artifact_complex
        {
            push_unique_blocker(
                &mut blockers,
                ExactArrangementBlocker::UnregularizedOpenSheetComplex,
            );
        }
        if shells_or_regions
            .as_ref()
            .is_some_and(|regions| regions.iter().any(|region| region.non_manifold_edges > 0))
        {
            let regularizable_closed_coincident =
                shells_or_regions.as_ref().is_some_and(|regions| {
                    regions.iter().all(|region| {
                        region.non_manifold_edges == 0
                            || (region.boundary_edges == 0 && region.source_sides.len() > 1)
                    })
                });
            if !regularizable_closed_coincident {
                // Closed coincident source sheets can still be selected from
                // exact face labels and canonicalized away in simplification.
                // Closed regularized solid open sheet contacts are also
                // supportable: the volume-boundary materializer may drop the
                // lower-dimensional contact while retaining exact provenance
                // for the selected cells. Explicit artifact-retention policy
                // likewise keeps mixed-source open sheet cells inspectable
                // without claiming regularized solid output. Other open or
                // non-regularized sheet complexes still report blockers.
                let blocker = if has_mixed_source_open_sheet_complex {
                    if regularized_closed_solid_sheet_complex || retained_sheet_artifact_complex {
                        None
                    } else {
                        Some(ExactArrangementBlocker::UnregularizedCoincidentSheetComplex)
                    }
                } else {
                    Some(ExactArrangementBlocker::NonManifoldCellComplex)
                };
                if let Some(blocker) = blocker {
                    push_unique_blocker(&mut blockers, blocker);
                }
            }
        }
        if policy == ExactRegularizationPolicy::REGULARIZED_SOLID
            && (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
            && shells_or_regions
                .as_ref()
                .is_some_and(|regions| regions.iter().any(|region| region.boundary_edges > 0))
        {
            blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
        }

        Ok(Self {
            vertices,
            edges,
            face_cells,
            carrier_plane_overlays,
            face_plane_arrangements,
            lower_dimensional_artifacts,
            shells_or_regions,
            volume_regions,
            volume_adjacencies,
            graph,
            topology,
            region_plan,
            blockers,
        })
    }

    /// Return whether construction reached a blocker-free arrangement handoff.
    pub(crate) fn is_complete(&self) -> bool {
        self.blockers.is_empty()
    }

    /// Validate arrangement-internal consistency without replaying source meshes.
    pub(crate) fn validate(&self) -> Result<(), ExactArrangementBlocker> {
        validate_lower_dimensional_artifacts(&self.lower_dimensional_artifacts)?;
        validate_arrangement_face_cells(&self.face_cells)?;
        self.graph
            .validate()
            .map_err(|error| ExactArrangementBlocker::InvalidIntersectionGraph(error.into()))?;
        let mut face_pair_relations = BTreeMap::new();
        for pair in &self.graph.face_pairs {
            face_pair_relations.insert((pair.left_face, pair.right_face), pair.relation);
        }
        for artifact in &self.lower_dimensional_artifacts {
            let (left_face, right_face) = match artifact {
                ArrangementLowerDimensionalArtifact::PointContact {
                    left_face,
                    right_face,
                    ..
                }
                | ArrangementLowerDimensionalArtifact::EdgeContact {
                    left_face,
                    right_face,
                    ..
                } => (*left_face, *right_face),
            };
            let Some(relation) = face_pair_relations.get(&(left_face, right_face)).copied() else {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            };
            if !matches!(
                relation,
                super::graph::intersection::MeshFacePairRelation::Candidate
                    | super::graph::intersection::MeshFacePairRelation::CoplanarTouching
            ) {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
        }
        if self.graph.has_unknowns()
            && !self
                .blockers
                .contains(&ExactArrangementBlocker::UnresolvedIntersection)
        {
            return Err(ExactArrangementBlocker::UnresolvedIntersection);
        }
        if let Some(topology) = &self.topology {
            let report = topology.validate();
            if !report.is_valid() {
                let mut blockers = Vec::new();
                extend_split_plan_blockers(&mut blockers, &report);
                return Err(blockers
                    .into_iter()
                    .next()
                    .unwrap_or(ExactArrangementBlocker::UnresolvedIntersection));
            }
        } else if self.blockers.is_empty() {
            return Err(ExactArrangementBlocker::UnresolvedIntersection);
        }
        if self.region_plan.is_some() && self.topology.is_none() {
            return Err(ExactArrangementBlocker::UnresolvedIntersection);
        }
        let Some(shells_or_regions) = &self.shells_or_regions else {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        };
        validate_arrangement_regions(shells_or_regions, &self.face_cells)?;
        if let Some(blocker) = self.retained_volume_graph_blockers().into_iter().next() {
            return Err(blocker);
        }
        if shells_or_regions.iter().any(|region| {
            region
                .face_cells
                .iter()
                .any(|&face_cell| face_cell >= self.face_cells.len())
        }) {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        Ok(())
    }

    /// Validate retained volume-region labels against shell orientation and
    /// volume adjacency evidence already stored in this arrangement.
    pub(crate) fn retained_volume_graph_blockers(&self) -> Vec<ExactArrangementBlocker> {
        let mut blockers = Vec::new();
        validate_arrangement_volume_graph(
            self.shells_or_regions.as_deref().unwrap_or(&[]),
            &self.face_cells,
            self.volume_regions.as_deref(),
            self.volume_adjacencies.as_deref(),
            &mut blockers,
        );
        blockers
    }

    /// Classify arrangement freshness under an explicit regularization policy.
    pub(crate) fn freshness_against_sources_with_policy(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> ExactArrangementFreshness {
        if self.validate().is_err() {
            return ExactArrangementFreshness::StaleArrangement;
        }
        match Self::from_meshes_with_policy(left, right, policy) {
            Ok(replay) if replay == *self => ExactArrangementFreshness::Current,
            Ok(_) => ExactArrangementFreshness::StaleArrangement,
            Err(_) => ExactArrangementFreshness::SourceReplayBlocked,
        }
    }

    /// Report the retained topology bridge under an explicit regularization
    /// policy.
    pub(crate) fn topology_assembly_report_with_policy(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> ExactTopologyAssemblyReport {
        let freshness = self.freshness_against_sources_with_policy(left, right, policy);
        let status = match freshness {
            ExactArrangementFreshness::SourceReplayBlocked => {
                ExactTopologyAssemblyStatus::SourceReplayBlocked
            }
            ExactArrangementFreshness::StaleArrangement => {
                ExactTopologyAssemblyStatus::StaleArrangement
            }
            ExactArrangementFreshness::Current if self.topology.is_none() => {
                ExactTopologyAssemblyStatus::MissingSplitTopology
            }
            ExactArrangementFreshness::Current if self.region_plan.is_none() => {
                ExactTopologyAssemblyStatus::MissingRegionPlan
            }
            ExactArrangementFreshness::Current
                if self.blockers.iter().any(|blocker| {
                    *blocker != ExactArrangementBlocker::UnresolvedRegionClassification
                }) =>
            {
                ExactTopologyAssemblyStatus::ArrangementBlocked
            }
            ExactArrangementFreshness::Current => ExactTopologyAssemblyStatus::Complete,
        };
        let (
            split_graph_vertices,
            split_edge_chains,
            split_graph_vertex_references,
            unresolved_vertex_lookups,
            unresolved_equalities,
            unknown_orderings,
        ) = self
            .topology
            .as_ref()
            .map_or((0, 0, 0, 0, 0, 0), |topology| {
                (
                    topology.graph_vertices.len(),
                    topology.edge_chains.len(),
                    topology.referenced_graph_vertices(),
                    topology.unresolved_vertex_lookups,
                    topology.unresolved_equalities,
                    topology.unknown_orderings,
                )
            });
        let (region_boundaries, region_boundary_nodes) =
            self.region_plan.as_ref().map_or((0, 0), |region_plan| {
                (
                    region_plan.regions.len(),
                    region_plan
                        .regions
                        .iter()
                        .map(|region| region.boundary.len())
                        .sum(),
                )
            });
        let volume_adjacency_face_sides =
            self.volume_adjacencies.as_ref().map_or(0, |adjacencies| {
                adjacencies
                    .iter()
                    .map(|adjacency| adjacency.oriented_face_sides.len())
                    .sum()
            });
        let volume_adjacency_separating_faces =
            self.volume_adjacencies.as_ref().map_or(0, |adjacencies| {
                adjacencies
                    .iter()
                    .map(|adjacency| adjacency.separating_face_cells.len())
                    .sum()
            });
        let (
            lower_dimensional_point_contacts,
            lower_dimensional_edge_contacts,
            lower_dimensional_edge_endpoints,
        ) = lower_dimensional_artifact_counts(&self.lower_dimensional_artifacts);
        let (arrangement_face_cell_boundary_nodes, arrangement_face_cell_boundary_points) =
            arrangement_face_cell_boundary_counts(&self.face_cells);
        let (
            arrangement_regions,
            arrangement_region_face_cells,
            arrangement_region_adjacencies,
            arrangement_region_edge_incidences,
            arrangement_region_oriented_sides,
            arrangement_region_boundary_edges,
            arrangement_region_non_manifold_edges,
        ) = arrangement_region_topology_counts(self.shells_or_regions.as_deref());
        ExactTopologyAssemblyReport {
            status,
            freshness,
            blockers: self.blockers.clone(),
            graph_face_pairs: self.graph.face_pairs.len(),
            graph_events: self.graph.event_count(),
            split_graph_vertices,
            split_edge_chains,
            split_graph_vertex_references,
            unresolved_vertex_lookups,
            unresolved_equalities,
            unknown_orderings,
            region_boundaries,
            region_boundary_nodes,
            arrangement_vertices: self.vertices.len(),
            arrangement_edges: self.edges.len(),
            arrangement_face_cells: self.face_cells.len(),
            arrangement_face_cell_boundary_nodes,
            arrangement_face_cell_boundary_points,
            arrangement_regions,
            arrangement_region_face_cells,
            arrangement_region_adjacencies,
            arrangement_region_edge_incidences,
            arrangement_region_oriented_sides,
            arrangement_region_boundary_edges,
            arrangement_region_non_manifold_edges,
            carrier_plane_overlays: self.carrier_plane_overlays.len(),
            face_plane_arrangements: self.face_plane_arrangements.len(),
            lower_dimensional_artifacts: self.lower_dimensional_artifacts.len(),
            lower_dimensional_point_contacts,
            lower_dimensional_edge_contacts,
            lower_dimensional_edge_endpoints,
            volume_regions: self.volume_regions.as_ref().map_or(0, Vec::len),
            volume_adjacencies: self.volume_adjacencies.as_ref().map_or(0, Vec::len),
            volume_adjacency_face_sides,
            volume_adjacency_separating_faces,
        }
    }

    /// Report retained region ownership under an explicit regularization
    /// policy.
    pub(crate) fn region_ownership_report_with_policy(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<ExactRegionOwnershipReport, ExactArrangementBlocker> {
        let labeling_policy = ExactRegularizationPolicy {
            unresolved: ExactUnresolvedPolicy::RetainArtifacts,
            ..policy
        };
        let labeled = self.label_regions(labeling_policy)?;
        let mut report = labeled.region_ownership_report(left, right, labeling_policy);
        report.freshness = match self.freshness_against_sources_with_policy(left, right, policy) {
            ExactArrangementFreshness::Current => ExactLabeledCellComplexFreshness::Current,
            ExactArrangementFreshness::SourceReplayBlocked => {
                ExactLabeledCellComplexFreshness::SourceReplayBlocked
            }
            ExactArrangementFreshness::StaleArrangement => {
                ExactLabeledCellComplexFreshness::StaleLabeledCells
            }
        };
        report.status = region_ownership_status(
            report.freshness,
            &report.blockers,
            report.face_cells,
            report.opposite_unknown_faces,
            report.volume_regions,
            report.volume_adjacencies,
            report.volume_selection_resolved,
        );
        Ok(report)
    }

    /// Convert retained arrangement cells into a labeled cell complex.
    pub(crate) fn label_regions(
        &self,
        policy: ExactRegularizationPolicy,
    ) -> Result<ExactLabeledCellComplex, ExactArrangementBlocker> {
        ExactCellComplex::from_arrangement(self.clone(), policy).label_regions(policy)
    }
}

fn extend_split_plan_blockers(
    blockers: &mut Vec<ExactArrangementBlocker>,
    report: &SplitPlanValidationReport,
) {
    for blocker in &report.blockers {
        blockers.push(ExactArrangementBlocker::InvalidSplitPlan(
            blocker.kind.into(),
        ));
        match blocker.kind {
            super::graph::SplitPlanBlockerKind::UnknownOrdering => {
                blockers.push(ExactArrangementBlocker::UndecidableOrdering)
            }
            super::graph::SplitPlanBlockerKind::UnresolvedEquality
            | super::graph::SplitPlanBlockerKind::UnresolvedVertexLookup
            | super::graph::SplitPlanBlockerKind::UnknownBoundaryIncidence => {
                blockers.push(ExactArrangementBlocker::UnresolvedIntersection)
            }
            _ => {}
        }
    }
}

fn arrangement_vertices(
    left: &ExactMesh,
    right: &ExactMesh,
    topology: Option<&ExactSplitTopologyPlan>,
    carrier_plane_overlays: &[ArrangementCarrierPlaneOverlay],
    face_plane_arrangements: &[ArrangementFacePlaneArrangement],
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Vec<ArrangementVertex> {
    let mut vertices = Vec::new();
    let mut merge_index = ArrangementVertexMergeIndex::default();
    for (index, point) in left.vertices().iter().enumerate() {
        push_arrangement_vertex(
            &mut vertices,
            &mut merge_index,
            point.clone(),
            ArrangementVertexProvenance::SourceVertex {
                side: MeshSide::Left,
                vertex: index,
            },
            blockers,
        );
    }
    for (index, point) in right.vertices().iter().enumerate() {
        push_arrangement_vertex(
            &mut vertices,
            &mut merge_index,
            point.clone(),
            ArrangementVertexProvenance::SourceVertex {
                side: MeshSide::Right,
                vertex: index,
            },
            blockers,
        );
    }
    if let Some(topology) = topology {
        for (index, vertex) in topology.graph_vertices.iter().enumerate() {
            push_arrangement_vertex(
                &mut vertices,
                &mut merge_index,
                vertex.point.clone(),
                ArrangementVertexProvenance::GraphIntersection {
                    graph_vertex: index,
                },
                blockers,
            );
        }
    }
    for (overlay_index, overlay) in carrier_plane_overlays.iter().enumerate() {
        for (vertex_index, vertex) in overlay.overlay.arrangement.vertices.iter().enumerate() {
            if let Some(point) = lift_carrier_plane_point(
                left,
                overlay.left_face,
                overlay.projection,
                &vertex.point,
                blockers,
            ) {
                push_arrangement_vertex(
                    &mut vertices,
                    &mut merge_index,
                    point,
                    ArrangementVertexProvenance::CarrierPlaneVertex {
                        overlay: overlay_index,
                        vertex: vertex_index,
                    },
                    blockers,
                );
            }
        }
    }
    for (arrangement_index, arrangement) in face_plane_arrangements.iter().enumerate() {
        let mesh = arrangement.side.mesh(left, right);
        for (vertex_index, vertex) in arrangement.arrangement.vertices.iter().enumerate() {
            if let Some(point) = lift_carrier_plane_point(
                mesh,
                arrangement.face,
                arrangement.projection,
                &vertex.point,
                blockers,
            ) {
                push_arrangement_vertex(
                    &mut vertices,
                    &mut merge_index,
                    point,
                    ArrangementVertexProvenance::FacePlaneVertex {
                        arrangement: arrangement_index,
                        vertex: vertex_index,
                    },
                    blockers,
                );
            }
        }
    }
    vertices
}

fn push_arrangement_vertex(
    vertices: &mut Vec<ArrangementVertex>,
    index: &mut ArrangementVertexMergeIndex,
    point: Point3,
    provenance: ArrangementVertexProvenance,
    blockers: &mut Vec<ExactArrangementBlocker>,
) {
    let point_key = exact_point3_key(&point);
    if let Some(existing) = index.find_matching(&point, point_key.as_ref(), vertices, blockers) {
        if !vertices[existing].provenance.contains(&provenance) {
            vertices[existing].provenance.push(provenance);
        }
        return;
    }
    let vertex_index = vertices.len();
    index.insert(vertex_index, point_key);
    vertices.push(ArrangementVertex {
        point,
        provenance: vec![provenance],
    });
}

#[derive(Default)]
struct ArrangementVertexMergeIndex {
    point_key_buckets: BTreeMap<ExactPoint3Key, Vec<usize>>,
    unkeyed_vertices: Vec<usize>,
}

impl ArrangementVertexMergeIndex {
    fn insert(&mut self, vertex_index: usize, point_key: Option<ExactPoint3Key>) {
        if let Some(key) = point_key {
            self.point_key_buckets
                .entry(key)
                .or_default()
                .push(vertex_index);
        } else {
            self.unkeyed_vertices.push(vertex_index);
        }
    }

    fn find_matching(
        &self,
        point: &Point3,
        point_key: Option<&ExactPoint3Key>,
        vertices: &[ArrangementVertex],
        blockers: &mut Vec<ExactArrangementBlocker>,
    ) -> Option<usize> {
        if let Some(key) = point_key {
            if let Some(bucket) = self.point_key_buckets.get(key)
                && let Some(index) =
                    find_matching_arrangement_vertex(point, vertices, bucket, blockers)
            {
                return Some(index);
            }
            return find_matching_arrangement_vertex(
                point,
                vertices,
                &self.unkeyed_vertices,
                blockers,
            );
        }

        for bucket in self.point_key_buckets.values() {
            if let Some(index) = find_matching_arrangement_vertex(point, vertices, bucket, blockers)
            {
                return Some(index);
            }
        }
        find_matching_arrangement_vertex(point, vertices, &self.unkeyed_vertices, blockers)
    }
}

fn find_matching_arrangement_vertex(
    point: &Point3,
    vertices: &[ArrangementVertex],
    candidates: &[usize],
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Option<usize> {
    for &index in candidates {
        match point3_equal(&vertices[index].point, point).value() {
            Some(true) => return Some(index),
            Some(false) => {}
            None => blockers.push(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    None
}

#[derive(Default)]
struct ArrangementPointUniquenessIndex {
    point_key_buckets: BTreeMap<ExactPoint3Key, Vec<usize>>,
    unkeyed_points: Vec<usize>,
}

impl ArrangementPointUniquenessIndex {
    fn push_unique(&mut self, points: &mut Vec<Point3>, point: Point3) {
        let point_key = exact_point3_key(&point);
        if self
            .find_matching(&point, point_key.as_ref(), points)
            .is_some()
        {
            return;
        }
        let index = points.len();
        if let Some(key) = point_key {
            self.point_key_buckets.entry(key).or_default().push(index);
        } else {
            self.unkeyed_points.push(index);
        }
        points.push(point);
    }

    fn find_matching(
        &self,
        point: &Point3,
        point_key: Option<&ExactPoint3Key>,
        points: &[Point3],
    ) -> Option<usize> {
        if let Some(key) = point_key {
            if let Some(bucket) = self.point_key_buckets.get(key)
                && let Some(index) = find_matching_arrangement_point(point, points, bucket)
            {
                return Some(index);
            }
            return find_matching_arrangement_point(point, points, &self.unkeyed_points);
        }

        for bucket in self.point_key_buckets.values() {
            if let Some(index) = find_matching_arrangement_point(point, points, bucket) {
                return Some(index);
            }
        }
        find_matching_arrangement_point(point, points, &self.unkeyed_points)
    }
}

fn find_matching_arrangement_point(
    point: &Point3,
    points: &[Point3],
    candidates: &[usize],
) -> Option<usize> {
    candidates
        .iter()
        .copied()
        .find(|&index| point3_equal(&points[index], point).value() == Some(true))
}

fn arrangement_edges(
    topology: Option<&ExactSplitTopologyPlan>,
    vertices: &[ArrangementVertex],
    carrier_plane_overlays: &[ArrangementCarrierPlaneOverlay],
    face_plane_arrangements: &[ArrangementFacePlaneArrangement],
) -> Vec<ArrangementEdge> {
    let mut edges = Vec::new();
    let mut edge_lookup = BTreeMap::<[usize; 2], usize>::new();
    let vertex_index = ArrangementVertexProvenanceIndex::new(vertices);
    if let Some(topology) = topology {
        for chain in &topology.edge_chains {
            for pair in chain.nodes.windows(2) {
                let Some(left) = vertex_index.get_provenance(&match pair[0] {
                    SplitEdgeNode::OriginalVertex {
                        side,
                        vertex: index,
                    } => ArrangementVertexProvenance::SourceVertex {
                        side,
                        vertex: index,
                    },
                    SplitEdgeNode::GraphVertex { graph_vertex } => {
                        ArrangementVertexProvenance::GraphIntersection { graph_vertex }
                    }
                }) else {
                    continue;
                };
                let Some(right) = vertex_index.get_provenance(&match pair[1] {
                    SplitEdgeNode::OriginalVertex {
                        side,
                        vertex: index,
                    } => ArrangementVertexProvenance::SourceVertex {
                        side,
                        vertex: index,
                    },
                    SplitEdgeNode::GraphVertex { graph_vertex } => {
                        ArrangementVertexProvenance::GraphIntersection { graph_vertex }
                    }
                }) else {
                    continue;
                };
                push_arrangement_edge(
                    &mut edges,
                    &mut edge_lookup,
                    left,
                    right,
                    ArrangementEdgeProvenance::Source {
                        side: chain.side,
                        edge: chain.edge,
                    },
                );
            }
        }
    }
    for (overlay_index, overlay) in carrier_plane_overlays.iter().enumerate() {
        for (edge_index, edge) in overlay.overlay.arrangement.edges.iter().enumerate() {
            let Some(left) =
                vertex_index.get_provenance(&ArrangementVertexProvenance::CarrierPlaneVertex {
                    overlay: overlay_index,
                    vertex: edge.vertices[0],
                })
            else {
                continue;
            };
            let Some(right) =
                vertex_index.get_provenance(&ArrangementVertexProvenance::CarrierPlaneVertex {
                    overlay: overlay_index,
                    vertex: edge.vertices[1],
                })
            else {
                continue;
            };
            push_arrangement_edge(
                &mut edges,
                &mut edge_lookup,
                left,
                right,
                ArrangementEdgeProvenance::CarrierPlane {
                    overlay: overlay_index,
                    edge: edge_index,
                },
            );
        }
    }
    for (arrangement_index, arrangement) in face_plane_arrangements.iter().enumerate() {
        for (edge_index, edge) in arrangement.arrangement.edges.iter().enumerate() {
            let Some(left) =
                vertex_index.get_provenance(&ArrangementVertexProvenance::FacePlaneVertex {
                    arrangement: arrangement_index,
                    vertex: edge.vertices[0],
                })
            else {
                continue;
            };
            let Some(right) =
                vertex_index.get_provenance(&ArrangementVertexProvenance::FacePlaneVertex {
                    arrangement: arrangement_index,
                    vertex: edge.vertices[1],
                })
            else {
                continue;
            };
            push_arrangement_edge(
                &mut edges,
                &mut edge_lookup,
                left,
                right,
                ArrangementEdgeProvenance::FacePlane {
                    arrangement: arrangement_index,
                    edge: edge_index,
                },
            );
        }
    }
    edges
}

fn push_arrangement_edge(
    edges: &mut Vec<ArrangementEdge>,
    edge_index: &mut BTreeMap<[usize; 2], usize>,
    left: usize,
    right: usize,
    provenance: ArrangementEdgeProvenance,
) {
    if left == right {
        return;
    }
    let key = if left < right {
        [left, right]
    } else {
        [right, left]
    };
    if let Some(index) = edge_index.get(&key).copied() {
        if !edges[index].provenance.contains(&provenance) {
            edges[index].provenance.push(provenance);
        }
    } else {
        let index = edges.len();
        edge_index.insert(key, index);
        edges.push(ArrangementEdge {
            vertices: key,
            provenance: vec![provenance],
        });
    }
}

type ArrangementVertexProvenanceKey = (usize, usize, usize);

struct ArrangementVertexProvenanceIndex {
    by_provenance: BTreeMap<ArrangementVertexProvenanceKey, usize>,
}

impl ArrangementVertexProvenanceIndex {
    fn new(vertices: &[ArrangementVertex]) -> Self {
        let mut by_provenance = BTreeMap::new();
        for (index, vertex) in vertices.iter().enumerate() {
            for provenance in &vertex.provenance {
                by_provenance
                    .entry(arrangement_vertex_provenance_key(provenance))
                    .or_insert(index);
            }
        }
        Self { by_provenance }
    }

    fn get_provenance(&self, provenance: &ArrangementVertexProvenance) -> Option<usize> {
        self.by_provenance
            .get(&arrangement_vertex_provenance_key(provenance))
            .copied()
    }
}

fn arrangement_vertex_provenance_key(
    provenance: &ArrangementVertexProvenance,
) -> ArrangementVertexProvenanceKey {
    match provenance {
        ArrangementVertexProvenance::SourceVertex { side, vertex } => (0, side_key(*side), *vertex),
        ArrangementVertexProvenance::GraphIntersection { graph_vertex } => (1, 0, *graph_vertex),
        ArrangementVertexProvenance::CarrierPlaneVertex { overlay, vertex } => {
            (2, *overlay, *vertex)
        }
        ArrangementVertexProvenance::FacePlaneVertex {
            arrangement,
            vertex,
        } => (3, *arrangement, *vertex),
    }
}

fn arrangement_face_cells(
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactRegularizationPolicy,
    region_plan: Option<&ExactFaceRegionPlan>,
    carrier_plane_overlays: &[ArrangementCarrierPlaneOverlay],
    face_plane_arrangements: &[ArrangementFacePlaneArrangement],
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Vec<ArrangementFaceCell> {
    let mut cells = Vec::new();
    let skipped_carriers = overlay_carriers(carrier_plane_overlays);
    let mut skipped_face_arrangements = BTreeSet::new();
    for arrangement in face_plane_arrangements {
        push_unique_carrier(
            &mut skipped_face_arrangements,
            arrangement.side,
            arrangement.face,
        );
    }

    if let Some(region_plan) = region_plan
        && !region_plan.regions.is_empty()
    {
        cells.extend(region_plan.regions.iter().filter_map(|region| {
            if skipped_carriers.contains(&carrier_key(region.side, region.face))
                || skipped_face_arrangements.contains(&carrier_key(region.side, region.face))
            {
                None
            } else {
                Some(face_cell_from_region(region, left, right, policy, blockers))
            }
        }));
        append_face_plane_arrangement_face_cells(
            &mut cells,
            face_plane_arrangements,
            left,
            right,
            policy,
            blockers,
        );
        append_carrier_plane_overlay_face_cells(
            &mut cells,
            carrier_plane_overlays,
            left,
            right,
            policy,
            blockers,
        );
        return cells;
    }

    for (face, triangle) in left.triangles().iter().enumerate() {
        if skipped_carriers.contains(&carrier_key(MeshSide::Left, face))
            || skipped_face_arrangements.contains(&carrier_key(MeshSide::Left, face))
        {
            continue;
        }
        cells.push(face_cell_from_original_triangle(
            MeshSide::Left,
            face,
            triangle.0,
            left,
            right,
            policy,
            blockers,
        ));
    }
    for (face, triangle) in right.triangles().iter().enumerate() {
        if skipped_carriers.contains(&carrier_key(MeshSide::Right, face))
            || skipped_face_arrangements.contains(&carrier_key(MeshSide::Right, face))
        {
            continue;
        }
        cells.push(face_cell_from_original_triangle(
            MeshSide::Right,
            face,
            triangle.0,
            left,
            right,
            policy,
            blockers,
        ));
    }
    append_face_plane_arrangement_face_cells(
        &mut cells,
        face_plane_arrangements,
        left,
        right,
        policy,
        blockers,
    );
    append_carrier_plane_overlay_face_cells(
        &mut cells,
        carrier_plane_overlays,
        left,
        right,
        policy,
        blockers,
    );
    cells
}

type ArrangementCarrierKey = (usize, usize);

fn carrier_key(side: MeshSide, face: usize) -> ArrangementCarrierKey {
    (side_key(side), face)
}

fn face_plane_arrangements(
    topology: Option<&ExactSplitTopologyPlan>,
    left: &ExactMesh,
    right: &ExactMesh,
    carrier_plane_overlays: &[ArrangementCarrierPlaneOverlay],
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Vec<ArrangementFacePlaneArrangement> {
    let Some(topology) = topology else {
        return Vec::new();
    };
    let skipped_carriers = overlay_carriers(carrier_plane_overlays);
    let mut pair_vertices = BTreeMap::<(usize, usize, usize, usize), BTreeSet<usize>>::new();
    for (graph_vertex, vertex) in topology.graph_vertices.iter().enumerate() {
        for source_use in &vertex.uses {
            let edge_carrier = match source_use.side {
                MeshSide::Left => source_use.face_pair[0],
                MeshSide::Right => source_use.face_pair[1],
            };
            pair_vertices
                .entry((
                    side_key(source_use.side),
                    edge_carrier,
                    source_use.face_pair[0],
                    source_use.face_pair[1],
                ))
                .or_default()
                .insert(graph_vertex);

            let plane_side = match source_use.side {
                MeshSide::Left => MeshSide::Right,
                MeshSide::Right => MeshSide::Left,
            };
            pair_vertices
                .entry((
                    side_key(plane_side),
                    source_use.plane_face,
                    source_use.face_pair[0],
                    source_use.face_pair[1],
                ))
                .or_default()
                .insert(graph_vertex);
        }
    }

    let mut per_face_groups = BTreeMap::<(usize, usize), Vec<Vec<usize>>>::new();
    for ((side, face, _, _), vertices) in pair_vertices {
        if vertices.len() < 2 {
            continue;
        }
        let side = side_from_key(side);
        if skipped_carriers.contains(&carrier_key(side, face)) {
            continue;
        }
        per_face_groups
            .entry((side_key(side), face))
            .or_default()
            .push(vertices.into_iter().collect());
    }

    let mut arrangements = Vec::new();
    for ((side, face), groups) in per_face_groups {
        let side = side_from_key(side);
        if let Some(arrangement) =
            face_plane_arrangement(side, face, groups, topology, left, right, blockers)
        {
            arrangements.push(arrangement);
        }
    }
    arrangements
}

const fn side_from_key(side: usize) -> MeshSide {
    match side {
        0 => MeshSide::Left,
        _ => MeshSide::Right,
    }
}

fn face_plane_arrangement(
    side: MeshSide,
    face: usize,
    groups: Vec<Vec<usize>>,
    topology: &ExactSplitTopologyPlan,
    left: &ExactMesh,
    right: &ExactMesh,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Option<ArrangementFacePlaneArrangement> {
    let mesh = side.mesh(left, right);
    let triangle = mesh.triangles().get(face)?.0;
    let projection = choose_triangle_projection(mesh, triangle, blockers)?;
    let mut segments = Vec::new();
    let mut next_source = 0usize;

    for edge in [
        [triangle[0], triangle[1]],
        [triangle[1], triangle[2]],
        [triangle[2], triangle[0]],
    ] {
        segments.push(ExactArrangement2dInputSegment::new(
            [
                project_point3(&mesh.vertices()[edge[0]], projection),
                project_point3(&mesh.vertices()[edge[1]], projection),
            ],
            ExactArrangement2dSegmentSource::Anonymous(next_source),
        ));
        next_source += 1;
    }

    let mut graph_vertices_on_face = BTreeSet::new();
    for mut group in groups {
        group.sort_by(|left_index, right_index| {
            let left_point =
                project_point3(&topology.graph_vertices[*left_index].point, projection);
            let right_point =
                project_point3(&topology.graph_vertices[*right_index].point, projection);
            compare_point2_lexicographic(&left_point, &right_point)
                .value()
                .unwrap_or_else(|| {
                    blockers.push(ExactArrangementBlocker::UndecidableOrdering);
                    Ordering::Equal
                })
        });
        group.dedup();
        for vertex in &group {
            graph_vertices_on_face.insert(*vertex);
        }
        for pair in group.windows(2) {
            let start = project_point3(&topology.graph_vertices[pair[0]].point, projection);
            let end = project_point3(&topology.graph_vertices[pair[1]].point, projection);
            match point2_equal(&start, &end).value() {
                Some(true) => continue,
                Some(false) => {}
                None => {
                    blockers.push(ExactArrangementBlocker::UndecidableOrdering);
                    continue;
                }
            }
            segments.push(ExactArrangement2dInputSegment::new(
                [start, end],
                ExactArrangement2dSegmentSource::Anonymous(next_source),
            ));
            next_source += 1;
        }
    }

    let arrangement = build_exact_arrangement2d(&segments);
    extend_arrangement2d_blockers(&arrangement.blockers, blockers);
    let graph_vertices_on_face = graph_vertices_on_face.into_iter().collect::<Vec<_>>();
    let vertex_provenance = arrangement
        .vertices
        .iter()
        .map(|vertex| {
            face_plane_vertex_provenance(
                side,
                face,
                &vertex.point,
                projection,
                &graph_vertices_on_face,
                topology,
                left,
                right,
                blockers,
            )
        })
        .collect();

    Some(ArrangementFacePlaneArrangement {
        side,
        face,
        projection,
        arrangement,
        vertex_provenance,
    })
}

fn choose_triangle_projection(
    mesh: &ExactMesh,
    triangle: [usize; 3],
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Option<CoplanarProjection> {
    let points = [
        mesh.vertices()[triangle[0]].clone(),
        mesh.vertices()[triangle[1]].clone(),
        mesh.vertices()[triangle[2]].clone(),
    ];
    for projection in [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        match compare_reals(
            &projected_triangle_area2(&points, projection),
            &Real::from(0),
        )
        .value()
        {
            Some(Ordering::Less | Ordering::Greater) => return Some(projection),
            Some(Ordering::Equal) => {}
            None => {
                blockers.push(ExactArrangementBlocker::UndecidableOrdering);
                return None;
            }
        }
    }
    blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
    None
}

fn face_plane_vertex_provenance(
    side: MeshSide,
    face: usize,
    point: &Point2,
    projection: CoplanarProjection,
    graph_vertices_on_face: &[usize],
    topology: &ExactSplitTopologyPlan,
    left: &ExactMesh,
    right: &ExactMesh,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Option<ArrangementFaceCellNode> {
    let mesh = side.mesh(left, right);
    let triangle = mesh.triangles()[face].0;
    for vertex in triangle {
        match point2_equal(&project_point3(&mesh.vertices()[vertex], projection), point).value() {
            Some(true) => return Some(ArrangementFaceCellNode::Source { side, vertex }),
            Some(false) => {}
            None => blockers.push(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    for graph_vertex in graph_vertices_on_face {
        match point2_equal(
            &project_point3(&topology.graph_vertices[*graph_vertex].point, projection),
            point,
        )
        .value()
        {
            Some(true) => {
                return Some(ArrangementFaceCellNode::Graph {
                    graph_vertex: *graph_vertex,
                });
            }
            Some(false) => {}
            None => blockers.push(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    None
}

fn overlay_carriers(
    carrier_plane_overlays: &[ArrangementCarrierPlaneOverlay],
) -> BTreeSet<ArrangementCarrierKey> {
    let mut carriers = BTreeSet::new();
    for overlay in carrier_plane_overlays {
        push_unique_carrier(&mut carriers, MeshSide::Left, overlay.left_face);
        push_unique_carrier(&mut carriers, MeshSide::Right, overlay.right_face);
    }
    carriers
}

fn push_unique_carrier(
    carriers: &mut BTreeSet<ArrangementCarrierKey>,
    side: MeshSide,
    face: usize,
) {
    carriers.insert(carrier_key(side, face));
}

fn lower_dimensional_artifacts(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactRegularizationPolicy,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Vec<ArrangementLowerDimensionalArtifact> {
    if policy.lower_dimensional == ExactLowerDimensionalPolicy::Drop {
        return Vec::new();
    }

    let mut artifacts = Vec::new();
    let mut artifact_index = LowerDimensionalArtifactBuildIndex::default();
    for pair in &graph.face_pairs {
        if pair.relation != super::graph::intersection::MeshFacePairRelation::Candidate {
            continue;
        }
        if pair.events.iter().any(|event| {
            matches!(
                event,
                super::graph::IntersectionEvent::SegmentPlane {
                    relation: hyperlimit::SegmentPlaneRelation::ProperCrossing,
                    ..
                }
            )
        }) {
            continue;
        }
        for event in &pair.events {
            if let Some(artifact) = non_coplanar_edge_contact_artifact(
                pair.left_face,
                pair.right_face,
                event,
                left,
                right,
            ) {
                push_lower_dimensional_artifact(&mut artifacts, &mut artifact_index, artifact);
                continue;
            }
            if let Some(artifact) = non_coplanar_point_contact_artifact(
                pair.left_face,
                pair.right_face,
                event,
                left,
                right,
            ) {
                push_lower_dimensional_artifact(&mut artifacts, &mut artifact_index, artifact);
            }
        }
    }
    let touching_pairs = graph
        .coplanar_overlap_graph_iter()
        .filter(|overlap| {
            overlap.relation == super::graph::intersection::MeshFacePairRelation::CoplanarTouching
        })
        .map(|overlap| ((overlap.left_face, overlap.right_face), overlap))
        .collect::<BTreeMap<_, _>>();

    if !touching_pairs.is_empty() {
        let split_plan = match graph.coplanar_overlap_split_plan(left, right) {
            Ok(split_plan) => split_plan,
            Err(_) => {
                blockers.push(ExactArrangementBlocker::UnresolvedIntersection);
                return artifacts;
            }
        };

        for split_graph in split_plan.graphs {
            if !touching_pairs.contains_key(&(split_graph.left_face, split_graph.right_face)) {
                continue;
            }
            for edge_split in &split_graph.edge_splits {
                for split_point in &edge_split.points {
                    push_lower_dimensional_artifact(
                        &mut artifacts,
                        &mut artifact_index,
                        ArrangementLowerDimensionalArtifact::PointContact {
                            left_face: split_graph.left_face,
                            right_face: split_graph.right_face,
                            point: split_point.point.clone(),
                        },
                    );
                }
                if let Some(interval) = &edge_split.interval {
                    push_lower_dimensional_artifact(
                        &mut artifacts,
                        &mut artifact_index,
                        ArrangementLowerDimensionalArtifact::EdgeContact {
                            left_face: split_graph.left_face,
                            right_face: split_graph.right_face,
                            endpoints: [
                                interval.endpoints[0].point.clone(),
                                interval.endpoints[1].point.clone(),
                            ],
                        },
                    );
                }
            }
            for vertex_overlap in &split_graph.vertex_overlaps {
                let mesh = vertex_overlap.vertex_side.mesh(left, right);
                if let Some(point) = mesh.vertices().get(vertex_overlap.vertex) {
                    push_lower_dimensional_artifact(
                        &mut artifacts,
                        &mut artifact_index,
                        ArrangementLowerDimensionalArtifact::PointContact {
                            left_face: split_graph.left_face,
                            right_face: split_graph.right_face,
                            point: point.clone(),
                        },
                    );
                }
            }
        }
    }

    artifacts
}

fn non_coplanar_point_contact_artifact(
    left_face: usize,
    right_face: usize,
    event: &super::graph::IntersectionEvent,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ArrangementLowerDimensionalArtifact> {
    let super::graph::IntersectionEvent::SegmentPlane {
        plane_side,
        plane_face,
        relation: hyperlimit::SegmentPlaneRelation::EndpointOnPlane,
        point: Some(point),
        ..
    } = event
    else {
        return None;
    };
    let plane_mesh = plane_side.mesh(left, right);
    let triangle = plane_mesh.triangles().get(*plane_face)?.0;
    let projection = choose_triangle_projection(
        plane_mesh,
        triangle,
        &mut Vec::<ExactArrangementBlocker>::new(),
    )?;
    let a = project_point3(plane_mesh.vertices().get(triangle[0])?, projection);
    let b = project_point3(plane_mesh.vertices().get(triangle[1])?, projection);
    let c = project_point3(plane_mesh.vertices().get(triangle[2])?, projection);
    let projected = project_point3(point, projection);
    let location = classify_point_triangle(&a, &b, &c, &projected).value()?;
    matches!(
        location,
        TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
    )
    .then_some(ArrangementLowerDimensionalArtifact::PointContact {
        left_face,
        right_face,
        point: point.clone(),
    })
}

fn non_coplanar_edge_contact_artifact(
    left_face: usize,
    right_face: usize,
    event: &super::graph::IntersectionEvent,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ArrangementLowerDimensionalArtifact> {
    let super::graph::IntersectionEvent::SegmentPlane {
        segment_side,
        edge,
        plane_side,
        plane_face,
        relation: hyperlimit::SegmentPlaneRelation::Coplanar,
        endpoint_sides:
            [
                Some(hyperlimit::PlaneSide::On),
                Some(hyperlimit::PlaneSide::On),
            ],
        ..
    } = event
    else {
        return None;
    };
    let segment_mesh = segment_side.mesh(left, right);
    let plane_mesh = plane_side.mesh(left, right);
    let start = segment_mesh.vertices().get(edge[0])?;
    let end = segment_mesh.vertices().get(edge[1])?;
    let triangle = plane_mesh.triangles().get(*plane_face)?.0;
    let projection = choose_triangle_projection(
        plane_mesh,
        triangle,
        &mut Vec::<ExactArrangementBlocker>::new(),
    )?;
    let a = project_point3(plane_mesh.vertices().get(triangle[0])?, projection);
    let b = project_point3(plane_mesh.vertices().get(triangle[1])?, projection);
    let c = project_point3(plane_mesh.vertices().get(triangle[2])?, projection);
    let endpoints = coplanar_segment_triangle_interval(start, end, [&a, &b, &c], projection)?;
    Some(ArrangementLowerDimensionalArtifact::EdgeContact {
        left_face,
        right_face,
        endpoints,
    })
}

fn coplanar_segment_triangle_interval(
    start: &Point3,
    end: &Point3,
    triangle: [&Point2; 3],
    projection: CoplanarProjection,
) -> Option<[Point3; 2]> {
    let segment_start = project_point3(start, projection);
    let segment_end = project_point3(end, projection);
    let mut points = Vec::<Point3>::new();
    let mut point_index = ArrangementPointUniquenessIndex::default();
    for (point, projected) in [(start, &segment_start), (end, &segment_end)] {
        let location =
            classify_point_triangle(triangle[0], triangle[1], triangle[2], projected).value()?;
        if matches!(
            location,
            TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
        ) {
            point_index.push_unique(&mut points, point.clone());
        }
    }

    for index in 0..3 {
        let a = triangle[index];
        let b = triangle[(index + 1) % 3];
        match classify_segment_intersection(&segment_start, &segment_end, a, b).value()? {
            SegmentIntersection::Disjoint => {}
            SegmentIntersection::Proper => {
                let point = proper_segment_intersection_point(&segment_start, &segment_end, a, b)
                    .value()??;
                point_index.push_unique(
                    &mut points,
                    lift_projected_point_to_segment(start, end, &point, projection)?,
                );
            }
            SegmentIntersection::EndpointTouch
            | SegmentIntersection::CollinearOverlap
            | SegmentIntersection::Identical => {
                for point in [a, b] {
                    if point_on_segment(&segment_start, &segment_end, point).value()? {
                        point_index.push_unique(
                            &mut points,
                            lift_projected_point_to_segment(start, end, point, projection)?,
                        );
                    }
                }
                for (point, projected) in [(start, &segment_start), (end, &segment_end)] {
                    if point_on_segment(a, b, projected).value()? {
                        point_index.push_unique(&mut points, point.clone());
                    }
                }
            }
        }
    }

    if points.len() < 2 {
        return None;
    }
    let axis = segment_order_axis(start, end)?;
    for index in 1..points.len() {
        let mut current = index;
        while current > 0 {
            let ordering =
                compare_point3_on_axis(&points[current - 1], &points[current], axis, start, end)?;
            if ordering != Ordering::Greater {
                break;
            }
            points.swap(current - 1, current);
            current -= 1;
        }
    }
    let first = points.first()?.clone();
    let last = points.last()?.clone();
    (!point3_equal(&first, &last).value()?).then_some([first, last])
}

fn lift_projected_point_to_segment(
    start: &Point3,
    end: &Point3,
    point: &Point2,
    projection: CoplanarProjection,
) -> Option<Point3> {
    let projected_start = project_point3(start, projection);
    let projected_end = project_point3(end, projection);
    let parameter = if compare_reals(&projected_start.x, &projected_end.x).value()?
        != Ordering::Equal
    {
        ((point.x.clone() - &projected_start.x) / &(projected_end.x - &projected_start.x)).ok()?
    } else if compare_reals(&projected_start.y, &projected_end.y).value()? != Ordering::Equal {
        ((point.y.clone() - &projected_start.y) / &(projected_end.y - &projected_start.y)).ok()?
    } else {
        return None;
    };
    Some(Point3::new(
        start.x.clone() + &((end.x.clone() - &start.x) * &parameter),
        start.y.clone() + &((end.y.clone() - &start.y) * &parameter),
        start.z.clone() + &((end.z.clone() - &start.z) * &parameter),
    ))
}

fn segment_order_axis(start: &Point3, end: &Point3) -> Option<usize> {
    if compare_reals(&start.x, &end.x).value()? != Ordering::Equal {
        Some(0)
    } else if compare_reals(&start.y, &end.y).value()? != Ordering::Equal {
        Some(1)
    } else if compare_reals(&start.z, &end.z).value()? != Ordering::Equal {
        Some(2)
    } else {
        None
    }
}

fn compare_point3_on_axis(
    left: &Point3,
    right: &Point3,
    axis: usize,
    start: &Point3,
    end: &Point3,
) -> Option<Ordering> {
    let (left_value, right_value, start_value, end_value) = match axis {
        0 => (&left.x, &right.x, &start.x, &end.x),
        1 => (&left.y, &right.y, &start.y, &end.y),
        2 => (&left.z, &right.z, &start.z, &end.z),
        _ => return None,
    };
    let order = compare_reals(left_value, right_value).value()?;
    if compare_reals(start_value, end_value).value()? == Ordering::Less {
        Some(order)
    } else {
        Some(order.reverse())
    }
}

fn push_lower_dimensional_artifact(
    artifacts: &mut Vec<ArrangementLowerDimensionalArtifact>,
    artifact_index: &mut LowerDimensionalArtifactBuildIndex,
    artifact: ArrangementLowerDimensionalArtifact,
) {
    artifact_index.push_unique(artifacts, artifact);
}

fn append_carrier_plane_overlay_face_cells(
    cells: &mut Vec<ArrangementFaceCell>,
    carrier_plane_overlays: &[ArrangementCarrierPlaneOverlay],
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactRegularizationPolicy,
    blockers: &mut Vec<ExactArrangementBlocker>,
) {
    for (overlay_index, overlay) in carrier_plane_overlays.iter().enumerate() {
        for overlay_face in &overlay.overlay.faces {
            if overlay_face.in_left
                && let Some(cell) = face_cell_from_carrier_plane_overlay(
                    overlay_index,
                    overlay,
                    overlay_face.face,
                    &overlay_face.witness,
                    MeshSide::Left,
                    left,
                    right,
                    policy,
                    blockers,
                )
            {
                cells.push(cell);
            }
            if overlay_face.in_right
                && let Some(cell) = face_cell_from_carrier_plane_overlay(
                    overlay_index,
                    overlay,
                    overlay_face.face,
                    &overlay_face.witness,
                    MeshSide::Right,
                    left,
                    right,
                    policy,
                    blockers,
                )
            {
                cells.push(cell);
            }
        }
    }
}

fn append_face_plane_arrangement_face_cells(
    cells: &mut Vec<ArrangementFaceCell>,
    face_plane_arrangements: &[ArrangementFacePlaneArrangement],
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactRegularizationPolicy,
    blockers: &mut Vec<ExactArrangementBlocker>,
) {
    for (arrangement_index, arrangement) in face_plane_arrangements.iter().enumerate() {
        for face in 0..arrangement.arrangement.faces.len() {
            if let Some(cell) = face_cell_from_face_plane_arrangement(
                arrangement_index,
                arrangement,
                face,
                left,
                right,
                policy,
                blockers,
            ) {
                cells.push(cell);
            }
        }
    }
}

fn face_cell_from_face_plane_arrangement(
    arrangement_index: usize,
    arrangement: &ArrangementFacePlaneArrangement,
    face: usize,
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactRegularizationPolicy,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Option<ArrangementFaceCell> {
    let mesh = arrangement.side.mesh(left, right);
    let triangle = mesh.triangles().get(arrangement.face)?.0;
    let arrangement_face = arrangement.arrangement.faces.get(face)?;
    let mut boundary = Vec::with_capacity(arrangement_face.vertices.len());
    let mut boundary_points = Vec::with_capacity(arrangement_face.vertices.len());
    for vertex in &arrangement_face.vertices {
        let point2 = &arrangement.arrangement.vertices.get(*vertex)?.point;
        let point3 = lift_carrier_plane_point(
            mesh,
            arrangement.face,
            arrangement.projection,
            point2,
            blockers,
        )?;
        boundary.push(arrangement.vertex_provenance[*vertex].clone().unwrap_or(
            ArrangementFaceCellNode::FacePlane {
                arrangement: arrangement_index,
                vertex: *vertex,
            },
        ));
        boundary_points.push(point3);
    }
    orient_overlay_boundary_to_carrier(
        mesh,
        arrangement.face,
        arrangement.projection,
        &mut boundary,
        &mut boundary_points,
        blockers,
    );

    let mut witness_blockers = Vec::new();
    let witness =
        exact_arrangement2d_face_witness(&arrangement.arrangement, face, &mut witness_blockers);
    extend_arrangement2d_blockers(&witness_blockers, blockers);
    let representative = lift_carrier_plane_point(
        mesh,
        arrangement.face,
        arrangement.projection,
        &witness?,
        blockers,
    )?;
    let opposite = Some(classify_opposite(
        arrangement.side,
        representative,
        left,
        right,
        policy,
        blockers,
    ));

    Some(ArrangementFaceCell {
        carrier: ArrangementFaceCarrier {
            side: arrangement.side,
            face: arrangement.face,
            triangle,
        },
        boundary,
        boundary_points,
        opposite,
    })
}

fn face_cell_from_carrier_plane_overlay(
    overlay_index: usize,
    overlay: &ArrangementCarrierPlaneOverlay,
    face: usize,
    witness: &Point2,
    side: MeshSide,
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactRegularizationPolicy,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Option<ArrangementFaceCell> {
    let carrier_face = match side {
        MeshSide::Left => overlay.left_face,
        MeshSide::Right => overlay.right_face,
    };
    let mesh = side.mesh(left, right);
    let triangle = mesh.triangles().get(carrier_face)?.0;
    let overlay_face = overlay.overlay.arrangement.faces.get(face)?;
    let mut boundary = Vec::with_capacity(overlay_face.vertices.len());
    let mut boundary_points = Vec::with_capacity(overlay_face.vertices.len());
    for vertex in &overlay_face.vertices {
        let point2 = &overlay.overlay.arrangement.vertices.get(*vertex)?.point;
        let point3 =
            lift_carrier_plane_point(mesh, carrier_face, overlay.projection, point2, blockers)?;
        boundary.push(ArrangementFaceCellNode::CarrierPlane {
            overlay: overlay_index,
            vertex: *vertex,
        });
        boundary_points.push(point3);
    }
    orient_overlay_boundary_to_carrier(
        mesh,
        carrier_face,
        overlay.projection,
        &mut boundary,
        &mut boundary_points,
        blockers,
    );
    let representative =
        lift_carrier_plane_point(mesh, carrier_face, overlay.projection, witness, blockers)?;
    let opposite = Some(classify_opposite(
        side,
        representative,
        left,
        right,
        policy,
        blockers,
    ));

    Some(ArrangementFaceCell {
        carrier: ArrangementFaceCarrier {
            side,
            face: carrier_face,
            triangle,
        },
        boundary,
        boundary_points,
        opposite,
    })
}

fn extend_arrangement2d_blockers(
    source: &[ExactArrangement2dBlocker],
    blockers: &mut Vec<ExactArrangementBlocker>,
) {
    for blocker in source {
        match blocker {
            ExactArrangement2dBlocker::UnresolvedPointEquality { .. }
            | ExactArrangement2dBlocker::UnresolvedSegmentRelation { .. }
            | ExactArrangement2dBlocker::UnresolvedProperIntersectionConstruction { .. }
            | ExactArrangement2dBlocker::UnresolvedPointOnSegment { .. } => {
                blockers.push(ExactArrangementBlocker::UnresolvedIntersection)
            }
            ExactArrangement2dBlocker::UnresolvedSegmentOrdering { .. }
            | ExactArrangement2dBlocker::UnresolvedAngleOrdering { .. }
            | ExactArrangement2dBlocker::UnresolvedFaceArea { .. }
            | ExactArrangement2dBlocker::UnresolvedRingNormalization { .. }
            | ExactArrangement2dBlocker::UnresolvedOutputLoopContainment { .. }
            | ExactArrangement2dBlocker::UnresolvedParentSelection { .. }
            | ExactArrangement2dBlocker::UnresolvedSelectedBoundaryOrdering { .. } => {
                blockers.push(ExactArrangementBlocker::UndecidableOrdering)
            }
            ExactArrangement2dBlocker::DegenerateSegment { .. }
            | ExactArrangement2dBlocker::IncompleteFaceWalk { .. }
            | ExactArrangement2dBlocker::InvalidRegionRing { .. }
            | ExactArrangement2dBlocker::UnresolvedFaceWitness { .. }
            | ExactArrangement2dBlocker::UnresolvedRingClassification { .. }
            | ExactArrangement2dBlocker::FaceWitnessOnBoundary { .. }
            | ExactArrangement2dBlocker::NonManifoldSelectedBoundary { .. }
            | ExactArrangement2dBlocker::DegenerateOutputLoop { .. }
            | ExactArrangement2dBlocker::OutputHoleWithoutOuter { .. }
            | ExactArrangement2dBlocker::OutputLoopBoundaryContainment { .. } => {
                blockers.push(ExactArrangementBlocker::NonManifoldCellComplex)
            }
        }
    }
}

fn orient_overlay_boundary_to_carrier(
    mesh: &ExactMesh,
    face: usize,
    projection: CoplanarProjection,
    boundary: &mut [ArrangementFaceCellNode],
    boundary_points: &mut [Point3],
    blockers: &mut Vec<ExactArrangementBlocker>,
) {
    let Some(triangle) = mesh.triangles().get(face).map(|triangle| triangle.0) else {
        blockers.push(ExactArrangementBlocker::UnresolvedIntersection);
        return;
    };
    let points = [
        mesh.vertices()[triangle[0]].clone(),
        mesh.vertices()[triangle[1]].clone(),
        mesh.vertices()[triangle[2]].clone(),
    ];
    match compare_reals(
        &projected_triangle_area2(&points, projection),
        &Real::from(0),
    )
    .value()
    {
        Some(Ordering::Less) => {
            boundary.reverse();
            boundary_points.reverse();
        }
        Some(Ordering::Greater) => {}
        Some(Ordering::Equal) => blockers.push(ExactArrangementBlocker::NonManifoldCellComplex),
        None => blockers.push(ExactArrangementBlocker::UndecidableOrdering),
    }
}

fn carrier_plane_overlays(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Vec<ArrangementCarrierPlaneOverlay> {
    graph
        .coplanar_overlap_graph_iter()
        .filter_map(|overlap| carrier_plane_overlay(&overlap, left, right, blockers))
        .collect()
}

fn carrier_plane_overlay(
    overlap: &CoplanarOverlapGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Option<ArrangementCarrierPlaneOverlay> {
    if overlap.validate().is_err() {
        blockers.push(ExactArrangementBlocker::UnresolvedIntersection);
        return None;
    }
    let left_ring = projected_face_ring(
        ExactArrangement2dRegion::Left,
        left,
        overlap.left_face,
        overlap.projection,
    )?;
    let right_ring = projected_face_ring(
        ExactArrangement2dRegion::Right,
        right,
        overlap.right_face,
        overlap.projection,
    )?;
    let overlay = build_exact_arrangement2d_overlay(
        &[left_ring, right_ring],
        ExactArrangement2dSetOperation::Union,
    );
    if !overlay.blockers.is_empty() {
        blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    Some(ArrangementCarrierPlaneOverlay {
        left_face: overlap.left_face,
        right_face: overlap.right_face,
        projection: overlap.projection,
        overlay,
    })
}

fn projected_face_ring(
    region: ExactArrangement2dRegion,
    mesh: &ExactMesh,
    face: usize,
    projection: CoplanarProjection,
) -> Option<ExactArrangement2dRegionRing> {
    let triangle = mesh.triangles().get(face)?.0;
    let vertices = triangle
        .iter()
        .map(|vertex| project_point3(&mesh.vertices()[*vertex], projection))
        .collect::<Vec<Point2>>();
    Some(ExactArrangement2dRegionRing::new(region, vertices))
}

fn arrangement_regions(
    face_cells: &[ArrangementFaceCell],
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Vec<ArrangementRegion> {
    if face_cells.is_empty() {
        return Vec::new();
    }
    let mut adjacency = vec![Vec::<usize>::new(); face_cells.len()];
    let edge_users = arrangement_edge_users(face_cells, blockers);
    let mut adjacent_pairs = Vec::<[usize; 2]>::new();
    for (_, users) in &edge_users {
        for left_index in 0..users.len() {
            for right_index in (left_index + 1)..users.len() {
                let left = users[left_index];
                let right = users[right_index];
                adjacency[left].push(right);
                adjacency[right].push(left);
                adjacent_pairs.push(if left <= right {
                    [left, right]
                } else {
                    [right, left]
                });
            }
        }
    }
    for neighbors in &mut adjacency {
        neighbors.sort_unstable();
        neighbors.dedup();
    }
    adjacent_pairs.sort_unstable();
    adjacent_pairs.dedup();

    let mut seen = vec![false; face_cells.len()];
    let mut regions = Vec::new();
    for start in 0..face_cells.len() {
        if seen[start] {
            continue;
        }
        let mut stack = vec![start];
        let mut component = Vec::new();
        seen[start] = true;
        while let Some(cell) = stack.pop() {
            component.push(cell);
            for neighbor in &adjacency[cell] {
                if !seen[*neighbor] {
                    seen[*neighbor] = true;
                    stack.push(*neighbor);
                }
            }
        }
        component.sort_unstable();
        let Some(membership) =
            ArrangementRegionComponentMembership::new(&component, face_cells.len())
        else {
            push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
            continue;
        };
        let adjacent_face_cells = adjacent_pairs
            .iter()
            .copied()
            .filter(|[left, right]| membership.contains(*left) && membership.contains(*right))
            .collect::<Vec<_>>();
        let edge_incidences =
            arrangement_region_edge_incidences(&membership, &edge_users, face_cells);
        let non_manifold_edges = edge_incidences
            .iter()
            .filter(|incidence| incidence.non_manifold)
            .count();
        let boundary_edges = edge_incidences
            .iter()
            .filter(|incidence| incidence.boundary)
            .count();
        let oriented_sides = arrangement_region_oriented_sides(&component, face_cells);
        let mut has_left_source = false;
        let mut has_right_source = false;
        for &face_cell in &component {
            match face_cells[face_cell].carrier.side {
                MeshSide::Left => has_left_source = true,
                MeshSide::Right => has_right_source = true,
            }
        }
        let mut source_sides = Vec::new();
        if has_left_source {
            source_sides.push(MeshSide::Left);
        }
        if has_right_source {
            source_sides.push(MeshSide::Right);
        }
        regions.push(ArrangementRegion {
            face_cells: component,
            adjacent_face_cells,
            edge_incidences,
            oriented_sides,
            boundary_edges,
            non_manifold_edges,
            source_sides,
            closed: boundary_edges == 0,
            manifold: non_manifold_edges == 0,
        });
    }
    regions.sort_by_key(|region| region.face_cells.first().copied().unwrap_or(usize::MAX));
    regions
}

struct ArrangementRegionComponentMembership {
    members: Vec<bool>,
}

impl ArrangementRegionComponentMembership {
    fn new(component: &[usize], face_cell_count: usize) -> Option<Self> {
        let mut members = vec![false; face_cell_count];
        for &cell in component {
            let member = members.get_mut(cell)?;
            *member = true;
        }
        Some(Self { members })
    }

    fn contains(&self, face_cell: usize) -> bool {
        self.members.get(face_cell).copied() == Some(true)
    }
}

fn arrangement_region_edge_incidences(
    membership: &ArrangementRegionComponentMembership,
    edge_users: &[([ArrangementFaceCellNode; 2], Vec<usize>)],
    face_cells: &[ArrangementFaceCell],
) -> Vec<ArrangementRegionEdgeIncidence> {
    edge_users
        .iter()
        .filter_map(|(edge, users)| {
            let mut incident_face_cells = users
                .iter()
                .copied()
                .filter(|&cell| membership.contains(cell))
                .collect::<Vec<_>>();
            if incident_face_cells.is_empty() {
                return None;
            }
            incident_face_cells.sort_unstable();
            let incident_count = regularized_incident_sheet_count(&incident_face_cells, face_cells);
            Some(ArrangementRegionEdgeIncidence {
                edge: edge.clone(),
                face_cells: incident_face_cells,
                boundary: incident_count == 1,
                non_manifold: incident_count > 2,
            })
        })
        .collect()
}

fn regularized_incident_sheet_count(
    incident_cells: &[usize],
    all_face_cells: &[ArrangementFaceCell],
) -> usize {
    let mut representatives = Vec::<usize>::new();
    'incident: for &cell in incident_cells {
        for &representative in &representatives {
            let Some(left) = all_face_cells.get(cell) else {
                continue;
            };
            let Some(right) = all_face_cells.get(representative) else {
                continue;
            };
            if exact_boundary_loops_equivalent(&left.boundary_points, &right.boundary_points) {
                continue 'incident;
            }
        }
        representatives.push(cell);
    }
    representatives.len()
}

fn exact_boundary_loops_equivalent(left: &[Point3], right: &[Point3]) -> bool {
    if left.len() != right.len() {
        return false;
    }
    if left.is_empty() {
        return true;
    }
    let matches = |reverse| {
        (0..right.len()).any(|offset| {
            (0..left.len()).all(|index| {
                let right_index = if reverse {
                    (offset + right.len() - index) % right.len()
                } else {
                    (offset + index) % right.len()
                };
                point3_equal(&left[index], &right[right_index]).value() == Some(true)
            })
        })
    };
    matches(false) || matches(true)
}

fn arrangement_region_oriented_sides(
    component: &[usize],
    face_cells: &[ArrangementFaceCell],
) -> Vec<ArrangementRegionSide> {
    component
        .iter()
        .map(|&face_cell| {
            let cell = &face_cells[face_cell];
            ArrangementRegionSide {
                face_cell,
                source: cell.carrier.side,
                source_face: cell.carrier.face,
                boundary: cell.boundary.clone(),
            }
        })
        .collect()
}

fn arrangement_volume_graph(
    shell_regions: &[ArrangementRegion],
    face_cells: &[ArrangementFaceCell],
    left: &ExactMesh,
    right: &ExactMesh,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> (
    Option<Vec<ArrangementVolumeRegion>>,
    Option<Vec<ArrangementVolumeAdjacency>>,
) {
    if shell_regions.is_empty()
        || shell_regions
            .iter()
            .any(|region| !region.closed || !region.manifold)
    {
        return (None, None);
    }

    if let Some(nested) =
        nested_shell_volume_graph(shell_regions, face_cells, left, right, blockers)
    {
        return nested;
    }

    let mut volume_regions = Vec::with_capacity(shell_regions.len() + 1);
    volume_regions.push(ArrangementVolumeRegion {
        index: 0,
        exterior: true,
        boundary_shells: (0..shell_regions.len()).collect(),
        source_sides: Vec::new(),
    });

    let mut volume_adjacencies = Vec::with_capacity(shell_regions.len());
    for (shell_index, shell) in shell_regions.iter().enumerate() {
        let interior_volume = volume_regions.len();
        volume_regions.push(ArrangementVolumeRegion {
            index: interior_volume,
            exterior: false,
            boundary_shells: vec![shell_index],
            source_sides: shell.source_sides.clone(),
        });
        volume_adjacencies.push(ArrangementVolumeAdjacency {
            shell_region: shell_index,
            exterior_volume: 0,
            interior_volume,
            separating_face_cells: shell.face_cells.clone(),
            oriented_face_sides: arrangement_volume_face_sides(shell, 0, interior_volume),
        });
    }

    (Some(volume_regions), Some(volume_adjacencies))
}

fn validate_arrangement_volume_graph(
    shell_regions: &[ArrangementRegion],
    face_cells: &[ArrangementFaceCell],
    volume_regions: Option<&[ArrangementVolumeRegion]>,
    volume_adjacencies: Option<&[ArrangementVolumeAdjacency]>,
    blockers: &mut Vec<ExactArrangementBlocker>,
) {
    if shell_regions.is_empty() {
        if volume_regions.is_some_and(|regions| !regions.is_empty())
            || volume_adjacencies.is_some_and(|adjacencies| !adjacencies.is_empty())
        {
            push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
        }
        return;
    }
    if shell_regions
        .iter()
        .any(|region| !region.closed || !region.manifold)
    {
        return;
    }
    let (Some(volume_regions), Some(volume_adjacencies)) = (volume_regions, volume_adjacencies)
    else {
        push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
        return;
    };
    if volume_regions
        .iter()
        .enumerate()
        .any(|(index, region)| region.index != index)
    {
        push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
    }
    if volume_regions
        .iter()
        .filter(|region| region.exterior)
        .count()
        != 1
    {
        push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
    }

    let mut shell_adjacency_counts = vec![0usize; shell_regions.len()];
    for adjacency in volume_adjacencies {
        if adjacency.shell_region >= shell_regions.len()
            || adjacency.exterior_volume >= volume_regions.len()
            || adjacency.interior_volume >= volume_regions.len()
            || adjacency.exterior_volume == adjacency.interior_volume
        {
            push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
            continue;
        }
        shell_adjacency_counts[adjacency.shell_region] += 1;
        let shell = &shell_regions[adjacency.shell_region];
        let separating_face_cells_match = sorted_unique_usize_set(&adjacency.separating_face_cells)
            .is_some_and(|left| {
                sorted_unique_usize_set(&shell.face_cells).is_some_and(|right| left == right)
            });
        if !separating_face_cells_match
            || !volume_face_sides_match_shell(adjacency, shell)
            || !volume_regions[adjacency.exterior_volume]
                .boundary_shells
                .contains(&adjacency.shell_region)
            || !volume_regions[adjacency.interior_volume]
                .boundary_shells
                .contains(&adjacency.shell_region)
        {
            push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
        }
    }
    if shell_adjacency_counts.into_iter().any(|count| count != 1) {
        push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
    }
    if !volume_region_boundary_shells_match_adjacencies(
        shell_regions.len(),
        volume_regions,
        volume_adjacencies,
    ) {
        push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
    }
    validate_volume_region_source_labels(
        shell_regions,
        face_cells,
        volume_regions,
        volume_adjacencies,
        blockers,
    );
}

fn volume_region_boundary_shells_match_adjacencies(
    shell_count: usize,
    volume_regions: &[ArrangementVolumeRegion],
    volume_adjacencies: &[ArrangementVolumeAdjacency],
) -> bool {
    let mut expected = vec![Vec::<usize>::new(); volume_regions.len()];
    for adjacency in volume_adjacencies {
        if adjacency.shell_region >= shell_count
            || adjacency.exterior_volume >= volume_regions.len()
            || adjacency.interior_volume >= volume_regions.len()
            || adjacency.exterior_volume == adjacency.interior_volume
        {
            return false;
        }
        expected[adjacency.exterior_volume].push(adjacency.shell_region);
        expected[adjacency.interior_volume].push(adjacency.shell_region);
    }

    for shells in &mut expected {
        shells.sort_unstable();
        shells.dedup();
    }

    volume_regions.iter().enumerate().all(|(index, region)| {
        let Some(boundary_shells) = sorted_unique_usize_set(&region.boundary_shells) else {
            return false;
        };
        boundary_shells == expected[index]
    })
}

pub(crate) fn sorted_unique_usize_set(values: &[usize]) -> Option<Vec<usize>> {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    let mut unique = sorted.clone();
    unique.dedup();
    (unique.len() == sorted.len()).then_some(sorted)
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct VolumeSourceState {
    source_sides: Vec<MeshSide>,
    source_shells: Vec<(MeshSide, usize)>,
}

fn validate_volume_region_source_labels(
    shell_regions: &[ArrangementRegion],
    face_cells: &[ArrangementFaceCell],
    volume_regions: &[ArrangementVolumeRegion],
    volume_adjacencies: &[ArrangementVolumeAdjacency],
    blockers: &mut Vec<ExactArrangementBlocker>,
) {
    let exterior_volume = volume_regions.iter().position(|region| region.exterior);
    let Some(exterior_volume) = exterior_volume else {
        push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
        return;
    };
    if !volume_regions[exterior_volume].source_sides.is_empty() {
        push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
    }
    let shell_orientations = match shell_regions
        .iter()
        .map(|region| {
            shell_region_mesh(region, face_cells).map(|mesh| exact_mesh_orientation(&mesh))
        })
        .collect::<Result<Vec<_>, _>>()
    {
        Ok(orientations) => orientations,
        Err(blocker) => {
            push_unique_blocker(blockers, blocker);
            return;
        }
    };
    if shell_orientations.iter().any(|orientation| {
        !matches!(
            orientation,
            ClosedMeshOrientation::Positive | ClosedMeshOrientation::Negative
        )
    }) {
        push_unique_blocker(
            blockers,
            ExactArrangementBlocker::UnresolvedRegionClassification,
        );
        return;
    }

    let mut expected = vec![None::<VolumeSourceState>; volume_regions.len()];
    expected[exterior_volume] = Some(VolumeSourceState {
        source_sides: Vec::new(),
        source_shells: Vec::new(),
    });
    let mut changed = true;
    while changed {
        changed = false;
        for adjacency in volume_adjacencies {
            if adjacency.exterior_volume >= volume_regions.len()
                || adjacency.interior_volume >= volume_regions.len()
                || adjacency.shell_region >= shell_regions.len()
            {
                continue;
            }
            let Some(exterior_state) = expected[adjacency.exterior_volume].clone() else {
                continue;
            };
            let mut interior_state = exterior_state;
            for side in &shell_regions[adjacency.shell_region].source_sides {
                apply_nested_shell_source_side(
                    &mut interior_state.source_sides,
                    &mut interior_state.source_shells,
                    *side,
                    adjacency.shell_region,
                    &shell_orientations,
                );
            }
            match &expected[adjacency.interior_volume] {
                Some(existing) if *existing != interior_state => {
                    push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
                }
                Some(_) => {}
                None => {
                    expected[adjacency.interior_volume] = Some(interior_state);
                    changed = true;
                }
            }
        }
    }

    for (index, region) in volume_regions.iter().enumerate() {
        match &expected[index] {
            Some(expected) if region.source_sides == expected.source_sides => {}
            _ => push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex),
        }
    }
}

fn volume_face_sides_match_shell(
    adjacency: &ArrangementVolumeAdjacency,
    shell: &ArrangementRegion,
) -> bool {
    if adjacency.oriented_face_sides.is_empty()
        || adjacency.oriented_face_sides.len() > shell.oriented_sides.len()
    {
        return false;
    }
    let every_volume_side_matches_shell = adjacency.oriented_face_sides.iter().all(|volume_side| {
        volume_side.exterior_volume == adjacency.exterior_volume
            && volume_side.interior_volume == adjacency.interior_volume
            && shell.oriented_sides.iter().any(|side| {
                volume_side.face_cell == side.face_cell
                    && volume_side.source == side.source
                    && volume_side.source_face == side.source_face
                    && volume_side.boundary == side.boundary
            })
    });
    every_volume_side_matches_shell
        && shell.oriented_sides.iter().all(|side| {
            adjacency.oriented_face_sides.iter().any(|volume_side| {
                exact_node_loops_equivalent(&volume_side.boundary, &side.boundary)
            })
        })
}

fn arrangement_volume_face_sides(
    shell: &ArrangementRegion,
    exterior_volume: usize,
    interior_volume: usize,
) -> Vec<ArrangementVolumeFaceSide> {
    let mut sides = Vec::<ArrangementVolumeFaceSide>::new();
    for side in &shell.oriented_sides {
        if sides
            .iter()
            .any(|existing| exact_node_loops_equivalent(&existing.boundary, &side.boundary))
        {
            continue;
        }
        sides.push(ArrangementVolumeFaceSide {
            face_cell: side.face_cell,
            source: side.source,
            source_face: side.source_face,
            boundary: side.boundary.clone(),
            exterior_volume,
            interior_volume,
        });
    }
    sides
}

pub(crate) fn exact_node_loops_equivalent(
    left: &[ArrangementFaceCellNode],
    right: &[ArrangementFaceCellNode],
) -> bool {
    if left.len() != right.len() {
        return false;
    }
    if left.is_empty() {
        return true;
    }
    let matches = |reverse| {
        (0..right.len()).any(|offset| {
            (0..left.len()).all(|index| {
                let right_index = if reverse {
                    (offset + right.len() - index) % right.len()
                } else {
                    (offset + index) % right.len()
                };
                left[index] == right[right_index]
            })
        })
    };
    matches(false) || matches(true)
}

type NestedVolumeGraph = (
    Option<Vec<ArrangementVolumeRegion>>,
    Option<Vec<ArrangementVolumeAdjacency>>,
);

fn nested_shell_volume_graph(
    shell_regions: &[ArrangementRegion],
    face_cells: &[ArrangementFaceCell],
    left: &ExactMesh,
    right: &ExactMesh,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Option<NestedVolumeGraph> {
    if shell_regions
        .iter()
        .any(|region| region.source_sides.is_empty())
    {
        return None;
    }

    let shell_meshes = match shell_regions
        .iter()
        .map(|region| shell_region_mesh(region, face_cells))
        .collect::<Result<Vec<_>, _>>()
    {
        Ok(meshes) => meshes,
        Err(blocker) => {
            push_unique_blocker(blockers, blocker);
            return None;
        }
    };
    let shell_orientations = shell_meshes
        .iter()
        .map(exact_mesh_orientation)
        .collect::<Vec<_>>();
    if shell_orientations.iter().any(|orientation| {
        !matches!(
            orientation,
            ClosedMeshOrientation::Positive | ClosedMeshOrientation::Negative
        )
    }) {
        push_unique_blocker(
            blockers,
            ExactArrangementBlocker::UnresolvedRegionClassification,
        );
        return None;
    }
    let mut contains = vec![vec![false; shell_regions.len()]; shell_regions.len()];
    for (contained, contained_by) in contains.iter_mut().enumerate() {
        let witnesses =
            shell_region_witnesses(shell_regions.get(contained)?, face_cells, left, right);
        if witnesses.is_empty() {
            push_unique_blocker(
                blockers,
                ExactArrangementBlocker::UnresolvedRegionClassification,
            );
            return None;
        }
        for (container, contained_by_container) in contained_by.iter_mut().enumerate() {
            if contained == container {
                continue;
            }
            match classify_shell_witnesses_against_container(&witnesses, &shell_meshes[container]) {
                ShellContainmentRelation::Inside => *contained_by_container = true,
                ShellContainmentRelation::Outside => {}
                ShellContainmentRelation::Boundary => {
                    push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
                    return None;
                }
                ShellContainmentRelation::Unknown => {
                    push_unique_blocker(
                        blockers,
                        ExactArrangementBlocker::UnresolvedRegionClassification,
                    );
                    return None;
                }
            }
        }
    }

    for (left, left_contains) in contains.iter().enumerate() {
        for (right, right_contains) in contains.iter().enumerate().skip(left + 1) {
            if left_contains[right] && right_contains[left] {
                push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
                return None;
            }
        }
    }

    let mut parents = vec![None; shell_regions.len()];
    for shell in 0..shell_regions.len() {
        let containers = (0..shell_regions.len())
            .filter(|&candidate| contains[shell][candidate])
            .collect::<Vec<_>>();
        let Some(parent) = deepest_containing_shell(&containers, &contains) else {
            continue;
        };
        parents[shell] = Some(parent);
    }
    diagnose_same_source_same_orientation_nesting(
        shell_regions,
        &shell_orientations,
        &parents,
        blockers,
    );

    let mut children = vec![Vec::<usize>::new(); shell_regions.len()];
    for (shell, parent) in parents.iter().enumerate() {
        if let Some(parent) = *parent {
            children[parent].push(shell);
        }
    }

    let roots = (0..shell_regions.len())
        .filter(|&shell| parents[shell].is_none())
        .collect::<Vec<_>>();
    let mut volume_regions = Vec::with_capacity(shell_regions.len() + 1);
    volume_regions.push(ArrangementVolumeRegion {
        index: 0,
        exterior: true,
        boundary_shells: roots.clone(),
        source_sides: Vec::new(),
    });
    let mut volume_adjacencies = Vec::with_capacity(shell_regions.len());
    let mut shell_volume = vec![None; shell_regions.len()];
    for root in roots {
        push_nested_shell_volume(
            root,
            0,
            &[],
            &[],
            &shell_orientations,
            shell_regions,
            &children,
            &mut shell_volume,
            &mut volume_regions,
            &mut volume_adjacencies,
        );
    }

    if shell_volume.iter().any(Option::is_none) {
        return None;
    }

    Some((Some(volume_regions), Some(volume_adjacencies)))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ShellContainmentRelation {
    Inside,
    Outside,
    Boundary,
    Unknown,
}

fn classify_shell_witnesses_against_container(
    witnesses: &[Point3],
    container: &ExactMesh,
) -> ShellContainmentRelation {
    let mut decided = None;
    let mut saw_boundary = false;
    for witness in witnesses {
        match classify_shell_witness_against_container(witness, container) {
            ShellContainmentRelation::Boundary => saw_boundary = true,
            ShellContainmentRelation::Unknown => {}
            relation @ (ShellContainmentRelation::Inside | ShellContainmentRelation::Outside) => {
                match decided {
                    Some(existing) if existing != relation => {
                        return ShellContainmentRelation::Boundary;
                    }
                    Some(_) => {}
                    None => decided = Some(relation),
                }
            }
        }
    }
    decided.unwrap_or(if saw_boundary {
        ShellContainmentRelation::Boundary
    } else {
        ShellContainmentRelation::Unknown
    })
}

fn classify_shell_witness_against_container(
    witness: &Point3,
    container: &ExactMesh,
) -> ShellContainmentRelation {
    let convex = classify_point_against_convex_solid_report(witness, container);
    if let Some(relation) = certified_convex_point_relation(convex.relation()) {
        return match relation {
            ConvexSolidPointRelation::Inside => ShellContainmentRelation::Inside,
            ConvexSolidPointRelation::Outside => ShellContainmentRelation::Outside,
            ConvexSolidPointRelation::Boundary => ShellContainmentRelation::Boundary,
            ConvexSolidPointRelation::Unknown | ConvexSolidPointRelation::NotCertifiedConvex => {
                ShellContainmentRelation::Unknown
            }
        };
    }

    ShellContainmentRelation::Unknown
}

fn deepest_containing_shell(containers: &[usize], contains: &[Vec<bool>]) -> Option<usize> {
    let mut best = None;
    let mut best_depth = 0usize;
    for &candidate in containers {
        let depth = containers
            .iter()
            .filter(|&&other| other != candidate && contains[candidate][other])
            .count();
        if best.is_none() || depth > best_depth {
            best = Some(candidate);
            best_depth = depth;
        } else if depth == best_depth {
            return None;
        }
    }
    best
}

fn diagnose_same_source_same_orientation_nesting(
    shell_regions: &[ArrangementRegion],
    shell_orientations: &[ClosedMeshOrientation],
    parents: &[Option<usize>],
    blockers: &mut Vec<ExactArrangementBlocker>,
) {
    for (shell, parent) in parents.iter().enumerate() {
        let Some(parent) = *parent else {
            continue;
        };
        for source in &shell_regions[shell].source_sides {
            if shell_regions[parent].source_sides.contains(source)
                && shell_orientations.get(shell) == shell_orientations.get(parent)
            {
                push_unique_blocker(blockers, ExactArrangementBlocker::NonManifoldCellComplex);
            }
        }
    }
}

fn push_unique_blocker(
    blockers: &mut Vec<ExactArrangementBlocker>,
    blocker: ExactArrangementBlocker,
) {
    if !blockers.contains(&blocker) {
        blockers.push(blocker);
    }
}

fn push_nested_shell_volume(
    shell: usize,
    exterior_volume: usize,
    exterior_source_sides: &[MeshSide],
    exterior_source_shells: &[(MeshSide, usize)],
    shell_orientations: &[ClosedMeshOrientation],
    shell_regions: &[ArrangementRegion],
    children: &[Vec<usize>],
    shell_volume: &mut [Option<usize>],
    volume_regions: &mut Vec<ArrangementVolumeRegion>,
    volume_adjacencies: &mut Vec<ArrangementVolumeAdjacency>,
) {
    let volume = volume_regions.len();
    let mut source_sides = exterior_source_sides.to_vec();
    let mut source_shells = exterior_source_shells.to_vec();
    for side in &shell_regions[shell].source_sides {
        apply_nested_shell_source_side(
            &mut source_sides,
            &mut source_shells,
            *side,
            shell,
            shell_orientations,
        );
    }
    let mut boundary_shells = Vec::with_capacity(children[shell].len() + 1);
    boundary_shells.push(shell);
    boundary_shells.extend(children[shell].iter().copied());
    volume_regions.push(ArrangementVolumeRegion {
        index: volume,
        exterior: false,
        boundary_shells,
        source_sides: source_sides.clone(),
    });
    shell_volume[shell] = Some(volume);
    volume_adjacencies.push(ArrangementVolumeAdjacency {
        shell_region: shell,
        exterior_volume,
        interior_volume: volume,
        separating_face_cells: shell_regions[shell].face_cells.clone(),
        oriented_face_sides: arrangement_volume_face_sides(
            &shell_regions[shell],
            exterior_volume,
            volume,
        ),
    });
    for &child in &children[shell] {
        push_nested_shell_volume(
            child,
            volume,
            &source_sides,
            &source_shells,
            shell_orientations,
            shell_regions,
            children,
            shell_volume,
            volume_regions,
            volume_adjacencies,
        );
    }
}

fn apply_nested_shell_source_side(
    sides: &mut Vec<MeshSide>,
    source_shells: &mut Vec<(MeshSide, usize)>,
    side: MeshSide,
    shell: usize,
    shell_orientations: &[ClosedMeshOrientation],
) {
    match source_shells
        .iter()
        .rposition(|(active_side, _)| *active_side == side)
        .and_then(|position| {
            source_shells
                .get(position)
                .map(|(_, active_shell)| (position, *active_shell))
        }) {
        Some((position, active_shell))
            if shell_orientations.get(active_shell) != shell_orientations.get(shell) =>
        {
            source_shells.remove(position);
            sides.retain(|active| *active != side);
        }
        Some(_) => {}
        None => {
            source_shells.push((side, shell));
            sides.push(side);
        }
    }
}

fn shell_region_witnesses(
    shell: &ArrangementRegion,
    face_cells: &[ArrangementFaceCell],
    left: &ExactMesh,
    right: &ExactMesh,
) -> Vec<Point3> {
    let mut witnesses = Vec::new();
    let mut witness_index = ArrangementPointUniquenessIndex::default();
    for cell in shell
        .face_cells
        .iter()
        .filter_map(|&cell| face_cells.get(cell))
    {
        for point in &cell.boundary_points {
            witness_index.push_unique(&mut witnesses, point.clone());
        }
        if cell.boundary_points.len() >= 3 {
            let mesh = cell.carrier.side.mesh(left, right);
            let mut blockers = Vec::new();
            if let Some(projection) =
                choose_triangle_projection(mesh, cell.carrier.triangle, &mut blockers)
            {
                let projected = cell
                    .boundary_points
                    .iter()
                    .map(|point| project_point3(point, projection))
                    .collect::<Vec<_>>();
                if let Ok(witness) = projected_loop_interior_witness(&projected)
                    && let Some(point) = lift_carrier_plane_point(
                        mesh,
                        cell.carrier.face,
                        projection,
                        &witness,
                        &mut blockers,
                    )
                {
                    witness_index.push_unique(&mut witnesses, point);
                }
            }
        }
    }
    witnesses
}

fn shell_region_mesh(
    shell: &ArrangementRegion,
    face_cells: &[ArrangementFaceCell],
) -> Result<ExactMesh, ExactArrangementBlocker> {
    let mut boundary_loops = Vec::<Vec<Point3>>::new();
    for &cell_index in &shell.face_cells {
        let cell = face_cells
            .get(cell_index)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
        if cell.boundary_points.len() < 3 {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        if boundary_loops
            .iter()
            .any(|existing| exact_boundary_loops_equivalent(existing, &cell.boundary_points))
        {
            continue;
        }
        boundary_loops.push(cell.boundary_points.clone());
    }
    let boundary_loop_groups = group_exact_coplanar_loops(boundary_loops)?;
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for boundary_group in &boundary_loop_groups {
        triangulate_exact_loop_group(boundary_group, &mut vertices, &mut triangles)?;
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact arrangement shell replay"),
        ExactMeshValidationPolicy::CLOSED,
    )
    .map_err(|_| ExactArrangementBlocker::NonManifoldCellComplex)
}

struct ArrangementFaceCellBoundaryEdge {
    nodes: [ArrangementFaceCellNode; 2],
    points: Option<[Point3; 2]>,
}

#[derive(Clone)]
struct ArrangementFaceCellRawBoundaryEdge {
    start: ArrangementFaceCellBoundaryPoint,
    end: ArrangementFaceCellBoundaryPoint,
    cell: usize,
}

#[derive(Clone)]
struct ArrangementFaceCellBoundaryPoint {
    node: ArrangementFaceCellNode,
    point: Point3,
}

fn arrangement_edge_users(
    face_cells: &[ArrangementFaceCell],
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Vec<([ArrangementFaceCellNode; 2], Vec<usize>)> {
    let mut raw_edges = Vec::new();
    for (cell, face_cell) in face_cells.iter().enumerate() {
        if face_cell.boundary.len() < 2 {
            continue;
        }
        for index in 0..face_cell.boundary.len() {
            let next = (index + 1) % face_cell.boundary.len();
            raw_edges.push(ArrangementFaceCellRawBoundaryEdge {
                start: ArrangementFaceCellBoundaryPoint {
                    node: face_cell.boundary[index].clone(),
                    point: face_cell.boundary_points[index].clone(),
                },
                end: ArrangementFaceCellBoundaryPoint {
                    node: face_cell.boundary[next].clone(),
                    point: face_cell.boundary_points[next].clone(),
                },
                cell,
            });
        }
    }
    if raw_edges.is_empty() {
        return Vec::new();
    }
    let endpoints = raw_edges
        .iter()
        .flat_map(|edge| [edge.start.clone(), edge.end.clone()])
        .collect::<Vec<_>>();
    let mut edge_users = ArrangementEdgeUserIndex::default();
    for edge in raw_edges {
        let mut split_points = vec![edge.start.clone(), edge.end.clone()];
        let mut split_point_index =
            ArrangementBoundaryPointUniquenessIndex::from_points(&split_points);
        for endpoint in &endpoints {
            if boundary_points_equal(endpoint, &edge.start)
                || boundary_points_equal(endpoint, &edge.end)
            {
                continue;
            }
            match point_on_segment3(&edge.start.point, &edge.end.point, &endpoint.point).value() {
                Some(true) => {
                    split_point_index.push_unique(&mut split_points, endpoint.clone());
                }
                Some(false) => {}
                None => push_unique_blocker(blockers, ExactArrangementBlocker::UndecidableOrdering),
            }
        }
        if sort_boundary_points_along_segment(&edge.start.point, &edge.end.point, &mut split_points)
            .is_err()
        {
            push_unique_blocker(blockers, ExactArrangementBlocker::UndecidableOrdering);
            edge_users.push(
                ArrangementFaceCellBoundaryEdge {
                    nodes: canonical_cell_edge(edge.start.node.clone(), edge.end.node.clone()),
                    points: Some([edge.start.point.clone(), edge.end.point.clone()]),
                },
                edge.cell,
            );
            continue;
        }
        for pair in split_points.windows(2) {
            if boundary_points_equal(&pair[0], &pair[1]) {
                continue;
            }
            edge_users.push(
                ArrangementFaceCellBoundaryEdge {
                    nodes: canonical_cell_edge(pair[0].node.clone(), pair[1].node.clone()),
                    points: Some([pair[0].point.clone(), pair[1].point.clone()]),
                },
                edge.cell,
            );
        }
    }
    edge_users
        .edge_users
        .into_iter()
        .map(|(edge, users)| (edge.nodes, users))
        .collect()
}

type ArrangementBoundaryNodeKey = [(usize, usize, usize); 2];

#[derive(Default)]
struct ArrangementEdgeUserIndex {
    edge_users: Vec<(ArrangementFaceCellBoundaryEdge, Vec<usize>)>,
    node_key_buckets: BTreeMap<ArrangementBoundaryNodeKey, usize>,
    point_key_buckets: BTreeMap<ExactUndirectedPoint3EdgeKey, usize>,
    unkeyed_edges: Vec<usize>,
}

impl ArrangementEdgeUserIndex {
    fn push(&mut self, edge: ArrangementFaceCellBoundaryEdge, cell: usize) {
        let node_key = [cell_node_key(&edge.nodes[0]), cell_node_key(&edge.nodes[1])];
        let point_key = edge
            .points
            .as_ref()
            .and_then(exact_undirected_point3_edge_key);
        if let Some(index) = self
            .node_key_buckets
            .get(&node_key)
            .copied()
            .or_else(|| {
                point_key
                    .as_ref()
                    .and_then(|key| self.point_key_buckets.get(key).copied())
            })
            .or_else(|| self.find_fallback(&edge, point_key.is_some()))
        {
            self.node_key_buckets.entry(node_key).or_insert(index);
            if let Some(key) = point_key {
                self.point_key_buckets.entry(key).or_insert(index);
            }
            if !self.edge_users[index].1.contains(&cell) {
                self.edge_users[index].1.push(cell);
            }
            return;
        }

        let index = self.edge_users.len();
        self.node_key_buckets.entry(node_key).or_insert(index);
        if let Some(key) = point_key {
            self.point_key_buckets.entry(key).or_insert(index);
        } else {
            self.unkeyed_edges.push(index);
        }
        self.edge_users.push((edge, vec![cell]));
    }

    fn find_fallback(
        &self,
        edge: &ArrangementFaceCellBoundaryEdge,
        has_point_key: bool,
    ) -> Option<usize> {
        if has_point_key {
            self.unkeyed_edges
                .iter()
                .copied()
                .find(|&index| boundary_edges_equivalent(&self.edge_users[index].0, edge))
        } else {
            self.edge_users
                .iter()
                .enumerate()
                .find(|(_, (existing, _))| boundary_edges_equivalent(existing, edge))
                .map(|(index, _)| index)
        }
    }
}

#[derive(Default)]
struct ArrangementBoundaryPointUniquenessIndex {
    point_key_buckets: BTreeMap<ExactPoint3Key, Vec<usize>>,
    unkeyed_points: Vec<usize>,
}

impl ArrangementBoundaryPointUniquenessIndex {
    fn from_points(points: &[ArrangementFaceCellBoundaryPoint]) -> Self {
        let mut index = Self::default();
        for (point_index, point) in points.iter().enumerate() {
            index.insert(point_index, exact_point3_key(&point.point));
        }
        index
    }

    fn push_unique(
        &mut self,
        points: &mut Vec<ArrangementFaceCellBoundaryPoint>,
        point: ArrangementFaceCellBoundaryPoint,
    ) {
        let point_key = exact_point3_key(&point.point);
        if let Some(existing) = self.find_matching(&point, point_key.as_ref(), points) {
            if cell_node_key(&point.node) < cell_node_key(&points[existing].node) {
                points[existing].node = point.node;
            }
            return;
        }
        let point_index = points.len();
        self.insert(point_index, point_key);
        points.push(point);
    }

    fn insert(&mut self, point_index: usize, point_key: Option<ExactPoint3Key>) {
        if let Some(key) = point_key {
            self.point_key_buckets
                .entry(key)
                .or_default()
                .push(point_index);
        } else {
            self.unkeyed_points.push(point_index);
        }
    }

    fn find_matching(
        &self,
        point: &ArrangementFaceCellBoundaryPoint,
        point_key: Option<&ExactPoint3Key>,
        points: &[ArrangementFaceCellBoundaryPoint],
    ) -> Option<usize> {
        if let Some(key) = point_key {
            if let Some(bucket) = self.point_key_buckets.get(key)
                && let Some(index) = find_matching_boundary_point(point, points, bucket)
            {
                return Some(index);
            }
            return find_matching_boundary_point(point, points, &self.unkeyed_points);
        }

        for bucket in self.point_key_buckets.values() {
            if let Some(index) = find_matching_boundary_point(point, points, bucket) {
                return Some(index);
            }
        }
        find_matching_boundary_point(point, points, &self.unkeyed_points)
    }
}

fn find_matching_boundary_point(
    point: &ArrangementFaceCellBoundaryPoint,
    points: &[ArrangementFaceCellBoundaryPoint],
    candidates: &[usize],
) -> Option<usize> {
    candidates
        .iter()
        .copied()
        .find(|&index| boundary_points_equal(&points[index], point))
}

fn boundary_points_equal(
    left: &ArrangementFaceCellBoundaryPoint,
    right: &ArrangementFaceCellBoundaryPoint,
) -> bool {
    point3_equal(&left.point, &right.point).value() == Some(true)
}

fn sort_boundary_points_along_segment(
    start: &Point3,
    end: &Point3,
    points: &mut Vec<ArrangementFaceCellBoundaryPoint>,
) -> Result<(), ExactArrangementBlocker> {
    let axis =
        segment_order_axis(start, end).ok_or(ExactArrangementBlocker::UndecidableOrdering)?;
    let mut ordered = Vec::<ArrangementFaceCellBoundaryPoint>::new();
    'points: for point in points.drain(..) {
        for index in 0..ordered.len() {
            match compare_point3_on_axis(&point.point, &ordered[index].point, axis, start, end) {
                Some(Ordering::Less) => {
                    ordered.insert(index, point);
                    continue 'points;
                }
                Some(Ordering::Equal | Ordering::Greater) => {}
                None => return Err(ExactArrangementBlocker::UndecidableOrdering),
            }
        }
        ordered.push(point);
    }
    *points = ordered;
    Ok(())
}

fn boundary_edges_equivalent(
    left: &ArrangementFaceCellBoundaryEdge,
    right: &ArrangementFaceCellBoundaryEdge,
) -> bool {
    left.nodes == right.nodes
        || match (&left.points, &right.points) {
            (Some(left), Some(right)) => {
                (point3_equal(&left[0], &right[0]).value() == Some(true)
                    && point3_equal(&left[1], &right[1]).value() == Some(true))
                    || (point3_equal(&left[0], &right[1]).value() == Some(true)
                        && point3_equal(&left[1], &right[0]).value() == Some(true))
            }
            _ => false,
        }
}

fn canonical_cell_edge(
    left: ArrangementFaceCellNode,
    right: ArrangementFaceCellNode,
) -> [ArrangementFaceCellNode; 2] {
    if cell_node_key(&left) <= cell_node_key(&right) {
        [left, right]
    } else {
        [right, left]
    }
}

fn cell_node_key(node: &ArrangementFaceCellNode) -> (usize, usize, usize) {
    match node {
        ArrangementFaceCellNode::Source { side, vertex } => (0, side_key(*side), *vertex),
        ArrangementFaceCellNode::Graph { graph_vertex } => (1, 0, *graph_vertex),
        ArrangementFaceCellNode::CarrierPlane { overlay, vertex } => (2, *overlay, *vertex),
        ArrangementFaceCellNode::FacePlane {
            arrangement,
            vertex,
        } => (3, *arrangement, *vertex),
    }
}

const fn side_key(side: MeshSide) -> usize {
    match side {
        MeshSide::Left => 0,
        MeshSide::Right => 1,
    }
}

fn face_cell_from_region(
    region: &FaceRegionBoundary,
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactRegularizationPolicy,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> ArrangementFaceCell {
    let boundary = region
        .boundary
        .iter()
        .filter_map(|node| match node {
            FaceSplitBoundaryNode::OriginalVertex { vertex, .. } => {
                Some(ArrangementFaceCellNode::Source {
                    side: region.side,
                    vertex: *vertex,
                })
            }
            FaceSplitBoundaryNode::GraphVertex { graph_vertex, .. } => {
                Some(ArrangementFaceCellNode::Graph {
                    graph_vertex: *graph_vertex,
                })
            }
            FaceSplitBoundaryNode::FaceInterior { .. } => None,
        })
        .collect::<Vec<_>>();
    let boundary_points = region
        .boundary
        .iter()
        .map(|node| match node {
            FaceSplitBoundaryNode::OriginalVertex { point, .. }
            | FaceSplitBoundaryNode::GraphVertex { point, .. }
            | FaceSplitBoundaryNode::FaceInterior { point } => point.clone(),
        })
        .collect::<Vec<_>>();
    let representative = face_region_interior_representative(region, left, right, blockers)
        .or_else(|| representative_from_boundary_nodes(&region.boundary));
    let opposite = representative
        .map(|point| classify_opposite(region.side, point, left, right, policy, blockers));
    ArrangementFaceCell {
        carrier: ArrangementFaceCarrier {
            side: region.side,
            face: region.face,
            triangle: region.triangle,
        },
        boundary,
        boundary_points,
        opposite,
    }
}

fn face_cell_from_original_triangle(
    side: MeshSide,
    face: usize,
    triangle: [usize; 3],
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactRegularizationPolicy,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> ArrangementFaceCell {
    let mesh = side.mesh(left, right);
    let boundary = triangle
        .iter()
        .map(|vertex| ArrangementFaceCellNode::Source {
            side,
            vertex: *vertex,
        })
        .collect();
    let boundary_points = triangle
        .iter()
        .map(|vertex| mesh.vertices()[*vertex].clone())
        .collect();
    let third = (Real::from(1) / &Real::from(3)).ok();
    let representative = third.map(|third| {
        let a = &mesh.vertices()[triangle[0]];
        let b = &mesh.vertices()[triangle[1]];
        let c = &mesh.vertices()[triangle[2]];
        Point3::new(
            (a.x.clone() + &b.x + &c.x) * &third,
            (a.y.clone() + &b.y + &c.y) * &third,
            (a.z.clone() + &b.z + &c.z) * &third,
        )
    });
    let opposite =
        representative.map(|point| classify_opposite(side, point, left, right, policy, blockers));
    if opposite.is_none() && policy.unresolved == ExactUnresolvedPolicy::Block {
        blockers.push(ExactArrangementBlocker::UnresolvedRegionClassification);
    }
    ArrangementFaceCell {
        carrier: ArrangementFaceCarrier {
            side,
            face,
            triangle,
        },
        boundary,
        boundary_points,
        opposite,
    }
}

fn lift_carrier_plane_point(
    mesh: &ExactMesh,
    face: usize,
    projection: CoplanarProjection,
    point: &Point2,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Option<Point3> {
    let triangle = mesh.triangles().get(face)?.0;
    let a = mesh.vertices().get(triangle[0])?;
    let b = mesh.vertices().get(triangle[1])?;
    let c = mesh.vertices().get(triangle[2])?;
    let ab = Point3::new(b.x.clone() - &a.x, b.y.clone() - &a.y, b.z.clone() - &a.z);
    let ac = Point3::new(c.x.clone() - &a.x, c.y.clone() - &a.y, c.z.clone() - &a.z);
    let normal = Point3::new(
        ab.y.clone() * &ac.z - &(ab.z.clone() * &ac.y),
        ab.z.clone() * &ac.x - &(ab.x.clone() * &ac.z),
        ab.x.clone() * &ac.y - &(ab.y.clone() * &ac.x),
    );
    let plane_value =
        normal.x.clone() * &a.x + &(normal.y.clone() * &a.y) + &(normal.z.clone() * &a.z);

    let lifted = match projection {
        CoplanarProjection::Xy => {
            let x = point.x.clone();
            let y = point.y.clone();
            let numerator =
                plane_value.clone() - &(normal.x.clone() * &x) - &(normal.y.clone() * &y);
            let z = match (numerator / &normal.z).ok() {
                Some(z) => z,
                None => {
                    blockers.push(ExactArrangementBlocker::UnresolvedIntersection);
                    return None;
                }
            };
            Point3::new(x, y, z)
        }
        CoplanarProjection::Xz => {
            let x = point.x.clone();
            let z = point.y.clone();
            let numerator =
                plane_value.clone() - &(normal.x.clone() * &x) - &(normal.z.clone() * &z);
            let y = match (numerator / &normal.y).ok() {
                Some(y) => y,
                None => {
                    blockers.push(ExactArrangementBlocker::UnresolvedIntersection);
                    return None;
                }
            };
            Point3::new(x, y, z)
        }
        CoplanarProjection::Yz => {
            let y = point.x.clone();
            let z = point.y.clone();
            let numerator =
                plane_value.clone() - &(normal.y.clone() * &y) - &(normal.z.clone() * &z);
            let x = match (numerator / &normal.x).ok() {
                Some(x) => x,
                None => {
                    blockers.push(ExactArrangementBlocker::UnresolvedIntersection);
                    return None;
                }
            };
            Point3::new(x, y, z)
        }
    };

    match point2_equal(&project_point3(&lifted, projection), point).value() {
        Some(true) => Some(lifted),
        Some(false) => {
            blockers.push(ExactArrangementBlocker::UnresolvedIntersection);
            None
        }
        None => {
            blockers.push(ExactArrangementBlocker::UndecidableOrdering);
            None
        }
    }
}

fn projected_triangle_area2(points: &[Point3; 3], projection: CoplanarProjection) -> Real {
    let a = project_point3(&points[0], projection);
    let b = project_point3(&points[1], projection);
    let c = project_point3(&points[2], projection);
    (b.x.clone() - &a.x) * &(c.y.clone() - &a.y) - &((b.y.clone() - &a.y) * &(c.x.clone() - &a.x))
}

fn face_region_interior_representative(
    region: &FaceRegionBoundary,
    left: &ExactMesh,
    right: &ExactMesh,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Option<Point3> {
    let mesh = region.side.mesh(left, right);
    let fallback = || representative_from_boundary_nodes(&region.boundary);
    let mut witness_blockers = Vec::new();
    let Some(projection) = choose_triangle_projection(mesh, region.triangle, &mut witness_blockers)
    else {
        let fallback = fallback();
        if fallback.is_none() {
            blockers.extend(witness_blockers);
        }
        return fallback;
    };
    let projected = region
        .boundary
        .iter()
        .map(|node| {
            let point = match node {
                FaceSplitBoundaryNode::OriginalVertex { point, .. }
                | FaceSplitBoundaryNode::GraphVertex { point, .. }
                | FaceSplitBoundaryNode::FaceInterior { point } => point,
            };
            project_point3(point, projection)
        })
        .collect::<Vec<_>>();
    let witness = match projected_loop_interior_witness(&projected) {
        Ok(witness) => witness,
        Err(blocker) => {
            witness_blockers.push(blocker);
            let fallback = fallback();
            if fallback.is_none() {
                blockers.extend(witness_blockers);
            }
            return fallback;
        }
    };
    let mut lift_blockers = Vec::new();
    let lifted =
        lift_carrier_plane_point(mesh, region.face, projection, &witness, &mut lift_blockers);
    if lifted.is_none() {
        let fallback = fallback();
        if fallback.is_none() {
            blockers.extend(lift_blockers);
        }
        return fallback;
    }
    lifted
}

fn classify_opposite(
    side: MeshSide,
    point: Point3,
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactRegularizationPolicy,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> ArrangementOppositeClassification {
    let target = match side {
        MeshSide::Left => right,
        MeshSide::Right => left,
    };
    let convex = classify_point_against_convex_solid_report(&point, target);
    let convex_certification = if certified_convex_point_relation(convex.relation()).is_some() {
        Some(convex)
    } else {
        None
    };
    let winding = classify_point_against_closed_mesh_winding_report(&point, target);
    if matches!(
        winding.relation(),
        ClosedMeshWindingRelation::Unknown | ClosedMeshWindingRelation::NotClosed
    ) && convex_certification.is_none()
        && policy.unresolved == ExactUnresolvedPolicy::Block
    {
        blockers.push(ExactArrangementBlocker::UnresolvedRegionClassification);
    }
    ArrangementOppositeClassification {
        representative: point,
        winding,
        convex_fallback: convex_certification,
    }
}

fn certified_convex_point_relation(
    relation: ConvexSolidPointRelation,
) -> Option<ConvexSolidPointRelation> {
    match relation {
        ConvexSolidPointRelation::Inside
        | ConvexSolidPointRelation::Boundary
        | ConvexSolidPointRelation::Outside => Some(relation),
        ConvexSolidPointRelation::Unknown | ConvexSolidPointRelation::NotCertifiedConvex => None,
    }
}

fn representative_from_boundary_nodes(nodes: &[FaceSplitBoundaryNode]) -> Option<Point3> {
    if nodes.is_empty() {
        return None;
    }
    let inv = (Real::from(1) / &Real::from(nodes.len() as i64)).ok()?;
    let mut x = Real::from(0);
    let mut y = Real::from(0);
    let mut z = Real::from(0);
    for node in nodes {
        let point = match node {
            FaceSplitBoundaryNode::OriginalVertex { point, .. }
            | FaceSplitBoundaryNode::GraphVertex { point, .. }
            | FaceSplitBoundaryNode::FaceInterior { point } => point,
        };
        x += &point.x;
        y += &point.y;
        z += &point.z;
    }
    Some(Point3::new(x * &inv, y * &inv, z * &inv))
}

#[cfg(test)]
mod tests;
