//! Exact split-region classification.
//!
//! This module consumes the pre-triangulation region loops produced by the
//! intersection graph and classifies them against opposite mesh face planes.
//! The stage is intentionally still pre-boolean-output: it prepares certified
//! side facts for later winding/inside-outside classification without using a
//! primitive-float representative point. This follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997): topology
//! updates consume certified predicate facts, and undecided cases remain
//! explicit.
//!
//! Plane-side classification is the same predicate boundary used by Moller,
//! "A Fast Triangle-Triangle Intersection Test," *Journal of Graphics Tools*
//! 2.2 (1997), and Guigue and Devillers, "Fast and Robust Triangle-Triangle
//! Overlap Test Using Orientation Predicates," *Journal of Graphics Tools*
//! 8.1 (2003), but routed through `hyperlimit::orient3d_report`.

use hyperlimit::{PlaneSide, Point3, orient3d_report};
#[cfg(feature = "exact-triangulation")]
use hyperlimit::{Point2 as PredicatePoint2, Sign, orient2d_report};

#[cfg(feature = "exact-triangulation")]
use super::coplanar::CoplanarProjection;
use super::graph::{ExactFaceRegionPlan, FaceSplitBoundaryNode, MeshSide};
use super::mesh::ExactMesh;
#[cfg(feature = "exact-triangulation")]
use super::mesh::{ExactPoint3, Triangle};
use super::provenance::PredicateUse;
#[cfg(feature = "exact-triangulation")]
use super::provenance::SourceProvenance;
#[cfg(feature = "exact-triangulation")]
use super::validation::ValidationPolicy;

/// Exact relation between a split region boundary and an opposite face plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FaceRegionPlaneRelation {
    /// Every boundary node is strictly above the oriented plane.
    StrictlyAbove,
    /// Every boundary node is strictly below the oriented plane.
    StrictlyBelow,
    /// Every boundary node is exactly on the oriented plane.
    Coplanar,
    /// Boundary nodes occur on both sides, or on one side plus the plane.
    Straddling,
    /// At least one required plane-side predicate was undecided.
    Unknown,
}

/// Certified classification of one region against one opposite face plane.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FaceRegionPlaneClassification {
    /// Region mesh side.
    pub region_side: MeshSide,
    /// Region source face.
    pub region_face: usize,
    /// Opposite mesh side owning the tested plane.
    pub plane_side: MeshSide,
    /// Opposite face index.
    pub plane_face: usize,
    /// Coarse exact relation.
    pub relation: FaceRegionPlaneRelation,
    /// Per-boundary-node side, or `None` when undecided.
    pub node_sides: Vec<Option<PlaneSide>>,
    /// Predicate certificates used by the orientation tests.
    pub predicates: Vec<PredicateUse>,
}

impl FaceRegionPlaneClassification {
    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }
}

/// Classify every split region against every opposite mesh face plane.
///
/// This is a certified input to later winding/side policy. It deliberately
/// does not collapse a full mesh-side decision into one boolean: a region can
/// be coplanar with one face, straddle another face plane, and remain unknown
/// against a symbolic face until refinement or a policy decision is available.
pub fn classify_face_regions_against_opposite_planes(
    regions: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Vec<FaceRegionPlaneClassification> {
    let mut classifications = Vec::new();
    for region in &regions.regions {
        let (plane_side, plane_mesh) = match region.side {
            MeshSide::Left => (MeshSide::Right, right),
            MeshSide::Right => (MeshSide::Left, left),
        };
        for plane_face in 0..plane_mesh.triangles().len() {
            classifications.push(classify_region_against_face_plane(
                region.side,
                region.face,
                &region.boundary,
                plane_side,
                plane_mesh,
                plane_face,
            ));
        }
    }
    classifications
}

/// Exact earcut triangulation of one split face region.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct FaceRegionTriangulation {
    /// Mesh side owning the source face.
    pub side: MeshSide,
    /// Source face index.
    pub face: usize,
    /// Certified projection used for triangulation.
    pub projection: CoplanarProjection,
    /// Exact 3D boundary nodes whose order matches [`Self::vertices`].
    pub boundary: Vec<FaceSplitBoundaryNode>,
    /// Projected exact vertices passed to `hypertri`.
    pub vertices: Vec<hypertri::ExactPoint>,
    /// Triangle index buffer returned by exact earcut.
    pub triangles: Vec<usize>,
}

/// Region selection policy for exact output assembly.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactRegionSelection {
    /// Keep regions originating from both meshes.
    KeepAll,
    /// Keep only regions originating from the left mesh.
    KeepLeft,
    /// Keep only regions originating from the right mesh.
    KeepRight,
}

#[cfg(feature = "exact-triangulation")]
impl ExactRegionSelection {
    const fn keeps(self, side: MeshSide) -> bool {
        matches!(
            (self, side),
            (Self::KeepAll, _)
                | (Self::KeepLeft, MeshSide::Left)
                | (Self::KeepRight, MeshSide::Right)
        )
    }
}

/// One exact output vertex in an assembled region mesh.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactOutputVertex {
    /// Exact 3D point.
    pub point: Point3,
    /// Boundary node that produced this vertex.
    pub source: FaceSplitBoundaryNode,
}

/// One exact output triangle with source-region provenance.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactOutputTriangle {
    /// Indices into [`ExactBooleanAssemblyPlan::vertices`].
    pub vertices: [usize; 3],
    /// Mesh side of the source split region.
    pub source_side: MeshSide,
    /// Source face index.
    pub source_face: usize,
}

/// Non-mutating exact output mesh assembly plan.
///
/// This plan is the first boolean-output handoff: exact region loops have been
/// classified and triangulated, and this type records the exact 3D triangles
/// that a future halfedge builder can materialize. It does not hide operation
/// semantics in tolerances; callers pass an explicit region-selection policy,
/// and undecided winding/inside-outside policy remains outside this assembly
/// step. See Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBooleanAssemblyPlan {
    /// Exact output vertices.
    pub vertices: Vec<ExactOutputVertex>,
    /// Exact output triangles.
    pub triangles: Vec<ExactOutputTriangle>,
}

#[cfg(feature = "exact-triangulation")]
impl ExactBooleanAssemblyPlan {
    /// Assemble exact output triangles from feature-gated region
    /// triangulations.
    pub fn from_region_triangulations(
        triangulations: &[FaceRegionTriangulation],
        selection: ExactRegionSelection,
    ) -> hypertri::Result<Self> {
        assemble_region_triangulations(triangulations, selection)
    }

    /// Validate that every output triangle references existing output
    /// vertices and has three distinct vertex handles.
    pub fn validate(&self) -> hypertri::Result<()> {
        for triangle in &self.triangles {
            if triangle
                .vertices
                .iter()
                .any(|&index| index >= self.vertices.len())
            {
                return Err(hypertri::Error::InvalidInput {
                    reason: "assembled output triangle references a missing vertex",
                });
            }
            let [a, b, c] = triangle.vertices;
            if a == b || b == c || a == c {
                return Err(hypertri::Error::InvalidInput {
                    reason: "assembled output triangle has repeated vertex handles",
                });
            }
        }
        Ok(())
    }

    /// Materialize the assembly plan as an [`ExactMesh`] through the normal
    /// validation pipeline.
    ///
    /// This is the exact replacement boundary for the legacy boolean mutation
    /// path: constructed output triangles are converted back into hypermesh
    /// exact vertices and triangle handles, then checked by the same manifold
    /// and geometric validators used for caller-supplied exact meshes.
    pub fn to_exact_mesh(
        &self,
        policy: ValidationPolicy,
    ) -> Result<ExactMesh, super::error::MeshError> {
        let vertices = self
            .vertices
            .iter()
            .map(|vertex| {
                ExactPoint3::new(
                    vertex.point.x.clone(),
                    vertex.point.y.clone(),
                    vertex.point.z.clone(),
                )
            })
            .collect::<Vec<_>>();
        let triangles = self
            .triangles
            .iter()
            .map(|triangle| Triangle(triangle.vertices))
            .collect::<Vec<_>>();
        ExactMesh::new_with_policy(
            vertices,
            triangles,
            SourceProvenance::exact("exact boolean assembly plan"),
            policy,
        )
    }
}

/// Triangulate split face-region loops with `hypertri` exact earcut.
///
/// This bridge is behind `hypermesh`'s `exact-triangulation` cargo feature, so
/// users that only need validation, broad phase, or event graphs do not build
/// or link triangulation code. The projection is selected only after a
/// certified nonzero orientation predicate, matching the projection discipline
/// used for coplanar overlap classification.
#[cfg(feature = "exact-triangulation")]
pub fn triangulate_face_regions_with_earcut(
    regions: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<Vec<FaceRegionTriangulation>> {
    regions
        .regions
        .iter()
        .map(|region| {
            let mesh = match region.side {
                MeshSide::Left => left,
                MeshSide::Right => right,
            };
            let projection = choose_region_projection(mesh, region.face)?;
            let vertices = region
                .boundary
                .iter()
                .map(|node| project_for_hypertri(boundary_node_point(node), projection))
                .collect::<Vec<_>>();
            let triangles = hypertri::earcut(&vertices, &[])?;
            Ok(FaceRegionTriangulation {
                side: region.side,
                face: region.face,
                projection,
                boundary: region.boundary.clone(),
                vertices,
                triangles,
            })
        })
        .collect()
}

/// Build a validated exact mesh from selected split regions of two inputs.
///
/// This is the feature-gated exact-stack pipeline that replaces the old
/// tolerance-driven "split then mutate" shape for the subset currently
/// supported by the exact port. It intentionally accepts an explicit
/// [`ExactRegionSelection`] instead of pretending that winding/inside-outside
/// policy has been solved by a floating representative point. The internal
/// stages remain Yap-style auditable artifacts: event graph, region loops,
/// exact triangulation, assembly plan, and final exact mesh validation.
#[cfg(feature = "exact-triangulation")]
pub fn build_selected_region_mesh(
    left: &ExactMesh,
    right: &ExactMesh,
    selection: ExactRegionSelection,
    policy: ValidationPolicy,
) -> Result<ExactMesh, super::error::MeshError> {
    let graph = super::graph::build_intersection_graph(left, right)?;
    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let triangulations =
        triangulate_face_regions_with_earcut(&region_plan, left, right).map_err(|error| {
            super::error::MeshError::one(super::error::MeshDiagnostic::new(
                super::error::Severity::Error,
                super::error::DiagnosticKind::DegenerateTriangle,
                format!("exact region triangulation failed: {error}"),
            ))
        })?;
    let assembly = ExactBooleanAssemblyPlan::from_region_triangulations(&triangulations, selection)
        .map_err(|error| {
            super::error::MeshError::one(super::error::MeshDiagnostic::new(
                super::error::Severity::Error,
                super::error::DiagnosticKind::IndexOutOfBounds,
                format!("exact boolean assembly failed: {error}"),
            ))
        })?;
    assembly.to_exact_mesh(policy)
}

#[cfg(feature = "exact-triangulation")]
fn assemble_region_triangulations(
    triangulations: &[FaceRegionTriangulation],
    selection: ExactRegionSelection,
) -> hypertri::Result<ExactBooleanAssemblyPlan> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();

    for triangulation in triangulations {
        if !selection.keeps(triangulation.side) {
            continue;
        }
        if triangulation.vertices.len() != triangulation.boundary.len() {
            return Err(hypertri::Error::InvalidInput {
                reason: "region triangulation vertex and boundary lengths differ",
            });
        }
        if triangulation.triangles.len() % 3 != 0 {
            return Err(hypertri::Error::InvalidInput {
                reason: "region triangulation index buffer is not triangular",
            });
        }

        let base = vertices.len();
        vertices.extend(triangulation.boundary.iter().cloned().map(|source| {
            let point = boundary_node_point(&source).clone();
            ExactOutputVertex { point, source }
        }));

        for tri in triangulation.triangles.chunks_exact(3) {
            if tri
                .iter()
                .any(|&index| index >= triangulation.boundary.len())
            {
                return Err(hypertri::Error::InvalidInput {
                    reason: "region triangulation references a missing boundary vertex",
                });
            }
            triangles.push(ExactOutputTriangle {
                vertices: [base + tri[0], base + tri[1], base + tri[2]],
                source_side: triangulation.side,
                source_face: triangulation.face,
            });
        }
    }

    let plan = ExactBooleanAssemblyPlan {
        vertices,
        triangles,
    };
    plan.validate()?;
    Ok(plan)
}

fn classify_region_against_face_plane(
    region_side: MeshSide,
    region_face: usize,
    boundary: &[FaceSplitBoundaryNode],
    plane_side: MeshSide,
    plane_mesh: &ExactMesh,
    plane_face: usize,
) -> FaceRegionPlaneClassification {
    let tri = plane_mesh.triangles()[plane_face].0;
    let a = plane_mesh.vertices()[tri[0]].to_hyperlimit_point();
    let b = plane_mesh.vertices()[tri[1]].to_hyperlimit_point();
    let c = plane_mesh.vertices()[tri[2]].to_hyperlimit_point();
    let mut predicates = Vec::with_capacity(boundary.len());
    let mut node_sides = Vec::with_capacity(boundary.len());

    for node in boundary {
        let report = orient3d_report(&a, &b, &c, boundary_node_point(node));
        predicates.push(PredicateUse::from_certificate(report.certificate));
        node_sides.push(report.value().map(PlaneSide::from));
    }

    let relation = relation_from_sides(&node_sides);
    FaceRegionPlaneClassification {
        region_side,
        region_face,
        plane_side,
        plane_face,
        relation,
        node_sides,
        predicates,
    }
}

#[cfg(feature = "exact-triangulation")]
fn choose_region_projection(mesh: &ExactMesh, face: usize) -> hypertri::Result<CoplanarProjection> {
    let triangle = mesh.triangles()[face].0;
    let a = mesh.vertices()[triangle[0]].to_hyperlimit_point();
    let b = mesh.vertices()[triangle[1]].to_hyperlimit_point();
    let c = mesh.vertices()[triangle[2]].to_hyperlimit_point();
    for projection in [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        let [pa, pb, pc] = [
            project_for_predicate(&a, projection),
            project_for_predicate(&b, projection),
            project_for_predicate(&c, projection),
        ];
        if matches!(
            orient2d_report(&pa, &pb, &pc).value(),
            Some(Sign::Negative | Sign::Positive)
        ) {
            return Ok(projection);
        }
    }
    Err(hypertri::Error::PredicateUndecided {
        predicate: "face_region_projection",
    })
}

#[cfg(feature = "exact-triangulation")]
fn project_for_predicate(point: &Point3, projection: CoplanarProjection) -> PredicatePoint2 {
    match projection {
        CoplanarProjection::Xy => PredicatePoint2::new(point.x.clone(), point.y.clone()),
        CoplanarProjection::Xz => PredicatePoint2::new(point.x.clone(), point.z.clone()),
        CoplanarProjection::Yz => PredicatePoint2::new(point.y.clone(), point.z.clone()),
    }
}

#[cfg(feature = "exact-triangulation")]
fn project_for_hypertri(point: &Point3, projection: CoplanarProjection) -> hypertri::ExactPoint {
    match projection {
        CoplanarProjection::Xy => hypertri::ExactPoint::new(point.x.clone(), point.y.clone()),
        CoplanarProjection::Xz => hypertri::ExactPoint::new(point.x.clone(), point.z.clone()),
        CoplanarProjection::Yz => hypertri::ExactPoint::new(point.y.clone(), point.z.clone()),
    }
}

fn relation_from_sides(sides: &[Option<PlaneSide>]) -> FaceRegionPlaneRelation {
    if sides.iter().any(Option::is_none) {
        return FaceRegionPlaneRelation::Unknown;
    }
    let above = sides
        .iter()
        .filter(|side| side == &&Some(PlaneSide::Above))
        .count();
    let below = sides
        .iter()
        .filter(|side| side == &&Some(PlaneSide::Below))
        .count();
    let on = sides
        .iter()
        .filter(|side| side == &&Some(PlaneSide::On))
        .count();

    match (above, below, on) {
        (0, 0, _) => FaceRegionPlaneRelation::Coplanar,
        (_, 0, 0) => FaceRegionPlaneRelation::StrictlyAbove,
        (0, _, 0) => FaceRegionPlaneRelation::StrictlyBelow,
        _ => FaceRegionPlaneRelation::Straddling,
    }
}

fn boundary_node_point(node: &FaceSplitBoundaryNode) -> &Point3 {
    match node {
        FaceSplitBoundaryNode::OriginalVertex { point, .. }
        | FaceSplitBoundaryNode::GraphVertex { point, .. } => point,
    }
}
