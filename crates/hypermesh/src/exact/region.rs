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

#[cfg(feature = "exact-triangulation")]
use std::cmp::Ordering;

use hyperlimit::{PlaneSide, Point3, orient3d_report};
#[cfg(feature = "exact-triangulation")]
use hyperlimit::{Point2 as PredicatePoint2, Sign, compare_reals, orient2d_report};

#[cfg(feature = "exact-triangulation")]
use super::coplanar::CoplanarProjection;
#[cfg(feature = "exact-triangulation")]
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
#[cfg(feature = "exact-triangulation")]
use super::graph::SplitPlanDiagnosticKind;
#[cfg(feature = "exact-triangulation")]
use super::graph::SplitPlanValidationReport;
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

/// Validate and classify split regions against every opposite face plane.
///
/// This is the checked handoff for future winding/inside-outside policy:
/// region loops must satisfy exact structural and source-face incidence
/// validation before plane-side facts are produced. The staging follows Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), by ensuring combinatorial consumers receive certified geometric
/// objects rather than unchecked boundary loops.
#[cfg(feature = "exact-triangulation")]
pub fn checked_classify_face_regions_against_opposite_planes(
    regions: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Vec<FaceRegionPlaneClassification>, MeshError> {
    let report = regions.validate(left, right);
    if report.is_valid() {
        Ok(classify_face_regions_against_opposite_planes(
            regions, left, right,
        ))
    } else {
        Err(region_plan_report_to_mesh_error(report))
    }
}

#[cfg(feature = "exact-triangulation")]
fn region_plan_report_to_mesh_error(report: SplitPlanValidationReport) -> MeshError {
    MeshError::new(
        report
            .diagnostics
            .into_iter()
            .map(|diagnostic| {
                let kind = match diagnostic.kind {
                    SplitPlanDiagnosticKind::UnknownBoundaryIncidence => {
                        DiagnosticKind::UnsupportedExactOperation
                    }
                    SplitPlanDiagnosticKind::BoundaryNodeOffFacePlane
                    | SplitPlanDiagnosticKind::EmptyOrShortRegionBoundary
                    | SplitPlanDiagnosticKind::DuplicateConsecutiveRegionNode => {
                        DiagnosticKind::DegenerateTriangle
                    }
                    _ => DiagnosticKind::UnsupportedExactOperation,
                };
                let mut mesh = MeshDiagnostic::new(Severity::Error, kind, diagnostic.message);
                if let Some(face) = diagnostic.face {
                    mesh = mesh.with_face(face);
                }
                if let Some(edge) = diagnostic.edge {
                    mesh = mesh.with_edge(edge);
                }
                mesh
            })
            .collect(),
    )
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

#[cfg(feature = "exact-triangulation")]
impl FaceRegionTriangulation {
    /// Validate projected triangulation output before assembly consumes it.
    ///
    /// `hypertri` returns a compact index buffer, while exact boolean
    /// assembly needs the stronger contract that every projected vertex still
    /// matches its retained 3D boundary source and that every output triangle
    /// is a certified non-degenerate projected triangle. This keeps the
    /// triangulation handoff in Yap's exact-geometric-computation model:
    /// algorithms may transform representation, but each combinatorial result
    /// must carry enough certified facts to be audited before downstream
    /// topology uses it. See Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub fn validate(&self) -> hypertri::Result<()> {
        if self.vertices.len() != self.boundary.len() {
            return Err(hypertri::Error::InvalidInput {
                reason: "region triangulation vertex and boundary lengths differ",
            });
        }
        if self.triangles.len() % 3 != 0 {
            return Err(hypertri::Error::InvalidInput {
                reason: "region triangulation index buffer is not triangular",
            });
        }

        for (vertex, source) in self.vertices.iter().zip(&self.boundary) {
            validate_projected_boundary_vertex(vertex, source, self.projection)?;
        }

        for tri in self.triangles.chunks_exact(3) {
            if tri.iter().any(|&index| index >= self.vertices.len()) {
                return Err(hypertri::Error::InvalidInput {
                    reason: "region triangulation references a missing boundary vertex",
                });
            }
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
                return Err(hypertri::Error::InvalidInput {
                    reason: "region triangulation has repeated vertex handles",
                });
            }
            validate_projected_triangle(
                &self.vertices[tri[0]],
                &self.vertices[tri[1]],
                &self.vertices[tri[2]],
            )?;
        }

        Ok(())
    }
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

    /// Validate output topology, geometry, and retained source-point provenance.
    ///
    /// Each output vertex stores both a materialized exact point and the
    /// boundary node that produced it. Validating their exact equality keeps
    /// the construction provenance attached to the topology that consumes it,
    /// and validating triangle point distinctness catches zero-area assembly
    /// artifacts before they reach mesh construction. These are exact
    /// `hyperlimit::compare_reals` checks, not tolerance comparisons,
    /// following Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7.1-2 (1997): geometric decisions carry certified object
    /// facts instead of trusting duplicated coordinates.
    pub fn validate(&self) -> hypertri::Result<()> {
        for vertex in &self.vertices {
            validate_output_vertex_source(vertex)?;
        }
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
            validate_output_triangle_distinct_points(self, triangle)?;
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

    /// Validate and materialize the assembly plan as an [`ExactMesh`].
    ///
    /// Direct callers sometimes hold an assembly plan before choosing an
    /// output policy. This checked entry point preserves the same handoff used
    /// by the selected-region pipeline: index/provenance invariants are
    /// checked before exact mesh validation consumes the output triangles.
    /// The separation follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), by validating constructed
    /// combinatorics before committing them to mesh topology.
    pub fn checked_to_exact_mesh(
        &self,
        policy: ValidationPolicy,
    ) -> Result<ExactMesh, super::error::MeshError> {
        self.validate().map_err(|error| {
            super::error::MeshError::one(super::error::MeshDiagnostic::new(
                super::error::Severity::Error,
                super::error::DiagnosticKind::IndexOutOfBounds,
                format!("exact boolean assembly validation failed: {error}"),
            ))
        })?;
        self.to_exact_mesh(policy)
    }

    /// Validate assembly invariants, source-face incidence, and materialize.
    ///
    /// This is the preferred output boundary for selected-region booleans. It
    /// combines local assembly validation with exact source-face incidence
    /// replay before exact mesh construction receives any topology. That keeps
    /// the final handoff aligned with Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997): every
    /// combinatorial consumer receives certified geometric facts, not a
    /// coordinate-only approximation of earlier construction history.
    pub fn checked_to_exact_mesh_with_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ValidationPolicy,
    ) -> Result<ExactMesh, super::error::MeshError> {
        self.validate().map_err(|error| {
            super::error::MeshError::one(super::error::MeshDiagnostic::new(
                super::error::Severity::Error,
                super::error::DiagnosticKind::IndexOutOfBounds,
                format!("exact boolean assembly validation failed: {error}"),
            ))
        })?;
        self.validate_source_face_incidence(left, right)
            .map_err(|error| {
                super::error::MeshError::one(super::error::MeshDiagnostic::new(
                    super::error::Severity::Error,
                    super::error::DiagnosticKind::DegenerateTriangle,
                    format!("exact boolean assembly source incidence failed: {error}"),
                ))
            })?;
        self.to_exact_mesh(policy)
    }

    /// Validate output triangles against their retained source face planes.
    ///
    /// Output triangles carry `source_side` and `source_face` so later boolean
    /// stages can audit where each triangle came from. This check replays that
    /// incidence with exact `hyperlimit::orient3d_report` predicates before
    /// materialization consumes the plan. It follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997): topology
    /// handoffs should retain and revalidate the geometric certificates they
    /// depend on.
    pub fn validate_source_face_incidence(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> hypertri::Result<()> {
        validate_assembly_source_face_incidence(self, left, right)
    }
}

#[cfg(feature = "exact-triangulation")]
fn validate_output_vertex_source(vertex: &ExactOutputVertex) -> hypertri::Result<()> {
    match points_equal(&vertex.point, boundary_node_point(&vertex.source)) {
        Some(true) => Ok(()),
        Some(false) => Err(hypertri::Error::InvalidInput {
            reason: "assembled output vertex does not match its retained source boundary node",
        }),
        None => Err(hypertri::Error::PredicateUndecided {
            predicate: "output_vertex_source_equality",
        }),
    }
}

#[cfg(feature = "exact-triangulation")]
fn validate_output_triangle_distinct_points(
    assembly: &ExactBooleanAssemblyPlan,
    triangle: &ExactOutputTriangle,
) -> hypertri::Result<()> {
    let [a, b, c] = triangle.vertices;
    for (left, right) in [(a, b), (b, c), (c, a)] {
        match points_equal(
            &assembly.vertices[left].point,
            &assembly.vertices[right].point,
        ) {
            Some(false) => {}
            Some(true) => {
                return Err(hypertri::Error::InvalidInput {
                    reason: "assembled output triangle has repeated exact vertex points",
                });
            }
            None => {
                return Err(hypertri::Error::PredicateUndecided {
                    predicate: "output_triangle_vertex_point_equality",
                });
            }
        }
    }
    Ok(())
}

#[cfg(feature = "exact-triangulation")]
fn validate_assembly_source_face_incidence(
    assembly: &ExactBooleanAssemblyPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<()> {
    for triangle in &assembly.triangles {
        let mesh = match triangle.source_side {
            MeshSide::Left => left,
            MeshSide::Right => right,
        };
        let Some(source_triangle) = mesh
            .triangles()
            .get(triangle.source_face)
            .map(|triangle| triangle.0)
        else {
            return Err(hypertri::Error::InvalidInput {
                reason: "assembled output triangle references a missing source face",
            });
        };
        let a = mesh.vertices()[source_triangle[0]].to_hyperlimit_point();
        let b = mesh.vertices()[source_triangle[1]].to_hyperlimit_point();
        let c = mesh.vertices()[source_triangle[2]].to_hyperlimit_point();
        for &vertex in &triangle.vertices {
            let Some(output_vertex) = assembly.vertices.get(vertex) else {
                return Err(hypertri::Error::InvalidInput {
                    reason: "assembled output triangle references a missing vertex",
                });
            };
            match orient3d_report(&a, &b, &c, &output_vertex.point).value() {
                Some(hyperlimit::Sign::Zero) => {}
                Some(hyperlimit::Sign::Negative | hyperlimit::Sign::Positive) => {
                    return Err(hypertri::Error::InvalidInput {
                        reason: "assembled output triangle vertex is off its source face plane",
                    });
                }
                None => {
                    return Err(hypertri::Error::PredicateUndecided {
                        predicate: "assembly_source_face_incidence",
                    });
                }
            }
        }
    }
    Ok(())
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

/// Validate and triangulate split face-region loops with `hypertri` exact earcut.
///
/// This is the checked handoff from exact graph geometry into triangulation:
/// region loops must first satisfy the structural and face-incidence
/// invariants enforced by [`ExactFaceRegionPlan::validate`]. Only then are
/// they projected and passed to `hypertri`. The staged contract follows Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): certified geometric facts are validated before an algorithm turns
/// them into downstream combinatorics.
#[cfg(feature = "exact-triangulation")]
pub fn checked_triangulate_face_regions_with_earcut(
    regions: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<Vec<FaceRegionTriangulation>> {
    let report = regions.validate(left, right);
    if !report.is_valid() {
        if report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::UnknownBoundaryIncidence)
        {
            return Err(hypertri::Error::PredicateUndecided {
                predicate: "face_region_boundary_incidence",
            });
        }
        return Err(hypertri::Error::InvalidInput {
            reason: "face region plan failed exact validation",
        });
    }
    let triangulations = triangulate_face_regions_with_earcut(regions, left, right)?;
    for triangulation in &triangulations {
        triangulation.validate()?;
    }
    Ok(triangulations)
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
    let triangulations = checked_triangulate_face_regions_with_earcut(&region_plan, left, right)
        .map_err(|error| {
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
    assembly.checked_to_exact_mesh_with_sources(left, right, policy)
}

#[cfg(feature = "exact-triangulation")]
fn assemble_region_triangulations(
    triangulations: &[FaceRegionTriangulation],
    selection: ExactRegionSelection,
) -> hypertri::Result<ExactBooleanAssemblyPlan> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();

    for triangulation in triangulations {
        triangulation.validate()?;
        if !selection.keeps(triangulation.side) {
            continue;
        }

        let base = vertices.len();
        vertices.extend(triangulation.boundary.iter().cloned().map(|source| {
            let point = boundary_node_point(&source).clone();
            ExactOutputVertex { point, source }
        }));

        for tri in triangulation.triangles.chunks_exact(3) {
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

#[cfg(feature = "exact-triangulation")]
fn validate_projected_boundary_vertex(
    vertex: &hypertri::ExactPoint,
    source: &FaceSplitBoundaryNode,
    projection: CoplanarProjection,
) -> hypertri::Result<()> {
    let expected = project_for_hypertri(boundary_node_point(source), projection);
    match exact_points_equal(vertex, &expected) {
        Some(true) => Ok(()),
        Some(false) => Err(hypertri::Error::InvalidInput {
            reason: "region triangulation vertex does not match retained boundary source",
        }),
        None => Err(hypertri::Error::PredicateUndecided {
            predicate: "region_triangulation_vertex_source_equality",
        }),
    }
}

#[cfg(feature = "exact-triangulation")]
fn validate_projected_triangle(
    a: &hypertri::ExactPoint,
    b: &hypertri::ExactPoint,
    c: &hypertri::ExactPoint,
) -> hypertri::Result<()> {
    for (left, right) in [(a, b), (b, c), (c, a)] {
        match exact_points_equal(left, right) {
            Some(false) => {}
            Some(true) => {
                return Err(hypertri::Error::InvalidInput {
                    reason: "region triangulation triangle has repeated exact projected points",
                });
            }
            None => {
                return Err(hypertri::Error::PredicateUndecided {
                    predicate: "region_triangulation_projected_point_equality",
                });
            }
        }
    }

    let pa = PredicatePoint2::new(a.x.clone(), a.y.clone());
    let pb = PredicatePoint2::new(b.x.clone(), b.y.clone());
    let pc = PredicatePoint2::new(c.x.clone(), c.y.clone());
    match orient2d_report(&pa, &pb, &pc).value() {
        Some(Sign::Negative | Sign::Positive) => Ok(()),
        Some(Sign::Zero) => Err(hypertri::Error::InvalidInput {
            reason: "region triangulation triangle is exactly collinear",
        }),
        None => Err(hypertri::Error::PredicateUndecided {
            predicate: "region_triangulation_projected_area",
        }),
    }
}

#[cfg(feature = "exact-triangulation")]
fn exact_points_equal(left: &hypertri::ExactPoint, right: &hypertri::ExactPoint) -> Option<bool> {
    let x = compare_reals(&left.x, &right.x).value()?;
    let y = compare_reals(&left.y, &right.y).value()?;
    Some(x == Ordering::Equal && y == Ordering::Equal)
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

#[cfg(feature = "exact-triangulation")]
fn points_equal(left: &Point3, right: &Point3) -> Option<bool> {
    let x = compare_reals(&left.x, &right.x).value()?;
    let y = compare_reals(&left.y, &right.y).value()?;
    let z = compare_reals(&left.z, &right.z).value()?;
    Some(x == Ordering::Equal && y == Ordering::Equal && z == Ordering::Equal)
}
