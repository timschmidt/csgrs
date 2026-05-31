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
use std::{cmp::Ordering, collections::BTreeMap};

use hyperlimit::{PlaneSide, Point3, orient3d_report};
#[cfg(feature = "exact-triangulation")]
use hyperlimit::{Point2 as PredicatePoint2, Sign, compare_reals, orient2d_report, project_point3};

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

/// Error returned when a retained region/plane classification is incoherent.
///
/// The classification stores the per-boundary-node plane-side facts used to
/// derive a coarser relation. Consumers such as future winding policy should
/// be able to audit that derivation directly, rather than trusting a summary
/// enum. This follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): combinatorial decisions must remain
/// tied to certified predicate facts and explicit unknowns.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FaceRegionPlaneValidationError {
    /// A region cannot be classified against a plane from the same mesh side.
    SameRegionAndPlaneSide {
        /// Side that owns the split region.
        region_side: MeshSide,
        /// Side that owns the retained plane.
        plane_side: MeshSide,
    },
    /// A region/plane classification cannot be derived from an empty boundary.
    EmptyNodeSides,
    /// The retained predicate count does not match the retained node-side
    /// count.
    PredicateCountMismatch {
        /// Number of node-side facts.
        expected: usize,
        /// Number of retained predicate certificates.
        actual: usize,
    },
    /// The coarse relation does not match the retained node-side facts.
    RelationMismatch {
        /// Relation derived from retained node-side facts.
        expected: FaceRegionPlaneRelation,
        /// Relation stored in the artifact.
        actual: FaceRegionPlaneRelation,
    },
    /// Recomputing the region/plane classification from source meshes did not
    /// reproduce the retained artifact.
    SourceReplayMismatch,
}

impl FaceRegionPlaneClassification {
    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }

    /// Return whether the classification is ready for winding policy.
    ///
    /// A retained region/plane side fact is only consumable by inside/outside
    /// policy when every predicate was proof-producing and the derived
    /// relation is decided. Keeping this as a named predicate mirrors Yap's
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997): topology stages consume certified combinatorial facts, while
    /// undecided relations remain explicit refinement state.
    pub fn is_decided_and_proof_producing(&self) -> bool {
        self.all_proof_producing() && !matches!(self.relation, FaceRegionPlaneRelation::Unknown)
    }

    /// Validate the coarse region/plane relation against retained node sides.
    ///
    /// This check is deliberately local: split-region topology and source-face
    /// incidence are validated by [`ExactFaceRegionPlan`],
    /// while this method verifies that the predicate-derived side vector still
    /// justifies the stored relation. It also checks that the retained plane
    /// came from the opposite mesh side, because winding and inside/outside
    /// policies consume these facts as cross-mesh evidence. Keeping that
    /// provenance executable follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997): a combinatorial
    /// handoff should retain the exact predicate context it depends on.
    pub fn validate(&self) -> Result<(), FaceRegionPlaneValidationError> {
        if self.region_side == self.plane_side {
            return Err(FaceRegionPlaneValidationError::SameRegionAndPlaneSide {
                region_side: self.region_side,
                plane_side: self.plane_side,
            });
        }
        if self.node_sides.is_empty() {
            return Err(FaceRegionPlaneValidationError::EmptyNodeSides);
        }
        if self.predicates.len() != self.node_sides.len() {
            return Err(FaceRegionPlaneValidationError::PredicateCountMismatch {
                expected: self.node_sides.len(),
                actual: self.predicates.len(),
            });
        }
        let expected = relation_from_sides(&self.node_sides);
        if self.relation != expected {
            return Err(FaceRegionPlaneValidationError::RelationMismatch {
                expected,
                actual: self.relation,
            });
        }
        Ok(())
    }

    /// Recompute this region/plane classification from the source meshes.
    ///
    /// Local validation proves that the retained node-side vector justifies the
    /// coarse relation. This source replay is stronger: it rebuilds the exact
    /// intersection graph, derives the face-region plan, reclassifies regions
    /// against opposite planes, and requires this artifact to appear in that
    /// recomputed set. Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), treats these predicate facts as
    /// computation history, so a future winding policy must not consume a
    /// copied or relabeled region/plane record.
    #[cfg(feature = "exact-triangulation")]
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), FaceRegionPlaneValidationError> {
        self.validate()?;
        let region_plan = replay_region_plan(left, right)
            .map_err(|_| FaceRegionPlaneValidationError::SourceReplayMismatch)?;
        let replay =
            checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)
                .map_err(|_| FaceRegionPlaneValidationError::SourceReplayMismatch)?;
        if replay.iter().any(|classification| classification == self) {
            Ok(())
        } else {
            Err(FaceRegionPlaneValidationError::SourceReplayMismatch)
        }
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
        if !self.triangles.len().is_multiple_of(3) {
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

    /// Recompute this exact region triangulation from the source meshes.
    ///
    /// The local audit checks projection and triangle-index invariants. This
    /// replay rebuilds the exact graph and region loops from the operands,
    /// reruns the feature-gated exact `hypertri` handoff, and requires this
    /// retained triangulation to match one recomputed artifact. That keeps
    /// split-region triangulation aligned with Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997): triangulated
    /// combinatorics remain tied to the exact source faces and graph vertices
    /// that produced them.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> hypertri::Result<()> {
        self.validate()?;
        let region_plan =
            replay_region_plan(left, right).map_err(|_| hypertri::Error::InvalidInput {
                reason: "region triangulation source replay could not rebuild region plan",
            })?;
        let replay = checked_triangulate_face_regions_with_earcut(&region_plan, left, right)
            .map_err(|_| hypertri::Error::InvalidInput {
                reason: "region triangulation source replay could not triangulate region plan",
            })?;
        if replay.iter().any(|triangulation| triangulation == self) {
            Ok(())
        } else {
            Err(hypertri::Error::InvalidInput {
                reason: "region triangulation source replay mismatch",
            })
        }
    }
}

#[cfg(feature = "exact-triangulation")]
fn replay_region_plan(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactFaceRegionPlan, super::error::MeshError> {
    let graph = super::graph::build_intersection_graph(left, right)?;
    graph
        .validate_against_meshes(left, right)
        .map_err(|error| {
            super::error::MeshError::one(super::error::MeshDiagnostic::new(
                super::error::Severity::Error,
                super::error::DiagnosticKind::UnsupportedExactOperation,
                format!("exact region source replay failed: {error:?}"),
            ))
        })?;
    let geometry = graph.face_split_geometry_plan(left, right)?;
    Ok(geometry.region_plan(left, right))
}

/// Region selection policy for exact output assembly.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactRegionSelection {
    /// Drop regions from both meshes.
    KeepNone,
    /// Keep regions originating from both meshes.
    KeepAll,
    /// Keep only regions originating from the left mesh.
    KeepLeft,
    /// Keep only regions originating from the right mesh.
    KeepRight,
}

/// Per-region retention decision for exact boolean assembly.
///
/// Named volumetric booleans sometimes need to keep a split source-face region
/// with its original orientation and sometimes with reversed orientation. The
/// difference operation is the canonical case: portions of the right operand
/// that are inside the left operand become inner boundary with reversed normal.
/// Keeping this as explicit assembly state follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): boolean
/// semantics are certified combinatorial choices, not post-hoc triangle-soup
/// rewrites.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactRegionRetention {
    /// Drop this triangulated split region.
    Drop,
    /// Keep this region preserving source-face orientation.
    Keep,
    /// Keep this region with source-face orientation reversed.
    KeepReversed,
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
    /// Whether the output preserves or reverses source-face orientation.
    pub orientation: ExactOutputTriangleOrientation,
}

/// Orientation relation between an assembled output triangle and its source
/// face.
///
/// The value is validated by exact projected orientation predicates during
/// source-incidence replay. It exists so named booleans can represent
/// orientation-changing semantics, especially right-hand shell reversal for
/// exact difference, without losing the source-face provenance required by
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactOutputTriangleOrientation {
    /// The output triangle has the same projected orientation as its source
    /// face.
    PreserveSource,
    /// The output triangle has the opposite projected orientation from its
    /// source face.
    ReverseSource,
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
        let should_keep =
            move |triangulation: &FaceRegionTriangulation| selection.keeps(triangulation.side);
        assemble_region_triangulations_with_retention(triangulations, None, &mut |triangulation| {
            if should_keep(triangulation) {
                ExactRegionRetention::Keep
            } else {
                ExactRegionRetention::Drop
            }
        })
    }

    /// Assemble exact output triangles with source-face orientation replay.
    ///
    /// `hypertri` correctly triangulates the projected region, but the index
    /// order it returns is a polygon-local convention. Boolean output topology
    /// needs a stronger contract: each kept triangle must preserve or reverse
    /// its original source-face orientation according to the operation policy.
    /// This source-aware entry point compares exact projected orientation
    /// predicates and flips individual emitted triangles as needed. That keeps
    /// the materialization boundary in Yap's exact-geometric-computation
    /// model: representation changes are allowed only when the predicate facts
    /// needed to justify their combinatorics are retained and replayable. See
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997).
    pub fn from_region_triangulations_with_sources(
        triangulations: &[FaceRegionTriangulation],
        selection: ExactRegionSelection,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> hypertri::Result<Self> {
        let should_keep =
            move |triangulation: &FaceRegionTriangulation| selection.keeps(triangulation.side);
        assemble_region_triangulations_with_retention(
            triangulations,
            Some((left, right)),
            &mut |triangulation| {
                if should_keep(triangulation) {
                    ExactRegionRetention::Keep
                } else {
                    ExactRegionRetention::Drop
                }
            },
        )
    }

    /// Assemble exact output triangles from feature-gated region
    /// triangulations with an arbitrary retention predicate.
    ///
    /// The same split-region triangulation can be reused under alternate
    /// inside/outside semantics without replaying the narrow phase. This
    /// follows Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7.1-2 (1997): split geometry is an exact intermediate
    /// artifact, while semantic policy stays explicit at the assembly boundary.
    pub fn from_region_triangulations_with_selection(
        triangulations: &[FaceRegionTriangulation],
        mut should_keep: impl FnMut(&FaceRegionTriangulation) -> bool,
    ) -> hypertri::Result<Self> {
        assemble_region_triangulations_with_retention(triangulations, None, &mut |triangulation| {
            if should_keep(triangulation) {
                ExactRegionRetention::Keep
            } else {
                ExactRegionRetention::Drop
            }
        })
    }

    /// Assemble exact output triangles from feature-gated region
    /// triangulations with explicit per-region orientation policy.
    ///
    /// This is the assembly hook used by winding-backed named booleans. The
    /// classifier decides whether each exact split region is inside or outside
    /// the opposite closed mesh; this method then records that decision as
    /// kept, dropped, or source-reversed output topology. The split geometry
    /// and semantic retention policy remain separate auditable artifacts, as
    /// required by Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub fn from_region_triangulations_with_retention(
        triangulations: &[FaceRegionTriangulation],
        mut retain: impl FnMut(&FaceRegionTriangulation) -> ExactRegionRetention,
    ) -> hypertri::Result<Self> {
        assemble_region_triangulations_with_retention(triangulations, None, &mut retain)
    }

    /// Assemble exact output triangles with explicit retention and source
    /// orientation replay.
    ///
    /// This is the named-boolean materialization hook: winding classification
    /// decides whether a split region is kept, dropped, or reversed, and this
    /// method uses exact source-face orientation predicates to make the emitted
    /// triangle order match that decision. The predicate replay follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997), by treating output orientation as certified topology rather
    /// than a convention inherited blindly from a triangulation index buffer.
    pub fn from_region_triangulations_with_retention_and_sources(
        triangulations: &[FaceRegionTriangulation],
        left: &ExactMesh,
        right: &ExactMesh,
        mut retain: impl FnMut(&FaceRegionTriangulation) -> ExactRegionRetention,
    ) -> hypertri::Result<Self> {
        assemble_region_triangulations_with_retention(
            triangulations,
            Some((left, right)),
            &mut retain,
        )
    }

    /// Assemble exact output triangles with per-cell retention policy.
    ///
    /// Constrained planar-cell extraction can emit several independently
    /// classified triangles for one source face. A face-wide keep/drop decision
    /// would collapse those exact cells back into an approximation, so this
    /// entry point exposes the local triangulation triangle to the caller's
    /// winding policy. Orientation replay is still source-aware and exact, in
    /// the Yap sense that every combinatorial output decision remains tied to
    /// predicate-certified source geometry. See Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn from_region_triangulations_with_triangle_retention_and_sources(
        triangulations: &[FaceRegionTriangulation],
        left: &ExactMesh,
        right: &ExactMesh,
        mut retain: impl FnMut(&FaceRegionTriangulation, [usize; 3]) -> ExactRegionRetention,
    ) -> hypertri::Result<Self> {
        assemble_region_triangulations_with_triangle_retention(
            triangulations,
            Some((left, right)),
            &mut retain,
        )
    }

    /// Validate output topology, geometry, and retained source-point provenance.
    ///
    /// Each output vertex stores both a materialized exact point and the
    /// boundary node that produced it. Validating their exact equality keeps
    /// the construction provenance attached to the topology that consumes it,
    /// rejecting unreferenced vertices keeps the handoff compact to the exact
    /// topology materialization consumes, and validating triangle point
    /// distinctness catches zero-area assembly artifacts before they reach mesh
    /// construction. These are exact
    /// `hyperlimit::compare_reals` checks, not tolerance comparisons,
    /// following Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7.1-2 (1997): geometric decisions carry certified object
    /// facts instead of trusting duplicated coordinates.
    pub fn validate(&self) -> hypertri::Result<()> {
        for vertex in &self.vertices {
            validate_output_vertex_source(vertex)?;
        }
        let mut referenced_vertices = vec![false; self.vertices.len()];
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
            referenced_vertices[a] = true;
            referenced_vertices[b] = true;
            referenced_vertices[c] = true;
            validate_output_triangle_distinct_points(self, triangle)?;
        }
        if referenced_vertices.iter().any(|&referenced| !referenced) {
            return Err(hypertri::Error::InvalidInput {
                reason: "assembled output retained an unreferenced vertex",
            });
        }
        Ok(())
    }

    /// Validate and materialize the assembly plan as an [`ExactMesh`].
    ///
    /// This is the exact replacement boundary for the legacy boolean mutation
    /// path: constructed output triangles are converted back into hypermesh
    /// exact vertices and triangle handles only after local assembly invariants
    /// have been audited. The resulting mesh is then checked by the same
    /// manifold and geometric validators used for caller-supplied exact meshes.
    ///
    /// The local validation step follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997): constructed
    /// combinatorics must carry certified source and incidence facts before a
    /// topology consumer treats them as mesh state.
    pub fn to_exact_mesh(
        &self,
        policy: ValidationPolicy,
    ) -> Result<ExactMesh, super::error::MeshError> {
        self.validate().map_err(assembly_validation_error)?;
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
        self.validate().map_err(assembly_validation_error)?;
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

    /// Split exact-equal assembly vertices whose retained triangle fans are
    /// disconnected.
    ///
    /// The default assembly welds exact-equal coordinates globally. That is the
    /// right handoff for ordinary edge-adjacent output, but winding-materialized
    /// booleans can produce regularized differences where two closed sheets touch
    /// at one exact coordinate while remaining distinct topological vertices. A
    /// global weld turns that point contact into a non-manifold vertex even
    /// though each incident fan is separately valid. This method replays only the
    /// retained triangle adjacency around each welded vertex and duplicates the
    /// vertex for every disconnected fan component.
    ///
    /// This is a topological decision made from exact combinatorial evidence:
    /// triangles are joined in a fan only when they share a retained output edge.
    /// No coordinate perturbation is introduced, and cloned vertices retain the
    /// same exact point and source witness. That preserves the object/predicate
    /// boundary advocated by Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), while allowing the mesh topology to
    /// represent point contacts as separate vertices.
    pub fn split_disconnected_vertex_fans(&mut self) -> hypertri::Result<usize> {
        let original_vertex_count = self.vertices.len();
        let mut cloned_vertices = 0;
        for vertex in 0..original_vertex_count {
            let incident = incident_triangles(self, vertex);
            if incident.len() <= 1 {
                continue;
            }
            let components = vertex_fan_components(self, vertex, &incident);
            if components.len() <= 1 {
                continue;
            }
            for component in components.into_iter().skip(1) {
                let clone = self.vertices[vertex].clone();
                let clone_index = self.vertices.len();
                self.vertices.push(clone);
                for triangle in component {
                    replace_triangle_vertex(&mut self.triangles[triangle], vertex, clone_index);
                }
                cloned_vertices += 1;
            }
        }
        self.validate()?;
        Ok(cloned_vertices)
    }

    /// Recompute this assembly plan from source meshes and a region selection.
    ///
    /// Local validation and source-face incidence prove that this plan is
    /// internally coherent and lies on claimed source faces. This replay also
    /// rebuilds the intersection graph, region plan, exact `hypertri`
    /// triangulations, and selected-region assembly for the supplied policy,
    /// then requires the retained plan to match the recomputed one. That makes
    /// the selected-region policy part of the exact artifact boundary, in the
    /// sense of Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997): downstream topology cannot
    /// consume a locally valid assembly that was relabeled from a different
    /// source pair or region-retention rule.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        selection: ExactRegionSelection,
    ) -> hypertri::Result<()> {
        self.validate()?;
        self.validate_source_face_incidence(left, right)?;
        let region_plan =
            replay_region_plan(left, right).map_err(|_| hypertri::Error::InvalidInput {
                reason: "assembly source replay could not rebuild region plan",
            })?;
        let triangulations =
            checked_triangulate_face_regions_with_earcut(&region_plan, left, right).map_err(
                |_| hypertri::Error::InvalidInput {
                    reason: "assembly source replay could not triangulate region plan",
                },
            )?;
        let replay = ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
            &triangulations,
            selection,
            left,
            right,
        )?;
        if self == &replay {
            Ok(())
        } else {
            Err(hypertri::Error::InvalidInput {
                reason: "assembly source replay mismatch",
            })
        }
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
        validate_output_triangle_source_orientation(assembly, triangle, mesh, source_triangle)?;
    }
    Ok(())
}

#[cfg(feature = "exact-triangulation")]
/// Validate that an output triangle preserves its retained source-face
/// orientation.
///
/// Source-face incidence only proves that emitted vertices remain on the source
/// plane. For boundary topology, the triangle also has to keep the same
/// projected orientation as the source face it claims. The projection is chosen
/// by a certified nonzero `hyperlimit::orient2d_report`, then both source and
/// output signs are compared exactly. This follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): a topology
/// handoff must retain the predicate facts that make orientation meaningful,
/// rather than trusting vertex order as an unchecked label.
fn validate_output_triangle_source_orientation(
    assembly: &ExactBooleanAssemblyPlan,
    triangle: &ExactOutputTriangle,
    mesh: &ExactMesh,
    source_triangle: [usize; 3],
) -> hypertri::Result<()> {
    let projection = choose_region_projection(mesh, triangle.source_face)?;
    let source_points = [
        mesh.vertices()[source_triangle[0]].to_hyperlimit_point(),
        mesh.vertices()[source_triangle[1]].to_hyperlimit_point(),
        mesh.vertices()[source_triangle[2]].to_hyperlimit_point(),
    ];
    let source_sign = orient2d_report(
        &project_for_predicate(&source_points[0], projection),
        &project_for_predicate(&source_points[1], projection),
        &project_for_predicate(&source_points[2], projection),
    )
    .value()
    .ok_or(hypertri::Error::PredicateUndecided {
        predicate: "assembly_source_orientation",
    })?;
    let output_sign = orient2d_report(
        &project_for_predicate(&assembly.vertices[triangle.vertices[0]].point, projection),
        &project_for_predicate(&assembly.vertices[triangle.vertices[1]].point, projection),
        &project_for_predicate(&assembly.vertices[triangle.vertices[2]].point, projection),
    )
    .value()
    .ok_or(hypertri::Error::PredicateUndecided {
        predicate: "assembly_output_orientation",
    })?;
    if output_sign == Sign::Zero {
        return Err(hypertri::Error::InvalidInput {
            reason: "assembled output triangle has zero projected source orientation",
        });
    }
    match triangle.orientation {
        ExactOutputTriangleOrientation::PreserveSource if source_sign != output_sign => {
            return Err(hypertri::Error::InvalidInput {
                reason: "assembled output triangle reverses its source face orientation",
            });
        }
        ExactOutputTriangleOrientation::ReverseSource if source_sign == output_sign => {
            return Err(hypertri::Error::InvalidInput {
                reason: "assembled output triangle failed to reverse source face orientation",
            });
        }
        ExactOutputTriangleOrientation::PreserveSource
        | ExactOutputTriangleOrientation::ReverseSource => {}
    }
    Ok(())
}

#[cfg(feature = "exact-triangulation")]
fn assembly_validation_error(error: hypertri::Error) -> super::error::MeshError {
    super::error::MeshError::one(super::error::MeshDiagnostic::new(
        super::error::Severity::Error,
        super::error::DiagnosticKind::IndexOutOfBounds,
        format!("exact boolean assembly validation failed: {error}"),
    ))
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
    graph
        .validate_against_meshes(left, right)
        .map_err(|error| {
            super::error::MeshError::one(super::error::MeshDiagnostic::new(
                super::error::Severity::Error,
                super::error::DiagnosticKind::UnsupportedExactOperation,
                format!("exact selected-region graph/source replay failed: {error:?}"),
            ))
        })?;
    if graph.has_unknowns() {
        return Err(super::error::MeshError::one(
            super::error::MeshDiagnostic::new(
                super::error::Severity::Error,
                super::error::DiagnosticKind::UnsupportedExactOperation,
                "exact selected-region graph contains unresolved predicate events",
            ),
        ));
    }
    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
    // `build_selected_region_mesh` is a mesh-only convenience wrapper around
    // the richer report API. It must still cross the same certified
    // winding-handoff boundary as `boolean_selected_regions`: every retained
    // region/plane fact is audited and proof-producing before triangulation
    // may materialize topology. This follows Yap, "Towards Exact Geometric
    // Computation," Comput. Geom. 7.1-2 (1997): a shorter API cannot erase
    // undecided predicate state just because it returns fewer report fields.
    if region_classifications
        .iter()
        .any(|classification| !classification.is_decided_and_proof_producing())
    {
        return Err(super::error::MeshError::one(
            super::error::MeshDiagnostic::new(
                super::error::Severity::Error,
                super::error::DiagnosticKind::UnsupportedExactOperation,
                "exact selected-region classification retained undecided predicate evidence",
            ),
        ));
    }
    let triangulations = checked_triangulate_face_regions_with_earcut(&region_plan, left, right)
        .map_err(|error| {
            super::error::MeshError::one(super::error::MeshDiagnostic::new(
                super::error::Severity::Error,
                super::error::DiagnosticKind::DegenerateTriangle,
                format!("exact region triangulation failed: {error}"),
            ))
        })?;
    let assembly = ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
        &triangulations,
        selection,
        left,
        right,
    )
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
fn assemble_region_triangulations_with_retention(
    triangulations: &[FaceRegionTriangulation],
    sources: Option<(&ExactMesh, &ExactMesh)>,
    retain: &mut impl FnMut(&FaceRegionTriangulation) -> ExactRegionRetention,
) -> hypertri::Result<ExactBooleanAssemblyPlan> {
    assemble_region_triangulations_with_triangle_retention(
        triangulations,
        sources,
        &mut |triangulation, _triangle| retain(triangulation),
    )
}

#[cfg(feature = "exact-triangulation")]
fn assemble_region_triangulations_with_triangle_retention(
    triangulations: &[FaceRegionTriangulation],
    sources: Option<(&ExactMesh, &ExactMesh)>,
    retain: &mut impl FnMut(&FaceRegionTriangulation, [usize; 3]) -> ExactRegionRetention,
) -> hypertri::Result<ExactBooleanAssemblyPlan> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();

    for triangulation in triangulations {
        triangulation.validate()?;
        let mut remap = vec![None; triangulation.boundary.len()];

        for tri in triangulation.triangles.chunks_exact(3) {
            let region_triangle = [tri[0], tri[1], tri[2]];
            let retention = retain(triangulation, region_triangle);
            if retention == ExactRegionRetention::Drop {
                continue;
            }
            let orientation = match retention {
                ExactRegionRetention::Drop => unreachable!("drop handled above"),
                ExactRegionRetention::Keep => ExactOutputTriangleOrientation::PreserveSource,
                ExactRegionRetention::KeepReversed => ExactOutputTriangleOrientation::ReverseSource,
            };
            let mut output_vertices = [
                remap_region_vertex(triangulation, &mut remap, &mut vertices, tri[0])?,
                remap_region_vertex(triangulation, &mut remap, &mut vertices, tri[1])?,
                remap_region_vertex(triangulation, &mut remap, &mut vertices, tri[2])?,
            ];
            if let Some((left, right)) = sources {
                orient_output_triangle_for_source(
                    triangulation,
                    &vertices,
                    &mut output_vertices,
                    orientation,
                    left,
                    right,
                )?;
            } else if retention == ExactRegionRetention::KeepReversed {
                output_vertices.swap(1, 2);
            }
            triangles.push(ExactOutputTriangle {
                vertices: output_vertices,
                source_side: triangulation.side,
                source_face: triangulation.face,
                orientation,
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

#[cfg(feature = "exact-triangulation")]
/// Orient one emitted output triangle against its retained source face.
///
/// Ear clipping works in the projected polygon's coordinate convention, while
/// boolean output topology is a 3D source-face contract. The exact
/// `orient2d_report` checks here replay both signs in the same certified
/// projection and swap the emitted triangle when its raw order disagrees with
/// the requested source orientation. This follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): an
/// algorithmic representation change cannot silently become a topological
/// decision unless exact predicates certify it.
fn orient_output_triangle_for_source(
    triangulation: &FaceRegionTriangulation,
    vertices: &[ExactOutputVertex],
    output_vertices: &mut [usize; 3],
    orientation: ExactOutputTriangleOrientation,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<()> {
    let source_mesh = match triangulation.side {
        MeshSide::Left => left,
        MeshSide::Right => right,
    };
    let source_sign = source_face_projected_orientation(
        source_mesh,
        triangulation.face,
        triangulation.projection,
    )?;
    let output_sign = output_triangle_projected_orientation(
        vertices,
        *output_vertices,
        triangulation.projection,
    )?;
    if output_sign == Sign::Zero {
        return Err(hypertri::Error::InvalidInput {
            reason: "assembled output triangle has zero projected source orientation",
        });
    }

    let preserves_source = output_sign == source_sign;
    let should_preserve = orientation == ExactOutputTriangleOrientation::PreserveSource;
    if preserves_source != should_preserve {
        output_vertices.swap(1, 2);
    }
    Ok(())
}

#[cfg(feature = "exact-triangulation")]
fn source_face_projected_orientation(
    mesh: &ExactMesh,
    face: usize,
    projection: CoplanarProjection,
) -> hypertri::Result<Sign> {
    let Some(triangle) = mesh.triangles().get(face).map(|triangle| triangle.0) else {
        return Err(hypertri::Error::InvalidInput {
            reason: "region triangulation references a missing source face",
        });
    };
    let points = [
        mesh.vertices()[triangle[0]].to_hyperlimit_point(),
        mesh.vertices()[triangle[1]].to_hyperlimit_point(),
        mesh.vertices()[triangle[2]].to_hyperlimit_point(),
    ];
    let sign = orient2d_report(
        &project_for_predicate(&points[0], projection),
        &project_for_predicate(&points[1], projection),
        &project_for_predicate(&points[2], projection),
    )
    .value()
    .ok_or(hypertri::Error::PredicateUndecided {
        predicate: "assembly_source_orientation",
    })?;
    if sign == Sign::Zero {
        return Err(hypertri::Error::InvalidInput {
            reason: "source face has zero projected orientation",
        });
    }
    Ok(sign)
}

#[cfg(feature = "exact-triangulation")]
fn output_triangle_projected_orientation(
    vertices: &[ExactOutputVertex],
    output_vertices: [usize; 3],
    projection: CoplanarProjection,
) -> hypertri::Result<Sign> {
    let [a, b, c] = output_vertices;
    let Some(a) = vertices.get(a) else {
        return Err(hypertri::Error::InvalidInput {
            reason: "assembled output triangle references a missing vertex",
        });
    };
    let Some(b) = vertices.get(b) else {
        return Err(hypertri::Error::InvalidInput {
            reason: "assembled output triangle references a missing vertex",
        });
    };
    let Some(c) = vertices.get(c) else {
        return Err(hypertri::Error::InvalidInput {
            reason: "assembled output triangle references a missing vertex",
        });
    };
    orient2d_report(
        &project_for_predicate(&a.point, projection),
        &project_for_predicate(&b.point, projection),
        &project_for_predicate(&c.point, projection),
    )
    .value()
    .ok_or(hypertri::Error::PredicateUndecided {
        predicate: "assembly_output_orientation",
    })
}

#[cfg(feature = "exact-triangulation")]
/// Map a region-local boundary vertex into the compact output assembly.
///
/// Region boundaries may carry split nodes that exact earcut does not consume in
/// any emitted triangle after degeneracy handling, and adjacent source regions
/// may describe the same exact 3D point through different boundary-node
/// provenance. The assembly therefore welds exact-equal points globally while
/// retaining one source witness for the vertex. This is a topological
/// operation certified by exact equality predicates, following Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997):
/// downstream topology receives shared object identity only when exact
/// predicate facts justify the merge.
fn remap_region_vertex(
    triangulation: &FaceRegionTriangulation,
    remap: &mut [Option<usize>],
    vertices: &mut Vec<ExactOutputVertex>,
    region_vertex: usize,
) -> hypertri::Result<usize> {
    if let Some(vertex) = remap[region_vertex] {
        return Ok(vertex);
    }
    let source = triangulation.boundary[region_vertex].clone();
    let point = boundary_node_point(&source).clone();
    for (index, vertex) in vertices.iter().enumerate() {
        match points_equal(&vertex.point, &point) {
            Some(true) => {
                remap[region_vertex] = Some(index);
                return Ok(index);
            }
            Some(false) => {}
            None => {
                return Err(hypertri::Error::PredicateUndecided {
                    predicate: "assembly_vertex_weld_equality",
                });
            }
        }
    }
    vertices.push(ExactOutputVertex { point, source });
    let index = vertices.len() - 1;
    remap[region_vertex] = Some(index);
    Ok(index)
}

#[cfg(feature = "exact-triangulation")]
fn incident_triangles(assembly: &ExactBooleanAssemblyPlan, vertex: usize) -> Vec<usize> {
    assembly
        .triangles
        .iter()
        .enumerate()
        .filter_map(|(triangle_index, triangle)| {
            triangle
                .vertices
                .contains(&vertex)
                .then_some(triangle_index)
        })
        .collect()
}

#[cfg(feature = "exact-triangulation")]
fn vertex_fan_components(
    assembly: &ExactBooleanAssemblyPlan,
    vertex: usize,
    incident: &[usize],
) -> Vec<Vec<usize>> {
    let mut fan = DisjointTriangleFan::new(incident.len());
    let mut edge_uses = BTreeMap::<usize, Vec<usize>>::new();
    for (local_triangle, &triangle_index) in incident.iter().enumerate() {
        for &corner in &assembly.triangles[triangle_index].vertices {
            if corner != vertex {
                edge_uses.entry(corner).or_default().push(local_triangle);
            }
        }
    }
    for uses in edge_uses.values() {
        if let [left, right] = uses.as_slice() {
            fan.union(*left, *right);
        }
    }

    let mut components = BTreeMap::<usize, Vec<usize>>::new();
    for (local_triangle, &triangle_index) in incident.iter().enumerate() {
        components
            .entry(fan.find(local_triangle))
            .or_default()
            .push(triangle_index);
    }
    components.into_values().collect()
}

#[cfg(feature = "exact-triangulation")]
fn replace_triangle_vertex(triangle: &mut ExactOutputTriangle, old: usize, new: usize) {
    for vertex in &mut triangle.vertices {
        if *vertex == old {
            *vertex = new;
        }
    }
}

#[cfg(feature = "exact-triangulation")]
struct DisjointTriangleFan {
    parent: Vec<usize>,
}

#[cfg(feature = "exact-triangulation")]
impl DisjointTriangleFan {
    fn new(len: usize) -> Self {
        Self {
            parent: (0..len).collect(),
        }
    }

    fn find(&mut self, index: usize) -> usize {
        let parent = self.parent[index];
        if parent == index {
            index
        } else {
            let root = self.find(parent);
            self.parent[index] = root;
            root
        }
    }

    fn union(&mut self, left: usize, right: usize) {
        let left_root = self.find(left);
        let right_root = self.find(right);
        if left_root != right_root {
            self.parent[right_root] = left_root;
        }
    }
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
pub(crate) fn choose_region_projection(
    mesh: &ExactMesh,
    face: usize,
) -> hypertri::Result<CoplanarProjection> {
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
pub(crate) fn project_for_predicate(
    point: &Point3,
    projection: CoplanarProjection,
) -> PredicatePoint2 {
    project_point3(point, projection)
}

#[cfg(feature = "exact-triangulation")]
pub(crate) fn project_for_hypertri(
    point: &Point3,
    projection: CoplanarProjection,
) -> hypertri::ExactPoint {
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

pub(crate) fn boundary_node_point(node: &FaceSplitBoundaryNode) -> &Point3 {
    match node {
        FaceSplitBoundaryNode::OriginalVertex { point, .. }
        | FaceSplitBoundaryNode::GraphVertex { point, .. }
        | FaceSplitBoundaryNode::FaceInterior { point } => point,
    }
}

#[cfg(feature = "exact-triangulation")]
fn points_equal(left: &Point3, right: &Point3) -> Option<bool> {
    let x = compare_reals(&left.x, &right.x).value()?;
    let y = compare_reals(&left.y, &right.y).value()?;
    let z = compare_reals(&left.z, &right.z).value()?;
    Some(x == Ordering::Equal && y == Ordering::Equal && z == Ordering::Equal)
}
