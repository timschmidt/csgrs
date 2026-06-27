//! Exact split-region classification.
//!
//! This module consumes the pre-triangulation region loops produced by the
//! intersection graph and classifies them against opposite mesh face planes.
//! The stage is intentionally still pre-boolean-output: it prepares certified
//! side facts for later winding/inside-outside classification without using a
//! primitive-float representative point. Topology updates consume certified
//! predicate facts, and undecided cases remain explicit.
//!

use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
};

use hyperlimit::{
    PlaneSide, Point3, TriangleLocation, classify_point_triangle, orient3d_report, point_on_segment,
};
use hyperlimit::{Point2 as PredicatePoint2, Sign, compare_reals, orient2d_report, project_point3};

use super::super::ExactMesh;
use super::super::Triangle;
use super::super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
use super::super::graph::SplitPlanBlockerKind;
use super::super::graph::{ExactFaceRegionPlan, FaceSplitBoundaryNode, MeshSide};
use super::super::validation::ExactMeshValidationPolicy;
use hyperlimit::CoplanarProjection;
use hyperlimit::PredicateUse;
use hyperlimit::SourceProvenance;

/// Exact relation between a split region boundary and an opposite face plane.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum FaceRegionPlaneRelation {
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
pub(crate) struct FaceRegionPlaneClassification {
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
/// derive a coarser relation. Exact region selection audits that derivation
/// directly instead of trusting a summary detached from predicate facts and
/// explicit unknowns.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum FaceRegionPlaneValidationError {
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
    #[cfg(test)]
    SourceReplayMismatch,
}

impl FaceRegionPlaneClassification {
    /// Return whether every retained predicate route was proof-producing.
    pub(crate) fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }

    /// Return whether the classification is ready for winding policy.
    ///
    /// A retained region/plane side fact is only consumable by inside/outside
    /// policy when every predicate was proof-producing and the derived
    /// undecided relations remain explicit refinement state.
    pub(crate) fn is_decided_and_proof_producing(&self) -> bool {
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
    /// handoff should retain the exact predicate context it depends on.
    pub(crate) fn validate(&self) -> Result<(), FaceRegionPlaneValidationError> {
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
    #[cfg(test)]
    pub(crate) fn validate_against_sources(
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

#[cfg(test)]
impl ExactFaceRegionPlan {
    /// Validate and classify split regions against every opposite face plane.
    pub(crate) fn classify_against_opposite_face_planes(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Vec<FaceRegionPlaneClassification>, ExactMeshError> {
        checked_classify_face_regions_against_opposite_planes(self, left, right)
    }

    /// Validate and triangulate split face-region loops with exact earcut.
    pub(crate) fn triangulate_with_earcut(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> hypertri::Result<Vec<FaceRegionTriangulation>> {
        checked_triangulate_face_regions_with_earcut(self, left, right)
    }
}

/// Classify every split region against every opposite mesh face plane.
///
/// This is a certified input to later winding/side policy. It deliberately
/// does not collapse a full mesh-side decision into one boolean: a region can
/// be coplanar with one face, straddle another face plane, and remain unknown
/// against a symbolic face until refinement or a policy decision is available.
pub(crate) fn classify_face_regions_against_opposite_planes(
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

pub(crate) fn checked_classify_face_regions_against_opposite_planes(
    regions: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Vec<FaceRegionPlaneClassification>, ExactMeshError> {
    let report = regions.validate(left, right);
    if report.is_valid() {
        Ok(classify_face_regions_against_opposite_planes(
            regions, left, right,
        ))
    } else {
        Err(ExactMeshError::new(
            report
                .blockers
                .into_iter()
                .map(|blocker| {
                    let kind = match blocker.kind {
                        SplitPlanBlockerKind::UnknownBoundaryIncidence => {
                            ExactMeshBlockerKind::UndecidablePredicate
                        }
                        SplitPlanBlockerKind::BoundaryNodeSourceVertexOutOfRange => {
                            ExactMeshBlockerKind::IndexOutOfBounds
                        }
                        SplitPlanBlockerKind::BoundaryNodeSourceVertexNotOnTriangle
                        | SplitPlanBlockerKind::BoundaryNodeSourcePointMismatch => {
                            ExactMeshBlockerKind::StaleFactReplay
                        }
                        SplitPlanBlockerKind::BoundaryNodeOffFacePlane
                        | SplitPlanBlockerKind::EmptyOrShortRegionBoundary
                        | SplitPlanBlockerKind::DuplicateConsecutiveRegionNode
                        | SplitPlanBlockerKind::BoundaryChainEdgeNotOnTriangle => {
                            ExactMeshBlockerKind::ExactConstructionFailure
                        }
                        _ => ExactMeshBlockerKind::ExactConstructionFailure,
                    };
                    let mut mesh = ExactMeshBlocker::new(kind, blocker.message);
                    if let Some(face) = blocker.face {
                        mesh = mesh.with_face(face);
                    }
                    if let Some(edge) = blocker.edge {
                        mesh = mesh.with_edge(edge);
                    }
                    mesh
                })
                .collect(),
        ))
    }
}

/// Exact earcut triangulation of one split face region.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct FaceRegionTriangulation {
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

impl FaceRegionTriangulation {
    /// Validate projected triangulation output before assembly consumes it.
    ///
    /// `hypertri` returns a compact index buffer, while exact boolean
    /// assembly needs the stronger contract that every projected vertex still
    /// matches its retained 3D boundary source and that every output triangle
    /// is a certified non-degenerate projected triangle. This keeps the
    /// algorithms may transform representation, but each combinatorial result
    /// must carry enough certified facts to be audited before downstream
    pub(crate) fn validate(&self) -> hypertri::Result<()> {
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

        let mut retained_cells = BTreeSet::<[usize; 3]>::new();
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
            let mut cell = [tri[0], tri[1], tri[2]];
            cell.sort_unstable();
            if !retained_cells.insert(cell) {
                return Err(hypertri::Error::InvalidInput {
                    reason: "region triangulation retained a duplicate triangle cell",
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
    #[cfg(test)]
    pub(crate) fn validate_against_sources(
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

fn replay_region_plan(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactFaceRegionPlan, super::super::error::ExactMeshError> {
    let graph = super::super::graph::build_validated_intersection_graph(left, right)?;
    let geometry = graph.face_split_geometry_plan(left, right)?;
    Ok(geometry.region_plan(left, right))
}

/// Region selection policy for exact output assembly.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(clippy::enum_variant_names)]
pub(crate) enum ExactRegionSelection {
    /// Drop regions from both meshes.
    KeepNone,
    /// Keep regions originating from both meshes.
    KeepAll,
    /// Keep only regions originating from the left mesh.
    KeepLeft,
}

/// Per-region retention decision for exact boolean assembly.
///
/// Named volumetric booleans sometimes need to keep a split source-face region
/// with its original orientation and sometimes with reversed orientation. The
/// difference operation is the canonical case: portions of the right operand
/// that are inside the left operand become inner boundary with reversed normal.
/// semantics are certified combinatorial choices, not post-hoc triangle-soup
/// rewrites.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactRegionRetention {
    /// Drop this triangulated split region.
    Drop,
    /// Keep this region preserving source-face orientation.
    Keep,
    /// Keep this region with source-face orientation reversed.
    KeepReversed,
}

impl ExactRegionSelection {
    /// Return whether this selection retains regions from `side`.
    pub const fn keeps(self, side: MeshSide) -> bool {
        matches!(
            (self, side),
            (Self::KeepAll, _) | (Self::KeepLeft, MeshSide::Left)
        )
    }
}

/// One exact output vertex in an assembled region mesh.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactOutputVertex {
    /// Exact 3D point.
    pub point: Point3,
    /// Boundary node that produced this vertex.
    pub source: FaceSplitBoundaryNode,
}

/// One exact output triangle with source-region provenance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ExactOutputTriangle {
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
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactOutputTriangleOrientation {
    /// The output triangle has the same projected orientation as its source
    /// face.
    PreserveSource,
    /// The output triangle has the opposite projected orientation from its
    /// source face.
    ReverseSource,
}

/// Non-mutating exact output mesh assembly plan.
///
/// Exact region loops have been classified and triangulated, and this type
/// records the exact 3D triangles ready for mesh materialization. It does not
/// hide operation semantics in tolerances; callers pass an explicit
/// region-selection policy, and undecided winding/inside-outside policy remains
/// outside this assembly.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactBooleanAssemblyPlan {
    /// Exact output vertices.
    pub vertices: Vec<ExactOutputVertex>,
    /// Exact output triangles.
    pub triangles: Vec<ExactOutputTriangle>,
}

impl ExactBooleanAssemblyPlan {
    /// Assemble exact output triangles with source-face orientation replay.
    ///
    /// `hypertri` correctly triangulates the projected region, but the index
    /// order it returns is a polygon-local convention. Boolean output topology
    /// needs a stronger contract: each kept triangle must preserve or reverse
    /// its original source-face orientation according to the operation policy.
    /// This source-aware entry point compares exact projected orientation
    /// predicates and flips individual emitted triangles as needed. That keeps
    /// model: representation changes are allowed only when the predicate facts
    /// needed to justify their combinatorics are retained and replayable. See
    pub(crate) fn from_region_triangulations_with_sources(
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

    /// Assemble exact output triangles with per-cell retention policy.
    ///
    /// Constrained planar-cell extraction can emit several independently
    /// classified triangles for one source face. A face-wide keep/drop decision
    /// would collapse those exact cells back into an approximation, so this
    /// entry point exposes the local triangulation triangle to the caller's
    /// winding policy. Orientation replay is still source-aware and exact, in
    pub(crate) fn from_region_triangulations_with_triangle_retention_and_sources(
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
    /// facts instead of trusting duplicated coordinates.
    pub(crate) fn validate(&self) -> hypertri::Result<()> {
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
    /// Constructed output triangles are converted back into hypermesh exact
    /// vertices and triangle handles only after local assembly invariants have
    /// been audited. The resulting mesh is then checked by the same manifold
    /// and geometric validators used for caller-supplied exact meshes.
    ///
    /// combinatorics must carry certified source and incidence facts before a
    /// topology consumer treats them as mesh state.
    pub(crate) fn to_exact_mesh(
        &self,
        policy: ExactMeshValidationPolicy,
    ) -> Result<ExactMesh, super::super::error::ExactMeshError> {
        self.validate().map_err(assembly_validation_error)?;
        let vertices = self
            .vertices
            .iter()
            .map(|vertex| {
                Point3::new(
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

    /// Validate assembly invariants, source-face incidence, and materialize.
    ///
    /// This is the preferred output boundary for selected-region booleans. It
    /// combines local assembly validation with exact source-face incidence
    /// replay before exact mesh construction receives any topology. That keeps
    /// combinatorial consumer receives certified geometric facts, not a
    /// coordinate-only approximation of earlier construction history.
    pub(crate) fn checked_to_exact_mesh_with_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactMeshValidationPolicy,
    ) -> Result<ExactMesh, super::super::error::ExactMeshError> {
        self.validate().map_err(assembly_validation_error)?;
        self.validate_source_face_incidence(left, right)
            .map_err(|error| {
                super::super::error::ExactMeshError::one(
                    super::super::error::ExactMeshBlocker::new(
                        super::super::error::ExactMeshBlockerKind::DegenerateTriangle,
                        format!("exact boolean assembly source incidence failed: {error}"),
                    ),
                )
            })?;
        self.to_exact_mesh(policy)
    }

    /// Canonicalize exact assembly topology before mesh materialization.
    ///
    /// This is the shared normal form used by Boolean materializers that retain
    /// an assembly artifact. It refines T-junction edges at already-retained
    /// exact vertices, removes duplicate selected triangle cells, splits
    /// disconnected exact-equal vertex fans, and orients paired edges
    /// consistently. All changes happen inside the assembly, so the output mesh
    /// can still replay exactly from retained source-face provenance.
    pub(crate) fn canonicalize_for_mesh_with_sources(
        &mut self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> hypertri::Result<()> {
        self.refine_edges_at_existing_vertices(left, right)?;
        self.remove_duplicate_triangle_vertex_sets()?;
        self.split_disconnected_vertex_fans()?;
        self.orient_paired_edge_uses()?;
        self.validate_source_face_incidence(left, right)?;
        Ok(())
    }

    /// Validate output triangles against their retained source face planes.
    ///
    /// Output triangles carry `source_side` and `source_face` so later boolean
    /// stages can audit where each triangle came from. This check replays that
    /// incidence with exact `hyperlimit::orient3d_report` predicates before
    /// handoffs should retain and revalidate the geometric certificates they
    /// depend on.
    pub(crate) fn validate_source_face_incidence(
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
    /// right handoff for ordinary edge-adjacent output, but arrangement-materialized
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
    /// represent point contacts as separate vertices.
    pub(crate) fn split_disconnected_vertex_fans(&mut self) -> hypertri::Result<usize> {
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

    /// Orient retained triangle components so paired edges have opposite use.
    ///
    /// Winding selection can retain exact cells from different source faces
    /// whose source-local orientation conventions do not yet agree across the
    /// selected volume boundary. This pass uses only retained topology: every
    /// two-triangle edge imposes a parity constraint, and flipping one side of
    /// a component also toggles the triangle's recorded source-orientation
    /// evidence. Boundary and non-manifold edges are left as blockers for
    /// later mesh validation; contradictory parity means the selected complex
    /// is not orientable as a triangle manifold.
    pub(crate) fn orient_paired_edge_uses(&mut self) -> hypertri::Result<usize> {
        let edge_uses = assembly_edge_uses(self);
        let mut adjacency = vec![Vec::<TriangleOrientationConstraint>::new(); self.triangles.len()];
        for uses in edge_uses.values() {
            let [left, right] = uses.as_slice() else {
                continue;
            };
            let same_direction = left.forward_with_key == right.forward_with_key;
            adjacency[left.triangle].push(TriangleOrientationConstraint {
                triangle: right.triangle,
                flip_relative_to_current: same_direction,
            });
            adjacency[right.triangle].push(TriangleOrientationConstraint {
                triangle: left.triangle,
                flip_relative_to_current: same_direction,
            });
        }

        let mut flips = vec![None; self.triangles.len()];
        for start in 0..self.triangles.len() {
            if flips[start].is_some() {
                continue;
            }
            flips[start] = Some(false);
            let mut stack = vec![start];
            while let Some(triangle) = stack.pop() {
                let current_flip = flips[triangle].ok_or(hypertri::Error::InvalidInput {
                    reason: "triangle orientation traversal lost assigned parity",
                })?;
                for constraint in &adjacency[triangle] {
                    let required = current_flip ^ constraint.flip_relative_to_current;
                    match flips[constraint.triangle] {
                        Some(existing) if existing != required => {
                            return Err(hypertri::Error::InvalidInput {
                                reason: "selected triangle component has contradictory edge orientation",
                            });
                        }
                        Some(_) => {}
                        None => {
                            flips[constraint.triangle] = Some(required);
                            stack.push(constraint.triangle);
                        }
                    }
                }
            }
        }

        let mut flipped = 0;
        for (triangle, flip) in self.triangles.iter_mut().zip(flips) {
            if flip == Some(true) {
                triangle.vertices.swap(1, 2);
                triangle.orientation = toggled_output_orientation(triangle.orientation);
                flipped += 1;
            }
        }
        self.validate()?;
        Ok(flipped)
    }

    /// Split retained triangle edges at exact vertices already present in the
    /// assembly.
    ///
    /// Volumetric cell extraction can legitimately retain one face-cell
    /// triangle whose edge is collinear with several smaller cells emitted
    /// from an adjacent source face. A triangle mesh cannot represent that
    /// T-junction as a closed two-manifold, so the larger triangle is refined
    /// by replaying source-face incidence and exact projected point-on-segment
    /// predicates. No new coordinates are introduced.
    pub(crate) fn refine_edges_at_existing_vertices(
        &mut self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> hypertri::Result<usize> {
        let mut splits = 0;
        loop {
            let Some(split) = find_existing_vertex_edge_split(self, left, right)? else {
                self.validate()?;
                return Ok(splits);
            };
            let prior_triangle_count = self.triangles.len();
            apply_existing_vertex_edge_split(self, split, left, right)?;
            if self.triangles.len() <= prior_triangle_count {
                return Err(hypertri::Error::InvalidInput {
                    reason: "assembly edge refinement did not make progress",
                });
            }
            splits += 1;
        }
    }

    /// Remove duplicate exact triangle handles after cell refinement.
    ///
    /// When both operands contribute the same boundary cell, selection keeps
    /// enough evidence to prove coincidence before this normal-form pass drops
    /// the duplicate topological face. The key is the sorted output vertex set;
    /// exact-equal vertices have already been welded by assembly.
    pub(crate) fn remove_duplicate_triangle_vertex_sets(&mut self) -> hypertri::Result<usize> {
        let original_len = self.triangles.len();
        let mut seen = std::collections::BTreeSet::<[usize; 3]>::new();
        self.triangles.retain(|triangle| {
            let mut key = triangle.vertices;
            key.sort_unstable();
            seen.insert(key)
        });
        self.validate()?;
        Ok(original_len - self.triangles.len())
    }

    /// Recompute this assembly plan from source meshes and a region selection.
    ///
    /// Local validation and source-face incidence prove that this plan is
    /// internally coherent and lies on claimed source faces. This replay also
    /// rebuilds the intersection graph, region plan, exact `hypertri`
    /// triangulations, and selected-region assembly for the supplied policy,
    /// then requires the retained plan to match the recomputed one. That makes
    /// the selected-region policy part of the exact artifact boundary, in the
    /// consume a locally valid assembly that was relabeled from a different
    /// source pair or region-retention rule.
    #[cfg(test)]
    pub(crate) fn validate_against_sources(
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
        let mut replay = ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
            &triangulations,
            selection,
            left,
            right,
        )?;
        replay.canonicalize_for_mesh_with_sources(left, right)?;
        if self == &replay {
            Ok(())
        } else {
            Err(hypertri::Error::InvalidInput {
                reason: "assembly source replay mismatch",
            })
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct AssemblyEdgeUse {
    triangle: usize,
    forward_with_key: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct TriangleOrientationConstraint {
    triangle: usize,
    flip_relative_to_current: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ExistingVertexEdgeSplit {
    triangle: usize,
    edge: usize,
    vertex: usize,
}

fn assembly_edge_uses(
    assembly: &ExactBooleanAssemblyPlan,
) -> BTreeMap<[usize; 2], Vec<AssemblyEdgeUse>> {
    let mut edge_uses = BTreeMap::<[usize; 2], Vec<AssemblyEdgeUse>>::new();
    for (triangle_index, triangle) in assembly.triangles.iter().enumerate() {
        for edge in [
            [triangle.vertices[0], triangle.vertices[1]],
            [triangle.vertices[1], triangle.vertices[2]],
            [triangle.vertices[2], triangle.vertices[0]],
        ] {
            let mut key = edge;
            key.sort_unstable();
            edge_uses.entry(key).or_default().push(AssemblyEdgeUse {
                triangle: triangle_index,
                forward_with_key: edge == key,
            });
        }
    }
    edge_uses
}

fn find_existing_vertex_edge_split(
    assembly: &ExactBooleanAssemblyPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<Option<ExistingVertexEdgeSplit>> {
    for (triangle_index, triangle) in assembly.triangles.iter().enumerate() {
        let projection = assembly_triangle_projection(triangle, left, right)?;
        for edge in 0..3 {
            let start = triangle.vertices[edge];
            let end = triangle.vertices[(edge + 1) % 3];
            for candidate in 0..assembly.vertices.len() {
                if triangle.vertices.contains(&candidate) {
                    continue;
                }
                if !assembly_vertex_lies_on_source_face(assembly, triangle, candidate, left, right)?
                {
                    continue;
                }
                if assembly_vertex_lies_strictly_on_projected_edge(
                    assembly, candidate, start, end, projection,
                )? {
                    return Ok(Some(ExistingVertexEdgeSplit {
                        triangle: triangle_index,
                        edge,
                        vertex: candidate,
                    }));
                }
            }
        }
    }
    Ok(None)
}

fn apply_existing_vertex_edge_split(
    assembly: &mut ExactBooleanAssemblyPlan,
    split: ExistingVertexEdgeSplit,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<()> {
    let original = assembly.triangles[split.triangle].clone();
    let a = original.vertices[split.edge];
    let b = original.vertices[(split.edge + 1) % 3];
    let c = original.vertices[(split.edge + 2) % 3];
    let mut first_vertices = [a, split.vertex, c];
    let mut second_vertices = [split.vertex, b, c];
    let triangulation = FaceRegionTriangulation {
        side: original.source_side,
        face: original.source_face,
        projection: assembly_triangle_projection(&original, left, right)?,
        boundary: Vec::new(),
        vertices: Vec::new(),
        triangles: Vec::new(),
    };
    orient_output_triangle_for_source(
        &triangulation,
        &assembly.vertices,
        &mut first_vertices,
        original.orientation,
        left,
        right,
    )?;
    orient_output_triangle_for_source(
        &triangulation,
        &assembly.vertices,
        &mut second_vertices,
        original.orientation,
        left,
        right,
    )?;
    let first = ExactOutputTriangle {
        vertices: first_vertices,
        source_side: original.source_side,
        source_face: original.source_face,
        orientation: original.orientation,
    };
    let second = ExactOutputTriangle {
        vertices: second_vertices,
        source_side: original.source_side,
        source_face: original.source_face,
        orientation: original.orientation,
    };
    validate_output_triangle_distinct_points(assembly, &first)?;
    validate_output_triangle_distinct_points(assembly, &second)?;
    assembly
        .triangles
        .splice(split.triangle..split.triangle + 1, [first, second]);
    Ok(())
}

fn assembly_triangle_projection(
    triangle: &ExactOutputTriangle,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<CoplanarProjection> {
    let mesh = match triangle.source_side {
        MeshSide::Left => left,
        MeshSide::Right => right,
    };
    choose_region_projection(mesh, triangle.source_face)
}

fn assembly_vertex_lies_on_source_face(
    assembly: &ExactBooleanAssemblyPlan,
    triangle: &ExactOutputTriangle,
    vertex: usize,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<bool> {
    let mesh = match triangle.source_side {
        MeshSide::Left => left,
        MeshSide::Right => right,
    };
    let source_triangle = mesh.triangles()[triangle.source_face].0;
    let a = &mesh.vertices()[source_triangle[0]];
    let b = &mesh.vertices()[source_triangle[1]];
    let c = &mesh.vertices()[source_triangle[2]];
    orient3d_report(a, b, c, &assembly.vertices[vertex].point)
        .value()
        .map(|sign| sign == Sign::Zero)
        .ok_or(hypertri::Error::PredicateUndecided {
            predicate: "assembly_refinement_source_face_incidence",
        })
}

fn assembly_vertex_lies_strictly_on_projected_edge(
    assembly: &ExactBooleanAssemblyPlan,
    candidate: usize,
    start: usize,
    end: usize,
    projection: CoplanarProjection,
) -> hypertri::Result<bool> {
    let candidate_point = &assembly.vertices[candidate].point;
    let start_point = &assembly.vertices[start].point;
    let end_point = &assembly.vertices[end].point;
    if points_equal(candidate_point, start_point) == Some(true)
        || points_equal(candidate_point, end_point) == Some(true)
    {
        return Ok(false);
    }
    point_on_segment(
        &project_for_predicate(start_point, projection),
        &project_for_predicate(end_point, projection),
        &project_for_predicate(candidate_point, projection),
    )
    .value()
    .ok_or(hypertri::Error::PredicateUndecided {
        predicate: "assembly_refinement_point_on_edge",
    })
}

const fn toggled_output_orientation(
    orientation: ExactOutputTriangleOrientation,
) -> ExactOutputTriangleOrientation {
    match orientation {
        ExactOutputTriangleOrientation::PreserveSource => {
            ExactOutputTriangleOrientation::ReverseSource
        }
        ExactOutputTriangleOrientation::ReverseSource => {
            ExactOutputTriangleOrientation::PreserveSource
        }
    }
}

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

fn validate_assembly_source_face_incidence(
    assembly: &ExactBooleanAssemblyPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<()> {
    let has_graph_vertex_source = assembly
        .vertices
        .iter()
        .any(|vertex| matches!(vertex.source, FaceSplitBoundaryNode::GraphVertex { .. }));
    let replayed_region_plan = if has_graph_vertex_source {
        let region_plan =
            replay_region_plan(left, right).map_err(|_| hypertri::Error::InvalidInput {
                reason: "assembled output graph vertex source replay could not rebuild region plan",
            })?;
        Some(region_plan)
    } else {
        None
    };
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
        let a = mesh.vertices()[source_triangle[0]].clone();
        let b = mesh.vertices()[source_triangle[1]].clone();
        let c = mesh.vertices()[source_triangle[2]].clone();
        for &vertex in &triangle.vertices {
            let Some(output_vertex) = assembly.vertices.get(vertex) else {
                return Err(hypertri::Error::InvalidInput {
                    reason: "assembled output triangle references a missing vertex",
                });
            };
            validate_assembly_output_vertex_source_against_sources(
                output_vertex,
                left,
                right,
                replayed_region_plan.as_ref(),
            )?;
            validate_assembly_face_interior_source_against_face(
                &output_vertex.source,
                mesh,
                triangle.source_face,
            )?;
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

fn validate_assembly_output_vertex_source_against_sources(
    vertex: &ExactOutputVertex,
    left: &ExactMesh,
    right: &ExactMesh,
    replayed_region_plan: Option<&ExactFaceRegionPlan>,
) -> hypertri::Result<()> {
    match &vertex.source {
        FaceSplitBoundaryNode::OriginalVertex {
            vertex: source_vertex,
            point,
        } => {
            let mut saw_source_vertex = false;
            let mut saw_unknown_equality = false;
            for mesh in [left, right] {
                let Some(source_point) = mesh.vertices().get(*source_vertex) else {
                    continue;
                };
                saw_source_vertex = true;
                match points_equal(point, source_point) {
                    Some(true) => return Ok(()),
                    Some(false) => {}
                    None => saw_unknown_equality = true,
                }
            }
            if !saw_source_vertex {
                return Err(hypertri::Error::InvalidInput {
                    reason: "assembled output vertex references a missing original source vertex",
                });
            }
            if saw_unknown_equality {
                return Err(hypertri::Error::PredicateUndecided {
                    predicate: "assembly_output_vertex_source_equality",
                });
            }
            Err(hypertri::Error::InvalidInput {
                reason: "assembled output vertex original source point does not match source meshes",
            })
        }
        FaceSplitBoundaryNode::GraphVertex {
            graph_vertex,
            point,
        } => validate_assembly_graph_vertex_source_against_region_plan(
            *graph_vertex,
            point,
            replayed_region_plan,
        ),
        FaceSplitBoundaryNode::FaceInterior { .. } => Ok(()),
    }
}

fn validate_assembly_graph_vertex_source_against_region_plan(
    graph_vertex: usize,
    point: &Point3,
    replayed_region_plan: Option<&ExactFaceRegionPlan>,
) -> hypertri::Result<()> {
    let Some(replayed_region_plan) = replayed_region_plan else {
        return Err(hypertri::Error::InvalidInput {
            reason: "assembled output graph vertex source replay is missing",
        });
    };
    let mut saw_graph_vertex = false;
    let mut saw_unknown_equality = false;
    for replayed_node in replayed_region_plan
        .regions
        .iter()
        .flat_map(|region| region.boundary.iter())
    {
        let FaceSplitBoundaryNode::GraphVertex {
            graph_vertex: replayed_graph_vertex,
            point: replayed_point,
        } = replayed_node
        else {
            continue;
        };
        if *replayed_graph_vertex != graph_vertex {
            continue;
        }
        saw_graph_vertex = true;
        match points_equal(point, replayed_point) {
            Some(true) => return Ok(()),
            Some(false) => {}
            None => saw_unknown_equality = true,
        }
    }
    if !saw_graph_vertex {
        return Err(hypertri::Error::InvalidInput {
            reason: "assembled output vertex references a missing graph vertex",
        });
    }
    if saw_unknown_equality {
        return Err(hypertri::Error::PredicateUndecided {
            predicate: "assembly_output_graph_vertex_source_equality",
        });
    }
    Err(hypertri::Error::InvalidInput {
        reason: "assembled output graph vertex point does not match source replay",
    })
}

fn validate_assembly_face_interior_source_against_face(
    source: &FaceSplitBoundaryNode,
    mesh: &ExactMesh,
    source_face: usize,
) -> hypertri::Result<()> {
    let FaceSplitBoundaryNode::FaceInterior { point } = source else {
        return Ok(());
    };
    let projection = choose_region_projection(mesh, source_face)?;
    let Some(source_triangle) = mesh.triangles().get(source_face).map(|triangle| triangle.0) else {
        return Err(hypertri::Error::InvalidInput {
            reason: "assembled output triangle references a missing source face",
        });
    };
    let location = classify_point_triangle(
        &project_for_predicate(&mesh.vertices()[source_triangle[0]], projection),
        &project_for_predicate(&mesh.vertices()[source_triangle[1]], projection),
        &project_for_predicate(&mesh.vertices()[source_triangle[2]], projection),
        &project_for_predicate(point, projection),
    )
    .value()
    .ok_or(hypertri::Error::PredicateUndecided {
        predicate: "assembly_face_interior_source_containment",
    })?;
    match location {
        TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex => Ok(()),
        TriangleLocation::Outside | TriangleLocation::Degenerate => {
            Err(hypertri::Error::InvalidInput {
                reason: "assembled output face-interior source point is outside its source triangle",
            })
        }
    }
}

/// Validate that an output triangle preserves its retained source-face
/// orientation.
///
/// Source-face incidence only proves that emitted vertices remain on the source
/// plane. For boundary topology, the triangle also has to keep the same
/// projected orientation as the source face it claims. The projection is chosen
/// by a certified nonzero `hyperlimit::orient2d_report`, then both source and
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
        mesh.vertices()[source_triangle[0]].clone(),
        mesh.vertices()[source_triangle[1]].clone(),
        mesh.vertices()[source_triangle[2]].clone(),
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

fn assembly_validation_error(error: hypertri::Error) -> super::super::error::ExactMeshError {
    super::super::error::ExactMeshError::one(super::super::error::ExactMeshBlocker::new(
        super::super::error::ExactMeshBlockerKind::IndexOutOfBounds,
        format!("exact boolean assembly validation failed: {error}"),
    ))
}

/// Triangulate split face-region loops with `hypertri` exact earcut.
///
/// The projection is selected only after a certified nonzero orientation
/// predicate, matching the projection discipline used for coplanar overlap
/// classification. The earcut call follows Held, "FIST:
/// Fast Industrial-Strength Triangulation of Polygons," *Algorithmica* 30
/// (2001), with exact projected coordinates supplied by `hypertri`.
pub(crate) fn triangulate_face_regions_with_earcut(
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
/// them into downstream combinatorics.
pub(crate) fn checked_triangulate_face_regions_with_earcut(
    regions: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<Vec<FaceRegionTriangulation>> {
    let report = regions.validate(left, right);
    if !report.is_valid() {
        if report
            .blockers
            .iter()
            .any(|blocker| blocker.kind == SplitPlanBlockerKind::UnknownBoundaryIncidence)
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
            let orientation = match retain(triangulation, region_triangle) {
                ExactRegionRetention::Drop => continue,
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
            } else if orientation == ExactOutputTriangleOrientation::ReverseSource {
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

/// Orient one emitted output triangle against its retained source face.
///
/// Ear clipping works in the projected polygon's coordinate convention, while
/// boolean output topology is a 3D source-face contract. The exact
/// `orient2d_report` checks here replay both signs in the same certified
/// projection and swap the emitted triangle when its raw order disagrees with
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
        mesh.vertices()[triangle[0]].clone(),
        mesh.vertices()[triangle[1]].clone(),
        mesh.vertices()[triangle[2]].clone(),
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

/// Map a region-local boundary vertex into the compact output assembly.
///
/// Region boundaries may carry split nodes that exact earcut does not consume in
/// any emitted triangle after degeneracy handling, and adjacent source regions
/// may describe the same exact 3D point through different boundary-node
/// provenance. The assembly therefore welds exact-equal points globally while
/// retaining one source witness for the vertex. This is a topological
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

fn vertex_fan_components(
    assembly: &ExactBooleanAssemblyPlan,
    vertex: usize,
    incident: &[usize],
) -> Vec<Vec<usize>> {
    let mut fan = DisjointTriangleFan::new(incident.len());
    let mut edge_uses = BTreeMap::<usize, Vec<VertexFanEdgeUse>>::new();
    for (local_triangle, &triangle_index) in incident.iter().enumerate() {
        for edge_use in vertex_fan_edge_uses(
            local_triangle,
            vertex,
            assembly.triangles[triangle_index].vertices,
        ) {
            edge_uses.entry(edge_use.other).or_default().push(edge_use);
        }
    }
    for uses in edge_uses.values() {
        if let [left, right] = uses.as_slice() {
            // Two triangles are adjacent across a retained edge only when the
            // edge is traversed in opposite directions. Same-direction uses
            // are separate sheets that must be split before ExactMesh
            // validation, otherwise they materialize as duplicate directed
            // edges.
            if left.forward_from_vertex != right.forward_from_vertex {
                fan.union(left.local_triangle, right.local_triangle);
            }
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct VertexFanEdgeUse {
    local_triangle: usize,
    other: usize,
    forward_from_vertex: bool,
}

fn vertex_fan_edge_uses(
    local_triangle: usize,
    vertex: usize,
    triangle: [usize; 3],
) -> Vec<VertexFanEdgeUse> {
    let mut uses = Vec::with_capacity(2);
    for index in 0..3 {
        let from = triangle[index];
        let to = triangle[(index + 1) % 3];
        if from == vertex {
            uses.push(VertexFanEdgeUse {
                local_triangle,
                other: to,
                forward_from_vertex: true,
            });
        } else if to == vertex {
            uses.push(VertexFanEdgeUse {
                local_triangle,
                other: from,
                forward_from_vertex: false,
            });
        }
    }
    uses
}

fn replace_triangle_vertex(triangle: &mut ExactOutputTriangle, old: usize, new: usize) {
    for vertex in &mut triangle.vertices {
        if *vertex == old {
            *vertex = new;
        }
    }
}

struct DisjointTriangleFan {
    parent: Vec<usize>,
}

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
    let a = plane_mesh.vertices()[tri[0]].clone();
    let b = plane_mesh.vertices()[tri[1]].clone();
    let c = plane_mesh.vertices()[tri[2]].clone();
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

pub(crate) fn choose_region_projection(
    mesh: &ExactMesh,
    face: usize,
) -> hypertri::Result<CoplanarProjection> {
    let triangle = mesh.triangles()[face].0;
    let a = mesh.vertices()[triangle[0]].clone();
    let b = mesh.vertices()[triangle[1]].clone();
    let c = mesh.vertices()[triangle[2]].clone();
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

pub(crate) fn project_for_predicate(
    point: &Point3,
    projection: CoplanarProjection,
) -> PredicatePoint2 {
    project_point3(point, projection)
}

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

fn points_equal(left: &Point3, right: &Point3) -> Option<bool> {
    let x = compare_reals(&left.x, &right.x).value()?;
    let y = compare_reals(&left.y, &right.y).value()?;
    let z = compare_reals(&left.z, &right.z).value()?;
    Some(x == Ordering::Equal && y == Ordering::Equal && z == Ordering::Equal)
}

#[cfg(test)]
mod tests {
    use super::super::super::graph::build_unvalidated_intersection_graph;
    use super::*;
    use hyperreal::Real;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn original(vertex: usize, point: Point3) -> FaceSplitBoundaryNode {
        FaceSplitBoundaryNode::OriginalVertex { vertex, point }
    }

    fn face_interior(point: Point3) -> FaceSplitBoundaryNode {
        FaceSplitBoundaryNode::FaceInterior { point }
    }

    #[test]
    fn region_triangulation_rejects_duplicate_triangle_cell() {
        let boundary = vec![
            original(0, p(0, 0, 0)),
            original(1, p(1, 0, 0)),
            original(2, p(0, 1, 0)),
        ];
        let vertices = boundary
            .iter()
            .map(|node| project_for_hypertri(boundary_node_point(node), CoplanarProjection::Xy))
            .collect();
        let triangulation = FaceRegionTriangulation {
            side: MeshSide::Left,
            face: 0,
            projection: CoplanarProjection::Xy,
            boundary,
            vertices,
            triangles: vec![0, 1, 2, 2, 1, 0],
        };

        assert!(triangulation.validate().is_err());
    }

    #[test]
    fn assembly_source_incidence_rejects_stale_graph_vertex_source() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[1, -1, -1, 1, 3, 1, 1, 3, -1],
            &[0, 1, 2],
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();
        let geometry = graph.face_split_geometry_plan(&left, &right).unwrap();
        let region_plan = geometry.region_plan(&left, &right);
        let triangulations =
            checked_triangulate_face_regions_with_earcut(&region_plan, &left, &right).unwrap();

        let mut assembly = None;
        'search: for triangulation in &triangulations {
            for graph_node in 0..triangulation.boundary.len() {
                if !matches!(
                    triangulation.boundary[graph_node],
                    FaceSplitBoundaryNode::GraphVertex { .. }
                ) {
                    continue;
                }
                for first in 0..triangulation.boundary.len() {
                    for second in 0..triangulation.boundary.len() {
                        if graph_node == first || graph_node == second || first == second {
                            continue;
                        }
                        for orientation in [
                            ExactOutputTriangleOrientation::PreserveSource,
                            ExactOutputTriangleOrientation::ReverseSource,
                        ] {
                            let candidate = ExactBooleanAssemblyPlan {
                                vertices: [graph_node, first, second]
                                    .into_iter()
                                    .map(|index| {
                                        let source = triangulation.boundary[index].clone();
                                        ExactOutputVertex {
                                            point: boundary_node_point(&source).clone(),
                                            source,
                                        }
                                    })
                                    .collect(),
                                triangles: vec![ExactOutputTriangle {
                                    vertices: [0, 1, 2],
                                    source_side: triangulation.side,
                                    source_face: triangulation.face,
                                    orientation,
                                }],
                            };
                            if candidate.validate().is_ok()
                                && candidate
                                    .validate_source_face_incidence(&left, &right)
                                    .is_ok()
                            {
                                assembly = Some(candidate);
                                break 'search;
                            }
                        }
                    }
                }
            }
        }
        let mut assembly = assembly.expect(
            "crossing split-region triangulations should provide a valid graph-vertex assembly",
        );

        let Some(FaceSplitBoundaryNode::GraphVertex { graph_vertex, .. }) = assembly
            .vertices
            .iter_mut()
            .find_map(|vertex| match &mut vertex.source {
                source @ FaceSplitBoundaryNode::GraphVertex { .. } => Some(source),
                FaceSplitBoundaryNode::OriginalVertex { .. }
                | FaceSplitBoundaryNode::FaceInterior { .. } => None,
            })
        else {
            panic!("crossing split-region assembly should retain a graph vertex source");
        };
        *graph_vertex = usize::MAX;

        assembly.validate().unwrap();
        assert!(
            assembly
                .validate_source_face_incidence(&left, &right)
                .is_err()
        );
    }

    #[test]
    fn assembly_source_incidence_rejects_face_interior_source_outside_triangle() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 2, 0, 0, 0, 2, 0],
            &[0, 1, 2],
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[],
            &[],
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        let assembly = ExactBooleanAssemblyPlan {
            vertices: vec![
                ExactOutputVertex {
                    point: p(0, 0, 0),
                    source: original(0, p(0, 0, 0)),
                },
                ExactOutputVertex {
                    point: p(3, 0, 0),
                    source: face_interior(p(3, 0, 0)),
                },
                ExactOutputVertex {
                    point: p(0, 2, 0),
                    source: original(2, p(0, 2, 0)),
                },
            ],
            triangles: vec![ExactOutputTriangle {
                vertices: [0, 1, 2],
                source_side: MeshSide::Left,
                source_face: 0,
                orientation: ExactOutputTriangleOrientation::PreserveSource,
            }],
        };

        assembly.validate().unwrap();
        assert!(
            assembly
                .validate_source_face_incidence(&left, &right)
                .is_err()
        );
    }

    #[test]
    fn assembly_canonicalization_refines_exact_t_junctions_before_mesh_handoff() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 2, 0, 0, 0, 2, 0],
            &[0, 1, 2],
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[],
            &[],
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        let mut assembly = ExactBooleanAssemblyPlan {
            vertices: vec![
                ExactOutputVertex {
                    point: p(0, 0, 0),
                    source: original(0, p(0, 0, 0)),
                },
                ExactOutputVertex {
                    point: p(2, 0, 0),
                    source: original(1, p(2, 0, 0)),
                },
                ExactOutputVertex {
                    point: p(0, 2, 0),
                    source: original(2, p(0, 2, 0)),
                },
                ExactOutputVertex {
                    point: p(1, 0, 0),
                    source: face_interior(p(1, 0, 0)),
                },
            ],
            triangles: vec![
                ExactOutputTriangle {
                    vertices: [0, 1, 2],
                    source_side: MeshSide::Left,
                    source_face: 0,
                    orientation: ExactOutputTriangleOrientation::PreserveSource,
                },
                ExactOutputTriangle {
                    vertices: [0, 3, 2],
                    source_side: MeshSide::Left,
                    source_face: 0,
                    orientation: ExactOutputTriangleOrientation::PreserveSource,
                },
            ],
        };
        assembly.validate().unwrap();

        assembly
            .canonicalize_for_mesh_with_sources(&left, &right)
            .unwrap();

        let mut triangle_sets = assembly
            .triangles
            .iter()
            .map(|triangle| {
                let mut vertices = triangle.vertices;
                vertices.sort_unstable();
                vertices
            })
            .collect::<Vec<_>>();
        triangle_sets.sort_unstable();
        assert_eq!(triangle_sets, vec![[0, 2, 3], [1, 2, 3]]);

        let mesh = assembly
            .checked_to_exact_mesh_with_sources(
                &left,
                &right,
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        assert_eq!(mesh.triangles().len(), 2);
        assert_eq!(mesh.facts().mesh.boundary_edges, 4);
        mesh.validate_retained_state().unwrap();
    }
}
