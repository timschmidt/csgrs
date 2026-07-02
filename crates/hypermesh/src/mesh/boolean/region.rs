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
use super::super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
use super::super::graph::SplitPlanBlockerKind;
use super::super::graph::{
    ExactFaceRegionPlan, FaceSplitBoundaryNode, MeshSide, validate_face_region_plan,
};
use super::super::validation::ExactMeshValidationPolicy;
use super::super::{Triangle, paired_triangle_orientation_flips};
use hyperlimit::CoplanarProjection;
use hyperlimit::PredicateUse;
use hyperlimit::SourceProvenance;

use super::{DisjointSets, point3_exact_equal};

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
) -> Result<Vec<FaceRegionPlaneClassification>, ExactMeshError> {
    let mut classifications = Vec::new();
    for region in &regions.regions {
        let (plane_side, plane_mesh) = match region.side {
            MeshSide::Left => (MeshSide::Right, right),
            MeshSide::Right => (MeshSide::Left, left),
        };
        for plane_face in 0..plane_mesh.facts().mesh.face_count {
            classifications.push(classify_region_against_face_plane(
                region.side,
                region.face,
                &region.boundary,
                plane_side,
                plane_mesh,
                plane_face,
            )?);
        }
    }
    Ok(classifications)
}

pub(crate) fn checked_classify_face_regions_against_opposite_planes(
    regions: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Vec<FaceRegionPlaneClassification>, ExactMeshError> {
    let report = validate_face_region_plan(regions, left, right);
    if report.blockers.is_empty() {
        classify_face_regions_against_opposite_planes(regions, left, right)
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
            let expected = project_for_hypertri(boundary_node_point(source), self.projection);
            match exact_points_equal(vertex, &expected) {
                Some(true) => {}
                Some(false) => {
                    return Err(hypertri::Error::InvalidInput {
                        reason: "region triangulation vertex does not match retained boundary source",
                    });
                }
                None => {
                    return Err(hypertri::Error::PredicateUndecided {
                        predicate: "region_triangulation_vertex_source_equality",
                    });
                }
            }
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
            let [a, b, c] = [
                &self.vertices[tri[0]],
                &self.vertices[tri[1]],
                &self.vertices[tri[2]],
            ];
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
                Some(Sign::Negative | Sign::Positive) => {}
                Some(Sign::Zero) => {
                    return Err(hypertri::Error::InvalidInput {
                        reason: "region triangulation triangle is exactly collinear",
                    });
                }
                None => {
                    return Err(hypertri::Error::PredicateUndecided {
                        predicate: "region_triangulation_projected_area",
                    });
                }
            }
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

    fn materialize_exact_mesh(
        &self,
        policy: ExactMeshValidationPolicy,
    ) -> Result<ExactMesh, super::super::error::ExactMeshError> {
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
        ExactMesh::new_with_policy_and_version(
            vertices,
            triangles,
            SourceProvenance::exact("exact boolean assembly plan"),
            policy,
            1,
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
        self.validate()
            .map_err(|error| hypertri_error_to_mesh_error("assembly validation", error))?;
        validate_assembly_source_face_incidence(self, left, right)
            .map_err(|error| hypertri_error_to_mesh_error("assembly source incidence", error))?;
        self.materialize_exact_mesh(policy)
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
        validate_assembly_source_face_incidence(self, left, right)?;
        Ok(())
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
            let incident = self
                .triangles
                .iter()
                .enumerate()
                .filter_map(|(triangle_index, triangle)| {
                    triangle
                        .vertices
                        .contains(&vertex)
                        .then_some(triangle_index)
                })
                .collect::<Vec<_>>();
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
                    for triangle_vertex in &mut self.triangles[triangle].vertices {
                        if *triangle_vertex == vertex {
                            *triangle_vertex = clone_index;
                        }
                    }
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
        let flips = paired_triangle_orientation_flips(
            self.triangles.iter().map(|triangle| triangle.vertices),
            self.triangles.len(),
        )
        .ok_or(hypertri::Error::InvalidInput {
            reason: "selected triangle component has contradictory edge orientation",
        })?;

        let mut flipped = 0;
        for (triangle, flip) in self.triangles.iter_mut().zip(flips) {
            if flip {
                triangle.vertices.swap(1, 2);
                triangle.orientation = match triangle.orientation {
                    ExactOutputTriangleOrientation::PreserveSource => {
                        ExactOutputTriangleOrientation::ReverseSource
                    }
                    ExactOutputTriangleOrientation::ReverseSource => {
                        ExactOutputTriangleOrientation::PreserveSource
                    }
                };
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
            let mut split = None;
            'search: for (triangle_index, triangle) in self.triangles.iter().enumerate() {
                let mesh = match triangle.source_side {
                    MeshSide::Left => left,
                    MeshSide::Right => right,
                };
                let projection = choose_region_projection(mesh, triangle.source_face)?;
                for edge in 0..3 {
                    let start = triangle.vertices[edge];
                    let end = triangle.vertices[(edge + 1) % 3];
                    for candidate in 0..self.vertices.len() {
                        if triangle.vertices.contains(&candidate) {
                            continue;
                        }
                        if !assembly_vertex_lies_on_source_face(
                            self, triangle, candidate, left, right,
                        )? {
                            continue;
                        }
                        if assembly_vertex_lies_strictly_on_projected_edge(
                            self, candidate, start, end, projection,
                        )? {
                            split = Some((triangle_index, edge, candidate));
                            break 'search;
                        }
                    }
                }
            }
            let Some((triangle, edge, vertex)) = split else {
                self.validate()?;
                return Ok(splits);
            };
            let prior_triangle_count = self.triangles.len();
            let original = self.triangles[triangle].clone();
            let a = original.vertices[edge];
            let b = original.vertices[(edge + 1) % 3];
            let c = original.vertices[(edge + 2) % 3];
            let mut first_vertices = [a, vertex, c];
            let mut second_vertices = [vertex, b, c];
            let source_mesh = match original.source_side {
                MeshSide::Left => left,
                MeshSide::Right => right,
            };
            let triangulation = FaceRegionTriangulation {
                side: original.source_side,
                face: original.source_face,
                projection: choose_region_projection(source_mesh, original.source_face)?,
                boundary: Vec::new(),
                vertices: Vec::new(),
                triangles: Vec::new(),
            };
            orient_output_triangle_for_source(
                &triangulation,
                &self.vertices,
                &mut first_vertices,
                original.orientation,
                left,
                right,
            )?;
            orient_output_triangle_for_source(
                &triangulation,
                &self.vertices,
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
            validate_output_triangle_distinct_points(self, &first)?;
            validate_output_triangle_distinct_points(self, &second)?;
            self.triangles
                .splice(triangle..triangle + 1, [first, second]);
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
        validate_assembly_source_face_incidence(self, left, right)?;
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
        let mut replay = ExactBooleanAssemblyPlan::from_region_triangulations_with_triangle_retention_and_sources(
            &triangulations,
            left,
            right,
            |triangulation, _triangle| {
                if selection.keeps(triangulation.side) {
                    ExactRegionRetention::Keep
                } else {
                    ExactRegionRetention::Drop
                }
            },
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
    let [a, b, c] = retained_source_face_points(
        mesh,
        triangle.source_face,
        "assembly triangle references a missing source face",
    )?;
    orient3d_report(a, b, c, &assembly.vertices[vertex].point)
        .value()
        .map(|sign| sign == Sign::Zero)
        .ok_or(hypertri::Error::PredicateUndecided {
            predicate: "assembly_refinement_source_face_incidence",
        })
}

pub(super) fn hypertri_error_to_mesh_error(
    context: &'static str,
    error: hypertri::Error,
) -> ExactMeshError {
    let kind = match &error {
        hypertri::Error::PredicateUndecided { .. } => ExactMeshBlockerKind::UndecidablePredicate,
        hypertri::Error::InvalidInput { .. } => ExactMeshBlockerKind::StaleFactReplay,
        hypertri::Error::NoEarFound => ExactMeshBlockerKind::ExactConstructionFailure,
        hypertri::Error::UnsupportedFeature { .. } => {
            ExactMeshBlockerKind::UnsupportedExactOperation
        }
    };
    ExactMeshError::one(ExactMeshBlocker::new(kind, format!("{context}: {error}")))
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
    for endpoint in [start_point, end_point] {
        match point3_exact_equal(candidate_point, endpoint) {
            Some(true) => return Ok(false),
            Some(false) => {}
            None => {
                return Err(hypertri::Error::PredicateUndecided {
                    predicate: "assembly_refinement_endpoint_equality",
                });
            }
        }
    }
    point_on_segment(
        &project_point3(start_point, projection),
        &project_point3(end_point, projection),
        &project_point3(candidate_point, projection),
    )
    .value()
    .ok_or(hypertri::Error::PredicateUndecided {
        predicate: "assembly_refinement_point_on_edge",
    })
}

fn validate_output_vertex_source(vertex: &ExactOutputVertex) -> hypertri::Result<()> {
    match point3_exact_equal(&vertex.point, boundary_node_point(&vertex.source)) {
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
        match point3_exact_equal(
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

/// Validate output triangles against their retained source face planes.
///
/// Output triangles carry `source_side` and `source_face` so later boolean
/// stages can audit where each triangle came from. This check replays that
/// incidence with exact `hyperlimit::orient3d_report` predicates before
/// handoffs should retain and revalidate the geometric certificates they
/// depend on.
pub(crate) fn validate_assembly_source_face_incidence(
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
        let [a, b, c] = retained_source_face_points(
            mesh,
            triangle.source_face,
            "assembled output triangle references a missing source face",
        )?;
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
            match orient3d_report(a, b, c, &output_vertex.point).value() {
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
        validate_output_triangle_source_orientation(assembly, triangle, mesh)?;
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
                let Ok(source_point) = mesh
                    .view()
                    .vertex(*source_vertex)
                    .map(|vertex| vertex.point())
                else {
                    continue;
                };
                saw_source_vertex = true;
                match point3_exact_equal(point, source_point) {
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
        match point3_exact_equal(point, replayed_point) {
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
    let [a, b, c] = retained_source_face_points(
        mesh,
        source_face,
        "assembled output triangle references a missing source face",
    )?;
    let location = classify_point_triangle(
        &project_point3(a, projection),
        &project_point3(b, projection),
        &project_point3(c, projection),
        &project_point3(point, projection),
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
) -> hypertri::Result<()> {
    let projection = choose_region_projection(mesh, triangle.source_face)?;
    let source_points = retained_source_face_points(
        mesh,
        triangle.source_face,
        "assembled output triangle references a missing source face",
    )?;
    let source_sign = orient2d_report(
        &project_point3(source_points[0], projection),
        &project_point3(source_points[1], projection),
        &project_point3(source_points[2], projection),
    )
    .value()
    .ok_or(hypertri::Error::PredicateUndecided {
        predicate: "assembly_source_orientation",
    })?;
    let output_sign = orient2d_report(
        &project_point3(&assembly.vertices[triangle.vertices[0]].point, projection),
        &project_point3(&assembly.vertices[triangle.vertices[1]].point, projection),
        &project_point3(&assembly.vertices[triangle.vertices[2]].point, projection),
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
/// invariants enforced by [`validate_face_region_plan`]. Only then are
/// them into downstream combinatorics.
pub(crate) fn checked_triangulate_face_regions_with_earcut(
    regions: &ExactFaceRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
) -> hypertri::Result<Vec<FaceRegionTriangulation>> {
    let report = validate_face_region_plan(regions, left, right);
    if !report.blockers.is_empty() {
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
    let source_points = retained_source_face_points(
        source_mesh,
        triangulation.face,
        "region triangulation references a missing source face",
    )?;
    let source_sign = orient2d_report(
        &project_point3(source_points[0], triangulation.projection),
        &project_point3(source_points[1], triangulation.projection),
        &project_point3(source_points[2], triangulation.projection),
    )
    .value()
    .ok_or(hypertri::Error::PredicateUndecided {
        predicate: "assembly_source_orientation",
    })?;
    if source_sign == Sign::Zero {
        return Err(hypertri::Error::InvalidInput {
            reason: "source face has zero projected orientation",
        });
    }
    let [a, b, c] = *output_vertices;
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
    let output_sign = orient2d_report(
        &project_point3(&a.point, triangulation.projection),
        &project_point3(&b.point, triangulation.projection),
        &project_point3(&c.point, triangulation.projection),
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

    let preserves_source = output_sign == source_sign;
    let should_preserve = orientation == ExactOutputTriangleOrientation::PreserveSource;
    if preserves_source != should_preserve {
        output_vertices.swap(1, 2);
    }
    Ok(())
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
        match point3_exact_equal(&vertex.point, &point) {
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

fn vertex_fan_components(
    assembly: &ExactBooleanAssemblyPlan,
    vertex: usize,
    incident: &[usize],
) -> Vec<Vec<usize>> {
    let mut fan = DisjointSets {
        parent: (0..incident.len()).collect(),
    };
    let mut edge_uses = BTreeMap::<usize, Vec<VertexFanEdgeUse>>::new();
    for (local_triangle, &triangle_index) in incident.iter().enumerate() {
        let triangle = assembly.triangles[triangle_index].vertices;
        for index in 0..3 {
            let from = triangle[index];
            let to = triangle[(index + 1) % 3];
            if from == vertex {
                edge_uses.entry(to).or_default().push(VertexFanEdgeUse {
                    local_triangle,
                    forward_from_vertex: true,
                });
            } else if to == vertex {
                edge_uses.entry(from).or_default().push(VertexFanEdgeUse {
                    local_triangle,
                    forward_from_vertex: false,
                });
            }
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
                let left_root = fan.find(left.local_triangle);
                let right_root = fan.find(right.local_triangle);
                if left_root != right_root {
                    fan.parent[right_root] = left_root;
                }
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
    forward_from_vertex: bool,
}

fn classify_region_against_face_plane(
    region_side: MeshSide,
    region_face: usize,
    boundary: &[FaceSplitBoundaryNode],
    plane_side: MeshSide,
    plane_mesh: &ExactMesh,
    plane_face: usize,
) -> Result<FaceRegionPlaneClassification, ExactMeshError> {
    let face = plane_mesh.view().face(plane_face)?;
    let [a, b, c] = face.vertices()?;
    let mut predicates = Vec::with_capacity(boundary.len());
    let mut node_sides = Vec::with_capacity(boundary.len());

    for node in boundary {
        let report = orient3d_report(a, b, c, boundary_node_point(node));
        predicates.push(PredicateUse::from_certificate(report.certificate));
        node_sides.push(report.value().map(PlaneSide::from));
    }

    let relation = relation_from_sides(&node_sides);
    Ok(FaceRegionPlaneClassification {
        region_side,
        region_face,
        plane_side,
        plane_face,
        relation,
        node_sides,
        predicates,
    })
}

pub(crate) fn choose_region_projection(
    mesh: &ExactMesh,
    face: usize,
) -> hypertri::Result<CoplanarProjection> {
    let [a, b, c] = retained_source_face_points(
        mesh,
        face,
        "face region projection references a missing source face",
    )?;
    for projection in [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        let [pa, pb, pc] = [
            project_point3(a, projection),
            project_point3(b, projection),
            project_point3(c, projection),
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

fn retained_source_face_points<'a>(
    mesh: &'a ExactMesh,
    face: usize,
    reason: &'static str,
) -> hypertri::Result<[&'a Point3; 3]> {
    mesh.view()
        .face(face)
        .map_err(|_| hypertri::Error::InvalidInput { reason })?
        .vertices()
        .map_err(|_| hypertri::Error::InvalidInput { reason })
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
    fn region_plane_classification_reports_stale_plane_face_rows() {
        let boundary = vec![
            original(0, p(0, 0, 0)),
            original(1, p(1, 0, 0)),
            original(2, p(0, 1, 0)),
        ];
        let mut plane_mesh = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 1, 1, 0, 1, 0, 1, 1],
            &[0, 1, 2],
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("test plane should construct");
        plane_mesh.facts.faces.clear();

        let error = classify_region_against_face_plane(
            MeshSide::Left,
            0,
            &boundary,
            MeshSide::Right,
            &plane_mesh,
            0,
        )
        .expect_err("stale retained face row should return a typed blocker");
        assert!(
            error.has_only_blocker_kinds(&[ExactMeshBlockerKind::StaleFactReplay]),
            "{error:?}"
        );
        assert_eq!(error.blockers()[0].face(), Some(0));
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
                                && validate_assembly_source_face_incidence(
                                    &candidate, &left, &right,
                                )
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
        assert!(validate_assembly_source_face_incidence(&assembly, &left, &right).is_err());
        let error = assembly
            .checked_to_exact_mesh_with_sources(
                &left,
                &right,
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap_err();
        assert!(
            error.has_only_blocker_kinds(&[ExactMeshBlockerKind::StaleFactReplay]),
            "{error:?}"
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
        assert!(validate_assembly_source_face_incidence(&assembly, &left, &right).is_err());
        let error = assembly
            .checked_to_exact_mesh_with_sources(
                &left,
                &right,
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap_err();
        assert!(
            error.has_only_blocker_kinds(&[ExactMeshBlockerKind::StaleFactReplay]),
            "{error:?}"
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
        assert_eq!(mesh.facts().mesh.face_count, 2);
        assert_eq!(mesh.facts().mesh.boundary_edges, 4);
        mesh.validate_retained_state().unwrap();
    }
}
