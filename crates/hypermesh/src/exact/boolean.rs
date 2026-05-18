//! Exact boolean operation entry points.
//!
//! The legacy boolmesh-derived public API mutates triangle topology through
//! primitive-float kernels. This module is the exact-stack replacement
//! boundary for the subset that is currently implemented: build certified
//! intersection events, form exact split-region loops, classify those regions,
//! triangulate them through feature-gated exact `hypertri`, assemble exact 3D
//! output triangles, and validate the resulting [`ExactMesh`].
//!
//! The operation policy is deliberately explicit. Until full winding and
//! inside/outside classification are ported, callers select which split-region
//! sides are retained rather than receiving a silently approximate
//! union/intersection/difference decision. This follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997): topology
//! decisions must be certified or represented as policy choices/unknowns.

#[cfg(feature = "exact-triangulation")]
use super::bounds::AabbIntersectionKind;
#[cfg(feature = "exact-triangulation")]
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
#[cfg(feature = "exact-triangulation")]
use super::graph::build_intersection_graph;
#[cfg(feature = "exact-triangulation")]
use super::intersection::MeshFacePairRelation;
#[cfg(feature = "exact-triangulation")]
use super::mesh::{ExactMesh, Triangle};
#[cfg(feature = "exact-triangulation")]
use super::provenance::PredicateUse;
#[cfg(feature = "exact-triangulation")]
use super::region::{
    ExactBooleanAssemblyPlan, ExactRegionSelection, FaceRegionPlaneClassification,
    FaceRegionTriangulation, checked_classify_face_regions_against_opposite_planes,
    checked_triangulate_face_regions_with_earcut,
};
#[cfg(feature = "exact-triangulation")]
use super::solid::{ConvexSolidMeshRelation, classify_mesh_vertices_against_convex_solid};
#[cfg(feature = "exact-triangulation")]
use super::surface::{
    CoplanarSurfaceContainment, certify_single_triangle_coplanar_containment,
    difference_single_triangle_coplanar_surfaces, intersect_single_triangle_coplanar_surfaces,
    union_single_triangle_coplanar_surfaces,
};
#[cfg(feature = "exact-triangulation")]
use super::validation::ValidationPolicy;
#[cfg(feature = "exact-triangulation")]
use hyperlimit::{compare_reals, compare_reals_report};
#[cfg(feature = "exact-triangulation")]
use std::cmp::Ordering;

/// Exact selected-region boolean policy.
///
/// This policy is intentionally narrower than a named boolean operation. It
/// records the currently certified operation semantics: retain selected split
/// regions, optionally reject unresolved graph events, then validate the
/// materialized exact output mesh.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactBooleanPolicy {
    /// Which source-side regions should be retained in the output assembly.
    pub selection: ExactRegionSelection,
    /// Validation policy for the materialized output mesh.
    pub validation: ValidationPolicy,
    /// Reject the operation if graph extraction retained unknown events.
    pub reject_unknowns: bool,
}

#[cfg(feature = "exact-triangulation")]
impl ExactBooleanPolicy {
    /// Keep all selected-region output and allow boundary meshes.
    pub const KEEP_ALL_BOUNDARY: Self = Self {
        selection: ExactRegionSelection::KeepAll,
        validation: ValidationPolicy::ALLOW_BOUNDARY,
        reject_unknowns: true,
    };
}

/// Exact boolean operation request.
///
/// Named booleans are represented now, but they intentionally do not fall back
/// to legacy float winding. Certified shortcut cases execute directly, while
/// remaining named overlaps return [`DiagnosticKind::UnsupportedExactOperation`]
/// until exact inside/outside classification is complete.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBooleanOperation {
    /// Assemble explicitly selected source-side split regions.
    SelectedRegions(ExactRegionSelection),
    /// Exact union once certified winding semantics are available.
    Union,
    /// Exact intersection once certified winding semantics are available.
    Intersection,
    /// Exact difference once certified winding semantics are available.
    Difference,
}

/// Boundary-only policy for named exact boolean operations.
///
/// Triangle meshes cannot represent lower-dimensional set intersections
/// without adding a separate curve/point output channel. Following Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), this policy makes that application-layer projection explicit:
/// certified coplanar-touching graphs are either rejected, or projected into a
/// triangle-mesh-only result that preserves separate shells and discards
/// lower-dimensional intersection geometry.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBoundaryBooleanPolicy {
    /// Reject boundary-only named booleans until a caller chooses a projection
    /// policy.
    Reject,
    /// Preserve separate shells for union, keep the left shell for difference,
    /// and return an empty triangle mesh for lower-dimensional intersections.
    PreserveSeparateShells,
}

/// Auditable result of an exact selected-region boolean pipeline.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBooleanResult {
    /// Whether graph extraction contained unknown events before policy checks.
    pub graph_had_unknowns: bool,
    /// Certified classifications of split regions against opposite face
    /// planes.
    pub region_classifications: Vec<FaceRegionPlaneClassification>,
    /// Exact projected triangulations used for assembly.
    pub triangulations: Vec<FaceRegionTriangulation>,
    /// Non-mutating exact output assembly.
    pub assembly: ExactBooleanAssemblyPlan,
    /// Materialized exact output mesh validated under the requested policy.
    pub mesh: ExactMesh,
}

/// Certified support level for a requested exact boolean operation.
///
/// This is the named-boolean staging boundary. Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), frames exact geometric
/// computing as an application-level contract: unresolved combinatorics must be
/// represented explicitly instead of being decided by approximate arithmetic.
/// These variants therefore distinguish executable certified shortcuts from
/// cases whose split regions are available but still need exact winding policy.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBooleanSupport {
    /// The request is an explicit selected-region assembly policy.
    SelectedRegionPolicy,
    /// A named operation was answered by exact empty-operand semantics.
    CertifiedEmptyOperand,
    /// A named operation was answered by certified disjoint AABBs.
    CertifiedBoundsDisjoint,
    /// A named operation was answered by exact coordinate and topology identity.
    CertifiedIdentical,
    /// A named operation was answered by exact coordinate equality and matching
    /// triangle vertex sets, ignoring per-face orientation.
    CertifiedSameSurface,
    /// A named operation was answered by exact no-intersection facts for open
    /// surface meshes.
    CertifiedOpenSurfaceDisjoint,
    /// A named operation was answered by certified closed-convex containment.
    CertifiedConvexContainment,
    /// A named operation was answered by certified single-triangle coplanar
    /// surface containment.
    CertifiedCoplanarSurfaceContainment,
    /// Intersection was materialized by exact clipping of two coplanar
    /// single-triangle surfaces.
    CertifiedCoplanarSurfaceIntersection,
    /// Union was materialized as a certified convex polygon for two coplanar
    /// single-triangle surfaces.
    CertifiedCoplanarSurfaceConvexUnion,
    /// Difference was materialized as a certified one-corner cut from a
    /// coplanar single-triangle surface.
    CertifiedCoplanarSurfaceCornerDifference,
    /// A named operation was answered by a certified no-intersection convex
    /// separated relation that was not caught by mesh-level AABBs.
    CertifiedConvexSeparated,
    /// The retained graph contains only certified coplanar touching events.
    /// A caller must choose a boundary/shared-feature policy before this can
    /// become named boolean output.
    RequiresBoundaryPolicy,
    /// Coplanar positive-area overlap is certified, but the requested named
    /// output needs planar arrangement materialization.
    RequiresPlanarArrangement,
    /// Split-region facts were produced, but named winding semantics are not
    /// yet certified for this nontrivial overlap.
    RequiresCertifiedWinding,
    /// Graph extraction retained unresolved predicate events; callers must
    /// refine, reject, or use a policy that explicitly accepts uncertainty.
    UnresolvedGraph,
}

/// Certification status for same-surface named boolean shortcuts.
///
/// Same-surface detection is stronger than identical storage equality: it
/// allows vertex reindexing and face orientation changes when exact coordinate
/// equality proves a bijection and the remapped triangle vertex sets match.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactSameSurfaceStatus {
    /// The meshes have different vertex counts.
    VertexCountMismatch,
    /// The meshes have different triangle counts.
    TriangleCountMismatch,
    /// At least one required coordinate equality predicate was undecided.
    VertexMatchingUndecided,
    /// No exact vertex bijection exists.
    VertexCoordinateMismatch,
    /// A vertex bijection exists, but remapped triangle sets differ.
    TriangleSetMismatch,
    /// Exact vertex bijection and remapped triangle-set equality were certified.
    Certified,
}

/// Auditable same-surface certification report.
///
/// This is the report form of the same-surface boolean shortcut. It retains
/// the exact vertex permutation, remapped triangle sets, and scalar equality
/// predicate certificates used to prove coordinate equality. The design
/// follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): shortcut topology decisions expose their certified
/// predicate trail rather than collapsing directly to `bool`.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactSameSurfaceReport {
    /// Coarse same-surface certification status.
    pub status: ExactSameSurfaceStatus,
    /// Mapping from each left vertex index to the matched right vertex index.
    pub left_to_right: Vec<usize>,
    /// Mapping from each right vertex index to the matched left vertex index.
    pub right_to_left: Vec<usize>,
    /// Sorted left triangle vertex sets.
    pub left_triangles: Vec<[usize; 3]>,
    /// Sorted right triangle vertex sets remapped into left vertex indices.
    pub right_triangles: Vec<[usize; 3]>,
    /// Predicate certificates used by exact coordinate equality checks.
    pub predicates: Vec<PredicateUse>,
}

#[cfg(feature = "exact-triangulation")]
impl ExactSameSurfaceReport {
    /// Return whether same-surface equivalence was certified.
    pub const fn is_certified(&self) -> bool {
        matches!(self.status, ExactSameSurfaceStatus::Certified)
    }

    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }
}

/// Preflight report for an exact boolean operation request.
///
/// The report gives callers a stable way to audit the current implementation
/// boundary. Shortcut variants are executable by [`boolean_exact`]. For
/// nontrivial named booleans, the report exposes the certified split-region
/// plane classifications that a future exact winding/inside-outside rule must
/// consume, without dispatching to the legacy tolerance kernel.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBooleanPreflight {
    /// Requested operation.
    pub operation: ExactBooleanOperation,
    /// Certified support level for the request.
    pub support: ExactBooleanSupport,
    /// Whether retained graph events contain explicit unknowns.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact broad/narrow scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records across all retained face pairs.
    pub retained_events: usize,
    /// Number of split-region boundaries produced for classification.
    pub region_count: usize,
    /// Certified classifications of split regions against opposite face
    /// planes.
    pub region_classifications: Vec<FaceRegionPlaneClassification>,
    /// Structured explanation for named operations that are certified enough
    /// to inspect but not yet executable by the selected policy.
    pub blocker: Option<ExactBooleanBlocker>,
}

/// Missing exact policy or refinement that blocks named boolean output.
///
/// This is a report object, not an error. Yap's exact-computation model treats
/// unresolved application-layer topology as first-class state: a caller should
/// be able to distinguish "needs exact winding" from "needs a boundary output
/// policy" or "needs predicate refinement" without interpreting prose
/// diagnostics.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactBooleanBlocker {
    /// Missing policy or refinement class.
    pub kind: ExactBooleanBlockerKind,
    /// Number of retained non-coplanar candidate face pairs.
    pub candidate_pairs: usize,
    /// Number of retained coplanar positive-overlap face pairs.
    pub coplanar_overlapping_pairs: usize,
    /// Number of retained coplanar touching face pairs.
    pub coplanar_touching_pairs: usize,
    /// Number of retained unknown face pairs.
    pub unknown_pairs: usize,
}

/// Exact boolean preflight blocker kind.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBooleanBlockerKind {
    /// Predicate or equality refinement is required before policy can run.
    NeedsRefinement,
    /// A lower-dimensional shared-boundary output policy is required.
    NeedsBoundaryPolicy,
    /// A planar arrangement output model is required for coplanar surfaces.
    NeedsPlanarArrangement,
    /// Full winding/inside-outside classification is required.
    NeedsWinding,
}

/// Run the exact selected-region boolean pipeline.
///
/// The returned report keeps the audit artifacts needed to inspect why an
/// output mesh was produced. It does not use primitive-float representatives
/// for topology, and it does not hide unresolved exact predicates unless the
/// caller explicitly disables [`ExactBooleanPolicy::reject_unknowns`].
#[cfg(feature = "exact-triangulation")]
pub fn boolean_selected_regions(
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactBooleanPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    let graph_had_unknowns = graph.has_unknowns();
    if policy.reject_unknowns && graph_had_unknowns {
        return Err(MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::DegenerateTriangle,
            "exact boolean graph contains unresolved predicate events",
        )));
    }

    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
    let triangulations = checked_triangulate_face_regions_with_earcut(&region_plan, left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::DegenerateTriangle,
                format!("exact region triangulation failed: {error}"),
            ))
        })?;
    let assembly =
        ExactBooleanAssemblyPlan::from_region_triangulations(&triangulations, policy.selection)
            .map_err(|error| {
                MeshError::one(MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::IndexOutOfBounds,
                    format!("exact boolean assembly failed: {error}"),
                ))
            })?;
    let mesh = assembly.checked_to_exact_mesh_with_sources(left, right, policy.validation)?;

    Ok(ExactBooleanResult {
        graph_had_unknowns,
        region_classifications,
        triangulations,
        assembly,
        mesh,
    })
}

/// Preflight an exact boolean operation without materializing output topology.
///
/// The preflight path deliberately shares the exact graph, region, and
/// classification stages with the executable selected-region pipeline. For
/// named booleans that are not covered by a certified shortcut, it returns
/// [`ExactBooleanSupport::RequiresCertifiedWinding`] once all available
/// classifications are proof-producing. This keeps the missing operation
/// semantics visible at the API boundary instead of approximating them.
#[cfg(feature = "exact-triangulation")]
pub fn preflight_boolean_exact(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactBooleanPreflight, MeshError> {
    let support = match operation {
        ExactBooleanOperation::SelectedRegions(_) => ExactBooleanSupport::SelectedRegionPolicy,
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if left.triangles().is_empty() || right.triangles().is_empty() =>
        {
            ExactBooleanSupport::CertifiedEmptyOperand
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_bounds_disjoint(left, right) =>
        {
            ExactBooleanSupport::CertifiedBoundsDisjoint
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_identical(left, right) =>
        {
            ExactBooleanSupport::CertifiedIdentical
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_same_surface(left, right) =>
        {
            ExactBooleanSupport::CertifiedSameSurface
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if certified_coplanar_surface_boolean_support(left, right, operation).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceContainment
        }
        ExactBooleanOperation::Intersection
            if intersect_single_triangle_coplanar_surfaces(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
        }
        ExactBooleanOperation::Union
            if union_single_triangle_coplanar_surfaces(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceConvexUnion
        }
        ExactBooleanOperation::Difference
            if difference_single_triangle_coplanar_surfaces(left, right).is_some() =>
        {
            ExactBooleanSupport::CertifiedCoplanarSurfaceCornerDifference
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_open_surface_disjoint(left, right)? =>
        {
            ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => certified_convex_boolean_support(left, right)?
            .unwrap_or(ExactBooleanSupport::RequiresCertifiedWinding),
    };

    if matches!(
        support,
        ExactBooleanSupport::CertifiedEmptyOperand
            | ExactBooleanSupport::CertifiedBoundsDisjoint
            | ExactBooleanSupport::CertifiedIdentical
            | ExactBooleanSupport::CertifiedSameSurface
            | ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
            | ExactBooleanSupport::CertifiedCoplanarSurfaceContainment
            | ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
            | ExactBooleanSupport::CertifiedCoplanarSurfaceConvexUnion
            | ExactBooleanSupport::CertifiedCoplanarSurfaceCornerDifference
            | ExactBooleanSupport::CertifiedConvexContainment
            | ExactBooleanSupport::CertifiedConvexSeparated
    ) {
        return Ok(ExactBooleanPreflight {
            operation,
            support,
            graph_had_unknowns: false,
            retained_face_pairs: 0,
            retained_events: 0,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: None,
        });
    }

    let graph = build_intersection_graph(left, right)?;
    let graph_had_unknowns = graph.has_unknowns();
    let retained_face_pairs = graph.face_pairs.len();
    let retained_events = graph.event_count();
    let relation_counts = graph_relation_counts(&graph);
    if graph_had_unknowns {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::UnresolvedGraph,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(relation_counts.into_blocker(ExactBooleanBlockerKind::NeedsRefinement)),
        });
    }
    if graph_requires_boundary_policy(&graph) {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::RequiresBoundaryPolicy,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(
                relation_counts.into_blocker(ExactBooleanBlockerKind::NeedsBoundaryPolicy),
            ),
        });
    }
    if graph_requires_planar_arrangement(&graph) && operation != ExactBooleanOperation::Intersection
    {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::RequiresPlanarArrangement,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(
                relation_counts.into_blocker(ExactBooleanBlockerKind::NeedsPlanarArrangement),
            ),
        });
    }

    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;

    Ok(ExactBooleanPreflight {
        operation,
        support,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        region_count: region_plan.regions.len(),
        region_classifications,
        blocker: Some(relation_counts.into_blocker(ExactBooleanBlockerKind::NeedsWinding)),
    })
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct GraphRelationCounts {
    candidate_pairs: usize,
    coplanar_overlapping_pairs: usize,
    coplanar_touching_pairs: usize,
    unknown_pairs: usize,
}

#[cfg(feature = "exact-triangulation")]
impl GraphRelationCounts {
    const fn into_blocker(self, kind: ExactBooleanBlockerKind) -> ExactBooleanBlocker {
        ExactBooleanBlocker {
            kind,
            candidate_pairs: self.candidate_pairs,
            coplanar_overlapping_pairs: self.coplanar_overlapping_pairs,
            coplanar_touching_pairs: self.coplanar_touching_pairs,
            unknown_pairs: self.unknown_pairs,
        }
    }
}

#[cfg(feature = "exact-triangulation")]
fn graph_relation_counts(graph: &super::graph::ExactIntersectionGraph) -> GraphRelationCounts {
    let mut counts = GraphRelationCounts::default();
    for pair in &graph.face_pairs {
        match pair.relation {
            MeshFacePairRelation::Candidate => counts.candidate_pairs += 1,
            MeshFacePairRelation::CoplanarOverlapping => counts.coplanar_overlapping_pairs += 1,
            MeshFacePairRelation::CoplanarTouching => counts.coplanar_touching_pairs += 1,
            MeshFacePairRelation::Unknown => counts.unknown_pairs += 1,
            MeshFacePairRelation::BoundsDisjoint | MeshFacePairRelation::PlaneSeparated => {}
        }
    }
    counts
}

#[cfg(feature = "exact-triangulation")]
fn graph_requires_boundary_policy(graph: &super::graph::ExactIntersectionGraph) -> bool {
    !graph.face_pairs.is_empty()
        && graph
            .face_pairs
            .iter()
            .all(|pair| pair.relation == MeshFacePairRelation::CoplanarTouching)
}

#[cfg(feature = "exact-triangulation")]
fn graph_requires_planar_arrangement(graph: &super::graph::ExactIntersectionGraph) -> bool {
    !graph.face_pairs.is_empty()
        && graph
            .face_pairs
            .iter()
            .all(|pair| pair.relation == MeshFacePairRelation::CoplanarOverlapping)
}

/// Run an exact boolean operation request.
///
/// This entry point makes unsupported named booleans explicit rather than
/// silently dispatching to legacy tolerance code. That is a deliberate
/// exact-computation boundary: unsupported topology semantics are diagnostics,
/// not approximate decisions.
#[cfg(feature = "exact-triangulation")]
pub fn boolean_exact(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    boolean_exact_with_boundary_policy(
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    )
}

/// Run an exact boolean operation request with an explicit boundary policy.
///
/// This entry point is still strict about general winding. The additional
/// policy only applies when the exact event graph contains certified
/// coplanar-touching pairs and no crossings, overlaps, or unknowns. In that
/// narrow case, [`ExactBoundaryBooleanPolicy::PreserveSeparateShells`] projects
/// lower-dimensional contact into triangle-mesh output instead of silently
/// invoking the legacy tolerance path.
#[cfg(feature = "exact-triangulation")]
pub fn boolean_exact_with_boundary_policy(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    match operation {
        ExactBooleanOperation::SelectedRegions(selection) => boolean_selected_regions(
            left,
            right,
            ExactBooleanPolicy {
                selection,
                validation,
                reject_unknowns: true,
            },
        ),
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if left.triangles().is_empty() || right.triangles().is_empty() =>
        {
            boolean_empty_operand(left, right, operation, validation)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_bounds_disjoint(left, right) =>
        {
            boolean_disjoint_meshes(left, right, operation, validation)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_identical(left, right) =>
        {
            boolean_identical_meshes(left, operation, validation)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if meshes_are_certified_same_surface(left, right) =>
        {
            boolean_same_surface_meshes(left, operation, validation)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => {
            if let Some(result) =
                boolean_coplanar_surface_containment(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_intersection(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_convex_union(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_corner_difference(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_open_surface_disjoint_meshes(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_convex_containment_meshes(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) = boolean_boundary_touching_meshes(
                left,
                right,
                operation,
                validation,
                boundary_policy,
            )? {
                return Ok(result);
            }
            Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                "named exact booleans require certified winding/inside-outside classification",
            )))
        }
    }
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_surface_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Intersection {
        return Ok(None);
    }
    let Some(intersection) = intersect_single_triangle_coplanar_surfaces(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &intersection.mesh,
        "exact coplanar surface partial-overlap intersection",
        validation,
    )?;
    Ok(Some(shortcut_result(mesh)))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_surface_convex_union(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Union {
        return Ok(None);
    }
    let Some(union) = union_single_triangle_coplanar_surfaces(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &union.mesh,
        "exact convex coplanar surface partial-overlap union",
        validation,
    )?;
    Ok(Some(shortcut_result(mesh)))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_surface_corner_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    let Some(difference) = difference_single_triangle_coplanar_surfaces(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &difference.mesh,
        "exact one-corner coplanar surface difference",
        validation,
    )?;
    Ok(Some(shortcut_result(mesh)))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_open_surface_disjoint_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if !meshes_are_certified_open_surface_disjoint(left, right)? {
        return Ok(None);
    }

    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact open-surface disjoint union",
            validation,
        )?,
        ExactBooleanOperation::Intersection => {
            empty_mesh("empty exact open-surface disjoint intersection", validation)?
        }
        ExactBooleanOperation::Difference => copy_mesh(
            left,
            "exact open-surface disjoint difference keeps left",
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(Some(shortcut_result(mesh)))
}

#[cfg(feature = "exact-triangulation")]
fn meshes_are_certified_open_surface_disjoint(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, MeshError> {
    if !mesh_is_open_surface(left) || !mesh_is_open_surface(right) {
        return Ok(false);
    }
    let graph = build_intersection_graph(left, right)?;
    Ok(!graph.has_unknowns() && graph.face_pairs.is_empty())
}

#[cfg(feature = "exact-triangulation")]
fn mesh_is_open_surface(mesh: &ExactMesh) -> bool {
    !mesh.triangles().is_empty()
        && !mesh.facts().mesh.closed_manifold
        && mesh.facts().mesh.boundary_edges > 0
        && mesh.facts().mesh.non_manifold_edges == 0
        && mesh.facts().mesh.non_manifold_vertices == 0
}

#[cfg(feature = "exact-triangulation")]
fn boolean_coplanar_surface_containment(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(containment) = certified_coplanar_surface_boolean_support(left, right, operation)
    else {
        return Ok(None);
    };

    let mesh = match (containment, operation) {
        (CoplanarSurfaceContainment::LeftInsideRight, ExactBooleanOperation::Union) => copy_mesh(
            right,
            "exact coplanar surface containment union keeps outer right",
            validation,
        )?,
        (CoplanarSurfaceContainment::LeftInsideRight, ExactBooleanOperation::Intersection) => {
            copy_mesh(
                left,
                "exact coplanar surface containment intersection keeps inner left",
                validation,
            )?
        }
        (CoplanarSurfaceContainment::LeftInsideRight, ExactBooleanOperation::Difference) => {
            empty_mesh(
                "empty exact coplanar surface containment difference",
                validation,
            )?
        }
        (CoplanarSurfaceContainment::RightInsideLeft, ExactBooleanOperation::Union) => copy_mesh(
            left,
            "exact coplanar surface containment union keeps outer left",
            validation,
        )?,
        (CoplanarSurfaceContainment::RightInsideLeft, ExactBooleanOperation::Intersection) => {
            copy_mesh(
                right,
                "exact coplanar surface containment intersection keeps inner right",
                validation,
            )?
        }
        (
            CoplanarSurfaceContainment::RightInsideLeft,
            ExactBooleanOperation::Difference | ExactBooleanOperation::SelectedRegions(_),
        )
        | (
            CoplanarSurfaceContainment::LeftInsideRight,
            ExactBooleanOperation::SelectedRegions(_),
        ) => unreachable!("unsupported or selected operation filtered by caller"),
    };

    Ok(Some(shortcut_result(mesh)))
}

#[cfg(feature = "exact-triangulation")]
fn certified_coplanar_surface_boolean_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<CoplanarSurfaceContainment> {
    let containment = certify_single_triangle_coplanar_containment(left, right)?;
    match (containment, operation) {
        (
            CoplanarSurfaceContainment::LeftInsideRight,
            ExactBooleanOperation::Union
            | ExactBooleanOperation::Intersection
            | ExactBooleanOperation::Difference,
        )
        | (
            CoplanarSurfaceContainment::RightInsideLeft,
            ExactBooleanOperation::Union | ExactBooleanOperation::Intersection,
        ) => Some(containment),
        (
            CoplanarSurfaceContainment::RightInsideLeft,
            ExactBooleanOperation::Difference | ExactBooleanOperation::SelectedRegions(_),
        )
        | (
            CoplanarSurfaceContainment::LeftInsideRight,
            ExactBooleanOperation::SelectedRegions(_),
        ) => None,
    }
}

#[cfg(feature = "exact-triangulation")]
fn boolean_same_surface_meshes(
    mesh: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection => {
            copy_mesh(mesh, "exact same-surface boolean result", validation)?
        }
        ExactBooleanOperation::Difference => {
            empty_mesh("empty exact same-surface difference", validation)?
        }
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(shortcut_result(mesh))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_boundary_touching_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if boundary_policy == ExactBoundaryBooleanPolicy::Reject {
        return Ok(None);
    }
    let graph = build_intersection_graph(left, right)?;
    if !graph_requires_boundary_policy(&graph) {
        return Ok(None);
    }

    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact boundary-touch union preserving separate shells",
            validation,
        )?,
        ExactBooleanOperation::Intersection => empty_mesh(
            "empty exact boundary-touch lower-dimensional intersection",
            validation,
        )?,
        ExactBooleanOperation::Difference => copy_mesh(
            left,
            "exact boundary-touch difference preserving left shell",
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(Some(shortcut_result(mesh)))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_convex_containment_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if certified_convex_boolean_support(left, right)?.is_none() {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_convex_solid(left, right);
    let right_in_left = classify_mesh_vertices_against_convex_solid(right, left);
    let mesh = match (left_in_right, right_in_left, operation) {
        (ConvexSolidMeshRelation::StrictlyInside, _, ExactBooleanOperation::Union) => copy_mesh(
            right,
            "exact convex containment union keeps outer right",
            validation,
        )?,
        (ConvexSolidMeshRelation::StrictlyInside, _, ExactBooleanOperation::Intersection) => {
            copy_mesh(
                left,
                "exact convex containment intersection keeps inner left",
                validation,
            )?
        }
        (ConvexSolidMeshRelation::StrictlyInside, _, ExactBooleanOperation::Difference) => {
            empty_mesh("empty exact convex containment difference", validation)?
        }
        (_, ConvexSolidMeshRelation::StrictlyInside, ExactBooleanOperation::Union) => copy_mesh(
            left,
            "exact convex containment union keeps outer left",
            validation,
        )?,
        (_, ConvexSolidMeshRelation::StrictlyInside, ExactBooleanOperation::Intersection) => {
            copy_mesh(
                right,
                "exact convex containment intersection keeps inner right",
                validation,
            )?
        }
        (_, ConvexSolidMeshRelation::StrictlyInside, ExactBooleanOperation::Difference) => {
            concatenate_meshes_with_options(
                left,
                right,
                true,
                "exact convex containment difference with inner reversed shell",
                validation,
            )?
        }
        (
            ConvexSolidMeshRelation::Outside,
            ConvexSolidMeshRelation::Outside,
            ExactBooleanOperation::Union,
        ) => concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact convex separated union",
            validation,
        )?,
        (
            ConvexSolidMeshRelation::Outside,
            ConvexSolidMeshRelation::Outside,
            ExactBooleanOperation::Intersection,
        ) => empty_mesh("empty exact convex separated intersection", validation)?,
        (
            ConvexSolidMeshRelation::Outside,
            ConvexSolidMeshRelation::Outside,
            ExactBooleanOperation::Difference,
        ) => copy_mesh(
            left,
            "exact convex separated difference keeps left",
            validation,
        )?,
        (_, _, ExactBooleanOperation::SelectedRegions(_)) => unreachable!("handled by caller"),
        _ => return Ok(None),
    };

    Ok(Some(shortcut_result(mesh)))
}

#[cfg(feature = "exact-triangulation")]
fn certified_convex_boolean_support(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ExactBooleanSupport>, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    if graph.has_unknowns() || !graph.face_pairs.is_empty() {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_convex_solid(left, right);
    let right_in_left = classify_mesh_vertices_against_convex_solid(right, left);
    Ok(match (left_in_right, right_in_left) {
        (ConvexSolidMeshRelation::StrictlyInside, _)
        | (_, ConvexSolidMeshRelation::StrictlyInside) => {
            Some(ExactBooleanSupport::CertifiedConvexContainment)
        }
        (ConvexSolidMeshRelation::Outside, ConvexSolidMeshRelation::Outside) => {
            Some(ExactBooleanSupport::CertifiedConvexSeparated)
        }
        _ => None,
    })
}

#[cfg(feature = "exact-triangulation")]
fn copy_mesh(
    mesh: &ExactMesh,
    label: &'static str,
    validation: ValidationPolicy,
) -> Result<ExactMesh, MeshError> {
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        mesh.triangles().to_vec(),
        super::provenance::SourceProvenance::exact(label),
        validation,
    )
}

#[cfg(feature = "exact-triangulation")]
fn concatenate_meshes_with_options(
    left: &ExactMesh,
    right: &ExactMesh,
    reverse_right: bool,
    label: &'static str,
    validation: ValidationPolicy,
) -> Result<ExactMesh, MeshError> {
    let mut vertices = left.vertices().to_vec();
    let right_offset = vertices.len();
    vertices.extend_from_slice(right.vertices());
    let mut triangles = left.triangles().to_vec();
    triangles.extend(right.triangles().iter().map(|triangle| {
        let [a, b, c] = triangle.0;
        if reverse_right {
            Triangle([a + right_offset, c + right_offset, b + right_offset])
        } else {
            Triangle([a + right_offset, b + right_offset, c + right_offset])
        }
    }));
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        super::provenance::SourceProvenance::exact(label),
        validation,
    )
}

#[cfg(feature = "exact-triangulation")]
fn meshes_are_certified_bounds_disjoint(left: &ExactMesh, right: &ExactMesh) -> bool {
    let (Some(left_bounds), Some(right_bounds)) = (&left.bounds().mesh, &right.bounds().mesh)
    else {
        return left.triangles().is_empty() || right.triangles().is_empty();
    };
    left_bounds.classify_intersection(right_bounds).value() == Some(AabbIntersectionKind::Disjoint)
}

#[cfg(feature = "exact-triangulation")]
fn meshes_are_certified_identical(left: &ExactMesh, right: &ExactMesh) -> bool {
    left.triangles() == right.triangles()
        && left.vertices().len() == right.vertices().len()
        && vertices_are_certified_equal(left, right)
}

#[cfg(feature = "exact-triangulation")]
fn meshes_are_certified_same_surface(left: &ExactMesh, right: &ExactMesh) -> bool {
    certify_same_surface_report(left, right).is_certified()
}

#[cfg(feature = "exact-triangulation")]
/// Certify whether two meshes represent the same triangle surface.
///
/// The report preserves the exact coordinate-equality predicate certificates
/// used to find a vertex bijection and the sorted triangle sets compared after
/// remapping. This is the auditable form of the same-surface shortcut used by
/// named exact booleans, following Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): exact topology decisions should
/// expose the predicate facts that justify them.
pub fn certify_same_surface_report(left: &ExactMesh, right: &ExactMesh) -> ExactSameSurfaceReport {
    if left.vertices().len() != right.vertices().len() {
        return same_surface_report(
            ExactSameSurfaceStatus::VertexCountMismatch,
            Vec::new(),
            Vec::new(),
            Vec::new(),
        );
    }
    if left.triangles().len() != right.triangles().len() {
        return same_surface_report(
            ExactSameSurfaceStatus::TriangleCountMismatch,
            Vec::new(),
            Vec::new(),
            Vec::new(),
        );
    }

    let (left_to_right, predicates, status) = certified_vertex_permutation_report(left, right);
    if status != ExactSameSurfaceStatus::Certified {
        return same_surface_report(status, left_to_right, Vec::new(), predicates);
    }
    let right_to_left = invert_permutation(&left_to_right);

    let mut left_triangles = sorted_triangle_sets(left, None);
    let mut right_triangles = sorted_triangle_sets(right, Some(&right_to_left));
    left_triangles.sort_unstable();
    right_triangles.sort_unstable();
    let status = if left_triangles == right_triangles {
        ExactSameSurfaceStatus::Certified
    } else {
        ExactSameSurfaceStatus::TriangleSetMismatch
    };

    ExactSameSurfaceReport {
        status,
        left_to_right,
        right_to_left,
        left_triangles,
        right_triangles,
        predicates,
    }
}

#[cfg(feature = "exact-triangulation")]
fn same_surface_report(
    status: ExactSameSurfaceStatus,
    left_to_right: Vec<usize>,
    right_to_left: Vec<usize>,
    predicates: Vec<PredicateUse>,
) -> ExactSameSurfaceReport {
    ExactSameSurfaceReport {
        status,
        left_to_right,
        right_to_left,
        left_triangles: Vec::new(),
        right_triangles: Vec::new(),
        predicates,
    }
}

#[cfg(feature = "exact-triangulation")]
fn vertices_are_certified_equal(left: &ExactMesh, right: &ExactMesh) -> bool {
    left.vertices()
        .iter()
        .zip(right.vertices())
        .all(|(left, right)| {
            let left = left.to_hyperlimit_point();
            let right = right.to_hyperlimit_point();
            compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
                && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
                && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
        })
}

#[cfg(feature = "exact-triangulation")]
fn certified_vertex_permutation_report(
    left: &ExactMesh,
    right: &ExactMesh,
) -> (Vec<usize>, Vec<PredicateUse>, ExactSameSurfaceStatus) {
    let mut left_to_right = Vec::with_capacity(left.vertices().len());
    let mut used_right = vec![false; right.vertices().len()];
    let mut predicates = Vec::new();

    for left_vertex in left.vertices() {
        let left_point = left_vertex.to_hyperlimit_point();
        let mut match_index = None;
        let mut saw_undecided = false;
        for (right_index, right_vertex) in right.vertices().iter().enumerate() {
            if used_right[right_index] {
                continue;
            }
            let right_point = right_vertex.to_hyperlimit_point();
            let x = compare_reals_report(&left_point.x, &right_point.x);
            let y = compare_reals_report(&left_point.y, &right_point.y);
            let z = compare_reals_report(&left_point.z, &right_point.z);
            predicates.push(PredicateUse::from_certificate(x.certificate));
            predicates.push(PredicateUse::from_certificate(y.certificate));
            predicates.push(PredicateUse::from_certificate(z.certificate));
            let Some(x_value) = x.outcome.value() else {
                saw_undecided = true;
                continue;
            };
            let Some(y_value) = y.outcome.value() else {
                saw_undecided = true;
                continue;
            };
            let Some(z_value) = z.outcome.value() else {
                saw_undecided = true;
                continue;
            };
            let equal = x_value == Ordering::Equal
                && y_value == Ordering::Equal
                && z_value == Ordering::Equal;
            if equal {
                match_index = Some(right_index);
                break;
            }
        }
        let Some(match_index) = match_index else {
            let status = if saw_undecided {
                ExactSameSurfaceStatus::VertexMatchingUndecided
            } else {
                ExactSameSurfaceStatus::VertexCoordinateMismatch
            };
            return (left_to_right, predicates, status);
        };
        used_right[match_index] = true;
        left_to_right.push(match_index);
    }

    (left_to_right, predicates, ExactSameSurfaceStatus::Certified)
}

#[cfg(feature = "exact-triangulation")]
fn invert_permutation(permutation: &[usize]) -> Vec<usize> {
    let mut inverse = vec![0; permutation.len()];
    for (left_index, &right_index) in permutation.iter().enumerate() {
        inverse[right_index] = left_index;
    }
    inverse
}

#[cfg(feature = "exact-triangulation")]
fn sorted_triangle_sets(mesh: &ExactMesh, right_to_left: Option<&[usize]>) -> Vec<[usize; 3]> {
    mesh.triangles()
        .iter()
        .map(|triangle| {
            let mut vertices = triangle.0.map(|vertex| match right_to_left {
                Some(mapping) => mapping[vertex],
                None => vertex,
            });
            vertices.sort_unstable();
            vertices
        })
        .collect()
}

#[cfg(feature = "exact-triangulation")]
fn boolean_disjoint_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes(left, right, validation)?,
        ExactBooleanOperation::Intersection => {
            empty_mesh("empty exact disjoint intersection", validation)?
        }
        ExactBooleanOperation::Difference => ExactMesh::new_with_policy(
            left.vertices().to_vec(),
            left.triangles().to_vec(),
            super::provenance::SourceProvenance::exact("exact disjoint left difference"),
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(shortcut_result(mesh))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_empty_operand(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes(left, right, validation)?,
        ExactBooleanOperation::Intersection => {
            empty_mesh("empty exact intersection with empty operand", validation)?
        }
        ExactBooleanOperation::Difference if left.triangles().is_empty() => {
            empty_mesh("empty exact difference from empty left operand", validation)?
        }
        ExactBooleanOperation::Difference => ExactMesh::new_with_policy(
            left.vertices().to_vec(),
            left.triangles().to_vec(),
            super::provenance::SourceProvenance::exact("exact difference with empty right operand"),
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(shortcut_result(mesh))
}

#[cfg(feature = "exact-triangulation")]
fn boolean_identical_meshes(
    mesh: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection => {
            ExactMesh::new_with_policy(
                mesh.vertices().to_vec(),
                mesh.triangles().to_vec(),
                super::provenance::SourceProvenance::exact("exact identical boolean result"),
                validation,
            )?
        }
        ExactBooleanOperation::Difference => ExactMesh::new_with_policy(
            Vec::new(),
            Vec::new(),
            super::provenance::SourceProvenance::exact("empty exact identical difference"),
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(shortcut_result(mesh))
}

#[cfg(feature = "exact-triangulation")]
fn empty_mesh(label: &'static str, validation: ValidationPolicy) -> Result<ExactMesh, MeshError> {
    ExactMesh::new_with_policy(
        Vec::new(),
        Vec::new(),
        super::provenance::SourceProvenance::exact(label),
        validation,
    )
}

#[cfg(feature = "exact-triangulation")]
fn shortcut_result(mesh: ExactMesh) -> ExactBooleanResult {
    ExactBooleanResult {
        graph_had_unknowns: false,
        region_classifications: Vec::new(),
        triangulations: Vec::new(),
        assembly: ExactBooleanAssemblyPlan {
            vertices: Vec::new(),
            triangles: Vec::new(),
        },
        mesh,
    }
}

#[cfg(feature = "exact-triangulation")]
fn concatenate_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactMesh, MeshError> {
    let mut vertices = left.vertices().to_vec();
    let right_offset = vertices.len();
    vertices.extend_from_slice(right.vertices());
    let mut triangles = left.triangles().to_vec();
    triangles.extend(right.triangles().iter().map(|triangle| {
        let [a, b, c] = triangle.0;
        Triangle([a + right_offset, b + right_offset, c + right_offset])
    }));
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        super::provenance::SourceProvenance::exact("exact disjoint union"),
        validation,
    )
}
