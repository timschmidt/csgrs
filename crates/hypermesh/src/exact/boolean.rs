//! Exact boolean operation entry points.
//!
//! The legacy boolmesh-derived public API mutates triangle topology through
//! primitive-float kernels. This module is the exact-stack replacement
//! boundary for the subset that is currently implemented: build certified
//! intersection events, form exact split-region loops, classify those regions,
//! triangulate them through exact `hypertri`, assemble exact 3D
//! output triangles, and validate the resulting [`ExactMesh`].
//!
//! The operation policy is deliberately explicit. No-intersection named
//! booleans are handled by certified empty/disjoint/identity, convex,
//! coplanar, or exact ray-parity winding shortcuts; remaining split-region
//! cases require a selected-region policy or an explicit unsupported report
//! instead of a silently approximate union/intersection/difference decision.
//! Topology decisions must be certified or represented as policy choices or
//! unknowns.

use super::adjacent::{
    full_face_adjacent_certificate, has_full_face_adjacent_union,
    materialize_full_face_adjacent_union_from_certificate,
};
use super::affine_box::{
    has_affine_box_difference, has_affine_box_intersection, has_affine_box_union,
    materialize_affine_box_difference, materialize_affine_box_intersection,
    materialize_affine_box_union,
};
use super::affine_solid::{
    AffineOrthogonalSolidOperation, has_affine_orthogonal_solid_cells,
    materialize_affine_orthogonal_solid_difference,
    materialize_affine_orthogonal_solid_intersection, materialize_affine_orthogonal_solid_union,
};
use super::affine_surface::{
    CoplanarAffineSurfaceBasis, arrange_coplanar_affine_surface_difference,
    arrange_coplanar_affine_surface_intersection, arrange_coplanar_affine_surface_union,
};
use super::arrangement2d::{
    ExactArrangement2dRegion, ExactArrangement2dRegionRing, ExactArrangement2dSetOperation,
    build_exact_arrangement2d_overlay,
};
use super::arrangement3d::ExactArrangement;
use super::boolmesh::{
    ExactBoolMeshKernelStage, ExactBoolMeshValidationError,
    exact_boolmesh_workspace_from_graph_for_support, execute_exact_boolmesh_bounds_disjoint,
    execute_exact_boolmesh_port, execute_exact_boolmesh_port_from_graph,
};
use super::bounds::AabbIntersectionKind;
use super::box_solid::{
    AxisAlignedBoxOperation, cell_difference_axis_aligned_boxes, cell_union_axis_aligned_boxes,
    empty_difference_axis_aligned_boxes, has_axis_aligned_box_cell_difference,
    has_axis_aligned_box_cell_union, has_axis_aligned_box_difference,
    has_axis_aligned_box_empty_difference, has_axis_aligned_box_intersection,
    has_axis_aligned_box_multi_difference, has_axis_aligned_box_nested_difference,
    has_axis_aligned_box_union, is_axis_aligned_box, materialize_simple_axis_aligned_box_operation,
    multi_difference_axis_aligned_boxes, nested_difference_axis_aligned_boxes,
};
use super::cells::triangulate_all_face_cells_with_cdt;
use super::construction::SegmentPlaneRelation;
use super::contained_adjacent::{
    ContainedBoundaryContainment, ContainedBoundaryDifferenceCertificate,
    contained_boundary_containment_from_graph,
    contained_boundary_difference_certificate_from_graph, contained_face_adjacent_certificate,
    has_contained_boundary_difference_from_graph, has_contained_face_adjacent_union,
    materialize_contained_boundary_difference_from_graph,
    materialize_contained_boundary_difference_from_retained_certificate,
    materialize_contained_face_adjacent_union_from_certificate,
};
use super::convex::{intersect_closed_convex_solids, subtract_closed_convex_solids_single_cap};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::graph::{FacePairEvents, IntersectionEvent, MeshSide, build_intersection_graph};
use super::intersection::MeshFacePairRelation;
use super::mesh::{ExactMesh, Triangle};
use super::orthogonal_solid::{
    AxisAlignedOrthogonalSolidOperation, has_axis_aligned_orthogonal_solid_cells,
    has_empty_axis_aligned_orthogonal_solid_cell_intersection,
    materialize_axis_aligned_orthogonal_solid_cells,
};
use super::orthogonal_surface::{
    CoplanarOrthogonalSurfaceOperation, arrange_coplanar_orthogonal_surface_difference,
    arrange_coplanar_orthogonal_surface_intersection, arrange_coplanar_orthogonal_surface_union,
};
use super::provenance::{PredicateUse, SourceProvenance};
use super::region::{
    ExactBooleanAssemblyPlan, ExactRegionRetention, ExactRegionSelection,
    FaceRegionPlaneClassification, FaceRegionTriangulation,
    checked_classify_face_regions_against_opposite_planes,
    checked_triangulate_face_regions_with_earcut,
};
use super::regularization::{ExactArrangementBlocker, ExactRegularizationPolicy};
use super::reports::{
    ExactBooleanBlocker, ExactBooleanBlockerKind, ExactBooleanPreflight, ExactBooleanResult,
    ExactBooleanResultKind, ExactBooleanShortcutKind, ExactBooleanSupport,
    ExactBoundaryTouchingReport, ExactBoundaryTouchingStatus, ExactOpenSurfaceDisjointReport,
    ExactOpenSurfaceDisjointStatus, ExactPlanarArrangementReport, ExactPlanarArrangementStatus,
    ExactRefinementReport, ExactRefinementStatus, ExactSameSurfaceReport, ExactSameSurfaceStatus,
    ExactWindingReadinessReport, ExactWindingReadinessStatus,
};
use super::solid::{
    ConvexSolidMeshClassification, ConvexSolidMeshRelation, ConvexSolidPointRelation,
    classify_mesh_vertices_against_convex_solid_report,
};
use super::surface::{
    CoplanarConvexSurfaceContainment, CoplanarConvexSurfaceContainmentCertificate,
    CoplanarSurfaceContainment, arrange_coplanar_convex_surface_component_holed_difference,
    arrange_coplanar_convex_surface_component_union, arrange_coplanar_convex_surface_difference,
    arrange_coplanar_convex_surface_holed_difference,
    arrange_coplanar_convex_surface_holed_difference_from_certificate,
    arrange_coplanar_convex_surface_intersection, arrange_coplanar_convex_surface_multi_difference,
    arrange_coplanar_convex_surface_multi_holed_difference,
    arrange_coplanar_convex_surface_multi_intersection,
    arrange_coplanar_convex_surface_multi_union, arrange_coplanar_convex_surface_union,
    arrange_coplanar_surface_component_difference,
    arrange_coplanar_surface_component_holed_difference,
    arrange_coplanar_surface_component_holed_intersection,
    arrange_coplanar_surface_component_holed_union,
    arrange_coplanar_surface_component_intersection, arrange_coplanar_surface_component_union,
    arrange_coplanar_surface_cutter_hole_contact_difference,
    arrange_coplanar_surface_multi_component_intersection,
    arrange_coplanar_surface_multi_component_union, arrange_coplanar_surface_multi_difference,
    arrange_coplanar_surface_point_touch_difference, arrange_coplanar_surface_point_touch_union,
    arrange_coplanar_surface_side_cutter_difference, arrange_single_triangle_coplanar_difference,
    arrange_single_triangle_coplanar_holed_difference, arrange_single_triangle_coplanar_union,
    certify_coplanar_convex_surface_containment, certify_coplanar_convex_surface_equivalence,
    certify_coplanar_surface_boundary_touch, certify_coplanar_surface_mesh_containment,
    certify_single_triangle_coplanar_containment, difference_single_triangle_coplanar_surfaces,
    intersect_single_triangle_coplanar_surfaces, order_mesh_boundary_loops,
    union_single_triangle_coplanar_surfaces,
};
use super::validation::ValidationPolicy;
use super::volumetric::{
    ExactVolumetricRegionClassification, ExactVolumetricRegionError, ExactVolumetricRegionRelation,
    classify_triangulated_regions_against_opposite_meshes,
};
use super::volumetric_cells::{
    CoplanarVolumetricCellEvidenceReport, CoplanarVolumetricCellObstacle,
};
use super::winding::{
    ClosedMeshWindingMeshRelation, ClosedMeshWindingMeshReport, ClosedMeshWindingRelation,
    WindingReportError, classify_mesh_vertices_against_closed_mesh_winding_report,
};
use hyperlimit::{
    CoplanarProjection, Point2, Point3, RingPointLocation, SegmentIntersection, Sign,
    TriangleLocation, classify_point_ring_even_odd, classify_point_triangle, compare_reals,
    compare_reals_report, orient3d_report, project_point3, projected_polygon_area2_value,
};
use hyperreal::Real;
use std::cmp::Ordering;

/// Exact selected-region boolean policy.
///
/// This policy is intentionally narrower than a named boolean operation. It
/// records the currently certified operation semantics: retain selected split
/// regions, optionally reject unresolved graph events, then validate the
/// materialized exact output mesh.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactBooleanPolicy {
    /// Which source-side regions should be retained in the output assembly.
    pub selection: ExactRegionSelection,
    /// Validation policy for the materialized output mesh.
    pub validation: ValidationPolicy,
    /// Reject the operation if graph extraction retained unknown events.
    pub reject_unknowns: bool,
}

impl ExactBooleanPolicy {
    /// Keep all selected-region output and allow boundary meshes.
    pub const KEEP_ALL_BOUNDARY: Self = Self {
        selection: ExactRegionSelection::KeepAll,
        validation: ValidationPolicy::ALLOW_BOUNDARY,
        reject_unknowns: true,
    };
}

/// Stage reached by an arrangement/cell-complex Boolean attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactArrangementBooleanStage {
    /// Arrangement construction was not attempted by the selected dispatch path.
    NotAttempted,
    /// The 3D arrangement was built.
    ArrangementBuilt,
    /// Arrangement face-cells were labeled.
    Labeled,
    /// Boolean selection completed.
    Selected,
    /// Exact simplification completed.
    Simplified,
    /// Exact triangulation completed.
    Triangulated,
    /// The triangulated mesh copied through the requested validation policy.
    Materialized,
}

/// Why an arrangement/cell-complex Boolean attempt declined to produce output.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactArrangementBooleanDecline {
    /// The dispatch mode intentionally left this case to older certified paths.
    DispatchGate,
    /// Arrangement construction completed with blockers.
    ArrangementBlockers(Vec<ExactArrangementBlocker>),
    /// Cell labeling failed.
    Labeling(ExactArrangementBlocker),
    /// Boolean cell selection failed.
    Selection(ExactArrangementBlocker),
    /// Exact simplification failed.
    Simplification(ExactArrangementBlocker),
    /// Exact triangulation failed.
    Triangulation(ExactArrangementBlocker),
    /// The triangulated mesh did not satisfy the requested validation policy.
    OutputValidation,
}

/// Auditable result of trying the arrangement/cell-complex Boolean pipeline.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactArrangementBooleanAttempt {
    /// Operation attempted.
    pub operation: ExactBooleanOperation,
    /// Regularization policy used by the arrangement pipeline.
    pub policy: ExactRegularizationPolicy,
    /// Furthest stage reached.
    pub stage: ExactArrangementBooleanStage,
    /// Reason no output was produced, when the attempt declined.
    pub decline: Option<ExactArrangementBooleanDecline>,
    /// Arrangement blocker count observed after construction.
    pub arrangement_blockers: usize,
    /// Arrangement face-cell count, when construction succeeded.
    pub face_cells: usize,
    /// Connected shell/region count, when construction succeeded.
    pub regions: usize,
    /// Selected face-cell count, when selection succeeded.
    pub selected_faces: usize,
    /// Output vertex count, when triangulation succeeded.
    pub output_vertices: usize,
    /// Output triangle count, when triangulation succeeded.
    pub output_triangles: usize,
}

/// Exact boolean operation request.
///
/// Named booleans are represented now, but they intentionally do not fall back
/// to legacy float winding. Certified shortcut cases execute directly, while
/// remaining named overlaps return [`DiagnosticKind::UnsupportedExactOperation`]
/// until split-region inside/outside classification is complete.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBooleanOperation {
    /// Assemble explicitly selected source-side split regions.
    SelectedRegions(ExactRegionSelection),
    /// Exact union through certified shortcuts or future split-region winding.
    Union,
    /// Exact intersection through certified shortcuts or future split-region
    /// winding.
    Intersection,
    /// Exact difference through certified shortcuts or future split-region
    /// winding.
    Difference,
}

/// Boundary-only policy for named exact boolean operations.
///
/// Triangle meshes cannot represent lower-dimensional set intersections
/// certified coplanar-touching graphs are either rejected, or projected into a
/// triangle-mesh-only result that preserves separate shells and discards
/// lower-dimensional intersection geometry.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBoundaryBooleanPolicy {
    /// Reject boundary-only named booleans until a caller chooses a projection
    /// policy.
    Reject,
    /// Preserve separate shells for union, keep the left shell for difference,
    /// and return an empty triangle mesh for lower-dimensional intersections.
    PreserveSeparateShells,
}

/// Run the exact selected-region boolean pipeline.
///
/// The returned report keeps the audit artifacts needed to inspect why an
/// output mesh was produced. It does not use primitive-float representatives
/// for topology, and it does not hide unresolved exact predicates unless the
/// caller explicitly disables [`ExactBooleanPolicy::reject_unknowns`].
pub fn boolean_selected_regions(
    left: &ExactMesh,
    right: &ExactMesh,
    policy: ExactBooleanPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    let graph_had_unknowns = graph.has_unknowns();
    if policy.reject_unknowns && graph_had_unknowns {
        return Err(MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
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
    let assembly = ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
        &triangulations,
        policy.selection,
        left,
        right,
    )
    .map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::IndexOutOfBounds,
            format!("exact boolean assembly failed: {error}"),
        ))
    })?;
    let mesh = assembly.checked_to_exact_mesh_with_sources(left, right, policy.validation)?;

    let result = ExactBooleanResult {
        kind: ExactBooleanResultKind::SelectedRegions {
            selection: policy.selection,
        },
        graph_had_unknowns,
        region_classifications,
        triangulations,
        assembly,
        volumetric_classifications: Vec::new(),
        mesh,
    };
    result.validate().map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact selected-region result validation failed: {error:?}"),
        ))
    })?;
    Ok(result)
}

/// Preflight an exact boolean operation without materializing output topology.
///
/// The preflight path deliberately shares the exact graph, region, and
/// classification stages with the executable selected-region pipeline. For
/// named booleans that are not covered by a certified shortcut, it returns
/// [`ExactBooleanSupport::RequiresCertifiedWinding`] once all available
/// classifications are proof-producing. This keeps the missing operation
/// semantics visible at the API boundary instead of approximating them.
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
        ExactBooleanOperation::Union if has_axis_aligned_box_union(left, right) => {
            ExactBooleanSupport::CertifiedAxisAlignedBoxUnion
        }
        ExactBooleanOperation::Intersection if has_axis_aligned_box_intersection(left, right) => {
            ExactBooleanSupport::CertifiedAxisAlignedBoxIntersection
        }
        ExactBooleanOperation::Difference if has_axis_aligned_box_difference(left, right) => {
            ExactBooleanSupport::CertifiedAxisAlignedBoxDifference
        }
        ExactBooleanOperation::Union if non_box_full_face_adjacency(left, right) => {
            ExactBooleanSupport::CertifiedFullFaceAdjacentUnion
        }
        ExactBooleanOperation::Union if has_contained_face_adjacent_union(left, right) => {
            ExactBooleanSupport::CertifiedContainedFaceAdjacentUnion
        }
        ExactBooleanOperation::Intersection if has_contained_face_adjacent_union(left, right) => {
            ExactBooleanSupport::CertifiedContainedFaceAdjacentIntersection
        }
        ExactBooleanOperation::Difference if has_contained_face_adjacent_union(left, right) => {
            ExactBooleanSupport::CertifiedContainedFaceAdjacentDifference
        }
        ExactBooleanOperation::Intersection if non_box_full_face_adjacency(left, right) => {
            ExactBooleanSupport::CertifiedFullFaceAdjacentIntersection
        }
        ExactBooleanOperation::Difference if non_box_full_face_adjacency(left, right) => {
            ExactBooleanSupport::CertifiedFullFaceAdjacentDifference
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => {
            preflight_direct_coplanar_surface_support(left, right, operation).unwrap_or_else(|| {
                preflight_tail_shortcut_support(left, right, operation)
                    .unwrap_or(ExactBooleanSupport::RequiresCertifiedWinding)
            })
        }
    };

    if matches!(
        support,
        ExactBooleanSupport::CertifiedEmptyOperand
            | ExactBooleanSupport::CertifiedBoundsDisjoint
            | ExactBooleanSupport::CertifiedIdentical
            | ExactBooleanSupport::CertifiedSameSurface
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceEquivalence
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceIntersection
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiUnion
            | ExactBooleanSupport::CertifiedCoplanarSurfaceMultiUnion
            | ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchUnion
            | ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchIntersection
            | ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
            | ExactBooleanSupport::CertifiedCoplanarSurfaceBoundaryTouchIntersection
            | ExactBooleanSupport::CertifiedCoplanarSurfaceBoundaryTouchDifference
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementDifference
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiDifference
            | ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
            | ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
            | ExactBooleanSupport::CertifiedCoplanarSurfaceSideCutterDifference
            | ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceUnion
            | ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceIntersection
            | ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceDifference
            | ExactBooleanSupport::CertifiedCoplanarAffineSurfaceUnion
            | ExactBooleanSupport::CertifiedCoplanarAffineSurfaceIntersection
            | ExactBooleanSupport::CertifiedCoplanarAffineSurfaceDifference
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceContainment
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceHoledDifference
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiHoledDifference
            | ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
            | ExactBooleanSupport::CertifiedAxisAlignedBoxUnion
            | ExactBooleanSupport::CertifiedAxisAlignedBoxIntersection
            | ExactBooleanSupport::CertifiedAxisAlignedBoxDifference
            | ExactBooleanSupport::CertifiedAxisAlignedBoxMultiDifference
            | ExactBooleanSupport::CertifiedAxisAlignedBoxNestedDifference
            | ExactBooleanSupport::CertifiedAxisAlignedBoxEmptyDifference
            | ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellUnion
            | ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellIntersection
            | ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellDifference
            | ExactBooleanSupport::CertifiedAffineBoxUnion
            | ExactBooleanSupport::CertifiedAffineBoxIntersection
            | ExactBooleanSupport::CertifiedAffineBoxDifference
            | ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellUnion
            | ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellIntersection
            | ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellDifference
            | ExactBooleanSupport::CertifiedFullFaceAdjacentUnion
            | ExactBooleanSupport::CertifiedContainedFaceAdjacentUnion
            | ExactBooleanSupport::CertifiedContainedBoundaryDifference
            | ExactBooleanSupport::CertifiedContainedBoundaryContainment
            | ExactBooleanSupport::CertifiedContainedFaceAdjacentIntersection
            | ExactBooleanSupport::CertifiedContainedFaceAdjacentDifference
            | ExactBooleanSupport::CertifiedFullFaceAdjacentIntersection
            | ExactBooleanSupport::CertifiedFullFaceAdjacentDifference
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
            | ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
            | ExactBooleanSupport::CertifiedCoplanarSurfaceContainment
            | ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
            | ExactBooleanSupport::CertifiedCoplanarSurfaceConvexUnion
            | ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
            | ExactBooleanSupport::CertifiedCoplanarSurfaceCornerDifference
            | ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
            | ExactBooleanSupport::CertifiedCoplanarSurfaceHoledDifference
            | ExactBooleanSupport::CertifiedConvexIntersection
            | ExactBooleanSupport::CertifiedConvexSingleCapDifference
            | ExactBooleanSupport::CertifiedConvexContainment
            | ExactBooleanSupport::CertifiedConvexSeparated
            | ExactBooleanSupport::CertifiedWindingContainment
            | ExactBooleanSupport::CertifiedWindingSeparated
            | ExactBooleanSupport::CertifiedBoolMeshSplit
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
            arrangement_readiness: None,
            coplanar_volumetric_evidence: None,
        });
    }

    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
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
            arrangement_readiness: None,
            coplanar_volumetric_evidence: None,
        });
    }
    if relation_counts.construction_failed_events > 0 {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::UnresolvedGraph,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(relation_counts.into_blocker(ExactBooleanBlockerKind::NeedsRefinement)),
            arrangement_readiness: None,
            coplanar_volumetric_evidence: None,
        });
    }
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        let geometry = graph.face_split_geometry_plan(left, right)?;
        let region_plan = geometry.region_plan(left, right);
        let region_classifications =
            checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::SelectedRegionPolicy,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: region_plan.regions.len(),
            region_classifications,
            blocker: None,
            arrangement_readiness: None,
            coplanar_volumetric_evidence: None,
        });
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(open_surface_support) =
            certified_open_surface_disjoint_support_from_graph(&graph, left, right, operation)
    {
        return Ok(certified_shortcut_preflight(
            operation,
            open_surface_support,
        ));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(convex_support) =
            certified_convex_boolean_support_from_graph(&graph, left, right, operation)?
    {
        return Ok(certified_shortcut_preflight(operation, convex_support));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(contained_support) = certified_contained_boundary_difference_support_from_graph(
            &graph, left, right, operation,
        )
        .or_else(|| {
            certified_contained_boundary_containment_support_from_graph(
                &graph, left, right, operation,
            )
        })
    {
        return Ok(certified_shortcut_preflight(operation, contained_support));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(boundary_support) = certified_closed_boundary_only_contact_support_from_graph(
            &graph, left, right, operation,
        )?
    {
        return Ok(certified_shortcut_preflight(operation, boundary_support));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(boundary_support) =
            certified_closed_boundary_touching_support_from_graph(&graph, left, right, operation)?
    {
        return Ok(certified_shortcut_preflight(operation, boundary_support));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && operation == ExactBooleanOperation::Intersection
        && has_empty_axis_aligned_orthogonal_solid_intersection(left, right)?
    {
        return Ok(certified_shortcut_preflight(
            operation,
            ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellIntersection,
        ));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(winding_support) =
            certified_winding_boolean_support_from_graph(&graph, left, right)?
    {
        return Ok(certified_shortcut_preflight(operation, winding_support));
    }
    if let Some((support, region_classifications, _triangulations)) =
        open_surface_arrangement_plan_from_graph(&graph, left, right, operation)?
    {
        let region_count = unique_classified_region_count(&region_classifications);
        return Ok(ExactBooleanPreflight {
            operation,
            support,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count,
            region_classifications,
            blocker: None,
            arrangement_readiness: None,
            coplanar_volumetric_evidence: None,
        });
    }
    let boundary_report = boundary_touching_report_from_graph(&graph, left, right)?;
    if boundary_report.is_certified() {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::RequiresBoundaryPolicy,
            graph_had_unknowns: boundary_report.graph_had_unknowns,
            retained_face_pairs: boundary_report.retained_face_pairs,
            retained_events: boundary_report.retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(boundary_report.blocker),
            arrangement_readiness: None,
            coplanar_volumetric_evidence: None,
        });
    }
    let planar_report = planar_arrangement_report_from_graph(&graph, left, right, operation)?;
    if planar_report.is_required() {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::RequiresPlanarArrangement,
            graph_had_unknowns: planar_report.graph_had_unknowns,
            retained_face_pairs: planar_report.retained_face_pairs,
            retained_events: planar_report.retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(planar_report.blocker),
            arrangement_readiness: planar_report.arrangement_readiness,
            coplanar_volumetric_evidence: None,
        });
    }
    let eager_axis_aligned_cell_support = match operation {
        ExactBooleanOperation::Union if has_axis_aligned_box_cell_union(left, right) => {
            Some(ExactBooleanSupport::CertifiedAxisAlignedBoxCellUnion)
        }
        ExactBooleanOperation::Difference if has_axis_aligned_box_cell_difference(left, right) => {
            Some(ExactBooleanSupport::CertifiedAxisAlignedBoxCellDifference)
        }
        _ => None,
    };
    if let Some(support) = eager_axis_aligned_cell_support {
        return Ok(ExactBooleanPreflight {
            operation,
            support,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: None,
            arrangement_readiness: None,
            coplanar_volumetric_evidence: None,
        });
    }
    if let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation)
        && has_axis_aligned_orthogonal_solid_cells(left, right, solid_operation)
    {
        return Ok(ExactBooleanPreflight {
            operation,
            support: axis_aligned_orthogonal_solid_support(solid_operation),
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: None,
            arrangement_readiness: None,
            coplanar_volumetric_evidence: None,
        });
    }
    if let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        &graph,
        left,
        right,
        operation,
        ValidationPolicy::CLOSED,
        VolumetricWindingMaterializationFailure::ReturnNone,
    )? {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::CertifiedWindingMaterialized,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: materialized.triangulations.len(),
            region_classifications: materialized.region_classifications,
            blocker: None,
            arrangement_readiness: None,
            coplanar_volumetric_evidence: coplanar_volumetric_evidence_if_required(
                &graph, left, right,
            ),
        });
    }
    if certified_boolmesh_split_support_from_graph(&graph, left, right, operation) {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::CertifiedBoolMeshSplit,
            graph_had_unknowns: false,
            retained_face_pairs: 0,
            retained_events: 0,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: None,
            arrangement_readiness: None,
            coplanar_volumetric_evidence: None,
        });
    }
    if graph_requires_coplanar_volumetric_cells_for_sources(&graph, left, right) {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::RequiresCoplanarVolumetricCells,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(
                relation_counts.into_blocker(ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells),
            ),
            arrangement_readiness: None,
            coplanar_volumetric_evidence: coplanar_volumetric_evidence_if_required(
                &graph, left, right,
            ),
        });
    }
    if support == ExactBooleanSupport::RequiresBoundaryPolicy {
        return Ok(ExactBooleanPreflight {
            operation,
            support,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(
                relation_counts.into_blocker(ExactBooleanBlockerKind::NeedsBoundaryPolicy),
            ),
            arrangement_readiness: None,
            coplanar_volumetric_evidence: None,
        });
    }

    let winding_report = winding_readiness_report_from_graph(&graph, left, right, operation)?;

    Ok(ExactBooleanPreflight {
        operation,
        support,
        graph_had_unknowns: winding_report.graph_had_unknowns,
        retained_face_pairs: winding_report.retained_face_pairs,
        retained_events: winding_report.retained_events,
        region_count: winding_report.region_count,
        region_classifications: winding_report.region_classifications,
        blocker: Some(winding_report.blocker),
        arrangement_readiness: None,
        coplanar_volumetric_evidence: winding_report.coplanar_volumetric_evidence,
    })
}

fn preflight_direct_coplanar_surface_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    match operation {
        ExactBooleanOperation::Union => {
            if certify_coplanar_convex_surface_equivalence(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceEquivalence);
            }
            if certify_coplanar_convex_surface_containment(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceContainment);
            }
            if arrange_coplanar_convex_surface_union(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion);
            }
            if arrange_coplanar_convex_surface_component_union(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion);
            }
            if arrange_coplanar_surface_component_union(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion);
            }
            if arrange_coplanar_surface_component_holed_union(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion);
            }
            if arrange_coplanar_surface_multi_component_union(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfaceMultiUnion);
            }
            if arrange_coplanar_convex_surface_multi_union(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiUnion);
            }
            if arrange_coplanar_surface_point_touch_union(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchUnion);
            }
            if arrange_coplanar_orthogonal_surface_union(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceUnion);
            }
            if arrange_coplanar_affine_surface_union(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarAffineSurfaceUnion);
            }
            None
        }
        ExactBooleanOperation::Intersection => {
            if certify_coplanar_convex_surface_equivalence(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceEquivalence);
            }
            if certify_coplanar_convex_surface_containment(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceContainment);
            }
            if arrange_coplanar_convex_surface_intersection(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceIntersection);
            }
            if arrange_coplanar_convex_surface_multi_intersection(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceIntersection);
            }
            if arrange_coplanar_surface_component_intersection(left, right).is_some()
                || arrange_coplanar_surface_multi_component_intersection(left, right).is_some()
            {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection);
            }
            if arrange_coplanar_surface_component_holed_intersection(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection);
            }
            if arrange_coplanar_orthogonal_surface_intersection(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceIntersection);
            }
            if arrange_coplanar_affine_surface_intersection(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarAffineSurfaceIntersection);
            }
            if certify_coplanar_surface_boundary_touch(left, right).is_some() {
                return Some(
                    ExactBooleanSupport::CertifiedCoplanarSurfaceBoundaryTouchIntersection,
                );
            }
            if arrange_coplanar_surface_point_touch_union(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchIntersection);
            }
            None
        }
        ExactBooleanOperation::Difference => {
            if certify_coplanar_convex_surface_equivalence(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceEquivalence);
            }
            if certify_coplanar_surface_boundary_touch(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfaceBoundaryTouchDifference);
            }
            if arrange_coplanar_surface_point_touch_union(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference);
            }
            if arrange_coplanar_convex_surface_difference(left, right).is_some() {
                return Some(
                    ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementDifference,
                );
            }
            if arrange_coplanar_convex_surface_multi_difference(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiDifference);
            }
            if arrange_coplanar_surface_component_difference(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference);
            }
            if arrange_coplanar_surface_multi_difference(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference);
            }
            if arrange_coplanar_surface_side_cutter_difference(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfaceSideCutterDifference);
            }
            if arrange_coplanar_surface_cutter_hole_contact_difference(left, right).is_some() {
                return Some(
                    ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference,
                );
            }
            if arrange_coplanar_convex_surface_holed_difference(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceHoledDifference);
            }
            if arrange_coplanar_convex_surface_multi_holed_difference(left, right).is_some() {
                return Some(
                    ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiHoledDifference,
                );
            }
            if arrange_coplanar_convex_surface_component_holed_difference(left, right).is_some()
                || arrange_coplanar_surface_component_holed_difference(left, right).is_some()
            {
                return Some(
                    ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference,
                );
            }
            if has_non_axis_aligned_affine_surface_difference(left, right) {
                return Some(ExactBooleanSupport::CertifiedCoplanarAffineSurfaceDifference);
            }
            if arrange_coplanar_surface_point_touch_difference(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference);
            }
            if arrange_coplanar_orthogonal_surface_difference(left, right).is_some() {
                return Some(ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceDifference);
            }
            None
        }
        ExactBooleanOperation::SelectedRegions(_) => None,
    }
}

fn preflight_tail_shortcut_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    match operation {
        ExactBooleanOperation::Difference if has_axis_aligned_box_multi_difference(left, right) => {
            Some(ExactBooleanSupport::CertifiedAxisAlignedBoxMultiDifference)
        }
        ExactBooleanOperation::Difference
            if has_axis_aligned_box_nested_difference(left, right) =>
        {
            Some(ExactBooleanSupport::CertifiedAxisAlignedBoxNestedDifference)
        }
        ExactBooleanOperation::Difference if has_axis_aligned_box_empty_difference(left, right) => {
            Some(ExactBooleanSupport::CertifiedAxisAlignedBoxEmptyDifference)
        }
        ExactBooleanOperation::Union if has_affine_box_union(left, right) => {
            Some(ExactBooleanSupport::CertifiedAffineBoxUnion)
        }
        ExactBooleanOperation::Intersection if has_affine_box_intersection(left, right) => {
            Some(ExactBooleanSupport::CertifiedAffineBoxIntersection)
        }
        ExactBooleanOperation::Difference if has_affine_box_difference(left, right) => {
            Some(ExactBooleanSupport::CertifiedAffineBoxDifference)
        }
        ExactBooleanOperation::Union
            if has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Union,
            ) =>
        {
            Some(ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellUnion)
        }
        ExactBooleanOperation::Intersection
            if has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Intersection,
            ) =>
        {
            Some(ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellIntersection)
        }
        ExactBooleanOperation::Difference
            if has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Difference,
            ) =>
        {
            Some(ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellDifference)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if certify_coplanar_convex_surface_containment(left, right).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedCoplanarConvexSurfaceContainment)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if certified_coplanar_surface_boolean_support(left, right, operation).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedCoplanarSurfaceContainment)
        }
        ExactBooleanOperation::Intersection
            if intersect_single_triangle_coplanar_surfaces(left, right).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection)
        }
        ExactBooleanOperation::Union
            if union_single_triangle_coplanar_surfaces(left, right).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedCoplanarSurfaceConvexUnion)
        }
        ExactBooleanOperation::Union
            if arrange_single_triangle_coplanar_union(left, right).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion)
        }
        ExactBooleanOperation::Difference
            if difference_single_triangle_coplanar_surfaces(left, right).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedCoplanarSurfaceCornerDifference)
        }
        ExactBooleanOperation::Difference
            if arrange_single_triangle_coplanar_difference(left, right).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference)
        }
        ExactBooleanOperation::Difference
            if arrange_single_triangle_coplanar_holed_difference(left, right).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedCoplanarSurfaceHoledDifference)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => {
            certified_convex_intersection_support(left, right, operation)
                .or_else(|| certified_convex_single_cap_difference_support(left, right, operation))
        }
        ExactBooleanOperation::SelectedRegions(_) => None,
    }
}

fn certified_shortcut_preflight(
    operation: ExactBooleanOperation,
    support: ExactBooleanSupport,
) -> ExactBooleanPreflight {
    ExactBooleanPreflight {
        operation,
        support,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: None,
        arrangement_readiness: None,
        coplanar_volumetric_evidence: None,
    }
}

fn certified_boolmesh_split_support_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> bool {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return false;
    }
    let workspace = exact_boolmesh_workspace_from_graph_for_support(left, right, operation, graph);
    workspace.validate().is_ok() && workspace.is_certified_split_boolean45()
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct GraphRelationCounts {
    candidate_pairs: usize,
    coplanar_overlapping_pairs: usize,
    coplanar_touching_pairs: usize,
    unknown_pairs: usize,
    construction_failed_events: usize,
}

impl GraphRelationCounts {
    const fn into_blocker(self, kind: ExactBooleanBlockerKind) -> ExactBooleanBlocker {
        ExactBooleanBlocker {
            kind,
            candidate_pairs: self.candidate_pairs,
            coplanar_overlapping_pairs: self.coplanar_overlapping_pairs,
            coplanar_touching_pairs: self.coplanar_touching_pairs,
            unknown_pairs: self.unknown_pairs,
            construction_failed_events: self.construction_failed_events,
        }
    }
}

fn graph_relation_counts(graph: &super::graph::ExactIntersectionGraph) -> GraphRelationCounts {
    let mut counts = GraphRelationCounts::default();
    for pair in &graph.face_pairs {
        let pair_has_unknown_event = pair
            .events
            .iter()
            .any(|event| matches!(event, super::graph::IntersectionEvent::Unknown));
        match pair.relation {
            MeshFacePairRelation::Candidate => counts.candidate_pairs += 1,
            MeshFacePairRelation::CoplanarOverlapping => counts.coplanar_overlapping_pairs += 1,
            MeshFacePairRelation::CoplanarTouching => counts.coplanar_touching_pairs += 1,
            MeshFacePairRelation::Unknown => counts.unknown_pairs += 1,
            MeshFacePairRelation::BoundsDisjoint | MeshFacePairRelation::PlaneSeparated => {}
        }
        if pair.relation != MeshFacePairRelation::Unknown && pair_has_unknown_event {
            counts.unknown_pairs += 1;
        }
        counts.construction_failed_events += pair
            .events
            .iter()
            .filter(|event| {
                matches!(
                    event,
                    super::graph::IntersectionEvent::SegmentPlane {
                        relation: super::construction::SegmentPlaneRelation::ConstructionFailed,
                        ..
                    }
                )
            })
            .count();
    }
    counts
}

fn unique_classified_region_count(classifications: &[FaceRegionPlaneClassification]) -> usize {
    let mut unique = Vec::new();
    for classification in classifications {
        let key = (classification.region_side, classification.region_face);
        if !unique.contains(&key) {
            unique.push(key);
        }
    }
    unique.len()
}

fn graph_requires_boundary_policy(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, MeshError> {
    if graph_has_only_coplanar_touching_pairs(graph) {
        return Ok(true);
    }
    if !graph_has_only_boundary_contact_pairs(graph, left, right) {
        return Ok(false);
    }
    certified_closed_boundary_contact(left, right)
}

fn graph_has_only_coplanar_touching_pairs(graph: &super::graph::ExactIntersectionGraph) -> bool {
    !graph.face_pairs.is_empty()
        && graph
            .face_pairs
            .iter()
            .all(|pair| pair.relation == MeshFacePairRelation::CoplanarTouching)
}

fn graph_has_only_coplanar_contact_pairs(graph: &super::graph::ExactIntersectionGraph) -> bool {
    !graph.face_pairs.is_empty()
        && graph.face_pairs.iter().all(|pair| {
            matches!(
                pair.relation,
                MeshFacePairRelation::CoplanarTouching | MeshFacePairRelation::CoplanarOverlapping
            )
        })
        && graph
            .face_pairs
            .iter()
            .any(|pair| pair.relation == MeshFacePairRelation::CoplanarOverlapping)
}

fn graph_requires_planar_arrangement(graph: &super::graph::ExactIntersectionGraph) -> bool {
    graph_has_only_coplanar_contact_pairs(graph)
}

fn graph_requires_coplanar_volumetric_cells(counts: &GraphRelationCounts) -> bool {
    // Coplanar source-face cells inside a closed volumetric overlap are not a
    // planar-surface output problem and not ordinary non-coplanar winding
    // state instead of approximating the cells or relabeling them as generic
    // winding readiness.
    counts.coplanar_overlapping_pairs + counts.coplanar_touching_pairs > 0
}

fn graph_requires_coplanar_volumetric_cells_for_sources(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    let counts = graph_relation_counts(graph);
    if !graph_requires_coplanar_volumetric_cells(&counts) {
        return false;
    }
    // This is the source-aware replacement for the coarse relation-count gate
    // above. A positive-area coplanar face pair is not automatically a
    // volumetric-cell blocker: opposite-side shared faces are boundary contact,
    // while same-side or undecided positive-area overlap needs the missing
    // coplanar volumetric-cell materializer. Keeping the decision in
    // consume replayable exact object evidence, not aggregate counters.
    CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right)
        .obstacle
        .requires_coplanar_volumetric_cells()
}

fn coplanar_volumetric_evidence_if_required(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarVolumetricCellEvidenceReport> {
    let counts = graph_relation_counts(graph);
    if !graph_requires_coplanar_volumetric_cells(&counts) {
        return None;
    }
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    evidence
        .obstacle
        .requires_coplanar_volumetric_cells()
        .then_some(evidence)
}

fn graph_has_only_boundary_contact_pairs(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    !graph.face_pairs.is_empty()
        && graph
            .face_pairs
            .iter()
            .all(|pair| boundary_contact_pair_shape(pair, left, right))
}

fn boundary_contact_pair_shape(pair: &FacePairEvents, left: &ExactMesh, right: &ExactMesh) -> bool {
    match pair.relation {
        MeshFacePairRelation::CoplanarTouching | MeshFacePairRelation::CoplanarOverlapping => true,
        MeshFacePairRelation::Candidate => pair
            .events
            .iter()
            .all(|event| boundary_contact_candidate_event(event, left, right)),
        MeshFacePairRelation::BoundsDisjoint
        | MeshFacePairRelation::PlaneSeparated
        | MeshFacePairRelation::Unknown => false,
    }
}

fn boundary_contact_candidate_event(
    event: &IntersectionEvent,
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    // Positive-area coplanar contact between closed solids also retains
    // adjacent non-coplanar face pairs where an endpoint or coplanar source
    // edge lies on the opposite plane. Those are still boundary facts, not
    // distinction instead of collapsing every retained candidate into the
    // same unsupported topology bucket.
    match event {
        IntersectionEvent::SegmentPlane { relation, .. } => {
            matches!(
                relation,
                SegmentPlaneRelation::Disjoint
                    | SegmentPlaneRelation::Coplanar
                    | SegmentPlaneRelation::EndpointOnPlane
            ) || (*relation == SegmentPlaneRelation::ProperCrossing
                && proper_crossing_outside_plane_face(event, left, right))
        }
        IntersectionEvent::CoplanarEdge { relation, .. } => {
            *relation != SegmentIntersection::Disjoint
        }
        IntersectionEvent::CoplanarVertex { location, .. } => matches!(
            location,
            TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
        ),
        IntersectionEvent::Unknown => false,
    }
}

fn proper_crossing_outside_plane_face(
    event: &IntersectionEvent,
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    let IntersectionEvent::SegmentPlane {
        relation: SegmentPlaneRelation::ProperCrossing,
        plane_side,
        plane_face,
        point: Some(point),
        ..
    } = event
    else {
        return false;
    };
    let Some(triangle) = triangle_points(mesh_for_side(*plane_side, left, right), *plane_face)
    else {
        return false;
    };
    let Some(projection) = choose_triangle_projection(&triangle) else {
        return false;
    };
    // A segment/supporting-plane crossing outside the finite opposite triangle
    // is retained construction evidence, but it is not a surface crossing.
    // this distinction exactly instead of treating every proper plane crossing
    // as volume overlap.
    classify_point_triangle(
        &project_point3(&triangle[0], projection),
        &project_point3(&triangle[1], projection),
        &project_point3(&triangle[2], projection),
        &project_point3(point, projection),
    )
    .value()
        == Some(TriangleLocation::Outside)
}

fn mesh_for_side<'a>(side: MeshSide, left: &'a ExactMesh, right: &'a ExactMesh) -> &'a ExactMesh {
    match side {
        MeshSide::Left => left,
        MeshSide::Right => right,
    }
}

fn triangle_points(mesh: &ExactMesh, face: usize) -> Option<[Point3; 3]> {
    let triangle = mesh.triangles().get(face)?.0;
    Some([
        mesh.vertices().get(triangle[0])?.clone(),
        mesh.vertices().get(triangle[1])?.clone(),
        mesh.vertices().get(triangle[2])?.clone(),
    ])
}

fn choose_triangle_projection(points: &[Point3; 3]) -> Option<CoplanarProjection> {
    [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ]
    .into_iter()
    .find(|&projection| {
        let area = projected_polygon_area2_value(points, projection);
        !matches!(real_sign(&area), Some(Sign::Zero) | None)
    })
}

fn real_sign(value: &Real) -> Option<Sign> {
    match compare_reals(value, &Real::from(0)).value()? {
        Ordering::Less => Some(Sign::Negative),
        Ordering::Equal => Some(Sign::Zero),
        Ordering::Greater => Some(Sign::Positive),
    }
}

fn both_axis_aligned_boxes(left: &ExactMesh, right: &ExactMesh) -> bool {
    is_axis_aligned_box(left) && is_axis_aligned_box(right)
}

fn non_box_full_face_adjacency(left: &ExactMesh, right: &ExactMesh) -> bool {
    !both_axis_aligned_boxes(left, right) && has_full_face_adjacent_union(left, right)
}

fn certified_closed_boundary_contact(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, MeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(false);
    }

    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    left_in_right.validate().map_err(winding_error)?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left.validate().map_err(winding_error)?;

    Ok(mesh_vertices_are_boundary_or_outside(&left_in_right)
        && mesh_vertices_are_boundary_or_outside(&right_in_left)
        && (mesh_vertices_touch_boundary(&left_in_right)
            || mesh_vertices_touch_boundary(&right_in_left)))
}

fn mesh_vertices_are_boundary_or_outside(report: &ClosedMeshWindingMeshReport) -> bool {
    report.target_closed
        && report.vertices.iter().all(|vertex| {
            matches!(
                vertex.relation,
                ClosedMeshWindingRelation::Outside | ClosedMeshWindingRelation::Boundary
            )
        })
}

fn mesh_vertices_touch_boundary(report: &ClosedMeshWindingMeshReport) -> bool {
    report
        .vertices
        .iter()
        .any(|vertex| vertex.relation == ClosedMeshWindingRelation::Boundary)
}

/// Run an exact boolean operation request.
///
/// This entry point makes unsupported named booleans explicit rather than
/// silently dispatching to legacy tolerance code. That is a deliberate
/// exact computation boundary: unsupported topology semantics are diagnostics,
/// not approximate decisions.
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
/// boundary-only contact: coplanar touching, closed-solid coplanar boundary
/// overlap, or closed-solid edge/vertex contact whose retained candidate
/// events have no proper crossings, construction failures, or unknowns. In
/// that narrow case, [`ExactBoundaryBooleanPolicy::PreserveSeparateShells`]
/// projects lower-dimensional contact into triangle-mesh output instead of
/// silently invoking the legacy tolerance path. Closed-solid regularized
/// intersection and difference do not need that projection policy once the
/// same exact boundary-touch report proves no shared interior volume; those
/// two operations use certified shortcuts before the policy layer.
pub fn boolean_exact_with_boundary_policy(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    if let ExactBooleanOperation::SelectedRegions(selection) = operation {
        return boolean_selected_regions(
            left,
            right,
            ExactBooleanPolicy {
                selection,
                validation,
                reject_unknowns: true,
            },
        );
    }
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return boolean_empty_operand(left, right, operation, validation);
    }
    if meshes_are_certified_bounds_disjoint(left, right) {
        return boolean_disjoint_meshes(left, right, operation, validation);
    }
    if meshes_are_certified_identical(left, right) {
        return boolean_identical_meshes(left, operation, validation);
    }
    if meshes_are_certified_same_surface(left, right) {
        return boolean_same_surface_meshes(left, operation, validation);
    }
    if let Some(result) =
        boolean_axis_aligned_box_operation_optional(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if let Some(result) = boolean_arrangement_cell_complex_meshes(
        left,
        right,
        operation,
        validation,
        ArrangementCellComplexDispatch::PreemptLegacy,
    )? {
        return Ok(result);
    }
    if let Some(result) = boolean_direct_adjacency_meshes(left, right, operation, validation)? {
        return Ok(result);
    }
    if let Some(result) =
        boolean_coplanar_mesh_overlay_optional(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if let Some(result) =
        boolean_direct_coplanar_surface_meshes(left, right, operation, validation)?
    {
        return Ok(result);
    }

    match operation {
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => {
            if let Some(result) = boolean_axis_aligned_box_special_difference_optional(
                left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) = boolean_affine_box_optional(left, right, operation, validation)? {
                return Ok(result);
            }
            if let Some(result) =
                boolean_affine_orthogonal_solid_optional(left, right, operation, validation)?
            {
                return Ok(result);
            }

            let graph = build_intersection_graph(left, right)?;
            validate_graph_source_handoff(&graph, left, right)?;
            match operation {
                ExactBooleanOperation::Union => {
                    if let Some(report) =
                        certified_closed_boundary_touching_union_report_from_graph(
                            &graph, left, right,
                        )?
                    {
                        return boolean_closed_boundary_touching_union(
                            &graph, left, right, validation, report,
                        );
                    }
                }
                ExactBooleanOperation::Intersection => {
                    if let Some(report) =
                        certified_closed_boundary_touching_regularized_report_from_graph(
                            &graph, left, right,
                        )?
                    {
                        return boolean_closed_boundary_touching_intersection(
                            &graph, left, right, validation, report,
                        );
                    }
                }
                ExactBooleanOperation::Difference => {
                    if let Some(report) =
                        certified_closed_boundary_touching_regularized_report_from_graph(
                            &graph, left, right,
                        )?
                    {
                        return boolean_closed_boundary_touching_difference(
                            &graph, left, right, validation, report,
                        );
                    }
                }
                ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled above"),
            }
            if let Some(containment) = certify_coplanar_convex_surface_containment(left, right) {
                return boolean_coplanar_convex_containment_surfaces(
                    left,
                    right,
                    operation,
                    validation,
                    containment,
                );
            }
            let single_triangle_containment =
                certify_single_triangle_coplanar_containment(left, right).is_some();
            if operation == ExactBooleanOperation::Intersection
                && !single_triangle_containment
                && let Some(result) =
                    boolean_coplanar_surface_intersection(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_containment(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if operation != ExactBooleanOperation::Intersection
                && let Some(result) =
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
                boolean_coplanar_surface_arrangement_union(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_corner_difference(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_arrangement_difference(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_surface_holed_difference(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) = boolean_coplanar_cutter_hole_contact_difference_optional(
                left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_convex_multi_holed_difference(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) = boolean_coplanar_convex_component_holed_difference_optional(
                left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_orthogonal_surface_optional(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_coplanar_affine_surface_optional(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) = boolean_open_surface_disjoint_or_arrangement_meshes_from_graph(
                &graph, left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) = boolean_convex_containment_meshes_from_graph(
                &graph, left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) = boolean_contained_boundary_difference_meshes_from_graph(
                &graph, left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) = boolean_contained_boundary_containment_meshes_from_graph(
                &graph, left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) =
                boolean_convex_intersection_meshes(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_convex_single_cap_difference_meshes(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if matches!(
                operation,
                ExactBooleanOperation::Union | ExactBooleanOperation::Difference
            ) && let Some(result) =
                boolean_axis_aligned_box_cell_meshes(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) = boolean_axis_aligned_orthogonal_solid_cell_meshes(
                left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) = boolean_retained_graph_fallback_meshes_from_graph(
                &graph,
                left,
                right,
                operation,
                validation,
                boundary_policy,
            )? {
                return Ok(result);
            }
            if let Some(result) = boolean_arrangement_cell_complex_meshes(
                left,
                right,
                operation,
                validation,
                ArrangementCellComplexDispatch::Fallback,
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

fn copied_shortcut_result_from_mesh(
    mesh: &ExactMesh,
    label: &'static str,
    shortcut: ExactBooleanShortcutKind,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = copy_mesh(mesh, label, validation)?;
    Ok(certified_shortcut_result(mesh, shortcut))
}

fn boolean_coplanar_surface_boundary_touch_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    if let Some(result) =
        boolean_boolmesh_split_meshes(left, right, ExactBooleanOperation::Intersection, validation)?
    {
        return Ok(result);
    }
    let mesh = empty_mesh(
        "empty exact coplanar boundary-touch surface intersection",
        validation,
    )?;
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceBoundaryTouchIntersection,
    ))
}

fn boolean_coplanar_surface_boundary_touch_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    if let Some(result) =
        boolean_boolmesh_split_meshes(left, right, ExactBooleanOperation::Difference, validation)?
    {
        return Ok(result);
    }
    let mesh = copy_mesh(
        left,
        "exact coplanar boundary-touch surface difference keeps left",
        validation,
    )?;
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceBoundaryTouchDifference,
    ))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ArrangementCellComplexDispatch {
    PreemptLegacy,
    Fallback,
}

enum ArrangementCellComplexOutcome {
    Materialized(ExactBooleanResult, ExactArrangementBooleanAttempt),
    Declined(ExactArrangementBooleanAttempt),
}

/// Report how far the arrangement/cell-complex Boolean pipeline gets for an
/// operation without falling through to legacy/specialized materializers.
pub fn exact_arrangement_boolean_attempt_report(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    policy: ExactRegularizationPolicy,
) -> Result<ExactArrangementBooleanAttempt, MeshError> {
    Ok(
        match run_arrangement_cell_complex_attempt(
            left,
            right,
            operation,
            policy,
            Some(ValidationPolicy::ALLOW_BOUNDARY),
        )? {
            ArrangementCellComplexOutcome::Materialized(_, attempt)
            | ArrangementCellComplexOutcome::Declined(attempt) => attempt,
        },
    )
}

fn boolean_arrangement_cell_complex_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    dispatch: ArrangementCellComplexDispatch,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if dispatch == ArrangementCellComplexDispatch::PreemptLegacy
        && !arrangement_cell_complex_should_preempt_legacy_paths(left, right, operation)
    {
        return Ok(None);
    }

    match run_arrangement_cell_complex_attempt(
        left,
        right,
        operation,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
        Some(validation),
    )? {
        ArrangementCellComplexOutcome::Materialized(result, _) => Ok(Some(result)),
        ArrangementCellComplexOutcome::Declined(_)
            if dispatch == ArrangementCellComplexDispatch::Fallback =>
        {
            match run_arrangement_cell_complex_attempt(
                left,
                right,
                operation,
                ExactRegularizationPolicy::RETAIN_ARTIFACTS,
                Some(validation),
            )? {
                ArrangementCellComplexOutcome::Materialized(result, _) => Ok(Some(result)),
                ArrangementCellComplexOutcome::Declined(_) => Ok(None),
            }
        }
        ArrangementCellComplexOutcome::Declined(_) => Ok(None),
    }
}

fn run_arrangement_cell_complex_attempt(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    policy: ExactRegularizationPolicy,
    validation: Option<ValidationPolicy>,
) -> Result<ArrangementCellComplexOutcome, MeshError> {
    let arrangement = ExactArrangement::from_meshes_with_policy(left, right, policy)?;
    let mut attempt = ExactArrangementBooleanAttempt {
        operation,
        policy,
        stage: ExactArrangementBooleanStage::ArrangementBuilt,
        decline: None,
        arrangement_blockers: arrangement.blockers.len(),
        face_cells: arrangement.face_cells.len(),
        regions: arrangement
            .shells_or_regions
            .as_ref()
            .map_or(0, |regions| regions.len()),
        selected_faces: 0,
        output_vertices: 0,
        output_triangles: 0,
    };

    if !arrangement.is_complete() {
        if let Some(result) = materialize_simple_coplanar_overlay_arrangement(
            left,
            right,
            operation,
            validation,
            &arrangement,
        )? {
            attempt.stage = ExactArrangementBooleanStage::Materialized;
            attempt.output_vertices = result.mesh.vertices().len();
            attempt.output_triangles = result.mesh.triangles().len();
            return Ok(ArrangementCellComplexOutcome::Materialized(result, attempt));
        }
        attempt.decline = Some(ExactArrangementBooleanDecline::ArrangementBlockers(
            arrangement.blockers.clone(),
        ));
        return Ok(ArrangementCellComplexOutcome::Declined(attempt));
    }

    let labeled = match arrangement.label_regions(policy) {
        Ok(labeled) => labeled,
        Err(blocker) => {
            attempt.decline = Some(ExactArrangementBooleanDecline::Labeling(blocker));
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Labeled;
    let selected = match labeled.select_with_policy(operation, policy) {
        Ok(selected) if selected.blockers.is_empty() => selected,
        Ok(selected) => {
            attempt.selected_faces = selected.selected_faces.len();
            attempt.decline = Some(ExactArrangementBooleanDecline::Selection(
                selected.blockers[0].clone(),
            ));
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
        Err(blocker) => {
            attempt.decline = Some(ExactArrangementBooleanDecline::Selection(blocker));
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Selected;
    attempt.selected_faces = selected.selected_faces.len();
    let simplified = match selected.simplify_exact_with_policy(policy) {
        Ok(simplified) if simplified.blockers.is_empty() => simplified,
        Ok(simplified) => {
            attempt.decline = Some(ExactArrangementBooleanDecline::Simplification(
                simplified.blockers[0].clone(),
            ));
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
        Err(blocker) => {
            attempt.decline = Some(ExactArrangementBooleanDecline::Simplification(blocker));
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Simplified;
    let mesh = match simplified.triangulate() {
        Ok(mesh) => mesh,
        Err(blocker) => {
            attempt.decline = Some(ExactArrangementBooleanDecline::Triangulation(blocker));
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Triangulated;
    attempt.output_vertices = mesh.vertices().len();
    attempt.output_triangles = mesh.triangles().len();
    let Some(validation) = validation else {
        return Ok(ArrangementCellComplexOutcome::Declined(attempt));
    };
    let mesh = match copy_mesh(
        &mesh,
        "exact arrangement cell-complex boolean result",
        validation,
    ) {
        Ok(mesh) => mesh,
        Err(_) => {
            attempt.decline = Some(ExactArrangementBooleanDecline::OutputValidation);
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Materialized;
    Ok(ArrangementCellComplexOutcome::Materialized(
        certified_shortcut_result(mesh, ExactBooleanShortcutKind::ArrangementCellComplex),
        attempt,
    ))
}

fn materialize_simple_coplanar_overlay_arrangement(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: Option<ValidationPolicy>,
    arrangement: &ExactArrangement,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if arrangement.carrier_plane_overlays.len() != 1
        || !arrangement.face_plane_arrangements.is_empty()
        || !arrangement
            .carrier_plane_overlays
            .iter()
            .all(|overlay| overlay.overlay.is_complete())
    {
        return Ok(None);
    }
    let Some(validation) = validation else {
        return Ok(None);
    };
    let overlay = &arrangement.carrier_plane_overlays[0];
    let operation = match operation {
        ExactBooleanOperation::Union => ExactArrangement2dSetOperation::Union,
        ExactBooleanOperation::Intersection => ExactArrangement2dSetOperation::Intersection,
        ExactBooleanOperation::Difference => ExactArrangement2dSetOperation::Difference,
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    let left_ring = projected_mesh_face_ring(
        ExactArrangement2dRegion::Left,
        left,
        overlay.left_face,
        overlay.projection,
    );
    let right_ring = projected_mesh_face_ring(
        ExactArrangement2dRegion::Right,
        right,
        overlay.right_face,
        overlay.projection,
    );
    let (Some(left_ring), Some(right_ring)) = (left_ring, right_ring) else {
        return Ok(None);
    };
    let requested_overlay = build_exact_arrangement2d_overlay(&[left_ring, right_ring], operation);
    if !requested_overlay.is_complete()
        || !requested_overlay.faces.iter().any(|face| face.selected)
        || requested_overlay.output_loops.is_empty()
    {
        return Ok(None);
    }

    let carrier = left.triangles()[overlay.left_face].0;
    let carrier_points = [
        left.vertices()[carrier[0]].clone(),
        left.vertices()[carrier[1]].clone(),
        left.vertices()[carrier[2]].clone(),
    ];
    let Some(mesh) = mesh_from_projected_overlay_loops(
        &requested_overlay.output_loops,
        &carrier_points,
        overlay.projection,
        "exact coplanar overlay arrangement",
    ) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &mesh,
        "exact coplanar overlay arrangement boolean result",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    )))
}

#[derive(Clone, Debug)]
struct MaterializedProjectedLoop {
    points: Vec<Point2>,
    lifted: Vec<Point3>,
    area_ordering: Ordering,
    signed_area_twice: Real,
}

#[derive(Clone, Debug)]
struct MaterializedProjectedComponent {
    outer: usize,
    holes: Vec<usize>,
}

fn boolean_coplanar_mesh_overlay_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if !coplanar_mesh_overlay_should_preempt_surface_paths(left, right, operation) {
        return Ok(None);
    }
    let operation = match operation {
        ExactBooleanOperation::Union => ExactArrangement2dSetOperation::Union,
        ExactBooleanOperation::Intersection => ExactArrangement2dSetOperation::Intersection,
        ExactBooleanOperation::Difference => ExactArrangement2dSetOperation::Difference,
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    let Some((carrier_points, projection)) = coplanar_mesh_overlay_carrier(left, right) else {
        return Ok(None);
    };
    let mut rings = Vec::with_capacity(left.triangles().len() + right.triangles().len());
    let Some(left_rings) =
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Left, left, projection)
    else {
        return Ok(None);
    };
    let Some(right_rings) =
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Right, right, projection)
    else {
        return Ok(None);
    };
    rings.extend(left_rings);
    rings.extend(right_rings);
    let overlay = build_exact_arrangement2d_overlay(&rings, operation);
    if !overlay.is_complete()
        || !overlay.faces.iter().any(|face| face.selected)
        || overlay.output_loops.is_empty()
    {
        return Ok(None);
    }
    let Some(mesh) = mesh_from_projected_overlay_loops(
        &overlay.output_loops,
        &carrier_points,
        projection,
        "exact coplanar mesh overlay arrangement",
    ) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &mesh,
        "exact coplanar mesh overlay arrangement boolean result",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    )))
}

fn mesh_from_projected_overlay_loops(
    output_loops: &[super::arrangement2d::ExactArrangement2dOutputLoop],
    carrier_points: &[Point3; 3],
    projection: CoplanarProjection,
    provenance: &'static str,
) -> Option<ExactMesh> {
    let loops = materialized_projected_overlay_loops(output_loops, carrier_points, projection)?;
    let components = group_projected_overlay_components(&loops)?;
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for component in components {
        let outer = &loops[component.outer];
        let mut projected = outer
            .points
            .iter()
            .map(point2_for_hypertri)
            .collect::<Vec<_>>();
        let mut polygon_points = outer.lifted.clone();
        let mut hole_indices = Vec::with_capacity(component.holes.len());
        for hole_index in component.holes {
            let hole = &loops[hole_index];
            hole_indices.push(projected.len());
            projected.extend(hole.points.iter().map(point2_for_hypertri));
            polygon_points.extend(hole.lifted.iter().cloned());
        }
        let local_to_global = polygon_points
            .iter()
            .map(|point| find_or_insert_exact_vertex(&mut vertices, point))
            .collect::<Option<Vec<_>>>()?;
        let indices = match hypertri::earcut(&projected, &hole_indices) {
            Ok(indices) if !indices.is_empty() && indices.len() % 3 == 0 => indices,
            _ => return None,
        };
        for triangle in indices.chunks_exact(3) {
            triangles.push(Triangle([
                local_to_global[triangle[0]],
                local_to_global[triangle[1]],
                local_to_global[triangle[2]],
            ]));
        }
    }
    if triangles.is_empty() {
        return None;
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(provenance),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn coplanar_mesh_overlay_should_preempt_surface_paths(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> bool {
    if left.facts().mesh.closed_manifold && right.facts().mesh.closed_manifold {
        return false;
    }
    let total_triangles = left.triangles().len() + right.triangles().len();
    if total_triangles <= 2 || total_triangles > 96 {
        return false;
    }
    if certify_coplanar_convex_surface_equivalence(left, right).is_some()
        || certify_coplanar_convex_surface_containment(left, right).is_some()
        || certify_coplanar_surface_mesh_containment(left, right).is_some()
    {
        return false;
    }
    match operation {
        ExactBooleanOperation::Union => {
            if total_triangles > 12
                && arrange_coplanar_surface_component_union(left, right).is_some()
            {
                return false;
            }
            if total_triangles > 24
                && arrange_coplanar_surface_component_holed_union(left, right).is_some()
            {
                return false;
            }
            arrange_coplanar_convex_surface_union(left, right).is_some()
                || arrange_coplanar_convex_surface_component_union(left, right).is_some()
                || arrange_coplanar_convex_surface_multi_union(left, right).is_some()
                || arrange_coplanar_surface_component_union(left, right).is_some()
                || arrange_coplanar_surface_component_holed_union(left, right).is_some()
                || arrange_coplanar_surface_multi_component_union(left, right).is_some()
                || arrange_coplanar_surface_point_touch_union(left, right).is_some()
                || arrange_coplanar_orthogonal_surface_union(left, right).is_some()
                || arrange_coplanar_affine_surface_union(left, right).is_some()
        }
        ExactBooleanOperation::Intersection => {
            if arrange_coplanar_orthogonal_surface_intersection(left, right).is_some()
                && arrange_coplanar_surface_component_holed_intersection(left, right).is_some()
            {
                return false;
            }
            if certify_coplanar_surface_boundary_touch(left, right).is_some()
                || arrange_coplanar_surface_point_touch_union(left, right).is_some()
                || intersect_single_triangle_coplanar_surfaces(left, right).is_some()
            {
                return false;
            }
            arrange_coplanar_convex_surface_intersection(left, right).is_some()
                || arrange_coplanar_convex_surface_multi_intersection(left, right).is_some()
                || arrange_coplanar_surface_component_intersection(left, right).is_some()
                || arrange_coplanar_surface_multi_component_intersection(left, right).is_some()
                || arrange_coplanar_surface_component_holed_intersection(left, right).is_some()
                || arrange_coplanar_affine_surface_intersection(left, right).is_some()
                || arrange_coplanar_orthogonal_surface_intersection(left, right).is_some()
        }
        ExactBooleanOperation::Difference => {
            if certify_coplanar_surface_boundary_touch(left, right).is_some()
                || arrange_coplanar_convex_surface_multi_difference(left, right).is_some()
                || arrange_coplanar_surface_multi_difference(left, right).is_some()
                || arrange_coplanar_surface_side_cutter_difference(left, right).is_some()
                || arrange_coplanar_surface_cutter_hole_contact_difference(left, right).is_some()
                || arrange_coplanar_convex_surface_holed_difference(left, right).is_some()
                || arrange_coplanar_convex_surface_multi_holed_difference(left, right).is_some()
                || arrange_coplanar_convex_surface_component_holed_difference(left, right)
                    .is_some()
                || arrange_coplanar_surface_component_holed_difference(left, right).is_some()
                || arrange_coplanar_orthogonal_surface_difference(left, right).is_some()
                || difference_single_triangle_coplanar_surfaces(left, right).is_some()
                || arrange_single_triangle_coplanar_difference(left, right).is_some()
                || arrange_single_triangle_coplanar_holed_difference(left, right).is_some()
            {
                return false;
            }
            arrange_coplanar_convex_surface_difference(left, right).is_some()
                || arrange_coplanar_surface_component_difference(left, right).is_some()
                || arrange_coplanar_surface_point_touch_difference(left, right).is_some()
                || arrange_coplanar_affine_surface_difference(left, right).is_some()
        }
        ExactBooleanOperation::SelectedRegions(_) => false,
    }
}

fn coplanar_mesh_overlay_carrier(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<([Point3; 3], CoplanarProjection)> {
    let carrier_points =
        first_projectable_triangle(left).or_else(|| first_projectable_triangle(right))?;
    let projection = choose_triangle_projection(&carrier_points)?;
    for mesh in [left, right] {
        for point in mesh.vertices() {
            if orient3d_report(
                &carrier_points[0],
                &carrier_points[1],
                &carrier_points[2],
                point,
            )
            .value()?
                != Sign::Zero
            {
                return None;
            }
        }
        for face in 0..mesh.triangles().len() {
            let ring =
                projected_mesh_face_ring(ExactArrangement2dRegion::Left, mesh, face, projection)?;
            if compare_reals(
                &projected_loop_signed_area_twice(&ring.vertices),
                &Real::from(0),
            )
            .value()?
                == Ordering::Equal
            {
                return None;
            }
        }
    }
    Some((carrier_points, projection))
}

fn first_projectable_triangle(mesh: &ExactMesh) -> Option<[Point3; 3]> {
    for triangle in mesh.triangles() {
        let points = [
            mesh.vertices().get(triangle.0[0])?.clone(),
            mesh.vertices().get(triangle.0[1])?.clone(),
            mesh.vertices().get(triangle.0[2])?.clone(),
        ];
        if choose_triangle_projection(&points).is_some() {
            return Some(points);
        }
    }
    None
}

fn projected_mesh_boundary_rings(
    region: ExactArrangement2dRegion,
    mesh: &ExactMesh,
    projection: CoplanarProjection,
) -> Option<Vec<ExactArrangement2dRegionRing>> {
    order_mesh_boundary_loops(mesh)?
        .into_iter()
        .map(|loop_vertices| {
            let vertices = loop_vertices
                .into_iter()
                .map(|vertex| Some(project_point3(mesh.vertices().get(vertex)?, projection)))
                .collect::<Option<Vec<_>>>()?;
            Some(ExactArrangement2dRegionRing::new(region, vertices))
        })
        .collect()
}

fn materialized_projected_overlay_loops(
    loops: &[super::arrangement2d::ExactArrangement2dOutputLoop],
    carrier_points: &[Point3; 3],
    projection: CoplanarProjection,
) -> Option<Vec<MaterializedProjectedLoop>> {
    let mut materialized = Vec::with_capacity(loops.len());
    for loop_ in loops {
        let loop_points = simplified_projected_loop_points(&loop_.points)?;
        if loop_points.len() < 3 {
            return None;
        }
        let signed_area_twice = projected_loop_signed_area_twice(&loop_points);
        let area_ordering = compare_reals(&signed_area_twice, &Real::from(0)).value()?;
        if area_ordering == Ordering::Equal {
            return None;
        }
        let lifted = loop_points
            .iter()
            .map(|point| lift_projected_point_to_carrier(point, carrier_points, projection))
            .collect::<Option<Vec<_>>>()?;
        materialized.push(MaterializedProjectedLoop {
            points: loop_points,
            lifted,
            area_ordering,
            signed_area_twice,
        });
    }
    Some(materialized)
}

fn group_projected_overlay_components(
    loops: &[MaterializedProjectedLoop],
) -> Option<Vec<MaterializedProjectedComponent>> {
    let mut components = loops
        .iter()
        .enumerate()
        .filter_map(|(index, loop_)| {
            (loop_.area_ordering == Ordering::Greater).then_some(MaterializedProjectedComponent {
                outer: index,
                holes: Vec::new(),
            })
        })
        .collect::<Vec<_>>();
    if components.is_empty() {
        return None;
    }

    for (hole_index, hole) in loops.iter().enumerate() {
        if hole.area_ordering != Ordering::Less {
            continue;
        }
        let witness = hole.points.first()?;
        let mut owner = None::<(usize, Real)>;
        for (component_index, component) in components.iter().enumerate() {
            let outer = &loops[component.outer];
            match classify_point_ring_even_odd(&outer.points, witness).value()? {
                RingPointLocation::Inside => {
                    let replace = owner.as_ref().is_none_or(|(_, owner_area)| {
                        compare_reals(&outer.signed_area_twice, owner_area).value()
                            == Some(Ordering::Less)
                    });
                    if replace {
                        owner = Some((component_index, outer.signed_area_twice.clone()));
                    }
                }
                RingPointLocation::Outside => {}
                RingPointLocation::Boundary => return None,
            }
        }
        let (owner, _) = owner?;
        components[owner].holes.push(hole_index);
    }
    Some(components)
}

fn projected_mesh_face_ring(
    region: ExactArrangement2dRegion,
    mesh: &ExactMesh,
    face: usize,
    projection: CoplanarProjection,
) -> Option<ExactArrangement2dRegionRing> {
    let triangle = mesh.triangles().get(face)?.0;
    let vertices = triangle
        .iter()
        .map(|vertex| Some(project_point3(mesh.vertices().get(*vertex)?, projection)))
        .collect::<Option<Vec<_>>>()?;
    Some(ExactArrangement2dRegionRing::new(region, vertices))
}

fn find_or_insert_exact_vertex(vertices: &mut Vec<Point3>, point: &Point3) -> Option<usize> {
    for (index, existing) in vertices.iter().enumerate() {
        if point3_exact_equal(existing, point)? {
            return Some(index);
        }
    }
    let index = vertices.len();
    vertices.push(point.clone());
    Some(index)
}

fn point3_exact_equal(left: &Point3, right: &Point3) -> Option<bool> {
    Some(
        compare_reals(&left.x, &right.x).value()? == Ordering::Equal
            && compare_reals(&left.y, &right.y).value()? == Ordering::Equal
            && compare_reals(&left.z, &right.z).value()? == Ordering::Equal,
    )
}

fn point2_for_hypertri(point: &Point2) -> hypertri::ExactPoint {
    hypertri::ExactPoint::new(point.x.clone(), point.y.clone())
}

fn projected_loop_signed_area_twice(points: &[Point2]) -> Real {
    let mut area = Real::from(0);
    for index in 0..points.len() {
        let current = &points[index];
        let next = &points[(index + 1) % points.len()];
        area = area + &(current.x.clone() * &next.y) - &(current.y.clone() * &next.x);
    }
    area
}

fn simplified_projected_loop_points(points: &[Point2]) -> Option<Vec<Point2>> {
    let mut simplified = Vec::<Point2>::new();
    for point in points {
        if simplified
            .last()
            .is_some_and(|previous| point2_exact_equal(previous, point) == Some(true))
        {
            continue;
        }
        simplified.push(point.clone());
    }
    if simplified.len() > 1
        && point2_exact_equal(simplified.first().unwrap(), simplified.last().unwrap()) == Some(true)
    {
        simplified.pop();
    }

    let mut changed = true;
    while changed && simplified.len() >= 3 {
        changed = false;
        let mut next = Vec::new();
        for index in 0..simplified.len() {
            let previous = &simplified[(index + simplified.len() - 1) % simplified.len()];
            let current = &simplified[index];
            let following = &simplified[(index + 1) % simplified.len()];
            match projected_turn_is_collinear(previous, current, following) {
                Some(true) => {
                    changed = true;
                }
                Some(false) => next.push(current.clone()),
                None => return None,
            }
        }
        simplified = next;
    }
    Some(simplified)
}

fn point2_exact_equal(left: &Point2, right: &Point2) -> Option<bool> {
    Some(
        compare_reals(&left.x, &right.x).value()? == Ordering::Equal
            && compare_reals(&left.y, &right.y).value()? == Ordering::Equal,
    )
}

fn projected_turn_is_collinear(
    previous: &Point2,
    current: &Point2,
    following: &Point2,
) -> Option<bool> {
    let abx = current.x.clone() - &previous.x;
    let aby = current.y.clone() - &previous.y;
    let bcx = following.x.clone() - &current.x;
    let bcy = following.y.clone() - &current.y;
    let area = abx * &bcy - &(aby * &bcx);
    Some(compare_reals(&area, &Real::from(0)).value()? == Ordering::Equal)
}

fn lift_projected_point_to_carrier(
    point: &Point2,
    carrier: &[Point3; 3],
    projection: CoplanarProjection,
) -> Option<Point3> {
    let projected = [
        project_point3(&carrier[0], projection),
        project_point3(&carrier[1], projection),
        project_point3(&carrier[2], projection),
    ];
    let ux = projected[1].x.clone() - &projected[0].x;
    let uy = projected[1].y.clone() - &projected[0].y;
    let vx = projected[2].x.clone() - &projected[0].x;
    let vy = projected[2].y.clone() - &projected[0].y;
    let wx = point.x.clone() - &projected[0].x;
    let wy = point.y.clone() - &projected[0].y;
    let det = ux.clone() * &vy - &(uy.clone() * &vx);
    let a = ((wx.clone() * &vy - &(wy.clone() * &vx)) / &det).ok()?;
    let b = ((ux * &wy - &(uy * &wx)) / &det).ok()?;
    let p1 = vector_between(&carrier[0], &carrier[1]);
    let p2 = vector_between(&carrier[0], &carrier[2]);
    Some(Point3::new(
        carrier[0].x.clone() + &(p1.x * &a) + &(p2.x * &b),
        carrier[0].y.clone() + &(p1.y * &a) + &(p2.y * &b),
        carrier[0].z.clone() + &(p1.z * &a) + &(p2.z * &b),
    ))
}

fn vector_between(from: &Point3, to: &Point3) -> Point3 {
    Point3::new(
        to.x.clone() - &from.x,
        to.y.clone() - &from.y,
        to.z.clone() - &from.z,
    )
}

fn arrangement_cell_complex_should_preempt_legacy_paths(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> bool {
    (matches!(
        operation,
        ExactBooleanOperation::Union | ExactBooleanOperation::Difference
    ) && non_box_full_face_adjacency(left, right))
        || single_triangle_coplanar_overlay_should_preempt_surface_paths(left, right, operation)
}

fn single_triangle_coplanar_overlay_should_preempt_surface_paths(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> bool {
    if left.triangles().len() != 1
        || right.triangles().len() != 1
        || (left.facts().mesh.closed_manifold && right.facts().mesh.closed_manifold)
    {
        return false;
    }
    if matches!(
        operation,
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
    ) && certify_single_triangle_coplanar_containment(left, right).is_some()
    {
        return false;
    }
    match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Difference => true,
        ExactBooleanOperation::Intersection => {
            intersect_single_triangle_coplanar_surfaces(left, right).is_some()
        }
        ExactBooleanOperation::SelectedRegions(_) => false,
    }
}

fn boolean_convex_intersection_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Intersection {
        return Ok(None);
    }
    let Some(intersection) = intersect_closed_convex_solids(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &intersection.mesh,
        "exact closed-convex solid intersection",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::ConvexIntersection,
    )))
}

fn boolean_coplanar_cutter_hole_contact_difference_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    arrange_coplanar_surface_cutter_hole_contact_difference(left, right)
        .map(|difference| {
            let mesh = copy_mesh(
                &difference.mesh,
                "exact coplanar cutter-hole contact difference",
                validation,
            )?;
            Ok(certified_shortcut_result(
                mesh,
                ExactBooleanShortcutKind::CoplanarSurfaceCutterHoleContactDifference,
            ))
        })
        .transpose()
}

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
        if let Some(intersection) = arrange_coplanar_surface_component_intersection(left, right) {
            let mesh = copy_mesh(
                &intersection.mesh,
                "exact coplanar nonconvex component intersection",
                validation,
            )?;
            return Ok(Some(certified_shortcut_result(
                mesh,
                ExactBooleanShortcutKind::CoplanarSurfaceIntersection,
            )));
        }
        if let Some(intersection) =
            arrange_coplanar_surface_multi_component_intersection(left, right)
        {
            let mesh = copy_mesh(
                &intersection.mesh,
                "exact coplanar nonconvex multi-component intersection",
                validation,
            )?;
            return Ok(Some(certified_shortcut_result(
                mesh,
                ExactBooleanShortcutKind::CoplanarSurfaceIntersection,
            )));
        }
        if let Some(intersection) =
            arrange_coplanar_surface_component_holed_intersection(left, right)
        {
            let mesh = copy_mesh(
                &intersection.mesh,
                "exact coplanar component-holed surface intersection",
                validation,
            )?;
            return Ok(Some(certified_shortcut_result(
                mesh,
                ExactBooleanShortcutKind::CoplanarSurfaceIntersection,
            )));
        }
        return Ok(None);
    };
    let mesh = copy_mesh(
        &intersection.mesh,
        "exact coplanar surface partial-overlap intersection",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceIntersection,
    )))
}

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
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceConvexUnion,
    )))
}

fn boolean_coplanar_surface_arrangement_union(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Union {
        return Ok(None);
    }
    let Some(union) = arrange_single_triangle_coplanar_union(left, right) else {
        if let Some(union) = arrange_coplanar_surface_component_union(left, right) {
            let mesh = copy_mesh(&union.mesh, "exact coplanar component union", validation)?;
            return Ok(Some(certified_shortcut_result(
                mesh,
                ExactBooleanShortcutKind::CoplanarSurfaceArrangementUnion,
            )));
        }
        if let Some(union) = arrange_coplanar_surface_component_holed_union(left, right) {
            let mesh = copy_mesh(
                &union.mesh,
                "exact coplanar component-holed surface union",
                validation,
            )?;
            return Ok(Some(certified_shortcut_result(
                mesh,
                ExactBooleanShortcutKind::CoplanarSurfaceArrangementUnion,
            )));
        }
        return arrange_coplanar_surface_multi_component_union(left, right)
            .map(|union| {
                let mesh = copy_mesh(
                    &union.mesh,
                    "exact coplanar nonconvex multi-component union",
                    validation,
                )?;
                Ok(certified_shortcut_result(
                    mesh,
                    ExactBooleanShortcutKind::CoplanarSurfaceMultiUnion,
                ))
            })
            .or_else(|| {
                arrange_coplanar_surface_point_touch_union(left, right).map(|union| {
                    let mesh = copy_mesh(
                        &union.mesh,
                        "exact coplanar point-touch surface union",
                        validation,
                    )?;
                    Ok(certified_shortcut_result(
                        mesh,
                        ExactBooleanShortcutKind::CoplanarSurfacePointTouchUnion,
                    ))
                })
            })
            .transpose();
    };
    let mesh = copy_mesh(
        &union.mesh,
        "exact planar-arrangement coplanar surface union",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceArrangementUnion,
    )))
}

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
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceCornerDifference,
    )))
}

fn boolean_coplanar_surface_arrangement_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    let Some(difference) = arrange_single_triangle_coplanar_difference(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &difference.mesh,
        "exact planar-arrangement coplanar surface difference",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceArrangementDifference,
    )))
}

fn boolean_coplanar_surface_holed_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    let Some(difference) = arrange_single_triangle_coplanar_holed_difference(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &difference.mesh,
        "exact holed coplanar surface difference",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceHoledDifference,
    )))
}

fn boolean_coplanar_convex_multi_holed_difference(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    let Some(difference) = arrange_coplanar_convex_surface_multi_holed_difference(left, right)
    else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &difference.mesh,
        "exact coplanar convex multi-holed difference",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiHoledDifference,
    )))
}

fn boolean_coplanar_convex_component_holed_difference_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    arrange_coplanar_convex_surface_component_holed_difference(left, right)
        .or_else(|| arrange_coplanar_surface_component_holed_difference(left, right))
        .map(|difference| {
            let mesh = copy_mesh(
                &difference.mesh,
                "exact coplanar component-holed difference",
                validation,
            )?;
            Ok(certified_shortcut_result(
                mesh,
                ExactBooleanShortcutKind::CoplanarConvexSurfaceComponentHoledDifference,
            ))
        })
        .transpose()
}

fn boolean_coplanar_orthogonal_surface_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let arrangement = match operation {
        ExactBooleanOperation::Union => arrange_coplanar_orthogonal_surface_union(left, right),
        ExactBooleanOperation::Intersection => {
            arrange_coplanar_orthogonal_surface_intersection(left, right)
        }
        ExactBooleanOperation::Difference => {
            arrange_coplanar_orthogonal_surface_difference(left, right)
        }
        ExactBooleanOperation::SelectedRegions(_) => None,
    };
    let Some(arrangement) = arrangement else {
        return Ok(None);
    };
    let (label, shortcut) = match arrangement.operation {
        CoplanarOrthogonalSurfaceOperation::Union => (
            "exact coplanar orthogonal surface union",
            ExactBooleanShortcutKind::CoplanarOrthogonalSurfaceUnion,
        ),
        CoplanarOrthogonalSurfaceOperation::Intersection => (
            "exact coplanar orthogonal surface intersection",
            ExactBooleanShortcutKind::CoplanarOrthogonalSurfaceIntersection,
        ),
        CoplanarOrthogonalSurfaceOperation::Difference => (
            "exact coplanar orthogonal surface difference",
            ExactBooleanShortcutKind::CoplanarOrthogonalSurfaceDifference,
        ),
    };
    let mesh = copy_mesh(&arrangement.mesh, label, validation)?;
    Ok(Some(certified_shortcut_result(mesh, shortcut)))
}

fn boolean_coplanar_affine_surface_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let arrangement = match operation {
        ExactBooleanOperation::Union => arrange_coplanar_affine_surface_union(left, right),
        ExactBooleanOperation::Intersection => {
            arrange_coplanar_affine_surface_intersection(left, right)
        }
        ExactBooleanOperation::Difference => {
            arrange_coplanar_affine_surface_difference(left, right)
        }
        ExactBooleanOperation::SelectedRegions(_) => None,
    };
    let Some(arrangement) = arrangement else {
        return Ok(None);
    };
    let (label, shortcut) = match arrangement.operation {
        CoplanarOrthogonalSurfaceOperation::Union => (
            "exact coplanar affine surface union",
            ExactBooleanShortcutKind::CoplanarAffineSurfaceUnion,
        ),
        CoplanarOrthogonalSurfaceOperation::Intersection => (
            "exact coplanar affine surface intersection",
            ExactBooleanShortcutKind::CoplanarAffineSurfaceIntersection,
        ),
        CoplanarOrthogonalSurfaceOperation::Difference => (
            "exact coplanar affine surface difference",
            ExactBooleanShortcutKind::CoplanarAffineSurfaceDifference,
        ),
    };
    let mesh = copy_mesh(&arrangement.mesh, label, validation)?;
    Ok(Some(certified_shortcut_result(mesh, shortcut)))
}

fn boolean_direct_coplanar_surface_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    match operation {
        ExactBooleanOperation::Union => {
            if certify_coplanar_convex_surface_equivalence(left, right).is_some() {
                return Ok(Some(boolean_coplanar_convex_equivalent_surfaces(
                    left, operation, validation,
                )?));
            }
            if let Some(containment) = certify_coplanar_convex_surface_containment(left, right) {
                return Ok(Some(boolean_coplanar_convex_containment_surfaces(
                    left,
                    right,
                    operation,
                    validation,
                    containment,
                )?));
            }
            if let Some(union) = arrange_coplanar_convex_surface_union(left, right) {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &union.mesh,
                    "exact coplanar convex arrangement union",
                    ExactBooleanShortcutKind::CoplanarConvexSurfaceArrangementUnion,
                    validation,
                )?));
            }
            if let Some(union) = arrange_coplanar_convex_surface_component_union(left, right) {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &union.mesh,
                    "exact coplanar convex arrangement union",
                    ExactBooleanShortcutKind::CoplanarConvexSurfaceArrangementUnion,
                    validation,
                )?));
            }
            if let Some(union) = arrange_coplanar_surface_component_union(left, right) {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &union.mesh,
                    "exact coplanar nonconvex component union",
                    ExactBooleanShortcutKind::CoplanarSurfaceArrangementUnion,
                    validation,
                )?));
            }
            if let Some(union) = arrange_coplanar_surface_component_holed_union(left, right) {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &union.mesh,
                    "exact coplanar component-holed surface union",
                    ExactBooleanShortcutKind::CoplanarSurfaceArrangementUnion,
                    validation,
                )?));
            }
            if let Some(union) = arrange_coplanar_surface_multi_component_union(left, right) {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &union.mesh,
                    "exact coplanar nonconvex multi-component union",
                    ExactBooleanShortcutKind::CoplanarSurfaceMultiUnion,
                    validation,
                )?));
            }
            if let Some(union) = arrange_coplanar_convex_surface_multi_union(left, right) {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &union.mesh,
                    "exact coplanar convex multi-component union",
                    ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiUnion,
                    validation,
                )?));
            }
            if let Some(union) = arrange_coplanar_surface_point_touch_union(left, right) {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &union.mesh,
                    "exact coplanar point-touch surface union",
                    ExactBooleanShortcutKind::CoplanarSurfacePointTouchUnion,
                    validation,
                )?));
            }
            if let Some(result) =
                boolean_coplanar_orthogonal_surface_optional(left, right, operation, validation)?
            {
                return Ok(Some(result));
            }
            boolean_coplanar_affine_surface_optional(left, right, operation, validation)
        }
        ExactBooleanOperation::Intersection => {
            if certify_coplanar_convex_surface_equivalence(left, right).is_some() {
                return Ok(Some(boolean_coplanar_convex_equivalent_surfaces(
                    left, operation, validation,
                )?));
            }
            if let Some(containment) = certify_coplanar_convex_surface_containment(left, right) {
                return Ok(Some(boolean_coplanar_convex_containment_surfaces(
                    left,
                    right,
                    operation,
                    validation,
                    containment,
                )?));
            }
            if let Some(intersection) = arrange_coplanar_convex_surface_intersection(left, right) {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &intersection.mesh,
                    "exact coplanar convex arrangement intersection",
                    ExactBooleanShortcutKind::CoplanarConvexSurfaceIntersection,
                    validation,
                )?));
            }
            if let Some(intersection) =
                arrange_coplanar_convex_surface_multi_intersection(left, right)
            {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &intersection.mesh,
                    "exact coplanar convex multi-component intersection",
                    ExactBooleanShortcutKind::CoplanarConvexSurfaceIntersection,
                    validation,
                )?));
            }
            if let Some(intersection) =
                arrange_coplanar_surface_component_holed_intersection(left, right)
            {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &intersection.mesh,
                    "exact coplanar component-holed surface intersection",
                    ExactBooleanShortcutKind::CoplanarSurfaceIntersection,
                    validation,
                )?));
            }
            if let Some(result) =
                boolean_coplanar_orthogonal_surface_optional(left, right, operation, validation)?
            {
                return Ok(Some(result));
            }
            if let Some(result) =
                boolean_coplanar_affine_surface_optional(left, right, operation, validation)?
            {
                return Ok(Some(result));
            }
            if certify_coplanar_surface_boundary_touch(left, right).is_some() {
                return Ok(Some(boolean_coplanar_surface_boundary_touch_intersection(
                    left, right, validation,
                )?));
            }
            if arrange_coplanar_surface_point_touch_union(left, right).is_some() {
                let mesh = empty_mesh(
                    "empty exact coplanar point-touch surface intersection",
                    validation,
                )?;
                return Ok(Some(certified_shortcut_result(
                    mesh,
                    ExactBooleanShortcutKind::CoplanarSurfacePointTouchIntersection,
                )));
            }
            Ok(None)
        }
        ExactBooleanOperation::Difference => {
            if certify_coplanar_convex_surface_equivalence(left, right).is_some() {
                return Ok(Some(boolean_coplanar_convex_equivalent_surfaces(
                    left, operation, validation,
                )?));
            }
            if certify_coplanar_surface_boundary_touch(left, right).is_some() {
                return Ok(Some(boolean_coplanar_surface_boundary_touch_difference(
                    left, right, validation,
                )?));
            }
            if arrange_coplanar_surface_point_touch_union(left, right).is_some() {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    left,
                    "exact coplanar point-touch surface difference keeps left",
                    ExactBooleanShortcutKind::CoplanarSurfacePointTouchDifference,
                    validation,
                )?));
            }
            if let Some(difference) = arrange_coplanar_convex_surface_difference(left, right) {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &difference.mesh,
                    "exact coplanar convex arrangement difference",
                    ExactBooleanShortcutKind::CoplanarConvexSurfaceArrangementDifference,
                    validation,
                )?));
            }
            if let Some(difference) = arrange_coplanar_convex_surface_multi_difference(left, right)
            {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &difference.mesh,
                    "exact coplanar convex multi-component difference",
                    ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiDifference,
                    validation,
                )?));
            }
            if let Some(difference) = arrange_coplanar_surface_component_difference(left, right) {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &difference.mesh,
                    "exact coplanar component difference",
                    ExactBooleanShortcutKind::CoplanarSurfaceArrangementDifference,
                    validation,
                )?));
            }
            if let Some(difference) = arrange_coplanar_surface_multi_difference(left, right) {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &difference.mesh,
                    "exact coplanar nonconvex multi-component difference",
                    ExactBooleanShortcutKind::CoplanarSurfaceMultiDifference,
                    validation,
                )?));
            }
            if let Some(difference) = arrange_coplanar_surface_side_cutter_difference(left, right) {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &difference.mesh,
                    "exact coplanar side-cutter difference",
                    ExactBooleanShortcutKind::CoplanarSurfaceSideCutterDifference,
                    validation,
                )?));
            }
            if let Some(result) = boolean_coplanar_cutter_hole_contact_difference_optional(
                left, right, operation, validation,
            )? {
                return Ok(Some(result));
            }
            if let Some(difference) = arrange_coplanar_convex_surface_holed_difference(left, right)
            {
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &difference.mesh,
                    "exact coplanar convex containment holed difference",
                    ExactBooleanShortcutKind::CoplanarConvexSurfaceHoledDifference,
                    validation,
                )?));
            }
            if let Some(result) =
                boolean_coplanar_convex_multi_holed_difference(left, right, operation, validation)?
            {
                return Ok(Some(result));
            }
            if let Some(result) = boolean_coplanar_convex_component_holed_difference_optional(
                left, right, operation, validation,
            )? {
                return Ok(Some(result));
            }
            if let Some(arrangement) = arrange_coplanar_affine_surface_difference(left, right)
                && affine_basis_is_non_axis_aligned(&arrangement.basis)
            {
                let mesh = copy_mesh(
                    &arrangement.mesh,
                    "exact coplanar affine surface difference",
                    validation,
                )?;
                return Ok(Some(certified_shortcut_result(
                    mesh,
                    ExactBooleanShortcutKind::CoplanarAffineSurfaceDifference,
                )));
            }
            if let Some(difference) = arrange_coplanar_surface_point_touch_difference(left, right) {
                // Keep the retained branch/deleted-ring certificate ahead of the
                // generic orthogonal-cell materializer. The exact topology is the
                return Ok(Some(copied_shortcut_result_from_mesh(
                    &difference.mesh,
                    "exact coplanar point-touch surface difference",
                    ExactBooleanShortcutKind::CoplanarSurfacePointTouchDifference,
                    validation,
                )?));
            }
            boolean_coplanar_orthogonal_surface_optional(left, right, operation, validation)
        }
        ExactBooleanOperation::SelectedRegions(_) => Ok(None),
    }
}

fn has_non_axis_aligned_affine_surface_difference(left: &ExactMesh, right: &ExactMesh) -> bool {
    arrange_coplanar_affine_surface_difference(left, right)
        .as_ref()
        .is_some_and(|arrangement| affine_basis_is_non_axis_aligned(&arrangement.basis))
}

fn affine_basis_is_non_axis_aligned(basis: &CoplanarAffineSurfaceBasis) -> bool {
    let u = project_point3(&basis.basis_u, basis.projection);
    let v = project_point3(&basis.basis_v, basis.projection);
    !(projected_vector_axis_aligned(&u) && projected_vector_axis_aligned(&v))
}

fn projected_vector_axis_aligned(vector: &hyperlimit::Point2) -> bool {
    let x_is_zero = compare_reals(&vector.x, &Real::from(0)).value() == Some(Ordering::Equal);
    let y_is_zero = compare_reals(&vector.y, &Real::from(0)).value() == Some(Ordering::Equal);
    x_is_zero ^ y_is_zero
}

fn boolean_axis_aligned_box_operation_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let (operation, shortcut) = match operation {
        ExactBooleanOperation::Union => (
            AxisAlignedBoxOperation::Union,
            ExactBooleanShortcutKind::AxisAlignedBoxUnion,
        ),
        ExactBooleanOperation::Intersection => (
            AxisAlignedBoxOperation::Intersection,
            ExactBooleanShortcutKind::AxisAlignedBoxIntersection,
        ),
        ExactBooleanOperation::Difference => (
            AxisAlignedBoxOperation::Difference,
            ExactBooleanShortcutKind::AxisAlignedBoxDifference,
        ),
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    let Some(mesh) =
        materialize_simple_axis_aligned_box_operation(left, right, operation, validation)?
    else {
        return Ok(None);
    };
    Ok(Some(certified_shortcut_result(mesh, shortcut)))
}

fn boolean_axis_aligned_box_special_difference_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    if let Some(mesh) = multi_difference_axis_aligned_boxes(left, right, validation)? {
        return Ok(Some(certified_shortcut_result(
            mesh,
            ExactBooleanShortcutKind::AxisAlignedBoxMultiDifference,
        )));
    }
    if let Some(mesh) = nested_difference_axis_aligned_boxes(left, right, validation)? {
        return Ok(Some(certified_shortcut_result(
            mesh,
            ExactBooleanShortcutKind::AxisAlignedBoxNestedDifference,
        )));
    }
    if let Some(mesh) = empty_difference_axis_aligned_boxes(left, right, validation)? {
        return Ok(Some(certified_shortcut_result(
            mesh,
            ExactBooleanShortcutKind::AxisAlignedBoxEmptyDifference,
        )));
    }
    Ok(None)
}

fn boolean_axis_aligned_box_cell_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let (mesh, shortcut) = match operation {
        ExactBooleanOperation::Union => {
            let Some(mesh) = cell_union_axis_aligned_boxes(left, right, validation)? else {
                return Ok(None);
            };
            (mesh, ExactBooleanShortcutKind::AxisAlignedBoxCellUnion)
        }
        ExactBooleanOperation::Difference => {
            let Some(mesh) = cell_difference_axis_aligned_boxes(left, right, validation)? else {
                return Ok(None);
            };
            (mesh, ExactBooleanShortcutKind::AxisAlignedBoxCellDifference)
        }
        ExactBooleanOperation::Intersection | ExactBooleanOperation::SelectedRegions(_) => {
            return Ok(None);
        }
    };
    Ok(Some(certified_shortcut_result(mesh, shortcut)))
}

fn boolean_axis_aligned_orthogonal_solid_cell_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation) else {
        return Ok(None);
    };
    let label = match solid_operation {
        AxisAlignedOrthogonalSolidOperation::Union => {
            "exact axis-aligned orthogonal solid cell union"
        }
        AxisAlignedOrthogonalSolidOperation::Intersection => {
            "exact axis-aligned orthogonal solid cell intersection"
        }
        AxisAlignedOrthogonalSolidOperation::Difference => {
            "exact axis-aligned orthogonal solid cell difference"
        }
    };
    let Some(mesh) = materialize_axis_aligned_orthogonal_solid_cells(
        left,
        right,
        solid_operation,
        label,
        validation,
    )?
    else {
        return Ok(None);
    };
    Ok(Some(certified_shortcut_result(
        mesh,
        axis_aligned_orthogonal_solid_shortcut(solid_operation),
    )))
}

/// Return whether exact orthogonal occupancy certifies an empty intersection.
///
/// This is intentionally narrower than the general orthogonal-cell shortcut:
/// ordinary nonempty unions/intersections/differences should keep the more
/// specific graph, box-cell, and boundary-touch certificates when available.
/// The empty cavity case can have overlapping AABBs and no graph events, so
/// this retained evidence witness is checked before falling through to winding,
fn has_empty_axis_aligned_orthogonal_solid_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, MeshError> {
    Ok(has_empty_axis_aligned_orthogonal_solid_cell_intersection(
        left, right,
    ))
}

const fn axis_aligned_orthogonal_solid_operation(
    operation: ExactBooleanOperation,
) -> Option<AxisAlignedOrthogonalSolidOperation> {
    match operation {
        ExactBooleanOperation::Union => Some(AxisAlignedOrthogonalSolidOperation::Union),
        ExactBooleanOperation::Intersection => {
            Some(AxisAlignedOrthogonalSolidOperation::Intersection)
        }
        ExactBooleanOperation::Difference => Some(AxisAlignedOrthogonalSolidOperation::Difference),
        ExactBooleanOperation::SelectedRegions(_) => None,
    }
}

const fn axis_aligned_orthogonal_solid_support(
    operation: AxisAlignedOrthogonalSolidOperation,
) -> ExactBooleanSupport {
    match operation {
        AxisAlignedOrthogonalSolidOperation::Union => {
            ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellUnion
        }
        AxisAlignedOrthogonalSolidOperation::Intersection => {
            ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellIntersection
        }
        AxisAlignedOrthogonalSolidOperation::Difference => {
            ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellDifference
        }
    }
}

const fn axis_aligned_orthogonal_solid_shortcut(
    operation: AxisAlignedOrthogonalSolidOperation,
) -> ExactBooleanShortcutKind {
    match operation {
        AxisAlignedOrthogonalSolidOperation::Union => {
            ExactBooleanShortcutKind::AxisAlignedOrthogonalSolidCellUnion
        }
        AxisAlignedOrthogonalSolidOperation::Intersection => {
            ExactBooleanShortcutKind::AxisAlignedOrthogonalSolidCellIntersection
        }
        AxisAlignedOrthogonalSolidOperation::Difference => {
            ExactBooleanShortcutKind::AxisAlignedOrthogonalSolidCellDifference
        }
    }
}

fn boolean_affine_box_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some((arrangement, shortcut)) = (match operation {
        ExactBooleanOperation::Union => materialize_affine_box_union(left, right, validation)?
            .map(|arrangement| (arrangement, ExactBooleanShortcutKind::AffineBoxUnion)),
        ExactBooleanOperation::Intersection => {
            materialize_affine_box_intersection(left, right, validation)?
                .map(|arrangement| (arrangement, ExactBooleanShortcutKind::AffineBoxIntersection))
        }
        ExactBooleanOperation::Difference => {
            materialize_affine_box_difference(left, right, validation)?
                .map(|arrangement| (arrangement, ExactBooleanShortcutKind::AffineBoxDifference))
        }
        ExactBooleanOperation::SelectedRegions(_) => None,
    }) else {
        return Ok(None);
    };
    Ok(Some(certified_shortcut_result(arrangement.mesh, shortcut)))
}

fn boolean_affine_orthogonal_solid_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let affine_operation = match operation {
        ExactBooleanOperation::Union => AffineOrthogonalSolidOperation::Union,
        ExactBooleanOperation::Intersection => AffineOrthogonalSolidOperation::Intersection,
        ExactBooleanOperation::Difference => AffineOrthogonalSolidOperation::Difference,
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    let arrangement = match affine_operation {
        AffineOrthogonalSolidOperation::Union => {
            materialize_affine_orthogonal_solid_union(left, right, validation)?
        }
        AffineOrthogonalSolidOperation::Intersection => {
            materialize_affine_orthogonal_solid_intersection(left, right, validation)?
        }
        AffineOrthogonalSolidOperation::Difference => {
            materialize_affine_orthogonal_solid_difference(left, right, validation)?
        }
    };
    let Some(arrangement) = arrangement else {
        return Ok(None);
    };
    let shortcut = match operation {
        ExactBooleanOperation::Union => ExactBooleanShortcutKind::AffineOrthogonalSolidCellUnion,
        ExactBooleanOperation::Intersection => {
            ExactBooleanShortcutKind::AffineOrthogonalSolidCellIntersection
        }
        ExactBooleanOperation::Difference => {
            ExactBooleanShortcutKind::AffineOrthogonalSolidCellDifference
        }
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("returned above"),
    };
    Ok(Some(certified_shortcut_result(arrangement.mesh, shortcut)))
}

fn materialize_open_surface_disjoint_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
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

    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::OpenSurfaceDisjoint,
    ))
}

fn boolean_open_surface_disjoint_or_arrangement_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if !mesh_is_open_surface(left) || !mesh_is_open_surface(right) {
        return Ok(None);
    }
    let disjoint_report = open_surface_disjoint_report_from_graph(graph, left, right);
    if disjoint_report.is_certified() {
        return materialize_open_surface_disjoint_meshes(left, right, operation, validation)
            .map(Some);
    }
    let graph_had_unknowns = graph.has_unknowns();
    let Some(plan) = open_surface_arrangement_plan_from_graph(graph, left, right, operation)?
    else {
        return Ok(None);
    };
    materialize_open_surface_arrangement_plan(
        left,
        right,
        operation,
        validation,
        graph_had_unknowns,
        plan,
    )
    .map(Some)
}

/// Certify whether two open surface meshes are disjoint by exact graph facts.
///
/// This is the report form of the open-surface named-boolean shortcut. It
/// validates the open-surface precondition from exact mesh facts, then records
/// the retained graph relation counts that prove no face pair survived exact
/// graph fact, not a tolerance side effect.
pub fn certify_open_surface_disjoint_report(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactOpenSurfaceDisjointReport, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    Ok(open_surface_disjoint_report_from_graph(&graph, left, right))
}

fn open_surface_disjoint_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactOpenSurfaceDisjointReport {
    let left_open_surface = mesh_is_open_surface(left);
    let right_open_surface = mesh_is_open_surface(right);
    if !left_open_surface || !right_open_surface {
        return open_surface_disjoint_report(
            ExactOpenSurfaceDisjointStatus::NotOpenSurface,
            left_open_surface,
            right_open_surface,
            false,
            0,
            0,
            GraphRelationCounts::default(),
        );
    }
    let graph_had_unknowns = graph.has_unknowns();
    let counts = graph_relation_counts(graph);
    let status = if graph_had_unknowns {
        ExactOpenSurfaceDisjointStatus::GraphUnknowns
    } else if graph.face_pairs.is_empty() {
        ExactOpenSurfaceDisjointStatus::Certified
    } else {
        ExactOpenSurfaceDisjointStatus::GraphHasFacePairs
    };
    open_surface_disjoint_report(
        status,
        left_open_surface,
        right_open_surface,
        graph_had_unknowns,
        graph.face_pairs.len(),
        graph.event_count(),
        counts,
    )
}

fn certified_open_surface_disjoint_support_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return None;
    }
    open_surface_disjoint_report_from_graph(graph, left, right)
        .is_certified()
        .then_some(ExactBooleanSupport::CertifiedOpenSurfaceDisjoint)
}

fn open_surface_disjoint_report(
    status: ExactOpenSurfaceDisjointStatus,
    left_open_surface: bool,
    right_open_surface: bool,
    graph_had_unknowns: bool,
    retained_face_pairs: usize,
    retained_events: usize,
    counts: GraphRelationCounts,
) -> ExactOpenSurfaceDisjointReport {
    let blocker_kind = if matches!(status, ExactOpenSurfaceDisjointStatus::GraphUnknowns) {
        ExactBooleanBlockerKind::NeedsRefinement
    } else {
        ExactBooleanBlockerKind::NeedsWinding
    };
    ExactOpenSurfaceDisjointReport {
        status,
        left_open_surface,
        right_open_surface,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        blocker: counts.into_blocker(blocker_kind),
    }
}

fn mesh_is_open_surface(mesh: &ExactMesh) -> bool {
    !mesh.triangles().is_empty()
        && !mesh.facts().mesh.closed_manifold
        && mesh.facts().mesh.boundary_edges > 0
        && mesh.facts().mesh.non_manifold_edges == 0
        && mesh.facts().mesh.non_manifold_vertices == 0
}

/// Retained split-region artifacts that certify an open-surface arrangement.
type OpenSurfaceArrangementPlan = (
    ExactBooleanSupport,
    Vec<FaceRegionPlaneClassification>,
    Vec<FaceRegionTriangulation>,
);

/// Materialize a named arrangement boolean for crossing open surfaces.
///
/// This is deliberately narrower than general surface booleans: both operands
/// must already be accepted open manifold surfaces, the graph must contain
/// proper non-coplanar crossings, and coplanar/boundary-only cases stay on
/// surface union retains every certified split region, regularized
/// intersection retains none because the crossing curve is lower-dimensional,
/// and regularized difference retains the left split regions. Triangle meshes
/// cannot represent the shared curve as an area cell, so that projection stays
/// explicit in the result kind and retained arrangement evidence.
fn materialize_open_surface_arrangement_plan(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    graph_had_unknowns: bool,
    plan: OpenSurfaceArrangementPlan,
) -> Result<ExactBooleanResult, MeshError> {
    let (_support, region_classifications, triangulations) = plan;
    let selection = match operation {
        ExactBooleanOperation::Union => ExactRegionSelection::KeepAll,
        ExactBooleanOperation::Intersection => ExactRegionSelection::KeepNone,
        ExactBooleanOperation::Difference => ExactRegionSelection::KeepLeft,
        ExactBooleanOperation::SelectedRegions(_) => {
            unreachable!("open-surface arrangement plan filters unsupported operations")
        }
    };
    // Open-surface arrangement is not a closed-volumetric inside/outside
    // split regions are retained by surface operation, and no winding label is
    // invented for a mesh that has no closed volume.
    let assembly = ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
        &triangulations,
        selection,
        left,
        right,
    )
    .map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::IndexOutOfBounds,
            format!("open-surface arrangement assembly failed: {error}"),
        ))
    })?;
    let mesh = assembly.checked_to_exact_mesh_with_sources(left, right, validation)?;
    let result = ExactBooleanResult {
        kind: ExactBooleanResultKind::OpenSurfaceArrangement { operation },
        graph_had_unknowns,
        region_classifications,
        triangulations,
        assembly,
        volumetric_classifications: Vec::new(),
        mesh,
    };
    result.validate().map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("open-surface arrangement validation failed: {error:?}"),
        ))
    })?;
    Ok(result)
}

/// Build the retained exact split-region plan for open-surface arrangement.
///
/// The returned classifications are not used to decide inside/outside; they
/// are retained proof-producing side facts that make the arrangement replayable
fn open_surface_arrangement_plan_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<OpenSurfaceArrangementPlan>, MeshError> {
    let support = match operation {
        ExactBooleanOperation::Union => ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion,
        ExactBooleanOperation::Intersection => {
            ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
        }
        ExactBooleanOperation::Difference => {
            ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference
        }
        ExactBooleanOperation::SelectedRegions(_) => {
            return Ok(None);
        }
    };
    if !mesh_is_open_surface(left) || !mesh_is_open_surface(right) {
        return Ok(None);
    }
    let counts = graph_relation_counts(graph);
    if graph.has_unknowns()
        || graph.face_pairs.is_empty()
        || counts.unknown_pairs != 0
        || counts.construction_failed_events != 0
        || counts.coplanar_overlapping_pairs != 0
        || counts.coplanar_touching_pairs != 0
        || !graph_has_proper_surface_crossing(graph)
    {
        return Ok(None);
    }

    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
    if region_classifications
        .iter()
        .any(|classification| !classification.is_decided_and_proof_producing())
    {
        return Ok(None);
    }
    let triangulations = checked_triangulate_face_regions_with_earcut(&region_plan, left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::DegenerateTriangle,
                format!("open-surface arrangement triangulation failed: {error}"),
            ))
        })?;
    Ok(Some((support, region_classifications, triangulations)))
}

/// Return whether the graph contains a genuine non-coplanar surface crossing.
///
/// Endpoint, edge-only, and coplanar contacts need separate topology policies.
/// This gate keeps the open-surface union shortcut tied to exact proper
/// segment/plane construction facts rather than a tolerance-style overlap
fn graph_has_proper_surface_crossing(graph: &super::graph::ExactIntersectionGraph) -> bool {
    graph.face_pairs.iter().any(|pair| {
        pair.relation == MeshFacePairRelation::Candidate
            && pair.events.iter().any(|event| {
                matches!(
                    event,
                    IntersectionEvent::SegmentPlane {
                        relation: SegmentPlaneRelation::ProperCrossing,
                        ..
                    }
                )
            })
    })
}

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

    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarSurfaceContainment,
    )))
}

fn certified_coplanar_surface_boolean_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<CoplanarSurfaceContainment> {
    let containment = certify_single_triangle_coplanar_containment(left, right)
        .or_else(|| certify_coplanar_surface_mesh_containment(left, right))?;
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

    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::SameSurface,
    ))
}

/// Materialize the certified full-face/fan-patch adjacent regularized union.
fn boolean_full_face_adjacent_union_from_artifact(
    union: super::adjacent::FullFaceAdjacentUnion,
) -> ExactBooleanResult {
    certified_shortcut_result(union.mesh, ExactBooleanShortcutKind::FullFaceAdjacentUnion)
}

/// Materialize the certified contained-face adjacent regularized union.
///
/// A strictly contained opposite-oriented boundary triangle is a bounded
/// coplanar-volumetric cell case: the contained source face is deleted, and
/// the containing source face is replaced by a holed remnant whose inner ring
/// welds to the other solid. The replayed certificate keeps the branch within
/// mesh tolerance merge.
fn boolean_contained_face_adjacent_union_from_artifact(
    union: super::contained_adjacent::ContainedFaceAdjacentUnion,
) -> ExactBooleanResult {
    certified_shortcut_result(
        union.mesh,
        ExactBooleanShortcutKind::ContainedFaceAdjacentUnion,
    )
}

fn boolean_direct_adjacency_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    match operation {
        ExactBooleanOperation::Union => {
            if let Some(result) =
                boolean_full_face_adjacency_optional(left, right, operation, validation)?
            {
                return Ok(Some(result));
            }
            if let Some(result) =
                boolean_contained_face_adjacency_optional(left, right, operation, validation)?
            {
                return Ok(Some(result));
            }
        }
        ExactBooleanOperation::Intersection => {
            if let Some(result) =
                boolean_contained_face_adjacency_optional(left, right, operation, validation)?
            {
                return Ok(Some(result));
            }
            if let Some(result) =
                boolean_full_face_adjacency_optional(left, right, operation, validation)?
            {
                return Ok(Some(result));
            }
        }
        ExactBooleanOperation::Difference => {
            if let Some(result) =
                boolean_contained_face_adjacency_optional(left, right, operation, validation)?
            {
                return Ok(Some(result));
            }
            if let Some(result) =
                boolean_full_face_adjacency_optional(left, right, operation, validation)?
            {
                return Ok(Some(result));
            }
        }
        ExactBooleanOperation::SelectedRegions(_) => {}
    }
    Ok(None)
}

fn boolean_full_face_adjacency_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if both_axis_aligned_boxes(left, right) {
        return Ok(None);
    }
    let Some(certificate) = full_face_adjacent_certificate(left, right) else {
        return Ok(None);
    };
    let result = match operation {
        ExactBooleanOperation::Union => {
            let Some(union) = materialize_full_face_adjacent_union_from_certificate(
                left,
                right,
                &certificate,
                validation,
            ) else {
                return Ok(None);
            };
            boolean_full_face_adjacent_union_from_artifact(union)
        }
        ExactBooleanOperation::Intersection => {
            boolean_full_face_adjacent_intersection(left, right, validation)?
        }
        ExactBooleanOperation::Difference => {
            boolean_full_face_adjacent_difference(left, right, validation)?
        }
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    Ok(Some(result))
}

fn boolean_contained_face_adjacency_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(certificate) = contained_face_adjacent_certificate(left, right) else {
        return Ok(None);
    };
    let result = match operation {
        ExactBooleanOperation::Union => {
            let Some(union) = materialize_contained_face_adjacent_union_from_certificate(
                left,
                right,
                &certificate,
                validation,
            ) else {
                return Ok(None);
            };
            boolean_contained_face_adjacent_union_from_artifact(union)
        }
        ExactBooleanOperation::Intersection => {
            boolean_contained_face_adjacent_intersection(left, right, validation)?
        }
        ExactBooleanOperation::Difference => {
            boolean_contained_face_adjacent_difference(left, right, validation)?
        }
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    Ok(Some(result))
}

/// Materialize the empty regularized intersection for contained-face adjacency.
///
/// A strictly contained opposite-oriented boundary face proves contact along a
/// two-dimensional boundary subset, not positive volume. The dispatch guard
/// has just replayed the contained-face adjacency certificate; the branch can
/// therefore return the empty mesh without constructing a union artifact it
/// and objects that justify each topological branch.
fn boolean_contained_face_adjacent_intersection(
    _left: &ExactMesh,
    _right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    Ok(certified_shortcut_result(
        empty_mesh(
            "empty exact contained-face adjacent regularized intersection",
            validation,
        )?,
        ExactBooleanShortcutKind::ContainedFaceAdjacentIntersection,
    ))
}

/// Materialize the left-preserving regularized difference for contained-face adjacency.
///
/// Boundary-only contained-face contact removes no left volume. The dispatch
/// guard has just replayed the contained-face certificate, so this branch
/// avoids constructing a union mesh that it discards.
fn boolean_contained_face_adjacent_difference(
    left: &ExactMesh,
    _right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    Ok(certified_shortcut_result(
        copy_mesh(
            left,
            "exact contained-face adjacent regularized difference keeps left",
            validation,
        )?,
        ExactBooleanShortcutKind::ContainedFaceAdjacentDifference,
    ))
}

/// Materialize a boundary-contained closed-solid difference.
///
/// This is the nonconvex-capable sibling of the convex boundary-containment
/// difference: the removed solid is certified inside the left container by
/// exact winding replay and touches the container through same-oriented
/// source-owned caps. The output replaces those container caps with exact
/// materializing only the retained exact cap object instead of inferring a
/// cavity from approximate representatives.
fn boolean_contained_boundary_difference_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    let Some(difference) =
        materialize_contained_boundary_difference_from_graph(left, right, graph, validation)
    else {
        return Ok(None);
    };
    Ok(Some(certified_shortcut_result(
        difference.mesh,
        ExactBooleanShortcutKind::ContainedBoundaryDifference,
    )))
}

/// Materialize regularized booleans for closed boundary-contained solids.
///
/// The same retained cap certificate used by
/// [`materialize_contained_boundary_difference`] proves that one closed solid
/// lies inside the other while sharing same-oriented source-owned boundary
/// caps. For union and intersection that certificate selects the outer or
/// inner source shell directly; for the reverse difference the contained left
/// volume is removed completely. The cavity-producing `container - removed`
/// case stays with [`boolean_contained_boundary_difference_meshes_from_graph`].
fn boolean_contained_boundary_containment_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(containment) = contained_boundary_containment_from_graph(left, right, graph) else {
        return Ok(None);
    };
    let mesh = match operation {
        ExactBooleanOperation::Union => match containment {
            ContainedBoundaryContainment::LeftContainsRight => copy_mesh(
                left,
                "exact contained-boundary containment union keeps outer left",
                validation,
            )?,
            ContainedBoundaryContainment::RightContainsLeft => copy_mesh(
                right,
                "exact contained-boundary containment union keeps outer right",
                validation,
            )?,
        },
        ExactBooleanOperation::Intersection => match containment {
            ContainedBoundaryContainment::LeftContainsRight => copy_mesh(
                right,
                "exact contained-boundary containment intersection keeps inner right",
                validation,
            )?,
            ContainedBoundaryContainment::RightContainsLeft => copy_mesh(
                left,
                "exact contained-boundary containment intersection keeps inner left",
                validation,
            )?,
        },
        ExactBooleanOperation::Difference => match containment {
            ContainedBoundaryContainment::RightContainsLeft => empty_mesh(
                "empty exact contained-boundary reverse difference",
                validation,
            )?,
            ContainedBoundaryContainment::LeftContainsRight => return Ok(None),
        },
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };

    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::ContainedBoundaryContainment,
    )))
}

/// Materialize the empty regularized intersection for certified adjacency.
///
/// Regularized solid booleans drop lower-dimensional boundary contact from
/// the intersection volume, so an exact full-face/fan-patch adjacency has no
/// volume to emit. The dispatch guard has just replayed the certificate, so
/// the branch can avoid constructing the adjacent union mesh that it discards,
/// while still tying the topology decision to exact retained combinatorial
fn boolean_full_face_adjacent_intersection(
    _left: &ExactMesh,
    _right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    Ok(certified_shortcut_result(
        empty_mesh(
            "empty exact full-face adjacent regularized intersection",
            validation,
        )?,
        ExactBooleanShortcutKind::FullFaceAdjacentIntersection,
    ))
}

/// Materialize the left-preserving regularized difference for certified adjacency.
///
/// A boundary-only full-face/fan-patch contact removes no left volume. The
/// dispatch guard has just replayed the exact adjacency certificate, so the
/// left source can be copied without materializing a union mesh that this
/// branch discards.
fn boolean_full_face_adjacent_difference(
    left: &ExactMesh,
    _right: &ExactMesh,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    Ok(certified_shortcut_result(
        copy_mesh(
            left,
            "exact full-face adjacent regularized difference keeps left",
            validation,
        )?,
        ExactBooleanShortcutKind::FullFaceAdjacentDifference,
    ))
}

fn boolean_coplanar_convex_equivalent_surfaces(
    mesh: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection => copy_mesh(
            mesh,
            "exact coplanar convex equivalent surface result",
            validation,
        )?,
        ExactBooleanOperation::Difference => empty_mesh(
            "empty exact coplanar convex equivalent surface difference",
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::CoplanarConvexSurfaceEquivalence,
    ))
}

fn boolean_coplanar_convex_containment_surfaces(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    containment: CoplanarConvexSurfaceContainmentCertificate,
) -> Result<ExactBooleanResult, MeshError> {
    let relation = containment.relation;
    let mesh = match (relation, operation) {
        (CoplanarConvexSurfaceContainment::LeftInsideRight, ExactBooleanOperation::Union) => {
            copy_mesh(
                right,
                "exact coplanar convex containment union keeps outer right",
                validation,
            )?
        }
        (
            CoplanarConvexSurfaceContainment::LeftInsideRight,
            ExactBooleanOperation::Intersection,
        ) => copy_mesh(
            left,
            "exact coplanar convex containment intersection keeps inner left",
            validation,
        )?,
        (CoplanarConvexSurfaceContainment::LeftInsideRight, ExactBooleanOperation::Difference) => {
            empty_mesh(
                "empty exact coplanar convex containment difference",
                validation,
            )?
        }
        (CoplanarConvexSurfaceContainment::RightInsideLeft, ExactBooleanOperation::Union) => {
            copy_mesh(
                left,
                "exact coplanar convex containment union keeps outer left",
                validation,
            )?
        }
        (
            CoplanarConvexSurfaceContainment::RightInsideLeft,
            ExactBooleanOperation::Intersection,
        ) => copy_mesh(
            right,
            "exact coplanar convex containment intersection keeps inner right",
            validation,
        )?,
        (CoplanarConvexSurfaceContainment::RightInsideLeft, ExactBooleanOperation::Difference) => {
            let Some(difference) =
                arrange_coplanar_convex_surface_holed_difference_from_certificate(containment)
            else {
                return Err(MeshError::one(MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::UnsupportedExactOperation,
                    "exact coplanar convex containment certificate did not materialize holed difference",
                )));
            };
            copy_mesh(
                &difference.mesh,
                "exact coplanar convex containment holed difference",
                validation,
            )?
        }
        (_, ExactBooleanOperation::SelectedRegions(_)) => unreachable!("handled by caller"),
    };

    let shortcut = if relation == CoplanarConvexSurfaceContainment::RightInsideLeft
        && operation == ExactBooleanOperation::Difference
    {
        ExactBooleanShortcutKind::CoplanarConvexSurfaceHoledDifference
    } else {
        ExactBooleanShortcutKind::CoplanarConvexSurfaceContainment
    };

    Ok(certified_shortcut_result(mesh, shortcut))
}

fn certified_closed_boundary_touching_union_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ExactBoundaryTouchingReport>, MeshError> {
    let Some(report) =
        certified_closed_boundary_touching_regularized_report_from_graph(graph, left, right)?
    else {
        return Ok(None);
    };
    // Regularized solid union may preserve separate shells when two closed
    // solids meet only on lower-dimensional boundary features. Positive-area
    // coplanar overlap is deliberately excluded here: those contacts need a
    // full face-patch or volumetric-cell certificate before the two closed
    // objects can be projected into one triangle mesh. This keeps the exact
    // regularized-set view of solid modeling described by Requicha,
    // "Representations for Rigid Solids: Theory, Methods, and Systems,"
    if report.blocker.candidate_pairs
        + report.blocker.coplanar_touching_pairs
        + report.blocker.coplanar_overlapping_pairs
        == 0
    {
        return Ok(None);
    }
    if report.blocker.coplanar_overlapping_pairs != 0 {
        let coplanar_evidence =
            CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
        coplanar_evidence.validate().map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!(
                    "exact closed-boundary-touch coplanar evidence validation failed: {error:?}"
                ),
            ))
        })?;
        if coplanar_evidence.positive_area_coplanar_overlapping_pairs != 0 {
            return Ok(None);
        }
    }
    Ok(Some(report))
}

fn certified_closed_boundary_touching_regularized_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ExactBoundaryTouchingReport>, MeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(None);
    }
    let report = boundary_touching_report_from_graph(graph, left, right)?;
    certified_closed_boundary_touching_regularized_report_from_report(report)
}

fn certified_closed_boundary_touching_regularized_report_from_report(
    report: ExactBoundaryTouchingReport,
) -> Result<Option<ExactBoundaryTouchingReport>, MeshError> {
    if !report.is_certified() {
        return Ok(None);
    }
    report.validate().map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact closed-boundary-touch report validation failed: {error:?}"),
        ))
    })?;
    Ok(Some(report))
}

fn certified_closed_boundary_touching_support_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ExactBooleanSupport>, MeshError> {
    Ok(match operation {
        ExactBooleanOperation::Union
            if certified_closed_boundary_touching_union_report_from_graph(graph, left, right)?
                .is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion)
        }
        ExactBooleanOperation::Intersection
            if certified_closed_boundary_touching_regularized_report_from_graph(
                graph, left, right,
            )?
            .is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection)
        }
        ExactBooleanOperation::Difference
            if certified_closed_boundary_touching_regularized_report_from_graph(
                graph, left, right,
            )?
            .is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference)
        }
        _ => None,
    })
}

fn certified_closed_boundary_only_contact_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, MeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(false);
    }
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    evidence.validate().map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact boundary-only coplanar evidence validation failed: {error:?}"),
        ))
    })?;
    Ok(evidence.obstacle == CoplanarVolumetricCellObstacle::BoundaryOnlyContact)
}

fn certified_closed_boundary_only_contact_support_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ExactBooleanSupport>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || !certified_closed_boundary_only_contact_from_graph(graph, left, right)?
    {
        return Ok(None);
    }
    if operation == ExactBooleanOperation::Union
        && certified_boolmesh_split_support_from_graph(graph, left, right, operation)
    {
        return Ok(Some(ExactBooleanSupport::CertifiedBoolMeshSplit));
    }
    Ok(Some(match operation {
        ExactBooleanOperation::Union => ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion,
        ExactBooleanOperation::Intersection => {
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
        }
        ExactBooleanOperation::Difference => {
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
        }
        ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
    }))
}

fn boolean_closed_boundary_only_contact_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if !certified_closed_boundary_only_contact_from_graph(graph, left, right)? {
        return Ok(None);
    }
    if operation == ExactBooleanOperation::Union
        && let Some(result) = boolean_boolmesh_split_meshes_from_graph(
            graph,
            left,
            right,
            ExactBooleanOperation::Union,
            validation,
        )?
    {
        return Ok(Some(result));
    }
    let (mesh, shortcut) = match operation {
        ExactBooleanOperation::Union => (
            concatenate_meshes_with_options(
                left,
                right,
                false,
                "exact closed-boundary-only regularized union preserving separate shells",
                validation,
            )?,
            ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion,
        ),
        ExactBooleanOperation::Intersection => (
            empty_mesh(
                "empty exact closed-boundary-only regularized intersection",
                validation,
            )?,
            ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection,
        ),
        ExactBooleanOperation::Difference => (
            copy_mesh(
                left,
                "exact closed-boundary-only regularized difference keeps left",
                validation,
            )?,
            ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
        ),
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    Ok(Some(certified_shortcut_result(mesh, shortcut)))
}

/// Materialize the regularized union for closed lower-dimensional contact.
fn boolean_closed_boundary_touching_union(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
    report: ExactBoundaryTouchingReport,
) -> Result<ExactBooleanResult, MeshError> {
    validate_consumed_boundary_touching_report(&report, "closed-boundary-touch union")?;
    if let Some(result) = boolean_boolmesh_split_meshes_from_graph(
        graph,
        left,
        right,
        ExactBooleanOperation::Union,
        validation,
    )? {
        return Ok(result);
    }
    Ok(certified_shortcut_result(
        concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact closed-boundary-touch regularized union preserving separate shells",
            validation,
        )?,
        ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion,
    ))
}

/// Materialize the empty regularized intersection for closed boundary contact.
fn boolean_closed_boundary_touching_intersection(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
    report: ExactBoundaryTouchingReport,
) -> Result<ExactBooleanResult, MeshError> {
    validate_consumed_boundary_touching_report(&report, "closed-boundary-touch intersection")?;
    if let Some(result) = boolean_boolmesh_split_meshes_from_graph(
        graph,
        left,
        right,
        ExactBooleanOperation::Intersection,
        validation,
    )? {
        return Ok(result);
    }
    Ok(certified_shortcut_result(
        empty_mesh(
            "empty exact closed-boundary-touch regularized intersection",
            validation,
        )?,
        ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection,
    ))
}

/// Materialize the left-preserving difference for closed boundary contact.
fn boolean_closed_boundary_touching_difference(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
    report: ExactBoundaryTouchingReport,
) -> Result<ExactBooleanResult, MeshError> {
    validate_consumed_boundary_touching_report(&report, "closed-boundary-touch difference")?;
    if let Some(result) = boolean_boolmesh_split_meshes_from_graph(
        graph,
        left,
        right,
        ExactBooleanOperation::Difference,
        validation,
    )? {
        return Ok(result);
    }
    Ok(certified_shortcut_result(
        copy_mesh(
            left,
            "exact closed-boundary-touch regularized difference keeps left",
            validation,
        )?,
        ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
    ))
}

fn validate_consumed_boundary_touching_report(
    report: &ExactBoundaryTouchingReport,
    label: &str,
) -> Result<(), MeshError> {
    report.validate().map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact {label} consumed invalid certificate: {error:?}"),
        ))
    })
}

fn boolean_boundary_touching_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if boundary_policy == ExactBoundaryBooleanPolicy::Reject {
        return Ok(None);
    }
    let report = boundary_touching_report_from_graph(graph, left, right)?;
    if !report.is_certified() {
        return Ok(None);
    }
    validate_consumed_boundary_touching_report(&report, "boundary-policy projection")?;

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

    Ok(Some(boundary_policy_shortcut_result(mesh, operation)))
}

/// Certify whether retained graph pairs are exclusively boundary-only contacts.
///
/// The report keeps the exact graph relation counts used by boundary-policy
/// preflight and by [`boolean_exact_with_boundary_policy`]. Boundary-only
/// topology is intentionally not silently materialized by the default named
/// triangle-mesh-only result to be an explicit caller policy.
pub fn certify_boundary_touching_report(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactBoundaryTouchingReport, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    boundary_touching_report_from_graph(&graph, left, right)
}

/// Certify whether a named operation needs planar arrangement output.
///
/// The report is intentionally narrower than full winding preflight. It only
/// answers the coplanar positive-area case where exact graph facts prove that
/// intersection, union, or difference output is a planar arrangement problem. Existing
/// single-triangle and convex multi-face coplanar shortcuts are reported as
/// already materialized so callers can distinguish a missing output model from
/// a handled certified fragment.
pub fn certify_planar_arrangement_report(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactPlanarArrangementReport, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(planar_arrangement_report(
            operation,
            ExactPlanarArrangementStatus::NotNamedOperation,
            false,
            0,
            0,
            GraphRelationCounts::default(),
            None,
        ));
    }

    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    planar_arrangement_report_from_graph(&graph, left, right, operation)
}

/// Certify whether exact graph construction needs refinement.
///
/// This is the standalone report form of the `UnresolvedGraph` preflight
/// branch. It separates unknown predicate outcomes and failed exact
/// constructions from later boundary, planar-arrangement, or winding policy,
/// rather than being folded into a generic unsupported boolean.
pub fn certify_refinement_report(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactRefinementReport, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    Ok(refinement_report_from_graph(&graph, operation))
}

/// Prepare and report the exact facts needed by a future winding policy.
///
/// This function stops at the same boundary as unsupported nontrivial named
/// booleans: it extracts the certified graph, rejects unresolved/boundary/
/// planar-arrangement cases into explicit statuses, then validates split
/// regions and records opposite-plane classifications. It is an auditable
pub fn certify_winding_readiness_report(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactWindingReadinessReport, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    winding_readiness_report_from_graph(&graph, left, right, operation)
}

/// Validate the retained graph/source-handle handoff for public reports.
///
/// Boolean preflight and report constructors are public exact computation
/// boundaries. They must reject a retained graph whose face, edge, vertex, or
/// plane handles no longer replay against the source meshes before policy
/// includes the combinatorial object handles attached to predicate evidence,
/// not just the numeric predicate result.
fn validate_graph_source_handoff(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<(), MeshError> {
    graph
        .validate_against_sources(left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("retained exact intersection graph failed source replay: {error:?}"),
            ))
        })
}

pub(crate) fn boundary_touching_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactBoundaryTouchingReport, MeshError> {
    let graph_had_unknowns = graph.has_unknowns();
    let counts = graph_relation_counts(graph);
    let status = if graph_had_unknowns {
        ExactBoundaryTouchingStatus::GraphUnknowns
    } else if graph_requires_boundary_policy(graph, left, right)? {
        ExactBoundaryTouchingStatus::Certified
    } else {
        ExactBoundaryTouchingStatus::NotBoundaryOnly
    };
    Ok(ExactBoundaryTouchingReport {
        status,
        graph_had_unknowns,
        retained_face_pairs: graph.face_pairs.len(),
        retained_events: graph.event_count(),
        blocker: counts.into_blocker(if graph_had_unknowns {
            ExactBooleanBlockerKind::NeedsRefinement
        } else {
            ExactBooleanBlockerKind::NeedsBoundaryPolicy
        }),
    })
}

fn refinement_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    operation: ExactBooleanOperation,
) -> ExactRefinementReport {
    let counts = graph_relation_counts(graph);
    let graph_had_unknowns = graph.has_unknowns();
    let needs_refinement = graph_had_unknowns || counts.construction_failed_events > 0;
    ExactRefinementReport {
        operation,
        status: if needs_refinement {
            ExactRefinementStatus::Required
        } else {
            ExactRefinementStatus::NotRequired
        },
        graph_had_unknowns,
        retained_face_pairs: graph.face_pairs.len(),
        retained_events: graph.event_count(),
        blocker: needs_refinement
            .then(|| counts.into_blocker(ExactBooleanBlockerKind::NeedsRefinement)),
    }
}

fn planar_arrangement_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactPlanarArrangementReport, MeshError> {
    let graph_had_unknowns = graph.has_unknowns();
    let counts = graph_relation_counts(graph);
    let arrangement_readiness = if graph_had_unknowns {
        None
    } else {
        Some(graph.coplanar_arrangement_readiness_report(left, right)?)
    };
    let status = if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        ExactPlanarArrangementStatus::NotNamedOperation
    } else if graph_had_unknowns {
        ExactPlanarArrangementStatus::GraphUnknowns
    } else if coplanar_surface_output_already_materialized(left, right, operation) {
        ExactPlanarArrangementStatus::AlreadyMaterialized
    } else if graph_requires_boundary_policy(graph, left, right)? {
        ExactPlanarArrangementStatus::BoundaryPolicyRequired
    } else if graph_requires_planar_arrangement(graph) {
        ExactPlanarArrangementStatus::Required
    } else {
        ExactPlanarArrangementStatus::NoPositiveOverlap
    };
    Ok(planar_arrangement_report(
        operation,
        status,
        graph_had_unknowns,
        graph.face_pairs.len(),
        graph.event_count(),
        counts,
        arrangement_readiness,
    ))
}

fn planar_arrangement_report(
    operation: ExactBooleanOperation,
    status: ExactPlanarArrangementStatus,
    graph_had_unknowns: bool,
    retained_face_pairs: usize,
    retained_events: usize,
    counts: GraphRelationCounts,
    arrangement_readiness: Option<super::graph::CoplanarArrangementReadinessReport>,
) -> ExactPlanarArrangementReport {
    let blocker_kind = match status {
        ExactPlanarArrangementStatus::GraphUnknowns => ExactBooleanBlockerKind::NeedsRefinement,
        ExactPlanarArrangementStatus::BoundaryPolicyRequired => {
            ExactBooleanBlockerKind::NeedsBoundaryPolicy
        }
        _ => ExactBooleanBlockerKind::NeedsPlanarArrangement,
    };
    ExactPlanarArrangementReport {
        operation,
        status,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        blocker: counts.into_blocker(blocker_kind),
        arrangement_readiness,
    }
}

fn coplanar_surface_output_already_materialized(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> bool {
    if certify_coplanar_convex_surface_equivalence(left, right).is_some()
        || certify_coplanar_convex_surface_containment(left, right).is_some()
    {
        return true;
    }
    if left.triangles().len() == 1 && right.triangles().len() == 1 {
        return single_triangle_coplanar_output_already_materialized(left, right, operation);
    }
    match operation {
        ExactBooleanOperation::Intersection => {
            arrange_coplanar_convex_surface_intersection(left, right).is_some()
                || arrange_coplanar_convex_surface_multi_intersection(left, right).is_some()
                || arrange_coplanar_orthogonal_surface_intersection(left, right).is_some()
                || arrange_coplanar_affine_surface_intersection(left, right).is_some()
                || intersect_single_triangle_coplanar_surfaces(left, right).is_some()
        }
        ExactBooleanOperation::Union => {
            arrange_coplanar_convex_surface_union(left, right).is_some()
                || arrange_coplanar_convex_surface_component_union(left, right).is_some()
                || arrange_coplanar_convex_surface_multi_union(left, right).is_some()
                || arrange_coplanar_surface_component_union(left, right).is_some()
                || arrange_coplanar_surface_component_holed_union(left, right).is_some()
                || arrange_coplanar_surface_multi_component_union(left, right).is_some()
                || arrange_coplanar_surface_point_touch_union(left, right).is_some()
                || arrange_coplanar_orthogonal_surface_union(left, right).is_some()
                || arrange_coplanar_affine_surface_union(left, right).is_some()
                || union_single_triangle_coplanar_surfaces(left, right).is_some()
                || arrange_single_triangle_coplanar_union(left, right).is_some()
        }
        ExactBooleanOperation::Difference => {
            arrange_coplanar_convex_surface_difference(left, right).is_some()
                || arrange_coplanar_convex_surface_multi_difference(left, right).is_some()
                || arrange_coplanar_surface_component_difference(left, right).is_some()
                || arrange_coplanar_surface_multi_difference(left, right).is_some()
                || arrange_coplanar_surface_cutter_hole_contact_difference(left, right).is_some()
                || arrange_coplanar_convex_surface_holed_difference(left, right).is_some()
                || arrange_coplanar_convex_surface_multi_holed_difference(left, right).is_some()
                || arrange_coplanar_orthogonal_surface_difference(left, right).is_some()
                || arrange_coplanar_affine_surface_difference(left, right).is_some()
                || difference_single_triangle_coplanar_surfaces(left, right).is_some()
                || arrange_single_triangle_coplanar_difference(left, right).is_some()
                || arrange_single_triangle_coplanar_holed_difference(left, right).is_some()
        }
        ExactBooleanOperation::SelectedRegions(_) => false,
    }
}

fn single_triangle_coplanar_output_already_materialized(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> bool {
    match operation {
        ExactBooleanOperation::Intersection => {
            intersect_single_triangle_coplanar_surfaces(left, right).is_some()
        }
        ExactBooleanOperation::Union => {
            union_single_triangle_coplanar_surfaces(left, right).is_some()
                || arrange_single_triangle_coplanar_union(left, right).is_some()
        }
        ExactBooleanOperation::Difference => {
            difference_single_triangle_coplanar_surfaces(left, right).is_some()
                || arrange_single_triangle_coplanar_difference(left, right).is_some()
                || arrange_single_triangle_coplanar_holed_difference(left, right).is_some()
        }
        ExactBooleanOperation::SelectedRegions(_) => false,
    }
}

fn winding_readiness_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactWindingReadinessReport, MeshError> {
    let graph_had_unknowns = graph.has_unknowns();
    let counts = graph_relation_counts(graph);
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::NotNamedOperation,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsWinding),
            None,
            None,
        ));
    }
    if graph_had_unknowns {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::GraphUnknowns,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsRefinement),
            None,
            None,
        ));
    }
    if graph_requires_boundary_policy(graph, left, right)? {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::BoundaryPolicyRequired,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsBoundaryPolicy),
            None,
            None,
        ));
    }
    let planar_report = planar_arrangement_report_from_graph(graph, left, right, operation)?;
    if planar_report.is_required() {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::PlanarArrangementRequired,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsPlanarArrangement),
            planar_report.arrangement_readiness,
            None,
        ));
    }
    if planar_report.status == ExactPlanarArrangementStatus::AlreadyMaterialized {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsPlanarArrangement),
            planar_report.arrangement_readiness,
            None,
        ));
    }
    if let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        graph,
        left,
        right,
        operation,
        ValidationPolicy::CLOSED,
        VolumetricWindingMaterializationFailure::ReturnNone,
    )? {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::Ready,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            materialized.triangulations.len(),
            materialized.region_classifications,
            if graph_requires_coplanar_volumetric_cells_for_sources(graph, left, right) {
                counts.into_blocker(ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells)
            } else {
                counts.into_blocker(ExactBooleanBlockerKind::NeedsWinding)
            },
            None,
            coplanar_volumetric_evidence_if_required(graph, left, right),
        ));
    }
    if graph_requires_coplanar_volumetric_cells_for_sources(graph, left, right) {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::CoplanarVolumetricCellsRequired,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells),
            None,
            coplanar_volumetric_evidence_if_required(graph, left, right),
        ));
    }
    if graph.face_pairs.is_empty() {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::NoNontrivialOverlap,
            graph_had_unknowns,
            0,
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsWinding),
            None,
            None,
        ));
    }

    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
    Ok(winding_readiness_report(
        operation,
        ExactWindingReadinessStatus::Ready,
        graph_had_unknowns,
        graph.face_pairs.len(),
        graph.event_count(),
        region_plan.regions.len(),
        region_classifications,
        counts.into_blocker(ExactBooleanBlockerKind::NeedsWinding),
        None,
        None,
    ))
}

fn winding_readiness_report(
    operation: ExactBooleanOperation,
    status: ExactWindingReadinessStatus,
    graph_had_unknowns: bool,
    retained_face_pairs: usize,
    retained_events: usize,
    region_count: usize,
    region_classifications: Vec<FaceRegionPlaneClassification>,
    blocker: ExactBooleanBlocker,
    arrangement_readiness: Option<super::graph::CoplanarArrangementReadinessReport>,
    coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
) -> ExactWindingReadinessReport {
    ExactWindingReadinessReport {
        operation,
        status,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        region_count,
        region_classifications,
        blocker,
        arrangement_readiness,
        coplanar_volumetric_evidence,
    }
}

fn boolean_convex_containment_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(shortcut) =
        certified_convex_boolean_shortcut_from_graph(graph, left, right, operation)?
    else {
        return Ok(None);
    };

    let mesh = match (
        shortcut.left_in_right.relation,
        shortcut.right_in_left.relation,
        operation,
    ) {
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
        (_, _, ExactBooleanOperation::Union)
            if convex_boundary_containment_is_supported(
                &shortcut.left_in_right,
                &shortcut.right_in_left,
            ) =>
        {
            copy_mesh(
                right,
                "exact convex boundary containment union keeps outer right",
                validation,
            )?
        }
        (_, _, ExactBooleanOperation::Intersection)
            if convex_boundary_containment_is_supported(
                &shortcut.left_in_right,
                &shortcut.right_in_left,
            ) =>
        {
            copy_mesh(
                left,
                "exact convex boundary containment intersection keeps inner left",
                validation,
            )?
        }
        (_, _, ExactBooleanOperation::Difference)
            if convex_boundary_containment_is_supported(
                &shortcut.left_in_right,
                &shortcut.right_in_left,
            ) =>
        {
            empty_mesh(
                "empty exact convex boundary containment difference",
                validation,
            )?
        }
        (_, _, ExactBooleanOperation::Union)
            if convex_boundary_containment_is_supported(
                &shortcut.right_in_left,
                &shortcut.left_in_right,
            ) =>
        {
            copy_mesh(
                left,
                "exact convex boundary containment union keeps outer left",
                validation,
            )?
        }
        (_, _, ExactBooleanOperation::Intersection)
            if convex_boundary_containment_is_supported(
                &shortcut.right_in_left,
                &shortcut.left_in_right,
            ) =>
        {
            copy_mesh(
                right,
                "exact convex boundary containment intersection keeps inner right",
                validation,
            )?
        }
        (_, _, ExactBooleanOperation::Difference)
            if convex_boundary_containment_is_supported(
                &shortcut.right_in_left,
                &shortcut.left_in_right,
            ) =>
        {
            let Some(certificate) = shortcut.contained_boundary_difference.as_ref() else {
                return Ok(None);
            };
            let Some(difference) =
                materialize_contained_boundary_difference_from_retained_certificate(
                    left,
                    right,
                    certificate,
                    validation,
                )
            else {
                return Ok(None);
            };
            difference.mesh
        }
        (_, _, ExactBooleanOperation::SelectedRegions(_)) => unreachable!("handled by caller"),
        _ => return Ok(None),
    };

    Ok(Some(certified_shortcut_result(
        mesh,
        match shortcut.support {
            ExactBooleanSupport::CertifiedConvexContainment => {
                ExactBooleanShortcutKind::ConvexContainment
            }
            ExactBooleanSupport::CertifiedConvexSeparated => {
                ExactBooleanShortcutKind::ConvexSeparated
            }
            _ => unreachable!("convex support helper returns only certified convex shortcuts"),
        },
    )))
}

fn materialize_winding_containment_meshes(
    shortcut: CertifiedWindingBooleanShortcut,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let mesh = match (
        shortcut.left_in_right.relation,
        shortcut.right_in_left.relation,
        operation,
    ) {
        (ClosedMeshWindingMeshRelation::StrictlyInside, _, ExactBooleanOperation::Union) => {
            copy_mesh(
                right,
                "exact winding containment union keeps outer right",
                validation,
            )?
        }
        (ClosedMeshWindingMeshRelation::StrictlyInside, _, ExactBooleanOperation::Intersection) => {
            copy_mesh(
                left,
                "exact winding containment intersection keeps inner left",
                validation,
            )?
        }
        (ClosedMeshWindingMeshRelation::StrictlyInside, _, ExactBooleanOperation::Difference) => {
            empty_mesh("empty exact winding containment difference", validation)?
        }
        (_, ClosedMeshWindingMeshRelation::StrictlyInside, ExactBooleanOperation::Union) => {
            copy_mesh(
                left,
                "exact winding containment union keeps outer left",
                validation,
            )?
        }
        (_, ClosedMeshWindingMeshRelation::StrictlyInside, ExactBooleanOperation::Intersection) => {
            copy_mesh(
                right,
                "exact winding containment intersection keeps inner right",
                validation,
            )?
        }
        (_, ClosedMeshWindingMeshRelation::StrictlyInside, ExactBooleanOperation::Difference) => {
            concatenate_meshes_with_options(
                left,
                right,
                true,
                "exact winding containment difference with inner reversed shell",
                validation,
            )?
        }
        (
            ClosedMeshWindingMeshRelation::Outside,
            ClosedMeshWindingMeshRelation::Outside,
            ExactBooleanOperation::Union,
        ) => concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact winding separated union",
            validation,
        )?,
        (
            ClosedMeshWindingMeshRelation::Outside,
            ClosedMeshWindingMeshRelation::Outside,
            ExactBooleanOperation::Intersection,
        ) => empty_mesh("empty exact winding separated intersection", validation)?,
        (
            ClosedMeshWindingMeshRelation::Outside,
            ClosedMeshWindingMeshRelation::Outside,
            ExactBooleanOperation::Difference,
        ) => copy_mesh(
            left,
            "exact winding separated difference keeps left",
            validation,
        )?,
        (_, _, ExactBooleanOperation::SelectedRegions(_)) => unreachable!("handled by caller"),
        _ => return Ok(None),
    };

    Ok(Some(certified_shortcut_result(
        mesh,
        match shortcut.support {
            ExactBooleanSupport::CertifiedWindingContainment => {
                ExactBooleanShortcutKind::WindingContainment
            }
            ExactBooleanSupport::CertifiedWindingSeparated => {
                ExactBooleanShortcutKind::WindingSeparated
            }
            _ => unreachable!("winding support helper returns only winding shortcuts"),
        },
    )))
}

type VolumetricWindingRegionPlan = (
    Vec<FaceRegionPlaneClassification>,
    Vec<FaceRegionTriangulation>,
    Vec<ExactVolumetricRegionClassification>,
);

struct MaterializedVolumetricWindingRegionPlan {
    region_classifications: Vec<FaceRegionPlaneClassification>,
    triangulations: Vec<FaceRegionTriangulation>,
    volumetric_classifications: Vec<ExactVolumetricRegionClassification>,
    assembly: ExactBooleanAssemblyPlan,
    mesh: ExactMesh,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum VolumetricWindingMaterializationFailure {
    ReturnError,
    ReturnNone,
}

fn boolean_volumetric_winding_regions_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        graph,
        left,
        right,
        operation,
        validation,
        VolumetricWindingMaterializationFailure::ReturnError,
    )?
    else {
        return Ok(None);
    };
    let result = ExactBooleanResult {
        kind: ExactBooleanResultKind::WindingMaterialized { operation },
        graph_had_unknowns: false,
        region_classifications: materialized.region_classifications,
        triangulations: materialized.triangulations,
        assembly: materialized.assembly,
        volumetric_classifications: materialized.volumetric_classifications,
        mesh: materialized.mesh,
    };
    result.validate().map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact winding-materialized result validation failed: {error:?}"),
        ))
    })?;
    Ok(Some(result))
}

fn boolean_retained_graph_fallback_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if let Some(result) = boolean_closed_boundary_only_contact_meshes_from_graph(
        graph, left, right, operation, validation,
    )? {
        return Ok(Some(result));
    }

    match boolean_volumetric_winding_regions_from_graph(graph, left, right, operation, validation) {
        Ok(Some(result)) => return Ok(Some(result)),
        Ok(None) => {}
        Err(error) => {
            if error_can_retry_certified_boolmesh(&error) {
                match execute_exact_boolmesh_port_from_graph(
                    left, right, operation, validation, graph,
                ) {
                    Ok(execution) => {
                        return Ok(Some(certified_shortcut_result(
                            execution.mesh,
                            execution.shortcut,
                        )));
                    }
                    Err(ExactBoolMeshValidationError::PortBlocked(
                        stage @ (ExactBoolMeshKernelStage::Triangulation
                        | ExactBoolMeshKernelStage::Cleanup),
                    )) => return Err(boolmesh_late_export_blocker_error(stage)),
                    _ => {}
                }
            }
            return Err(error);
        }
    }

    if let Some(result) =
        boolean_boolmesh_port_meshes_from_graph(graph, left, right, operation, validation)?
    {
        return Ok(Some(result));
    }

    if let Some(shortcut) = certified_winding_boolean_shortcut_from_graph(graph, left, right)?
        && let Some(result) =
            materialize_winding_containment_meshes(shortcut, left, right, operation, validation)?
    {
        return Ok(Some(result));
    }

    if let Some(result) = boolean_boundary_touching_meshes_from_graph(
        graph,
        left,
        right,
        operation,
        validation,
        boundary_policy,
    )? {
        return Ok(Some(result));
    }

    Ok(None)
}

fn materialize_volumetric_winding_region_plan_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    failure: VolumetricWindingMaterializationFailure,
) -> Result<Option<MaterializedVolumetricWindingRegionPlan>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let Some((region_classifications, triangulations, volumetric_classifications)) =
        volumetric_winding_region_plan_from_graph(graph, left, right)?
    else {
        return Ok(None);
    };
    if !volumetric_classifications
        .iter()
        .all(|classification| classification.relation.is_materialization_decided())
    {
        return Ok(None);
    }
    if !operation_retains_any_volumetric_region(
        operation,
        &triangulations,
        &volumetric_classifications,
    ) {
        return Ok(None);
    }

    let assembly_result =
        ExactBooleanAssemblyPlan::from_region_triangulations_with_triangle_retention_and_sources(
            &triangulations,
            left,
            right,
            |triangulation, triangle| {
                volumetric_retention_for_operation(
                    operation,
                    triangulation,
                    triangle,
                    &volumetric_classifications,
                )
            },
        );
    let mut assembly = match assembly_result {
        Ok(assembly) => assembly,
        Err(_error) if failure == VolumetricWindingMaterializationFailure::ReturnNone => {
            return Ok(None);
        }
        Err(error) => {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexOutOfBounds,
                format!("exact winding region assembly failed: {error}"),
            )));
        }
    };
    if let Err(error) = assembly.split_disconnected_vertex_fans() {
        if failure == VolumetricWindingMaterializationFailure::ReturnNone {
            return Ok(None);
        }
        return Err(MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::IndexOutOfBounds,
            format!("exact winding region vertex-fan split failed: {error}"),
        )));
    }
    let mesh = match assembly.checked_to_exact_mesh_with_sources(left, right, validation) {
        Ok(mesh) => mesh,
        Err(_error)
            if failure == VolumetricWindingMaterializationFailure::ReturnNone
                || graph_requires_coplanar_volumetric_cells_for_sources(graph, left, right) =>
        {
            return Ok(None);
        }
        Err(error) => return Err(error),
    };
    Ok(Some(MaterializedVolumetricWindingRegionPlan {
        region_classifications,
        triangulations,
        volumetric_classifications,
        assembly,
        mesh,
    }))
}

fn volumetric_winding_region_plan_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<VolumetricWindingRegionPlan>, MeshError> {
    let counts = graph_relation_counts(graph);
    if graph.has_unknowns()
        || graph.face_pairs.is_empty()
        || counts.unknown_pairs != 0
        || counts.construction_failed_events != 0
        || !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
    {
        return Ok(None);
    }
    if graph_requires_boundary_policy(graph, left, right)? {
        return Ok(None);
    }

    let cell_plan = match triangulate_all_face_cells_with_cdt(graph, left, right) {
        Ok(plan) => plan,
        Err(_error) if graph_requires_coplanar_volumetric_cells_for_sources(graph, left, right) => {
            // Coplanar source-face overlaps can expose constraint-normalization
            // cases that are not part of the current bounded volumetric cell
            // receives `RequiresCoplanarVolumetricCells` instead of a generic
            // triangulation failure or a tolerance fallback.
            return Ok(None);
        }
        Err(error) => {
            return Err(MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::DegenerateTriangle,
                format!("exact winding face-cell triangulation failed: {error}"),
            )));
        }
    };
    let Some((region_plan, triangulations)) = cell_plan else {
        return Ok(None);
    };
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
    let volumetric_classifications =
        classify_triangulated_regions_against_opposite_meshes(&triangulations, left, right)
            .map_err(volumetric_error)?;
    Ok(Some((
        region_classifications,
        triangulations,
        volumetric_classifications,
    )))
}

fn operation_retains_any_volumetric_region(
    operation: ExactBooleanOperation,
    triangulations: &[FaceRegionTriangulation],
    classifications: &[ExactVolumetricRegionClassification],
) -> bool {
    triangulations.iter().any(|triangulation| {
        triangulation.triangles.chunks_exact(3).any(|triangle| {
            volumetric_retention_for_operation(
                operation,
                triangulation,
                [triangle[0], triangle[1], triangle[2]],
                classifications,
            ) != ExactRegionRetention::Drop
        })
    })
}

fn volumetric_retention_for_operation(
    operation: ExactBooleanOperation,
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
    classifications: &[ExactVolumetricRegionClassification],
) -> ExactRegionRetention {
    let Some(classification) = classifications.iter().find(|classification| {
        classification.region_side == triangulation.side
            && classification.region_face == triangulation.face
            && classification.triangle == triangle
    }) else {
        return ExactRegionRetention::Drop;
    };
    // Boundary cells arise when every exact representative for a source-face
    // cell lies on the opposite closed mesh boundary. In mixed coplanar
    // volumetric overlaps that means the same geometric patch is normally
    // explicit, so we consume it through a deterministic owner policy instead
    // of pretending it is inside or outside: union/intersection keep the left
    // copy and drop the coincident right copy; difference drops coincident
    // boundary cells because the overlapped volume is removed from the left
    // operand and right boundary faces are only used as reversed interior caps.
    match (operation, triangulation.side, classification.relation) {
        (ExactBooleanOperation::Union, _, ExactVolumetricRegionRelation::Outside)
        | (ExactBooleanOperation::Union, MeshSide::Left, ExactVolumetricRegionRelation::Boundary)
        | (ExactBooleanOperation::Intersection, _, ExactVolumetricRegionRelation::Inside)
        | (
            ExactBooleanOperation::Intersection,
            MeshSide::Left,
            ExactVolumetricRegionRelation::Boundary,
        )
        | (
            ExactBooleanOperation::Difference,
            MeshSide::Left,
            ExactVolumetricRegionRelation::Outside,
        ) => ExactRegionRetention::Keep,
        (
            ExactBooleanOperation::Difference,
            MeshSide::Right,
            ExactVolumetricRegionRelation::Inside,
        ) => ExactRegionRetention::KeepReversed,
        _ => ExactRegionRetention::Drop,
    }
}

fn boolean_convex_single_cap_difference_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Difference {
        return Ok(None);
    }
    let Some(difference) = subtract_closed_convex_solids_single_cap(left, right) else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &difference.mesh,
        "exact closed-convex single-cap difference",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::ConvexSingleCapDifference,
    )))
}

fn certified_convex_intersection_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    match operation {
        ExactBooleanOperation::Intersection
            if intersect_closed_convex_solids(left, right).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedConvexIntersection)
        }
        _ => None,
    }
}

fn certified_convex_single_cap_difference_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    match operation {
        ExactBooleanOperation::Difference
            if subtract_closed_convex_solids_single_cap(left, right).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedConvexSingleCapDifference)
        }
        _ => None,
    }
}

fn certified_contained_boundary_difference_support_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    match operation {
        ExactBooleanOperation::Difference
            if has_contained_boundary_difference_from_graph(left, right, graph) =>
        {
            Some(ExactBooleanSupport::CertifiedContainedBoundaryDifference)
        }
        _ => None,
    }
}

fn certified_contained_boundary_containment_support_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    let containment = contained_boundary_containment_from_graph(left, right, graph)?;
    match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection => {
            Some(ExactBooleanSupport::CertifiedContainedBoundaryContainment)
        }
        ExactBooleanOperation::Difference
            if containment == ContainedBoundaryContainment::RightContainsLeft =>
        {
            Some(ExactBooleanSupport::CertifiedContainedBoundaryContainment)
        }
        ExactBooleanOperation::Difference => None,
        ExactBooleanOperation::SelectedRegions(_) => None,
    }
}

struct CertifiedConvexBooleanShortcut {
    support: ExactBooleanSupport,
    left_in_right: ConvexSolidMeshClassification,
    right_in_left: ConvexSolidMeshClassification,
    contained_boundary_difference: Option<ContainedBoundaryDifferenceCertificate>,
}

fn certified_convex_boolean_support_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ExactBooleanSupport>, MeshError> {
    Ok(
        certified_convex_boolean_shortcut_from_graph(graph, left, right, operation)?
            .map(|shortcut| shortcut.support),
    )
}

fn certified_convex_boolean_shortcut_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<CertifiedConvexBooleanShortcut>, MeshError> {
    let relation_counts = graph_relation_counts(graph);
    if graph.has_unknowns() || relation_counts.construction_failed_events > 0 {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_convex_solid_report(left, right);
    let right_in_left = classify_mesh_vertices_against_convex_solid_report(right, left);
    if graph.face_pairs.is_empty() {
        let support = match (left_in_right.relation, right_in_left.relation) {
            (ConvexSolidMeshRelation::StrictlyInside, _)
            | (_, ConvexSolidMeshRelation::StrictlyInside) => {
                Some(ExactBooleanSupport::CertifiedConvexContainment)
            }
            (ConvexSolidMeshRelation::Outside, ConvexSolidMeshRelation::Outside) => {
                Some(ExactBooleanSupport::CertifiedConvexSeparated)
            }
            _ => None,
        };
        return Ok(support.map(|support| CertifiedConvexBooleanShortcut {
            support,
            left_in_right,
            right_in_left,
            contained_boundary_difference: None,
        }));
    }

    let left_boundary_inside_right =
        convex_boundary_containment_is_supported(&left_in_right, &right_in_left);
    let right_boundary_inside_left =
        convex_boundary_containment_is_supported(&right_in_left, &left_in_right);
    if matches!(
        operation,
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
    ) && (left_boundary_inside_right || right_boundary_inside_left)
    {
        return Ok(Some(CertifiedConvexBooleanShortcut {
            support: ExactBooleanSupport::CertifiedConvexContainment,
            left_in_right,
            right_in_left,
            contained_boundary_difference: None,
        }));
    }
    let contained_boundary_difference =
        if operation == ExactBooleanOperation::Difference && right_boundary_inside_left {
            contained_boundary_difference_certificate_from_graph(left, right, graph)
        } else {
            None
        };
    if operation == ExactBooleanOperation::Difference
        && (left_boundary_inside_right || contained_boundary_difference.is_some())
    {
        return Ok(Some(CertifiedConvexBooleanShortcut {
            support: ExactBooleanSupport::CertifiedConvexContainment,
            left_in_right,
            right_in_left,
            contained_boundary_difference,
        }));
    }

    Ok(None)
}

/// Return whether one certified convex solid is contained in another while
/// touching its boundary.
///
/// argues that such topology decisions must be retained as exact predicate
/// facts: every subject vertex is certified inside or on the container, at
/// least one vertex is exactly on the boundary, the container has at least one
/// vertex outside the subject so the relation is not relabeled equality, and
/// both meshes have been certified as convex solids by the two retained
/// reports. Convexity is the key promotion gate: once every vertex of one
/// convex solid is inside or on the other convex solid, a separate sampled
/// graph traversal is not allowed to veto the containment with a stale
/// tolerance-style crossing interpretation.
fn convex_boundary_containment_is_supported(
    subject_in_container: &ConvexSolidMeshClassification,
    container_in_subject: &ConvexSolidMeshClassification,
) -> bool {
    subject_in_container.solid_facts.is_certified_convex()
        && container_in_subject.solid_facts.is_certified_convex()
        && subject_in_container.vertices.iter().all(|vertex| {
            matches!(
                vertex.relation,
                ConvexSolidPointRelation::Inside | ConvexSolidPointRelation::Boundary
            )
        })
        && subject_in_container
            .vertices
            .iter()
            .any(|vertex| matches!(vertex.relation, ConvexSolidPointRelation::Boundary))
        && container_in_subject
            .vertices
            .iter()
            .any(|vertex| matches!(vertex.relation, ConvexSolidPointRelation::Outside))
}

struct CertifiedWindingBooleanShortcut {
    support: ExactBooleanSupport,
    left_in_right: ClosedMeshWindingMeshReport,
    right_in_left: ClosedMeshWindingMeshReport,
}

fn certified_winding_boolean_support_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ExactBooleanSupport>, MeshError> {
    Ok(
        certified_winding_boolean_shortcut_from_graph(graph, left, right)?
            .map(|shortcut| shortcut.support),
    )
}

fn certified_winding_boolean_shortcut_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<CertifiedWindingBooleanShortcut>, MeshError> {
    if graph.has_unknowns()
        || !graph.face_pairs.is_empty()
        || !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
    {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    left_in_right.validate().map_err(winding_error)?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left.validate().map_err(winding_error)?;
    let support = match (left_in_right.relation, right_in_left.relation) {
        (ClosedMeshWindingMeshRelation::StrictlyInside, _)
        | (_, ClosedMeshWindingMeshRelation::StrictlyInside) => {
            Some(ExactBooleanSupport::CertifiedWindingContainment)
        }
        (ClosedMeshWindingMeshRelation::Outside, ClosedMeshWindingMeshRelation::Outside) => {
            Some(ExactBooleanSupport::CertifiedWindingSeparated)
        }
        _ => None,
    };
    Ok(support.map(|support| CertifiedWindingBooleanShortcut {
        support,
        left_in_right,
        right_in_left,
    }))
}

fn winding_error(error: WindingReportError) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("exact winding report/source replay failed: {error:?}"),
    ))
}

fn volumetric_error(error: ExactVolumetricRegionError) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("exact volumetric winding region report/source replay failed: {error:?}"),
    ))
}

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

fn meshes_are_certified_bounds_disjoint(left: &ExactMesh, right: &ExactMesh) -> bool {
    let (Some(left_bounds), Some(right_bounds)) = (&left.bounds().mesh, &right.bounds().mesh)
    else {
        return left.triangles().is_empty() || right.triangles().is_empty();
    };
    left_bounds.classify_intersection(right_bounds).value() == Some(AabbIntersectionKind::Disjoint)
}

fn meshes_are_certified_identical(left: &ExactMesh, right: &ExactMesh) -> bool {
    left.triangles() == right.triangles()
        && left.vertices().len() == right.vertices().len()
        && vertices_are_certified_equal(left, right)
}

fn meshes_are_certified_same_surface(left: &ExactMesh, right: &ExactMesh) -> bool {
    certify_same_surface_report(left, right).is_certified()
}

/// Certify whether two meshes represent the same triangle surface.
///
/// The report preserves the exact coordinate-equality predicate certificates
/// used to find a vertex bijection and the sorted triangle sets compared after
/// remapping. This is the auditable form of the same-surface shortcut used by
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

fn vertices_are_certified_equal(left: &ExactMesh, right: &ExactMesh) -> bool {
    left.vertices()
        .iter()
        .zip(right.vertices())
        .all(|(left, right)| {
            let left = left.clone();
            let right = right.clone();
            compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
                && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
                && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
        })
}

fn certified_vertex_permutation_report(
    left: &ExactMesh,
    right: &ExactMesh,
) -> (Vec<usize>, Vec<PredicateUse>, ExactSameSurfaceStatus) {
    let mut left_to_right = Vec::with_capacity(left.vertices().len());
    let mut used_right = vec![false; right.vertices().len()];
    let mut predicates = Vec::new();

    for left_vertex in left.vertices() {
        let left_point = left_vertex.clone();
        let mut match_index = None;
        let mut saw_undecided = false;
        for (right_index, right_vertex) in right.vertices().iter().enumerate() {
            if used_right[right_index] {
                continue;
            }
            let right_point = right_vertex.clone();
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

fn invert_permutation(permutation: &[usize]) -> Vec<usize> {
    let mut inverse = vec![0; permutation.len()];
    for (left_index, &right_index) in permutation.iter().enumerate() {
        inverse[right_index] = left_index;
    }
    inverse
}

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

fn boolean_disjoint_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let execution = execute_exact_boolmesh_bounds_disjoint(left, right, operation, validation)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("exact boolmesh bounds-disjoint slice failed: {error:?}"),
            ))
        })?;

    Ok(certified_shortcut_result(
        execution.mesh,
        execution.shortcut,
    ))
}

/// Execute the landed direct boolmesh port from the public exact boolean path.
///
/// This is deliberately a fall-through adapter, not a compatibility fallback:
/// a completed boolmesh workspace materializes from the retained `boolean45`
/// mesh export and becomes an audited certified shortcut result, while
/// [`ExactBoolMeshValidationError::PortBlocked`] means the exact port has
/// reached an unported boolmesh stage and the historical exact materializers
/// may still handle the case.  Other boolmesh validation errors are reported
/// topology-changing decision is either certified, explicitly unknown, or a
/// validation failure.
fn boolean_boolmesh_port_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    match execute_exact_boolmesh_port_from_graph(left, right, operation, validation, graph) {
        Ok(execution) => Ok(Some(certified_shortcut_result(
            execution.mesh,
            execution.shortcut,
        ))),
        Err(ExactBoolMeshValidationError::PortBlocked(
            stage @ (ExactBoolMeshKernelStage::Triangulation | ExactBoolMeshKernelStage::Cleanup),
        )) => Err(boolmesh_late_export_blocker_error(stage)),
        Err(ExactBoolMeshValidationError::PortBlocked(_)) => Ok(None),
        Err(error) => Err(MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact boolmesh graph-backed port failed: {error:?}"),
        ))),
    }
}

fn boolean_boolmesh_split_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    match execute_exact_boolmesh_port(left, right, operation, validation) {
        Ok(execution) if execution.shortcut == ExactBooleanShortcutKind::BoolMeshSplit => Ok(Some(
            certified_shortcut_result(execution.mesh, execution.shortcut),
        )),
        Ok(_) | Err(ExactBoolMeshValidationError::PortBlocked(_)) => Ok(None),
        Err(error) => Err(MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact boolmesh split failed: {error:?}"),
        ))),
    }
}

fn boolean_boolmesh_split_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    match execute_exact_boolmesh_port_from_graph(left, right, operation, validation, graph) {
        Ok(execution) if execution.shortcut == ExactBooleanShortcutKind::BoolMeshSplit => Ok(Some(
            certified_shortcut_result(execution.mesh, execution.shortcut),
        )),
        Ok(_) | Err(ExactBoolMeshValidationError::PortBlocked(_)) => Ok(None),
        Err(error) => Err(MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact boolmesh graph-backed split failed: {error:?}"),
        ))),
    }
}

fn boolmesh_late_export_blocker_error(stage: ExactBoolMeshKernelStage) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("exact boolmesh port blocked before certified mesh export: {stage:?}"),
    ))
}

fn error_can_retry_certified_boolmesh(error: &MeshError) -> bool {
    error.diagnostics.iter().any(|diagnostic| {
        matches!(
            diagnostic.kind,
            DiagnosticKind::BoundaryEdge
                | DiagnosticKind::DegenerateTriangle
                | DiagnosticKind::DuplicateDirectedEdge
        )
    })
}

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

    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::EmptyOperand,
    ))
}

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

    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::Identical,
    ))
}

fn empty_mesh(label: &'static str, validation: ValidationPolicy) -> Result<ExactMesh, MeshError> {
    ExactMesh::new_with_policy(
        Vec::new(),
        Vec::new(),
        super::provenance::SourceProvenance::exact(label),
        validation,
    )
}

fn certified_shortcut_result(
    mesh: ExactMesh,
    shortcut: ExactBooleanShortcutKind,
) -> ExactBooleanResult {
    ExactBooleanResult {
        kind: ExactBooleanResultKind::CertifiedShortcut { shortcut },
        graph_had_unknowns: false,
        region_classifications: Vec::new(),
        triangulations: Vec::new(),
        assembly: ExactBooleanAssemblyPlan {
            vertices: Vec::new(),
            triangles: Vec::new(),
        },
        volumetric_classifications: Vec::new(),
        mesh,
    }
}

fn boundary_policy_shortcut_result(
    mesh: ExactMesh,
    operation: ExactBooleanOperation,
) -> ExactBooleanResult {
    ExactBooleanResult {
        kind: ExactBooleanResultKind::BoundaryPolicyShortcut { operation },
        graph_had_unknowns: false,
        region_classifications: Vec::new(),
        triangulations: Vec::new(),
        assembly: ExactBooleanAssemblyPlan {
            vertices: Vec::new(),
            triangles: Vec::new(),
        },
        volumetric_classifications: Vec::new(),
        mesh,
    }
}

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coplanar_volumetric_gate_uses_source_side_evidence() {
        let boundary_left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let boundary_right = axis_aligned_box_i64([2, 0, 0], [4, 2, 2]);
        let boundary_graph = build_intersection_graph(&boundary_left, &boundary_right).unwrap();
        assert!(graph_requires_coplanar_volumetric_cells(
            &graph_relation_counts(&boundary_graph)
        ));
        assert!(!graph_requires_coplanar_volumetric_cells_for_sources(
            &boundary_graph,
            &boundary_left,
            &boundary_right
        ));

        let same_side_left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let same_side_right = same_side_left.clone();
        let same_side_graph = build_intersection_graph(&same_side_left, &same_side_right).unwrap();
        assert!(graph_requires_coplanar_volumetric_cells_for_sources(
            &same_side_graph,
            &same_side_left,
            &same_side_right
        ));
    }

    fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
            ],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
    }

    fn axis_aligned_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0],
                max[1], min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1],
                max[2], min[0], max[1], max[2],
            ],
            &[
                0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2,
                7, 6, 3, 0, 4, 3, 4, 7,
            ],
        )
        .unwrap()
    }
}
