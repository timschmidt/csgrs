//! Exact boolean operation entry points.
//!
//! This module is the exact-stack Boolean boundary for the subset that is
//! currently implemented: build certified
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

use std::collections::{BTreeMap, BTreeSet};

use super::adjacent::{
    full_face_adjacent_certificate, materialize_full_face_adjacent_union_from_certificate,
};
use super::affine_box::{
    has_affine_box_difference, has_affine_box_intersection, has_affine_box_union,
};
use super::affine_solid::{
    AffineOrthogonalSolidOperation, has_affine_orthogonal_solid_cells,
    has_empty_affine_orthogonal_solid_cell_intersection,
    materialize_affine_orthogonal_solid_difference,
    materialize_affine_orthogonal_solid_intersection, materialize_affine_orthogonal_solid_union,
};
use super::arrangement2d::{
    ExactArrangement2dBlocker, ExactArrangement2dBoundaryPolicy, ExactArrangement2dOverlay,
    ExactArrangement2dRegion, ExactArrangement2dRegionRing, ExactArrangement2dSetOperation,
    build_exact_arrangement2d_overlay, build_exact_arrangement2d_overlay_with_boundary_policy,
};
use super::arrangement3d::ExactArrangement;
use super::bounds::AabbIntersectionKind;
use super::box_solid::{
    has_axis_aligned_box_cell_difference, has_axis_aligned_box_cell_union, is_axis_aligned_box,
};
use super::cell_complex::{
    arrangement_region_classification_blockers_are_volume_resolved,
    selected_region_selection_ignores_opposite_classification,
};
use super::cells::triangulate_all_face_cells_with_cdt;
use super::construction::SegmentPlaneRelation;
use super::contained_adjacent::{
    contained_face_adjacent_certificate, materialize_contained_face_adjacent_union_from_certificate,
};
use super::convex::{
    intersect_closed_convex_solids, subtract_closed_convex_solids, union_closed_convex_solids,
};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::graph::{FacePairEvents, IntersectionEvent, MeshSide, build_intersection_graph};
use super::intersection::MeshFacePairRelation;
use super::loop_triangulation::triangulate_exact_loop_group;
use super::mesh::{ExactMesh, Triangle};
use super::orthogonal_solid::{
    AxisAlignedOrthogonalSolidOperation, axis_aligned_orthogonal_solid_cell_plan,
    has_axis_aligned_orthogonal_solid_cells,
    has_empty_axis_aligned_orthogonal_solid_cell_intersection,
    has_non_empty_axis_aligned_orthogonal_solid_cell_intersection,
    materialize_axis_aligned_orthogonal_solid_cell_plan,
    orthogonal_cell_plan_is_single_rectangular_block,
};
use super::region::{
    ExactBooleanAssemblyPlan, ExactRegionRetention, ExactRegionSelection,
    FaceRegionPlaneClassification, FaceRegionTriangulation,
    checked_classify_face_regions_against_opposite_planes,
    checked_triangulate_face_regions_with_earcut, choose_region_projection,
};
use super::regularization::{
    ExactArrangementBlocker, ExactRegularizationPolicy, ExactUnresolvedPolicy,
};
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
use super::validation::ValidationPolicy;
use super::volumetric::{
    ExactVolumetricRegionClassification, ExactVolumetricRegionError, ExactVolumetricRegionRelation,
    classify_triangulated_regions_against_opposite_meshes,
};
use super::volumetric_cells::{
    CoplanarVolumetricCellEvidenceReport, CoplanarVolumetricCellObstacle,
};
use super::winding::{
    ClosedMeshWindingMeshReport, ClosedMeshWindingRelation, WindingReportError,
    classify_mesh_vertices_against_closed_mesh_winding_report,
};
use hyperlimit::{
    CoplanarProjection, Point2, Point3, SegmentIntersection, Sign, TriangleLocation,
    classify_point_triangle, compare_reals, compare_reals_report, orient3d_report, project_point3,
    projected_polygon_area2_value,
};
use hyperlimit::{PredicateUse, SourceProvenance};
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
    /// Certified shortcut that materialized output, when one did.
    pub materialized_shortcut: Option<ExactBooleanShortcutKind>,
    /// Arrangement blocker count observed after construction.
    pub arrangement_blockers: usize,
    /// Arrangement face-cell count, when construction succeeded.
    pub face_cells: usize,
    /// Connected shell/region count, when construction succeeded.
    pub regions: usize,
    /// Volume-region count, when closed shell topology produced a volume graph.
    pub volume_regions: usize,
    /// Volume adjacency count, when closed shell topology produced a volume graph.
    pub volume_adjacencies: usize,
    /// Retained lower-dimensional artifact count.
    pub lower_dimensional_artifacts: usize,
    /// Selected face-cell count, when selection succeeded.
    pub selected_faces: usize,
    /// Selected volume-region count, when selection succeeded.
    pub selected_volume_regions: usize,
    /// Output vertex count, when triangulation succeeded.
    pub output_vertices: usize,
    /// Output triangle count, when triangulation succeeded.
    pub output_triangles: usize,
}

/// Exact boolean operation request.
///
/// Named booleans are represented now, but they intentionally do not fall back
/// to approximate float winding. Certified shortcut cases execute directly, while
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
/// classification stages with the executable arrangement pipeline. For named
/// booleans that still need unresolved inside/outside semantics, it returns
/// [`ExactBooleanSupport::RequiresCertifiedWinding`] with replayable facts
/// instead of approximating them.
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
            if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
                && meshes_are_certified_identical(left, right) =>
        {
            ExactBooleanSupport::CertifiedIdentical
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
                && meshes_are_certified_same_surface(left, right) =>
        {
            ExactBooleanSupport::CertifiedSameSurface
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => {
            preflight_tail_shortcut_support(left, right, operation)
                .or_else(|| certified_mixed_dimensional_regularized_solid_support(left, right))
                .unwrap_or(ExactBooleanSupport::RequiresCertifiedWinding)
        }
    };

    if support == ExactBooleanSupport::CertifiedArrangementCellComplex {
        let graph = build_intersection_graph(left, right)?;
        validate_graph_source_handoff(&graph, left, right)?;
        return Ok(ExactBooleanPreflight {
            operation,
            support,
            graph_had_unknowns: graph.has_unknowns(),
            retained_face_pairs: graph.face_pairs.len(),
            retained_events: graph.event_count(),
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: None,
            arrangement_readiness: None,
            coplanar_volumetric_evidence: coplanar_volumetric_evidence_if_required(
                &graph, left, right,
            ),
        });
    }

    if matches!(
        support,
        ExactBooleanSupport::CertifiedEmptyOperand
            | ExactBooleanSupport::CertifiedBoundsDisjoint
            | ExactBooleanSupport::CertifiedIdentical
            | ExactBooleanSupport::CertifiedSameSurface
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
            | ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
            | ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid
            | ExactBooleanSupport::CertifiedConvexUnion
            | ExactBooleanSupport::CertifiedConvexIntersection
            | ExactBooleanSupport::CertifiedConvexDifference
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
    let mut certified_arrangement_preflight = None;
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
        && let Some(preflight) = cached_certified_arrangement_cell_complex_preflight(
            &mut certified_arrangement_preflight,
            operation,
            &graph,
            left,
            right,
        )?
    {
        return Ok(preflight);
    }
    if matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    ) && certified_arrangement_regularized_boundary_contact_from_graph(
        &graph, left, right, operation,
    )? {
        return Ok(certified_shortcut_preflight(
            operation,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
        ));
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
        && let Some(boundary_support) = certified_closed_boundary_only_contact_support_from_graph(
            &graph, left, right, operation,
        )?
    {
        return Ok(certified_shortcut_preflight(operation, boundary_support));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && operation != ExactBooleanOperation::Difference
        && let Some(convex_support) =
            certified_convex_boolean_support_from_graph(&graph, left, right, operation)?
    {
        return Ok(certified_shortcut_preflight(operation, convex_support));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && operation == ExactBooleanOperation::Difference
        && let Some(convex_support) =
            certified_convex_boolean_support_from_graph(&graph, left, right, operation)?
    {
        return Ok(certified_shortcut_preflight(operation, convex_support));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(convex_support) =
            certified_convex_boolean_support_from_graph(&graph, left, right, operation)?
    {
        return Ok(certified_shortcut_preflight(operation, convex_support));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && operation == ExactBooleanOperation::Union
        && has_non_empty_axis_aligned_orthogonal_solid_cell_intersection(left, right)
        && !graph_requires_coplanar_volumetric_cells_for_sources(&graph, left, right)
        && let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation)
        && has_axis_aligned_orthogonal_solid_cells(left, right, solid_operation)
    {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::CertifiedArrangementCellComplex,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: None,
            arrangement_readiness: None,
            coplanar_volumetric_evidence: coplanar_volumetric_evidence_if_required(
                &graph, left, right,
            ),
        });
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
            ExactBooleanSupport::CertifiedArrangementCellComplex,
        ));
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
        if let Some(preflight) = cached_certified_arrangement_cell_complex_preflight(
            &mut certified_arrangement_preflight,
            operation,
            &graph,
            left,
            right,
        )? {
            return Ok(preflight);
        }
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
    if planar_report.status == ExactPlanarArrangementStatus::AlreadyMaterialized
        && let Some(preflight) = cached_certified_arrangement_cell_complex_preflight(
            &mut certified_arrangement_preflight,
            operation,
            &graph,
            left,
            right,
        )?
    {
        return Ok(preflight);
    }
    let eager_axis_aligned_cell_support = match operation {
        ExactBooleanOperation::Union if has_axis_aligned_box_cell_union(left, right) => {
            Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
        }
        ExactBooleanOperation::Difference if has_axis_aligned_box_cell_difference(left, right) => {
            Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
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
            coplanar_volumetric_evidence: coplanar_volumetric_evidence_if_required(
                &graph, left, right,
            ),
        });
    }
    if let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation)
        && has_axis_aligned_orthogonal_solid_cells(left, right, solid_operation)
    {
        return Ok(ExactBooleanPreflight {
            operation,
            support: ExactBooleanSupport::CertifiedArrangementCellComplex,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: None,
            arrangement_readiness: None,
            coplanar_volumetric_evidence: coplanar_volumetric_evidence_if_required(
                &graph, left, right,
            ),
        });
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && !graph_requires_coplanar_volumetric_cells_for_sources(&graph, left, right)
        && let Some(convex_support) =
            certified_direct_convex_boolean_support(left, right, operation)
    {
        return Ok(certified_shortcut_preflight(operation, convex_support));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(convex_support) = certified_convex_difference_support(left, right, operation)
    {
        return Ok(certified_shortcut_preflight(operation, convex_support));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(convex_support) = certified_convex_union_support(left, right, operation)
    {
        return Ok(certified_shortcut_preflight(operation, convex_support));
    }
    if graph_requires_coplanar_volumetric_cells_for_sources(&graph, left, right) {
        if let Some(preflight) = cached_certified_arrangement_cell_complex_preflight(
            &mut certified_arrangement_preflight,
            operation,
            &graph,
            left,
            right,
        )? {
            return Ok(preflight);
        }
        if let Some(convex_support) = certified_convex_union_support(left, right, operation) {
            return Ok(certified_shortcut_preflight(operation, convex_support));
        }
        if let Some(convex_support) =
            certified_direct_convex_boolean_support(left, right, operation)
        {
            return Ok(certified_shortcut_preflight(operation, convex_support));
        }
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
    if winding_report.status == ExactWindingReadinessStatus::Ready
        && materialize_volumetric_winding_region_plan_from_graph(
            &graph,
            left,
            right,
            operation,
            ValidationPolicy::CLOSED,
        )?
        .is_some()
    {
        return Ok(certified_arrangement_cell_complex_preflight_from_graph(
            operation, &graph, left, right,
        ));
    }

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

fn preflight_tail_shortcut_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    match operation {
        ExactBooleanOperation::Union if has_affine_box_union(left, right) => {
            Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
        }
        ExactBooleanOperation::Intersection if has_affine_box_intersection(left, right) => {
            Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
        }
        ExactBooleanOperation::Difference if has_affine_box_difference(left, right) => {
            Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
        }
        ExactBooleanOperation::Union
            if has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Union,
            ) =>
        {
            Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
        }
        ExactBooleanOperation::Intersection
            if has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Intersection,
            ) =>
        {
            Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
        }
        ExactBooleanOperation::Difference
            if has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Difference,
            ) =>
        {
            Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => None,
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

fn certified_arrangement_cell_complex_preflight_from_graph(
    operation: ExactBooleanOperation,
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactBooleanPreflight {
    ExactBooleanPreflight {
        operation,
        support: ExactBooleanSupport::CertifiedArrangementCellComplex,
        graph_had_unknowns: graph.has_unknowns(),
        retained_face_pairs: graph.face_pairs.len(),
        retained_events: graph.event_count(),
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: None,
        arrangement_readiness: None,
        coplanar_volumetric_evidence: coplanar_volumetric_evidence_if_required(graph, left, right),
    }
}

fn certified_arrangement_cell_complex_preflight_if_materialized(
    operation: ExactBooleanOperation,
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ExactBooleanPreflight>, MeshError> {
    if arrangement_cell_complex_materializes_for_preflight(left, right, operation, false)?
        || arrangement_cell_complex_materializes_for_preflight(left, right, operation, true)?
        || coplanar_surface_output_materializes_for_preflight(left, right, operation)?
    {
        Ok(Some(
            certified_arrangement_cell_complex_preflight_from_graph(operation, graph, left, right),
        ))
    } else {
        Ok(None)
    }
}

fn cached_certified_arrangement_cell_complex_preflight(
    cache: &mut Option<Option<ExactBooleanPreflight>>,
    operation: ExactBooleanOperation,
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ExactBooleanPreflight>, MeshError> {
    if cache.is_none() {
        *cache = Some(
            certified_arrangement_cell_complex_preflight_if_materialized(
                operation, graph, left, right,
            )?,
        );
    }
    Ok(cache.clone().flatten())
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
    if exact_cell_complexes_certify_boundary_contact_without_shared_volume(left, right) {
        return Ok(true);
    }
    certified_closed_boundary_contact(left, right)
}

fn exact_cell_complexes_certify_boundary_contact_without_shared_volume(
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    has_empty_axis_aligned_orthogonal_solid_cell_intersection(left, right)
        || has_empty_affine_orthogonal_solid_cell_intersection(left, right)
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

fn contained_face_adjacency_should_yield_to_stronger_kernel(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> bool {
    if both_axis_aligned_boxes(left, right) {
        return true;
    }
    match operation {
        ExactBooleanOperation::Union => {
            has_axis_aligned_box_cell_union(left, right)
                || axis_aligned_orthogonal_solid_operation(operation).is_some_and(|operation| {
                    has_axis_aligned_orthogonal_solid_cells(left, right, operation)
                })
                || has_affine_box_union(left, right)
                || has_affine_orthogonal_solid_cells(
                    left,
                    right,
                    AffineOrthogonalSolidOperation::Union,
                )
        }
        ExactBooleanOperation::Intersection => {
            axis_aligned_orthogonal_solid_operation(operation).is_some_and(|operation| {
                has_axis_aligned_orthogonal_solid_cells(left, right, operation)
            }) || has_affine_box_intersection(left, right)
                || has_affine_orthogonal_solid_cells(
                    left,
                    right,
                    AffineOrthogonalSolidOperation::Intersection,
                )
        }
        ExactBooleanOperation::Difference => true,
        ExactBooleanOperation::SelectedRegions(_) => true,
    }
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
/// silently dispatching to specialized tolerance code. That is a deliberate
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
/// silently invoking the specialized tolerance path. Closed-solid regularized
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
    if let Some(result) =
        boolean_closed_regularized_lower_dimensional_optional(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return boolean_empty_operand(left, right, operation, validation);
    }
    if meshes_are_certified_bounds_disjoint(left, right) {
        return boolean_disjoint_meshes(left, right, operation, validation);
    }
    if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
        && meshes_are_certified_identical(left, right)
    {
        return boolean_identical_meshes(left, operation, validation);
    }
    if let Some(result) =
        boolean_arrangement_cell_complex_meshes(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
        && meshes_are_certified_same_surface(left, right)
    {
        return boolean_same_surface_meshes(left, operation, validation);
    }
    match operation {
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => {
            let graph = build_intersection_graph(left, right)?;
            validate_graph_source_handoff(&graph, left, right)?;
            match operation {
                ExactBooleanOperation::Union => {}
                ExactBooleanOperation::Intersection => {
                    if let Some(result) =
                        boolean_arrangement_regularized_boundary_contact_from_graph(
                            &graph, left, right, operation, validation,
                        )?
                    {
                        return Ok(result);
                    }
                }
                ExactBooleanOperation::Difference => {
                    if let Some(result) =
                        boolean_arrangement_regularized_boundary_contact_from_graph(
                            &graph, left, right, operation, validation,
                        )?
                    {
                        return Ok(result);
                    }
                }
                ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled above"),
            }
            if let Some(result) = boolean_open_surface_disjoint_meshes_from_graph(
                &graph, left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) = boolean_boundary_touching_meshes_from_graph(
                &graph,
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

enum ArrangementCellComplexOutcome {
    Materialized(ExactBooleanResult, ExactArrangementBooleanAttempt),
    Declined(ExactArrangementBooleanAttempt),
}

/// Report how far the arrangement/cell-complex Boolean pipeline gets for an
/// operation without falling through to specialized materializers.
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
            true,
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
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let outcome = match run_arrangement_cell_complex_attempt(
        left,
        right,
        operation,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
        Some(validation),
        true,
    ) {
        Ok(outcome) => outcome,
        Err(_) => return Ok(None),
    };
    match outcome {
        ArrangementCellComplexOutcome::Materialized(result, _) => Ok(Some(result)),
        ArrangementCellComplexOutcome::Declined(_) => Ok(None),
    }
}

fn arrangement_cell_complex_attempt_is_certified_for_preflight(
    attempt: &ExactArrangementBooleanAttempt,
) -> bool {
    attempt.decline.is_none()
        && (attempt.arrangement_blockers == 0
            || attempt.materialized_shortcut
                == Some(ExactBooleanShortcutKind::ArrangementCellComplex))
}

fn arrangement_cell_complex_materializes_for_preflight(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    regularize_unregularized_sheet_complex: bool,
) -> Result<bool, MeshError> {
    for validation in [ValidationPolicy::CLOSED, ValidationPolicy::ALLOW_BOUNDARY] {
        match run_arrangement_cell_complex_attempt(
            left,
            right,
            operation,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
            Some(validation),
            regularize_unregularized_sheet_complex,
        ) {
            Ok(ArrangementCellComplexOutcome::Materialized(_, attempt))
                if arrangement_cell_complex_attempt_is_certified_for_preflight(&attempt) =>
            {
                return Ok(true);
            }
            Ok(_) | Err(_) => {}
        }
    }
    Ok(false)
}

fn boolean_arrangement_regularized_boundary_contact_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if !matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    ) {
        return Ok(None);
    }
    if meshes_are_certified_identical(left, right) || meshes_are_certified_same_surface(left, right)
    {
        return Ok(None);
    }
    if let Some(report) =
        certified_closed_boundary_touching_regularized_report_from_graph(graph, left, right)?
    {
        validate_consumed_boundary_touching_report(
            &report,
            "arrangement regularized boundary contact",
        )?;
    } else if !certified_arrangement_regularized_boundary_contact_from_graph(
        graph, left, right, operation,
    )? {
        return Ok(None);
    }
    let mesh = match operation {
        ExactBooleanOperation::Intersection => empty_mesh(
            "empty exact arrangement regularized boundary-contact intersection",
            validation,
        )?,
        ExactBooleanOperation::Difference => copy_mesh(
            left,
            "exact arrangement regularized boundary-contact difference keeps left",
            validation,
        )?,
        ExactBooleanOperation::Union | ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
    };
    Ok(Some(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    )))
}

fn certified_arrangement_regularized_boundary_contact_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<bool, MeshError> {
    if !matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    ) {
        return Ok(false);
    }
    if meshes_are_certified_identical(left, right) || meshes_are_certified_same_surface(left, right)
    {
        return Ok(false);
    }
    if matches!(
        certified_convex_boolean_support_from_graph(graph, left, right, operation)?,
        Some(ExactBooleanSupport::CertifiedConvexContainment)
    ) {
        return Ok(false);
    }
    if certified_closed_boundary_touching_regularized_report_from_graph(graph, left, right)?
        .is_some()
    {
        return Ok(true);
    }
    if !certified_closed_boundary_only_contact_from_graph(graph, left, right)? {
        return Ok(false);
    }
    Ok(true)
}

fn run_arrangement_cell_complex_attempt(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    policy: ExactRegularizationPolicy,
    validation: Option<ValidationPolicy>,
    regularize_unregularized_sheet_complex: bool,
) -> Result<ArrangementCellComplexOutcome, MeshError> {
    let arrangement = ExactArrangement::from_meshes_with_policy(left, right, policy)?;
    let mut attempt = ExactArrangementBooleanAttempt {
        operation,
        policy,
        stage: ExactArrangementBooleanStage::ArrangementBuilt,
        decline: None,
        materialized_shortcut: None,
        arrangement_blockers: arrangement.blockers.len(),
        face_cells: arrangement.face_cells.len(),
        regions: arrangement
            .shells_or_regions
            .as_ref()
            .map_or(0, |regions| regions.len()),
        volume_regions: arrangement
            .volume_regions
            .as_ref()
            .map_or(0, |regions| regions.len()),
        volume_adjacencies: arrangement
            .volume_adjacencies
            .as_ref()
            .map_or(0, |adjacencies| adjacencies.len()),
        lower_dimensional_artifacts: arrangement.lower_dimensional_artifacts.len(),
        selected_faces: 0,
        selected_volume_regions: 0,
        output_vertices: 0,
        output_triangles: 0,
    };
    let regularized_sheet_recovery_surface =
        arrangement_has_regularized_closed_sheet_recovery_surface(&arrangement, left, right);
    let volume_resolves_region_classification =
        arrangement_region_classification_blockers_are_volume_resolved(&arrangement);
    let selected_regions_ignore_unresolved_classification =
        selected_region_selection_ignores_opposite_classification(operation)
            && arrangement
                .blockers
                .iter()
                .all(|blocker| *blocker == ExactArrangementBlocker::UnresolvedRegionClassification);

    let axis_aligned_box_difference_cell_result =
        has_axis_aligned_box_difference_cell_result(left, right, operation);
    if let Some(validation) = validation
        && let Some(outcome) = arrangement_orthogonal_solid_cell_recovery_outcome(
            &mut attempt,
            left,
            right,
            operation,
            validation,
            !axis_aligned_box_difference_cell_result,
        )?
    {
        return Ok(outcome);
    }

    if let Some(validation) = validation
        && let Some(result) =
            boolean_arrangement_adjacency_union_completion(left, right, operation, validation)?
    {
        attempt.stage = ExactArrangementBooleanStage::Materialized;
        attempt.materialized_shortcut = Some(ExactBooleanShortcutKind::ArrangementCellComplex);
        attempt.output_vertices = result.mesh.vertices().len();
        attempt.output_triangles = result.mesh.triangles().len();
        return Ok(ArrangementCellComplexOutcome::Materialized(result, attempt));
    }

    if let Some(validation) = validation
        && let Some(result) = boolean_arrangement_regularized_boundary_contact_from_graph(
            &arrangement.graph,
            left,
            right,
            operation,
            validation,
        )?
    {
        attempt.stage = ExactArrangementBooleanStage::Materialized;
        attempt.materialized_shortcut = Some(ExactBooleanShortcutKind::ArrangementCellComplex);
        attempt.output_vertices = result.mesh.vertices().len();
        attempt.output_triangles = result.mesh.triangles().len();
        return Ok(ArrangementCellComplexOutcome::Materialized(result, attempt));
    }

    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && let Some(validation) = validation
        && let Some(result) =
            boolean_coplanar_mesh_overlay_optional(left, right, operation, validation)?
    {
        attempt.stage = ExactArrangementBooleanStage::Materialized;
        attempt.materialized_shortcut = Some(ExactBooleanShortcutKind::ArrangementCellComplex);
        attempt.output_vertices = result.mesh.vertices().len();
        attempt.output_triangles = result.mesh.triangles().len();
        return Ok(ArrangementCellComplexOutcome::Materialized(result, attempt));
    }

    if let Some(validation) = validation
        && let Some(outcome) = arrangement_open_surface_recovery_outcome(
            &mut attempt,
            &arrangement.graph,
            left,
            right,
            operation,
            validation,
        )?
    {
        return Ok(outcome);
    }

    if !arrangement.is_complete()
        && !volume_resolves_region_classification
        && !selected_regions_ignore_unresolved_classification
    {
        if let Some(result) = materialize_simple_coplanar_overlay_arrangement(
            left,
            right,
            operation,
            validation,
            &arrangement,
        )? {
            attempt.stage = ExactArrangementBooleanStage::Materialized;
            attempt.materialized_shortcut = Some(ExactBooleanShortcutKind::ArrangementCellComplex);
            attempt.output_vertices = result.mesh.vertices().len();
            attempt.output_triangles = result.mesh.triangles().len();
            return Ok(ArrangementCellComplexOutcome::Materialized(result, attempt));
        }
        if regularize_unregularized_sheet_complex
            && arrangement_blockers_are_unregularized_sheet_complex(&arrangement.blockers)
            && let Some(validation) = validation
        {
            if let Some(result) = boolean_arrangement_regularized_sheet_complex_from_graph(
                &arrangement.graph,
                left,
                right,
                operation,
                validation,
            )? {
                attempt.stage = ExactArrangementBooleanStage::Materialized;
                attempt.materialized_shortcut =
                    Some(ExactBooleanShortcutKind::ArrangementCellComplex);
                attempt.arrangement_blockers = 0;
                attempt.output_vertices = result.mesh.vertices().len();
                attempt.output_triangles = result.mesh.triangles().len();
                return Ok(ArrangementCellComplexOutcome::Materialized(result, attempt));
            }
        }
        if let Some(outcome) = arrangement_cell_complex_recovery_outcome_if_available(
            regularize_unregularized_sheet_complex,
            regularized_sheet_recovery_surface,
            validation,
            &mut attempt,
            &arrangement.graph,
            left,
            right,
            operation,
        )? {
            return Ok(outcome);
        }
        if arrangement_blockers_are_unregularized_sheet_complex(&arrangement.blockers)
            && let Some(validation) = validation
            && let Some(outcome) = arrangement_convex_regularized_sheet_recovery_outcome(
                &mut attempt,
                left,
                right,
                operation,
                validation,
            )?
        {
            return Ok(outcome);
        }
        attempt.decline = Some(ExactArrangementBooleanDecline::ArrangementBlockers(
            arrangement.blockers.clone(),
        ));
        return Ok(ArrangementCellComplexOutcome::Declined(attempt));
    }

    let labeling_policy = if volume_resolves_region_classification
        || selected_regions_ignore_unresolved_classification
    {
        ExactRegularizationPolicy {
            unresolved: ExactUnresolvedPolicy::RetainArtifacts,
            ..policy
        }
    } else {
        policy
    };
    let labeled = match arrangement.label_regions(labeling_policy) {
        Ok(labeled) => labeled,
        Err(blocker) => {
            if let Some(outcome) = arrangement_cell_complex_recovery_outcome_if_available(
                regularize_unregularized_sheet_complex,
                regularized_sheet_recovery_surface,
                validation,
                &mut attempt,
                &arrangement.graph,
                left,
                right,
                operation,
            )? {
                return Ok(outcome);
            }
            attempt.decline = Some(ExactArrangementBooleanDecline::Labeling(blocker));
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Labeled;
    let selected_result = if volume_resolves_region_classification {
        labeled.select_volume_resolved_with_policy(operation, policy)
    } else {
        labeled.select_with_policy(operation, policy)
    };
    let selected = match selected_result {
        Ok(selected) if selected.blockers.is_empty() => selected,
        Ok(selected) => {
            attempt.selected_faces = selected.selected_faces.len();
            if let Some(outcome) = arrangement_cell_complex_recovery_outcome_if_available(
                regularize_unregularized_sheet_complex,
                regularized_sheet_recovery_surface,
                validation,
                &mut attempt,
                &arrangement.graph,
                left,
                right,
                operation,
            )? {
                return Ok(outcome);
            }
            attempt.decline = Some(ExactArrangementBooleanDecline::Selection(
                selected.blockers[0].clone(),
            ));
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
        Err(blocker) => {
            if let Some(outcome) = arrangement_cell_complex_recovery_outcome_if_available(
                regularize_unregularized_sheet_complex,
                regularized_sheet_recovery_surface,
                validation,
                &mut attempt,
                &arrangement.graph,
                left,
                right,
                operation,
            )? {
                return Ok(outcome);
            }
            attempt.decline = Some(ExactArrangementBooleanDecline::Selection(blocker));
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Selected;
    attempt.selected_faces = selected.selected_faces.len();
    attempt.selected_volume_regions = selected.selected_volume_regions.len();
    let simplified = match selected.simplify_exact_with_policy(policy) {
        Ok(simplified) if simplified.blockers.is_empty() => simplified,
        Ok(simplified) => {
            if let Some(outcome) = arrangement_cell_complex_recovery_outcome_if_available(
                regularize_unregularized_sheet_complex,
                regularized_sheet_recovery_surface,
                validation,
                &mut attempt,
                &arrangement.graph,
                left,
                right,
                operation,
            )? {
                return Ok(outcome);
            }
            attempt.decline = Some(ExactArrangementBooleanDecline::Simplification(
                simplified.blockers[0].clone(),
            ));
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
        Err(blocker) => {
            if let Some(outcome) = arrangement_cell_complex_recovery_outcome_if_available(
                regularize_unregularized_sheet_complex,
                regularized_sheet_recovery_surface,
                validation,
                &mut attempt,
                &arrangement.graph,
                left,
                right,
                operation,
            )? {
                return Ok(outcome);
            }
            attempt.decline = Some(ExactArrangementBooleanDecline::Simplification(blocker));
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Simplified;
    let mesh = match simplified.triangulate() {
        Ok(mesh) => mesh,
        Err(blocker) => {
            if let Some(outcome) = arrangement_cell_complex_recovery_outcome_if_available(
                regularize_unregularized_sheet_complex,
                regularized_sheet_recovery_surface,
                validation,
                &mut attempt,
                &arrangement.graph,
                left,
                right,
                operation,
            )? {
                return Ok(outcome);
            }
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
            if validation == ValidationPolicy::CLOSED
                && let Some(mesh) = close_exact_coplanar_boundary_loops(
                    &mesh,
                    "exact arrangement cell-complex closed coplanar-boundary result",
                    validation,
                )
            {
                attempt.stage = ExactArrangementBooleanStage::Materialized;
                attempt.materialized_shortcut =
                    Some(ExactBooleanShortcutKind::ArrangementCellComplex);
                attempt.output_vertices = mesh.vertices().len();
                attempt.output_triangles = mesh.triangles().len();
                return Ok(ArrangementCellComplexOutcome::Materialized(
                    certified_shortcut_result(
                        mesh,
                        ExactBooleanShortcutKind::ArrangementCellComplex,
                    ),
                    attempt,
                ));
            }
            if let Some(outcome) = arrangement_cell_complex_recovery_outcome_if_available(
                regularize_unregularized_sheet_complex,
                regularized_sheet_recovery_surface,
                Some(validation),
                &mut attempt,
                &arrangement.graph,
                left,
                right,
                operation,
            )? {
                return Ok(outcome);
            }
            attempt.decline = Some(ExactArrangementBooleanDecline::OutputValidation);
            return Ok(ArrangementCellComplexOutcome::Declined(attempt));
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Materialized;
    attempt.materialized_shortcut = Some(ExactBooleanShortcutKind::ArrangementCellComplex);
    if volume_resolves_region_classification {
        attempt.arrangement_blockers = 0;
    }
    Ok(ArrangementCellComplexOutcome::Materialized(
        certified_shortcut_result(mesh, ExactBooleanShortcutKind::ArrangementCellComplex),
        attempt,
    ))
}

fn boolean_arrangement_orthogonal_solid_cell_recovery(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    require_single_rectangular_block: bool,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation) else {
        return Ok(None);
    };
    let Some(plan) = axis_aligned_orthogonal_solid_cell_plan(left, right, solid_operation) else {
        return Ok(None);
    };
    if require_single_rectangular_block && !orthogonal_cell_plan_is_single_rectangular_block(&plan)
    {
        return Ok(None);
    }
    let label = match solid_operation {
        AxisAlignedOrthogonalSolidOperation::Union => {
            "exact arrangement orthogonal solid cell union recovery"
        }
        AxisAlignedOrthogonalSolidOperation::Intersection => {
            "exact arrangement orthogonal solid cell intersection recovery"
        }
        AxisAlignedOrthogonalSolidOperation::Difference => {
            "exact arrangement orthogonal solid cell difference recovery"
        }
    };
    let mesh = materialize_axis_aligned_orthogonal_solid_cell_plan(plan, label, validation)?;
    let result = certified_shortcut_result(mesh, ExactBooleanShortcutKind::ArrangementCellComplex);
    if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

fn arrangement_orthogonal_solid_cell_recovery_outcome(
    attempt: &mut ExactArrangementBooleanAttempt,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    require_single_rectangular_block: bool,
) -> Result<Option<ArrangementCellComplexOutcome>, MeshError> {
    let Some(result) = boolean_arrangement_orthogonal_solid_cell_recovery(
        left,
        right,
        operation,
        validation,
        require_single_rectangular_block,
    )?
    else {
        return Ok(None);
    };
    attempt.stage = ExactArrangementBooleanStage::Materialized;
    attempt.decline = None;
    attempt.materialized_shortcut = Some(ExactBooleanShortcutKind::ArrangementCellComplex);
    attempt.arrangement_blockers = 0;
    attempt.output_vertices = result.mesh.vertices().len();
    attempt.output_triangles = result.mesh.triangles().len();
    Ok(Some(ArrangementCellComplexOutcome::Materialized(
        result,
        attempt.clone(),
    )))
}

fn arrangement_open_surface_recovery_outcome(
    attempt: &mut ExactArrangementBooleanAttempt,
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ArrangementCellComplexOutcome>, MeshError> {
    if !mesh_is_open_surface(left) || !mesh_is_open_surface(right) {
        return Ok(None);
    }
    let Some(plan) = open_surface_arrangement_plan_from_graph(graph, left, right, operation)?
    else {
        return Ok(None);
    };
    let result = materialize_open_surface_arrangement_plan(
        left,
        right,
        operation,
        validation,
        graph.has_unknowns(),
        plan,
    )?;
    attempt.stage = ExactArrangementBooleanStage::Materialized;
    attempt.decline = None;
    attempt.output_vertices = result.mesh.vertices().len();
    attempt.output_triangles = result.mesh.triangles().len();
    Ok(Some(ArrangementCellComplexOutcome::Materialized(
        result,
        attempt.clone(),
    )))
}

fn arrangement_affine_orthogonal_solid_recovery_outcome(
    attempt: &mut ExactArrangementBooleanAttempt,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ArrangementCellComplexOutcome>, MeshError> {
    let Some(result) =
        boolean_arrangement_affine_orthogonal_solid_recovery(left, right, operation, validation)?
    else {
        return Ok(None);
    };
    attempt.stage = ExactArrangementBooleanStage::Materialized;
    attempt.decline = None;
    attempt.materialized_shortcut = Some(ExactBooleanShortcutKind::ArrangementCellComplex);
    attempt.arrangement_blockers = 0;
    attempt.output_vertices = result.mesh.vertices().len();
    attempt.output_triangles = result.mesh.triangles().len();
    Ok(Some(ArrangementCellComplexOutcome::Materialized(
        result,
        attempt.clone(),
    )))
}

fn boolean_arrangement_adjacency_union_completion(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if operation != ExactBooleanOperation::Union
        || both_axis_aligned_boxes(left, right)
        || !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
    {
        return Ok(None);
    }

    if let Some(certificate) = full_face_adjacent_certificate(left, right)
        && let Some(union) = materialize_full_face_adjacent_union_from_certificate(
            left,
            right,
            &certificate,
            validation,
        )
    {
        return Ok(Some(certified_shortcut_result(
            union.mesh,
            ExactBooleanShortcutKind::ArrangementCellComplex,
        )));
    }

    if contained_face_adjacency_should_yield_to_stronger_kernel(left, right, operation) {
        return Ok(None);
    }
    if let Some(certificate) = contained_face_adjacent_certificate(left, right)
        && let Some(union) = materialize_contained_face_adjacent_union_from_certificate(
            left,
            right,
            &certificate,
            validation,
        )
    {
        return Ok(Some(certified_shortcut_result(
            union.mesh,
            ExactBooleanShortcutKind::ArrangementCellComplex,
        )));
    }

    Ok(None)
}

fn arrangement_blockers_are_unregularized_sheet_complex(
    blockers: &[ExactArrangementBlocker],
) -> bool {
    blockers.contains(&ExactArrangementBlocker::UnregularizedOpenSheetComplex)
        && blockers.iter().all(|blocker| {
            matches!(
                blocker,
                ExactArrangementBlocker::UnregularizedCoincidentSheetComplex
                    | ExactArrangementBlocker::UnregularizedOpenSheetComplex
            )
        })
}

fn arrangement_has_mixed_source_sheet_complex(arrangement: &ExactArrangement) -> bool {
    arrangement
        .shells_or_regions
        .as_ref()
        .is_some_and(|regions| {
            regions
                .iter()
                .any(|region| region.non_manifold_edges > 0 && region.source_sides.len() > 1)
        })
}

fn arrangement_has_regularized_closed_sheet_recovery_surface(
    arrangement: &ExactArrangement,
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    left.facts().mesh.closed_manifold
        && right.facts().mesh.closed_manifold
        && arrangement_has_mixed_source_sheet_complex(arrangement)
}

fn boolean_arrangement_regularized_sheet_complex_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    // Unregularized sheet arrangements already retain exact split cells but can
    // lack a closed shell graph. The volumetric split-cell assembly supplies
    // the missing regularized caps without changing predicates or tolerances.
    if let Some(result) = boolean_arrangement_volumetric_split_cell_recovery_from_graph(
        graph, left, right, operation, validation,
    )? {
        return Ok(Some(result));
    }
    boolean_arrangement_regularized_no_volume_overlap_from_graph(
        graph, left, right, operation, validation,
    )
}

fn boolean_arrangement_regularized_sheet_or_boundary_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if let Some(result) = boolean_arrangement_regularized_sheet_complex_from_graph(
        graph, left, right, operation, validation,
    )? {
        return Ok(Some(result));
    }
    Ok(None)
}

fn boolean_arrangement_regularized_no_volume_overlap_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
    {
        return Ok(None);
    }

    let Some(left_minus_right) = boolean_arrangement_volumetric_split_cell_recovery_from_graph(
        graph,
        left,
        right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )?
    else {
        return Ok(None);
    };
    if !arrangement_difference_preserves_source_surface(&left_minus_right, left, MeshSide::Left) {
        return Ok(None);
    }

    let reverse_graph = build_intersection_graph(right, left)?;
    validate_graph_source_handoff(&reverse_graph, right, left)?;
    let Some(right_minus_left) = boolean_arrangement_volumetric_split_cell_recovery_from_graph(
        &reverse_graph,
        right,
        left,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )?
    else {
        return Ok(None);
    };
    if !arrangement_difference_preserves_source_surface(&right_minus_left, right, MeshSide::Left) {
        return Ok(None);
    }

    let (mesh, shortcut) = match operation {
        ExactBooleanOperation::Union => (
            concatenate_meshes_with_options(
                left,
                right,
                false,
                "exact arrangement no-volume-overlap regularized union preserving separate shells",
                validation,
            )?,
            ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion,
        ),
        ExactBooleanOperation::Intersection => (
            empty_mesh(
                "empty exact arrangement no-volume-overlap regularized intersection",
                validation,
            )?,
            ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection,
        ),
        ExactBooleanOperation::Difference => (
            copy_mesh(
                left,
                "exact arrangement no-volume-overlap difference preserving left shell",
                validation,
            )?,
            ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
        ),
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled above"),
    };
    let result = certified_shortcut_result(mesh, shortcut);
    if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

fn arrangement_difference_preserves_source_surface(
    result: &ExactBooleanResult,
    source: &ExactMesh,
    source_side: MeshSide,
) -> bool {
    if !matches!(
        result.kind,
        ExactBooleanResultKind::ArrangementCellComplexMaterialized {
            operation: ExactBooleanOperation::Difference
        }
    ) {
        return false;
    }
    if result.validate().is_err() {
        return false;
    }
    let mut retained_area_by_face = vec![Real::from(0); source.triangles().len()];
    for triangle in &result.assembly.triangles {
        if triangle.source_side != source_side || triangle.source_face >= source.triangles().len() {
            return false;
        }
        let Ok(projection) = choose_region_projection(source, triangle.source_face) else {
            return false;
        };
        let Some(points) = triangle
            .vertices
            .iter()
            .map(|vertex| {
                result
                    .assembly
                    .vertices
                    .get(*vertex)
                    .map(|vertex| vertex.point.clone())
            })
            .collect::<Option<Vec<_>>>()
        else {
            return false;
        };
        let Some(area) = real_abs(&projected_polygon_area2_value(&points, projection)) else {
            return false;
        };
        if compare_reals(&area, &Real::from(0)).value() != Some(Ordering::Greater) {
            return false;
        }
        retained_area_by_face[triangle.source_face] =
            retained_area_by_face[triangle.source_face].clone() + area;
    }

    source.triangles().iter().enumerate().all(|(face, _)| {
        let Some(points) = triangle_points(source, face) else {
            return false;
        };
        let Ok(projection) = choose_region_projection(source, face) else {
            return false;
        };
        let Some(source_area) = real_abs(&projected_polygon_area2_value(&points, projection))
        else {
            return false;
        };
        compare_reals(&retained_area_by_face[face], &source_area).value() == Some(Ordering::Equal)
    })
}

fn real_abs(value: &Real) -> Option<Real> {
    match real_sign(value)? {
        Sign::Negative => Some(Real::from(0) - value),
        Sign::Zero | Sign::Positive => Some(value.clone()),
    }
}

fn arrangement_volumetric_split_cell_recovery_outcome(
    attempt: &mut ExactArrangementBooleanAttempt,
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ArrangementCellComplexOutcome>, MeshError> {
    let Some(result) = boolean_arrangement_volumetric_split_cell_recovery_from_graph(
        graph, left, right, operation, validation,
    )?
    else {
        return Ok(None);
    };
    attempt.stage = ExactArrangementBooleanStage::Materialized;
    attempt.decline = None;
    attempt.materialized_shortcut = Some(ExactBooleanShortcutKind::ArrangementCellComplex);
    attempt.arrangement_blockers = 0;
    attempt.output_vertices = result.mesh.vertices().len();
    attempt.output_triangles = result.mesh.triangles().len();
    Ok(Some(ArrangementCellComplexOutcome::Materialized(
        result,
        attempt.clone(),
    )))
}

fn arrangement_convex_regularized_sheet_recovery_outcome(
    attempt: &mut ExactArrangementBooleanAttempt,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ArrangementCellComplexOutcome>, MeshError> {
    let Some(result) =
        boolean_arrangement_convex_regularized_sheet_recovery(left, right, operation, validation)?
    else {
        return Ok(None);
    };
    attempt.stage = ExactArrangementBooleanStage::Materialized;
    attempt.decline = None;
    attempt.materialized_shortcut = Some(ExactBooleanShortcutKind::ArrangementCellComplex);
    attempt.arrangement_blockers = 0;
    attempt.output_vertices = result.mesh.vertices().len();
    attempt.output_triangles = result.mesh.triangles().len();
    Ok(Some(ArrangementCellComplexOutcome::Materialized(
        result,
        attempt.clone(),
    )))
}

fn arrangement_cell_complex_recovery_outcome_if_available(
    enabled: bool,
    regularized_sheet_recovery_surface: bool,
    validation: Option<ValidationPolicy>,
    attempt: &mut ExactArrangementBooleanAttempt,
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ArrangementCellComplexOutcome>, MeshError> {
    if enabled
        && regularized_sheet_recovery_surface
        && let Some(validation) = validation
    {
        if let Some(result) = boolean_arrangement_regularized_sheet_or_boundary_from_graph(
            graph, left, right, operation, validation,
        )? {
            attempt.stage = ExactArrangementBooleanStage::Materialized;
            attempt.decline = None;
            attempt.materialized_shortcut = Some(ExactBooleanShortcutKind::ArrangementCellComplex);
            attempt.arrangement_blockers = 0;
            attempt.output_vertices = result.mesh.vertices().len();
            attempt.output_triangles = result.mesh.triangles().len();
            return Ok(Some(ArrangementCellComplexOutcome::Materialized(
                result,
                attempt.clone(),
            )));
        }
        if let Some(result) = boolean_arrangement_convex_regularized_sheet_recovery(
            left, right, operation, validation,
        )? {
            attempt.stage = ExactArrangementBooleanStage::Materialized;
            attempt.decline = None;
            attempt.materialized_shortcut = Some(ExactBooleanShortcutKind::ArrangementCellComplex);
            attempt.arrangement_blockers = 0;
            attempt.output_vertices = result.mesh.vertices().len();
            attempt.output_triangles = result.mesh.triangles().len();
            return Ok(Some(ArrangementCellComplexOutcome::Materialized(
                result,
                attempt.clone(),
            )));
        }
    }
    if let Some(validation) = validation.filter(|_| enabled)
        && let Some(outcome) = arrangement_volumetric_split_cell_recovery_outcome(
            attempt, graph, left, right, operation, validation,
        )?
    {
        return Ok(Some(outcome));
    }
    let Some(validation) = validation else {
        return Ok(None);
    };
    if let Some(outcome) = arrangement_orthogonal_solid_cell_recovery_outcome(
        attempt, left, right, operation, validation, false,
    )? {
        return Ok(Some(outcome));
    }
    arrangement_affine_orthogonal_solid_recovery_outcome(
        attempt, left, right, operation, validation,
    )
}

fn boolean_arrangement_convex_regularized_sheet_recovery(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let (mesh, label) = match operation {
        ExactBooleanOperation::Union => {
            let Some(union) = union_closed_convex_solids(left, right) else {
                return Ok(None);
            };
            (
                union.mesh,
                "exact arrangement regularized convex sheet union",
            )
        }
        ExactBooleanOperation::Intersection => {
            let Some(intersection) = intersect_closed_convex_solids(left, right) else {
                return Ok(None);
            };
            (
                intersection.mesh,
                "exact arrangement regularized convex sheet intersection",
            )
        }
        ExactBooleanOperation::Difference => {
            let Some(difference) = subtract_closed_convex_solids(left, right) else {
                return Ok(None);
            };
            (
                difference.mesh,
                "exact arrangement regularized convex sheet difference",
            )
        }
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    let mesh = copy_mesh(&mesh, label, validation)?;
    let result = certified_shortcut_result(mesh, ExactBooleanShortcutKind::ArrangementCellComplex);
    if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

fn boolean_arrangement_volumetric_split_cell_recovery_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        graph, left, right, operation, validation,
    )?
    else {
        return Ok(None);
    };
    let result = ExactBooleanResult {
        kind: ExactBooleanResultKind::ArrangementCellComplexMaterialized { operation },
        graph_had_unknowns: false,
        region_classifications: materialized.region_classifications,
        triangulations: materialized.triangulations,
        assembly: materialized.assembly,
        volumetric_classifications: materialized.volumetric_classifications,
        mesh: materialized.mesh,
    };
    if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

fn close_exact_coplanar_boundary_loops(
    mesh: &ExactMesh,
    label: &'static str,
    validation: ValidationPolicy,
) -> Option<ExactMesh> {
    close_exact_coplanar_boundary_loops_from_loops(
        mesh,
        directed_boundary_loops(mesh)?,
        label,
        validation,
    )
}

fn close_exact_coplanar_boundary_loops_from_loops(
    mesh: &ExactMesh,
    boundary_loops: Vec<Vec<usize>>,
    label: &'static str,
    validation: ValidationPolicy,
) -> Option<ExactMesh> {
    if mesh.facts().mesh.closed_manifold || mesh.facts().mesh.boundary_edges == 0 {
        return None;
    }
    if boundary_loops.is_empty() {
        return None;
    }

    let cap_groups = coplanar_boundary_loop_groups(mesh, boundary_loops)?;
    let vertices = mesh.vertices().to_vec();
    let mut cap_triangles = Vec::new();
    for group in cap_groups {
        let loops = group
            .loops
            .iter()
            .map(|boundary_loop| {
                boundary_loop
                    .iter()
                    .map(|&vertex| mesh.vertices().get(vertex).cloned())
                    .collect::<Option<Vec<_>>>()
            })
            .collect::<Option<Vec<_>>>()?;
        let mut group_vertices = Vec::new();
        let mut group_triangles = Vec::new();
        triangulate_exact_loop_group(&loops, &mut group_vertices, &mut group_triangles).ok()?;
        let local_to_global = group_vertices
            .iter()
            .map(|point| find_exact_mesh_vertex(&vertices, point))
            .collect::<Option<Vec<_>>>()?;
        cap_triangles.extend(group_triangles.into_iter().map(|triangle| {
            Triangle([
                local_to_global[triangle.0[0]],
                local_to_global[triangle.0[1]],
                local_to_global[triangle.0[2]],
            ])
        }));
    }

    let mut triangles = mesh.triangles().to_vec();
    triangles.extend(cap_triangles.iter().copied());
    match ExactMesh::new_with_policy(
        vertices.clone(),
        triangles,
        SourceProvenance::exact(label),
        validation,
    ) {
        Ok(mesh) => Some(mesh),
        Err(_) => {
            let mut triangles = mesh.triangles().to_vec();
            triangles.extend(cap_triangles.into_iter().map(|triangle| {
                let [a, b, c] = triangle.0;
                Triangle([a, c, b])
            }));
            ExactMesh::new_with_policy(
                vertices,
                triangles,
                SourceProvenance::exact(label),
                validation,
            )
            .ok()
        }
    }
}

struct CoplanarBoundaryLoopGroup {
    carrier: [Point3; 3],
    loops: Vec<Vec<usize>>,
}

fn coplanar_boundary_loop_groups(
    mesh: &ExactMesh,
    boundary_loops: Vec<Vec<usize>>,
) -> Option<Vec<CoplanarBoundaryLoopGroup>> {
    let mut groups = Vec::<CoplanarBoundaryLoopGroup>::new();
    for boundary_loop in boundary_loops {
        if boundary_loop.len() < 3 {
            return None;
        }
        let (a, b, c) = exact_non_collinear_loop_carrier(mesh, &boundary_loop)?;
        let carrier = [a.clone(), b.clone(), c.clone()];
        let mut group_index = None;
        for (index, group) in groups.iter().enumerate() {
            if loop_is_exactly_coplanar(mesh, &boundary_loop, group.carrier_refs()) {
                group_index = Some(index);
                break;
            }
        }
        match group_index {
            Some(index) => groups[index].loops.push(boundary_loop),
            None => {
                if !loop_is_exactly_coplanar(
                    mesh,
                    &boundary_loop,
                    (&carrier[0], &carrier[1], &carrier[2]),
                ) {
                    return None;
                }
                groups.push(CoplanarBoundaryLoopGroup {
                    carrier,
                    loops: vec![boundary_loop],
                });
            }
        }
    }
    (!groups.is_empty()).then_some(groups)
}

impl CoplanarBoundaryLoopGroup {
    fn carrier_refs(&self) -> (&Point3, &Point3, &Point3) {
        (&self.carrier[0], &self.carrier[1], &self.carrier[2])
    }
}

fn find_exact_mesh_vertex(vertices: &[Point3], point: &Point3) -> Option<usize> {
    for (index, existing) in vertices.iter().enumerate() {
        if point3_exact_equal(existing, point)? {
            return Some(index);
        }
    }
    None
}

fn point3_exact_equal(left: &Point3, right: &Point3) -> Option<bool> {
    Some(
        compare_reals(&left.x, &right.x).value()? == Ordering::Equal
            && compare_reals(&left.y, &right.y).value()? == Ordering::Equal
            && compare_reals(&left.z, &right.z).value()? == Ordering::Equal,
    )
}

fn directed_boundary_loops(mesh: &ExactMesh) -> Option<Vec<Vec<usize>>> {
    let mut edge_uses: BTreeMap<[usize; 2], Vec<(usize, usize)>> = BTreeMap::new();
    for triangle in mesh.triangles() {
        let [a, b, c] = triangle.0;
        for (start, end) in [(a, b), (b, c), (c, a)] {
            let key = if start < end {
                [start, end]
            } else {
                [end, start]
            };
            edge_uses.entry(key).or_default().push((start, end));
        }
    }

    let mut next_by_start = BTreeMap::new();
    let mut incoming = BTreeMap::<usize, usize>::new();
    let mut boundary_edge_count = 0_usize;
    for uses in edge_uses.values() {
        if uses.len() == 1 {
            let (start, end) = uses[0];
            if next_by_start.insert(start, end).is_some() {
                return None;
            }
            *incoming.entry(end).or_default() += 1;
            boundary_edge_count += 1;
        } else if uses.len() > 2 {
            return None;
        }
    }
    if boundary_edge_count < 3 {
        return None;
    }
    for start in next_by_start.keys() {
        if incoming.get(start).copied().unwrap_or(0) != 1 {
            return None;
        }
    }

    let mut loops = Vec::new();
    let mut used_starts = BTreeSet::new();
    while let Some(start) = next_by_start
        .keys()
        .copied()
        .find(|start| !used_starts.contains(start))
    {
        let mut loop_vertices = Vec::new();
        let mut current = start;
        for _ in 0..boundary_edge_count {
            if !used_starts.insert(current) {
                return None;
            }
            loop_vertices.push(current);
            current = *next_by_start.get(&current)?;
            if current == start {
                break;
            }
        }
        if current != start || loop_vertices.len() < 3 {
            return None;
        }
        loops.push(loop_vertices);
    }
    if used_starts.len() != boundary_edge_count || loops.is_empty() {
        return None;
    }
    Some(loops)
}

fn exact_non_collinear_loop_carrier<'a>(
    mesh: &'a ExactMesh,
    loop_vertices: &[usize],
) -> Option<(&'a Point3, &'a Point3, &'a Point3)> {
    let anchor = mesh.vertices().get(*loop_vertices.first()?)?;
    for first_index in 1..loop_vertices.len() - 1 {
        for second_index in first_index + 1..loop_vertices.len() {
            let first = mesh.vertices().get(loop_vertices[first_index])?;
            let second = mesh.vertices().get(loop_vertices[second_index])?;
            if !exact_points_are_collinear(anchor, first, second)? {
                return Some((anchor, first, second));
            }
        }
    }
    None
}

fn loop_is_exactly_coplanar(
    mesh: &ExactMesh,
    loop_vertices: &[usize],
    carrier: (&Point3, &Point3, &Point3),
) -> bool {
    let (a, b, c) = carrier;
    loop_vertices.iter().all(|vertex| {
        mesh.vertices()
            .get(*vertex)
            .and_then(|point| orient3d_report(a, b, c, point).value())
            == Some(Sign::Zero)
    })
}

fn exact_points_are_collinear(a: &Point3, b: &Point3, c: &Point3) -> Option<bool> {
    let abx = b.x.clone() - &a.x;
    let aby = b.y.clone() - &a.y;
    let abz = b.z.clone() - &a.z;
    let acx = c.x.clone() - &a.x;
    let acy = c.y.clone() - &a.y;
    let acz = c.z.clone() - &a.z;
    let cross_x = aby.clone() * &acz - &(abz.clone() * &acy);
    let cross_y = abz * &acx - &(abx.clone() * &acz);
    let cross_z = abx * &acy - &(aby * &acx);
    Some(
        compare_reals(&cross_x, &Real::from(0)).value()? == Ordering::Equal
            && compare_reals(&cross_y, &Real::from(0)).value()? == Ordering::Equal
            && compare_reals(&cross_z, &Real::from(0)).value()? == Ordering::Equal,
    )
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
    {
        return Ok(None);
    }
    let Some(validation) = validation else {
        return Ok(None);
    };
    let overlay = &arrangement.carrier_plane_overlays[0];
    let allow_empty = matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    );
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
        && !overlay_allows_selected_face_materialization(&requested_overlay)
    {
        return Ok(None);
    }
    let has_selected_area = requested_overlay.faces.iter().any(|face| face.selected);
    if !has_selected_area {
        if allow_empty {
            let mesh = empty_mesh("empty exact coplanar overlay arrangement", validation)?;
            return Ok(Some(certified_shortcut_result(
                mesh,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            )));
        }
        return Ok(None);
    }

    let carrier = left.triangles()[overlay.left_face].0;
    let carrier_points = [
        left.vertices()[carrier[0]].clone(),
        left.vertices()[carrier[1]].clone(),
        left.vertices()[carrier[2]].clone(),
    ];
    let Some(mesh) = mesh_from_selected_projected_overlay_faces(
        &requested_overlay,
        &carrier_points,
        overlay.projection,
        "exact coplanar selected-face overlay arrangement",
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

fn boolean_coplanar_mesh_overlay_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if !coplanar_mesh_overlay_should_preempt_surface_paths(left, right, operation) {
        return Ok(None);
    }
    if coplanar_mesh_overlay_should_yield_to_closed_boundary_shortcut(left, right, operation)? {
        return Ok(None);
    }
    let allow_empty_overlay = matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    );
    let boundary_policy = match operation {
        ExactBooleanOperation::Difference => {
            coplanar_mesh_overlay_materialized_difference_boundary_policy(left, right)
                .unwrap_or(ExactArrangement2dBoundaryPolicy::SimplifyCollinear)
        }
        ExactBooleanOperation::Intersection => {
            coplanar_mesh_overlay_surface_intersection_boundary_policy(left, right)
                .unwrap_or(ExactArrangement2dBoundaryPolicy::SimplifyCollinear)
        }
        ExactBooleanOperation::Union | ExactBooleanOperation::SelectedRegions(_) => {
            ExactArrangement2dBoundaryPolicy::SimplifyCollinear
        }
    };
    let operation = match operation {
        ExactBooleanOperation::Union => ExactArrangement2dSetOperation::Union,
        ExactBooleanOperation::Intersection => ExactArrangement2dSetOperation::Intersection,
        ExactBooleanOperation::Difference => ExactArrangement2dSetOperation::Difference,
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    let Some(mesh) = materialize_coplanar_mesh_overlay_mesh(
        left,
        right,
        operation,
        boundary_policy,
        "exact coplanar mesh overlay arrangement",
        allow_empty_overlay,
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

fn coplanar_mesh_overlay_should_yield_to_closed_boundary_shortcut(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<bool, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
    {
        return Ok(false);
    }

    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    Ok(
        certified_closed_boundary_only_contact_from_graph(&graph, left, right)?
            || certified_closed_boundary_touching_support_from_graph(
                &graph, left, right, operation,
            )?
            .is_some(),
    )
}

pub(crate) fn materialize_coplanar_mesh_overlay_mesh(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactArrangement2dSetOperation,
    boundary_policy: ExactArrangement2dBoundaryPolicy,
    provenance: &'static str,
    allow_empty: bool,
) -> Option<ExactMesh> {
    let (carrier_points, projection) = coplanar_mesh_overlay_carrier(left, right)?;
    let mut rings = Vec::with_capacity(left.triangles().len() + right.triangles().len());
    rings.extend(projected_mesh_boundary_rings(
        ExactArrangement2dRegion::Left,
        left,
        projection,
    )?);
    rings.extend(projected_mesh_boundary_rings(
        ExactArrangement2dRegion::Right,
        right,
        projection,
    )?);
    let overlay =
        build_exact_arrangement2d_overlay_with_boundary_policy(&rings, operation, boundary_policy);
    if !overlay.is_complete() && !overlay_allows_selected_face_materialization(&overlay) {
        return None;
    }
    if !overlay.faces.iter().any(|face| face.selected) {
        return allow_empty
            .then(|| {
                ExactMesh::new_with_policy(
                    Vec::new(),
                    Vec::new(),
                    SourceProvenance::exact(provenance),
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .ok()
            })
            .flatten();
    }
    mesh_from_selected_projected_overlay_faces(&overlay, &carrier_points, projection, provenance)
}

fn overlay_allows_selected_face_materialization(overlay: &ExactArrangement2dOverlay) -> bool {
    overlay.faces.iter().any(|face| face.selected)
        && overlay.blockers.iter().all(|blocker| {
            matches!(
                blocker,
                ExactArrangement2dBlocker::OutputHoleWithoutOuter { .. }
                    | ExactArrangement2dBlocker::UnresolvedOutputLoopContainment { .. }
                    | ExactArrangement2dBlocker::OutputLoopBoundaryContainment { .. }
            )
        })
}

fn mesh_from_selected_projected_overlay_faces(
    overlay: &ExactArrangement2dOverlay,
    carrier_points: &[Point3; 3],
    projection: CoplanarProjection,
    provenance: &'static str,
) -> Option<ExactMesh> {
    match mesh_from_projected_overlay_output_components(
        overlay,
        carrier_points,
        projection,
        provenance,
    ) {
        Some(mesh) => Some(mesh),
        None if !overlay.output_components.is_empty() => None,
        None if overlay_allows_selected_face_materialization(overlay) => {
            mesh_from_projected_overlay_selected_faces(
                overlay,
                carrier_points,
                projection,
                provenance,
            )
        }
        None => None,
    }
}

fn mesh_from_projected_overlay_output_components(
    overlay: &ExactArrangement2dOverlay,
    carrier_points: &[Point3; 3],
    projection: CoplanarProjection,
    provenance: &'static str,
) -> Option<ExactMesh> {
    if overlay.output_components.is_empty() {
        return None;
    }

    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for component in &overlay.output_components {
        let mut loop_indices = Vec::with_capacity(component.hole_loops.len() + 1);
        loop_indices.push(component.outer_loop);
        loop_indices.extend(component.hole_loops.iter().copied());
        let lifted_loops = loop_indices
            .into_iter()
            .map(|loop_index| {
                let loop_ = overlay.output_loops.get(loop_index)?;
                if loop_.points.len() < 3 {
                    return None;
                }
                loop_
                    .points
                    .iter()
                    .map(|point| lift_projected_point_to_carrier(point, carrier_points, projection))
                    .collect::<Option<Vec<_>>>()
            })
            .collect::<Option<Vec<_>>>()?;

        let mut component_vertices = Vec::new();
        let mut component_triangles = Vec::new();
        triangulate_exact_loop_group(
            &lifted_loops,
            &mut component_vertices,
            &mut component_triangles,
        )
        .ok()?;
        let component_offset = vertices.len();
        triangles.extend(component_triangles.into_iter().map(|triangle| {
            Triangle([
                component_offset + triangle.0[0],
                component_offset + triangle.0[1],
                component_offset + triangle.0[2],
            ])
        }));
        vertices.extend(component_vertices);
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

fn mesh_from_projected_overlay_selected_faces(
    overlay: &ExactArrangement2dOverlay,
    carrier_points: &[Point3; 3],
    projection: CoplanarProjection,
    provenance: &'static str,
) -> Option<ExactMesh> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for overlay_face in overlay.faces.iter().filter(|face| face.selected) {
        let face = overlay.arrangement.faces.get(overlay_face.face)?;
        let boundary = face
            .vertices
            .iter()
            .map(|vertex| {
                let point = &overlay.arrangement.vertices.get(*vertex)?.point;
                lift_projected_point_to_carrier(point, carrier_points, projection)
            })
            .collect::<Option<Vec<_>>>()?;
        triangulate_exact_loop_group(&[boundary], &mut vertices, &mut triangles).ok()?;
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
    match operation {
        ExactBooleanOperation::Union => {
            coplanar_mesh_overlay_surface_union_boundary_policy(left, right).is_some()
        }
        ExactBooleanOperation::Intersection => {
            coplanar_mesh_overlay_surface_intersection_boundary_policy(left, right).is_some()
        }
        ExactBooleanOperation::Difference => {
            coplanar_mesh_overlay_difference_materializes(left, right)
        }
        ExactBooleanOperation::SelectedRegions(_) => false,
    }
}

pub(crate) fn coplanar_mesh_overlay_carrier(
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

fn order_mesh_boundary_loops(mesh: &ExactMesh) -> Option<Vec<Vec<usize>>> {
    let mut edge_counts: Vec<((usize, usize), usize)> = Vec::new();
    for triangle in mesh.triangles() {
        for (a, b) in mesh_triangle_edges(triangle.0) {
            let edge = canonical_mesh_edge(a, b);
            if let Some((_, count)) = edge_counts
                .iter_mut()
                .find(|(candidate, _)| *candidate == edge)
            {
                *count += 1;
            } else {
                edge_counts.push((edge, 1));
            }
        }
    }
    if edge_counts
        .iter()
        .any(|(_, count)| *count == 0 || *count > 2)
    {
        return None;
    }
    let boundary_edges = edge_counts
        .into_iter()
        .filter_map(|(edge, count)| (count == 1).then_some(edge))
        .collect::<Vec<_>>();
    if boundary_edges.len() < 3 {
        return None;
    }

    let mut boundary_vertices = Vec::new();
    for &(a, b) in &boundary_edges {
        if !boundary_vertices.contains(&a) {
            boundary_vertices.push(a);
        }
        if !boundary_vertices.contains(&b) {
            boundary_vertices.push(b);
        }
    }
    for &vertex in &boundary_vertices {
        let degree = boundary_edges
            .iter()
            .filter(|(a, b)| *a == vertex || *b == vertex)
            .count();
        if degree != 2 {
            return None;
        }
    }

    let mut used = vec![false; boundary_edges.len()];
    let mut loops = Vec::new();
    while let Some(seed) = used.iter().position(|used| !*used) {
        let (a, b) = boundary_edges[seed];
        let start = a.min(b);
        let mut previous = None;
        let mut current = start;
        let mut loop_vertices = Vec::new();
        loop {
            loop_vertices.push(current);
            let mut candidates = boundary_edges
                .iter()
                .enumerate()
                .filter_map(|(index, (edge_a, edge_b))| {
                    if used[index] {
                        return None;
                    }
                    if *edge_a == current {
                        Some((index, *edge_b))
                    } else if *edge_b == current {
                        Some((index, *edge_a))
                    } else {
                        None
                    }
                })
                .collect::<Vec<_>>();
            candidates.sort_by_key(|(_, next)| *next);
            let (edge_index, next) = match previous {
                Some(previous) => candidates
                    .into_iter()
                    .find(|(_, candidate)| *candidate != previous)?,
                None => candidates.into_iter().next()?,
            };
            used[edge_index] = true;
            if next == start {
                break;
            }
            if loop_vertices.contains(&next) {
                return None;
            }
            previous = Some(current);
            current = next;
            if loop_vertices.len() > boundary_edges.len() {
                return None;
            }
        }
        if loop_vertices.len() < 3 {
            return None;
        }
        loops.push(loop_vertices);
    }
    if loops.is_empty() || used.iter().any(|used| !*used) {
        None
    } else {
        Some(loops)
    }
}

fn mesh_triangle_edges(triangle: [usize; 3]) -> [(usize, usize); 3] {
    [
        (triangle[0], triangle[1]),
        (triangle[1], triangle[2]),
        (triangle[2], triangle[0]),
    ]
}

fn canonical_mesh_edge(a: usize, b: usize) -> (usize, usize) {
    if a <= b { (a, b) } else { (b, a) }
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

fn coplanar_mesh_overlay_materialized_difference_boundary_policy(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ExactArrangement2dBoundaryPolicy> {
    coplanar_mesh_overlay_materialized_boundary_policy(
        left,
        right,
        ExactArrangement2dSetOperation::Difference,
        true,
    )
}

fn coplanar_mesh_overlay_materialized_boundary_policy(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactArrangement2dSetOperation,
    allow_empty: bool,
) -> Option<ExactArrangement2dBoundaryPolicy> {
    for boundary_policy in [
        ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
        ExactArrangement2dBoundaryPolicy::PreserveCollinear,
    ] {
        if materialize_coplanar_mesh_overlay_mesh(
            left,
            right,
            operation,
            boundary_policy,
            "exact coplanar mesh overlay arrangement",
            allow_empty,
        )
        .is_some()
        {
            return Some(boundary_policy);
        }
    }
    None
}

fn coplanar_mesh_overlay_difference_materializes(left: &ExactMesh, right: &ExactMesh) -> bool {
    coplanar_mesh_overlay_materialized_difference_boundary_policy(left, right).is_some()
}

fn coplanar_mesh_overlay_surface_intersection_boundary_policy(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ExactArrangement2dBoundaryPolicy> {
    coplanar_mesh_overlay_materialized_boundary_policy(
        left,
        right,
        ExactArrangement2dSetOperation::Intersection,
        true,
    )
}

fn coplanar_mesh_overlay_surface_union_boundary_policy(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ExactArrangement2dBoundaryPolicy> {
    coplanar_mesh_overlay_materialized_boundary_policy(
        left,
        right,
        ExactArrangement2dSetOperation::Union,
        false,
    )
}

#[cfg(test)]
fn exact_meshes_have_same_shape(left: &ExactMesh, right: &ExactMesh) -> bool {
    (exact_mesh_vertex_sets_match(left, right) && left.triangles().len() == right.triangles().len())
        || exact_mesh_boundary_edges_match(left, right)
}

#[cfg(test)]
fn exact_mesh_vertex_sets_match(left: &ExactMesh, right: &ExactMesh) -> bool {
    left.vertices().len() == right.vertices().len()
        && left.vertices().iter().all(|left_point| {
            right
                .vertices()
                .iter()
                .any(|right_point| point3_exact_equal(left_point, right_point) == Some(true))
        })
        && right.vertices().iter().all(|right_point| {
            left.vertices()
                .iter()
                .any(|left_point| point3_exact_equal(left_point, right_point) == Some(true))
        })
}

#[cfg(test)]
#[derive(Clone, Debug)]
struct ExactBoundaryEdge {
    endpoints: [Point3; 2],
    count: usize,
}

#[cfg(test)]
fn exact_mesh_boundary_edges_match(left: &ExactMesh, right: &ExactMesh) -> bool {
    let Some(left_edges) = exact_mesh_boundary_edges(left) else {
        return false;
    };
    let Some(right_edges) = exact_mesh_boundary_edges(right) else {
        return false;
    };
    !left_edges.is_empty()
        && left_edges.len() == right_edges.len()
        && left_edges.iter().all(|left_edge| {
            right_edges.iter().any(|right_edge| {
                left_edge.count == right_edge.count
                    && point3_edge_exact_equal(&left_edge.endpoints, &right_edge.endpoints)
                        == Some(true)
            })
        })
        && right_edges.iter().all(|right_edge| {
            left_edges.iter().any(|left_edge| {
                left_edge.count == right_edge.count
                    && point3_edge_exact_equal(&right_edge.endpoints, &left_edge.endpoints)
                        == Some(true)
            })
        })
}

#[cfg(test)]
fn exact_mesh_boundary_edges(mesh: &ExactMesh) -> Option<Vec<ExactBoundaryEdge>> {
    let mut edges = Vec::<ExactBoundaryEdge>::new();
    for triangle in mesh.triangles() {
        for [start, end] in triangle_edges(triangle) {
            let edge = [
                mesh.vertices().get(start)?.clone(),
                mesh.vertices().get(end)?.clone(),
            ];
            if let Some(existing) = edges
                .iter_mut()
                .find(|existing| point3_edge_exact_equal(&existing.endpoints, &edge) == Some(true))
            {
                existing.count += 1;
            } else {
                edges.push(ExactBoundaryEdge {
                    endpoints: edge,
                    count: 1,
                });
            }
        }
    }
    if edges.iter().any(|edge| edge.count > 2) {
        return None;
    }
    Some(edges.into_iter().filter(|edge| edge.count == 1).collect())
}

#[cfg(test)]
fn triangle_edges(triangle: &Triangle) -> [[usize; 2]; 3] {
    [
        [triangle.0[0], triangle.0[1]],
        [triangle.0[1], triangle.0[2]],
        [triangle.0[2], triangle.0[0]],
    ]
}

#[cfg(test)]
fn point3_edge_exact_equal(left: &[Point3; 2], right: &[Point3; 2]) -> Option<bool> {
    Some(
        (point3_exact_equal(&left[0], &right[0])? && point3_exact_equal(&left[1], &right[1])?)
            || (point3_exact_equal(&left[0], &right[1])?
                && point3_exact_equal(&left[1], &right[0])?),
    )
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

fn has_axis_aligned_box_difference_cell_result(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> bool {
    operation == ExactBooleanOperation::Difference
        && both_axis_aligned_boxes(left, right)
        && axis_aligned_orthogonal_solid_cell_plan(
            left,
            right,
            AxisAlignedOrthogonalSolidOperation::Difference,
        )
        .is_some()
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

fn boolean_arrangement_affine_orthogonal_solid_recovery(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if match operation {
        ExactBooleanOperation::Union => has_affine_box_union(left, right),
        ExactBooleanOperation::Intersection => has_affine_box_intersection(left, right),
        ExactBooleanOperation::Difference => has_affine_box_difference(left, right),
        ExactBooleanOperation::SelectedRegions(_) => false,
    } {
        return Ok(None);
    }
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
    arrangement.validate_against_sources(left, right)?;
    Ok(Some(certified_shortcut_result(
        arrangement.mesh,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    )))
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

fn boolean_open_surface_disjoint_meshes_from_graph(
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
    Ok(None)
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

fn certified_mixed_dimensional_regularized_solid_support(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ExactBooleanSupport> {
    let left_closed = left.facts().mesh.closed_manifold;
    let right_closed = right.facts().mesh.closed_manifold;
    let left_open_surface = mesh_is_open_surface(left);
    let right_open_surface = mesh_is_open_surface(right);
    if (left_closed && right_open_surface) || (left_open_surface && right_closed) {
        Some(ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid)
    } else {
        None
    }
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
    } else if coplanar_surface_output_materializes_for_preflight(left, right, operation)? {
        ExactPlanarArrangementStatus::AlreadyMaterialized
    } else if graph_requires_boundary_policy(graph, left, right)? {
        ExactPlanarArrangementStatus::BoundaryPolicyRequired
    } else if graph_requires_planar_arrangement(graph)
        && certified_arrangement_cell_complex_preflight_if_materialized(
            operation, graph, left, right,
        )?
        .is_some()
    {
        ExactPlanarArrangementStatus::AlreadyMaterialized
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

fn coplanar_surface_output_materializes_for_preflight(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<bool, MeshError> {
    boolean_coplanar_mesh_overlay_optional(left, right, operation, ValidationPolicy::ALLOW_BOUNDARY)
        .map(|result| result.is_some())
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
        if certified_arrangement_cell_complex_preflight_if_materialized(
            operation, graph, left, right,
        )?
        .is_some()
        {
            return Ok(winding_readiness_report(
                operation,
                ExactWindingReadinessStatus::CoplanarVolumetricCellsAlreadyMaterialized,
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

fn materialize_volumetric_winding_region_plan_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
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
        return match operation {
            ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference => {
                Ok(Some(MaterializedVolumetricWindingRegionPlan {
                    region_classifications,
                    triangulations,
                    volumetric_classifications,
                    assembly: ExactBooleanAssemblyPlan {
                        vertices: Vec::new(),
                        triangles: Vec::new(),
                    },
                    mesh: empty_mesh(
                        "empty exact volumetric arrangement cell-complex result",
                        validation,
                    )?,
                }))
            }
            ExactBooleanOperation::Union | ExactBooleanOperation::SelectedRegions(_) => Ok(None),
        };
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
        Err(_) => return Ok(None),
    };
    if assembly
        .refine_edges_at_existing_vertices(left, right)
        .is_err()
    {
        return Ok(None);
    }
    if assembly.orient_paired_edge_uses().is_err() {
        return Ok(None);
    }
    if assembly.remove_duplicate_triangle_vertex_sets().is_err() {
        return Ok(None);
    }
    if assembly.orient_paired_edge_uses().is_err() {
        return Ok(None);
    }
    if operation == ExactBooleanOperation::Difference
        && assembly.split_disconnected_vertex_fans().is_err()
    {
        return Ok(None);
    }
    let mesh = match assembly.checked_to_exact_mesh_with_sources(left, right, validation) {
        Ok(mesh) => mesh,
        Err(_error) => return Ok(None),
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

fn certified_convex_union_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    match operation {
        ExactBooleanOperation::Union if union_closed_convex_solids(left, right).is_some() => {
            Some(ExactBooleanSupport::CertifiedConvexUnion)
        }
        _ => None,
    }
}

fn certified_convex_difference_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    match operation {
        ExactBooleanOperation::Difference
            if subtract_closed_convex_solids(left, right).is_some() =>
        {
            Some(ExactBooleanSupport::CertifiedConvexDifference)
        }
        _ => None,
    }
}

fn certified_direct_convex_boolean_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    certified_convex_intersection_support(left, right, operation)
}

fn certified_convex_boolean_support_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ExactBooleanSupport>, MeshError> {
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
        return Ok(support);
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
        return Ok(Some(ExactBooleanSupport::CertifiedConvexContainment));
    }
    if operation == ExactBooleanOperation::Difference && left_boundary_inside_right {
        return Ok(Some(ExactBooleanSupport::CertifiedConvexContainment));
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
        hyperlimit::SourceProvenance::exact(label),
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
        hyperlimit::SourceProvenance::exact(label),
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

fn boolean_closed_regularized_lower_dimensional_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(left_kind) = closed_regularized_operand_kind(left) else {
        return Ok(None);
    };
    let Some(right_kind) = closed_regularized_operand_kind(right) else {
        return Ok(None);
    };
    if left_kind.has_volume() && right_kind.has_volume() {
        return Ok(None);
    }
    if !left_kind.has_volume()
        && !right_kind.has_volume()
        && !matches!(validation, ValidationPolicy::CLOSED)
    {
        return Ok(None);
    }

    let (mesh, shortcut) = match (left_kind, right_kind, operation) {
        (
            ClosedRegularizedOperandKind::ClosedSolid,
            ClosedRegularizedOperandKind::LowerDimensional,
            ExactBooleanOperation::Union | ExactBooleanOperation::Difference,
        ) => (
            copy_mesh(
                left,
                "exact mixed-dimensional regularized solid keeps left",
                validation,
            )?,
            ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid,
        ),
        (
            ClosedRegularizedOperandKind::LowerDimensional,
            ClosedRegularizedOperandKind::ClosedSolid,
            ExactBooleanOperation::Union,
        ) => (
            copy_mesh(
                right,
                "exact mixed-dimensional regularized solid keeps right",
                validation,
            )?,
            ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid,
        ),
        (
            ClosedRegularizedOperandKind::ClosedSolid,
            ClosedRegularizedOperandKind::LowerDimensional,
            ExactBooleanOperation::Intersection,
        )
        | (
            ClosedRegularizedOperandKind::LowerDimensional,
            ClosedRegularizedOperandKind::ClosedSolid,
            ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference,
        ) => (
            empty_mesh(
                "empty exact mixed-dimensional regularized solid result",
                validation,
            )?,
            ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid,
        ),
        (
            ClosedRegularizedOperandKind::LowerDimensional,
            ClosedRegularizedOperandKind::LowerDimensional,
            ExactBooleanOperation::Union
            | ExactBooleanOperation::Intersection
            | ExactBooleanOperation::Difference,
        ) => (
            empty_mesh(
                "empty exact lower-dimensional regularized solid result",
                validation,
            )?,
            ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid,
        ),
        _ => return Ok(None),
    };

    Ok(Some(certified_shortcut_result(mesh, shortcut)))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ClosedRegularizedOperandKind {
    ClosedSolid,
    LowerDimensional,
}

impl ClosedRegularizedOperandKind {
    fn has_volume(self) -> bool {
        matches!(self, Self::ClosedSolid)
    }
}

fn closed_regularized_operand_kind(mesh: &ExactMesh) -> Option<ClosedRegularizedOperandKind> {
    if !mesh.triangles().is_empty() && mesh.facts().mesh.closed_manifold {
        Some(ClosedRegularizedOperandKind::ClosedSolid)
    } else if mesh.triangles().is_empty() || mesh_is_open_surface(mesh) {
        Some(ClosedRegularizedOperandKind::LowerDimensional)
    } else {
        None
    }
}

fn boolean_disjoint_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes(left, right, validation)?,
        ExactBooleanOperation::Intersection => {
            empty_mesh("empty exact bounds-disjoint intersection", validation)?
        }
        ExactBooleanOperation::Difference => copy_mesh(
            left,
            "exact bounds-disjoint difference keeps left",
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };
    Ok(certified_shortcut_result(
        mesh,
        ExactBooleanShortcutKind::BoundsDisjoint,
    ))
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
            hyperlimit::SourceProvenance::exact("exact difference with empty right operand"),
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
                hyperlimit::SourceProvenance::exact("exact identical boolean result"),
                validation,
            )?
        }
        ExactBooleanOperation::Difference => ExactMesh::new_with_policy(
            Vec::new(),
            Vec::new(),
            hyperlimit::SourceProvenance::exact("empty exact identical difference"),
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
        hyperlimit::SourceProvenance::exact(label),
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
        hyperlimit::SourceProvenance::exact("exact disjoint union"),
        validation,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_mesh_shape_accepts_same_boundary_with_different_triangulation() {
        let diagonal = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let centered = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, 2, 2, 0],
            &[0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        assert!(exact_mesh_boundary_edges_match(&diagonal, &centered));
        assert!(exact_meshes_have_same_shape(&diagonal, &centered));
    }

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

    #[test]
    fn selected_overlay_faces_triangulate_simple_coplanar_difference_cells() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let (carrier_points, projection) = coplanar_mesh_overlay_carrier(&left, &right).unwrap();
        let boundary_policy =
            coplanar_mesh_overlay_materialized_difference_boundary_policy(&left, &right).unwrap();
        let mut rings =
            projected_mesh_boundary_rings(ExactArrangement2dRegion::Left, &left, projection)
                .unwrap();
        rings.extend(
            projected_mesh_boundary_rings(ExactArrangement2dRegion::Right, &right, projection)
                .unwrap(),
        );
        let overlay = build_exact_arrangement2d_overlay_with_boundary_policy(
            &rings,
            ExactArrangement2dSetOperation::Difference,
            boundary_policy,
        );
        assert!(overlay.is_complete());
        let selected_faces = mesh_from_selected_projected_overlay_faces(
            &overlay,
            &carrier_points,
            projection,
            "test selected-face coplanar overlay difference",
        )
        .expect("selected arrangement faces should triangulate directly");
        let canonical = materialize_coplanar_mesh_overlay_mesh(
            &left,
            &right,
            ExactArrangement2dSetOperation::Difference,
            boundary_policy,
            "test canonical coplanar overlay difference",
            false,
        )
        .expect("canonical overlay should materialize");
        assert!(exact_meshes_have_same_shape(&selected_faces, &canonical));
    }

    #[test]
    fn selected_overlay_faces_recover_point_touching_hole_components() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 8, 0, 0, 8, 8, 0, 0, 8, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
                3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let (carrier_points, projection) = coplanar_mesh_overlay_carrier(&left, &right).unwrap();
        let boundary_policy =
            coplanar_mesh_overlay_materialized_difference_boundary_policy(&left, &right).unwrap();
        let mut rings =
            projected_mesh_boundary_rings(ExactArrangement2dRegion::Left, &left, projection)
                .unwrap();
        rings.extend(
            projected_mesh_boundary_rings(ExactArrangement2dRegion::Right, &right, projection)
                .unwrap(),
        );
        let overlay = build_exact_arrangement2d_overlay_with_boundary_policy(
            &rings,
            ExactArrangement2dSetOperation::Difference,
            boundary_policy,
        );
        assert!(overlay.is_complete(), "{:?}", overlay.blockers);

        let selected_faces = mesh_from_selected_projected_overlay_faces(
            &overlay,
            &carrier_points,
            projection,
            "test selected-face point-touching hole overlay difference",
        )
        .expect("selected arrangement faces should recover component loops");
        let canonical = materialize_coplanar_mesh_overlay_mesh(
            &left,
            &right,
            ExactArrangement2dSetOperation::Difference,
            boundary_policy,
            "test canonical point-touching hole overlay difference",
            false,
        )
        .expect("canonical overlay should materialize");
        assert!(exact_meshes_have_same_shape(&selected_faces, &canonical));
    }

    #[test]
    fn selected_overlay_faces_absorb_contained_union_components() {
        let outer_square = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let inner_square = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 2, 1, 0, 2, 2, 0, 1, 2, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let (carrier_points, projection) =
            coplanar_mesh_overlay_carrier(&outer_square, &inner_square).unwrap();
        let mut rings = projected_mesh_boundary_rings(
            ExactArrangement2dRegion::Left,
            &outer_square,
            projection,
        )
        .unwrap();
        rings.extend(
            projected_mesh_boundary_rings(
                ExactArrangement2dRegion::Right,
                &inner_square,
                projection,
            )
            .unwrap(),
        );
        let overlay = build_exact_arrangement2d_overlay_with_boundary_policy(
            &rings,
            ExactArrangement2dSetOperation::Union,
            ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
        );
        assert!(overlay.is_complete(), "{:?}", overlay.blockers);

        let selected_faces = mesh_from_selected_projected_overlay_faces(
            &overlay,
            &carrier_points,
            projection,
            "test selected-face contained union overlay",
        )
        .expect("selected arrangement faces should absorb contained components");
        assert!(exact_meshes_have_same_shape(&selected_faces, &outer_square));
    }

    #[test]
    fn projected_overlay_mesh_uses_certified_output_components() {
        let ring = |region, points: &[(i64, i64)]| {
            ExactArrangement2dRegionRing::new(
                region,
                points
                    .iter()
                    .map(|&(x, y)| Point2::new(Real::from(x), Real::from(y)))
                    .collect(),
            )
        };
        let overlay = build_exact_arrangement2d_overlay(
            &[
                ring(
                    ExactArrangement2dRegion::Left,
                    &[(0, 0), (8, 0), (8, 8), (0, 8)],
                ),
                ring(
                    ExactArrangement2dRegion::Left,
                    &[(1, 1), (1, 7), (7, 7), (7, 1)],
                ),
                ring(
                    ExactArrangement2dRegion::Left,
                    &[(3, 3), (5, 3), (5, 5), (3, 5)],
                ),
            ],
            ExactArrangement2dSetOperation::Union,
        );
        assert!(overlay.is_complete(), "{:?}", overlay.blockers);
        assert_eq!(overlay.output_components.len(), 2);

        let mut output_only_overlay = overlay.clone();
        output_only_overlay.faces.clear();
        let carrier_points = [
            Point3::new(Real::from(0), Real::from(0), Real::from(0)),
            Point3::new(Real::from(1), Real::from(0), Real::from(0)),
            Point3::new(Real::from(0), Real::from(1), Real::from(0)),
        ];
        let projection = choose_triangle_projection(&carrier_points).unwrap();

        let mesh = mesh_from_selected_projected_overlay_faces(
            &output_only_overlay,
            &carrier_points,
            projection,
            "test certified output-component overlay",
        )
        .expect("certified output components should triangulate without face-walk replay");
        mesh.validate_retained_state().unwrap();
        assert!(!mesh.triangles().is_empty());

        let mut stale_overlay = overlay;
        let outer_loop = stale_overlay.output_components[0].outer_loop;
        stale_overlay.output_loops[outer_loop].points.truncate(2);
        assert!(
            mesh_from_selected_projected_overlay_faces(
                &stale_overlay,
                &carrier_points,
                projection,
                "test stale certified output-component overlay",
            )
            .is_none()
        );
    }

    #[test]
    fn selected_overlay_faces_recover_when_output_loop_ownership_is_blocked() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 8, 0, 0, 8, 8, 0, 0, 8, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
                3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let (carrier_points, projection) = coplanar_mesh_overlay_carrier(&left, &right).unwrap();
        let boundary_policy =
            coplanar_mesh_overlay_materialized_difference_boundary_policy(&left, &right).unwrap();
        let mut rings =
            projected_mesh_boundary_rings(ExactArrangement2dRegion::Left, &left, projection)
                .unwrap();
        rings.extend(
            projected_mesh_boundary_rings(ExactArrangement2dRegion::Right, &right, projection)
                .unwrap(),
        );
        let overlay = build_exact_arrangement2d_overlay_with_boundary_policy(
            &rings,
            ExactArrangement2dSetOperation::Difference,
            boundary_policy,
        );
        assert!(overlay.is_complete(), "{:?}", overlay.blockers);

        let canonical = mesh_from_selected_projected_overlay_faces(
            &overlay,
            &carrier_points,
            projection,
            "test canonical selected-face overlay",
        )
        .expect("complete overlay should materialize through output components");

        let mut blocked_loop_ownership = overlay;
        blocked_loop_ownership.output_loops.clear();
        blocked_loop_ownership.output_components.clear();
        blocked_loop_ownership.blockers.push(
            ExactArrangement2dBlocker::OutputLoopBoundaryContainment {
                container_loop: 0,
                child_loop: 1,
            },
        );

        let recovered = mesh_from_selected_projected_overlay_faces(
            &blocked_loop_ownership,
            &carrier_points,
            projection,
            "test selected-face recovery overlay",
        )
        .expect("selected faces should recover when only loop ownership is blocked");
        recovered.validate_retained_state().unwrap();
        assert!(exact_meshes_have_same_shape(&recovered, &canonical));
    }

    #[test]
    fn selected_overlay_faces_do_not_recover_selected_boundary_topology_blockers() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let (carrier_points, projection) = coplanar_mesh_overlay_carrier(&left, &right).unwrap();
        let mut rings =
            projected_mesh_boundary_rings(ExactArrangement2dRegion::Left, &left, projection)
                .unwrap();
        rings.extend(
            projected_mesh_boundary_rings(ExactArrangement2dRegion::Right, &right, projection)
                .unwrap(),
        );
        let mut overlay =
            build_exact_arrangement2d_overlay(&rings, ExactArrangement2dSetOperation::Difference);
        assert!(overlay.is_complete(), "{:?}", overlay.blockers);
        overlay.output_loops.clear();
        overlay.output_components.clear();
        overlay
            .blockers
            .push(ExactArrangement2dBlocker::NonManifoldSelectedBoundary { vertex: 0 });

        assert!(
            mesh_from_selected_projected_overlay_faces(
                &overlay,
                &carrier_points,
                projection,
                "test rejected selected-boundary topology blocker",
            )
            .is_none()
        );
    }

    #[test]
    fn coplanar_overlay_certifies_component_holed_contact_difference() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let opening_plus_hole = ExactMesh::from_i64_triangles_with_policy(
            &[
                8, 8, 0, 12, 10, 0, 8, 12, 0, //
                0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0, //
                15, 15, 0, 17, 15, 0, 17, 17, 0, 15, 17, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6, //
                7, 8, 9, 7, 9, 10,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        assert!(coplanar_mesh_overlay_difference_materializes(
            &left,
            &opening_plus_hole
        ));
        let result = boolean_coplanar_mesh_overlay_optional(
            &left,
            &opening_plus_hole,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .expect("certified overlay should materialize component-holed difference");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
    }

    #[test]
    fn coplanar_overlay_materializes_point_touching_hole_difference() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 8, 0, 0, 8, 8, 0, 0, 8, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let touching_holes = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
                3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        let result = boolean_coplanar_mesh_overlay_optional(
            &left,
            &touching_holes,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .expect("point-touching holed difference should materialize");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        result.mesh.validate_retained_state().unwrap();
    }

    #[test]
    fn coplanar_overlay_materializes_containment_union_and_intersection() {
        let outer_triangle = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let inner_triangle = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 2, 1, 0, 1, 2, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let outer_square = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let inner_square = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 2, 1, 0, 2, 2, 0, 1, 2, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        for (outer, inner) in [
            (&outer_triangle, &inner_triangle),
            (&outer_square, &inner_square),
        ] {
            let union = materialize_coplanar_mesh_overlay_mesh(
                outer,
                inner,
                ExactArrangement2dSetOperation::Union,
                ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
                "test coplanar containment union overlay",
                false,
            )
            .expect("containment union should materialize through arrangement overlay");
            assert!(exact_meshes_have_same_shape(&union, outer));

            let intersection = materialize_coplanar_mesh_overlay_mesh(
                outer,
                inner,
                ExactArrangement2dSetOperation::Intersection,
                ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
                "test coplanar containment intersection overlay",
                false,
            )
            .expect("containment intersection should materialize through arrangement overlay");
            assert!(exact_meshes_have_same_shape(&intersection, inner));
        }
    }

    #[test]
    fn arrangement_preempts_multi_triangle_coplanar_overlay_including_containment() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 2, 0, 6, 2, 0, 6, 6, 0, 2, 6, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        assert!(coplanar_mesh_overlay_should_preempt_surface_paths(
            &left,
            &right,
            ExactBooleanOperation::Union
        ));

        let inner = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 2, 1, 0, 1, 2, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let union = boolean_exact(
            &inner,
            &left,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("contained coplanar union should materialize through arrangement");
        assert_eq!(
            union.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
    }

    #[test]
    fn materialized_arrangement_preflight_probe_certifies_full_pipeline_output() {
        let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
        let graph = build_intersection_graph(&left, &right).unwrap();

        let preflight = certified_arrangement_cell_complex_preflight_if_materialized(
            ExactBooleanOperation::Union,
            &graph,
            &left,
            &right,
        )
        .unwrap()
        .expect("overlapping exact boxes should materialize through arrangement");

        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex
        );
        assert!(preflight.blocker.is_none());
        assert_eq!(preflight.retained_face_pairs, graph.face_pairs.len());
        assert_eq!(preflight.retained_events, graph.event_count());
    }

    fn arrangement_attempt_certified_for_preflight_with_validation(
        left: &ExactMesh,
        right: &ExactMesh,
        operation: ExactBooleanOperation,
        validation: ValidationPolicy,
    ) -> bool {
        match run_arrangement_cell_complex_attempt(
            left,
            right,
            operation,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
            Some(validation),
            true,
        ) {
            Ok(ArrangementCellComplexOutcome::Materialized(_, attempt)) => {
                arrangement_cell_complex_attempt_is_certified_for_preflight(&attempt)
            }
            Ok(ArrangementCellComplexOutcome::Declined(_)) | Err(_) => false,
        }
    }

    #[test]
    fn arrangement_preflight_probe_accepts_boundary_valid_open_output() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 5, 1, 0, 1, 5, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
        ] {
            assert!(
                !arrangement_attempt_certified_for_preflight_with_validation(
                    &left,
                    &right,
                    operation,
                    ValidationPolicy::CLOSED
                )
            );
            assert!(arrangement_attempt_certified_for_preflight_with_validation(
                &left,
                &right,
                operation,
                ValidationPolicy::ALLOW_BOUNDARY
            ));
            assert!(
                arrangement_cell_complex_materializes_for_preflight(&left, &right, operation, true)
                    .unwrap()
            );
        }
    }

    #[test]
    fn crossing_open_surface_boolean_materializes_inside_arrangement_attempt() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[1, -1, -1, 1, 3, 1, 1, 3, -1],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let attempt = exact_arrangement_boolean_attempt_report(
                &left,
                &right,
                operation,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            )
            .expect("arrangement attempt should run");
            assert_eq!(
                attempt.stage,
                ExactArrangementBooleanStage::Materialized,
                "{operation:?}: {attempt:?}"
            );
            assert!(attempt.decline.is_none(), "{operation:?}: {attempt:?}");
            if !matches!(operation, ExactBooleanOperation::Intersection) {
                assert!(attempt.output_triangles > 0, "{operation:?}: {attempt:?}");
            }

            let result = boolean_exact(&left, &right, operation, ValidationPolicy::ALLOW_BOUNDARY)
                .expect("open-surface crossing should materialize");
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::OpenSurfaceArrangement { operation }
            );
            result.validate().unwrap();
            result.validate_against_sources(&left, &right).unwrap();
        }
    }

    #[test]
    fn partial_face_boundary_touch_is_regularized_without_coplanar_cell_blocker() {
        let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
        let right = tetrahedron_i64([2, 2, 2], [4, 1, 1], [1, 4, 1], [3, 3, 3]);

        let intersection =
            preflight_boolean_exact(&left, &right, ExactBooleanOperation::Intersection).unwrap();
        assert_eq!(
            intersection.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex
        );
        assert!(intersection.blocker.is_none());

        let difference =
            preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference).unwrap();
        assert_eq!(
            difference.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex
        );
        assert!(difference.blocker.is_none());

        let intersection = boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert!(intersection.mesh.triangles().is_empty());

        let difference = boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert!(exact_meshes_have_same_shape(&difference.mesh, &left));
    }

    #[test]
    fn nested_closed_shell_booleans_materialize_through_arrangement_pipeline() {
        let left = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
        let right = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);

        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let preflight = preflight_boolean_exact(&left, &right, operation).unwrap();
            assert_eq!(
                preflight.support,
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                "{operation:?}: {preflight:?}"
            );
            assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");

            let attempt = exact_arrangement_boolean_attempt_report(
                &left,
                &right,
                operation,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            )
            .unwrap();
            assert_eq!(
                attempt.materialized_shortcut,
                Some(ExactBooleanShortcutKind::ArrangementCellComplex),
                "{operation:?}: {attempt:?}"
            );
            assert!(attempt.decline.is_none(), "{operation:?}: {attempt:?}");

            let result = boolean_exact(&left, &right, operation, ValidationPolicy::CLOSED).unwrap();
            result.validate().unwrap();
            result.validate_against_sources(&left, &right).unwrap();
            assert!(
                result.mesh.facts().mesh.closed_manifold,
                "{operation:?}: {:?}",
                result.mesh.facts().mesh
            );
        }
    }

    #[test]
    fn closed_boundary_touching_union_materializes_through_arrangement_pipeline() {
        let left = axis_aligned_box_i64([0, 0, 0], [1, 1, 1]);
        let right = axis_aligned_box_i64([1, 0, 0], [2, 1, 1]);

        let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
            .expect("preflight should certify face-touching closed union");
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{preflight:?}"
        );
        assert!(preflight.blocker.is_none(), "{preflight:?}");

        let attempt = exact_arrangement_boolean_attempt_report(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .expect("arrangement attempt should run");
        assert_eq!(
            attempt.materialized_shortcut,
            Some(ExactBooleanShortcutKind::ArrangementCellComplex),
            "{attempt:?}"
        );
        assert!(attempt.decline.is_none(), "{attempt:?}");

        let result = boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .expect("face-touching closed union should materialize");
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert!(result.mesh.facts().mesh.closed_manifold);
    }

    #[test]
    fn boundary_attached_contained_tetrahedron_difference_materializes() {
        let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
        let right = tetrahedron_i64([2, 2, 2], [4, 1, 1], [1, 4, 1], [1, 1, 1]);

        let preflight =
            preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference).unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex
        );
        assert!(preflight.blocker.is_none(), "{preflight:?}");

        let difference = boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        difference.validate().unwrap();
        difference.validate_against_sources(&left, &right).unwrap();
        assert!(difference.mesh.triangles().len() >= left.triangles().len());
    }

    #[test]
    fn straddling_coplanar_crossing_tetrahedron_boundary_attempt_materializes() {
        let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
        let right = tetrahedron_i64([2, 2, 2], [8, -1, -1], [-1, 8, -1], [3, 2, 0]);

        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let preflight = preflight_boolean_exact(&left, &right, operation).unwrap();
            assert_eq!(
                preflight.support,
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                "{operation:?}: {preflight:?}"
            );
            assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");

            let result = boolean_exact(&left, &right, operation, ValidationPolicy::CLOSED).unwrap();
            result.validate().unwrap();
            result.validate_against_sources(&left, &right).unwrap();
            assert!(
                result.mesh.facts().mesh.closed_manifold,
                "{operation:?}: {:?}",
                result.mesh.facts().mesh
            );

            let attempt = exact_arrangement_boolean_attempt_report(
                &left,
                &right,
                operation,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            )
            .unwrap();
            assert_eq!(
                attempt.materialized_shortcut,
                Some(ExactBooleanShortcutKind::ArrangementCellComplex),
                "{operation:?}: {attempt:?}"
            );
            assert_eq!(attempt.decline, None, "{operation:?}: {attempt:?}");
            assert!(attempt.output_triangles > 0, "{operation:?}: {attempt:?}");
        }
    }

    #[test]
    fn exact_coplanar_boundary_closer_handles_multiple_planar_loops() {
        let mesh = two_open_boxes_missing_top_i64([0, 0, 0], [4, 0, 0]);
        assert_eq!(mesh.facts().mesh.boundary_edges, 8);
        assert!(!mesh.facts().mesh.closed_manifold);

        let closed = close_exact_coplanar_boundary_loops(
            &mesh,
            "test exact multi-loop coplanar boundary closure",
            ValidationPolicy::CLOSED,
        )
        .expect("two planar cap loops should close exactly");

        assert!(closed.facts().mesh.closed_manifold);
        assert_eq!(closed.vertices().len(), mesh.vertices().len());
        assert_eq!(closed.triangles().len(), mesh.triangles().len() + 4);
    }

    #[test]
    fn exact_coplanar_boundary_closer_preserves_hole_loop_groups() {
        let mesh = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, //
                1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
                0, 0, 1, 4, 0, 1, 4, 4, 1, 0, 4, 1, //
                1, 1, 1, 3, 1, 1, 3, 3, 1, 1, 3, 1,
            ],
            &[
                0, 1, 9, 0, 9, 8, //
                1, 2, 10, 1, 10, 9, //
                2, 3, 11, 2, 11, 10, //
                3, 0, 8, 3, 8, 11, //
                4, 12, 13, 4, 13, 5, //
                5, 13, 14, 5, 14, 6, //
                6, 14, 15, 6, 15, 7, //
                7, 15, 12, 7, 12, 4,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        assert_eq!(mesh.facts().mesh.boundary_edges, 16);

        let closed = close_exact_coplanar_boundary_loops(
            &mesh,
            "test exact annular cap closure",
            ValidationPolicy::CLOSED,
        )
        .expect("annular cap loop groups should close exactly");

        assert!(
            closed.facts().mesh.closed_manifold,
            "{:?}",
            closed.facts().mesh
        );
        assert_eq!(closed.vertices().len(), mesh.vertices().len());
        assert!(closed.triangles().len() > mesh.triangles().len());
        assert!(
            closed.vertices().iter().all(|point| point3_exact_equal(
                point,
                &Point3::new(Real::from(2), Real::from(2), Real::from(0))
            ) == Some(false)),
            "annular caps should not introduce a center vertex that fills the hole"
        );
    }

    #[test]
    fn closed_identical_solids_route_through_arrangement_pipeline() {
        let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let right = left.clone();

        let preflight =
            preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union).unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex
        );

        let union = boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            union.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        assert!(exact_meshes_have_same_shape(&union.mesh, &left));

        let difference = boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            difference.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        assert!(difference.mesh.triangles().is_empty());
    }

    #[test]
    fn closed_same_surface_solids_route_through_arrangement_pipeline() {
        let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let right = ExactMesh::from_i64_triangles(
            &[
                4, 0, 0, //
                0, 0, 0, //
                0, 4, 0, //
                0, 0, 4,
            ],
            &[
                1, 2, 0, //
                1, 0, 3, //
                0, 2, 3, //
                2, 1, 3,
            ],
        )
        .unwrap();
        assert!(!meshes_are_certified_identical(&left, &right));
        assert!(meshes_are_certified_same_surface(&left, &right));

        let attempt = exact_arrangement_boolean_attempt_report(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
        assert_eq!(attempt.decline, None);
        assert_eq!(
            attempt.materialized_shortcut,
            Some(ExactBooleanShortcutKind::ArrangementCellComplex)
        );

        let preflight =
            preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union).unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex
        );

        let union = boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            union.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        assert!(exact_meshes_have_same_shape(&union.mesh, &left));

        let difference = boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            difference.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        assert!(difference.mesh.triangles().is_empty());
    }

    #[test]
    fn closed_same_surface_reversed_orientation_routes_through_arrangement_pipeline() {
        let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let right = ExactMesh::from_i64_triangles(
            &[
                4, 0, 0, //
                0, 0, 0, //
                0, 4, 0, //
                0, 0, 4,
            ],
            &[
                1, 0, 2, //
                1, 3, 0, //
                0, 3, 2, //
                2, 3, 1,
            ],
        )
        .unwrap();
        assert!(meshes_are_certified_same_surface(&left, &right));

        let union_attempt = exact_arrangement_boolean_attempt_report(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
        assert_eq!(union_attempt.decline, None);
        assert_eq!(union_attempt.selected_faces, 4);
        assert_eq!(union_attempt.output_triangles, 4);

        let difference_attempt = exact_arrangement_boolean_attempt_report(
            &left,
            &right,
            ExactBooleanOperation::Difference,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
        assert_eq!(difference_attempt.decline, None);
        assert_eq!(difference_attempt.selected_faces, 0);
        assert_eq!(difference_attempt.output_triangles, 0);

        let union = boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            union.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        assert!(exact_meshes_have_same_shape(&union.mesh, &left));

        let difference = boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            difference.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        assert!(difference.mesh.triangles().is_empty());
    }

    #[test]
    fn open_same_surface_sheets_remain_certified() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[4, 0, 0, 0, 4, 0, 0, 0, 0],
            &[2, 0, 1],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        assert!(meshes_are_certified_same_surface(&left, &right));

        let preflight =
            preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union).unwrap();
        assert!(matches!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex
                | ExactBooleanSupport::CertifiedSameSurface
        ));

        let union = boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        assert_eq!(
            union.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
    }

    #[test]
    fn open_identical_sheets_keep_identity_shortcut() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = left.clone();

        let preflight =
            preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union).unwrap();
        assert_eq!(preflight.support, ExactBooleanSupport::CertifiedIdentical);

        let union = boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        assert_eq!(
            union.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::Identical
            }
        );
    }

    #[test]
    fn coplanar_overlay_regularizes_nonconvex_boundary_touch_intersection_to_empty() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 10, 0, 0, 10, 4, 0, 7, 4, 0, 6, 6, 0, 10, 8, 0, 10, 12, 0, 0, 12, 0,
            ],
            &[
                0, 1, 2, //
                0, 2, 3, //
                0, 3, 4, //
                0, 4, 7, //
                7, 4, 5, //
                7, 5, 6,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[4, 12, 0, 6, 12, 0, 6, 14, 0, 4, 14, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        let result = boolean_coplanar_mesh_overlay_optional(
            &left,
            &right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .expect("regularized boundary-touch intersection should materialize through overlay");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        assert!(result.mesh.triangles().is_empty());
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

    fn two_open_boxes_missing_top_i64(first_min: [i64; 3], second_min: [i64; 3]) -> ExactMesh {
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for min in [first_min, second_min] {
            let max = [min[0] + 2, min[1] + 2, min[2] + 2];
            let start = vertices.len() / 3;
            vertices.extend([
                min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0],
                max[1], min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1],
                max[2], min[0], max[1], max[2],
            ]);
            triangles.extend([
                start,
                start + 2,
                start + 1,
                start,
                start + 3,
                start + 2,
                start,
                start + 1,
                start + 5,
                start,
                start + 5,
                start + 4,
                start + 1,
                start + 2,
                start + 6,
                start + 1,
                start + 6,
                start + 5,
                start + 2,
                start + 3,
                start + 7,
                start + 2,
                start + 7,
                start + 6,
                start + 3,
                start,
                start + 4,
                start + 3,
                start + 4,
                start + 7,
            ]);
        }
        ExactMesh::from_i64_triangles_with_policy(
            &vertices,
            &triangles,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
    }
}
