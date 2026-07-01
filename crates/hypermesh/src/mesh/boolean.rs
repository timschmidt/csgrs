//! Exact boolean operation entry points.
//!
//! This module is the exact-stack Boolean boundary for the subset that is
//! currently implemented: build certified
//! intersection events, form exact split-region loops, classify those regions,
//! triangulate them through exact `hypertri`, assemble exact 3D
//! output triangles, and validate the resulting [`ExactMesh`].
//!
//! The operation policy is deliberately explicit. Named booleans converge on
//! the graph-backed arrangement/cell-complex path; shortcut materializers stay
//! only where they can prove coverage for cases that path does not yet support.
//! Remaining split-region cases require a selected-region policy or an explicit
//! unsupported report instead of a silently approximate
//! union/intersection/difference decision. Topology decisions must be certified
//! or represented as policy choices or unknowns.

pub(crate) mod adjacent;
pub(crate) mod affine_solid;
pub(crate) mod cells;
pub(crate) mod contained_adjacent;
pub(crate) mod convex;
pub(crate) mod evidence;
pub(crate) mod orthogonal_solid;
pub(crate) mod region;
pub(crate) mod solid;
pub(crate) mod volumetric;
pub(crate) mod volumetric_cells;
pub(crate) mod winding;

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::ops::ControlFlow;

use hyperlimit::SegmentPlaneRelation;

use super::arrangement3d::arrangement2d::{
    ExactArrangement2dBlocker, ExactArrangement2dBoundaryPolicy, ExactArrangement2dOverlay,
    ExactArrangement2dRegion, ExactArrangement2dRegionRing, ExactArrangement2dSetOperation,
    build_exact_arrangement2d_overlay_with_boundary_policy,
};
use super::arrangement3d::cell_complex::simplify::{
    ExactSimplifiedCellComplex, simplify_selected_cell_complex, triangulate_simplified_cell_complex,
};
use super::arrangement3d::cell_complex::{
    ExactLabeledCellComplex, ExactRegionOwnershipReport, arrangement_cell_complex_labeling_policy,
    arrangement_region_classification_blockers_resolve_operation, select_arrangement_for_replay,
};
use super::arrangement3d::loop_triangulation::{
    group_exact_coplanar_loops, triangulate_exact_loop_group,
};
use super::arrangement3d::regularization::{ExactArrangementBlocker, ExactRegularizationPolicy};
use super::arrangement3d::{
    ExactArrangement3d, ExactTopologyAssemblyReport, ExactTopologyAssemblyStatus,
};
use super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
#[cfg(test)]
use super::graph::FacePairEvents;
#[cfg(test)]
use super::graph::build_unvalidated_intersection_graph;
use super::graph::intersection::MeshFacePairRelation;
use super::graph::{
    ExactIntersectionGraph, IntersectionEvent, MeshSide, build_validated_intersection_graph,
};
use super::prepared::PreparedMeshPair;
use super::validation::ExactMeshValidationPolicy;
use super::view::MeshView;
use super::{ExactMesh, Triangle, point3_exact_equal};
use adjacent::{
    full_face_adjacent_certificate_from_graph,
    materialize_full_face_adjacent_union_from_certificate,
};
use affine_solid::{
    AffineOrthogonalSolidOperation, affine_orthogonal_solid_cell_selected_count,
    materialize_affine_orthogonal_solid_operation,
};
use cells::triangulate_all_face_cells_with_cdt;
use contained_adjacent::{
    contained_face_adjacent_certificate_from_graph,
    materialize_contained_face_adjacent_union_from_certificate,
};
use convex::{
    intersect_closed_convex_solids, subtract_closed_convex_solids, union_closed_convex_solids,
};
use evidence::{
    ExactAdjacentUnionCompletionReport, ExactAdjacentUnionCompletionStatus,
    ExactArrangementBooleanAttempt, ExactArrangementBooleanDecline, ExactArrangementBooleanStage,
    ExactArrangementCellComplexShortcutFacts, ExactBooleanBlocker, ExactBooleanBlockerKind,
    ExactBooleanPreflight, ExactBooleanResult, ExactBooleanResultKind, ExactBooleanShortcutKind,
    ExactBooleanSupport, ExactBoundaryTouchingReport, ExactBoundaryTouchingStatus,
    ExactEvidenceValidationError, ExactIdenticalMeshStatus, ExactOpenSurfaceDisjointReport,
    ExactOpenSurfaceDisjointStatus, ExactPlanarArrangementReport, ExactPlanarArrangementStatus,
    ExactSameSurfaceStatus, ExactVolumetricBoundaryClosureReport,
    ExactVolumetricBoundaryClosureStatus, ExactWindingEvidenceReport, ExactWindingEvidenceStatus,
    certified_convex_operation_shortcut_support, meshes_are_certified_bounds_disjoint,
};
use hyperlimit::SourceProvenance;
use hyperlimit::{
    CoplanarProjection, Point2, Point3, SegmentIntersection, Sign, TriangleLocation,
    classify_point_triangle, compare_reals, orient3d_report, point_on_segment3, project_point3,
    projected_polygon_area2_value,
};
use hyperreal::Real;
use orthogonal_solid::{
    AxisAlignedOrthogonalSolidOperation, axis_aligned_orthogonal_solid_cell_selected_count,
    materialize_axis_aligned_orthogonal_solid_cell_output,
};
#[cfg(test)]
use orthogonal_solid::{
    axis_aligned_orthogonal_solid_cell_plan, is_axis_aligned_box,
    try_certified_axis_aligned_box_pair,
};
use region::{
    ExactBooleanAssemblyPlan, ExactRegionRetention, ExactRegionSelection,
    FaceRegionPlaneClassification, FaceRegionPlaneRelation, FaceRegionTriangulation,
    checked_classify_face_regions_against_opposite_planes,
    checked_triangulate_face_regions_with_earcut, choose_region_projection,
};
use solid::{ConvexSolidMeshRelation, classify_mesh_vertices_against_convex_solid_report};
use std::cmp::Ordering;
use std::rc::Rc;
use volumetric::{
    ExactVolumetricRegionClassification, ExactVolumetricRegionRelation,
    classify_triangulated_regions_against_opposite_meshes,
};
use volumetric_cells::{
    CoplanarVolumetricCellEvidenceError, CoplanarVolumetricCellEvidenceReport,
    CoplanarVolumetricCellObstacle,
};
use winding::{
    ClosedMeshWindingMeshRelation, ClosedMeshWindingMeshReport,
    classify_mesh_vertices_against_closed_mesh_winding_report,
};

#[derive(Clone, Debug, Eq, PartialEq)]
struct DisjointSets {
    parent: Vec<usize>,
}

impl DisjointSets {
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
}

fn closed_boundary_contact_only(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactMeshError> {
    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    left_in_right.validate().map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "adjacent boundary-contact left-in-right winding report failed validation",
            error,
        )
    })?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left.validate().map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "adjacent boundary-contact right-in-left winding report failed validation",
            error,
        )
    })?;
    let Some(left_touches_right_boundary) = left_in_right.boundary_or_outside_touch() else {
        return Ok(false);
    };
    let Some(right_touches_left_boundary) = right_in_left.boundary_or_outside_touch() else {
        return Ok(false);
    };
    Ok(left_touches_right_boundary || right_touches_left_boundary)
}

impl ExactArrangementBooleanAttempt {
    /// Validate this attempt by replaying it for an exact Boolean request.
    pub(crate) fn validate_against_sources_for_request(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        request: ExactBooleanRequest,
    ) -> Result<(), ExactEvidenceValidationError> {
        self.validate()?;
        let shortcut_facts = ExactArrangementCellComplexShortcutFacts::from_sources(left, right);
        if self.materialized_arrangement_cell_complex_shortcut_output()
            && orthogonal_solid_cell_materializes_for_preflight(left, right, request.operation)
                .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            && let Some(replay) = arrangement_cell_complex_shortcut_attempt_with_facts(
                left,
                right,
                request,
                self.policy,
                &shortcut_facts,
            )
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
        {
            replay.validate_for_request_policy(request, self.policy)?;
            return if self == &replay || self.materialized_output_matches_replay(&replay) {
                Ok(())
            } else {
                Err(ExactEvidenceValidationError::SourceReplayMismatch)
            };
        }
        let replay = match ExactArrangement3d::from_meshes_with_policy(left, right, self.policy) {
            Ok(arrangement) => {
                let attempt = match run_arrangement_cell_complex_attempt_from_arrangement(
                    &arrangement,
                    left,
                    right,
                    request,
                    self.policy,
                    true,
                ) {
                    Ok(
                        ArrangementCellComplexOutcome::Materialized(_, attempt)
                        | ArrangementCellComplexOutcome::Declined(attempt),
                    ) => attempt,
                    Err(_) => arrangement_cell_complex_shortcut_attempt_with_facts(
                        left,
                        right,
                        request,
                        self.policy,
                        &shortcut_facts,
                    )
                    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
                    .ok_or(ExactEvidenceValidationError::SourceReplayMismatch)?,
                };
                arrangement_cell_complex_attempt_or_shortcut(
                    left,
                    right,
                    request,
                    self.policy,
                    &shortcut_facts,
                    attempt,
                )
                .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            }
            Err(_) => arrangement_cell_complex_shortcut_attempt_with_facts(
                left,
                right,
                request,
                self.policy,
                &shortcut_facts,
            )
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            .ok_or(ExactEvidenceValidationError::SourceReplayMismatch)?,
        };
        replay.validate_for_request_policy(request, self.policy)?;
        if self == &replay || self.materialized_output_matches_replay(&replay) {
            Ok(())
        } else {
            Err(ExactEvidenceValidationError::SourceReplayMismatch)
        }
    }
}

fn arrangement_cell_complex_attempt_or_shortcut(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
    attempt: ExactArrangementBooleanAttempt,
) -> Result<ExactArrangementBooleanAttempt, ExactMeshError> {
    if attempt.materialized_arrangement_cell_complex_output() {
        return Ok(attempt);
    }
    Ok(arrangement_cell_complex_shortcut_attempt_with_facts(
        left,
        right,
        request,
        policy,
        shortcut_facts,
    )?
    .unwrap_or(attempt))
}

fn retained_evidence_validation_error(
    context: &'static str,
    error: ExactEvidenceValidationError,
    fallback_kind: ExactMeshBlockerKind,
) -> ExactMeshError {
    let kind = match error {
        ExactEvidenceValidationError::SourceReplayMismatch
        | ExactEvidenceValidationError::OutputSourceReplayMismatch => {
            ExactMeshBlockerKind::StaleFactReplay
        }
        _ => fallback_kind,
    };
    ExactMeshError::one(ExactMeshBlocker::new(kind, format!("{context}: {error:?}")))
}

fn arrangement_blocker_error(
    context: &'static str,
    blocker: ExactArrangementBlocker,
) -> ExactMeshError {
    let kind = match blocker {
        ExactArrangementBlocker::UndecidableOrdering
        | ExactArrangementBlocker::UnresolvedIntersection
        | ExactArrangementBlocker::UnresolvedRegionClassification => {
            ExactMeshBlockerKind::UndecidablePredicate
        }
        ExactArrangementBlocker::InvalidIntersectionGraph(_)
        | ExactArrangementBlocker::InvalidSplitPlan(_) => ExactMeshBlockerKind::StaleFactReplay,
        ExactArrangementBlocker::NonManifoldCellComplex
        | ExactArrangementBlocker::UnregularizedCoincidentSheetComplex
        | ExactArrangementBlocker::UnregularizedOpenSheetComplex => {
            ExactMeshBlockerKind::ExactConstructionFailure
        }
    };
    ExactMeshError::one(ExactMeshBlocker::new(
        kind,
        format!("{context}: {blocker:?}"),
    ))
}

fn arrangement_error_declines_or_replays_stale<T>(
    error: ExactMeshError,
) -> Result<Option<T>, ExactMeshError> {
    if error.has_only_blocker_kinds(&[ExactMeshBlockerKind::StaleFactReplay]) {
        Err(error)
    } else {
        Ok(None)
    }
}

fn arrangement_blocker_declines_or_replays_stale<T>(
    context: &'static str,
    blocker: ExactArrangementBlocker,
) -> Result<Option<T>, ExactMeshError> {
    if matches!(
        blocker,
        ExactArrangementBlocker::InvalidIntersectionGraph(_)
            | ExactArrangementBlocker::InvalidSplitPlan(_)
    ) {
        Err(arrangement_blocker_error(context, blocker))
    } else {
        Ok(None)
    }
}

/// Exact boolean operation request.
///
/// Named booleans are represented now, but they intentionally do not fall back
/// to approximate float winding. They prefer the exact graph-backed
/// arrangement/cell-complex path; certified shortcut cases execute only where
/// they cover cases that path does not yet support. Remaining named overlaps
/// return [`ExactMeshBlockerKind::MissingRequiredEvidence`] until split-region
/// inside/outside classification is complete.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactBooleanOperation {
    /// Assemble explicitly selected source-side split regions.
    SelectedRegions(ExactRegionSelection),
    /// Exact union through the graph-backed arrangement/cell-complex path.
    Union,
    /// Exact intersection through the graph-backed arrangement/cell-complex
    /// path.
    Intersection,
    /// Exact difference through the graph-backed arrangement/cell-complex path.
    Difference,
}

impl ExactBooleanOperation {
    pub(crate) const fn is_selected_regions(self) -> bool {
        matches!(self, Self::SelectedRegions(_))
    }

    fn closed_boundary_touching_support(self) -> Option<ExactBooleanSupport> {
        match self {
            Self::Union => Some(ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion),
            Self::Intersection => {
                Some(ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection)
            }
            Self::Difference => {
                Some(ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference)
            }
            Self::SelectedRegions(_) => None,
        }
    }

    fn closed_boundary_touching_shortcut(self) -> Option<ExactBooleanShortcutKind> {
        match self {
            Self::Union => Some(ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion),
            Self::Intersection => {
                Some(ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection)
            }
            Self::Difference => Some(ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference),
            Self::SelectedRegions(_) => None,
        }
    }

    fn convex_operation_support(self) -> Option<ExactBooleanSupport> {
        match self {
            Self::Union => Some(ExactBooleanSupport::CertifiedConvexUnion),
            Self::Intersection => Some(ExactBooleanSupport::CertifiedConvexIntersection),
            Self::Difference => Some(ExactBooleanSupport::CertifiedConvexDifference),
            Self::SelectedRegions(_) => None,
        }
    }

    fn convex_operation_shortcut(self) -> Option<ExactBooleanShortcutKind> {
        match self {
            Self::Union => Some(ExactBooleanShortcutKind::ConvexUnion),
            Self::Intersection => Some(ExactBooleanShortcutKind::ConvexIntersection),
            Self::Difference => Some(ExactBooleanShortcutKind::ConvexDifference),
            Self::SelectedRegions(_) => None,
        }
    }

    fn coplanar_overlay_set_operation(self) -> Option<ExactArrangement2dSetOperation> {
        match self {
            Self::Union => Some(ExactArrangement2dSetOperation::Union),
            Self::Intersection => Some(ExactArrangement2dSetOperation::Intersection),
            Self::Difference => Some(ExactArrangement2dSetOperation::Difference),
            Self::SelectedRegions(_) => None,
        }
    }

    const fn coplanar_overlay_allows_empty(self) -> bool {
        matches!(self, Self::Intersection | Self::Difference)
    }

    const fn coplanar_overlay_boundary_policy(
        self,
        materialized_boundary_policy: ExactArrangement2dBoundaryPolicy,
    ) -> Option<ExactArrangement2dBoundaryPolicy> {
        match self {
            Self::Union => Some(ExactArrangement2dBoundaryPolicy::SimplifyCollinear),
            Self::Intersection | Self::Difference => Some(materialized_boundary_policy),
            Self::SelectedRegions(_) => None,
        }
    }

    fn axis_aligned_orthogonal_solid_operation(
        self,
    ) -> Option<AxisAlignedOrthogonalSolidOperation> {
        match self {
            Self::Union => Some(AxisAlignedOrthogonalSolidOperation::Union),
            Self::Intersection => Some(AxisAlignedOrthogonalSolidOperation::Intersection),
            Self::Difference => Some(AxisAlignedOrthogonalSolidOperation::Difference),
            Self::SelectedRegions(_) => None,
        }
    }

    fn affine_orthogonal_solid_operation(self) -> Option<AffineOrthogonalSolidOperation> {
        match self {
            Self::Union => Some(AffineOrthogonalSolidOperation::Union),
            Self::Intersection => Some(AffineOrthogonalSolidOperation::Intersection),
            Self::Difference => Some(AffineOrthogonalSolidOperation::Difference),
            Self::SelectedRegions(_) => None,
        }
    }

    fn open_surface_region_selection(self) -> Option<ExactRegionSelection> {
        match self {
            Self::Union => Some(ExactRegionSelection::KeepAll),
            Self::Intersection => Some(ExactRegionSelection::KeepNone),
            Self::Difference => Some(ExactRegionSelection::KeepLeft),
            Self::SelectedRegions(_) => None,
        }
    }

    fn open_surface_arrangement_support(self) -> Option<ExactBooleanSupport> {
        match self {
            Self::Union => Some(ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion),
            Self::Intersection => {
                Some(ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection)
            }
            Self::Difference => {
                Some(ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference)
            }
            Self::SelectedRegions(_) => None,
        }
    }
}

/// Complete policy for an exact boolean request.
///
/// The request keeps operation semantics and output validation together so
/// preflight, certification, and materialization replay the same exact
/// contract. Boundary-only contact is always retained as blocker evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ExactBooleanRequest {
    /// Named or selected-region operation to evaluate.
    pub(crate) operation: ExactBooleanOperation,
    /// Output mesh validation policy.
    pub(crate) validation: ExactMeshValidationPolicy,
}

impl ExactBooleanRequest {
    /// Creates a request using the kernel's default exact materialization
    /// policy. Boundary-only contacts are retained as explicit blockers.
    pub(crate) const fn new(
        operation: ExactBooleanOperation,
        validation: ExactMeshValidationPolicy,
    ) -> Self {
        Self {
            operation,
            validation,
        }
    }
}

fn graph_for_certified_materialization<'a>(
    retained_graph: Option<&'a ExactIntersectionGraph>,
    owned_graph: &'a mut Option<ExactIntersectionGraph>,
    prepared_graph: Option<&'a mut Option<Rc<ExactIntersectionGraph>>>,
    prepared_pair: Option<&PreparedMeshPair<'_, '_>>,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<&'a ExactIntersectionGraph, ExactMeshError> {
    if let Some(graph) = retained_graph {
        validate_graph_source_replay(graph, left, right)?;
        return Ok(graph);
    }
    if let (Some(prepared_graph), Some(pair)) = (prepared_graph, prepared_pair) {
        if prepared_graph.is_none() {
            *prepared_graph = Some(pair.validated_intersection_graph()?);
        }
        return prepared_graph.as_deref().ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::ExactConstructionFailure,
                "certified prepared materialization graph was not retained",
            ))
        });
    }
    if owned_graph.is_none() {
        *owned_graph = Some(super::graph::build_validated_intersection_graph(
            left, right,
        )?);
    }
    owned_graph.as_ref().ok_or_else(|| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "certified materialization graph was not retained",
        ))
    })
}

fn boolean_validation_error(
    kind: ExactMeshBlockerKind,
    context: &'static str,
    error: impl fmt::Debug,
) -> ExactMeshError {
    ExactMeshError::one(ExactMeshBlocker::new(kind, format!("{context}: {error:?}")))
}

fn validate_boolean_result(
    result: &ExactBooleanResult,
    context: &'static str,
) -> Result<(), ExactMeshError> {
    result.validate().map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::ExactConstructionFailure,
            context,
            error,
        )
    })
}

fn validate_closed_winding_report(
    report: &ClosedMeshWindingMeshReport,
    sources: Option<(&ExactMesh, &ExactMesh)>,
) -> Result<(), ExactMeshError> {
    let result = match sources {
        Some((subject, target)) => report.validate_against_sources(subject, target),
        None => report.validate(),
    };
    result.map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::StaleFactReplay,
            "exact winding report/source replay failed",
            error,
        )
    })
}

fn materialize_certified_arrangement_cell_complex_support_with_arrangement(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_graph: Option<&ExactIntersectionGraph>,
    retained_regularized_arrangement: Option<&ExactArrangement3d>,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let retained_arrangement_attempt = retained_arrangement_attempt
        .map(|attempt| {
            attempt.validate_for_request_policy(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            )?;
            attempt.validate_against_sources_for_request(left, right, request)?;
            Ok(attempt)
        })
        .transpose()
        .map_err(|error| {
            retained_evidence_validation_error(
                "retained arrangement attempt failed validation",
                error,
                ExactMeshBlockerKind::ExactConstructionFailure,
            )
        })?;
    if shortcut_facts.materializes_operation(operation)
        && let Some(result) =
            boolean_arrangement_cell_complex_recovery(left, right, operation, validation)?
    {
        return Ok(Some(result));
    }
    let mut owned_graph = None;
    let graph = graph_for_certified_materialization(
        retained_graph,
        &mut owned_graph,
        None,
        None,
        left,
        right,
    )?;
    if operation == ExactBooleanOperation::Difference
        && let Some((result, _evidence)) =
            materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
                graph, left, right, operation, validation,
            )?
    {
        return Ok(Some(result));
    }
    if let Some(result) = materialize_arrangement_volumetric_split_cell_result_from_graph(
        graph, left, right, operation, validation,
    )? {
        return Ok(Some(result));
    }
    if let Some(attempt) = retained_arrangement_attempt
        && let Some(result) =
            materialize_retained_arrangement_cell_complex_attempt(left, right, request, attempt)?
    {
        return Ok(Some(result));
    }
    if let Some(result) =
        replay_generic_arrangement_cell_complex_result(graph, left, right, request)?
    {
        return Ok(Some(result));
    }
    if let Some(result) = materialize_arrangement_lower_dimensional_intersection_from_graph(
        graph,
        left,
        right,
        request,
        retained_arrangement_attempt,
        shortcut_facts,
    )? {
        return Ok(Some(result));
    }
    if let Some(arrangement) = retained_regularized_arrangement {
        let outcome = run_arrangement_cell_complex_attempt_from_arrangement(
            arrangement,
            left,
            right,
            request,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
            true,
        )?;
        if let ArrangementCellComplexOutcome::Materialized(result, attempt) = outcome
            && arrangement_cell_complex_result_is_certified_for_preflight(
                &result, &attempt, left, right,
            )?
        {
            return Ok(Some(*result));
        }
    }
    if operation == ExactBooleanOperation::Union
        && let Some((result, _report)) = materialize_adjacent_union_completion_from_graph(
            graph, left, right, operation, validation,
        )?
    {
        return Ok(Some(result));
    }
    if let Some((mesh, closure_report)) =
        materialize_volumetric_coplanar_boundary_closure_output_from_graph(
            graph, left, right, operation, validation,
        )?
    {
        let result = certified_shortcut_result(
            mesh,
            operation,
            ExactBooleanShortcutKind::ArrangementCellComplex,
        );
        let arrangement = ExactArrangement3d::from_source_certified_intersection_graph_with_policy(
            graph.clone(),
            left,
            right,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )?;
        let result =
            result_with_arrangement_gate_reports(result, &arrangement, left, right, operation)?;
        validate_boolean_result(
            &result,
            "exact arrangement cell-complex boundary result validation failed",
        )?;
        validate_volumetric_boundary_closure_report(&closure_report)?;
        let arrangement_mesh = copy_mesh(
            &result.mesh,
            "exact arrangement cell-complex boundary materialization",
            validation,
        )?;
        let arrangement_result = certified_shortcut_result(
            arrangement_mesh,
            operation,
            ExactBooleanShortcutKind::ArrangementCellComplex,
        );
        return Ok(Some(arrangement_result));
    }
    if let Some(result) =
        certified_arrangement_cell_complex_result_from_graph(graph, left, right, request, true)?
    {
        return Ok(Some(result));
    }
    if let Some(result) = request_replayable_result(
        boolean_arrangement_cell_complex_recovery(left, right, operation, validation)?,
        left,
        right,
        request,
        retained_arrangement_attempt,
    ) {
        return Ok(Some(result));
    }
    if let Some((result, _evidence)) =
        materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
            graph, left, right, operation, validation,
        )?
    {
        let arrangement_mesh = copy_mesh(
            &result.mesh,
            "exact arrangement cell-complex no-volume materialization",
            validation,
        )?;
        let arrangement_result = certified_shortcut_result(
            arrangement_mesh,
            operation,
            ExactBooleanShortcutKind::ArrangementCellComplex,
        );
        return Ok(Some(arrangement_result));
    }
    Ok(None)
}

fn materialize_retained_arrangement_cell_complex_attempt(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    attempt: &ExactArrangementBooleanAttempt,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    attempt
        .validate_against_sources_for_request(left, right, request)
        .map_err(|error| {
            boolean_validation_error(
                ExactMeshBlockerKind::StaleFactReplay,
                "retained arrangement attempt failed replay",
                error,
            )
        })?;
    if attempt.materialized_arrangement_cell_complex_shortcut_output() {
        let Some(result) = boolean_arrangement_cell_complex_recovery(
            left,
            right,
            request.operation,
            request.validation,
        )?
        else {
            return Ok(None);
        };
        return if arrangement_cell_complex_result_is_certified_for_preflight(
            &result, attempt, left, right,
        )? {
            Ok(Some(result))
        } else {
            Ok(None)
        };
    }
    if !attempt.materialized_arrangement_cell_complex_output() {
        return Ok(None);
    }
    let Some(simplified) = attempt.simplified_cell_complex_with_retained_gate_reports() else {
        return Ok(None);
    };
    let Some(result) =
        rematerialize_simplified_arrangement_cell_complex(request, simplified, false)?
    else {
        return Ok(None);
    };
    if arrangement_cell_complex_result_is_certified_for_preflight(&result, attempt, left, right)? {
        Ok(Some(result))
    } else {
        Ok(None)
    }
}

fn rematerialize_simplified_arrangement_cell_complex(
    request: ExactBooleanRequest,
    simplified: &ExactSimplifiedCellComplex,
    strict_retained_evidence: bool,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if simplified.operation != request.operation {
        return Ok(None);
    }
    if let Err(blocker) = simplified.validate() {
        return if strict_retained_evidence {
            Err(arrangement_blocker_error(
                "retained simplified cell complex failed validation",
                blocker,
            ))
        } else {
            Ok(None)
        };
    }
    let mesh = match triangulate_simplified_cell_complex(simplified) {
        Ok(mesh) => mesh,
        Err(blocker) => {
            return if strict_retained_evidence {
                Err(arrangement_blocker_error(
                    "retained simplified cell complex triangulation failed",
                    blocker,
                ))
            } else {
                Ok(None)
            };
        }
    };
    let Some((mesh, _closed_by_coplanar_boundary)) = copy_mesh_or_closed_coplanar_boundary_closure(
        &mesh,
        "exact arrangement cell-complex boolean result",
        "exact arrangement cell-complex closed coplanar-boundary result",
        request.validation,
    )?
    else {
        return Ok(None);
    };
    let mut result = certified_shortcut_result(
        mesh,
        request.operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    result.topology_assembly_report = simplified.topology_assembly_report.clone();
    result.region_ownership_report = simplified.region_ownership_report.clone();
    Ok(Some(result))
}

fn replay_generic_arrangement_cell_complex_result(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let operation = request.operation;
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    validate_graph_source_replay(graph, left, right)?;
    let policy = ExactRegularizationPolicy::REGULARIZED_SOLID;
    let arrangement = match ExactArrangement3d::from_source_certified_intersection_graph_with_policy(
        graph.clone(),
        left,
        right,
        policy,
    ) {
        Ok(arrangement) => arrangement,
        Err(error) => return arrangement_error_declines_or_replays_stale(error),
    };
    let selected = match select_arrangement_for_replay(arrangement, left, right, operation, policy)
    {
        Ok(selected) => selected,
        Err(blocker) => {
            return arrangement_blocker_declines_or_replays_stale(
                "exact generic arrangement replay selection failed",
                blocker,
            );
        }
    };
    let simplified = match simplify_selected_cell_complex(selected, policy) {
        Ok(simplified) => simplified,
        Err(blocker) => {
            return arrangement_blocker_declines_or_replays_stale(
                "exact generic arrangement replay simplification failed",
                blocker,
            );
        }
    };
    let Some(result) =
        rematerialize_simplified_arrangement_cell_complex(request, &simplified, false)?
    else {
        return Ok(None);
    };
    validate_boolean_result(
        &result,
        "exact generic arrangement replay result validation failed",
    )?;
    Ok(Some(result))
}

fn replay_selected_region_boolean_result_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    selection: ExactRegionSelection,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactBooleanResult, ExactMeshError> {
    validate_graph_source_replay(graph, left, right)?;
    let graph_had_unknowns = graph.has_unknowns();
    if graph_had_unknowns {
        return Err(ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::UndecidablePredicate,
            "exact boolean graph contains unresolved predicate events",
        )));
    }

    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
    let triangulations = checked_triangulate_face_regions_with_earcut(&region_plan, left, right)
        .map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::DegenerateTriangle,
                format!("exact region triangulation failed: {error}"),
            ))
        })?;
    let (assembly, mesh) = assemble_region_selection_mesh(
        &triangulations,
        left,
        right,
        selection,
        validation,
        "exact boolean assembly failed",
        "exact boolean assembly canonicalization failed",
    )?;

    let result = ExactBooleanResult {
        kind: ExactBooleanResultKind::SelectedRegions { selection },
        graph_had_unknowns,
        region_classifications,
        triangulations,
        assembly,
        volumetric_classifications: Vec::new(),
        topology_assembly_report: None,
        region_ownership_report: None,
        mesh,
    };
    result.validate().map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "exact selected-region result validation failed",
            error,
        )
    })?;
    if !matches!(
        result.kind,
        ExactBooleanResultKind::SelectedRegions {
            selection: result_selection,
        } if result_selection == selection
    ) || result.mesh.validation_policy() != validation
    {
        return Err(ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::StaleFactReplay,
            "exact selected-region replay returned mismatched operation or validation policy",
        )));
    }
    Ok(result)
}

fn assemble_region_selection_mesh(
    triangulations: &[FaceRegionTriangulation],
    left: &ExactMesh,
    right: &ExactMesh,
    selection: ExactRegionSelection,
    validation: ExactMeshValidationPolicy,
    assembly_error_label: &'static str,
    canonicalization_error_label: &'static str,
) -> Result<(ExactBooleanAssemblyPlan, ExactMesh), ExactMeshError> {
    let mut assembly =
        ExactBooleanAssemblyPlan::from_region_triangulations_with_triangle_retention_and_sources(
            triangulations,
            left,
            right,
            |triangulation, _triangle| {
                if selection.keeps(triangulation.side) {
                    ExactRegionRetention::Keep
                } else {
                    ExactRegionRetention::Drop
                }
            },
        )
        .map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::IndexOutOfBounds,
                format!("{assembly_error_label}: {error}"),
            ))
        })?;
    assembly
        .canonicalize_for_mesh_with_sources(left, right)
        .map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::IndexOutOfBounds,
                format!("{canonicalization_error_label}: {error}"),
            ))
        })?;
    let mesh = assembly.checked_to_exact_mesh_with_sources(left, right, validation)?;
    Ok((assembly, mesh))
}

/// Preflight an exact boolean operation without materializing output topology.
///
/// The preflight path deliberately shares the exact graph, region, and
/// classification stages with the executable arrangement pipeline. For named
/// booleans that still need unresolved inside/outside semantics, it returns
/// [`ExactBooleanSupport::RequiresCertifiedWinding`] with replayable facts
/// instead of approximating them.
fn preflight_boolean_exact_request_from_graph_core(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_attempt: Option<&ExactArrangementBooleanAttempt>,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<ExactBooleanPreflight, ExactMeshError> {
    let operation = request.operation;
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
                && evidence::identical_mesh_report_from_sources(left, right).status
                    == ExactIdenticalMeshStatus::Certified =>
        {
            ExactBooleanSupport::CertifiedIdentical
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
                && evidence::same_surface_report_from_sources(left, right).status
                    == ExactSameSurfaceStatus::Certified =>
        {
            ExactBooleanSupport::CertifiedSameSurface
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => {
            if shortcut_facts.materializes_operation(operation) {
                ExactBooleanSupport::CertifiedArrangementCellComplex
            } else {
                certified_mixed_dimensional_regularized_solid_support(left, right)
                    .unwrap_or(ExactBooleanSupport::RequiresCertifiedWinding)
            }
        }
    };
    let requires_certified_winding = support == ExactBooleanSupport::RequiresCertifiedWinding;
    if support == ExactBooleanSupport::CertifiedArrangementCellComplex {
        return Ok(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        )?);
    }
    if support.is_certified()
        && !(support == ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid
            && operation == ExactBooleanOperation::Intersection)
        && !matches!(
            support,
            ExactBooleanSupport::SelectedRegionPolicy
                | ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
                | ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
                | ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference
                | ExactBooleanSupport::CertifiedArrangementCellComplex
        )
    {
        return Ok(certified_preflight(operation, support, Some(graph), None));
    }
    if !operation.is_selected_regions()
        && graph.face_pairs.is_empty()
        && let Some((left_in_right, right_in_left)) =
            closed_winding_vertex_relations_from_empty_graph(graph, left, right)?
        && left_in_right == ClosedMeshWindingMeshRelation::Outside
        && right_in_left == ClosedMeshWindingMeshRelation::Outside
    {
        return Ok(certified_preflight(
            operation,
            ExactBooleanSupport::CertifiedClosedWindingSeparated,
            Some(graph),
            None,
        ));
    }

    if operation == ExactBooleanOperation::Difference && shortcut_facts.axis_aligned_box_pair {
        return Ok(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        )?);
    }
    if operation == ExactBooleanOperation::Difference
        && let Some(evidence) = coplanar_volumetric_evidence_from_graph(graph, left, right)?
            .filter(coplanar_evidence_is_positive_area_boundary_only)
    {
        return Ok(certified_preflight(
            operation,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            Some(graph),
            Some(evidence),
        ));
    }
    let graph_counts = retained_graph_counts(graph);
    let graph_had_unknowns = graph_counts.graph_had_unknowns;
    let relation_counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    let coplanar_volumetric_evidence = coplanar_volumetric_evidence_from_graph(graph, left, right)?
        .filter(coplanar_evidence_requires_volumetric_cells);
    let requires_coplanar_volumetric_cells = coplanar_volumetric_evidence.is_some();
    let mut certified_arrangement_preflight = None;
    if graph_had_unknowns || relation_counts.construction_failed_events > 0 {
        return Ok(graph_counts.into_preflight(
            operation,
            ExactBooleanSupport::UnresolvedGraph,
            0,
            Vec::new(),
            Some(relation_counts.into_blocker(ExactBooleanBlockerKind::Refinement)),
            None,
            None,
        ));
    }
    if operation.is_selected_regions() {
        return region_plan_preflight_from_graph(
            graph,
            left,
            right,
            operation,
            ExactBooleanSupport::SelectedRegionPolicy,
            None,
            None,
        );
    }
    if requires_certified_winding
        && let Some(preflight) = certified_winding_shortcut_preflight_from_graph(
            graph,
            left,
            right,
            request,
            retained_attempt,
            requires_coplanar_volumetric_cells,
            &mut certified_arrangement_preflight,
        )?
    {
        return Ok(preflight);
    }
    if requires_certified_winding
        && matches!(
            operation,
            ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
        )
        && let Some(preflight) =
            certified_closed_boundary_only_contact_preflight(graph, left, right, operation)?
    {
        return Ok(preflight);
    }
    if operation == ExactBooleanOperation::Intersection
        && certified_arrangement_regularized_boundary_contact_from_graph(
            graph, left, right, operation,
        )?
    {
        return Ok(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        )?);
    }
    if requires_certified_winding
        && operation == ExactBooleanOperation::Union
        && let Some(preflight) =
            certified_closed_boundary_only_contact_preflight(graph, left, right, operation)?
    {
        return Ok(preflight);
    }
    let boundary_only_coplanar_evidence =
        coplanar_volumetric_evidence_from_graph(graph, left, right)?
            .filter(coplanar_evidence_is_positive_area_boundary_only);
    if requires_certified_winding && boundary_only_coplanar_evidence.is_none() {
        let boundary_or_no_volume_materialized =
            if materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
                graph,
                left,
                right,
                operation,
                request.validation,
            )?
            .is_some()
            {
                true
            } else {
                materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
                    graph,
                    left,
                    right,
                    operation,
                    request.validation,
                )?
                .is_some()
            };
        if boundary_or_no_volume_materialized
            && let Some(boundary_support) = operation.closed_boundary_touching_support()
        {
            return Ok(certified_preflight(
                operation,
                boundary_support,
                Some(graph),
                None,
            ));
        }
    }
    if requires_certified_winding
        && operation == ExactBooleanOperation::Intersection
        // The empty cavity case can have overlapping AABBs and no graph
        // events, so this retained evidence witness is checked before falling
        // through to winding.
        && axis_aligned_orthogonal_solid_cell_selected_count(
            left,
            right,
            AxisAlignedOrthogonalSolidOperation::Intersection,
        ) == Some(0)
    {
        return Ok(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        )?);
    }
    let open_surface_arrangement_plan =
        open_surface_arrangement_plan_from_graph(graph, left, right, operation)?;
    if requires_certified_winding
        && operation == ExactBooleanOperation::Intersection
        && request.validation == ExactMeshValidationPolicy::CLOSED
        && closed_regularized_operand_kind(left)
            == Some(ClosedRegularizedOperandKind::LowerDimensional)
        && closed_regularized_operand_kind(right)
            == Some(ClosedRegularizedOperandKind::LowerDimensional)
        && !graph.face_pairs.is_empty()
        && open_surface_arrangement_plan.is_some()
    {
        return Ok(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        )?);
    }
    if let Some(plan) = open_surface_arrangement_plan {
        let region_count = unique_classified_region_count(&plan.region_classifications);
        return Ok(graph_counts.into_preflight(
            operation,
            plan.support,
            region_count,
            plan.region_classifications,
            None,
            None,
            None,
        ));
    }
    let boundary_report = boundary_touching_report_from_graph(graph, left, right).ok();
    if let Some(boundary_report) = boundary_report
        && boundary_report.status == ExactBoundaryTouchingStatus::Certified
    {
        return Ok(
            RetainedGraphCounts::from_boundary_touching_report(&boundary_report).into_preflight(
                operation,
                ExactBooleanSupport::RequiresBoundaryOnlyContact,
                0,
                Vec::new(),
                Some(boundary_report.blocker),
                None,
                None,
            ),
        );
    }
    let planar_report = planar_arrangement_report_from_graph_with_cell_complex_cache(
        graph,
        left,
        right,
        operation,
        &mut certified_arrangement_preflight,
        Some(request),
        retained_attempt,
    )
    .ok();
    if let Some(planar_report) = planar_report.as_ref()
        && matches!(planar_report.status, ExactPlanarArrangementStatus::Required)
    {
        if let Some(preflight) = cached_certified_arrangement_cell_complex_preflight(
            &mut certified_arrangement_preflight,
            operation,
            graph,
            left,
            right,
            Some(request),
            retained_attempt,
        )? {
            return Ok(preflight);
        }
        return Ok(
            RetainedGraphCounts::from_planar_arrangement_report(planar_report).into_preflight(
                operation,
                ExactBooleanSupport::RequiresPlanarArrangement,
                0,
                Vec::new(),
                Some(planar_report.blocker.clone()),
                planar_report
                    .coplanar_arrangement_evidence
                    .as_ref()
                    .cloned(),
                None,
            ),
        );
    }
    let planar_arrangement_already_materialized = if let Some(report) = planar_report.as_ref() {
        matches!(
            report.status,
            ExactPlanarArrangementStatus::AlreadyMaterialized
        )
    } else {
        false
    };
    if planar_arrangement_already_materialized
        && let Some(preflight) = cached_certified_arrangement_cell_complex_preflight(
            &mut certified_arrangement_preflight,
            operation,
            graph,
            left,
            right,
            Some(request),
            retained_attempt,
        )?
    {
        return Ok(preflight);
    }
    let convex_operation_preflight_allowed = match operation {
        ExactBooleanOperation::Intersection => !requires_coplanar_volumetric_cells,
        ExactBooleanOperation::Union | ExactBooleanOperation::Difference => true,
        ExactBooleanOperation::SelectedRegions(_) => false,
    };
    if requires_certified_winding
        && convex_operation_preflight_allowed
        && let Some(convex_support) =
            certified_convex_operation_shortcut_support(left, right, operation)
    {
        return Ok(certified_preflight(
            operation,
            convex_support,
            Some(graph),
            None,
        ));
    }
    if requires_coplanar_volumetric_cells {
        let coplanar_closure_available_for_closed_request =
            if request.validation == ExactMeshValidationPolicy::CLOSED {
                coplanar_boundary_closure_available_from_graph(graph, left, right, operation)?
            } else {
                false
            };
        if coplanar_closure_available_for_closed_request {
            return Ok(certified_arrangement_cell_complex_preflight(
                operation, graph, left, right,
            )?);
        }
        if let Some(preflight) = cached_certified_arrangement_cell_complex_preflight(
            &mut certified_arrangement_preflight,
            operation,
            graph,
            left,
            right,
            Some(request),
            retained_attempt,
        )? {
            return Ok(preflight);
        }
        if matches!(
            operation,
            ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
        ) && let Some(convex_support) =
            certified_convex_operation_shortcut_support(left, right, operation)
        {
            return Ok(certified_preflight(
                operation,
                convex_support,
                Some(graph),
                None,
            ));
        }
        let winding_evidence = winding_evidence_report_from_graph_with_facts(
            graph,
            left,
            right,
            operation,
            &shortcut_facts,
        )?;
        if matches!(
            winding_evidence.status,
            ExactWindingEvidenceStatus::Ready
                | ExactWindingEvidenceStatus::NoNontrivialOverlap
                | ExactWindingEvidenceStatus::VolumetricAssemblyRequired
        ) && winding_evidence.blocker.kind == ExactBooleanBlockerKind::CoplanarVolumetricCells
        {
            return Ok(
                RetainedGraphCounts::from_winding_evidence_report(&winding_evidence)
                    .into_preflight(
                        winding_evidence.operation,
                        ExactBooleanSupport::RequiresCertifiedWinding,
                        winding_evidence.region_count,
                        winding_evidence.region_classifications,
                        Some(winding_evidence.blocker),
                        winding_evidence.coplanar_arrangement_evidence,
                        winding_evidence.coplanar_volumetric_evidence,
                    ),
            );
        }
        return Ok(graph_counts.into_preflight(
            operation,
            ExactBooleanSupport::RequiresCoplanarVolumetricCells,
            0,
            Vec::new(),
            Some(relation_counts.into_blocker(ExactBooleanBlockerKind::CoplanarVolumetricCells)),
            None,
            coplanar_volumetric_evidence.clone(),
        ));
    }
    if support == ExactBooleanSupport::RequiresBoundaryOnlyContact {
        return Ok(graph_counts.into_preflight(
            operation,
            support,
            0,
            Vec::new(),
            Some(relation_counts.into_blocker(ExactBooleanBlockerKind::BoundaryOnlyContact)),
            None,
            None,
        ));
    }

    let winding_report = match winding_evidence_report_from_graph_with_facts(
        graph,
        left,
        right,
        operation,
        &shortcut_facts,
    ) {
        Ok(report) => report,
        Err(_) => {
            return region_plan_preflight_from_graph(
                graph,
                left,
                right,
                operation,
                support,
                Some(relation_counts.into_blocker(ExactBooleanBlockerKind::Winding)),
                coplanar_volumetric_evidence.clone(),
            );
        }
    };
    let volumetric_winding_materialized =
        if winding_report.status == ExactWindingEvidenceStatus::Ready {
            materialize_volumetric_winding_region_plan_from_graph(
                graph,
                left,
                right,
                operation,
                ExactMeshValidationPolicy::CLOSED,
            )?
            .is_some()
        } else {
            false
        };
    let closed_boundary_caps_materialized =
        if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
            false
        } else if let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
            graph,
            left,
            right,
            operation,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )? {
            certified_coplanar_boundary_closure_from_materialized(
                &materialized,
                left,
                right,
                operation,
                ExactMeshValidationPolicy::CLOSED,
            )?
            .is_some()
        } else {
            false
        };
    if matches!(
        winding_report.status,
        ExactWindingEvidenceStatus::PlanarArrangementAlreadyMaterialized
            | ExactWindingEvidenceStatus::CoplanarVolumetricCellsAlreadyMaterialized
            | ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized
    ) || volumetric_winding_materialized
        || closed_boundary_caps_materialized
    {
        return Ok(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        )?);
    }

    Ok(
        RetainedGraphCounts::from_winding_evidence_report(&winding_report).into_preflight(
            winding_report.operation,
            support,
            winding_report.region_count,
            winding_report.region_classifications,
            Some(winding_report.blocker),
            None,
            winding_report.coplanar_volumetric_evidence,
        ),
    )
}

fn certified_winding_shortcut_preflight_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_attempt: Option<&ExactArrangementBooleanAttempt>,
    requires_coplanar_volumetric_cells: bool,
    certified_arrangement_preflight: &mut Option<Option<ExactBooleanPreflight>>,
) -> Result<Option<ExactBooleanPreflight>, ExactMeshError> {
    let operation = request.operation;
    if !operation.is_selected_regions()
        && open_surface_disjoint_report_from_graph(graph, left, right).status
            == ExactOpenSurfaceDisjointStatus::Certified
    {
        return Ok(Some(certified_preflight(
            operation,
            ExactBooleanSupport::CertifiedOpenSurfaceDisjoint,
            Some(graph),
            None,
        )));
    }
    let coplanar_closure_available =
        coplanar_boundary_closure_available_from_graph(graph, left, right, operation)?;
    if requires_coplanar_volumetric_cells && coplanar_closure_available {
        return Ok(Some(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        )?));
    }
    if let Some(preflight) = cached_certified_arrangement_cell_complex_preflight(
        certified_arrangement_preflight,
        operation,
        graph,
        left,
        right,
        Some(request),
        retained_attempt,
    )? {
        return Ok(Some(preflight));
    }
    if let Some(convex_support) = certified_convex_relation_shortcut_from_graph(
        graph, left, right, operation,
    )?
    .map(|relation| match relation {
        ConvexRelationShortcut::Separated => ExactBooleanSupport::CertifiedConvexSeparated,
        ConvexRelationShortcut::LeftInsideRight | ConvexRelationShortcut::RightInsideLeft => {
            ExactBooleanSupport::CertifiedConvexContainment
        }
    }) {
        return Ok(Some(certified_preflight(
            operation,
            convex_support,
            Some(graph),
            None,
        )));
    }
    if let Some(convex_support) =
        certified_convex_operation_shortcut_support(left, right, operation)
    {
        return Ok(Some(certified_preflight(
            operation,
            convex_support,
            Some(graph),
            None,
        )));
    }
    if !operation.is_selected_regions()
        && certified_closed_winding_containment_relation_from_graph(graph, left, right)?.is_some()
    {
        return Ok(Some(certified_preflight(
            operation,
            ExactBooleanSupport::CertifiedClosedWindingContainment,
            Some(graph),
            None,
        )));
    }
    if !operation.is_selected_regions()
        && left.facts().mesh.closed_manifold
        && right.facts().mesh.closed_manifold
        && coplanar_volumetric_evidence_from_graph(graph, left, right)?
            .is_some_and(|evidence| coplanar_evidence_is_zero_area_boundary_only(&evidence))
    {
        let Some(boundary_support) = operation.closed_boundary_touching_support() else {
            return Ok(None);
        };
        return Ok(Some(certified_preflight(
            operation,
            boundary_support,
            Some(graph),
            None,
        )));
    }
    Ok(None)
}

/// Preflight a graph-backed exact boolean operation for a specific output
/// validation policy.
///
pub(crate) fn preflight_boolean_exact_request_from_graph_with_retained_attempt(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_attempt: Option<&ExactArrangementBooleanAttempt>,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<ExactBooleanPreflight, ExactMeshError> {
    validate_graph_source_replay(graph, left, right)?;
    let operation = request.operation;
    let validation = request.validation;
    if let Some(support) =
        closed_validation_regularized_solid_support(left, right, operation, validation)
        && !(support == ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid
            && operation == ExactBooleanOperation::Intersection)
    {
        return Ok(certified_preflight(operation, support, Some(graph), None));
    }
    let mut preflight = preflight_boolean_exact_request_from_graph_core(
        graph,
        left,
        right,
        request,
        retained_attempt,
        shortcut_facts,
    )?;
    if operation == ExactBooleanOperation::Union
        && let (report, Some(_)) = adjacent_union_completion_certification_from_graph(
            graph,
            left,
            right,
            operation,
            Some(validation),
        )?
        && matches!(
            report.status,
            ExactAdjacentUnionCompletionStatus::CertifiedFullFace
                | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
        )
    {
        return Ok(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        )?);
    }
    if validation != ExactMeshValidationPolicy::CLOSED
        && !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && matches!(
            preflight.support,
            ExactBooleanSupport::RequiresCertifiedWinding
                | ExactBooleanSupport::RequiresCoplanarVolumetricCells
        )
        && materialize_arrangement_volumetric_split_cell_result_from_graph(
            graph, left, right, operation, validation,
        )?
        .is_some()
    {
        preflight = certified_arrangement_cell_complex_preflight(operation, graph, left, right)?;
    }
    Ok(preflight)
}

fn volumetric_boundary_closure_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactVolumetricBoundaryClosureReport, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(ExactVolumetricBoundaryClosureReport::no_materialized(
            operation,
        ));
    }

    let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        graph,
        left,
        right,
        operation,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )?
    else {
        return Ok(ExactVolumetricBoundaryClosureReport::no_materialized(
            operation,
        ));
    };
    volumetric_boundary_closure_report_from_materialized_with_prevalidated_closure(
        &materialized,
        operation,
        None,
    )
}

fn coplanar_boundary_closure_available_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<bool, ExactMeshError> {
    let report = volumetric_boundary_closure_report_from_graph(graph, left, right, operation)?;
    validate_volumetric_boundary_closure_report(&report)?;
    Ok(matches!(
        report.status,
        ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
    ))
}

#[derive(Clone, Copy, Default)]
struct VolumetricBoundaryClosureEvidence {
    output_triangles: usize,
    boundary_edges: usize,
    boundary_loops: usize,
    boundary_vertices_with_invalid_outgoing_degree: usize,
    boundary_vertices_with_invalid_incoming_degree: usize,
    overused_boundary_edges: usize,
    noncoplanar_boundary_loops: usize,
    repeated_exact_boundary_points: usize,
    self_contact_exact_points: usize,
    self_contact_topological_vertices: usize,
    self_contact_degenerate_cycles: usize,
    self_contact_nondegenerate_cycles: usize,
    coplanar_loop_groups: usize,
}

impl VolumetricBoundaryClosureEvidence {
    fn retain_self_contact(&mut self, self_contact: &BoundaryLoopSelfContactEvidence) {
        self.repeated_exact_boundary_points = self_contact.repeated_exact_point_pairs;
        self.self_contact_exact_points = self_contact.exact_points;
        self.self_contact_topological_vertices = self_contact.topological_vertices;
        self.self_contact_degenerate_cycles = self_contact.degenerate_cycles;
        self.self_contact_nondegenerate_cycles = self_contact.nondegenerate_cycles;
    }

    fn into_report(
        self,
        operation: ExactBooleanOperation,
        status: ExactVolumetricBoundaryClosureStatus,
    ) -> ExactVolumetricBoundaryClosureReport {
        ExactVolumetricBoundaryClosureReport {
            operation,
            status,
            output_triangles: self.output_triangles,
            boundary_edges: self.boundary_edges,
            boundary_loops: self.boundary_loops,
            boundary_vertices_with_invalid_outgoing_degree: self
                .boundary_vertices_with_invalid_outgoing_degree,
            boundary_vertices_with_invalid_incoming_degree: self
                .boundary_vertices_with_invalid_incoming_degree,
            overused_boundary_edges: self.overused_boundary_edges,
            noncoplanar_boundary_loops: self.noncoplanar_boundary_loops,
            repeated_exact_boundary_points: self.repeated_exact_boundary_points,
            self_contact_exact_points: self.self_contact_exact_points,
            self_contact_topological_vertices: self.self_contact_topological_vertices,
            self_contact_degenerate_cycles: self.self_contact_degenerate_cycles,
            self_contact_nondegenerate_cycles: self.self_contact_nondegenerate_cycles,
            coplanar_loop_groups: self.coplanar_loop_groups,
        }
    }
}

impl ExactVolumetricBoundaryClosureReport {
    pub(crate) fn no_materialized(operation: ExactBooleanOperation) -> Self {
        VolumetricBoundaryClosureEvidence::default().into_report(
            operation,
            ExactVolumetricBoundaryClosureStatus::NoMaterializedBoundaryOutput,
        )
    }
}

fn volumetric_boundary_closure_report_from_materialized_with_prevalidated_closure(
    materialized: &MaterializedVolumetricWindingRegionPlan,
    operation: ExactBooleanOperation,
    prevalidated_coplanar_closure_available: Option<bool>,
) -> Result<ExactVolumetricBoundaryClosureReport, ExactMeshError> {
    materialized
        .mesh
        .validate_retained_state()
        .map_err(|error| {
            boolean_validation_error(
                ExactMeshBlockerKind::ExactConstructionFailure,
                "volumetric boundary closure source mesh validation failed",
                error,
            )
        })?;
    let retained_output = VolumetricBoundaryClosureEvidence {
        output_triangles: materialized.mesh.triangles().len(),
        ..VolumetricBoundaryClosureEvidence::default()
    };
    let boundary_edges = materialized.mesh.facts().mesh.boundary_edges;
    if materialized.mesh.facts().mesh.closed_manifold || boundary_edges == 0 {
        return Ok(retained_output.into_report(
            operation,
            ExactVolumetricBoundaryClosureStatus::AlreadyClosed,
        ));
    }
    let mut retained_boundary = VolumetricBoundaryClosureEvidence {
        boundary_edges,
        ..retained_output
    };
    let boundary_loops = match directed_boundary_loops(materialized.mesh.view()) {
        Ok(boundary_loops) => boundary_loops,
        Err(boundary_topology) => {
            retained_boundary.boundary_vertices_with_invalid_outgoing_degree =
                boundary_topology.invalid_outgoing_degree_vertices;
            retained_boundary.boundary_vertices_with_invalid_incoming_degree =
                boundary_topology.invalid_incoming_degree_vertices;
            retained_boundary.overused_boundary_edges = boundary_topology.overused_edges;
            return Ok(retained_boundary.into_report(
                operation,
                ExactVolumetricBoundaryClosureStatus::BoundaryTopologyNotLoop,
            ));
        }
    };
    retained_boundary.boundary_loops = boundary_loops.len();
    let output_vertices = materialized.mesh.view().vertices();
    let boundary_points = boundary_loops
        .iter()
        .map(|boundary_loop| {
            required_cloned_indexed_points(
                output_vertices,
                boundary_loop.iter().copied(),
                "volumetric boundary closure report referenced a missing output vertex",
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let boundary_points = boundary_points
        .into_iter()
        .map(|boundary| {
            split_cyclic_self_contact_cycles(boundary, &|left, right| {
                point3_exact_equal(left, right).ok_or(ExactArrangementBlocker::UndecidableOrdering)
            })
        })
        .collect::<Result<Vec<_>, _>>()
        .map(|split| split.into_iter().flatten().collect::<Vec<_>>())
        .map_err(|blocker| {
            boolean_validation_error(
                ExactMeshBlockerKind::ExactConstructionFailure,
                "volumetric boundary closure self-contact canonicalization failed",
                blocker,
            )
        })?;
    let mut self_contact = BoundaryLoopSelfContactEvidence::default();
    for boundary in &boundary_points {
        match boundary_loop_self_contact_evidence(boundary) {
            Ok(evidence) => {
                self_contact.repeated_exact_point_pairs += evidence.repeated_exact_point_pairs;
                self_contact.exact_points += evidence.exact_points;
                self_contact.topological_vertices += evidence.topological_vertices;
                self_contact.degenerate_cycles += evidence.degenerate_cycles;
                self_contact.nondegenerate_cycles += evidence.nondegenerate_cycles;
            }
            Err(blocker) => {
                retained_boundary.retain_self_contact(&self_contact);
                return Ok(retained_boundary.into_report(
                    operation,
                    ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(blocker),
                ));
            }
        }
    }
    retained_boundary.retain_self_contact(&self_contact);
    if self_contact.repeated_exact_point_pairs != 0 {
        return Ok(retained_boundary.into_report(
            operation,
            ExactVolumetricBoundaryClosureStatus::BoundaryLoopExactSelfContact,
        ));
    }
    let mut noncoplanar_boundary_loops = 0;
    for boundary in &boundary_points {
        match exact_loop_is_coplanar(boundary) {
            Ok(true) => {}
            Ok(false) => noncoplanar_boundary_loops += 1,
            Err(blocker) => {
                retained_boundary.noncoplanar_boundary_loops = noncoplanar_boundary_loops;
                return Ok(retained_boundary.into_report(
                    operation,
                    ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(blocker),
                ));
            }
        }
    }
    retained_boundary.noncoplanar_boundary_loops = noncoplanar_boundary_loops;
    if noncoplanar_boundary_loops != 0 {
        return Ok(retained_boundary.into_report(
            operation,
            ExactVolumetricBoundaryClosureStatus::NonCoplanarBoundaryClosureRequired,
        ));
    }
    match group_exact_coplanar_loops(boundary_points) {
        Ok(groups) => {
            retained_boundary.coplanar_loop_groups = groups.len();
            let coplanar_closure_available = match prevalidated_coplanar_closure_available {
                Some(available) => available,
                None => optional_coplanar_boundary_closure(
                    &materialized.mesh,
                    "exact volumetric boundary closure certification cap",
                    ExactMeshValidationPolicy::CLOSED,
                )?
                .is_some(),
            };
            if coplanar_closure_available {
                Ok(retained_boundary.into_report(
                    operation,
                    ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
                ))
            } else {
                Ok(retained_boundary.into_report(
                    operation,
                    ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(
                        ExactArrangementBlocker::NonManifoldCellComplex,
                    ),
                ))
            }
        }
        Err(blocker) => Ok(retained_boundary.into_report(
            operation,
            ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(blocker),
        )),
    }
}

fn certified_preflight(
    operation: ExactBooleanOperation,
    support: ExactBooleanSupport,
    graph: Option<&super::graph::ExactIntersectionGraph>,
    coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
) -> ExactBooleanPreflight {
    let graph_counts = graph.map_or_else(RetainedGraphCounts::empty, retained_graph_counts);
    graph_counts.into_preflight(
        operation,
        support,
        0,
        Vec::new(),
        None,
        None,
        coplanar_volumetric_evidence,
    )
}

fn certified_arrangement_cell_complex_preflight(
    operation: ExactBooleanOperation,
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactBooleanPreflight, ExactMeshError> {
    Ok(certified_preflight(
        operation,
        ExactBooleanSupport::CertifiedArrangementCellComplex,
        Some(graph),
        coplanar_volumetric_evidence_from_graph(graph, left, right)?
            .filter(coplanar_evidence_certifies_arrangement_cell_complex),
    ))
}

fn region_plan_preflight_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    support: ExactBooleanSupport,
    blocker: Option<ExactBooleanBlocker>,
    coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
) -> Result<ExactBooleanPreflight, ExactMeshError> {
    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
    let graph_counts = retained_graph_counts(graph);
    Ok(graph_counts.into_preflight(
        operation,
        support,
        region_plan.regions.len(),
        region_classifications,
        blocker,
        None,
        coplanar_volumetric_evidence,
    ))
}

#[derive(Clone, Copy)]
struct RetainedGraphCounts {
    graph_had_unknowns: bool,
    retained_face_pairs: usize,
    retained_events: usize,
}

impl RetainedGraphCounts {
    const fn empty() -> Self {
        Self {
            graph_had_unknowns: false,
            retained_face_pairs: 0,
            retained_events: 0,
        }
    }

    fn from_graph(graph: &super::graph::ExactIntersectionGraph) -> Self {
        Self {
            graph_had_unknowns: graph.has_unknowns(),
            retained_face_pairs: graph.face_pairs.len(),
            retained_events: graph.event_count(),
        }
    }

    const fn from_boundary_touching_report(report: &ExactBoundaryTouchingReport) -> Self {
        Self {
            graph_had_unknowns: report.graph_had_unknowns,
            retained_face_pairs: report.retained_face_pairs,
            retained_events: report.retained_events,
        }
    }

    const fn from_planar_arrangement_report(report: &ExactPlanarArrangementReport) -> Self {
        Self {
            graph_had_unknowns: report.graph_had_unknowns,
            retained_face_pairs: report.retained_face_pairs,
            retained_events: report.retained_events,
        }
    }

    const fn from_winding_evidence_report(report: &ExactWindingEvidenceReport) -> Self {
        Self {
            graph_had_unknowns: report.graph_had_unknowns,
            retained_face_pairs: report.retained_face_pairs,
            retained_events: report.retained_events,
        }
    }

    const fn with_retained_face_pairs(self, retained_face_pairs: usize) -> Self {
        Self {
            retained_face_pairs,
            ..self
        }
    }

    fn into_preflight(
        self,
        operation: ExactBooleanOperation,
        support: ExactBooleanSupport,
        region_count: usize,
        region_classifications: Vec<FaceRegionPlaneClassification>,
        blocker: Option<ExactBooleanBlocker>,
        coplanar_arrangement_evidence: Option<super::graph::CoplanarArrangementEvidence>,
        coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
    ) -> ExactBooleanPreflight {
        ExactBooleanPreflight {
            operation,
            support,
            graph_had_unknowns: self.graph_had_unknowns,
            retained_face_pairs: self.retained_face_pairs,
            retained_events: self.retained_events,
            region_count,
            region_classifications,
            blocker,
            coplanar_arrangement_evidence,
            coplanar_volumetric_evidence,
        }
    }

    fn into_adjacent_union_completion_report(
        self,
        operation: ExactBooleanOperation,
        status: ExactAdjacentUnionCompletionStatus,
        left_closed: bool,
        right_closed: bool,
        axis_aligned_box_pair: bool,
        stronger_kernel_available: bool,
        counts: ExactBooleanBlocker,
        full_face_shared_faces: usize,
        full_face_shared_patches: usize,
        contained_containing_side: Option<MeshSide>,
        contained_faces: usize,
        containing_faces: usize,
    ) -> ExactAdjacentUnionCompletionReport {
        let blocker_kind = match status {
            ExactAdjacentUnionCompletionStatus::GraphUnresolved => {
                ExactBooleanBlockerKind::Refinement
            }
            ExactAdjacentUnionCompletionStatus::CertifiedFullFace
            | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace => {
                ExactBooleanBlockerKind::BoundaryOnlyContact
            }
            _ => counts.inferred_kind(),
        };
        ExactAdjacentUnionCompletionReport {
            operation,
            status,
            left_closed,
            right_closed,
            axis_aligned_box_pair,
            stronger_kernel_available,
            graph_had_unknowns: self.graph_had_unknowns,
            retained_face_pairs: self.retained_face_pairs,
            retained_events: self.retained_events,
            blocker: counts.into_blocker(blocker_kind),
            full_face_shared_faces,
            full_face_shared_patches,
            contained_containing_side,
            contained_faces,
            containing_faces,
        }
    }

    fn into_boundary_touching_report(
        self,
        status: ExactBoundaryTouchingStatus,
        blocker: ExactBooleanBlocker,
    ) -> ExactBoundaryTouchingReport {
        ExactBoundaryTouchingReport {
            status,
            graph_had_unknowns: self.graph_had_unknowns,
            retained_face_pairs: self.retained_face_pairs,
            retained_events: self.retained_events,
            blocker,
        }
    }

    fn into_planar_arrangement_report(
        self,
        operation: ExactBooleanOperation,
        status: ExactPlanarArrangementStatus,
        counts: ExactBooleanBlocker,
        coplanar_arrangement_evidence: Option<super::graph::CoplanarArrangementEvidence>,
    ) -> ExactPlanarArrangementReport {
        let blocker_kind = match status {
            ExactPlanarArrangementStatus::GraphUnknowns => ExactBooleanBlockerKind::Refinement,
            ExactPlanarArrangementStatus::BoundaryOnlyContactRequired => {
                ExactBooleanBlockerKind::BoundaryOnlyContact
            }
            ExactPlanarArrangementStatus::Required => ExactBooleanBlockerKind::PlanarArrangement,
            ExactPlanarArrangementStatus::NotNamedOperation
            | ExactPlanarArrangementStatus::AlreadyMaterialized
            | ExactPlanarArrangementStatus::NoPositiveOverlap => counts.inferred_kind(),
        };
        ExactPlanarArrangementReport {
            operation,
            status,
            graph_had_unknowns: self.graph_had_unknowns,
            retained_face_pairs: self.retained_face_pairs,
            retained_events: self.retained_events,
            blocker: counts.into_blocker(blocker_kind),
            coplanar_arrangement_evidence,
        }
    }

    fn into_open_surface_disjoint_report(
        self,
        status: ExactOpenSurfaceDisjointStatus,
        left_open_surface: bool,
        right_open_surface: bool,
        blocker: ExactBooleanBlocker,
    ) -> ExactOpenSurfaceDisjointReport {
        ExactOpenSurfaceDisjointReport {
            status,
            left_open_surface,
            right_open_surface,
            graph_had_unknowns: self.graph_had_unknowns,
            retained_face_pairs: self.retained_face_pairs,
            retained_events: self.retained_events,
            blocker,
        }
    }

    #[cfg(test)]
    fn into_refinement_report(
        self,
        operation: ExactBooleanOperation,
        status: evidence::ExactRefinementStatus,
        blocker: Option<ExactBooleanBlocker>,
    ) -> evidence::ExactRefinementReport {
        evidence::ExactRefinementReport {
            operation,
            status,
            graph_had_unknowns: self.graph_had_unknowns,
            retained_face_pairs: self.retained_face_pairs,
            retained_events: self.retained_events,
            blocker,
        }
    }

    fn into_winding_evidence_report(
        self,
        operation: ExactBooleanOperation,
        status: ExactWindingEvidenceStatus,
        region_count: usize,
        region_classifications: Vec<FaceRegionPlaneClassification>,
        blocker: ExactBooleanBlocker,
        coplanar_arrangement_evidence: Option<super::graph::CoplanarArrangementEvidence>,
        coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
    ) -> ExactWindingEvidenceReport {
        ExactWindingEvidenceReport {
            operation,
            status,
            graph_had_unknowns: self.graph_had_unknowns,
            retained_face_pairs: self.retained_face_pairs,
            retained_events: self.retained_events,
            region_count,
            region_classifications,
            blocker,
            coplanar_arrangement_evidence,
            coplanar_volumetric_evidence,
        }
    }
}

fn retained_graph_counts(graph: &super::graph::ExactIntersectionGraph) -> RetainedGraphCounts {
    RetainedGraphCounts::from_graph(graph)
}

fn certified_closed_boundary_only_contact_preflight(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ExactBooleanPreflight>, ExactMeshError> {
    let Some(boundary_support) = operation.closed_boundary_touching_support() else {
        return Ok(None);
    };
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(None);
    }
    let Some(evidence) = coplanar_volumetric_evidence_from_graph(graph, left, right)?
        .filter(coplanar_evidence_is_boundary_only_contact)
    else {
        return Ok(None);
    };
    if evidence.positive_area_coplanar_overlapping_pairs != 0 {
        return Ok(Some(certified_preflight(
            operation,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            Some(graph),
            Some(evidence),
        )));
    }
    let consumed_evidence = if operation == ExactBooleanOperation::Union {
        None
    } else {
        coplanar_volumetric_evidence_from_graph(graph, left, right)?
            .filter(coplanar_evidence_is_positive_area_boundary_only)
    };
    Ok(Some(certified_preflight(
        operation,
        boundary_support,
        Some(graph),
        consumed_evidence,
    )))
}

fn coplanar_volumetric_evidence_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<CoplanarVolumetricCellEvidenceReport>, ExactMeshError> {
    let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    if counts.coplanar_overlapping_pairs == 0 && counts.coplanar_touching_pairs == 0 {
        return Ok(None);
    }
    validate_graph_source_replay(graph, left, right)?;
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right)?;
    evidence.validate().map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "exact coplanar volumetric evidence validation failed",
            error,
        )
    })?;
    Ok(Some(evidence))
}

fn coplanar_evidence_requires_volumetric_cells(
    evidence: &CoplanarVolumetricCellEvidenceReport,
) -> bool {
    matches!(
        evidence.obstacle,
        CoplanarVolumetricCellObstacle::NeedsCoplanarVolumetricCells
            | CoplanarVolumetricCellObstacle::MixedCoplanarAndCrossingCells
    )
}

fn coplanar_evidence_is_boundary_only_contact(
    evidence: &CoplanarVolumetricCellEvidenceReport,
) -> bool {
    matches!(
        evidence.obstacle,
        CoplanarVolumetricCellObstacle::BoundaryOnlyContact
    )
}

fn coplanar_evidence_is_positive_area_boundary_only(
    evidence: &CoplanarVolumetricCellEvidenceReport,
) -> bool {
    coplanar_evidence_is_boundary_only_contact(evidence)
        && evidence.positive_area_coplanar_overlapping_pairs != 0
}

fn coplanar_evidence_is_zero_area_boundary_only(
    evidence: &CoplanarVolumetricCellEvidenceReport,
) -> bool {
    coplanar_evidence_is_boundary_only_contact(evidence)
        && evidence.positive_area_coplanar_overlapping_pairs == 0
}

fn coplanar_evidence_certifies_arrangement_cell_complex(
    evidence: &CoplanarVolumetricCellEvidenceReport,
) -> bool {
    coplanar_evidence_requires_volumetric_cells(evidence)
        || coplanar_evidence_is_positive_area_boundary_only(evidence)
}

fn certified_arrangement_cell_complex_preflight_if_materialized(
    operation: ExactBooleanOperation,
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ExactBooleanPreflight>, ExactMeshError> {
    let orthogonal_cell_materializes =
        orthogonal_solid_cell_materializes_for_preflight(left, right, operation)?;
    let arrangement_materializes = if orthogonal_cell_materializes {
        false
    } else {
        let validation_policies: &[ExactMeshValidationPolicy] =
            if left.facts().mesh.closed_manifold && right.facts().mesh.closed_manifold {
                &[ExactMeshValidationPolicy::CLOSED]
            } else {
                &[
                    ExactMeshValidationPolicy::CLOSED,
                    ExactMeshValidationPolicy::ALLOW_BOUNDARY,
                ]
            };
        let mut materializes = false;
        'arrangement_probe: for regularize_sheet_complex in [false, true] {
            for &validation in validation_policies {
                if certified_arrangement_cell_complex_result_from_graph(
                    graph,
                    left,
                    right,
                    ExactBooleanRequest::new(operation, validation),
                    regularize_sheet_complex,
                )?
                .is_some()
                {
                    materializes = true;
                    break 'arrangement_probe;
                }
            }
        }
        materializes
    };
    if orthogonal_cell_materializes
        || arrangement_materializes
        || boolean_coplanar_mesh_overlay_optional(
            left,
            right,
            operation,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )?
        .is_some()
    {
        Ok(Some(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        )?))
    } else {
        Ok(None)
    }
}

fn orthogonal_solid_cell_materializes_for_preflight(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<bool, ExactMeshError> {
    let Some(solid_operation) = operation.axis_aligned_orthogonal_solid_operation() else {
        return Ok(false);
    };
    let validation_policies: &[ExactMeshValidationPolicy] =
        if left.facts().mesh.closed_manifold && right.facts().mesh.closed_manifold {
            &[ExactMeshValidationPolicy::CLOSED]
        } else {
            &[
                ExactMeshValidationPolicy::CLOSED,
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
            ]
        };
    for &validation in validation_policies {
        if materialize_axis_aligned_orthogonal_solid_cell_output(
            left,
            right,
            solid_operation,
            "exact arrangement orthogonal solid cell preflight probe",
            validation,
        )?
        .is_some()
        {
            return Ok(true);
        }
    }
    Ok(false)
}

fn certified_arrangement_cell_complex_preflight_from_retained_attempt(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    attempt: &ExactArrangementBooleanAttempt,
) -> Result<Option<ExactBooleanPreflight>, ExactMeshError> {
    if materialize_retained_arrangement_cell_complex_attempt(left, right, request, attempt)?
        .is_some()
    {
        Ok(Some(certified_arrangement_cell_complex_preflight(
            request.operation,
            graph,
            left,
            right,
        )?))
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
    retained_request: Option<ExactBooleanRequest>,
    retained_attempt: Option<&ExactArrangementBooleanAttempt>,
) -> Result<Option<ExactBooleanPreflight>, ExactMeshError> {
    if let Some(preflight) = cache.as_ref() {
        return Ok(preflight.clone());
    }
    let retained_preflight = retained_request
        .zip(retained_attempt)
        .filter(|(request, _)| request.operation == operation)
        .map(|(request, attempt)| {
            certified_arrangement_cell_complex_preflight_from_retained_attempt(
                graph, left, right, request, attempt,
            )
        })
        .transpose()?
        .flatten();
    let preflight = match retained_preflight {
        Some(preflight) => Some(preflight),
        None => certified_arrangement_cell_complex_preflight_if_materialized(
            operation, graph, left, right,
        )?,
    };
    *cache = Some(preflight.clone());
    Ok(preflight)
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

fn graph_requires_boundary_only_contact(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactMeshError> {
    if !graph.face_pairs.is_empty()
        && graph
            .face_pairs
            .iter()
            .all(|pair| pair.relation == MeshFacePairRelation::CoplanarTouching)
    {
        return Ok(true);
    }
    if !graph_has_only_boundary_contact_pairs(graph, left, right) {
        return Ok(false);
    }
    let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    if counts.coplanar_overlapping_pairs == 0
        && (mesh_is_open_surface(left) || mesh_is_open_surface(right))
    {
        return Ok(true);
    }
    if axis_aligned_orthogonal_solid_cell_selected_count(
        left,
        right,
        AxisAlignedOrthogonalSolidOperation::Intersection,
    ) == Some(0)
        || affine_orthogonal_solid_cell_selected_count(
            left,
            right,
            AffineOrthogonalSolidOperation::Intersection,
        ) == Some(0)
    {
        return Ok(true);
    }
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(false);
    }

    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    validate_closed_winding_report(&left_in_right, None)?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    validate_closed_winding_report(&right_in_left, None)?;

    let Some(left_touches_right_boundary) = left_in_right.boundary_or_outside_touch() else {
        return Ok(false);
    };
    let Some(right_touches_left_boundary) = right_in_left.boundary_or_outside_touch() else {
        return Ok(false);
    };
    Ok(left_touches_right_boundary || right_touches_left_boundary)
}

fn graph_has_only_boundary_contact_pairs(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    !graph.face_pairs.is_empty()
        && graph.face_pairs.iter().all(|pair| match pair.relation {
            MeshFacePairRelation::CoplanarTouching | MeshFacePairRelation::CoplanarOverlapping => {
                true
            }
            MeshFacePairRelation::Candidate => pair.events.iter().all(|event| {
                // Positive-area coplanar contact between closed solids also
                // retains adjacent non-coplanar face pairs where an endpoint
                // or coplanar source edge lies on the opposite plane. Those
                // are still boundary facts, not volume overlap.
                match event {
                    IntersectionEvent::SegmentPlane {
                        relation:
                            SegmentPlaneRelation::Disjoint
                            | SegmentPlaneRelation::Coplanar
                            | SegmentPlaneRelation::EndpointOnPlane,
                        ..
                    } => true,
                    IntersectionEvent::SegmentPlane {
                        relation: SegmentPlaneRelation::ProperCrossing,
                        plane_side,
                        plane_face,
                        point: Some(point),
                        ..
                    } => {
                        let Ok(face) = plane_side.mesh(left, right).view().face(*plane_face) else {
                            return false;
                        };
                        let Ok([a, b, c]) = face.vertices() else {
                            return false;
                        };
                        let triangle = [a.clone(), b.clone(), c.clone()];
                        let Some(projection) = choose_nonzero_projected_polygon_area(&triangle)
                        else {
                            return false;
                        };
                        classify_point_triangle(
                            &project_point3(&triangle[0], projection),
                            &project_point3(&triangle[1], projection),
                            &project_point3(&triangle[2], projection),
                            &project_point3(point, projection),
                        )
                        .value()
                            == Some(TriangleLocation::Outside)
                    }
                    IntersectionEvent::SegmentPlane { .. } => false,
                    IntersectionEvent::CoplanarEdge { relation, .. } => {
                        *relation != SegmentIntersection::Disjoint
                    }
                    IntersectionEvent::CoplanarVertex { location, .. } => matches!(
                        location,
                        TriangleLocation::Inside
                            | TriangleLocation::OnEdge
                            | TriangleLocation::OnVertex
                    ),
                    IntersectionEvent::Unknown => false,
                }
            }),
            MeshFacePairRelation::PlaneSeparated | MeshFacePairRelation::Unknown => false,
        })
}

fn closed_winding_vertex_relations_from_empty_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<(ClosedMeshWindingMeshRelation, ClosedMeshWindingMeshRelation)>, ExactMeshError>
{
    if !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
        || graph.has_unknowns()
        || !graph.face_pairs.is_empty()
    {
        return Ok(None);
    }
    let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    if counts.construction_failed_events != 0 {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    validate_closed_winding_report(&left_in_right, Some((left, right)))?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    validate_closed_winding_report(&right_in_left, Some((right, left)))?;
    Ok(Some((left_in_right.relation, right_in_left.relation)))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ClosedWindingContainmentRelation {
    LeftInsideRight,
    RightInsideLeft,
}

fn certified_closed_winding_containment_relation_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ClosedWindingContainmentRelation>, ExactMeshError> {
    let Some((left_in_right, right_in_left)) =
        closed_winding_vertex_relations_from_empty_graph(graph, left, right)?
    else {
        return Ok(None);
    };
    match (left_in_right, right_in_left) {
        (ClosedMeshWindingMeshRelation::StrictlyInside, _) => {
            Ok(Some(ClosedWindingContainmentRelation::LeftInsideRight))
        }
        (_, ClosedMeshWindingMeshRelation::StrictlyInside) => {
            Ok(Some(ClosedWindingContainmentRelation::RightInsideLeft))
        }
        _ => Ok(None),
    }
}

fn boolean_closed_winding_containment_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let Some(containment) =
        certified_closed_winding_containment_relation_from_graph(graph, left, right)?
    else {
        return Ok(None);
    };

    let mesh = match (operation, containment) {
        (ExactBooleanOperation::Union, ClosedWindingContainmentRelation::LeftInsideRight) => {
            copy_mesh(
                right,
                "exact closed-winding containment union keeps right",
                validation,
            )?
        }
        (ExactBooleanOperation::Union, ClosedWindingContainmentRelation::RightInsideLeft) => {
            copy_mesh(
                left,
                "exact closed-winding containment union keeps left",
                validation,
            )?
        }
        (
            ExactBooleanOperation::Intersection,
            ClosedWindingContainmentRelation::LeftInsideRight,
        ) => copy_mesh(
            left,
            "exact closed-winding containment intersection keeps left",
            validation,
        )?,
        (
            ExactBooleanOperation::Intersection,
            ClosedWindingContainmentRelation::RightInsideLeft,
        ) => copy_mesh(
            right,
            "exact closed-winding containment intersection keeps right",
            validation,
        )?,
        (ExactBooleanOperation::Difference, ClosedWindingContainmentRelation::LeftInsideRight) => {
            empty_mesh(
                "empty exact closed-winding containment difference",
                validation,
            )?
        }
        (ExactBooleanOperation::Difference, ClosedWindingContainmentRelation::RightInsideLeft) => {
            concatenate_meshes_with_options(
                left,
                right,
                true,
                "exact closed-winding containment difference with cavity",
                validation,
            )?
        }
        (ExactBooleanOperation::SelectedRegions(_), _) => return Ok(None),
    };
    Ok(Some(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ClosedWindingContainment,
    )))
}

fn boolean_closed_winding_separated_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let Some((left_in_right, right_in_left)) =
        closed_winding_vertex_relations_from_empty_graph(graph, left, right)?
    else {
        return Ok(None);
    };
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || left_in_right != ClosedMeshWindingMeshRelation::Outside
        || right_in_left != ClosedMeshWindingMeshRelation::Outside
    {
        return Ok(None);
    }

    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact closed-winding separated union",
            validation,
        )?,
        ExactBooleanOperation::Intersection => empty_mesh(
            "empty exact closed-winding separated intersection",
            validation,
        )?,
        ExactBooleanOperation::Difference => copy_mesh(
            left,
            "exact closed-winding separated difference keeps left",
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    Ok(Some(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ClosedWindingSeparated,
    )))
}

fn request_replayable_result(
    result: Option<ExactBooleanResult>,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
) -> Option<ExactBooleanResult> {
    let result = result?;
    let retained_arrangement_attempt = result
        .kind
        .arrangement_cell_complex_operation()
        .is_some()
        .then_some(retained_arrangement_attempt)
        .flatten();
    result
        .validate_request_against_sources_with_retained_attempt(
            left,
            right,
            request,
            retained_arrangement_attempt,
        )
        .is_ok()
        .then_some(result)
}

fn materialize_arrangement_lower_dimensional_intersection_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let operation = request.operation;
    let validation = request.validation;
    if validation != ExactMeshValidationPolicy::CLOSED
        || operation != ExactBooleanOperation::Intersection
        || closed_regularized_operand_kind(left)
            != Some(ClosedRegularizedOperandKind::LowerDimensional)
        || closed_regularized_operand_kind(right)
            != Some(ClosedRegularizedOperandKind::LowerDimensional)
    {
        return Ok(None);
    }
    let evidence = winding_evidence_report_for_request_from_graph_and_attempt(
        graph,
        left,
        right,
        request,
        retained_arrangement_attempt,
        shortcut_facts,
    )?;
    evidence.validate().map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "exact arrangement lower-dimensional evidence validation failed",
            error,
        )
    })?;
    if !matches!(
        evidence.status,
        ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized
            | ExactWindingEvidenceStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized
    ) {
        return Ok(None);
    }
    let mesh = empty_mesh(
        "empty exact arrangement cell-complex lower-dimensional intersection",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    )))
}

/// Materialize an exact boolean operation under an output validation contract.
///
/// This path is still strict about general winding. Boundary-only contact is
/// retained as certified evidence and returned as a blocker unless a complete
/// kernel materializer can prove a triangle-mesh result. Projection policy for
/// lower-dimensional contact belongs above `hypermesh`.
pub(crate) fn materialize_boolean_operation(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    retained_graph: Option<&ExactIntersectionGraph>,
    prepared_pair: Option<&PreparedMeshPair<'_, '_>>,
) -> Result<ExactBooleanResult, ExactMeshError> {
    left.validate_retained_bounds_certificate()?;
    right.validate_retained_bounds_certificate()?;
    let mut owned_graph = None;
    let mut prepared_graph = None;
    if let ExactBooleanOperation::SelectedRegions(selection) = operation {
        let graph = graph_for_certified_materialization(
            retained_graph,
            &mut owned_graph,
            Some(&mut prepared_graph),
            prepared_pair,
            left,
            right,
        )?;
        return replay_selected_region_boolean_result_from_graph(
            graph, left, right, selection, validation,
        );
    }
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return boolean_empty_operand(left, right, operation, validation);
    }
    if meshes_are_certified_bounds_disjoint(left, right) {
        return boolean_disjoint_meshes(left, right, operation, validation);
    }
    let mut shortcut_facts = None;
    let request = ExactBooleanRequest::new(operation, validation);
    if validation == ExactMeshValidationPolicy::CLOSED
        && operation == ExactBooleanOperation::Intersection
    {
        let graph = graph_for_certified_materialization(
            retained_graph,
            &mut owned_graph,
            Some(&mut prepared_graph),
            prepared_pair,
            left,
            right,
        )?;
        let shortcut_facts =
            cached_arrangement_shortcut_facts(&mut shortcut_facts, prepared_pair, left, right)?;
        if let Some(result) = materialize_arrangement_lower_dimensional_intersection_from_graph(
            graph,
            left,
            right,
            request,
            None,
            &shortcut_facts,
        )? {
            return Ok(result);
        }
    }
    if let Some(result) =
        boolean_closed_regularized_lower_dimensional_optional(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
        && evidence::identical_mesh_report_from_sources(left, right).status
            == ExactIdenticalMeshStatus::Certified
    {
        return boolean_identical_meshes(left, operation, validation);
    }
    if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
        && evidence::same_surface_report_from_sources(left, right).status
            == ExactSameSurfaceStatus::Certified
    {
        return boolean_same_surface_meshes(left, operation, validation);
    }
    let ready_graph = if let Some(graph) = retained_graph {
        validate_graph_source_replay(graph, left, right)?;
        Some(graph)
    } else {
        owned_graph.as_ref()
    };
    if let Some(graph) = ready_graph {
        let shortcut_facts =
            cached_arrangement_shortcut_facts(&mut shortcut_facts, prepared_pair, left, right)?;
        return materialize_boolean_operation_from_ready_graph(
            graph,
            left,
            right,
            request,
            shortcut_facts,
        );
    }
    if let Some(pair) = prepared_pair {
        match pair.current_arrangement_for_reuse() {
            Ok(arrangement) => {
                let graph = graph_for_certified_materialization(
                    retained_graph,
                    &mut owned_graph,
                    Some(&mut prepared_graph),
                    prepared_pair,
                    left,
                    right,
                )?;
                let shortcut_facts = cached_arrangement_shortcut_facts(
                    &mut shortcut_facts,
                    prepared_pair,
                    left,
                    right,
                )?;
                let result =
                    materialize_certified_arrangement_cell_complex_support_with_arrangement(
                        left,
                        right,
                        request,
                        Some(graph),
                        Some(arrangement.as_ref()),
                        None,
                        &shortcut_facts,
                    )?;
                if let Some(result) = result {
                    return Ok(result);
                }
            }
            Err(error)
                if error
                    .has_only_blocker_kinds(&[ExactMeshBlockerKind::MissingRequiredEvidence]) => {}
            Err(error) => return Err(error),
        }
    }
    if let Some(pair) = prepared_pair {
        let graph = graph_for_certified_materialization(
            retained_graph,
            &mut owned_graph,
            Some(&mut prepared_graph),
            Some(pair),
            left,
            right,
        )?;
        let shortcut_facts =
            cached_arrangement_shortcut_facts(&mut shortcut_facts, prepared_pair, left, right)?;
        return materialize_boolean_operation_from_ready_graph(
            graph,
            left,
            right,
            request,
            shortcut_facts,
        );
    }

    match build_validated_intersection_graph(left, right) {
        Ok(graph) => {
            let shortcut_facts =
                cached_arrangement_shortcut_facts(&mut shortcut_facts, prepared_pair, left, right)?;
            materialize_boolean_operation_from_ready_graph(
                &graph,
                left,
                right,
                request,
                shortcut_facts,
            )
        }
        Err(error) => {
            if let Some(result) =
                boolean_arrangement_cell_complex_recovery(left, right, operation, validation)?
            {
                return Ok(result);
            }
            if let Some(result) =
                boolean_convex_meshes_optional(left, right, operation, validation)?
            {
                return Ok(result);
            }
            Err(error)
        }
    }
}

fn cached_arrangement_shortcut_facts<'facts>(
    cached: &'facts mut Option<ExactArrangementCellComplexShortcutFacts>,
    prepared_pair: Option<&PreparedMeshPair<'_, '_>>,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<&'facts ExactArrangementCellComplexShortcutFacts, ExactMeshError> {
    if cached.is_none() {
        *cached = Some(match prepared_pair {
            Some(pair) => pair.prepare_arrangement_cell_complex_shortcut_facts()?,
            None => ExactArrangementCellComplexShortcutFacts::from_sources(left, right),
        });
    }
    cached.as_ref().ok_or_else(|| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::MissingRequiredEvidence,
            "arrangement shortcut facts were not retained after initialization",
        ))
    })
}

fn materialize_boolean_operation_from_ready_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<ExactBooleanResult, ExactMeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let prefer_boundary_or_no_volume = matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    );
    if let Some(result) = boolean_closed_winding_separated_meshes_from_graph(
        graph, left, right, operation, validation,
    )? {
        return Ok(result);
    }
    if shortcut_facts.materializes_operation(operation)
        && let Some(result) =
            boolean_arrangement_cell_complex_recovery(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if let Some(result) =
        certified_arrangement_cell_complex_result_from_graph(graph, left, right, request, true)?
    {
        return Ok(result);
    }
    if let Some(result) = boolean_closed_winding_containment_meshes_from_graph(
        graph, left, right, operation, validation,
    )? {
        return Ok(result);
    }
    if prefer_boundary_or_no_volume
        && let Some((result, _evidence)) =
            materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
                graph, left, right, operation, validation,
            )?
    {
        return Ok(result);
    }
    if prefer_boundary_or_no_volume
        && let Some((result, _evidence)) =
            materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
                graph, left, right, operation, validation,
            )?
    {
        return Ok(result);
    }
    if let Some(result) = boolean_convex_relation_meshes_optional_from_graph(
        graph, left, right, operation, validation,
    )? {
        return Ok(result);
    }
    if let Some(result) = boolean_convex_meshes_optional(left, right, operation, validation)? {
        return Ok(result);
    }

    if operation == ExactBooleanOperation::Union
        && let Some((result, _report)) = materialize_adjacent_union_completion_from_graph(
            graph, left, right, operation, validation,
        )?
    {
        return Ok(result);
    }
    if !prefer_boundary_or_no_volume
        && let Some((result, _evidence)) =
            materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
                graph, left, right, operation, validation,
            )?
    {
        return Ok(result);
    }
    if !prefer_boundary_or_no_volume
        && let Some((result, _evidence)) =
            materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
                graph, left, right, operation, validation,
            )?
    {
        return Ok(result);
    }
    if let Some(result) = materialize_arrangement_volumetric_split_cell_result_from_graph(
        graph, left, right, operation, validation,
    )? {
        return Ok(result);
    }
    match operation {
        ExactBooleanOperation::SelectedRegions(_) => {
            Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::UnsupportedExactOperation,
                format!(
                    "selected-region materialization requires the selected-region request path: {operation:?}"
                ),
            )))
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => {
            if matches!(
                operation,
                ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
            ) && let Some(result) = boolean_arrangement_regularized_boundary_contact_from_graph(
                graph, left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) = boolean_open_surface_disjoint_meshes_from_graph(
                graph, left, right, operation, validation,
            )? {
                return Ok(result);
            }
            if let Some(result) = open_surface_arrangement_result_from_graph(
                graph, left, right, operation, validation,
            )? {
                return Ok(result);
            }
            Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::MissingRequiredEvidence,
                "named exact booleans require certified winding/inside-outside classification",
            )))
        }
    }
}

enum ArrangementCellComplexOutcome {
    Materialized(Box<ExactBooleanResult>, ExactArrangementBooleanAttempt),
    Declined(ExactArrangementBooleanAttempt),
}

fn materialized_arrangement_attempt_outcome(
    attempt: &mut ExactArrangementBooleanAttempt,
    mut result: ExactBooleanResult,
    clear_arrangement_blockers: bool,
    materialized_shortcut: Option<ExactBooleanShortcutKind>,
) -> ArrangementCellComplexOutcome {
    if let Some((topology, ownership)) = attempt.retained_gate_reports() {
        if result.topology_assembly_report.is_none() {
            result.topology_assembly_report = Some(topology.clone());
        }
        if result.region_ownership_report.is_none() {
            result.region_ownership_report = Some(ownership.clone());
        }
    }
    let shortcut_reason = materialized_shortcut.map(|_| attempt.recovered_shortcut_reason());
    attempt.stage = ExactArrangementBooleanStage::Materialized;
    attempt.decline = None;
    attempt.materialized_shortcut = materialized_shortcut;
    attempt.shortcut_reason = shortcut_reason;
    if materialized_shortcut.is_some() {
        attempt.selected_cell_complex = None;
        attempt.simplified_cell_complex = None;
    }
    if clear_arrangement_blockers {
        attempt.arrangement_blockers = 0;
    }
    attempt.retain_output_mesh(&result.mesh);
    ArrangementCellComplexOutcome::Materialized(Box::new(result), attempt.clone())
}

fn not_attempted_arrangement_attempt_for_request(
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
) -> ExactArrangementBooleanAttempt {
    ExactArrangementBooleanAttempt::new(request, policy, ExactArrangementBooleanStage::NotAttempted)
}

fn declined_output_validation_attempt_outcome_with_counts(
    attempt: &mut ExactArrangementBooleanAttempt,
    output_counts: Option<(usize, usize)>,
) -> ArrangementCellComplexOutcome {
    if let Some((vertices, triangles)) = output_counts {
        attempt.output_vertices = vertices;
        attempt.output_triangles = triangles;
        attempt.output_facts = None;
    }
    attempt.stage = ExactArrangementBooleanStage::Triangulated;
    attempt.decline = Some(ExactArrangementBooleanDecline::OutputValidation);
    ArrangementCellComplexOutcome::Declined(attempt.clone())
}

struct ArrangementCellComplexRecoveryContext<'a> {
    enabled: bool,
    regularized_sheet_recovery_surface: bool,
    validation: ExactMeshValidationPolicy,
    graph: &'a super::graph::ExactIntersectionGraph,
    left: &'a ExactMesh,
    right: &'a ExactMesh,
    operation: ExactBooleanOperation,
}

impl ArrangementCellComplexRecoveryContext<'_> {
    fn outcome_if_available(
        &self,
        attempt: &mut ExactArrangementBooleanAttempt,
    ) -> Result<Option<ArrangementCellComplexOutcome>, ExactMeshError> {
        if self.enabled && self.regularized_sheet_recovery_surface {
            if let Some(result) = boolean_arrangement_regularized_sheet_complex_from_graph(
                self.graph,
                self.left,
                self.right,
                self.operation,
                self.validation,
            )? {
                return Ok(Some(materialized_arrangement_attempt_outcome(
                    attempt,
                    result,
                    true,
                    Some(ExactBooleanShortcutKind::ArrangementCellComplex),
                )));
            }
            if let Some(result) = boolean_arrangement_convex_regularized_sheet_recovery(
                self.left,
                self.right,
                self.operation,
                self.validation,
            )? {
                return Ok(Some(materialized_arrangement_attempt_outcome(
                    attempt,
                    result,
                    true,
                    Some(ExactBooleanShortcutKind::ArrangementCellComplex),
                )));
            }
        }
        if self.enabled {
            if let Some(result) = materialize_arrangement_volumetric_split_cell_result_from_graph(
                self.graph,
                self.left,
                self.right,
                self.operation,
                self.validation,
            )? {
                return Ok(Some(materialized_arrangement_attempt_outcome(
                    attempt,
                    result,
                    true,
                    Some(ExactBooleanShortcutKind::ArrangementCellComplex),
                )));
            }
            if self.validation == ExactMeshValidationPolicy::CLOSED
                && let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
                    self.graph,
                    self.left,
                    self.right,
                    self.operation,
                    ExactMeshValidationPolicy::ALLOW_BOUNDARY,
                )?
                && !materialized.mesh.facts().mesh.closed_manifold
                && !materialized.mesh.triangles().is_empty()
                && !matches!(
                    &volumetric_boundary_closure_report_from_materialized_with_prevalidated_closure(
                        &materialized,
                        self.operation,
                        None,
                    )?
                    .status,
                    &ExactVolumetricBoundaryClosureStatus::AlreadyClosed
                        | &ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
                )
            {
                return Ok(Some(
                    declined_output_validation_attempt_outcome_with_counts(
                        attempt,
                        Some((
                            materialized.mesh.vertices().len(),
                            materialized.mesh.triangles().len(),
                        )),
                    ),
                ));
            }
        }
        if !matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
            && open_surface_disjoint_report_from_graph(self.graph, self.left, self.right).status
                != ExactOpenSurfaceDisjointStatus::Certified
        {
            match boolean_coplanar_mesh_overlay_optional(
                self.left,
                self.right,
                self.operation,
                self.validation,
            ) {
                Ok(Some(result)) => {
                    return Ok(Some(materialized_arrangement_attempt_outcome(
                        attempt,
                        result,
                        false,
                        Some(ExactBooleanShortcutKind::ArrangementCellComplex),
                    )));
                }
                Ok(None) => {}
                Err(_) => {
                    let output_counts = coplanar_mesh_overlay_candidate_counts(
                        self.left,
                        self.right,
                        self.operation,
                    );
                    return Ok(Some(
                        declined_output_validation_attempt_outcome_with_counts(
                            attempt,
                            output_counts,
                        ),
                    ));
                }
            }
        }
        if let Some(plan) = open_surface_arrangement_plan_from_graph(
            self.graph,
            self.left,
            self.right,
            self.operation,
        )? {
            let result = match materialize_open_surface_arrangement_plan(
                self.left,
                self.right,
                self.operation,
                self.validation,
                self.graph.has_unknowns(),
                plan.clone(),
            ) {
                Ok(Some(result)) => result,
                outcome => {
                    let output_counts = materialize_open_surface_arrangement_plan(
                        self.left,
                        self.right,
                        self.operation,
                        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
                        self.graph.has_unknowns(),
                        plan,
                    )
                    .ok()
                    .flatten()
                    .map(|result| (result.mesh.vertices().len(), result.mesh.triangles().len()));
                    if let Err(error) = outcome
                        && output_counts.is_none()
                    {
                        return Err(error);
                    }
                    return Ok(Some(
                        declined_output_validation_attempt_outcome_with_counts(
                            attempt,
                            output_counts,
                        ),
                    ));
                }
            };
            return Ok(Some(materialized_arrangement_attempt_outcome(
                attempt,
                result,
                false,
                Some(ExactBooleanShortcutKind::ArrangementCellComplex),
            )));
        }
        let Some(result) = boolean_arrangement_cell_complex_recovery(
            self.left,
            self.right,
            self.operation,
            self.validation,
        )?
        else {
            return Ok(None);
        };
        Ok(Some(materialized_arrangement_attempt_outcome(
            attempt,
            result,
            true,
            Some(ExactBooleanShortcutKind::ArrangementCellComplex),
        )))
    }
}

fn arrangement_cell_complex_decline_after_recovery(
    recovery: &ArrangementCellComplexRecoveryContext<'_>,
    mut attempt: ExactArrangementBooleanAttempt,
    decline: ExactArrangementBooleanDecline,
) -> Result<ArrangementCellComplexOutcome, ExactMeshError> {
    if let Some(outcome) = recovery.outcome_if_available(&mut attempt)? {
        return Ok(outcome);
    }
    attempt.decline = Some(decline);
    Ok(ArrangementCellComplexOutcome::Declined(attempt))
}

pub(crate) fn arrangement_cell_complex_shortcut_attempt_with_facts(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<Option<ExactArrangementBooleanAttempt>, ExactMeshError> {
    if policy != ExactRegularizationPolicy::REGULARIZED_SOLID {
        return Ok(None);
    }
    if !shortcut_facts.materializes_operation(request.operation) {
        return Ok(None);
    }
    let Some(result) = boolean_arrangement_cell_complex_recovery(
        left,
        right,
        request.operation,
        request.validation,
    )?
    else {
        return Ok(None);
    };
    let mut attempt = not_attempted_arrangement_attempt_for_request(request, policy);
    match materialized_arrangement_attempt_outcome(
        &mut attempt,
        result,
        false,
        Some(ExactBooleanShortcutKind::ArrangementCellComplex),
    ) {
        ArrangementCellComplexOutcome::Materialized(_, attempt) => Ok(Some(attempt)),
        ArrangementCellComplexOutcome::Declined(_) => {
            Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::UnsupportedCellMaterializer,
                "certified arrangement-cell shortcut declined during support-only materialization",
            )))
        }
    }
}

fn arrangement_cell_complex_result_is_certified_for_preflight(
    result: &ExactBooleanResult,
    attempt: &ExactArrangementBooleanAttempt,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactMeshError> {
    let Some(operation) = result.kind.arrangement_cell_complex_operation() else {
        return Ok(false);
    };
    if attempt.operation != operation
        || attempt.policy != ExactRegularizationPolicy::REGULARIZED_SOLID
    {
        return Ok(false);
    }
    attempt.validate().map_err(|error| {
        retained_evidence_validation_error(
            "retained arrangement attempt failed preflight certification",
            error,
            ExactMeshBlockerKind::ExactConstructionFailure,
        )
    })?;
    if !attempt.materialized_arrangement_cell_complex_output() {
        return Ok(false);
    }
    if let Err(error) = result.validate_against_sources(left, right) {
        return if result.kind.is_arrangement_cell_complex_shortcut()
            && attempt.materialized_arrangement_cell_complex_shortcut_output()
        {
            Ok(false)
        } else {
            Err(retained_evidence_validation_error(
                "arrangement cell-complex result failed preflight certification replay",
                error,
                ExactMeshBlockerKind::ExactConstructionFailure,
            ))
        };
    }
    if let Some((topology, ownership)) = attempt.retained_gate_reports() {
        if result.topology_assembly_report.as_ref() != Some(topology)
            || result.region_ownership_report.as_ref() != Some(ownership)
        {
            return Ok(false);
        }
    } else if result.topology_assembly_report.is_some() || result.region_ownership_report.is_some()
    {
        return Ok(false);
    }
    let Some(output_facts) = attempt.output_facts.as_ref() else {
        return Ok(false);
    };
    Ok(result.mesh.vertices().len() == attempt.output_vertices
        && result.mesh.facts().mesh.face_count == attempt.output_triangles
        && &result.mesh.facts().mesh == output_facts)
}

fn certified_arrangement_cell_complex_result_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    regularize_unregularized_sheet_complex: bool,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    validate_graph_source_replay(graph, left, right)?;
    let arrangement = match ExactArrangement3d::from_source_certified_intersection_graph_with_policy(
        graph.clone(),
        left,
        right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    ) {
        Ok(arrangement) => arrangement,
        Err(error) => return arrangement_error_declines_or_replays_stale(error),
    };
    let outcome = run_arrangement_cell_complex_attempt_from_arrangement(
        &arrangement,
        left,
        right,
        request,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
        regularize_unregularized_sheet_complex,
    )?;
    let ArrangementCellComplexOutcome::Materialized(result, attempt) = outcome else {
        return Ok(None);
    };
    if arrangement_cell_complex_result_is_certified_for_preflight(&result, &attempt, left, right)? {
        Ok(Some(*result))
    } else {
        Ok(None)
    }
}

fn boolean_arrangement_regularized_boundary_contact_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if !matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    ) {
        return Ok(None);
    }
    if evidence::identical_mesh_report_from_sources(left, right).status
        == ExactIdenticalMeshStatus::Certified
        || evidence::same_surface_report_from_sources(left, right).status
            == ExactSameSurfaceStatus::Certified
    {
        return Ok(None);
    }
    if let Some(report) =
        certified_closed_boundary_touching_regularized_report_from_graph(graph, left, right)?
    {
        report
            .validate_against_sources(left, right)
            .map_err(|error| {
                boolean_validation_error(
                    ExactMeshBlockerKind::StaleFactReplay,
                    "exact arrangement regularized boundary contact consumed invalid certificate",
                    error,
                )
            })?;
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
        ExactBooleanOperation::Union | ExactBooleanOperation::SelectedRegions(_) => {
            return Ok(None);
        }
    };
    Ok(Some(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    )))
}

fn certified_arrangement_regularized_boundary_contact_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<bool, ExactMeshError> {
    if !matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    ) {
        return Ok(false);
    }
    if evidence::identical_mesh_report_from_sources(left, right).status
        == ExactIdenticalMeshStatus::Certified
        || evidence::same_surface_report_from_sources(left, right).status
            == ExactSameSurfaceStatus::Certified
    {
        return Ok(false);
    }
    if matches!(
        certified_convex_relation_shortcut_from_graph(graph, left, right, operation)?,
        Some(ConvexRelationShortcut::LeftInsideRight | ConvexRelationShortcut::RightInsideLeft)
    ) {
        return Ok(false);
    }
    if let Some(report) =
        certified_closed_boundary_touching_regularized_report_from_graph(graph, left, right)?
    {
        return Ok(report.validate_against_sources(left, right).is_ok());
    }
    if !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
        || coplanar_volumetric_evidence_from_graph(graph, left, right)?
            .filter(coplanar_evidence_is_boundary_only_contact)
            .is_none()
    {
        return Ok(false);
    }
    Ok(true)
}

fn run_arrangement_cell_complex_attempt_from_arrangement(
    arrangement: &ExactArrangement3d,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
    regularize_unregularized_sheet_complex: bool,
) -> Result<ArrangementCellComplexOutcome, ExactMeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let mut attempt = ExactArrangementBooleanAttempt::new(
        request,
        policy,
        ExactArrangementBooleanStage::ArrangementBuilt,
    );
    attempt.retain_arrangement_counts(arrangement);
    let regularized_sheet_recovery_surface = left.facts().mesh.closed_manifold
        && right.facts().mesh.closed_manifold
        && if let Some(regions) = arrangement.shells_or_regions.as_ref() {
            regions
                .iter()
                .any(|region| region.non_manifold_edges > 0 && region.source_sides.len() > 1)
        } else {
            false
        };
    let volume_resolves_region_classification =
        arrangement_region_classification_blockers_resolve_operation(arrangement, operation);
    let selected_regions_ignore_unresolved_classification =
        matches!(operation, ExactBooleanOperation::SelectedRegions(_))
            && arrangement
                .blockers
                .iter()
                .all(|blocker| *blocker == ExactArrangementBlocker::UnresolvedRegionClassification);
    let recovery = ArrangementCellComplexRecoveryContext {
        enabled: regularize_unregularized_sheet_complex,
        regularized_sheet_recovery_surface,
        validation,
        graph: &arrangement.graph,
        left,
        right,
        operation,
    };

    if !arrangement.blockers.is_empty()
        && !volume_resolves_region_classification
        && !selected_regions_ignore_unresolved_classification
    {
        let unregularized_sheet_complex = arrangement
            .blockers
            .contains(&ExactArrangementBlocker::UnregularizedOpenSheetComplex)
            && arrangement.blockers.iter().all(|blocker| {
                matches!(
                    blocker,
                    ExactArrangementBlocker::UnregularizedCoincidentSheetComplex
                        | ExactArrangementBlocker::UnregularizedOpenSheetComplex
                )
            });
        match materialize_simple_coplanar_overlay_arrangement(
            recovery.left,
            recovery.right,
            recovery.operation,
            Some(validation),
            arrangement,
        ) {
            Ok(Some(result)) => {
                return Ok(materialized_arrangement_attempt_outcome(
                    &mut attempt,
                    result,
                    false,
                    Some(ExactBooleanShortcutKind::ArrangementCellComplex),
                ));
            }
            Ok(None) => {}
            Err(_) => {
                let output_counts = coplanar_mesh_overlay_candidate_counts(
                    recovery.left,
                    recovery.right,
                    recovery.operation,
                );
                return Ok(declined_output_validation_attempt_outcome_with_counts(
                    &mut attempt,
                    output_counts,
                ));
            }
        }
        if regularize_unregularized_sheet_complex
            && unregularized_sheet_complex
            && let Some(result) = boolean_arrangement_regularized_sheet_complex_from_graph(
                recovery.graph,
                recovery.left,
                recovery.right,
                recovery.operation,
                validation,
            )?
        {
            return Ok(materialized_arrangement_attempt_outcome(
                &mut attempt,
                result,
                true,
                Some(ExactBooleanShortcutKind::ArrangementCellComplex),
            ));
        }
        if let Some(outcome) = recovery.outcome_if_available(&mut attempt)? {
            return Ok(outcome);
        }
        if unregularized_sheet_complex
            && let Some(result) = boolean_arrangement_convex_regularized_sheet_recovery(
                recovery.left,
                recovery.right,
                recovery.operation,
                validation,
            )?
        {
            return Ok(materialized_arrangement_attempt_outcome(
                &mut attempt,
                result,
                true,
                Some(ExactBooleanShortcutKind::ArrangementCellComplex),
            ));
        }
        attempt.decline = Some(ExactArrangementBooleanDecline::ArrangementBlockers(
            arrangement.blockers.clone(),
        ));
        return Ok(ArrangementCellComplexOutcome::Declined(attempt));
    }

    let ArrangementCellComplexGateEvidence {
        mut attempt,
        labeled,
        topology_report,
        ownership_report,
    } = match arrangement_cell_complex_gate_evidence_from_arrangement(
        arrangement,
        left,
        right,
        operation,
        policy,
        &recovery,
        attempt,
    )? {
        ControlFlow::Continue(evidence) => evidence,
        ControlFlow::Break(outcome) => return Ok(outcome),
    };
    let selected = match if ownership_report.volume_selection_resolves_operation(operation) {
        labeled.select_volume_resolved(operation)
    } else {
        labeled.select_with_policy(operation, policy)
    } {
        Ok(mut selected) if selected.blockers.is_empty() => {
            selected.topology_assembly_report = Some(topology_report.clone());
            selected.region_ownership_report = Some(ownership_report.clone());
            selected
        }
        Ok(selected) => {
            let counts = selected.counts();
            attempt.retain_selected_counts(counts);
            return arrangement_cell_complex_decline_after_recovery(
                &recovery,
                attempt,
                ExactArrangementBooleanDecline::Selection(selected.blockers[0].clone()),
            );
        }
        Err(blocker) => {
            return arrangement_cell_complex_decline_after_recovery(
                &recovery,
                attempt,
                ExactArrangementBooleanDecline::Selection(blocker),
            );
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Selected;
    let counts = selected.counts();
    attempt.retain_selected_counts(counts);
    attempt.selected_cell_complex = Some(selected.clone());
    let simplified = match simplify_selected_cell_complex(selected, policy) {
        Ok(simplified) if simplified.blockers.is_empty() => simplified,
        Ok(simplified) => {
            return arrangement_cell_complex_decline_after_recovery(
                &recovery,
                attempt,
                ExactArrangementBooleanDecline::Simplification(simplified.blockers[0].clone()),
            );
        }
        Err(blocker) => {
            return arrangement_cell_complex_decline_after_recovery(
                &recovery,
                attempt,
                ExactArrangementBooleanDecline::Simplification(blocker),
            );
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Simplified;
    attempt.simplified_cell_complex = Some(simplified.clone());
    let mesh = match triangulate_simplified_cell_complex(&simplified) {
        Ok(mesh) => mesh,
        Err(blocker) => {
            return arrangement_cell_complex_decline_after_recovery(
                &recovery,
                attempt,
                ExactArrangementBooleanDecline::Triangulation(blocker),
            );
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Triangulated;
    attempt.retain_output_mesh(&mesh);
    let Some((mesh, closed_by_coplanar_boundary)) = copy_mesh_or_closed_coplanar_boundary_closure(
        &mesh,
        "exact arrangement cell-complex boolean result",
        "exact arrangement cell-complex closed coplanar-boundary result",
        validation,
    )?
    else {
        return arrangement_cell_complex_decline_after_recovery(
            &recovery,
            attempt,
            ExactArrangementBooleanDecline::OutputValidation,
        );
    };
    let result = certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    Ok(materialized_arrangement_attempt_outcome(
        &mut attempt,
        result,
        volume_resolves_region_classification && !closed_by_coplanar_boundary,
        closed_by_coplanar_boundary.then_some(ExactBooleanShortcutKind::ArrangementCellComplex),
    ))
}

fn copy_mesh_or_closed_coplanar_boundary_closure(
    mesh: &ExactMesh,
    copy_label: &'static str,
    closure_label: &'static str,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<(ExactMesh, bool)>, ExactMeshError> {
    match copy_mesh(mesh, copy_label, validation) {
        Ok(mesh) => Ok(Some((mesh, false))),
        Err(_) if validation == ExactMeshValidationPolicy::CLOSED => Ok(
            optional_coplanar_boundary_closure(mesh, closure_label, validation)?
                .map(|mesh| (mesh, true)),
        ),
        Err(_) => Ok(None),
    }
}

fn optional_coplanar_boundary_closure(
    mesh: &ExactMesh,
    label: &'static str,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    match close_exact_coplanar_boundary_loops(mesh, label, validation) {
        Ok(mesh) => Ok(mesh),
        Err(error)
            if error.has_only_blocker_kinds(&[
                ExactMeshBlockerKind::BoundaryEdge,
                ExactMeshBlockerKind::NonManifoldEdge,
                ExactMeshBlockerKind::DuplicateDirectedEdge,
                ExactMeshBlockerKind::NonManifoldVertexLink,
                ExactMeshBlockerKind::DegenerateTriangle,
                ExactMeshBlockerKind::DuplicateTriangle,
            ]) =>
        {
            Ok(None)
        }
        Err(error) => Err(error),
    }
}

struct ArrangementCellComplexGateEvidence {
    attempt: ExactArrangementBooleanAttempt,
    labeled: ExactLabeledCellComplex,
    topology_report: ExactTopologyAssemblyReport,
    ownership_report: ExactRegionOwnershipReport,
}

fn arrangement_cell_complex_gate_evidence_from_arrangement(
    arrangement: &ExactArrangement3d,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    policy: ExactRegularizationPolicy,
    recovery: &ArrangementCellComplexRecoveryContext<'_>,
    mut attempt: ExactArrangementBooleanAttempt,
) -> Result<
    ControlFlow<ArrangementCellComplexOutcome, ArrangementCellComplexGateEvidence>,
    ExactMeshError,
> {
    let topology_report = arrangement.topology_assembly_report_with_policy(left, right, policy);
    attempt.topology_assembly = Some(topology_report.status);
    attempt.topology_assembly_report = Some(topology_report.clone());
    topology_report.validate().map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "exact topology assembly report validation failed",
            error,
        )
    })?;
    if !matches!(
        topology_report.status,
        ExactTopologyAssemblyStatus::Complete
    ) {
        return Ok(ControlFlow::Break(
            arrangement_cell_complex_decline_after_recovery(
                recovery,
                attempt,
                ExactArrangementBooleanDecline::TopologyAssembly(topology_report.status),
            )?,
        ));
    }

    let labeling_policy =
        arrangement_cell_complex_labeling_policy(arrangement, Some(operation), policy);
    let labeled = match arrangement.label_regions(labeling_policy) {
        Ok(labeled) => labeled,
        Err(blocker) => {
            return Ok(ControlFlow::Break(
                arrangement_cell_complex_decline_after_recovery(
                    recovery,
                    attempt,
                    ExactArrangementBooleanDecline::Labeling(blocker),
                )?,
            ));
        }
    };
    attempt.stage = ExactArrangementBooleanStage::Labeled;

    let ownership_report = labeled.region_ownership_report(left, right, labeling_policy);
    attempt.region_ownership = Some(ownership_report.status);
    attempt.region_ownership_report = Some(ownership_report.clone());
    ownership_report.validate().map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "exact region ownership report validation failed",
            error,
        )
    })?;
    if !ownership_report.resolves_operation_selection(operation) {
        return Ok(ControlFlow::Break(
            arrangement_cell_complex_decline_after_recovery(
                recovery,
                attempt,
                ExactArrangementBooleanDecline::RegionOwnership(ownership_report.status),
            )?,
        ));
    }

    Ok(ControlFlow::Continue(ArrangementCellComplexGateEvidence {
        attempt,
        labeled,
        topology_report,
        ownership_report,
    }))
}

pub(crate) fn adjacent_union_completion_certification_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    materialization_validation: Option<ExactMeshValidationPolicy>,
) -> Result<
    (
        ExactAdjacentUnionCompletionReport,
        Option<ExactBooleanResult>,
    ),
    ExactMeshError,
> {
    let left_closed = left.facts().mesh.closed_manifold;
    let right_closed = right.facts().mesh.closed_manifold;
    if operation != ExactBooleanOperation::Union {
        return Ok((
            RetainedGraphCounts::empty().into_adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::NotUnion,
                left_closed,
                right_closed,
                false,
                false,
                ExactBooleanBlocker::default(),
                0,
                0,
                None,
                0,
                0,
            ),
            None,
        ));
    }
    if !left_closed || !right_closed {
        return Ok((
            RetainedGraphCounts::empty().into_adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::NotClosedSolid,
                left_closed,
                right_closed,
                false,
                false,
                ExactBooleanBlocker::default(),
                0,
                0,
                None,
                0,
                0,
            ),
            None,
        ));
    }
    let axis_aligned_box_pair = orthogonal_solid::try_certified_axis_aligned_box_pair(left, right)?;
    if axis_aligned_box_pair {
        return Ok((
            RetainedGraphCounts::empty().into_adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::AxisAlignedBoxPair,
                left_closed,
                right_closed,
                true,
                false,
                ExactBooleanBlocker::default(),
                0,
                0,
                None,
                0,
                0,
            ),
            None,
        ));
    }

    let graph_counts = retained_graph_counts(graph);
    let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    if graph_counts.graph_had_unknowns || counts.construction_failed_events != 0 {
        return Ok((
            graph_counts.into_adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::GraphUnresolved,
                left_closed,
                right_closed,
                false,
                false,
                counts,
                0,
                0,
                None,
                0,
                0,
            ),
            None,
        ));
    }

    if let Some(certificate) = full_face_adjacent_certificate_from_graph(left, right, graph)?
        && let Some(union) = materialize_full_face_adjacent_union_from_certificate(
            left,
            right,
            &certificate,
            materialization_validation.unwrap_or(ExactMeshValidationPolicy::CLOSED),
        )?
    {
        let full_face_shared_faces = union.shared_faces.len();
        let full_face_shared_patches = union.shared_patches.len();
        let result = if materialization_validation.is_some() {
            let result = certified_shortcut_result(
                union.mesh,
                ExactBooleanOperation::Union,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            );
            validate_boolean_result(
                &result,
                "exact full-face adjacent-union completion result validation failed",
            )?;
            Some(result)
        } else {
            None
        };
        return Ok((
            graph_counts.into_adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::CertifiedFullFace,
                left_closed,
                right_closed,
                false,
                false,
                counts,
                full_face_shared_faces,
                full_face_shared_patches,
                None,
                0,
                0,
            ),
            result,
        ));
    }

    if certified_convex_operation_shortcut_support(left, right, operation).is_some()
        || orthogonal_solid::try_certified_axis_aligned_box_pair(left, right)?
        || match operation {
            ExactBooleanOperation::Union => matches!(
                affine_orthogonal_solid_cell_selected_count(
                    left,
                    right,
                    AffineOrthogonalSolidOperation::Union,
                ),
                Some(_)
            ),
            ExactBooleanOperation::Intersection => matches!(
                affine_orthogonal_solid_cell_selected_count(
                    left,
                    right,
                    AffineOrthogonalSolidOperation::Intersection,
                ),
                Some(_)
            ),
            ExactBooleanOperation::Difference | ExactBooleanOperation::SelectedRegions(_) => true,
        }
    {
        return Ok((
            graph_counts.into_adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::StrongerKernelAvailable,
                left_closed,
                right_closed,
                false,
                true,
                counts,
                0,
                0,
                None,
                0,
                0,
            ),
            None,
        ));
    }

    if let Some(certificate) = contained_face_adjacent_certificate_from_graph(left, right, graph)?
        && let Some(union) = materialize_contained_face_adjacent_union_from_certificate(
            left,
            right,
            &certificate,
            materialization_validation.unwrap_or(ExactMeshValidationPolicy::CLOSED),
        )?
    {
        let contained_containing_side = Some(union.containing_side);
        let contained_faces = union.contained_faces.len();
        let containing_faces = union.containing_faces.len();
        let result = if materialization_validation.is_some() {
            let result = certified_shortcut_result(
                union.mesh,
                ExactBooleanOperation::Union,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            );
            validate_boolean_result(
                &result,
                "exact contained-face adjacent-union completion result validation failed",
            )?;
            Some(result)
        } else {
            None
        };
        return Ok((
            graph_counts.into_adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::CertifiedContainedFace,
                left_closed,
                right_closed,
                false,
                false,
                counts,
                0,
                0,
                contained_containing_side,
                contained_faces,
                containing_faces,
            ),
            result,
        ));
    }

    Ok((
        graph_counts.into_adjacent_union_completion_report(
            operation,
            ExactAdjacentUnionCompletionStatus::NoAdjacencyCertificate,
            left_closed,
            right_closed,
            false,
            false,
            counts,
            0,
            0,
            None,
            0,
            0,
        ),
        None,
    ))
}

fn materialize_adjacent_union_completion_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<(ExactBooleanResult, ExactAdjacentUnionCompletionReport)>, ExactMeshError> {
    let (report, result) = adjacent_union_completion_certification_from_graph(
        graph,
        left,
        right,
        operation,
        Some(validation),
    )?;
    if !matches!(
        report.status,
        ExactAdjacentUnionCompletionStatus::CertifiedFullFace
            | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
    ) {
        return Ok(None);
    }
    report.validate().map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "exact adjacent-union completion report validation failed",
            error,
        )
    })?;
    report
        .validate_against_sources(left, right)
        .map_err(|error| {
            retained_evidence_validation_error(
                "exact adjacent-union completion report failed source replay",
                error,
                ExactMeshBlockerKind::ExactConstructionFailure,
            )
        })?;
    let Some(result) = result else {
        return Ok(None);
    };
    validate_boolean_result(
        &result,
        "exact adjacent-union completion result validation failed",
    )?;
    Ok(Some((result, report)))
}

fn boolean_arrangement_regularized_sheet_complex_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    // Unregularized sheet arrangements already retain exact split cells but can
    // lack a closed shell graph. The volumetric split-cell assembly supplies
    // the missing regularized caps without changing predicates or tolerances.
    if let Some(result) = materialize_arrangement_volumetric_split_cell_result_from_graph(
        graph, left, right, operation, validation,
    )? {
        return Ok(Some(result));
    }
    boolean_arrangement_regularized_no_volume_overlap_from_graph(
        graph, left, right, operation, validation,
    )
}

fn boolean_arrangement_regularized_no_volume_overlap_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
    {
        return Ok(None);
    }
    let Some(_evidence) = coplanar_volumetric_evidence_from_graph(graph, left, right)?
        .filter(coplanar_evidence_is_positive_area_boundary_only)
    else {
        return Ok(None);
    };
    if operation == ExactBooleanOperation::Union {
        let mesh = concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact arrangement no-volume-overlap regularized union preserving separate shells",
            validation,
        )?;
        let result = certified_shortcut_result(
            mesh,
            operation,
            ExactBooleanShortcutKind::ArrangementCellComplex,
        );
        validate_boolean_result(
            &result,
            "exact no-volume-overlap union result validation failed",
        )?;
        return Ok(Some(result));
    }

    let Some(left_minus_right) = materialize_arrangement_volumetric_split_cell_result_from_graph(
        graph,
        left,
        right,
        ExactBooleanOperation::Difference,
        ExactMeshValidationPolicy::CLOSED,
    )?
    else {
        return Ok(None);
    };
    if !arrangement_difference_preserves_source_surface(&left_minus_right, left, MeshSide::Left)? {
        return Ok(None);
    }

    let reverse_graph = build_validated_intersection_graph(right, left)?;
    let Some(right_minus_left) = materialize_arrangement_volumetric_split_cell_result_from_graph(
        &reverse_graph,
        right,
        left,
        ExactBooleanOperation::Difference,
        ExactMeshValidationPolicy::CLOSED,
    )?
    else {
        return Ok(None);
    };
    if !arrangement_difference_preserves_source_surface(&right_minus_left, right, MeshSide::Left)? {
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
            ExactBooleanShortcutKind::ArrangementCellComplex,
        ),
        ExactBooleanOperation::Intersection => (
            empty_mesh(
                "empty exact arrangement no-volume-overlap regularized intersection",
                validation,
            )?,
            ExactBooleanShortcutKind::ArrangementCellComplex,
        ),
        ExactBooleanOperation::Difference => (
            copy_mesh(
                left,
                "exact arrangement no-volume-overlap difference preserving left shell",
                validation,
            )?,
            ExactBooleanShortcutKind::ArrangementCellComplex,
        ),
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    let result = certified_shortcut_result(mesh, operation, shortcut);
    validate_boolean_result(&result, "exact no-volume-overlap result validation failed")?;
    Ok(Some(result))
}

pub(crate) fn materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let Some(evidence) = coplanar_volumetric_evidence_from_graph(graph, left, right)?
        .filter(coplanar_evidence_is_positive_area_boundary_only)
    else {
        return Ok(None);
    };
    let result = match operation {
        ExactBooleanOperation::Union => {
            let Some(result) = boolean_arrangement_regularized_no_volume_overlap_from_graph(
                graph, left, right, operation, validation,
            )?
            else {
                return Ok(None);
            };
            result
        }
        ExactBooleanOperation::Intersection => {
            let mesh = empty_mesh(
                "empty exact arrangement cell-complex no-volume-overlap intersection",
                validation,
            )?;
            certified_shortcut_result(
                mesh,
                operation,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            )
        }
        ExactBooleanOperation::Difference => {
            let mesh = copy_mesh(
                left,
                "exact arrangement cell-complex no-volume-overlap difference preserving left shell",
                validation,
            )?;
            certified_shortcut_result(
                mesh,
                operation,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            )
        }
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    validate_boolean_result(
        &result,
        "exact no-volume-overlap materialized result validation failed",
    )?;
    validate_coplanar_volumetric_evidence_against_sources(
        &evidence,
        left,
        right,
        "exact no-volume-overlap materialized evidence failed source replay",
    )?;
    Ok(Some((result, evidence)))
}

fn validate_coplanar_volumetric_evidence_against_sources(
    evidence: &CoplanarVolumetricCellEvidenceReport,
    left: &ExactMesh,
    right: &ExactMesh,
    context: &'static str,
) -> Result<(), ExactMeshError> {
    evidence
        .validate_against_sources(left, right)
        .map_err(|error| {
            let kind = match error {
                CoplanarVolumetricCellEvidenceError::SourceReplayMismatch => {
                    ExactMeshBlockerKind::StaleFactReplay
                }
                _ => ExactMeshBlockerKind::ExactConstructionFailure,
            };
            ExactMeshError::one(ExactMeshBlocker::new(kind, format!("{context}: {error:?}")))
        })
}

fn arrangement_difference_preserves_source_surface(
    result: &ExactBooleanResult,
    source: &ExactMesh,
    source_side: MeshSide,
) -> Result<bool, ExactMeshError> {
    if result.kind.arrangement_cell_complex_operation() != Some(ExactBooleanOperation::Difference) {
        return Ok(false);
    }
    validate_boolean_result(
        result,
        "exact arrangement difference source-preservation result validation failed",
    )?;
    let mut retained_area_by_face = vec![Real::from(0); source.triangles().len()];
    for triangle in &result.assembly.triangles {
        if triangle.source_side != source_side || triangle.source_face >= source.triangles().len() {
            return Ok(false);
        }
        let Ok(projection) = choose_region_projection(source, triangle.source_face) else {
            return Ok(false);
        };
        let points = triangle
            .vertices
            .iter()
            .map(|vertex| {
                result
                    .assembly
                    .vertices
                    .get(*vertex)
                    .map(|vertex| vertex.point.clone())
                    .ok_or_else(|| {
                        ExactMeshError::one(
                            ExactMeshBlocker::new(
                                ExactMeshBlockerKind::StaleFactReplay,
                                "exact arrangement difference source-preservation assembly references a missing retained vertex",
                            )
                            .with_vertex(*vertex),
                        )
                    })
            })
            .collect::<Result<Vec<_>, _>>()?;
        let area = projected_polygon_area2_value(&points, projection);
        let Some(area) = (match compare_reals(&area, &Real::from(0)).value() {
            Some(Ordering::Less) => Some(Real::from(0) - area),
            Some(Ordering::Equal | Ordering::Greater) => Some(area),
            None => None,
        }) else {
            return Ok(false);
        };
        if compare_reals(&area, &Real::from(0)).value() != Some(Ordering::Greater) {
            return Ok(false);
        }
        retained_area_by_face[triangle.source_face] =
            retained_area_by_face[triangle.source_face].clone() + area;
    }

    Ok(source.triangles().iter().enumerate().all(|(face, _)| {
        let Ok(face_ref) = source.view().face(face) else {
            return false;
        };
        let Ok([a, b, c]) = face_ref.vertices() else {
            return false;
        };
        let points = [a.clone(), b.clone(), c.clone()];
        let Ok(projection) = choose_region_projection(source, face) else {
            return false;
        };
        let source_area = projected_polygon_area2_value(&points, projection);
        let Some(source_area) = (match compare_reals(&source_area, &Real::from(0)).value() {
            Some(Ordering::Less) => Some(Real::from(0) - source_area),
            Some(Ordering::Equal | Ordering::Greater) => Some(source_area),
            None => None,
        }) else {
            return false;
        };
        compare_reals(&retained_area_by_face[face], &source_area).value() == Some(Ordering::Equal)
    }))
}

fn boolean_convex_meshes_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let (mesh, label) = match operation {
        ExactBooleanOperation::Union => {
            let Some(union) = union_closed_convex_solids(left, right)? else {
                return Ok(None);
            };
            (union.mesh, "exact closed-convex solid union boolean result")
        }
        ExactBooleanOperation::Intersection => {
            let Some(intersection) = intersect_closed_convex_solids(left, right)? else {
                return Ok(None);
            };
            (
                intersection.mesh,
                "exact closed-convex solid intersection boolean result",
            )
        }
        ExactBooleanOperation::Difference => {
            let Some(difference) = subtract_closed_convex_solids(left, right)? else {
                return Ok(None);
            };
            (
                difference.mesh,
                "exact closed-convex solid difference boolean result",
            )
        }
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    let Some(shortcut) = operation.convex_operation_shortcut() else {
        return Ok(None);
    };
    let mesh = copy_mesh(&mesh, label, validation)?;
    let result = certified_shortcut_result(mesh, operation, shortcut);
    result
        .validate_against_sources(left, right)
        .map_err(|error| {
            retained_evidence_validation_error(
                "exact convex boolean result/source replay failed",
                error,
                ExactMeshBlockerKind::ExactConstructionFailure,
            )
        })?;
    Ok(Some(result))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ConvexRelationShortcut {
    LeftInsideRight,
    RightInsideLeft,
    Separated,
}

fn certified_convex_relation_shortcut_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ConvexRelationShortcut>, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let relation_counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    if graph.has_unknowns() || relation_counts.construction_failed_events > 0 {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_convex_solid_report(left, right);
    left_in_right
        .validate_against_sources(left, right)
        .map_err(|error| {
            boolean_validation_error(
                ExactMeshBlockerKind::StaleFactReplay,
                "left convex relation replay failed",
                error,
            )
        })?;
    let right_in_left = classify_mesh_vertices_against_convex_solid_report(right, left);
    right_in_left
        .validate_against_sources(right, left)
        .map_err(|error| {
            boolean_validation_error(
                ExactMeshBlockerKind::StaleFactReplay,
                "right convex relation replay failed",
                error,
            )
        })?;

    if graph.face_pairs.is_empty() {
        return Ok(match (left_in_right.relation, right_in_left.relation) {
            (ConvexSolidMeshRelation::StrictlyInside, _) => {
                Some(ConvexRelationShortcut::LeftInsideRight)
            }
            (_, ConvexSolidMeshRelation::StrictlyInside) => {
                Some(ConvexRelationShortcut::RightInsideLeft)
            }
            (ConvexSolidMeshRelation::Outside, ConvexSolidMeshRelation::Outside) => {
                Some(ConvexRelationShortcut::Separated)
            }
            _ => None,
        });
    }

    let left_boundary_inside_right =
        left_in_right.supports_boundary_containment_against(&right_in_left);
    let right_boundary_inside_left =
        right_in_left.supports_boundary_containment_against(&left_in_right);
    Ok(match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
            if left_boundary_inside_right =>
        {
            Some(ConvexRelationShortcut::LeftInsideRight)
        }
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
            if right_boundary_inside_left =>
        {
            Some(ConvexRelationShortcut::RightInsideLeft)
        }
        ExactBooleanOperation::Difference if left_boundary_inside_right => {
            Some(ConvexRelationShortcut::LeftInsideRight)
        }
        _ => None,
    })
}

fn boolean_convex_relation_meshes_optional_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    validate_graph_source_replay(graph, left, right)?;
    let Some(relation) =
        certified_convex_relation_shortcut_from_graph(graph, left, right, operation)?
    else {
        return Ok(None);
    };

    let (mesh, shortcut) = match relation {
        ConvexRelationShortcut::Separated => {
            let mesh = match operation {
                ExactBooleanOperation::Union => concatenate_meshes_with_options(
                    left,
                    right,
                    false,
                    "exact closed-convex separated union",
                    validation,
                )?,
                ExactBooleanOperation::Intersection => empty_mesh(
                    "empty exact closed-convex separated intersection",
                    validation,
                )?,
                ExactBooleanOperation::Difference => copy_mesh(
                    left,
                    "exact closed-convex separated difference keeps left",
                    validation,
                )?,
                ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
            };
            (mesh, ExactBooleanShortcutKind::ConvexSeparated)
        }
        ConvexRelationShortcut::LeftInsideRight => {
            let mesh = match operation {
                ExactBooleanOperation::Union => copy_mesh(
                    right,
                    "exact closed-convex containment union keeps right",
                    validation,
                )?,
                ExactBooleanOperation::Intersection => copy_mesh(
                    left,
                    "exact closed-convex containment intersection keeps left",
                    validation,
                )?,
                ExactBooleanOperation::Difference => empty_mesh(
                    "empty exact closed-convex containment difference",
                    validation,
                )?,
                ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
            };
            (mesh, ExactBooleanShortcutKind::ConvexContainment)
        }
        ConvexRelationShortcut::RightInsideLeft => {
            let mesh = match operation {
                ExactBooleanOperation::Union => copy_mesh(
                    left,
                    "exact closed-convex containment union keeps left",
                    validation,
                )?,
                ExactBooleanOperation::Intersection => copy_mesh(
                    right,
                    "exact closed-convex containment intersection keeps right",
                    validation,
                )?,
                ExactBooleanOperation::Difference if graph.face_pairs.is_empty() => {
                    concatenate_meshes_with_options(
                        left,
                        right,
                        true,
                        "exact closed-convex containment difference with cavity",
                        validation,
                    )?
                }
                ExactBooleanOperation::Difference => return Ok(None),
                ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
            };
            (mesh, ExactBooleanShortcutKind::ConvexContainment)
        }
    };
    let result = certified_shortcut_result(mesh, operation, shortcut);
    result
        .validate_against_sources(left, right)
        .map_err(|error| {
            retained_evidence_validation_error(
                "exact convex relation result/source replay failed",
                error,
                ExactMeshBlockerKind::ExactConstructionFailure,
            )
        })?;
    Ok(Some(result))
}

/// Certify and materialize a named boolean for closed convex solids.
///
/// This replay helper follows the retained exact materialization path
/// precedence: it only materializes when preflight certifies the requested
/// operation as a convex operation or convex relation shortcut. Inputs handled
/// by earlier exact shortcuts, such as orthogonal-cell recovery or bounds
/// disjointness, return `None` so replay provenance remains stable.
fn boolean_arrangement_convex_regularized_sheet_recovery(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let (mesh, label) = match operation {
        ExactBooleanOperation::Union => {
            let Some(union) = union_closed_convex_solids(left, right)? else {
                return Ok(None);
            };
            (
                union.mesh,
                "exact arrangement regularized convex sheet union",
            )
        }
        ExactBooleanOperation::Intersection => {
            let Some(intersection) = intersect_closed_convex_solids(left, right)? else {
                return Ok(None);
            };
            (
                intersection.mesh,
                "exact arrangement regularized convex sheet intersection",
            )
        }
        ExactBooleanOperation::Difference => {
            let Some(difference) = subtract_closed_convex_solids(left, right)? else {
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
    let result = certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    result
        .validate_against_sources(left, right)
        .map_err(|error| {
            retained_evidence_validation_error(
                "exact arrangement convex sheet recovery result/source replay failed",
                error,
                ExactMeshBlockerKind::ExactConstructionFailure,
            )
        })?;
    Ok(Some(result))
}

fn result_with_arrangement_gate_reports(
    mut result: ExactBooleanResult,
    arrangement: &ExactArrangement3d,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactBooleanResult, ExactMeshError> {
    let topology_report = arrangement.topology_assembly_report_with_policy(
        left,
        right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    let ownership_policy = arrangement_cell_complex_labeling_policy(
        &arrangement,
        Some(operation),
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    let ownership_report = arrangement
        .region_ownership_report_with_policy(left, right, ownership_policy)
        .map_err(|blocker| {
            boolean_validation_error(
                ExactMeshBlockerKind::ExactConstructionFailure,
                "exact region ownership report failed",
                blocker,
            )
        })?;
    result.topology_assembly_report = Some(topology_report);
    result.region_ownership_report = Some(ownership_report);
    Ok(result)
}

pub(crate) fn materialize_volumetric_coplanar_boundary_closure_output_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<(ExactMesh, ExactVolumetricBoundaryClosureReport)>, ExactMeshError> {
    let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        graph,
        left,
        right,
        operation,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )?
    else {
        return Ok(None);
    };
    if materialized.mesh.facts().mesh.closed_manifold || materialized.mesh.triangles().is_empty() {
        return Ok(None);
    }
    let Some(mesh) = optional_coplanar_boundary_closure(
        &materialized.mesh,
        "exact volumetric split-cell coplanar boundary closure",
        validation,
    )?
    else {
        return Ok(None);
    };
    let closure_report =
        volumetric_boundary_closure_report_from_materialized_with_prevalidated_closure(
            &materialized,
            operation,
            Some(true),
        )?;
    if !matches!(
        closure_report.status,
        ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
    ) {
        return Ok(None);
    }
    validate_volumetric_boundary_closure_report(&closure_report)?;
    Ok(Some((mesh, closure_report)))
}

fn validate_volumetric_boundary_closure_report(
    report: &ExactVolumetricBoundaryClosureReport,
) -> Result<(), ExactMeshError> {
    report.validate().map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "exact volumetric boundary closure report validation failed",
            error,
        )
    })
}

/// Materialize a named boolean from graph-backed volumetric split-cell facts.
///
/// This is a primary arrangement/cell-complex materialization path. Callers
/// that use it as a fallback should wrap the returned result in their own
/// recovery-specific attempt provenance.
fn materialize_arrangement_volumetric_split_cell_result_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if validation == ExactMeshValidationPolicy::CLOSED {
        let Some(mut materialized) = materialize_volumetric_winding_region_plan_from_graph(
            graph,
            left,
            right,
            operation,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )?
        else {
            return Ok(None);
        };
        if materialized.mesh.facts().mesh.closed_manifold
            || materialized.mesh.triangles().is_empty()
        {
            materialized.mesh = copy_mesh(
                &materialized.mesh,
                "exact closed volumetric arrangement cell-complex result",
                validation,
            )?;
            let result = volumetric_arrangement_cell_complex_result(operation, materialized);
            if validate_volumetric_arrangement_result_against_graph(
                &result, graph, None, left, right, operation, validation,
            )
            .is_err()
            {
                return Ok(None);
            }
            return Ok(Some(result));
        }
        if let Some(mesh) = certified_coplanar_boundary_closure_from_materialized(
            &materialized,
            left,
            right,
            operation,
            validation,
        )? {
            let result = certified_shortcut_result(
                mesh,
                operation,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            );
            let arrangement =
                ExactArrangement3d::from_source_certified_intersection_graph_with_policy(
                    graph.clone(),
                    left,
                    right,
                    ExactRegularizationPolicy::REGULARIZED_SOLID,
                )?;
            let result =
                result_with_arrangement_gate_reports(result, &arrangement, left, right, operation)?;
            validate_boolean_result(
                &result,
                "exact volumetric arrangement boundary result validation failed",
            )?;
            result
                .validate_arrangement_cell_complex_gate_reports_against_arrangement(
                    &arrangement,
                    left,
                    right,
                    Some(operation),
                )
                .map_err(|error| {
                    retained_evidence_validation_error(
                        "exact volumetric arrangement boundary gate reports failed validation",
                        error,
                        ExactMeshBlockerKind::ExactConstructionFailure,
                    )
                })?;
            return Ok(Some(result));
        }
        return Ok(None);
    }

    let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        graph, left, right, operation, validation,
    )?
    else {
        return Ok(None);
    };
    let result = volumetric_arrangement_cell_complex_result(operation, materialized);
    if validate_volumetric_arrangement_result_against_graph(
        &result, graph, None, left, right, operation, validation,
    )
    .is_err()
    {
        return Ok(None);
    }
    Ok(Some(result))
}

fn validate_volumetric_arrangement_result_against_graph(
    result: &ExactBooleanResult,
    graph: &super::graph::ExactIntersectionGraph,
    retained_regularized_arrangement: Option<&ExactArrangement3d>,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<(), ExactEvidenceValidationError> {
    result.validate()?;
    if let Some(arrangement) = retained_regularized_arrangement {
        result.validate_arrangement_cell_complex_gate_reports_against_arrangement(
            arrangement,
            left,
            right,
            Some(operation),
        )?;
    } else {
        result
            .validate_arrangement_cell_complex_gate_reports_against_sources(graph, left, right)?;
    }
    if result.mesh.validation_policy() != validation {
        return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
    }
    let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        graph, left, right, operation, validation,
    )
    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
    else {
        return Err(ExactEvidenceValidationError::SourceReplayMismatch);
    };
    let mut replay = volumetric_arrangement_cell_complex_result(operation, materialized);
    replay.topology_assembly_report = result.topology_assembly_report.clone();
    replay.region_ownership_report = result.region_ownership_report.clone();
    replay.validate()?;
    if result == &replay {
        Ok(())
    } else {
        Err(ExactEvidenceValidationError::SourceReplayMismatch)
    }
}

fn volumetric_arrangement_cell_complex_result(
    operation: ExactBooleanOperation,
    materialized: MaterializedVolumetricWindingRegionPlan,
) -> ExactBooleanResult {
    ExactBooleanResult {
        kind: ExactBooleanResultKind::ArrangementCellComplexMaterialized { operation },
        graph_had_unknowns: false,
        region_classifications: materialized.region_classifications,
        triangulations: materialized.triangulations,
        assembly: materialized.assembly,
        volumetric_classifications: materialized.volumetric_classifications,
        topology_assembly_report: None,
        region_ownership_report: None,
        mesh: materialized.mesh,
    }
}

fn close_exact_coplanar_boundary_loops(
    mesh: &ExactMesh,
    label: &'static str,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    if mesh.facts().mesh.closed_manifold || mesh.facts().mesh.boundary_edges == 0 {
        return Ok(None);
    }
    let Ok(boundary_loops) = directed_boundary_loops(mesh.view()) else {
        return Ok(None);
    };
    if !boundary_loops_are_exactly_coplanar_without_self_contact(mesh, &boundary_loops)? {
        return Ok(None);
    }
    close_exact_coplanar_boundary_loops_from_loops(mesh, boundary_loops, label, validation)
}

fn cloned_indexed_points(
    vertices: &[Point3],
    indices: impl IntoIterator<Item = usize>,
) -> Option<Vec<Point3>> {
    indices
        .into_iter()
        .map(|vertex| vertices.get(vertex).cloned())
        .collect()
}

fn required_cloned_indexed_points(
    vertices: &[Point3],
    indices: impl IntoIterator<Item = usize>,
    context: &'static str,
) -> Result<Vec<Point3>, ExactMeshError> {
    indices
        .into_iter()
        .map(|vertex| {
            vertices.get(vertex).cloned().ok_or_else(|| {
                ExactMeshError::one(
                    ExactMeshBlocker::new(ExactMeshBlockerKind::IndexOutOfBounds, context)
                        .with_vertex(vertex),
                )
            })
        })
        .collect()
}

fn boundary_loops_are_exactly_coplanar_without_self_contact(
    mesh: &ExactMesh,
    boundary_loops: &[Vec<usize>],
) -> Result<bool, ExactMeshError> {
    let mut boundary_points = Vec::new();
    let vertices = mesh.view().vertices();
    for boundary_loop in boundary_loops {
        let Some(points) = cloned_indexed_points(vertices, boundary_loop.iter().copied()) else {
            return Ok(false);
        };
        let split = split_cyclic_self_contact_cycles(points, &|left, right| {
            point3_exact_equal(left, right).ok_or(ExactArrangementBlocker::UndecidableOrdering)
        })
        .map_err(|blocker| {
            arrangement_blocker_error(
                "exact coplanar boundary closure self-contact split failed",
                blocker,
            )
        })?;
        boundary_points.extend(split);
    }
    if boundary_points.is_empty() {
        return Ok(false);
    }
    for boundary in &boundary_points {
        if boundary.len() < 3 {
            return Ok(false);
        }
        let self_contact = boundary_loop_self_contact_evidence(boundary).map_err(|blocker| {
            arrangement_blocker_error(
                "exact coplanar boundary closure self-contact evidence failed",
                blocker,
            )
        })?;
        if self_contact.repeated_exact_point_pairs != 0 {
            return Ok(false);
        }
        match exact_loop_is_coplanar(boundary) {
            Ok(true) => {}
            Ok(false) | Err(ExactArrangementBlocker::NonManifoldCellComplex) => return Ok(false),
            Err(blocker) => {
                return Err(arrangement_blocker_error(
                    "exact coplanar boundary closure coplanarity check failed",
                    blocker,
                ));
            }
        }
    }
    Ok(true)
}

fn certified_coplanar_boundary_closure_from_materialized(
    materialized: &MaterializedVolumetricWindingRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    let Some(mesh) = optional_coplanar_boundary_closure(
        &materialized.mesh,
        "exact volumetric split-cell coplanar boundary closure",
        validation,
    )?
    else {
        return Ok(None);
    };
    let closure_report =
        volumetric_boundary_closure_report_from_materialized_with_prevalidated_closure(
            materialized,
            operation,
            Some(true),
        )?;
    if !matches!(
        closure_report.status,
        ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
    ) {
        return Ok(None);
    }
    validate_volumetric_boundary_closure_report(&closure_report)?;
    if closure_report
        .validate_against_sources(left, right)
        .is_err()
    {
        return Ok(None);
    }
    Ok(Some(mesh))
}

fn close_exact_coplanar_boundary_loops_from_loops(
    mesh: &ExactMesh,
    boundary_loops: Vec<Vec<usize>>,
    label: &'static str,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    if mesh.facts().mesh.closed_manifold || mesh.facts().mesh.boundary_edges == 0 {
        return Ok(None);
    }
    if boundary_loops.is_empty() {
        return Ok(None);
    }

    let boundary_edges = directed_boundary_edges(mesh.view());
    let mesh_vertices = mesh.view().vertices();
    let mut split_boundary_loops = Vec::new();
    for boundary_loop in boundary_loops {
        let split = split_cyclic_self_contact_cycles(boundary_loop, &|left, right| {
            let left = mesh_vertices
                .get(*left)
                .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
            let right = mesh_vertices
                .get(*right)
                .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
            point3_exact_equal(left, right).ok_or(ExactArrangementBlocker::UndecidableOrdering)
        })
        .map_err(|blocker| {
            arrangement_blocker_error(
                "exact coplanar boundary closure vertex self-contact split failed",
                blocker,
            )
        })?;
        split_boundary_loops.extend(split);
    }
    if split_boundary_loops.is_empty() {
        return Ok(None);
    }
    if split_boundary_loops
        .iter()
        .all(|boundary_loop| boundary_loop.len() == 3)
    {
        let mut cap_triangles = Vec::new();
        for boundary_loop in &split_boundary_loops {
            let Some(points) = cloned_indexed_points(mesh_vertices, boundary_loop.iter().copied())
            else {
                return Ok(None);
            };
            let self_contact = boundary_loop_self_contact_evidence(&points).map_err(|blocker| {
                arrangement_blocker_error(
                    "exact coplanar boundary closure triangle self-contact evidence failed",
                    blocker,
                )
            })?;
            if self_contact.repeated_exact_point_pairs != 0
                || !exact_loop_is_coplanar(&points).map_err(|blocker| {
                    arrangement_blocker_error(
                        "exact coplanar boundary closure triangle coplanarity check failed",
                        blocker,
                    )
                })?
            {
                return Ok(None);
            }
            cap_triangles.push(Triangle([
                boundary_loop[0],
                boundary_loop[1],
                boundary_loop[2],
            ]));
        }
        let Some(cap_triangles) =
            orient_cap_group_against_mesh_boundary(&boundary_edges, cap_triangles)
        else {
            return Ok(None);
        };
        let mut triangles = mesh.triangles().to_vec();
        triangles.extend(cap_triangles);
        let mut seen = BTreeSet::new();
        triangles.retain(|triangle| {
            let mut key = triangle.0;
            key.sort_unstable();
            seen.insert(key)
        });
        return ExactMesh::new_with_policy_and_version(
            mesh.vertices().to_vec(),
            triangles,
            SourceProvenance::exact(label),
            validation,
            1,
        )
        .map(Some);
    }

    let cap_groups =
        group_exact_coplanar_vertex_loops(mesh, split_boundary_loops).map_err(|blocker| {
            arrangement_blocker_error("exact coplanar boundary closure grouping failed", blocker)
        })?;
    let mut vertices = mesh.vertices().to_vec();
    let mut cap_triangles = Vec::new();
    for vertex_loops in cap_groups {
        let loops = vertex_loops
            .iter()
            .map(|boundary_loop| {
                cloned_indexed_points(mesh_vertices, boundary_loop.iter().copied()).ok_or_else(
                    || {
                        ExactMeshError::one(ExactMeshBlocker::new(
                            ExactMeshBlockerKind::StaleFactReplay,
                            "exact coplanar boundary closure references a missing boundary vertex",
                        ))
                    },
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let mut group_vertices = Vec::new();
        let mut group_triangles = Vec::new();
        triangulate_exact_loop_group(&loops, &mut group_vertices, &mut group_triangles).map_err(
            |blocker| {
                arrangement_blocker_error(
                    "exact coplanar boundary closure loop triangulation failed",
                    blocker,
                )
            },
        )?;
        let local_to_global = map_cap_vertices_to_boundary_or_insert(
            mesh,
            &vertex_loops,
            &mut vertices,
            group_vertices,
        )?;
        let triangles = group_triangles.into_iter().map(|triangle| {
            Triangle([
                local_to_global[triangle.0[0]],
                local_to_global[triangle.0[1]],
                local_to_global[triangle.0[2]],
            ])
        });
        let Some(oriented) =
            orient_cap_group_against_mesh_boundary(&boundary_edges, triangles.collect())
        else {
            return Ok(None);
        };
        cap_triangles.extend(oriented);
    }

    let mut triangles = mesh.triangles().to_vec();
    triangles.extend(cap_triangles);
    let mut seen = BTreeSet::new();
    triangles.retain(|triangle| {
        let mut key = triangle.0;
        key.sort_unstable();
        seen.insert(key)
    });
    ExactMesh::new_with_policy_and_version(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        validation,
        1,
    )
    .map(Some)
}

fn group_exact_coplanar_vertex_loops(
    mesh: &ExactMesh,
    boundaries: Vec<Vec<usize>>,
) -> Result<Vec<Vec<Vec<usize>>>, ExactArrangementBlocker> {
    let mut groups = Vec::<([Point3; 3], Vec<Vec<usize>>)>::new();
    for boundary in boundaries {
        if boundary.len() < 3 {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        let points = cloned_indexed_points(mesh.view().vertices(), boundary.iter().copied())
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
        let carrier =
            exact_loop_carrier(&points)?.ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
        let mut group_index = None;
        for (index, (group_carrier, _)) in groups.iter().enumerate() {
            let mut is_coplanar = true;
            for point in &points {
                match orient3d_report(
                    &group_carrier[0],
                    &group_carrier[1],
                    &group_carrier[2],
                    point,
                )
                .value()
                {
                    Some(Sign::Zero) => {}
                    Some(Sign::Negative | Sign::Positive) => {
                        is_coplanar = false;
                        break;
                    }
                    None => return Err(ExactArrangementBlocker::UndecidableOrdering),
                }
            }
            if is_coplanar {
                group_index = Some(index);
                break;
            }
        }
        if let Some(index) = group_index {
            groups[index].1.push(boundary);
        } else {
            groups.push((carrier, vec![boundary]));
        }
    }
    Ok(groups.into_iter().map(|(_, loops)| loops).collect())
}

fn map_cap_vertices_to_boundary_or_insert(
    mesh: &ExactMesh,
    boundary_loops: &[Vec<usize>],
    vertices: &mut Vec<Point3>,
    cap_vertices: Vec<Point3>,
) -> Result<Vec<usize>, ExactMeshError> {
    let boundary_vertices = boundary_loops.iter().flatten().copied().collect::<Vec<_>>();
    let mut used_boundary_vertices = vec![false; boundary_vertices.len()];
    let source_vertices = mesh.view().vertices();
    cap_vertices
        .into_iter()
        .map(|point| {
            for (index, &boundary_vertex) in boundary_vertices.iter().enumerate() {
                if used_boundary_vertices[index] {
                    continue;
                }
                let existing = source_vertices.get(boundary_vertex).ok_or_else(|| {
                    ExactMeshError::one(ExactMeshBlocker::new(
                        ExactMeshBlockerKind::StaleFactReplay,
                        "exact coplanar boundary closure cap references a missing boundary vertex",
                    ))
                })?;
                match point3_exact_equal(existing, &point) {
                    Some(true) => {
                        used_boundary_vertices[index] = true;
                        return Ok(boundary_vertex);
                    }
                    Some(false) => {}
                    None => {
                        return Err(ExactMeshError::one(ExactMeshBlocker::new(
                            ExactMeshBlockerKind::ExactConstructionFailure,
                            "exact coplanar boundary closure boundary vertex equality is undecidable",
                        )));
                    }
                }
            }
            for (index, existing) in vertices.iter().enumerate() {
                match point3_exact_equal(existing, &point) {
                    Some(true) => return Ok(index),
                    Some(false) => {}
                    None => {
                        return Err(ExactMeshError::one(ExactMeshBlocker::new(
                            ExactMeshBlockerKind::ExactConstructionFailure,
                            "exact coplanar boundary closure cap vertex equality is undecidable",
                        )));
                    }
                }
            }
            let index = vertices.len();
            vertices.push(point);
            Ok(index)
        })
        .collect()
}

fn point3_lies_strictly_on_segment(start: &Point3, end: &Point3, point: &Point3) -> Option<bool> {
    if point3_exact_equal(point, start)? || point3_exact_equal(point, end)? {
        return Some(false);
    }
    point_on_segment3(start, end, point).value()
}

fn split_output_triangle_edge(
    vertices: &[Point3],
    triangles: &mut Vec<Triangle>,
    split_vertex: usize,
) -> Option<()> {
    let split_point = vertices.get(split_vertex)?;
    let mut triangle_index = 0;
    while triangle_index < triangles.len() {
        let triangle = triangles[triangle_index].0;
        if triangle.contains(&split_vertex) {
            triangle_index += 1;
            continue;
        }
        for edge in 0..3 {
            let a = triangle[edge];
            let b = triangle[(edge + 1) % 3];
            let opposite = triangle[(edge + 2) % 3];
            let a_point = vertices.get(a)?;
            let b_point = vertices.get(b)?;
            if point3_lies_strictly_on_segment(a_point, b_point, split_point)? {
                triangles.splice(
                    triangle_index..triangle_index + 1,
                    [
                        Triangle([a, split_vertex, opposite]),
                        Triangle([split_vertex, b, opposite]),
                    ],
                );
                return Some(());
            }
        }
        triangle_index += 1;
    }
    None
}

fn choose_nonzero_projected_polygon_area(points: &[Point3]) -> Option<CoplanarProjection> {
    for &projection in &[
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        let area = projected_polygon_area2_value(points, projection);
        if compare_reals(&area, &Real::from(0)).value()? != Ordering::Equal {
            return Some(projection);
        }
    }
    None
}

#[derive(Default)]
struct BoundaryLoopSelfContactEvidence {
    repeated_exact_point_pairs: usize,
    exact_points: usize,
    topological_vertices: usize,
    degenerate_cycles: usize,
    nondegenerate_cycles: usize,
}

fn boundary_loop_self_contact_evidence(
    points: &[Point3],
) -> Result<BoundaryLoopSelfContactEvidence, ExactArrangementBlocker> {
    if points.is_empty() {
        return Ok(BoundaryLoopSelfContactEvidence::default());
    }
    let mut exact_point_classes = Vec::<Vec<usize>>::new();
    'points: for (index, point) in points.iter().enumerate() {
        for class in &mut exact_point_classes {
            match point3_exact_equal(&points[class[0]], point) {
                Some(true) => {
                    class.push(index);
                    continue 'points;
                }
                Some(false) => {}
                None => return Err(ExactArrangementBlocker::UndecidableOrdering),
            }
        }
        exact_point_classes.push(vec![index]);
    }

    let mut evidence = BoundaryLoopSelfContactEvidence::default();
    for class in exact_point_classes {
        if class.len() < 2 {
            continue;
        }
        evidence.repeated_exact_point_pairs += class.len() * (class.len() - 1) / 2;
        evidence.exact_points += 1;
        evidence.topological_vertices += class.len();
        for index in 0..class.len() {
            let start = class[index];
            let end = class[(index + 1) % class.len()];
            if cyclic_interval_distinct_items(points, start, end, &|left, right| {
                point3_exact_equal(left, right).ok_or(ExactArrangementBlocker::UndecidableOrdering)
            })? < 3
            {
                evidence.degenerate_cycles += 1;
            } else {
                evidence.nondegenerate_cycles += 1;
            }
        }
    }
    Ok(evidence)
}

fn canonicalize_degenerate_cyclic_self_contact<T: Clone>(
    mut items: Vec<T>,
    equal: &impl Fn(&T, &T) -> Result<bool, ExactArrangementBlocker>,
) -> Result<Vec<T>, ExactArrangementBlocker> {
    loop {
        let mut removed = false;
        'scan: for left in 0..items.len() {
            for right in left + 1..items.len() {
                if equal(&items[left], &items[right])? {
                    if cyclic_interval_distinct_items(&items, left, right, equal)? < 3 {
                        items = remove_degenerate_cyclic_interval(items, left, right);
                        removed = true;
                        break 'scan;
                    }
                    if cyclic_interval_distinct_items(&items, right, left, equal)? < 3 {
                        items = remove_degenerate_cyclic_interval(items, right, left);
                        removed = true;
                        break 'scan;
                    }
                }
            }
        }
        if !removed {
            return Ok(items);
        }
    }
}

fn split_cyclic_self_contact_cycles<T: Clone>(
    items: Vec<T>,
    equal: &impl Fn(&T, &T) -> Result<bool, ExactArrangementBlocker>,
) -> Result<Vec<Vec<T>>, ExactArrangementBlocker> {
    let items = canonicalize_degenerate_cyclic_self_contact(items, equal)?;
    if items.len() < 3 {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    for left in 0..items.len() {
        for right in left + 1..items.len() {
            if equal(&items[left], &items[right])? {
                let left_to_right = cyclic_interval_distinct_items(&items, left, right, equal)?;
                let right_to_left = cyclic_interval_distinct_items(&items, right, left, equal)?;
                if left_to_right < 3 || right_to_left < 3 {
                    return Err(ExactArrangementBlocker::NonManifoldCellComplex);
                }
                let mut split = split_cyclic_self_contact_cycles(
                    cyclic_interval_items(&items, left, right)?,
                    equal,
                )?;
                split.extend(split_cyclic_self_contact_cycles(
                    cyclic_interval_items(&items, right, left)?,
                    equal,
                )?);
                return Ok(split);
            }
        }
    }
    Ok(vec![items])
}

fn cyclic_interval_items<T: Clone>(
    items: &[T],
    start: usize,
    end: usize,
) -> Result<Vec<T>, ExactArrangementBlocker> {
    let span = if end >= start {
        end - start
    } else {
        items.len() - start + end
    };
    let mut interval = Vec::with_capacity(span + 1);
    for offset in 0..=span {
        interval.push(
            items
                .get((start + offset) % items.len())
                .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?
                .clone(),
        );
    }
    Ok(interval)
}

fn remove_degenerate_cyclic_interval<T: Clone>(points: Vec<T>, start: usize, end: usize) -> Vec<T> {
    if points.len() < 2 || start == end {
        return points;
    }
    let mut retained = Vec::with_capacity(points.len().saturating_sub(1));
    retained.push(points[start].clone());
    if end > start {
        retained.extend(points[end + 1..].iter().cloned());
        retained.extend(points[..start].iter().cloned());
    } else {
        retained.extend(points[end + 1..start].iter().cloned());
    }
    retained
}

fn cyclic_interval_distinct_items<T: Clone>(
    items: &[T],
    start: usize,
    end: usize,
    equal: &impl Fn(&T, &T) -> Result<bool, ExactArrangementBlocker>,
) -> Result<usize, ExactArrangementBlocker> {
    let mut distinct = Vec::<T>::new();
    for item in cyclic_interval_items(items, start, end)? {
        let mut already_seen = false;
        for existing in &distinct {
            if equal(existing, &item)? {
                already_seen = true;
                break;
            }
        }
        if !already_seen {
            distinct.push(item);
        }
    }
    Ok(distinct.len())
}

fn exact_loop_is_coplanar(points: &[Point3]) -> Result<bool, ExactArrangementBlocker> {
    if points.len() < 3 {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    let Some(carrier) = exact_loop_carrier(points)? else {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    };
    for point in points {
        match orient3d_report(&carrier[0], &carrier[1], &carrier[2], point).value() {
            Some(Sign::Zero) => {}
            Some(Sign::Negative | Sign::Positive) => return Ok(false),
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    Ok(true)
}

fn exact_loop_carrier(points: &[Point3]) -> Result<Option<[Point3; 3]>, ExactArrangementBlocker> {
    let Some(anchor) = points.first() else {
        return Ok(None);
    };
    for first_index in 1..points.len().saturating_sub(1) {
        for second_index in first_index + 1..points.len() {
            let first = &points[first_index];
            let second = &points[second_index];
            let abx = first.x.clone() - &anchor.x;
            let aby = first.y.clone() - &anchor.y;
            let abz = first.z.clone() - &anchor.z;
            let acx = second.x.clone() - &anchor.x;
            let acy = second.y.clone() - &anchor.y;
            let acz = second.z.clone() - &anchor.z;
            let cross_x = aby.clone() * &acz - &(abz.clone() * &acy);
            let cross_y = abz * &acx - &(abx.clone() * &acz);
            let cross_z = abx * &acy - &(aby * &acx);
            let is_collinear = compare_reals(&cross_x, &Real::from(0))
                .value()
                .ok_or(ExactArrangementBlocker::UndecidableOrdering)?
                == Ordering::Equal
                && compare_reals(&cross_y, &Real::from(0))
                    .value()
                    .ok_or(ExactArrangementBlocker::UndecidableOrdering)?
                    == Ordering::Equal
                && compare_reals(&cross_z, &Real::from(0))
                    .value()
                    .ok_or(ExactArrangementBlocker::UndecidableOrdering)?
                    == Ordering::Equal;
            if !is_collinear {
                return Ok(Some([anchor.clone(), first.clone(), second.clone()]));
            }
        }
    }
    Ok(None)
}

#[derive(Clone, Copy, Default)]
struct BoundaryTopologyEvidence {
    invalid_outgoing_degree_vertices: usize,
    invalid_incoming_degree_vertices: usize,
    overused_edges: usize,
}

fn retained_triangle_edge_uses(mesh: MeshView<'_>) -> BTreeMap<[usize; 2], Vec<(usize, usize)>> {
    let mut edge_uses: BTreeMap<[usize; 2], Vec<(usize, usize)>> = BTreeMap::new();
    for triangle in mesh.triangles() {
        let [a, b, c] = triangle.vertex_indices();
        for (start, end) in [(a, b), (b, c), (c, a)] {
            let key = if start < end {
                [start, end]
            } else {
                [end, start]
            };
            edge_uses.entry(key).or_default().push((start, end));
        }
    }
    edge_uses
}

fn directed_boundary_edges(mesh: MeshView<'_>) -> BTreeMap<[usize; 2], (usize, usize)> {
    retained_triangle_edge_uses(mesh)
        .into_iter()
        .filter_map(|(key, uses)| (uses.len() == 1).then(|| uses[0]).map(|edge| (key, edge)))
        .collect::<BTreeMap<_, _>>()
}

fn orient_cap_group_against_mesh_boundary(
    mesh_boundary_edges: &BTreeMap<[usize; 2], (usize, usize)>,
    triangles: Vec<Triangle>,
) -> Option<Vec<Triangle>> {
    let mut edge_uses: BTreeMap<[usize; 2], Vec<(usize, usize)>> = BTreeMap::new();
    for triangle in &triangles {
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

    let mut same_as_mesh = false;
    let mut opposite_mesh = false;
    for (key, uses) in edge_uses {
        if uses.len() != 1 {
            continue;
        }
        let Some(&(mesh_start, mesh_end)) = mesh_boundary_edges.get(&key) else {
            continue;
        };
        let (cap_start, cap_end) = uses[0];
        if (cap_start, cap_end) == (mesh_start, mesh_end) {
            same_as_mesh = true;
        } else if (cap_start, cap_end) == (mesh_end, mesh_start) {
            opposite_mesh = true;
        } else {
            return None;
        }
    }

    match (same_as_mesh, opposite_mesh) {
        (false, true) => Some(triangles),
        (true, false) => Some(
            triangles
                .into_iter()
                .map(|triangle| {
                    let [a, b, c] = triangle.0;
                    Triangle([a, c, b])
                })
                .collect(),
        ),
        _ => None,
    }
}

fn directed_boundary_loops(
    mesh: MeshView<'_>,
) -> Result<Vec<Vec<usize>>, BoundaryTopologyEvidence> {
    let edge_uses = retained_triangle_edge_uses(mesh);
    let mut next_by_start = BTreeMap::new();
    let mut outgoing = BTreeMap::<usize, usize>::new();
    let mut incoming = BTreeMap::<usize, usize>::new();
    let mut boundary_vertices = BTreeSet::<usize>::new();
    let mut boundary_edge_count = 0_usize;
    let mut duplicate_outgoing = false;
    let mut overused_edges = 0;
    for uses in edge_uses.values() {
        if uses.len() == 1 {
            let (start, end) = uses[0];
            *outgoing.entry(start).or_default() += 1;
            *incoming.entry(end).or_default() += 1;
            boundary_vertices.insert(start);
            boundary_vertices.insert(end);
            if next_by_start.insert(start, end).is_some() {
                duplicate_outgoing = true;
            }
            boundary_edge_count += 1;
        } else if uses.len() > 2 {
            overused_edges += 1;
        }
    }

    let topology = BoundaryTopologyEvidence {
        invalid_outgoing_degree_vertices: boundary_vertices
            .iter()
            .filter(|&&vertex| outgoing.get(&vertex).copied().unwrap_or(0) != 1)
            .count(),
        invalid_incoming_degree_vertices: boundary_vertices
            .iter()
            .filter(|&&vertex| incoming.get(&vertex).copied().unwrap_or(0) != 1)
            .count(),
        overused_edges,
    };
    if duplicate_outgoing
        || overused_edges > 0
        || boundary_edge_count < 3
        || next_by_start
            .keys()
            .any(|start| incoming.get(start).copied().unwrap_or(0) != 1)
    {
        return Err(topology);
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
                return Err(topology);
            }
            loop_vertices.push(current);
            current = *next_by_start.get(&current).ok_or(topology)?;
            if current == start {
                break;
            }
        }
        if current != start || loop_vertices.len() < 3 {
            return Err(topology);
        }
        loops.push(loop_vertices);
    }
    if used_starts.len() != boundary_edge_count || loops.is_empty() {
        return Err(topology);
    }
    Ok(loops)
}

fn materialize_simple_coplanar_overlay_arrangement(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: Option<ExactMeshValidationPolicy>,
    arrangement: &ExactArrangement3d,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if arrangement.carrier_plane_overlays.len() != 1
        || !arrangement.face_plane_arrangements.is_empty()
    {
        return Ok(None);
    }
    let Some(validation) = validation else {
        return Ok(None);
    };
    let overlay = &arrangement.carrier_plane_overlays[0];
    let Some(set_operation) = operation.coplanar_overlay_set_operation() else {
        return Ok(None);
    };
    let allow_empty = operation.coplanar_overlay_allows_empty();
    let left_ring = projected_mesh_face_ring(
        ExactArrangement2dRegion::Left,
        left,
        overlay.left_face,
        overlay.projection,
    )?;
    let right_ring = projected_mesh_face_ring(
        ExactArrangement2dRegion::Right,
        right,
        overlay.right_face,
        overlay.projection,
    )?;
    let (Some(left_ring), Some(right_ring)) = (left_ring, right_ring) else {
        return Ok(None);
    };
    let requested_overlay = build_exact_arrangement2d_overlay_with_boundary_policy(
        &[left_ring, right_ring],
        set_operation,
        ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
    );
    if !requested_overlay.blockers.is_empty()
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
                operation,
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
    )?
    else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &mesh,
        "exact coplanar overlay arrangement boolean result",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    )))
}

pub(crate) fn boolean_coplanar_mesh_overlay_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let Some(plan) = coplanar_mesh_overlay_plan(left, right, operation) else {
        return Ok(None);
    };
    let Some(mesh) = materialize_coplanar_mesh_overlay_mesh(
        left,
        right,
        plan.set_operation,
        plan.boundary_policy,
        "exact coplanar mesh overlay arrangement",
        plan.allow_empty,
    )?
    else {
        return Ok(None);
    };
    let mesh = copy_mesh(
        &mesh,
        "exact coplanar mesh overlay arrangement boolean result",
        validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    )))
}

fn coplanar_mesh_overlay_candidate_counts(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<(usize, usize)> {
    let plan = coplanar_mesh_overlay_plan(left, right, operation)?;
    materialize_coplanar_mesh_overlay_mesh(
        left,
        right,
        plan.set_operation,
        plan.boundary_policy,
        "exact coplanar mesh overlay arrangement",
        plan.allow_empty,
    )
    .ok()
    .flatten()
    .map(|mesh| (mesh.vertices().len(), mesh.triangles().len()))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct CoplanarMeshOverlayPlan {
    set_operation: ExactArrangement2dSetOperation,
    boundary_policy: ExactArrangement2dBoundaryPolicy,
    allow_empty: bool,
}

fn coplanar_mesh_overlay_plan(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<CoplanarMeshOverlayPlan> {
    if left.facts().mesh.closed_manifold && right.facts().mesh.closed_manifold {
        return None;
    }
    let set_operation = operation.coplanar_overlay_set_operation()?;
    let allow_empty = operation.coplanar_overlay_allows_empty();
    let materialized_boundary_policy = [
        ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
        ExactArrangement2dBoundaryPolicy::PreserveCollinear,
    ]
    .into_iter()
    .find(|&boundary_policy| {
        matches!(
            materialize_coplanar_mesh_overlay_mesh(
                left,
                right,
                set_operation,
                boundary_policy,
                "exact coplanar mesh overlay arrangement",
                allow_empty,
            ),
            Ok(Some(_))
        )
    })?;
    let boundary_policy =
        operation.coplanar_overlay_boundary_policy(materialized_boundary_policy)?;
    Some(CoplanarMeshOverlayPlan {
        set_operation,
        boundary_policy,
        allow_empty,
    })
}

pub(crate) fn materialize_coplanar_mesh_overlay_mesh(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactArrangement2dSetOperation,
    boundary_policy: ExactArrangement2dBoundaryPolicy,
    provenance: &'static str,
    allow_empty: bool,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    let Some((carrier_points, projection)) = coplanar_mesh_overlay_carrier(left, right)? else {
        return Ok(None);
    };
    let mut rings = Vec::with_capacity(left.triangles().len() + right.triangles().len());
    let Some(left_rings) =
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Left, left, projection)?
    else {
        return Ok(None);
    };
    rings.extend(left_rings);
    let Some(right_rings) =
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Right, right, projection)?
    else {
        return Ok(None);
    };
    rings.extend(right_rings);
    let overlay =
        build_exact_arrangement2d_overlay_with_boundary_policy(&rings, operation, boundary_policy);
    if !overlay.blockers.is_empty() && !overlay_allows_selected_face_materialization(&overlay) {
        return Ok(None);
    }
    if !overlay.faces.iter().any(|face| face.selected) {
        if allow_empty {
            return ExactMesh::new_with_policy_and_version(
                Vec::new(),
                Vec::new(),
                SourceProvenance::exact(provenance),
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
                1,
            )
            .map(Some);
        }
        return Ok(None);
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
                    | ExactArrangement2dBlocker::NonManifoldSelectedBoundary { .. }
            )
        })
}

fn mesh_from_selected_projected_overlay_faces(
    overlay: &ExactArrangement2dOverlay,
    carrier_points: &[Point3; 3],
    projection: CoplanarProjection,
    provenance: &'static str,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    match mesh_from_projected_overlay_output_components(
        overlay,
        carrier_points,
        projection,
        provenance,
    )? {
        Some(mesh) => Ok(Some(mesh)),
        None if !overlay.output_components.is_empty() => Ok(None),
        None if overlay_allows_selected_face_materialization(overlay) => {
            mesh_from_projected_overlay_selected_faces(
                overlay,
                carrier_points,
                projection,
                provenance,
            )
        }
        None => Ok(None),
    }
}

fn mesh_from_projected_overlay_output_components(
    overlay: &ExactArrangement2dOverlay,
    carrier_points: &[Point3; 3],
    projection: CoplanarProjection,
    provenance: &'static str,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    if overlay.output_components.is_empty() {
        return Ok(None);
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
                let loop_ = overlay.output_loops.get(loop_index).ok_or_else(|| {
                    ExactMeshError::one(ExactMeshBlocker::new(
                        ExactMeshBlockerKind::StaleFactReplay,
                        "exact coplanar output component references a missing output loop",
                    ))
                })?;
                if loop_.points.len() < 3 {
                    return Ok(None);
                }
                Ok(lift_projected_points_to_carrier(
                    loop_.points.iter(),
                    carrier_points,
                    projection,
                ))
            })
            .collect::<Result<Option<Vec<_>>, _>>()?;
        let Some(lifted_loops) = lifted_loops else {
            return Ok(None);
        };

        let mut component_vertices = Vec::new();
        let mut component_triangles = Vec::new();
        triangulate_exact_loop_group(
            &lifted_loops,
            &mut component_vertices,
            &mut component_triangles,
        )
        .map_err(|blocker| {
            arrangement_blocker_error(
                "exact coplanar output-component triangulation failed",
                blocker,
            )
        })?;
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
        return Ok(None);
    }
    ExactMesh::new_with_policy_and_version(
        vertices,
        triangles,
        SourceProvenance::exact(provenance),
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        1,
    )
    .map(Some)
}

fn mesh_from_projected_overlay_selected_faces(
    overlay: &ExactArrangement2dOverlay,
    carrier_points: &[Point3; 3],
    projection: CoplanarProjection,
    provenance: &'static str,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for overlay_face in overlay.faces.iter().filter(|face| face.selected) {
        let Some(face) = overlay.arrangement.faces.get(overlay_face.face) else {
            return Ok(None);
        };
        let boundary_points = face
            .vertices
            .iter()
            .map(|vertex| {
                overlay
                    .arrangement
                    .vertices
                    .get(*vertex)
                    .map(|vertex| &vertex.point)
                    .ok_or_else(|| {
                        ExactMeshError::one(ExactMeshBlocker::new(
                            ExactMeshBlockerKind::StaleFactReplay,
                            "exact coplanar selected-face references a missing arrangement vertex",
                        ))
                    })
            })
            .collect::<Result<Vec<_>, _>>()?;
        let boundary =
            lift_projected_points_to_carrier(boundary_points, carrier_points, projection);
        let Some(boundary) = boundary else {
            return Ok(None);
        };
        let mut face_vertices = Vec::new();
        let mut face_triangles = Vec::new();
        triangulate_exact_loop_group(&[boundary], &mut face_vertices, &mut face_triangles)
            .map_err(|blocker| {
                arrangement_blocker_error(
                    "exact coplanar selected-face triangulation failed",
                    blocker,
                )
            })?;
        let face_to_mesh = face_vertices
            .into_iter()
            .map(|point| intern_coplanar_output_vertex(&mut vertices, point))
            .collect::<Result<Vec<_>, _>>()?;
        triangles.extend(face_triangles.into_iter().map(|triangle| {
            Triangle([
                face_to_mesh[triangle.0[0]],
                face_to_mesh[triangle.0[1]],
                face_to_mesh[triangle.0[2]],
            ])
        }));
    }
    if triangles.is_empty() {
        return Ok(None);
    }
    ExactMesh::new_with_policy_and_version(
        vertices,
        triangles,
        SourceProvenance::exact(provenance),
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        1,
    )
    .map(Some)
}

fn intern_coplanar_output_vertex(
    vertices: &mut Vec<Point3>,
    point: Point3,
) -> Result<usize, ExactMeshError> {
    for (index, existing) in vertices.iter().enumerate() {
        match point3_exact_equal(existing, &point) {
            Some(true) => return Ok(index),
            Some(false) => {}
            None => {
                return Err(ExactMeshError::one(ExactMeshBlocker::new(
                    ExactMeshBlockerKind::ExactConstructionFailure,
                    "exact coplanar output vertex equality is undecidable",
                )));
            }
        }
    }
    let index = vertices.len();
    vertices.push(point);
    Ok(index)
}

fn lift_projected_points_to_carrier<'a>(
    points: impl IntoIterator<Item = &'a Point2>,
    carrier_points: &[Point3; 3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    points
        .into_iter()
        .map(|point| lift_projected_point_to_carrier(point, carrier_points, projection))
        .collect()
}

pub(crate) fn coplanar_mesh_overlay_carrier(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<([Point3; 3], CoplanarProjection)>, ExactMeshError> {
    let mut carrier_points = None;
    'meshes: for mesh in [left, right] {
        for triangle in mesh.view().triangles() {
            let points = triangle.vertices()?.map(|point| point.clone());
            if choose_nonzero_projected_polygon_area(&points).is_some() {
                carrier_points = Some(points);
                break 'meshes;
            }
        }
    }
    let Some(carrier_points) = carrier_points else {
        return Ok(None);
    };
    let Some(projection) = choose_nonzero_projected_polygon_area(&carrier_points) else {
        return Ok(None);
    };
    for mesh in [left, right] {
        for point in mesh.vertices() {
            match orient3d_report(
                &carrier_points[0],
                &carrier_points[1],
                &carrier_points[2],
                point,
            )
            .value()
            {
                Some(Sign::Zero) => {}
                Some(Sign::Negative | Sign::Positive) | None => return Ok(None),
            }
        }
        for face in 0..mesh.triangles().len() {
            let Some(ring) =
                projected_mesh_face_ring(ExactArrangement2dRegion::Left, mesh, face, projection)?
            else {
                return Ok(None);
            };
            let mut area = Real::from(0);
            for index in 0..ring.vertices.len() {
                let current = &ring.vertices[index];
                let next = &ring.vertices[(index + 1) % ring.vertices.len()];
                area += &(current.x.clone() * &next.y) - &(current.y.clone() * &next.x);
            }
            match compare_reals(&area, &Real::from(0)).value() {
                Some(Ordering::Less | Ordering::Greater) => {}
                Some(Ordering::Equal) | None => return Ok(None),
            }
        }
    }
    Ok(Some((carrier_points, projection)))
}

fn projected_mesh_boundary_rings(
    region: ExactArrangement2dRegion,
    mesh: &ExactMesh,
    projection: CoplanarProjection,
) -> Result<Option<Vec<ExactArrangement2dRegionRing>>, ExactMeshError> {
    let vertices = mesh.view().vertices();
    if mesh.facts().mesh.boundary_edges == 0 {
        return Ok(None);
    }
    let boundary_loops = directed_boundary_loops(mesh.view()).map_err(|topology| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::StaleFactReplay,
            format!(
                "exact coplanar overlay retained boundary topology is not loop-shaped: \
                 invalid_outgoing={}, invalid_incoming={}, overused_edges={}",
                topology.invalid_outgoing_degree_vertices,
                topology.invalid_incoming_degree_vertices,
                topology.overused_edges
            ),
        ))
    })?;
    boundary_loops
        .into_iter()
        .map(|loop_vertices| {
            let vertices = loop_vertices
                .into_iter()
                .map(|vertex| {
                    vertices
                        .get(vertex)
                        .map(|point| project_point3(point, projection))
                        .ok_or_else(|| {
                            ExactMeshError::one(
                                ExactMeshBlocker::new(
                                    ExactMeshBlockerKind::StaleFactReplay,
                                    "exact coplanar overlay boundary references a missing vertex",
                                )
                                .with_vertex(vertex),
                            )
                        })
                })
                .collect::<Result<Vec<_>, _>>()?;
            Ok(ExactArrangement2dRegionRing { region, vertices })
        })
        .collect::<Result<Vec<_>, _>>()
        .map(Some)
}

fn projected_mesh_face_ring(
    region: ExactArrangement2dRegion,
    mesh: &ExactMesh,
    face: usize,
    projection: CoplanarProjection,
) -> Result<Option<ExactArrangement2dRegionRing>, ExactMeshError> {
    let Ok(face_ref) = mesh.view().face(face) else {
        return Err(ExactMeshError::one(
            ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                "exact coplanar overlay face ring references a missing face",
            )
            .with_face(face),
        ));
    };
    let vertices = face_ref
        .vertices()?
        .into_iter()
        .map(|vertex| project_point3(vertex, projection))
        .collect::<Vec<_>>();
    Ok(Some(ExactArrangement2dRegionRing { region, vertices }))
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
    let p1x = carrier[1].x.clone() - &carrier[0].x;
    let p1y = carrier[1].y.clone() - &carrier[0].y;
    let p1z = carrier[1].z.clone() - &carrier[0].z;
    let p2x = carrier[2].x.clone() - &carrier[0].x;
    let p2y = carrier[2].y.clone() - &carrier[0].y;
    let p2z = carrier[2].z.clone() - &carrier[0].z;
    Some(Point3::new(
        carrier[0].x.clone() + &(p1x * &a) + &(p2x * &b),
        carrier[0].y.clone() + &(p1y * &a) + &(p2y * &b),
        carrier[0].z.clone() + &(p1z * &a) + &(p2z * &b),
    ))
}

fn boolean_arrangement_cell_complex_recovery(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if let Some(result) =
        boolean_arrangement_orthogonal_solid_cell_recovery(left, right, operation, validation)?
    {
        return Ok(Some(result));
    }
    let Some(affine_operation) = operation.affine_orthogonal_solid_operation() else {
        return Ok(None);
    };
    let Some(arrangement) =
        materialize_affine_orthogonal_solid_operation(left, right, affine_operation, validation)?
    else {
        return Ok(None);
    };
    arrangement.validate_against_sources(left, right)?;
    let result = certified_shortcut_result(
        arrangement.mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    validate_boolean_result(
        &result,
        "exact arrangement affine orthogonal solid recovery result validation failed",
    )?;
    Ok(Some(result))
}

fn boolean_arrangement_orthogonal_solid_cell_recovery(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let Some(solid_operation) = operation.axis_aligned_orthogonal_solid_operation() else {
        return Ok(None);
    };
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
    let Some(mesh) = materialize_axis_aligned_orthogonal_solid_cell_output(
        left,
        right,
        solid_operation,
        label,
        validation,
    )?
    else {
        return Ok(None);
    };
    let result = certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    validate_boolean_result(
        &result,
        "exact arrangement orthogonal solid recovery result validation failed",
    )?;
    Ok(Some(result))
}

pub(crate) fn materialize_open_surface_disjoint_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactBooleanResult, ExactMeshError> {
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
        ExactBooleanOperation::SelectedRegions(_) => {
            return Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::UnsupportedExactOperation,
                format!(
                    "open-surface disjoint materialization requires a named boolean operation: {operation:?}"
                ),
            )));
        }
    };

    Ok(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::OpenSurfaceDisjoint,
    ))
}

fn boolean_open_surface_disjoint_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || meshes_are_certified_bounds_disjoint(left, right)
        || closed_validation_regularized_solid_support(left, right, operation, validation).is_some()
    {
        return Ok(None);
    }
    let disjoint_report = open_surface_disjoint_report_from_graph(graph, left, right);
    if disjoint_report.status == ExactOpenSurfaceDisjointStatus::Certified {
        if !graph_source_replay_certificate_is_current(graph, left, right)? {
            return Ok(None);
        }
        let result = materialize_open_surface_disjoint_meshes(left, right, operation, validation)?;
        disjoint_report
            .validate_against_sources(left, right)
            .map_err(|error| {
                retained_evidence_validation_error(
                    "exact open-surface disjoint report/source replay failed",
                    error,
                    ExactMeshBlockerKind::ExactConstructionFailure,
                )
            })?;
        if !matches!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: result_operation,
                shortcut: ExactBooleanShortcutKind::OpenSurfaceDisjoint,
            } if result_operation == operation
        ) {
            return Err(boolean_validation_error(
                ExactMeshBlockerKind::ExactConstructionFailure,
                "exact open-surface disjoint result kind mismatch",
                result.kind,
            ));
        }
        validate_boolean_result(
            &result,
            "exact open-surface disjoint result validation failed",
        )?;
        return Ok(Some(result));
    }
    Ok(None)
}

pub(crate) fn open_surface_disjoint_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactOpenSurfaceDisjointReport {
    let left_open_surface = mesh_is_open_surface(left);
    let right_open_surface = mesh_is_open_surface(right);
    let open_surface_pair = left_open_surface && right_open_surface;
    let graph_counts = if open_surface_pair {
        retained_graph_counts(graph)
    } else {
        RetainedGraphCounts::empty()
    };
    let counts = if open_surface_pair {
        ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding)
    } else {
        ExactBooleanBlocker::default()
    };
    let status = if !open_surface_pair {
        ExactOpenSurfaceDisjointStatus::NotOpenSurface
    } else if graph_counts.graph_had_unknowns {
        ExactOpenSurfaceDisjointStatus::GraphUnknowns
    } else if graph_counts.retained_face_pairs == 0 {
        ExactOpenSurfaceDisjointStatus::Certified
    } else {
        ExactOpenSurfaceDisjointStatus::GraphHasFacePairs
    };
    let blocker_kind = counts.inferred_kind();
    graph_counts.into_open_surface_disjoint_report(
        status,
        left_open_surface,
        right_open_surface,
        counts.into_blocker(blocker_kind),
    )
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
    let left_closed = !left.triangles().is_empty() && left.facts().mesh.closed_manifold;
    let right_closed = !right.triangles().is_empty() && right.facts().mesh.closed_manifold;
    let left_open_surface = mesh_is_open_surface(left);
    let right_open_surface = mesh_is_open_surface(right);
    if (left_closed && right_open_surface) || (left_open_surface && right_closed) {
        Some(ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid)
    } else {
        None
    }
}

fn closed_validation_regularized_solid_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Option<ExactBooleanSupport> {
    if validation != ExactMeshValidationPolicy::CLOSED
        || matches!(operation, ExactBooleanOperation::SelectedRegions(_))
    {
        return None;
    }
    if !left.triangles().is_empty()
        && !right.triangles().is_empty()
        && let (Some(left_kind), Some(right_kind)) = (
            closed_regularized_operand_kind(left),
            closed_regularized_operand_kind(right),
        )
        && matches!(left_kind, ClosedRegularizedOperandKind::LowerDimensional)
        && matches!(right_kind, ClosedRegularizedOperandKind::LowerDimensional)
    {
        return Some(ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid);
    }
    certified_mixed_dimensional_regularized_solid_support(left, right)
}

/// Retained split-region artifacts that certify an open-surface arrangement.
#[derive(Clone, Debug)]
struct OpenSurfaceArrangementPlan {
    support: ExactBooleanSupport,
    region_classifications: Vec<FaceRegionPlaneClassification>,
    triangulations: Vec<FaceRegionTriangulation>,
}

pub(crate) fn open_surface_arrangement_result_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let Some(plan) = open_surface_arrangement_plan_from_graph(graph, left, right, operation)?
    else {
        return Ok(None);
    };
    materialize_open_surface_arrangement_plan(
        left,
        right,
        operation,
        validation,
        graph.has_unknowns(),
        plan,
    )
}

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
    validation: ExactMeshValidationPolicy,
    graph_had_unknowns: bool,
    plan: OpenSurfaceArrangementPlan,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let OpenSurfaceArrangementPlan {
        support: _,
        region_classifications,
        triangulations,
    } = plan;
    let Some(selection) = operation.open_surface_region_selection() else {
        return Ok(None);
    };
    // Open-surface arrangement is not a closed-volumetric inside/outside
    // split regions are retained by surface operation, and no winding label is
    // invented for a mesh that has no closed volume.
    let Ok((assembly, mesh)) = assemble_region_selection_mesh(
        &triangulations,
        left,
        right,
        selection,
        validation,
        "open-surface arrangement assembly failed",
        "open-surface arrangement assembly canonicalization failed",
    ) else {
        return Ok(None);
    };
    let result = ExactBooleanResult {
        kind: ExactBooleanResultKind::OpenSurfaceArrangement { operation },
        graph_had_unknowns,
        region_classifications,
        triangulations,
        assembly,
        volumetric_classifications: Vec::new(),
        topology_assembly_report: None,
        region_ownership_report: None,
        mesh,
    };
    result.validate().map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "open-surface arrangement validation failed",
            error,
        )
    })?;
    Ok(Some(result))
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
) -> Result<Option<OpenSurfaceArrangementPlan>, ExactMeshError> {
    let Some(support) = operation.open_surface_arrangement_support() else {
        return Ok(None);
    };
    if !mesh_is_open_surface(left) || !mesh_is_open_surface(right) {
        return Ok(None);
    }
    let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    // Endpoint, edge-only, and coplanar contacts need separate topology
    // policies. This keeps open-surface arrangement tied to exact proper
    // segment/plane construction facts rather than a tolerance-style overlap.
    let has_proper_surface_crossing = graph.face_pairs.iter().any(|pair| {
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
    });
    if graph.has_unknowns()
        || graph.face_pairs.is_empty()
        || counts.unknown_pairs != 0
        || counts.construction_failed_events != 0
        || counts.coplanar_overlapping_pairs != 0
        || counts.coplanar_touching_pairs != 0
        || !has_proper_surface_crossing
    {
        return Ok(None);
    }

    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
    if region_classifications.iter().any(|classification| {
        !classification.all_proof_producing()
            || matches!(classification.relation, FaceRegionPlaneRelation::Unknown)
    }) {
        return Ok(None);
    }
    let triangulations = checked_triangulate_face_regions_with_earcut(&region_plan, left, right)
        .map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::DegenerateTriangle,
                format!("open-surface arrangement triangulation failed: {error}"),
            ))
        })?;
    Ok(Some(OpenSurfaceArrangementPlan {
        support,
        region_classifications,
        triangulations,
    }))
}

pub(crate) fn boolean_same_surface_meshes(
    mesh: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactBooleanResult, ExactMeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection => {
            copy_mesh(mesh, "exact same-surface boolean result", validation)?
        }
        ExactBooleanOperation::Difference => {
            empty_mesh("empty exact same-surface difference", validation)?
        }
        ExactBooleanOperation::SelectedRegions(_) => {
            return Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::UnsupportedExactOperation,
                format!(
                    "same-surface materialization requires a named boolean operation: {operation:?}"
                ),
            )));
        }
    };

    Ok(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::SameSurface,
    ))
}

fn certified_closed_boundary_touching_regularized_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ExactBoundaryTouchingReport>, ExactMeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(None);
    }
    let report = boundary_touching_report_from_graph(graph, left, right)?;
    if report.status != ExactBoundaryTouchingStatus::Certified {
        return Ok(None);
    }
    report.validate().map_err(|error| {
        boolean_validation_error(
            ExactMeshBlockerKind::ExactConstructionFailure,
            "exact closed-boundary-touch report validation failed",
            error,
        )
    })?;
    Ok(Some(report))
}

pub(crate) fn materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>, ExactMeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(None);
    }
    let Some(evidence) = coplanar_volumetric_evidence_from_graph(graph, left, right)?
        .filter(coplanar_evidence_is_zero_area_boundary_only)
    else {
        return Ok(None);
    };
    if operation.is_selected_regions() {
        return Ok(None);
    }
    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact closed-boundary-touching union preserving separate shells",
            validation,
        )?,
        ExactBooleanOperation::Intersection => empty_mesh(
            "empty exact closed-boundary-touching intersection",
            validation,
        )?,
        ExactBooleanOperation::Difference => copy_mesh(
            left,
            "exact closed-boundary-touching difference keeps left",
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    let Some(shortcut) = operation.closed_boundary_touching_shortcut() else {
        return Ok(None);
    };
    let result = certified_shortcut_result(mesh, operation, shortcut);
    validate_boolean_result(
        &result,
        "exact closed-boundary-touching result validation failed",
    )?;
    validate_coplanar_volumetric_evidence_against_sources(
        &evidence,
        left,
        right,
        "exact closed-boundary-touching evidence failed source replay",
    )?;
    Ok(Some((result, evidence)))
}

fn winding_evidence_report_for_request_from_graph_and_attempt(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<ExactWindingEvidenceReport, ExactMeshError> {
    if request.validation == ExactMeshValidationPolicy::ALLOW_BOUNDARY {
        return winding_evidence_report_from_graph_with_facts(
            graph,
            left,
            right,
            request.operation,
            shortcut_facts,
        );
    }

    let operation = request.operation;
    let validation = request.validation;
    let retained_arrangement_attempt_materializes_output =
        if let Some(attempt) = retained_arrangement_attempt {
            attempt
                .validate_for_request_policy(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .is_ok()
                && attempt.materialized_arrangement_cell_complex_output()
        } else {
            false
        };
    if retained_arrangement_attempt_materializes_output {
        return Ok(
            arrangement_cell_complex_already_materialized_winding_evidence(
                graph, left, right, operation,
            )?,
        );
    }
    let closed_regularized_support =
        closed_validation_regularized_solid_support(left, right, operation, validation);
    let defer_lower_dimensional_intersection = closed_regularized_support
        == Some(ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid)
        && operation == ExactBooleanOperation::Intersection;
    let evidence = if let Some(support) =
        closed_regularized_support.filter(|_| !defer_lower_dimensional_intersection)
    {
        let status = match support {
            ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid => {
                ExactWindingEvidenceStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized
            }
            ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid => {
                ExactWindingEvidenceStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized
            }
            _ => {
                return Err(ExactMeshError::one(ExactMeshBlocker::new(
                    ExactMeshBlockerKind::ExactConstructionFailure,
                    "closed validation gate returned unsupported winding evidence support",
                )));
            }
        };
        RetainedGraphCounts::empty().into_winding_evidence_report(
            operation,
            status,
            0,
            Vec::new(),
            ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::Winding),
            None,
            None,
        )
    } else {
        let evidence = winding_evidence_report_from_graph_with_facts(
            graph,
            left,
            right,
            operation,
            shortcut_facts,
        )?;
        if validation == ExactMeshValidationPolicy::CLOSED
            || matches!(operation, ExactBooleanOperation::SelectedRegions(_))
            || !matches!(
                evidence.status,
                ExactWindingEvidenceStatus::VolumetricAssemblyRequired
                    | ExactWindingEvidenceStatus::CoplanarVolumetricCellsRequired
            )
        {
            evidence
        } else if materialize_arrangement_volumetric_split_cell_result_from_graph(
            graph, left, right, operation, validation,
        )?
        .is_some()
        {
            arrangement_cell_complex_already_materialized_winding_evidence(
                graph, left, right, operation,
            )?
        } else {
            evidence
        }
    };
    Ok(evidence)
}

fn arrangement_cell_complex_already_materialized_winding_evidence(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactWindingEvidenceReport, ExactMeshError> {
    let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    let (blocker_kind, coplanar_evidence) =
        arrangement_materialized_evidence_blocker_kind_and_evidence(graph, left, right)?;
    Ok(retained_graph_counts(graph).into_winding_evidence_report(
        operation,
        ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
        0,
        Vec::new(),
        counts.into_blocker(blocker_kind),
        None,
        coplanar_evidence,
    ))
}

fn arrangement_materialized_evidence_blocker_kind_and_evidence(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<
    (
        ExactBooleanBlockerKind,
        Option<CoplanarVolumetricCellEvidenceReport>,
    ),
    ExactMeshError,
> {
    let coplanar_evidence = coplanar_volumetric_evidence_from_graph(graph, left, right)?
        .filter(coplanar_evidence_certifies_arrangement_cell_complex);
    let blocker_kind = match coplanar_evidence.as_ref().map(|evidence| evidence.obstacle) {
        Some(CoplanarVolumetricCellObstacle::BoundaryOnlyContact) => {
            ExactBooleanBlockerKind::BoundaryOnlyContact
        }
        Some(
            CoplanarVolumetricCellObstacle::NeedsCoplanarVolumetricCells
            | CoplanarVolumetricCellObstacle::MixedCoplanarAndCrossingCells,
        ) => ExactBooleanBlockerKind::CoplanarVolumetricCells,
        _ if graph_has_only_boundary_contact_pairs(graph, left, right) => {
            ExactBooleanBlockerKind::BoundaryOnlyContact
        }
        _ if coplanar_volumetric_evidence_from_graph(graph, left, right)?
            .is_some_and(|evidence| coplanar_evidence_requires_volumetric_cells(&evidence)) =>
        {
            ExactBooleanBlockerKind::CoplanarVolumetricCells
        }
        _ => ExactBooleanBlockerKind::Winding,
    };
    Ok((blocker_kind, coplanar_evidence))
}

fn arrangement_cell_complex_preflight_materialized_winding_evidence(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    counts: ExactBooleanBlocker,
    arrangement_cell_complex_shortcut_materializes: bool,
) -> Result<ExactWindingEvidenceReport, ExactMeshError> {
    let (blocker_kind, mut coplanar_evidence) =
        arrangement_materialized_evidence_blocker_kind_and_evidence(graph, left, right)?;
    let blocker_kind = if arrangement_cell_complex_shortcut_materializes {
        coplanar_evidence = None;
        ExactBooleanBlockerKind::Winding
    } else {
        blocker_kind
    };
    let blocker = counts.into_blocker(blocker_kind);
    let (graph_counts, blocker, coplanar_evidence) = if coplanar_evidence.is_some()
        || blocker
            .validate_for_kind(ExactBooleanBlockerKind::Winding)
            .is_ok()
    {
        (retained_graph_counts(graph), blocker, coplanar_evidence)
    } else {
        (
            RetainedGraphCounts::empty(),
            ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::Winding),
            None,
        )
    };
    Ok(graph_counts.into_winding_evidence_report(
        operation,
        ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
        0,
        Vec::new(),
        blocker,
        None,
        coplanar_evidence,
    ))
}

/// Validate retained graph handles against their source meshes.
///
/// Boolean preflight and materialization must reject a retained graph whose
/// face, edge, vertex, or plane handles no longer replay against the source
/// meshes. Predicate evidence is only useful when the combinatorial object
/// handles attached to it are still current.
fn validate_graph_source_replay(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<(), ExactMeshError> {
    graph
        .validate_against_sources(left, right)
        .map_err(|error| {
            boolean_validation_error(
                ExactMeshBlockerKind::StaleFactReplay,
                "retained exact intersection graph failed source replay",
                error,
            )
        })
}

/// Return whether a retained graph carries a current source-replay certificate.
///
/// The source handles are still audited first. A graph that replays but lacks
/// the cheap current certificate may be used for evidence collection, but
/// shortcut materializers that require pre-certified retained facts must decline.
fn graph_source_replay_certificate_is_current(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactMeshError> {
    validate_graph_source_replay(graph, left, right)?;
    Ok(graph.source_replay_validated)
}

pub(crate) fn boundary_touching_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactBoundaryTouchingReport, ExactMeshError> {
    let graph_counts = retained_graph_counts(graph);
    let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    let status = if graph_counts.graph_had_unknowns {
        ExactBoundaryTouchingStatus::GraphUnknowns
    } else if graph_requires_boundary_only_contact(graph, left, right)? {
        ExactBoundaryTouchingStatus::Certified
    } else {
        ExactBoundaryTouchingStatus::NotBoundaryOnly
    };
    let blocker_kind = match status {
        ExactBoundaryTouchingStatus::GraphUnknowns => ExactBooleanBlockerKind::Refinement,
        ExactBoundaryTouchingStatus::Certified => ExactBooleanBlockerKind::BoundaryOnlyContact,
        ExactBoundaryTouchingStatus::NotBoundaryOnly => counts.inferred_kind(),
    };
    Ok(graph_counts.into_boundary_touching_report(status, counts.into_blocker(blocker_kind)))
}

fn planar_arrangement_report_from_graph_with_cell_complex_cache(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    arrangement_cell_complex_preflight: &mut Option<Option<ExactBooleanPreflight>>,
    retained_request: Option<ExactBooleanRequest>,
    retained_attempt: Option<&ExactArrangementBooleanAttempt>,
) -> Result<ExactPlanarArrangementReport, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(RetainedGraphCounts::empty().into_planar_arrangement_report(
            operation,
            ExactPlanarArrangementStatus::NotNamedOperation,
            ExactBooleanBlocker::default(),
            None,
        ));
    }

    let graph_counts = retained_graph_counts(graph);
    let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    let coplanar_arrangement_evidence = if graph_counts.graph_had_unknowns {
        None
    } else {
        Some(graph.coplanar_arrangement_evidence(left, right)?)
    };
    let requires_planar_arrangement = !graph.face_pairs.is_empty()
        && graph.face_pairs.iter().all(|pair| {
            matches!(
                pair.relation,
                MeshFacePairRelation::CoplanarTouching | MeshFacePairRelation::CoplanarOverlapping
            )
        })
        && graph
            .face_pairs
            .iter()
            .any(|pair| pair.relation == MeshFacePairRelation::CoplanarOverlapping);
    let status = if graph_counts.graph_had_unknowns {
        ExactPlanarArrangementStatus::GraphUnknowns
    } else if boolean_coplanar_mesh_overlay_optional(
        left,
        right,
        operation,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )?
    .is_some()
    {
        ExactPlanarArrangementStatus::AlreadyMaterialized
    } else if graph_requires_boundary_only_contact(graph, left, right)? {
        ExactPlanarArrangementStatus::BoundaryOnlyContactRequired
    } else if requires_planar_arrangement
        && cached_certified_arrangement_cell_complex_preflight(
            arrangement_cell_complex_preflight,
            operation,
            graph,
            left,
            right,
            retained_request,
            retained_attempt,
        )?
        .is_some()
    {
        ExactPlanarArrangementStatus::AlreadyMaterialized
    } else if requires_planar_arrangement {
        ExactPlanarArrangementStatus::Required
    } else {
        ExactPlanarArrangementStatus::NoPositiveOverlap
    };
    Ok(graph_counts.into_planar_arrangement_report(
        operation,
        status,
        counts,
        coplanar_arrangement_evidence,
    ))
}

fn winding_evidence_report_from_graph_with_facts(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<ExactWindingEvidenceReport, ExactMeshError> {
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        let source_shortcut_status = if left.triangles().is_empty() || right.triangles().is_empty()
        {
            Some(ExactWindingEvidenceStatus::EmptyOperandAlreadyMaterialized)
        } else if meshes_are_certified_bounds_disjoint(left, right) {
            Some(ExactWindingEvidenceStatus::BoundsDisjointAlreadyMaterialized)
        } else if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
            && (evidence::identical_mesh_report_from_sources(left, right).status
                == ExactIdenticalMeshStatus::Certified
                || evidence::same_surface_report_from_sources(left, right).status
                    == ExactSameSurfaceStatus::Certified)
        {
            Some(ExactWindingEvidenceStatus::SurfaceEqualityAlreadyMaterialized)
        } else if certified_mixed_dimensional_regularized_solid_support(left, right).is_some() {
            Some(ExactWindingEvidenceStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized)
        } else {
            None
        };
        if let Some(status) = source_shortcut_status {
            return Ok(RetainedGraphCounts::empty().into_winding_evidence_report(
                operation,
                status,
                0,
                Vec::new(),
                ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::Winding),
                None,
                None,
            ));
        }
    }

    let graph_counts = retained_graph_counts(graph);
    let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        let blocker_kind = counts.inferred_kind();
        return Ok(graph_counts.into_winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::NotNamedOperation,
            0,
            Vec::new(),
            counts.into_blocker(blocker_kind),
            None,
            None,
        ));
    }
    if graph_counts.graph_had_unknowns {
        return Ok(graph_counts.into_winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::GraphUnknowns,
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::Refinement),
            None,
            None,
        ));
    }
    let arrangement_cell_complex_shortcut_materializes =
        shortcut_facts.materializes_operation(operation);
    let mut arrangement_cell_complex_preflight = None;
    if !arrangement_cell_complex_shortcut_materializes
        && coplanar_volumetric_evidence_from_graph(graph, left, right)?
            .is_some_and(|evidence| coplanar_evidence_requires_volumetric_cells(&evidence))
        && coplanar_boundary_closure_available_from_graph(graph, left, right, operation)?
    {
        return Ok(
            arrangement_cell_complex_already_materialized_winding_evidence(
                graph, left, right, operation,
            )?,
        );
    }
    if !graph.face_pairs.is_empty()
        && !arrangement_cell_complex_shortcut_materializes
        && cached_certified_arrangement_cell_complex_preflight(
            &mut arrangement_cell_complex_preflight,
            operation,
            graph,
            left,
            right,
            None,
            None,
        )?
        .is_none()
        && certified_convex_relation_shortcut_from_graph(graph, left, right, operation)?.is_some()
    {
        return Ok(winding_evidence_report_with_validated_winding_blocker(
            operation,
            ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized,
            graph,
            counts,
        ));
    }
    if operation == ExactBooleanOperation::Difference
        && !arrangement_cell_complex_shortcut_materializes
        && let Some(evidence) = coplanar_volumetric_evidence_from_graph(graph, left, right)?
            .filter(coplanar_evidence_is_positive_area_boundary_only)
    {
        return Ok(graph_counts.into_winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized,
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::BoundaryOnlyContact),
            None,
            Some(evidence),
        ));
    }
    if !graph.face_pairs.is_empty()
        && cached_certified_arrangement_cell_complex_preflight(
            &mut arrangement_cell_complex_preflight,
            operation,
            graph,
            left,
            right,
            None,
            None,
        )?
        .is_some()
    {
        return Ok(
            arrangement_cell_complex_preflight_materialized_winding_evidence(
                graph,
                left,
                right,
                operation,
                counts,
                arrangement_cell_complex_shortcut_materializes,
            )?,
        );
    }
    if !arrangement_cell_complex_shortcut_materializes
        && certified_convex_operation_shortcut_support(left, right, operation).is_some()
    {
        return Ok(winding_evidence_report_with_validated_winding_blocker(
            operation,
            ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized,
            graph,
            counts,
        ));
    }
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && open_surface_disjoint_report_from_graph(graph, left, right).status
            == ExactOpenSurfaceDisjointStatus::Certified
    {
        return Ok(graph_counts
            .with_retained_face_pairs(0)
            .into_winding_evidence_report(
                operation,
                ExactWindingEvidenceStatus::OpenSurfaceDisjointAlreadyMaterialized,
                0,
                Vec::new(),
                counts.into_blocker(ExactBooleanBlockerKind::Winding),
                None,
                None,
            ));
    }
    if let Some(plan) = open_surface_arrangement_plan_from_graph(graph, left, right, operation)? {
        let region_count = unique_classified_region_count(&plan.region_classifications);
        return Ok(graph_counts.into_winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::OpenSurfaceArrangementAlreadyMaterialized,
            region_count,
            plan.region_classifications,
            counts.into_blocker(ExactBooleanBlockerKind::Winding),
            None,
            None,
        ));
    }
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && left.facts().mesh.closed_manifold
        && right.facts().mesh.closed_manifold
        && coplanar_volumetric_evidence_from_graph(graph, left, right)?
            .is_some_and(|evidence| coplanar_evidence_is_zero_area_boundary_only(&evidence))
    {
        return Ok(graph_counts.into_winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized,
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::BoundaryOnlyContact),
            None,
            None,
        ));
    }
    if !arrangement_cell_complex_shortcut_materializes
        && matches!(
            operation,
            ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
        )
        && left.facts().mesh.closed_manifold
        && right.facts().mesh.closed_manifold
        && coplanar_volumetric_evidence_from_graph(graph, left, right)?
            .is_some_and(|evidence| coplanar_evidence_is_boundary_only_contact(&evidence))
    {
        return Ok(graph_counts.into_winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized,
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::BoundaryOnlyContact),
            None,
            coplanar_volumetric_evidence_from_graph(graph, left, right)?
                .filter(coplanar_evidence_is_positive_area_boundary_only),
        ));
    }
    let boundary_only_contact_required = graph_requires_boundary_only_contact(graph, left, right)?;
    if arrangement_cell_complex_shortcut_materializes && boundary_only_contact_required {
        return Ok(winding_evidence_report_with_validated_winding_blocker(
            operation,
            ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
            graph,
            counts,
        ));
    }
    if boundary_only_contact_required {
        return Ok(graph_counts.into_winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::BoundaryOnlyContactRequired,
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::BoundaryOnlyContact),
            None,
            None,
        ));
    }
    let planar_report = planar_arrangement_report_from_graph_with_cell_complex_cache(
        graph,
        left,
        right,
        operation,
        &mut arrangement_cell_complex_preflight,
        None,
        None,
    )?;
    if matches!(planar_report.status, ExactPlanarArrangementStatus::Required) {
        return Ok(graph_counts.into_winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::PlanarArrangementRequired,
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::PlanarArrangement),
            planar_report
                .coplanar_arrangement_evidence
                .as_ref()
                .cloned(),
            None,
        ));
    }
    if matches!(
        planar_report.status,
        ExactPlanarArrangementStatus::AlreadyMaterialized
    ) {
        return Ok(graph_counts.into_winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::PlanarArrangementAlreadyMaterialized,
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::PlanarArrangement),
            planar_report
                .coplanar_arrangement_evidence
                .as_ref()
                .cloned(),
            None,
        ));
    }
    if arrangement_cell_complex_shortcut_materializes {
        let mut report = arrangement_cell_complex_already_materialized_winding_evidence(
            graph, left, right, operation,
        )?;
        report.blocker = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding)
            .into_blocker(ExactBooleanBlockerKind::Winding);
        return Ok(report);
    }
    if let Some(report) = volumetric_winding_region_plan_evidence_from_graph(
        graph,
        left,
        right,
        operation,
        graph_counts,
        counts,
    )? {
        return Ok(report);
    }
    if let Some(coplanar_volumetric_evidence) =
        coplanar_volumetric_evidence_from_graph(graph, left, right)?
            .filter(coplanar_evidence_requires_volumetric_cells)
    {
        if cached_certified_arrangement_cell_complex_preflight(
            &mut arrangement_cell_complex_preflight,
            operation,
            graph,
            left,
            right,
            None,
            None,
        )?
        .is_some()
        {
            return Ok(graph_counts.into_winding_evidence_report(
                operation,
                ExactWindingEvidenceStatus::CoplanarVolumetricCellsAlreadyMaterialized,
                0,
                Vec::new(),
                counts.into_blocker(ExactBooleanBlockerKind::CoplanarVolumetricCells),
                None,
                Some(coplanar_volumetric_evidence),
            ));
        }
        return Ok(graph_counts.into_winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::CoplanarVolumetricCellsRequired,
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::CoplanarVolumetricCells),
            None,
            Some(coplanar_volumetric_evidence),
        ));
    }
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && let Some((left_in_right, right_in_left)) =
            closed_winding_vertex_relations_from_empty_graph(graph, left, right)?
        && left_in_right == ClosedMeshWindingMeshRelation::Outside
        && right_in_left == ClosedMeshWindingMeshRelation::Outside
    {
        return Ok(graph_counts
            .with_retained_face_pairs(0)
            .into_winding_evidence_report(
                operation,
                ExactWindingEvidenceStatus::ClosedWindingSeparatedAlreadyMaterialized,
                0,
                Vec::new(),
                counts.into_blocker(ExactBooleanBlockerKind::Winding),
                None,
                None,
            ));
    }
    if graph.face_pairs.is_empty()
        && !meshes_are_certified_bounds_disjoint(left, right)
        && cached_certified_arrangement_cell_complex_preflight(
            &mut arrangement_cell_complex_preflight,
            operation,
            graph,
            left,
            right,
            None,
            None,
        )?
        .is_some()
    {
        return Ok(
            arrangement_cell_complex_already_materialized_winding_evidence(
                graph, left, right, operation,
            )?,
        );
    }
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && certified_closed_winding_containment_relation_from_graph(graph, left, right)?.is_some()
    {
        return Ok(graph_counts
            .with_retained_face_pairs(0)
            .into_winding_evidence_report(
                operation,
                ExactWindingEvidenceStatus::ClosedWindingContainmentAlreadyMaterialized,
                0,
                Vec::new(),
                counts.into_blocker(ExactBooleanBlockerKind::Winding),
                None,
                None,
            ));
    }
    if graph.face_pairs.is_empty() {
        return Ok(graph_counts
            .with_retained_face_pairs(0)
            .into_winding_evidence_report(
                operation,
                ExactWindingEvidenceStatus::NoNontrivialOverlap,
                0,
                Vec::new(),
                counts.into_blocker(ExactBooleanBlockerKind::Winding),
                None,
                None,
            ));
    }

    let geometry = graph.face_split_geometry_plan(left, right)?;
    let region_plan = geometry.region_plan(left, right);
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
    Ok(graph_counts.into_winding_evidence_report(
        operation,
        ExactWindingEvidenceStatus::Ready,
        region_plan.regions.len(),
        region_classifications,
        counts.into_blocker(ExactBooleanBlockerKind::Winding),
        None,
        None,
    ))
}

fn volumetric_winding_region_plan_evidence_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    graph_counts: RetainedGraphCounts,
    counts: ExactBooleanBlocker,
) -> Result<Option<ExactWindingEvidenceReport>, ExactMeshError> {
    let Some(plan) = volumetric_winding_region_plan_from_graph(graph, left, right)? else {
        return Ok(None);
    };
    let VolumetricWindingRegionPlan {
        region_classifications,
        triangulations,
        volumetric_classifications,
    } = plan;

    let coplanar_volumetric_evidence = coplanar_volumetric_evidence_from_graph(graph, left, right)?
        .filter(coplanar_evidence_requires_volumetric_cells);
    let blocker_kind = match &coplanar_volumetric_evidence {
        Some(_) => ExactBooleanBlockerKind::CoplanarVolumetricCells,
        None => ExactBooleanBlockerKind::Winding,
    };
    if let Some(materialized) = materialize_volumetric_winding_region_plan(
        region_classifications.clone(),
        triangulations.clone(),
        volumetric_classifications.clone(),
        left,
        right,
        operation,
        ExactMeshValidationPolicy::CLOSED,
    )? {
        return Ok(Some(graph_counts.into_winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::Ready,
            materialized.triangulations.len(),
            materialized.region_classifications,
            counts.into_blocker(blocker_kind),
            None,
            coplanar_volumetric_evidence,
        )));
    }
    if let Some(materialized) = materialize_volumetric_winding_region_plan(
        region_classifications.clone(),
        triangulations.clone(),
        volumetric_classifications.clone(),
        left,
        right,
        operation,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )? && certified_coplanar_boundary_closure_from_materialized(
        &materialized,
        left,
        right,
        operation,
        ExactMeshValidationPolicy::CLOSED,
    )?
    .is_some()
    {
        return Ok(Some(
            arrangement_cell_complex_already_materialized_winding_evidence(
                graph, left, right, operation,
            )?,
        ));
    }
    if volumetric_classifications.iter().all(|classification| {
        matches!(
            classification.relation,
            ExactVolumetricRegionRelation::Inside
                | ExactVolumetricRegionRelation::Outside
                | ExactVolumetricRegionRelation::Boundary
        )
    }) {
        let region_count = unique_classified_region_count(&region_classifications);
        return Ok(Some(graph_counts.into_winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::VolumetricAssemblyRequired,
            region_count,
            region_classifications,
            counts.into_blocker(blocker_kind),
            None,
            coplanar_volumetric_evidence,
        )));
    }

    Ok(None)
}

fn winding_evidence_report_with_validated_winding_blocker(
    operation: ExactBooleanOperation,
    status: ExactWindingEvidenceStatus,
    graph: &super::graph::ExactIntersectionGraph,
    counts: ExactBooleanBlocker,
) -> ExactWindingEvidenceReport {
    let blocker = counts.into_blocker(ExactBooleanBlockerKind::Winding);
    let (graph_counts, blocker) = if blocker
        .validate_for_kind(ExactBooleanBlockerKind::Winding)
        .is_ok()
    {
        (retained_graph_counts(graph), blocker)
    } else {
        (
            RetainedGraphCounts::empty(),
            ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::Winding),
        )
    };
    graph_counts.into_winding_evidence_report(operation, status, 0, Vec::new(), blocker, None, None)
}

struct VolumetricWindingRegionPlan {
    region_classifications: Vec<FaceRegionPlaneClassification>,
    triangulations: Vec<FaceRegionTriangulation>,
    volumetric_classifications: Vec<ExactVolumetricRegionClassification>,
}

pub(crate) struct MaterializedVolumetricWindingRegionPlan {
    pub(crate) region_classifications: Vec<FaceRegionPlaneClassification>,
    pub(crate) triangulations: Vec<FaceRegionTriangulation>,
    pub(crate) volumetric_classifications: Vec<ExactVolumetricRegionClassification>,
    pub(crate) assembly: ExactBooleanAssemblyPlan,
    pub(crate) mesh: ExactMesh,
}

fn materialize_volumetric_winding_region_plan_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<MaterializedVolumetricWindingRegionPlan>, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let Some(plan) = volumetric_winding_region_plan_from_graph(graph, left, right)? else {
        return Ok(None);
    };
    materialize_volumetric_winding_region_plan(
        plan.region_classifications,
        plan.triangulations,
        plan.volumetric_classifications,
        left,
        right,
        operation,
        validation,
    )
}

fn materialize_volumetric_winding_region_plan(
    region_classifications: Vec<FaceRegionPlaneClassification>,
    triangulations: Vec<FaceRegionTriangulation>,
    volumetric_classifications: Vec<ExactVolumetricRegionClassification>,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<MaterializedVolumetricWindingRegionPlan>, ExactMeshError> {
    if !volumetric_classifications.iter().all(|classification| {
        matches!(
            classification.relation,
            ExactVolumetricRegionRelation::Inside
                | ExactVolumetricRegionRelation::Outside
                | ExactVolumetricRegionRelation::Boundary
        )
    }) {
        return Ok(None);
    }
    if !triangulations.iter().any(|triangulation| {
        triangulation.triangles.chunks_exact(3).any(|triangle| {
            volumetric_retention_for_operation(
                operation,
                triangulation,
                [triangle[0], triangle[1], triangle[2]],
                &volumetric_classifications,
            ) != ExactRegionRetention::Drop
        })
    }) {
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
        .canonicalize_for_mesh_with_sources(left, right)
        .is_err()
    {
        return Ok(None);
    }
    let mesh = match assembly.checked_to_exact_mesh_with_sources(left, right, validation) {
        Ok(mesh) => mesh,
        Err(_) => return Ok(None),
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
) -> Result<Option<VolumetricWindingRegionPlan>, ExactMeshError> {
    let counts = ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding);
    if graph.has_unknowns()
        || graph.face_pairs.is_empty()
        || counts.unknown_pairs != 0
        || counts.construction_failed_events != 0
        || !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
    {
        return Ok(None);
    }
    if graph_requires_boundary_only_contact(graph, left, right)? {
        return Ok(None);
    }

    let cell_plan = match triangulate_all_face_cells_with_cdt(graph, left, right) {
        Ok(plan) => plan,
        Err(_error)
            if coplanar_volumetric_evidence_from_graph(graph, left, right)?
                .is_some_and(|evidence| coplanar_evidence_requires_volumetric_cells(&evidence)) =>
        {
            // Coplanar source-face overlaps can expose constraint-normalization
            // cases that are not part of the current bounded volumetric cell
            // receives `RequiresCoplanarVolumetricCells` instead of a generic
            // triangulation failure or a tolerance fallback.
            return Ok(None);
        }
        Err(error) => {
            return Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::DegenerateTriangle,
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
            .map_err(|error| {
                boolean_validation_error(
                    ExactMeshBlockerKind::StaleFactReplay,
                    "exact volumetric winding region report/source replay failed",
                    error,
                )
            })?;
    Ok(Some(VolumetricWindingRegionPlan {
        region_classifications,
        triangulations,
        volumetric_classifications,
    }))
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

fn copy_mesh(
    mesh: &ExactMesh,
    label: &'static str,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactMesh, ExactMeshError> {
    ExactMesh::new_with_policy_and_version(
        mesh.vertices().to_vec(),
        mesh.triangles().to_vec(),
        hyperlimit::SourceProvenance::exact(label),
        validation,
        1,
    )
}

fn concatenate_meshes_with_options(
    left: &ExactMesh,
    right: &ExactMesh,
    reverse_right: bool,
    label: &'static str,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactMesh, ExactMeshError> {
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
    ExactMesh::new_with_policy_and_version(
        vertices,
        triangles,
        hyperlimit::SourceProvenance::exact(label),
        validation,
        1,
    )
}

fn boolean_closed_regularized_lower_dimensional_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return Ok(None);
    }
    let Some(left_kind) = closed_regularized_operand_kind(left) else {
        return Ok(None);
    };
    let Some(right_kind) = closed_regularized_operand_kind(right) else {
        return Ok(None);
    };
    if matches!(left_kind, ClosedRegularizedOperandKind::ClosedSolid)
        && matches!(right_kind, ClosedRegularizedOperandKind::ClosedSolid)
    {
        return Ok(None);
    }
    if matches!(left_kind, ClosedRegularizedOperandKind::LowerDimensional)
        && matches!(right_kind, ClosedRegularizedOperandKind::LowerDimensional)
        && !matches!(validation, ExactMeshValidationPolicy::CLOSED)
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

    Ok(Some(certified_shortcut_result(mesh, operation, shortcut)))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum ClosedRegularizedOperandKind {
    ClosedSolid,
    LowerDimensional,
}

pub(super) fn closed_regularized_operand_kind(
    mesh: &ExactMesh,
) -> Option<ClosedRegularizedOperandKind> {
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
    validation: ExactMeshValidationPolicy,
) -> Result<ExactBooleanResult, ExactMeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union => {
            concatenate_meshes_with_options(left, right, false, "exact disjoint union", validation)?
        }
        ExactBooleanOperation::Intersection => {
            empty_mesh("empty exact bounds-disjoint intersection", validation)?
        }
        ExactBooleanOperation::Difference => copy_mesh(
            left,
            "exact bounds-disjoint difference keeps left",
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => {
            return Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::UnsupportedExactOperation,
                format!(
                    "bounds-disjoint materialization requires a named boolean operation: {operation:?}"
                ),
            )));
        }
    };
    Ok(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::BoundsDisjoint,
    ))
}

fn boolean_empty_operand(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactBooleanResult, ExactMeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union
            if validation == ExactMeshValidationPolicy::CLOSED
                && (left.triangles().is_empty() || right.triangles().is_empty())
                && matches!(
                    (
                        closed_regularized_operand_kind(left),
                        closed_regularized_operand_kind(right),
                    ),
                    (
                        Some(ClosedRegularizedOperandKind::LowerDimensional),
                        Some(ClosedRegularizedOperandKind::LowerDimensional)
                    )
                ) =>
        {
            empty_mesh(
                "empty exact closed regularized union with empty operand",
                validation,
            )?
        }
        ExactBooleanOperation::Union => {
            concatenate_meshes_with_options(left, right, false, "exact disjoint union", validation)?
        }
        ExactBooleanOperation::Intersection => {
            empty_mesh("empty exact intersection with empty operand", validation)?
        }
        ExactBooleanOperation::Difference if left.triangles().is_empty() => {
            empty_mesh("empty exact difference from empty left operand", validation)?
        }
        ExactBooleanOperation::Difference
            if validation == ExactMeshValidationPolicy::CLOSED
                && right.triangles().is_empty()
                && closed_regularized_operand_kind(left)
                    == Some(ClosedRegularizedOperandKind::LowerDimensional) =>
        {
            empty_mesh(
                "empty exact closed regularized difference with empty right operand",
                validation,
            )?
        }
        ExactBooleanOperation::Difference => ExactMesh::new_with_policy_and_version(
            left.vertices().to_vec(),
            left.triangles().to_vec(),
            hyperlimit::SourceProvenance::exact("exact difference with empty right operand"),
            validation,
            1,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => {
            return Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::UnsupportedExactOperation,
                format!(
                    "empty-operand materialization requires a named boolean operation: {operation:?}"
                ),
            )));
        }
    };

    Ok(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::EmptyOperand,
    ))
}

fn boolean_identical_meshes(
    mesh: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactBooleanResult, ExactMeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection => {
            ExactMesh::new_with_policy_and_version(
                mesh.vertices().to_vec(),
                mesh.triangles().to_vec(),
                hyperlimit::SourceProvenance::exact("exact identical boolean result"),
                validation,
                1,
            )?
        }
        ExactBooleanOperation::Difference => ExactMesh::new_with_policy_and_version(
            Vec::new(),
            Vec::new(),
            hyperlimit::SourceProvenance::exact("empty exact identical difference"),
            validation,
            1,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => {
            return Err(ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::UnsupportedExactOperation,
                format!(
                    "identical-mesh materialization requires a named boolean operation: {operation:?}"
                ),
            )));
        }
    };

    Ok(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::Identical,
    ))
}

fn empty_mesh(
    label: &'static str,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactMesh, ExactMeshError> {
    ExactMesh::new_with_policy_and_version(
        Vec::new(),
        Vec::new(),
        hyperlimit::SourceProvenance::exact(label),
        validation,
        1,
    )
}

fn certified_shortcut_result(
    mesh: ExactMesh,
    operation: ExactBooleanOperation,
    shortcut: ExactBooleanShortcutKind,
) -> ExactBooleanResult {
    ExactBooleanResult {
        kind: ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut,
        },
        graph_had_unknowns: false,
        region_classifications: Vec::new(),
        triangulations: Vec::new(),
        assembly: ExactBooleanAssemblyPlan {
            vertices: Vec::new(),
            triangles: Vec::new(),
        },
        volumetric_classifications: Vec::new(),
        topology_assembly_report: None,
        region_ownership_report: None,
        mesh,
    }
}

#[cfg(test)]
mod replay;
#[cfg(test)]
mod tests;
