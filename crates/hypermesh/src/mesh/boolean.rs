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

use hyperlimit::SegmentPlaneRelation;

use super::arrangement3d::arrangement2d::{
    ExactArrangement2dBlocker, ExactArrangement2dBoundaryPolicy, ExactArrangement2dOverlay,
    ExactArrangement2dRegion, ExactArrangement2dRegionRing, ExactArrangement2dSetOperation,
    build_exact_arrangement2d_overlay, build_exact_arrangement2d_overlay_with_boundary_policy,
};
use super::arrangement3d::cell_complex::simplify::ExactSimplifiedCellComplex;
use super::arrangement3d::cell_complex::{
    ExactLabeledCellComplex, ExactRegionOwnershipReport, ExactSelectedCellComplex,
    arrangement_cell_complex_labeling_policy,
    arrangement_region_classification_blockers_resolve_operation, select_arrangement_for_replay,
};
use super::arrangement3d::loop_triangulation::{
    group_exact_coplanar_loops, triangulate_exact_loop_group,
};
use super::arrangement3d::regularization::{ExactArrangementBlocker, ExactRegularizationPolicy};
use super::arrangement3d::{ExactArrangement, ExactTopologyAssemblyReport};
use super::error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
#[cfg(test)]
use super::graph::FacePairEvents;
#[cfg(test)]
use super::graph::build_unvalidated_intersection_graph;
use super::graph::intersection::MeshFacePairRelation;
use super::graph::{
    ExactIntersectionGraph, IntersectionEvent, MeshSide, build_validated_intersection_graph,
    build_validated_intersection_graph_from_prepared_pair,
};
use super::validation::ExactMeshValidationPolicy;
use super::view::PreparedMeshPair;
use super::{ExactMesh, Triangle};
use adjacent::{
    full_face_adjacent_certificate_from_graph,
    materialize_full_face_adjacent_union_from_certificate,
};
use affine_solid::{
    AffineOrthogonalSolidOperation, has_affine_orthogonal_solid_cells,
    has_empty_affine_orthogonal_solid_cell_intersection,
    materialize_affine_orthogonal_solid_difference,
    materialize_affine_orthogonal_solid_intersection, materialize_affine_orthogonal_solid_union,
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
    ExactEvidenceValidationError, ExactOpenSurfaceDisjointReport, ExactOpenSurfaceDisjointStatus,
    ExactPlanarArrangementReport, ExactPlanarArrangementStatus,
    ExactVolumetricBoundaryClosureReport, ExactVolumetricBoundaryClosureStatus,
    ExactWindingEvidenceReport, ExactWindingEvidenceStatus,
    certified_convex_operation_shortcut_support, meshes_are_certified_bounds_disjoint,
};
#[cfg(test)]
use evidence::{
    ExactBooleanCertificationSet, ExactBooleanEvaluation, ExactBooleanSourceFacts,
    ExactRefinementReport, ExactRefinementStatus,
};
use hyperlimit::SourceProvenance;
use hyperlimit::{
    CoplanarProjection, Point2, Point3, SegmentIntersection, Sign, TriangleLocation,
    classify_point_triangle, compare_reals, orient3d_report, project_point3,
    projected_polygon_area2_value,
};
use hyperreal::Real;
use orthogonal_solid::{
    AxisAlignedOrthogonalSolidOperation, has_empty_axis_aligned_orthogonal_solid_cell_intersection,
    materialize_axis_aligned_orthogonal_solid_cell_output,
};
#[cfg(test)]
use orthogonal_solid::{
    axis_aligned_orthogonal_solid_cell_plan, is_axis_aligned_box,
    materialize_axis_aligned_orthogonal_solid_cell_plan, try_is_axis_aligned_box,
};
use region::{
    ExactBooleanAssemblyPlan, ExactRegionRetention, ExactRegionSelection,
    FaceRegionPlaneClassification, FaceRegionTriangulation,
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
use volumetric_cells::{CoplanarVolumetricCellEvidenceReport, CoplanarVolumetricCellObstacle};
use winding::{
    ClosedMeshWindingMeshRelation, WindingReportError,
    classify_mesh_vertices_against_closed_mesh_winding_report,
};

impl ExactArrangementBooleanAttempt {
    /// Validate this attempt by replaying it for an exact Boolean request.
    pub(crate) fn validate_against_sources_for_request(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        request: ExactBooleanRequest,
    ) -> Result<(), ExactEvidenceValidationError> {
        self.validate()?;
        if self.materialized_arrangement_cell_complex_shortcut()
            && orthogonal_solid_cell_materializes_for_preflight(left, right, request.operation)
                .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            && let Some(replay) =
                arrangement_cell_complex_shortcut_attempt(left, right, request, self.policy)
                    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
        {
            replay.validate_for_request_policy(request, self.policy)?;
            return if self == &replay || self.materialized_output_matches_replay(&replay) {
                Ok(())
            } else {
                Err(ExactEvidenceValidationError::SourceReplayMismatch)
            };
        }
        let replay = match ExactArrangement::from_meshes_with_policy(left, right, self.policy) {
            Ok(arrangement) => {
                let attempt = match arrangement_boolean_attempt_report_from_arrangement(
                    left,
                    right,
                    request,
                    self.policy,
                    &arrangement,
                ) {
                    Ok(attempt) => attempt,
                    Err(_) => {
                        arrangement_cell_complex_shortcut_attempt(left, right, request, self.policy)
                            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
                            .ok_or(ExactEvidenceValidationError::SourceReplayMismatch)?
                    }
                };
                if attempt.materialized_arrangement_cell_complex_output() {
                    attempt
                } else {
                    arrangement_cell_complex_shortcut_attempt(left, right, request, self.policy)
                        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
                        .unwrap_or(attempt)
                }
            }
            Err(_) => arrangement_cell_complex_shortcut_attempt(left, right, request, self.policy)
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

#[cfg(test)]
pub(crate) fn exact_boolean_evaluation_for_replay(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactBooleanEvaluation, ExactEvidenceValidationError> {
    exact_boolean_evaluation_for_replay_result(left, right, request)
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)
}

#[cfg(test)]
fn exact_boolean_evaluation_for_replay_result(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactBooleanEvaluation, ExactMeshError> {
    exact_boolean_evaluation_for_replay_result_with_materialization(left, right, request, true)
}

#[cfg(test)]
fn exact_boolean_evaluation_for_replay_result_with_materialization(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    materialize_result: bool,
) -> Result<ExactBooleanEvaluation, ExactMeshError> {
    left.validate_retained_bounds()?;
    right.validate_retained_bounds()?;
    let source_facts = ExactBooleanSourceFacts::from_sources(left, right);
    let shortcut_facts = source_facts.arrangement_cell_complex_shortcuts().clone();
    let graph = build_validated_intersection_graph(left, right)?;
    let mut regularized_arrangement = None;
    let mut regularized_attempt = None;
    let mut preflight = exact_boolean_replay_preflight(
        left,
        right,
        request,
        &graph,
        &shortcut_facts,
        regularized_attempt.as_ref(),
    )?;
    let certified_by_coplanar_boundary_closure = preflight.support()
        == ExactBooleanSupport::CertifiedArrangementCellComplex
        && request.validation == ExactMeshValidationPolicy::CLOSED
        && preflight.coplanar_volumetric_evidence().is_some();
    let certified_by_orthogonal_cell_materialization = preflight.support()
        == ExactBooleanSupport::CertifiedArrangementCellComplex
        && orthogonal_solid_cell_materializes_for_preflight(left, right, request.operation)?;
    let should_replay_arrangement = !certified_by_coplanar_boundary_closure
        && !certified_by_orthogonal_cell_materialization
        && matches!(
            preflight.support(),
            ExactBooleanSupport::CertifiedArrangementCellComplex
                | ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
                | ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
                | ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference
        )
        || (!certified_by_coplanar_boundary_closure
            && !certified_by_orthogonal_cell_materialization
            && !graph.face_pairs.is_empty()
            && matches!(
                preflight.support(),
                ExactBooleanSupport::CertifiedConvexUnion
                    | ExactBooleanSupport::CertifiedConvexIntersection
                    | ExactBooleanSupport::CertifiedConvexDifference
            ));
    if should_replay_arrangement {
        replay_regularized_arrangement_attempt(
            left,
            right,
            request,
            &graph,
            &shortcut_facts,
            &mut regularized_arrangement,
            &mut regularized_attempt,
        )?;
        if regularized_attempt.is_some() {
            preflight = exact_boolean_replay_preflight(
                left,
                right,
                request,
                &graph,
                &shortcut_facts,
                regularized_attempt.as_ref(),
            )?;
        }
    }
    let certifications = certification_set_from_graph_and_regularized_arrangement(
        &graph,
        left,
        right,
        request,
        regularized_arrangement.as_ref(),
        regularized_attempt.as_ref(),
        &source_facts,
    )?;
    let result = if materialize_result && preflight.is_certified() {
        if matches!(
            preflight.support(),
            ExactBooleanSupport::SelectedRegionPolicy
        ) {
            try_materialize_certified_boolean_support_with_artifacts(
                left,
                right,
                request,
                preflight.support(),
                Some(&graph),
                regularized_arrangement.as_ref(),
                regularized_attempt.as_ref(),
                &shortcut_facts,
            )
            .ok()
            .flatten()
        } else {
            try_materialize_certified_boolean_support_with_artifacts(
                left,
                right,
                request,
                preflight.support(),
                Some(&graph),
                regularized_arrangement.as_ref(),
                regularized_attempt.as_ref(),
                &shortcut_facts,
            )?
        }
    } else {
        None
    };
    ExactBooleanEvaluation::from_parts_with_missing_result_policy(
        request,
        preflight,
        certifications,
        result,
        !materialize_result,
    )
    .map_err(|error| {
        retained_evidence_validation_error(
            RETAINED_EVIDENCE_REPLAY_CONTEXT,
            error,
            ExactMeshBlockerKind::StaleFactReplay,
        )
    })
}

#[cfg(test)]
fn replay_regularized_arrangement_attempt(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    graph: &ExactIntersectionGraph,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
    retained_arrangement: &mut Option<ExactArrangement>,
    retained_attempt: &mut Option<ExactArrangementBooleanAttempt>,
) -> Result<(), ExactMeshError> {
    let policy = ExactRegularizationPolicy::REGULARIZED_SOLID;
    if let Some(attempt) = retained_attempt.as_ref() {
        attempt
            .validate_for_request_policy(request, policy)
            .and_then(|()| attempt.validate_against_sources_for_request(left, right, request))
            .map_err(|error| {
                retained_evidence_validation_error(
                    RETAINED_EVIDENCE_REPLAY_CONTEXT,
                    error,
                    ExactMeshBlockerKind::StaleFactReplay,
                )
            })?;
        return Ok(());
    }
    let attempt = match retained_arrangement {
        Some(arrangement) => {
            let attempt = arrangement_boolean_attempt_report_from_arrangement(
                left,
                right,
                request,
                policy,
                arrangement,
            )?;
            if attempt.materialized_arrangement_cell_complex_output() {
                attempt
            } else {
                arrangement_cell_complex_shortcut_attempt_with_facts(
                    left,
                    right,
                    request,
                    policy,
                    shortcut_facts,
                )?
                .unwrap_or(attempt)
            }
        }
        None => match ExactArrangement::from_intersection_graph_with_policy(
            graph.clone(),
            left,
            right,
            policy,
        ) {
            Ok(arrangement) => {
                arrangement.validate().map_err(|blocker| {
                    ExactMeshError::one(ExactMeshBlocker::new(
                        ExactMeshBlockerKind::ExactConstructionFailure,
                        format!("exact boolean arrangement report failed: {blocker:?}"),
                    ))
                })?;
                let attempt = arrangement_boolean_attempt_report_from_arrangement(
                    left,
                    right,
                    request,
                    policy,
                    &arrangement,
                )?;
                *retained_arrangement = Some(arrangement);
                if attempt.materialized_arrangement_cell_complex_output() {
                    attempt
                } else {
                    arrangement_cell_complex_shortcut_attempt_with_facts(
                        left,
                        right,
                        request,
                        policy,
                        shortcut_facts,
                    )?
                    .unwrap_or(attempt)
                }
            }
            Err(error) => {
                if let Some(attempt) = arrangement_cell_complex_shortcut_attempt_with_facts(
                    left,
                    right,
                    request,
                    policy,
                    shortcut_facts,
                )? {
                    attempt
                } else {
                    return Err(error);
                }
            }
        },
    };
    attempt
        .validate_for_request_policy(request, policy)
        .map_err(|error| {
            retained_evidence_validation_error(
                RETAINED_EVIDENCE_REPLAY_CONTEXT,
                error,
                ExactMeshBlockerKind::StaleFactReplay,
            )
        })?;
    *retained_attempt = Some(attempt);
    Ok(())
}

fn exact_boolean_replay_preflight(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    graph: &ExactIntersectionGraph,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
    retained_attempt: Option<&ExactArrangementBooleanAttempt>,
) -> Result<ExactBooleanPreflight, ExactMeshError> {
    let graph_preflight_has_source_arrangement_shortcut = shortcut_facts
        .certified_support(request.operation)
        == Some(ExactBooleanSupport::CertifiedArrangementCellComplex);
    let graph_preflight_has_certified_axis_aligned_box_pair =
        shortcut_facts.certifies_axis_aligned_box_pair();
    let graph_preflight = preflight_boolean_exact_request_from_graph_with_retained_attempt(
        graph,
        left,
        right,
        request,
        retained_attempt,
        shortcut_facts,
    )?;
    if graph_preflight.operation() != request.operation {
        return Err(retained_evidence_validation_error(
            RETAINED_EVIDENCE_REPLAY_CONTEXT,
            ExactEvidenceValidationError::StatusEvidenceMismatch,
            ExactMeshBlockerKind::StaleFactReplay,
        ));
    }
    graph_preflight.validate().map_err(|error| {
        retained_evidence_validation_error(
            RETAINED_EVIDENCE_REPLAY_CONTEXT,
            error,
            ExactMeshBlockerKind::StaleFactReplay,
        )
    })?;
    if matches!(
        graph_preflight.support(),
        ExactBooleanSupport::CertifiedEmptyOperand
            | ExactBooleanSupport::CertifiedBoundsDisjoint
            | ExactBooleanSupport::CertifiedIdentical
            | ExactBooleanSupport::CertifiedSameSurface
            | ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
            | ExactBooleanSupport::CertifiedClosedWindingSeparated
            | ExactBooleanSupport::CertifiedClosedWindingContainment
            | ExactBooleanSupport::CertifiedConvexSeparated
            | ExactBooleanSupport::CertifiedConvexContainment
    ) || (matches!(
        graph_preflight.support(),
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
    ) && !graph_preflight_has_source_arrangement_shortcut
        && !graph_preflight_has_certified_axis_aligned_box_pair)
    {
        return Ok(graph_preflight);
    }
    if (!(request.validation == ExactMeshValidationPolicy::ALLOW_BOUNDARY
        && request.boundary_policy == ExactBoundaryBooleanPolicy::Reject)
        || graph_preflight_has_source_arrangement_shortcut
        || graph_preflight_has_certified_axis_aligned_box_pair)
        && let Some(attempt) = retained_attempt
        && let Ok(Some(preflight)) =
            certified_arrangement_cell_complex_preflight_from_retained_attempt(
                graph,
                left,
                right,
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                attempt,
            )
    {
        preflight.validate().map_err(|error| {
            retained_evidence_validation_error(
                RETAINED_EVIDENCE_REPLAY_CONTEXT,
                error,
                ExactMeshBlockerKind::StaleFactReplay,
            )
        })?;
        return Ok(preflight);
    }
    Ok(graph_preflight)
}

pub(crate) fn preflight_report_for_request_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactBooleanPreflight, ExactMeshError> {
    let shortcut_facts = ExactArrangementCellComplexShortcutFacts::from_sources(left, right);
    exact_boolean_replay_preflight(left, right, request, graph, &shortcut_facts, None)
}

const RETAINED_EVIDENCE_REPLAY_CONTEXT: &str =
    "exact boolean retained evidence failed replay validation";

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

fn retained_arrangement_attempt_for_request<'a>(
    retained: Option<&'a ExactArrangementBooleanAttempt>,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
) -> Result<Option<&'a ExactArrangementBooleanAttempt>, ExactEvidenceValidationError> {
    let Some(attempt) = retained else {
        return Ok(None);
    };
    attempt.validate_for_request_policy(request, policy)?;
    attempt.validate_against_sources_for_request(left, right, request)?;
    Ok(Some(attempt))
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

/// Boundary-only policy for named exact boolean operations.
///
/// Triangle meshes cannot represent lower-dimensional set intersections
/// certified coplanar-touching graphs are either rejected, or projected into a
/// triangle-mesh-only result that preserves separate shells and discards
/// lower-dimensional intersection geometry.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactBoundaryBooleanPolicy {
    /// Reject boundary-only named booleans until a caller chooses a projection
    /// policy.
    Reject,
    /// Preserve separate shells for union, keep the left shell for difference,
    /// and return an empty triangle mesh for lower-dimensional intersections.
    PreserveSeparateShells,
}

/// Complete policy for an exact boolean request.
///
/// The request keeps operation semantics, output validation, and
/// lower-dimensional boundary projection policy together so preflight,
/// certification, and materialization replay the same exact contract.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ExactBooleanRequest {
    /// Named or selected-region operation to evaluate.
    pub(crate) operation: ExactBooleanOperation,
    /// Output mesh validation policy.
    pub(crate) validation: ExactMeshValidationPolicy,
    /// Explicit boundary-only projection policy.
    pub(crate) boundary_policy: ExactBoundaryBooleanPolicy,
}

impl ExactBooleanRequest {
    /// Creates a request using the default exact materialization policy.
    ///
    /// Certified boundary-only contact is supportable by the triangle-mesh
    /// output contract: union preserves separate shells, difference keeps the
    /// left shell, and intersection yields the representable empty triangle
    /// mesh for lower-dimensional contact. Call
    /// [`Self::with_boundary_policy`] with [`ExactBoundaryBooleanPolicy::Reject`]
    /// when a caller wants to retain that state as an explicit blocker.
    pub(crate) const fn new(
        operation: ExactBooleanOperation,
        validation: ExactMeshValidationPolicy,
    ) -> Self {
        Self {
            operation,
            validation,
            boundary_policy: ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        }
    }

    /// Creates a request with an explicit boundary projection policy.
    pub(crate) const fn with_boundary_policy(
        operation: ExactBooleanOperation,
        validation: ExactMeshValidationPolicy,
        boundary_policy: ExactBoundaryBooleanPolicy,
    ) -> Self {
        Self {
            operation,
            validation,
            boundary_policy,
        }
    }
}

#[cfg(test)]
fn certification_set_from_graph_and_regularized_arrangement(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_regularized_arrangement: Option<&ExactArrangement>,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
    source_facts: &ExactBooleanSourceFacts,
) -> Result<ExactBooleanCertificationSet, ExactMeshError> {
    validate_graph_source_replay(graph, left, right)?;
    if let Some(attempt) = retained_arrangement_attempt {
        attempt
            .validate_for_request_policy(request, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .map_err(|error| {
                retained_evidence_validation_error(
                    "retained arrangement attempt failed validation",
                    error,
                    ExactMeshBlockerKind::ExactConstructionFailure,
                )
            })?;
    }
    let trivial = source_facts.trivial().clone();
    let regularized_solid = source_facts.regularized_solid().clone();
    let refinement = refinement_report_from_graph(graph, request.operation);
    let boundary_touching = boundary_touching_report_from_graph(graph, left, right)
        .unwrap_or_else(|_| not_boundary_only_report_from_graph(graph));
    let open_surface_disjoint = open_surface_disjoint_report_from_graph(graph, left, right);
    let adjacent_union_completion = adjacent_union_completion_certification_from_graph(
        graph,
        left,
        right,
        request.operation,
        None,
    )?
    .0;
    let adjacent_union_completion_certified = adjacent_union_completion.is_certified();
    let identical = source_facts.identical().clone();
    let same_surface = source_facts.same_surface().clone();
    let closed_winding_left_in_right = source_facts.closed_winding_left_in_right().clone();
    let closed_winding_right_in_left = source_facts.closed_winding_right_in_left().clone();
    let convex_left_in_right = source_facts.convex_left_in_right().clone();
    let convex_right_in_left = source_facts.convex_right_in_left().clone();
    let convex_capabilities = source_facts.convex_capabilities().clone();
    let arrangement_cell_complex_shortcuts = source_facts.arrangement_cell_complex_shortcuts();
    let reject_boundary_evidence_request = request.validation
        == ExactMeshValidationPolicy::ALLOW_BOUNDARY
        && request.boundary_policy == ExactBoundaryBooleanPolicy::Reject;
    let planar_arrangement = planar_arrangement_certification_report_from_graph(
        graph,
        left,
        right,
        request,
        retained_arrangement_attempt,
    );
    let volumetric_boundary_closure = volumetric_boundary_closure_certification_report_from_graph(
        graph,
        left,
        right,
        request,
        adjacent_union_completion_certified,
        reject_boundary_evidence_request,
    );
    let arrangement_attempt = arrangement_attempt_for_certification_set(
        graph,
        left,
        right,
        request,
        retained_regularized_arrangement,
        retained_arrangement_attempt,
        arrangement_cell_complex_shortcuts,
        adjacent_union_completion_certified,
        reject_boundary_evidence_request,
    )?;
    let winding_evidence = match winding_evidence_report_for_request_from_graph_and_attempt(
        graph,
        left,
        right,
        request,
        arrangement_attempt.as_ref(),
        arrangement_cell_complex_shortcuts,
    ) {
        Ok(report) => report,
        Err(_) => {
            let geometry = graph.face_split_geometry_plan(left, right)?;
            let region_plan = geometry.region_plan(left, right);
            let region_classifications =
                checked_classify_face_regions_against_opposite_planes(&region_plan, left, right)?;
            let counts = retained_graph_counts(graph);
            winding_evidence_report(
                request.operation,
                ExactWindingEvidenceStatus::VolumetricAssemblyRequired,
                graph.has_unknowns(),
                graph.face_pairs.len(),
                graph.event_count(),
                region_plan.regions.len(),
                region_classifications,
                counts.into_blocker(ExactBooleanBlockerKind::Winding),
                None,
                coplanar_volumetric_evidence_if_required(graph, left, right),
            )
        }
    };
    Ok(ExactBooleanCertificationSet::from_reports(
        trivial,
        regularized_solid,
        refinement,
        boundary_touching,
        open_surface_disjoint,
        adjacent_union_completion,
        identical,
        same_surface,
        closed_winding_left_in_right,
        closed_winding_right_in_left,
        convex_left_in_right,
        convex_right_in_left,
        convex_capabilities,
        arrangement_cell_complex_shortcuts.clone(),
        planar_arrangement,
        winding_evidence,
        volumetric_boundary_closure,
        arrangement_attempt,
    ))
}

#[cfg(test)]
fn arrangement_attempt_for_certification_set(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_regularized_arrangement: Option<&ExactArrangement>,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
    arrangement_cell_complex_shortcuts: &ExactArrangementCellComplexShortcutFacts,
    adjacent_union_completion_certified: bool,
    reject_boundary_evidence_request: bool,
) -> Result<Option<ExactArrangementBooleanAttempt>, ExactMeshError> {
    if adjacent_union_completion_certified {
        return Ok(None);
    }

    let retained_arrangement_attempt_materializes_output = retained_arrangement_attempt
        .is_some_and(|attempt| {
            attempt.certifies_regularized_arrangement_cell_complex_output_for_request(request)
        });
    if let Some(attempt) = retained_arrangement_attempt
        && retained_arrangement_attempt_materializes_output
    {
        return Ok(Some(attempt.clone()));
    }

    let retained_arrangement_cell_complex_shortcut_attempt =
        retained_arrangement_attempt.filter(|attempt| {
            attempt.certifies_regularized_arrangement_cell_complex_shortcut_for_request(request)
        });
    let arrangement_cell_complex_shortcut_certified = arrangement_cell_complex_shortcuts
        .certified_support(request.operation)
        == Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
        && retained_arrangement_cell_complex_shortcut_attempt.is_some();
    if arrangement_cell_complex_shortcut_certified {
        return Ok(retained_arrangement_cell_complex_shortcut_attempt.cloned());
    }
    if let Some(attempt) = retained_arrangement_attempt {
        return Ok(Some(attempt.clone()));
    }

    if matches!(request.operation, ExactBooleanOperation::SelectedRegions(_))
        || reject_boundary_evidence_request
        || request.validation == ExactMeshValidationPolicy::CLOSED
    {
        return Ok(None);
    }

    if let Some(arrangement) = retained_regularized_arrangement {
        return arrangement_boolean_attempt_report_from_arrangement(
            left,
            right,
            request,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
            arrangement,
        )
        .map(Some);
    }

    let arrangement = ExactArrangement::from_intersection_graph_with_policy(
        graph.clone(),
        left,
        right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )?;
    arrangement_boolean_attempt_report_from_arrangement(
        left,
        right,
        request,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
        &arrangement,
    )
    .map(Some)
}

#[cfg(test)]
fn planar_arrangement_certification_report_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
) -> ExactPlanarArrangementReport {
    if matches!(request.operation, ExactBooleanOperation::SelectedRegions(_)) {
        return not_named_planar_arrangement_report(request.operation);
    }

    let mut arrangement_cell_complex_preflight: CertifiedArrangementCellComplexPreflightCache =
        None;
    planar_arrangement_report_from_graph_with_cell_complex_cache(
        graph,
        left,
        right,
        request.operation,
        &mut arrangement_cell_complex_preflight,
        Some(request),
        retained_arrangement_attempt,
    )
    .unwrap_or_else(|_| {
        planar_arrangement_report(
            request.operation,
            ExactPlanarArrangementStatus::NoPositiveOverlap,
            graph.has_unknowns(),
            graph.face_pairs.len(),
            graph.event_count(),
            retained_graph_counts(graph),
            None,
        )
    })
}

#[cfg(test)]
fn volumetric_boundary_closure_certification_report_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    adjacent_union_completion_certified: bool,
    reject_boundary_evidence_request: bool,
) -> Option<ExactVolumetricBoundaryClosureReport> {
    if matches!(request.operation, ExactBooleanOperation::SelectedRegions(_)) {
        return None;
    }
    if adjacent_union_completion_certified {
        return Some(no_materialized_boundary_output_report(request.operation));
    }
    if reject_boundary_evidence_request {
        return None;
    }

    let report =
        volumetric_boundary_closure_report_from_graph(graph, left, right, request.operation)
            .ok()?;
    if request.validation == ExactMeshValidationPolicy::CLOSED {
        return report.is_coplanar_closure_available().then_some(report);
    }
    Some(report)
}

fn graph_for_certified_materialization<'a>(
    retained_graph: Option<&'a ExactIntersectionGraph>,
    owned_graph: &'a mut Option<ExactIntersectionGraph>,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<&'a ExactIntersectionGraph, ExactMeshError> {
    if let Some(graph) = retained_graph {
        validate_graph_source_replay(graph, left, right)?;
        return Ok(graph);
    }
    if owned_graph.is_none() {
        *owned_graph = Some(super::graph::build_validated_intersection_graph(
            left, right,
        )?);
    }
    owned_graph.as_ref().ok_or_else(|| {
        exact_boolean_internal_error("certified materialization graph was not retained")
    })
}

fn graph_for_certified_materialization_with_prepared<'a>(
    retained_graph: Option<&'a ExactIntersectionGraph>,
    owned_graph: &'a mut Option<ExactIntersectionGraph>,
    prepared_graph: &'a mut Option<Rc<ExactIntersectionGraph>>,
    prepared_pair: Option<&PreparedMeshPair<'_, '_>>,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<&'a ExactIntersectionGraph, ExactMeshError> {
    if let Some(graph) = retained_graph {
        validate_graph_source_replay(graph, left, right)?;
        return Ok(graph);
    }
    if let Some(pair) = prepared_pair {
        if prepared_graph.is_none() {
            *prepared_graph = Some(build_validated_intersection_graph_from_prepared_pair(pair)?);
        }
        return prepared_graph.as_deref().ok_or_else(|| {
            exact_boolean_internal_error(
                "certified prepared materialization graph was not retained",
            )
        });
    }
    if owned_graph.is_none() {
        *owned_graph = Some(super::graph::build_validated_intersection_graph(
            left, right,
        )?);
    }
    owned_graph.as_ref().ok_or_else(|| {
        exact_boolean_internal_error("certified materialization graph was not retained")
    })
}

#[cfg(test)]
fn unsupported_certified_materialization_error(support: ExactBooleanSupport) -> ExactMeshError {
    ExactMeshError::one(ExactMeshBlocker::new(
        ExactMeshBlockerKind::UnsupportedCellMaterializer,
        format!("certified exact boolean support did not materialize: {support:?}"),
    ))
}

fn exact_boolean_internal_error(message: impl Into<String>) -> ExactMeshError {
    ExactMeshError::one(ExactMeshBlocker::new(
        ExactMeshBlockerKind::ExactConstructionFailure,
        message,
    ))
}

fn unsupported_boolean_operation_error(
    operation: ExactBooleanOperation,
    context: &'static str,
) -> ExactMeshError {
    ExactMeshError::one(ExactMeshBlocker::new(
        ExactMeshBlockerKind::UnsupportedExactOperation,
        format!("{context}: {operation:?}"),
    ))
}

#[cfg(test)]
pub(crate) fn try_materialize_certified_boolean_support_with_artifacts(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    support: ExactBooleanSupport,
    retained_graph: Option<&ExactIntersectionGraph>,
    retained_regularized_arrangement: Option<&ExactArrangement>,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let mut owned_graph = None;
    let result = match support {
        ExactBooleanSupport::SelectedRegionPolicy => {
            let ExactBooleanOperation::SelectedRegions(selection) = operation else {
                return Err(unsupported_certified_materialization_error(support));
            };
            let graph =
                graph_for_certified_materialization(retained_graph, &mut owned_graph, left, right)?;
            Some(replay_selected_region_boolean_result_from_graph(
                graph, left, right, selection, validation,
            )?)
        }
        ExactBooleanSupport::CertifiedBoundaryPolicyShortcut => {
            let graph =
                graph_for_certified_materialization(retained_graph, &mut owned_graph, left, right)?;
            let boundary_policy = request.boundary_policy;
            if boundary_policy != ExactBoundaryBooleanPolicy::Reject {
                let Some(result) = boolean_boundary_touching_meshes_from_graph(
                    graph,
                    left,
                    right,
                    operation,
                    validation,
                    boundary_policy,
                )?
                else {
                    return Ok(None);
                };
                return Ok(request_replayable_result(
                    Some(result),
                    left,
                    right,
                    ExactBooleanRequest::with_boundary_policy(
                        operation,
                        validation,
                        boundary_policy,
                    ),
                    retained_arrangement_attempt,
                ));
            }
            materialize_graph_shortcut_from_graph_for_request(
                graph,
                left,
                right,
                request,
                support,
                retained_arrangement_attempt,
            )?
        }
        ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
        | ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
        | ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference => {
            let graph =
                graph_for_certified_materialization(retained_graph, &mut owned_graph, left, right)?;
            if let Some(result) = open_surface_arrangement_result_from_graph(
                graph, left, right, operation, validation,
            )? && result.validate_against_sources(left, right).is_ok()
            {
                return Ok(Some(result));
            }
            materialize_certified_arrangement_cell_complex_support_with_arrangement(
                left,
                right,
                request,
                Some(graph),
                retained_regularized_arrangement,
                retained_arrangement_attempt,
                shortcut_facts,
            )?
        }
        ExactBooleanSupport::CertifiedArrangementCellComplex => {
            materialize_certified_arrangement_cell_complex_support_with_arrangement(
                left,
                right,
                request,
                retained_graph,
                retained_regularized_arrangement,
                retained_arrangement_attempt,
                shortcut_facts,
            )?
        }
        ExactBooleanSupport::CertifiedEmptyOperand => {
            if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
                || (!left.triangles().is_empty() && !right.triangles().is_empty())
            {
                None
            } else {
                request_replayable_result(
                    Some(boolean_empty_operand(left, right, operation, validation)?),
                    left,
                    right,
                    ExactBooleanRequest::with_boundary_policy(
                        operation,
                        validation,
                        ExactBoundaryBooleanPolicy::Reject,
                    ),
                    retained_arrangement_attempt,
                )
            }
        }
        ExactBooleanSupport::CertifiedBoundsDisjoint => {
            if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
                || left.triangles().is_empty()
                || right.triangles().is_empty()
                || !meshes_are_certified_bounds_disjoint(left, right)
                || closed_validation_regularized_solid_support(left, right, operation, validation)
                    .is_some()
            {
                None
            } else {
                request_replayable_result(
                    Some(boolean_disjoint_meshes(left, right, operation, validation)?),
                    left,
                    right,
                    ExactBooleanRequest::with_boundary_policy(
                        operation,
                        validation,
                        ExactBoundaryBooleanPolicy::Reject,
                    ),
                    retained_arrangement_attempt,
                )
            }
        }
        ExactBooleanSupport::CertifiedIdentical => {
            if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
                || (left.facts().mesh.closed_manifold && right.facts().mesh.closed_manifold)
                || closed_validation_regularized_solid_support(left, right, operation, validation)
                    .is_some()
                || !evidence::meshes_are_certified_identical(left, right)
            {
                None
            } else {
                request_replayable_result(
                    Some(boolean_identical_meshes(left, operation, validation)?),
                    left,
                    right,
                    ExactBooleanRequest::with_boundary_policy(
                        operation,
                        validation,
                        ExactBoundaryBooleanPolicy::Reject,
                    ),
                    retained_arrangement_attempt,
                )
            }
        }
        ExactBooleanSupport::CertifiedSameSurface => {
            if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
                || (left.facts().mesh.closed_manifold && right.facts().mesh.closed_manifold)
                || closed_validation_regularized_solid_support(left, right, operation, validation)
                    .is_some()
                || evidence::meshes_are_certified_identical(left, right)
                || !evidence::meshes_are_certified_same_surface(left, right)
            {
                None
            } else {
                request_replayable_result(
                    Some(boolean_same_surface_meshes(left, operation, validation)?),
                    left,
                    right,
                    ExactBooleanRequest::with_boundary_policy(
                        operation,
                        validation,
                        ExactBoundaryBooleanPolicy::Reject,
                    ),
                    retained_arrangement_attempt,
                )
            }
        }
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
        | ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
        | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference => {
            let graph =
                graph_for_certified_materialization(retained_graph, &mut owned_graph, left, right)?;
            materialize_closed_boundary_or_no_volume_overlap_from_graph(
                graph, left, right, operation, validation,
            )?
        }
        ExactBooleanSupport::CertifiedOpenSurfaceDisjoint => {
            let graph =
                graph_for_certified_materialization(retained_graph, &mut owned_graph, left, right)?;
            materialize_graph_shortcut_from_graph_for_request(
                graph,
                left,
                right,
                request,
                support,
                retained_arrangement_attempt,
            )?
        }
        ExactBooleanSupport::CertifiedClosedWindingSeparated
        | ExactBooleanSupport::CertifiedClosedWindingContainment => {
            let graph =
                graph_for_certified_materialization(retained_graph, &mut owned_graph, left, right)?;
            materialize_graph_shortcut_from_graph_for_request(
                graph,
                left,
                right,
                request,
                support,
                retained_arrangement_attempt,
            )?
        }
        ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid => {
            if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
                || certified_mixed_dimensional_regularized_solid_support(left, right).is_none()
                || (validation != ExactMeshValidationPolicy::CLOSED
                    && meshes_are_certified_bounds_disjoint(left, right))
            {
                None
            } else {
                request_replayable_result(
                    boolean_closed_regularized_lower_dimensional_optional(
                        left, right, operation, validation,
                    )?,
                    left,
                    right,
                    ExactBooleanRequest::with_boundary_policy(
                        operation,
                        validation,
                        ExactBoundaryBooleanPolicy::Reject,
                    ),
                    retained_arrangement_attempt,
                )
            }
        }
        ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid => {
            if request_uses_arrangement_lower_dimensional_regularized_shortcut(request)
                && let Some(result) =
                    materialize_certified_arrangement_cell_complex_support_with_arrangement(
                        left,
                        right,
                        request,
                        retained_graph,
                        retained_regularized_arrangement,
                        retained_arrangement_attempt,
                        shortcut_facts,
                    )?
            {
                Some(result)
            } else {
                request_replayable_result(
                    boolean_closed_regularized_lower_dimensional_optional(
                        left, right, operation, validation,
                    )?,
                    left,
                    right,
                    ExactBooleanRequest::with_boundary_policy(
                        operation,
                        validation,
                        ExactBoundaryBooleanPolicy::Reject,
                    ),
                    retained_arrangement_attempt,
                )
            }
        }
        ExactBooleanSupport::CertifiedConvexUnion
        | ExactBooleanSupport::CertifiedConvexIntersection
        | ExactBooleanSupport::CertifiedConvexDifference => request_replayable_result(
            boolean_convex_meshes_optional(left, right, operation, validation)?,
            left,
            right,
            ExactBooleanRequest::with_boundary_policy(
                operation,
                validation,
                ExactBoundaryBooleanPolicy::Reject,
            ),
            retained_arrangement_attempt,
        ),
        ExactBooleanSupport::CertifiedConvexSeparated
        | ExactBooleanSupport::CertifiedConvexContainment => {
            let graph =
                graph_for_certified_materialization(retained_graph, &mut owned_graph, left, right)?;
            request_replayable_result(
                boolean_convex_relation_meshes_optional_from_graph(
                    graph, left, right, operation, validation,
                )?,
                left,
                right,
                ExactBooleanRequest::with_boundary_policy(
                    operation,
                    validation,
                    ExactBoundaryBooleanPolicy::Reject,
                ),
                retained_arrangement_attempt,
            )
        }
        ExactBooleanSupport::RequiresBoundaryPolicy
        | ExactBooleanSupport::RequiresPlanarArrangement
        | ExactBooleanSupport::RequiresCoplanarVolumetricCells
        | ExactBooleanSupport::RequiresCertifiedWinding
        | ExactBooleanSupport::UnresolvedGraph => None,
    };
    if result.is_none()
        && !matches!(
            support,
            ExactBooleanSupport::CertifiedArrangementCellComplex
        )
    {
        return Err(unsupported_certified_materialization_error(support));
    }
    Ok(result)
}

fn materialize_certified_arrangement_cell_complex_support_with_arrangement(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_graph: Option<&ExactIntersectionGraph>,
    retained_regularized_arrangement: Option<&ExactArrangement>,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let retained_arrangement_attempt = retained_arrangement_attempt_for_request(
        retained_arrangement_attempt,
        left,
        right,
        request,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .map_err(|error| {
        retained_evidence_validation_error(
            "retained arrangement attempt failed validation",
            error,
            ExactMeshBlockerKind::ExactConstructionFailure,
        )
    })?;
    if shortcut_facts.certified_support(operation)
        == Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
        && let Some(result) =
            boolean_arrangement_cell_complex_recovery(left, right, operation, validation)?
    {
        return Ok(Some(result));
    }
    let mut owned_graph = None;
    let graph = graph_for_certified_materialization(retained_graph, &mut owned_graph, left, right)?;
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
        replay_generic_arrangement_cell_complex_result(left, right, operation, validation)?
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
            )
        {
            return Ok(Some(*result));
        }
    }
    if operation == ExactBooleanOperation::Union
        && let Some((result, _report)) =
            materialize_adjacent_union_completion_from_graph_for_request(
                graph, left, right, request,
            )?
    {
        return Ok(Some(result));
    }
    if let Some((result, _closure)) =
        materialize_volumetric_coplanar_boundary_closure_boolean_from_graph(
            graph, left, right, operation, validation,
        )?
    {
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
    if let Some(result) = certified_arrangement_cell_complex_result_from_graph(
        graph, left, right, operation, validation, true,
    )? {
        return Ok(Some(result));
    }
    if let Some(result) = request_replayable_result(
        boolean_arrangement_cell_complex_recovery(left, right, operation, validation)?,
        left,
        right,
        ExactBooleanRequest::with_boundary_policy(
            operation,
            validation,
            ExactBoundaryBooleanPolicy::Reject,
        ),
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
    if shortcut_facts.certified_support(operation)
        == Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
    {
        return Ok(request_replayable_result(
            boolean_arrangement_cell_complex_recovery(left, right, operation, validation)?,
            left,
            right,
            ExactBooleanRequest::with_boundary_policy(
                operation,
                validation,
                ExactBoundaryBooleanPolicy::Reject,
            ),
            retained_arrangement_attempt,
        ));
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
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!("retained arrangement attempt failed replay: {error:?}"),
            ))
        })?;
    if attempt.materialized_arrangement_cell_complex_shortcut() {
        let Some(result) = boolean_arrangement_cell_complex_recovery(
            left,
            right,
            request.operation,
            request.validation,
        )?
        else {
            return Ok(None);
        };
        return if arrangement_cell_complex_result_matches_retained_attempt(&result, attempt) {
            Ok(Some(result))
        } else {
            Ok(None)
        };
    }
    let Some(result) = rematerialize_retained_arrangement_cell_complex_attempt(request, attempt)?
    else {
        return Ok(None);
    };
    if arrangement_cell_complex_result_matches_retained_attempt(&result, attempt) {
        Ok(Some(result))
    } else {
        Ok(None)
    }
}

pub(crate) fn rematerialize_retained_arrangement_cell_complex_attempt(
    request: ExactBooleanRequest,
    attempt: &ExactArrangementBooleanAttempt,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if !attempt.materialized_arrangement_cell_complex_output()
        || attempt.materialized_arrangement_cell_complex_shortcut()
    {
        return Ok(None);
    }
    let Some(simplified) = attempt.simplified_cell_complex_with_retained_gate_reports() else {
        return Ok(None);
    };
    if simplified.operation != request.operation || simplified.validate().is_err() {
        return Ok(None);
    }
    rematerialize_simplified_arrangement_cell_complex(request, simplified)
}

fn rematerialize_simplified_arrangement_cell_complex(
    request: ExactBooleanRequest,
    simplified: &ExactSimplifiedCellComplex,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if simplified.operation != request.operation || simplified.validate().is_err() {
        return Ok(None);
    }
    let mesh = match simplified.triangulate() {
        Ok(mesh) => mesh,
        Err(_) => return Ok(None),
    };
    let mesh = match copy_mesh(
        &mesh,
        "exact arrangement cell-complex boolean result",
        request.validation,
    ) {
        Ok(mesh) => mesh,
        Err(_) if request.validation == ExactMeshValidationPolicy::CLOSED => {
            let Some(mesh) = close_exact_coplanar_boundary_loops(
                &mesh,
                "exact arrangement cell-complex closed coplanar-boundary result",
                request.validation,
            )
            .ok()
            .flatten() else {
                return Ok(None);
            };
            mesh
        }
        Err(_) => return Ok(None),
    };
    let result = certified_shortcut_result(
        mesh,
        request.operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    )
    .with_gate_reports(
        simplified.topology_assembly_report.clone(),
        simplified.region_ownership_report.clone(),
    );
    Ok(Some(result))
}

pub(crate) fn replay_generic_arrangement_cell_complex_result(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let policy = ExactRegularizationPolicy::REGULARIZED_SOLID;
    let arrangement = match ExactArrangement::from_meshes_with_policy(left, right, policy) {
        Ok(arrangement) => arrangement,
        Err(_) => return Ok(None),
    };
    let selected = match select_arrangement_for_replay(arrangement, left, right, operation, policy)
    {
        Ok(selected) => selected,
        Err(_) => return Ok(None),
    };
    let simplified = match selected.simplify_exact_with_policy(policy) {
        Ok(simplified) => simplified,
        Err(_) => return Ok(None),
    };
    let request = ExactBooleanRequest::new(operation, validation);
    let Some(result) = rematerialize_simplified_arrangement_cell_complex(request, &simplified)?
    else {
        return Ok(None);
    };
    if result.validate().is_ok() {
        Ok(Some(result))
    } else {
        Ok(None)
    }
}

fn materialize_selected_region_result_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    selection: ExactRegionSelection,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactBooleanResult, ExactMeshError> {
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
    let mut assembly = ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
        &triangulations,
        selection,
        left,
        right,
    )
    .map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::IndexOutOfBounds,
            format!("exact boolean assembly failed: {error}"),
        ))
    })?;
    assembly
        .canonicalize_for_mesh_with_sources(left, right)
        .map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::IndexOutOfBounds,
                format!("exact boolean assembly canonicalization failed: {error}"),
            ))
        })?;
    let mesh = assembly.checked_to_exact_mesh_with_sources(left, right, validation)?;

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
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("exact selected-region result validation failed: {error:?}"),
        ))
    })?;
    Ok(result)
}

pub(crate) fn replay_selected_region_boolean_result(
    left: &ExactMesh,
    right: &ExactMesh,
    selection: ExactRegionSelection,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactBooleanResult, ExactMeshError> {
    let graph = build_validated_intersection_graph(left, right)?;
    replay_selected_region_boolean_result_from_graph(&graph, left, right, selection, validation)
}

fn replay_selected_region_boolean_result_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    selection: ExactRegionSelection,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactBooleanResult, ExactMeshError> {
    validate_graph_source_replay(graph, left, right)?;
    let result =
        materialize_selected_region_result_from_graph(graph, left, right, selection, validation)?;
    if !matches!(
        result.kind(),
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

/// Preflight an exact boolean operation without materializing output topology.
///
/// The preflight path deliberately shares the exact graph, region, and
/// classification stages with the executable arrangement pipeline. For named
/// booleans that still need unresolved inside/outside semantics, it returns
/// [`ExactBooleanSupport::RequiresCertifiedWinding`] with replayable facts
/// instead of approximating them.
fn initial_reject_boundary_preflight_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> ExactBooleanSupport {
    match operation {
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
                && evidence::meshes_are_certified_identical(left, right) =>
        {
            ExactBooleanSupport::CertifiedIdentical
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference
            if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
                && evidence::meshes_are_certified_same_surface(left, right) =>
        {
            ExactBooleanSupport::CertifiedSameSurface
        }
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => shortcut_facts
            .certified_support(operation)
            .or_else(|| certified_mixed_dimensional_regularized_solid_support(left, right))
            .unwrap_or(ExactBooleanSupport::RequiresCertifiedWinding),
    }
}

fn preflight_boolean_exact_reject_boundary_policy_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_attempt: Option<&ExactArrangementBooleanAttempt>,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<ExactBooleanPreflight, ExactMeshError> {
    let operation = request.operation;
    let support = initial_reject_boundary_preflight_support(left, right, operation, shortcut_facts);
    let requires_certified_winding = support == ExactBooleanSupport::RequiresCertifiedWinding;
    if support == ExactBooleanSupport::CertifiedArrangementCellComplex {
        return Ok(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        ));
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
                | ExactBooleanSupport::CertifiedBoundaryPolicyShortcut
        )
    {
        return Ok(certified_preflight(operation, support, Some(graph), None));
    }
    if let Some(preflight) = certified_closed_winding_separated_preflight_from_empty_graph(
        graph, left, right, operation,
    )? {
        return Ok(preflight);
    }

    if operation == ExactBooleanOperation::Difference
        && shortcut_facts.certifies_axis_aligned_box_pair()
    {
        return Ok(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        ));
    }
    if operation == ExactBooleanOperation::Difference
        && let Some(evidence) = coplanar_boundary_only_evidence_if_consumed(graph, left, right)?
    {
        return Ok(certified_preflight(
            operation,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            Some(graph),
            Some(evidence),
        ));
    }
    let graph_had_unknowns = graph.has_unknowns();
    let retained_face_pairs = graph.face_pairs.len();
    let retained_events = graph.event_count();
    let relation_counts = retained_graph_counts(graph);
    let requires_coplanar_volumetric_cells =
        graph_requires_coplanar_volumetric_cells_for_sources(graph, left, right);
    let mut certified_arrangement_preflight = None;
    if graph_had_unknowns || relation_counts.construction_failed_events() > 0 {
        return Ok(ExactBooleanPreflight::new(
            operation,
            ExactBooleanSupport::UnresolvedGraph,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            0,
            Vec::new(),
            Some(relation_counts.into_blocker(ExactBooleanBlockerKind::Refinement)),
            None,
            None,
        ));
    }
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
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
            support,
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
        ));
    }
    if requires_certified_winding
        && operation == ExactBooleanOperation::Union
        && let Some(preflight) =
            certified_closed_boundary_only_contact_preflight(graph, left, right, operation)?
    {
        return Ok(preflight);
    }
    if requires_certified_winding
        && coplanar_boundary_only_evidence_if_consumed(graph, left, right)
            .ok()
            .flatten()
            .is_none()
        && materialize_closed_boundary_or_no_volume_overlap_from_graph(
            graph,
            left,
            right,
            operation,
            request.validation,
        )
        .ok()
        .flatten()
        .is_some()
        && let Some(boundary_support) = certified_closed_boundary_touching_support(operation)
    {
        return Ok(certified_preflight(
            operation,
            boundary_support,
            Some(graph),
            None,
        ));
    }
    if requires_certified_winding
        && operation == ExactBooleanOperation::Intersection
        // The empty cavity case can have overlapping AABBs and no graph
        // events, so this retained evidence witness is checked before falling
        // through to winding.
        && has_empty_axis_aligned_orthogonal_solid_cell_intersection(left, right)
    {
        return Ok(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        ));
    }
    if requires_certified_winding
        && let Some(preflight) =
            lower_dimensional_intersection_arrangement_preflight(graph, left, right, request)
    {
        return Ok(preflight);
    }
    if let Some(preflight) =
        open_surface_arrangement_preflight_from_graph(graph, left, right, operation)
    {
        return Ok(preflight);
    }
    if let Some(preflight) = boundary_or_planar_report_preflight_from_graph(
        graph,
        left,
        right,
        request,
        retained_attempt,
        &mut certified_arrangement_preflight,
    )? {
        return Ok(preflight);
    }
    let convex_operation_preflight_allowed = match operation {
        ExactBooleanOperation::Intersection => !requires_coplanar_volumetric_cells,
        ExactBooleanOperation::Union | ExactBooleanOperation::Difference => true,
        ExactBooleanOperation::SelectedRegions(_) => false,
    };
    if requires_certified_winding
        && convex_operation_preflight_allowed
        && let Some(preflight) = certified_convex_operation_preflight(left, right, operation, graph)
    {
        return Ok(preflight);
    }
    if requires_coplanar_volumetric_cells {
        if request.validation == ExactMeshValidationPolicy::CLOSED
            && let Some(preflight) =
                certified_coplanar_boundary_closure_preflight(graph, left, right, operation)
        {
            return Ok(preflight);
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
        ) && let Some(preflight) =
            certified_convex_operation_preflight(left, right, operation, graph)
        {
            return Ok(preflight);
        }
        let winding_evidence = winding_evidence_report_from_graph(graph, left, right, operation)?;
        if winding_evidence.status().routes_to_certified_winding()
            && winding_evidence.blocker().kind() == ExactBooleanBlockerKind::CoplanarVolumetricCells
        {
            return Ok(winding_evidence
                .into_preflight(ExactBooleanSupport::RequiresCertifiedWinding, true));
        }
        return Ok(ExactBooleanPreflight::new(
            operation,
            ExactBooleanSupport::RequiresCoplanarVolumetricCells,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            0,
            Vec::new(),
            Some(relation_counts.into_blocker(ExactBooleanBlockerKind::CoplanarVolumetricCells)),
            None,
            coplanar_volumetric_evidence_if_required(graph, left, right),
        ));
    }
    if support == ExactBooleanSupport::RequiresBoundaryPolicy {
        return Ok(ExactBooleanPreflight::new(
            operation,
            support,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            0,
            Vec::new(),
            Some(relation_counts.into_blocker(ExactBooleanBlockerKind::BoundaryPolicy)),
            None,
            None,
        ));
    }

    let winding_report = match winding_evidence_report_from_graph(graph, left, right, operation) {
        Ok(report) => report,
        Err(_) => {
            return region_plan_preflight_from_graph(
                graph,
                left,
                right,
                operation,
                support,
                Some(relation_counts.into_blocker(ExactBooleanBlockerKind::Winding)),
                coplanar_volumetric_evidence_if_required(graph, left, right),
            );
        }
    };
    if winding_report
        .status()
        .materializes_arrangement_cell_complex()
        || (winding_report.status() == ExactWindingEvidenceStatus::Ready
            && materialize_volumetric_winding_region_plan_from_graph(
                graph,
                left,
                right,
                operation,
                ExactMeshValidationPolicy::CLOSED,
            )
            .ok()
            .flatten()
            .is_some())
        || materialize_closed_volumetric_winding_boundary_caps_from_graph(
            graph, left, right, operation,
        )
        .ok()
        .flatten()
        .is_some()
    {
        return Ok(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        ));
    }

    Ok(winding_report.into_preflight(support, false))
}

fn certified_winding_shortcut_preflight_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_attempt: Option<&ExactArrangementBooleanAttempt>,
    fallback_support: ExactBooleanSupport,
    requires_coplanar_volumetric_cells: bool,
    certified_arrangement_preflight: &mut CertifiedArrangementCellComplexPreflightCache,
) -> Result<Option<ExactBooleanPreflight>, ExactMeshError> {
    let operation = request.operation;
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && open_surface_disjoint_report_from_graph(graph, left, right).is_certified()
    {
        return Ok(Some(certified_preflight(
            operation,
            ExactBooleanSupport::CertifiedOpenSurfaceDisjoint,
            Some(graph),
            None,
        )));
    }
    if requires_coplanar_volumetric_cells
        && let Some(preflight) =
            certified_coplanar_boundary_closure_preflight(graph, left, right, operation)
    {
        return Ok(Some(preflight));
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
    if let Some(preflight) = certified_convex_operation_preflight(left, right, operation, graph) {
        return Ok(Some(preflight));
    }
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && certified_closed_winding_containment_relation_from_graph(graph, left, right)?.is_some()
    {
        return Ok(Some(certified_preflight(
            operation,
            ExactBooleanSupport::CertifiedClosedWindingContainment,
            Some(graph),
            None,
        )));
    }
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && closed_zero_area_boundary_contact_evidence_from_graph(graph, left, right)?.is_some()
    {
        let Some(boundary_support) = certified_closed_boundary_touching_support(operation) else {
            return Ok(Some(certified_preflight(
                operation,
                fallback_support,
                Some(graph),
                None,
            )));
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

fn boundary_or_planar_report_preflight_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_attempt: Option<&ExactArrangementBooleanAttempt>,
    certified_arrangement_preflight: &mut CertifiedArrangementCellComplexPreflightCache,
) -> Result<Option<ExactBooleanPreflight>, ExactMeshError> {
    let operation = request.operation;
    let boundary_report = boundary_touching_report_from_graph(graph, left, right).ok();
    if let Some(boundary_report) = boundary_report
        && boundary_report.is_certified()
    {
        return Ok(Some(boundary_policy_required_preflight_from_report(
            operation,
            &boundary_report,
        )));
    }

    let planar_report = planar_arrangement_report_from_graph(graph, left, right, operation).ok();
    if let Some(planar_report) = planar_report.as_ref()
        && planar_report.is_required()
    {
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
        return Ok(Some(planar_arrangement_required_preflight_from_report(
            operation,
            planar_report,
        )));
    }
    if planar_report
        .as_ref()
        .is_some_and(ExactPlanarArrangementReport::is_already_materialized)
        && let Some(preflight) = cached_certified_arrangement_cell_complex_preflight(
            certified_arrangement_preflight,
            operation,
            graph,
            left,
            right,
            Some(request),
            retained_attempt,
        )?
    {
        return Ok(Some(preflight));
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
    let boundary_policy = request.boundary_policy;
    if let Some(support) =
        closed_validation_regularized_solid_support(left, right, operation, validation)
        && !(support == ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid
            && operation == ExactBooleanOperation::Intersection)
    {
        return Ok(certified_preflight(operation, support, Some(graph), None));
    }
    let mut preflight = preflight_boolean_exact_reject_boundary_policy_from_graph(
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
        && report.is_certified()
    {
        return Ok(certified_arrangement_cell_complex_preflight(
            operation, graph, left, right,
        ));
    }
    if boundary_policy != ExactBoundaryBooleanPolicy::Reject
        && !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && !preflight.support().is_certified()
        && boolean_boundary_touching_meshes_from_graph(
            graph,
            left,
            right,
            operation,
            validation,
            boundary_policy,
        )?
        .is_some()
    {
        return Ok(certified_preflight(
            operation,
            ExactBooleanSupport::CertifiedBoundaryPolicyShortcut,
            Some(graph),
            None,
        ));
    }
    if validation != ExactMeshValidationPolicy::CLOSED
        && !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && matches!(
            preflight.support(),
            ExactBooleanSupport::RequiresCertifiedWinding
                | ExactBooleanSupport::RequiresCoplanarVolumetricCells
        )
        && materialize_arrangement_volumetric_split_cell_result_from_graph(
            graph, left, right, operation, validation,
        )?
        .is_some()
    {
        preflight = certified_arrangement_cell_complex_preflight(operation, graph, left, right);
    }
    if boundary_policy == ExactBoundaryBooleanPolicy::Reject
        || matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || preflight.support() != ExactBooleanSupport::RequiresBoundaryPolicy
    {
        return Ok(preflight);
    }
    if boolean_boundary_touching_meshes_from_graph(
        graph,
        left,
        right,
        operation,
        validation,
        boundary_policy,
    )?
    .is_some()
    {
        return Ok(certified_preflight(
            operation,
            ExactBooleanSupport::CertifiedBoundaryPolicyShortcut,
            Some(graph),
            None,
        ));
    }
    Ok(preflight)
}

pub(crate) fn volumetric_boundary_closure_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactVolumetricBoundaryClosureReport, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(no_materialized_boundary_output_report(operation));
    }

    let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        graph,
        left,
        right,
        operation,
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )?
    else {
        return Ok(no_materialized_boundary_output_report(operation));
    };
    volumetric_boundary_closure_report_from_materialized(&materialized, operation)
}

pub(crate) fn no_materialized_boundary_output_report(
    operation: ExactBooleanOperation,
) -> ExactVolumetricBoundaryClosureReport {
    ExactVolumetricBoundaryClosureReport::new(
        operation,
        ExactVolumetricBoundaryClosureStatus::NoMaterializedBoundaryOutput,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
}

fn volumetric_boundary_closure_report_from_materialized(
    materialized: &MaterializedVolumetricWindingRegionPlan,
    operation: ExactBooleanOperation,
) -> Result<ExactVolumetricBoundaryClosureReport, ExactMeshError> {
    volumetric_boundary_closure_report_from_materialized_with_prevalidated_closure(
        materialized,
        operation,
        None,
    )
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
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::ExactConstructionFailure,
                format!("volumetric boundary closure source mesh validation failed: {error:?}"),
            ))
        })?;
    let output_triangles = materialized.mesh.triangles().len();
    let boundary_edges = materialized.mesh.facts().mesh.boundary_edges;
    if materialized.mesh.facts().mesh.closed_manifold || boundary_edges == 0 {
        return Ok(ExactVolumetricBoundaryClosureReport::new(
            operation,
            ExactVolumetricBoundaryClosureStatus::AlreadyClosed,
            output_triangles,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ));
    }
    let boundary_topology = boundary_topology_evidence(&materialized.mesh);
    let Some(boundary_loops) = directed_boundary_loops(&materialized.mesh) else {
        return Ok(ExactVolumetricBoundaryClosureReport::new(
            operation,
            ExactVolumetricBoundaryClosureStatus::BoundaryTopologyNotLoop,
            output_triangles,
            boundary_edges,
            0,
            boundary_topology.invalid_outgoing_degree_vertices,
            boundary_topology.invalid_incoming_degree_vertices,
            boundary_topology.overused_edges,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ));
    };
    let boundary_points = boundary_loops
        .iter()
        .map(|boundary_loop| {
            boundary_loop
                .iter()
                .map(|&vertex| materialized.mesh.vertices().get(vertex).cloned())
                .collect::<Option<Vec<_>>>()
        })
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::IndexOutOfBounds,
                "volumetric boundary closure report referenced a missing output vertex",
            ))
        })?;
    let boundary_points = boundary_points
        .into_iter()
        .map(split_boundary_self_contact_cycles)
        .collect::<Result<Vec<_>, _>>()
        .map(|split| split.into_iter().flatten().collect::<Vec<_>>())
        .map_err(|blocker| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::ExactConstructionFailure,
                format!(
                    "volumetric boundary closure self-contact canonicalization failed: {blocker:?}"
                ),
            ))
        })?;
    let mut self_contact = BoundaryLoopSelfContactEvidence::default();
    for boundary in &boundary_points {
        match boundary_loop_self_contact_evidence(boundary) {
            Ok(evidence) => self_contact.add(evidence),
            Err(blocker) => {
                return Ok(ExactVolumetricBoundaryClosureReport::new(
                    operation,
                    ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(blocker),
                    output_triangles,
                    boundary_edges,
                    boundary_loops.len(),
                    0,
                    0,
                    0,
                    0,
                    self_contact.repeated_exact_point_pairs,
                    self_contact.exact_points,
                    self_contact.topological_vertices,
                    self_contact.degenerate_cycles,
                    self_contact.nondegenerate_cycles,
                    0,
                ));
            }
        }
    }
    if self_contact.repeated_exact_point_pairs != 0 {
        return Ok(ExactVolumetricBoundaryClosureReport::new(
            operation,
            ExactVolumetricBoundaryClosureStatus::BoundaryLoopExactSelfContact,
            output_triangles,
            boundary_edges,
            boundary_loops.len(),
            0,
            0,
            0,
            0,
            self_contact.repeated_exact_point_pairs,
            self_contact.exact_points,
            self_contact.topological_vertices,
            self_contact.degenerate_cycles,
            self_contact.nondegenerate_cycles,
            0,
        ));
    }
    let repeated_exact_boundary_points = self_contact.repeated_exact_point_pairs;
    let mut noncoplanar_boundary_loops = 0;
    for boundary in &boundary_points {
        match exact_loop_is_coplanar(boundary) {
            Ok(true) => {}
            Ok(false) => noncoplanar_boundary_loops += 1,
            Err(blocker) => {
                return Ok(ExactVolumetricBoundaryClosureReport::new(
                    operation,
                    ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(blocker),
                    output_triangles,
                    boundary_edges,
                    boundary_loops.len(),
                    0,
                    0,
                    0,
                    noncoplanar_boundary_loops,
                    repeated_exact_boundary_points,
                    self_contact.exact_points,
                    self_contact.topological_vertices,
                    self_contact.degenerate_cycles,
                    self_contact.nondegenerate_cycles,
                    0,
                ));
            }
        }
    }
    if noncoplanar_boundary_loops != 0 {
        return Ok(ExactVolumetricBoundaryClosureReport::new(
            operation,
            ExactVolumetricBoundaryClosureStatus::NonCoplanarBoundaryClosureRequired,
            output_triangles,
            boundary_edges,
            boundary_loops.len(),
            0,
            0,
            0,
            noncoplanar_boundary_loops,
            repeated_exact_boundary_points,
            self_contact.exact_points,
            self_contact.topological_vertices,
            self_contact.degenerate_cycles,
            self_contact.nondegenerate_cycles,
            0,
        ));
    }
    match group_exact_coplanar_loops(boundary_points) {
        Ok(groups) => {
            let coplanar_loop_groups = groups.len();
            let coplanar_closure_available = match prevalidated_coplanar_closure_available {
                Some(available) => available,
                None => close_exact_coplanar_boundary_loops_from_loops(
                    &materialized.mesh,
                    boundary_loops.clone(),
                    "exact volumetric boundary closure certification cap",
                    ExactMeshValidationPolicy::CLOSED,
                )?
                .is_some(),
            };
            if coplanar_closure_available {
                Ok(ExactVolumetricBoundaryClosureReport::new(
                    operation,
                    ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
                    output_triangles,
                    boundary_edges,
                    boundary_loops.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    coplanar_loop_groups,
                ))
            } else {
                Ok(ExactVolumetricBoundaryClosureReport::new(
                    operation,
                    ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(
                        ExactArrangementBlocker::NonManifoldCellComplex,
                    ),
                    output_triangles,
                    boundary_edges,
                    boundary_loops.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    coplanar_loop_groups,
                ))
            }
        }
        Err(blocker) => Ok(ExactVolumetricBoundaryClosureReport::new(
            operation,
            ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(blocker),
            output_triangles,
            boundary_edges,
            boundary_loops.len(),
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )),
    }
}

fn certified_preflight(
    operation: ExactBooleanOperation,
    support: ExactBooleanSupport,
    graph: Option<&super::graph::ExactIntersectionGraph>,
    coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
) -> ExactBooleanPreflight {
    let (graph_had_unknowns, retained_face_pairs, retained_events) =
        graph.map_or((false, 0, 0), |graph| {
            (
                graph.has_unknowns(),
                graph.face_pairs.len(),
                graph.event_count(),
            )
        });
    ExactBooleanPreflight::new(
        operation,
        support,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
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
) -> ExactBooleanPreflight {
    certified_preflight(
        operation,
        ExactBooleanSupport::CertifiedArrangementCellComplex,
        Some(graph),
        certified_arrangement_cell_complex_coplanar_evidence(graph, left, right),
    )
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
    Ok(ExactBooleanPreflight::new(
        operation,
        support,
        graph.has_unknowns(),
        graph.face_pairs.len(),
        graph.event_count(),
        region_plan.regions.len(),
        region_classifications,
        blocker,
        None,
        coplanar_volumetric_evidence,
    ))
}

fn open_surface_arrangement_preflight_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanPreflight> {
    let (support, region_classifications, _triangulations) =
        open_surface_arrangement_plan_from_graph(graph, left, right, operation)
            .ok()
            .flatten()?;
    let region_count = unique_classified_region_count(&region_classifications);
    Some(ExactBooleanPreflight::new(
        operation,
        support,
        graph.has_unknowns(),
        graph.face_pairs.len(),
        graph.event_count(),
        region_count,
        region_classifications,
        None,
        None,
        None,
    ))
}

fn lower_dimensional_intersection_arrangement_preflight(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Option<ExactBooleanPreflight> {
    if request.operation != ExactBooleanOperation::Intersection
        || request.validation != ExactMeshValidationPolicy::CLOSED
        || closed_regularized_operand_kind(left)
            != Some(ClosedRegularizedOperandKind::LowerDimensional)
        || closed_regularized_operand_kind(right)
            != Some(ClosedRegularizedOperandKind::LowerDimensional)
        || graph.face_pairs.is_empty()
        || open_surface_arrangement_plan_from_graph(graph, left, right, request.operation)
            .ok()
            .flatten()
            .is_none()
    {
        return None;
    }
    Some(certified_arrangement_cell_complex_preflight(
        request.operation,
        graph,
        left,
        right,
    ))
}

fn boundary_policy_required_preflight_from_report(
    operation: ExactBooleanOperation,
    report: &ExactBoundaryTouchingReport,
) -> ExactBooleanPreflight {
    ExactBooleanPreflight::new(
        operation,
        ExactBooleanSupport::RequiresBoundaryPolicy,
        report.graph_had_unknowns(),
        report.retained_face_pairs(),
        report.retained_events(),
        0,
        Vec::new(),
        Some(*report.blocker()),
        None,
        None,
    )
}

fn planar_arrangement_required_preflight_from_report(
    operation: ExactBooleanOperation,
    report: &ExactPlanarArrangementReport,
) -> ExactBooleanPreflight {
    ExactBooleanPreflight::new(
        operation,
        ExactBooleanSupport::RequiresPlanarArrangement,
        report.graph_had_unknowns(),
        report.retained_face_pairs(),
        report.retained_events(),
        0,
        Vec::new(),
        Some(*report.blocker()),
        report.coplanar_arrangement_evidence().cloned(),
        None,
    )
}

fn certified_closed_boundary_touching_support(
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    match operation {
        ExactBooleanOperation::Union => {
            Some(ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion)
        }
        ExactBooleanOperation::Intersection => {
            Some(ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection)
        }
        ExactBooleanOperation::Difference => {
            Some(ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference)
        }
        ExactBooleanOperation::SelectedRegions(_) => None,
    }
}

fn certified_closed_winding_separated_preflight_from_empty_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ExactBooleanPreflight>, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || !graph.face_pairs.is_empty()
    {
        return Ok(None);
    }
    let Some((left_in_right, right_in_left)) =
        closed_winding_vertex_relations_from_empty_graph(graph, left, right)?
    else {
        return Ok(None);
    };
    if left_in_right != ClosedMeshWindingMeshRelation::Outside
        || right_in_left != ClosedMeshWindingMeshRelation::Outside
    {
        return Ok(None);
    }
    Ok(Some(certified_preflight(
        operation,
        ExactBooleanSupport::CertifiedClosedWindingSeparated,
        Some(graph),
        None,
    )))
}

fn certified_closed_boundary_only_contact_preflight(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ExactBooleanPreflight>, ExactMeshError> {
    let Some(boundary_support) = certified_closed_boundary_touching_support(operation) else {
        return Ok(None);
    };
    if !certified_closed_boundary_only_contact_from_graph(graph, left, right).unwrap_or(false) {
        return Ok(None);
    }
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    let validation_label = match operation {
        ExactBooleanOperation::Union => "exact no-volume-overlap union evidence validation failed",
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference => {
            "exact no-volume-overlap evidence validation failed"
        }
        ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
    };
    evidence.validate().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("{validation_label}: {error:?}"),
        ))
    })?;
    if evidence.positive_area_coplanar_overlapping_pairs() != 0 {
        return Ok(Some(certified_preflight(
            operation,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            Some(graph),
            Some(evidence),
        )));
    }
    let consumed_evidence = match operation {
        ExactBooleanOperation::Union => None,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference => {
            coplanar_boundary_only_evidence_if_consumed(graph, left, right)?
        }
        ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
    };
    Ok(Some(certified_preflight(
        operation,
        boundary_support,
        Some(graph),
        consumed_evidence,
    )))
}

fn certified_convex_operation_preflight(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    graph: &super::graph::ExactIntersectionGraph,
) -> Option<ExactBooleanPreflight> {
    certified_convex_operation_shortcut_support(left, right, operation)
        .map(|support| certified_preflight(operation, support, Some(graph), None))
}

fn certified_coplanar_boundary_closure_preflight(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanPreflight> {
    volumetric_boundary_closure_report_from_graph(graph, left, right, operation)
        .ok()
        .is_some_and(|report| report.is_coplanar_closure_available())
        .then(|| certified_arrangement_cell_complex_preflight(operation, graph, left, right))
}

fn certified_arrangement_cell_complex_coplanar_evidence(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarVolumetricCellEvidenceReport> {
    let counts = retained_graph_counts(graph);
    if !graph_requires_coplanar_volumetric_cells(&counts) {
        return None;
    }
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    if validate_graph_source_replay(graph, left, right).is_err() || evidence.validate().is_err() {
        return None;
    }
    evidence.is_arrangement_materializable().then_some(evidence)
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
        [false, true]
            .into_iter()
            .try_fold(false, |materialized, regularize_sheet_complex| {
                if materialized {
                    Ok(true)
                } else {
                    arrangement_cell_complex_materializes_for_preflight_from_graph(
                        graph,
                        left,
                        right,
                        operation,
                        regularize_sheet_complex,
                    )
                }
            })?
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
        )))
    } else {
        Ok(None)
    }
}

fn orthogonal_solid_cell_materializes_for_preflight(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<bool, ExactMeshError> {
    let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation) else {
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

pub(crate) fn certified_arrangement_cell_complex_preflight_from_retained_attempt(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
    attempt: &ExactArrangementBooleanAttempt,
) -> Result<Option<ExactBooleanPreflight>, ExactMeshError> {
    attempt
        .validate_for_request_policy(request, policy)
        .and_then(|()| attempt.validate_against_sources_for_request(left, right, request))
        .map_err(|error| {
            retained_evidence_validation_error(
                "retained arrangement attempt failed validation",
                error,
                ExactMeshBlockerKind::ExactConstructionFailure,
            )
        })?;
    if materialize_retained_arrangement_cell_complex_attempt(left, right, request, attempt)?
        .is_some()
    {
        Ok(Some(certified_arrangement_cell_complex_preflight(
            request.operation,
            graph,
            left,
            right,
        )))
    } else {
        Ok(None)
    }
}

type CertifiedArrangementCellComplexPreflightCache = Option<Option<ExactBooleanPreflight>>;

fn cached_certified_arrangement_cell_complex_preflight(
    cache: &mut CertifiedArrangementCellComplexPreflightCache,
    operation: ExactBooleanOperation,
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    retained_request: Option<ExactBooleanRequest>,
    retained_attempt: Option<&ExactArrangementBooleanAttempt>,
) -> Result<Option<ExactBooleanPreflight>, ExactMeshError> {
    if cache.is_none() {
        let retained_preflight = retained_request
            .zip(retained_attempt)
            .filter(|(request, _)| request.operation == operation)
            .map(|(request, attempt)| {
                certified_arrangement_cell_complex_preflight_from_retained_attempt(
                    graph,
                    left,
                    right,
                    request,
                    ExactRegularizationPolicy::REGULARIZED_SOLID,
                    attempt,
                )
            })
            .transpose()?
            .flatten();
        *cache = Some(match retained_preflight {
            Some(preflight) => Some(preflight),
            None => certified_arrangement_cell_complex_preflight_if_materialized(
                operation, graph, left, right,
            )?,
        });
    }
    Ok(cache.clone().flatten())
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
    let counts = retained_graph_counts(graph);
    if counts.coplanar_overlapping_pairs() == 0
        && (mesh_is_open_surface(left) || mesh_is_open_surface(right))
    {
        return Ok(true);
    }
    if has_empty_axis_aligned_orthogonal_solid_cell_intersection(left, right)
        || has_empty_affine_orthogonal_solid_cell_intersection(left, right)
    {
        return Ok(true);
    }
    certified_closed_boundary_contact(left, right)
}

fn graph_requires_coplanar_volumetric_cells(counts: &ExactBooleanBlocker) -> bool {
    // Coplanar source-face cells inside a closed volumetric overlap are not a
    // planar-surface output problem and not ordinary non-coplanar winding
    // state instead of approximating the cells or relabeling them as generic
    // winding evidence.
    counts.coplanar_overlapping_pairs() + counts.coplanar_touching_pairs() > 0
}

fn graph_requires_coplanar_volumetric_cells_for_sources(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    let counts = retained_graph_counts(graph);
    if !graph_requires_coplanar_volumetric_cells(&counts) {
        return false;
    }
    if validate_graph_source_replay(graph, left, right).is_err() {
        return false;
    }
    // This is the source-aware replacement for the coarse relation-count gate
    // above. A positive-area coplanar face pair is not automatically a
    // volumetric-cell blocker: opposite-side shared faces are boundary contact,
    // while same-side or undecided positive-area overlap needs the missing
    // coplanar volumetric-cell materializer. Keeping the decision in
    // consume replayable exact object evidence, not aggregate counters.
    CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right)
        .requires_coplanar_volumetric_cells()
}

fn coplanar_volumetric_evidence_if_required(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarVolumetricCellEvidenceReport> {
    let counts = retained_graph_counts(graph);
    if !graph_requires_coplanar_volumetric_cells(&counts) {
        return None;
    }
    if validate_graph_source_replay(graph, left, right).is_err() {
        return None;
    }
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    if evidence.validate().is_err() {
        return None;
    }
    evidence
        .requires_coplanar_volumetric_cells()
        .then_some(evidence)
}

fn coplanar_boundary_only_evidence_if_consumed(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<CoplanarVolumetricCellEvidenceReport>, ExactMeshError> {
    validate_graph_source_replay(graph, left, right)?;
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    evidence.validate().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("exact boundary-only coplanar evidence validation failed: {error:?}"),
        ))
    })?;
    Ok(evidence
        .is_boundary_only_positive_area_contact()
        .then_some(evidence))
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
                        let Some(triangle) =
                            triangle_points(plane_side.mesh(left, right), *plane_face)
                        else {
                            return false;
                        };
                        let Some(projection) = choose_triangle_projection(&triangle) else {
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

fn certified_closed_boundary_contact(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactMeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(false);
    }

    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    left_in_right.validate().map_err(winding_error)?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left.validate().map_err(winding_error)?;

    Ok(left_in_right.vertices_are_boundary_or_outside()
        && right_in_left.vertices_are_boundary_or_outside()
        && (left_in_right.vertices_touch_boundary() || right_in_left.vertices_touch_boundary()))
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
    let counts = retained_graph_counts(graph);
    if counts.construction_failed_events() != 0 {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    left_in_right.validate().map_err(winding_error)?;
    left_in_right
        .validate_against_sources(left, right)
        .map_err(winding_error)?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left.validate().map_err(winding_error)?;
    right_in_left
        .validate_against_sources(right, left)
        .map_err(winding_error)?;
    Ok(Some((left_in_right.relation(), right_in_left.relation())))
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

#[cfg(test)]
fn materialize_graph_shortcut_from_graph_for_request(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    support: ExactBooleanSupport,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let result = match support {
        ExactBooleanSupport::CertifiedBoundaryPolicyShortcut => {
            let boundary_policy = request.boundary_policy;
            if let Some((result, _evidence)) =
                materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
                    graph, left, right, operation, validation,
                )?
            {
                if result
                    .validate_request_against_sources_with_retained_attempt(
                        left,
                        right,
                        ExactBooleanRequest::with_boundary_policy(
                            operation,
                            validation,
                            ExactBoundaryBooleanPolicy::Reject,
                        ),
                        retained_arrangement_attempt,
                    )
                    .is_err()
                {
                    return Ok(None);
                }
                return Ok(request_replayable_result(
                    Some(result),
                    left,
                    right,
                    ExactBooleanRequest::with_boundary_policy(
                        operation,
                        validation,
                        boundary_policy,
                    ),
                    retained_arrangement_attempt,
                ));
            }
            let Some(result) = boolean_boundary_touching_meshes_from_graph(
                graph,
                left,
                right,
                operation,
                validation,
                boundary_policy,
            )?
            else {
                return Ok(None);
            };
            return Ok(request_replayable_result(
                Some(result),
                left,
                right,
                ExactBooleanRequest::with_boundary_policy(operation, validation, boundary_policy),
                retained_arrangement_attempt,
            ));
        }
        ExactBooleanSupport::CertifiedOpenSurfaceDisjoint => {
            if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
                || meshes_are_certified_bounds_disjoint(left, right)
                || closed_validation_regularized_solid_support(left, right, operation, validation)
                    .is_some()
            {
                return Ok(None);
            }
            boolean_open_surface_disjoint_meshes_from_graph(
                graph, left, right, operation, validation,
            )?
        }
        ExactBooleanSupport::CertifiedClosedWindingSeparated
        | ExactBooleanSupport::CertifiedClosedWindingContainment => {
            if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
                || left.triangles().is_empty()
                || right.triangles().is_empty()
                || meshes_are_certified_bounds_disjoint(left, right)
            {
                return Ok(None);
            }
            match support {
                ExactBooleanSupport::CertifiedClosedWindingSeparated => {
                    boolean_closed_winding_separated_meshes_from_graph(
                        graph, left, right, operation, validation,
                    )?
                }
                ExactBooleanSupport::CertifiedClosedWindingContainment => {
                    boolean_closed_winding_containment_meshes_from_graph(
                        graph, left, right, operation, validation,
                    )?
                }
                _ => return Ok(None),
            }
        }
        _ => return Ok(None),
    };
    Ok(request_replayable_result(
        result,
        left,
        right,
        ExactBooleanRequest::with_boundary_policy(
            operation,
            validation,
            ExactBoundaryBooleanPolicy::Reject,
        ),
        retained_arrangement_attempt,
    ))
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
    let retained_arrangement_attempt = matches!(
        result.kind(),
        ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
            | ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
                ..
            }
    )
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

fn request_uses_arrangement_lower_dimensional_regularized_shortcut(
    request: ExactBooleanRequest,
) -> bool {
    request.validation == ExactMeshValidationPolicy::CLOSED
        && (request.operation == ExactBooleanOperation::Intersection
            || request.boundary_policy == ExactBoundaryBooleanPolicy::PreserveSeparateShells)
}

fn materialize_arrangement_lower_dimensional_intersection_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if !request_uses_arrangement_lower_dimensional_regularized_shortcut(request)
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
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("exact arrangement lower-dimensional evidence validation failed: {error:?}"),
        ))
    })?;
    if !matches!(
        evidence.status(),
        ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized
            | ExactWindingEvidenceStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized
    ) {
        return Ok(None);
    }
    let mesh = empty_mesh(
        "empty exact arrangement cell-complex lower-dimensional intersection",
        request.validation,
    )?;
    Ok(Some(certified_shortcut_result(
        mesh,
        request.operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    )))
}

/// Materialize an exact boolean request.
///
/// This path is still strict about general winding. The additional
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
pub(crate) fn replay_boolean_exact_request_for_result_validation(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactBooleanResult, ExactMeshError> {
    materialize_boolean_exact_request(left, right, request)
}

pub(crate) fn materialize_boolean_exact_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactBooleanResult, ExactMeshError> {
    materialize_boolean_exact_request_with_graph(left, right, request, None, None)
}

pub(crate) fn materialize_boolean_exact_request_with_prepared_pair(
    pair: &PreparedMeshPair<'_, '_>,
    request: ExactBooleanRequest,
) -> Result<ExactBooleanResult, ExactMeshError> {
    let left = pair.left().view().mesh();
    let right = pair.right().view().mesh();
    materialize_boolean_exact_request_with_graph(left, right, request, None, Some(pair))
}

pub(crate) fn materialize_closed_union_with_prepared_pair(
    pair: &PreparedMeshPair<'_, '_>,
) -> Result<ExactMesh, ExactMeshError> {
    materialize_closed_named_boolean_with_prepared_pair(pair, ExactBooleanOperation::Union)
}

pub(crate) fn materialize_closed_intersection_with_prepared_pair(
    pair: &PreparedMeshPair<'_, '_>,
) -> Result<ExactMesh, ExactMeshError> {
    materialize_closed_named_boolean_with_prepared_pair(pair, ExactBooleanOperation::Intersection)
}

pub(crate) fn materialize_closed_difference_with_prepared_pair(
    pair: &PreparedMeshPair<'_, '_>,
) -> Result<ExactMesh, ExactMeshError> {
    materialize_closed_named_boolean_with_prepared_pair(pair, ExactBooleanOperation::Difference)
}

fn materialize_closed_named_boolean_with_prepared_pair(
    pair: &PreparedMeshPair<'_, '_>,
    operation: ExactBooleanOperation,
) -> Result<ExactMesh, ExactMeshError> {
    let request = ExactBooleanRequest::new(operation, ExactMeshValidationPolicy::CLOSED);
    materialize_boolean_exact_request_with_prepared_pair(pair, request)
        .map(ExactBooleanResult::into_mesh)
}

fn arrangement_shortcut_facts_for_request(
    prepared_pair: Option<&PreparedMeshPair<'_, '_>>,
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactArrangementCellComplexShortcutFacts {
    prepared_pair.map_or_else(
        || ExactArrangementCellComplexShortcutFacts::from_sources(left, right),
        PreparedMeshPair::arrangement_cell_complex_shortcut_facts,
    )
}

fn materialize_boolean_exact_request_with_graph(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_graph: Option<&ExactIntersectionGraph>,
    prepared_pair: Option<&PreparedMeshPair<'_, '_>>,
) -> Result<ExactBooleanResult, ExactMeshError> {
    left.validate_retained_bounds_certificate()?;
    right.validate_retained_bounds_certificate()?;
    let operation = request.operation;
    let validation = request.validation;
    let mut owned_graph = None;
    let mut prepared_graph = None;
    if let ExactBooleanOperation::SelectedRegions(selection) = operation {
        let graph = graph_for_certified_materialization_with_prepared(
            retained_graph,
            &mut owned_graph,
            &mut prepared_graph,
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
    if request_uses_arrangement_lower_dimensional_regularized_shortcut(request) {
        let graph = graph_for_certified_materialization_with_prepared(
            retained_graph,
            &mut owned_graph,
            &mut prepared_graph,
            prepared_pair,
            left,
            right,
        )?;
        let shortcut_facts = arrangement_shortcut_facts_for_request(prepared_pair, left, right);
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
        && evidence::meshes_are_certified_identical(left, right)
    {
        return boolean_identical_meshes(left, operation, validation);
    }
    if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
        && evidence::meshes_are_certified_same_surface(left, right)
    {
        return boolean_same_surface_meshes(left, operation, validation);
    }
    if let Some(graph) = retained_graph {
        return materialize_boolean_exact_request_from_ready_graph(graph, left, right, request);
    }
    if let Some(graph) = owned_graph.as_ref() {
        return materialize_boolean_exact_request_from_ready_graph(graph, left, right, request);
    }
    if let Some(pair) = prepared_pair {
        if let Some(result) = pair.with_retained_arrangement(|arrangement| {
            let graph = graph_for_certified_materialization_with_prepared(
                retained_graph,
                &mut owned_graph,
                &mut prepared_graph,
                prepared_pair,
                left,
                right,
            )?;
            let shortcut_facts = arrangement_shortcut_facts_for_request(prepared_pair, left, right);
            materialize_certified_arrangement_cell_complex_support_with_arrangement(
                left,
                right,
                request,
                Some(graph),
                Some(arrangement),
                None,
                &shortcut_facts,
            )
        }) {
            if let Some(result) = result? {
                return Ok(result);
            }
        }
    }
    if let Some(graph) = prepared_graph.as_deref() {
        return materialize_boolean_exact_request_from_ready_graph(graph, left, right, request);
    }

    if let Some(pair) = prepared_pair {
        let graph = build_validated_intersection_graph_from_prepared_pair(pair)?;
        return materialize_boolean_exact_request_from_ready_graph(&graph, left, right, request);
    }

    match build_validated_intersection_graph(left, right) {
        Ok(graph) => {
            materialize_boolean_exact_request_from_ready_graph(&graph, left, right, request)
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

fn materialize_boolean_exact_request_from_ready_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactBooleanResult, ExactMeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let boundary_policy = request.boundary_policy;
    let prefer_boundary_or_no_volume = matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    );
    if let Some(result) = boolean_closed_winding_separated_meshes_from_graph(
        graph, left, right, operation, validation,
    )? {
        return Ok(result);
    }
    let shortcut_facts = ExactArrangementCellComplexShortcutFacts::from_sources(left, right);
    if shortcut_facts.certified_support(operation)
        == Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
        && let Some(result) =
            boolean_arrangement_cell_complex_recovery(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if let Some(result) = certified_arrangement_cell_complex_result_from_graph(
        graph, left, right, operation, validation, true,
    )? {
        return Ok(result);
    }
    if let Some(result) = boolean_closed_winding_containment_meshes_from_graph(
        graph, left, right, operation, validation,
    )? {
        return Ok(result);
    }
    if prefer_boundary_or_no_volume
        && let Some(result) = materialize_closed_boundary_or_no_volume_overlap_from_graph(
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
        && let Some((result, _report)) =
            materialize_adjacent_union_completion_from_graph_for_request(
                graph, left, right, request,
            )?
    {
        return Ok(result);
    }
    if !prefer_boundary_or_no_volume
        && let Some(result) = materialize_closed_boundary_or_no_volume_overlap_from_graph(
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
    if let Some(result) = certified_arrangement_cell_complex_result_from_graph(
        graph, left, right, operation, validation, true,
    )? {
        return Ok(result);
    }
    match operation {
        ExactBooleanOperation::SelectedRegions(_) => Err(unsupported_boolean_operation_error(
            operation,
            "selected-region materialization requires the selected-region request path",
        )),
        ExactBooleanOperation::Union
        | ExactBooleanOperation::Intersection
        | ExactBooleanOperation::Difference => {
            match operation {
                ExactBooleanOperation::Union => {}
                ExactBooleanOperation::Intersection => {
                    if let Some(result) =
                        boolean_arrangement_regularized_boundary_contact_from_graph(
                            graph, left, right, operation, validation,
                        )?
                    {
                        return Ok(result);
                    }
                }
                ExactBooleanOperation::Difference => {
                    if let Some(result) =
                        boolean_arrangement_regularized_boundary_contact_from_graph(
                            graph, left, right, operation, validation,
                        )?
                    {
                        return Ok(result);
                    }
                }
                ExactBooleanOperation::SelectedRegions(_) => {
                    return Err(unsupported_boolean_operation_error(
                        operation,
                        "selected-region materialization requires the selected-region request path",
                    ));
                }
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
            if let Some(result) = boolean_boundary_touching_meshes_from_graph(
                graph,
                left,
                right,
                operation,
                validation,
                boundary_policy,
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

fn materialize_closed_boundary_or_no_volume_overlap_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if let Some((result, _evidence)) =
        materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
            graph, left, right, operation, validation,
        )?
    {
        return Ok(Some(result));
    }
    materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
        graph, left, right, operation, validation,
    )
    .map(|result| result.map(|(result, _evidence)| result))
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
        result.retain_missing_gate_reports(Some(topology), Some(ownership));
    }
    attempt.retain_materialized_output(
        &result.mesh,
        materialized_shortcut,
        clear_arrangement_blockers,
    );
    ArrangementCellComplexOutcome::Materialized(Box::new(result), attempt.clone())
}

fn not_attempted_arrangement_attempt_for_request(
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
) -> ExactArrangementBooleanAttempt {
    ExactArrangementBooleanAttempt {
        operation: request.operation,
        policy,
        output_validation: request.validation,
        boundary_policy: request.boundary_policy,
        stage: ExactArrangementBooleanStage::NotAttempted,
        decline: None,
        materialized_shortcut: None,
        shortcut_reason: None,
        arrangement_blockers: 0,
        face_cells: 0,
        regions: 0,
        volume_regions: 0,
        volume_adjacencies: 0,
        lower_dimensional_artifacts: 0,
        topology_assembly: None,
        topology_assembly_report: None,
        region_ownership: None,
        region_ownership_report: None,
        selected_faces: 0,
        reversed_selected_faces: 0,
        volume_oriented_selected_faces: 0,
        label_oriented_selected_faces: 0,
        selected_volume_regions: 0,
        selected_cell_complex: None,
        simplified_cell_complex: None,
        output_vertices: 0,
        output_triangles: 0,
        output_facts: None,
    }
}

fn declined_output_validation_attempt_outcome_with_counts(
    attempt: &mut ExactArrangementBooleanAttempt,
    output_counts: Option<(usize, usize)>,
) -> ArrangementCellComplexOutcome {
    attempt.decline_output_validation(output_counts);
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
        if self.enabled
            && let Some(outcome) = arrangement_volumetric_split_cell_recovery_outcome(
                attempt,
                self.graph,
                self.left,
                self.right,
                self.operation,
                self.validation,
            )?
        {
            return Ok(Some(outcome));
        }
        if !matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
            && !open_surface_disjoint_report_from_graph(self.graph, self.left, self.right)
                .is_certified()
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
        if let Some(outcome) = arrangement_open_surface_recovery_outcome(
            attempt,
            self.graph,
            self.left,
            self.right,
            self.operation,
            self.validation,
        )? {
            return Ok(Some(outcome));
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
    attempt.record_decline(decline);
    Ok(ArrangementCellComplexOutcome::Declined(attempt))
}

fn blocked_arrangement_cell_complex_outcome(
    recovery: &ArrangementCellComplexRecoveryContext<'_>,
    mut attempt: ExactArrangementBooleanAttempt,
    arrangement: &ExactArrangement,
    validation: ExactMeshValidationPolicy,
    regularize_unregularized_sheet_complex: bool,
) -> Result<ArrangementCellComplexOutcome, ExactMeshError> {
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
    attempt.record_decline(ExactArrangementBooleanDecline::ArrangementBlockers(
        arrangement.blockers.clone(),
    ));
    Ok(ArrangementCellComplexOutcome::Declined(attempt))
}

pub(crate) fn arrangement_cell_complex_shortcut_attempt(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
) -> Result<Option<ExactArrangementBooleanAttempt>, ExactMeshError> {
    let shortcut_facts = ExactArrangementCellComplexShortcutFacts::from_sources(left, right);
    arrangement_cell_complex_shortcut_attempt_with_facts(
        left,
        right,
        request,
        policy,
        &shortcut_facts,
    )
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
    if shortcut_facts.certified_support(request.operation)
        != Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
    {
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

pub(crate) fn arrangement_boolean_attempt_report_from_arrangement(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
    arrangement: &ExactArrangement,
) -> Result<ExactArrangementBooleanAttempt, ExactMeshError> {
    let outcome = run_arrangement_cell_complex_attempt_from_arrangement(
        arrangement,
        left,
        right,
        request,
        policy,
        true,
    )?;
    Ok(match outcome {
        ArrangementCellComplexOutcome::Materialized(_, attempt)
        | ArrangementCellComplexOutcome::Declined(attempt) => attempt,
    })
}

fn arrangement_cell_complex_result_is_certified_for_preflight(
    result: &ExactBooleanResult,
    attempt: &ExactArrangementBooleanAttempt,
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    let operation = match result.kind() {
        ExactBooleanResultKind::ArrangementCellComplexMaterialized { operation }
        | ExactBooleanResultKind::CertifiedShortcut {
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            operation,
        } => operation,
        _ => return false,
    };
    if !attempt.certifies_arrangement_cell_complex_output_for_operation(operation)
        || result.validate_against_sources(left, right).is_err()
    {
        return false;
    }
    if let Some((topology, ownership)) = attempt.retained_gate_reports() {
        if result.topology_assembly_report.as_ref() != Some(topology)
            || result.region_ownership_report.as_ref() != Some(ownership)
        {
            return false;
        }
    } else if result.topology_assembly_report.is_some() || result.region_ownership_report.is_some()
    {
        return false;
    }
    attempt.certifies_output_mesh(&result.mesh)
}

fn arrangement_cell_complex_result_matches_retained_attempt(
    result: &ExactBooleanResult,
    attempt: &ExactArrangementBooleanAttempt,
) -> bool {
    let operation = match result.kind() {
        ExactBooleanResultKind::ArrangementCellComplexMaterialized { operation }
        | ExactBooleanResultKind::CertifiedShortcut {
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            operation,
        } => operation,
        _ => return false,
    };
    if !attempt.certifies_arrangement_cell_complex_output_for_operation(operation)
        || result.validate().is_err()
    {
        return false;
    }
    if let Some((topology, ownership)) = attempt.retained_gate_reports() {
        if result.topology_assembly_report.as_ref() != Some(topology)
            || result.region_ownership_report.as_ref() != Some(ownership)
        {
            return false;
        }
    } else if result.topology_assembly_report.is_some() || result.region_ownership_report.is_some()
    {
        return false;
    }
    attempt.certifies_output_mesh(&result.mesh)
}

fn arrangement_cell_complex_materializes_for_preflight_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    regularize_unregularized_sheet_complex: bool,
) -> Result<bool, ExactMeshError> {
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
        if certified_arrangement_cell_complex_result_from_graph(
            graph,
            left,
            right,
            operation,
            validation,
            regularize_unregularized_sheet_complex,
        )?
        .is_some()
        {
            return Ok(true);
        }
    }
    Ok(false)
}

fn certified_arrangement_cell_complex_result_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    regularize_unregularized_sheet_complex: bool,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let outcome = match run_arrangement_cell_complex_attempt_from_graph(
        graph,
        left,
        right,
        ExactBooleanRequest::new(operation, validation),
        ExactRegularizationPolicy::REGULARIZED_SOLID,
        regularize_unregularized_sheet_complex,
    ) {
        Ok(outcome) => outcome,
        Err(_) => return Ok(None),
    };
    let ArrangementCellComplexOutcome::Materialized(result, attempt) = outcome else {
        return Ok(None);
    };
    if arrangement_cell_complex_result_is_certified_for_preflight(&result, &attempt, left, right) {
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
    if evidence::meshes_are_certified_identical(left, right)
        || evidence::meshes_are_certified_same_surface(left, right)
    {
        return Ok(None);
    }
    if let Some(report) =
        certified_closed_boundary_touching_regularized_report_from_graph(graph, left, right)?
    {
        report.validate_against_sources(left, right).map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(ExactMeshBlockerKind::StaleFactReplay,
                format!(
                    "exact arrangement regularized boundary contact consumed invalid certificate: {error:?}"
                ),
            ))
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
    if evidence::meshes_are_certified_identical(left, right)
        || evidence::meshes_are_certified_same_surface(left, right)
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
    if !certified_closed_boundary_only_contact_from_graph(graph, left, right)? {
        return Ok(false);
    }
    Ok(true)
}

fn run_arrangement_cell_complex_attempt_from_graph(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
    regularize_unregularized_sheet_complex: bool,
) -> Result<ArrangementCellComplexOutcome, ExactMeshError> {
    let arrangement =
        ExactArrangement::from_intersection_graph_with_policy(graph.clone(), left, right, policy)?;
    run_arrangement_cell_complex_attempt_from_arrangement(
        &arrangement,
        left,
        right,
        request,
        policy,
        regularize_unregularized_sheet_complex,
    )
}

fn arrangement_cell_complex_started_attempt(
    arrangement: &ExactArrangement,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
) -> ExactArrangementBooleanAttempt {
    ExactArrangementBooleanAttempt {
        operation: request.operation,
        policy,
        output_validation: request.validation,
        boundary_policy: request.boundary_policy,
        stage: ExactArrangementBooleanStage::ArrangementBuilt,
        decline: None,
        materialized_shortcut: None,
        shortcut_reason: None,
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
        topology_assembly: None,
        topology_assembly_report: None,
        region_ownership: None,
        region_ownership_report: None,
        selected_faces: 0,
        reversed_selected_faces: 0,
        volume_oriented_selected_faces: 0,
        label_oriented_selected_faces: 0,
        selected_volume_regions: 0,
        selected_cell_complex: None,
        simplified_cell_complex: None,
        output_vertices: 0,
        output_triangles: 0,
        output_facts: None,
    }
}

fn run_arrangement_cell_complex_attempt_from_arrangement(
    arrangement: &ExactArrangement,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
    regularize_unregularized_sheet_complex: bool,
) -> Result<ArrangementCellComplexOutcome, ExactMeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let attempt = arrangement_cell_complex_started_attempt(arrangement, request, policy);
    let regularized_sheet_recovery_surface = left.facts().mesh.closed_manifold
        && right.facts().mesh.closed_manifold
        && arrangement
            .shells_or_regions
            .as_ref()
            .is_some_and(|regions| {
                regions
                    .iter()
                    .any(|region| region.non_manifold_edges > 0 && region.source_sides.len() > 1)
            });
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

    if !arrangement.is_complete()
        && !volume_resolves_region_classification
        && !selected_regions_ignore_unresolved_classification
    {
        return blocked_arrangement_cell_complex_outcome(
            &recovery,
            attempt,
            arrangement,
            validation,
            regularize_unregularized_sheet_complex,
        );
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
        ArrangementCellComplexGateOutcome::Ready(evidence) => evidence,
        ArrangementCellComplexGateOutcome::Declined(outcome) => return Ok(outcome),
    };
    let selected = match select_arrangement_cell_complex_with_ownership_report(
        labeled,
        &ownership_report,
        operation,
        policy,
    ) {
        Ok(selected) => {
            selected.with_gate_reports(topology_report.clone(), ownership_report.clone())
        }
        Err(ArrangementCellComplexSelectionDecline::Blocked(blocker)) => {
            return arrangement_cell_complex_decline_after_recovery(
                &recovery,
                attempt,
                ExactArrangementBooleanDecline::Selection(blocker),
            );
        }
        Err(ArrangementCellComplexSelectionDecline::SelectedWithBlockers(selected)) => {
            let counts = selected.counts();
            attempt.selected_faces = counts.selected_faces;
            attempt.selected_volume_regions = counts.selected_volume_regions;
            attempt.reversed_selected_faces = counts.reversed_selected_faces;
            attempt.volume_oriented_selected_faces = counts.volume_oriented_selected_faces;
            attempt.label_oriented_selected_faces = counts.label_oriented_selected_faces;
            return arrangement_cell_complex_decline_after_recovery(
                &recovery,
                attempt,
                ExactArrangementBooleanDecline::Selection(selected.blockers[0].clone()),
            );
        }
    };
    attempt.retain_selected_cell_complex(selected.clone());
    let simplified = match selected.simplify_exact_with_policy(policy) {
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
    attempt.retain_simplified_cell_complex(simplified.clone());
    let mesh = match simplified.triangulate() {
        Ok(mesh) => mesh,
        Err(blocker) => {
            return arrangement_cell_complex_decline_after_recovery(
                &recovery,
                attempt,
                ExactArrangementBooleanDecline::Triangulation(blocker),
            );
        }
    };
    materialize_triangulated_arrangement_cell_complex_attempt(
        &recovery,
        attempt,
        mesh,
        operation,
        validation,
        volume_resolves_region_classification,
    )
}

struct ArrangementCellComplexGateEvidence {
    attempt: ExactArrangementBooleanAttempt,
    labeled: ExactLabeledCellComplex,
    topology_report: ExactTopologyAssemblyReport,
    ownership_report: ExactRegionOwnershipReport,
}

enum ArrangementCellComplexGateOutcome {
    Ready(ArrangementCellComplexGateEvidence),
    Declined(ArrangementCellComplexOutcome),
}

fn arrangement_cell_complex_gate_evidence_from_arrangement(
    arrangement: &ExactArrangement,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    policy: ExactRegularizationPolicy,
    recovery: &ArrangementCellComplexRecoveryContext<'_>,
    mut attempt: ExactArrangementBooleanAttempt,
) -> Result<ArrangementCellComplexGateOutcome, ExactMeshError> {
    let topology_report = arrangement.topology_assembly_report_with_policy(left, right, policy);
    attempt.retain_topology_assembly_report(topology_report.clone());
    if topology_report.validate().is_err() || !topology_report.is_complete() {
        return Ok(ArrangementCellComplexGateOutcome::Declined(
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
            return Ok(ArrangementCellComplexGateOutcome::Declined(
                arrangement_cell_complex_decline_after_recovery(
                    recovery,
                    attempt,
                    ExactArrangementBooleanDecline::Labeling(blocker),
                )?,
            ));
        }
    };
    attempt.mark_labeled();

    let ownership_report = labeled.region_ownership_report(left, right, labeling_policy);
    attempt.retain_region_ownership_report(ownership_report.clone());
    if ownership_report.validate().is_err() {
        attempt.record_decline(ExactArrangementBooleanDecline::RegionOwnership(
            ownership_report.status,
        ));
        return Ok(ArrangementCellComplexGateOutcome::Declined(
            ArrangementCellComplexOutcome::Declined(attempt),
        ));
    }
    let ownership_resolves_named_selection = ownership_report
        .resolves_operation_selection(operation)
        || matches!(operation, ExactBooleanOperation::SelectedRegions(_));
    if !ownership_resolves_named_selection {
        return Ok(ArrangementCellComplexGateOutcome::Declined(
            arrangement_cell_complex_decline_after_recovery(
                recovery,
                attempt,
                ExactArrangementBooleanDecline::RegionOwnership(ownership_report.status),
            )?,
        ));
    }

    Ok(ArrangementCellComplexGateOutcome::Ready(
        ArrangementCellComplexGateEvidence {
            attempt,
            labeled,
            topology_report,
            ownership_report,
        },
    ))
}

enum ArrangementCellComplexSelectionDecline {
    Blocked(ExactArrangementBlocker),
    SelectedWithBlockers(ExactSelectedCellComplex),
}

fn select_arrangement_cell_complex_with_ownership_report(
    labeled: ExactLabeledCellComplex,
    ownership_report: &ExactRegionOwnershipReport,
    operation: ExactBooleanOperation,
    policy: ExactRegularizationPolicy,
) -> Result<ExactSelectedCellComplex, ArrangementCellComplexSelectionDecline> {
    let selected = if ownership_report.volume_selection_resolves_operation(operation) {
        labeled.select_volume_resolved(operation)
    } else {
        labeled.select_with_policy(operation, policy)
    }
    .map_err(ArrangementCellComplexSelectionDecline::Blocked)?;

    if selected.blockers.is_empty() {
        Ok(selected)
    } else {
        Err(ArrangementCellComplexSelectionDecline::SelectedWithBlockers(selected))
    }
}

fn materialize_triangulated_arrangement_cell_complex_attempt(
    recovery: &ArrangementCellComplexRecoveryContext<'_>,
    mut attempt: ExactArrangementBooleanAttempt,
    mesh: ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    volume_resolves_region_classification: bool,
) -> Result<ArrangementCellComplexOutcome, ExactMeshError> {
    attempt.mark_triangulated();
    attempt.retain_output_mesh(&mesh);
    let mesh = match copy_mesh(
        &mesh,
        "exact arrangement cell-complex boolean result",
        validation,
    ) {
        Ok(mesh) => mesh,
        Err(_) => {
            if validation == ExactMeshValidationPolicy::CLOSED {
                let maybe_mesh = close_exact_coplanar_boundary_loops(
                    &mesh,
                    "exact arrangement cell-complex closed coplanar-boundary result",
                    validation,
                )
                .ok()
                .flatten();
                if let Some(mesh) = maybe_mesh {
                    let result = certified_shortcut_result(
                        mesh,
                        operation,
                        ExactBooleanShortcutKind::ArrangementCellComplex,
                    );
                    return Ok(materialized_arrangement_attempt_outcome(
                        &mut attempt,
                        result,
                        false,
                        Some(ExactBooleanShortcutKind::ArrangementCellComplex),
                    ));
                }
            }
            return arrangement_cell_complex_decline_after_recovery(
                recovery,
                attempt,
                ExactArrangementBooleanDecline::OutputValidation,
            );
        }
    };
    let result = certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    Ok(materialized_arrangement_attempt_outcome(
        &mut attempt,
        result,
        volume_resolves_region_classification,
        None,
    ))
}

fn arrangement_open_surface_recovery_outcome(
    attempt: &mut ExactArrangementBooleanAttempt,
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ArrangementCellComplexOutcome>, ExactMeshError> {
    let Some(plan) = open_surface_arrangement_plan_from_graph(graph, left, right, operation)?
    else {
        return Ok(None);
    };
    let result = match materialize_open_surface_arrangement_plan(
        left,
        right,
        operation,
        validation,
        graph.has_unknowns(),
        plan.clone(),
    ) {
        Ok(Some(result)) => result,
        Ok(None) => {
            let output_counts = materialize_open_surface_arrangement_plan(
                left,
                right,
                operation,
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
                graph.has_unknowns(),
                plan,
            )
            .ok()
            .flatten()
            .map(|result| (result.mesh.vertices().len(), result.mesh.triangles().len()));
            return Ok(Some(
                declined_output_validation_attempt_outcome_with_counts(attempt, output_counts),
            ));
        }
        Err(error) => {
            let output_counts = materialize_open_surface_arrangement_plan(
                left,
                right,
                operation,
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
                graph.has_unknowns(),
                plan,
            )
            .ok()
            .flatten()
            .map(|result| (result.mesh.vertices().len(), result.mesh.triangles().len()));
            if output_counts.is_some() {
                return Ok(Some(
                    declined_output_validation_attempt_outcome_with_counts(attempt, output_counts),
                ));
            }
            return Err(error);
        }
    };
    Ok(Some(materialized_arrangement_attempt_outcome(
        attempt,
        result,
        false,
        Some(ExactBooleanShortcutKind::ArrangementCellComplex),
    )))
}

fn adjacent_union_completion_report(
    operation: ExactBooleanOperation,
    status: ExactAdjacentUnionCompletionStatus,
    left_closed: bool,
    right_closed: bool,
    axis_aligned_box_pair: bool,
    stronger_kernel_available: bool,
    graph_had_unknowns: bool,
    retained_face_pairs: usize,
    retained_events: usize,
    counts: ExactBooleanBlocker,
    full_face_shared_faces: usize,
    full_face_shared_patches: usize,
    contained_containing_side: Option<MeshSide>,
    contained_faces: usize,
    containing_faces: usize,
) -> ExactAdjacentUnionCompletionReport {
    let blocker_kind = match status {
        ExactAdjacentUnionCompletionStatus::GraphUnresolved => ExactBooleanBlockerKind::Refinement,
        ExactAdjacentUnionCompletionStatus::CertifiedFullFace
        | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace => {
            ExactBooleanBlockerKind::BoundaryPolicy
        }
        _ => counts.inferred_kind(),
    };
    ExactAdjacentUnionCompletionReport::new(
        operation,
        status,
        left_closed,
        right_closed,
        axis_aligned_box_pair,
        stronger_kernel_available,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        counts.into_blocker(blocker_kind),
        full_face_shared_faces,
        full_face_shared_patches,
        contained_containing_side,
        contained_faces,
        containing_faces,
    )
}

pub(crate) fn adjacent_union_completion_certification(
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
            adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::NotUnion,
                left_closed,
                right_closed,
                false,
                false,
                false,
                0,
                0,
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
            adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::NotClosedSolid,
                left_closed,
                right_closed,
                false,
                false,
                false,
                0,
                0,
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
            adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::AxisAlignedBoxPair,
                left_closed,
                right_closed,
                true,
                false,
                false,
                0,
                0,
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
    let graph = build_validated_intersection_graph(left, right)?;
    adjacent_union_completion_certification_from_graph(
        &graph,
        left,
        right,
        operation,
        materialization_validation,
    )
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
            adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::NotUnion,
                left_closed,
                right_closed,
                false,
                false,
                false,
                0,
                0,
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
            adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::NotClosedSolid,
                left_closed,
                right_closed,
                false,
                false,
                false,
                0,
                0,
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
            adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::AxisAlignedBoxPair,
                left_closed,
                right_closed,
                true,
                false,
                false,
                0,
                0,
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

    let graph_had_unknowns = graph.has_unknowns();
    let retained_face_pairs = graph.face_pairs.len();
    let retained_events = graph.event_count();
    let counts = retained_graph_counts(graph);
    if graph_had_unknowns || counts.construction_failed_events() != 0 {
        return Ok((
            adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::GraphUnresolved,
                left_closed,
                right_closed,
                false,
                false,
                graph_had_unknowns,
                retained_face_pairs,
                retained_events,
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
        let result = materialization_validation.and_then(|_| {
            let result = certified_shortcut_result(
                union.mesh,
                ExactBooleanOperation::Union,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            );
            result.validate().is_ok().then_some(result)
        });
        return Ok((
            adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::CertifiedFullFace,
                left_closed,
                right_closed,
                false,
                false,
                graph_had_unknowns,
                retained_face_pairs,
                retained_events,
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
            ExactBooleanOperation::Union => has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Union,
            ),
            ExactBooleanOperation::Intersection => has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Intersection,
            ),
            ExactBooleanOperation::Difference | ExactBooleanOperation::SelectedRegions(_) => true,
        }
    {
        return Ok((
            adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::StrongerKernelAvailable,
                left_closed,
                right_closed,
                false,
                true,
                graph_had_unknowns,
                retained_face_pairs,
                retained_events,
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
        let result = materialization_validation.and_then(|_| {
            let result = certified_shortcut_result(
                union.mesh,
                ExactBooleanOperation::Union,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            );
            result.validate().is_ok().then_some(result)
        });
        return Ok((
            adjacent_union_completion_report(
                operation,
                ExactAdjacentUnionCompletionStatus::CertifiedContainedFace,
                left_closed,
                right_closed,
                false,
                false,
                graph_had_unknowns,
                retained_face_pairs,
                retained_events,
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
        adjacent_union_completion_report(
            operation,
            ExactAdjacentUnionCompletionStatus::NoAdjacencyCertificate,
            left_closed,
            right_closed,
            false,
            false,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
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

pub(crate) fn materialize_adjacent_union_completion_from_graph_for_request(
    graph: &ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<(ExactBooleanResult, ExactAdjacentUnionCompletionReport)>, ExactMeshError> {
    let (report, result) = adjacent_union_completion_certification_from_graph(
        graph,
        left,
        right,
        request.operation,
        Some(request.validation),
    )?;
    if !report.is_certified() {
        return Ok(None);
    }
    report.validate().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("exact adjacent-union completion report validation failed: {error:?}"),
        ))
    })?;
    if report.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    let Some(result) = result else {
        return Ok(None);
    };
    if result.validate().is_err() {
        return Ok(None);
    }
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
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    evidence.validate().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("exact no-volume-overlap evidence validation failed: {error:?}"),
        ))
    })?;
    if evidence.validate_against_sources(left, right).is_err()
        || !evidence.is_boundary_only_positive_area_contact()
    {
        return Ok(None);
    }
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
        if result.validate().is_err() {
            return Ok(None);
        }
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
    if !arrangement_difference_preserves_source_surface(&left_minus_right, left, MeshSide::Left) {
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
    if result.validate().is_err() {
        return Ok(None);
    }
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
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    evidence.validate().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("exact no-volume-overlap evidence validation failed: {error:?}"),
        ))
    })?;
    if !evidence.is_boundary_only_positive_area_contact() {
        return Ok(None);
    }
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
    if result.validate().is_err() {
        return Ok(None);
    }
    if evidence.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some((result, evidence)))
}

fn arrangement_difference_preserves_source_surface(
    result: &ExactBooleanResult,
    source: &ExactMesh,
    source_side: MeshSide,
) -> bool {
    if !result.is_arrangement_cell_complex_materialized_for(ExactBooleanOperation::Difference) {
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
        let area = projected_polygon_area2_value(&points, projection);
        let Some(area) = (match real_sign(&area) {
            Some(Sign::Negative) => Some(Real::from(0) - area),
            Some(Sign::Zero | Sign::Positive) => Some(area),
            None => None,
        }) else {
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
        let source_area = projected_polygon_area2_value(&points, projection);
        let Some(source_area) = (match real_sign(&source_area) {
            Some(Sign::Negative) => Some(Real::from(0) - source_area),
            Some(Sign::Zero | Sign::Positive) => Some(source_area),
            None => None,
        }) else {
            return false;
        };
        compare_reals(&retained_area_by_face[face], &source_area).value() == Some(Ordering::Equal)
    })
}

fn arrangement_volumetric_split_cell_recovery_outcome(
    attempt: &mut ExactArrangementBooleanAttempt,
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ArrangementCellComplexOutcome>, ExactMeshError> {
    let Some(result) = materialize_arrangement_volumetric_split_cell_result_from_graph(
        graph, left, right, operation, validation,
    )?
    else {
        if validation == ExactMeshValidationPolicy::CLOSED
            && let Some(output_counts) =
                volumetric_winding_open_boundary_candidate_counts(graph, left, right, operation)?
        {
            return Ok(Some(
                declined_output_validation_attempt_outcome_with_counts(
                    attempt,
                    Some(output_counts),
                ),
            ));
        }
        return Ok(None);
    };
    Ok(Some(materialized_arrangement_attempt_outcome(
        attempt,
        result,
        true,
        Some(ExactBooleanShortcutKind::ArrangementCellComplex),
    )))
}

fn boolean_convex_meshes_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let (mesh, shortcut, label) = match operation {
        ExactBooleanOperation::Union => {
            let Some(union) = union_closed_convex_solids(left, right)? else {
                return Ok(None);
            };
            (
                union.mesh,
                ExactBooleanShortcutKind::ConvexUnion,
                "exact closed-convex solid union boolean result",
            )
        }
        ExactBooleanOperation::Intersection => {
            let Some(intersection) = intersect_closed_convex_solids(left, right)? else {
                return Ok(None);
            };
            (
                intersection.mesh,
                ExactBooleanShortcutKind::ConvexIntersection,
                "exact closed-convex solid intersection boolean result",
            )
        }
        ExactBooleanOperation::Difference => {
            let Some(difference) = subtract_closed_convex_solids(left, right)? else {
                return Ok(None);
            };
            (
                difference.mesh,
                ExactBooleanShortcutKind::ConvexDifference,
                "exact closed-convex solid difference boolean result",
            )
        }
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    let mesh = copy_mesh(&mesh, label, validation)?;
    let result = certified_shortcut_result(mesh, operation, shortcut);
    if result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
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
    let relation_counts = retained_graph_counts(graph);
    if graph.has_unknowns() || relation_counts.construction_failed_events() > 0 {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_convex_solid_report(left, right);
    left_in_right
        .validate_against_sources(left, right)
        .map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!("left convex relation replay failed: {error:?}"),
            ))
        })?;
    let right_in_left = classify_mesh_vertices_against_convex_solid_report(right, left);
    right_in_left
        .validate_against_sources(right, left)
        .map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!("right convex relation replay failed: {error:?}"),
            ))
        })?;

    if graph.face_pairs.is_empty() {
        return Ok(match (left_in_right.relation(), right_in_left.relation()) {
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
    if result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
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
    if result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

fn materialize_volumetric_coplanar_boundary_closure_boolean_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<(ExactBooleanResult, ExactVolumetricBoundaryClosureReport)>, ExactMeshError> {
    let Some((mesh, closure_report)) =
        materialize_volumetric_coplanar_boundary_closure_output_from_graph(
            graph, left, right, operation, validation,
        )?
    else {
        return Ok(None);
    };
    let result = certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    let result =
        result_with_arrangement_gate_reports_from_graph(result, graph, left, right, operation)?;
    if result.validate().is_err() || closure_report.validate().is_err() {
        return Ok(None);
    }
    Ok(Some((result, closure_report)))
}

fn result_with_arrangement_gate_reports_from_graph(
    result: ExactBooleanResult,
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactBooleanResult, ExactMeshError> {
    let arrangement = ExactArrangement::from_intersection_graph_with_policy(
        graph.clone(),
        left,
        right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )?;
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
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::ExactConstructionFailure,
                format!("exact region ownership report failed: {blocker:?}"),
            ))
        })?;
    Ok(result.with_gate_reports(Some(topology_report), Some(ownership_report)))
}

pub(crate) fn materialize_volumetric_coplanar_boundary_closure_output(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<(ExactMesh, ExactVolumetricBoundaryClosureReport)>, ExactMeshError> {
    let graph = build_validated_intersection_graph(left, right)?;
    materialize_volumetric_coplanar_boundary_closure_output_from_graph(
        &graph, left, right, operation, validation,
    )
}

fn materialize_volumetric_coplanar_boundary_closure_output_from_graph(
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
    let Some(mesh) = close_exact_coplanar_boundary_loops(
        &materialized.mesh,
        "exact volumetric split-cell coplanar boundary closure",
        validation,
    )
    .ok()
    .flatten() else {
        return Ok(None);
    };
    let closure_report =
        volumetric_boundary_closure_report_from_materialized_with_prevalidated_closure(
            &materialized,
            operation,
            Some(true),
        )?;
    if !closure_report.is_coplanar_closure_available() || closure_report.validate().is_err() {
        return Ok(None);
    }
    Ok(Some((mesh, closure_report)))
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
            let result = result_with_arrangement_gate_reports_from_graph(
                result, graph, left, right, operation,
            )?;
            if result.validate().is_err()
                || result
                    .validate_arrangement_cell_complex_gate_reports_against_arrangement(
                        &ExactArrangement::from_intersection_graph_with_policy(
                            graph.clone(),
                            left,
                            right,
                            ExactRegularizationPolicy::REGULARIZED_SOLID,
                        )?,
                        left,
                        right,
                        Some(operation),
                    )
                    .is_err()
            {
                return Ok(None);
            }
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
    retained_regularized_arrangement: Option<&ExactArrangement>,
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
        result.validate_arrangement_cell_complex_gate_reports_against_sources(left, right)?;
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
    let replay = volumetric_arrangement_cell_complex_result(operation, materialized)
        .with_gate_reports(
            result.topology_assembly_report().cloned(),
            result.region_ownership_report().cloned(),
        );
    replay.validate()?;
    if result == &replay {
        Ok(())
    } else {
        Err(ExactEvidenceValidationError::SourceReplayMismatch)
    }
}

fn volumetric_winding_open_boundary_candidate_counts(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<(usize, usize)>, ExactMeshError> {
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
    if matches!(
        volumetric_boundary_closure_report_from_materialized(&materialized, operation)?.status(),
        &ExactVolumetricBoundaryClosureStatus::AlreadyClosed
            | &ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
    ) {
        return Ok(None);
    }
    Ok(Some((
        materialized.mesh.vertices().len(),
        materialized.mesh.triangles().len(),
    )))
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
    let Some(boundary_loops) = directed_boundary_loops(mesh) else {
        return Ok(None);
    };
    if !boundary_loops_are_exactly_coplanar_without_self_contact(mesh, &boundary_loops)? {
        return Ok(None);
    }
    close_exact_coplanar_boundary_loops_from_loops(mesh, boundary_loops, label, validation)
}

fn boundary_loops_are_exactly_coplanar_without_self_contact(
    mesh: &ExactMesh,
    boundary_loops: &[Vec<usize>],
) -> Result<bool, ExactMeshError> {
    let mut boundary_points = Vec::new();
    for boundary_loop in boundary_loops {
        let Some(points) = boundary_loop
            .iter()
            .map(|&vertex| mesh.vertices().get(vertex).cloned())
            .collect::<Option<Vec<_>>>()
        else {
            return Ok(false);
        };
        let split = split_boundary_self_contact_cycles(points).map_err(|blocker| {
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
        if !exact_loop_is_coplanar(boundary).map_err(|blocker| {
            arrangement_blocker_error(
                "exact coplanar boundary closure coplanarity check failed",
                blocker,
            )
        })? {
            return Ok(false);
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
    let Some(mesh) = close_exact_coplanar_boundary_loops(
        &materialized.mesh,
        "exact volumetric split-cell coplanar boundary closure",
        validation,
    )
    .ok()
    .flatten() else {
        return Ok(None);
    };
    let closure_report =
        volumetric_boundary_closure_report_from_materialized_with_prevalidated_closure(
            materialized,
            operation,
            Some(true),
        )?;
    if !closure_report.is_coplanar_closure_available()
        || closure_report.validate().is_err()
        || closure_report
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

    let boundary_edges = directed_boundary_edges(mesh);
    let mut split_boundary_loops = Vec::new();
    for boundary_loop in boundary_loops {
        let split =
            split_boundary_vertex_self_contact_cycles(mesh, boundary_loop).map_err(|blocker| {
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
            let Some(points) = boundary_loop
                .iter()
                .map(|&vertex| mesh.vertices().get(vertex).cloned())
                .collect::<Option<Vec<_>>>()
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
        remove_duplicate_triangle_vertex_sets(&mut triangles);
        return ExactMesh::new_with_policy(
            mesh.vertices().to_vec(),
            triangles,
            SourceProvenance::exact(label),
            validation,
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
        let Some(loops) = vertex_loops
            .iter()
            .map(|boundary_loop| {
                boundary_loop
                    .iter()
                    .map(|&vertex| mesh.vertices().get(vertex).cloned())
                    .collect::<Option<Vec<_>>>()
            })
            .collect::<Option<Vec<_>>>()
        else {
            return Ok(None);
        };
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
        let Some(local_to_global) = map_cap_vertices_to_boundary_or_insert(
            mesh,
            &vertex_loops,
            &mut vertices,
            group_vertices,
        ) else {
            return Ok(None);
        };
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
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        validation,
    )
    .map(Some)
}

fn remove_duplicate_triangle_vertex_sets(triangles: &mut Vec<Triangle>) {
    let mut seen = BTreeSet::new();
    triangles.retain(|triangle| {
        let mut key = triangle.0;
        key.sort_unstable();
        seen.insert(key)
    });
}

fn find_or_insert_exact_mesh_vertex(vertices: &mut Vec<Point3>, point: Point3) -> Option<usize> {
    for (index, existing) in vertices.iter().enumerate() {
        match point3_exact_equal(existing, &point) {
            Some(true) => return Some(index),
            Some(false) => {}
            None => return None,
        }
    }
    let index = vertices.len();
    vertices.push(point);
    Some(index)
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
        let points = boundary
            .iter()
            .map(|&vertex| {
                mesh.vertices()
                    .get(vertex)
                    .cloned()
                    .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let carrier =
            exact_loop_carrier(&points)?.ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
        let mut group_index = None;
        for (index, (group_carrier, _)) in groups.iter().enumerate() {
            if exact_loop_is_coplanar_with_carrier(&points, group_carrier)? {
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

fn exact_loop_is_coplanar_with_carrier(
    points: &[Point3],
    carrier: &[Point3; 3],
) -> Result<bool, ExactArrangementBlocker> {
    for point in points {
        match orient3d_report(&carrier[0], &carrier[1], &carrier[2], point).value() {
            Some(Sign::Zero) => {}
            Some(Sign::Negative | Sign::Positive) => return Ok(false),
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    Ok(true)
}

fn map_cap_vertices_to_boundary_or_insert(
    mesh: &ExactMesh,
    boundary_loops: &[Vec<usize>],
    vertices: &mut Vec<Point3>,
    cap_vertices: Vec<Point3>,
) -> Option<Vec<usize>> {
    let boundary_vertices = boundary_loops.iter().flatten().copied().collect::<Vec<_>>();
    let mut used_boundary_vertices = vec![false; boundary_vertices.len()];
    cap_vertices
        .into_iter()
        .map(|point| {
            for (index, &boundary_vertex) in boundary_vertices.iter().enumerate() {
                if used_boundary_vertices[index] {
                    continue;
                }
                let existing = mesh.vertices().get(boundary_vertex)?;
                match point3_exact_equal(existing, &point) {
                    Some(true) => {
                        used_boundary_vertices[index] = true;
                        return Some(boundary_vertex);
                    }
                    Some(false) => {}
                    None => return None,
                }
            }
            find_or_insert_exact_mesh_vertex(vertices, point)
        })
        .collect()
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

#[derive(Default)]
struct BoundaryLoopSelfContactEvidence {
    repeated_exact_point_pairs: usize,
    exact_points: usize,
    topological_vertices: usize,
    degenerate_cycles: usize,
    nondegenerate_cycles: usize,
}

impl BoundaryLoopSelfContactEvidence {
    fn add(&mut self, other: Self) {
        self.repeated_exact_point_pairs += other.repeated_exact_point_pairs;
        self.exact_points += other.exact_points;
        self.topological_vertices += other.topological_vertices;
        self.degenerate_cycles += other.degenerate_cycles;
        self.nondegenerate_cycles += other.nondegenerate_cycles;
    }
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
            if cyclic_interval_distinct_items(points, start, end, &point3s_exact_equal)? < 3 {
                evidence.degenerate_cycles += 1;
            } else {
                evidence.nondegenerate_cycles += 1;
            }
        }
    }
    Ok(evidence)
}

#[cfg(test)]
fn canonicalize_degenerate_boundary_self_contact(
    points: Vec<Point3>,
) -> Result<Vec<Point3>, ExactArrangementBlocker> {
    canonicalize_degenerate_cyclic_self_contact(points, &point3s_exact_equal)
}

fn split_boundary_self_contact_cycles(
    points: Vec<Point3>,
) -> Result<Vec<Vec<Point3>>, ExactArrangementBlocker> {
    split_cyclic_self_contact_cycles(points, &point3s_exact_equal)
}

fn split_boundary_vertex_self_contact_cycles(
    mesh: &ExactMesh,
    vertices: Vec<usize>,
) -> Result<Vec<Vec<usize>>, ExactArrangementBlocker> {
    split_cyclic_self_contact_cycles(vertices, &|left, right| {
        boundary_vertices_exact_equal(mesh, *left, *right)
    })
}

fn point3s_exact_equal(left: &Point3, right: &Point3) -> Result<bool, ExactArrangementBlocker> {
    point3_exact_equal(left, right).ok_or(ExactArrangementBlocker::UndecidableOrdering)
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

fn boundary_vertices_exact_equal(
    mesh: &ExactMesh,
    left: usize,
    right: usize,
) -> Result<bool, ExactArrangementBlocker> {
    let left = mesh
        .vertices()
        .get(left)
        .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
    let right = mesh
        .vertices()
        .get(right)
        .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
    point3_exact_equal(left, right).ok_or(ExactArrangementBlocker::UndecidableOrdering)
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
            if !exact_points_are_collinear(anchor, first, second)? {
                return Ok(Some([anchor.clone(), first.clone(), second.clone()]));
            }
        }
    }
    Ok(None)
}

fn exact_points_are_collinear(
    a: &Point3,
    b: &Point3,
    c: &Point3,
) -> Result<bool, ExactArrangementBlocker> {
    let abx = b.x.clone() - &a.x;
    let aby = b.y.clone() - &a.y;
    let abz = b.z.clone() - &a.z;
    let acx = c.x.clone() - &a.x;
    let acy = c.y.clone() - &a.y;
    let acz = c.z.clone() - &a.z;
    let cross_x = aby.clone() * &acz - &(abz.clone() * &acy);
    let cross_y = abz * &acx - &(abx.clone() * &acz);
    let cross_z = abx * &acy - &(aby * &acx);
    Ok(compare_reals(&cross_x, &Real::from(0))
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
            == Ordering::Equal)
}

#[derive(Clone, Copy, Default)]
struct BoundaryTopologyEvidence {
    invalid_outgoing_degree_vertices: usize,
    invalid_incoming_degree_vertices: usize,
    overused_edges: usize,
}

fn boundary_topology_evidence(mesh: &ExactMesh) -> BoundaryTopologyEvidence {
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

    let mut outgoing = BTreeMap::<usize, usize>::new();
    let mut incoming = BTreeMap::<usize, usize>::new();
    let mut boundary_vertices = BTreeSet::<usize>::new();
    let mut overused_edges = 0;
    for uses in edge_uses.values() {
        if uses.len() == 1 {
            let (start, end) = uses[0];
            *outgoing.entry(start).or_default() += 1;
            *incoming.entry(end).or_default() += 1;
            boundary_vertices.insert(start);
            boundary_vertices.insert(end);
        } else if uses.len() > 2 {
            overused_edges += 1;
        }
    }

    BoundaryTopologyEvidence {
        invalid_outgoing_degree_vertices: boundary_vertices
            .iter()
            .filter(|&&vertex| outgoing.get(&vertex).copied().unwrap_or(0) != 1)
            .count(),
        invalid_incoming_degree_vertices: boundary_vertices
            .iter()
            .filter(|&&vertex| incoming.get(&vertex).copied().unwrap_or(0) != 1)
            .count(),
        overused_edges,
    }
}

fn directed_boundary_edges(mesh: &ExactMesh) -> BTreeMap<[usize; 2], (usize, usize)> {
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

    edge_uses
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

fn materialize_simple_coplanar_overlay_arrangement(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: Option<ExactMeshValidationPolicy>,
    arrangement: &ExactArrangement,
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
    let allow_empty = matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    );
    let set_operation = match operation {
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
    let requested_overlay =
        build_exact_arrangement2d_overlay(&[left_ring, right_ring], set_operation);
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
    if !coplanar_mesh_overlay_should_preempt_surface_paths(left, right, operation) {
        return Ok(None);
    }
    let allow_empty_overlay = matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    );
    let Some(boundary_policy) = coplanar_mesh_overlay_boundary_policy(left, right, operation)
    else {
        return Ok(None);
    };
    let Some(set_operation) = coplanar_mesh_overlay_set_operation(operation) else {
        return Ok(None);
    };
    let Some(mesh) = materialize_coplanar_mesh_overlay_mesh(
        left,
        right,
        set_operation,
        boundary_policy,
        "exact coplanar mesh overlay arrangement",
        allow_empty_overlay,
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
    if !coplanar_mesh_overlay_should_preempt_surface_paths(left, right, operation) {
        return None;
    }
    let allow_empty_overlay = matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    );
    let boundary_policy = coplanar_mesh_overlay_boundary_policy(left, right, operation)?;
    let set_operation = coplanar_mesh_overlay_set_operation(operation)?;
    materialize_coplanar_mesh_overlay_mesh(
        left,
        right,
        set_operation,
        boundary_policy,
        "exact coplanar mesh overlay arrangement",
        allow_empty_overlay,
    )
    .ok()
    .flatten()
    .map(|mesh| (mesh.vertices().len(), mesh.triangles().len()))
}

fn coplanar_mesh_overlay_boundary_policy(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactArrangement2dBoundaryPolicy> {
    Some(match operation {
        ExactBooleanOperation::Difference => coplanar_mesh_overlay_materialized_boundary_policy(
            left,
            right,
            ExactArrangement2dSetOperation::Difference,
            true,
        )
        .unwrap_or(ExactArrangement2dBoundaryPolicy::SimplifyCollinear),
        ExactBooleanOperation::Intersection => coplanar_mesh_overlay_materialized_boundary_policy(
            left,
            right,
            ExactArrangement2dSetOperation::Intersection,
            true,
        )
        .unwrap_or(ExactArrangement2dBoundaryPolicy::SimplifyCollinear),
        ExactBooleanOperation::Union => ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
        ExactBooleanOperation::SelectedRegions(_) => return None,
    })
}

fn coplanar_mesh_overlay_set_operation(
    operation: ExactBooleanOperation,
) -> Option<ExactArrangement2dSetOperation> {
    Some(match operation {
        ExactBooleanOperation::Union => ExactArrangement2dSetOperation::Union,
        ExactBooleanOperation::Intersection => ExactArrangement2dSetOperation::Intersection,
        ExactBooleanOperation::Difference => ExactArrangement2dSetOperation::Difference,
        ExactBooleanOperation::SelectedRegions(_) => return None,
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
    let Some((carrier_points, projection)) = coplanar_mesh_overlay_carrier(left, right) else {
        return Ok(None);
    };
    let mut rings = Vec::with_capacity(left.triangles().len() + right.triangles().len());
    let Some(left_rings) =
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Left, left, projection)
    else {
        return Ok(None);
    };
    rings.extend(left_rings);
    let Some(right_rings) =
        projected_mesh_boundary_rings(ExactArrangement2dRegion::Right, right, projection)
    else {
        return Ok(None);
    };
    rings.extend(right_rings);
    let overlay =
        build_exact_arrangement2d_overlay_with_boundary_policy(&rings, operation, boundary_policy);
    if !overlay.is_complete() && !overlay_allows_selected_face_materialization(&overlay) {
        return Ok(None);
    }
    if !overlay.faces.iter().any(|face| face.selected) {
        if allow_empty {
            return ExactMesh::new_with_policy(
                Vec::new(),
                Vec::new(),
                SourceProvenance::exact(provenance),
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
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
            .collect::<Option<Vec<_>>>();
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
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(provenance),
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
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
        let boundary = face
            .vertices
            .iter()
            .map(|vertex| {
                let point = &overlay.arrangement.vertices.get(*vertex)?.point;
                lift_projected_point_to_carrier(point, carrier_points, projection)
            })
            .collect::<Option<Vec<_>>>();
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
            .map(|point| {
                if let Some(existing) = find_exact_mesh_vertex(&vertices, &point) {
                    Some(existing)
                } else {
                    let index = vertices.len();
                    vertices.push(point);
                    Some(index)
                }
            })
            .collect::<Option<Vec<_>>>();
        let Some(face_to_mesh) = face_to_mesh else {
            return Ok(None);
        };
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
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(provenance),
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .map(Some)
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
        ExactBooleanOperation::Union => coplanar_mesh_overlay_materialized_boundary_policy(
            left,
            right,
            ExactArrangement2dSetOperation::Union,
            false,
        )
        .is_some(),
        ExactBooleanOperation::Intersection => coplanar_mesh_overlay_materialized_boundary_policy(
            left,
            right,
            ExactArrangement2dSetOperation::Intersection,
            true,
        )
        .is_some(),
        ExactBooleanOperation::Difference => coplanar_mesh_overlay_materialized_boundary_policy(
            left,
            right,
            ExactArrangement2dSetOperation::Difference,
            true,
        )
        .is_some(),
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

fn coplanar_mesh_overlay_materialized_boundary_policy(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactArrangement2dSetOperation,
    allow_empty: bool,
) -> Option<ExactArrangement2dBoundaryPolicy> {
    [
        ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
        ExactArrangement2dBoundaryPolicy::PreserveCollinear,
    ]
    .into_iter()
    .find(|&boundary_policy| {
        matches!(
            materialize_coplanar_mesh_overlay_mesh(
                left,
                right,
                operation,
                boundary_policy,
                "exact coplanar mesh overlay arrangement",
                allow_empty,
            ),
            Ok(Some(_))
        )
    })
}

fn projected_loop_signed_area_twice(points: &[Point2]) -> Real {
    let mut area = Real::from(0);
    for index in 0..points.len() {
        let current = &points[index];
        let next = &points[(index + 1) % points.len()];
        area += &(current.x.clone() * &next.y) - &(current.y.clone() * &next.x);
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
    boolean_arrangement_affine_orthogonal_solid_recovery(left, right, operation, validation)
}

fn boolean_arrangement_orthogonal_solid_cell_recovery(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation) else {
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
    if result.validate().is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

fn boolean_arrangement_affine_orthogonal_solid_recovery(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
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
    let result = certified_shortcut_result(
        arrangement.mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    if result.validate().is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

fn materialize_open_surface_disjoint_meshes(
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
            return Err(unsupported_boolean_operation_error(
                operation,
                "open-surface disjoint materialization requires a named boolean operation",
            ));
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
    let disjoint_report = open_surface_disjoint_report_from_graph(graph, left, right);
    if disjoint_report.is_certified() {
        let result = materialize_open_surface_disjoint_meshes(left, right, operation, validation)?;
        return Ok((disjoint_report
            .validate_against_sources(left, right)
            .is_ok()
            && result.is_certified_shortcut_kind_for(
                operation,
                ExactBooleanShortcutKind::OpenSurfaceDisjoint,
            )
            && result.validate().is_ok())
        .then_some(result));
    }
    Ok(None)
}

pub(crate) fn open_surface_disjoint_result_matches_sources(
    result: &ExactBooleanResult,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> bool {
    let Ok(graph) = build_validated_intersection_graph(left, right) else {
        return false;
    };
    let report = open_surface_disjoint_report_from_graph(&graph, left, right);
    if !report.is_certified()
        || report.validate_against_sources(left, right).is_err()
        || !result.is_certified_shortcut_kind_for(
            operation,
            ExactBooleanShortcutKind::OpenSurfaceDisjoint,
        )
        || result.validate().is_err()
    {
        return false;
    }
    let Ok(expected) = materialize_open_surface_disjoint_meshes(left, right, operation, validation)
    else {
        return false;
    };
    expected.validate().is_ok() && result == &expected
}

pub(crate) fn open_surface_disjoint_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> ExactOpenSurfaceDisjointReport {
    let left_open_surface = mesh_is_open_surface(left);
    let right_open_surface = mesh_is_open_surface(right);
    let graph_had_unknowns = left_open_surface && right_open_surface && graph.has_unknowns();
    let counts = if left_open_surface && right_open_surface {
        retained_graph_counts(graph)
    } else {
        ExactBooleanBlocker::default()
    };
    let status = if !left_open_surface || !right_open_surface {
        ExactOpenSurfaceDisjointStatus::NotOpenSurface
    } else if graph_had_unknowns {
        ExactOpenSurfaceDisjointStatus::GraphUnknowns
    } else if graph.face_pairs.is_empty() {
        ExactOpenSurfaceDisjointStatus::Certified
    } else {
        ExactOpenSurfaceDisjointStatus::GraphHasFacePairs
    };
    let blocker_kind = counts.inferred_kind();
    ExactOpenSurfaceDisjointReport::new(
        status,
        left_open_surface,
        right_open_surface,
        graph_had_unknowns,
        if left_open_surface && right_open_surface {
            graph.face_pairs.len()
        } else {
            0
        },
        if left_open_surface && right_open_surface {
            graph.event_count()
        } else {
            0
        },
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
        && !left_kind.has_volume()
        && !right_kind.has_volume()
    {
        return Some(ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid);
    }
    certified_mixed_dimensional_regularized_solid_support(left, right)
}

/// Retained split-region artifacts that certify an open-surface arrangement.
type OpenSurfaceArrangementPlan = (
    ExactBooleanSupport,
    Vec<FaceRegionPlaneClassification>,
    Vec<FaceRegionTriangulation>,
);

pub(crate) fn replay_open_surface_arrangement_result(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let graph = build_validated_intersection_graph(left, right)?;
    let Some(result) =
        open_surface_arrangement_result_from_graph(&graph, left, right, operation, validation)?
    else {
        return Ok(None);
    };
    if !result.is_open_surface_arrangement_for(operation)
        || result.mesh.validation_policy() != validation
    {
        return Ok(None);
    }
    Ok(Some(result))
}

fn open_surface_arrangement_result_from_graph(
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
    let (_support, region_classifications, triangulations) = plan;
    let selection = match operation {
        ExactBooleanOperation::Union => ExactRegionSelection::KeepAll,
        ExactBooleanOperation::Intersection => ExactRegionSelection::KeepNone,
        ExactBooleanOperation::Difference => ExactRegionSelection::KeepLeft,
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    // Open-surface arrangement is not a closed-volumetric inside/outside
    // split regions are retained by surface operation, and no winding label is
    // invented for a mesh that has no closed volume.
    let mut assembly = ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
        &triangulations,
        selection,
        left,
        right,
    )
    .map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::IndexOutOfBounds,
            format!("open-surface arrangement assembly failed: {error}"),
        ))
    })?;
    assembly
        .canonicalize_for_mesh_with_sources(left, right)
        .map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::IndexOutOfBounds,
                format!("open-surface arrangement assembly canonicalization failed: {error}"),
            ))
        })?;
    let Ok(mesh) = assembly.checked_to_exact_mesh_with_sources(left, right, validation) else {
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
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("open-surface arrangement validation failed: {error:?}"),
        ))
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
    let counts = retained_graph_counts(graph);
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
        || counts.unknown_pairs() != 0
        || counts.construction_failed_events() != 0
        || counts.coplanar_overlapping_pairs() != 0
        || counts.coplanar_touching_pairs() != 0
        || !has_proper_surface_crossing
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
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::DegenerateTriangle,
                format!("open-surface arrangement triangulation failed: {error}"),
            ))
        })?;
    Ok(Some((support, region_classifications, triangulations)))
}

fn boolean_same_surface_meshes(
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
            return Err(unsupported_boolean_operation_error(
                operation,
                "same-surface materialization requires a named boolean operation",
            ));
        }
    };

    Ok(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::SameSurface,
    ))
}

pub(crate) fn replay_closed_same_surface_boolean_result_if_certified(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
        || !evidence::meshes_are_certified_same_surface(left, right)
    {
        return Ok(None);
    }
    boolean_same_surface_meshes(left, operation, validation).map(Some)
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
    if !report.is_certified() {
        return Ok(None);
    }
    report.validate().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("exact closed-boundary-touch report validation failed: {error:?}"),
        ))
    })?;
    Ok(Some(report))
}

fn certified_closed_boundary_only_contact_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactMeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(false);
    }
    validate_graph_source_replay(graph, left, right)?;
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    evidence.validate().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("exact boundary-only coplanar evidence validation failed: {error:?}"),
        ))
    })?;
    Ok(evidence.is_boundary_only_contact())
}

fn closed_zero_area_boundary_contact_evidence_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<CoplanarVolumetricCellEvidenceReport>, ExactMeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(None);
    }
    validate_graph_source_replay(graph, left, right)?;
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    evidence.validate().map_err(|error| {
        ExactMeshError::one(ExactMeshBlocker::new(
            ExactMeshBlockerKind::ExactConstructionFailure,
            format!("exact zero-area boundary contact evidence validation failed: {error:?}"),
        ))
    })?;
    Ok(evidence
        .is_zero_area_boundary_only_contact()
        .then_some(evidence))
}

pub(crate) fn materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>, ExactMeshError> {
    let Some(evidence) = closed_zero_area_boundary_contact_evidence_from_graph(graph, left, right)?
    else {
        return Ok(None);
    };
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || !evidence.is_zero_area_boundary_only_contact()
    {
        return Ok(None);
    }
    let (mesh, shortcut) = match operation {
        ExactBooleanOperation::Union => (
            concatenate_meshes_with_options(
                left,
                right,
                false,
                "exact closed-boundary-touching union preserving separate shells",
                validation,
            )?,
            ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion,
        ),
        ExactBooleanOperation::Intersection => (
            empty_mesh(
                "empty exact closed-boundary-touching intersection",
                validation,
            )?,
            ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection,
        ),
        ExactBooleanOperation::Difference => (
            copy_mesh(
                left,
                "exact closed-boundary-touching difference keeps left",
                validation,
            )?,
            ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
        ),
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    let result = certified_shortcut_result(mesh, operation, shortcut);
    Ok(
        (result.validate().is_ok() && evidence.validate_against_sources(left, right).is_ok())
            .then_some((result, evidence)),
    )
}

fn materialize_boundary_policy_shortcut_result(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union => concatenate_meshes_with_options(
            left,
            right,
            false,
            "exact boundary-touch union preserving separate shells",
            validation,
        ),
        ExactBooleanOperation::Intersection => empty_mesh(
            "empty exact boundary-touch lower-dimensional intersection",
            validation,
        ),
        ExactBooleanOperation::Difference => copy_mesh(
            left,
            "exact boundary-touch difference preserving left shell",
            validation,
        ),
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    };
    let Ok(mesh) = mesh else {
        return Ok(None);
    };
    Ok(Some(ExactBooleanResult {
        kind: ExactBooleanResultKind::BoundaryPolicyShortcut { operation },
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
    }))
}

pub(crate) fn boundary_policy_shortcut_result_matches_sources(
    result: &ExactBooleanResult,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> bool {
    if boundary_policy != ExactBoundaryBooleanPolicy::PreserveSeparateShells {
        return false;
    }
    let Ok(graph) = build_validated_intersection_graph(left, right) else {
        return false;
    };
    let Ok(report) = boundary_touching_report_from_graph(&graph, left, right) else {
        return false;
    };
    if !report.is_certified()
        || report.validate_against_sources(left, right).is_err()
        || !result.is_boundary_policy_shortcut_for(operation)
        || result.validate().is_err()
    {
        return false;
    }
    let Ok(Some(expected)) =
        materialize_boundary_policy_shortcut_result(left, right, operation, validation)
    else {
        return false;
    };
    expected.validate().is_ok() && result == &expected
}

fn boolean_boundary_touching_meshes_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Result<Option<ExactBooleanResult>, ExactMeshError> {
    if boundary_policy == ExactBoundaryBooleanPolicy::Reject {
        return Ok(None);
    }
    let report = boundary_touching_report_from_graph(graph, left, right)?;
    if !report.is_certified() {
        return Ok(None);
    }
    report
        .validate_against_sources(left, right)
        .map_err(|error| {
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!("exact boundary-policy projection consumed invalid certificate: {error:?}"),
            ))
        })?;

    let Some(result) =
        materialize_boundary_policy_shortcut_result(left, right, operation, validation)?
    else {
        return Ok(None);
    };
    Ok(
        (boundary_policy == ExactBoundaryBooleanPolicy::PreserveSeparateShells
            && result.is_boundary_policy_shortcut_for(operation)
            && result.validate().is_ok())
        .then_some(result),
    )
}

#[cfg(test)]
pub(crate) fn winding_evidence_report_for_request_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactWindingEvidenceReport, ExactMeshError> {
    let shortcut_facts = ExactArrangementCellComplexShortcutFacts::from_sources(left, right);
    winding_evidence_report_for_request_from_graph_and_attempt(
        graph,
        left,
        right,
        request,
        None,
        &shortcut_facts,
    )
}

fn winding_evidence_report_for_request_from_graph_and_attempt(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<ExactWindingEvidenceReport, ExactMeshError> {
    if request.validation == ExactMeshValidationPolicy::ALLOW_BOUNDARY
        && request.boundary_policy == ExactBoundaryBooleanPolicy::Reject
    {
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
    let boundary_policy = request.boundary_policy;
    if retained_arrangement_attempt.is_some_and(|attempt| {
        attempt.certifies_regularized_arrangement_cell_complex_output_for_request(request)
    }) {
        return Ok(
            arrangement_cell_complex_already_materialized_winding_evidence(
                graph, left, right, operation,
            ),
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
                return Err(exact_boolean_internal_error(
                    "closed validation gate returned unsupported winding evidence support",
                ));
            }
        };
        winding_evidence_report(
            operation,
            status,
            false,
            0,
            0,
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
                evidence.status(),
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
            )
        } else {
            evidence
        }
    };
    if boundary_policy == ExactBoundaryBooleanPolicy::Reject
        || matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || evidence.status() != ExactWindingEvidenceStatus::BoundaryPolicyRequired
    {
        return Ok(evidence);
    }

    if boolean_boundary_touching_meshes_from_graph(
        graph,
        left,
        right,
        operation,
        validation,
        boundary_policy,
    )?
    .is_some()
    {
        let counts = retained_graph_counts(graph);
        return Ok(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::BoundaryPolicyShortcutAlreadyMaterialized,
            graph.has_unknowns(),
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::BoundaryPolicy),
            None,
            None,
        ));
    }
    Ok(evidence)
}

fn arrangement_cell_complex_already_materialized_winding_evidence(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> ExactWindingEvidenceReport {
    let counts = retained_graph_counts(graph);
    let (blocker_kind, coplanar_evidence) =
        arrangement_materialized_evidence_blocker_kind_and_evidence(graph, left, right);
    winding_evidence_report(
        operation,
        ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
        graph.has_unknowns(),
        graph.face_pairs.len(),
        graph.event_count(),
        0,
        Vec::new(),
        counts.into_blocker(blocker_kind),
        None,
        coplanar_evidence,
    )
}

fn arrangement_materialized_evidence_blocker_kind_and_evidence(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> (
    ExactBooleanBlockerKind,
    Option<CoplanarVolumetricCellEvidenceReport>,
) {
    let coplanar_evidence =
        certified_arrangement_cell_complex_coplanar_evidence(graph, left, right);
    let blocker_kind = match coplanar_evidence
        .as_ref()
        .map(|evidence| evidence.obstacle())
    {
        Some(CoplanarVolumetricCellObstacle::BoundaryOnlyContact) => {
            ExactBooleanBlockerKind::BoundaryPolicy
        }
        Some(obstacle) if obstacle.requires_coplanar_volumetric_cells() => {
            ExactBooleanBlockerKind::CoplanarVolumetricCells
        }
        _ if graph_has_only_boundary_contact_pairs(graph, left, right) => {
            ExactBooleanBlockerKind::BoundaryPolicy
        }
        _ if graph_requires_coplanar_volumetric_cells_for_sources(graph, left, right) => {
            ExactBooleanBlockerKind::CoplanarVolumetricCells
        }
        _ => ExactBooleanBlockerKind::Winding,
    };
    (blocker_kind, coplanar_evidence)
}

fn arrangement_cell_complex_preflight_materialized_winding_evidence(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    graph_had_unknowns: bool,
    counts: ExactBooleanBlocker,
    arrangement_cell_complex_shortcut_materializes: bool,
) -> ExactWindingEvidenceReport {
    let (blocker_kind, mut coplanar_evidence) =
        arrangement_materialized_evidence_blocker_kind_and_evidence(graph, left, right);
    let blocker_kind = if arrangement_cell_complex_shortcut_materializes {
        coplanar_evidence = None;
        ExactBooleanBlockerKind::Winding
    } else {
        blocker_kind
    };
    let blocker = counts.into_blocker(blocker_kind);
    let (retained_face_pairs, retained_events, blocker, coplanar_evidence) = if coplanar_evidence
        .is_some()
        || blocker
            .validate_for_kind(ExactBooleanBlockerKind::Winding)
            .is_ok()
    {
        (
            graph.face_pairs.len(),
            graph.event_count(),
            blocker,
            coplanar_evidence,
        )
    } else {
        (
            0,
            0,
            ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::Winding),
            None,
        )
    };
    winding_evidence_report(
        operation,
        ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        0,
        Vec::new(),
        blocker,
        None,
        coplanar_evidence,
    )
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
            ExactMeshError::one(ExactMeshBlocker::new(
                ExactMeshBlockerKind::StaleFactReplay,
                format!("retained exact intersection graph failed source replay: {error:?}"),
            ))
        })
}

fn retained_graph_counts(graph: &super::graph::ExactIntersectionGraph) -> ExactBooleanBlocker {
    ExactBooleanBlocker::from_graph(graph, ExactBooleanBlockerKind::Winding)
}

pub(crate) fn boundary_touching_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactBoundaryTouchingReport, ExactMeshError> {
    let graph_had_unknowns = graph.has_unknowns();
    let counts = retained_graph_counts(graph);
    let status = if graph_had_unknowns {
        ExactBoundaryTouchingStatus::GraphUnknowns
    } else if graph_requires_boundary_policy(graph, left, right)? {
        ExactBoundaryTouchingStatus::Certified
    } else {
        ExactBoundaryTouchingStatus::NotBoundaryOnly
    };
    let blocker_kind = match status {
        ExactBoundaryTouchingStatus::GraphUnknowns => ExactBooleanBlockerKind::Refinement,
        ExactBoundaryTouchingStatus::Certified => ExactBooleanBlockerKind::BoundaryPolicy,
        ExactBoundaryTouchingStatus::NotBoundaryOnly => counts.inferred_kind(),
    };
    Ok(ExactBoundaryTouchingReport::new(
        status,
        graph_had_unknowns,
        graph.face_pairs.len(),
        graph.event_count(),
        counts.into_blocker(blocker_kind),
    ))
}

#[cfg(test)]
fn not_boundary_only_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
) -> ExactBoundaryTouchingReport {
    let counts = retained_graph_counts(graph);
    let blocker_kind = counts.inferred_kind();
    ExactBoundaryTouchingReport::new(
        ExactBoundaryTouchingStatus::NotBoundaryOnly,
        graph.has_unknowns(),
        graph.face_pairs.len(),
        graph.event_count(),
        counts.into_blocker(blocker_kind),
    )
}

#[cfg(test)]
pub(crate) fn refinement_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    operation: ExactBooleanOperation,
) -> ExactRefinementReport {
    let counts = retained_graph_counts(graph);
    let graph_had_unknowns = graph.has_unknowns();
    let needs_refinement = graph_had_unknowns || counts.construction_failed_events() > 0;
    ExactRefinementReport::new(
        operation,
        if needs_refinement {
            ExactRefinementStatus::Required
        } else {
            ExactRefinementStatus::NotRequired
        },
        graph_had_unknowns,
        graph.face_pairs.len(),
        graph.event_count(),
        needs_refinement.then(|| counts.into_blocker(ExactBooleanBlockerKind::Refinement)),
    )
}

pub(crate) fn planar_arrangement_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactPlanarArrangementReport, ExactMeshError> {
    let mut arrangement_cell_complex_preflight: CertifiedArrangementCellComplexPreflightCache =
        None;
    planar_arrangement_report_from_graph_with_cell_complex_cache(
        graph,
        left,
        right,
        operation,
        &mut arrangement_cell_complex_preflight,
        None,
        None,
    )
}

fn planar_arrangement_report_from_graph_with_cell_complex_cache(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    arrangement_cell_complex_preflight: &mut CertifiedArrangementCellComplexPreflightCache,
    retained_request: Option<ExactBooleanRequest>,
    retained_attempt: Option<&ExactArrangementBooleanAttempt>,
) -> Result<ExactPlanarArrangementReport, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(not_named_planar_arrangement_report(operation));
    }

    let graph_had_unknowns = graph.has_unknowns();
    let counts = retained_graph_counts(graph);
    let coplanar_arrangement_evidence = if graph_had_unknowns {
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
    let status = if graph_had_unknowns {
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
    } else if graph_requires_boundary_policy(graph, left, right)? {
        ExactPlanarArrangementStatus::BoundaryPolicyRequired
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
    Ok(planar_arrangement_report(
        operation,
        status,
        graph_had_unknowns,
        graph.face_pairs.len(),
        graph.event_count(),
        counts,
        coplanar_arrangement_evidence,
    ))
}

pub(crate) fn not_named_planar_arrangement_report(
    operation: ExactBooleanOperation,
) -> ExactPlanarArrangementReport {
    planar_arrangement_report(
        operation,
        ExactPlanarArrangementStatus::NotNamedOperation,
        false,
        0,
        0,
        ExactBooleanBlocker::default(),
        None,
    )
}

fn planar_arrangement_report(
    operation: ExactBooleanOperation,
    status: ExactPlanarArrangementStatus,
    graph_had_unknowns: bool,
    retained_face_pairs: usize,
    retained_events: usize,
    counts: ExactBooleanBlocker,
    coplanar_arrangement_evidence: Option<super::graph::CoplanarArrangementEvidence>,
) -> ExactPlanarArrangementReport {
    let blocker_kind = match status {
        ExactPlanarArrangementStatus::GraphUnknowns => ExactBooleanBlockerKind::Refinement,
        ExactPlanarArrangementStatus::BoundaryPolicyRequired => {
            ExactBooleanBlockerKind::BoundaryPolicy
        }
        ExactPlanarArrangementStatus::Required => ExactBooleanBlockerKind::PlanarArrangement,
        ExactPlanarArrangementStatus::NotNamedOperation
        | ExactPlanarArrangementStatus::AlreadyMaterialized
        | ExactPlanarArrangementStatus::NoPositiveOverlap => counts.inferred_kind(),
    };
    ExactPlanarArrangementReport::new(
        operation,
        status,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        counts.into_blocker(blocker_kind),
        coplanar_arrangement_evidence,
    )
}

fn winding_evidence_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactWindingEvidenceReport, ExactMeshError> {
    let shortcut_facts = ExactArrangementCellComplexShortcutFacts::from_sources(left, right);
    winding_evidence_report_from_graph_with_facts(graph, left, right, operation, &shortcut_facts)
}

fn winding_evidence_report_from_graph_with_facts(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    shortcut_facts: &ExactArrangementCellComplexShortcutFacts,
) -> Result<ExactWindingEvidenceReport, ExactMeshError> {
    if let Some(report) = source_shortcut_winding_evidence_from_sources(left, right, operation) {
        return Ok(report);
    }

    let graph_had_unknowns = graph.has_unknowns();
    let counts = retained_graph_counts(graph);
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        let blocker_kind = counts.inferred_kind();
        return Ok(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::NotNamedOperation,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(blocker_kind),
            None,
            None,
        ));
    }
    if graph_had_unknowns {
        return Ok(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::GraphUnknowns,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::Refinement),
            None,
            None,
        ));
    }
    let arrangement_cell_complex_shortcut_support = shortcut_facts.certified_support(operation);
    let arrangement_cell_complex_shortcut_materializes = arrangement_cell_complex_shortcut_support
        == Some(ExactBooleanSupport::CertifiedArrangementCellComplex);
    let mut arrangement_cell_complex_preflight: CertifiedArrangementCellComplexPreflightCache =
        None;
    if !arrangement_cell_complex_shortcut_materializes
        && graph_requires_coplanar_volumetric_cells_for_sources(graph, left, right)
        && volumetric_boundary_closure_report_from_graph(graph, left, right, operation)?
            .is_coplanar_closure_available()
    {
        return Ok(
            arrangement_cell_complex_already_materialized_winding_evidence(
                graph, left, right, operation,
            ),
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
            graph_had_unknowns,
            graph,
            counts,
        ));
    }
    if operation == ExactBooleanOperation::Difference
        && !arrangement_cell_complex_shortcut_materializes
        && let Some(evidence) = coplanar_boundary_only_evidence_if_consumed(graph, left, right)?
    {
        return Ok(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::BoundaryPolicy),
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
                graph_had_unknowns,
                counts,
                arrangement_cell_complex_shortcut_materializes,
            ),
        );
    }
    if !arrangement_cell_complex_shortcut_materializes
        && certified_convex_operation_shortcut_support(left, right, operation).is_some()
    {
        return Ok(winding_evidence_report_with_validated_winding_blocker(
            operation,
            ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized,
            graph_had_unknowns,
            graph,
            counts,
        ));
    }
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && open_surface_disjoint_report_from_graph(graph, left, right).is_certified()
    {
        return Ok(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::OpenSurfaceDisjointAlreadyMaterialized,
            graph_had_unknowns,
            0,
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::Winding),
            None,
            None,
        ));
    }
    if let Some((_support, region_classifications, _triangulations)) =
        open_surface_arrangement_plan_from_graph(graph, left, right, operation)?
    {
        let region_count = unique_classified_region_count(&region_classifications);
        return Ok(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::OpenSurfaceArrangementAlreadyMaterialized,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            region_count,
            region_classifications,
            counts.into_blocker(ExactBooleanBlockerKind::Winding),
            None,
            None,
        ));
    }
    if let Some(report) = closed_boundary_touching_winding_evidence_from_graph(
        graph,
        left,
        right,
        operation,
        graph_had_unknowns,
        counts,
        arrangement_cell_complex_shortcut_materializes,
    )? {
        return Ok(report);
    }
    if let Some(report) = boundary_policy_or_planar_arrangement_winding_evidence(
        graph,
        left,
        right,
        operation,
        graph_had_unknowns,
        counts,
        arrangement_cell_complex_shortcut_materializes,
        &mut arrangement_cell_complex_preflight,
    )? {
        return Ok(report);
    }
    if arrangement_cell_complex_shortcut_materializes {
        return Ok(
            arrangement_cell_complex_already_materialized_winding_evidence(
                graph, left, right, operation,
            )
            .with_blocker(
                retained_graph_counts(graph).into_blocker(ExactBooleanBlockerKind::Winding),
            ),
        );
    }
    if let Some(report) = volumetric_winding_region_plan_evidence_from_graph(
        graph,
        left,
        right,
        operation,
        graph_had_unknowns,
        counts,
    )? {
        return Ok(report);
    }
    if graph_requires_coplanar_volumetric_cells_for_sources(graph, left, right) {
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
            return Ok(winding_evidence_report(
                operation,
                ExactWindingEvidenceStatus::CoplanarVolumetricCellsAlreadyMaterialized,
                graph_had_unknowns,
                graph.face_pairs.len(),
                graph.event_count(),
                0,
                Vec::new(),
                counts.into_blocker(ExactBooleanBlockerKind::CoplanarVolumetricCells),
                None,
                coplanar_volumetric_evidence_if_required(graph, left, right),
            ));
        }
        return Ok(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::CoplanarVolumetricCellsRequired,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::CoplanarVolumetricCells),
            None,
            coplanar_volumetric_evidence_if_required(graph, left, right),
        ));
    }
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && let Some((left_in_right, right_in_left)) =
            closed_winding_vertex_relations_from_empty_graph(graph, left, right)?
        && left_in_right == ClosedMeshWindingMeshRelation::Outside
        && right_in_left == ClosedMeshWindingMeshRelation::Outside
    {
        return Ok(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::ClosedWindingSeparatedAlreadyMaterialized,
            graph_had_unknowns,
            0,
            graph.event_count(),
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
            ),
        );
    }
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && certified_closed_winding_containment_relation_from_graph(graph, left, right)?.is_some()
    {
        return Ok(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::ClosedWindingContainmentAlreadyMaterialized,
            graph_had_unknowns,
            0,
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::Winding),
            None,
            None,
        ));
    }
    if graph.face_pairs.is_empty() {
        return Ok(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::NoNontrivialOverlap,
            graph_had_unknowns,
            0,
            graph.event_count(),
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
    Ok(winding_evidence_report(
        operation,
        ExactWindingEvidenceStatus::Ready,
        graph_had_unknowns,
        graph.face_pairs.len(),
        graph.event_count(),
        region_plan.regions.len(),
        region_classifications,
        counts.into_blocker(ExactBooleanBlockerKind::Winding),
        None,
        None,
    ))
}

fn closed_boundary_touching_winding_evidence_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    graph_had_unknowns: bool,
    counts: ExactBooleanBlocker,
    arrangement_cell_complex_shortcut_materializes: bool,
) -> Result<Option<ExactWindingEvidenceReport>, ExactMeshError> {
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && closed_zero_area_boundary_contact_evidence_from_graph(graph, left, right)?.is_some()
    {
        return Ok(Some(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::BoundaryPolicy),
            None,
            None,
        )));
    }
    if !arrangement_cell_complex_shortcut_materializes
        && matches!(
            operation,
            ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
        )
        && certified_closed_boundary_only_contact_from_graph(graph, left, right)?
    {
        return Ok(Some(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::BoundaryPolicy),
            None,
            coplanar_boundary_only_evidence_if_consumed(graph, left, right)?,
        )));
    }
    Ok(None)
}

fn volumetric_winding_region_plan_evidence_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    graph_had_unknowns: bool,
    counts: ExactBooleanBlocker,
) -> Result<Option<ExactWindingEvidenceReport>, ExactMeshError> {
    let Some((region_classifications, triangulations, volumetric_classifications)) =
        volumetric_winding_region_plan_from_graph(graph, left, right)?
    else {
        return Ok(None);
    };

    let needs_coplanar_volumetric =
        graph_requires_coplanar_volumetric_cells_for_sources(graph, left, right);
    let blocker_kind = if needs_coplanar_volumetric {
        ExactBooleanBlockerKind::CoplanarVolumetricCells
    } else {
        ExactBooleanBlockerKind::Winding
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
        return Ok(Some(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::Ready,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            materialized.triangulations.len(),
            materialized.region_classifications,
            counts.into_blocker(blocker_kind),
            None,
            coplanar_volumetric_evidence_if_required(graph, left, right),
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
            ),
        ));
    }
    if volumetric_classifications
        .iter()
        .all(|classification| classification.is_materialization_decided())
    {
        let region_count = unique_classified_region_count(&region_classifications);
        return Ok(Some(winding_evidence_report(
            operation,
            ExactWindingEvidenceStatus::VolumetricAssemblyRequired,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            region_count,
            region_classifications,
            counts.into_blocker(blocker_kind),
            None,
            coplanar_volumetric_evidence_if_required(graph, left, right),
        )));
    }

    Ok(None)
}

fn boundary_policy_or_planar_arrangement_winding_evidence(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    graph_had_unknowns: bool,
    counts: ExactBooleanBlocker,
    arrangement_cell_complex_shortcut_materializes: bool,
    arrangement_cell_complex_preflight: &mut CertifiedArrangementCellComplexPreflightCache,
) -> Result<Option<ExactWindingEvidenceReport>, ExactMeshError> {
    let boundary_policy_required = graph_requires_boundary_policy(graph, left, right)?;
    if arrangement_cell_complex_shortcut_materializes && boundary_policy_required {
        return Ok(Some(
            winding_evidence_report_with_validated_winding_blocker(
                operation,
                ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized,
                graph_had_unknowns,
                graph,
                counts,
            ),
        ));
    }
    if boundary_policy_required {
        return Ok(Some(boundary_policy_required_winding_evidence_report(
            graph,
            operation,
            graph_had_unknowns,
            counts,
        )));
    }
    let planar_report = planar_arrangement_report_from_graph_with_cell_complex_cache(
        graph,
        left,
        right,
        operation,
        arrangement_cell_complex_preflight,
        None,
        None,
    )?;
    if planar_report.is_required() {
        return Ok(Some(planar_arrangement_winding_evidence_report(
            graph,
            operation,
            ExactWindingEvidenceStatus::PlanarArrangementRequired,
            graph_had_unknowns,
            counts,
            planar_report.coplanar_arrangement_evidence().cloned(),
        )));
    }
    if planar_report.is_already_materialized() {
        return Ok(Some(planar_arrangement_winding_evidence_report(
            graph,
            operation,
            ExactWindingEvidenceStatus::PlanarArrangementAlreadyMaterialized,
            graph_had_unknowns,
            counts,
            planar_report.coplanar_arrangement_evidence().cloned(),
        )));
    }
    Ok(None)
}

fn winding_evidence_report(
    operation: ExactBooleanOperation,
    status: ExactWindingEvidenceStatus,
    graph_had_unknowns: bool,
    retained_face_pairs: usize,
    retained_events: usize,
    region_count: usize,
    region_classifications: Vec<FaceRegionPlaneClassification>,
    blocker: ExactBooleanBlocker,
    coplanar_arrangement_evidence: Option<super::graph::CoplanarArrangementEvidence>,
    coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
) -> ExactWindingEvidenceReport {
    ExactWindingEvidenceReport::new(
        operation,
        status,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        region_count,
        region_classifications,
        blocker,
        coplanar_arrangement_evidence,
        coplanar_volumetric_evidence,
    )
}

fn source_shortcut_winding_evidence_report(
    operation: ExactBooleanOperation,
    status: ExactWindingEvidenceStatus,
) -> ExactWindingEvidenceReport {
    winding_evidence_report(
        operation,
        status,
        false,
        0,
        0,
        0,
        Vec::new(),
        ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::Winding),
        None,
        None,
    )
}

fn source_shortcut_winding_evidence_from_sources(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactWindingEvidenceReport> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return None;
    }
    let status = if left.triangles().is_empty() || right.triangles().is_empty() {
        ExactWindingEvidenceStatus::EmptyOperandAlreadyMaterialized
    } else if meshes_are_certified_bounds_disjoint(left, right) {
        ExactWindingEvidenceStatus::BoundsDisjointAlreadyMaterialized
    } else if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
        && (evidence::meshes_are_certified_identical(left, right)
            || evidence::meshes_are_certified_same_surface(left, right))
    {
        ExactWindingEvidenceStatus::SurfaceEqualityAlreadyMaterialized
    } else if certified_mixed_dimensional_regularized_solid_support(left, right).is_some() {
        ExactWindingEvidenceStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized
    } else {
        return None;
    };
    Some(source_shortcut_winding_evidence_report(operation, status))
}

fn boundary_policy_required_winding_evidence_report(
    graph: &super::graph::ExactIntersectionGraph,
    operation: ExactBooleanOperation,
    graph_had_unknowns: bool,
    counts: ExactBooleanBlocker,
) -> ExactWindingEvidenceReport {
    winding_evidence_report(
        operation,
        ExactWindingEvidenceStatus::BoundaryPolicyRequired,
        graph_had_unknowns,
        graph.face_pairs.len(),
        graph.event_count(),
        0,
        Vec::new(),
        counts.into_blocker(ExactBooleanBlockerKind::BoundaryPolicy),
        None,
        None,
    )
}

fn planar_arrangement_winding_evidence_report(
    graph: &super::graph::ExactIntersectionGraph,
    operation: ExactBooleanOperation,
    status: ExactWindingEvidenceStatus,
    graph_had_unknowns: bool,
    counts: ExactBooleanBlocker,
    coplanar_arrangement_evidence: Option<super::graph::CoplanarArrangementEvidence>,
) -> ExactWindingEvidenceReport {
    winding_evidence_report(
        operation,
        status,
        graph_had_unknowns,
        graph.face_pairs.len(),
        graph.event_count(),
        0,
        Vec::new(),
        counts.into_blocker(ExactBooleanBlockerKind::PlanarArrangement),
        coplanar_arrangement_evidence,
        None,
    )
}

fn winding_evidence_report_with_validated_winding_blocker(
    operation: ExactBooleanOperation,
    status: ExactWindingEvidenceStatus,
    graph_had_unknowns: bool,
    graph: &super::graph::ExactIntersectionGraph,
    counts: ExactBooleanBlocker,
) -> ExactWindingEvidenceReport {
    let blocker = counts.into_blocker(ExactBooleanBlockerKind::Winding);
    let (retained_face_pairs, retained_events, blocker) = if blocker
        .validate_for_kind(ExactBooleanBlockerKind::Winding)
        .is_ok()
    {
        (graph.face_pairs.len(), graph.event_count(), blocker)
    } else {
        (
            0,
            0,
            ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::Winding),
        )
    };
    winding_evidence_report(
        operation,
        status,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        0,
        Vec::new(),
        blocker,
        None,
        None,
    )
}

type VolumetricWindingRegionPlan = (
    Vec<FaceRegionPlaneClassification>,
    Vec<FaceRegionTriangulation>,
    Vec<ExactVolumetricRegionClassification>,
);

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
    let Some((region_classifications, triangulations, volumetric_classifications)) =
        volumetric_winding_region_plan_from_graph(graph, left, right)?
    else {
        return Ok(None);
    };
    materialize_volumetric_winding_region_plan(
        region_classifications,
        triangulations,
        volumetric_classifications,
        left,
        right,
        operation,
        validation,
    )
}

fn materialize_closed_volumetric_winding_boundary_caps_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ExactMesh>, ExactMeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
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
    certified_coplanar_boundary_closure_from_materialized(
        &materialized,
        left,
        right,
        operation,
        ExactMeshValidationPolicy::CLOSED,
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
    if !volumetric_classifications
        .iter()
        .all(|classification| classification.is_materialization_decided())
    {
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
    let counts = retained_graph_counts(graph);
    if graph.has_unknowns()
        || graph.face_pairs.is_empty()
        || counts.unknown_pairs() != 0
        || counts.construction_failed_events() != 0
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
                ExactMeshError::one(ExactMeshBlocker::new(
                    ExactMeshBlockerKind::StaleFactReplay,
                    format!(
                        "exact volumetric winding region report/source replay failed: {error:?}"
                    ),
                ))
            })?;
    Ok(Some((
        region_classifications,
        triangulations,
        volumetric_classifications,
    )))
}

fn volumetric_retention_for_operation(
    operation: ExactBooleanOperation,
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
    classifications: &[ExactVolumetricRegionClassification],
) -> ExactRegionRetention {
    let Some(classification) = classifications
        .iter()
        .find(|classification| classification.matches_triangulated_cell(triangulation, triangle))
    else {
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
    match (operation, triangulation.side, classification.relation()) {
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

fn winding_error(error: WindingReportError) -> ExactMeshError {
    ExactMeshError::one(ExactMeshBlocker::new(
        ExactMeshBlockerKind::StaleFactReplay,
        format!("exact winding report/source replay failed: {error:?}"),
    ))
}

fn copy_mesh(
    mesh: &ExactMesh,
    label: &'static str,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactMesh, ExactMeshError> {
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
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        hyperlimit::SourceProvenance::exact(label),
        validation,
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
    if left_kind.has_volume() && right_kind.has_volume() {
        return Ok(None);
    }
    if !left_kind.has_volume()
        && !right_kind.has_volume()
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
    validation: ExactMeshValidationPolicy,
) -> Result<ExactBooleanResult, ExactMeshError> {
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
        ExactBooleanOperation::SelectedRegions(_) => {
            return Err(unsupported_boolean_operation_error(
                operation,
                "bounds-disjoint materialization requires a named boolean operation",
            ));
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
        ExactBooleanOperation::Union => concatenate_meshes(left, right, validation)?,
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
        ExactBooleanOperation::Difference => ExactMesh::new_with_policy(
            left.vertices().to_vec(),
            left.triangles().to_vec(),
            hyperlimit::SourceProvenance::exact("exact difference with empty right operand"),
            validation,
        )?,
        ExactBooleanOperation::SelectedRegions(_) => {
            return Err(unsupported_boolean_operation_error(
                operation,
                "empty-operand materialization requires a named boolean operation",
            ));
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
        ExactBooleanOperation::SelectedRegions(_) => {
            return Err(unsupported_boolean_operation_error(
                operation,
                "identical-mesh materialization requires a named boolean operation",
            ));
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
    ExactMesh::new_with_policy(
        Vec::new(),
        Vec::new(),
        hyperlimit::SourceProvenance::exact(label),
        validation,
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

fn concatenate_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ExactMeshValidationPolicy,
) -> Result<ExactMesh, ExactMeshError> {
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
mod tests;
