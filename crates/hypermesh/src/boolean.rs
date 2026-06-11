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

use hyperlimit::SegmentPlaneRelation;

use super::adjacent::{
    full_face_adjacent_certificate, materialize_full_face_adjacent_union_from_certificate,
};
#[cfg(test)]
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
use super::box_solid::is_axis_aligned_box;
use super::cell_complex::{
    arrangement_region_classification_blockers_are_volume_resolved,
    selected_region_selection_ignores_opposite_classification,
};
use super::cells::triangulate_all_face_cells_with_cdt;
use super::contained_adjacent::{
    contained_face_adjacent_certificate, materialize_contained_face_adjacent_union_from_certificate,
};
use super::convex::{
    intersect_closed_convex_solids, subtract_closed_convex_solids, union_closed_convex_solids,
};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::graph::{FacePairEvents, IntersectionEvent, MeshSide, build_intersection_graph};
use super::intersection::MeshFacePairRelation;
use super::loop_triangulation::{group_exact_coplanar_loops, triangulate_exact_loop_group};
use super::mesh::{ExactMesh, Triangle};
use super::orthogonal_solid::{
    AxisAlignedOrthogonalSolidOperation, axis_aligned_orthogonal_solid_cell_plan,
    has_axis_aligned_orthogonal_solid_cells,
    has_empty_axis_aligned_orthogonal_solid_cell_intersection,
    has_non_empty_axis_aligned_orthogonal_solid_cell_intersection,
    materialize_axis_aligned_orthogonal_solid_cell_plan,
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
    ExactAdjacentUnionCompletionReport, ExactAdjacentUnionCompletionStatus, ExactBooleanBlocker,
    ExactBooleanBlockerKind, ExactBooleanPreflight, ExactBooleanResult, ExactBooleanResultKind,
    ExactBooleanShortcutKind, ExactBooleanSupport, ExactBoundaryTouchingReport,
    ExactBoundaryTouchingStatus, ExactOpenSurfaceDisjointReport, ExactOpenSurfaceDisjointStatus,
    ExactPlanarArrangementReport, ExactPlanarArrangementStatus, ExactRefinementReport,
    ExactRefinementStatus, ExactReportFreshness, ExactReportValidationError,
    ExactSameSurfaceReport, ExactSameSurfaceStatus, ExactVolumetricBoundaryClosureReport,
    ExactVolumetricBoundaryClosureStatus, ExactWindingReadinessReport, ExactWindingReadinessStatus,
};
use super::solid::{
    ConvexSolidMeshClassification, ConvexSolidMeshRelation, ConvexSolidPointRelation,
    classify_mesh_vertices_against_convex_solid_report,
};
use super::topology::mesh_for_side;
#[cfg(test)]
use super::topology::triangle_edges as topology_triangle_edges;
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
    /// Output validation policy used by shortcut recovery and final mesh copy.
    pub output_validation: ValidationPolicy,
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

impl ExactArrangementBooleanAttempt {
    /// Validate this retained arrangement/cell-complex attempt as a coherent
    /// audit artifact.
    ///
    /// The attempt report is a public provenance object for a staged topology
    /// construction. Its stage, decline reason, shortcut materialization, and
    /// retained counts must describe one path through that state machine rather
    /// than an arbitrary mix of successful output and blockers.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if self.selected_faces > self.face_cells
            || self.selected_volume_regions > self.volume_regions
            || (self.volume_regions != 0 && self.regions == 0)
            || (self.volume_adjacencies != 0 && self.volume_regions < 2)
            || (self.selected_volume_regions != 0 && self.volume_regions == 0)
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }

        match &self.decline {
            Some(decline) => {
                if self.materialized_shortcut.is_some()
                    || self.stage == ExactArrangementBooleanStage::Materialized
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if let ExactArrangementBooleanDecline::ArrangementBlockers(blockers) = decline
                    && (blockers.is_empty() || blockers.len() != self.arrangement_blockers)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if !arrangement_attempt_decline_matches_stage(decline, self.stage) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            None => {
                if self.stage != ExactArrangementBooleanStage::Materialized
                    || self.materialized_shortcut.is_none()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
        }

        if let Some(shortcut) = self.materialized_shortcut {
            if shortcut != ExactBooleanShortcutKind::ArrangementCellComplex {
                return Err(ExactReportValidationError::StatusEvidenceMismatch);
            }
        }
        if self.output_triangles != 0 && self.output_vertices == 0 {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if self.decline.is_none()
            && self.operation == ExactBooleanOperation::Union
            && self.output_triangles == 0
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if !arrangement_attempt_counts_match_stage(self) {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        Ok(())
    }

    /// Validate this attempt by replaying it from the source meshes.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay = arrangement_boolean_attempt_report(
            left,
            right,
            ExactBooleanRequest::new(self.operation, self.output_validation),
            self.policy,
        )
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        replay.validate()?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Validate this attempt by replaying it under an explicit output
    /// validation policy.
    pub fn validate_against_sources_with_validation(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        validation: ValidationPolicy,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay = arrangement_boolean_attempt_report(
            left,
            right,
            ExactBooleanRequest::new(self.operation, validation),
            self.policy,
        )
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        replay.validate()?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained arrangement attempt is fresh.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match arrangement_boolean_attempt_report(
            left,
            right,
            ExactBooleanRequest::new(self.operation, self.output_validation),
            self.policy,
        ) {
            Ok(replay) if replay.validate().is_ok() && self == &replay => {
                ExactReportFreshness::Current
            }
            Ok(_) | Err(_) => ExactReportFreshness::SourceReplayMismatch,
        }
    }

    /// Classify whether this retained arrangement attempt is fresh under an
    /// explicit output validation policy.
    pub fn freshness_against_sources_with_validation(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        validation: ValidationPolicy,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match arrangement_boolean_attempt_report(
            left,
            right,
            ExactBooleanRequest::new(self.operation, validation),
            self.policy,
        ) {
            Ok(replay) if replay.validate().is_ok() && self == &replay => {
                ExactReportFreshness::Current
            }
            Ok(_) | Err(_) => ExactReportFreshness::SourceReplayMismatch,
        }
    }
}

fn arrangement_attempt_decline_matches_stage(
    decline: &ExactArrangementBooleanDecline,
    stage: ExactArrangementBooleanStage,
) -> bool {
    matches!(
        (decline, stage),
        (
            ExactArrangementBooleanDecline::DispatchGate,
            ExactArrangementBooleanStage::NotAttempted
        ) | (
            ExactArrangementBooleanDecline::ArrangementBlockers(_)
                | ExactArrangementBooleanDecline::Labeling(_),
            ExactArrangementBooleanStage::ArrangementBuilt
        ) | (
            ExactArrangementBooleanDecline::Selection(_),
            ExactArrangementBooleanStage::Labeled
        ) | (
            ExactArrangementBooleanDecline::Simplification(_),
            ExactArrangementBooleanStage::Selected
        ) | (
            ExactArrangementBooleanDecline::Triangulation(_),
            ExactArrangementBooleanStage::Simplified
        ) | (
            ExactArrangementBooleanDecline::OutputValidation,
            ExactArrangementBooleanStage::Triangulated
        )
    )
}

fn arrangement_attempt_counts_match_stage(attempt: &ExactArrangementBooleanAttempt) -> bool {
    let stage = attempt.stage;
    if stage == ExactArrangementBooleanStage::NotAttempted {
        return attempt.arrangement_blockers == 0
            && attempt.face_cells == 0
            && attempt.regions == 0
            && attempt.volume_regions == 0
            && attempt.volume_adjacencies == 0
            && attempt.lower_dimensional_artifacts == 0
            && attempt.selected_faces == 0
            && attempt.selected_volume_regions == 0
            && attempt.output_vertices == 0
            && attempt.output_triangles == 0;
    }
    if !arrangement_attempt_stage_reaches(stage, ExactArrangementBooleanStage::Labeled)
        && (attempt.selected_faces != 0 || attempt.selected_volume_regions != 0)
    {
        return false;
    }
    if !arrangement_attempt_stage_reaches(stage, ExactArrangementBooleanStage::Triangulated)
        && (attempt.output_vertices != 0 || attempt.output_triangles != 0)
    {
        return false;
    }
    true
}

fn arrangement_attempt_stage_reaches(
    stage: ExactArrangementBooleanStage,
    target: ExactArrangementBooleanStage,
) -> bool {
    arrangement_attempt_stage_rank(stage) >= arrangement_attempt_stage_rank(target)
}

const fn arrangement_attempt_stage_rank(stage: ExactArrangementBooleanStage) -> u8 {
    match stage {
        ExactArrangementBooleanStage::NotAttempted => 0,
        ExactArrangementBooleanStage::ArrangementBuilt => 1,
        ExactArrangementBooleanStage::Labeled => 2,
        ExactArrangementBooleanStage::Selected => 3,
        ExactArrangementBooleanStage::Simplified => 4,
        ExactArrangementBooleanStage::Triangulated => 5,
        ExactArrangementBooleanStage::Materialized => 6,
    }
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

/// Complete policy for an exact boolean request.
///
/// The request keeps operation semantics, output validation, and
/// lower-dimensional boundary projection policy together so preflight,
/// certification, and materialization replay the same exact contract.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactBooleanRequest {
    /// Named or selected-region operation to evaluate.
    pub operation: ExactBooleanOperation,
    /// Output mesh validation policy.
    pub validation: ValidationPolicy,
    /// Explicit boundary-only projection policy.
    pub boundary_policy: ExactBoundaryBooleanPolicy,
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
    pub const fn new(operation: ExactBooleanOperation, validation: ValidationPolicy) -> Self {
        Self {
            operation,
            validation,
            boundary_policy: ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        }
    }

    /// Creates a request with an explicit boundary projection policy.
    pub const fn with_boundary_policy(
        operation: ExactBooleanOperation,
        validation: ValidationPolicy,
        boundary_policy: ExactBoundaryBooleanPolicy,
    ) -> Self {
        Self {
            operation,
            validation,
            boundary_policy,
        }
    }

    /// Preflight this request against source meshes.
    pub fn preflight(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<ExactBooleanPreflight, MeshError> {
        preflight_boolean_exact_request(left, right, self)
    }

    /// Evaluate this request into a certified result or retained exact
    /// blockers/provenance.
    pub fn evaluate(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<ExactBooleanEvaluation, MeshError> {
        evaluate_boolean_exact_request(left, right, self)
    }

    /// Materialize this request, returning an error when the retained exact
    /// state is blocked by policy, refinement, or unsupported topology.
    pub fn materialize(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<ExactBooleanResult, MeshError> {
        materialize_boolean_exact_request(left, right, self)
    }

    /// Report how far this request gets through the arrangement/cell-complex
    /// Boolean pipeline without falling through to unrelated materializers.
    pub fn arrangement_attempt(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<ExactArrangementBooleanAttempt, MeshError> {
        arrangement_boolean_attempt_report(left, right, self, policy)
    }

    /// Report exact winding/inside-outside readiness under this request's
    /// validation and boundary projection policy.
    pub fn winding_readiness(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<ExactWindingReadinessReport, MeshError> {
        winding_readiness_report_for_request(left, right, self)
    }

    /// Certify whether retained volumetric boundary output for this request can
    /// be closed by the existing exact cap generators.
    pub fn volumetric_boundary_closure(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<ExactVolumetricBoundaryClosureReport, MeshError> {
        volumetric_boundary_closure_report_for_request(left, right, self)
    }

    /// Report whether exact graph extraction retained unresolved predicate or
    /// construction evidence before topology policy is considered.
    pub fn refinement_report(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<ExactRefinementReport, MeshError> {
        refinement_report_for_request(left, right, self)
    }

    /// Report whether this request is blocked on exact planar-arrangement
    /// output for positive-area coplanar topology.
    pub fn planar_arrangement_report(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<ExactPlanarArrangementReport, MeshError> {
        planar_arrangement_report_for_request(left, right, self)
    }

    /// Certify whether adjacent closed-solid union completion can materialize
    /// this request.
    pub fn adjacent_union_completion_report(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<ExactAdjacentUnionCompletionReport, MeshError> {
        adjacent_union_completion_report_for_request(left, right, self)
    }

    /// Certify whether this request's retained graph is boundary-only contact.
    pub fn boundary_touching_report(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<ExactBoundaryTouchingReport, MeshError> {
        boundary_touching_report_for_request(left, right, self)
    }

    /// Certify whether open-surface operands are graph-disjoint for this
    /// request.
    pub fn open_surface_disjoint_report(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<ExactOpenSurfaceDisjointReport, MeshError> {
        open_surface_disjoint_report_for_request(left, right, self)
    }

    /// Certify whether source meshes are exactly identical for boolean
    /// shortcut replay.
    pub fn identical_mesh_report(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactIdenticalMeshReport {
        identical_mesh_report_for_request(left, right, self)
    }

    /// Certify whether source meshes represent the same triangle surface for
    /// boolean shortcut replay.
    pub fn same_surface_report(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactSameSurfaceReport {
        same_surface_report_for_request(left, right, self)
    }

    /// Materialize the empty-operand shortcut for this request, when it owns
    /// the replay provenance.
    pub fn materialize_empty_operand(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_empty_operand_for_request(left, right, self)
    }

    /// Materialize the bounds-disjoint shortcut for this request, when exact
    /// AABB facts prove the shortcut owns the replay provenance.
    pub fn materialize_bounds_disjoint(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_bounds_disjoint_for_request(left, right, self)
    }

    /// Materialize the identical-mesh shortcut for this request, when it owns
    /// the replay provenance.
    pub fn materialize_identical_mesh(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_identical_mesh_for_request(left, right, self)
    }

    /// Materialize the same-surface shortcut for this request, when it owns
    /// the replay provenance.
    pub fn materialize_same_surface(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_same_surface_for_request(left, right, self)
    }

    /// Materialize closed-output lower-dimensional regularization for this
    /// request, when it owns the replay provenance.
    pub fn materialize_closed_regularized_lower_dimensional(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_closed_regularized_lower_dimensional_for_request(left, right, self)
    }

    /// Materialize mixed-dimensional closed-solid regularization for this
    /// request, when it owns the replay provenance.
    pub fn materialize_mixed_dimensional_regularized_solid(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_mixed_dimensional_regularized_solid_for_request(left, right, self)
    }

    /// Materialize the open-surface disjoint shortcut for this request, when
    /// exact graph facts prove the shortcut owns the replay provenance.
    pub fn materialize_open_surface_disjoint(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_open_surface_disjoint_for_request(left, right, self)
    }

    /// Materialize explicit boundary-only projection for this request, when
    /// its boundary policy allows that projection.
    pub fn materialize_boundary_touching_policy(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_boundary_touching_policy_for_request(left, right, self)
    }

    /// Materialize closed same-surface arrangement output for this request,
    /// when that shortcut owns replay provenance.
    pub fn materialize_closed_same_surface(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_closed_same_surface_for_request(left, right, self)
    }

    /// Materialize positive-area closed boundary contact with no shared
    /// volume, returning the exact evidence consumed by this request.
    pub fn materialize_closed_no_volume_overlap_regularized_with_evidence(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>, MeshError> {
        materialize_closed_no_volume_overlap_regularized_with_evidence_for_request(
            left, right, self,
        )
    }

    /// Materialize zero-area closed boundary contact, returning the exact
    /// evidence consumed by this request.
    pub fn materialize_closed_boundary_touching_regularized_with_evidence(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>, MeshError> {
        materialize_closed_boundary_touching_regularized_with_evidence_for_request(
            left, right, self,
        )
    }

    /// Materialize the closed-winding containment shortcut for this request,
    /// when exact winding facts own the replay provenance.
    pub fn materialize_closed_winding_containment(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_closed_winding_containment_for_request(left, right, self)
    }

    /// Materialize the closed-winding separation shortcut for this request,
    /// when exact winding facts own the replay provenance.
    pub fn materialize_closed_winding_separated(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_closed_winding_separated_for_request(left, right, self)
    }

    /// Materialize a closed-convex shortcut for this request, when convex facts
    /// own the replay provenance.
    pub fn materialize_closed_convex(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_closed_convex_for_request(left, right, self)
    }

    /// Materialize axis-aligned orthogonal solid recovery for this request,
    /// when the orthogonal-cell model owns replay provenance.
    pub fn materialize_axis_aligned_orthogonal_solid(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_axis_aligned_orthogonal_solid_for_request(left, right, self)
    }

    /// Materialize affine orthogonal solid recovery for this request, when the
    /// affine-cell model owns replay provenance.
    pub fn materialize_affine_orthogonal_solid(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<ExactBooleanResult>, MeshError> {
        materialize_affine_orthogonal_solid_for_request(left, right, self)
    }

    /// Materialize adjacent closed-solid union completion for this request,
    /// returning the exact report consumed by the materializer.
    pub fn materialize_adjacent_union_completion(
        self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<Option<(ExactBooleanResult, ExactAdjacentUnionCompletionReport)>, MeshError> {
        materialize_adjacent_union_completion_for_request(left, right, self)
    }
}

/// Replayable certification bundle for an exact boolean request.
///
/// These reports are intentionally redundant with the preflight summary. The
/// summary is the scheduling decision, while this bundle keeps the Yap-style
/// exact facts that explain which stage certified, blocked, or declined the
/// requested operation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBooleanCertificationSet {
    /// Source-shape facts used by trivial shortcut supports.
    pub trivial: ExactTrivialBooleanFacts,
    /// Source-shape facts used by closed regularized-solid shortcut supports.
    pub regularized_solid: ExactRegularizedSolidBooleanFacts,
    /// Exact graph refinement status.
    pub refinement: ExactRefinementReport,
    /// Boundary-contact policy status.
    pub boundary_touching: ExactBoundaryTouchingReport,
    /// Open-surface disjointness shortcut status.
    pub open_surface_disjoint: ExactOpenSurfaceDisjointReport,
    /// Identical-mesh shortcut status.
    pub identical: ExactIdenticalMeshReport,
    /// Same-surface shortcut status.
    pub same_surface: ExactSameSurfaceReport,
    /// Left vertices classified against the right closed mesh.
    pub closed_winding_left_in_right: ClosedMeshWindingMeshReport,
    /// Right vertices classified against the left closed mesh.
    pub closed_winding_right_in_left: ClosedMeshWindingMeshReport,
    /// Left vertices classified against the right convex solid.
    pub convex_left_in_right: ConvexSolidMeshClassification,
    /// Right vertices classified against the left convex solid.
    pub convex_right_in_left: ConvexSolidMeshClassification,
    /// Direct closed-convex boolean capabilities.
    pub convex_capabilities: ExactConvexBooleanCapabilityFacts,
    /// Direct arrangement-cell shortcut capabilities that do not consume the
    /// full arrangement attempt report.
    pub arrangement_cell_complex_shortcuts: ExactArrangementCellComplexShortcutFacts,
    /// Planar-arrangement readiness for coplanar surface output.
    pub planar_arrangement: ExactPlanarArrangementReport,
    /// Winding/inside-outside readiness for named volumetric output.
    pub winding_readiness: ExactWindingReadinessReport,
    /// Volumetric boundary closure readiness, when meaningful for the request.
    pub volumetric_boundary_closure: Option<ExactVolumetricBoundaryClosureReport>,
    /// Arrangement/cell-complex materialization attempt.
    pub arrangement_attempt: Option<ExactArrangementBooleanAttempt>,
}

impl ExactBooleanCertificationSet {
    fn from_sources(
        left: &ExactMesh,
        right: &ExactMesh,
        request: ExactBooleanRequest,
    ) -> Result<Self, MeshError> {
        let graph = build_intersection_graph(left, right)?;
        validate_graph_source_handoff(&graph, left, right)?;
        let trivial = ExactTrivialBooleanFacts::from_sources(left, right);
        let regularized_solid = ExactRegularizedSolidBooleanFacts::from_sources(left, right);
        let refinement = refinement_report_from_graph(&graph, request.operation);
        let boundary_touching = boundary_touching_report_from_graph(&graph, left, right)?;
        let open_surface_disjoint = open_surface_disjoint_report_from_graph(&graph, left, right);
        let identical = request.identical_mesh_report(left, right);
        let same_surface = request.same_surface_report(left, right);
        let closed_winding_left_in_right =
            classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
        let closed_winding_right_in_left =
            classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
        let convex_left_in_right = classify_mesh_vertices_against_convex_solid_report(left, right);
        let convex_right_in_left = classify_mesh_vertices_against_convex_solid_report(right, left);
        let convex_capabilities = ExactConvexBooleanCapabilityFacts::from_sources(left, right);
        let arrangement_cell_complex_shortcuts =
            ExactArrangementCellComplexShortcutFacts::from_sources(left, right);
        let planar_arrangement =
            planar_arrangement_certification_from_graph(&graph, left, right, request.operation)?;
        let winding_readiness = winding_readiness_report_with_boundary_policy_from_graph(
            &graph,
            left,
            right,
            request.operation,
            request.validation,
            request.boundary_policy,
        )?;
        let volumetric_boundary_closure =
            if matches!(request.operation, ExactBooleanOperation::SelectedRegions(_)) {
                None
            } else {
                Some(volumetric_boundary_closure_report_from_graph(
                    &graph,
                    left,
                    right,
                    request.operation,
                )?)
            };
        let arrangement_attempt =
            if matches!(request.operation, ExactBooleanOperation::SelectedRegions(_)) {
                None
            } else {
                Some(request.arrangement_attempt(
                    left,
                    right,
                    ExactRegularizationPolicy::REGULARIZED_SOLID,
                )?)
            };
        Ok(Self {
            trivial,
            regularized_solid,
            refinement,
            boundary_touching,
            open_surface_disjoint,
            identical,
            same_surface,
            closed_winding_left_in_right,
            closed_winding_right_in_left,
            convex_left_in_right,
            convex_right_in_left,
            convex_capabilities,
            arrangement_cell_complex_shortcuts,
            planar_arrangement,
            winding_readiness,
            volumetric_boundary_closure,
            arrangement_attempt,
        })
    }

    /// Validate this certification bundle against the request it claims to
    /// explain, without replaying source geometry.
    pub fn validate_for_request(
        &self,
        request: ExactBooleanRequest,
    ) -> Result<(), ExactReportValidationError> {
        self.trivial.validate()?;
        self.regularized_solid.validate()?;
        self.refinement.validate()?;
        self.boundary_touching.validate()?;
        self.open_surface_disjoint.validate()?;
        self.identical.validate()?;
        self.same_surface.validate()?;
        self.closed_winding_left_in_right
            .validate()
            .map_err(|_| ExactReportValidationError::StatusEvidenceMismatch)?;
        self.closed_winding_right_in_left
            .validate()
            .map_err(|_| ExactReportValidationError::StatusEvidenceMismatch)?;
        self.convex_left_in_right
            .validate()
            .map_err(|_| ExactReportValidationError::StatusEvidenceMismatch)?;
        self.convex_right_in_left
            .validate()
            .map_err(|_| ExactReportValidationError::StatusEvidenceMismatch)?;
        self.convex_capabilities.validate()?;
        self.arrangement_cell_complex_shortcuts.validate()?;
        self.planar_arrangement.validate()?;
        self.winding_readiness.validate()?;
        if self.refinement.operation != request.operation
            || self.planar_arrangement.operation != request.operation
            || self.winding_readiness.operation != request.operation
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if self.refinement.graph_had_unknowns != self.boundary_touching.graph_had_unknowns
            || self.refinement.retained_face_pairs != self.boundary_touching.retained_face_pairs
            || self.refinement.retained_events != self.boundary_touching.retained_events
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if matches!(request.operation, ExactBooleanOperation::SelectedRegions(_)) {
            if self.volumetric_boundary_closure.is_some() || self.arrangement_attempt.is_some() {
                return Err(ExactReportValidationError::StatusEvidenceMismatch);
            }
            return Ok(());
        }
        if self.refinement.graph_had_unknowns != self.planar_arrangement.graph_had_unknowns
            || self.refinement.retained_face_pairs != self.planar_arrangement.retained_face_pairs
            || self.refinement.retained_events != self.planar_arrangement.retained_events
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        let Some(report) = self.volumetric_boundary_closure.as_ref() else {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        };
        report.validate()?;
        if report.operation != request.operation {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        let Some(attempt) = self.arrangement_attempt.as_ref() else {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        };
        attempt.validate()?;
        if attempt.operation != request.operation
            || attempt.policy != ExactRegularizationPolicy::REGULARIZED_SOLID
            || attempt.output_validation != request.validation
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        Ok(())
    }

    /// Validate this retained certification bundle by replaying every report
    /// from the source meshes under the request policy that produced it.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        request: ExactBooleanRequest,
    ) -> Result<(), ExactReportValidationError> {
        self.validate_for_request(request)?;
        let replay = Self::from_sources(left, right, request)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained certification bundle is fresh for the
    /// source meshes and request policy.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        request: ExactBooleanRequest,
    ) -> ExactReportFreshness {
        match self.validate_against_sources(left, right, request) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }
}

/// Replayable source-shape facts for exact boolean shortcuts that do not need
/// graph topology.
///
/// These facts deliberately mirror preflight shortcut semantics rather than the
/// lower-level bounds helper: an empty operand is certified as empty, not as a
/// bounds-disjoint non-empty pair even when it has no mesh bounds.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExactTrivialBooleanFacts {
    /// The left source has no input triangles.
    pub left_empty: bool,
    /// The right source has no input triangles.
    pub right_empty: bool,
    /// Both sources are non-empty and their exact mesh AABBs are disjoint.
    pub bounds_disjoint: bool,
}

/// Replayable source-shape facts for closed regularized-solid shortcut
/// supports.
///
/// These facts retain the exact mesh-topology predicates used to classify
/// whether an operand contributes closed volume. Empty operands are not
/// represented as lower-dimensional here because the public dispatcher gives
/// them distinct empty-operand provenance before regularized-solid shortcuts.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExactRegularizedSolidBooleanFacts {
    /// The left source is a non-empty closed manifold solid.
    pub left_closed_solid: bool,
    /// The right source is a non-empty closed manifold solid.
    pub right_closed_solid: bool,
    /// The left source is a supported non-empty open manifold surface.
    pub left_open_surface: bool,
    /// The right source is a supported non-empty open manifold surface.
    pub right_open_surface: bool,
}

impl ExactRegularizedSolidBooleanFacts {
    fn from_sources(left: &ExactMesh, right: &ExactMesh) -> Self {
        Self {
            left_closed_solid: !left.triangles().is_empty() && left.facts().mesh.closed_manifold,
            right_closed_solid: !right.triangles().is_empty() && right.facts().mesh.closed_manifold,
            left_open_surface: mesh_is_open_surface(left),
            right_open_surface: mesh_is_open_surface(right),
        }
    }

    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if (self.left_closed_solid && self.left_open_surface)
            || (self.right_closed_solid && self.right_open_surface)
        {
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        } else {
            Ok(())
        }
    }

    fn supports_mixed_dimensional_regularized_solid(&self) -> bool {
        (self.left_closed_solid && self.right_open_surface)
            || (self.left_open_surface && self.right_closed_solid)
    }

    fn supports_lower_dimensional_regularized_solid(&self) -> bool {
        self.left_open_surface && self.right_open_surface
    }
}

/// Replayable source facts for direct closed-convex boolean materialization.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactConvexBooleanCapabilityFacts {
    /// Exact closed-convex union can be materialized directly.
    pub can_union: bool,
    /// Exact closed-convex intersection can be materialized directly.
    pub can_intersection: bool,
    /// Exact closed-convex difference can be materialized directly.
    pub can_difference: bool,
}

impl ExactConvexBooleanCapabilityFacts {
    fn from_sources(left: &ExactMesh, right: &ExactMesh) -> Self {
        Self {
            can_union: union_closed_convex_solids(left, right).is_some(),
            can_intersection: intersect_closed_convex_solids(left, right).is_some(),
            can_difference: subtract_closed_convex_solids(left, right).is_some(),
        }
    }

    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        Ok(())
    }

    fn supports(&self, support: ExactBooleanSupport) -> bool {
        match support {
            ExactBooleanSupport::CertifiedConvexUnion => self.can_union,
            ExactBooleanSupport::CertifiedConvexIntersection => self.can_intersection,
            ExactBooleanSupport::CertifiedConvexDifference => self.can_difference,
            _ => false,
        }
    }
}

/// Replayable source facts for direct arrangement-cell-complex shortcut
/// materializers that do not consume the general arrangement attempt report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactArrangementCellComplexShortcutFacts {
    /// Axis-aligned orthogonal cell decomposition supports union.
    pub axis_aligned_union: bool,
    /// Axis-aligned orthogonal cell decomposition supports intersection.
    pub axis_aligned_intersection: bool,
    /// Axis-aligned orthogonal cell decomposition supports difference.
    pub axis_aligned_difference: bool,
    /// Affine orthogonal cell decomposition supports union.
    pub affine_union: bool,
    /// Affine orthogonal cell decomposition supports intersection.
    pub affine_intersection: bool,
    /// Affine orthogonal cell decomposition supports difference.
    pub affine_difference: bool,
}

impl ExactArrangementCellComplexShortcutFacts {
    fn from_sources(left: &ExactMesh, right: &ExactMesh) -> Self {
        Self {
            axis_aligned_union: has_axis_aligned_orthogonal_solid_cells(
                left,
                right,
                AxisAlignedOrthogonalSolidOperation::Union,
            ),
            axis_aligned_intersection: has_axis_aligned_orthogonal_solid_cells(
                left,
                right,
                AxisAlignedOrthogonalSolidOperation::Intersection,
            ),
            axis_aligned_difference: has_axis_aligned_orthogonal_solid_cells(
                left,
                right,
                AxisAlignedOrthogonalSolidOperation::Difference,
            ),
            affine_union: has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Union,
            ),
            affine_intersection: has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Intersection,
            ),
            affine_difference: has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Difference,
            ),
        }
    }

    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        Ok(())
    }

    fn supports(&self, operation: ExactBooleanOperation) -> bool {
        match operation {
            ExactBooleanOperation::Union => self.axis_aligned_union || self.affine_union,
            ExactBooleanOperation::Intersection => {
                self.axis_aligned_intersection || self.affine_intersection
            }
            ExactBooleanOperation::Difference => {
                self.axis_aligned_difference || self.affine_difference
            }
            ExactBooleanOperation::SelectedRegions(_) => false,
        }
    }
}

/// Replayable exact identity certificate for the identical-mesh shortcut.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactIdenticalMeshReport {
    /// Coarse identity status.
    pub status: ExactIdenticalMeshStatus,
    /// Number of left source vertices compared in original order.
    pub left_vertices: usize,
    /// Number of right source vertices compared in original order.
    pub right_vertices: usize,
    /// Number of left source triangles compared in original order.
    pub left_triangles: usize,
    /// Number of right source triangles compared in original order.
    pub right_triangles: usize,
    /// Exact coordinate comparison predicates used for original-order vertex
    /// identity.
    pub predicates: Vec<PredicateUse>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactIdenticalMeshStatus {
    /// Vertex counts differ.
    VertexCountMismatch,
    /// A coordinate comparison was undecided.
    VertexCoordinateUndecided,
    /// At least one same-index vertex coordinate differs.
    VertexCoordinateMismatch,
    /// Triangle counts or same-index triangle records differ.
    TriangleSequenceMismatch,
    /// Vertices and triangles are exactly identical in source order.
    Certified,
}

impl ExactIdenticalMeshReport {
    pub const fn is_certified(&self) -> bool {
        matches!(self.status, ExactIdenticalMeshStatus::Certified)
    }

    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if self.predicates.len() > self.left_vertices.saturating_mul(3) {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        match self.status {
            ExactIdenticalMeshStatus::VertexCountMismatch => {
                if self.left_vertices == self.right_vertices || !self.predicates.is_empty() {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactIdenticalMeshStatus::VertexCoordinateUndecided
            | ExactIdenticalMeshStatus::VertexCoordinateMismatch => {
                if self.left_vertices != self.right_vertices || self.predicates.is_empty() {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactIdenticalMeshStatus::TriangleSequenceMismatch => {
                if self.left_vertices != self.right_vertices
                    || self.predicates.len() != self.left_vertices.saturating_mul(3)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactIdenticalMeshStatus::Certified => {
                if self.left_vertices != self.right_vertices
                    || self.left_triangles != self.right_triangles
                    || self.predicates.len() != self.left_vertices.saturating_mul(3)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
        }
        Ok(())
    }
}

impl ExactTrivialBooleanFacts {
    fn from_sources(left: &ExactMesh, right: &ExactMesh) -> Self {
        let left_empty = left.triangles().is_empty();
        let right_empty = right.triangles().is_empty();
        Self {
            left_empty,
            right_empty,
            bounds_disjoint: !left_empty
                && !right_empty
                && meshes_are_certified_bounds_disjoint(left, right),
        }
    }

    fn validate(&self) -> Result<(), ExactReportValidationError> {
        if self.bounds_disjoint && (self.left_empty || self.right_empty) {
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        } else {
            Ok(())
        }
    }

    fn has_empty_operand(&self) -> bool {
        self.left_empty || self.right_empty
    }
}

fn planar_arrangement_certification_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactPlanarArrangementReport, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        Ok(planar_arrangement_report(
            operation,
            ExactPlanarArrangementStatus::NotNamedOperation,
            false,
            0,
            0,
            ExactBooleanBlocker::default(),
            None,
        ))
    } else {
        planar_arrangement_report_from_graph(graph, left, right, operation)
    }
}

/// Complete exact boolean evaluation outcome.
///
/// `result` is present only when the request materialized under retained exact
/// evidence. When it is absent, `preflight` and `certifications` retain the
/// blocker/provenance facts instead of collapsing the request to an
/// approximate or prose-only error.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBooleanEvaluation {
    /// Request policy evaluated.
    pub request: ExactBooleanRequest,
    /// Exact preflight/scheduling result.
    pub preflight: ExactBooleanPreflight,
    /// Replayable exact certification reports for the request.
    pub certifications: ExactBooleanCertificationSet,
    /// Materialized exact result, when certified under `request`.
    pub result: Option<ExactBooleanResult>,
}

impl ExactBooleanEvaluation {
    /// Returns whether this request produced a materialized exact result.
    pub fn is_materialized(&self) -> bool {
        self.result.is_some()
    }

    /// Returns whether exact support was certified for this request.
    pub fn is_certified(&self) -> bool {
        self.preflight.is_certified()
    }

    /// Returns the retained blocker kind when materialization did not proceed.
    pub fn required_blocker_kind(&self) -> Option<ExactBooleanBlockerKind> {
        self.preflight.required_blocker_kind()
    }

    /// Validate the retained evaluation shape without replaying sources.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if self.preflight.operation != self.request.operation {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        self.preflight.validate()?;
        self.certifications.validate_for_request(self.request)?;
        if !exact_boolean_preflight_matches_certifications(&self.preflight, &self.certifications) {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if let Some(result) = self.result.as_ref() {
            if !self.preflight.is_certified() {
                return Err(ExactReportValidationError::StatusEvidenceMismatch);
            }
            result.validate()?;
            if !exact_boolean_result_kind_matches_request(result, self.request)
                || result.mesh.validation_policy() != self.request.validation
                || !exact_boolean_result_facts_match_preflight(result, &self.preflight)
                || !exact_boolean_result_matches_certifications(result, &self.certifications)
                || !exact_boolean_result_kind_matches_preflight_support(
                    result,
                    self.preflight.support,
                )
            {
                return Err(ExactReportValidationError::StatusEvidenceMismatch);
            }
        } else if self.preflight.is_certified() {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        Ok(())
    }

    /// Validate the retained evaluation by replaying all source-bound reports
    /// and the materialized result under the original request policy.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        self.preflight
            .validate_against_sources_with_boundary_policy(
                left,
                right,
                self.request.validation,
                self.request.boundary_policy,
            )?;
        self.certifications
            .validate_against_sources(left, right, self.request)?;
        if let Some(result) = self.result.as_ref() {
            result.validate_operation_against_sources(
                left,
                right,
                self.request.operation,
                self.request.validation,
                self.request.boundary_policy,
            )?;
        } else if self.preflight.is_certified() {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        Ok(())
    }

    /// Classify whether this retained evaluation is fresh for the source
    /// meshes under its original request policy.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactReportFreshness {
        match self.validate_against_sources(left, right) {
            Ok(()) => ExactReportFreshness::Current,
            Err(error) => error.into(),
        }
    }
}

fn exact_boolean_preflight_matches_certifications(
    preflight: &ExactBooleanPreflight,
    certifications: &ExactBooleanCertificationSet,
) -> bool {
    let status = &certifications.winding_readiness.status;
    match preflight.support {
        ExactBooleanSupport::SelectedRegionPolicy => {
            *status == ExactWindingReadinessStatus::NotNamedOperation
                && exact_boolean_preflight_matches_selected_region_policy(preflight, certifications)
        }
        ExactBooleanSupport::CertifiedBoundaryPolicyShortcut => {
            certifications.boundary_touching.is_certified()
                && *status == ExactWindingReadinessStatus::BoundaryPolicyShortcutAlreadyMaterialized
                && exact_boolean_preflight_matches_boundary_report(
                    preflight,
                    &certifications.boundary_touching,
                    false,
                )
        }
        ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
        | ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
        | ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference => {
            *status == ExactWindingReadinessStatus::OpenSurfaceArrangementAlreadyMaterialized
                && exact_boolean_preflight_matches_open_surface_arrangement(
                    preflight,
                    &certifications.winding_readiness,
                )
        }
        ExactBooleanSupport::CertifiedArrangementCellComplex => {
            exact_boolean_preflight_matches_direct_arrangement_cell_complex_shortcut(
                preflight,
                certifications,
            ) || (winding_readiness_status_materializes_arrangement_cell_complex(status)
                && exact_boolean_arrangement_attempt_materialized(
                    &certifications.arrangement_attempt,
                )
                && exact_boolean_preflight_matches_arrangement_cell_complex(
                    preflight,
                    &certifications.winding_readiness,
                ))
        }
        ExactBooleanSupport::CertifiedEmptyOperand => {
            *status == ExactWindingReadinessStatus::EmptyOperandAlreadyMaterialized
                && certifications.trivial.has_empty_operand()
        }
        ExactBooleanSupport::CertifiedBoundsDisjoint => {
            *status == ExactWindingReadinessStatus::BoundsDisjointAlreadyMaterialized
                && certifications.trivial.bounds_disjoint
        }
        ExactBooleanSupport::CertifiedIdentical => {
            *status == ExactWindingReadinessStatus::SurfaceEqualityAlreadyMaterialized
                && certifications.identical.is_certified()
                && certifications.same_surface.is_certified()
        }
        ExactBooleanSupport::CertifiedSameSurface => {
            *status == ExactWindingReadinessStatus::SurfaceEqualityAlreadyMaterialized
                && certifications.same_surface.is_certified()
        }
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
        | ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
        | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference => {
            *status == ExactWindingReadinessStatus::ClosedBoundaryTouchingAlreadyMaterialized
                && exact_boolean_preflight_matches_closed_boundary_touching(
                    preflight,
                    certifications,
                )
        }
        ExactBooleanSupport::CertifiedOpenSurfaceDisjoint => {
            *status == ExactWindingReadinessStatus::OpenSurfaceDisjointAlreadyMaterialized
                && exact_boolean_preflight_matches_open_surface_disjoint(
                    preflight,
                    &certifications.open_surface_disjoint,
                )
        }
        ExactBooleanSupport::CertifiedClosedWindingSeparated => {
            *status == ExactWindingReadinessStatus::ClosedWindingSeparatedAlreadyMaterialized
                && exact_boolean_closed_winding_reports_separated(certifications)
        }
        ExactBooleanSupport::CertifiedClosedWindingContainment => {
            *status == ExactWindingReadinessStatus::ClosedWindingContainmentAlreadyMaterialized
                && exact_boolean_closed_winding_reports_containment(certifications)
        }
        ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid => {
            *status
                == ExactWindingReadinessStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized
                && certifications
                    .regularized_solid
                    .supports_mixed_dimensional_regularized_solid()
        }
        ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid => {
            *status
                == ExactWindingReadinessStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized
                && certifications
                    .regularized_solid
                    .supports_lower_dimensional_regularized_solid()
        }
        ExactBooleanSupport::CertifiedConvexUnion
        | ExactBooleanSupport::CertifiedConvexIntersection
        | ExactBooleanSupport::CertifiedConvexDifference => {
            *status == ExactWindingReadinessStatus::ConvexBooleanAlreadyMaterialized
                && exact_boolean_convex_reports_match_support(preflight, certifications)
        }
        ExactBooleanSupport::CertifiedConvexSeparated => {
            matches!(
                status,
                ExactWindingReadinessStatus::ConvexBooleanAlreadyMaterialized
                    | ExactWindingReadinessStatus::ClosedWindingSeparatedAlreadyMaterialized
            ) && exact_boolean_convex_reports_match_support(preflight, certifications)
        }
        ExactBooleanSupport::CertifiedConvexContainment => {
            matches!(
                status,
                ExactWindingReadinessStatus::ConvexBooleanAlreadyMaterialized
                    | ExactWindingReadinessStatus::ClosedWindingContainmentAlreadyMaterialized
            ) && exact_boolean_convex_reports_match_support(preflight, certifications)
        }
        ExactBooleanSupport::RequiresBoundaryPolicy => {
            certifications.boundary_touching.is_certified()
                && *status == ExactWindingReadinessStatus::BoundaryPolicyRequired
                && exact_boolean_preflight_matches_boundary_report(
                    preflight,
                    &certifications.boundary_touching,
                    true,
                )
        }
        ExactBooleanSupport::RequiresPlanarArrangement => {
            *status == ExactWindingReadinessStatus::PlanarArrangementRequired
                && exact_boolean_preflight_matches_planar_report(
                    preflight,
                    &certifications.planar_arrangement,
                )
        }
        ExactBooleanSupport::RequiresCoplanarVolumetricCells => {
            *status == ExactWindingReadinessStatus::CoplanarVolumetricCellsRequired
                && exact_boolean_preflight_matches_winding_handoff(
                    preflight,
                    &certifications.winding_readiness,
                )
        }
        ExactBooleanSupport::UnresolvedGraph => {
            *status == ExactWindingReadinessStatus::GraphUnknowns
                && exact_boolean_preflight_matches_winding_handoff(
                    preflight,
                    &certifications.winding_readiness,
                )
        }
        ExactBooleanSupport::RequiresCertifiedWinding => {
            matches!(
                status,
                ExactWindingReadinessStatus::Ready
                    | ExactWindingReadinessStatus::NoNontrivialOverlap
                    | ExactWindingReadinessStatus::VolumetricAssemblyRequired
            ) && exact_boolean_preflight_matches_winding_handoff(
                preflight,
                &certifications.winding_readiness,
            )
        }
    }
}

fn exact_boolean_preflight_matches_open_surface_disjoint(
    preflight: &ExactBooleanPreflight,
    open_surface_disjoint: &ExactOpenSurfaceDisjointReport,
) -> bool {
    open_surface_disjoint.is_certified()
        && preflight.graph_had_unknowns == open_surface_disjoint.graph_had_unknowns
        && preflight.retained_face_pairs == open_surface_disjoint.retained_face_pairs
        && preflight.retained_events == open_surface_disjoint.retained_events
        && preflight.region_count == 0
        && preflight.region_classifications.is_empty()
        && preflight.blocker.is_none()
        && preflight.arrangement_readiness.is_none()
        && preflight.coplanar_volumetric_evidence.is_none()
}

fn exact_boolean_preflight_matches_selected_region_policy(
    preflight: &ExactBooleanPreflight,
    certifications: &ExactBooleanCertificationSet,
) -> bool {
    matches!(
        preflight.operation,
        ExactBooleanOperation::SelectedRegions(_)
    ) && preflight.graph_had_unknowns == certifications.refinement.graph_had_unknowns
        && preflight.retained_face_pairs == certifications.refinement.retained_face_pairs
        && preflight.retained_events == certifications.refinement.retained_events
        && preflight.graph_had_unknowns == certifications.winding_readiness.graph_had_unknowns
        && preflight.retained_face_pairs == certifications.winding_readiness.retained_face_pairs
        && preflight.retained_events == certifications.winding_readiness.retained_events
        && preflight.blocker.is_none()
        && preflight.arrangement_readiness.is_none()
        && preflight.coplanar_volumetric_evidence.is_none()
        && certifications.winding_readiness.region_count == 0
        && certifications
            .winding_readiness
            .region_classifications
            .is_empty()
        && certifications
            .winding_readiness
            .arrangement_readiness
            .is_none()
        && certifications
            .winding_readiness
            .coplanar_volumetric_evidence
            .is_none()
}

fn exact_boolean_closed_winding_reports_separated(
    certifications: &ExactBooleanCertificationSet,
) -> bool {
    certifications.closed_winding_left_in_right.relation == ClosedMeshWindingMeshRelation::Outside
        && certifications.closed_winding_right_in_left.relation
            == ClosedMeshWindingMeshRelation::Outside
}

fn exact_boolean_closed_winding_reports_containment(
    certifications: &ExactBooleanCertificationSet,
) -> bool {
    certifications.closed_winding_left_in_right.relation
        == ClosedMeshWindingMeshRelation::StrictlyInside
        || certifications.closed_winding_right_in_left.relation
            == ClosedMeshWindingMeshRelation::StrictlyInside
}

fn exact_boolean_convex_reports_match_support(
    preflight: &ExactBooleanPreflight,
    certifications: &ExactBooleanCertificationSet,
) -> bool {
    if !certifications
        .convex_left_in_right
        .solid_facts
        .is_certified_convex()
        || !certifications
            .convex_right_in_left
            .solid_facts
            .is_certified_convex()
    {
        return false;
    }
    match preflight.support {
        ExactBooleanSupport::CertifiedConvexUnion
        | ExactBooleanSupport::CertifiedConvexIntersection
        | ExactBooleanSupport::CertifiedConvexDifference => certifications
            .convex_capabilities
            .supports(preflight.support),
        ExactBooleanSupport::CertifiedConvexSeparated
        | ExactBooleanSupport::CertifiedConvexContainment => true,
        _ => false,
    }
}

fn exact_boolean_preflight_matches_closed_boundary_touching(
    preflight: &ExactBooleanPreflight,
    certifications: &ExactBooleanCertificationSet,
) -> bool {
    (certifications.boundary_touching.is_certified()
        && exact_boolean_preflight_matches_boundary_report(
            preflight,
            &certifications.boundary_touching,
            false,
        ))
        || exact_boolean_preflight_matches_closed_boundary_coplanar_handoff(
            preflight,
            &certifications.winding_readiness,
        )
}

fn exact_boolean_preflight_matches_closed_boundary_coplanar_handoff(
    preflight: &ExactBooleanPreflight,
    winding_readiness: &ExactWindingReadinessReport,
) -> bool {
    preflight.graph_had_unknowns == winding_readiness.graph_had_unknowns
        && preflight.retained_face_pairs == winding_readiness.retained_face_pairs
        && preflight.retained_events == winding_readiness.retained_events
        && preflight.region_count == winding_readiness.region_count
        && preflight.region_classifications == winding_readiness.region_classifications
        && preflight.blocker.is_none()
        && preflight.arrangement_readiness.is_none()
        && preflight.coplanar_volumetric_evidence.is_some()
        && preflight.coplanar_volumetric_evidence == winding_readiness.coplanar_volumetric_evidence
}

fn exact_boolean_preflight_matches_open_surface_arrangement(
    preflight: &ExactBooleanPreflight,
    winding_readiness: &ExactWindingReadinessReport,
) -> bool {
    preflight.graph_had_unknowns == winding_readiness.graph_had_unknowns
        && preflight.retained_face_pairs == winding_readiness.retained_face_pairs
        && preflight.retained_events == winding_readiness.retained_events
        && preflight.region_count == winding_readiness.region_count
        && preflight.region_classifications == winding_readiness.region_classifications
        && preflight.blocker.is_none()
        && preflight.arrangement_readiness.is_none()
        && preflight.coplanar_volumetric_evidence.is_none()
        && winding_readiness.coplanar_volumetric_evidence.is_none()
}

fn exact_boolean_arrangement_attempt_materialized(
    attempt: &Option<ExactArrangementBooleanAttempt>,
) -> bool {
    attempt.as_ref().is_some_and(|attempt| {
        attempt.stage == ExactArrangementBooleanStage::Materialized
            && attempt.decline.is_none()
            && attempt.materialized_shortcut
                == Some(ExactBooleanShortcutKind::ArrangementCellComplex)
    })
}

fn exact_boolean_preflight_matches_direct_arrangement_cell_complex_shortcut(
    preflight: &ExactBooleanPreflight,
    certifications: &ExactBooleanCertificationSet,
) -> bool {
    certifications
        .arrangement_cell_complex_shortcuts
        .supports(preflight.operation)
        && preflight.graph_had_unknowns == certifications.refinement.graph_had_unknowns
        && preflight.retained_face_pairs == certifications.refinement.retained_face_pairs
        && preflight.retained_events == certifications.refinement.retained_events
        && preflight.region_count == 0
        && preflight.region_classifications.is_empty()
        && preflight.blocker.is_none()
        && preflight.arrangement_readiness.is_none()
}

fn exact_boolean_preflight_matches_arrangement_cell_complex(
    preflight: &ExactBooleanPreflight,
    winding_readiness: &ExactWindingReadinessReport,
) -> bool {
    let region_handoff_matches = (preflight.region_count == winding_readiness.region_count
        && preflight.region_classifications == winding_readiness.region_classifications)
        || (preflight.region_count == 0 && preflight.region_classifications.is_empty());
    preflight.graph_had_unknowns == winding_readiness.graph_had_unknowns
        && preflight.retained_face_pairs == winding_readiness.retained_face_pairs
        && preflight.retained_events == winding_readiness.retained_events
        && region_handoff_matches
        && preflight.blocker.is_none()
        && preflight.arrangement_readiness == winding_readiness.arrangement_readiness
        && preflight.coplanar_volumetric_evidence == winding_readiness.coplanar_volumetric_evidence
}

fn exact_boolean_preflight_matches_boundary_report(
    preflight: &ExactBooleanPreflight,
    boundary_touching: &ExactBoundaryTouchingReport,
    requires_blocker: bool,
) -> bool {
    preflight.graph_had_unknowns == boundary_touching.graph_had_unknowns
        && preflight.retained_face_pairs == boundary_touching.retained_face_pairs
        && preflight.retained_events == boundary_touching.retained_events
        && preflight.region_count == 0
        && preflight.region_classifications.is_empty()
        && preflight.arrangement_readiness.is_none()
        && preflight.coplanar_volumetric_evidence.is_none()
        && if requires_blocker {
            preflight.blocker.as_ref() == Some(&boundary_touching.blocker)
        } else {
            preflight.blocker.is_none()
        }
}

fn exact_boolean_preflight_matches_planar_report(
    preflight: &ExactBooleanPreflight,
    planar_arrangement: &ExactPlanarArrangementReport,
) -> bool {
    preflight.graph_had_unknowns == planar_arrangement.graph_had_unknowns
        && preflight.retained_face_pairs == planar_arrangement.retained_face_pairs
        && preflight.retained_events == planar_arrangement.retained_events
        && preflight.region_count == 0
        && preflight.region_classifications.is_empty()
        && preflight.blocker.as_ref() == Some(&planar_arrangement.blocker)
        && preflight.arrangement_readiness == planar_arrangement.arrangement_readiness
        && preflight.coplanar_volumetric_evidence.is_none()
}

fn exact_boolean_preflight_matches_winding_handoff(
    preflight: &ExactBooleanPreflight,
    winding_readiness: &ExactWindingReadinessReport,
) -> bool {
    preflight.graph_had_unknowns == winding_readiness.graph_had_unknowns
        && preflight.retained_face_pairs == winding_readiness.retained_face_pairs
        && preflight.retained_events == winding_readiness.retained_events
        && preflight.region_count == winding_readiness.region_count
        && preflight.region_classifications == winding_readiness.region_classifications
        && preflight.blocker.as_ref() == Some(&winding_readiness.blocker)
        && preflight.arrangement_readiness.is_none()
        && preflight.coplanar_volumetric_evidence == winding_readiness.coplanar_volumetric_evidence
}

fn exact_boolean_result_kind_matches_request(
    result: &ExactBooleanResult,
    request: ExactBooleanRequest,
) -> bool {
    match result.kind {
        ExactBooleanResultKind::SelectedRegions { selection } => {
            request.operation == ExactBooleanOperation::SelectedRegions(selection)
        }
        ExactBooleanResultKind::CertifiedShortcut { operation, .. }
        | ExactBooleanResultKind::BoundaryPolicyShortcut { operation }
        | ExactBooleanResultKind::OpenSurfaceArrangement { operation }
        | ExactBooleanResultKind::ArrangementCellComplexMaterialized { operation } => {
            operation == request.operation
        }
    }
}

fn exact_boolean_result_kind_matches_preflight_support(
    result: &ExactBooleanResult,
    support: ExactBooleanSupport,
) -> bool {
    match support {
        ExactBooleanSupport::SelectedRegionPolicy => {
            matches!(result.kind, ExactBooleanResultKind::SelectedRegions { .. })
        }
        ExactBooleanSupport::CertifiedBoundaryPolicyShortcut => {
            matches!(
                result.kind,
                ExactBooleanResultKind::BoundaryPolicyShortcut { .. }
            )
        }
        ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
        | ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
        | ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference => {
            matches!(
                result.kind,
                ExactBooleanResultKind::OpenSurfaceArrangement { .. }
            )
        }
        ExactBooleanSupport::CertifiedArrangementCellComplex => matches!(
            result.kind,
            ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
                | ExactBooleanResultKind::CertifiedShortcut {
                    shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
                    ..
                }
        ),
        ExactBooleanSupport::CertifiedEmptyOperand => {
            exact_boolean_result_has_shortcut(result, ExactBooleanShortcutKind::EmptyOperand)
        }
        ExactBooleanSupport::CertifiedBoundsDisjoint => {
            exact_boolean_result_has_shortcut(result, ExactBooleanShortcutKind::BoundsDisjoint)
        }
        ExactBooleanSupport::CertifiedIdentical => {
            exact_boolean_result_has_shortcut(result, ExactBooleanShortcutKind::Identical)
        }
        ExactBooleanSupport::CertifiedSameSurface => {
            exact_boolean_result_has_shortcut(result, ExactBooleanShortcutKind::SameSurface)
        }
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion => {
            exact_boolean_result_has_shortcut(
                result,
                ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion,
            )
        }
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection => {
            exact_boolean_result_has_shortcut(
                result,
                ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection,
            )
        }
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference => {
            exact_boolean_result_has_shortcut(
                result,
                ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
            )
        }
        ExactBooleanSupport::CertifiedOpenSurfaceDisjoint => {
            exact_boolean_result_has_shortcut(result, ExactBooleanShortcutKind::OpenSurfaceDisjoint)
        }
        ExactBooleanSupport::CertifiedClosedWindingSeparated => exact_boolean_result_has_shortcut(
            result,
            ExactBooleanShortcutKind::ClosedWindingSeparated,
        ),
        ExactBooleanSupport::CertifiedClosedWindingContainment => {
            exact_boolean_result_has_shortcut(
                result,
                ExactBooleanShortcutKind::ClosedWindingContainment,
            )
        }
        ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid => {
            exact_boolean_result_has_shortcut(
                result,
                ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid,
            )
        }
        ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid => {
            exact_boolean_result_has_shortcut(
                result,
                ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid,
            )
        }
        ExactBooleanSupport::CertifiedConvexContainment => {
            exact_boolean_result_has_shortcut(result, ExactBooleanShortcutKind::ConvexContainment)
        }
        ExactBooleanSupport::CertifiedConvexUnion => {
            exact_boolean_result_has_shortcut(result, ExactBooleanShortcutKind::ConvexUnion)
        }
        ExactBooleanSupport::CertifiedConvexIntersection => {
            exact_boolean_result_has_shortcut(result, ExactBooleanShortcutKind::ConvexIntersection)
        }
        ExactBooleanSupport::CertifiedConvexDifference => {
            exact_boolean_result_has_shortcut(result, ExactBooleanShortcutKind::ConvexDifference)
        }
        ExactBooleanSupport::CertifiedConvexSeparated => {
            exact_boolean_result_has_shortcut(result, ExactBooleanShortcutKind::ConvexSeparated)
        }
        ExactBooleanSupport::RequiresBoundaryPolicy
        | ExactBooleanSupport::RequiresPlanarArrangement
        | ExactBooleanSupport::RequiresCoplanarVolumetricCells
        | ExactBooleanSupport::RequiresCertifiedWinding
        | ExactBooleanSupport::UnresolvedGraph => false,
    }
}

fn exact_boolean_result_facts_match_preflight(
    result: &ExactBooleanResult,
    preflight: &ExactBooleanPreflight,
) -> bool {
    match result.kind {
        ExactBooleanResultKind::SelectedRegions { .. }
        | ExactBooleanResultKind::OpenSurfaceArrangement { .. } => {
            result.graph_had_unknowns == preflight.graph_had_unknowns
                && result.region_classifications == preflight.region_classifications
        }
        ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
        | ExactBooleanResultKind::BoundaryPolicyShortcut { .. }
        | ExactBooleanResultKind::CertifiedShortcut { .. } => true,
    }
}

fn exact_boolean_result_matches_certifications(
    result: &ExactBooleanResult,
    certifications: &ExactBooleanCertificationSet,
) -> bool {
    match result.kind {
        ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. } => {
            exact_boolean_arrangement_attempt_materialized(&certifications.arrangement_attempt)
        }
        ExactBooleanResultKind::CertifiedShortcut {
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            operation,
        } => {
            certifications
                .arrangement_cell_complex_shortcuts
                .supports(operation)
                || exact_boolean_arrangement_attempt_materialized(
                    &certifications.arrangement_attempt,
                )
        }
        _ => true,
    }
}

fn exact_boolean_result_has_shortcut(
    result: &ExactBooleanResult,
    expected: ExactBooleanShortcutKind,
) -> bool {
    matches!(
        result.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            shortcut,
            ..
        } if shortcut == expected
    )
}

/// Evaluate an exact boolean request into either a certified result or retained
/// exact blockers/provenance.
fn evaluate_boolean_exact_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactBooleanEvaluation, MeshError> {
    let preflight = request.preflight(left, right)?;
    let certifications = ExactBooleanCertificationSet::from_sources(left, right, request)?;
    let result = if preflight.is_certified() {
        Some(materialize_certified_boolean_support(
            left,
            right,
            request,
            preflight.support,
        )?)
    } else {
        None
    };
    let evaluation = ExactBooleanEvaluation {
        request,
        preflight,
        certifications,
        result,
    };
    evaluation.validate().map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact boolean evaluation local validation failed: {error:?}"),
        ))
    })?;
    evaluation
        .validate_against_sources(left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("exact boolean evaluation source replay failed: {error:?}"),
            ))
        })?;
    Ok(evaluation)
}

fn certified_boolean_support_did_not_materialize_error(support: ExactBooleanSupport) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        format!("certified exact boolean support did not materialize: {support:?}"),
    ))
}

fn materialize_certified_boolean_support(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    support: ExactBooleanSupport,
) -> Result<ExactBooleanResult, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let result = match support {
        ExactBooleanSupport::SelectedRegionPolicy => {
            let ExactBooleanOperation::SelectedRegions(selection) = operation else {
                return Err(certified_boolean_support_did_not_materialize_error(support));
            };
            Some(materialize_selected_region_boolean(
                left, right, selection, validation,
            )?)
        }
        ExactBooleanSupport::CertifiedBoundaryPolicyShortcut => {
            ExactBooleanRequest::with_boundary_policy(operation, validation, request.boundary_policy,).materialize_boundary_touching_policy(left, right)?
        }
        ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
        | ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
        | ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference => {
            materialize_open_surface_arrangement(left, right, operation, validation)?
        }
        ExactBooleanSupport::CertifiedArrangementCellComplex => {
            materialize_certified_arrangement_cell_complex_support(
                left, right, operation, validation,
            )?
        }
        ExactBooleanSupport::CertifiedEmptyOperand => {
            Some(boolean_empty_operand(left, right, operation, validation)?)
        }
        ExactBooleanSupport::CertifiedBoundsDisjoint => {
            Some(boolean_disjoint_meshes(left, right, operation, validation)?)
        }
        ExactBooleanSupport::CertifiedIdentical => {
            Some(boolean_identical_meshes(left, operation, validation)?)
        }
        ExactBooleanSupport::CertifiedSameSurface => {
            Some(boolean_same_surface_meshes(left, operation, validation)?)
        }
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
        | ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
        | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference => {
            let result = boolean_closed_boundary_touching_regularized_meshes(
                left, right, operation, validation,
            )?;
            if result.is_some() {
                result
            } else {
                boolean_closed_no_volume_overlap_regularized_meshes(
                    left, right, operation, validation,
                )?
            }
        }
        ExactBooleanSupport::CertifiedOpenSurfaceDisjoint => {
            let graph = build_intersection_graph(left, right)?;
            validate_graph_source_handoff(&graph, left, right)?;
            boolean_open_surface_disjoint_meshes_from_graph(
                &graph, left, right, operation, validation,
            )?
        }
        ExactBooleanSupport::CertifiedClosedWindingSeparated => {
            boolean_closed_winding_separated_meshes(left, right, operation, validation)?
        }
        ExactBooleanSupport::CertifiedClosedWindingContainment => {
            boolean_closed_winding_containment_meshes(left, right, operation, validation)?
        }
        ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid
        | ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid => {
            boolean_closed_regularized_lower_dimensional_optional(
                left, right, operation, validation,
            )?
        }
        ExactBooleanSupport::CertifiedConvexUnion
        | ExactBooleanSupport::CertifiedConvexIntersection
        | ExactBooleanSupport::CertifiedConvexDifference => {
            boolean_convex_meshes_optional(left, right, operation, validation)?
        }
        ExactBooleanSupport::CertifiedConvexSeparated
        | ExactBooleanSupport::CertifiedConvexContainment => {
            boolean_convex_relation_meshes_optional(left, right, operation, validation)?
        }
        ExactBooleanSupport::RequiresBoundaryPolicy
        | ExactBooleanSupport::RequiresPlanarArrangement
        | ExactBooleanSupport::RequiresCoplanarVolumetricCells
        | ExactBooleanSupport::RequiresCertifiedWinding
        | ExactBooleanSupport::UnresolvedGraph => None,
    };
    result.ok_or_else(|| certified_boolean_support_did_not_materialize_error(support))
}

fn materialize_certified_arrangement_cell_complex_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if let Some(result) =
        boolean_arrangement_orthogonal_solid_cell_recovery(left, right, operation, validation)?
    {
        return Ok(Some(result));
    }
    if let Some(result) =
        boolean_arrangement_affine_orthogonal_solid_recovery(left, right, operation, validation)?
    {
        return Ok(Some(result));
    }
    if let Some(result) =
        boolean_arrangement_volumetric_split_cell_recovery(left, right, operation, validation)?
    {
        return Ok(Some(result));
    }
    if let Some((result, _closure)) = materialize_volumetric_coplanar_boundary_closure_boolean(
        left, right, operation, validation,
    )? {
        return Ok(Some(result));
    }
    boolean_arrangement_cell_complex_meshes(left, right, operation, validation)
}

fn materialize_selected_region_boolean(
    left: &ExactMesh,
    right: &ExactMesh,
    selection: ExactRegionSelection,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    materialize_selected_region_result_from_graph(&graph, left, right, selection, validation)
}

fn materialize_selected_region_result_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    selection: ExactRegionSelection,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let graph_had_unknowns = graph.has_unknowns();
    if graph_had_unknowns {
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
    let mut assembly = ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
        &triangulations,
        selection,
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
    assembly
        .canonicalize_for_mesh_with_sources(left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexOutOfBounds,
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

pub(crate) fn replay_selected_region_boolean_result(
    left: &ExactMesh,
    right: &ExactMesh,
    selection: ExactRegionSelection,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    materialize_selected_region_result_from_graph(&graph, left, right, selection, validation)
}

/// Preflight an exact boolean operation without materializing output topology.
///
/// The preflight path deliberately shares the exact graph, region, and
/// classification stages with the executable arrangement pipeline. For named
/// booleans that still need unresolved inside/outside semantics, it returns
/// [`ExactBooleanSupport::RequiresCertifiedWinding`] with replayable facts
/// instead of approximating them.
fn preflight_boolean_exact_reject_boundary_policy(
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
        return Ok(certified_arrangement_cell_complex_preflight_from_graph(
            operation, &graph, left, right,
        ));
    }

    if matches!(
        support,
        ExactBooleanSupport::CertifiedEmptyOperand
            | ExactBooleanSupport::CertifiedBoundsDisjoint
            | ExactBooleanSupport::CertifiedIdentical
            | ExactBooleanSupport::CertifiedSameSurface
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
            | ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
            | ExactBooleanSupport::CertifiedClosedWindingSeparated
            | ExactBooleanSupport::CertifiedClosedWindingContainment
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
    let relation_counts = retained_graph_counts(&graph);
    let mut certified_arrangement_preflight = None;
    if graph_had_unknowns || relation_counts.construction_failed_events > 0 {
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
        && let Some(convex_support) =
            certified_convex_materialized_boolean_support(left, right, operation)
    {
        return Ok(certified_shortcut_preflight_from_graph(
            operation,
            convex_support,
            &graph,
        ));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(convex_support) =
            certified_convex_boolean_support_from_graph(&graph, left, right, operation)?
    {
        return Ok(certified_shortcut_preflight_from_graph(
            operation,
            convex_support,
            &graph,
        ));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(separated_support) =
            certified_closed_winding_separated_support_from_graph(&graph, left, right, operation)?
    {
        return Ok(certified_shortcut_preflight(operation, separated_support));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(containment_support) =
            certified_closed_winding_containment_support_from_graph(&graph, left, right, operation)?
    {
        return Ok(certified_shortcut_preflight(operation, containment_support));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(boundary_support) =
            certified_closed_zero_area_boundary_contact_support_from_graph(
                &graph, left, right, operation,
            )?
    {
        return Ok(certified_shortcut_preflight_from_graph(
            operation,
            boundary_support,
            &graph,
        ));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && matches!(
            operation,
            ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
        )
        && let Some(boundary_support) = certified_closed_boundary_only_contact_support_from_graph(
            &graph, left, right, operation,
        )?
    {
        let mut preflight =
            certified_shortcut_preflight_from_graph(operation, boundary_support, &graph);
        preflight.coplanar_volumetric_evidence =
            coplanar_boundary_only_evidence_if_consumed(&graph, left, right)?;
        return Ok(preflight);
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
        return Ok(certified_arrangement_cell_complex_preflight_from_graph(
            operation, &graph, left, right,
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
        && operation == ExactBooleanOperation::Union
        && let Some(boundary_support) = certified_closed_boundary_only_contact_support_from_graph(
            &graph, left, right, operation,
        )?
    {
        if operation == ExactBooleanOperation::Union {
            let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(&graph, left, right);
            evidence.validate().map_err(|error| {
                MeshError::one(MeshDiagnostic::new(
                    Severity::Error,
                    DiagnosticKind::UnsupportedExactOperation,
                    format!("exact no-volume-overlap union evidence validation failed: {error:?}"),
                ))
            })?;
            if evidence.positive_area_coplanar_overlapping_pairs != 0 {
                let mut preflight = certified_arrangement_cell_complex_preflight_from_graph(
                    operation, &graph, left, right,
                );
                preflight.coplanar_volumetric_evidence = Some(evidence);
                return Ok(preflight);
            }
        }
        return Ok(certified_shortcut_preflight_from_graph(
            operation,
            boundary_support,
            &graph,
        ));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && operation == ExactBooleanOperation::Union
        && has_non_empty_axis_aligned_orthogonal_solid_cell_intersection(left, right)
        && !graph_requires_coplanar_volumetric_cells_for_sources(&graph, left, right)
        && let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation)
        && has_axis_aligned_orthogonal_solid_cells(left, right, solid_operation)
    {
        return Ok(certified_arrangement_cell_complex_preflight_from_graph(
            operation, &graph, left, right,
        ));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && let Some(boundary_support) =
            certified_closed_boundary_touching_support_from_graph(&graph, left, right, operation)?
    {
        return Ok(certified_shortcut_preflight_from_graph(
            operation,
            boundary_support,
            &graph,
        ));
    }
    if support == ExactBooleanSupport::RequiresCertifiedWinding
        && operation == ExactBooleanOperation::Intersection
        && has_empty_axis_aligned_orthogonal_solid_intersection(left, right)?
    {
        return Ok(certified_arrangement_cell_complex_preflight_from_graph(
            operation, &graph, left, right,
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
    if let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation)
        && has_axis_aligned_orthogonal_solid_cells(left, right, solid_operation)
    {
        return Ok(certified_arrangement_cell_complex_preflight_from_graph(
            operation, &graph, left, right,
        ));
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
    if winding_readiness_status_materializes_arrangement_cell_complex(&winding_report.status)
        || (winding_report.status == ExactWindingReadinessStatus::Ready
            && materialize_volumetric_winding_region_plan_from_graph(
                &graph,
                left,
                right,
                operation,
                ValidationPolicy::CLOSED,
            )?
            .is_some())
        || materialize_closed_volumetric_winding_boundary_caps_from_graph(
            &graph, left, right, operation,
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

/// Preflight an exact boolean operation for a specific output validation policy.
///
/// [`ExactBooleanRequest::preflight`] preserves the strict closed-output boundary for
/// named solid booleans. This policy-aware variant keeps that default contract
/// for `CLOSED`, but can also certify exact arrangement/cell-complex support
/// when the same retained split-cell facts materialize under a less restrictive
/// output policy such as [`ValidationPolicy::ALLOW_BOUNDARY`].
fn preflight_boolean_exact_with_validation_reject_boundary_policy(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanPreflight, MeshError> {
    if validation == ValidationPolicy::CLOSED
        && !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && let Some(support) = certified_closed_validation_regularized_solid_support(left, right)
    {
        return Ok(certified_shortcut_preflight(operation, support));
    }
    let preflight = preflight_boolean_exact_reject_boundary_policy(left, right, operation)?;
    if validation == ValidationPolicy::CLOSED
        || matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || !matches!(
            preflight.support,
            ExactBooleanSupport::RequiresCertifiedWinding
                | ExactBooleanSupport::RequiresCoplanarVolumetricCells
        )
    {
        return Ok(preflight);
    }

    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    if boolean_arrangement_volumetric_split_cell_recovery_from_graph(
        &graph, left, right, operation, validation,
    )?
    .is_some()
    {
        return Ok(certified_arrangement_cell_complex_preflight_from_graph(
            operation, &graph, left, right,
        ));
    }
    Ok(preflight)
}

/// Preflight an exact boolean request for explicit output validation and
/// boundary-only projection policies.
///
/// A rejecting boundary policy keeps lower-dimensional boundary contact as
/// [`ExactBooleanSupport::RequiresBoundaryPolicy`]. The default request policy
/// proves when the chosen boundary projection is sufficient for the current
/// graph and validation contract.
fn preflight_boolean_exact_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactBooleanPreflight, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let boundary_policy = request.boundary_policy;
    let preflight = preflight_boolean_exact_with_validation_reject_boundary_policy(
        left, right, operation, validation,
    )?;
    if boundary_policy == ExactBoundaryBooleanPolicy::Reject
        || matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || preflight.support != ExactBooleanSupport::RequiresBoundaryPolicy
    {
        return Ok(preflight);
    }

    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    if boolean_boundary_touching_meshes_from_graph(
        &graph,
        left,
        right,
        operation,
        validation,
        boundary_policy,
    )?
    .is_some()
    {
        return Ok(certified_boundary_policy_preflight_from_graph(
            operation, &graph,
        ));
    }
    Ok(preflight)
}

/// Certify why retained volumetric boundary output can or cannot become closed.
///
/// This report is intentionally narrower than [`ExactBooleanRequest::materialize`]: it asks whether
/// the exact split-cell materializer can produce boundary output under
/// [`ValidationPolicy::ALLOW_BOUNDARY`], then audits the remaining boundary
/// loops for the existing coplanar cap generator. Non-coplanar loops remain an
/// explicit topology-construction obligation rather than a silent closure.
fn volumetric_boundary_closure_report_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactVolumetricBoundaryClosureReport, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    volumetric_boundary_closure_report_from_graph(&graph, left, right, request.operation)
}

fn volumetric_boundary_closure_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactVolumetricBoundaryClosureReport, MeshError> {
    let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        graph,
        left,
        right,
        operation,
        ValidationPolicy::ALLOW_BOUNDARY,
    )?
    else {
        return Ok(ExactVolumetricBoundaryClosureReport {
            operation,
            status: ExactVolumetricBoundaryClosureStatus::NoMaterializedBoundaryOutput,
            output_triangles: 0,
            boundary_edges: 0,
            boundary_loops: 0,
            boundary_vertices_with_invalid_outgoing_degree: 0,
            boundary_vertices_with_invalid_incoming_degree: 0,
            overused_boundary_edges: 0,
            noncoplanar_boundary_loops: 0,
            repeated_exact_boundary_points: 0,
            self_contact_exact_points: 0,
            self_contact_topological_vertices: 0,
            self_contact_degenerate_cycles: 0,
            self_contact_nondegenerate_cycles: 0,
            coplanar_loop_groups: 0,
        });
    };
    volumetric_boundary_closure_report_from_materialized(&materialized, operation)
}

fn volumetric_boundary_closure_report_from_materialized(
    materialized: &MaterializedVolumetricWindingRegionPlan,
    operation: ExactBooleanOperation,
) -> Result<ExactVolumetricBoundaryClosureReport, MeshError> {
    materialized
        .mesh
        .validate_retained_state()
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("volumetric boundary closure source mesh validation failed: {error:?}"),
            ))
        })?;
    let output_triangles = materialized.mesh.triangles().len();
    let boundary_edges = materialized.mesh.facts().mesh.boundary_edges;
    if materialized.mesh.facts().mesh.closed_manifold || boundary_edges == 0 {
        return Ok(ExactVolumetricBoundaryClosureReport {
            operation,
            status: ExactVolumetricBoundaryClosureStatus::AlreadyClosed,
            output_triangles,
            boundary_edges: 0,
            boundary_loops: 0,
            boundary_vertices_with_invalid_outgoing_degree: 0,
            boundary_vertices_with_invalid_incoming_degree: 0,
            overused_boundary_edges: 0,
            noncoplanar_boundary_loops: 0,
            repeated_exact_boundary_points: 0,
            self_contact_exact_points: 0,
            self_contact_topological_vertices: 0,
            self_contact_degenerate_cycles: 0,
            self_contact_nondegenerate_cycles: 0,
            coplanar_loop_groups: 0,
        });
    }
    let boundary_topology = boundary_topology_evidence(&materialized.mesh);
    let Some(boundary_loops) = directed_boundary_loops(&materialized.mesh) else {
        return Ok(ExactVolumetricBoundaryClosureReport {
            operation,
            status: ExactVolumetricBoundaryClosureStatus::BoundaryTopologyNotLoop,
            output_triangles,
            boundary_edges,
            boundary_loops: 0,
            boundary_vertices_with_invalid_outgoing_degree: boundary_topology
                .invalid_outgoing_degree_vertices,
            boundary_vertices_with_invalid_incoming_degree: boundary_topology
                .invalid_incoming_degree_vertices,
            overused_boundary_edges: boundary_topology.overused_edges,
            noncoplanar_boundary_loops: 0,
            repeated_exact_boundary_points: 0,
            self_contact_exact_points: 0,
            self_contact_topological_vertices: 0,
            self_contact_degenerate_cycles: 0,
            self_contact_nondegenerate_cycles: 0,
            coplanar_loop_groups: 0,
        });
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
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexOutOfBounds,
                "volumetric boundary closure report referenced a missing output vertex",
            ))
        })?;
    let boundary_points = boundary_points
        .into_iter()
        .map(split_boundary_self_contact_cycles)
        .collect::<Result<Vec<_>, _>>()
        .map(|split| split.into_iter().flatten().collect::<Vec<_>>())
        .map_err(|blocker| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
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
                return Ok(ExactVolumetricBoundaryClosureReport {
                    operation,
                    status: ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(blocker),
                    output_triangles,
                    boundary_edges,
                    boundary_loops: boundary_loops.len(),
                    boundary_vertices_with_invalid_outgoing_degree: 0,
                    boundary_vertices_with_invalid_incoming_degree: 0,
                    overused_boundary_edges: 0,
                    noncoplanar_boundary_loops: 0,
                    repeated_exact_boundary_points: self_contact.repeated_exact_point_pairs,
                    self_contact_exact_points: self_contact.exact_points,
                    self_contact_topological_vertices: self_contact.topological_vertices,
                    self_contact_degenerate_cycles: self_contact.degenerate_cycles,
                    self_contact_nondegenerate_cycles: self_contact.nondegenerate_cycles,
                    coplanar_loop_groups: 0,
                });
            }
        }
    }
    if self_contact.repeated_exact_point_pairs != 0 {
        return Ok(ExactVolumetricBoundaryClosureReport {
            operation,
            status: ExactVolumetricBoundaryClosureStatus::BoundaryLoopExactSelfContact,
            output_triangles,
            boundary_edges,
            boundary_loops: boundary_loops.len(),
            boundary_vertices_with_invalid_outgoing_degree: 0,
            boundary_vertices_with_invalid_incoming_degree: 0,
            overused_boundary_edges: 0,
            noncoplanar_boundary_loops: 0,
            repeated_exact_boundary_points: self_contact.repeated_exact_point_pairs,
            self_contact_exact_points: self_contact.exact_points,
            self_contact_topological_vertices: self_contact.topological_vertices,
            self_contact_degenerate_cycles: self_contact.degenerate_cycles,
            self_contact_nondegenerate_cycles: self_contact.nondegenerate_cycles,
            coplanar_loop_groups: 0,
        });
    }
    let repeated_exact_boundary_points = self_contact.repeated_exact_point_pairs;
    let mut noncoplanar_boundary_loops = 0;
    for boundary in &boundary_points {
        match exact_loop_is_coplanar(boundary) {
            Ok(true) => {}
            Ok(false) => noncoplanar_boundary_loops += 1,
            Err(blocker) => {
                return Ok(ExactVolumetricBoundaryClosureReport {
                    operation,
                    status: ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(blocker),
                    output_triangles,
                    boundary_edges,
                    boundary_loops: boundary_loops.len(),
                    boundary_vertices_with_invalid_outgoing_degree: 0,
                    boundary_vertices_with_invalid_incoming_degree: 0,
                    overused_boundary_edges: 0,
                    noncoplanar_boundary_loops,
                    repeated_exact_boundary_points,
                    self_contact_exact_points: self_contact.exact_points,
                    self_contact_topological_vertices: self_contact.topological_vertices,
                    self_contact_degenerate_cycles: self_contact.degenerate_cycles,
                    self_contact_nondegenerate_cycles: self_contact.nondegenerate_cycles,
                    coplanar_loop_groups: 0,
                });
            }
        }
    }
    if noncoplanar_boundary_loops != 0 {
        return Ok(ExactVolumetricBoundaryClosureReport {
            operation,
            status: ExactVolumetricBoundaryClosureStatus::NonCoplanarBoundaryClosureRequired,
            output_triangles,
            boundary_edges,
            boundary_loops: boundary_loops.len(),
            boundary_vertices_with_invalid_outgoing_degree: 0,
            boundary_vertices_with_invalid_incoming_degree: 0,
            overused_boundary_edges: 0,
            noncoplanar_boundary_loops,
            repeated_exact_boundary_points,
            self_contact_exact_points: self_contact.exact_points,
            self_contact_topological_vertices: self_contact.topological_vertices,
            self_contact_degenerate_cycles: self_contact.degenerate_cycles,
            self_contact_nondegenerate_cycles: self_contact.nondegenerate_cycles,
            coplanar_loop_groups: 0,
        });
    }
    match group_exact_coplanar_loops(boundary_points) {
        Ok(groups) => {
            let coplanar_loop_groups = groups.len();
            if close_exact_coplanar_boundary_loops_from_loops(
                &materialized.mesh,
                boundary_loops.clone(),
                "exact volumetric boundary closure certification cap",
                ValidationPolicy::CLOSED,
            )
            .is_some()
            {
                Ok(ExactVolumetricBoundaryClosureReport {
                    operation,
                    status: ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
                    output_triangles,
                    boundary_edges,
                    boundary_loops: boundary_loops.len(),
                    boundary_vertices_with_invalid_outgoing_degree: 0,
                    boundary_vertices_with_invalid_incoming_degree: 0,
                    overused_boundary_edges: 0,
                    noncoplanar_boundary_loops: 0,
                    repeated_exact_boundary_points: 0,
                    self_contact_exact_points: 0,
                    self_contact_topological_vertices: 0,
                    self_contact_degenerate_cycles: 0,
                    self_contact_nondegenerate_cycles: 0,
                    coplanar_loop_groups,
                })
            } else {
                Ok(ExactVolumetricBoundaryClosureReport {
                    operation,
                    status: ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(
                        ExactArrangementBlocker::NonManifoldCellComplex,
                    ),
                    output_triangles,
                    boundary_edges,
                    boundary_loops: boundary_loops.len(),
                    boundary_vertices_with_invalid_outgoing_degree: 0,
                    boundary_vertices_with_invalid_incoming_degree: 0,
                    overused_boundary_edges: 0,
                    noncoplanar_boundary_loops: 0,
                    repeated_exact_boundary_points: 0,
                    self_contact_exact_points: 0,
                    self_contact_topological_vertices: 0,
                    self_contact_degenerate_cycles: 0,
                    self_contact_nondegenerate_cycles: 0,
                    coplanar_loop_groups,
                })
            }
        }
        Err(blocker) => Ok(ExactVolumetricBoundaryClosureReport {
            operation,
            status: ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(blocker),
            output_triangles,
            boundary_edges,
            boundary_loops: boundary_loops.len(),
            boundary_vertices_with_invalid_outgoing_degree: 0,
            boundary_vertices_with_invalid_incoming_degree: 0,
            overused_boundary_edges: 0,
            noncoplanar_boundary_loops: 0,
            repeated_exact_boundary_points: 0,
            self_contact_exact_points: 0,
            self_contact_topological_vertices: 0,
            self_contact_degenerate_cycles: 0,
            self_contact_nondegenerate_cycles: 0,
            coplanar_loop_groups: 0,
        }),
    }
}

#[cfg(test)]
const fn winding_readiness_status_already_materialized(
    status: &ExactWindingReadinessStatus,
) -> bool {
    matches!(
        status,
        ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized
            | ExactWindingReadinessStatus::CoplanarVolumetricCellsAlreadyMaterialized
            | ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized
            | ExactWindingReadinessStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized
            | ExactWindingReadinessStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized
            | ExactWindingReadinessStatus::ConvexBooleanAlreadyMaterialized
            | ExactWindingReadinessStatus::OpenSurfaceArrangementAlreadyMaterialized
            | ExactWindingReadinessStatus::SurfaceEqualityAlreadyMaterialized
            | ExactWindingReadinessStatus::ClosedBoundaryTouchingAlreadyMaterialized
            | ExactWindingReadinessStatus::BoundaryPolicyShortcutAlreadyMaterialized
            | ExactWindingReadinessStatus::EmptyOperandAlreadyMaterialized
            | ExactWindingReadinessStatus::BoundsDisjointAlreadyMaterialized
            | ExactWindingReadinessStatus::OpenSurfaceDisjointAlreadyMaterialized
            | ExactWindingReadinessStatus::ClosedWindingSeparatedAlreadyMaterialized
            | ExactWindingReadinessStatus::ClosedWindingContainmentAlreadyMaterialized
    )
}

const fn winding_readiness_status_materializes_arrangement_cell_complex(
    status: &ExactWindingReadinessStatus,
) -> bool {
    matches!(
        status,
        ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized
            | ExactWindingReadinessStatus::CoplanarVolumetricCellsAlreadyMaterialized
            | ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized
    )
}

fn preflight_tail_shortcut_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    if let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation)
        && has_axis_aligned_orthogonal_solid_cells(left, right, solid_operation)
    {
        return Some(ExactBooleanSupport::CertifiedArrangementCellComplex);
    }
    match operation {
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

fn certified_shortcut_preflight_from_graph(
    operation: ExactBooleanOperation,
    support: ExactBooleanSupport,
    graph: &super::graph::ExactIntersectionGraph,
) -> ExactBooleanPreflight {
    ExactBooleanPreflight {
        operation,
        support,
        graph_had_unknowns: graph.has_unknowns(),
        retained_face_pairs: graph.face_pairs.len(),
        retained_events: graph.event_count(),
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
        coplanar_volumetric_evidence: coplanar_volumetric_evidence_for_certified_arrangement(
            graph, left, right,
        ),
    }
}

fn certified_boundary_policy_preflight_from_graph(
    operation: ExactBooleanOperation,
    graph: &super::graph::ExactIntersectionGraph,
) -> ExactBooleanPreflight {
    ExactBooleanPreflight {
        operation,
        support: ExactBooleanSupport::CertifiedBoundaryPolicyShortcut,
        graph_had_unknowns: graph.has_unknowns(),
        retained_face_pairs: graph.face_pairs.len(),
        retained_events: graph.event_count(),
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: None,
        arrangement_readiness: None,
        coplanar_volumetric_evidence: None,
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
    let counts = retained_graph_counts(graph);
    if counts.coplanar_overlapping_pairs == 0
        && (mesh_is_open_surface(left) || mesh_is_open_surface(right))
    {
        return Ok(true);
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

fn graph_requires_coplanar_volumetric_cells(counts: &ExactBooleanBlocker) -> bool {
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
    let counts = retained_graph_counts(graph);
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
    let counts = retained_graph_counts(graph);
    if !graph_requires_coplanar_volumetric_cells(&counts) {
        return None;
    }
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    evidence
        .obstacle
        .requires_coplanar_volumetric_cells()
        .then_some(evidence)
}

fn coplanar_volumetric_evidence_for_certified_arrangement(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarVolumetricCellEvidenceReport> {
    let counts = retained_graph_counts(graph);
    if !graph_requires_coplanar_volumetric_cells(&counts) {
        return None;
    }
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    (evidence.obstacle.requires_coplanar_volumetric_cells()
        || (evidence.obstacle == CoplanarVolumetricCellObstacle::BoundaryOnlyContact
            && evidence.positive_area_coplanar_overlapping_pairs != 0))
        .then_some(evidence)
}

fn coplanar_boundary_only_evidence_if_consumed(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<CoplanarVolumetricCellEvidenceReport>, MeshError> {
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    evidence.validate().map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact boundary-only coplanar evidence validation failed: {error:?}"),
        ))
    })?;
    Ok(
        (evidence.obstacle == CoplanarVolumetricCellObstacle::BoundaryOnlyContact
            && evidence.positive_area_coplanar_overlapping_pairs != 0)
            .then_some(evidence),
    )
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
            axis_aligned_orthogonal_solid_operation(operation).is_some_and(|operation| {
                has_axis_aligned_orthogonal_solid_cells(left, right, operation)
            }) || has_affine_orthogonal_solid_cells(
                left,
                right,
                AffineOrthogonalSolidOperation::Union,
            )
        }
        ExactBooleanOperation::Intersection => {
            axis_aligned_orthogonal_solid_operation(operation).is_some_and(|operation| {
                has_axis_aligned_orthogonal_solid_cells(left, right, operation)
            }) || has_affine_orthogonal_solid_cells(
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

fn certified_closed_winding_separated_support_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ExactBooleanSupport>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let Some((left_in_right, right_in_left)) =
        closed_winding_vertex_relations_from_empty_graph(graph, left, right)?
    else {
        return Ok(None);
    };
    if left_in_right == ClosedMeshWindingMeshRelation::Outside
        && right_in_left == ClosedMeshWindingMeshRelation::Outside
    {
        Ok(Some(ExactBooleanSupport::CertifiedClosedWindingSeparated))
    } else {
        Ok(None)
    }
}

fn closed_winding_vertex_relations_from_empty_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<(ClosedMeshWindingMeshRelation, ClosedMeshWindingMeshRelation)>, MeshError> {
    if !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
        || graph.has_unknowns()
        || !graph.face_pairs.is_empty()
    {
        return Ok(None);
    }
    let counts = retained_graph_counts(graph);
    if counts.construction_failed_events != 0 {
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
) -> Result<Option<ClosedWindingContainmentRelation>, MeshError> {
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

fn certified_closed_winding_containment_support_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ExactBooleanSupport>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    Ok(
        certified_closed_winding_containment_relation_from_graph(graph, left, right)?
            .map(|_| ExactBooleanSupport::CertifiedClosedWindingContainment),
    )
}

fn boolean_closed_winding_containment_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || left.triangles().is_empty()
        || right.triangles().is_empty()
        || meshes_are_certified_bounds_disjoint(left, right)
    {
        return Ok(None);
    }
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    let Some(containment) =
        certified_closed_winding_containment_relation_from_graph(&graph, left, right)?
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
        (ExactBooleanOperation::SelectedRegions(_), _) => unreachable!("handled by caller"),
    };
    Ok(Some(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ClosedWindingContainment,
    )))
}

/// Certify and materialize a named boolean for closed solids with empty graph
/// containment proven by exact winding reports.
///
/// This path requires no retained face intersections and replays vertex
/// winding classifications to prove one closed operand lies strictly inside
/// the other. Unsupported contacts or non-containment relations return `None`
/// rather than falling back to tolerance geometry.
fn materialize_closed_winding_containment_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    Ok(public_operation_replayable_result(
        boolean_closed_winding_containment_meshes(left, right, operation, validation)?,
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    ))
}

fn boolean_closed_winding_separated_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || left.triangles().is_empty()
        || right.triangles().is_empty()
        || meshes_are_certified_bounds_disjoint(left, right)
    {
        return Ok(None);
    }
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    if certified_closed_winding_separated_support_from_graph(&graph, left, right, operation)?
        .is_none()
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
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by support check"),
    };
    Ok(Some(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ClosedWindingSeparated,
    )))
}

/// Certify and materialize a named boolean for closed solids with empty graph
/// separation proven by exact winding reports.
///
/// The retained empty intersection graph and bidirectional closed-mesh winding
/// classifications must prove both operands are outside the other. Unsupported
/// contacts or containment relations return `None` rather than falling back to
/// tolerance geometry.
fn materialize_closed_winding_separated_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    Ok(public_operation_replayable_result(
        boolean_closed_winding_separated_meshes(left, right, operation, validation)?,
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    ))
}

fn public_operation_replayable_result(
    result: Option<ExactBooleanResult>,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Option<ExactBooleanResult> {
    let result = result?;
    exact_boolean_result_matches_public_operation_replay(
        &result,
        left,
        right,
        operation,
        validation,
        boundary_policy,
    )
    .then_some(result)
}

fn exact_boolean_result_matches_public_operation_replay(
    result: &ExactBooleanResult,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> bool {
    result
        .validate_operation_against_sources(left, right, operation, validation, boundary_policy)
        .is_ok()
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
fn materialize_boolean_exact_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactBooleanResult, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let boundary_policy = request.boundary_policy;
    if let ExactBooleanOperation::SelectedRegions(selection) = operation {
        return materialize_selected_region_boolean(left, right, selection, validation);
    }
    if let Some(result) =
        boolean_closed_validation_regularized_meshes(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return boolean_empty_operand(left, right, operation, validation);
    }
    if meshes_are_certified_bounds_disjoint(left, right) {
        return boolean_disjoint_meshes(left, right, operation, validation);
    }
    if let Some(result) =
        boolean_closed_regularized_lower_dimensional_optional(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
        && meshes_are_certified_identical(left, right)
    {
        return boolean_identical_meshes(left, operation, validation);
    }
    if (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
        && meshes_are_certified_same_surface(left, right)
    {
        return boolean_same_surface_meshes(left, operation, validation);
    }
    if let Some(result) =
        boolean_arrangement_orthogonal_solid_cell_recovery(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if let Some(result) =
        boolean_arrangement_affine_orthogonal_solid_recovery(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if let Some(result) = boolean_convex_meshes_optional(left, right, operation, validation)? {
        return Ok(result);
    }
    if let Some(result) =
        boolean_convex_relation_meshes_optional(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if let Some(result) =
        boolean_closed_winding_separated_meshes(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if let Some(result) =
        boolean_closed_winding_containment_meshes(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if let Some(result) =
        boolean_closed_boundary_touching_regularized_meshes(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if operation != ExactBooleanOperation::Union
        && let Some(result) =
            boolean_closed_no_volume_overlap_regularized_meshes(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if let Some(result) =
        boolean_arrangement_volumetric_split_cell_recovery(left, right, operation, validation)?
    {
        return Ok(result);
    }
    if let Some(result) =
        boolean_arrangement_cell_complex_meshes(left, right, operation, validation)?
    {
        return Ok(result);
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
    Materialized(Box<ExactBooleanResult>, ExactArrangementBooleanAttempt),
    Declined(ExactArrangementBooleanAttempt),
}

impl ArrangementCellComplexOutcome {
    fn materialized(
        result: ExactBooleanResult,
        attempt: ExactArrangementBooleanAttempt,
    ) -> ArrangementCellComplexOutcome {
        ArrangementCellComplexOutcome::Materialized(Box::new(result), attempt)
    }
}

fn materialized_arrangement_attempt_outcome(
    attempt: &mut ExactArrangementBooleanAttempt,
    result: ExactBooleanResult,
    clear_arrangement_blockers: bool,
) -> ArrangementCellComplexOutcome {
    attempt.stage = ExactArrangementBooleanStage::Materialized;
    attempt.decline = None;
    attempt.materialized_shortcut = Some(ExactBooleanShortcutKind::ArrangementCellComplex);
    if clear_arrangement_blockers {
        attempt.arrangement_blockers = 0;
    }
    attempt.output_vertices = result.mesh.vertices().len();
    attempt.output_triangles = result.mesh.triangles().len();
    ArrangementCellComplexOutcome::materialized(result, attempt.clone())
}

fn declined_output_validation_attempt_outcome(
    attempt: &mut ExactArrangementBooleanAttempt,
) -> ArrangementCellComplexOutcome {
    attempt.stage = ExactArrangementBooleanStage::Triangulated;
    attempt.decline = Some(ExactArrangementBooleanDecline::OutputValidation);
    ArrangementCellComplexOutcome::Declined(attempt.clone())
}

fn declined_output_validation_attempt_outcome_with_counts(
    attempt: &mut ExactArrangementBooleanAttempt,
    output_counts: Option<(usize, usize)>,
) -> ArrangementCellComplexOutcome {
    if let Some((vertices, triangles)) = output_counts {
        attempt.output_vertices = vertices;
        attempt.output_triangles = triangles;
    }
    declined_output_validation_attempt_outcome(attempt)
}

/// Report how far the arrangement/cell-complex Boolean pipeline gets for a
/// request without falling through to specialized materializers.
fn arrangement_boolean_attempt_report(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
    policy: ExactRegularizationPolicy,
) -> Result<ExactArrangementBooleanAttempt, MeshError> {
    Ok(
        match run_arrangement_cell_complex_attempt(
            left,
            right,
            request.operation,
            policy,
            Some(request.validation),
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
        ArrangementCellComplexOutcome::Materialized(result, _) => Ok(Some(*result)),
        ArrangementCellComplexOutcome::Declined(_) => Ok(None),
    }
}

/// Certify and materialize a named boolean through the arrangement cell-complex
/// pipeline.
///
/// This exposes the same arrangement-certified materialization used by
/// [`ExactBooleanRequest::materialize`]. It only runs when policy-aware preflight has already
/// certified [`ExactBooleanSupport::CertifiedArrangementCellComplex`], so
/// stronger exact paths such as convex, boundary-touching, winding, and trivial
/// shortcuts keep their dispatcher provenance. After that guard it delegates to
/// [`ExactBooleanRequest::materialize`] so earlier arrangement materializers keep their retained
/// result kind.
pub fn materialize_arrangement_cell_complex_boolean(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let preflight = ExactBooleanRequest::new(operation, validation).preflight(left, right)?;
    if preflight.support != ExactBooleanSupport::CertifiedArrangementCellComplex {
        return Ok(None);
    }
    ExactBooleanRequest::new(operation, validation)
        .materialize(left, right)
        .map(Some)
}

fn arrangement_cell_complex_result_is_certified_for_preflight(
    result: &ExactBooleanResult,
    attempt: &ExactArrangementBooleanAttempt,
) -> bool {
    attempt.decline.is_none()
        && (attempt.arrangement_blockers == 0
            || attempt.materialized_shortcut
                == Some(ExactBooleanShortcutKind::ArrangementCellComplex))
        && matches!(
            result.kind,
            ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
                | ExactBooleanResultKind::CertifiedShortcut {
                    shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
                    ..
                }
        )
}

fn arrangement_cell_complex_materializes_for_preflight(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    regularize_unregularized_sheet_complex: bool,
) -> Result<bool, MeshError> {
    let validation_policies: &[ValidationPolicy] =
        if left.facts().mesh.closed_manifold && right.facts().mesh.closed_manifold {
            &[ValidationPolicy::CLOSED]
        } else {
            &[ValidationPolicy::CLOSED, ValidationPolicy::ALLOW_BOUNDARY]
        };
    for &validation in validation_policies {
        match run_arrangement_cell_complex_attempt(
            left,
            right,
            operation,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
            Some(validation),
            regularize_unregularized_sheet_complex,
        ) {
            Ok(ArrangementCellComplexOutcome::Materialized(result, attempt))
                if arrangement_cell_complex_result_is_certified_for_preflight(
                    &result, &attempt,
                ) =>
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
            left,
            right,
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
        operation,
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
        output_validation: validation.unwrap_or_default(),
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

    if let Some(validation) = validation
        && let Some(outcome) = arrangement_orthogonal_solid_cell_recovery_outcome(
            &mut attempt,
            left,
            right,
            operation,
            validation,
        )?
    {
        return Ok(outcome);
    }

    if let Some(validation) = validation
        && let Some(result) =
            boolean_arrangement_adjacency_union_completion(left, right, operation, validation)?
    {
        return Ok(materialized_arrangement_attempt_outcome(
            &mut attempt,
            result,
            false,
        ));
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
        return Ok(materialized_arrangement_attempt_outcome(
            &mut attempt,
            result,
            false,
        ));
    }

    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && let Some(validation) = validation
    {
        match boolean_coplanar_mesh_overlay_optional(left, right, operation, validation) {
            Ok(Some(result)) => {
                return Ok(materialized_arrangement_attempt_outcome(
                    &mut attempt,
                    result,
                    false,
                ));
            }
            Ok(None) => {}
            Err(_) => {
                let output_counts = coplanar_mesh_overlay_candidate_counts(left, right, operation);
                return Ok(declined_output_validation_attempt_outcome_with_counts(
                    &mut attempt,
                    output_counts,
                ));
            }
        }
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
        match materialize_simple_coplanar_overlay_arrangement(
            left,
            right,
            operation,
            validation,
            &arrangement,
        ) {
            Ok(Some(result)) => {
                return Ok(materialized_arrangement_attempt_outcome(
                    &mut attempt,
                    result,
                    false,
                ));
            }
            Ok(None) => {}
            Err(_) => {
                let output_counts = coplanar_mesh_overlay_candidate_counts(left, right, operation);
                return Ok(declined_output_validation_attempt_outcome_with_counts(
                    &mut attempt,
                    output_counts,
                ));
            }
        }
        if regularize_unregularized_sheet_complex
            && arrangement_blockers_are_unregularized_sheet_complex(&arrangement.blockers)
            && let Some(validation) = validation
            && let Some(result) = boolean_arrangement_regularized_sheet_complex_from_graph(
                &arrangement.graph,
                left,
                right,
                operation,
                validation,
            )?
        {
            return Ok(materialized_arrangement_attempt_outcome(
                &mut attempt,
                result,
                true,
            ));
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
                let result = certified_shortcut_result(
                    mesh,
                    operation,
                    ExactBooleanShortcutKind::ArrangementCellComplex,
                );
                return Ok(materialized_arrangement_attempt_outcome(
                    &mut attempt,
                    result,
                    false,
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
    let result = certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    Ok(materialized_arrangement_attempt_outcome(
        &mut attempt,
        result,
        volume_resolves_region_classification,
    ))
}

fn boolean_arrangement_orthogonal_solid_cell_recovery(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation) else {
        return Ok(None);
    };
    let Some(plan) = axis_aligned_orthogonal_solid_cell_plan(left, right, solid_operation) else {
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
    let mesh = materialize_axis_aligned_orthogonal_solid_cell_plan(plan, label, validation)?;
    let result = certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

/// Certify and materialize a named boolean for axis-aligned orthogonal solids.
///
/// This exposes the exact cell-recovery path used by [`ExactBooleanRequest::materialize`] as an
/// [`ExactBooleanResult`], retaining the named operation and the
/// arrangement-cell shortcut provenance. Inputs outside the supportable
/// orthogonal cell model return `None` rather than falling through to unrelated
/// topology paths.
fn materialize_axis_aligned_orthogonal_solid_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    Ok(public_operation_replayable_result(
        boolean_arrangement_orthogonal_solid_cell_recovery(left, right, operation, validation)?,
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    ))
}

fn arrangement_orthogonal_solid_cell_recovery_outcome(
    attempt: &mut ExactArrangementBooleanAttempt,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ArrangementCellComplexOutcome>, MeshError> {
    let Some(result) =
        boolean_arrangement_orthogonal_solid_cell_recovery(left, right, operation, validation)?
    else {
        return Ok(None);
    };
    Ok(Some(materialized_arrangement_attempt_outcome(
        attempt, result, true,
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
            let output_counts = open_surface_arrangement_candidate_counts(
                left,
                right,
                operation,
                graph.has_unknowns(),
                plan,
            );
            return Ok(Some(
                declined_output_validation_attempt_outcome_with_counts(attempt, output_counts),
            ));
        }
        Err(error) => {
            let output_counts = open_surface_arrangement_candidate_counts(
                left,
                right,
                operation,
                graph.has_unknowns(),
                plan,
            );
            if output_counts.is_some() {
                return Ok(Some(
                    declined_output_validation_attempt_outcome_with_counts(attempt, output_counts),
                ));
            }
            return Err(error);
        }
    };
    Ok(Some(materialized_arrangement_attempt_outcome(
        attempt, result, false,
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
    Ok(Some(materialized_arrangement_attempt_outcome(
        attempt, result, true,
    )))
}

fn boolean_arrangement_adjacency_union_completion(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    Ok(adjacent_union_completion_certification(left, right, operation, Some(validation))?.1)
}

fn adjacent_union_completion_blocker_kind(
    status: &ExactAdjacentUnionCompletionStatus,
    counts: ExactBooleanBlocker,
) -> ExactBooleanBlockerKind {
    match status {
        ExactAdjacentUnionCompletionStatus::GraphUnresolved => {
            ExactBooleanBlockerKind::NeedsRefinement
        }
        ExactAdjacentUnionCompletionStatus::CertifiedFullFace
        | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace => {
            ExactBooleanBlockerKind::NeedsBoundaryPolicy
        }
        _ => retained_graph_blocker_kind(counts),
    }
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
    let blocker_kind = adjacent_union_completion_blocker_kind(&status, counts);
    ExactAdjacentUnionCompletionReport {
        operation,
        status,
        left_closed,
        right_closed,
        axis_aligned_box_pair,
        stronger_kernel_available,
        graph_had_unknowns,
        retained_face_pairs,
        retained_events,
        blocker: counts.into_blocker(blocker_kind),
        full_face_shared_faces,
        full_face_shared_patches,
        contained_containing_side,
        contained_faces,
        containing_faces,
    }
}

fn adjacent_union_completion_report_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactAdjacentUnionCompletionReport, MeshError> {
    Ok(adjacent_union_completion_certification(left, right, request.operation, None)?.0)
}

fn adjacent_union_completion_certification(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    materialization_validation: Option<ValidationPolicy>,
) -> Result<
    (
        ExactAdjacentUnionCompletionReport,
        Option<ExactBooleanResult>,
    ),
    MeshError,
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
    let axis_aligned_box_pair = both_axis_aligned_boxes(left, right);
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
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    let graph_had_unknowns = graph.has_unknowns();
    let retained_face_pairs = graph.face_pairs.len();
    let retained_events = graph.event_count();
    let counts = retained_graph_counts(&graph);
    if graph_had_unknowns || counts.construction_failed_events != 0 {
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

    if let Some(certificate) = full_face_adjacent_certificate(left, right)
        && let Some(union) = materialize_full_face_adjacent_union_from_certificate(
            left,
            right,
            &certificate,
            materialization_validation.unwrap_or(ValidationPolicy::CLOSED),
        )
    {
        let full_face_shared_faces = union.shared_faces.len();
        let full_face_shared_patches = union.shared_patches.len();
        let result = materialization_validation.and_then(|_| {
            let result = certified_shortcut_result(
                union.mesh,
                ExactBooleanOperation::Union,
                ExactBooleanShortcutKind::ArrangementCellComplex,
            );
            (result.validate().is_ok() && result.validate_against_sources(left, right).is_ok())
                .then_some(result)
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

    if certified_convex_materialized_boolean_support(left, right, operation).is_some() {
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

    if contained_face_adjacency_should_yield_to_stronger_kernel(left, right, operation) {
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

    if let Some(certificate) = contained_face_adjacent_certificate(left, right)
        && let Some(union) = materialize_contained_face_adjacent_union_from_certificate(
            left,
            right,
            &certificate,
            materialization_validation.unwrap_or(ValidationPolicy::CLOSED),
        )
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
            (result.validate().is_ok() && result.validate_against_sources(left, right).is_ok())
                .then_some(result)
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

fn materialize_adjacent_union_completion_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<(ExactBooleanResult, ExactAdjacentUnionCompletionReport)>, MeshError> {
    let (report, result) = adjacent_union_completion_certification(
        left,
        right,
        request.operation,
        Some(request.validation),
    )?;
    if !report.is_certified() {
        return Ok(None);
    }
    report.validate().map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact adjacent-union completion report validation failed: {error:?}"),
        ))
    })?;
    if report.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    let Some(result) = result else {
        return Ok(None);
    };
    if result
        .validate_operation_against_sources(
            left,
            right,
            request.operation,
            request.validation,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .is_err()
    {
        return Ok(None);
    }
    Ok(Some((result, report)))
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
    if operation == ExactBooleanOperation::Union {
        let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
        evidence.validate().map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("exact no-volume-overlap union evidence validation failed: {error:?}"),
            ))
        })?;
        if evidence.obstacle != CoplanarVolumetricCellObstacle::BoundaryOnlyContact
            || evidence.positive_area_coplanar_overlapping_pairs == 0
        {
            return Ok(None);
        }
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
        if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
            return Ok(None);
        }
        return Ok(Some(result));
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
            ExactBooleanShortcutKind::ArrangementCellComplex,
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
    let result = certified_shortcut_result(mesh, operation, shortcut);
    if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

/// Certify and materialize a regularized closed-solid boolean for positive-area
/// boundary contact with no shared volume.
///
/// Certify and materialize a regularized closed-solid boolean for positive-area
/// boundary contact, returning the exact coplanar-volumetric evidence consumed
/// by the decision.
///
/// evidence certifies boundary-only positive-area coplanar contact with no
/// shared volume; unsupported or non-boundary-only contact states return
/// `None`.
fn materialize_closed_no_volume_overlap_regularized_with_evidence_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    let Some((result, evidence)) =
        materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
            &graph, left, right, operation, validation,
        )?
    else {
        return Ok(None);
    };
    if result
        .validate_operation_against_sources(
            left,
            right,
            operation,
            validation,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .is_err()
        || evidence.validate_against_sources(left, right).is_err()
    {
        return Ok(None);
    }
    Ok(Some((result, evidence)))
}

fn materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    evidence.validate().map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact no-volume-overlap evidence validation failed: {error:?}"),
        ))
    })?;
    if evidence.obstacle != CoplanarVolumetricCellObstacle::BoundaryOnlyContact
        || evidence.positive_area_coplanar_overlapping_pairs == 0
    {
        return Ok(None);
    }
    let Some(result) = materialize_closed_no_volume_overlap_regularized_result_from_evidence(
        graph, left, right, operation, validation, &evidence,
    )?
    else {
        return Ok(None);
    };
    Ok(Some((result, evidence)))
}

fn boolean_closed_no_volume_overlap_regularized_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    Ok(
        materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
            &graph, left, right, operation, validation,
        )?
        .map(|(result, _evidence)| result),
    )
}

fn materialize_closed_no_volume_overlap_regularized_result_from_evidence(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    evidence: &CoplanarVolumetricCellEvidenceReport,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || evidence.obstacle != CoplanarVolumetricCellObstacle::BoundaryOnlyContact
        || evidence.positive_area_coplanar_overlapping_pairs == 0
    {
        return Ok(None);
    }
    match operation {
        ExactBooleanOperation::Union => {
            boolean_arrangement_regularized_no_volume_overlap_from_graph(
                graph, left, right, operation, validation,
            )
        }
        ExactBooleanOperation::Intersection => {
            let mesh = empty_mesh(
                "empty exact no-volume-overlap regularized intersection",
                validation,
            )?;
            let result = certified_shortcut_result(
                mesh,
                operation,
                ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection,
            );
            if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
                return Ok(None);
            }
            Ok(Some(result))
        }
        ExactBooleanOperation::Difference => {
            let mesh = copy_mesh(
                left,
                "exact no-volume-overlap difference preserving left shell",
                validation,
            )?;
            let result = certified_shortcut_result(
                mesh,
                operation,
                ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
            );
            if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
                return Ok(None);
            }
            Ok(Some(result))
        }
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled above"),
    }
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
        if validation == ValidationPolicy::CLOSED
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
        attempt, result, true,
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
    Ok(Some(materialized_arrangement_attempt_outcome(
        attempt, result, true,
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
            return Ok(Some(materialized_arrangement_attempt_outcome(
                attempt, result, true,
            )));
        }
        if let Some(result) = boolean_arrangement_convex_regularized_sheet_recovery(
            left, right, operation, validation,
        )? {
            return Ok(Some(materialized_arrangement_attempt_outcome(
                attempt, result, true,
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
        attempt, left, right, operation, validation,
    )? {
        return Ok(Some(outcome));
    }
    arrangement_affine_orthogonal_solid_recovery_outcome(
        attempt, left, right, operation, validation,
    )
}

fn boolean_convex_meshes_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let (mesh, shortcut, label) = match operation {
        ExactBooleanOperation::Union => {
            let Some(union) = union_closed_convex_solids(left, right) else {
                return Ok(None);
            };
            (
                union.mesh,
                ExactBooleanShortcutKind::ConvexUnion,
                "exact closed-convex solid union boolean result",
            )
        }
        ExactBooleanOperation::Intersection => {
            let Some(intersection) = intersect_closed_convex_solids(left, right) else {
                return Ok(None);
            };
            (
                intersection.mesh,
                ExactBooleanShortcutKind::ConvexIntersection,
                "exact closed-convex solid intersection boolean result",
            )
        }
        ExactBooleanOperation::Difference => {
            let Some(difference) = subtract_closed_convex_solids(left, right) else {
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
    if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
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
) -> Result<Option<ConvexRelationShortcut>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let relation_counts = retained_graph_counts(graph);
    if graph.has_unknowns() || relation_counts.construction_failed_events > 0 {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_convex_solid_report(left, right);
    left_in_right
        .validate_against_sources(left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("left convex relation replay failed: {error:?}"),
            ))
        })?;
    let right_in_left = classify_mesh_vertices_against_convex_solid_report(right, left);
    right_in_left
        .validate_against_sources(right, left)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("right convex relation replay failed: {error:?}"),
            ))
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
        convex_boundary_containment_is_supported(&left_in_right, &right_in_left);
    let right_boundary_inside_left =
        convex_boundary_containment_is_supported(&right_in_left, &left_in_right);
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

fn boolean_convex_relation_meshes_optional(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    let Some(relation) =
        certified_convex_relation_shortcut_from_graph(&graph, left, right, operation)?
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
                ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
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
                ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
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
                ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
            };
            (mesh, ExactBooleanShortcutKind::ConvexContainment)
        }
    };
    let result = certified_shortcut_result(mesh, operation, shortcut);
    if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

/// Certify and materialize a named boolean for closed convex solids.
///
/// This public wrapper follows [`ExactBooleanRequest::materialize`] precedence: it only
/// materializes when preflight certifies the requested operation as a direct
/// convex operation or convex relation shortcut. Inputs handled by earlier
/// exact shortcuts, such as orthogonal-cell recovery or bounds disjointness,
/// return `None` so replay provenance remains stable.
fn materialize_closed_convex_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
        .preflight(left, right)?;
    let result = match preflight.support {
        ExactBooleanSupport::CertifiedConvexUnion
        | ExactBooleanSupport::CertifiedConvexIntersection
        | ExactBooleanSupport::CertifiedConvexDifference => {
            boolean_convex_meshes_optional(left, right, operation, validation)?
        }
        ExactBooleanSupport::CertifiedConvexContainment
        | ExactBooleanSupport::CertifiedConvexSeparated => {
            boolean_convex_relation_meshes_optional(left, right, operation, validation)?
        }
        _ => return Ok(None),
    };
    Ok(public_operation_replayable_result(
        result,
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    ))
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
    let result = certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

/// Certify and materialize a closed-solid arrangement boolean from exact
/// volumetric winding region classifications.
///
/// Directly closed or boundary-valid output retains the split-region plane
/// classifications, triangulations, volumetric classifications, assembly plan,
/// output mesh, and source-replay freshness checks needed to audit the named
/// cell-complex decision. Closed outputs whose boundary-valid split-cell
/// assembly has exact non-self-contacting coplanar cap loops materialize as a
/// certified arrangement-cell-complex shortcut; callers can audit that cap
/// decision with [`ExactBooleanRequest::volumetric_boundary_closure`]. Cases outside
/// the currently supportable exact winding arrangement path return `None`
/// rather than falling back to approximate winding.
pub fn materialize_volumetric_winding_arrangement(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    Ok(public_operation_replayable_result(
        boolean_arrangement_volumetric_split_cell_recovery(left, right, operation, validation)?,
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    ))
}

/// Certify and materialize a closed volumetric winding arrangement whose
/// remaining boundary is closed by exact coplanar cap loops.
///
/// This is the provenance-retaining form of the coplanar cap path used by
/// [`materialize_volumetric_winding_arrangement`]. It returns both the
/// certified Boolean result and the exact closure report that authorized the
/// cap decision, so callers can replay the volumetric split-cell output and
/// the cap-readiness evidence together. Non-coplanar, self-contacting, or
/// otherwise blocked boundary output returns `None`.
pub fn materialize_volumetric_coplanar_boundary_closure_boolean(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<(ExactBooleanResult, ExactVolumetricBoundaryClosureReport)>, MeshError> {
    let Some((mesh, closure_report)) = materialize_volumetric_coplanar_boundary_closure_output(
        left, right, operation, validation,
    )?
    else {
        return Ok(None);
    };
    let result = certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::ArrangementCellComplex,
    );
    if result
        .validate_operation_against_sources(
            left,
            right,
            operation,
            validation,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .is_err()
        || closure_report
            .validate_against_sources(left, right)
            .is_err()
    {
        return Ok(None);
    }
    Ok(Some((result, closure_report)))
}

pub(crate) fn materialize_volumetric_coplanar_boundary_closure_output(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<(ExactMesh, ExactVolumetricBoundaryClosureReport)>, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    materialize_volumetric_coplanar_boundary_closure_output_from_graph(
        &graph, left, right, operation, validation,
    )
}

fn materialize_volumetric_coplanar_boundary_closure_output_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<(ExactMesh, ExactVolumetricBoundaryClosureReport)>, MeshError> {
    let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        graph,
        left,
        right,
        operation,
        ValidationPolicy::ALLOW_BOUNDARY,
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
    ) else {
        return Ok(None);
    };
    let closure_report =
        volumetric_boundary_closure_report_from_materialized(&materialized, operation)?;
    if closure_report.status != ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
        || closure_report.validate().is_err()
        || closure_report
            .validate_against_sources(left, right)
            .is_err()
    {
        return Ok(None);
    }
    Ok(Some((mesh, closure_report)))
}

fn boolean_arrangement_volumetric_split_cell_recovery(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    boolean_arrangement_volumetric_split_cell_recovery_from_graph(
        &graph, left, right, operation, validation,
    )
}

fn boolean_arrangement_volumetric_split_cell_recovery_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if validation == ValidationPolicy::CLOSED {
        let Some(mut materialized) = materialize_volumetric_winding_region_plan_from_graph(
            graph,
            left,
            right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
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
            if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
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
            if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
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
    if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

fn volumetric_winding_open_boundary_candidate_counts(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<(usize, usize)>, MeshError> {
    let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        graph,
        left,
        right,
        operation,
        ValidationPolicy::ALLOW_BOUNDARY,
    )?
    else {
        return Ok(None);
    };
    if materialized.mesh.facts().mesh.closed_manifold || materialized.mesh.triangles().is_empty() {
        return Ok(None);
    }
    if matches!(
        volumetric_boundary_closure_report_from_materialized(&materialized, operation)?.status,
        ExactVolumetricBoundaryClosureStatus::AlreadyClosed
            | ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
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
        mesh: materialized.mesh,
    }
}

fn close_exact_coplanar_boundary_loops(
    mesh: &ExactMesh,
    label: &'static str,
    validation: ValidationPolicy,
) -> Option<ExactMesh> {
    if mesh.facts().mesh.closed_manifold || mesh.facts().mesh.boundary_edges == 0 {
        return None;
    }
    let boundary_loops = directed_boundary_loops(mesh)?;
    if !boundary_loops_are_exactly_coplanar_without_self_contact(mesh, &boundary_loops)? {
        return None;
    }
    close_exact_coplanar_boundary_loops_from_loops(mesh, boundary_loops, label, validation)
}

fn boundary_loops_are_exactly_coplanar_without_self_contact(
    mesh: &ExactMesh,
    boundary_loops: &[Vec<usize>],
) -> Option<bool> {
    let boundary_points = boundary_loops
        .iter()
        .map(|boundary_loop| {
            boundary_loop
                .iter()
                .map(|&vertex| mesh.vertices().get(vertex).cloned())
                .collect::<Option<Vec<_>>>()
                .and_then(|points| split_boundary_self_contact_cycles(points).ok())
        })
        .collect::<Option<Vec<_>>>()?
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
    if boundary_points.is_empty() {
        return None;
    }
    for boundary in &boundary_points {
        if boundary.len() < 3 {
            return None;
        }
        let self_contact = boundary_loop_self_contact_evidence(boundary).ok()?;
        if self_contact.repeated_exact_point_pairs != 0 {
            return None;
        }
        if !exact_loop_is_coplanar(boundary).ok()? {
            return None;
        }
    }
    Some(true)
}

fn certified_coplanar_boundary_closure_from_materialized(
    materialized: &MaterializedVolumetricWindingRegionPlan,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactMesh>, MeshError> {
    let Some(mesh) = close_exact_coplanar_boundary_loops(
        &materialized.mesh,
        "exact volumetric split-cell coplanar boundary closure",
        validation,
    ) else {
        return Ok(None);
    };
    let closure_report =
        volumetric_boundary_closure_report_from_materialized(materialized, operation)?;
    if closure_report.status != ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
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
    validation: ValidationPolicy,
) -> Option<ExactMesh> {
    if mesh.facts().mesh.closed_manifold || mesh.facts().mesh.boundary_edges == 0 {
        return None;
    }
    if boundary_loops.is_empty() {
        return None;
    }

    let boundary_edges = directed_boundary_edges(mesh);
    let split_boundary_loops = boundary_loops
        .into_iter()
        .map(|boundary_loop| split_boundary_vertex_self_contact_cycles(mesh, boundary_loop).ok())
        .collect::<Option<Vec<_>>>()?
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
    if split_boundary_loops.is_empty() {
        return None;
    }
    if split_boundary_loops
        .iter()
        .all(|boundary_loop| boundary_loop.len() == 3)
    {
        let cap_triangles = split_boundary_loops
            .iter()
            .map(|boundary_loop| {
                let points = boundary_loop
                    .iter()
                    .map(|&vertex| mesh.vertices().get(vertex).cloned())
                    .collect::<Option<Vec<_>>>()?;
                if boundary_loop_self_contact_evidence(&points)
                    .ok()?
                    .repeated_exact_point_pairs
                    != 0
                    || !exact_loop_is_coplanar(&points).ok()?
                {
                    return None;
                }
                Some(Triangle([
                    boundary_loop[0],
                    boundary_loop[1],
                    boundary_loop[2],
                ]))
            })
            .collect::<Option<Vec<_>>>()?;
        let cap_triangles = orient_cap_group_against_mesh_boundary(&boundary_edges, cap_triangles)?;
        let mut triangles = mesh.triangles().to_vec();
        triangles.extend(cap_triangles);
        return ExactMesh::new_with_policy(
            mesh.vertices().to_vec(),
            triangles,
            SourceProvenance::exact(label),
            validation,
        )
        .ok();
    }

    let cap_groups = group_exact_coplanar_vertex_loops(mesh, split_boundary_loops).ok()?;
    let mut vertices = mesh.vertices().to_vec();
    let mut cap_triangles = Vec::new();
    for vertex_loops in cap_groups {
        let loops = vertex_loops
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
        cap_triangles.extend(orient_cap_group_against_mesh_boundary(
            &boundary_edges,
            triangles.collect(),
        )?);
    }

    let mut triangles = mesh.triangles().to_vec();
    triangles.extend(cap_triangles);
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        validation,
    )
    .ok()
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
                let Some(existing) = mesh.vertices().get(boundary_vertex) else {
                    return None;
                };
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
        operation,
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
    let allow_empty_overlay = coplanar_mesh_overlay_allows_empty(operation);
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
    let allow_empty_overlay = coplanar_mesh_overlay_allows_empty(operation);
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
    .map(|mesh| (mesh.vertices().len(), mesh.triangles().len()))
}

fn coplanar_mesh_overlay_allows_empty(operation: ExactBooleanOperation) -> bool {
    matches!(
        operation,
        ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
    )
}

fn coplanar_mesh_overlay_boundary_policy(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactArrangement2dBoundaryPolicy> {
    Some(match operation {
        ExactBooleanOperation::Difference => {
            coplanar_mesh_overlay_materialized_difference_boundary_policy(left, right)
                .unwrap_or(ExactArrangement2dBoundaryPolicy::SimplifyCollinear)
        }
        ExactBooleanOperation::Intersection => {
            coplanar_mesh_overlay_surface_intersection_boundary_policy(left, right)
                .unwrap_or(ExactArrangement2dBoundaryPolicy::SimplifyCollinear)
        }
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

/// Certify and materialize a named coplanar mesh-overlay arrangement boolean.
///
/// This path is intentionally limited to exact coplanar surface overlays whose
/// selected planar cells can be replayed through the retained 2D arrangement.
/// Unsupported topology returns `None` rather than relaxing to tolerance-based
/// geometry. The returned [`ExactBooleanResult`] retains the arrangement-cell
/// shortcut kind, output mesh provenance, and source-replay freshness checks.
pub fn materialize_coplanar_mesh_overlay_arrangement(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let Some(result) = boolean_coplanar_mesh_overlay_optional(left, right, operation, validation)?
    else {
        return Ok(None);
    };
    if result
        .validate_operation_against_sources(
            left,
            right,
            operation,
            validation,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .is_err()
    {
        return Ok(None);
    }
    Ok(Some(result))
}

pub(crate) fn replay_coplanar_mesh_overlay_result(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    boolean_coplanar_mesh_overlay_optional(left, right, operation, validation)
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
                    | ExactArrangement2dBlocker::NonManifoldSelectedBoundary { .. }
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
        let mut face_vertices = Vec::new();
        let mut face_triangles = Vec::new();
        triangulate_exact_loop_group(&[boundary], &mut face_vertices, &mut face_triangles).ok()?;
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
            .collect::<Option<Vec<_>>>()?;
        triangles.extend(face_triangles.into_iter().map(|triangle| {
            Triangle([
                face_to_mesh[triangle.0[0]],
                face_to_mesh[triangle.0[1]],
                face_to_mesh[triangle.0[2]],
            ])
        }));
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
    [
        ExactArrangement2dBoundaryPolicy::SimplifyCollinear,
        ExactArrangement2dBoundaryPolicy::PreserveCollinear,
    ]
    .into_iter()
    .find(|&boundary_policy| {
        materialize_coplanar_mesh_overlay_mesh(
            left,
            right,
            operation,
            boundary_policy,
            "exact coplanar mesh overlay arrangement",
            allow_empty,
        )
        .is_some()
    })
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
    topology_triangle_edges(triangle.0)
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
    if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

/// Certify and materialize a named boolean for affine orthogonal solids.
///
/// The output is the boolean-result wrapper around the exact affine-cell
/// materializer, so callers can validate both the output mesh and the retained
/// decision provenance through [`ExactBooleanResult`] replay.
fn materialize_affine_orthogonal_solid_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let Some(result) =
        boolean_arrangement_affine_orthogonal_solid_recovery(left, right, operation, validation)?
    else {
        return Ok(None);
    };
    if result
        .validate_operation_against_sources(
            left,
            right,
            operation,
            validation,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .is_err()
    {
        return Ok(None);
    }
    Ok(Some(result))
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
        operation,
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
        let result = materialize_open_surface_disjoint_meshes(left, right, operation, validation)?;
        return Ok(open_surface_disjoint_result_consumes_report(
            &result,
            left,
            right,
            operation,
            &disjoint_report,
        )
        .then_some(result));
    }
    Ok(None)
}

pub(crate) fn open_surface_disjoint_result_matches_sources(
    result: &ExactBooleanResult,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> bool {
    let Ok(report) = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
        .open_surface_disjoint_report(left, right)
    else {
        return false;
    };
    if report.validate().is_err() || !report.is_certified() {
        return false;
    }
    if !open_surface_disjoint_result_consumes_report(result, left, right, operation, &report) {
        return false;
    }
    let Ok(expected) = materialize_open_surface_disjoint_meshes(left, right, operation, validation)
    else {
        return false;
    };
    expected.validate().is_ok() && result == &expected
}

fn open_surface_disjoint_result_consumes_report(
    result: &ExactBooleanResult,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    report: &ExactOpenSurfaceDisjointReport,
) -> bool {
    report.validate().is_ok()
        && report.is_certified()
        && report.validate_against_sources(left, right).is_ok()
        && matches!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: result_operation,
                shortcut: ExactBooleanShortcutKind::OpenSurfaceDisjoint
            } if result_operation == operation
        )
        && result.validate().is_ok()
}

/// Certify and materialize a named boolean for open surfaces with no retained
/// exact face-pair intersections.
///
/// Bounds-disjoint inputs are handled by an earlier exact shortcut, so this
/// function returns `None` for that case and only materializes the graph-backed
/// open-surface disjoint certificate used by [`ExactBooleanRequest::materialize`].
fn materialize_open_surface_disjoint_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || meshes_are_certified_bounds_disjoint(left, right)
        || (validation == ValidationPolicy::CLOSED
            && certified_closed_validation_regularized_solid_support(left, right).is_some())
    {
        return Ok(None);
    }
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    Ok(public_operation_replayable_result(
        boolean_open_surface_disjoint_meshes_from_graph(
            &graph, left, right, operation, validation,
        )?,
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    ))
}

/// Certify whether two open surface meshes are disjoint by exact graph facts.
///
/// This is the report form of the open-surface named-boolean shortcut. It
/// validates the open-surface precondition from exact mesh facts, then records
/// the retained graph relation counts that prove no face pair survived exact
/// graph fact, not a tolerance side effect.
fn open_surface_disjoint_report_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    _request: ExactBooleanRequest,
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
            ExactBooleanBlocker::default(),
        );
    }
    let graph_had_unknowns = graph.has_unknowns();
    let counts = retained_graph_counts(graph);
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
    counts: ExactBooleanBlocker,
) -> ExactOpenSurfaceDisjointReport {
    let blocker_kind = retained_graph_blocker_kind(counts);
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

fn certified_lower_dimensional_regularized_solid_support(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ExactBooleanSupport> {
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return None;
    }
    let left_kind = closed_regularized_operand_kind(left)?;
    let right_kind = closed_regularized_operand_kind(right)?;
    (!left_kind.has_volume() && !right_kind.has_volume())
        .then_some(ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid)
}

fn certified_closed_validation_regularized_solid_support(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ExactBooleanSupport> {
    certified_lower_dimensional_regularized_solid_support(left, right)
        .or_else(|| certified_mixed_dimensional_regularized_solid_support(left, right))
}

fn boolean_closed_validation_regularized_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if validation != ValidationPolicy::CLOSED
        || matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || certified_closed_validation_regularized_solid_support(left, right).is_none()
    {
        return Ok(None);
    }
    boolean_closed_regularized_lower_dimensional_optional(left, right, operation, validation)
}

/// Retained split-region artifacts that certify an open-surface arrangement.
type OpenSurfaceArrangementPlan = (
    ExactBooleanSupport,
    Vec<FaceRegionPlaneClassification>,
    Vec<FaceRegionTriangulation>,
);

/// Certify and materialize a named arrangement boolean for crossing open surfaces.
///
/// The result is returned as an [`ExactBooleanResult`] because that artifact
/// already retains the exact split-region classifications, triangulations,
/// assembly, output mesh, and source-replay freshness checks for this bounded
/// surface arrangement path. Unsupported open-surface contacts return `None`
/// rather than falling back to approximate winding.
pub fn materialize_open_surface_arrangement(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    let Some(plan) = open_surface_arrangement_plan_from_graph(&graph, left, right, operation)?
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
    Ok(public_operation_replayable_result(
        result,
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    ))
}

pub(crate) fn replay_open_surface_arrangement_result(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    let Some(plan) = open_surface_arrangement_plan_from_graph(&graph, left, right, operation)?
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
    validation: ValidationPolicy,
    graph_had_unknowns: bool,
    plan: OpenSurfaceArrangementPlan,
) -> Result<Option<ExactBooleanResult>, MeshError> {
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
    let mut assembly = ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
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
    assembly
        .canonicalize_for_mesh_with_sources(left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::IndexOutOfBounds,
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
        mesh,
    };
    result.validate().map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("open-surface arrangement validation failed: {error:?}"),
        ))
    })?;
    Ok(Some(result))
}

fn open_surface_arrangement_candidate_counts(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    graph_had_unknowns: bool,
    plan: OpenSurfaceArrangementPlan,
) -> Option<(usize, usize)> {
    materialize_open_surface_arrangement_plan(
        left,
        right,
        operation,
        ValidationPolicy::ALLOW_BOUNDARY,
        graph_had_unknowns,
        plan,
    )
    .ok()
    .flatten()
    .map(|result| (result.mesh.vertices().len(), result.mesh.triangles().len()))
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
    let counts = retained_graph_counts(graph);
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
        operation,
        ExactBooleanShortcutKind::SameSurface,
    ))
}

/// Certify and materialize a named boolean for identical non-closed meshes.
///
/// This exposes the same exact shortcut used by [`ExactBooleanRequest::materialize`]. Closed
/// solids that are identical route through the closed arrangement path instead,
/// so this function returns `None` for those cases to preserve dispatcher
/// provenance. Unsupported operations or non-identical sources also return
/// `None`.
fn materialize_identical_mesh_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || (left.facts().mesh.closed_manifold && right.facts().mesh.closed_manifold)
        || (validation == ValidationPolicy::CLOSED
            && certified_closed_validation_regularized_solid_support(left, right).is_some())
        || !meshes_are_certified_identical(left, right)
    {
        return Ok(None);
    }
    Ok(public_operation_replayable_result(
        Some(boolean_identical_meshes(left, operation, validation)?),
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    ))
}

/// Certify and materialize a named boolean for equal non-closed surfaces with
/// different retained mesh encodings.
///
/// The retained same-surface report proves exact coordinate equality and
/// triangle-set equality after vertex remapping. Byte-identical meshes and
/// closed solids are deliberately left to their earlier exact dispatcher paths,
/// so they return `None` here instead of changing replay provenance.
fn materialize_same_surface_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || (left.facts().mesh.closed_manifold && right.facts().mesh.closed_manifold)
        || (validation == ValidationPolicy::CLOSED
            && certified_closed_validation_regularized_solid_support(left, right).is_some())
        || meshes_are_certified_identical(left, right)
        || !meshes_are_certified_same_surface(left, right)
    {
        return Ok(None);
    }
    Ok(public_operation_replayable_result(
        Some(boolean_same_surface_meshes(left, operation, validation)?),
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    ))
}

/// Certify and materialize the closed same-surface arrangement boolean.
///
/// Closed solids with identical or same-surface boundaries can route through
/// the arrangement cell-complex path in [`ExactBooleanRequest::materialize`], rather than the
/// non-closed same-surface shortcut. This exposes that named materializer when
/// preflight certifies arrangement provenance, while yielding to earlier exact
/// paths such as direct convex materialization.
fn materialize_closed_same_surface_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || !left.facts().mesh.closed_manifold
        || !right.facts().mesh.closed_manifold
        || !meshes_are_certified_same_surface(left, right)
    {
        return Ok(None);
    }
    let preflight = ExactBooleanRequest::new(operation, validation).preflight(left, right)?;
    if preflight.support != ExactBooleanSupport::CertifiedArrangementCellComplex {
        return Ok(None);
    }
    boolean_arrangement_cell_complex_meshes(left, right, operation, validation)
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

fn closed_zero_area_boundary_contact_evidence_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<CoplanarVolumetricCellEvidenceReport>, MeshError> {
    if !left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold {
        return Ok(None);
    }
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right);
    evidence.validate().map_err(|error| {
        MeshError::one(MeshDiagnostic::new(
            Severity::Error,
            DiagnosticKind::UnsupportedExactOperation,
            format!("exact zero-area boundary contact evidence validation failed: {error:?}"),
        ))
    })?;
    Ok(
        (evidence.obstacle == CoplanarVolumetricCellObstacle::BoundaryOnlyContact
            && evidence.positive_area_coplanar_overlapping_pairs == 0)
            .then_some(evidence),
    )
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

fn certified_closed_zero_area_boundary_contact_support_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<Option<ExactBooleanSupport>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || closed_zero_area_boundary_contact_evidence_from_graph(graph, left, right)?.is_none()
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

fn materialize_closed_boundary_touching_regularized_result_from_evidence(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    evidence: &CoplanarVolumetricCellEvidenceReport,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || evidence.obstacle != CoplanarVolumetricCellObstacle::BoundaryOnlyContact
        || evidence.positive_area_coplanar_overlapping_pairs != 0
    {
        return Ok(None);
    };
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
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by evidence check"),
    };
    let result = certified_shortcut_result(mesh, operation, shortcut);
    if result.validate().is_err() || result.validate_against_sources(left, right).is_err() {
        return Ok(None);
    }
    Ok(Some(result))
}

fn boolean_closed_boundary_touching_regularized_meshes(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    Ok(
        materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
            &graph, left, right, operation, validation,
        )?
        .map(|(result, _evidence)| result),
    )
}

fn materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>, MeshError> {
    let Some(evidence) = closed_zero_area_boundary_contact_evidence_from_graph(graph, left, right)?
    else {
        return Ok(None);
    };
    let Some(result) = materialize_closed_boundary_touching_regularized_result_from_evidence(
        left, right, operation, validation, &evidence,
    )?
    else {
        return Ok(None);
    };
    Ok(Some((result, evidence)))
}

/// Certify and materialize a named regularized boolean for zero-area closed
/// boundary contact, returning the exact coplanar volumetric evidence consumed.
///
/// evidence proves the contact is boundary-only and has no positive-area
/// coplanar overlap, separating this zero-area shortcut from the positive-area
/// no-volume-overlap materializer.
fn materialize_closed_boundary_touching_regularized_with_evidence_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<(ExactBooleanResult, CoplanarVolumetricCellEvidenceReport)>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    let Some((result, evidence)) =
        materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
            &graph, left, right, operation, validation,
        )?
    else {
        return Ok(None);
    };
    if result
        .validate_operation_against_sources(
            left,
            right,
            operation,
            validation,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .is_err()
        || evidence.validate_against_sources(left, right).is_err()
    {
        return Ok(None);
    }
    Ok(Some((result, evidence)))
}

fn validate_consumed_boundary_touching_report(
    report: &ExactBoundaryTouchingReport,
    left: &ExactMesh,
    right: &ExactMesh,
    label: &str,
) -> Result<(), MeshError> {
    report
        .validate_against_sources(left, right)
        .map_err(|error| {
            MeshError::one(MeshDiagnostic::new(
                Severity::Error,
                DiagnosticKind::UnsupportedExactOperation,
                format!("exact {label} consumed invalid certificate: {error:?}"),
            ))
        })
}

fn materialize_boundary_policy_shortcut_result(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<ExactBooleanResult>, MeshError> {
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
    Ok(Some(boundary_policy_shortcut_result(mesh, operation)))
}

pub(crate) fn boundary_policy_shortcut_result_matches_sources(
    result: &ExactBooleanResult,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> bool {
    if boundary_policy != ExactBoundaryBooleanPolicy::PreserveSeparateShells {
        return false;
    }
    let Ok(report) = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
        .boundary_touching_report(left, right)
    else {
        return false;
    };
    if report.validate().is_err() || !report.is_certified() {
        return false;
    }
    if !boundary_policy_shortcut_result_consumes_report(
        result,
        left,
        right,
        operation,
        boundary_policy,
        &report,
    ) {
        return false;
    }
    let Ok(Some(expected)) =
        materialize_boundary_policy_shortcut_result(left, right, operation, validation)
    else {
        return false;
    };
    expected.validate().is_ok() && result == &expected
}

fn boundary_policy_shortcut_result_consumes_report(
    result: &ExactBooleanResult,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    boundary_policy: ExactBoundaryBooleanPolicy,
    report: &ExactBoundaryTouchingReport,
) -> bool {
    boundary_policy == ExactBoundaryBooleanPolicy::PreserveSeparateShells
        && report.validate().is_ok()
        && report.is_certified()
        && report.validate_against_sources(left, right).is_ok()
        && matches!(
            result.kind,
            ExactBooleanResultKind::BoundaryPolicyShortcut {
                operation: result_operation
            } if result_operation == operation
        )
        && result.validate().is_ok()
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
    validate_consumed_boundary_touching_report(&report, left, right, "boundary-policy projection")?;

    let Some(result) =
        materialize_boundary_policy_shortcut_result(left, right, operation, validation)?
    else {
        return Ok(None);
    };
    Ok(boundary_policy_shortcut_result_consumes_report(
        &result,
        left,
        right,
        operation,
        boundary_policy,
        &report,
    )
    .then_some(result))
}

/// Certify and materialize a named boolean for exact boundary-only contact
/// under an explicit boundary-output policy.
///
/// This is the direct materializer form of
/// [`ExactBooleanRequest::with_boundary_policy`] and [`ExactBooleanRequest::preflight`]: strict boundary-only
/// contact remains blocked until the caller chooses
/// [`ExactBoundaryBooleanPolicy::PreserveSeparateShells`]. Unsupported contact
/// states or rejected policies return `None` rather than projecting
/// lower-dimensional contact implicitly.
fn materialize_boundary_touching_policy_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    let boundary_policy = request.boundary_policy;
    if let Some(result) =
        boolean_closed_validation_regularized_meshes(left, right, operation, validation)?
    {
        return Ok(public_operation_replayable_result(
            Some(result),
            left,
            right,
            operation,
            validation,
            boundary_policy,
        ));
    }
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    if let Some((result, evidence)) =
        materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
            &graph, left, right, operation, validation,
        )?
    {
        if result
            .validate_operation_against_sources(
                left,
                right,
                operation,
                validation,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .is_err()
            || evidence.validate_against_sources(left, right).is_err()
        {
            return Ok(None);
        }
        return Ok(public_operation_replayable_result(
            Some(result),
            left,
            right,
            operation,
            validation,
            boundary_policy,
        ));
    }
    let Some(result) = boolean_boundary_touching_meshes_from_graph(
        &graph,
        left,
        right,
        operation,
        validation,
        boundary_policy,
    )?
    else {
        return Ok(None);
    };
    Ok(public_operation_replayable_result(
        Some(result),
        left,
        right,
        operation,
        validation,
        boundary_policy,
    ))
}

/// Certify whether retained graph pairs are exclusively boundary-only contacts.
///
/// The report keeps the exact graph relation counts used by boundary-policy
/// preflight and by [`ExactBooleanRequest::with_boundary_policy`] and [`ExactBooleanRequest::materialize`]. Boundary-only
/// topology is intentionally not silently materialized by the default named
/// triangle-mesh-only result to be an explicit caller policy.
fn boundary_touching_report_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    _request: ExactBooleanRequest,
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
fn planar_arrangement_report_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactPlanarArrangementReport, MeshError> {
    let operation = request.operation;
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(planar_arrangement_report(
            operation,
            ExactPlanarArrangementStatus::NotNamedOperation,
            false,
            0,
            0,
            ExactBooleanBlocker::default(),
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
fn refinement_report_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactRefinementReport, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    Ok(refinement_report_from_graph(&graph, request.operation))
}

/// Prepare and report exact winding handoff facts for explicit output
/// validation and boundary-only projection policies.
///
/// The strict readiness report keeps boundary-only contact blocked on
/// [`ExactWindingReadinessStatus::BoundaryPolicyRequired`]. This policy-aware
/// variant mirrors [`ExactBooleanRequest::with_boundary_policy`] and [`ExactBooleanRequest::preflight`]: when the
/// caller supplies [`ExactBoundaryBooleanPolicy::PreserveSeparateShells`] and
/// the retained graph certifies boundary-only contact, the winding handoff is
/// marked as already materialized by that explicit projection policy.
fn winding_readiness_report_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<ExactWindingReadinessReport, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    if request.validation == ValidationPolicy::ALLOW_BOUNDARY
        && request.boundary_policy == ExactBoundaryBooleanPolicy::Reject
    {
        return winding_readiness_report_with_shortcuts_from_graph(
            &graph,
            left,
            right,
            request.operation,
        );
    }
    winding_readiness_report_with_boundary_policy_from_graph(
        &graph,
        left,
        right,
        request.operation,
        request.validation,
        request.boundary_policy,
    )
}

fn winding_readiness_report_with_boundary_policy_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
    boundary_policy: ExactBoundaryBooleanPolicy,
) -> Result<ExactWindingReadinessReport, MeshError> {
    let readiness = winding_readiness_report_with_validation_from_graph(
        graph, left, right, operation, validation,
    )?;
    if boundary_policy == ExactBoundaryBooleanPolicy::Reject
        || matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || readiness.status != ExactWindingReadinessStatus::BoundaryPolicyRequired
    {
        return Ok(readiness);
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
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::BoundaryPolicyShortcutAlreadyMaterialized,
            graph.has_unknowns(),
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsBoundaryPolicy),
            None,
            None,
        ));
    }
    Ok(readiness)
}

fn winding_readiness_report_with_validation_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactWindingReadinessReport, MeshError> {
    if validation == ValidationPolicy::CLOSED
        && !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && let Some(support) = certified_closed_validation_regularized_solid_support(left, right)
    {
        let status = match support {
            ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid => {
                ExactWindingReadinessStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized
            }
            ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid => {
                ExactWindingReadinessStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized
            }
            _ => unreachable!("closed validation gate only certifies regularized support"),
        };
        return Ok(winding_readiness_report(
            operation,
            status,
            false,
            0,
            0,
            0,
            Vec::new(),
            ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::NeedsWinding),
            None,
            None,
        ));
    }

    let readiness =
        winding_readiness_report_with_shortcuts_from_graph(graph, left, right, operation)?;
    if validation == ValidationPolicy::CLOSED
        || matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || !matches!(
            readiness.status,
            ExactWindingReadinessStatus::VolumetricAssemblyRequired
                | ExactWindingReadinessStatus::CoplanarVolumetricCellsRequired
        )
    {
        return Ok(readiness);
    }

    if boolean_arrangement_volumetric_split_cell_recovery_from_graph(
        graph, left, right, operation, validation,
    )?
    .is_some()
    {
        let counts = retained_graph_counts(graph);
        let needs_coplanar_volumetric =
            graph_requires_coplanar_volumetric_cells_for_sources(graph, left, right);
        let blocker_kind = if needs_coplanar_volumetric {
            ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells
        } else {
            ExactBooleanBlockerKind::NeedsWinding
        };
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
            graph.has_unknowns(),
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(blocker_kind),
            None,
            if needs_coplanar_volumetric {
                coplanar_volumetric_evidence_if_required(graph, left, right)
            } else {
                None
            },
        ));
    }
    Ok(readiness)
}

fn winding_readiness_report_with_shortcuts_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Result<ExactWindingReadinessReport, MeshError> {
    if let Some(report) = winding_readiness_shortcut_report(left, right, operation) {
        return Ok(report);
    }
    winding_readiness_report_from_graph(graph, left, right, operation)
}

fn winding_readiness_shortcut_report(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactWindingReadinessReport> {
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && (left.triangles().is_empty() || right.triangles().is_empty())
    {
        return Some(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::EmptyOperandAlreadyMaterialized,
            false,
            0,
            0,
            0,
            Vec::new(),
            ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::NeedsWinding),
            None,
            None,
        ));
    }
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && meshes_are_certified_bounds_disjoint(left, right)
    {
        return Some(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::BoundsDisjointAlreadyMaterialized,
            false,
            0,
            0,
            0,
            Vec::new(),
            ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::NeedsWinding),
            None,
            None,
        ));
    }
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && (!left.facts().mesh.closed_manifold || !right.facts().mesh.closed_manifold)
        && (meshes_are_certified_identical(left, right)
            || meshes_are_certified_same_surface(left, right))
    {
        return Some(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::SurfaceEqualityAlreadyMaterialized,
            false,
            0,
            0,
            0,
            Vec::new(),
            ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::NeedsWinding),
            None,
            None,
        ));
    }
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && certified_mixed_dimensional_regularized_solid_support(left, right).is_some()
    {
        return Some(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized,
            false,
            0,
            0,
            0,
            Vec::new(),
            ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::NeedsWinding),
            None,
            None,
        ));
    }
    None
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

fn retained_graph_counts(graph: &super::graph::ExactIntersectionGraph) -> ExactBooleanBlocker {
    ExactBooleanBlocker::from_graph_counts(graph, ExactBooleanBlockerKind::NeedsWinding)
}

pub(crate) fn boundary_touching_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactBoundaryTouchingReport, MeshError> {
    let graph_had_unknowns = graph.has_unknowns();
    let counts = retained_graph_counts(graph);
    let status = if graph_had_unknowns {
        ExactBoundaryTouchingStatus::GraphUnknowns
    } else if graph_requires_boundary_policy(graph, left, right)? {
        ExactBoundaryTouchingStatus::Certified
    } else {
        ExactBoundaryTouchingStatus::NotBoundaryOnly
    };
    let blocker_kind = boundary_touching_blocker_kind(&status, counts);
    Ok(ExactBoundaryTouchingReport {
        status,
        graph_had_unknowns,
        retained_face_pairs: graph.face_pairs.len(),
        retained_events: graph.event_count(),
        blocker: counts.into_blocker(blocker_kind),
    })
}

fn boundary_touching_blocker_kind(
    status: &ExactBoundaryTouchingStatus,
    counts: ExactBooleanBlocker,
) -> ExactBooleanBlockerKind {
    match status {
        ExactBoundaryTouchingStatus::GraphUnknowns => ExactBooleanBlockerKind::NeedsRefinement,
        ExactBoundaryTouchingStatus::Certified => ExactBooleanBlockerKind::NeedsBoundaryPolicy,
        ExactBoundaryTouchingStatus::NotBoundaryOnly => retained_graph_blocker_kind(counts),
    }
}

fn refinement_report_from_graph(
    graph: &super::graph::ExactIntersectionGraph,
    operation: ExactBooleanOperation,
) -> ExactRefinementReport {
    let counts = retained_graph_counts(graph);
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
    let counts = retained_graph_counts(graph);
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
    counts: ExactBooleanBlocker,
    arrangement_readiness: Option<super::graph::CoplanarArrangementReadinessReport>,
) -> ExactPlanarArrangementReport {
    let blocker_kind = match status {
        ExactPlanarArrangementStatus::GraphUnknowns => ExactBooleanBlockerKind::NeedsRefinement,
        ExactPlanarArrangementStatus::BoundaryPolicyRequired => {
            ExactBooleanBlockerKind::NeedsBoundaryPolicy
        }
        ExactPlanarArrangementStatus::Required => ExactBooleanBlockerKind::NeedsPlanarArrangement,
        ExactPlanarArrangementStatus::NotNamedOperation
        | ExactPlanarArrangementStatus::AlreadyMaterialized
        | ExactPlanarArrangementStatus::NoPositiveOverlap => retained_graph_blocker_kind(counts),
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
    let counts = retained_graph_counts(graph);
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        let blocker_kind = retained_graph_blocker_kind(counts);
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::NotNamedOperation,
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
    if preflight_tail_shortcut_support(left, right, operation)
        != Some(ExactBooleanSupport::CertifiedArrangementCellComplex)
        && certified_convex_materialized_boolean_support(left, right, operation).is_some()
    {
        let blocker = counts.into_blocker(ExactBooleanBlockerKind::NeedsWinding);
        let (retained_face_pairs, retained_events, blocker) = if blocker
            .validate_for_kind(ExactBooleanBlockerKind::NeedsWinding)
            .is_ok()
        {
            (graph.face_pairs.len(), graph.event_count(), blocker)
        } else {
            (
                0,
                0,
                ExactBooleanBlocker::default().into_blocker(ExactBooleanBlockerKind::NeedsWinding),
            )
        };
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::ConvexBooleanAlreadyMaterialized,
            graph_had_unknowns,
            retained_face_pairs,
            retained_events,
            0,
            Vec::new(),
            blocker,
            None,
            None,
        ));
    }
    if certified_open_surface_disjoint_support_from_graph(graph, left, right, operation).is_some() {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::OpenSurfaceDisjointAlreadyMaterialized,
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
    if let Some((_support, region_classifications, _triangulations)) =
        open_surface_arrangement_plan_from_graph(graph, left, right, operation)?
    {
        let region_count = unique_classified_region_count(&region_classifications);
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::OpenSurfaceArrangementAlreadyMaterialized,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            region_count,
            region_classifications,
            counts.into_blocker(ExactBooleanBlockerKind::NeedsWinding),
            None,
            None,
        ));
    }
    let tail_shortcut_materializes = preflight_tail_shortcut_support(left, right, operation)
        == Some(ExactBooleanSupport::CertifiedArrangementCellComplex);
    let boundary_policy_required = graph_requires_boundary_policy(graph, left, right)?;
    if let Some(_support) = certified_closed_zero_area_boundary_contact_support_from_graph(
        graph, left, right, operation,
    )? {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::ClosedBoundaryTouchingAlreadyMaterialized,
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
    if !tail_shortcut_materializes
        && matches!(
            operation,
            ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
        )
        && certified_closed_boundary_only_contact_from_graph(graph, left, right)?
    {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::ClosedBoundaryTouchingAlreadyMaterialized,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(ExactBooleanBlockerKind::NeedsBoundaryPolicy),
            None,
            coplanar_boundary_only_evidence_if_consumed(graph, left, right)?,
        ));
    }
    if tail_shortcut_materializes && boundary_policy_required {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
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
    if boundary_policy_required {
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
    if tail_shortcut_materializes {
        let needs_coplanar_volumetric =
            graph_requires_coplanar_volumetric_cells_for_sources(graph, left, right);
        let blocker_kind = if needs_coplanar_volumetric {
            ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells
        } else {
            ExactBooleanBlockerKind::NeedsWinding
        };
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
            graph_had_unknowns,
            graph.face_pairs.len(),
            graph.event_count(),
            0,
            Vec::new(),
            counts.into_blocker(blocker_kind),
            None,
            if needs_coplanar_volumetric {
                coplanar_volumetric_evidence_if_required(graph, left, right)
            } else {
                None
            },
        ));
    }
    if let Some((region_classifications, triangulations, volumetric_classifications)) =
        volumetric_winding_region_plan_from_graph(graph, left, right)?
    {
        let needs_coplanar_volumetric =
            graph_requires_coplanar_volumetric_cells_for_sources(graph, left, right);
        let blocker_kind = if needs_coplanar_volumetric {
            ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells
        } else {
            ExactBooleanBlockerKind::NeedsWinding
        };
        if let Some(materialized) = materialize_volumetric_winding_region_plan(
            region_classifications.clone(),
            triangulations.clone(),
            volumetric_classifications.clone(),
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
                counts.into_blocker(blocker_kind),
                None,
                coplanar_volumetric_evidence_if_required(graph, left, right),
            ));
        }
        if let Some(materialized) = materialize_volumetric_winding_region_plan(
            region_classifications.clone(),
            triangulations.clone(),
            volumetric_classifications.clone(),
            left,
            right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
        )? && certified_coplanar_boundary_closure_from_materialized(
            &materialized,
            left,
            right,
            operation,
            ValidationPolicy::CLOSED,
        )?
        .is_some()
        {
            return Ok(winding_readiness_report(
                operation,
                ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
                graph_had_unknowns,
                graph.face_pairs.len(),
                graph.event_count(),
                0,
                Vec::new(),
                counts.into_blocker(blocker_kind),
                None,
                coplanar_volumetric_evidence_if_required(graph, left, right),
            ));
        }
        if volumetric_classifications
            .iter()
            .all(|classification| classification.relation.is_materialization_decided())
        {
            let region_count = unique_classified_region_count(&region_classifications);
            return Ok(winding_readiness_report(
                operation,
                ExactWindingReadinessStatus::VolumetricAssemblyRequired,
                graph_had_unknowns,
                graph.face_pairs.len(),
                graph.event_count(),
                region_count,
                region_classifications,
                counts.into_blocker(blocker_kind),
                None,
                coplanar_volumetric_evidence_if_required(graph, left, right),
            ));
        }
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
    if certified_closed_winding_separated_support_from_graph(graph, left, right, operation)?
        .is_some()
    {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::ClosedWindingSeparatedAlreadyMaterialized,
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
    if certified_closed_winding_containment_support_from_graph(graph, left, right, operation)?
        .is_some()
    {
        return Ok(winding_readiness_report(
            operation,
            ExactWindingReadinessStatus::ClosedWindingContainmentAlreadyMaterialized,
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
    if graph.face_pairs.is_empty() {
        if !meshes_are_certified_bounds_disjoint(left, right)
            && certified_arrangement_cell_complex_preflight_if_materialized(
                operation, graph, left, right,
            )?
            .is_some()
        {
            return Ok(winding_readiness_report(
                operation,
                ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
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

fn retained_graph_blocker_kind(counts: ExactBooleanBlocker) -> ExactBooleanBlockerKind {
    if counts.unknown_pairs > 0 || counts.construction_failed_events > 0 {
        ExactBooleanBlockerKind::NeedsRefinement
    } else if counts.coplanar_overlapping_pairs + counts.coplanar_touching_pairs > 0 {
        if counts.candidate_pairs == 0 && counts.coplanar_overlapping_pairs > 0 {
            ExactBooleanBlockerKind::NeedsPlanarArrangement
        } else if counts.candidate_pairs == 0 {
            ExactBooleanBlockerKind::NeedsBoundaryPolicy
        } else {
            ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells
        }
    } else {
        ExactBooleanBlockerKind::NeedsWinding
    }
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
) -> Result<Option<ExactMesh>, MeshError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let Some(materialized) = materialize_volumetric_winding_region_plan_from_graph(
        graph,
        left,
        right,
        operation,
        ValidationPolicy::ALLOW_BOUNDARY,
    )?
    else {
        return Ok(None);
    };
    certified_coplanar_boundary_closure_from_materialized(
        &materialized,
        left,
        right,
        operation,
        ValidationPolicy::CLOSED,
    )
}

fn materialize_volumetric_winding_region_plan(
    region_classifications: Vec<FaceRegionPlaneClassification>,
    triangulations: Vec<FaceRegionTriangulation>,
    volumetric_classifications: Vec<ExactVolumetricRegionClassification>,
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<MaterializedVolumetricWindingRegionPlan>, MeshError> {
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
) -> Result<Option<VolumetricWindingRegionPlan>, MeshError> {
    let counts = retained_graph_counts(graph);
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

pub(crate) fn replay_materialized_volumetric_winding_region_plan(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<Option<MaterializedVolumetricWindingRegionPlan>, MeshError> {
    let graph = build_intersection_graph(left, right)?;
    validate_graph_source_handoff(&graph, left, right)?;
    materialize_volumetric_winding_region_plan_from_graph(
        &graph, left, right, operation, validation,
    )
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

fn certified_convex_materialized_boolean_support(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    certified_convex_union_support(left, right, operation)
        .or_else(|| certified_convex_intersection_support(left, right, operation))
        .or_else(|| certified_convex_difference_support(left, right, operation))
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
    let relation_counts = retained_graph_counts(graph);
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
    ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::ALLOW_BOUNDARY)
        .identical_mesh_report(left, right)
        .is_certified()
}

fn meshes_are_certified_same_surface(left: &ExactMesh, right: &ExactMesh) -> bool {
    ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::ALLOW_BOUNDARY)
        .same_surface_report(left, right)
        .is_certified()
}

/// Certify whether two meshes represent the same triangle surface.
///
/// The report preserves the exact coordinate-equality predicate certificates
/// used to find a vertex bijection and the sorted triangle sets compared after
/// remapping. This is the auditable form of the same-surface shortcut used by
/// expose the predicate facts that justify them.
fn same_surface_report_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    _request: ExactBooleanRequest,
) -> ExactSameSurfaceReport {
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

/// Certify whether two meshes are exactly identical in source vertex and
/// triangle order.
fn identical_mesh_report_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    _request: ExactBooleanRequest,
) -> ExactIdenticalMeshReport {
    let mut predicates = Vec::new();
    if left.vertices().len() != right.vertices().len() {
        return identical_mesh_report(
            ExactIdenticalMeshStatus::VertexCountMismatch,
            left,
            right,
            predicates,
        );
    }

    for (left_vertex, right_vertex) in left.vertices().iter().zip(right.vertices()) {
        let x = compare_reals_report(&left_vertex.x, &right_vertex.x);
        let y = compare_reals_report(&left_vertex.y, &right_vertex.y);
        let z = compare_reals_report(&left_vertex.z, &right_vertex.z);
        predicates.push(PredicateUse::from_certificate(x.certificate));
        predicates.push(PredicateUse::from_certificate(y.certificate));
        predicates.push(PredicateUse::from_certificate(z.certificate));
        let Some(x_value) = x.outcome.value() else {
            return identical_mesh_report(
                ExactIdenticalMeshStatus::VertexCoordinateUndecided,
                left,
                right,
                predicates,
            );
        };
        let Some(y_value) = y.outcome.value() else {
            return identical_mesh_report(
                ExactIdenticalMeshStatus::VertexCoordinateUndecided,
                left,
                right,
                predicates,
            );
        };
        let Some(z_value) = z.outcome.value() else {
            return identical_mesh_report(
                ExactIdenticalMeshStatus::VertexCoordinateUndecided,
                left,
                right,
                predicates,
            );
        };
        if x_value != Ordering::Equal || y_value != Ordering::Equal || z_value != Ordering::Equal {
            return identical_mesh_report(
                ExactIdenticalMeshStatus::VertexCoordinateMismatch,
                left,
                right,
                predicates,
            );
        }
    }

    let status = if left.triangles() == right.triangles() {
        ExactIdenticalMeshStatus::Certified
    } else {
        ExactIdenticalMeshStatus::TriangleSequenceMismatch
    };
    identical_mesh_report(status, left, right, predicates)
}

fn identical_mesh_report(
    status: ExactIdenticalMeshStatus,
    left: &ExactMesh,
    right: &ExactMesh,
    predicates: Vec<PredicateUse>,
) -> ExactIdenticalMeshReport {
    ExactIdenticalMeshReport {
        status,
        left_vertices: left.vertices().len(),
        right_vertices: right.vertices().len(),
        left_triangles: left.triangles().len(),
        right_triangles: right.triangles().len(),
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

    Ok(Some(certified_shortcut_result(mesh, operation, shortcut)))
}

/// Certify and materialize a named closed-regularized boolean when at least
/// one operand has no closed-volume contribution.
///
/// This exposes the exact regularization path used by [`ExactBooleanRequest::materialize`]: a
/// closed solid combined with a lower-dimensional surface keeps or drops the
/// solid according to the named operation, while two lower-dimensional
/// operands regularize to an empty closed-solid result. Unsupported operands or
/// validation policies return `None` rather than falling back to tolerance
/// geometry.
fn materialize_closed_regularized_lower_dimensional_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    Ok(public_operation_replayable_result(
        boolean_closed_regularized_lower_dimensional_optional(left, right, operation, validation)?,
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    ))
}

/// Certify and materialize a named regularized boolean between a closed solid
/// and a lower-dimensional operand.
///
/// This exposes the mixed-dimensional exact regularization path used by
/// [`ExactBooleanRequest::materialize`]. A closed solid combined with an open surface contributes
/// no additional closed volume: union keeps the solid, intersection is empty,
/// and difference keeps or drops the solid according to operand order.
/// Lower-dimensional-only inputs return `None` here so the lower-dimensional
/// regularized materializer keeps that provenance.
fn materialize_mixed_dimensional_regularized_solid_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || certified_mixed_dimensional_regularized_solid_support(left, right).is_none()
        || (validation != ValidationPolicy::CLOSED
            && meshes_are_certified_bounds_disjoint(left, right))
    {
        return Ok(None);
    }
    Ok(public_operation_replayable_result(
        boolean_closed_regularized_lower_dimensional_optional(left, right, operation, validation)?,
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    ))
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
        operation,
        ExactBooleanShortcutKind::BoundsDisjoint,
    ))
}

/// Certify and materialize a named boolean for operands whose exact mesh bounds
/// are disjoint.
///
/// The AABB intersection classification is an exact retained fact. Unsupported
/// selected-region operations or sources whose bounds are not certified
/// disjoint return `None` rather than invoking later topology paths.
fn materialize_bounds_disjoint_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || left.triangles().is_empty()
        || right.triangles().is_empty()
        || !meshes_are_certified_bounds_disjoint(left, right)
        || (validation == ValidationPolicy::CLOSED
            && certified_closed_validation_regularized_solid_support(left, right).is_some())
    {
        return Ok(None);
    }
    Ok(public_operation_replayable_result(
        Some(boolean_disjoint_meshes(left, right, operation, validation)?),
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
    ))
}

fn boolean_empty_operand(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
) -> Result<ExactBooleanResult, MeshError> {
    let mesh = match operation {
        ExactBooleanOperation::Union
            if empty_operand_union_regularizes_to_empty_closed_output(left, right, validation) =>
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
            if empty_right_difference_regularizes_to_empty_closed_output(
                left, right, validation,
            ) =>
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
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled by caller"),
    };

    Ok(certified_shortcut_result(
        mesh,
        operation,
        ExactBooleanShortcutKind::EmptyOperand,
    ))
}

fn empty_operand_union_regularizes_to_empty_closed_output(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> bool {
    validation == ValidationPolicy::CLOSED
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
        )
}

fn empty_right_difference_regularizes_to_empty_closed_output(
    left: &ExactMesh,
    right: &ExactMesh,
    validation: ValidationPolicy,
) -> bool {
    validation == ValidationPolicy::CLOSED
        && right.triangles().is_empty()
        && closed_regularized_operand_kind(left)
            == Some(ClosedRegularizedOperandKind::LowerDimensional)
}

/// Certify and materialize a named boolean when either operand has no
/// triangles.
///
/// Empty-source handling is the first exact named-boolean shortcut in the
/// dispatcher. Unsupported selected-region operations or non-empty operand
/// pairs return `None`.
fn materialize_empty_operand_for_request(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<Option<ExactBooleanResult>, MeshError> {
    let operation = request.operation;
    let validation = request.validation;
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || (!left.triangles().is_empty() && !right.triangles().is_empty())
    {
        return Ok(None);
    }
    Ok(public_operation_replayable_result(
        Some(boolean_empty_operand(left, right, operation, validation)?),
        left,
        right,
        operation,
        validation,
        ExactBoundaryBooleanPolicy::Reject,
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
        operation,
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

    fn assert_current_arrangement_attempt(
        attempt: &ExactArrangementBooleanAttempt,
        left: &ExactMesh,
        right: &ExactMesh,
    ) {
        attempt.validate().unwrap();
        attempt.validate_against_sources(left, right).unwrap();
        assert_eq!(
            attempt.freshness_against_sources(left, right),
            ExactReportFreshness::Current,
            "{attempt:?}"
        );
    }

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
    fn boundary_policy_shortcut_rejects_selected_region_operation_relabel() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 2, 0, 0, 0, 2, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 0, 0, 0, 2, 0, 2, 2, 2],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let mut projected = ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .materialize(&left, &right)
        .unwrap();
        projected.validate_against_sources(&left, &right).unwrap();
        let mut stale_mesh = projected.clone();
        stale_mesh.mesh = empty_mesh(
            "empty exact stale boundary-policy projection",
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        stale_mesh.validate().unwrap();
        assert!(stale_mesh.validate_against_sources(&left, &right).is_err());
        projected.kind = ExactBooleanResultKind::BoundaryPolicyShortcut {
            operation: ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll),
        };
        assert_eq!(
            projected.validate(),
            Err(crate::ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn open_surface_disjoint_graph_shortcut_replays_sources_before_acceptance() {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 4, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let separated_right = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 1, 4, 0, 5, 0, 4, 1],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let overlapping_right = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 4, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        let stale_graph = build_intersection_graph(&left, &separated_right).unwrap();
        assert!(
            boolean_open_surface_disjoint_meshes_from_graph(
                &stale_graph,
                &left,
                &separated_right,
                ExactBooleanOperation::Union,
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap()
            .is_some()
        );

        assert!(
            boolean_open_surface_disjoint_meshes_from_graph(
                &stale_graph,
                &left,
                &overlapping_right,
                ExactBooleanOperation::Union,
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap()
            .is_none()
        );
    }

    #[test]
    fn coplanar_volumetric_gate_uses_source_side_evidence() {
        let boundary_left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let boundary_right = axis_aligned_box_i64([2, 0, 0], [4, 2, 2]);
        let boundary_graph = build_intersection_graph(&boundary_left, &boundary_right).unwrap();
        assert!(graph_requires_coplanar_volumetric_cells(
            &ExactBooleanBlocker::from_graph_counts(
                &boundary_graph,
                ExactBooleanBlockerKind::NeedsWinding
            )
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
    fn exact_boolean_blocker_counts_include_unknown_segment_plane_events() {
        let graph = super::super::graph::ExactIntersectionGraph {
            face_pairs: vec![FacePairEvents {
                left_face: 0,
                right_face: 0,
                relation: MeshFacePairRelation::Candidate,
                projection: None,
                events: vec![IntersectionEvent::SegmentPlane {
                    segment_side: MeshSide::Left,
                    edge: [0, 1],
                    plane_side: MeshSide::Right,
                    plane_face: 0,
                    relation: SegmentPlaneRelation::Unknown,
                    point: None,
                    parameter: None,
                    parameter_ratio: None,
                    construction_failure: None,
                    endpoint_sides: [None, Some(hyperlimit::PlaneSide::Above)],
                }],
            }],
        };

        let counts = retained_graph_counts(&graph);
        assert_eq!(counts.candidate_pairs, 1);
        assert_eq!(counts.unknown_pairs, 1);
        assert_eq!(
            counts.into_blocker(ExactBooleanBlockerKind::NeedsRefinement),
            ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::NeedsRefinement,
                candidate_pairs: 1,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 1,
                construction_failed_events: 0,
            }
        );
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
        assert_eq!(
            selected_faces.facts().mesh.boundary_edges,
            canonical.facts().mesh.boundary_edges
        );

        let readiness =
            ExactBooleanRequest::with_boundary_policy(ExactBooleanOperation::Difference, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right)
                .unwrap();
        assert_eq!(
            readiness.status,
            ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized,
            "{readiness:?}"
        );
        assert_eq!(
            readiness.blocker.kind,
            ExactBooleanBlockerKind::NeedsPlanarArrangement
        );
        readiness.validate().unwrap();
        readiness.validate_against_sources(&left, &right).unwrap();
    }

    #[test]
    fn selected_region_winding_readiness_classifies_retained_graph_blocker() {
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
        let readiness = ExactBooleanRequest::with_boundary_policy(ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll), ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right)
        .unwrap();
        assert_eq!(
            readiness.status,
            ExactWindingReadinessStatus::NotNamedOperation
        );
        assert_eq!(
            readiness.blocker.kind,
            ExactBooleanBlockerKind::NeedsPlanarArrangement
        );
        assert_eq!(readiness.blocker.coplanar_overlapping_pairs, 1);
        assert_eq!(readiness.blocker.coplanar_touching_pairs, 2);
        readiness.validate().unwrap();
        readiness.validate_against_sources(&left, &right).unwrap();

        let mut stale = readiness.clone();
        stale.blocker.kind = ExactBooleanBlockerKind::NeedsWinding;
        assert_eq!(
            stale.validate(),
            Err(crate::ExactReportValidationError::WrongBlockerKind)
        );

        let disjoint_right = ExactMesh::from_i64_triangles_with_policy(
            &[8, 0, 0, 12, 0, 0, 12, 4, 0, 8, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let disjoint_readiness = ExactBooleanRequest::with_boundary_policy(ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll), ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &disjoint_right)
        .unwrap();
        assert_eq!(
            disjoint_readiness.status,
            ExactWindingReadinessStatus::NotNamedOperation
        );
        assert_eq!(
            disjoint_readiness.blocker.kind,
            ExactBooleanBlockerKind::NeedsWinding
        );
        assert_eq!(disjoint_readiness.retained_face_pairs, 0);
        disjoint_readiness.validate().unwrap();
        disjoint_readiness
            .validate_against_sources(&left, &disjoint_right)
            .unwrap();

        let mut relabeled_empty = disjoint_readiness;
        relabeled_empty.blocker.kind = ExactBooleanBlockerKind::NeedsBoundaryPolicy;
        assert_eq!(
            relabeled_empty.validate(),
            Err(crate::ExactReportValidationError::WrongBlockerKind)
        );
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
    fn selected_overlay_faces_recover_selected_boundary_topology_blockers() {
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
        let canonical = mesh_from_selected_projected_overlay_faces(
            &overlay,
            &carrier_points,
            projection,
            "test canonical selected-boundary topology overlay",
        )
        .expect("complete overlay should materialize");

        overlay.output_loops.clear();
        overlay.output_components.clear();
        overlay
            .blockers
            .push(ExactArrangement2dBlocker::NonManifoldSelectedBoundary { vertex: 0 });

        let recovered = mesh_from_selected_projected_overlay_faces(
            &overlay,
            &carrier_points,
            projection,
            "test recovered selected-boundary topology blocker",
        )
        .expect("selected faces should recover when topology blocker is stale");
        recovered.validate_retained_state().unwrap();
        assert!(exact_meshes_have_same_shape(&recovered, &canonical));
        assert_eq!(
            recovered.facts().mesh.boundary_edges,
            canonical.facts().mesh.boundary_edges
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
        let preflight = ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .preflight(&left, &opening_plus_hole)
        .unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{preflight:?}"
        );
        assert!(preflight.blocker.is_none(), "{preflight:?}");
        preflight
            .validate_against_sources(&left, &opening_plus_hole)
            .unwrap();
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
                operation: ExactBooleanOperation::Difference,
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
        let disjoint_holes = ExactMesh::from_i64_triangles_with_policy(
            &[
                20, 20, 0, 22, 20, 0, 22, 22, 0, 20, 22, 0, //
                22, 22, 0, 24, 22, 0, 24, 24, 0, 22, 24, 0,
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
                operation: ExactBooleanOperation::Difference,
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        result.mesh.validate_retained_state().unwrap();
        result
            .validate_against_sources(&left, &touching_holes)
            .unwrap();
        assert!(
            result
                .validate_against_sources(&left, &disjoint_holes)
                .is_err(),
            "{result:?}"
        );
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
        let union = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .materialize(&inner, &left)
        .expect("contained coplanar union should materialize through arrangement");
        assert_eq!(
            union.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Union,
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

    #[test]
    fn axis_aligned_orthogonal_cell_booleans_materialize_from_shortcut_support() {
        let left = axis_aligned_orthogonal_l_solid_i64();
        let right = axis_aligned_box_i64([1, 0, 0], [3, 1, 1]);

        assert!(!is_axis_aligned_box(&left));

        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            assert_eq!(
                preflight_tail_shortcut_support(&left, &right, operation),
                Some(ExactBooleanSupport::CertifiedArrangementCellComplex),
                "{operation:?}"
            );

            let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .preflight(&left, &right)
                .unwrap();
            assert_eq!(
                preflight.support,
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                "{operation:?}: {preflight:?}"
            );
            assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");
            let mut stale_preflight = preflight.clone();
            stale_preflight
                .coplanar_volumetric_evidence
                .as_mut()
                .expect("orthogonal overlap should retain consumed coplanar-cell evidence")
                .retained_face_pair_count += 1;
            assert!(
                stale_preflight.validate().is_err(),
                "{operation:?}: {stale_preflight:?}"
            );

            let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            assert!(winding_readiness_status_already_materialized(
                &readiness.status
            ));
            assert_eq!(
                readiness.blocker.kind,
                ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells,
                "{operation:?}: {readiness:?}"
            );
            readiness.validate().unwrap();
            readiness.validate_against_sources(&left, &right).unwrap();

            let planar = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .planar_arrangement_report(&left, &right)
                .unwrap();
            planar.validate().unwrap();
            planar.validate_against_sources(&left, &right).unwrap();

            let direct = boolean_arrangement_orthogonal_solid_cell_recovery(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
            )
            .unwrap()
            .expect("orthogonal cell shortcut should materialize directly");
            direct.validate().unwrap();
            direct.validate_against_sources(&left, &right).unwrap();

            let attempt = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();
            attempt.validate().unwrap();
            attempt.validate_against_sources(&left, &right).unwrap();
            assert_eq!(
                attempt.stage,
                ExactArrangementBooleanStage::Materialized,
                "{operation:?}: {attempt:?}"
            );
            assert_eq!(
                attempt.materialized_shortcut,
                Some(ExactBooleanShortcutKind::ArrangementCellComplex),
                "{operation:?}: {attempt:?}"
            );

            let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .expect("certified orthogonal cell support should materialize");
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
                },
                "{operation:?}: {result:?}"
            );
            result.validate().unwrap();
            result.validate_against_sources(&left, &right).unwrap();
            assert!(
                result.mesh.facts().mesh.closed_manifold || result.mesh.triangles().is_empty(),
                "{operation:?}: {:?}",
                result.mesh.facts().mesh
            );
        }
    }

    #[test]
    fn affine_box_booleans_materialize_from_certified_preflight_support() {
        let left = affine_box_i64([0, 0, 0], [2, 2, 2]);
        let right = affine_box_i64([1, 0, 0], [3, 2, 2]);

        assert!(has_affine_box_union(&left, &right));
        assert!(has_affine_box_intersection(&left, &right));
        assert!(has_affine_box_difference(&left, &right));

        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .preflight(&left, &right)
                .unwrap();
            assert_eq!(
                preflight.support,
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                "{operation:?}: {preflight:?}"
            );
            assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");

            let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .expect("certified affine-box support should materialize");
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
                },
                "{operation:?}: {result:?}"
            );
            result.validate().unwrap();
            result.validate_against_sources(&left, &right).unwrap();
            assert!(
                result.mesh.facts().mesh.closed_manifold,
                "{operation:?}: {:?}",
                result.mesh.facts().mesh
            );
            assert!(
                !result.mesh.triangles().is_empty(),
                "{operation:?}: {result:?}"
            );
        }
    }

    #[test]
    fn affine_empty_intersection_materializes_without_winding_fallback() {
        let left = skew_affine_box_i64([0, 0, 0], [1, 1, 1]);
        let right = skew_affine_box_i64([2, 0, 0], [3, 1, 1]);

        assert!(!meshes_are_certified_bounds_disjoint(&left, &right));
        assert!(has_empty_affine_orthogonal_solid_cell_intersection(
            &left, &right
        ));
        assert!(has_affine_box_union(&left, &right));
        assert!(has_affine_box_intersection(&left, &right));
        assert!(has_affine_box_difference(&left, &right));

        let preflight = ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .preflight(&left, &right)
        .unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{preflight:?}"
        );
        assert!(preflight.blocker.is_none(), "{preflight:?}");

        let result = ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        )
        .materialize(&left, &right)
        .expect("empty affine-normalized intersection should materialize");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Intersection,
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert!(result.mesh.triangles().is_empty());
    }

    #[test]
    fn affine_shortcut_winding_report_retains_already_materialized_status() {
        let left = skew_affine_box_i64([0, 0, 0], [2, 2, 2]);
        let right = skew_affine_box_i64([1, 1, 1], [3, 3, 3]);

        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .preflight(&left, &right)
                .unwrap();
            assert_eq!(
                preflight.support,
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                "{operation:?}: {preflight:?}"
            );

            let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            assert!(winding_readiness_status_already_materialized(
                &readiness.status
            ));
            assert_eq!(
                readiness.blocker.kind,
                ExactBooleanBlockerKind::NeedsWinding,
                "{operation:?}: {readiness:?}"
            );
            readiness.validate().unwrap();
            readiness.validate_against_sources(&left, &right).unwrap();

            let planar = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .planar_arrangement_report(&left, &right)
                .unwrap();
            planar.validate().unwrap();
            planar.validate_against_sources(&left, &right).unwrap();
        }
    }

    #[test]
    fn winding_readiness_status_partition_identifies_materialized_handoffs() {
        for status in [
            ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized,
            ExactWindingReadinessStatus::CoplanarVolumetricCellsAlreadyMaterialized,
            ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
            ExactWindingReadinessStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized,
            ExactWindingReadinessStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized,
            ExactWindingReadinessStatus::ConvexBooleanAlreadyMaterialized,
            ExactWindingReadinessStatus::OpenSurfaceArrangementAlreadyMaterialized,
            ExactWindingReadinessStatus::SurfaceEqualityAlreadyMaterialized,
            ExactWindingReadinessStatus::ClosedBoundaryTouchingAlreadyMaterialized,
            ExactWindingReadinessStatus::BoundaryPolicyShortcutAlreadyMaterialized,
            ExactWindingReadinessStatus::EmptyOperandAlreadyMaterialized,
            ExactWindingReadinessStatus::BoundsDisjointAlreadyMaterialized,
            ExactWindingReadinessStatus::OpenSurfaceDisjointAlreadyMaterialized,
            ExactWindingReadinessStatus::ClosedWindingSeparatedAlreadyMaterialized,
            ExactWindingReadinessStatus::ClosedWindingContainmentAlreadyMaterialized,
        ] {
            assert!(winding_readiness_status_already_materialized(&status));
        }

        for status in [
            ExactWindingReadinessStatus::NotNamedOperation,
            ExactWindingReadinessStatus::GraphUnknowns,
            ExactWindingReadinessStatus::BoundaryPolicyRequired,
            ExactWindingReadinessStatus::PlanarArrangementRequired,
            ExactWindingReadinessStatus::CoplanarVolumetricCellsRequired,
            ExactWindingReadinessStatus::VolumetricAssemblyRequired,
            ExactWindingReadinessStatus::NoNontrivialOverlap,
            ExactWindingReadinessStatus::Ready,
        ] {
            assert!(!winding_readiness_status_already_materialized(&status));
        }

        for status in [
            ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized,
            ExactWindingReadinessStatus::CoplanarVolumetricCellsAlreadyMaterialized,
            ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
        ] {
            assert!(winding_readiness_status_materializes_arrangement_cell_complex(&status));
        }

        assert!(
            !winding_readiness_status_materializes_arrangement_cell_complex(
                &ExactWindingReadinessStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized,
            )
        );
        assert!(
            !winding_readiness_status_materializes_arrangement_cell_complex(
                &ExactWindingReadinessStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized,
            )
        );
        assert!(
            !winding_readiness_status_materializes_arrangement_cell_complex(
                &ExactWindingReadinessStatus::ConvexBooleanAlreadyMaterialized,
            )
        );
        assert!(
            !winding_readiness_status_materializes_arrangement_cell_complex(
                &ExactWindingReadinessStatus::OpenSurfaceArrangementAlreadyMaterialized,
            )
        );
        assert!(
            !winding_readiness_status_materializes_arrangement_cell_complex(
                &ExactWindingReadinessStatus::SurfaceEqualityAlreadyMaterialized,
            )
        );
        assert!(
            !winding_readiness_status_materializes_arrangement_cell_complex(
                &ExactWindingReadinessStatus::ClosedBoundaryTouchingAlreadyMaterialized,
            )
        );
        assert!(
            !winding_readiness_status_materializes_arrangement_cell_complex(
                &ExactWindingReadinessStatus::BoundaryPolicyShortcutAlreadyMaterialized,
            )
        );
        for status in [
            ExactWindingReadinessStatus::EmptyOperandAlreadyMaterialized,
            ExactWindingReadinessStatus::BoundsDisjointAlreadyMaterialized,
            ExactWindingReadinessStatus::OpenSurfaceDisjointAlreadyMaterialized,
            ExactWindingReadinessStatus::ClosedWindingSeparatedAlreadyMaterialized,
            ExactWindingReadinessStatus::ClosedWindingContainmentAlreadyMaterialized,
        ] {
            assert!(!winding_readiness_status_materializes_arrangement_cell_complex(&status));
        }
    }

    #[test]
    fn trivial_shortcuts_report_materialized_readiness() {
        let empty =
            empty_mesh("empty operand readiness fixture", ValidationPolicy::CLOSED).unwrap();
        let solid = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let far_solid = axis_aligned_box_i64([4, 0, 0], [6, 2, 2]);
        let left_open = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, //
                4, 0, 4, //
                0, 4, 0,
            ],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right_open = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 1, //
                4, 0, 5, //
                0, 4, 1,
            ],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        assert!(!meshes_are_certified_bounds_disjoint(
            &left_open,
            &right_open
        ));

        for (left, right, validation, support, status, shortcut) in [
            (
                &empty,
                &solid,
                ValidationPolicy::CLOSED,
                ExactBooleanSupport::CertifiedEmptyOperand,
                ExactWindingReadinessStatus::EmptyOperandAlreadyMaterialized,
                ExactBooleanShortcutKind::EmptyOperand,
            ),
            (
                &solid,
                &far_solid,
                ValidationPolicy::CLOSED,
                ExactBooleanSupport::CertifiedBoundsDisjoint,
                ExactWindingReadinessStatus::BoundsDisjointAlreadyMaterialized,
                ExactBooleanShortcutKind::BoundsDisjoint,
            ),
            (
                &left_open,
                &right_open,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBooleanSupport::CertifiedOpenSurfaceDisjoint,
                ExactWindingReadinessStatus::OpenSurfaceDisjointAlreadyMaterialized,
                ExactBooleanShortcutKind::OpenSurfaceDisjoint,
            ),
        ] {
            for operation in [
                ExactBooleanOperation::Union,
                ExactBooleanOperation::Intersection,
                ExactBooleanOperation::Difference,
            ] {
                let preflight =
                    ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                        .preflight(left, right)
                        .unwrap();
                assert_eq!(preflight.support, support, "{operation:?}: {preflight:?}");
                assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");

                let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(left, right).unwrap();
                assert_eq!(readiness.status, status, "{operation:?}: {readiness:?}");
                assert_eq!(
                    readiness.blocker.kind,
                    ExactBooleanBlockerKind::NeedsWinding,
                    "{operation:?}: {readiness:?}"
                );
                assert_eq!(readiness.retained_face_pairs, 0, "{operation:?}");
                assert_eq!(readiness.retained_events, 0, "{operation:?}");
                assert_eq!(readiness.region_count, 0, "{operation:?}");
                assert!(winding_readiness_status_already_materialized(
                    &readiness.status
                ));
                assert!(
                    !winding_readiness_status_materializes_arrangement_cell_complex(
                        &readiness.status
                    )
                );
                readiness.validate().unwrap();
                readiness.validate_against_sources(left, right).unwrap();

                let result = ExactBooleanRequest::new(operation, validation)
                    .materialize(left, right)
                    .unwrap();
                assert_eq!(
                    result.kind,
                    ExactBooleanResultKind::CertifiedShortcut {
                        operation,
                        shortcut
                    },
                    "{operation:?}: {result:?}"
                );
                result.validate().unwrap();
                result.validate_against_sources(left, right).unwrap();
            }
        }
    }

    #[test]
    fn graph_empty_closed_winding_containment_materializes_named_booleans() {
        let outer = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
        let disjoint_shell = tetrahedron_i64([20, 0, 0], [21, 0, 0], [20, 1, 0], [20, 0, 1]);
        let container = concatenate_meshes(&outer, &disjoint_shell, ValidationPolicy::CLOSED)
            .expect("disconnected closed container fixture should validate");
        let contained = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
        let uncontained = tetrahedron_i64([30, 0, 0], [31, 0, 0], [30, 1, 0], [30, 0, 1]);

        assert!(container.facts().mesh.closed_manifold);
        assert!(contained.facts().mesh.closed_manifold);
        assert!(!meshes_are_certified_bounds_disjoint(
            &container, &contained
        ));
        let graph = build_intersection_graph(&container, &contained).unwrap();
        validate_graph_source_handoff(&graph, &container, &contained).unwrap();
        assert!(!graph.has_unknowns());
        assert!(graph.face_pairs.is_empty());

        for (left, right, right_inside_left) in [
            (&container, &contained, true),
            (&contained, &container, false),
        ] {
            for operation in [
                ExactBooleanOperation::Union,
                ExactBooleanOperation::Intersection,
                ExactBooleanOperation::Difference,
            ] {
                let preflight =
                    ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                        .preflight(left, right)
                        .unwrap();
                assert_eq!(
                    preflight.support,
                    ExactBooleanSupport::CertifiedClosedWindingContainment,
                    "{right_inside_left:?} {operation:?}: {preflight:?}"
                );
                assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");
                preflight.validate().unwrap();
                preflight.validate_against_sources(left, right).unwrap();

                let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(left, right).unwrap();
                assert_eq!(
                    readiness.status,
                    ExactWindingReadinessStatus::ClosedWindingContainmentAlreadyMaterialized,
                    "{right_inside_left:?} {operation:?}: {readiness:?}"
                );
                assert_eq!(
                    readiness.blocker.kind,
                    ExactBooleanBlockerKind::NeedsWinding,
                    "{operation:?}: {readiness:?}"
                );
                assert_eq!(readiness.retained_face_pairs, 0, "{operation:?}");
                assert_eq!(readiness.retained_events, 0, "{operation:?}");
                assert!(winding_readiness_status_already_materialized(
                    &readiness.status
                ));
                assert!(
                    !winding_readiness_status_materializes_arrangement_cell_complex(
                        &readiness.status
                    )
                );
                readiness.validate().unwrap();
                readiness.validate_against_sources(left, right).unwrap();

                let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                    .materialize(left, right)
                    .expect("strict closed containment should materialize");
                assert_eq!(
                    result.kind,
                    ExactBooleanResultKind::CertifiedShortcut {
                        operation,
                        shortcut: ExactBooleanShortcutKind::ClosedWindingContainment
                    },
                    "{right_inside_left:?} {operation:?}: {result:?}"
                );
                result.validate().unwrap();
                result.validate_against_sources(left, right).unwrap();
                let stale_sources_rejected = if right_inside_left {
                    result.validate_against_sources(left, &uncontained).is_err()
                } else {
                    result
                        .validate_against_sources(&uncontained, right)
                        .is_err()
                };
                assert!(
                    stale_sources_rejected,
                    "{right_inside_left:?} {operation:?}: {result:?}"
                );

                match (operation, right_inside_left) {
                    (ExactBooleanOperation::Union, _) => {
                        assert!(exact_meshes_have_same_shape(&result.mesh, &container));
                    }
                    (ExactBooleanOperation::Intersection, _) => {
                        assert!(exact_meshes_have_same_shape(&result.mesh, &contained));
                    }
                    (ExactBooleanOperation::Difference, false) => {
                        assert!(result.mesh.triangles().is_empty());
                    }
                    (ExactBooleanOperation::Difference, true) => {
                        assert!(result.mesh.facts().mesh.closed_manifold);
                        assert_eq!(
                            result.mesh.triangles().len(),
                            container.triangles().len() + contained.triangles().len()
                        );
                    }
                    (ExactBooleanOperation::SelectedRegions(_), _) => unreachable!(),
                }
            }
        }
    }

    #[test]
    fn graph_empty_closed_winding_separation_materializes_without_bounds_disjointness() {
        let left_a = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
        let left_b = tetrahedron_i64([10, 0, 0], [11, 0, 0], [10, 1, 0], [10, 0, 1]);
        let left = concatenate_meshes(&left_a, &left_b, ValidationPolicy::CLOSED).unwrap();
        let right = tetrahedron_i64([5, 0, 0], [6, 0, 0], [5, 1, 0], [5, 0, 1]);
        let intersecting_right = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);

        assert!(left.facts().mesh.closed_manifold);
        assert!(right.facts().mesh.closed_manifold);
        assert!(!meshes_are_certified_bounds_disjoint(&left, &right));
        let graph = build_intersection_graph(&left, &right).unwrap();
        validate_graph_source_handoff(&graph, &left, &right).unwrap();
        assert!(!graph.has_unknowns());
        assert!(graph.face_pairs.is_empty());

        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .preflight(&left, &right)
                .unwrap();
            assert_eq!(
                preflight.support,
                ExactBooleanSupport::CertifiedClosedWindingSeparated,
                "{operation:?}: {preflight:?}"
            );
            assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");
            preflight.validate().unwrap();
            preflight.validate_against_sources(&left, &right).unwrap();

            let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::ClosedWindingSeparatedAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(
                readiness.blocker.kind,
                ExactBooleanBlockerKind::NeedsWinding,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(readiness.retained_face_pairs, 0, "{operation:?}");
            assert_eq!(readiness.retained_events, 0, "{operation:?}");
            assert!(winding_readiness_status_already_materialized(
                &readiness.status
            ));
            assert!(
                !winding_readiness_status_materializes_arrangement_cell_complex(&readiness.status)
            );
            readiness.validate().unwrap();
            readiness.validate_against_sources(&left, &right).unwrap();

            let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: ExactBooleanShortcutKind::ClosedWindingSeparated
                },
                "{operation:?}: {result:?}"
            );
            result.validate().unwrap();
            result.validate_against_sources(&left, &right).unwrap();
            assert!(
                result
                    .validate_against_sources(&left, &intersecting_right)
                    .is_err(),
                "{operation:?}: {result:?}"
            );
            match operation {
                ExactBooleanOperation::Union => {
                    assert_eq!(
                        result.mesh.triangles().len(),
                        left.triangles().len() + right.triangles().len()
                    );
                }
                ExactBooleanOperation::Intersection => {
                    assert!(result.mesh.triangles().is_empty());
                }
                ExactBooleanOperation::Difference => {
                    assert!(exact_meshes_have_same_shape(&result.mesh, &left));
                }
                ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
            }
        }
    }

    #[test]
    fn mixed_dimensional_regularized_solid_reports_materialized_readiness() {
        let solid = axis_aligned_box_i64([0, 0, 0], [4, 4, 4]);
        let sheet = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 1, 3, 1, 1, 1, 3, 1],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        for (left, right) in [(&solid, &sheet), (&sheet, &solid)] {
            for operation in [
                ExactBooleanOperation::Union,
                ExactBooleanOperation::Intersection,
                ExactBooleanOperation::Difference,
            ] {
                let preflight =
                    ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                        .preflight(left, right)
                        .unwrap();
                assert_eq!(
                    preflight.support,
                    ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid,
                    "{operation:?}: {preflight:?}"
                );
                assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");

                let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(left, right).unwrap();
                assert_eq!(
                    readiness.status,
                    ExactWindingReadinessStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized,
                    "{operation:?}: {readiness:?}"
                );
                assert_eq!(
                    readiness.blocker.kind,
                    ExactBooleanBlockerKind::NeedsWinding,
                    "{operation:?}: {readiness:?}"
                );
                assert_eq!(readiness.retained_face_pairs, 0);
                assert_eq!(readiness.retained_events, 0);
                assert!(winding_readiness_status_already_materialized(
                    &readiness.status
                ));
                assert!(
                    !winding_readiness_status_materializes_arrangement_cell_complex(
                        &readiness.status
                    )
                );
                readiness.validate().unwrap();
                readiness.validate_against_sources(left, right).unwrap();

                let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                    .materialize(left, right)
                    .expect("mixed-dimensional regularized solid should shortcut");
                assert_eq!(
                    result.kind,
                    ExactBooleanResultKind::CertifiedShortcut {
                        operation,
                        shortcut: ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid
                    },
                    "{operation:?}: {result:?}"
                );
                result.validate().unwrap();
                result.validate_against_sources(left, right).unwrap();
                assert!(
                    result.validate_against_sources(&sheet, &sheet).is_err(),
                    "{operation:?}: {result:?}"
                );

                let keeps_solid = matches!(operation, ExactBooleanOperation::Union)
                    || (std::ptr::eq(left, &solid)
                        && matches!(operation, ExactBooleanOperation::Difference));
                if keeps_solid {
                    assert!(exact_meshes_have_same_shape(&result.mesh, &solid));
                } else {
                    assert!(
                        result.mesh.triangles().is_empty(),
                        "{operation:?}: {result:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn lower_dimensional_regularized_solid_reports_materialized_readiness() {
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
            let readiness = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .winding_readiness(&left, &right)
                .unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(
                readiness.blocker.kind,
                ExactBooleanBlockerKind::NeedsWinding,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(readiness.retained_face_pairs, 0);
            assert_eq!(readiness.retained_events, 0);
            assert_eq!(readiness.region_count, 0);
            assert!(winding_readiness_status_already_materialized(
                &readiness.status
            ));
            assert!(
                !winding_readiness_status_materializes_arrangement_cell_complex(&readiness.status)
            );
            readiness.validate().unwrap();
            readiness
                .validate_against_sources_with_validation(&left, &right, ValidationPolicy::CLOSED)
                .unwrap();
        }
    }

    #[test]
    fn closed_preflight_does_not_certify_boundary_only_arrangement_output() {
        let left = ExactMesh::from_i64_triangles(
            &[
                0, 0, 0, //
                4, 0, 0, //
                0, 4, 0, //
                0, 0, 4, //
                2, 2, 3,
            ],
            &[
                0, 2, 1, //
                1, 2, 3, //
                2, 0, 3, //
                0, 1, 4, //
                1, 3, 4, //
                3, 0, 4,
            ],
        )
        .unwrap();
        let right = tetrahedron_i64([1, 1, 1], [5, 1, 1], [1, 5, 1], [1, 1, 5]);
        assert!(left.facts().mesh.closed_manifold, "{:?}", left.facts().mesh);
        assert!(right.facts().mesh.closed_manifold);
        let graph = build_intersection_graph(&left, &right).unwrap();
        validate_graph_source_handoff(&graph, &left, &right).unwrap();

        let preflight = ExactBooleanRequest::with_boundary_policy(
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .preflight(&left, &right)
        .unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::RequiresCertifiedWinding,
            "{preflight:?}"
        );
        assert!(preflight.blocker.is_some(), "{preflight:?}");
        preflight.validate().unwrap();
        preflight
            .validate_against_sources_with_boundary_policy(
                &left,
                &right,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        let fake_shortcut = ExactBooleanResult {
            kind: ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Union,
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            },
            graph_had_unknowns: false,
            region_classifications: Vec::new(),
            triangulations: Vec::new(),
            assembly: ExactBooleanAssemblyPlan {
                vertices: Vec::new(),
                triangles: Vec::new(),
            },
            volumetric_classifications: Vec::new(),
            mesh: empty_mesh(
                "fake closed arrangement shortcut for unresolved winding case",
                ValidationPolicy::CLOSED,
            )
            .unwrap(),
        };
        assert!(
            fake_shortcut.validate().is_err(),
            "empty arrangement-cell union shortcut must fail local shape validation"
        );
        assert!(
            fake_shortcut
                .validate_against_sources(&left, &right)
                .is_err(),
            "resolved graph alone must not certify an arrangement-cell shortcut"
        );

        let boundary_preflight = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .preflight(&left, &right)
        .unwrap();
        assert_eq!(
            boundary_preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{boundary_preflight:?}"
        );
        assert!(
            boundary_preflight.blocker.is_none(),
            "{boundary_preflight:?}"
        );
        assert_eq!(
            boundary_preflight.retained_face_pairs,
            graph.face_pairs.len()
        );
        assert_eq!(boundary_preflight.retained_events, graph.event_count());
        boundary_preflight.validate().unwrap();
        boundary_preflight
            .validate_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        assert!(
            boundary_preflight
                .validate_against_sources_with_boundary_policy(
                    &left,
                    &right,
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .is_err(),
            "closed replay should not certify an allow-boundary preflight"
        );

        let readiness =
            ExactBooleanRequest::with_boundary_policy(ExactBooleanOperation::Union, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
        assert_eq!(
            readiness.status,
            ExactWindingReadinessStatus::VolumetricAssemblyRequired,
            "{readiness:?}"
        );
        assert!(readiness.region_count > 0, "{readiness:?}");
        readiness.validate().unwrap();
        readiness.validate_against_sources(&left, &right).unwrap();

        let boundary_readiness = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .winding_readiness(&left, &right)
        .unwrap();
        assert_eq!(
            boundary_readiness.status,
            ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
            "{boundary_readiness:?}"
        );
        assert_eq!(
            boundary_readiness.blocker.kind,
            ExactBooleanBlockerKind::NeedsWinding,
            "{boundary_readiness:?}"
        );
        assert_eq!(
            boundary_readiness.retained_face_pairs,
            graph.face_pairs.len()
        );
        assert_eq!(boundary_readiness.retained_events, graph.event_count());
        assert_eq!(boundary_readiness.region_count, 0);
        assert!(winding_readiness_status_already_materialized(
            &boundary_readiness.status
        ));
        assert!(
            winding_readiness_status_materializes_arrangement_cell_complex(
                &boundary_readiness.status
            )
        );
        boundary_readiness.validate().unwrap();
        boundary_readiness
            .validate_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        assert!(
            boundary_readiness
                .validate_against_sources(&left, &right)
                .is_err(),
            "closed replay should not certify allow-boundary readiness"
        );

        let result =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
                .materialize(&left, &right);
        assert!(result.is_err(), "{result:?}");

        let materialized = materialize_volumetric_winding_region_plan_from_graph(
            &graph,
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .expect("volumetric split-cell materializer should retain boundary-output assembly");
        materialized.assembly.validate().unwrap();
        materialized
            .assembly
            .validate_source_face_incidence(&left, &right)
            .unwrap();
        materialized.mesh.validate_retained_state().unwrap();
        assert!(!materialized.mesh.facts().mesh.closed_manifold);
        assert!(!materialized.assembly.triangles.is_empty());
        assert!(
            close_exact_coplanar_boundary_loops(
                &materialized.mesh,
                "test self-contacting boundary must not close by coordinate dedup",
                ValidationPolicy::CLOSED,
            )
            .is_none(),
            "self-contacting boundary caps require a topology-preserving quotient before closure"
        );

        let closure = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .volumetric_boundary_closure(&left, &right)
        .unwrap();
        assert_eq!(
            closure.status,
            ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(
                ExactArrangementBlocker::NonManifoldCellComplex
            ),
            "{closure:?}"
        );
        assert_eq!(closure.boundary_loops, 1, "{closure:?}");
        assert_eq!(closure.noncoplanar_boundary_loops, 0, "{closure:?}");
        assert_eq!(closure.repeated_exact_boundary_points, 0, "{closure:?}");
        assert_eq!(closure.self_contact_exact_points, 0, "{closure:?}");
        assert_eq!(closure.self_contact_topological_vertices, 0, "{closure:?}");
        assert_eq!(closure.self_contact_degenerate_cycles, 0, "{closure:?}");
        assert_eq!(closure.self_contact_nondegenerate_cycles, 0, "{closure:?}");
        assert_eq!(closure.coplanar_loop_groups, 1, "{closure:?}");
        assert!(closure.boundary_edges > 0, "{closure:?}");
        assert!(closure.output_triangles > 0, "{closure:?}");
        closure.validate().unwrap();
        closure.validate_against_sources(&left, &right).unwrap();

        let boundary_result = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .materialize(&left, &right)
        .expect("public exact boolean should support boundary output");
        assert_eq!(
            boundary_result.kind,
            ExactBooleanResultKind::ArrangementCellComplexMaterialized {
                operation: ExactBooleanOperation::Union
            }
        );
        boundary_result.validate().unwrap();
        boundary_result
            .validate_against_sources(&left, &right)
            .unwrap();
        let mut stale_region_fact = boundary_result.clone();
        let stale_classification = stale_region_fact
            .region_classifications
            .iter_mut()
            .find(|classification| {
                classification.relation != crate::region::FaceRegionPlaneRelation::StrictlyAbove
            })
            .expect("materialized arrangement output should retain replayable region-plane facts");
        stale_classification
            .node_sides
            .fill(Some(hyperlimit::PlaneSide::Above));
        stale_classification.relation = crate::region::FaceRegionPlaneRelation::StrictlyAbove;
        assert!(
            stale_region_fact.validate().is_ok(),
            "{stale_region_fact:?}"
        );
        assert!(
            stale_region_fact
                .validate_against_sources(&left, &right)
                .is_err(),
            "materialized arrangement output must replay retained region-plane facts"
        );
        boundary_result
            .validate_operation_against_sources(
                &left,
                &right,
                ExactBooleanOperation::Union,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
    }

    #[test]
    fn volumetric_boundary_closure_report_certifies_triangular_coplanar_cap() {
        let left = ExactMesh::from_i64_triangles(
            &[
                0, 0, 0, //
                4, 0, 0, //
                0, 4, 0, //
                0, 0, 4, //
                2, 2, 3,
            ],
            &[
                0, 2, 1, //
                1, 2, 3, //
                2, 0, 3, //
                0, 1, 4, //
                1, 3, 4, //
                3, 0, 4,
            ],
        )
        .unwrap();
        let right = tetrahedron_i64([-1, 1, 0], [3, 1, 0], [-1, 5, 0], [-1, 1, 4]);

        let closure = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .volumetric_boundary_closure(&left, &right)
        .unwrap();
        assert_eq!(
            closure.status,
            ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
            "{closure:?}"
        );
        assert_eq!(closure.boundary_loops, 1, "{closure:?}");
        assert_eq!(closure.coplanar_loop_groups, 1, "{closure:?}");
        closure.validate().unwrap();
        closure.validate_against_sources(&left, &right).unwrap();
    }

    #[test]
    fn volumetric_coplanar_boundary_closure_materializes_closed_output() {
        let left = ExactMesh::from_i64_triangles(
            &[
                0, 0, 0, //
                4, 0, 0, //
                0, 4, 0, //
                0, 0, 4, //
                2, 2, 3,
            ],
            &[
                0, 2, 1, //
                1, 2, 3, //
                2, 0, 3, //
                0, 1, 4, //
                1, 3, 4, //
                3, 0, 4,
            ],
        )
        .unwrap();
        let right = tetrahedron_i64([-1, 1, 0], [3, 1, 0], [-1, 5, 0], [-1, 1, 4]);
        let graph = build_intersection_graph(&left, &right).unwrap();
        validate_graph_source_handoff(&graph, &left, &right).unwrap();

        let union_closure = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .volumetric_boundary_closure(&left, &right)
        .unwrap();
        assert_eq!(
            union_closure.status,
            ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
            "{union_closure:?}"
        );
        union_closure.validate().unwrap();
        union_closure
            .validate_against_sources(&left, &right)
            .unwrap();

        let difference_closure = ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .volumetric_boundary_closure(&left, &right)
        .unwrap();
        assert_eq!(
            difference_closure.status,
            ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
            "{difference_closure:?}"
        );
        difference_closure.validate().unwrap();

        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let closure = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .volumetric_boundary_closure(&left, &right)
                .unwrap();
            assert_eq!(
                closure.status,
                ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
                "{operation:?}: {closure:?}"
            );
            assert_eq!(closure.boundary_loops, 1, "{operation:?}: {closure:?}");
            assert_eq!(
                closure.coplanar_loop_groups, 1,
                "{operation:?}: {closure:?}"
            );
            closure.validate().unwrap();
            closure.validate_against_sources(&left, &right).unwrap();

            let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .preflight(&left, &right)
                .unwrap();
            assert_eq!(
                preflight.support,
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                "{operation:?}: {preflight:?}"
            );
            assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");
            preflight.validate().unwrap();
            preflight.validate_against_sources(&left, &right).unwrap();

            let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            readiness.validate().unwrap();

            let result = boolean_arrangement_volumetric_split_cell_recovery_from_graph(
                &graph,
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
            )
            .unwrap()
            .expect("coplanar boundary closure should materialize closed output");
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
                },
                "{operation:?}: {result:?}"
            );
            result.validate().unwrap();
            assert!(
                result.mesh.facts().mesh.closed_manifold || result.mesh.triangles().is_empty(),
                "{operation:?}: {:?}",
                result.mesh.facts().mesh
            );
            result.validate_against_sources(&left, &right).unwrap();
            assert!(
                result.validate_against_sources(&right, &left).is_err(),
                "closed cap shortcut must retain source-owned provenance"
            );

            let public = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .expect("closed exact boolean should consume coplanar cap support");
            assert_eq!(public.kind, result.kind, "{operation:?}: {public:?}");
            public.validate().unwrap();
        }
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
            Ok(ArrangementCellComplexOutcome::Materialized(result, attempt)) => {
                arrangement_cell_complex_result_is_certified_for_preflight(&result, &attempt)
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
            let expected_support = match operation {
                ExactBooleanOperation::Union => {
                    ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
                }
                ExactBooleanOperation::Intersection => {
                    ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
                }
                ExactBooleanOperation::Difference => {
                    ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference
                }
                ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
            };
            let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .preflight(&left, &right)
                .unwrap();
            assert_eq!(
                preflight.support, expected_support,
                "{operation:?}: {preflight:?}"
            );
            assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");
            assert!(preflight.region_count > 0, "{operation:?}: {preflight:?}");
            assert!(preflight.validate().is_ok(), "{operation:?}: {preflight:?}");
            preflight.validate_against_sources(&left, &right).unwrap();

            let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::OpenSurfaceArrangementAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(
                readiness.blocker.kind,
                ExactBooleanBlockerKind::NeedsWinding,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(readiness.region_count, preflight.region_count);
            assert_eq!(
                readiness.region_classifications,
                preflight.region_classifications
            );
            assert!(winding_readiness_status_already_materialized(
                &readiness.status
            ));
            assert!(
                !winding_readiness_status_materializes_arrangement_cell_complex(&readiness.status)
            );
            readiness.validate().unwrap();
            readiness.validate_against_sources(&left, &right).unwrap();

            let attempt = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .expect("arrangement attempt should run");
            assert_eq!(
                attempt.stage,
                ExactArrangementBooleanStage::Materialized,
                "{operation:?}: {attempt:?}"
            );
            assert_eq!(
                attempt.materialized_shortcut,
                Some(ExactBooleanShortcutKind::ArrangementCellComplex),
                "{operation:?}: {attempt:?}"
            );
            assert!(attempt.decline.is_none(), "{operation:?}: {attempt:?}");
            if !matches!(operation, ExactBooleanOperation::Intersection) {
                assert!(attempt.output_triangles > 0, "{operation:?}: {attempt:?}");
            }
            assert_current_arrangement_attempt(&attempt, &left, &right);

            let result = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .materialize(&left, &right)
                .expect("open-surface crossing should materialize");
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::OpenSurfaceArrangement { operation }
            );
            result.validate().unwrap();
            result.validate_against_sources(&left, &right).unwrap();
            let mut stale_region_fact = result.clone();
            let classification = stale_region_fact
                .region_classifications
                .first_mut()
                .expect("open-surface arrangement should retain region classifications");
            match classification.relation {
                crate::region::FaceRegionPlaneRelation::StrictlyAbove => {
                    classification.relation = crate::region::FaceRegionPlaneRelation::StrictlyBelow;
                    classification
                        .node_sides
                        .fill(Some(hyperlimit::PlaneSide::Below));
                }
                _ => {
                    classification.relation = crate::region::FaceRegionPlaneRelation::StrictlyAbove;
                    classification
                        .node_sides
                        .fill(Some(hyperlimit::PlaneSide::Above));
                }
            }
            stale_region_fact.validate().unwrap();
            assert!(
                stale_region_fact
                    .validate_against_sources(&left, &right)
                    .is_err(),
                "{operation:?}: stale region classification should fail source replay"
            );
            let mut stale_triangulation_fact = result.clone();
            let triangulation = stale_triangulation_fact
                .triangulations
                .iter_mut()
                .find(|triangulation| triangulation.triangles.len() >= 3)
                .expect("open-surface arrangement should retain triangulations");
            triangulation.triangles.swap(0, 1);
            stale_triangulation_fact.validate().unwrap();
            assert!(
                stale_triangulation_fact
                    .validate_against_sources(&left, &right)
                    .is_err(),
                "{operation:?}: stale triangulation should fail source replay"
            );
            if matches!(operation, ExactBooleanOperation::Intersection) {
                let mut incomplete_region_set = result.clone();
                let dropped = incomplete_region_set
                    .triangulations
                    .pop()
                    .expect("open-surface arrangement should retain triangulations");
                incomplete_region_set
                    .region_classifications
                    .retain(|classification| {
                        classification.region_side != dropped.side
                            || classification.region_face != dropped.face
                    });
                incomplete_region_set.validate().unwrap();
                assert!(
                    incomplete_region_set
                        .validate_against_sources(&left, &right)
                        .is_err(),
                    "open-surface intersection must retain the complete replayed region set"
                );
            }
            let selection = match operation {
                ExactBooleanOperation::Union => ExactRegionSelection::KeepAll,
                ExactBooleanOperation::Intersection => ExactRegionSelection::KeepNone,
                ExactBooleanOperation::Difference => ExactRegionSelection::KeepLeft,
                ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
            };
            result
                .assembly
                .validate_against_sources(&left, &right, selection)
                .unwrap();
            result
                .validate_operation_against_sources(
                    &left,
                    &right,
                    operation,
                    ValidationPolicy::ALLOW_BOUNDARY,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();
        }
    }

    #[test]
    fn partial_face_boundary_touch_is_regularized_without_coplanar_cell_blocker() {
        let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
        let right = tetrahedron_i64([2, 2, 2], [4, 1, 1], [1, 4, 1], [3, 3, 3]);

        let intersection = ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .preflight(&left, &right)
        .unwrap();
        assert_eq!(
            intersection.support,
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
        );
        assert!(intersection.retained_face_pairs > 0, "{intersection:?}");
        assert!(intersection.blocker.is_none());
        intersection.validate().unwrap();
        intersection
            .validate_against_sources(&left, &right)
            .unwrap();

        let difference = ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .preflight(&left, &right)
        .unwrap();
        assert_eq!(
            difference.support,
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
        );
        assert!(difference.retained_face_pairs > 0, "{difference:?}");
        assert!(difference.blocker.is_none());
        difference.validate().unwrap();
        difference.validate_against_sources(&left, &right).unwrap();

        let intersection = ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        )
        .materialize(&left, &right)
        .unwrap();
        assert_eq!(
            intersection.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Intersection,
                shortcut: ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection
            }
        );
        assert!(intersection.mesh.triangles().is_empty());

        let difference =
            ExactBooleanRequest::new(ExactBooleanOperation::Difference, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
        assert_eq!(
            difference.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Difference,
                shortcut: ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference
            }
        );
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
            let (expected_support, expected_shortcut) = match operation {
                ExactBooleanOperation::Union => (
                    ExactBooleanSupport::CertifiedConvexUnion,
                    ExactBooleanShortcutKind::ConvexUnion,
                ),
                ExactBooleanOperation::Intersection => (
                    ExactBooleanSupport::CertifiedConvexIntersection,
                    ExactBooleanShortcutKind::ConvexIntersection,
                ),
                ExactBooleanOperation::Difference => (
                    ExactBooleanSupport::CertifiedConvexDifference,
                    ExactBooleanShortcutKind::ConvexDifference,
                ),
                ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
            };
            let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .preflight(&left, &right)
                .unwrap();
            assert_eq!(
                preflight.support, expected_support,
                "{operation:?}: {preflight:?}"
            );
            assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");
            assert_eq!(
                preflight.retained_face_pairs, 0,
                "{operation:?}: {preflight:?}"
            );

            let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::ConvexBooleanAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(
                readiness.blocker.kind,
                ExactBooleanBlockerKind::NeedsWinding,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(readiness.retained_face_pairs, 0);
            assert_eq!(readiness.retained_events, 0);
            readiness.validate().unwrap();
            readiness.validate_against_sources(&left, &right).unwrap();

            let attempt = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();
            assert_eq!(
                attempt.materialized_shortcut,
                Some(ExactBooleanShortcutKind::ArrangementCellComplex),
                "{operation:?}: {attempt:?}"
            );
            assert!(attempt.decline.is_none(), "{operation:?}: {attempt:?}");
            assert_current_arrangement_attempt(&attempt, &left, &right);

            let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: expected_shortcut
                },
                "{operation:?}: {result:?}"
            );
            result.validate().unwrap();
            result.validate_against_sources(&left, &right).unwrap();
            result
                .validate_operation_against_sources(
                    &left,
                    &right,
                    operation,
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();
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

        let preflight = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .preflight(&left, &right)
        .expect("preflight should certify face-touching closed union");
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{preflight:?}"
        );
        assert!(preflight.blocker.is_none(), "{preflight:?}");

        let attempt = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
        .expect("arrangement attempt should run");
        assert_eq!(
            attempt.materialized_shortcut,
            Some(ExactBooleanShortcutKind::ArrangementCellComplex),
            "{attempt:?}"
        );
        assert!(attempt.decline.is_none(), "{attempt:?}");
        assert_current_arrangement_attempt(&attempt, &left, &right);

        let result =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .expect("face-touching closed union should materialize");
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert!(result.mesh.facts().mesh.closed_manifold);
    }

    #[test]
    fn boundary_touching_orthogonal_shortcuts_report_materialized_readiness() {
        let left = axis_aligned_box_i64([0, 0, 0], [1, 1, 1]);
        let right = axis_aligned_box_i64([1, 0, 0], [2, 1, 1]);

        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .preflight(&left, &right)
                .unwrap();
            assert_eq!(
                preflight.support,
                ExactBooleanSupport::CertifiedArrangementCellComplex,
                "{operation:?}: {preflight:?}"
            );
            assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");

            let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(
                readiness.blocker.kind,
                ExactBooleanBlockerKind::NeedsBoundaryPolicy,
                "{operation:?}: {readiness:?}"
            );
            assert!(winding_readiness_status_already_materialized(
                &readiness.status
            ));
            readiness.validate().unwrap();
            readiness.validate_against_sources(&left, &right).unwrap();
        }
    }

    #[test]
    fn nonorthogonal_closed_boundary_touching_shortcuts_report_provenance() {
        let left_a = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let left_b = tetrahedron_i64([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
        let left = concatenate_meshes(&left_a, &left_b, ValidationPolicy::CLOSED)
            .expect("disconnected nonconvex boundary fixture should validate");
        let right = tetrahedron_i64([0, 0, 0], [-4, 0, 0], [0, -4, 0], [0, 0, -4]);
        let separated_right = tetrahedron_i64([100, 0, 0], [104, 0, 0], [100, 4, 0], [100, 0, 4]);
        let overlapping_right = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
        let graph = build_intersection_graph(&left, &right).unwrap();
        validate_graph_source_handoff(&graph, &left, &right).unwrap();
        assert!(!graph.has_unknowns());
        assert!(!graph.face_pairs.is_empty());
        assert!(
            boundary_touching_report_from_graph(&graph, &left, &right)
                .unwrap()
                .is_certified()
        );
        assert!(
            !ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::ALLOW_BOUNDARY).boundary_touching_report(&left, &overlapping_right)
                .unwrap()
                .is_certified()
        );
        assert!(
            boolean_boundary_touching_meshes_from_graph(
                &graph,
                &left,
                &right,
                ExactBooleanOperation::Difference,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap()
            .is_some()
        );
        assert!(
            boolean_boundary_touching_meshes_from_graph(
                &graph,
                &left,
                &overlapping_right,
                ExactBooleanOperation::Difference,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap()
            .is_none()
        );

        for (operation, support, shortcut) in [
            (
                ExactBooleanOperation::Union,
                ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion,
                ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion,
            ),
            (
                ExactBooleanOperation::Intersection,
                ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection,
                ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection,
            ),
            (
                ExactBooleanOperation::Difference,
                ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference,
                ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
            ),
        ] {
            let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .preflight(&left, &right)
                .unwrap();
            assert_eq!(preflight.support, support, "{operation:?}: {preflight:?}");
            assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");
            preflight.validate().unwrap();
            preflight.validate_against_sources(&left, &right).unwrap();

            let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::ClosedBoundaryTouchingAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(
                readiness.blocker.kind,
                ExactBooleanBlockerKind::NeedsBoundaryPolicy,
                "{operation:?}: {readiness:?}"
            );
            assert!(winding_readiness_status_already_materialized(
                &readiness.status
            ));
            readiness.validate().unwrap();
            readiness.validate_against_sources(&left, &right).unwrap();

            let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut
                },
                "{operation:?}: {result:?}"
            );
            result.validate().unwrap();
            result.validate_against_sources(&left, &right).unwrap();
            assert!(
                result
                    .validate_against_sources(&left, &separated_right)
                    .is_err(),
                "{operation:?}: {result:?}"
            );
            match operation {
                ExactBooleanOperation::Union => {
                    assert_eq!(
                        result.mesh.triangles().len(),
                        left.triangles().len() + right.triangles().len()
                    );
                }
                ExactBooleanOperation::Intersection => {
                    assert!(result.mesh.triangles().is_empty());
                }
                ExactBooleanOperation::Difference => {
                    assert!(exact_meshes_have_same_shape(&result.mesh, &left));
                }
                ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
            }
        }
    }

    #[test]
    fn boundary_attached_contained_tetrahedron_difference_materializes() {
        let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
        let right = tetrahedron_i64([2, 2, 2], [4, 1, 1], [1, 4, 1], [1, 1, 1]);

        let preflight = ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .preflight(&left, &right)
        .unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedConvexDifference
        );
        assert!(preflight.blocker.is_none(), "{preflight:?}");
        assert!(preflight.retained_face_pairs > 0, "{preflight:?}");
        assert!(preflight.retained_events > 0, "{preflight:?}");
        let mut relabeled_preflight = preflight.clone();
        relabeled_preflight.operation = ExactBooleanOperation::Union;
        assert_eq!(
            relabeled_preflight.validate(),
            Err(crate::ExactReportValidationError::StatusEvidenceMismatch)
        );
        let readiness =
            ExactBooleanRequest::with_boundary_policy(ExactBooleanOperation::Difference, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right)
                .unwrap();
        assert_eq!(
            readiness.status,
            ExactWindingReadinessStatus::ConvexBooleanAlreadyMaterialized,
            "{readiness:?}"
        );
        readiness.validate().unwrap();
        readiness.validate_against_sources(&left, &right).unwrap();

        let difference =
            ExactBooleanRequest::new(ExactBooleanOperation::Difference, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
        assert_eq!(
            difference.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Difference,
                shortcut: ExactBooleanShortcutKind::ConvexDifference
            }
        );
        difference.validate().unwrap();
        difference.validate_against_sources(&left, &right).unwrap();
        let mut relabeled = difference.clone();
        relabeled.kind = ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: ExactBooleanShortcutKind::ConvexDifference,
        };
        assert_eq!(
            relabeled.validate(),
            Err(crate::ExactReportValidationError::StatusEvidenceMismatch)
        );
        difference
            .validate_operation_against_sources(
                &left,
                &right,
                ExactBooleanOperation::Difference,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert!(difference.mesh.triangles().len() >= left.triangles().len());
    }

    #[test]
    fn noncoplanar_convex_shortcut_reports_retain_graph_counts() {
        let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
        let right = tetrahedron_i64([1, 1, 1], [5, 1, 2], [1, 5, 1], [2, 1, 5]);
        let graph = build_intersection_graph(&left, &right).unwrap();
        validate_graph_source_handoff(&graph, &left, &right).unwrap();
        assert!(!graph.has_unknowns());
        assert_eq!(graph.face_pairs.len(), 3);
        assert_eq!(graph.event_count(), 12);

        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let expected_support = match operation {
                ExactBooleanOperation::Union => ExactBooleanSupport::CertifiedConvexUnion,
                ExactBooleanOperation::Intersection => {
                    ExactBooleanSupport::CertifiedConvexIntersection
                }
                ExactBooleanOperation::Difference => ExactBooleanSupport::CertifiedConvexDifference,
                ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
            };
            let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .preflight(&left, &right)
                .unwrap();
            assert_eq!(
                preflight.support, expected_support,
                "{operation:?}: {preflight:?}"
            );
            assert_eq!(preflight.retained_face_pairs, graph.face_pairs.len());
            assert_eq!(preflight.retained_events, graph.event_count());
            assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");
            preflight.validate().unwrap();
            preflight.validate_against_sources(&left, &right).unwrap();

            let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::ConvexBooleanAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(readiness.retained_face_pairs, graph.face_pairs.len());
            assert_eq!(readiness.retained_events, graph.event_count());
            assert_eq!(
                readiness.blocker.kind,
                ExactBooleanBlockerKind::NeedsWinding
            );
            assert_eq!(readiness.blocker.candidate_pairs, graph.face_pairs.len());
            readiness.validate().unwrap();
            readiness.validate_against_sources(&left, &right).unwrap();
        }
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
            let (expected_support, expected_shortcut) = match operation {
                ExactBooleanOperation::Union => (
                    ExactBooleanSupport::CertifiedConvexUnion,
                    ExactBooleanShortcutKind::ConvexUnion,
                ),
                ExactBooleanOperation::Intersection => (
                    ExactBooleanSupport::CertifiedConvexIntersection,
                    ExactBooleanShortcutKind::ConvexIntersection,
                ),
                ExactBooleanOperation::Difference => (
                    ExactBooleanSupport::CertifiedConvexDifference,
                    ExactBooleanShortcutKind::ConvexDifference,
                ),
                ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
            };
            let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .preflight(&left, &right)
                .unwrap();
            assert_eq!(
                preflight.support, expected_support,
                "{operation:?}: {preflight:?}"
            );
            assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");
            assert!(
                preflight.retained_face_pairs > 0,
                "{operation:?}: {preflight:?}"
            );
            assert!(
                preflight.retained_events > 0,
                "{operation:?}: {preflight:?}"
            );

            let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::ConvexBooleanAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            readiness.validate().unwrap();
            readiness.validate_against_sources(&left, &right).unwrap();

            let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: expected_shortcut
                },
                "{operation:?}: {result:?}"
            );
            result.validate().unwrap();
            result.validate_against_sources(&left, &right).unwrap();
            result
                .validate_operation_against_sources(
                    &left,
                    &right,
                    operation,
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();
            assert!(
                result.mesh.facts().mesh.closed_manifold,
                "{operation:?}: {:?}",
                result.mesh.facts().mesh
            );

            let attempt = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();
            assert_eq!(
                attempt.materialized_shortcut,
                Some(ExactBooleanShortcutKind::ArrangementCellComplex),
                "{operation:?}: {attempt:?}"
            );
            assert_eq!(attempt.decline, None, "{operation:?}: {attempt:?}");
            assert!(attempt.output_triangles > 0, "{operation:?}: {attempt:?}");
            assert_current_arrangement_attempt(&attempt, &left, &right);
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
    fn exact_coplanar_boundary_closer_can_append_cap_vertices() {
        let mut vertices = vec![
            Point3::new(Real::from(0), Real::from(0), Real::from(0)),
            Point3::new(Real::from(4), Real::from(0), Real::from(0)),
            Point3::new(Real::from(0), Real::from(4), Real::from(0)),
        ];
        let reused = find_or_insert_exact_mesh_vertex(
            &mut vertices,
            Point3::new(Real::from(4), Real::from(0), Real::from(0)),
        )
        .expect("exact existing cap vertex should be reusable");
        assert_eq!(reused, 1);
        assert_eq!(vertices.len(), 3);

        let inserted = find_or_insert_exact_mesh_vertex(
            &mut vertices,
            Point3::new(
                (Real::from(4) / &Real::from(3)).unwrap(),
                (Real::from(4) / &Real::from(3)).unwrap(),
                Real::from(0),
            ),
        )
        .expect("exact cap triangulation vertex should be appendable");
        assert_eq!(inserted, 3);
        assert_eq!(vertices.len(), 4);
    }

    #[test]
    fn exact_coplanar_boundary_canonicalizes_only_degenerate_self_contact_spurs() {
        let a = Point3::new(Real::from(0), Real::from(0), Real::from(0));
        let b = Point3::new(Real::from(1), Real::from(0), Real::from(0));
        let c = Point3::new(Real::from(1), Real::from(1), Real::from(0));
        let d = Point3::new(Real::from(0), Real::from(1), Real::from(0));
        let e = Point3::new(Real::from(-1), Real::from(0), Real::from(0));

        let degenerate_spur = canonicalize_degenerate_boundary_self_contact(vec![
            a.clone(),
            b.clone(),
            a.clone(),
            c.clone(),
            d.clone(),
        ])
        .expect("exact degenerate spur canonicalization should decide");
        assert_eq!(degenerate_spur.len(), 3);
        assert_eq!(point3_exact_equal(&degenerate_spur[0], &a), Some(true));
        assert_eq!(point3_exact_equal(&degenerate_spur[1], &c), Some(true));
        assert_eq!(point3_exact_equal(&degenerate_spur[2], &d), Some(true));
        assert_eq!(
            boundary_loop_self_contact_evidence(&degenerate_spur)
                .unwrap()
                .repeated_exact_point_pairs,
            0
        );
        assert!(exact_loop_is_coplanar(&degenerate_spur).unwrap());

        let nondegenerate_self_contact =
            canonicalize_degenerate_boundary_self_contact(vec![a.clone(), b, c, a, d, e])
                .expect("exact nondegenerate self-contact classification should decide");
        assert_eq!(nondegenerate_self_contact.len(), 6);
        assert_eq!(
            boundary_loop_self_contact_evidence(&nondegenerate_self_contact)
                .unwrap()
                .nondegenerate_cycles,
            2
        );

        let split = split_boundary_self_contact_cycles(nondegenerate_self_contact)
            .expect("exact self-contact cycle splitting should decide");
        assert_eq!(split.len(), 2);
        assert!(split.iter().all(|cycle| cycle.len() == 3));
        assert!(split.iter().all(|cycle| {
            boundary_loop_self_contact_evidence(cycle)
                .unwrap()
                .repeated_exact_point_pairs
                == 0
        }));
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
    fn exact_coplanar_boundary_closer_orients_cap_groups_independently() {
        let mesh = two_open_boxes_missing_opposite_caps_i64([0, 0, 0], [4, 0, 0]);
        assert_eq!(mesh.facts().mesh.boundary_edges, 8);
        assert!(!mesh.facts().mesh.closed_manifold);

        let closed = close_exact_coplanar_boundary_loops(
            &mesh,
            "test exact independently oriented coplanar boundary closure",
            ValidationPolicy::CLOSED,
        )
        .expect("opposite cap groups should close with independently certified orientations");

        assert!(
            closed.facts().mesh.closed_manifold,
            "{:?}",
            closed.facts().mesh
        );
        assert_eq!(closed.vertices().len(), mesh.vertices().len());
        assert_eq!(closed.triangles().len(), mesh.triangles().len() + 4);
    }

    #[test]
    fn closed_identical_solids_route_through_arrangement_pipeline() {
        let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let right = left.clone();

        let preflight = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .preflight(&left, &right)
        .unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex
        );

        let union =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
        assert_eq!(
            union.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Union,
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        assert!(exact_meshes_have_same_shape(&union.mesh, &left));

        let difference =
            ExactBooleanRequest::new(ExactBooleanOperation::Difference, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
        assert_eq!(
            difference.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Difference,
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

        let attempt = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
        assert_eq!(attempt.decline, None);
        assert_eq!(
            attempt.materialized_shortcut,
            Some(ExactBooleanShortcutKind::ArrangementCellComplex)
        );
        assert_current_arrangement_attempt(&attempt, &left, &right);

        let preflight = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .preflight(&left, &right)
        .unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex
        );

        let union =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
        assert_eq!(
            union.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Union,
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        assert!(exact_meshes_have_same_shape(&union.mesh, &left));

        let difference =
            ExactBooleanRequest::new(ExactBooleanOperation::Difference, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
        assert_eq!(
            difference.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Difference,
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

        let union_attempt = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
        assert_eq!(union_attempt.decline, None);
        assert_eq!(union_attempt.selected_faces, 4);
        assert_eq!(union_attempt.output_triangles, 4);
        assert_current_arrangement_attempt(&union_attempt, &left, &right);
        let mut stale_selected_faces = union_attempt.clone();
        stale_selected_faces.selected_faces = stale_selected_faces.face_cells + 1;
        assert_eq!(
            stale_selected_faces.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let mut stale_selected_volumes = union_attempt.clone();
        stale_selected_volumes.selected_volume_regions = stale_selected_volumes.volume_regions + 1;
        assert_eq!(
            stale_selected_volumes.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let mut stale_volume_regions = union_attempt.clone();
        stale_volume_regions.regions = 0;
        assert_eq!(
            stale_volume_regions.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let mut stale_volume_adjacencies = union_attempt.clone();
        stale_volume_adjacencies.volume_regions = 1;
        stale_volume_adjacencies.volume_adjacencies = 1;
        assert_eq!(
            stale_volume_adjacencies.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let mut stale_union_counts = union_attempt.clone();
        stale_union_counts.output_vertices = 0;
        stale_union_counts.output_triangles = 0;
        assert_eq!(
            stale_union_counts.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
        let mut impossible_output_counts = union_attempt.clone();
        impossible_output_counts.output_vertices = 0;
        assert_eq!(
            impossible_output_counts.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let difference_attempt = ExactBooleanRequest::new(
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
        assert_eq!(difference_attempt.decline, None);
        assert_eq!(difference_attempt.selected_faces, 0);
        assert_eq!(difference_attempt.output_triangles, 0);
        assert_current_arrangement_attempt(&difference_attempt, &left, &right);

        let union =
            ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
        assert_eq!(
            union.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Union,
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        assert!(exact_meshes_have_same_shape(&union.mesh, &left));

        let difference =
            ExactBooleanRequest::new(ExactBooleanOperation::Difference, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
        assert_eq!(
            difference.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Difference,
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

        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .preflight(&left, &right)
                .unwrap();
            assert_eq!(
                preflight.support,
                ExactBooleanSupport::CertifiedSameSurface,
                "{operation:?}: {preflight:?}"
            );
            let readiness = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::SurfaceEqualityAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(readiness.retained_face_pairs, 0);
            assert_eq!(readiness.retained_events, 0);
            readiness.validate().unwrap();
            readiness.validate_against_sources(&left, &right).unwrap();

            let result = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .materialize(&left, &right)
                .unwrap();
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: ExactBooleanShortcutKind::SameSurface
                },
                "{operation:?}: {result:?}"
            );
            result.validate().unwrap();
            result.validate_against_sources(&left, &right).unwrap();
            result
                .validate_operation_against_sources(
                    &left,
                    &right,
                    operation,
                    ValidationPolicy::ALLOW_BOUNDARY,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();
            if matches!(operation, ExactBooleanOperation::Difference) {
                assert!(result.mesh.triangles().is_empty(), "{result:?}");
            } else {
                assert!(exact_meshes_have_same_shape(&result.mesh, &left));
            }
        }
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

        let preflight = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .preflight(&left, &right)
        .unwrap();
        assert_eq!(preflight.support, ExactBooleanSupport::CertifiedIdentical);
        let readiness =
            ExactBooleanRequest::with_boundary_policy(ExactBooleanOperation::Union, ValidationPolicy::ALLOW_BOUNDARY, ExactBoundaryBooleanPolicy::Reject).winding_readiness(&left, &right).unwrap();
        assert_eq!(
            readiness.status,
            ExactWindingReadinessStatus::SurfaceEqualityAlreadyMaterialized,
            "{readiness:?}"
        );
        readiness.validate().unwrap();
        readiness.validate_against_sources(&left, &right).unwrap();

        let union = ExactBooleanRequest::new(
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .materialize(&left, &right)
        .unwrap();
        assert_eq!(
            union.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Union,
                shortcut: ExactBooleanShortcutKind::Identical
            }
        );
        union
            .validate_operation_against_sources(
                &left,
                &right,
                ExactBooleanOperation::Union,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
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
        let graph = build_intersection_graph(&left, &right).unwrap();
        validate_graph_source_handoff(&graph, &left, &right).unwrap();
        let preflight = ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .preflight(&left, &right)
        .unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{preflight:?}"
        );
        assert!(preflight.blocker.is_none(), "{preflight:?}");
        assert_eq!(preflight.retained_face_pairs, graph.face_pairs.len());
        assert_eq!(preflight.retained_events, graph.event_count());
        preflight.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Intersection,
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

    fn axis_aligned_orthogonal_l_solid_i64() -> ExactMesh {
        let horizontal = axis_aligned_box_i64([0, 0, 0], [2, 1, 1]);
        let vertical = axis_aligned_box_i64([0, 1, 0], [1, 2, 1]);
        let plan = axis_aligned_orthogonal_solid_cell_plan(
            &horizontal,
            &vertical,
            AxisAlignedOrthogonalSolidOperation::Union,
        )
        .expect("L solid should have an orthogonal cell plan");
        materialize_axis_aligned_orthogonal_solid_cell_plan(
            plan,
            "test axis-aligned orthogonal L solid",
            ValidationPolicy::CLOSED,
        )
        .unwrap()
    }

    fn affine_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
        let p = |u: i64, v: i64, w: i64| [2 * u + v, 2 * v, 2 * w];
        affine_box_from_map_i64(min, max, p)
    }

    fn skew_affine_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
        let p = |u: i64, v: i64, w: i64| [u + 10 * v, v, w];
        affine_box_from_map_i64(min, max, p)
    }

    fn affine_box_from_map_i64(
        min: [i64; 3],
        max: [i64; 3],
        p: impl Fn(i64, i64, i64) -> [i64; 3],
    ) -> ExactMesh {
        let corners = [
            p(min[0], min[1], min[2]),
            p(max[0], min[1], min[2]),
            p(max[0], max[1], min[2]),
            p(min[0], max[1], min[2]),
            p(min[0], min[1], max[2]),
            p(max[0], min[1], max[2]),
            p(max[0], max[1], max[2]),
            p(min[0], max[1], max[2]),
        ];
        ExactMesh::from_i64_triangles(
            &corners
                .iter()
                .flat_map(|point| point.iter().copied())
                .collect::<Vec<_>>(),
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

    fn two_open_boxes_missing_opposite_caps_i64(
        missing_top_min: [i64; 3],
        missing_bottom_min: [i64; 3],
    ) -> ExactMesh {
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for (min, missing_top) in [(missing_top_min, true), (missing_bottom_min, false)] {
            let max = [min[0] + 2, min[1] + 2, min[2] + 2];
            let start = vertices.len() / 3;
            vertices.extend([
                min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0],
                max[1], min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1],
                max[2], min[0], max[1], max[2],
            ]);
            if !missing_top {
                triangles.extend([
                    start + 4,
                    start + 5,
                    start + 6,
                    start + 4,
                    start + 6,
                    start + 7,
                ]);
            }
            if missing_top {
                triangles.extend([start, start + 2, start + 1, start, start + 3, start + 2]);
            }
            triangles.extend([
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
