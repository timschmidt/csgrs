//! Auditable exact boolean reports.
//!
//! These types are internal evidence objects produced by the exact boolean
//! staging layer. They carry graph counts, predicate certificates, and checked
//! kernel artifacts instead of collapsing exact topology decisions to `bool`.
//! Downstream policy layers should consume narrower borrowed kernel views.

use hyperlimit::{
    ApproximationPolicy, MeshSource, Point3, TriangleLocation, classify_point_triangle,
    compare_reals, project_point3, projected_polygon_area2_value,
};
use hyperreal::Real;
use std::cmp::Ordering;

use super::ExactMesh;
use super::adjacent::materialize_full_face_adjacent_union;
use super::arrangement3d::{ExactArrangement, ExactTopologyAssemblyReport};
use super::boolean::affine_solid::{
    materialize_affine_orthogonal_solid_difference,
    materialize_affine_orthogonal_solid_intersection, materialize_affine_orthogonal_solid_union,
};
use super::boolean::contained_adjacent::materialize_contained_face_adjacent_union;
#[cfg(test)]
use super::boolean::materialize_boolean_exact_request;
use super::boolean::orthogonal_solid::{
    AxisAlignedOrthogonalSolidOperation, materialize_axis_aligned_orthogonal_solid_cell_output,
};
use super::boolean::volumetric::{
    ExactVolumetricRegionClassification, ExactVolumetricRegionError, ExactVolumetricRegionRelation,
};
use super::boolean::volumetric_cells::{
    CoplanarVolumetricCellEvidenceReport, CoplanarVolumetricCellObstacle,
};
use super::boolean::{
    ExactArrangementBooleanAttempt, ExactBooleanOperation, ExactBooleanRequest,
    ExactBoundaryBooleanPolicy, adjacent_union_completion_certification,
    boolean_coplanar_mesh_overlay_optional, boundary_policy_shortcut_result_matches_sources,
    boundary_touching_report_from_graph, exact_boolean_evaluation_for_replay,
    materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph,
    materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph,
    materialize_volumetric_coplanar_boundary_closure_output,
    no_materialized_boundary_output_report, open_surface_disjoint_report_from_graph,
    open_surface_disjoint_result_matches_sources,
    rematerialize_retained_arrangement_cell_complex_attempt,
    replay_boolean_exact_request_for_result_validation,
    replay_closed_same_surface_boolean_result_if_certified,
    replay_generic_arrangement_cell_complex_result, replay_open_surface_arrangement_result,
    replay_selected_region_boolean_result, same_surface_report_from_sources,
    volumetric_boundary_closure_report_from_graph,
};
#[cfg(test)]
use super::boolean::{
    exact_boolean_report_evaluation_for_replay, preflight_report_for_request_from_graph,
    winding_evidence_report_for_request_from_graph,
};
use super::bounds::AabbIntersectionKind;
use super::cell_complex::{
    ExactRegionOwnershipReport, arrangement_cell_complex_labeling_policy,
    validate_selected_gate_reports,
};
use super::convex::{
    intersect_closed_convex_solids, subtract_closed_convex_solids, union_closed_convex_solids,
};
use super::graph::MeshSide;
use super::graph::{
    CoplanarArrangementEvidence, CoplanarArrangementEvidenceStatus, ExactIntersectionGraph,
    IntersectionEvent, build_validated_intersection_graph,
};
use super::intersection::MeshFacePairRelation;
use super::region::{
    ExactBooleanAssemblyPlan, ExactOutputTriangle, ExactOutputTriangleOrientation,
    ExactRegionSelection, FaceRegionPlaneClassification, FaceRegionPlaneValidationError,
    FaceRegionTriangulation, boundary_node_point,
};
use super::regularization::ExactArrangementBlocker;
use super::regularization::ExactRegularizationPolicy;
use super::solid::{
    ConvexSolidMeshClassification, ConvexSolidMeshRelation, ConvexSolidPointRelation,
    classify_mesh_vertices_against_convex_solid_report,
};
use super::validation::ExactMeshValidationPolicy;
use super::winding::{
    ClosedMeshWindingMeshRelation, classify_mesh_vertices_against_closed_mesh_winding_report,
};
use hyperlimit::PredicateUse;

/// Validation failure for an exact report object.
///
/// Report validation checks the evidence object itself, not the original
/// geometry. It lets tests, fuzzing, and downstream policy code assert that
/// status, blocker kind, graph counts, and retained artifacts agree before a
/// report is trusted as certified evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum ExactReportValidationError {
    /// A certified shortcut report unexpectedly carried a blocker.
    CertifiedReportHasBlocker,
    /// A blocked or unresolved report did not carry the required blocker.
    MissingBlocker,
    /// The blocker kind does not match the report support/status.
    WrongBlockerKind,
    /// The blocker kind and retained relation counts contradict each other.
    InvalidBlockerCounts,
    /// A report that should not materialize split-region facts retained them.
    UnexpectedRegionFacts,
    /// A winding-evidence report did not retain checked region facts.
    MissingRegionFacts,
    /// A selected-region triangulation has no matching retained region/plane
    /// classification for its source region.
    UnclassifiedRegionTriangulation,
    /// A retained region/plane classification has no matching triangulated
    /// source region.
    OrphanedRegionClassification,
    /// An assembled selected-region output triangle has no matching retained
    /// source-region triangulation.
    UntriangulatedAssemblyRegion,
    /// An assembled selected-region output triangle uses a welded vertex whose
    /// exact point is absent from the retained triangulation boundary for its
    /// source region.
    AssemblyVertexOutsideTriangulation,
    /// A selected-region assembly retained an output vertex that no output
    /// triangle references.
    UnreferencedAssemblyVertex,
    /// A retained region/plane classification failed its own side-fact audit.
    InvalidRegionClassification(FaceRegionPlaneValidationError),
    /// A winding-evidence report retained a region/plane classification that still
    /// depends on undecided or non-proof-producing predicate evidence.
    RegionClassificationNotProofProducing,
    /// The retained region count does not match the unique classified source
    /// regions.
    RegionCountMismatch,
    /// The report retained the same source-region/opposite-plane
    /// classification more than once.
    DuplicateRegionClassification,
    /// The result retained more than one triangulation for the same source
    /// region.
    DuplicateRegionTriangulation,
    /// A retained split-region triangulation failed its own audit.
    InvalidTriangulation,
    /// A retained output assembly plan failed its own audit.
    InvalidAssembly,
    /// A retained output assembly plan contains a duplicate topological
    /// triangle.
    DuplicateAssemblyTriangle,
    /// A retained volumetric winding region classification failed its audit.
    InvalidVolumetricClassification(ExactVolumetricRegionError),
    /// An arrangement-materialized result did not retain volumetric region facts.
    MissingVolumetricClassifications,
    /// A result that was not arrangement-materialized retained volumetric region
    /// facts.
    UnexpectedVolumetricClassifications,
    /// A volumetric classification has no matching retained source-region
    /// triangulation.
    OrphanedVolumetricClassification,
    /// A retained source-region triangulation has no matching volumetric
    /// classification.
    UnclassifiedVolumetricTriangulation,
    /// Volumetric classifications are not in retained triangulation/cell order.
    VolumetricClassificationOrderMismatch,
    /// An arrangement-materialized result retained boundary, unknown, or nonclosed
    /// region evidence.
    VolumetricClassificationNotDecided,
    /// The materialized output mesh failed retained-state validation.
    InvalidOutputMesh,
    /// The materialized output mesh was not constructed at a boolean-output
    /// exact provenance boundary.
    InvalidOutputMeshProvenance,
    /// A selected-region result's assembly and materialized mesh disagree.
    OutputMeshAssemblyMismatch,
    /// A selected-region result's retained output assembly no longer replays
    /// against the supplied source meshes.
    OutputSourceReplayMismatch,
    /// A shortcut result retained selected-region classification,
    /// triangulation, or assembly artifacts.
    ShortcutResultHasAssemblyArtifacts,
    /// A certified shortcut or boundary-policy result claimed unresolved graph
    /// events after materializing output topology.
    ShortcutResultHasUnknownGraph,
    /// A selected-region result claimed unresolved graph events after
    /// materializing output topology.
    SelectedRegionResultHasUnknownGraph,
    /// A selected-region result retained output triangles from a source side
    /// excluded by its declared selection policy.
    SelectedRegionAssemblyViolatesSelection,
    /// A selected-region result did not retain materialized evidence for a
    /// source region selected by its declared policy.
    SelectedRegionAssemblyMissingSelectedRegion,
    /// A volumetric materialized result retained output triangles that do not
    /// match the declared operation's per-cell volumetric retention policy.
    VolumetricMaterializedAssemblyViolatesOperation,
    /// A certified graph shortcut retained graph events that contradict the
    /// shortcut status.
    UnexpectedGraphEvents,
    /// A required boundary or planar-arrangement report has no matching
    /// retained relation count.
    MissingRelationCount,
    /// A planar-arrangement report did not retain the checked coplanar graph
    /// evidence summary required for its status.
    MissingCoplanarArrangementEvidence,
    /// A planar-arrangement report retained an evidence summary where none is
    /// coherent for its status.
    UnexpectedCoplanarArrangementEvidence,
    /// A retained planar-arrangement evidence summary failed its own count
    /// audit.
    InvalidCoplanarArrangementEvidence,
    /// A coplanar-volumetric blocker did not retain its source-aware evidence.
    MissingCoplanarVolumetricEvidence,
    /// Coplanar-volumetric evidence was retained for a report state that
    /// cannot consume it.
    UnexpectedCoplanarVolumetricEvidence,
    /// Retained coplanar-volumetric evidence failed its local count audit.
    InvalidCoplanarVolumetricEvidence,
    /// Retained coplanar-volumetric evidence disagrees with the report's
    /// blocker counts or status.
    CoplanarVolumetricEvidenceMismatch,
    /// The report's unknown-graph flag contradicts its status.
    GraphUnknownStatusMismatch,
    /// The report status contradicts retained preconditions, relation counts,
    /// operation class, or graph evidence.
    StatusEvidenceMismatch,
    /// Planar-arrangement blocker counts and retained evidence counts
    /// disagree.
    CoplanarArrangementEvidenceMismatch,
    /// A same-surface report retained a non-bijective vertex permutation.
    InvalidPermutation,
    /// A certified same-surface report retained unequal remapped triangle sets.
    MismatchedTriangleSets,
    /// A retained report no longer matches facts recomputed from the supplied
    /// source meshes.
    SourceReplayMismatch,
}

fn validated_report_intersection_graph(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<ExactIntersectionGraph, ExactReportValidationError> {
    build_validated_intersection_graph(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)
}

fn blocker_kind(
    blocker: Option<&ExactBooleanBlocker>,
    expected: ExactBooleanBlockerKind,
) -> Result<(), ExactReportValidationError> {
    match blocker {
        Some(blocker) if blocker.kind == expected => Ok(()),
        Some(_) => Err(ExactReportValidationError::WrongBlockerKind),
        None => Err(ExactReportValidationError::MissingBlocker),
    }
}

fn no_region_facts(
    region_count: usize,
    classifications: &[FaceRegionPlaneClassification],
) -> Result<(), ExactReportValidationError> {
    if region_count == 0 && classifications.is_empty() {
        Ok(())
    } else {
        Err(ExactReportValidationError::UnexpectedRegionFacts)
    }
}

fn validate_blocker_count_bounds(
    blocker: &ExactBooleanBlocker,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactReportValidationError> {
    let Some(classified_relation_pairs) = blocker
        .candidate_pairs
        .checked_add(blocker.coplanar_overlapping_pairs)
        .and_then(|count| count.checked_add(blocker.coplanar_touching_pairs))
    else {
        return Err(ExactReportValidationError::InvalidBlockerCounts);
    };
    // `unknown_pairs` can overlap a classified relation when a candidate pair
    // carries an unknown event, but every retained graph pair must still be
    // covered by either a classified relation counter or unknown evidence.
    let Some(covered_relation_pairs) = classified_relation_pairs.checked_add(blocker.unknown_pairs)
    else {
        return Err(ExactReportValidationError::InvalidBlockerCounts);
    };
    let retained_graph_is_partial = (retained_face_pairs == 0) != (retained_events == 0);
    let retained_pairs_without_evidence =
        retained_face_pairs != 0 && !blocker_has_any_evidence(blocker);
    if retained_graph_is_partial
        || retained_pairs_without_evidence
        || classified_relation_pairs > retained_face_pairs
        || blocker.unknown_pairs > retained_face_pairs
        || covered_relation_pairs < retained_face_pairs
        || blocker.construction_failed_events > retained_events
    {
        Err(ExactReportValidationError::InvalidBlockerCounts)
    } else {
        Ok(())
    }
}

fn validate_retained_graph_count_shape(
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactReportValidationError> {
    if (retained_face_pairs == 0 && retained_events != 0)
        || (retained_face_pairs != 0 && retained_events == 0)
        || retained_events < retained_face_pairs
    {
        Err(ExactReportValidationError::StatusEvidenceMismatch)
    } else {
        Ok(())
    }
}

fn validate_coplanar_arrangement_evidence_matches_blocker(
    evidence: &CoplanarArrangementEvidence,
    blocker: &ExactBooleanBlocker,
) -> Result<(), ExactReportValidationError> {
    // The compact evidence report and blocker are two projections of the same
    // exact graph state; downstream planar-cell or winding checks must not
    // consume a summary with relabeled graph counts.
    if evidence.overlapping_graphs != blocker.coplanar_overlapping_pairs
        || evidence.touching_graphs != blocker.coplanar_touching_pairs
        || evidence.graph_count
            != blocker
                .coplanar_overlapping_pairs
                .checked_add(blocker.coplanar_touching_pairs)
                .ok_or(ExactReportValidationError::CoplanarArrangementEvidenceMismatch)?
    {
        Err(ExactReportValidationError::CoplanarArrangementEvidenceMismatch)
    } else {
        Ok(())
    }
}

const fn blocker_has_any_evidence(blocker: &ExactBooleanBlocker) -> bool {
    blocker.candidate_pairs != 0
        || blocker.coplanar_overlapping_pairs != 0
        || blocker.coplanar_touching_pairs != 0
        || blocker.unknown_pairs != 0
        || blocker.construction_failed_events != 0
}

fn blocker_has_refinement_evidence(blocker: &ExactBooleanBlocker) -> bool {
    blocker.unknown_pairs != 0 || blocker.construction_failed_events != 0
}

fn validate_adjacent_certified_boundary_blocker(
    blocker: &ExactBooleanBlocker,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactReportValidationError> {
    if retained_face_pairs == 0 && retained_events == 0 && !blocker_has_any_evidence(blocker) {
        return (blocker.kind == ExactBooleanBlockerKind::BoundaryPolicy)
            .then_some(())
            .ok_or(ExactReportValidationError::WrongBlockerKind);
    }
    blocker.validate_for_kind(ExactBooleanBlockerKind::BoundaryPolicy)
}

fn validate_refinement_partition(
    graph_unknown_status: bool,
    blocker: &ExactBooleanBlocker,
) -> Result<(), ExactReportValidationError> {
    // Unknown predicate outcomes and failed exact constructions are both
    // boundary, planar-cell, and winding reports must not consume unresolved
    // construction state under a resolved status label.
    if graph_unknown_status {
        if blocker_has_refinement_evidence(blocker) {
            Ok(())
        } else {
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        }
    } else if blocker_has_refinement_evidence(blocker) {
        Err(ExactReportValidationError::StatusEvidenceMismatch)
    } else {
        Ok(())
    }
}

const fn certified_preflight_support_matches_operation(
    support: ExactBooleanSupport,
    operation: ExactBooleanOperation,
) -> bool {
    matches!(
        (support, operation),
        (
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
                | ExactBooleanSupport::CertifiedConvexUnion,
            ExactBooleanOperation::Union,
        ) | (
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
                | ExactBooleanSupport::CertifiedConvexIntersection,
            ExactBooleanOperation::Intersection,
        ) | (
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
                | ExactBooleanSupport::CertifiedConvexDifference,
            ExactBooleanOperation::Difference,
        ) | (
            ExactBooleanSupport::CertifiedEmptyOperand
                | ExactBooleanSupport::CertifiedBoundsDisjoint
                | ExactBooleanSupport::CertifiedIdentical
                | ExactBooleanSupport::CertifiedSameSurface
                | ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
                | ExactBooleanSupport::CertifiedClosedWindingSeparated
                | ExactBooleanSupport::CertifiedClosedWindingContainment
                | ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid
                | ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid
                | ExactBooleanSupport::CertifiedBoundaryPolicyShortcut
                | ExactBooleanSupport::CertifiedConvexContainment
                | ExactBooleanSupport::CertifiedConvexSeparated,
            ExactBooleanOperation::Union
                | ExactBooleanOperation::Intersection
                | ExactBooleanOperation::Difference,
        )
    )
}

fn checked_region_facts(
    region_count: usize,
    classifications: &[FaceRegionPlaneClassification],
) -> Result<(), ExactReportValidationError> {
    if region_count == 0 || classifications.is_empty() {
        return Err(ExactReportValidationError::MissingRegionFacts);
    }
    let mut unique_regions = Vec::new();
    let mut unique_classifications = Vec::new();
    for classification in classifications {
        classification
            .validate()
            .map_err(ExactReportValidationError::InvalidRegionClassification)?;
        let key = (classification.region_side, classification.region_face);
        if !unique_regions.contains(&key) {
            unique_regions.push(key);
        }
        let classification_key = (
            classification.region_side,
            classification.region_face,
            classification.plane_side,
            classification.plane_face,
        );
        // Each region/plane side fact is retained numerical structure, not a
        // duplicate certificates would let later winding code over-count or
        // relabel already-consumed side evidence.
        if unique_classifications.contains(&classification_key) {
            return Err(ExactReportValidationError::DuplicateRegionClassification);
        }
        unique_classifications.push(classification_key);
        // Winding-ready evidence must carry decided side facts, not an
        // "unknown" region/plane relation. Undecided predicates remain
        // explicit blockers instead of being mislabeled as classified regions.
        if !classification.is_decided_and_proof_producing() {
            return Err(ExactReportValidationError::RegionClassificationNotProofProducing);
        }
    }
    // `region_count` is a retained combinatorial fact, not a display counter.
    // It must match the unique region handles covered by plane classifications
    // so a later winding policy cannot silently consume stale or relabeled
    if unique_regions.len() != region_count {
        return Err(ExactReportValidationError::RegionCountMismatch);
    }
    Ok(())
}

fn validate_coplanar_volumetric_evidence_matches_blocker(
    evidence: &CoplanarVolumetricCellEvidenceReport,
    blocker: &ExactBooleanBlocker,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactReportValidationError> {
    validate_coplanar_volumetric_evidence_counts(evidence, retained_face_pairs, retained_events)?;
    if evidence.candidate_pairs != blocker.candidate_pairs
        || evidence.coplanar_touching_pairs != blocker.coplanar_touching_pairs
        || evidence.coplanar_overlapping_pairs != blocker.coplanar_overlapping_pairs
        || evidence.unknown_pairs != blocker.unknown_pairs
        || evidence.construction_failed_events != blocker.construction_failed_events
    {
        return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
    }
    Ok(())
}

fn validate_coplanar_volumetric_evidence_counts(
    evidence: &CoplanarVolumetricCellEvidenceReport,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactReportValidationError> {
    evidence
        .validate()
        .map_err(|_| ExactReportValidationError::InvalidCoplanarVolumetricEvidence)?;
    let Some(explicit_unknown_events) = evidence
        .unknown_events
        .checked_sub(evidence.unknown_segment_plane_events)
    else {
        return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
    };
    let Some(retained_evidence_events) = evidence
        .segment_plane_events
        .checked_add(evidence.coplanar_edge_events)
        .and_then(|count| count.checked_add(evidence.coplanar_vertex_events))
        .and_then(|count| count.checked_add(explicit_unknown_events))
    else {
        return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
    };
    if evidence.retained_face_pair_count != retained_face_pairs
        || retained_evidence_events != retained_events
    {
        return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
    }
    Ok(())
}

fn validate_coplanar_volumetric_evidence_shape(
    evidence: &CoplanarVolumetricCellEvidenceReport,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactReportValidationError> {
    validate_coplanar_volumetric_evidence_counts(evidence, retained_face_pairs, retained_events)?;
    if !evidence.obstacle.requires_coplanar_volumetric_cells() {
        return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
    }
    Ok(())
}

fn validate_coplanar_boundary_only_evidence_shape(
    evidence: &CoplanarVolumetricCellEvidenceReport,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactReportValidationError> {
    validate_coplanar_volumetric_evidence_counts(evidence, retained_face_pairs, retained_events)?;
    if evidence.obstacle != CoplanarVolumetricCellObstacle::BoundaryOnlyContact
        || evidence.positive_area_coplanar_overlapping_pairs == 0
    {
        return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
    }
    Ok(())
}

fn validate_arrangement_materialized_coplanar_evidence(
    evidence: &CoplanarVolumetricCellEvidenceReport,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactReportValidationError> {
    validate_coplanar_volumetric_evidence_counts(evidence, retained_face_pairs, retained_events)?;
    if !evidence.obstacle.requires_coplanar_volumetric_cells()
        && (evidence.obstacle != CoplanarVolumetricCellObstacle::BoundaryOnlyContact
            || evidence.positive_area_coplanar_overlapping_pairs == 0)
    {
        return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
    }
    Ok(())
}

/// Auditable result of an exact selected-region boolean pipeline.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactBooleanResult {
    /// Declared production path for this result.
    pub(crate) kind: ExactBooleanResultKind,
    /// Whether graph extraction contained unknown events before policy checks.
    pub(crate) graph_had_unknowns: bool,
    /// Certified classifications of split regions against opposite face
    /// planes.
    pub(crate) region_classifications: Vec<FaceRegionPlaneClassification>,
    /// Exact projected triangulations used for assembly.
    pub(crate) triangulations: Vec<FaceRegionTriangulation>,
    /// Non-mutating exact output assembly.
    pub(crate) assembly: ExactBooleanAssemblyPlan,
    /// Exact winding classifications used by volumetric arrangement materialization.
    pub(crate) volumetric_classifications: Vec<ExactVolumetricRegionClassification>,
    /// Topology assembly report consumed by an arrangement/cell-complex output,
    /// when the materialization path retained that gate evidence.
    pub(crate) topology_assembly_report: Option<ExactTopologyAssemblyReport>,
    /// Region ownership report consumed by an arrangement/cell-complex output,
    /// when the materialization path retained that gate evidence.
    pub(crate) region_ownership_report: Option<ExactRegionOwnershipReport>,
    /// Materialized exact output mesh validated under the requested policy.
    pub(crate) mesh: ExactMesh,
}

impl ExactBooleanResult {
    /// Return the declared production path for this result.
    pub(crate) fn kind(&self) -> ExactBooleanResultKind {
        self.kind
    }

    /// Replace the declared production path for this result.
    #[cfg(test)]
    pub(crate) fn replace_kind(&mut self, kind: ExactBooleanResultKind) -> ExactBooleanResultKind {
        std::mem::replace(&mut self.kind, kind)
    }

    /// Return whether graph extraction contained unknown events before policy checks.
    pub(crate) fn graph_had_unknowns(&self) -> bool {
        self.graph_had_unknowns
    }

    /// Return retained topology assembly gate evidence, when present.
    pub(crate) fn topology_assembly_report(&self) -> Option<&ExactTopologyAssemblyReport> {
        self.topology_assembly_report.as_ref()
    }

    /// Return retained region ownership gate evidence, when present.
    pub(crate) fn region_ownership_report(&self) -> Option<&ExactRegionOwnershipReport> {
        self.region_ownership_report.as_ref()
    }

    /// Consume this result and return the materialized exact output mesh.
    pub(crate) fn into_mesh(self) -> ExactMesh {
        self.mesh
    }

    pub(crate) fn with_gate_reports(
        mut self,
        topology_assembly_report: Option<ExactTopologyAssemblyReport>,
        region_ownership_report: Option<ExactRegionOwnershipReport>,
    ) -> Self {
        self.topology_assembly_report = topology_assembly_report;
        self.region_ownership_report = region_ownership_report;
        self
    }

    pub(crate) fn retain_missing_gate_reports(
        &mut self,
        topology_assembly_report: Option<&ExactTopologyAssemblyReport>,
        region_ownership_report: Option<&ExactRegionOwnershipReport>,
    ) {
        if self.topology_assembly_report.is_none() {
            self.topology_assembly_report = topology_assembly_report.cloned();
        }
        if self.region_ownership_report.is_none() {
            self.region_ownership_report = region_ownership_report.cloned();
        }
    }

    pub(crate) fn matches_retained_replay(&self, replay: &Self) -> bool {
        retained_boolean_result_matches(self, replay)
    }
}

/// Declared production path for an exact boolean result.
///
/// Result kind is explicit so validation does not infer semantic intent from
/// empty vectors. That distinction matters for exact computing: selected-region
/// assembly, certified shortcuts, and boundary-policy projections are different
/// result shapes even when they all produce an empty mesh.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactBooleanResultKind {
    /// The result came from split-region classification and selected assembly.
    SelectedRegions {
        /// Requested split-region retention rule.
        selection: ExactRegionSelection,
    },
    /// The result came from a certified named-boolean shortcut.
    CertifiedShortcut {
        /// Named operation executed by this shortcut.
        operation: ExactBooleanOperation,
        /// Specific shortcut proof boundary that produced the result.
        shortcut: ExactBooleanShortcutKind,
    },
    /// The result came from an explicit lower-dimensional boundary projection
    /// policy.
    BoundaryPolicyShortcut {
        /// Named operation whose lower-dimensional contacts were projected into
        /// a triangle-mesh output.
        operation: ExactBooleanOperation,
    },
    /// The result came from exact split-region assembly for an open-surface
    /// named boolean.
    ///
    /// Open non-coplanar surfaces do not enclose volumes, so the retained
    /// region classifications are proof-producing arrangement facts rather
    /// than winding facts. Keeping this as a named result kind, instead of
    /// relabeling it as caller-selected regions, preserves the operation
    OpenSurfaceArrangement {
        /// Named open-surface operation executed by split-region assembly.
        operation: ExactBooleanOperation,
    },
    /// The result was produced by regularizing exact arrangement cell-complex
    /// evidence for a named volumetric boolean.
    ArrangementCellComplexMaterialized {
        /// Named operation executed by the arrangement cell-complex pipeline.
        operation: ExactBooleanOperation,
    },
}

/// Executable certified shortcut used to produce a named boolean result.
///
/// This enum is intentionally narrower than [`ExactBooleanSupport`]: it names
/// only cases that have already materialized output topology. Retaining the
/// exact shortcut reason on [`ExactBooleanResultKind`] gives downstream audit
/// reducing all shortcut outputs to an undifferentiated mesh.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactBooleanShortcutKind {
    /// Exact empty-operand semantics.
    EmptyOperand,
    /// Certified disjoint mesh AABBs.
    BoundsDisjoint,
    /// Exact coordinate and topology identity.
    Identical,
    /// Exact coordinate equality and matching triangle sets, modulo indexing
    /// and face orientation.
    SameSurface,
    /// Certified regularized union of closed solids whose exact graph proves
    /// only lower-dimensional boundary contact.
    ClosedBoundaryTouchingUnion,
    /// Certified empty regularized intersection of closed solids whose exact
    /// graph proves only lower-dimensional boundary contact.
    ClosedBoundaryTouchingIntersection,
    /// Certified left-preserving regularized difference of closed solids whose
    /// exact graph proves only lower-dimensional boundary contact.
    ClosedBoundaryTouchingDifference,
    /// Certified graph absence for open surfaces.
    OpenSurfaceDisjoint,
    /// Certified closed-solid separation from an empty intersection graph and
    /// exact vertex winding reports.
    ClosedWindingSeparated,
    /// Certified closed-solid containment from an empty intersection graph and
    /// exact vertex winding reports.
    ClosedWindingContainment,
    /// Certified regularized closed-solid result for a mixed closed solid and
    /// lower-dimensional open surface.
    MixedDimensionalRegularizedSolid,
    /// Certified empty regularized closed-solid result for operands with no
    /// closed-volume contribution.
    LowerDimensionalRegularizedSolid,
    /// Certified closed-convex containment.
    ConvexContainment,
    /// Certified closed-convex union materialized by exact source-face
    /// subtraction.
    ConvexUnion,
    /// Certified closed-convex intersection materialized by exact halfspace
    /// clipping.
    ConvexIntersection,
    /// Certified closed-convex difference materialized by exact source-face
    /// cell subtraction.
    ConvexDifference,
    /// Certified closed-convex separation.
    ConvexSeparated,
    /// Certified exact arrangement/cell-complex materialization.
    ///
    /// The output was produced by building retained 3D arrangement cells,
    /// labeling them against the opposite mesh, selecting the named Boolean
    /// boundary cells, exact-simplifying the selected cell complex, and only
    /// then triangulating to an [`ExactMesh`].
    ArrangementCellComplex,
}

impl ExactBooleanResult {
    /// Return whether this result is a certified shortcut for `operation`.
    #[cfg(test)]
    pub(crate) fn is_certified_shortcut_for(&self, operation: ExactBooleanOperation) -> bool {
        matches!(
            self.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: result_operation,
                ..
            } if result_operation == operation
        )
    }

    /// Return whether this result is the requested certified shortcut class.
    pub(crate) fn is_certified_shortcut_kind_for(
        &self,
        operation: ExactBooleanOperation,
        shortcut: ExactBooleanShortcutKind,
    ) -> bool {
        matches!(
            self.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: result_operation,
                shortcut: result_shortcut,
            } if result_operation == operation && result_shortcut == shortcut
        )
    }

    /// Return whether this result is the arrangement/cell-complex shortcut.
    pub(crate) fn is_arrangement_cell_complex_shortcut_for(
        &self,
        operation: ExactBooleanOperation,
    ) -> bool {
        matches!(
            self.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: result_operation,
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            } if result_operation == operation
        )
    }

    /// Return whether this result is a caller boundary-policy projection.
    pub(crate) fn is_boundary_policy_shortcut_for(&self, operation: ExactBooleanOperation) -> bool {
        matches!(
            self.kind,
            ExactBooleanResultKind::BoundaryPolicyShortcut {
                operation: result_operation,
            } if result_operation == operation
        )
    }

    /// Return whether this result is an open-surface arrangement output.
    pub(crate) fn is_open_surface_arrangement_for(&self, operation: ExactBooleanOperation) -> bool {
        matches!(
            self.kind,
            ExactBooleanResultKind::OpenSurfaceArrangement {
                operation: result_operation,
            } if result_operation == operation
        )
    }

    /// Return whether this result was materialized by the arrangement/cell-complex path.
    pub(crate) fn is_arrangement_cell_complex_materialized_for(
        &self,
        operation: ExactBooleanOperation,
    ) -> bool {
        matches!(
            self.kind,
            ExactBooleanResultKind::ArrangementCellComplexMaterialized {
                operation: result_operation,
            }
                | ExactBooleanResultKind::CertifiedShortcut {
                    operation: result_operation,
                    shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
                } if result_operation == operation
        )
    }

    /// Returns whether this result kind witnesses the requested operation.
    pub(crate) fn matches_request(&self, request: ExactBooleanRequest) -> bool {
        match self.kind {
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

    /// Returns whether this result witnesses the request operation and carries
    /// an output mesh satisfying the request validation policy.
    pub(crate) fn satisfies_request_shape(&self, request: ExactBooleanRequest) -> bool {
        self.matches_request(request) && self.mesh.validation_policy().satisfies(request.validation)
    }

    /// Returns whether this result kind is a valid materialized witness for
    /// the retained preflight support that produced it.
    pub(crate) fn matches_preflight_support(&self, support: ExactBooleanSupport) -> bool {
        let expected_shortcut = match support {
            ExactBooleanSupport::SelectedRegionPolicy => {
                return matches!(self.kind, ExactBooleanResultKind::SelectedRegions { .. });
            }
            ExactBooleanSupport::CertifiedBoundaryPolicyShortcut => {
                return matches!(
                    self.kind,
                    ExactBooleanResultKind::BoundaryPolicyShortcut { .. }
                );
            }
            ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
            | ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
            | ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference => {
                return matches!(
                    self.kind,
                    ExactBooleanResultKind::OpenSurfaceArrangement { .. }
                );
            }
            ExactBooleanSupport::CertifiedArrangementCellComplex => {
                if matches!(
                    self.kind,
                    ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
                ) {
                    return true;
                }
                ExactBooleanShortcutKind::ArrangementCellComplex
            }
            ExactBooleanSupport::CertifiedEmptyOperand => ExactBooleanShortcutKind::EmptyOperand,
            ExactBooleanSupport::CertifiedBoundsDisjoint => {
                ExactBooleanShortcutKind::BoundsDisjoint
            }
            ExactBooleanSupport::CertifiedIdentical => ExactBooleanShortcutKind::Identical,
            ExactBooleanSupport::CertifiedSameSurface => ExactBooleanShortcutKind::SameSurface,
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion => {
                ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion
            }
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection => {
                ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection
            }
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference => {
                ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference
            }
            ExactBooleanSupport::CertifiedOpenSurfaceDisjoint => {
                ExactBooleanShortcutKind::OpenSurfaceDisjoint
            }
            ExactBooleanSupport::CertifiedClosedWindingSeparated => {
                ExactBooleanShortcutKind::ClosedWindingSeparated
            }
            ExactBooleanSupport::CertifiedClosedWindingContainment => {
                ExactBooleanShortcutKind::ClosedWindingContainment
            }
            ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid => {
                ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid
            }
            ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid => {
                if matches!(
                    self.kind,
                    ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
                        | ExactBooleanResultKind::CertifiedShortcut {
                            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
                            ..
                        }
                ) {
                    return true;
                }
                ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid
            }
            ExactBooleanSupport::CertifiedConvexContainment => {
                ExactBooleanShortcutKind::ConvexContainment
            }
            ExactBooleanSupport::CertifiedConvexUnion => ExactBooleanShortcutKind::ConvexUnion,
            ExactBooleanSupport::CertifiedConvexIntersection => {
                ExactBooleanShortcutKind::ConvexIntersection
            }
            ExactBooleanSupport::CertifiedConvexDifference => {
                ExactBooleanShortcutKind::ConvexDifference
            }
            ExactBooleanSupport::CertifiedConvexSeparated => {
                ExactBooleanShortcutKind::ConvexSeparated
            }
            ExactBooleanSupport::RequiresBoundaryPolicy
            | ExactBooleanSupport::RequiresPlanarArrangement
            | ExactBooleanSupport::RequiresCoplanarVolumetricCells
            | ExactBooleanSupport::RequiresCertifiedWinding
            | ExactBooleanSupport::UnresolvedGraph => return false,
        };
        matches!(
            self.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut,
                ..
            } if shortcut == expected_shortcut
        )
    }

    /// Validate the retained artifacts in this selected-region or shortcut
    /// boolean result.
    ///
    /// Shortcut booleans can return a certified mesh only when no split-region
    /// artifacts are retained. Selected-region results audit every
    /// region/plane classification,
    /// triangulation, assembly invariant, and the materialized output mesh,
    /// then checks that assembly vertices and triangles still match the mesh.
    /// A selected-region result must retain nonempty region classifications
    /// and triangulations because those are the checked handoff facts that
    /// justify the assembly; otherwise a caller could relabel an empty
    /// shortcut-like object as a selected-region boolean.
    /// Every retained triangulation must also have at least one matching
    /// retained region/plane classification for its source side and face, so
    /// the mesh handoff cannot contain triangulated topology disconnected from
    /// the exact side facts prepared for winding policy. Conversely, every
    /// retained region/plane classification must belong to a triangulated
    /// source region so stale or relabeled side facts cannot be interpreted as
    /// part of the output proof. Selected-region reports also require those
    /// side facts to be proof-producing and decided, rather than carrying an
    /// unknown relation beside a materialized output. Duplicate
    /// region/opposite-plane classifications are rejected for the same reason:
    /// retained side evidence is exact state, not a multiset that later
    /// winding code can count twice. The same rule applies to retained
    /// triangulations: each source region has one checked polygon-to-triangle
    /// handoff. Output assembly triangles must likewise point back to retained
    /// triangulated source regions,
    /// preventing post-hoc provenance relabeling after materialization, and
    /// their vertex sources must be members of the retained triangulation
    /// boundary for that source region; welded vertices may carry a different
    /// source witness, but their exact point must still replay to the retained
    /// boundary. The retained assembly must also avoid dead vertices so the
    /// topology handoff is the exact set consumed by mesh materialization. That
    /// rather than an opaque output mesh.
    pub(crate) fn validate(&self) -> Result<(), ExactReportValidationError> {
        let retains_region_artifacts = matches!(
            self.kind,
            ExactBooleanResultKind::SelectedRegions { .. }
                | ExactBooleanResultKind::OpenSurfaceArrangement { .. }
                | ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
        );
        let retains_volumetric_artifacts = matches!(
            self.kind,
            ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
        );
        if !retains_region_artifacts
            && (!self.region_classifications.is_empty()
                || !self.triangulations.is_empty()
                || !self.assembly.vertices.is_empty()
                || !self.assembly.triangles.is_empty())
        {
            return Err(ExactReportValidationError::ShortcutResultHasAssemblyArtifacts);
        }
        if retains_volumetric_artifacts && self.volumetric_classifications.is_empty() {
            return Err(ExactReportValidationError::MissingVolumetricClassifications);
        }
        if !retains_volumetric_artifacts && !self.volumetric_classifications.is_empty() {
            return Err(ExactReportValidationError::UnexpectedVolumetricClassifications);
        }
        if !retains_region_artifacts && self.graph_had_unknowns {
            return Err(ExactReportValidationError::ShortcutResultHasUnknownGraph);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut,
        } = self.kind
            && !shortcut_operation_matches(shortcut, operation)
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut,
        } = self.kind
        {
            validate_shortcut_output_shape(shortcut, operation, &self.mesh)?;
        }
        if let ExactBooleanResultKind::BoundaryPolicyShortcut { operation }
        | ExactBooleanResultKind::OpenSurfaceArrangement { operation }
        | ExactBooleanResultKind::ArrangementCellComplexMaterialized { operation } = self.kind
            && matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        self.validate_arrangement_cell_complex_gate_reports()?;
        if retains_region_artifacts && self.graph_had_unknowns {
            return Err(ExactReportValidationError::SelectedRegionResultHasUnknownGraph);
        }
        if retains_region_artifacts
            && (self.region_classifications.is_empty() || self.triangulations.is_empty())
        {
            return Err(ExactReportValidationError::MissingRegionFacts);
        }

        let mut unique_classifications = Vec::new();
        for classification in &self.region_classifications {
            classification
                .validate()
                .map_err(ExactReportValidationError::InvalidRegionClassification)?;
            let classification_key = (
                classification.region_side,
                classification.region_face,
                classification.plane_side,
                classification.plane_face,
            );
            // the exact state. A selected-region result cannot retain the same
            // region/plane side fact twice and still be a coherent winding
            // handoff.
            if unique_classifications.contains(&classification_key) {
                return Err(ExactReportValidationError::DuplicateRegionClassification);
            }
            unique_classifications.push(classification_key);
            if retains_region_artifacts && !classification.is_decided_and_proof_producing() {
                return Err(ExactReportValidationError::RegionClassificationNotProofProducing);
            }
        }
        let mut unique_triangulations = Vec::new();
        for triangulation in &self.triangulations {
            triangulation
                .validate()
                .map_err(|_| ExactReportValidationError::InvalidTriangulation)?;
            let triangulation_key = (triangulation.side, triangulation.face);
            // Each triangulation is the exact image of one retained
            // auditable object; duplicating it would make output assembly
            // provenance ambiguous even if the triangle soup still validates.
            if unique_triangulations.contains(&triangulation_key) {
                return Err(ExactReportValidationError::DuplicateRegionTriangulation);
            }
            unique_triangulations.push(triangulation_key);
        }
        let mut unique_volumetric_classifications = Vec::new();
        for classification in &self.volumetric_classifications {
            classification
                .validate()
                .map_err(ExactReportValidationError::InvalidVolumetricClassification)?;
            let classification_key = (
                classification.region_side,
                classification.region_face,
                classification.triangle,
            );
            if unique_volumetric_classifications.contains(&classification_key) {
                return Err(ExactReportValidationError::DuplicateRegionClassification);
            }
            unique_volumetric_classifications.push(classification_key);
            if retains_volumetric_artifacts && !classification.relation.is_materialization_decided()
            {
                return Err(ExactReportValidationError::VolumetricClassificationNotDecided);
            }
        }
        if retains_region_artifacts
            && self.triangulations.iter().any(|triangulation| {
                !self.region_classifications.iter().any(|classification| {
                    classification.region_side == triangulation.side
                        && classification.region_face == triangulation.face
                })
            })
        {
            return Err(ExactReportValidationError::UnclassifiedRegionTriangulation);
        }
        if retains_region_artifacts
            && self.region_classifications.iter().any(|classification| {
                !self.triangulations.iter().any(|triangulation| {
                    triangulation.side == classification.region_side
                        && triangulation.face == classification.region_face
                })
            })
        {
            return Err(ExactReportValidationError::OrphanedRegionClassification);
        }
        if retains_volumetric_artifacts
            && self.triangulations.iter().any(|triangulation| {
                triangulation.triangles.chunks_exact(3).any(|triangle| {
                    !self
                        .volumetric_classifications
                        .iter()
                        .any(|classification| {
                            classification.region_side == triangulation.side
                                && classification.region_face == triangulation.face
                                && classification.triangle
                                    == [triangle[0], triangle[1], triangle[2]]
                        })
                })
            })
        {
            return Err(ExactReportValidationError::UnclassifiedVolumetricTriangulation);
        }
        if retains_volumetric_artifacts
            && self
                .volumetric_classifications
                .iter()
                .any(|classification| {
                    !self.triangulations.iter().any(|triangulation| {
                        triangulation.side == classification.region_side
                            && triangulation.face == classification.region_face
                            && triangulation.triangles.chunks_exact(3).any(|triangle| {
                                classification.triangle == [triangle[0], triangle[1], triangle[2]]
                            })
                    })
                })
        {
            return Err(ExactReportValidationError::OrphanedVolumetricClassification);
        }
        if retains_volumetric_artifacts {
            let expected_volumetric_classifications = self
                .triangulations
                .iter()
                .flat_map(|triangulation| {
                    triangulation
                        .triangles
                        .chunks_exact(3)
                        .map(move |triangle| (triangulation.side, triangulation.face, triangle))
                })
                .collect::<Vec<_>>();
            if expected_volumetric_classifications.len() != self.volumetric_classifications.len()
                || !expected_volumetric_classifications
                    .iter()
                    .zip(&self.volumetric_classifications)
                    .all(|(&(side, face, triangle), classification)| {
                        classification.region_side == side
                            && classification.region_face == face
                            && classification.triangle == [triangle[0], triangle[1], triangle[2]]
                    })
            {
                return Err(ExactReportValidationError::VolumetricClassificationOrderMismatch);
            }
        }
        if retains_volumetric_artifacts {
            for classification in &self.volumetric_classifications {
                let Some(triangulation) = self.triangulations.iter().find(|triangulation| {
                    triangulation.side == classification.region_side
                        && triangulation.face == classification.region_face
                        && triangulation.triangles.chunks_exact(3).any(|triangle| {
                            classification.triangle == [triangle[0], triangle[1], triangle[2]]
                        })
                }) else {
                    return Err(ExactReportValidationError::OrphanedVolumetricClassification);
                };
                classification
                    .validate_representatives_against_triangulation(triangulation)
                    .map_err(ExactReportValidationError::InvalidVolumetricClassification)?;
            }
        }
        if retains_region_artifacts
            && self.assembly.triangles.iter().any(|triangle| {
                !self.triangulations.iter().any(|triangulation| {
                    triangulation.side == triangle.source_side
                        && triangulation.face == triangle.source_face
                })
            })
        {
            return Err(ExactReportValidationError::UntriangulatedAssemblyRegion);
        }
        if retains_region_artifacts {
            for triangle in &self.assembly.triangles {
                let Some(triangulation) = self.triangulations.iter().find(|triangulation| {
                    triangulation.side == triangle.source_side
                        && triangulation.face == triangle.source_face
                }) else {
                    return Err(ExactReportValidationError::UntriangulatedAssemblyRegion);
                };
                for &vertex in &triangle.vertices {
                    let Some(assembly_vertex) = self.assembly.vertices.get(vertex) else {
                        return Err(ExactReportValidationError::InvalidAssembly);
                    };
                    if !retains_volumetric_artifacts
                        && !triangulation.boundary.iter().any(|source| {
                            source == &assembly_vertex.source
                                || points_equal(&assembly_vertex.point, boundary_node_point(source))
                        })
                    {
                        return Err(ExactReportValidationError::AssemblyVertexOutsideTriangulation);
                    }
                }
            }
            if self
                .assembly
                .vertices
                .iter()
                .enumerate()
                .any(|(vertex, _)| {
                    !self
                        .assembly
                        .triangles
                        .iter()
                        .any(|triangle| triangle.vertices.contains(&vertex))
                })
            {
                return Err(ExactReportValidationError::UnreferencedAssemblyVertex);
            }
        }
        self.assembly
            .validate()
            .map_err(|_| ExactReportValidationError::InvalidAssembly)?;
        if retains_region_artifacts {
            let mut seen_triangle_vertex_sets = Vec::<[usize; 3]>::new();
            for triangle in &self.assembly.triangles {
                let mut vertices = triangle.vertices;
                vertices.sort_unstable();
                if seen_triangle_vertex_sets.contains(&vertices) {
                    return Err(ExactReportValidationError::DuplicateAssemblyTriangle);
                }
                seen_triangle_vertex_sets.push(vertices);
            }
        }
        self.mesh
            .validate_retained_state()
            .map_err(|_| ExactReportValidationError::InvalidOutputMesh)?;
        let output_source = &self.mesh.provenance().source;
        let has_exact_boolean_label = output_source.label.starts_with("exact ")
            || output_source.label.starts_with("empty exact ");
        let has_arrangement_label = output_source.label.contains("arrangement")
            || output_source.label.contains("cell-complex")
            || output_source.label.contains("volumetric split-cell")
            || output_source.label.contains("orthogonal solid cell")
            || output_source.label.contains("full-face adjacent")
            || output_source.label.contains("contained-face adjacent");
        let label_matches_kind = if let ExactBooleanResultKind::CertifiedShortcut {
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            ..
        } = self.kind
        {
            has_exact_boolean_label && has_arrangement_label
        } else {
            has_exact_boolean_label
        };
        if output_source.source != MeshSource::Exact
            || output_source.approximation != ApproximationPolicy::ExactOnly
            || !label_matches_kind
        {
            return Err(ExactReportValidationError::InvalidOutputMeshProvenance);
        }

        if retains_region_artifacts {
            validate_output_mesh_matches_assembly(&self.assembly, &self.mesh)?;
        }

        if let ExactBooleanResultKind::ArrangementCellComplexMaterialized { operation } = self.kind
        {
            validate_volumetric_materialized_assembly_matches_operation(
                operation,
                &self.triangulations,
                &self.volumetric_classifications,
                &self.assembly,
            )?;
        }

        let selection = match self.kind {
            ExactBooleanResultKind::SelectedRegions { selection } => Some(selection),
            ExactBooleanResultKind::OpenSurfaceArrangement { operation } => Some(match operation {
                ExactBooleanOperation::Intersection => ExactRegionSelection::KeepNone,
                ExactBooleanOperation::Union => ExactRegionSelection::KeepAll,
                ExactBooleanOperation::Difference => ExactRegionSelection::KeepLeft,
                ExactBooleanOperation::SelectedRegions(_) => {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }),
            _ => None,
        };
        let Some(selection) = selection else {
            return Ok(());
        };

        if self
            .assembly
            .triangles
            .iter()
            .any(|triangle| !selection_keeps(selection, triangle.source_side))
        {
            return Err(ExactReportValidationError::SelectedRegionAssemblyViolatesSelection);
        }
        validate_selected_region_assembly_covers_selection(
            selection,
            &self.triangulations,
            &self.assembly,
        )?;

        Ok(())
    }

    fn validate_arrangement_cell_complex_gate_reports(
        &self,
    ) -> Result<(), ExactReportValidationError> {
        if !self.has_arrangement_cell_complex_gate_reports() {
            return Ok(());
        }
        let operation = self
            .arrangement_cell_complex_operation()
            .ok_or(ExactReportValidationError::StatusEvidenceMismatch)?;
        let topology = self
            .topology_assembly_report
            .as_ref()
            .ok_or(ExactReportValidationError::StatusEvidenceMismatch)?;
        let ownership = self
            .region_ownership_report
            .as_ref()
            .ok_or(ExactReportValidationError::StatusEvidenceMismatch)?;
        validate_selected_gate_reports(Some(topology), Some(ownership), operation)
            .map_err(|_| ExactReportValidationError::StatusEvidenceMismatch)?;
        if topology.arrangement_face_cells != ownership.face_cells
            || topology.arrangement_face_cell_boundary_nodes != ownership.face_cell_boundary_nodes
            || topology.arrangement_face_cell_boundary_points != ownership.face_cell_boundary_points
            || topology.lower_dimensional_artifacts != ownership.lower_dimensional_artifacts
            || topology.lower_dimensional_point_contacts
                != ownership.lower_dimensional_point_contacts
            || topology.lower_dimensional_edge_contacts != ownership.lower_dimensional_edge_contacts
            || topology.lower_dimensional_edge_endpoints
                != ownership.lower_dimensional_edge_endpoints
            || self.triangulations.len() > topology.arrangement_face_cells
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        Ok(())
    }

    /// Validate this result and replay retained source-face provenance.
    ///
    /// [`Self::validate`] audits the report as a self-contained artifact. This
    /// stronger check also requires the original source meshes and replays each
    /// selected-region output triangle against the retained `source_side` and
    /// `source_face` labels. That source-aware replay is the executable form of
    /// topology must remain tied to the geometric objects and predicate facts
    /// that produced it, not just to a locally consistent output mesh.
    pub(crate) fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        self.validate_arrangement_cell_complex_gate_reports_against_sources(left, right)?;
        if matches!(
            self.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
                ..
            }
        ) {
            validated_report_intersection_graph(left, right)?;
            return Ok(());
        }
        let mut arrangement_cell_complex_output_replayed = false;
        if let ExactBooleanResultKind::SelectedRegions { selection } = self.kind {
            let replay = replay_selected_region_boolean_result(
                left,
                right,
                selection,
                self.mesh.validation_policy(),
            )
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
            if !retained_split_region_result_matches(self, &replay) {
                return Err(ExactReportValidationError::SourceReplayMismatch);
            }
        }
        if let ExactBooleanResultKind::OpenSurfaceArrangement { operation } = self.kind {
            let replay = replay_open_surface_arrangement_result(
                left,
                right,
                operation,
                self.mesh.validation_policy(),
            )
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            .ok_or(ExactReportValidationError::SourceReplayMismatch)?;
            if !retained_split_region_result_matches(self, &replay) {
                return Err(ExactReportValidationError::SourceReplayMismatch);
            }
        }
        if let ExactBooleanResultKind::ArrangementCellComplexMaterialized { operation } = self.kind
        {
            let mut replay = replay_generic_arrangement_cell_complex_result(
                left,
                right,
                operation,
                self.mesh.validation_policy(),
            )
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            .ok_or(ExactReportValidationError::SourceReplayMismatch)?;
            replay.kind = self.kind;
            if !retained_split_region_result_matches(self, &replay) {
                return Err(ExactReportValidationError::SourceReplayMismatch);
            }
        }
        if matches!(
            self.kind,
            ExactBooleanResultKind::SelectedRegions { .. }
                | ExactBooleanResultKind::OpenSurfaceArrangement { .. }
                | ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
        ) {
            self.assembly
                .validate_source_face_incidence(left, right)
                .map_err(|_| ExactReportValidationError::OutputSourceReplayMismatch)?;
        }
        if matches!(
            self.kind,
            ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
        ) {
            for classification in &self.volumetric_classifications {
                let Some(triangulation) = self.triangulations.iter().find(|triangulation| {
                    triangulation.side == classification.region_side
                        && triangulation.face == classification.region_face
                        && triangulation.triangles.chunks_exact(3).any(|triangle| {
                            classification.triangle == [triangle[0], triangle[1], triangle[2]]
                        })
                }) else {
                    return Err(ExactReportValidationError::OrphanedVolumetricClassification);
                };
                let target = match classification.region_side {
                    MeshSide::Left => right,
                    MeshSide::Right => left,
                };
                classification
                    .validate_against_sources(triangulation, target)
                    .map_err(ExactReportValidationError::InvalidVolumetricClassification)?;
            }
        }
        if let ExactBooleanResultKind::BoundaryPolicyShortcut { operation } = self.kind
            && !boundary_policy_shortcut_result_matches_sources(
                self,
                left,
                right,
                operation,
                self.mesh.validation_policy(),
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
        {
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut:
                shortcut @ (ExactBooleanShortcutKind::EmptyOperand
                | ExactBooleanShortcutKind::BoundsDisjoint
                | ExactBooleanShortcutKind::Identical
                | ExactBooleanShortcutKind::SameSurface
                | ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid
                | ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid),
        } = self.kind
            && !certified_shortcut_output_matches_sources(
                shortcut,
                operation,
                self.mesh.validation_policy(),
                &self.mesh,
                left,
                right,
            )?
        {
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut: ExactBooleanShortcutKind::OpenSurfaceDisjoint,
        } = self.kind
            && !open_surface_disjoint_result_matches_sources(
                self,
                left,
                right,
                operation,
                self.mesh.validation_policy(),
            )
        {
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
        } = self.kind
            && let Some(matches_output) = arrangement_cell_complex_output_matches_sources(
                operation,
                self.mesh.validation_policy(),
                &self.mesh,
                left,
                right,
            )
            .unwrap_or(None)
        {
            if !matches_output {
                return Err(ExactReportValidationError::SourceReplayMismatch);
            }
            arrangement_cell_complex_output_replayed = true;
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
        } = self.kind
            && !arrangement_cell_complex_output_replayed
            && let Some(replay) = boolean_coplanar_mesh_overlay_optional(
                left,
                right,
                operation,
                self.mesh.validation_policy(),
            )
            .ok()
            .flatten()
            && self.matches_retained_replay(&replay)
        {
            arrangement_cell_complex_output_replayed = true;
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut:
                shortcut @ (ExactBooleanShortcutKind::ConvexUnion
                | ExactBooleanShortcutKind::ConvexIntersection
                | ExactBooleanShortcutKind::ConvexDifference
                | ExactBooleanShortcutKind::ConvexContainment
                | ExactBooleanShortcutKind::ConvexSeparated),
        } = self.kind
            && !convex_operation_output_matches_sources(
                shortcut, operation, &self.mesh, left, right,
            )?
        {
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut:
                shortcut @ (ExactBooleanShortcutKind::ClosedWindingSeparated
                | ExactBooleanShortcutKind::ClosedWindingContainment),
        } = self.kind
            && !closed_winding_output_matches_sources(
                shortcut,
                operation,
                self.mesh.validation_policy(),
                &self.mesh,
                left,
                right,
            )?
        {
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut:
                shortcut @ (ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion
                | ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection
                | ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference),
        } = self.kind
            && !closed_boundary_touching_output_matches_sources(
                shortcut,
                operation,
                self.mesh.validation_policy(),
                &self.mesh,
                left,
                right,
            )?
        {
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut,
        } = self.kind
            && shortcut != ExactBooleanShortcutKind::ArrangementCellComplex
            && !certified_shortcut_sources_match(
                shortcut,
                operation,
                self.mesh.validation_policy(),
                left,
                right,
            )?
        {
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            ..
        } = self.kind
            && !arrangement_cell_complex_output_replayed
        {
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        Ok(())
    }

    pub(crate) fn validate_arrangement_cell_complex_gate_reports_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        if !self.has_arrangement_cell_complex_gate_reports() {
            return Ok(());
        }
        let arrangement = ExactArrangement::from_meshes_with_policy(
            left,
            right,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        self.validate_arrangement_cell_complex_gate_reports_against_arrangement(
            &arrangement,
            left,
            right,
            self.arrangement_cell_complex_operation(),
        )
    }

    pub(crate) fn validate_arrangement_cell_complex_gate_reports_against_arrangement(
        &self,
        arrangement: &ExactArrangement,
        left: &ExactMesh,
        right: &ExactMesh,
        operation: Option<ExactBooleanOperation>,
    ) -> Result<(), ExactReportValidationError> {
        if !self.has_arrangement_cell_complex_gate_reports() {
            return Ok(());
        }
        let replay_topology = arrangement.topology_assembly_report_with_policy(
            left,
            right,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        );
        if self.topology_assembly_report.as_ref() != Some(&replay_topology) {
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        let ownership_policy = arrangement_cell_complex_labeling_policy(
            arrangement,
            operation,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        );
        let replay_ownership = arrangement
            .region_ownership_report_with_policy(left, right, ownership_policy)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self.region_ownership_report.as_ref() != Some(&replay_ownership) {
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        Ok(())
    }

    fn arrangement_cell_complex_operation(&self) -> Option<ExactBooleanOperation> {
        match self.kind {
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            }
            | ExactBooleanResultKind::ArrangementCellComplexMaterialized { operation } => {
                Some(operation)
            }
            _ => None,
        }
    }

    fn has_arrangement_cell_complex_gate_reports(&self) -> bool {
        self.topology_assembly_report.is_some() || self.region_ownership_report.is_some()
    }

    /// Validate this result against the operation and policies that produced it.
    ///
    /// [`Self::validate_against_sources`] audits retained source provenance,
    /// including arrangement-cell-complex gate reports when present. This
    /// stronger replay accepts a retained certified arrangement attempt only
    /// when its materialized mesh and gate reports match the result, otherwise
    /// it recomputes the named exact boolean entry point for the same
    /// operands, operation, validation policy, and boundary policy. That closes
    /// the shortcut replay gap: a certified output mesh cannot be relabeled as
    /// a different named operation or shortcut kind while still passing the
    /// source audit.
    pub(crate) fn validate_request_against_sources_with_retained_attempt(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        request: ExactBooleanRequest,
        retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
    ) -> Result<(), ExactReportValidationError> {
        if !self.matches_request(request) {
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        self.validate()?;
        if self.can_use_retained_arrangement_attempt_for_request(request)
            && let Some(attempt) = retained_arrangement_attempt
            && attempt.certifies_arrangement_cell_complex_output_for_request(
                request,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            )
        {
            let matches_attempt = self
                .retained_arrangement_attempt_matches_output_for_request(request, Some(attempt))?;
            if matches_attempt {
                attempt.validate_against_sources_for_request(left, right, request)?;
                return Ok(());
            }
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        if matches!(
            self.kind,
            ExactBooleanResultKind::OpenSurfaceArrangement { .. }
        ) && self.satisfies_request_shape(request)
        {
            self.validate_against_sources(left, right)?;
            return Ok(());
        }
        if self.is_arrangement_cell_complex_shortcut_for(request.operation)
            && self.satisfies_request_shape(request)
        {
            if request.validation == ExactMeshValidationPolicy::CLOSED
                && lower_dimensional_regularized_sources(left, right)
                && mesh_output_is_empty(&self.mesh)
            {
                return Ok(());
            }
            validated_report_intersection_graph(left, right)?;
            return Ok(());
        }
        if self.topology_assembly_report.is_some()
            && self.region_ownership_report.is_some()
            && self.arrangement_cell_complex_operation() == Some(request.operation)
            && self.satisfies_request_shape(request)
        {
            let replay = replay_boolean_exact_request_for_result_validation(left, right, request)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
            return if self.matches_retained_replay(&replay) {
                Ok(())
            } else {
                Err(ExactReportValidationError::SourceReplayMismatch)
            };
        }
        let replay = replay_boolean_exact_request_for_result_validation(left, right, request)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self.matches_retained_replay(&replay) {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    fn can_use_retained_arrangement_attempt_for_request(
        &self,
        request: ExactBooleanRequest,
    ) -> bool {
        matches!(
            self.kind,
            ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
                | ExactBooleanResultKind::CertifiedShortcut {
                    shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
                    ..
                }
        ) && self.arrangement_cell_complex_operation() == Some(request.operation)
            && self.satisfies_request_shape(request)
    }

    fn retained_arrangement_attempt_matches_output_for_request(
        &self,
        request: ExactBooleanRequest,
        retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
    ) -> Result<bool, ExactReportValidationError> {
        if !self.can_use_retained_arrangement_attempt_for_request(request) {
            return Ok(false);
        }
        let Some(attempt) = retained_arrangement_attempt else {
            return Ok(false);
        };
        if !attempt.certifies_arrangement_cell_complex_output_for_request(
            request,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        ) {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if attempt.materialized_without_shortcut() {
            if !self.is_arrangement_cell_complex_shortcut_for(request.operation)
                || self.has_arrangement_cell_complex_gate_reports()
            {
                let replay =
                    rematerialize_retained_arrangement_cell_complex_attempt(request, attempt)
                        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
                        .ok_or(ExactReportValidationError::SourceReplayMismatch)?;
                if !retained_output_mesh_matches(&self.mesh, &replay.mesh)
                    || self.topology_assembly_report != replay.topology_assembly_report
                    || self.region_ownership_report != replay.region_ownership_report
                {
                    return Ok(false);
                }
            }
        } else if attempt.materialized_arrangement_cell_complex_shortcut() {
            if let Some((topology, ownership)) = attempt.retained_gate_reports() {
                if self.topology_assembly_report.as_ref() != Some(topology)
                    || self.region_ownership_report.as_ref() != Some(ownership)
                {
                    return Ok(false);
                }
            } else if self.has_arrangement_cell_complex_gate_reports() {
                return Ok(false);
            }
        } else {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        let Some(output_facts) = attempt.output_facts.as_ref() else {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        };
        if self.mesh.vertices().len() != attempt.output_vertices
            || self.mesh.triangles().len() != attempt.output_triangles
            || &self.mesh.facts().mesh != output_facts
        {
            return Ok(false);
        }
        Ok(true)
    }
}

fn certified_shortcut_sources_match(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    if !shortcut_operation_matches(shortcut, operation) {
        return Ok(false);
    }
    match shortcut {
        ExactBooleanShortcutKind::EmptyOperand => {
            Ok(left.triangles().is_empty() || right.triangles().is_empty())
        }
        ExactBooleanShortcutKind::BoundsDisjoint => {
            Ok(meshes_are_certified_bounds_disjoint(left, right))
        }
        ExactBooleanShortcutKind::Identical => Ok(meshes_are_certified_identical(left, right)),
        ExactBooleanShortcutKind::SameSurface => {
            let report = same_surface_report_from_sources(left, right);
            report.validate()?;
            Ok(report.is_certified())
        }
        ExactBooleanShortcutKind::OpenSurfaceDisjoint => {
            let graph = validated_report_intersection_graph(left, right)?;
            let report = open_surface_disjoint_report_from_graph(&graph, left, right);
            report.validate()?;
            Ok(report.is_certified())
        }
        ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid => {
            Ok(mixed_dimensional_regularized_sources(left, right))
        }
        ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid => {
            Ok(lower_dimensional_regularized_sources(left, right))
        }
        ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion
        | ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection
        | ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference => {
            closed_boundary_touching_sources_match(shortcut, left, right)
        }
        ExactBooleanShortcutKind::ClosedWindingSeparated
        | ExactBooleanShortcutKind::ClosedWindingContainment => {
            closed_winding_sources_match(shortcut, left, right)
        }
        ExactBooleanShortcutKind::ConvexContainment
        | ExactBooleanShortcutKind::ConvexUnion
        | ExactBooleanShortcutKind::ConvexIntersection
        | ExactBooleanShortcutKind::ConvexDifference
        | ExactBooleanShortcutKind::ConvexSeparated => {
            convex_shortcut_sources_match(shortcut, left, right)
        }
        ExactBooleanShortcutKind::ArrangementCellComplex => {
            arrangement_cell_complex_sources_match(operation, validation, left, right)
        }
    }
}

fn certified_shortcut_output_matches_sources(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    if !certified_shortcut_sources_match(shortcut, operation, validation, left, right)? {
        return Ok(false);
    }
    Ok(match shortcut {
        ExactBooleanShortcutKind::EmptyOperand => {
            empty_operand_output_matches_sources(operation, validation, mesh, left, right)
        }
        ExactBooleanShortcutKind::BoundsDisjoint => {
            bounds_disjoint_output_matches_sources(operation, validation, mesh, left, right)
        }
        ExactBooleanShortcutKind::Identical => {
            identical_output_matches_sources(operation, validation, mesh, left, right)
        }
        ExactBooleanShortcutKind::SameSurface => {
            !meshes_are_certified_identical(left, right)
                && identical_output_matches_sources(operation, validation, mesh, left, right)
        }
        ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid => {
            if let Some(true) = arrangement_cell_complex_output_matches_sources(
                operation, validation, mesh, left, right,
            )? {
                return Ok(false);
            }
            mixed_dimensional_regularized_output_matches_sources(
                operation, validation, mesh, left, right,
            )
        }
        ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid => {
            if validation == ExactMeshValidationPolicy::CLOSED
                && operation == ExactBooleanOperation::Intersection
                && lower_dimensional_regularized_sources(left, right)
            {
                let graph = validated_report_intersection_graph(left, right)?;
                if !graph.has_unknowns()
                    && !graph.face_pairs.is_empty()
                    && mesh_output_is_empty(mesh)
                {
                    return Ok(false);
                }
            }
            if let Some(true) = arrangement_cell_complex_output_matches_sources(
                operation, validation, mesh, left, right,
            )? {
                return Ok(false);
            }
            validation == ExactMeshValidationPolicy::CLOSED
                && !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
                && mesh_output_is_empty(mesh)
        }
        ExactBooleanShortcutKind::OpenSurfaceDisjoint
        | ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion
        | ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection
        | ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference
        | ExactBooleanShortcutKind::ClosedWindingSeparated
        | ExactBooleanShortcutKind::ClosedWindingContainment
        | ExactBooleanShortcutKind::ConvexContainment
        | ExactBooleanShortcutKind::ConvexUnion
        | ExactBooleanShortcutKind::ConvexIntersection
        | ExactBooleanShortcutKind::ConvexDifference
        | ExactBooleanShortcutKind::ConvexSeparated
        | ExactBooleanShortcutKind::ArrangementCellComplex => false,
    })
}

fn empty_operand_output_matches_sources(
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    if !left.triangles().is_empty() && !right.triangles().is_empty() {
        return false;
    }
    match operation {
        ExactBooleanOperation::Union
            if validation == ExactMeshValidationPolicy::CLOSED
                && mesh_is_lower_dimensional(left)
                && mesh_is_lower_dimensional(right) =>
        {
            mesh_output_is_empty(mesh)
        }
        ExactBooleanOperation::Union => concatenated_mesh_output_matches(mesh, left, right, false),
        ExactBooleanOperation::Intersection => mesh_output_is_empty(mesh),
        ExactBooleanOperation::Difference if left.triangles().is_empty() => {
            mesh_output_is_empty(mesh)
        }
        ExactBooleanOperation::Difference
            if validation == ExactMeshValidationPolicy::CLOSED
                && right.triangles().is_empty()
                && mesh_is_lower_dimensional(left) =>
        {
            mesh_output_is_empty(mesh)
        }
        ExactBooleanOperation::Difference => mesh_output_matches(mesh, left),
        ExactBooleanOperation::SelectedRegions(_) => false,
    }
}

fn bounds_disjoint_output_matches_sources(
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    if left.triangles().is_empty()
        || right.triangles().is_empty()
        || (validation == ExactMeshValidationPolicy::CLOSED
            && (lower_dimensional_regularized_sources(left, right)
                || mixed_dimensional_regularized_sources(left, right)))
    {
        return false;
    }
    match operation {
        ExactBooleanOperation::Union => concatenated_mesh_output_matches(mesh, left, right, false),
        ExactBooleanOperation::Intersection => mesh_output_is_empty(mesh),
        ExactBooleanOperation::Difference => mesh_output_matches(mesh, left),
        ExactBooleanOperation::SelectedRegions(_) => false,
    }
}

fn identical_output_matches_sources(
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    if (mesh_is_closed_solid(left) && mesh_is_closed_solid(right))
        || (validation == ExactMeshValidationPolicy::CLOSED
            && (lower_dimensional_regularized_sources(left, right)
                || mixed_dimensional_regularized_sources(left, right)))
    {
        return false;
    }
    match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection => {
            mesh_output_matches(mesh, left)
        }
        ExactBooleanOperation::Difference => mesh_output_is_empty(mesh),
        ExactBooleanOperation::SelectedRegions(_) => false,
    }
}

fn mixed_dimensional_regularized_output_matches_sources(
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
) -> bool {
    if validation != ExactMeshValidationPolicy::CLOSED
        && meshes_are_certified_bounds_disjoint(left, right)
    {
        return false;
    }
    let left_closed = mesh_is_closed_solid(left);
    let right_closed = mesh_is_closed_solid(right);
    match operation {
        ExactBooleanOperation::Union => {
            (left_closed && mesh_output_matches(mesh, left))
                || (right_closed && mesh_output_matches(mesh, right))
        }
        ExactBooleanOperation::Intersection => mesh_output_is_empty(mesh),
        ExactBooleanOperation::Difference => {
            if left_closed {
                mesh_output_matches(mesh, left)
            } else {
                mesh_output_is_empty(mesh)
            }
        }
        ExactBooleanOperation::SelectedRegions(_) => false,
    }
}

fn retained_split_region_result_matches(
    retained: &ExactBooleanResult,
    replay: &ExactBooleanResult,
) -> bool {
    retained.kind == replay.kind
        && retained.graph_had_unknowns == replay.graph_had_unknowns
        && retained.region_classifications == replay.region_classifications
        && retained.triangulations == replay.triangulations
        && retained.volumetric_classifications == replay.volumetric_classifications
        && retained.assembly == replay.assembly
        && retained_output_mesh_matches(&retained.mesh, &replay.mesh)
}

fn retained_boolean_result_matches(
    retained: &ExactBooleanResult,
    replay: &ExactBooleanResult,
) -> bool {
    retained.kind == replay.kind
        && retained.graph_had_unknowns == replay.graph_had_unknowns
        && retained.region_classifications == replay.region_classifications
        && retained.triangulations == replay.triangulations
        && retained.assembly == replay.assembly
        && retained.volumetric_classifications == replay.volumetric_classifications
        && retained_gate_reports_match(retained, replay)
        && retained_output_mesh_matches(&retained.mesh, &replay.mesh)
}

fn retained_gate_reports_match(retained: &ExactBooleanResult, replay: &ExactBooleanResult) -> bool {
    if retained.topology_assembly_report == replay.topology_assembly_report
        && retained.region_ownership_report == replay.region_ownership_report
    {
        return true;
    }
    matches!(
        retained.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            ..
        }
    ) && retained.topology_assembly_report.is_none()
        && retained.region_ownership_report.is_none()
}

fn retained_output_mesh_matches(left: &ExactMesh, right: &ExactMesh) -> bool {
    mesh_output_matches(left, right)
        && left.bounds() == right.bounds()
        && left.facts().mesh == right.facts().mesh
        && left.validation_policy() == right.validation_policy()
        && left.provenance() == right.provenance()
}

fn validate_shortcut_output_shape(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
    mesh: &ExactMesh,
) -> Result<(), ExactReportValidationError> {
    let requires_empty_output = matches!(
        (shortcut, operation),
        (
            ExactBooleanShortcutKind::BoundsDisjoint
                | ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection
                | ExactBooleanShortcutKind::ClosedWindingSeparated
                | ExactBooleanShortcutKind::ConvexSeparated
                | ExactBooleanShortcutKind::OpenSurfaceDisjoint,
            ExactBooleanOperation::Intersection
        ) | (
            ExactBooleanShortcutKind::Identical | ExactBooleanShortcutKind::SameSurface,
            ExactBooleanOperation::Difference
        )
    );
    let requires_closed_solid_output = matches!(
        shortcut,
        ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion
            | ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference
            | ExactBooleanShortcutKind::ConvexUnion
            | ExactBooleanShortcutKind::ConvexIntersection
            | ExactBooleanShortcutKind::ConvexDifference
    ) || matches!(
        (shortcut, operation),
        (
            ExactBooleanShortcutKind::ClosedWindingSeparated
                | ExactBooleanShortcutKind::ConvexSeparated,
            ExactBooleanOperation::Union | ExactBooleanOperation::Difference
        )
    );
    let requires_empty_or_closed_solid_output = matches!(
        shortcut,
        ExactBooleanShortcutKind::ClosedWindingContainment
            | ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid
            | ExactBooleanShortcutKind::ConvexContainment
    );
    let requires_lower_dimensional_output = matches!(
        shortcut,
        ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid
    );

    if requires_empty_output && !mesh_output_is_empty(mesh) {
        return Err(ExactReportValidationError::StatusEvidenceMismatch);
    }
    if requires_closed_solid_output && !mesh_is_closed_solid(mesh) {
        return Err(ExactReportValidationError::StatusEvidenceMismatch);
    }
    if requires_empty_or_closed_solid_output
        && !mesh.triangles().is_empty()
        && !mesh_is_closed_solid(mesh)
    {
        return Err(ExactReportValidationError::StatusEvidenceMismatch);
    }
    if requires_lower_dimensional_output && !mesh_is_lower_dimensional(mesh) {
        return Err(ExactReportValidationError::StatusEvidenceMismatch);
    }
    Ok(())
}

fn convex_operation_output_matches_sources(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    if !shortcut_operation_matches(shortcut, operation) {
        return Ok(false);
    }
    if matches!(
        shortcut,
        ExactBooleanShortcutKind::ConvexContainment | ExactBooleanShortcutKind::ConvexSeparated
    ) {
        return convex_relation_output_matches_sources(shortcut, operation, mesh, left, right);
    }
    let Some(replay) = (match shortcut {
        ExactBooleanShortcutKind::ConvexUnion => union_closed_convex_solids(left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            .map(|result| result.mesh),
        ExactBooleanShortcutKind::ConvexIntersection => intersect_closed_convex_solids(left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            .map(|result| result.mesh),
        ExactBooleanShortcutKind::ConvexDifference => subtract_closed_convex_solids(left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            .map(|result| result.mesh),
        _ => unreachable!("only convex operation shortcuts are replayed here"),
    }) else {
        return Ok(false);
    };
    replay
        .validate_retained_state()
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    Ok(mesh_output_matches(mesh, &replay))
}

fn convex_relation_output_matches_sources(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    let Some(relation) = certified_convex_relation_from_sources(operation, left, right)? else {
        return Ok(false);
    };
    match relation {
        ReportConvexRelation::Separated => {
            if shortcut != ExactBooleanShortcutKind::ConvexSeparated {
                return Ok(false);
            }
            Ok(match operation {
                ExactBooleanOperation::Union => {
                    concatenated_mesh_output_matches(mesh, left, right, false)
                }
                ExactBooleanOperation::Intersection => mesh_output_is_empty(mesh),
                ExactBooleanOperation::Difference => mesh_output_matches(mesh, left),
                ExactBooleanOperation::SelectedRegions(_) => false,
            })
        }
        ReportConvexRelation::LeftInsideRight => {
            if shortcut != ExactBooleanShortcutKind::ConvexContainment {
                return Ok(false);
            }
            Ok(match operation {
                ExactBooleanOperation::Union => mesh_output_matches(mesh, right),
                ExactBooleanOperation::Intersection => mesh_output_matches(mesh, left),
                ExactBooleanOperation::Difference => mesh_output_is_empty(mesh),
                ExactBooleanOperation::SelectedRegions(_) => false,
            })
        }
        ReportConvexRelation::RightInsideLeft { graph_empty } => {
            if shortcut != ExactBooleanShortcutKind::ConvexContainment {
                return Ok(false);
            }
            Ok(match operation {
                ExactBooleanOperation::Union => mesh_output_matches(mesh, left),
                ExactBooleanOperation::Intersection => mesh_output_matches(mesh, right),
                ExactBooleanOperation::Difference if graph_empty => {
                    concatenated_mesh_output_matches(mesh, left, right, true)
                }
                ExactBooleanOperation::Difference | ExactBooleanOperation::SelectedRegions(_) => {
                    false
                }
            })
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ReportConvexRelation {
    Separated,
    LeftInsideRight,
    RightInsideLeft { graph_empty: bool },
}

fn certified_convex_relation_from_sources(
    operation: ExactBooleanOperation,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ReportConvexRelation>, ExactReportValidationError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let graph = validated_report_intersection_graph(left, right)?;
    if graph.has_unknowns() {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_convex_solid_report(left, right);
    left_in_right
        .validate_against_sources(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    let right_in_left = classify_mesh_vertices_against_convex_solid_report(right, left);
    right_in_left
        .validate_against_sources(right, left)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;

    if graph.face_pairs.is_empty() {
        return Ok(match (left_in_right.relation, right_in_left.relation) {
            (ConvexSolidMeshRelation::StrictlyInside, _) => {
                Some(ReportConvexRelation::LeftInsideRight)
            }
            (_, ConvexSolidMeshRelation::StrictlyInside) => {
                Some(ReportConvexRelation::RightInsideLeft { graph_empty: true })
            }
            (ConvexSolidMeshRelation::Outside, ConvexSolidMeshRelation::Outside) => {
                Some(ReportConvexRelation::Separated)
            }
            _ => None,
        });
    }

    let left_boundary_inside_right =
        convex_boundary_containment_sources_match(&left_in_right, &right_in_left);
    let right_boundary_inside_left =
        convex_boundary_containment_sources_match(&right_in_left, &left_in_right);
    Ok(match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
            if left_boundary_inside_right =>
        {
            Some(ReportConvexRelation::LeftInsideRight)
        }
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
            if right_boundary_inside_left =>
        {
            Some(ReportConvexRelation::RightInsideLeft { graph_empty: false })
        }
        ExactBooleanOperation::Difference if left_boundary_inside_right => {
            Some(ReportConvexRelation::LeftInsideRight)
        }
        _ => None,
    })
}

fn mesh_output_matches(left: &ExactMesh, right: &ExactMesh) -> bool {
    left.vertices().len() == right.vertices().len()
        && left.triangles() == right.triangles()
        && left
            .vertices()
            .iter()
            .zip(right.vertices())
            .all(|(left, right)| points_equal(left, right))
}

fn mesh_output_is_empty(mesh: &ExactMesh) -> bool {
    mesh.vertices().is_empty() && mesh.triangles().is_empty()
}

fn concatenated_mesh_output_matches(
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
    reverse_right: bool,
) -> bool {
    if mesh.vertices().len() != left.vertices().len() + right.vertices().len()
        || mesh.triangles().len() != left.triangles().len() + right.triangles().len()
    {
        return false;
    }
    if !mesh
        .vertices()
        .iter()
        .take(left.vertices().len())
        .zip(left.vertices())
        .all(|(candidate, expected)| points_equal(candidate, expected))
    {
        return false;
    }
    if !mesh
        .vertices()
        .iter()
        .skip(left.vertices().len())
        .zip(right.vertices())
        .all(|(candidate, expected)| points_equal(candidate, expected))
    {
        return false;
    }
    if mesh.triangles()[..left.triangles().len()] != *left.triangles() {
        return false;
    }
    let right_offset = left.vertices().len();
    mesh.triangles()[left.triangles().len()..]
        .iter()
        .zip(right.triangles())
        .all(|(candidate, expected)| {
            let [a, b, c] = expected.0;
            let expected = if reverse_right {
                [a + right_offset, c + right_offset, b + right_offset]
            } else {
                [a + right_offset, b + right_offset, c + right_offset]
            };
            candidate.0 == expected
        })
}

const fn shortcut_operation_matches(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
) -> bool {
    match (shortcut, operation) {
        (_, ExactBooleanOperation::SelectedRegions(_)) => false,
        (
            ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion
            | ExactBooleanShortcutKind::ConvexUnion,
            ExactBooleanOperation::Union,
        )
        | (
            ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection
            | ExactBooleanShortcutKind::ConvexIntersection,
            ExactBooleanOperation::Intersection,
        )
        | (
            ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference
            | ExactBooleanShortcutKind::ConvexDifference,
            ExactBooleanOperation::Difference,
        ) => true,
        (
            ExactBooleanShortcutKind::EmptyOperand
            | ExactBooleanShortcutKind::BoundsDisjoint
            | ExactBooleanShortcutKind::Identical
            | ExactBooleanShortcutKind::SameSurface
            | ExactBooleanShortcutKind::OpenSurfaceDisjoint
            | ExactBooleanShortcutKind::ClosedWindingSeparated
            | ExactBooleanShortcutKind::ClosedWindingContainment
            | ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid
            | ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid
            | ExactBooleanShortcutKind::ConvexContainment
            | ExactBooleanShortcutKind::ConvexSeparated
            | ExactBooleanShortcutKind::ArrangementCellComplex,
            ExactBooleanOperation::Union
            | ExactBooleanOperation::Intersection
            | ExactBooleanOperation::Difference,
        ) => true,
        _ => false,
    }
}

fn meshes_are_certified_bounds_disjoint(left: &ExactMesh, right: &ExactMesh) -> bool {
    if left.validate_retained_bounds_certificate().is_err()
        || right.validate_retained_bounds_certificate().is_err()
    {
        return false;
    }
    let (Some(left_bounds), Some(right_bounds)) = (left.bounds().mesh(), right.bounds().mesh())
    else {
        return left.triangles().is_empty() || right.triangles().is_empty();
    };
    left_bounds.classify_intersection(right_bounds).value() == Some(AabbIntersectionKind::Disjoint)
}

fn meshes_are_certified_identical(left: &ExactMesh, right: &ExactMesh) -> bool {
    left.triangles() == right.triangles()
        && left.vertices().len() == right.vertices().len()
        && left
            .vertices()
            .iter()
            .zip(right.vertices())
            .all(|(left, right)| points_equal(left, right))
}

fn mixed_dimensional_regularized_sources(left: &ExactMesh, right: &ExactMesh) -> bool {
    let left_closed = mesh_is_closed_solid(left);
    let right_closed = mesh_is_closed_solid(right);
    let left_lower = mesh_is_lower_dimensional(left);
    let right_lower = mesh_is_lower_dimensional(right);
    (left_closed && right_lower) || (left_lower && right_closed)
}

fn lower_dimensional_regularized_sources(left: &ExactMesh, right: &ExactMesh) -> bool {
    mesh_is_lower_dimensional(left) && mesh_is_lower_dimensional(right)
}

fn closed_boundary_touching_sources_match(
    shortcut: ExactBooleanShortcutKind,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    if !mesh_is_closed_solid(left) || !mesh_is_closed_solid(right) {
        return Ok(false);
    }
    let graph = validated_report_intersection_graph(left, right)?;
    let report = boundary_touching_report_from_graph(&graph, left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    report.validate()?;
    if !report.is_certified() {
        if matches!(
            shortcut,
            ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection
                | ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference
        ) {
            let graph = validated_report_intersection_graph(left, right)?;
            let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(&graph, left, right);
            evidence
                .validate()
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
            return Ok(
                evidence.obstacle == CoplanarVolumetricCellObstacle::BoundaryOnlyContact
                    && evidence.positive_area_coplanar_overlapping_pairs != 0,
            );
        }
        return Ok(false);
    }
    if shortcut == ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion
        && report.blocker.coplanar_overlapping_pairs != 0
    {
        let graph = validated_report_intersection_graph(left, right)?;
        let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(&graph, left, right);
        evidence
            .validate()
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if evidence.positive_area_coplanar_overlapping_pairs != 0 {
            return Ok(false);
        }
    }
    Ok(true)
}

fn closed_boundary_touching_output_matches_sources(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    if let Some(true) =
        arrangement_cell_complex_output_matches_sources(operation, validation, mesh, left, right)?
    {
        return Ok(false);
    }
    if !closed_boundary_touching_sources_match(shortcut, left, right)? {
        return Ok(false);
    }
    Ok(match (shortcut, operation) {
        (ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion, ExactBooleanOperation::Union) => {
            concatenated_mesh_output_matches(mesh, left, right, false)
        }
        (
            ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection,
            ExactBooleanOperation::Intersection,
        ) => mesh_output_is_empty(mesh),
        (
            ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
            ExactBooleanOperation::Difference,
        ) => mesh_output_matches(mesh, left),
        _ => false,
    })
}

fn closed_winding_sources_match(
    shortcut: ExactBooleanShortcutKind,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    if !mesh_is_closed_solid(left) || !mesh_is_closed_solid(right) {
        return Ok(false);
    }
    let graph = validated_report_intersection_graph(left, right)?;
    if graph.has_unknowns() || !graph.face_pairs.is_empty() {
        return Ok(false);
    }

    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    left_in_right
        .validate_against_sources(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left
        .validate_against_sources(right, left)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;

    Ok(match shortcut {
        ExactBooleanShortcutKind::ClosedWindingSeparated => {
            left_in_right.relation == ClosedMeshWindingMeshRelation::Outside
                && right_in_left.relation == ClosedMeshWindingMeshRelation::Outside
        }
        ExactBooleanShortcutKind::ClosedWindingContainment => {
            left_in_right.relation == ClosedMeshWindingMeshRelation::StrictlyInside
                || right_in_left.relation == ClosedMeshWindingMeshRelation::StrictlyInside
        }
        _ => unreachable!("only closed winding shortcuts are replayed here"),
    })
}

fn closed_winding_output_matches_sources(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
    _validation: ExactMeshValidationPolicy,
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    let Some(relation) = certified_closed_winding_relation_from_sources(left, right)? else {
        return Ok(false);
    };
    match relation {
        ReportClosedWindingRelation::Separated => {
            if shortcut != ExactBooleanShortcutKind::ClosedWindingSeparated {
                return Ok(false);
            }
            Ok(match operation {
                ExactBooleanOperation::Union => {
                    concatenated_mesh_output_matches(mesh, left, right, false)
                }
                ExactBooleanOperation::Intersection => mesh_output_is_empty(mesh),
                ExactBooleanOperation::Difference => mesh_output_matches(mesh, left),
                ExactBooleanOperation::SelectedRegions(_) => false,
            })
        }
        ReportClosedWindingRelation::LeftInsideRight => {
            if shortcut != ExactBooleanShortcutKind::ClosedWindingContainment {
                return Ok(false);
            }
            Ok(match operation {
                ExactBooleanOperation::Union => mesh_output_matches(mesh, right),
                ExactBooleanOperation::Intersection => mesh_output_matches(mesh, left),
                ExactBooleanOperation::Difference => mesh_output_is_empty(mesh),
                ExactBooleanOperation::SelectedRegions(_) => false,
            })
        }
        ReportClosedWindingRelation::RightInsideLeft => {
            if shortcut != ExactBooleanShortcutKind::ClosedWindingContainment {
                return Ok(false);
            }
            Ok(match operation {
                ExactBooleanOperation::Union => mesh_output_matches(mesh, left),
                ExactBooleanOperation::Intersection => mesh_output_matches(mesh, right),
                ExactBooleanOperation::Difference => {
                    concatenated_mesh_output_matches(mesh, left, right, true)
                }
                ExactBooleanOperation::SelectedRegions(_) => false,
            })
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ReportClosedWindingRelation {
    Separated,
    LeftInsideRight,
    RightInsideLeft,
}

fn certified_closed_winding_relation_from_sources(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<ReportClosedWindingRelation>, ExactReportValidationError> {
    if !mesh_is_closed_solid(left) || !mesh_is_closed_solid(right) {
        return Ok(None);
    }
    let graph = validated_report_intersection_graph(left, right)?;
    let counts = ExactBooleanBlocker::from_graph(&graph, ExactBooleanBlockerKind::Winding);
    if graph.has_unknowns()
        || !graph.face_pairs.is_empty()
        || counts.construction_failed_events != 0
    {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_closed_mesh_winding_report(left, right);
    left_in_right
        .validate_against_sources(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left
        .validate_against_sources(right, left)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;

    Ok(match (left_in_right.relation, right_in_left.relation) {
        (ClosedMeshWindingMeshRelation::Outside, ClosedMeshWindingMeshRelation::Outside) => {
            Some(ReportClosedWindingRelation::Separated)
        }
        (ClosedMeshWindingMeshRelation::StrictlyInside, _) => {
            Some(ReportClosedWindingRelation::LeftInsideRight)
        }
        (_, ClosedMeshWindingMeshRelation::StrictlyInside) => {
            Some(ReportClosedWindingRelation::RightInsideLeft)
        }
        _ => None,
    })
}

fn convex_shortcut_sources_match(
    shortcut: ExactBooleanShortcutKind,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    Ok(match shortcut {
        ExactBooleanShortcutKind::ConvexUnion => union_closed_convex_solids(left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            .is_some(),
        ExactBooleanShortcutKind::ConvexIntersection => intersect_closed_convex_solids(left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            .is_some(),
        ExactBooleanShortcutKind::ConvexDifference => subtract_closed_convex_solids(left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            .is_some(),
        ExactBooleanShortcutKind::ConvexContainment | ExactBooleanShortcutKind::ConvexSeparated => {
            convex_relation_shortcut_sources_match(shortcut, left, right)?
        }
        _ => unreachable!("only convex shortcuts are replayed here"),
    })
}

fn convex_relation_shortcut_sources_match(
    shortcut: ExactBooleanShortcutKind,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    let graph = validated_report_intersection_graph(left, right)?;
    if graph.has_unknowns() {
        return Ok(false);
    }
    let left_in_right = classify_mesh_vertices_against_convex_solid_report(left, right);
    left_in_right
        .validate_against_sources(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    let right_in_left = classify_mesh_vertices_against_convex_solid_report(right, left);
    right_in_left
        .validate_against_sources(right, left)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;

    Ok(match shortcut {
        ExactBooleanShortcutKind::ConvexContainment if graph.face_pairs.is_empty() => {
            left_in_right.relation == ConvexSolidMeshRelation::StrictlyInside
                || right_in_left.relation == ConvexSolidMeshRelation::StrictlyInside
        }
        ExactBooleanShortcutKind::ConvexContainment => {
            convex_boundary_containment_sources_match(&left_in_right, &right_in_left)
                || convex_boundary_containment_sources_match(&right_in_left, &left_in_right)
        }
        ExactBooleanShortcutKind::ConvexSeparated => {
            graph.face_pairs.is_empty()
                && left_in_right.relation == ConvexSolidMeshRelation::Outside
                && right_in_left.relation == ConvexSolidMeshRelation::Outside
        }
        _ => unreachable!("only convex relation shortcuts are replayed here"),
    })
}

fn convex_boundary_containment_sources_match(
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

fn arrangement_cell_complex_sources_match(
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    if validation == ExactMeshValidationPolicy::CLOSED
        && lower_dimensional_regularized_sources(left, right)
    {
        return Ok(true);
    }
    if operation == ExactBooleanOperation::Union {
        let (report, _) = adjacent_union_completion_certification(left, right, operation, None)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        report.validate()?;
        if matches!(
            report.status,
            ExactAdjacentUnionCompletionStatus::CertifiedFullFace
                | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
        ) {
            return Ok(true);
        }
    }
    let graph = validated_report_intersection_graph(left, right)?;
    if graph.has_unknowns() {
        return Ok(false);
    }
    if operation == ExactBooleanOperation::Union {
        let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(&graph, left, right);
        evidence
            .validate()
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if evidence.obstacle == CoplanarVolumetricCellObstacle::BoundaryOnlyContact
            && evidence.positive_area_coplanar_overlapping_pairs > 0
        {
            return Ok(true);
        }
    }
    let evaluation = exact_boolean_evaluation_for_replay(
        left,
        right,
        ExactBooleanRequest::new(operation, validation),
    )?;
    let preflight = evaluation.preflight();
    preflight.validate()?;
    Ok(preflight.support == ExactBooleanSupport::CertifiedArrangementCellComplex)
}

fn axis_aligned_orthogonal_solid_operation(
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

fn axis_aligned_orthogonal_solid_output_matches_sources(
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<bool>, ExactReportValidationError> {
    let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(operation) else {
        return Ok(None);
    };
    let Some(replay) = materialize_axis_aligned_orthogonal_solid_cell_output(
        left,
        right,
        solid_operation,
        "exact arrangement orthogonal solid cell replay",
        validation,
    )
    .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
    else {
        return Ok(None);
    };
    Ok(Some(mesh_output_matches(mesh, &replay)))
}

fn arrangement_cell_complex_output_matches_sources(
    operation: ExactBooleanOperation,
    validation: ExactMeshValidationPolicy,
    mesh: &ExactMesh,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Option<bool>, ExactReportValidationError> {
    let mut retained_mismatch = false;
    if let Some(matches_output) = axis_aligned_orthogonal_solid_output_matches_sources(
        operation, validation, mesh, left, right,
    )? {
        if matches_output {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if let Some((replay, closure_report)) =
        materialize_volumetric_coplanar_boundary_closure_output(left, right, operation, validation)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
    {
        closure_report.validate()?;
        if mesh_output_matches(mesh, &replay) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if let Some(replay) =
        replay_generic_arrangement_cell_complex_result(left, right, operation, validation)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
    {
        if mesh_output_matches(mesh, &replay.mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if let Some(replay) = boolean_coplanar_mesh_overlay_optional(left, right, operation, validation)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
    {
        if mesh_output_matches(mesh, &replay.mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    let graph = super::graph::build_unvalidated_intersection_graph(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;

    if let Some((replay, evidence)) =
        materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
            &graph, left, right, operation, validation,
        )
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
    {
        evidence
            .validate()
            .map_err(|_| ExactReportValidationError::StatusEvidenceMismatch)?;
        if mesh_output_matches(mesh, &replay.mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if let Some((replay, evidence)) =
        materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
            &graph, left, right, operation, validation,
        )
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
    {
        evidence
            .validate()
            .map_err(|_| ExactReportValidationError::StatusEvidenceMismatch)?;
        if mesh_output_matches(mesh, &replay.mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if validation == ExactMeshValidationPolicy::CLOSED
        && !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && lower_dimensional_regularized_sources(left, right)
    {
        if mesh_output_is_empty(mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    match operation {
        ExactBooleanOperation::Union => {
            if let Some(replay) = materialize_affine_orthogonal_solid_union(left, right, validation)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            {
                if mesh_output_matches(mesh, &replay.mesh) {
                    return Ok(Some(true));
                }
                retained_mismatch = true;
            }
        }
        ExactBooleanOperation::Intersection => {
            if let Some(replay) =
                materialize_affine_orthogonal_solid_intersection(left, right, validation)
                    .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            {
                if mesh_output_matches(mesh, &replay.mesh) {
                    return Ok(Some(true));
                }
                retained_mismatch = true;
            }
        }
        ExactBooleanOperation::Difference => {
            if let Some(replay) =
                materialize_affine_orthogonal_solid_difference(left, right, validation)
                    .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            {
                if mesh_output_matches(mesh, &replay.mesh) {
                    return Ok(Some(true));
                }
                retained_mismatch = true;
            }
        }
        ExactBooleanOperation::SelectedRegions(_) => return Ok(None),
    }

    if let Some(replay) =
        replay_closed_same_surface_boolean_result_if_certified(left, right, operation, validation)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
    {
        if mesh_output_matches(mesh, &replay.mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if operation != ExactBooleanOperation::Union {
        return Ok(retained_mismatch.then_some(false));
    }

    let (adjacent_report, _) =
        adjacent_union_completion_certification(left, right, operation, None)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    adjacent_report.validate()?;
    match adjacent_report.status {
        ExactAdjacentUnionCompletionStatus::CertifiedFullFace => {
            let Some(replay) = materialize_full_face_adjacent_union(left, right, validation)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            else {
                return Ok(retained_mismatch.then_some(false));
            };
            if mesh_output_matches(mesh, &replay.mesh) {
                return Ok(Some(true));
            }
            retained_mismatch = true;
        }
        ExactAdjacentUnionCompletionStatus::CertifiedContainedFace => {
            let Some(replay) = materialize_contained_face_adjacent_union(left, right, validation)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            else {
                return Ok(retained_mismatch.then_some(false));
            };
            if mesh_output_matches(mesh, &replay.mesh) {
                return Ok(Some(true));
            }
            retained_mismatch = true;
        }
        _ => {}
    }
    if adjacent_report.status() != ExactAdjacentUnionCompletionStatus::NoAdjacencyCertificate {
        return Ok(retained_mismatch.then_some(false));
    }
    if !mesh_is_closed_solid(left) || !mesh_is_closed_solid(right) {
        return Ok(retained_mismatch.then_some(false));
    }
    let graph = validated_report_intersection_graph(left, right)?;
    if graph.has_unknowns() || graph.face_pairs.is_empty() {
        return Ok(retained_mismatch.then_some(false));
    }
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(&graph, left, right);
    evidence
        .validate()
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    if evidence.obstacle == CoplanarVolumetricCellObstacle::BoundaryOnlyContact
        && evidence.positive_area_coplanar_overlapping_pairs > 0
    {
        if concatenated_mesh_output_matches(mesh, left, right, false) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }
    Ok(retained_mismatch.then_some(false))
}

fn mesh_is_closed_solid(mesh: &ExactMesh) -> bool {
    !mesh.triangles().is_empty() && mesh.facts().mesh.closed_manifold
}

fn mesh_is_lower_dimensional(mesh: &ExactMesh) -> bool {
    mesh.triangles().is_empty() || mesh_is_open_surface(mesh)
}

fn mesh_is_open_surface(mesh: &ExactMesh) -> bool {
    !mesh.triangles().is_empty()
        && !mesh.facts().mesh.closed_manifold
        && mesh.facts().mesh.boundary_edges > 0
        && mesh.facts().mesh.non_manifold_edges == 0
        && mesh.facts().mesh.non_manifold_vertices == 0
}

/// Local per-cell retention state for an arrangement-materialized result.
///
/// This mirrors the named-boolean assembly policy inside the report validator
/// so a copied result can be audited without re-running the boolean executor.
/// The retained predicate facts must still justify exactly the emitted
/// combinatorics.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum VolumetricCellRetention {
    Drop,
    Keep,
    KeepReversed,
}

fn validate_volumetric_materialized_assembly_matches_operation(
    operation: ExactBooleanOperation,
    triangulations: &[FaceRegionTriangulation],
    classifications: &[ExactVolumetricRegionClassification],
    assembly: &ExactBooleanAssemblyPlan,
) -> Result<(), ExactReportValidationError> {
    for triangulation in triangulations {
        for triangle in triangulation.triangles.chunks_exact(3) {
            let triangle = [triangle[0], triangle[1], triangle[2]];
            let expected = volumetric_cell_retention_for_operation(
                operation,
                triangulation,
                triangle,
                classifications,
            );
            let retained_source_cells = assembly
                .triangles
                .iter()
                .filter(|output| {
                    output.source_side == triangulation.side
                        && output.source_face == triangulation.face
                        && output_triangle_matches_triangulated_cell(
                            output,
                            assembly,
                            triangulation,
                            triangle,
                        )
                })
                .count();
            let retained_source_subcells = assembly
                .triangles
                .iter()
                .filter(|output| {
                    output.source_side == triangulation.side
                        && output.source_face == triangulation.face
                        && output_triangle_lies_in_triangulated_cell(
                            output,
                            assembly,
                            triangulation,
                            triangle,
                        )
                })
                .count();
            let retained_duplicate_cells = assembly
                .triangles
                .iter()
                .filter(|output| {
                    (output.source_side != triangulation.side
                        || output.source_face != triangulation.face)
                        && output_triangle_matches_triangulated_cell(
                            output,
                            assembly,
                            triangulation,
                            triangle,
                        )
                })
                .count();
            let expected_orientation = match expected {
                VolumetricCellRetention::Keep => {
                    Some(ExactOutputTriangleOrientation::PreserveSource)
                }
                VolumetricCellRetention::KeepReversed => {
                    Some(ExactOutputTriangleOrientation::ReverseSource)
                }
                VolumetricCellRetention::Drop => None,
            };
            if let Some(expected_orientation) = expected_orientation
                && assembly.triangles.iter().any(|output| {
                    output.source_side == triangulation.side
                        && output.source_face == triangulation.face
                        && output_triangle_lies_in_triangulated_cell(
                            output,
                            assembly,
                            triangulation,
                            triangle,
                        )
                        && output.orientation != expected_orientation
                })
            {
                return Err(
                    ExactReportValidationError::VolumetricMaterializedAssemblyViolatesOperation,
                );
            }
            let retained_source_subcells_cover_cell =
                expected_orientation.is_some_and(|expected_orientation| {
                    output_triangles_cover_triangulated_cell(
                        assembly.triangles.iter().filter(|output| {
                            output.source_side == triangulation.side
                                && output.source_face == triangulation.face
                                && output.orientation == expected_orientation
                                && output_triangle_lies_in_triangulated_cell(
                                    output,
                                    assembly,
                                    triangulation,
                                    triangle,
                                )
                        }),
                        assembly,
                        triangulation,
                        triangle,
                    )
                });
            match expected {
                VolumetricCellRetention::Drop
                    if retained_source_cells != 0 || retained_source_subcells != 0 =>
                {
                    return Err(
                        ExactReportValidationError::VolumetricMaterializedAssemblyViolatesOperation,
                    );
                }
                VolumetricCellRetention::Keep | VolumetricCellRetention::KeepReversed
                    if !retained_source_subcells_cover_cell && retained_duplicate_cells == 0 =>
                {
                    return Err(
                        ExactReportValidationError::VolumetricMaterializedAssemblyViolatesOperation,
                    );
                }
                VolumetricCellRetention::Keep
                | VolumetricCellRetention::KeepReversed
                | VolumetricCellRetention::Drop => {}
            }
        }
    }

    Ok(())
}

fn output_triangles_cover_triangulated_cell<'a>(
    outputs: impl Iterator<Item = &'a ExactOutputTriangle>,
    assembly: &ExactBooleanAssemblyPlan,
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
) -> bool {
    let Some(cell_area) = triangulated_cell_projected_area2_abs(triangulation, triangle) else {
        return false;
    };
    let mut output_area = Real::from(0);
    let mut found = false;
    for output in outputs {
        let Some(area) = output_triangle_projected_area2_abs(output, assembly, triangulation)
        else {
            return false;
        };
        output_area += &area;
        found = true;
    }
    found && compare_reals(&output_area, &cell_area).value() == Some(Ordering::Equal)
}

fn triangulated_cell_projected_area2_abs(
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
) -> Option<Real> {
    let points = triangle
        .iter()
        .map(|&vertex| triangulation.boundary.get(vertex).map(boundary_node_point))
        .collect::<Option<Vec<_>>>()?;
    real_abs(&projected_polygon_area2_value(
        &[points[0].clone(), points[1].clone(), points[2].clone()],
        triangulation.projection,
    ))
}

fn output_triangle_projected_area2_abs(
    output: &ExactOutputTriangle,
    assembly: &ExactBooleanAssemblyPlan,
    triangulation: &FaceRegionTriangulation,
) -> Option<Real> {
    let points = output
        .vertices
        .iter()
        .map(|&vertex| assembly.vertices.get(vertex).map(|vertex| &vertex.point))
        .collect::<Option<Vec<_>>>()?;
    real_abs(&projected_polygon_area2_value(
        &[points[0].clone(), points[1].clone(), points[2].clone()],
        triangulation.projection,
    ))
}

fn real_abs(value: &Real) -> Option<Real> {
    match compare_reals(value, &Real::from(0)).value()? {
        Ordering::Less => Some(-value.clone()),
        Ordering::Equal | Ordering::Greater => Some(value.clone()),
    }
}

fn output_triangle_lies_in_triangulated_cell(
    output: &ExactOutputTriangle,
    assembly: &ExactBooleanAssemblyPlan,
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
) -> bool {
    let Some(cell_points) = triangle
        .iter()
        .map(|&vertex| triangulation.boundary.get(vertex).map(boundary_node_point))
        .collect::<Option<Vec<_>>>()
    else {
        return false;
    };
    output.vertices.iter().all(|&vertex| {
        let Some(output_point) = assembly.vertices.get(vertex).map(|vertex| &vertex.point) else {
            return false;
        };
        classify_point_triangle(
            &project_point3(cell_points[0], triangulation.projection),
            &project_point3(cell_points[1], triangulation.projection),
            &project_point3(cell_points[2], triangulation.projection),
            &project_point3(output_point, triangulation.projection),
        )
        .value()
        .is_some_and(|location| {
            matches!(
                location,
                TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
            )
        })
    })
}

fn validate_selected_region_assembly_covers_selection(
    selection: ExactRegionSelection,
    triangulations: &[FaceRegionTriangulation],
    assembly: &ExactBooleanAssemblyPlan,
) -> Result<(), ExactReportValidationError> {
    for triangulation in triangulations {
        if !selection_keeps(selection, triangulation.side) || triangulation.triangles.is_empty() {
            continue;
        }

        // Duplicate exact cells may be canonicalized to one retained
        // topological copy after both sides have supplied the predicate
        // evidence proving coincidence. Every selected cell must still be
        // represented either by its own source label or by an exact duplicate
        // retained from the opposite side.
        let selected_cells_retained = triangulation.triangles.chunks_exact(3).all(|triangle| {
            let triangle = [triangle[0], triangle[1], triangle[2]];
            assembly.triangles.iter().any(|output| {
                output_triangle_matches_triangulated_cell(output, assembly, triangulation, triangle)
            })
        });
        if !selected_cells_retained {
            return Err(ExactReportValidationError::SelectedRegionAssemblyMissingSelectedRegion);
        }
    }

    Ok(())
}

fn volumetric_cell_retention_for_operation(
    operation: ExactBooleanOperation,
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
    classifications: &[ExactVolumetricRegionClassification],
) -> VolumetricCellRetention {
    let Some(classification) = classifications.iter().find(|classification| {
        classification.region_side == triangulation.side
            && classification.region_face == triangulation.face
            && classification.triangle == triangle
    }) else {
        return VolumetricCellRetention::Drop;
    };
    // Boundary cells are exact non-strict facts, not inside/outside guesses.
    // The executor consumes them through the deterministic owner policy
    // documented in `boolean::volumetric_retention_for_operation`: union and
    // intersection keep the left boundary copy and drop the coincident right
    // copy, while difference drops coincident boundary cells. This preserves
    // explicit in retained report validation.
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
        ) => VolumetricCellRetention::Keep,
        (
            ExactBooleanOperation::Difference,
            MeshSide::Right,
            ExactVolumetricRegionRelation::Inside,
        ) => VolumetricCellRetention::KeepReversed,
        _ => VolumetricCellRetention::Drop,
    }
}

fn output_triangle_matches_triangulated_cell(
    output: &ExactOutputTriangle,
    assembly: &ExactBooleanAssemblyPlan,
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
) -> bool {
    let Some(output_points) = output
        .vertices
        .iter()
        .map(|&vertex| assembly.vertices.get(vertex).map(|vertex| &vertex.point))
        .collect::<Option<Vec<_>>>()
    else {
        return false;
    };
    let Some(cell_points) = triangle
        .iter()
        .map(|&vertex| triangulation.boundary.get(vertex).map(boundary_node_point))
        .collect::<Option<Vec<_>>>()
    else {
        return false;
    };
    exact_point_sets_equal(&output_points, &cell_points)
}

fn exact_point_sets_equal(left: &[&Point3], right: &[&Point3]) -> bool {
    if left.len() != right.len() {
        return false;
    }
    let mut matched = vec![false; right.len()];
    for left_point in left {
        let Some(index) = right.iter().enumerate().position(|(index, right_point)| {
            !matched[index] && points_equal(left_point, right_point)
        }) else {
            return false;
        };
        matched[index] = true;
    }
    true
}

fn validate_output_mesh_matches_assembly(
    assembly: &ExactBooleanAssemblyPlan,
    mesh: &ExactMesh,
) -> Result<(), ExactReportValidationError> {
    if assembly.vertices.len() != mesh.vertices().len()
        || assembly.triangles.len() != mesh.triangles().len()
    {
        return Err(ExactReportValidationError::OutputMeshAssemblyMismatch);
    }
    // The materialized mesh is an edge artifact of the retained assembly, not
    // combinatorial chain as part of the exact object state, so the triangle
    // soup returned to callers must replay exactly from the audited assembly
    // plan for both selected-region and arrangement-materialized outputs.
    for (assembly_vertex, mesh_vertex) in assembly.vertices.iter().zip(mesh.vertices()) {
        let mesh_point = mesh_vertex.clone();
        if !points_equal(&assembly_vertex.point, &mesh_point) {
            return Err(ExactReportValidationError::OutputMeshAssemblyMismatch);
        }
    }
    for (assembly_triangle, mesh_triangle) in assembly.triangles.iter().zip(mesh.triangles()) {
        if assembly_triangle.vertices != mesh_triangle.0 {
            return Err(ExactReportValidationError::OutputMeshAssemblyMismatch);
        }
    }
    Ok(())
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    matches!(
        compare_reals(&left.x, &right.x).value(),
        Some(Ordering::Equal)
    ) && matches!(
        compare_reals(&left.y, &right.y).value(),
        Some(Ordering::Equal)
    ) && matches!(
        compare_reals(&left.z, &right.z).value(),
        Some(Ordering::Equal)
    )
}

fn selection_keeps(selection: ExactRegionSelection, side: MeshSide) -> bool {
    matches!(
        (selection, side),
        (ExactRegionSelection::KeepAll, _) | (ExactRegionSelection::KeepLeft, MeshSide::Left)
    )
}

/// Certified support level for a requested exact boolean operation.
///
/// computing as an application-level contract: unresolved combinatorics must be
/// represented explicitly instead of being decided by approximate arithmetic.
/// These variants therefore distinguish executable certified shortcuts from
/// cases whose split regions are available but still need exact winding policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactBooleanSupport {
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
    /// Union was materialized by preserving separate closed shells because
    /// closed solids only touch at exact lower-dimensional boundary features
    /// and share no interior volume.
    CertifiedClosedBoundaryTouchingUnion,
    /// Intersection was certified empty because closed solids only touch at
    /// exact boundary features and share no interior volume.
    CertifiedClosedBoundaryTouchingIntersection,
    /// Difference was certified as the left solid because closed solids only
    /// touch at exact boundary features and share no interior volume.
    CertifiedClosedBoundaryTouchingDifference,
    /// A named operation was answered by exact no-intersection facts for open
    /// surface meshes.
    CertifiedOpenSurfaceDisjoint,
    /// A named operation was answered by an empty exact intersection graph and
    /// replayable closed-mesh winding reports proving both closed solids are
    /// strictly outside the other.
    CertifiedClosedWindingSeparated,
    /// A named operation was answered by an empty exact intersection graph and
    /// replayable closed-mesh winding reports proving one closed solid is
    /// strictly inside the other.
    CertifiedClosedWindingContainment,
    /// A named operation was answered by closed-output regularization for one
    /// closed solid and one lower-dimensional open surface.
    CertifiedMixedDimensionalRegularizedSolid,
    /// A named operation was answered by closed-output regularization for two
    /// lower-dimensional operands, yielding an empty closed solid.
    CertifiedLowerDimensionalRegularizedSolid,
    /// Open non-coplanar surfaces were unioned by exact split-region assembly.
    CertifiedOpenSurfaceArrangementUnion,
    /// Open non-coplanar surfaces were intersected by exact split-region
    /// assembly and projected to an empty triangle mesh because the exact
    /// intersection is lower-dimensional crossing curves.
    CertifiedOpenSurfaceArrangementIntersection,
    /// Open non-coplanar surfaces were differenced by retaining the left split
    /// regions and discarding lower-dimensional crossing curves.
    CertifiedOpenSurfaceArrangementDifference,
    /// A named operation was answered by certified closed-convex containment.
    CertifiedConvexContainment,
    /// Union was materialized for two overlapping closed convex solids.
    CertifiedConvexUnion,
    /// Intersection was materialized for two overlapping closed convex solids.
    CertifiedConvexIntersection,
    /// Difference was materialized for two overlapping closed convex solids.
    CertifiedConvexDifference,
    /// A named operation was answered by a certified no-intersection convex
    /// separated relation that was not caught by mesh-level AABBs.
    CertifiedConvexSeparated,
    /// A named operation was materialized by the exact arrangement/cell-complex
    /// pipeline with specialized surface materializers retained only as proof
    /// fixtures.
    CertifiedArrangementCellComplex,
    /// A caller supplied a certified boundary-output policy, so boundary-only
    /// contact can be projected into triangle-mesh output without treating the
    /// lower-dimensional contact itself as volume.
    CertifiedBoundaryPolicyShortcut,
    /// The retained graph contains certified boundary contact events. This
    /// includes coplanar touching and the closed-solid case where positive-area
    /// coplanar overlaps plus adjacent contact-only candidates are proven
    /// boundary-only by exact winding evidence. A caller must choose a
    /// boundary/shared-feature policy before this can become named boolean
    /// output.
    RequiresBoundaryPolicy,
    /// Coplanar positive-area overlap is certified, but the requested named
    /// output needs planar arrangement materialization.
    RequiresPlanarArrangement,
    /// Closed-volumetric overlap includes coplanar source-face cells that are
    /// not lower-dimensional boundary contact and not an open-surface planar
    /// arrangement.
    RequiresCoplanarVolumetricCells,
    /// Split-region facts were produced, but named winding semantics are not
    /// yet certified for this nontrivial overlap.
    RequiresCertifiedWinding,
    /// Graph extraction retained unresolved predicate events; callers must
    /// refine, reject, or use a policy that explicitly accepts uncertainty.
    UnresolvedGraph,
}

impl ExactBooleanSupport {
    /// Returns whether this support state represents an executable exact
    /// decision rather than a retained blocker.
    pub(crate) const fn is_certified(self) -> bool {
        matches!(
            self,
            Self::SelectedRegionPolicy
                | Self::CertifiedEmptyOperand
                | Self::CertifiedBoundsDisjoint
                | Self::CertifiedIdentical
                | Self::CertifiedSameSurface
                | Self::CertifiedClosedBoundaryTouchingUnion
                | Self::CertifiedClosedBoundaryTouchingIntersection
                | Self::CertifiedClosedBoundaryTouchingDifference
                | Self::CertifiedOpenSurfaceDisjoint
                | Self::CertifiedClosedWindingSeparated
                | Self::CertifiedClosedWindingContainment
                | Self::CertifiedMixedDimensionalRegularizedSolid
                | Self::CertifiedLowerDimensionalRegularizedSolid
                | Self::CertifiedOpenSurfaceArrangementUnion
                | Self::CertifiedOpenSurfaceArrangementIntersection
                | Self::CertifiedOpenSurfaceArrangementDifference
                | Self::CertifiedConvexContainment
                | Self::CertifiedConvexUnion
                | Self::CertifiedConvexIntersection
                | Self::CertifiedConvexDifference
                | Self::CertifiedConvexSeparated
                | Self::CertifiedArrangementCellComplex
                | Self::CertifiedBoundaryPolicyShortcut
        )
    }
}

/// Preflight report for an exact boolean operation request.
///
/// The report gives internal callers a stable way to audit the current
/// implementation boundary. Shortcut variants are retained as materializable
/// exact results. For nontrivial named booleans, the report retains certified
/// split-region plane classifications without dispatching to the specialized
/// tolerance kernel.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactBooleanPreflight {
    /// Requested operation.
    pub(crate) operation: ExactBooleanOperation,
    /// Certified support level for the request.
    pub(crate) support: ExactBooleanSupport,
    /// Whether retained graph events contain explicit unknowns.
    pub(crate) graph_had_unknowns: bool,
    /// Retained face-pair records after exact broad/narrow scheduling.
    pub(crate) retained_face_pairs: usize,
    /// Total retained event records across all retained face pairs.
    pub(crate) retained_events: usize,
    /// Number of split-region boundaries produced for classification.
    pub(crate) region_count: usize,
    /// Certified classifications of split regions against opposite face
    /// planes.
    pub(crate) region_classifications: Vec<FaceRegionPlaneClassification>,
    /// Structured explanation for named operations that are certified enough
    /// to inspect but not yet executable by the selected policy.
    pub(crate) blocker: Option<ExactBooleanBlocker>,
    /// Checked coplanar-overlap evidence retained when preflight stops at a
    /// planar arrangement boundary.
    ///
    /// This keeps positive-area coplanar graph evidence visible to structured
    /// replay instead of flattening it into a generic "unsupported" boolean.
    pub(crate) coplanar_arrangement_evidence: Option<CoplanarArrangementEvidence>,
    /// Source-aware coplanar volumetric-cell evidence retained when the
    /// preflight crosses that exact boundary.
    ///
    /// This report separates boundary-only opposite-side shared faces from
    /// same-side or undecided positive-area coplanar overlap. Retaining it
    /// exact object evidence that authorized a blocker, a no-volume boundary
    /// shortcut, or an arrangement-materialized consumption of coplanar
    /// source-face cells.
    pub(crate) coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
}

/// Closure status for a materialized volumetric boundary-output Boolean.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum ExactVolumetricBoundaryClosureStatus {
    /// No retained volumetric boundary output was materialized for the request.
    NoMaterializedBoundaryOutput,
    /// The materialized output is already closed under the requested topology.
    AlreadyClosed,
    /// Every boundary loop is exactly coplanar and can be handled by the
    /// existing coplanar cap generator.
    CoplanarClosureAvailable,
    /// Boundary loops are valid, but at least one loop is not exactly
    /// coplanar and needs non-coplanar cap-cell generation.
    NonCoplanarBoundaryClosureRequired,
    /// A directed boundary loop reuses an exact 3D point at distinct
    /// topological vertices, so cap construction must first regularize the
    /// self-contact.
    BoundaryLoopExactSelfContact,
    /// Boundary edges could not be organized into simple directed loops.
    BoundaryTopologyNotLoop,
    /// The coplanar loop grouping or closure check hit an exact arrangement
    /// blocker.
    BoundaryClosureBlocked(ExactArrangementBlocker),
}

/// Auditable closure-evidence report for volumetric split-cell output.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ExactVolumetricBoundaryClosureReport {
    /// Requested named operation.
    pub(crate) operation: ExactBooleanOperation,
    /// Certified closure status.
    pub(crate) status: ExactVolumetricBoundaryClosureStatus,
    /// Number of output triangles in the retained boundary materialization.
    pub(crate) output_triangles: usize,
    /// Number of boundary edges retained by the materialized output mesh.
    pub(crate) boundary_edges: usize,
    /// Number of directed boundary loops, when loop extraction succeeded.
    pub(crate) boundary_loops: usize,
    /// Number of boundary vertices whose outgoing directed boundary-edge count
    /// is not exactly one.
    pub(crate) boundary_vertices_with_invalid_outgoing_degree: usize,
    /// Number of boundary vertices whose incoming directed boundary-edge count
    /// is not exactly one.
    pub(crate) boundary_vertices_with_invalid_incoming_degree: usize,
    /// Number of undirected mesh edges used more than twice by output
    /// triangles, proving non-manifold topology before boundary-loop walking.
    pub(crate) overused_boundary_edges: usize,
    /// Number of boundary loops proven not exactly coplanar.
    pub(crate) noncoplanar_boundary_loops: usize,
    /// Number of repeated exact point pairs found inside directed boundary loops.
    pub(crate) repeated_exact_boundary_points: usize,
    /// Number of exact point classes that appear at multiple topological
    /// vertices inside directed boundary loops.
    pub(crate) self_contact_exact_points: usize,
    /// Number of topological boundary vertices participating in exact
    /// self-contact point classes.
    pub(crate) self_contact_topological_vertices: usize,
    /// Number of split cycles around exact self-contact points with fewer than
    /// three distinct exact points.
    pub(crate) self_contact_degenerate_cycles: usize,
    /// Number of split cycles around exact self-contact points with at least
    /// three distinct exact points.
    pub(crate) self_contact_nondegenerate_cycles: usize,
    /// Number of coplanar loop groups produced by exact loop grouping.
    pub(crate) coplanar_loop_groups: usize,
}

impl ExactVolumetricBoundaryClosureReport {
    /// Return the certified closure status.
    pub(crate) const fn status(&self) -> &ExactVolumetricBoundaryClosureStatus {
        &self.status
    }

    /// Return the directed boundary loop count.
    #[cfg(test)]
    pub(crate) const fn boundary_loops(&self) -> usize {
        self.boundary_loops
    }

    /// Return the coplanar loop group count.
    #[cfg(test)]
    pub(crate) const fn coplanar_loop_groups(&self) -> usize {
        self.coplanar_loop_groups
    }

    /// Return whether retained evidence proves coplanar boundary closure is
    /// available for the materialized volumetric output.
    pub(crate) const fn is_coplanar_closure_available(&self) -> bool {
        matches!(
            self.status,
            ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
        )
    }

    /// Validate this report against the source meshes that produced it.
    pub(crate) fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay = if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
            no_materialized_boundary_output_report(self.operation)
        } else {
            let graph = validated_report_intersection_graph(left, right)?;
            volumetric_boundary_closure_report_from_graph(&graph, left, right, self.operation)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
        };
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Validate status and retained closure counts.
    pub(crate) fn validate(&self) -> Result<(), ExactReportValidationError> {
        if self.has_impossible_boundary_count_bounds() {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        match &self.status {
            ExactVolumetricBoundaryClosureStatus::NoMaterializedBoundaryOutput => {
                if self.output_triangles != 0
                    || self.boundary_edges != 0
                    || self.boundary_loops != 0
                    || self.has_boundary_topology_failure_evidence()
                    || self.noncoplanar_boundary_loops != 0
                    || self.repeated_exact_boundary_points != 0
                    || self.self_contact_exact_points != 0
                    || self.self_contact_topological_vertices != 0
                    || self.self_contact_degenerate_cycles != 0
                    || self.self_contact_nondegenerate_cycles != 0
                    || self.coplanar_loop_groups != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactVolumetricBoundaryClosureStatus::AlreadyClosed => {
                if self.boundary_edges != 0
                    || self.boundary_loops != 0
                    || self.has_boundary_topology_failure_evidence()
                    || self.noncoplanar_boundary_loops != 0
                    || self.repeated_exact_boundary_points != 0
                    || self.self_contact_exact_points != 0
                    || self.self_contact_topological_vertices != 0
                    || self.self_contact_degenerate_cycles != 0
                    || self.self_contact_nondegenerate_cycles != 0
                    || self.coplanar_loop_groups != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable => {
                if self.output_triangles == 0
                    || self.boundary_edges == 0
                    || self.boundary_loops == 0
                    || self.has_boundary_topology_failure_evidence()
                    || self.noncoplanar_boundary_loops != 0
                    || self.repeated_exact_boundary_points != 0
                    || self.self_contact_exact_points != 0
                    || self.self_contact_topological_vertices != 0
                    || self.self_contact_degenerate_cycles != 0
                    || self.self_contact_nondegenerate_cycles != 0
                    || self.coplanar_loop_groups == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactVolumetricBoundaryClosureStatus::NonCoplanarBoundaryClosureRequired => {
                if self.output_triangles == 0
                    || self.boundary_edges == 0
                    || self.boundary_loops == 0
                    || self.has_boundary_topology_failure_evidence()
                    || self.noncoplanar_boundary_loops == 0
                    || self.repeated_exact_boundary_points != 0
                    || self.self_contact_exact_points != 0
                    || self.self_contact_topological_vertices != 0
                    || self.self_contact_degenerate_cycles != 0
                    || self.self_contact_nondegenerate_cycles != 0
                    || self.coplanar_loop_groups != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactVolumetricBoundaryClosureStatus::BoundaryLoopExactSelfContact => {
                if self.output_triangles == 0
                    || self.boundary_edges == 0
                    || self.boundary_loops == 0
                    || self.has_boundary_topology_failure_evidence()
                    || self.noncoplanar_boundary_loops != 0
                    || !self.has_valid_self_contact_evidence()
                    || self.coplanar_loop_groups != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactVolumetricBoundaryClosureStatus::BoundaryTopologyNotLoop => {
                if self.output_triangles == 0
                    || self.boundary_edges == 0
                    || self.boundary_loops != 0
                    || !self.has_boundary_topology_failure_evidence()
                    || self.noncoplanar_boundary_loops != 0
                    || self.repeated_exact_boundary_points != 0
                    || self.self_contact_exact_points != 0
                    || self.self_contact_topological_vertices != 0
                    || self.self_contact_degenerate_cycles != 0
                    || self.self_contact_nondegenerate_cycles != 0
                    || self.coplanar_loop_groups != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(blocker) => {
                if self.output_triangles == 0
                    || self.boundary_edges == 0
                    || self.boundary_loops == 0
                    || self.has_boundary_topology_failure_evidence()
                    || !self.has_valid_optional_self_contact_evidence()
                    || !matches!(
                        blocker,
                        ExactArrangementBlocker::UndecidableOrdering
                            | ExactArrangementBlocker::NonManifoldCellComplex
                    )
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if self.coplanar_loop_groups != 0
                    && (*blocker != ExactArrangementBlocker::NonManifoldCellComplex
                        || self.noncoplanar_boundary_loops != 0
                        || self.repeated_exact_boundary_points != 0
                        || self.self_contact_exact_points != 0
                        || self.self_contact_topological_vertices != 0
                        || self.self_contact_degenerate_cycles != 0
                        || self.self_contact_nondegenerate_cycles != 0)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
        }
        Ok(())
    }

    fn has_boundary_topology_failure_evidence(&self) -> bool {
        self.boundary_vertices_with_invalid_outgoing_degree != 0
            || self.boundary_vertices_with_invalid_incoming_degree != 0
            || self.overused_boundary_edges != 0
    }

    fn has_impossible_boundary_count_bounds(&self) -> bool {
        let Some(max_triangle_edges) = self.output_triangles.checked_mul(3) else {
            return true;
        };
        if self.boundary_edges > max_triangle_edges {
            return true;
        }
        if self.boundary_loops != 0 && self.boundary_loops > self.boundary_edges / 3 {
            return true;
        }
        if self.boundary_vertices_with_invalid_outgoing_degree > self.boundary_edges
            || self.boundary_vertices_with_invalid_incoming_degree > self.boundary_edges
            || self.noncoplanar_boundary_loops > self.boundary_loops
            || self.coplanar_loop_groups > self.boundary_loops
        {
            return true;
        }
        if self.overused_boundary_edges > max_triangle_edges {
            return true;
        }
        if self.self_contact_topological_vertices > self.boundary_edges
            || self.self_contact_exact_points > self.self_contact_topological_vertices / 2
        {
            return true;
        }
        let Some(max_repeated_ordered_pairs) = self
            .self_contact_topological_vertices
            .checked_mul(self.self_contact_topological_vertices.saturating_sub(1))
        else {
            return true;
        };
        let max_repeated = max_repeated_ordered_pairs / 2;
        self.repeated_exact_boundary_points > max_repeated
    }

    fn has_valid_optional_self_contact_evidence(&self) -> bool {
        if self.repeated_exact_boundary_points == 0
            && self.self_contact_exact_points == 0
            && self.self_contact_topological_vertices == 0
            && self.self_contact_degenerate_cycles == 0
            && self.self_contact_nondegenerate_cycles == 0
        {
            true
        } else {
            self.has_valid_self_contact_evidence()
        }
    }

    fn has_valid_self_contact_evidence(&self) -> bool {
        let Some(min_topological_vertices) = 2_usize.checked_mul(self.self_contact_exact_points)
        else {
            return false;
        };
        let Some(cycle_count) = self
            .self_contact_degenerate_cycles
            .checked_add(self.self_contact_nondegenerate_cycles)
        else {
            return false;
        };
        self.repeated_exact_boundary_points != 0
            && self.self_contact_exact_points != 0
            && self.self_contact_topological_vertices >= min_topological_vertices
            && self.repeated_exact_boundary_points
                >= self.self_contact_topological_vertices - self.self_contact_exact_points
            && cycle_count == self.self_contact_topological_vertices
    }
}

#[cfg(test)]
fn validate_winding_evidence_against_sources_for_request(
    report: &ExactWindingEvidenceReport,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<(), ExactReportValidationError> {
    let graph = validated_report_intersection_graph(left, right)?;
    if report.operation == request.operation
        && report.status == ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized
        && !graph.has_unknowns()
        && !matches!(report.operation, ExactBooleanOperation::SelectedRegions(_))
        && report.retained_face_pairs == graph.face_pairs.len()
        && report.retained_events == graph.event_count()
        && report.region_count == 0
        && report.region_classifications.is_empty()
        && report.coplanar_arrangement_evidence.is_none()
        && let Some(evidence) = report.coplanar_volumetric_evidence.as_ref()
        && evidence == &CoplanarVolumetricCellEvidenceReport::from_graph(&graph, left, right)
        && volumetric_boundary_closure_report_from_graph(&graph, left, right, report.operation)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            .is_coplanar_closure_available()
    {
        return Ok(());
    }
    if axis_aligned_orthogonal_solid_winding_evidence_matches_sources(report, left, right, request)?
    {
        return Ok(());
    }
    if let Ok(replay) = winding_evidence_report_for_request_from_graph(&graph, left, right, request)
        && report == &replay
    {
        return Ok(());
    }

    if let Ok(evaluation) = exact_boolean_evaluation_for_replay(left, right, request)
        && report == evaluation.certifications().winding_evidence()
    {
        return Ok(());
    }

    // Some retained witnesses, such as selected-region blockers and older
    // lower-dimensional shortcut reports, are still exact even when the
    // canonical evaluation cannot yet return them or supersedes them with an
    // arrangement/cell-complex materialization status.
    Err(ExactReportValidationError::SourceReplayMismatch)
}

#[cfg(test)]
fn axis_aligned_orthogonal_solid_winding_evidence_matches_sources(
    report: &ExactWindingEvidenceReport,
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<bool, ExactReportValidationError> {
    if report.operation != request.operation
        || report.status != ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized
        || matches!(report.operation, ExactBooleanOperation::SelectedRegions(_))
        || report.region_count != 0
        || !report.region_classifications.is_empty()
        || report.coplanar_arrangement_evidence.is_some()
    {
        return Ok(false);
    }
    let graph = validated_report_intersection_graph(left, right)?;
    if graph.has_unknowns() {
        return Ok(false);
    }
    let retains_graph_evidence = report.retained_face_pairs == graph.face_pairs.len()
        && report.retained_events == graph.event_count()
        && report
            .coplanar_volumetric_evidence
            .as_ref()
            .is_none_or(|evidence| {
                evidence == &CoplanarVolumetricCellEvidenceReport::from_graph(&graph, left, right)
            });
    let collapsed_winding_evidence = report.retained_face_pairs == 0
        && report.retained_events == 0
        && report.coplanar_volumetric_evidence.is_none()
        && report.blocker == ExactBooleanBlocker::default();
    if retains_graph_evidence || collapsed_winding_evidence {
        axis_aligned_orthogonal_solid_preflight_matches_sources(left, right, request)
    } else {
        Ok(false)
    }
}

#[cfg(test)]
fn axis_aligned_orthogonal_solid_preflight_matches_sources(
    left: &ExactMesh,
    right: &ExactMesh,
    request: ExactBooleanRequest,
) -> Result<bool, ExactReportValidationError> {
    let Some(solid_operation) = axis_aligned_orthogonal_solid_operation(request.operation) else {
        return Ok(false);
    };
    materialize_axis_aligned_orthogonal_solid_cell_output(
        left,
        right,
        solid_operation,
        "exact arrangement orthogonal solid cell preflight replay",
        request.validation,
    )
    .map(|mesh| mesh.is_some())
    .map_err(|_| ExactReportValidationError::SourceReplayMismatch)
}

impl ExactBooleanPreflight {
    /// Returns whether this preflight has certified support for materializing
    /// the requested operation under the policy used to produce the report.
    pub(crate) fn is_certified(&self) -> bool {
        self.support.is_certified() && self.blocker.is_none()
    }

    /// Validate this preflight report against source meshes and request.
    ///
    /// Boundary-only named booleans are intentionally blocked until a caller
    /// chooses how to project lower-dimensional contact. Request-native replay
    /// preserves that complete choice instead of splitting validation and
    /// boundary policy away from the operation they certify.
    #[cfg(test)]
    pub(crate) fn validate_against_sources_for_request(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        request: ExactBooleanRequest,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let graph = validated_report_intersection_graph(left, right)?;
        if self.operation == request.operation
            && self.support == ExactBooleanSupport::CertifiedArrangementCellComplex
            && self.blocker.is_none()
            && self.retained_face_pairs == graph.face_pairs.len()
            && self.retained_events == graph.event_count()
            && self.region_count == 0
            && self.region_classifications.is_empty()
            && self.coplanar_arrangement_evidence.is_none()
            && let Some(evidence) = self.coplanar_volumetric_evidence.as_ref()
            && evidence == &CoplanarVolumetricCellEvidenceReport::from_graph(&graph, left, right)
            && volumetric_boundary_closure_report_from_graph(&graph, left, right, request.operation)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
                .is_coplanar_closure_available()
        {
            return Ok(());
        }
        if self.operation == request.operation
            && self.support == ExactBooleanSupport::CertifiedArrangementCellComplex
            && self.blocker.is_none()
            && self.retained_face_pairs == graph.face_pairs.len()
            && self.retained_events == graph.event_count()
            && self.region_count == 0
            && self.region_classifications.is_empty()
            && self.coplanar_arrangement_evidence.is_none()
            && self.coplanar_volumetric_evidence.is_none()
            && axis_aligned_orthogonal_solid_preflight_matches_sources(left, right, request)?
        {
            return Ok(());
        }
        if let Ok(replay) = preflight_report_for_request_from_graph(&graph, left, right, request)
            && self == &replay
        {
            return Ok(());
        }
        let replay = exact_boolean_evaluation_for_replay(left, right, request)?
            .preflight()
            .clone();
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Validate support, blocker, and retained artifact consistency.
    pub(crate) fn validate(&self) -> Result<(), ExactReportValidationError> {
        // Preflight connects exact graph construction to later selection and
        // keeps contradictions visible as structured state rather than hiding
        // them behind a boolean success/failure bit.
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)?;
        if self.coplanar_volumetric_evidence.is_some()
            && !matches!(
                self.support,
                ExactBooleanSupport::CertifiedArrangementCellComplex
                    | ExactBooleanSupport::CertifiedIdentical
                    | ExactBooleanSupport::CertifiedSameSurface
                    | ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
                    | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
                    | ExactBooleanSupport::RequiresCoplanarVolumetricCells
                    | ExactBooleanSupport::RequiresCertifiedWinding
            )
        {
            return Err(ExactReportValidationError::UnexpectedCoplanarVolumetricEvidence);
        }
        match self.support {
            ExactBooleanSupport::CertifiedEmptyOperand
            | ExactBooleanSupport::CertifiedBoundsDisjoint
            | ExactBooleanSupport::CertifiedIdentical
            | ExactBooleanSupport::CertifiedSameSurface
            | ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
            | ExactBooleanSupport::CertifiedClosedWindingSeparated
            | ExactBooleanSupport::CertifiedClosedWindingContainment
            | ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid
            | ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid => {
                if self.blocker.is_some() {
                    return Err(ExactReportValidationError::CertifiedReportHasBlocker);
                }
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || !certified_preflight_support_matches_operation(self.support, self.operation)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if let Some(evidence) = self.coplanar_volumetric_evidence.as_ref() {
                    if !matches!(
                        self.support,
                        ExactBooleanSupport::CertifiedIdentical
                            | ExactBooleanSupport::CertifiedSameSurface
                    ) {
                        return Err(
                            ExactReportValidationError::UnexpectedCoplanarVolumetricEvidence,
                        );
                    }
                    validate_coplanar_volumetric_evidence_counts(
                        evidence,
                        self.retained_face_pairs,
                        self.retained_events,
                    )?;
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::CertifiedConvexUnion
            | ExactBooleanSupport::CertifiedConvexIntersection
            | ExactBooleanSupport::CertifiedConvexDifference => {
                if self.blocker.is_some() {
                    return Err(ExactReportValidationError::CertifiedReportHasBlocker);
                }
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || !certified_preflight_support_matches_operation(self.support, self.operation)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
            | ExactBooleanSupport::CertifiedConvexContainment
            | ExactBooleanSupport::CertifiedConvexSeparated => {
                if self.blocker.is_some() {
                    return Err(ExactReportValidationError::CertifiedReportHasBlocker);
                }
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || !certified_preflight_support_matches_operation(self.support, self.operation)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if let Some(evidence) = self.coplanar_volumetric_evidence.as_ref() {
                    if !matches!(
                        self.support,
                        ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
                            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
                    ) {
                        return Err(
                            ExactReportValidationError::UnexpectedCoplanarVolumetricEvidence,
                        );
                    }
                    validate_coplanar_boundary_only_evidence_shape(
                        evidence,
                        self.retained_face_pairs,
                        self.retained_events,
                    )?;
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::CertifiedBoundaryPolicyShortcut => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.blocker.is_some()
                    || self.retained_face_pairs == 0
                    || self.coplanar_arrangement_evidence.is_some()
                    || !certified_preflight_support_matches_operation(self.support, self.operation)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::CertifiedArrangementCellComplex => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.blocker.is_some()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                if let Some(evidence) = self.coplanar_volumetric_evidence.as_ref() {
                    validate_arrangement_materialized_coplanar_evidence(
                        evidence,
                        self.retained_face_pairs,
                        self.retained_events,
                    )?;
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
            | ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
            | ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference => {
                let expected_operation = match self.support {
                    ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion => {
                        ExactBooleanOperation::Union
                    }
                    ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection => {
                        ExactBooleanOperation::Intersection
                    }
                    ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference => {
                        ExactBooleanOperation::Difference
                    }
                    _ => unreachable!("matched open-surface arrangement support"),
                };
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.operation != expected_operation
                    || self.graph_had_unknowns
                    || self.blocker.is_some()
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresBoundaryPolicy => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::BoundaryPolicy,
                )?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::BoundaryPolicy)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresPlanarArrangement => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::PlanarArrangement,
                )?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::PlanarArrangement)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let evidence = self
                    .coplanar_arrangement_evidence
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingCoplanarArrangementEvidence)?;
                evidence
                    .validate()
                    .map_err(|_| ExactReportValidationError::InvalidCoplanarArrangementEvidence)?;
                validate_coplanar_arrangement_evidence_matches_blocker(
                    evidence,
                    self.blocker.as_ref().unwrap(),
                )?;
                if !evidence.needs_planar_cells()
                    || self.blocker.as_ref().unwrap().coplanar_touching_pairs != 0
                {
                    return Err(ExactReportValidationError::CoplanarArrangementEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresCoplanarVolumetricCells => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::CoplanarVolumetricCells,
                )?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::CoplanarVolumetricCells)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                let evidence = self
                    .coplanar_volumetric_evidence
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingCoplanarVolumetricEvidence)?;
                validate_coplanar_volumetric_evidence_matches_blocker(
                    evidence,
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                    return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresCertifiedWinding => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                let blocker = self.blocker.as_ref().unwrap();
                let expected = match blocker.kind {
                    ExactBooleanBlockerKind::CoplanarVolumetricCells => {
                        ExactBooleanBlockerKind::CoplanarVolumetricCells
                    }
                    _ => ExactBooleanBlockerKind::Winding,
                };
                blocker_kind(self.blocker.as_ref(), expected)?;
                blocker.validate_for_kind(expected)?;
                validate_blocker_count_bounds(
                    blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                match (expected, self.coplanar_volumetric_evidence.as_ref()) {
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_matches_blocker(
                            evidence,
                            blocker,
                            self.retained_face_pairs,
                            self.retained_events,
                        )?;
                        if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                            return Err(
                                ExactReportValidationError::CoplanarVolumetricEvidenceMismatch,
                            );
                        }
                    }
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, None) => {
                        return Err(ExactReportValidationError::MissingCoplanarVolumetricEvidence);
                    }
                    (_, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_shape(
                            evidence,
                            self.retained_face_pairs,
                            self.retained_events,
                        )?;
                    }
                    (_, None) => {}
                }
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::UnresolvedGraph => {
                if !self.graph_had_unknowns
                    && !self
                        .blocker
                        .as_ref()
                        .is_some_and(blocker_has_refinement_evidence)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(self.blocker.as_ref(), ExactBooleanBlockerKind::Refinement)?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::Refinement)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::SelectedRegionPolicy => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                if !matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.blocker.is_some()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if self.region_count == 0 {
                    no_region_facts(self.region_count, &self.region_classifications)
                } else {
                    checked_region_facts(self.region_count, &self.region_classifications)
                }
            }
        }
    }
}

/// Missing exact policy or refinement that blocks named boolean output.
///
/// unresolved application-layer topology as first-class state: a caller should
/// be able to distinguish "needs exact winding" from "needs a boundary output
/// policy" or "needs predicate refinement" without interpreting prose
/// diagnostics.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ExactBooleanBlocker {
    /// Missing policy or refinement class.
    pub(crate) kind: ExactBooleanBlockerKind,
    /// Number of retained non-coplanar candidate face pairs.
    pub(crate) candidate_pairs: usize,
    /// Number of retained coplanar positive-overlap face pairs.
    pub(crate) coplanar_overlapping_pairs: usize,
    /// Number of retained coplanar touching face pairs.
    pub(crate) coplanar_touching_pairs: usize,
    /// Number of retained unknown face pairs.
    pub(crate) unknown_pairs: usize,
    /// Number of retained segment/plane events whose endpoint predicates
    /// certified a crossing but whose exact construction failed.
    pub(crate) construction_failed_events: usize,
}

impl Default for ExactBooleanBlocker {
    fn default() -> Self {
        Self {
            kind: ExactBooleanBlockerKind::Winding,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        }
    }
}

impl ExactBooleanBlocker {
    /// Return this exact graph-count blocker with a different semantic kind.
    pub(crate) fn into_blocker(mut self, kind: ExactBooleanBlockerKind) -> Self {
        self.kind = kind;
        self
    }

    /// Build a blocker of `kind` from exact intersection-graph relation
    /// counts.
    ///
    /// This is the shared provenance-count boundary for preflight blockers and
    /// source replay. Keeping the counts on the public blocker shape prevents
    /// executor and report code from drifting on how unknown candidate events
    /// and failed exact constructions are retained.
    pub(crate) fn from_graph(
        graph: &ExactIntersectionGraph,
        kind: ExactBooleanBlockerKind,
    ) -> Self {
        let mut blocker = Self {
            kind,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        };
        for pair in &graph.face_pairs {
            let pair_has_unknown_event = pair
                .events
                .iter()
                .any(IntersectionEvent::has_unknown_relation);
            match pair.relation {
                MeshFacePairRelation::Candidate => blocker.candidate_pairs += 1,
                MeshFacePairRelation::CoplanarOverlapping => {
                    blocker.coplanar_overlapping_pairs += 1;
                }
                MeshFacePairRelation::CoplanarTouching => {
                    blocker.coplanar_touching_pairs += 1;
                }
                MeshFacePairRelation::Unknown => blocker.unknown_pairs += 1,
                MeshFacePairRelation::PlaneSeparated => {}
            }
            if pair.relation != MeshFacePairRelation::Unknown && pair_has_unknown_event {
                blocker.unknown_pairs += 1;
            }
            blocker.construction_failed_events += pair
                .events
                .iter()
                .filter(|event| {
                    matches!(
                        event,
                        IntersectionEvent::SegmentPlane {
                            relation: hyperlimit::SegmentPlaneRelation::ConstructionFailed,
                            ..
                        }
                    )
                })
                .count();
        }
        blocker
    }

    /// Infer the narrowest blocker kind justified by retained graph counts.
    ///
    /// This keeps executor reports and validation replay on the same
    /// provenance-count interpretation: refinement evidence outranks topology
    /// policy, coplanar-only graphs route to planar cells or boundary policy,
    /// mixed coplanar/non-coplanar graphs need volumetric coplanar handling, and
    /// remaining resolved non-coplanar graph state needs winding.
    pub(crate) fn inferred_kind(&self) -> ExactBooleanBlockerKind {
        if blocker_has_refinement_evidence(self) {
            ExactBooleanBlockerKind::Refinement
        } else if self.coplanar_overlapping_pairs != 0 || self.coplanar_touching_pairs != 0 {
            if self.candidate_pairs == 0 && self.coplanar_overlapping_pairs > 0 {
                ExactBooleanBlockerKind::PlanarArrangement
            } else if self.candidate_pairs == 0 {
                ExactBooleanBlockerKind::BoundaryPolicy
            } else {
                ExactBooleanBlockerKind::CoplanarVolumetricCells
            }
        } else {
            ExactBooleanBlockerKind::Winding
        }
    }

    /// Validate that this blocker kind is justified by retained graph relation
    /// counts.
    ///
    /// The counts are exact graph evidence, not decoration. A blocker that
    /// says "needs planar arrangement" while retaining unknown or non-coplanar
    /// candidate pairs would collapse distinct exact computation states into
    /// states to stay explicit.
    pub(crate) fn validate_for_kind(
        &self,
        expected: ExactBooleanBlockerKind,
    ) -> Result<(), ExactReportValidationError> {
        if self.kind != expected {
            return Err(ExactReportValidationError::WrongBlockerKind);
        }
        let valid = match expected {
            ExactBooleanBlockerKind::Refinement => {
                self.unknown_pairs > 0 || self.construction_failed_events > 0
            }
            ExactBooleanBlockerKind::BoundaryPolicy => {
                (self.candidate_pairs != 0
                    || self.coplanar_touching_pairs != 0
                    || self.coplanar_overlapping_pairs != 0)
                    && self.unknown_pairs == 0
                    && self.construction_failed_events == 0
            }
            ExactBooleanBlockerKind::PlanarArrangement => {
                self.coplanar_overlapping_pairs > 0
                    && self.unknown_pairs == 0
                    && self.construction_failed_events == 0
                    && self.candidate_pairs == 0
            }
            ExactBooleanBlockerKind::CoplanarVolumetricCells => {
                (self.coplanar_touching_pairs != 0 || self.coplanar_overlapping_pairs != 0)
                    && self.unknown_pairs == 0
                    && self.construction_failed_events == 0
            }
            ExactBooleanBlockerKind::Winding => {
                self.unknown_pairs == 0
                    && self.construction_failed_events == 0
                    && self.coplanar_overlapping_pairs == 0
                    && self.coplanar_touching_pairs == 0
            }
        };
        if valid {
            Ok(())
        } else {
            Err(ExactReportValidationError::InvalidBlockerCounts)
        }
    }
}

/// Exact boolean preflight blocker kind.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactBooleanBlockerKind {
    /// Predicate or equality refinement is required before policy can run.
    Refinement,
    /// A lower-dimensional shared-boundary output policy is required.
    BoundaryPolicy,
    /// A planar arrangement output model is required for coplanar surfaces.
    PlanarArrangement,
    /// Coplanar source-face cells must be materialized before closed
    /// volumetric winding can decide named output.
    CoplanarVolumetricCells,
    /// Full winding/inside-outside classification is required.
    Winding,
}

/// Certification status for exact refinement preflight.
///
/// Refinement is the stage before application-level topology policy: exact
/// graph extraction retained an unknown predicate outcome or a construction
/// whose endpoint predicates certified an event but whose exact point/parameter
/// from winding or planar-arrangement policy, so it has a separate report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactRefinementStatus {
    /// The graph contains no retained unknowns or construction failures.
    NotRequired,
    /// The graph contains retained evidence that must be refined before policy.
    Required,
}

/// Auditable report for unresolved exact graph refinement.
///
/// This report is intentionally narrower than boolean preflight. It answers
/// only whether exact graph construction is blocked by unknown predicates or
/// failed exact constructions, retaining the graph counts that justify the
/// answer. Later boundary, planar-arrangement, or winding reports should only
/// run as policy once this report is not required.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ExactRefinementReport {
    /// Named operation whose graph was inspected.
    pub(crate) operation: ExactBooleanOperation,
    /// Coarse refinement status.
    pub(crate) status: ExactRefinementStatus,
    /// Whether graph extraction retained unknown predicate outcomes.
    pub(crate) graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub(crate) retained_face_pairs: usize,
    /// Total retained event records.
    pub(crate) retained_events: usize,
    /// Refinement blocker counts, present only when refinement is required.
    pub(crate) blocker: Option<ExactBooleanBlocker>,
}

impl ExactRefinementReport {
    /// Return whether graph extraction retained unknown predicate outcomes.
    pub(crate) const fn graph_had_unknowns(&self) -> bool {
        self.graph_had_unknowns
    }

    /// Return the retained face-pair record count.
    pub(crate) const fn retained_face_pairs(&self) -> usize {
        self.retained_face_pairs
    }

    /// Return the retained event record count.
    pub(crate) const fn retained_events(&self) -> usize {
        self.retained_events
    }

    /// Validate status, retained counts, and refinement blocker consistency.
    pub(crate) fn validate(&self) -> Result<(), ExactReportValidationError> {
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)
            .map_err(|_| ExactReportValidationError::InvalidBlockerCounts)?;
        match self.status {
            ExactRefinementStatus::Required => {
                blocker_kind(self.blocker.as_ref(), ExactBooleanBlockerKind::Refinement)?;
                let blocker = self.blocker.as_ref().unwrap();
                blocker.validate_for_kind(ExactBooleanBlockerKind::Refinement)?;
                validate_blocker_count_bounds(
                    blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.graph_had_unknowns != (blocker.unknown_pairs > 0) {
                    return Err(ExactReportValidationError::InvalidBlockerCounts);
                }
            }
            ExactRefinementStatus::NotRequired => {
                if self.blocker.is_some() {
                    return Err(ExactReportValidationError::UnexpectedGraphEvents);
                }
                if self.graph_had_unknowns {
                    return Err(ExactReportValidationError::InvalidBlockerCounts);
                }
            }
        }
        Ok(())
    }
}

/// Certification status for same-surface named boolean shortcuts.
///
/// Same-surface detection is stronger than identical storage equality: it
/// allows vertex reindexing and face orientation changes when exact coordinate
/// equality proves a bijection and the remapped triangle vertex sets match.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactSameSurfaceStatus {
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
/// predicate certificates used to prove coordinate equality.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ExactSameSurfaceReport {
    /// Coarse same-surface certification status.
    pub(crate) status: ExactSameSurfaceStatus,
    /// Mapping from each left vertex index to the matched right vertex index.
    pub(crate) left_to_right: Vec<usize>,
    /// Mapping from each right vertex index to the matched left vertex index.
    pub(crate) right_to_left: Vec<usize>,
    /// Sorted left triangle vertex sets.
    pub(crate) left_triangles: Vec<[usize; 3]>,
    /// Sorted right triangle vertex sets remapped into left vertex indices.
    pub(crate) right_triangles: Vec<[usize; 3]>,
    /// Predicate certificates used by exact coordinate equality checks.
    pub(crate) predicates: Vec<PredicateUse>,
}

impl ExactSameSurfaceReport {
    /// Return whether same-surface equivalence was certified.
    pub(crate) const fn is_certified(&self) -> bool {
        matches!(self.status, ExactSameSurfaceStatus::Certified)
    }

    /// Return whether every retained predicate route was proof-producing.
    pub(crate) fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }

    /// Validate same-surface report invariants.
    ///
    /// Rejection statuses are still evidence states: count mismatches must not
    /// retain coordinate predicates, vertex-matching failures may keep only the
    /// partial left-to-right matches and predicate trail, and triangle-set
    /// mismatches must retain a valid full vertex permutation.
    pub(crate) fn validate(&self) -> Result<(), ExactReportValidationError> {
        match self.status {
            ExactSameSurfaceStatus::VertexCountMismatch
            | ExactSameSurfaceStatus::TriangleCountMismatch => {
                if !self.left_to_right.is_empty()
                    || !self.right_to_left.is_empty()
                    || !self.left_triangles.is_empty()
                    || !self.right_triangles.is_empty()
                    || !self.predicates.is_empty()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactSameSurfaceStatus::VertexMatchingUndecided
            | ExactSameSurfaceStatus::VertexCoordinateMismatch => {
                let mut seen_right_vertices = Vec::with_capacity(self.left_to_right.len());
                if !self.right_to_left.is_empty()
                    || !self.left_triangles.is_empty()
                    || !self.right_triangles.is_empty()
                    || self.predicates.is_empty()
                    || self.left_to_right.iter().any(|&right| {
                        if seen_right_vertices.contains(&right) {
                            true
                        } else {
                            seen_right_vertices.push(right);
                            false
                        }
                    })
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if matches!(
                    self.status,
                    ExactSameSurfaceStatus::VertexCoordinateMismatch
                ) && !self.all_proof_producing()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactSameSurfaceStatus::TriangleSetMismatch => {
                validate_full_permutation(&self.left_to_right, &self.right_to_left)?;
                if self.left_triangles.is_empty()
                    || self.right_triangles.is_empty()
                    || self.left_triangles == self.right_triangles
                {
                    return Err(ExactReportValidationError::MismatchedTriangleSets);
                }
            }
            ExactSameSurfaceStatus::Certified => {
                validate_full_permutation(&self.left_to_right, &self.right_to_left)?;
                if self.left_triangles != self.right_triangles {
                    return Err(ExactReportValidationError::MismatchedTriangleSets);
                }
                if !self.all_proof_producing() {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
        }
        Ok(())
    }
}

fn validate_full_permutation(
    left_to_right: &[usize],
    right_to_left: &[usize],
) -> Result<(), ExactReportValidationError> {
    if left_to_right.len() != right_to_left.len() {
        return Err(ExactReportValidationError::InvalidPermutation);
    }
    for (left, &right) in left_to_right.iter().enumerate() {
        if right >= right_to_left.len() || right_to_left[right] != left {
            return Err(ExactReportValidationError::InvalidPermutation);
        }
    }
    Ok(())
}

/// Certification status for an open-surface disjoint shortcut.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactOpenSurfaceDisjointStatus {
    /// At least one input is not an open surface mesh under exact validation facts.
    NotOpenSurface,
    /// Exact graph extraction retained unresolved events.
    GraphUnknowns,
    /// Exact graph extraction retained intersections or contacts.
    GraphHasFacePairs,
    /// Both inputs are open surfaces and the exact graph has no retained pairs.
    Certified,
}

/// Auditable report for certified open-surface disjointness.
///
/// This report retains the mesh-shape precondition and exact graph relation
/// as a hidden primitive-float or AABB-only decision.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ExactOpenSurfaceDisjointReport {
    /// Coarse certification status.
    pub(crate) status: ExactOpenSurfaceDisjointStatus,
    /// Whether the left mesh satisfies the exact open-surface precondition.
    pub(crate) left_open_surface: bool,
    /// Whether the right mesh satisfies the exact open-surface precondition.
    pub(crate) right_open_surface: bool,
    /// Whether graph extraction retained unknown events.
    pub(crate) graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub(crate) retained_face_pairs: usize,
    /// Total retained event records.
    pub(crate) retained_events: usize,
    /// Relation counts for retained face pairs.
    pub(crate) blocker: ExactBooleanBlocker,
}

impl ExactOpenSurfaceDisjointReport {
    /// Return whether open-surface disjointness was certified.
    pub(crate) const fn is_certified(&self) -> bool {
        matches!(self.status, ExactOpenSurfaceDisjointStatus::Certified)
    }

    /// Validate this open-surface report against the source meshes.
    ///
    /// Open-surface disjointness is certified graph absence plus mesh-shape
    /// preconditions. This method recomputes both from `left` and `right`
    pub(crate) fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let graph = validated_report_intersection_graph(left, right)?;
        let replay = open_surface_disjoint_report_from_graph(&graph, left, right);
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Validate status, graph counts, and blocker consistency.
    pub(crate) fn validate(&self) -> Result<(), ExactReportValidationError> {
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)?;
        if matches!(self.status, ExactOpenSurfaceDisjointStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        let expected_kind = match self.status {
            ExactOpenSurfaceDisjointStatus::GraphUnknowns => ExactBooleanBlockerKind::Refinement,
            ExactOpenSurfaceDisjointStatus::NotOpenSurface
            | ExactOpenSurfaceDisjointStatus::GraphHasFacePairs
            | ExactOpenSurfaceDisjointStatus::Certified => self.blocker.inferred_kind(),
        };
        blocker_kind(Some(&self.blocker), expected_kind)?;
        self.blocker.validate_for_kind(expected_kind)?;
        validate_refinement_partition(
            matches!(self.status, ExactOpenSurfaceDisjointStatus::GraphUnknowns),
            &self.blocker,
        )?;
        validate_blocker_count_bounds(
            &self.blocker,
            self.retained_face_pairs,
            self.retained_events,
        )?;
        // Status is certified combinatorial state, not a label layered over
        // mesh-shape preconditions and graph evidence.
        if matches!(self.status, ExactOpenSurfaceDisjointStatus::NotOpenSurface) {
            if (self.left_open_surface && self.right_open_surface)
                || self.graph_had_unknowns
                || self.retained_face_pairs != 0
                || self.retained_events != 0
                || blocker_has_any_evidence(&self.blocker)
            {
                return Err(ExactReportValidationError::StatusEvidenceMismatch);
            }
        } else if !self.left_open_surface || !self.right_open_surface {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if self.is_certified() && (self.retained_face_pairs != 0 || self.retained_events != 0) {
            return Err(ExactReportValidationError::UnexpectedGraphEvents);
        }
        if self.status == ExactOpenSurfaceDisjointStatus::GraphHasFacePairs
            && self.retained_face_pairs == 0
        {
            return Err(ExactReportValidationError::MissingRelationCount);
        }
        Ok(())
    }
}

/// Certification status for boundary-only graph shortcuts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactBoundaryTouchingStatus {
    /// Exact graph extraction retained unresolved events.
    GraphUnknowns,
    /// Retained graph pairs were not exclusively certified boundary-only
    /// contact pairs.
    NotBoundaryOnly,
    /// The graph contains certified boundary-only contact pairs. Closed-solid
    /// contact may be positive-area coplanar overlap, edge touch, or vertex
    /// touch; source replay must prove retained candidate pairs contain
    /// contact-only events before this status is constructed.
    Certified,
}

/// Auditable report for certified boundary-only contacts.
///
/// Boundary-only contacts require a caller-selected output policy because a
/// triangle mesh cannot encode the lower-dimensional intersection itself.
/// This report retains the exact graph counts that justify that policy gap,
/// computation sense.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ExactBoundaryTouchingReport {
    /// Coarse boundary-touching certification status.
    pub(crate) status: ExactBoundaryTouchingStatus,
    /// Whether graph extraction retained unknown events.
    pub(crate) graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub(crate) retained_face_pairs: usize,
    /// Total retained event records.
    pub(crate) retained_events: usize,
    /// Relation counts for retained face pairs.
    pub(crate) blocker: ExactBooleanBlocker,
}

/// Certification status for closed-solid adjacent union completion.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactAdjacentUnionCompletionStatus {
    /// The requested operation is not union, so this completion path cannot
    /// apply.
    NotUnion,
    /// At least one source mesh is not a closed manifold.
    NotClosedSolid,
    /// The operands are axis-aligned boxes handled by a stronger orthogonal
    /// solid certificate.
    AxisAlignedBoxPair,
    /// Another exact materializer owns dispatcher provenance for this case.
    StrongerKernelAvailable,
    /// Exact graph extraction retained unresolved or failed construction
    /// evidence.
    GraphUnresolved,
    /// No supported full-face or contained-face adjacency certificate replayed
    /// from these sources.
    NoAdjacencyCertificate,
    /// A full-face or full-patch adjacency certificate replays and can
    /// materialize the union.
    CertifiedFullFace,
    /// A contained-face adjacency certificate replays and can materialize the
    /// union.
    CertifiedContainedFace,
}

/// Auditable report for adjacent closed-solid union completion.
///
/// This report is the decision certificate for the boolean wrapper around the
/// full-face and contained-face adjacency union artifacts. It retains exact
/// graph counts plus the public consumed topology counts, while
/// [`Self::validate_against_sources`] recomputes the private adjacency
/// certificates to prove the report still belongs to the supplied sources.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ExactAdjacentUnionCompletionReport {
    /// Requested named operation.
    pub(crate) operation: ExactBooleanOperation,
    /// Coarse certification status.
    pub(crate) status: ExactAdjacentUnionCompletionStatus,
    /// Whether the left source mesh was a closed manifold.
    pub(crate) left_closed: bool,
    /// Whether the right source mesh was a closed manifold.
    pub(crate) right_closed: bool,
    /// Whether the stronger axis-aligned box path owns this pair.
    pub(crate) axis_aligned_box_pair: bool,
    /// Whether another exact kernel should materialize this union first.
    pub(crate) stronger_kernel_available: bool,
    /// Whether graph extraction retained unknown events.
    pub(crate) graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub(crate) retained_face_pairs: usize,
    /// Total retained event records.
    pub(crate) retained_events: usize,
    /// Relation counts for retained face pairs.
    pub(crate) blocker: ExactBooleanBlocker,
    /// Count of exact whole-face pairs consumed by full-face completion.
    pub(crate) full_face_shared_faces: usize,
    /// Count of exact source-owned full patches consumed by full-face
    /// completion.
    pub(crate) full_face_shared_patches: usize,
    /// Source side whose faces contain the opposite caps for contained-face
    /// completion.
    pub(crate) contained_containing_side: Option<MeshSide>,
    /// Count of opposite-source faces removed by contained-face completion.
    pub(crate) contained_faces: usize,
    /// Count of source faces replaced by holed remnants in contained-face
    /// completion.
    pub(crate) containing_faces: usize,
}

impl ExactAdjacentUnionCompletionReport {
    /// Return the requested named operation.
    pub(crate) const fn operation(&self) -> ExactBooleanOperation {
        self.operation
    }

    /// Return the coarse adjacent-union completion status.
    pub(crate) const fn status(&self) -> ExactAdjacentUnionCompletionStatus {
        self.status
    }

    /// Return whether graph extraction retained unknown events.
    pub(crate) const fn graph_had_unknowns(&self) -> bool {
        self.graph_had_unknowns
    }

    /// Return the retained face-pair record count.
    pub(crate) const fn retained_face_pairs(&self) -> usize {
        self.retained_face_pairs
    }

    /// Return the retained event record count.
    pub(crate) const fn retained_events(&self) -> usize {
        self.retained_events
    }

    /// Return whether adjacent union completion was certified.
    pub(crate) const fn is_certified(&self) -> bool {
        matches!(
            self.status,
            ExactAdjacentUnionCompletionStatus::CertifiedFullFace
                | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
        )
    }

    /// Validate status, graph counts, and consumed topology counts.
    pub(crate) fn validate(&self) -> Result<(), ExactReportValidationError> {
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)?;
        if matches!(
            self.status,
            ExactAdjacentUnionCompletionStatus::GraphUnresolved
        ) && !self.graph_had_unknowns
            && self.blocker.construction_failed_events == 0
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        if !matches!(
            self.status,
            ExactAdjacentUnionCompletionStatus::GraphUnresolved
        ) && (self.graph_had_unknowns || self.blocker.construction_failed_events != 0)
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        let expected_kind = match self.status {
            ExactAdjacentUnionCompletionStatus::GraphUnresolved => {
                ExactBooleanBlockerKind::Refinement
            }
            ExactAdjacentUnionCompletionStatus::CertifiedFullFace
            | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace => {
                ExactBooleanBlockerKind::BoundaryPolicy
            }
            _ => self.blocker.inferred_kind(),
        };
        blocker_kind(Some(&self.blocker), expected_kind)?;
        if matches!(
            self.status,
            ExactAdjacentUnionCompletionStatus::CertifiedFullFace
                | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
        ) {
            validate_adjacent_certified_boundary_blocker(
                &self.blocker,
                self.retained_face_pairs,
                self.retained_events,
            )?;
        } else {
            self.blocker.validate_for_kind(expected_kind)?;
        }
        validate_refinement_partition(
            matches!(
                self.status,
                ExactAdjacentUnionCompletionStatus::GraphUnresolved
            ),
            &self.blocker,
        )?;
        validate_blocker_count_bounds(
            &self.blocker,
            self.retained_face_pairs,
            self.retained_events,
        )?;

        let Some(full_face_counts) = self
            .full_face_shared_faces
            .checked_add(self.full_face_shared_patches)
        else {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        };
        let Some(contained_counts) = self.contained_faces.checked_add(self.containing_faces) else {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        };
        if (self.retained_face_pairs != 0 && full_face_counts > self.retained_face_pairs)
            || self.contained_faces > self.retained_face_pairs
            || self.containing_faces > self.retained_face_pairs
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if matches!(
            self.status,
            ExactAdjacentUnionCompletionStatus::NotUnion
                | ExactAdjacentUnionCompletionStatus::NotClosedSolid
                | ExactAdjacentUnionCompletionStatus::AxisAlignedBoxPair
        ) && (self.retained_face_pairs != 0
            || self.retained_events != 0
            || self.blocker.candidate_pairs != 0
            || self.blocker.coplanar_overlapping_pairs != 0
            || self.blocker.coplanar_touching_pairs != 0
            || self.blocker.unknown_pairs != 0)
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        match self.status {
            ExactAdjacentUnionCompletionStatus::NotUnion => {
                if matches!(self.operation, ExactBooleanOperation::Union) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactAdjacentUnionCompletionStatus::NotClosedSolid => {
                if self.operation != ExactBooleanOperation::Union
                    || (self.left_closed && self.right_closed)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactAdjacentUnionCompletionStatus::AxisAlignedBoxPair => {
                if self.operation != ExactBooleanOperation::Union
                    || !self.left_closed
                    || !self.right_closed
                    || !self.axis_aligned_box_pair
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactAdjacentUnionCompletionStatus::StrongerKernelAvailable => {
                if self.operation != ExactBooleanOperation::Union
                    || !self.left_closed
                    || !self.right_closed
                    || self.axis_aligned_box_pair
                    || !self.stronger_kernel_available
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactAdjacentUnionCompletionStatus::GraphUnresolved => {
                if self.operation != ExactBooleanOperation::Union
                    || !self.left_closed
                    || !self.right_closed
                    || self.axis_aligned_box_pair
                    || self.stronger_kernel_available
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactAdjacentUnionCompletionStatus::NoAdjacencyCertificate => {
                if self.operation != ExactBooleanOperation::Union
                    || !self.left_closed
                    || !self.right_closed
                    || self.axis_aligned_box_pair
                    || self.stronger_kernel_available
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactAdjacentUnionCompletionStatus::CertifiedFullFace => {
                if self.operation != ExactBooleanOperation::Union
                    || !self.left_closed
                    || !self.right_closed
                    || self.axis_aligned_box_pair
                    || self.stronger_kernel_available
                    || full_face_counts == 0
                    || contained_counts != 0
                    || self.contained_containing_side.is_some()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                validate_adjacent_certified_boundary_blocker(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
            }
            ExactAdjacentUnionCompletionStatus::CertifiedContainedFace => {
                if self.operation != ExactBooleanOperation::Union
                    || !self.left_closed
                    || !self.right_closed
                    || self.axis_aligned_box_pair
                    || self.stronger_kernel_available
                    || full_face_counts != 0
                    || self.contained_faces == 0
                    || self.containing_faces == 0
                    || self.contained_containing_side.is_none()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                validate_adjacent_certified_boundary_blocker(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
            }
        }
        if !self.is_certified()
            && (full_face_counts != 0
                || contained_counts != 0
                || self.contained_containing_side.is_some())
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        Ok(())
    }

    /// Validate this report by replaying adjacency completion from source
    /// operands.
    pub(crate) fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let (replay, _) =
            adjacent_union_completion_certification(left, right, self.operation, None)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }
}

impl ExactBoundaryTouchingReport {
    /// Return whether graph extraction retained unknown events.
    pub(crate) const fn graph_had_unknowns(&self) -> bool {
        self.graph_had_unknowns
    }

    /// Return the retained face-pair record count.
    pub(crate) const fn retained_face_pairs(&self) -> usize {
        self.retained_face_pairs
    }

    /// Return the retained event record count.
    pub(crate) const fn retained_events(&self) -> usize {
        self.retained_events
    }

    /// Return whether the graph is certified boundary-only contact.
    pub(crate) const fn is_certified(&self) -> bool {
        matches!(self.status, ExactBoundaryTouchingStatus::Certified)
    }

    /// Validate status, retained relation counts, and blocker consistency.
    pub(crate) fn validate(&self) -> Result<(), ExactReportValidationError> {
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)?;
        if matches!(self.status, ExactBoundaryTouchingStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        let expected_kind = match self.status {
            ExactBoundaryTouchingStatus::GraphUnknowns => ExactBooleanBlockerKind::Refinement,
            ExactBoundaryTouchingStatus::Certified => ExactBooleanBlockerKind::BoundaryPolicy,
            ExactBoundaryTouchingStatus::NotBoundaryOnly => {
                let coplanar_pairs = self.blocker.coplanar_overlapping_pairs != 0
                    || self.blocker.coplanar_touching_pairs != 0;
                if blocker_has_refinement_evidence(&self.blocker) {
                    ExactBooleanBlockerKind::Refinement
                } else if self.blocker.candidate_pairs == 0 && !coplanar_pairs {
                    ExactBooleanBlockerKind::Winding
                } else if self.blocker.candidate_pairs == 0
                    && self.blocker.coplanar_overlapping_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                } else if coplanar_pairs {
                    if self.blocker.candidate_pairs == 0
                        && self.blocker.coplanar_overlapping_pairs > 0
                    {
                        ExactBooleanBlockerKind::PlanarArrangement
                    } else {
                        ExactBooleanBlockerKind::CoplanarVolumetricCells
                    }
                } else {
                    ExactBooleanBlockerKind::Winding
                }
            }
        };
        blocker_kind(Some(&self.blocker), expected_kind)?;
        self.blocker.validate_for_kind(expected_kind)?;
        validate_refinement_partition(
            matches!(self.status, ExactBoundaryTouchingStatus::GraphUnknowns),
            &self.blocker,
        )?;
        validate_blocker_count_bounds(
            &self.blocker,
            self.retained_face_pairs,
            self.retained_events,
        )?;
        if self.is_certified()
            && self.blocker.candidate_pairs == 0
            && self.blocker.coplanar_touching_pairs == 0
            && self.blocker.coplanar_overlapping_pairs == 0
        {
            return Err(ExactReportValidationError::MissingRelationCount);
        }
        if self.is_certified() {
            self.blocker
                .validate_for_kind(ExactBooleanBlockerKind::BoundaryPolicy)?;
        }
        Ok(())
    }

    /// Validate this boundary-touching report against the source meshes.
    ///
    /// Boundary-only contact is a policy boundary over a resolved exact graph.
    /// Recomputing the report from the source meshes ensures the retained
    pub(crate) fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let graph = validated_report_intersection_graph(left, right)?;
        let replay = boundary_touching_report_from_graph(&graph, left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }
}

/// Certification status for planar-arrangement blockers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactPlanarArrangementStatus {
    /// Selected-region assembly already carries its own explicit region policy.
    NotNamedOperation,
    /// Exact graph extraction retained unresolved events.
    GraphUnknowns,
    /// The requested named operation is already handled by a narrower certified
    /// coplanar surface output path.
    AlreadyMaterialized,
    /// The exact graph does not consist solely of positive-area coplanar
    /// overlaps requiring planar arrangement output.
    NoPositiveOverlap,
    /// Closed-solid coplanar contact was certified as a boundary-only policy
    /// case before planar-cell output should be considered.
    BoundaryPolicyRequired,
    /// Certified positive-area coplanar overlap requires a planar arrangement
    /// output model before this named operation can be materialized.
    Required,
}

/// Auditable report for planar-arrangement work left at the exact boundary.
///
/// Coplanar positive-area overlaps are real topology, not numerical noise.
/// This report records when the exact graph proves that a named intersection,
/// union, or difference needs planar arrangement materialization instead of a
/// volumetric winding rule. Narrow single-triangle outputs are reported
/// topology is explicit certified state rather than an approximate fallback.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ExactPlanarArrangementReport {
    /// Requested named operation.
    pub(crate) operation: ExactBooleanOperation,
    /// Coarse planar-arrangement certification status.
    pub(crate) status: ExactPlanarArrangementStatus,
    /// Whether graph extraction retained unknown events.
    pub(crate) graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub(crate) retained_face_pairs: usize,
    /// Total retained event records.
    pub(crate) retained_events: usize,
    /// Relation counts for retained face pairs.
    pub(crate) blocker: ExactBooleanBlocker,
    /// Checked coplanar-overlap evidence summary retained from the graph
    /// layer.
    pub(crate) coplanar_arrangement_evidence: Option<CoplanarArrangementEvidence>,
}

impl ExactPlanarArrangementReport {
    /// Return the requested named operation.
    pub(crate) const fn operation(&self) -> ExactBooleanOperation {
        self.operation
    }

    /// Return whether graph extraction retained unknown events.
    pub(crate) const fn graph_had_unknowns(&self) -> bool {
        self.graph_had_unknowns
    }

    /// Return the retained face-pair record count.
    pub(crate) const fn retained_face_pairs(&self) -> usize {
        self.retained_face_pairs
    }

    /// Return the retained event record count.
    pub(crate) const fn retained_events(&self) -> usize {
        self.retained_events
    }

    /// Return the retained relation-count blocker.
    pub(crate) const fn blocker(&self) -> &ExactBooleanBlocker {
        &self.blocker
    }

    /// Return the retained coplanar arrangement evidence summary.
    pub(crate) fn coplanar_arrangement_evidence(&self) -> Option<&CoplanarArrangementEvidence> {
        self.coplanar_arrangement_evidence.as_ref()
    }

    /// Return whether this operation is blocked on planar arrangement output.
    pub(crate) const fn is_required(&self) -> bool {
        matches!(self.status, ExactPlanarArrangementStatus::Required)
    }

    /// Return whether planar arrangement topology has already been
    /// materialized by a narrower certified path.
    pub(crate) const fn is_already_materialized(&self) -> bool {
        matches!(
            self.status,
            ExactPlanarArrangementStatus::AlreadyMaterialized
        )
    }

    /// Validate status, retained relation counts, and blocker consistency.
    pub(crate) fn validate(&self) -> Result<(), ExactReportValidationError> {
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)?;
        if matches!(self.status, ExactPlanarArrangementStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        // A graph-unknown arrangement report has not reached planar-cell
        // policy. It is still blocked on predicate/construction refinement, a
        let expected_kind = match self.status {
            ExactPlanarArrangementStatus::GraphUnknowns => ExactBooleanBlockerKind::Refinement,
            ExactPlanarArrangementStatus::BoundaryPolicyRequired => {
                ExactBooleanBlockerKind::BoundaryPolicy
            }
            ExactPlanarArrangementStatus::Required => ExactBooleanBlockerKind::PlanarArrangement,
            ExactPlanarArrangementStatus::NotNamedOperation
            | ExactPlanarArrangementStatus::AlreadyMaterialized
            | ExactPlanarArrangementStatus::NoPositiveOverlap => self.blocker.inferred_kind(),
        };
        blocker_kind(Some(&self.blocker), expected_kind)?;
        self.blocker.validate_for_kind(expected_kind)?;
        validate_refinement_partition(
            matches!(self.status, ExactPlanarArrangementStatus::GraphUnknowns),
            &self.blocker,
        )?;
        validate_blocker_count_bounds(
            &self.blocker,
            self.retained_face_pairs,
            self.retained_events,
        )?;
        // Planar-cell extraction is a distinct topological obligation. These
        // selected-region calls, unresolved graphs, already materialized
        // shortcuts, and missing planar arrangements must not masquerade as
        // one another.
        match self.status {
            ExactPlanarArrangementStatus::NotNamedOperation => {
                if !matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                    || blocker_has_any_evidence(&self.blocker)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactPlanarArrangementStatus::GraphUnknowns => {}
            ExactPlanarArrangementStatus::AlreadyMaterialized
            | ExactPlanarArrangementStatus::NoPositiveOverlap
            | ExactPlanarArrangementStatus::BoundaryPolicyRequired => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if matches!(self.status, ExactPlanarArrangementStatus::NoPositiveOverlap)
                    && self.blocker.candidate_pairs == 0
                    && self.blocker.coplanar_overlapping_pairs != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactPlanarArrangementStatus::Required => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
        }
        if self.is_required() && self.blocker.coplanar_overlapping_pairs == 0 {
            return Err(ExactReportValidationError::MissingRelationCount);
        }
        match self.status {
            ExactPlanarArrangementStatus::Required => {
                let evidence = self
                    .coplanar_arrangement_evidence
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingCoplanarArrangementEvidence)?;
                evidence
                    .validate()
                    .map_err(|_| ExactReportValidationError::InvalidCoplanarArrangementEvidence)?;
                validate_coplanar_arrangement_evidence_matches_blocker(evidence, &self.blocker)?;
                if !evidence.needs_planar_cells()
                    || self.blocker.coplanar_touching_pairs != 0
                    || evidence.graph_count != self.blocker.coplanar_overlapping_pairs
                {
                    return Err(ExactReportValidationError::CoplanarArrangementEvidenceMismatch);
                }
            }
            ExactPlanarArrangementStatus::AlreadyMaterialized
            | ExactPlanarArrangementStatus::NoPositiveOverlap
            | ExactPlanarArrangementStatus::BoundaryPolicyRequired => {
                let evidence = self
                    .coplanar_arrangement_evidence
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingCoplanarArrangementEvidence)?;
                evidence
                    .validate()
                    .map_err(|_| ExactReportValidationError::InvalidCoplanarArrangementEvidence)?;
                validate_coplanar_arrangement_evidence_matches_blocker(evidence, &self.blocker)?;
                if evidence.status == CoplanarArrangementEvidenceStatus::NoCoplanarOverlap
                    && (self.blocker.coplanar_overlapping_pairs != 0
                        || self.blocker.coplanar_touching_pairs != 0)
                {
                    return Err(ExactReportValidationError::CoplanarArrangementEvidenceMismatch);
                }
            }
            ExactPlanarArrangementStatus::NotNamedOperation
            | ExactPlanarArrangementStatus::GraphUnknowns => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
            }
        }
        if self.is_required() {
            self.blocker
                .validate_for_kind(ExactBooleanBlockerKind::PlanarArrangement)?;
        } else if matches!(
            self.status,
            ExactPlanarArrangementStatus::BoundaryPolicyRequired
        ) {
            self.blocker
                .validate_for_kind(ExactBooleanBlockerKind::BoundaryPolicy)?;
        }
        Ok(())
    }

    /// Validate this planar-arrangement report against source meshes and request.
    ///
    /// The retained arrangement-evidence summary is a compact view of exact
    /// coplanar graph state. This source replay recomputes that view for the
    /// same operation and rejects stale count/blocker summaries before a
    #[cfg(test)]
    pub(crate) fn validate_against_sources_for_request(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        request: ExactBooleanRequest,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        if let Ok(evaluation) = exact_boolean_report_evaluation_for_replay(left, right, request)
            && self == evaluation.certifications().planar_arrangement()
        {
            return Ok(());
        }
        Err(ExactReportValidationError::SourceReplayMismatch)
    }
}

/// Certification status for the remaining exact winding evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactWindingEvidenceStatus {
    /// Selected-region assembly already carries its own explicit region policy.
    NotNamedOperation,
    /// Exact graph extraction retained unresolved events.
    GraphUnknowns,
    /// Retained graph pairs are boundary-only contacts and need boundary
    /// output policy rather than winding.
    BoundaryPolicyRequired,
    /// Retained graph pairs are positive-area coplanar overlaps and need a
    /// planar arrangement output model rather than volumetric winding.
    PlanarArrangementRequired,
    /// The positive-area coplanar overlap was already handled by a certified
    /// planar-arrangement shortcut, so no volumetric winding evidence is needed.
    PlanarArrangementAlreadyMaterialized,
    /// Coplanar source-face cells are part of a closed-volumetric overlap and
    /// must be materialized before winding can consume the split cells.
    CoplanarVolumetricCellsRequired,
    /// Coplanar source-face cells were required, but the certified
    /// arrangement/cell-complex path has already materialized them, so no
    /// unresolved winding blocker remains in this evidence.
    CoplanarVolumetricCellsAlreadyMaterialized,
    /// Exact volumetric winding classifications are decided, but the retained
    /// split cells could not yet be assembled into certified output topology.
    VolumetricAssemblyRequired,
    /// A certified arrangement/cell-complex shortcut has already materialized
    /// this named Boolean, so no unresolved winding blocker remains in this
    /// evidence.
    ArrangementCellComplexAlreadyMaterialized,
    /// The named Boolean was already answered by regularized-solid semantics
    /// for one closed solid and one lower-dimensional open surface, so no
    /// winding evidence is needed.
    MixedDimensionalRegularizedSolidAlreadyMaterialized,
    /// The named Boolean was already answered by closed-output regularization
    /// of two lower-dimensional operands, so no winding evidence is needed.
    LowerDimensionalRegularizedSolidAlreadyMaterialized,
    /// The named Boolean was already answered by closed-convex exact
    /// materialization, so no winding evidence is needed.
    ConvexBooleanAlreadyMaterialized,
    /// The named Boolean was already answered by exact open-surface
    /// split-region arrangement, so no volumetric winding evidence is needed.
    OpenSurfaceArrangementAlreadyMaterialized,
    /// The named Boolean was already answered by exact surface identity or
    /// same-surface equality, so no winding evidence is needed.
    SurfaceEqualityAlreadyMaterialized,
    /// The named Boolean was already answered by certified closed-boundary
    /// touching regularized semantics, so no winding evidence is needed.
    ClosedBoundaryTouchingAlreadyMaterialized,
    /// A caller supplied a certified boundary-output policy, so boundary-only
    /// contact has already been projected into output without volumetric
    /// winding.
    BoundaryPolicyShortcutAlreadyMaterialized,
    /// The named Boolean was already answered by exact empty-operand
    /// semantics, so no winding evidence is needed.
    EmptyOperandAlreadyMaterialized,
    /// The named Boolean was already answered by certified disjoint mesh
    /// bounds, so no winding evidence is needed.
    BoundsDisjointAlreadyMaterialized,
    /// The named Boolean was already answered by certified open-surface graph
    /// disjointness, so no winding evidence is needed.
    OpenSurfaceDisjointAlreadyMaterialized,
    /// The named Boolean was already answered by an empty exact intersection
    /// graph and replayable closed-mesh winding reports proving separation.
    ClosedWindingSeparatedAlreadyMaterialized,
    /// The named Boolean was already answered by an empty exact intersection
    /// graph and replayable closed-mesh winding reports proving containment.
    ClosedWindingContainmentAlreadyMaterialized,
    /// The graph contains no retained face pairs requiring winding.
    NoNontrivialOverlap,
    /// Split regions and opposite-plane classifications were checked and can
    /// be consumed by exact winding/inside-outside selection.
    Ready,
}

impl ExactWindingEvidenceStatus {
    /// Returns whether this evidence state records a certified materialized
    /// path rather than an unresolved winding blocker.
    #[cfg(test)]
    pub(crate) const fn is_already_materialized(&self) -> bool {
        matches!(
            self,
            Self::PlanarArrangementAlreadyMaterialized
                | Self::CoplanarVolumetricCellsAlreadyMaterialized
                | Self::ArrangementCellComplexAlreadyMaterialized
                | Self::MixedDimensionalRegularizedSolidAlreadyMaterialized
                | Self::LowerDimensionalRegularizedSolidAlreadyMaterialized
                | Self::ConvexBooleanAlreadyMaterialized
                | Self::OpenSurfaceArrangementAlreadyMaterialized
                | Self::SurfaceEqualityAlreadyMaterialized
                | Self::ClosedBoundaryTouchingAlreadyMaterialized
                | Self::BoundaryPolicyShortcutAlreadyMaterialized
                | Self::EmptyOperandAlreadyMaterialized
                | Self::BoundsDisjointAlreadyMaterialized
                | Self::OpenSurfaceDisjointAlreadyMaterialized
                | Self::ClosedWindingSeparatedAlreadyMaterialized
                | Self::ClosedWindingContainmentAlreadyMaterialized
        )
    }

    /// Returns whether the materialized path specifically produced the exact
    /// arrangement/cell-complex topology needed before winding policy.
    pub(crate) const fn materializes_arrangement_cell_complex(&self) -> bool {
        matches!(
            self,
            Self::PlanarArrangementAlreadyMaterialized
                | Self::CoplanarVolumetricCellsAlreadyMaterialized
                | Self::ArrangementCellComplexAlreadyMaterialized
        )
    }

    /// Returns whether this state belongs to the certified-winding evidence
    /// support path rather than to a shortcut, caller policy, or arrangement
    /// prerequisite.
    pub(crate) const fn routes_to_certified_winding(&self) -> bool {
        matches!(
            self,
            Self::Ready | Self::NoNontrivialOverlap | Self::VolumetricAssemblyRequired
        )
    }
}

/// Auditable report for the nontrivial overlap winding evidence.
///
/// This report is the certified boundary immediately before full named
/// union/intersection/difference winding semantics. It retains exact graph
/// counts and checked split-region plane classifications, but deliberately
/// topological policy remains explicit state instead of a hidden tolerance
/// decision.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactWindingEvidenceReport {
    /// Requested named operation.
    pub(crate) operation: ExactBooleanOperation,
    /// Coarse evidence status.
    pub(crate) status: ExactWindingEvidenceStatus,
    /// Whether graph extraction retained unknown events.
    pub(crate) graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub(crate) retained_face_pairs: usize,
    /// Total retained event records.
    pub(crate) retained_events: usize,
    /// Number of checked split regions prepared for winding.
    pub(crate) region_count: usize,
    /// Certified region-vs-opposite-plane classifications.
    pub(crate) region_classifications: Vec<FaceRegionPlaneClassification>,
    /// Relation counts for the blocker represented by this report.
    pub(crate) blocker: ExactBooleanBlocker,
    /// Checked coplanar-overlap evidence retained when winding is blocked by
    /// planar-cell extraction rather than by volumetric inside/outside policy.
    pub(crate) coplanar_arrangement_evidence: Option<CoplanarArrangementEvidence>,
    /// Source-aware coplanar volumetric-cell evidence retained when evidence
    /// is blocked by, or has just consumed, coplanar source-face cells.
    ///
    /// The winding evidence must not reduce this state to raw coplanar pair
    /// counts: exact side evidence is what distinguishes boundary-only contact
    /// from a real volumetric-cell topology obligation.
    pub(crate) coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
}

impl ExactWindingEvidenceReport {
    /// Return the requested named operation.
    pub(crate) const fn operation(&self) -> ExactBooleanOperation {
        self.operation
    }

    /// Return the coarse winding-evidence status.
    pub(crate) const fn status(&self) -> ExactWindingEvidenceStatus {
        self.status
    }

    /// Return whether graph extraction retained unknown events.
    pub(crate) const fn graph_had_unknowns(&self) -> bool {
        self.graph_had_unknowns
    }

    /// Return the retained face-pair record count.
    pub(crate) const fn retained_face_pairs(&self) -> usize {
        self.retained_face_pairs
    }

    /// Return the retained event record count.
    pub(crate) const fn retained_events(&self) -> usize {
        self.retained_events
    }

    /// Return the checked split-region count.
    pub(crate) const fn region_count(&self) -> usize {
        self.region_count
    }

    /// Return the retained split-region classifications.
    pub(crate) fn region_classifications(&self) -> &[FaceRegionPlaneClassification] {
        &self.region_classifications
    }

    /// Return the retained relation-count blocker.
    pub(crate) const fn blocker(&self) -> &ExactBooleanBlocker {
        &self.blocker
    }

    /// Return the retained coplanar arrangement evidence summary.
    pub(crate) fn coplanar_arrangement_evidence(&self) -> Option<&CoplanarArrangementEvidence> {
        self.coplanar_arrangement_evidence.as_ref()
    }

    /// Return the retained coplanar volumetric-cell evidence.
    pub(crate) fn coplanar_volumetric_evidence(
        &self,
    ) -> Option<&CoplanarVolumetricCellEvidenceReport> {
        self.coplanar_volumetric_evidence.as_ref()
    }

    /// Return whether the report reached the winding-evidence handoff.
    pub(crate) const fn is_ready(&self) -> bool {
        matches!(self.status, ExactWindingEvidenceStatus::Ready)
    }

    /// Validate this winding-evidence report against the source meshes.
    ///
    /// Winding evidence retains exact split-region and opposite-plane facts.
    /// This replay recomputes the report for the same operation, making stale
    /// region facts and blocker summaries fail before downstream topology
    #[cfg(test)]
    pub(crate) fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let request = ExactBooleanRequest::with_boundary_policy(
            self.operation,
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        );
        validate_winding_evidence_against_sources_for_request(self, left, right, request)
    }

    /// Validate status, blocker, and checked-region artifact consistency.
    pub(crate) fn validate(&self) -> Result<(), ExactReportValidationError> {
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)?;
        if matches!(self.status, ExactWindingEvidenceStatus::GraphUnknowns)
            != self.graph_had_unknowns
            && !matches!(self.status, ExactWindingEvidenceStatus::NotNamedOperation)
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        validate_refinement_partition(
            matches!(self.status, ExactWindingEvidenceStatus::GraphUnknowns)
                || (matches!(self.status, ExactWindingEvidenceStatus::NotNamedOperation)
                    && self.graph_had_unknowns),
            &self.blocker,
        )?;
        if self.coplanar_volumetric_evidence.is_some()
            && !matches!(
                self.status,
                ExactWindingEvidenceStatus::Ready
                    | ExactWindingEvidenceStatus::VolumetricAssemblyRequired
                    | ExactWindingEvidenceStatus::CoplanarVolumetricCellsRequired
                    | ExactWindingEvidenceStatus::SurfaceEqualityAlreadyMaterialized
                    | ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized
            )
            && !self.status.materializes_arrangement_cell_complex()
        {
            return Err(ExactReportValidationError::UnexpectedCoplanarVolumetricEvidence);
        }
        match self.status {
            ExactWindingEvidenceStatus::GraphUnknowns => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::Refinement)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::Refinement)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::BoundaryPolicyRequired => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::BoundaryPolicy)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::BoundaryPolicy)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::PlanarArrangementRequired => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::PlanarArrangement,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::PlanarArrangement)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let evidence = self
                    .coplanar_arrangement_evidence
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingCoplanarArrangementEvidence)?;
                evidence
                    .validate()
                    .map_err(|_| ExactReportValidationError::InvalidCoplanarArrangementEvidence)?;
                validate_coplanar_arrangement_evidence_matches_blocker(evidence, &self.blocker)?;
                if !evidence.needs_planar_cells() || self.blocker.coplanar_touching_pairs != 0 {
                    return Err(ExactReportValidationError::CoplanarArrangementEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::PlanarArrangementAlreadyMaterialized => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::PlanarArrangement,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::PlanarArrangement)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let evidence = self
                    .coplanar_arrangement_evidence
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingCoplanarArrangementEvidence)?;
                evidence
                    .validate()
                    .map_err(|_| ExactReportValidationError::InvalidCoplanarArrangementEvidence)?;
                validate_coplanar_arrangement_evidence_matches_blocker(evidence, &self.blocker)?;
                if !evidence.needs_planar_cells() {
                    return Err(ExactReportValidationError::CoplanarArrangementEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::CoplanarVolumetricCellsRequired => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::CoplanarVolumetricCells,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::CoplanarVolumetricCells)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let evidence = self
                    .coplanar_volumetric_evidence
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingCoplanarVolumetricEvidence)?;
                validate_coplanar_volumetric_evidence_matches_blocker(
                    evidence,
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                    return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::CoplanarVolumetricCellsAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::CoplanarVolumetricCells,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::CoplanarVolumetricCells)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let evidence = self
                    .coplanar_volumetric_evidence
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingCoplanarVolumetricEvidence)?;
                validate_coplanar_volumetric_evidence_matches_blocker(
                    evidence,
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                    return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::VolumetricAssemblyRequired => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                let expected = match self.blocker.kind {
                    ExactBooleanBlockerKind::CoplanarVolumetricCells => {
                        ExactBooleanBlockerKind::CoplanarVolumetricCells
                    }
                    _ => ExactBooleanBlockerKind::Winding,
                };
                blocker_kind(Some(&self.blocker), expected)?;
                self.blocker.validate_for_kind(expected)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                match (
                    self.blocker.kind,
                    self.coplanar_volumetric_evidence.as_ref(),
                ) {
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_matches_blocker(
                            evidence,
                            &self.blocker,
                            self.retained_face_pairs,
                            self.retained_events,
                        )?;
                        if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                            return Err(
                                ExactReportValidationError::CoplanarVolumetricEvidenceMismatch,
                            );
                        }
                    }
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, None) => {
                        return Err(ExactReportValidationError::MissingCoplanarVolumetricEvidence);
                    }
                    (_, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_shape(
                            evidence,
                            self.retained_face_pairs,
                            self.retained_events,
                        )?;
                    }
                    (_, None) => {}
                }
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                let expected = match self.blocker.kind {
                    ExactBooleanBlockerKind::BoundaryPolicy => {
                        ExactBooleanBlockerKind::BoundaryPolicy
                    }
                    ExactBooleanBlockerKind::CoplanarVolumetricCells => {
                        ExactBooleanBlockerKind::CoplanarVolumetricCells
                    }
                    _ => ExactBooleanBlockerKind::Winding,
                };
                blocker_kind(Some(&self.blocker), expected)?;
                self.blocker.validate_for_kind(expected)?;
                match (expected, self.coplanar_volumetric_evidence.as_ref()) {
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_matches_blocker(
                            evidence,
                            &self.blocker,
                            self.retained_face_pairs,
                            self.retained_events,
                        )?;
                        if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                            return Err(
                                ExactReportValidationError::CoplanarVolumetricEvidenceMismatch,
                            );
                        }
                    }
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, None) => {
                        return Err(ExactReportValidationError::MissingCoplanarVolumetricEvidence);
                    }
                    (ExactBooleanBlockerKind::BoundaryPolicy, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_matches_blocker(
                            evidence,
                            &self.blocker,
                            self.retained_face_pairs,
                            self.retained_events,
                        )?;
                        validate_arrangement_materialized_coplanar_evidence(
                            evidence,
                            self.retained_face_pairs,
                            self.retained_events,
                        )?;
                    }
                    (ExactBooleanBlockerKind::BoundaryPolicy, None)
                    | (ExactBooleanBlockerKind::Winding, None) => {
                        validate_blocker_count_bounds(
                            &self.blocker,
                            self.retained_face_pairs,
                            self.retained_events,
                        )?;
                    }
                    (
                        ExactBooleanBlockerKind::Refinement
                        | ExactBooleanBlockerKind::PlanarArrangement,
                        None,
                    ) => {
                        return Err(ExactReportValidationError::StatusEvidenceMismatch);
                    }
                    (_, Some(_)) => {
                        return Err(
                            ExactReportValidationError::UnexpectedCoplanarVolumetricEvidence,
                        );
                    }
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized
            | ExactWindingEvidenceStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::Winding)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::Winding)?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::Winding)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::Winding)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::OpenSurfaceArrangementAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::Winding)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::Winding)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::SurfaceEqualityAlreadyMaterialized => {
                let has_coplanar_evidence = self.coplanar_volumetric_evidence.is_some();
                if self.coplanar_arrangement_evidence.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || (!has_coplanar_evidence
                        && (self.retained_face_pairs != 0 || self.retained_events != 0))
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::Winding)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::Winding)?;
                if let Some(evidence) = self.coplanar_volumetric_evidence.as_ref() {
                    validate_coplanar_volumetric_evidence_counts(
                        evidence,
                        self.retained_face_pairs,
                        self.retained_events,
                    )?;
                    validate_blocker_count_bounds(
                        &self.blocker,
                        self.retained_face_pairs,
                        self.retained_events,
                    )?;
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::BoundaryPolicy)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::BoundaryPolicy)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                match self.coplanar_volumetric_evidence.as_ref() {
                    Some(evidence) => {
                        validate_coplanar_volumetric_evidence_matches_blocker(
                            evidence,
                            &self.blocker,
                            self.retained_face_pairs,
                            self.retained_events,
                        )?;
                        validate_coplanar_boundary_only_evidence_shape(
                            evidence,
                            self.retained_face_pairs,
                            self.retained_events,
                        )?;
                    }
                    None if self.blocker.coplanar_overlapping_pairs != 0 => {
                        return Err(ExactReportValidationError::MissingCoplanarVolumetricEvidence);
                    }
                    None => {}
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::BoundaryPolicyShortcutAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::BoundaryPolicy)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::BoundaryPolicy)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::EmptyOperandAlreadyMaterialized
            | ExactWindingEvidenceStatus::BoundsDisjointAlreadyMaterialized
            | ExactWindingEvidenceStatus::OpenSurfaceDisjointAlreadyMaterialized
            | ExactWindingEvidenceStatus::ClosedWindingSeparatedAlreadyMaterialized
            | ExactWindingEvidenceStatus::ClosedWindingContainmentAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::Winding)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::Winding)?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::Ready => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                let expected = match self.blocker.kind {
                    ExactBooleanBlockerKind::CoplanarVolumetricCells => {
                        ExactBooleanBlockerKind::CoplanarVolumetricCells
                    }
                    _ => ExactBooleanBlockerKind::Winding,
                };
                blocker_kind(Some(&self.blocker), expected)?;
                self.blocker.validate_for_kind(expected)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                match (
                    self.blocker.kind,
                    self.coplanar_volumetric_evidence.as_ref(),
                ) {
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_matches_blocker(
                            evidence,
                            &self.blocker,
                            self.retained_face_pairs,
                            self.retained_events,
                        )?;
                        if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                            return Err(
                                ExactReportValidationError::CoplanarVolumetricEvidenceMismatch,
                            );
                        }
                    }
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, None) => {
                        return Err(ExactReportValidationError::MissingCoplanarVolumetricEvidence);
                    }
                    (_, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_shape(
                            evidence,
                            self.retained_face_pairs,
                            self.retained_events,
                        )?;
                    }
                    (_, None) => {}
                }
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::NotNamedOperation
            | ExactWindingEvidenceStatus::NoNontrivialOverlap => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(ExactReportValidationError::UnexpectedCoplanarArrangementEvidence);
                }
                match self.status {
                    ExactWindingEvidenceStatus::NotNamedOperation
                        if !matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) =>
                    {
                        return Err(ExactReportValidationError::StatusEvidenceMismatch);
                    }
                    ExactWindingEvidenceStatus::NoNontrivialOverlap
                        if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                            || self.retained_face_pairs != 0 =>
                    {
                        return Err(ExactReportValidationError::StatusEvidenceMismatch);
                    }
                    _ => {}
                }
                if matches!(self.status, ExactWindingEvidenceStatus::NotNamedOperation) {
                    let expected = self.blocker.inferred_kind();
                    blocker_kind(Some(&self.blocker), expected)?;
                    self.blocker.validate_for_kind(expected)?;
                } else {
                    blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::Winding)?;
                    self.blocker
                        .validate_for_kind(ExactBooleanBlockerKind::Winding)?;
                }
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::ExactBooleanRequest;
    use crate::graph::FaceSplitBoundaryNode;
    use crate::region::{ExactOutputVertex, FaceRegionPlaneRelation};

    #[test]
    fn selected_region_preflight_accepts_empty_region_plan_with_boundary_face_pairs() {
        let mut preflight = ExactBooleanPreflight {
            operation: ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll),
            support: ExactBooleanSupport::SelectedRegionPolicy,
            graph_had_unknowns: false,
            retained_face_pairs: 1,
            retained_events: 1,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: None,
            coplanar_arrangement_evidence: None,
            coplanar_volumetric_evidence: None,
        };

        preflight.validate().unwrap();

        preflight.region_count = 1;
        assert_eq!(
            preflight.validate(),
            Err(ExactReportValidationError::MissingRegionFacts)
        );
    }

    fn report_test_triangle(points: &[[i64; 3]; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles_with_policy(
            &[
                points[0][0],
                points[0][1],
                points[0][2],
                points[1][0],
                points[1][1],
                points[1][2],
                points[2][0],
                points[2][1],
                points[2][2],
            ],
            &[0, 1, 2],
            ExactMeshValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
    }

    #[test]
    fn selected_region_result_rejects_duplicate_assembly_triangle() {
        let left = report_test_triangle(&[[0, 0, 0], [4, 0, 0], [0, 4, 0]]);
        let right = report_test_triangle(&[[1, -1, -1], [1, 3, 1], [1, 3, -1]]);
        let mut result = materialize_boolean_exact_request(
            &left,
            &right,
            ExactBooleanRequest::new(
                ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll),
                ExactMeshValidationPolicy::ALLOW_BOUNDARY,
            ),
        )
        .unwrap();
        result.validate().unwrap();
        assert!(!result.assembly.triangles.is_empty());

        let duplicate = result.assembly.triangles[0].clone();
        result.assembly.triangles.push(duplicate);

        assert_eq!(
            result.validate(),
            Err(ExactReportValidationError::DuplicateAssemblyTriangle)
        );
    }

    #[test]
    fn selected_region_result_rejects_missing_assembly_cell_with_retained_source_label() {
        let p0 = point(0, 0, 0);
        let p1 = point(2, 0, 0);
        let p2 = point(2, 2, 0);
        let p3 = point(0, 2, 0);
        let boundary = vec![
            FaceSplitBoundaryNode::FaceInterior { point: p0.clone() },
            FaceSplitBoundaryNode::FaceInterior { point: p1.clone() },
            FaceSplitBoundaryNode::FaceInterior { point: p2.clone() },
            FaceSplitBoundaryNode::FaceInterior { point: p3.clone() },
        ];
        let triangulation = FaceRegionTriangulation {
            side: MeshSide::Left,
            face: 0,
            projection: hyperlimit::CoplanarProjection::Xy,
            vertices: vec![
                hypertri::ExactPoint::new(p0.x.clone(), p0.y.clone()),
                hypertri::ExactPoint::new(p1.x.clone(), p1.y.clone()),
                hypertri::ExactPoint::new(p2.x.clone(), p2.y.clone()),
                hypertri::ExactPoint::new(p3.x.clone(), p3.y.clone()),
            ],
            boundary: boundary.clone(),
            triangles: vec![0, 1, 2, 0, 2, 3],
        };
        let proof = PredicateUse::from_certificate(
            hyperlimit::orient3d_report(&p0, &p1, &p2, &point(0, 0, 1)).certificate,
        );
        let classification = FaceRegionPlaneClassification {
            region_side: MeshSide::Left,
            region_face: 0,
            plane_side: MeshSide::Right,
            plane_face: 0,
            relation: FaceRegionPlaneRelation::StrictlyAbove,
            node_sides: vec![Some(hyperlimit::PlaneSide::Above); 4],
            predicates: vec![proof; 4],
        };
        let assembly = ExactBooleanAssemblyPlan {
            vertices: vec![
                ExactOutputVertex {
                    point: p0,
                    source: boundary[0].clone(),
                },
                ExactOutputVertex {
                    point: p1,
                    source: boundary[1].clone(),
                },
                ExactOutputVertex {
                    point: p2,
                    source: boundary[2].clone(),
                },
            ],
            triangles: vec![ExactOutputTriangle {
                vertices: [0, 1, 2],
                source_side: MeshSide::Left,
                source_face: 0,
                orientation: ExactOutputTriangleOrientation::PreserveSource,
            }],
        };
        let mesh = assembly
            .to_exact_mesh(ExactMeshValidationPolicy::ALLOW_BOUNDARY)
            .unwrap();
        let result = ExactBooleanResult {
            kind: ExactBooleanResultKind::SelectedRegions {
                selection: ExactRegionSelection::KeepAll,
            },
            graph_had_unknowns: false,
            region_classifications: vec![classification],
            triangulations: vec![triangulation],
            assembly,
            volumetric_classifications: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
            mesh,
        };

        assert_eq!(
            result.validate(),
            Err(ExactReportValidationError::SelectedRegionAssemblyMissingSelectedRegion)
        );
    }

    #[test]
    fn volumetric_cell_coverage_rejects_partial_retained_subcell() {
        let p0 = point(0, 0, 0);
        let p1 = point(2, 0, 0);
        let p2 = point(0, 2, 0);
        let p3 = point(1, 0, 0);
        let p4 = point(0, 1, 0);
        let boundary = vec![
            FaceSplitBoundaryNode::FaceInterior { point: p0.clone() },
            FaceSplitBoundaryNode::FaceInterior { point: p1.clone() },
            FaceSplitBoundaryNode::FaceInterior { point: p2.clone() },
        ];
        let triangulation = FaceRegionTriangulation {
            side: MeshSide::Left,
            face: 0,
            projection: hyperlimit::CoplanarProjection::Xy,
            vertices: vec![
                hypertri::ExactPoint::new(p0.x.clone(), p0.y.clone()),
                hypertri::ExactPoint::new(p1.x.clone(), p1.y.clone()),
                hypertri::ExactPoint::new(p2.x.clone(), p2.y.clone()),
            ],
            boundary: boundary.clone(),
            triangles: vec![0, 1, 2],
        };
        let partial = ExactBooleanAssemblyPlan {
            vertices: vec![
                ExactOutputVertex {
                    point: p0.clone(),
                    source: boundary[0].clone(),
                },
                ExactOutputVertex {
                    point: p3.clone(),
                    source: FaceSplitBoundaryNode::FaceInterior { point: p3 },
                },
                ExactOutputVertex {
                    point: p4.clone(),
                    source: FaceSplitBoundaryNode::FaceInterior { point: p4 },
                },
            ],
            triangles: vec![ExactOutputTriangle {
                vertices: [0, 1, 2],
                source_side: MeshSide::Left,
                source_face: 0,
                orientation: ExactOutputTriangleOrientation::PreserveSource,
            }],
        };
        let whole = ExactBooleanAssemblyPlan {
            vertices: vec![
                ExactOutputVertex {
                    point: p0,
                    source: boundary[0].clone(),
                },
                ExactOutputVertex {
                    point: p1,
                    source: boundary[1].clone(),
                },
                ExactOutputVertex {
                    point: p2,
                    source: boundary[2].clone(),
                },
            ],
            triangles: vec![ExactOutputTriangle {
                vertices: [0, 1, 2],
                source_side: MeshSide::Left,
                source_face: 0,
                orientation: ExactOutputTriangleOrientation::PreserveSource,
            }],
        };

        assert!(!output_triangles_cover_triangulated_cell(
            partial.triangles.iter().filter(|output| {
                output_triangle_lies_in_triangulated_cell(
                    output,
                    &partial,
                    &triangulation,
                    [0, 1, 2],
                )
            }),
            &partial,
            &triangulation,
            [0, 1, 2],
        ));
        assert!(output_triangles_cover_triangulated_cell(
            whole.triangles.iter().filter(|output| {
                output_triangle_lies_in_triangulated_cell(output, &whole, &triangulation, [0, 1, 2])
            }),
            &whole,
            &triangulation,
            [0, 1, 2],
        ));
    }

    fn point(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(
            hyperreal::Real::from(x),
            hyperreal::Real::from(y),
            hyperreal::Real::from(z),
        )
    }

    #[test]
    fn empty_shortcut_result_rejects_retained_orphan_vertices() {
        let mesh = ExactMesh::from_i64_triangles(&[0, 0, 0], &[]).unwrap();
        assert!(mesh.triangles().is_empty());
        assert!(!mesh.vertices().is_empty());
        let result = ExactBooleanResult {
            kind: ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Intersection,
                shortcut: ExactBooleanShortcutKind::BoundsDisjoint,
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
        };

        assert_eq!(
            result.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn arrangement_union_shortcut_shape_allows_empty_output() {
        let result = ExactBooleanResult {
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
            topology_assembly_report: None,
            region_ownership_report: None,
            mesh: ExactMesh::new(
                Vec::new(),
                Vec::new(),
                hyperlimit::SourceProvenance::exact("empty exact arrangement union shortcut"),
            )
            .unwrap(),
        };

        result.validate().unwrap();
    }

    fn valid_self_contact_closure_report() -> ExactVolumetricBoundaryClosureReport {
        ExactVolumetricBoundaryClosureReport {
            operation: ExactBooleanOperation::Union,
            status: ExactVolumetricBoundaryClosureStatus::BoundaryLoopExactSelfContact,
            output_triangles: 1,
            boundary_edges: 3,
            boundary_loops: 1,
            boundary_vertices_with_invalid_outgoing_degree: 0,
            boundary_vertices_with_invalid_incoming_degree: 0,
            overused_boundary_edges: 0,
            noncoplanar_boundary_loops: 0,
            repeated_exact_boundary_points: 1,
            self_contact_exact_points: 1,
            self_contact_topological_vertices: 2,
            self_contact_degenerate_cycles: 2,
            self_contact_nondegenerate_cycles: 0,
            coplanar_loop_groups: 0,
        }
    }

    fn valid_blocked_closure_report() -> ExactVolumetricBoundaryClosureReport {
        ExactVolumetricBoundaryClosureReport {
            operation: ExactBooleanOperation::Union,
            status: ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(
                ExactArrangementBlocker::UndecidableOrdering,
            ),
            output_triangles: 1,
            boundary_edges: 3,
            boundary_loops: 1,
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
        }
    }

    fn valid_topology_not_loop_closure_report() -> ExactVolumetricBoundaryClosureReport {
        ExactVolumetricBoundaryClosureReport {
            operation: ExactBooleanOperation::Union,
            status: ExactVolumetricBoundaryClosureStatus::BoundaryTopologyNotLoop,
            output_triangles: 1,
            boundary_edges: 2,
            boundary_loops: 0,
            boundary_vertices_with_invalid_outgoing_degree: 1,
            boundary_vertices_with_invalid_incoming_degree: 1,
            overused_boundary_edges: 0,
            noncoplanar_boundary_loops: 0,
            repeated_exact_boundary_points: 0,
            self_contact_exact_points: 0,
            self_contact_topological_vertices: 0,
            self_contact_degenerate_cycles: 0,
            self_contact_nondegenerate_cycles: 0,
            coplanar_loop_groups: 0,
        }
    }

    fn valid_noncoplanar_closure_report() -> ExactVolumetricBoundaryClosureReport {
        ExactVolumetricBoundaryClosureReport {
            operation: ExactBooleanOperation::Union,
            status: ExactVolumetricBoundaryClosureStatus::NonCoplanarBoundaryClosureRequired,
            output_triangles: 1,
            boundary_edges: 3,
            boundary_loops: 1,
            boundary_vertices_with_invalid_outgoing_degree: 0,
            boundary_vertices_with_invalid_incoming_degree: 0,
            overused_boundary_edges: 0,
            noncoplanar_boundary_loops: 1,
            repeated_exact_boundary_points: 0,
            self_contact_exact_points: 0,
            self_contact_topological_vertices: 0,
            self_contact_degenerate_cycles: 0,
            self_contact_nondegenerate_cycles: 0,
            coplanar_loop_groups: 0,
        }
    }

    #[test]
    fn volumetric_boundary_already_closed_report_accepts_empty_output() {
        let report = ExactVolumetricBoundaryClosureReport {
            operation: ExactBooleanOperation::Intersection,
            status: ExactVolumetricBoundaryClosureStatus::AlreadyClosed,
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
        };
        report.validate().unwrap();

        let mut stale = report;
        stale.boundary_edges = 1;
        assert_eq!(
            stale.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_self_contact_report_rejects_contradictory_status_evidence() {
        let mut report = valid_self_contact_closure_report();
        report.validate().unwrap();

        report.noncoplanar_boundary_loops = 1;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_self_contact_closure_report();
        report.coplanar_loop_groups = 1;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_self_contact_report_rejects_incoherent_contact_counts() {
        let mut report = valid_self_contact_closure_report();
        report.self_contact_topological_vertices = 1;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_self_contact_closure_report();
        report.repeated_exact_boundary_points = 0;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_self_contact_closure_report();
        report.self_contact_degenerate_cycles = 1;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_report_rejects_impossible_count_bounds() {
        let mut report = valid_noncoplanar_closure_report();
        report.status = ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable;
        report.noncoplanar_boundary_loops = 0;
        report.coplanar_loop_groups = 1;
        report.validate().unwrap();
        assert!(report.is_coplanar_closure_available());

        let mut report = valid_noncoplanar_closure_report();
        report.boundary_loops = 2;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_noncoplanar_closure_report();
        report.noncoplanar_boundary_loops = 2;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_noncoplanar_closure_report();
        report.boundary_edges = 4;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_noncoplanar_closure_report();
        report.output_triangles = usize::MAX;
        report.boundary_edges = usize::MAX;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_topology_not_loop_closure_report();
        report.boundary_vertices_with_invalid_outgoing_degree = 3;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_self_contact_closure_report();
        report.repeated_exact_boundary_points = 2;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_self_contact_closure_report();
        report.repeated_exact_boundary_points = 3;
        report.self_contact_topological_vertices = 4;
        report.self_contact_degenerate_cycles = 4;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_self_contact_closure_report();
        report.output_triangles = usize::MAX;
        report.boundary_edges = usize::MAX;
        report.repeated_exact_boundary_points = usize::MAX;
        report.self_contact_topological_vertices = usize::MAX;
        report.self_contact_degenerate_cycles = usize::MAX;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_noncoplanar_closure_report();
        report.status = ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable;
        report.noncoplanar_boundary_loops = 0;
        report.coplanar_loop_groups = 2;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_blocked_report_rejects_stale_closure_evidence() {
        let mut report = valid_blocked_closure_report();
        report.validate().unwrap();

        report.coplanar_loop_groups = 1;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_blocked_closure_report();
        report.status = ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(
            ExactArrangementBlocker::NonManifoldCellComplex,
        );
        report.coplanar_loop_groups = 1;
        report.validate().unwrap();
        report.repeated_exact_boundary_points = 1;
        report.self_contact_exact_points = 1;
        report.self_contact_topological_vertices = 2;
        report.self_contact_degenerate_cycles = 2;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_blocked_closure_report();
        report.self_contact_exact_points = 1;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_blocked_closure_report();
        report.repeated_exact_boundary_points = 1;
        report.self_contact_exact_points = 1;
        report.self_contact_topological_vertices = 2;
        report.self_contact_degenerate_cycles = 1;
        report.self_contact_nondegenerate_cycles = 1;
        report.validate().unwrap();
    }

    #[test]
    fn volumetric_boundary_blocked_report_rejects_nonclosure_blocker() {
        let mut report = valid_blocked_closure_report();
        report.validate().unwrap();

        report.status = ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(
            ExactArrangementBlocker::UnresolvedRegionClassification,
        );
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_topology_report_requires_topology_failure_evidence() {
        let mut report = valid_topology_not_loop_closure_report();
        report.validate().unwrap();

        report.boundary_vertices_with_invalid_outgoing_degree = 0;
        report.boundary_vertices_with_invalid_incoming_degree = 0;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_topology_not_loop_closure_report();
        report.boundary_loops = 1;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_topology_not_loop_closure_report();
        report.noncoplanar_boundary_loops = 1;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_noncoplanar_report_rejects_stale_coplanar_grouping() {
        let mut report = valid_noncoplanar_closure_report();
        report.validate().unwrap();

        report.coplanar_loop_groups = 1;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_loop_statuses_reject_stale_topology_failure_evidence() {
        let mut report = valid_self_contact_closure_report();
        report.boundary_vertices_with_invalid_outgoing_degree = 1;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_blocked_closure_report();
        report.overused_boundary_edges = 1;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn planar_arrangement_required_accepts_named_intersection_but_rejects_selected_regions() {
        let mut report = ExactPlanarArrangementReport {
            operation: ExactBooleanOperation::Difference,
            status: ExactPlanarArrangementStatus::Required,
            graph_had_unknowns: false,
            retained_face_pairs: 1,
            retained_events: 1,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::PlanarArrangement,
                candidate_pairs: 0,
                coplanar_overlapping_pairs: 1,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            coplanar_arrangement_evidence: Some(CoplanarArrangementEvidence {
                status: CoplanarArrangementEvidenceStatus::NeedsPlanarCells,
                graph_count: 1,
                overlapping_graphs: 1,
                touching_graphs: 0,
                edge_overlap_count: 1,
                vertex_overlap_count: 0,
                point_split_count: 0,
                interval_overlap_count: 0,
                interval_endpoint_count: 0,
            }),
        };
        report.validate().unwrap();

        report.operation = ExactBooleanOperation::Intersection;
        report.validate().unwrap();

        report.operation = ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll);
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let mut preflight = ExactBooleanPreflight {
            operation: ExactBooleanOperation::Difference,
            support: ExactBooleanSupport::RequiresPlanarArrangement,
            graph_had_unknowns: false,
            retained_face_pairs: 1,
            retained_events: 1,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(report.blocker),
            coplanar_arrangement_evidence: report.coplanar_arrangement_evidence.clone(),
            coplanar_volumetric_evidence: None,
        };
        preflight.validate().unwrap();

        preflight.operation = ExactBooleanOperation::Intersection;
        preflight.validate().unwrap();

        preflight.operation = ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll);
        assert_eq!(
            preflight.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn planar_arrangement_no_positive_overlap_rejects_relabelled_pure_coplanar_overlap() {
        let mut report = ExactPlanarArrangementReport {
            operation: ExactBooleanOperation::Union,
            status: ExactPlanarArrangementStatus::NoPositiveOverlap,
            graph_had_unknowns: false,
            retained_face_pairs: 1,
            retained_events: 1,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::PlanarArrangement,
                candidate_pairs: 0,
                coplanar_overlapping_pairs: 1,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            coplanar_arrangement_evidence: Some(CoplanarArrangementEvidence {
                status: CoplanarArrangementEvidenceStatus::NeedsPlanarCells,
                graph_count: 1,
                overlapping_graphs: 1,
                touching_graphs: 0,
                edge_overlap_count: 1,
                vertex_overlap_count: 0,
                point_split_count: 0,
                interval_overlap_count: 0,
                interval_endpoint_count: 0,
            }),
        };
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        report.retained_face_pairs = 2;
        report.retained_events = 2;
        report.blocker.kind = ExactBooleanBlockerKind::CoplanarVolumetricCells;
        report.blocker.candidate_pairs = 1;
        report.validate().unwrap();
    }

    #[test]
    fn retained_graph_reports_reject_impossible_event_totals() {
        let refinement = ExactRefinementReport {
            operation: ExactBooleanOperation::Union,
            status: ExactRefinementStatus::Required,
            graph_had_unknowns: true,
            retained_face_pairs: 2,
            retained_events: 1,
            blocker: Some(ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::Refinement,
                candidate_pairs: 0,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 2,
                construction_failed_events: 0,
            }),
        };
        assert_eq!(
            refinement.validate(),
            Err(ExactReportValidationError::InvalidBlockerCounts)
        );

        let overflowing_blocker = ExactRefinementReport {
            operation: ExactBooleanOperation::Union,
            status: ExactRefinementStatus::Required,
            graph_had_unknowns: true,
            retained_face_pairs: usize::MAX,
            retained_events: usize::MAX,
            blocker: Some(ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::Refinement,
                candidate_pairs: usize::MAX,
                coplanar_overlapping_pairs: 1,
                coplanar_touching_pairs: 0,
                unknown_pairs: 1,
                construction_failed_events: 0,
            }),
        };
        assert_eq!(
            overflowing_blocker.validate(),
            Err(ExactReportValidationError::InvalidBlockerCounts)
        );

        let open_disjoint = ExactOpenSurfaceDisjointReport {
            status: ExactOpenSurfaceDisjointStatus::GraphHasFacePairs,
            left_open_surface: true,
            right_open_surface: true,
            graph_had_unknowns: false,
            retained_face_pairs: 2,
            retained_events: 1,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::Winding,
                candidate_pairs: 2,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
        };
        assert_eq!(
            open_disjoint.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let adjacent = ExactAdjacentUnionCompletionReport {
            operation: ExactBooleanOperation::Union,
            status: ExactAdjacentUnionCompletionStatus::NoAdjacencyCertificate,
            left_closed: true,
            right_closed: true,
            axis_aligned_box_pair: false,
            stronger_kernel_available: false,
            graph_had_unknowns: false,
            retained_face_pairs: 2,
            retained_events: 1,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::Winding,
                candidate_pairs: 2,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            full_face_shared_faces: 0,
            full_face_shared_patches: 0,
            contained_containing_side: None,
            contained_faces: 0,
            containing_faces: 0,
        };
        assert_eq!(
            adjacent.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let certified_full_face = ExactAdjacentUnionCompletionReport {
            operation: ExactBooleanOperation::Union,
            status: ExactAdjacentUnionCompletionStatus::CertifiedFullFace,
            left_closed: true,
            right_closed: true,
            axis_aligned_box_pair: false,
            stronger_kernel_available: false,
            graph_had_unknowns: false,
            retained_face_pairs: 2,
            retained_events: 2,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::BoundaryPolicy,
                candidate_pairs: 2,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            full_face_shared_faces: 3,
            full_face_shared_patches: 0,
            contained_containing_side: None,
            contained_faces: 0,
            containing_faces: 0,
        };
        assert_eq!(
            certified_full_face.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let certified_contained_face = ExactAdjacentUnionCompletionReport {
            operation: ExactBooleanOperation::Union,
            status: ExactAdjacentUnionCompletionStatus::CertifiedContainedFace,
            left_closed: true,
            right_closed: true,
            axis_aligned_box_pair: false,
            stronger_kernel_available: false,
            graph_had_unknowns: false,
            retained_face_pairs: 2,
            retained_events: 2,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::BoundaryPolicy,
                candidate_pairs: 2,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            full_face_shared_faces: 0,
            full_face_shared_patches: 0,
            contained_containing_side: Some(MeshSide::Left),
            contained_faces: 1,
            containing_faces: 3,
        };
        assert_eq!(
            certified_contained_face.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let boundary = ExactBoundaryTouchingReport {
            status: ExactBoundaryTouchingStatus::NotBoundaryOnly,
            graph_had_unknowns: false,
            retained_face_pairs: 2,
            retained_events: 1,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::Winding,
                candidate_pairs: 2,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
        };
        assert_eq!(
            boundary.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let planar = ExactPlanarArrangementReport {
            operation: ExactBooleanOperation::Union,
            status: ExactPlanarArrangementStatus::NoPositiveOverlap,
            graph_had_unknowns: false,
            retained_face_pairs: 2,
            retained_events: 1,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::Winding,
                candidate_pairs: 2,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            coplanar_arrangement_evidence: Some(CoplanarArrangementEvidence {
                status: CoplanarArrangementEvidenceStatus::NoCoplanarOverlap,
                graph_count: 0,
                overlapping_graphs: 0,
                touching_graphs: 0,
                edge_overlap_count: 0,
                vertex_overlap_count: 0,
                point_split_count: 0,
                interval_overlap_count: 0,
                interval_endpoint_count: 0,
            }),
        };
        assert_eq!(
            planar.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let preflight = ExactBooleanPreflight {
            operation: ExactBooleanOperation::Union,
            support: ExactBooleanSupport::CertifiedConvexUnion,
            graph_had_unknowns: false,
            retained_face_pairs: 2,
            retained_events: 1,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: None,
            coplanar_arrangement_evidence: None,
            coplanar_volumetric_evidence: None,
        };
        assert_eq!(
            preflight.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );

        let evidence = ExactWindingEvidenceReport {
            operation: ExactBooleanOperation::Union,
            status: ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized,
            graph_had_unknowns: false,
            retained_face_pairs: 2,
            retained_events: 1,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::Winding,
                candidate_pairs: 2,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            coplanar_arrangement_evidence: None,
            coplanar_volumetric_evidence: None,
        };
        assert_eq!(
            evidence.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn retained_graph_reports_reject_unaccounted_face_pairs() {
        let mut refinement = ExactRefinementReport {
            operation: ExactBooleanOperation::Union,
            status: ExactRefinementStatus::Required,
            graph_had_unknowns: true,
            retained_face_pairs: 2,
            retained_events: 2,
            blocker: Some(ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::Refinement,
                candidate_pairs: 0,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 1,
                construction_failed_events: 0,
            }),
        };
        assert_eq!(
            refinement.validate(),
            Err(ExactReportValidationError::InvalidBlockerCounts)
        );

        let blocker = refinement.blocker.as_mut().unwrap();
        blocker.candidate_pairs = 1;
        assert!(
            refinement.validate().is_ok(),
            "unknown-event evidence can overlap a classified candidate pair"
        );

        let evidence = ExactWindingEvidenceReport {
            operation: ExactBooleanOperation::Union,
            status: ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized,
            graph_had_unknowns: false,
            retained_face_pairs: 2,
            retained_events: 2,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::Winding,
                candidate_pairs: 1,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            coplanar_arrangement_evidence: None,
            coplanar_volumetric_evidence: None,
        };
        assert_eq!(
            evidence.validate(),
            Err(ExactReportValidationError::InvalidBlockerCounts)
        );
    }

    #[test]
    fn planar_arrangement_named_statuses_require_retained_evidence() {
        let mut already_materialized = ExactPlanarArrangementReport {
            operation: ExactBooleanOperation::Union,
            status: ExactPlanarArrangementStatus::AlreadyMaterialized,
            graph_had_unknowns: false,
            retained_face_pairs: 1,
            retained_events: 1,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::PlanarArrangement,
                candidate_pairs: 0,
                coplanar_overlapping_pairs: 1,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            coplanar_arrangement_evidence: Some(CoplanarArrangementEvidence {
                status: CoplanarArrangementEvidenceStatus::NeedsPlanarCells,
                graph_count: 1,
                overlapping_graphs: 1,
                touching_graphs: 0,
                edge_overlap_count: 1,
                vertex_overlap_count: 0,
                point_split_count: 0,
                interval_overlap_count: 0,
                interval_endpoint_count: 0,
            }),
        };
        already_materialized.validate().unwrap();
        assert!(already_materialized.is_already_materialized());
        assert!(!already_materialized.is_required());
        already_materialized.coplanar_arrangement_evidence = None;
        assert_eq!(
            already_materialized.validate(),
            Err(ExactReportValidationError::MissingCoplanarArrangementEvidence)
        );

        let mut no_positive_overlap = ExactPlanarArrangementReport {
            operation: ExactBooleanOperation::Intersection,
            status: ExactPlanarArrangementStatus::NoPositiveOverlap,
            graph_had_unknowns: false,
            retained_face_pairs: 1,
            retained_events: 1,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::Winding,
                candidate_pairs: 1,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            coplanar_arrangement_evidence: Some(CoplanarArrangementEvidence {
                status: CoplanarArrangementEvidenceStatus::NoCoplanarOverlap,
                graph_count: 0,
                overlapping_graphs: 0,
                touching_graphs: 0,
                edge_overlap_count: 0,
                vertex_overlap_count: 0,
                point_split_count: 0,
                interval_overlap_count: 0,
                interval_endpoint_count: 0,
            }),
        };
        no_positive_overlap.validate().unwrap();
        assert!(!no_positive_overlap.is_already_materialized());
        assert!(!no_positive_overlap.is_required());
        no_positive_overlap.coplanar_arrangement_evidence = None;
        assert_eq!(
            no_positive_overlap.validate(),
            Err(ExactReportValidationError::MissingCoplanarArrangementEvidence)
        );

        let mut boundary_policy = ExactPlanarArrangementReport {
            operation: ExactBooleanOperation::Difference,
            status: ExactPlanarArrangementStatus::BoundaryPolicyRequired,
            graph_had_unknowns: false,
            retained_face_pairs: 1,
            retained_events: 1,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::BoundaryPolicy,
                candidate_pairs: 0,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 1,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            coplanar_arrangement_evidence: Some(CoplanarArrangementEvidence {
                status: CoplanarArrangementEvidenceStatus::BoundaryOnly,
                graph_count: 1,
                overlapping_graphs: 0,
                touching_graphs: 1,
                edge_overlap_count: 1,
                vertex_overlap_count: 0,
                point_split_count: 0,
                interval_overlap_count: 0,
                interval_endpoint_count: 0,
            }),
        };
        boundary_policy.validate().unwrap();
        boundary_policy.coplanar_arrangement_evidence = None;
        assert_eq!(
            boundary_policy.validate(),
            Err(ExactReportValidationError::MissingCoplanarArrangementEvidence)
        );
    }

    #[test]
    fn winding_planar_arrangement_materialized_requires_retained_evidence() {
        let evidence = CoplanarArrangementEvidence {
            status: CoplanarArrangementEvidenceStatus::NeedsPlanarCells,
            graph_count: 1,
            overlapping_graphs: 1,
            touching_graphs: 0,
            edge_overlap_count: 1,
            vertex_overlap_count: 0,
            point_split_count: 0,
            interval_overlap_count: 0,
            interval_endpoint_count: 0,
        };
        let mut report = ExactWindingEvidenceReport {
            operation: ExactBooleanOperation::Union,
            status: ExactWindingEvidenceStatus::PlanarArrangementAlreadyMaterialized,
            graph_had_unknowns: false,
            retained_face_pairs: 1,
            retained_events: 1,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::PlanarArrangement,
                candidate_pairs: 0,
                coplanar_overlapping_pairs: 1,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            coplanar_arrangement_evidence: Some(evidence),
            coplanar_volumetric_evidence: None,
        };
        report.validate().unwrap();

        report.coplanar_arrangement_evidence = None;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::MissingCoplanarArrangementEvidence)
        );
    }

    #[test]
    fn winding_closed_boundary_touching_materialized_requires_positive_area_evidence() {
        let evidence = CoplanarVolumetricCellEvidenceReport {
            left_closed_manifold: true,
            right_closed_manifold: true,
            retained_face_pair_count: 1,
            candidate_pairs: 0,
            proper_crossing_candidate_pairs: 0,
            coplanar_touching_pairs: 0,
            coplanar_overlapping_pairs: 1,
            positive_area_coplanar_overlapping_pairs: 1,
            opposite_side_coplanar_overlapping_pairs: 1,
            same_side_coplanar_overlapping_pairs: 0,
            undecided_side_coplanar_overlapping_pairs: 0,
            unknown_pairs: 0,
            segment_plane_events: 0,
            proper_crossing_events: 0,
            boundary_segment_events: 0,
            construction_failed_events: 0,
            unknown_segment_plane_events: 0,
            unknown_events: 0,
            coplanar_edge_events: 1,
            coplanar_vertex_events: 0,
            obstacle: CoplanarVolumetricCellObstacle::BoundaryOnlyContact,
        };
        evidence.validate().unwrap();

        let mut report = ExactWindingEvidenceReport {
            operation: ExactBooleanOperation::Intersection,
            status: ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized,
            graph_had_unknowns: false,
            retained_face_pairs: 1,
            retained_events: 1,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::BoundaryPolicy,
                candidate_pairs: 0,
                coplanar_overlapping_pairs: 1,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            coplanar_arrangement_evidence: None,
            coplanar_volumetric_evidence: Some(evidence.clone()),
        };
        report.validate().unwrap();

        report.coplanar_volumetric_evidence = None;
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::MissingCoplanarVolumetricEvidence)
        );

        let mut relabeled_evidence = evidence;
        relabeled_evidence.coplanar_overlapping_pairs = 0;
        relabeled_evidence.coplanar_touching_pairs = 1;
        relabeled_evidence.positive_area_coplanar_overlapping_pairs = 0;
        relabeled_evidence.opposite_side_coplanar_overlapping_pairs = 0;
        relabeled_evidence.validate().unwrap();
        report.coplanar_volumetric_evidence = Some(relabeled_evidence);
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch)
        );
    }

    #[test]
    fn coplanar_volumetric_evidence_must_match_retained_graph_totals() {
        let evidence = CoplanarVolumetricCellEvidenceReport {
            left_closed_manifold: true,
            right_closed_manifold: true,
            retained_face_pair_count: 2,
            candidate_pairs: 1,
            proper_crossing_candidate_pairs: 1,
            coplanar_touching_pairs: 0,
            coplanar_overlapping_pairs: 1,
            positive_area_coplanar_overlapping_pairs: 1,
            opposite_side_coplanar_overlapping_pairs: 0,
            same_side_coplanar_overlapping_pairs: 1,
            undecided_side_coplanar_overlapping_pairs: 0,
            unknown_pairs: 0,
            segment_plane_events: 1,
            proper_crossing_events: 1,
            boundary_segment_events: 0,
            construction_failed_events: 0,
            unknown_segment_plane_events: 0,
            unknown_events: 0,
            coplanar_edge_events: 3,
            coplanar_vertex_events: 0,
            obstacle: CoplanarVolumetricCellObstacle::MixedCoplanarAndCrossingCells,
        };
        evidence.validate().unwrap();

        let blocker = ExactBooleanBlocker {
            kind: ExactBooleanBlockerKind::CoplanarVolumetricCells,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 1,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        };
        let mut preflight = ExactBooleanPreflight {
            operation: ExactBooleanOperation::Intersection,
            support: ExactBooleanSupport::RequiresCoplanarVolumetricCells,
            graph_had_unknowns: false,
            retained_face_pairs: 2,
            retained_events: 4,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: Some(blocker),
            coplanar_arrangement_evidence: None,
            coplanar_volumetric_evidence: Some(evidence.clone()),
        };
        preflight.validate().unwrap();

        preflight.retained_events = 5;
        assert_eq!(
            preflight.validate(),
            Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch)
        );

        let mut evidence = ExactWindingEvidenceReport {
            operation: ExactBooleanOperation::Intersection,
            status: ExactWindingEvidenceStatus::CoplanarVolumetricCellsRequired,
            graph_had_unknowns: false,
            retained_face_pairs: 2,
            retained_events: 4,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker,
            coplanar_arrangement_evidence: None,
            coplanar_volumetric_evidence: Some(evidence),
        };
        evidence.validate().unwrap();

        evidence.retained_events = 5;
        assert_eq!(
            evidence.validate(),
            Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch)
        );

        let mut overflowing_evidence = evidence
            .coplanar_volumetric_evidence
            .as_ref()
            .unwrap()
            .clone();
        overflowing_evidence.segment_plane_events = usize::MAX;
        overflowing_evidence.proper_crossing_events = usize::MAX;
        overflowing_evidence.validate().unwrap();
        evidence.retained_events = usize::MAX;
        evidence.coplanar_volumetric_evidence = Some(overflowing_evidence);
        assert_eq!(
            evidence.validate(),
            Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch)
        );
    }

    #[test]
    fn blocker_source_counts_include_unknown_segment_plane_events() {
        let graph = ExactIntersectionGraph::from_face_pairs(vec![crate::graph::FacePairEvents {
            left_face: 0,
            right_face: 0,
            relation: MeshFacePairRelation::Candidate,
            projection: None,
            events: vec![IntersectionEvent::SegmentPlane {
                segment_side: MeshSide::Left,
                edge: [0, 1],
                plane_side: MeshSide::Right,
                plane_face: 0,
                relation: hyperlimit::SegmentPlaneRelation::Unknown,
                point: None,
                parameter: None,
                parameter_ratio: None,
                construction_failure: None,
                endpoint_sides: [None, Some(hyperlimit::PlaneSide::Above)],
            }],
        }]);

        let blocker = ExactBooleanBlocker::from_graph(&graph, ExactBooleanBlockerKind::Refinement);
        assert_eq!(blocker.candidate_pairs, 1);
        assert_eq!(blocker.unknown_pairs, 1);
        assert_eq!(blocker.construction_failed_events, 0);
        assert!(
            blocker
                .validate_for_kind(ExactBooleanBlockerKind::Refinement)
                .is_ok()
        );
    }

    #[test]
    fn refinement_report_allows_unknown_event_on_candidate_pair() {
        let report = ExactRefinementReport {
            operation: ExactBooleanOperation::Union,
            status: ExactRefinementStatus::Required,
            graph_had_unknowns: true,
            retained_face_pairs: 1,
            retained_events: 1,
            blocker: Some(ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::Refinement,
                candidate_pairs: 1,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 1,
                construction_failed_events: 0,
            }),
        };

        assert!(report.validate().is_ok());
    }
}
