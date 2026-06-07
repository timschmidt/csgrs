//! Auditable exact boolean reports.
//!
//! These types are the public evidence objects produced by the exact boolean
//! staging layer. They intentionally carry graph counts, predicate
//! certificates, and checked handoff artifacts instead of collapsing exact
//! topology decisions to `bool`. Callers can inspect whether a combinatorial
//! decision was certified, unsupported, or blocked on an application-level
//! policy.

use hyperlimit::{Point3, compare_reals};
use std::cmp::Ordering;

use super::ExactMesh;
use super::boolean::{
    ExactBooleanOperation, ExactBoundaryBooleanPolicy, boolean_exact_with_boundary_policy,
    certify_adjacent_union_completion_report, certify_boundary_touching_report,
    certify_open_surface_disjoint_report, certify_planar_arrangement_report,
    certify_refinement_report, certify_same_surface_report,
    certify_volumetric_boundary_closure_report, certify_winding_readiness_report,
    materialize_adjacent_union_completion_boolean, materialize_closed_same_surface_boolean,
    preflight_boolean_exact, preflight_boolean_exact_with_boundary_policy,
    preflight_boolean_exact_with_validation, replay_volumetric_winding_region_plan,
};
use super::bounds::AabbIntersectionKind;
use super::convex::{
    intersect_closed_convex_solids, subtract_closed_convex_solids, union_closed_convex_solids,
};
use super::graph::MeshSide;
use super::graph::{
    CoplanarArrangementReadinessReport, CoplanarArrangementReadinessStatus, ExactIntersectionGraph,
    IntersectionEvent, build_intersection_graph,
};
use super::intersection::MeshFacePairRelation;
use super::region::{
    ExactBooleanAssemblyPlan, ExactOutputTriangle, ExactRegionSelection,
    FaceRegionPlaneClassification, FaceRegionPlaneValidationError, FaceRegionTriangulation,
    boundary_node_point, replay_region_facts_against_sources,
};
use super::regularization::ExactArrangementBlocker;
use super::solid::{
    ConvexSolidMeshClassification, ConvexSolidMeshRelation, ConvexSolidPointRelation,
    classify_mesh_vertices_against_convex_solid_report,
};
use super::validation::ValidationPolicy;
use super::volumetric::{ExactVolumetricRegionClassification, ExactVolumetricRegionError};
use super::volumetric_cells::{
    CoplanarVolumetricCellEvidenceReport, CoplanarVolumetricCellObstacle,
};
use super::winding::{
    ClosedMeshWindingMeshRelation, classify_mesh_vertices_against_closed_mesh_winding_report,
};
use hyperlimit::PredicateUse;

/// Validation failure for an exact report object.
///
/// Report validation checks the evidence object itself, not the original
/// geometry. It lets tests, fuzzing, and downstream policy code assert that
/// status, blocker kind, graph counts, and retained artifacts agree before
/// metadata consistency part of the certified boundary.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactReportValidationError {
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
    /// A winding-ready report did not retain checked region facts.
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
    /// A winding-ready report retained a region/plane classification that still
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
    /// An arrangement-materialized result retained boundary, unknown, or nonclosed
    /// region evidence.
    VolumetricClassificationNotDecided,
    /// The materialized output mesh failed retained-state validation.
    InvalidOutputMesh,
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
    /// readiness summary required for its status.
    MissingArrangementReadiness,
    /// A planar-arrangement report retained a readiness summary where none is
    /// coherent for its status.
    UnexpectedArrangementReadiness,
    /// A retained planar-arrangement readiness summary failed its own count
    /// audit.
    InvalidArrangementReadiness,
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
    /// Planar-arrangement blocker counts and retained readiness counts
    /// disagree.
    ArrangementReadinessMismatch,
    /// A same-surface report retained a non-bijective vertex permutation.
    InvalidPermutation,
    /// A certified same-surface report retained unequal remapped triangle sets.
    MismatchedTriangleSets,
    /// A retained report no longer matches facts recomputed from the supplied
    /// source meshes.
    SourceReplayMismatch,
}

/// Shared freshness status for copied exact boolean reports.
///
/// warrants replayable exact state at predicate/construction/topology
/// boundaries, but not redundant metadata vocabularies for each wrapper. The
/// shared status preserves the useful distinction between local report drift
/// and source-replay drift while keeping [`ExactReportValidationError`] as the
/// detailed diagnostic surface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactReportFreshness {
    /// The report validates locally and replays exactly from the source meshes.
    Current,
    /// The unknown-graph flag no longer matches the reported status.
    StaleGraphUnknownStatus,
    /// Blocker kind, relation counts, or required relation evidence drifted.
    StaleBlockerEvidence,
    /// Status, operation, or precondition evidence no longer agree.
    StaleStatusEvidence,
    /// Retained region, triangulation, or assembly provenance is missing,
    /// duplicated, invalid, or stale.
    StaleRegionFacts,
    /// Required coplanar-readiness evidence is absent.
    MissingArrangementReadiness,
    /// Coplanar-readiness evidence is present for a status that cannot use it.
    UnexpectedArrangementReadiness,
    /// The retained coplanar-readiness summary failed its own validation.
    InvalidArrangementReadiness,
    /// Readiness counts no longer agree with retained blocker counts.
    StaleArrangementReadiness,
    /// Required coplanar-volumetric evidence is absent.
    MissingCoplanarVolumetricEvidence,
    /// Coplanar-volumetric evidence is present for a status that cannot use it.
    UnexpectedCoplanarVolumetricEvidence,
    /// The retained coplanar-volumetric evidence failed its own validation.
    InvalidCoplanarVolumetricEvidence,
    /// Volumetric-cell evidence counts no longer agree with retained blocker
    /// counts or report status.
    StaleCoplanarVolumetricEvidence,
    /// A validation error outside the report's freshness categories occurred.
    InvalidReportShape,
    /// The report is locally valid but no longer replays from the sources.
    SourceReplayMismatch,
    /// The report replays from the sources, but not for the requested
    /// operation, validation policy, or boundary policy.
    OperationReplayMismatch,
}

impl From<ExactReportValidationError> for ExactReportFreshness {
    fn from(error: ExactReportValidationError) -> Self {
        match error {
            ExactReportValidationError::GraphUnknownStatusMismatch => Self::StaleGraphUnknownStatus,
            ExactReportValidationError::CertifiedReportHasBlocker
            | ExactReportValidationError::MissingBlocker
            | ExactReportValidationError::WrongBlockerKind
            | ExactReportValidationError::InvalidBlockerCounts
            | ExactReportValidationError::MissingRelationCount => Self::StaleBlockerEvidence,
            ExactReportValidationError::StatusEvidenceMismatch
            | ExactReportValidationError::InvalidPermutation
            | ExactReportValidationError::MismatchedTriangleSets => Self::StaleStatusEvidence,
            ExactReportValidationError::UnexpectedRegionFacts
            | ExactReportValidationError::MissingRegionFacts
            | ExactReportValidationError::UnclassifiedRegionTriangulation
            | ExactReportValidationError::OrphanedRegionClassification
            | ExactReportValidationError::UntriangulatedAssemblyRegion
            | ExactReportValidationError::AssemblyVertexOutsideTriangulation
            | ExactReportValidationError::UnreferencedAssemblyVertex
            | ExactReportValidationError::InvalidRegionClassification(_)
            | ExactReportValidationError::RegionClassificationNotProofProducing
            | ExactReportValidationError::RegionCountMismatch
            | ExactReportValidationError::DuplicateRegionClassification
            | ExactReportValidationError::DuplicateRegionTriangulation
            | ExactReportValidationError::InvalidTriangulation
            | ExactReportValidationError::InvalidAssembly
            | ExactReportValidationError::InvalidVolumetricClassification(_)
            | ExactReportValidationError::MissingVolumetricClassifications
            | ExactReportValidationError::UnexpectedVolumetricClassifications
            | ExactReportValidationError::OrphanedVolumetricClassification
            | ExactReportValidationError::UnclassifiedVolumetricTriangulation
            | ExactReportValidationError::VolumetricClassificationNotDecided
            | ExactReportValidationError::InvalidOutputMesh
            | ExactReportValidationError::ShortcutResultHasAssemblyArtifacts
            | ExactReportValidationError::OutputMeshAssemblyMismatch => Self::StaleRegionFacts,
            ExactReportValidationError::ShortcutResultHasUnknownGraph
            | ExactReportValidationError::SelectedRegionResultHasUnknownGraph
            | ExactReportValidationError::UnexpectedGraphEvents => Self::StaleGraphUnknownStatus,
            ExactReportValidationError::OutputSourceReplayMismatch => Self::SourceReplayMismatch,
            ExactReportValidationError::SelectedRegionAssemblyViolatesSelection
            | ExactReportValidationError::VolumetricMaterializedAssemblyViolatesOperation => {
                Self::StaleStatusEvidence
            }
            ExactReportValidationError::MissingArrangementReadiness => {
                Self::MissingArrangementReadiness
            }
            ExactReportValidationError::UnexpectedArrangementReadiness => {
                Self::UnexpectedArrangementReadiness
            }
            ExactReportValidationError::InvalidArrangementReadiness => {
                Self::InvalidArrangementReadiness
            }
            ExactReportValidationError::ArrangementReadinessMismatch => {
                Self::StaleArrangementReadiness
            }
            ExactReportValidationError::MissingCoplanarVolumetricEvidence => {
                Self::MissingCoplanarVolumetricEvidence
            }
            ExactReportValidationError::UnexpectedCoplanarVolumetricEvidence => {
                Self::UnexpectedCoplanarVolumetricEvidence
            }
            ExactReportValidationError::InvalidCoplanarVolumetricEvidence => {
                Self::InvalidCoplanarVolumetricEvidence
            }
            ExactReportValidationError::CoplanarVolumetricEvidenceMismatch => {
                Self::StaleCoplanarVolumetricEvidence
            }
            ExactReportValidationError::SourceReplayMismatch => Self::SourceReplayMismatch,
        }
    }
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

fn blocker_pair_count(blocker: &ExactBooleanBlocker) -> usize {
    blocker.candidate_pairs
        + blocker.coplanar_overlapping_pairs
        + blocker.coplanar_touching_pairs
        + blocker.unknown_pairs
}

fn validate_blocker_count_bounds(
    blocker: &ExactBooleanBlocker,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactReportValidationError> {
    let classified_relation_pairs = blocker
        .candidate_pairs
        .saturating_add(blocker.coplanar_overlapping_pairs)
        .saturating_add(blocker.coplanar_touching_pairs);
    if retained_face_pairs == 0 && retained_events != 0
        || retained_face_pairs != 0 && retained_events == 0
        || (retained_face_pairs != 0 && !blocker_has_any_evidence(blocker))
        || classified_relation_pairs > retained_face_pairs
        || blocker.unknown_pairs > retained_face_pairs
        || blocker.construction_failed_events > retained_events
    {
        Err(ExactReportValidationError::InvalidBlockerCounts)
    } else {
        Ok(())
    }
}

fn validate_arrangement_readiness_matches_blocker(
    readiness: &CoplanarArrangementReadinessReport,
    blocker: &ExactBooleanBlocker,
) -> Result<(), ExactReportValidationError> {
    // The compact readiness report and the blocker are two public views of the
    // structure as part of the exact state; a later planar-cell or winding
    // policy must not be able to consume a summary with relabeled graph counts.
    if readiness.overlapping_graphs != blocker.coplanar_overlapping_pairs
        || readiness.touching_graphs != blocker.coplanar_touching_pairs
        || readiness.graph_count
            != blocker.coplanar_overlapping_pairs + blocker.coplanar_touching_pairs
    {
        Err(ExactReportValidationError::ArrangementReadinessMismatch)
    } else {
        Ok(())
    }
}

fn blocker_has_any_evidence(blocker: &ExactBooleanBlocker) -> bool {
    blocker_pair_count(blocker) != 0 || blocker.construction_failed_events != 0
}

fn blocker_has_refinement_evidence(blocker: &ExactBooleanBlocker) -> bool {
    blocker.unknown_pairs != 0 || blocker.construction_failed_events != 0
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct BlockerSourceCounts {
    candidate_pairs: usize,
    coplanar_overlapping_pairs: usize,
    coplanar_touching_pairs: usize,
    unknown_pairs: usize,
    construction_failed_events: usize,
}

impl BlockerSourceCounts {
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

fn blocker_source_counts(graph: &ExactIntersectionGraph) -> BlockerSourceCounts {
    let mut counts = BlockerSourceCounts::default();
    for pair in &graph.face_pairs {
        let pair_has_unknown_event = pair
            .events
            .iter()
            .any(IntersectionEvent::has_unknown_relation);
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
                    IntersectionEvent::SegmentPlane {
                        relation: hyperlimit::SegmentPlaneRelation::ConstructionFailed,
                        ..
                    }
                )
            })
            .count();
    }
    counts
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

fn operation_is_selected_region(operation: ExactBooleanOperation) -> bool {
    matches!(operation, ExactBooleanOperation::SelectedRegions(_))
}

const fn certified_preflight_support_matches_operation(
    support: ExactBooleanSupport,
    operation: ExactBooleanOperation,
) -> bool {
    match (support, operation) {
        (
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
            | ExactBooleanSupport::CertifiedConvexUnion,
            ExactBooleanOperation::Union,
        )
        | (
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
            | ExactBooleanSupport::CertifiedConvexIntersection,
            ExactBooleanOperation::Intersection,
        )
        | (
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
            | ExactBooleanSupport::CertifiedConvexDifference,
            ExactBooleanOperation::Difference,
        ) => true,
        (
            ExactBooleanSupport::CertifiedEmptyOperand
            | ExactBooleanSupport::CertifiedBoundsDisjoint
            | ExactBooleanSupport::CertifiedIdentical
            | ExactBooleanSupport::CertifiedSameSurface
            | ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
            | ExactBooleanSupport::CertifiedClosedWindingSeparated
            | ExactBooleanSupport::CertifiedClosedWindingContainment
            | ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid
            | ExactBooleanSupport::CertifiedBoundaryPolicyShortcut
            | ExactBooleanSupport::CertifiedConvexContainment
            | ExactBooleanSupport::CertifiedConvexSeparated,
            ExactBooleanOperation::Union
            | ExactBooleanOperation::Intersection
            | ExactBooleanOperation::Difference,
        ) => true,
        _ => false,
    }
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
        // A winding-ready handoff is stronger than a stored classification
        // artifact: future inside/outside policy must be able to consume
        // decided side facts, not an "unknown" region/plane relation. This is
        // predicates remain explicit blockers instead of being mislabeled as a
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
) -> Result<(), ExactReportValidationError> {
    evidence
        .validate()
        .map_err(|_| ExactReportValidationError::InvalidCoplanarVolumetricEvidence)?;
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
    if evidence.retained_face_pair_count != retained_face_pairs
        || coplanar_volumetric_evidence_event_count(evidence) != retained_events
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

fn coplanar_boundary_only_evidence_is_positive_area(
    evidence: &CoplanarVolumetricCellEvidenceReport,
) -> bool {
    evidence.obstacle == CoplanarVolumetricCellObstacle::BoundaryOnlyContact
        && evidence.positive_area_coplanar_overlapping_pairs != 0
}

fn validate_coplanar_boundary_only_evidence_shape(
    evidence: &CoplanarVolumetricCellEvidenceReport,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactReportValidationError> {
    validate_coplanar_volumetric_evidence_counts(evidence, retained_face_pairs, retained_events)?;
    if !coplanar_boundary_only_evidence_is_positive_area(evidence) {
        return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
    }
    Ok(())
}

fn validate_certified_arrangement_coplanar_evidence_shape(
    evidence: &CoplanarVolumetricCellEvidenceReport,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactReportValidationError> {
    validate_coplanar_volumetric_evidence_counts(evidence, retained_face_pairs, retained_events)?;
    if !evidence.obstacle.requires_coplanar_volumetric_cells()
        && !coplanar_boundary_only_evidence_is_positive_area(evidence)
    {
        return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
    }
    Ok(())
}

fn coplanar_volumetric_evidence_event_count(
    evidence: &CoplanarVolumetricCellEvidenceReport,
) -> usize {
    let explicit_unknown_events = evidence
        .unknown_events
        .saturating_sub(evidence.unknown_segment_plane_events);
    evidence
        .segment_plane_events
        .saturating_add(evidence.coplanar_edge_events)
        .saturating_add(evidence.coplanar_vertex_events)
        .saturating_add(explicit_unknown_events)
}

/// Auditable result of an exact selected-region boolean pipeline.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBooleanResult {
    /// Declared production path for this result.
    pub kind: ExactBooleanResultKind,
    /// Whether graph extraction contained unknown events before policy checks.
    pub graph_had_unknowns: bool,
    /// Certified classifications of split regions against opposite face
    /// planes.
    pub region_classifications: Vec<FaceRegionPlaneClassification>,
    /// Exact projected triangulations used for assembly.
    pub triangulations: Vec<FaceRegionTriangulation>,
    /// Non-mutating exact output assembly.
    pub assembly: ExactBooleanAssemblyPlan,
    /// Exact winding classifications used by volumetric arrangement materialization.
    pub volumetric_classifications: Vec<ExactVolumetricRegionClassification>,
    /// Materialized exact output mesh validated under the requested policy.
    pub mesh: ExactMesh,
}

/// Declared production path for an exact boolean result.
///
/// Result kind is explicit so validation does not infer semantic intent from
/// empty vectors. That distinction matters for exact computing: selected-region
/// assembly, certified shortcuts, and boundary-policy projections are different
/// application contracts even when they all produce an empty mesh. The design
/// topology.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBooleanResultKind {
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
pub enum ExactBooleanShortcutKind {
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
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
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
        if let ExactBooleanResultKind::BoundaryPolicyShortcut { operation }
        | ExactBooleanResultKind::OpenSurfaceArrangement { operation }
        | ExactBooleanResultKind::ArrangementCellComplexMaterialized { operation } = self.kind
            && operation_is_selected_region(operation)
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
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
        self.mesh
            .validate_retained_state()
            .map_err(|_| ExactReportValidationError::InvalidOutputMesh)?;

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
            ExactBooleanResultKind::OpenSurfaceArrangement { operation } => {
                Some(open_surface_arrangement_selection(operation)?)
            }
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
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        if matches!(
            self.kind,
            ExactBooleanResultKind::SelectedRegions { .. }
                | ExactBooleanResultKind::OpenSurfaceArrangement { .. }
        ) {
            let replay = replay_region_facts_against_sources(left, right)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
            if self.region_classifications != replay.0 || self.triangulations != replay.1 {
                return Err(ExactReportValidationError::SourceReplayMismatch);
            }
        }
        if matches!(
            self.kind,
            ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
        ) {
            let replay = replay_volumetric_winding_region_plan(left, right)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
                .ok_or(ExactReportValidationError::SourceReplayMismatch)?;
            if self.region_classifications != replay.0 || self.triangulations != replay.1 {
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
        if let ExactBooleanResultKind::BoundaryPolicyShortcut { operation } = self.kind {
            let replay = certify_boundary_touching_report(left, right)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
            replay.validate()?;
            if !replay.is_certified() {
                return Err(ExactReportValidationError::SourceReplayMismatch);
            }
            let replay = boolean_exact_with_boundary_policy(
                left,
                right,
                operation,
                self.mesh.validation_policy(),
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
            if self != &replay {
                return Err(ExactReportValidationError::SourceReplayMismatch);
            }
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut:
                ExactBooleanShortcutKind::EmptyOperand
                | ExactBooleanShortcutKind::BoundsDisjoint
                | ExactBooleanShortcutKind::Identical
                | ExactBooleanShortcutKind::SameSurface
                | ExactBooleanShortcutKind::OpenSurfaceDisjoint
                | ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid
                | ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid,
        } = self.kind
        {
            let replay = boolean_exact_with_boundary_policy(
                left,
                right,
                operation,
                self.mesh.validation_policy(),
                ExactBoundaryBooleanPolicy::Reject,
            )
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
            if self != &replay {
                return Err(ExactReportValidationError::SourceReplayMismatch);
            }
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut,
        } = self.kind
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
        Ok(())
    }

    /// Classify whether this retained result is fresh for the source meshes.
    ///
    /// Local report integrity is checked before source replay so copied
    /// materialized outputs can distinguish stale retained artifacts from
    /// source-geometry drift.
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

    /// Validate this result against the operation and policies that produced it.
    ///
    /// [`Self::validate_against_sources`] audits retained source provenance for
    /// selected-region assembly and local mesh state for shortcuts. This
    /// stronger replay recomputes the public exact boolean entry point for the
    /// same operands, operation, validation policy, and boundary policy, then
    /// requires the whole result object to match. That closes the shortcut
    /// replay gap: a certified output mesh cannot be relabeled as a different
    /// named operation or shortcut kind while still passing the source audit.
    /// itself as part of the exact computation history.
    pub fn validate_operation_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        operation: ExactBooleanOperation,
        validation: ValidationPolicy,
        boundary_policy: ExactBoundaryBooleanPolicy,
    ) -> Result<(), ExactReportValidationError> {
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation: retained_operation,
            ..
        } = self.kind
            && retained_operation != operation
        {
            return Err(ExactReportValidationError::SourceReplayMismatch);
        }
        self.validate_against_sources(left, right)?;
        let replay =
            boolean_exact_with_boundary_policy(left, right, operation, validation, boundary_policy)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this result still matches its full operation replay.
    pub fn operation_freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        operation: ExactBooleanOperation,
        validation: ValidationPolicy,
        boundary_policy: ExactBoundaryBooleanPolicy,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate_against_sources(left, right) {
            return error.into();
        }
        match boolean_exact_with_boundary_policy(
            left,
            right,
            operation,
            validation,
            boundary_policy,
        ) {
            Ok(replay) if self == &replay => ExactReportFreshness::Current,
            Ok(_) | Err(_) => ExactReportFreshness::OperationReplayMismatch,
        }
    }
}

fn certified_shortcut_sources_match(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
    validation: ValidationPolicy,
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
            let report = certify_same_surface_report(left, right);
            report.validate()?;
            Ok(report.is_certified())
        }
        ExactBooleanShortcutKind::OpenSurfaceDisjoint => {
            let report = certify_open_surface_disjoint_report(left, right)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
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
    let (Some(left_bounds), Some(right_bounds)) = (&left.bounds().mesh, &right.bounds().mesh)
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
    let report = certify_boundary_touching_report(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    report.validate()?;
    if !report.is_certified() {
        if matches!(
            shortcut,
            ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection
                | ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference
        ) {
            let graph = build_intersection_graph(left, right)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
            graph
                .validate_against_sources(left, right)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
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
        let graph = build_intersection_graph(left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        graph
            .validate_against_sources(left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
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

fn closed_winding_sources_match(
    shortcut: ExactBooleanShortcutKind,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    if !mesh_is_closed_solid(left) || !mesh_is_closed_solid(right) {
        return Ok(false);
    }
    let graph = build_intersection_graph(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    graph
        .validate_against_sources(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
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

fn convex_shortcut_sources_match(
    shortcut: ExactBooleanShortcutKind,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    Ok(match shortcut {
        ExactBooleanShortcutKind::ConvexUnion => union_closed_convex_solids(left, right).is_some(),
        ExactBooleanShortcutKind::ConvexIntersection => {
            intersect_closed_convex_solids(left, right).is_some()
        }
        ExactBooleanShortcutKind::ConvexDifference => {
            subtract_closed_convex_solids(left, right).is_some()
        }
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
    let graph = build_intersection_graph(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    graph
        .validate_against_sources(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
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
    validation: ValidationPolicy,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<bool, ExactReportValidationError> {
    if operation == ExactBooleanOperation::Union
        && materialize_adjacent_union_completion_boolean(left, right, operation, validation)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
            .is_some()
    {
        return Ok(true);
    }
    if materialize_closed_same_surface_boolean(left, right, operation, validation)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?
        .is_some()
    {
        return Ok(true);
    }

    let graph = build_intersection_graph(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    graph
        .validate_against_sources(left, right)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    if graph.has_unknowns() || graph.face_pairs.is_empty() {
        return Ok(false);
    }
    let preflight = preflight_boolean_exact_with_validation(left, right, operation, validation)
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
    preflight.validate()?;
    Ok(preflight.support == ExactBooleanSupport::CertifiedArrangementCellComplex)
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

fn open_surface_arrangement_selection(
    operation: ExactBooleanOperation,
) -> Result<ExactRegionSelection, ExactReportValidationError> {
    match operation {
        ExactBooleanOperation::Intersection => Ok(ExactRegionSelection::KeepNone),
        ExactBooleanOperation::Union => Ok(ExactRegionSelection::KeepAll),
        ExactBooleanOperation::Difference => Ok(ExactRegionSelection::KeepLeft),
        ExactBooleanOperation::SelectedRegions(_) => {
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        }
    }
}

/// Local per-cell retention state for an arrangement-materialized result.
///
/// This mirrors the named-boolean assembly policy, but lives in the public
/// report validator so a copied result can be audited without re-running the
/// boolean executor. Keeping the operation decision replayable from retained
/// only valid while the retained predicate facts still justify exactly the
/// emitted combinatorics.
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
            let source_matches = assembly
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
                .collect::<Vec<_>>();
            let geometric_matches = assembly
                .triangles
                .iter()
                .filter(|output| {
                    output_triangle_matches_triangulated_cell(
                        output,
                        assembly,
                        triangulation,
                        triangle,
                    )
                })
                .collect::<Vec<_>>();
            match expected {
                VolumetricCellRetention::Drop if !source_matches.is_empty() => {
                    return Err(
                        ExactReportValidationError::VolumetricMaterializedAssemblyViolatesOperation,
                    );
                }
                VolumetricCellRetention::Keep | VolumetricCellRetention::KeepReversed => {
                    let _ = geometric_matches;
                }
                VolumetricCellRetention::Drop => {}
            }
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
        (
            ExactBooleanOperation::Union,
            _,
            super::volumetric::ExactVolumetricRegionRelation::Outside,
        )
        | (
            ExactBooleanOperation::Union,
            MeshSide::Left,
            super::volumetric::ExactVolumetricRegionRelation::Boundary,
        )
        | (
            ExactBooleanOperation::Intersection,
            _,
            super::volumetric::ExactVolumetricRegionRelation::Inside,
        )
        | (
            ExactBooleanOperation::Intersection,
            MeshSide::Left,
            super::volumetric::ExactVolumetricRegionRelation::Boundary,
        )
        | (
            ExactBooleanOperation::Difference,
            MeshSide::Left,
            super::volumetric::ExactVolumetricRegionRelation::Outside,
        ) => VolumetricCellRetention::Keep,
        (
            ExactBooleanOperation::Difference,
            MeshSide::Right,
            super::volumetric::ExactVolumetricRegionRelation::Inside,
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
        (ExactRegionSelection::KeepAll, _)
            | (ExactRegionSelection::KeepLeft, MeshSide::Left)
            | (ExactRegionSelection::KeepRight, MeshSide::Right)
    )
}

/// Certified support level for a requested exact boolean operation.
///
/// computing as an application-level contract: unresolved combinatorics must be
/// represented explicitly instead of being decided by approximate arithmetic.
/// These variants therefore distinguish executable certified shortcuts from
/// cases whose split regions are available but still need exact winding policy.
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

/// Preflight report for an exact boolean operation request.
///
/// The report gives callers a stable way to audit the current implementation
/// boundary. Shortcut variants are executable by `boolean_exact`. For
/// nontrivial named booleans, the report exposes the certified split-region
/// plane classifications that a future exact winding/inside-outside rule must
/// consume, without dispatching to the specialized tolerance kernel.
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
    /// Checked coplanar-overlap readiness retained when preflight stops at a
    /// planar arrangement boundary.
    ///
    /// This deliberately keeps the exact graph handoff visible at the public
    /// structured program state; the positive-area coplanar graph evidence
    /// must not be flattened into a generic "unsupported" boolean.
    pub arrangement_readiness: Option<CoplanarArrangementReadinessReport>,
    /// Source-aware coplanar volumetric-cell evidence retained when the
    /// preflight crosses that exact boundary.
    ///
    /// This report separates boundary-only opposite-side shared faces from
    /// same-side or undecided positive-area coplanar overlap. Retaining it
    /// exact object evidence that authorized a blocker, a no-volume boundary
    /// shortcut, or an arrangement-materialized consumption of coplanar
    /// source-face cells.
    pub coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
}

/// Closure status for a materialized volumetric boundary-output Boolean.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactVolumetricBoundaryClosureStatus {
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

/// Auditable closure-readiness report for volumetric split-cell output.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactVolumetricBoundaryClosureReport {
    /// Requested named operation.
    pub operation: ExactBooleanOperation,
    /// Certified closure status.
    pub status: ExactVolumetricBoundaryClosureStatus,
    /// Number of output triangles in the retained boundary materialization.
    pub output_triangles: usize,
    /// Number of boundary edges retained by the materialized output mesh.
    pub boundary_edges: usize,
    /// Number of directed boundary loops, when loop extraction succeeded.
    pub boundary_loops: usize,
    /// Number of boundary vertices whose outgoing directed boundary-edge count
    /// is not exactly one.
    pub boundary_vertices_with_invalid_outgoing_degree: usize,
    /// Number of boundary vertices whose incoming directed boundary-edge count
    /// is not exactly one.
    pub boundary_vertices_with_invalid_incoming_degree: usize,
    /// Number of undirected mesh edges used more than twice by output
    /// triangles, proving non-manifold topology before boundary-loop walking.
    pub overused_boundary_edges: usize,
    /// Number of boundary loops proven not exactly coplanar.
    pub noncoplanar_boundary_loops: usize,
    /// Number of repeated exact point pairs found inside directed boundary loops.
    pub repeated_exact_boundary_points: usize,
    /// Number of exact point classes that appear at multiple topological
    /// vertices inside directed boundary loops.
    pub self_contact_exact_points: usize,
    /// Number of topological boundary vertices participating in exact
    /// self-contact point classes.
    pub self_contact_topological_vertices: usize,
    /// Number of split cycles around exact self-contact points with fewer than
    /// three distinct exact points.
    pub self_contact_degenerate_cycles: usize,
    /// Number of split cycles around exact self-contact points with at least
    /// three distinct exact points.
    pub self_contact_nondegenerate_cycles: usize,
    /// Number of coplanar loop groups produced by exact loop grouping.
    pub coplanar_loop_groups: usize,
}

impl ExactVolumetricBoundaryClosureReport {
    /// Validate this report against the source meshes that produced it.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay = certify_volumetric_boundary_closure_report(left, right, self.operation)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained boundary-closure report is fresh.
    ///
    /// Local status/count coherence is checked before source replay, so callers
    /// can distinguish stale closure evidence from source-geometry drift.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match certify_volumetric_boundary_closure_report(left, right, self.operation) {
            Ok(replay) if self == &replay => ExactReportFreshness::Current,
            Ok(_) | Err(_) => ExactReportFreshness::SourceReplayMismatch,
        }
    }

    /// Validate status and retained closure counts.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
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
                if self.output_triangles == 0
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
                    || self.coplanar_loop_groups != 0
                    || !self.has_valid_optional_self_contact_evidence()
                    || !volumetric_boundary_closure_blocker_is_supported(blocker)
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
        self.repeated_exact_boundary_points != 0
            && self.self_contact_exact_points != 0
            && self.self_contact_topological_vertices >= 2 * self.self_contact_exact_points
            && self.repeated_exact_boundary_points
                >= self.self_contact_topological_vertices - self.self_contact_exact_points
            && self.self_contact_degenerate_cycles + self.self_contact_nondegenerate_cycles
                == self.self_contact_topological_vertices
    }
}

fn volumetric_boundary_closure_blocker_is_supported(blocker: &ExactArrangementBlocker) -> bool {
    matches!(
        blocker,
        ExactArrangementBlocker::UndecidableOrdering
            | ExactArrangementBlocker::NonManifoldCellComplex
    )
}

impl ExactBooleanPreflight {
    /// Validate this preflight report against the supplied source meshes.
    ///
    /// [`validate`](Self::validate) checks internal consistency. This method
    /// also replays the exact preflight construction from the original meshes
    /// frames exact geometric state as certified computation history; a cached
    /// preflight report must therefore stay tied to the sources that produced
    /// its graph counts, blockers, and retained classifications.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay = preflight_boolean_exact(left, right, self.operation)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Validate this preflight report against source meshes and an explicit
    /// output validation policy.
    ///
    /// The default source replay intentionally uses the strict closed-output
    /// preflight contract. Policy-aware callers that accepted boundary output
    /// need replay to include that policy, otherwise a materialized
    /// arrangement/cell-complex preflight could be incorrectly compared
    /// against the closed-output blocker report.
    pub fn validate_against_sources_with_validation(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        validation: ValidationPolicy,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay =
            preflight_boolean_exact_with_validation(left, right, self.operation, validation)
                .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Validate this preflight report against source meshes, validation policy,
    /// and boundary-output policy.
    ///
    /// Boundary-only named booleans are intentionally blocked by the default
    /// preflight until a caller chooses how to project lower-dimensional
    /// contact. This replay includes that chosen policy, allowing a retained
    /// `CertifiedBoundaryPolicyShortcut` preflight to prove it still matches
    /// the exact graph and output validation contract.
    pub fn validate_against_sources_with_boundary_policy(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        validation: ValidationPolicy,
        boundary_policy: ExactBoundaryBooleanPolicy,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay = preflight_boolean_exact_with_boundary_policy(
            left,
            right,
            self.operation,
            validation,
            boundary_policy,
        )
        .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained preflight is fresh for the source meshes.
    ///
    /// This uses the default strict closed-output preflight contract. Use
    /// [`Self::freshness_against_sources_with_validation`] when a caller
    /// deliberately accepted a different output validation policy.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match preflight_boolean_exact(left, right, self.operation) {
            Ok(replay) if self == &replay => ExactReportFreshness::Current,
            Ok(_) | Err(_) => ExactReportFreshness::SourceReplayMismatch,
        }
    }

    /// Classify whether this retained preflight is fresh under `validation`.
    pub fn freshness_against_sources_with_validation(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        validation: ValidationPolicy,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match preflight_boolean_exact_with_validation(left, right, self.operation, validation) {
            Ok(replay) if self == &replay => ExactReportFreshness::Current,
            Ok(_) | Err(_) => ExactReportFreshness::SourceReplayMismatch,
        }
    }

    /// Classify whether this retained preflight is fresh under `validation`
    /// and `boundary_policy`.
    pub fn freshness_against_sources_with_boundary_policy(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        validation: ValidationPolicy,
        boundary_policy: ExactBoundaryBooleanPolicy,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match preflight_boolean_exact_with_boundary_policy(
            left,
            right,
            self.operation,
            validation,
            boundary_policy,
        ) {
            Ok(replay) if self == &replay => ExactReportFreshness::Current,
            Ok(_) | Err(_) => ExactReportFreshness::SourceReplayMismatch,
        }
    }

    /// Validate support, blocker, and retained artifact consistency.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        // Preflight is the public contract between exact graph construction and
        // expose exact state rather than hide contradictions behind a boolean
        // success/failure bit.
        if (self.retained_face_pairs == 0 && self.retained_events != 0)
            || (self.retained_face_pairs != 0 && self.retained_events == 0)
        {
            return Err(ExactReportValidationError::StatusEvidenceMismatch);
        }
        if self.coplanar_volumetric_evidence.is_some()
            && !matches!(
                self.support,
                ExactBooleanSupport::CertifiedArrangementCellComplex
                    | ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
                    | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
                    | ExactBooleanSupport::RequiresCoplanarVolumetricCells
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
            | ExactBooleanSupport::CertifiedConvexUnion
            | ExactBooleanSupport::CertifiedConvexIntersection
            | ExactBooleanSupport::CertifiedConvexDifference => {
                if self.blocker.is_some() {
                    return Err(ExactReportValidationError::CertifiedReportHasBlocker);
                }
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if operation_is_selected_region(self.operation)
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
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
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if operation_is_selected_region(self.operation)
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
                if operation_is_selected_region(self.operation)
                    || self.graph_had_unknowns
                    || self.blocker.is_some()
                    || self.retained_face_pairs == 0
                    || self.arrangement_readiness.is_some()
                    || !certified_preflight_support_matches_operation(self.support, self.operation)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::CertifiedArrangementCellComplex => {
                if operation_is_selected_region(self.operation)
                    || self.graph_had_unknowns
                    || self.blocker.is_some()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if let Some(evidence) = self.coplanar_volumetric_evidence.as_ref() {
                    validate_certified_arrangement_coplanar_evidence_shape(
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
                if operation_is_selected_region(self.operation)
                    || self.operation != expected_operation
                    || self.graph_had_unknowns
                    || self.blocker.is_some()
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresBoundaryPolicy => {
                if operation_is_selected_region(self.operation) || self.graph_had_unknowns {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::NeedsBoundaryPolicy,
                )?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsBoundaryPolicy)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresPlanarArrangement => {
                if operation_is_selected_region(self.operation)
                    || matches!(self.operation, ExactBooleanOperation::Intersection)
                    || self.graph_had_unknowns
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::NeedsPlanarArrangement,
                )?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsPlanarArrangement)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let readiness = self
                    .arrangement_readiness
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingArrangementReadiness)?;
                readiness
                    .validate()
                    .map_err(|_| ExactReportValidationError::InvalidArrangementReadiness)?;
                validate_arrangement_readiness_matches_blocker(
                    readiness,
                    self.blocker.as_ref().unwrap(),
                )?;
                if !readiness.needs_planar_cells()
                    || self.blocker.as_ref().unwrap().coplanar_touching_pairs != 0
                {
                    return Err(ExactReportValidationError::ArrangementReadinessMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresCoplanarVolumetricCells => {
                if operation_is_selected_region(self.operation) || self.graph_had_unknowns {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells,
                )?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                let evidence = self
                    .coplanar_volumetric_evidence
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingCoplanarVolumetricEvidence)?;
                validate_coplanar_volumetric_evidence_matches_blocker(
                    evidence,
                    self.blocker.as_ref().unwrap(),
                )?;
                if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                    return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresCertifiedWinding => {
                if operation_is_selected_region(self.operation)
                    || self.graph_had_unknowns
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(self.blocker.as_ref(), ExactBooleanBlockerKind::NeedsWinding)?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsWinding)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
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
                blocker_kind(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::NeedsRefinement,
                )?;
                self.blocker
                    .as_ref()
                    .unwrap()
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsRefinement)?;
                validate_blocker_count_bounds(
                    self.blocker.as_ref().unwrap(),
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::SelectedRegionPolicy => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if !operation_is_selected_region(self.operation)
                    || self.graph_had_unknowns
                    || self.blocker.is_some()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                if self.retained_face_pairs == 0 {
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
    /// Number of retained segment/plane events whose endpoint predicates
    /// certified a crossing but whose exact construction failed.
    pub construction_failed_events: usize,
}

impl ExactBooleanBlocker {
    /// Validate that this blocker kind is justified by retained graph relation
    /// counts.
    ///
    /// The counts are exact graph evidence, not decoration. A blocker that
    /// says "needs planar arrangement" while retaining unknown or non-coplanar
    /// candidate pairs would collapse distinct exact computation states into
    /// states to stay explicit.
    pub fn validate_for_kind(
        &self,
        expected: ExactBooleanBlockerKind,
    ) -> Result<(), ExactReportValidationError> {
        if self.kind != expected {
            return Err(ExactReportValidationError::WrongBlockerKind);
        }
        let valid = match expected {
            ExactBooleanBlockerKind::NeedsRefinement => {
                self.unknown_pairs > 0 || self.construction_failed_events > 0
            }
            ExactBooleanBlockerKind::NeedsBoundaryPolicy => {
                self.candidate_pairs
                    + self.coplanar_touching_pairs
                    + self.coplanar_overlapping_pairs
                    > 0
                    && self.unknown_pairs == 0
                    && self.construction_failed_events == 0
            }
            ExactBooleanBlockerKind::NeedsPlanarArrangement => {
                self.coplanar_overlapping_pairs > 0
                    && self.unknown_pairs == 0
                    && self.construction_failed_events == 0
                    && self.candidate_pairs == 0
            }
            ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells => {
                self.coplanar_touching_pairs + self.coplanar_overlapping_pairs > 0
                    && self.unknown_pairs == 0
                    && self.construction_failed_events == 0
            }
            ExactBooleanBlockerKind::NeedsWinding => {
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

    /// Validate this blocker against source meshes that produced its graph counts.
    ///
    /// [`Self::validate_for_kind`] checks whether the stored counters justify a
    /// requested blocker class. Source replay rebuilds the exact intersection
    /// graph from `left` and `right`, recomputes those counters, and requires
    /// this public blocker to match the replay for its retained kind. This is
    /// and construction evidence that blocked the named boolean.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate_for_kind(self.kind)?;
        let graph = build_intersection_graph(left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        graph
            .validate_against_sources(left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        let replay = blocker_source_counts(&graph).into_blocker(self.kind);
        if replay.validate_for_kind(self.kind).is_ok() && self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained blocker is fresh for the source meshes.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate_for_kind(self.kind) {
            return error.into();
        }
        let Ok(graph) = build_intersection_graph(left, right) else {
            return ExactReportFreshness::SourceReplayMismatch;
        };
        if graph.validate_against_sources(left, right).is_err() {
            return ExactReportFreshness::SourceReplayMismatch;
        }
        let replay = blocker_source_counts(&graph).into_blocker(self.kind);
        if replay.validate_for_kind(self.kind).is_ok() && self == &replay {
            ExactReportFreshness::Current
        } else {
            ExactReportFreshness::SourceReplayMismatch
        }
    }
}

/// Exact boolean preflight blocker kind.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactBooleanBlockerKind {
    /// Predicate or equality refinement is required before policy can run.
    NeedsRefinement,
    /// A lower-dimensional shared-boundary output policy is required.
    NeedsBoundaryPolicy,
    /// A planar arrangement output model is required for coplanar surfaces.
    NeedsPlanarArrangement,
    /// Coplanar source-face cells must be materialized before closed
    /// volumetric winding can decide named output.
    NeedsCoplanarVolumetricCells,
    /// Full winding/inside-outside classification is required.
    NeedsWinding,
}

/// Certification status for exact refinement preflight.
///
/// Refinement is the stage before application-level topology policy: exact
/// graph extraction retained an unknown predicate outcome or a construction
/// whose endpoint predicates certified an event but whose exact point/parameter
/// from winding or planar-arrangement policy, so it has a separate report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactRefinementStatus {
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
pub struct ExactRefinementReport {
    /// Named operation whose graph was inspected.
    pub operation: ExactBooleanOperation,
    /// Coarse refinement status.
    pub status: ExactRefinementStatus,
    /// Whether graph extraction retained unknown predicate outcomes.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records.
    pub retained_events: usize,
    /// Refinement blocker counts, present only when refinement is required.
    pub blocker: Option<ExactBooleanBlocker>,
}

impl ExactRefinementReport {
    /// Return whether exact predicate/construction refinement is required.
    pub const fn is_required(&self) -> bool {
        matches!(self.status, ExactRefinementStatus::Required)
    }

    /// Validate this refinement report against the source meshes.
    ///
    /// The local audit checks status/blocker/count coherence. This replay
    /// recomputes the retained graph report from `left` and `right` for the
    /// same operation and requires equality, keeping refinement evidence tied
    /// to the source objects whose exact predicates produced it as required by
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay = certify_refinement_report(left, right, self.operation)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained refinement report is fresh.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match certify_refinement_report(left, right, self.operation) {
            Ok(replay) if self == &replay => ExactReportFreshness::Current,
            Ok(_) | Err(_) => ExactReportFreshness::SourceReplayMismatch,
        }
    }

    /// Validate status, retained counts, and refinement blocker consistency.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if (self.retained_face_pairs == 0 && self.retained_events != 0)
            || (self.retained_face_pairs != 0 && self.retained_events == 0)
        {
            return Err(ExactReportValidationError::InvalidBlockerCounts);
        }
        match self.status {
            ExactRefinementStatus::Required => {
                blocker_kind(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::NeedsRefinement,
                )?;
                let blocker = self.blocker.as_ref().unwrap();
                blocker.validate_for_kind(ExactBooleanBlockerKind::NeedsRefinement)?;
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
/// predicate trail rather than collapsing directly to `bool`.
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

    /// Validate same-surface report invariants.
    ///
    /// Rejection statuses are still evidence states: count mismatches must not
    /// retain coordinate predicates, vertex-matching failures may keep only the
    /// partial left-to-right matches and predicate trail, and triangle-set
    /// mismatches must retain a valid full vertex permutation. This keeps a
    /// allowing callers to attach arbitrary topology artifacts to a rejection.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
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
                if !self.right_to_left.is_empty()
                    || !self.left_triangles.is_empty()
                    || !self.right_triangles.is_empty()
                    || self.predicates.is_empty()
                    || !is_partial_injective_mapping(&self.left_to_right)
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

    /// Validate this report against the source meshes that produced it.
    ///
    /// [`Self::validate`] checks that the retained permutation, remapped
    /// triangle sets, and predicate-use trail are locally coherent. This
    /// stronger check recomputes the same-surface certificate from `left` and
    /// a shortcut certificate is retained numerical and combinatorial state
    /// attached to particular source objects, not a portable label that can be
    /// pasted onto another mesh pair.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        if self == &certify_same_surface_report(left, right) {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained same-surface report is fresh.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        if self == &certify_same_surface_report(left, right) {
            ExactReportFreshness::Current
        } else {
            ExactReportFreshness::SourceReplayMismatch
        }
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

fn is_partial_injective_mapping(mapping: &[usize]) -> bool {
    let mut seen = Vec::with_capacity(mapping.len());
    for &right in mapping {
        if seen.contains(&right) {
            return false;
        }
        seen.push(right);
    }
    true
}

/// Certification status for an open-surface disjoint shortcut.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactOpenSurfaceDisjointStatus {
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
pub struct ExactOpenSurfaceDisjointReport {
    /// Coarse certification status.
    pub status: ExactOpenSurfaceDisjointStatus,
    /// Whether the left mesh satisfies the exact open-surface precondition.
    pub left_open_surface: bool,
    /// Whether the right mesh satisfies the exact open-surface precondition.
    pub right_open_surface: bool,
    /// Whether graph extraction retained unknown events.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records.
    pub retained_events: usize,
    /// Relation counts for retained face pairs.
    pub blocker: ExactBooleanBlocker,
}

impl ExactOpenSurfaceDisjointReport {
    /// Return whether open-surface disjointness was certified.
    pub const fn is_certified(&self) -> bool {
        matches!(self.status, ExactOpenSurfaceDisjointStatus::Certified)
    }

    /// Validate this open-surface report against the source meshes.
    ///
    /// Open-surface disjointness is certified graph absence plus mesh-shape
    /// preconditions. This method recomputes both from `left` and `right`
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay = certify_open_surface_disjoint_report(left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained open-surface disjoint report is fresh.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match certify_open_surface_disjoint_report(left, right) {
            Ok(replay) if self == &replay => ExactReportFreshness::Current,
            Ok(_) | Err(_) => ExactReportFreshness::SourceReplayMismatch,
        }
    }

    /// Validate status, graph counts, and blocker consistency.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if matches!(self.status, ExactOpenSurfaceDisjointStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        // Graph unknowns are refinement state, not open-surface topology
        // later policy stage must not consume an unresolved predicate as if it
        // were certified no-intersection or winding evidence.
        let expected_kind = if matches!(self.status, ExactOpenSurfaceDisjointStatus::GraphUnknowns)
        {
            ExactBooleanBlockerKind::NeedsRefinement
        } else {
            ExactBooleanBlockerKind::NeedsWinding
        };
        if self.blocker.kind != expected_kind {
            return Err(ExactReportValidationError::WrongBlockerKind);
        }
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
        // refinement, topology policy, and certified shortcuts are not
        // accidentally conflated.
        match self.status {
            ExactOpenSurfaceDisjointStatus::NotOpenSurface => {
                if (self.left_open_surface && self.right_open_surface)
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                    || blocker_has_any_evidence(&self.blocker)
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactOpenSurfaceDisjointStatus::GraphUnknowns => {
                if !self.left_open_surface || !self.right_open_surface {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactOpenSurfaceDisjointStatus::GraphHasFacePairs => {
                if !self.left_open_surface || !self.right_open_surface {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactOpenSurfaceDisjointStatus::Certified => {
                if !self.left_open_surface || !self.right_open_surface {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
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
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactBoundaryTouchingStatus {
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
pub struct ExactBoundaryTouchingReport {
    /// Coarse boundary-touching certification status.
    pub status: ExactBoundaryTouchingStatus,
    /// Whether graph extraction retained unknown events.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records.
    pub retained_events: usize,
    /// Relation counts for retained face pairs.
    pub blocker: ExactBooleanBlocker,
}

/// Certification status for closed-solid adjacent union completion.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactAdjacentUnionCompletionStatus {
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
pub struct ExactAdjacentUnionCompletionReport {
    /// Requested named operation.
    pub operation: ExactBooleanOperation,
    /// Coarse certification status.
    pub status: ExactAdjacentUnionCompletionStatus,
    /// Whether the left source mesh was a closed manifold.
    pub left_closed: bool,
    /// Whether the right source mesh was a closed manifold.
    pub right_closed: bool,
    /// Whether the stronger axis-aligned box path owns this pair.
    pub axis_aligned_box_pair: bool,
    /// Whether another exact kernel should materialize this union first.
    pub stronger_kernel_available: bool,
    /// Whether graph extraction retained unknown events.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records.
    pub retained_events: usize,
    /// Relation counts for retained face pairs.
    pub blocker: ExactBooleanBlocker,
    /// Count of exact whole-face pairs consumed by full-face completion.
    pub full_face_shared_faces: usize,
    /// Count of exact source-owned full patches consumed by full-face
    /// completion.
    pub full_face_shared_patches: usize,
    /// Source side whose faces contain the opposite caps for contained-face
    /// completion.
    pub contained_containing_side: Option<MeshSide>,
    /// Count of opposite-source faces removed by contained-face completion.
    pub contained_faces: usize,
    /// Count of source faces replaced by holed remnants in contained-face
    /// completion.
    pub containing_faces: usize,
}

impl ExactAdjacentUnionCompletionReport {
    /// Return whether adjacent union completion was certified.
    pub const fn is_certified(&self) -> bool {
        matches!(
            self.status,
            ExactAdjacentUnionCompletionStatus::CertifiedFullFace
                | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
        )
    }

    /// Validate status, graph counts, and consumed topology counts.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
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
        self.blocker.validate_for_kind(self.blocker.kind)?;
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

        let full_face_counts = self.full_face_shared_faces + self.full_face_shared_patches;
        let contained_counts = self.contained_faces + self.containing_faces;
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
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsBoundaryPolicy)?;
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
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsBoundaryPolicy)?;
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
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay = certify_adjacent_union_completion_report(left, right, self.operation)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained report is fresh for the source operands.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match certify_adjacent_union_completion_report(left, right, self.operation) {
            Ok(replay) if self == &replay => ExactReportFreshness::Current,
            Ok(_) | Err(_) => ExactReportFreshness::SourceReplayMismatch,
        }
    }
}

impl ExactBoundaryTouchingReport {
    /// Return whether the graph is certified boundary-only contact.
    pub const fn is_certified(&self) -> bool {
        matches!(self.status, ExactBoundaryTouchingStatus::Certified)
    }

    /// Validate status, retained relation counts, and blocker consistency.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if matches!(self.status, ExactBoundaryTouchingStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        // A boundary-only policy is meaningful only after the graph is
        // resolved. Unknown graph events remain refinement blockers, preserving
        // application-level topology policy. Positive-area coplanar overlaps
        // can still be boundary-only for closed solids, but that certification
        // is source-replayed by the report constructor; local validation only
        // audits the retained relation-count shape.
        let expected_kind = if matches!(self.status, ExactBoundaryTouchingStatus::GraphUnknowns) {
            ExactBooleanBlockerKind::NeedsRefinement
        } else {
            ExactBooleanBlockerKind::NeedsBoundaryPolicy
        };
        if self.blocker.kind != expected_kind {
            return Err(ExactReportValidationError::WrongBlockerKind);
        }
        validate_refinement_partition(
            matches!(self.status, ExactBoundaryTouchingStatus::GraphUnknowns),
            &self.blocker,
        )?;
        validate_blocker_count_bounds(
            &self.blocker,
            self.retained_face_pairs,
            self.retained_events,
        )?;
        // Boundary-only contact is an application policy boundary. Keep its
        // evidence separated from graph refinement and non-boundary winding
        match self.status {
            ExactBoundaryTouchingStatus::GraphUnknowns => {}
            ExactBoundaryTouchingStatus::NotBoundaryOnly => {
                if self.retained_face_pairs != 0
                    && self.blocker.candidate_pairs == 0
                    && self.blocker.coplanar_overlapping_pairs == 0
                    && self
                        .blocker
                        .validate_for_kind(ExactBooleanBlockerKind::NeedsBoundaryPolicy)
                        .is_ok()
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
            ExactBoundaryTouchingStatus::Certified => {}
        }
        if self.is_certified()
            && self.blocker.candidate_pairs
                + self.blocker.coplanar_touching_pairs
                + self.blocker.coplanar_overlapping_pairs
                == 0
        {
            return Err(ExactReportValidationError::MissingRelationCount);
        }
        if self.is_certified() {
            self.blocker
                .validate_for_kind(ExactBooleanBlockerKind::NeedsBoundaryPolicy)?;
        }
        Ok(())
    }

    /// Validate this boundary-touching report against the source meshes.
    ///
    /// Boundary-only contact is a policy boundary over a resolved exact graph.
    /// Recomputing the report from the source meshes ensures the retained
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay = certify_boundary_touching_report(left, right)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained boundary-touching report is fresh.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match certify_boundary_touching_report(left, right) {
            Ok(replay) if self == &replay => ExactReportFreshness::Current,
            Ok(_) | Err(_) => ExactReportFreshness::SourceReplayMismatch,
        }
    }
}

/// Certification status for planar-arrangement blockers.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactPlanarArrangementStatus {
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
pub struct ExactPlanarArrangementReport {
    /// Requested named operation.
    pub operation: ExactBooleanOperation,
    /// Coarse planar-arrangement certification status.
    pub status: ExactPlanarArrangementStatus,
    /// Whether graph extraction retained unknown events.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records.
    pub retained_events: usize,
    /// Relation counts for retained face pairs.
    pub blocker: ExactBooleanBlocker,
    /// Checked coplanar-overlap readiness summary retained from the graph
    /// layer.
    pub arrangement_readiness: Option<CoplanarArrangementReadinessReport>,
}

impl ExactPlanarArrangementReport {
    /// Return whether this operation is blocked on planar arrangement output.
    pub const fn is_required(&self) -> bool {
        matches!(self.status, ExactPlanarArrangementStatus::Required)
    }

    /// Validate status, retained relation counts, and blocker consistency.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if matches!(self.status, ExactPlanarArrangementStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        // A graph-unknown arrangement report has not reached planar-cell
        // policy. It is still blocked on predicate/construction refinement, a
        let expected_kind = match self.status {
            ExactPlanarArrangementStatus::GraphUnknowns => ExactBooleanBlockerKind::NeedsRefinement,
            ExactPlanarArrangementStatus::BoundaryPolicyRequired => {
                ExactBooleanBlockerKind::NeedsBoundaryPolicy
            }
            _ => ExactBooleanBlockerKind::NeedsPlanarArrangement,
        };
        if self.blocker.kind != expected_kind {
            return Err(ExactReportValidationError::WrongBlockerKind);
        }
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
            }
            ExactPlanarArrangementStatus::Required => {
                if matches!(
                    self.operation,
                    ExactBooleanOperation::SelectedRegions(_) | ExactBooleanOperation::Intersection
                ) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
            }
        }
        if self.is_required() && self.blocker.coplanar_overlapping_pairs == 0 {
            return Err(ExactReportValidationError::MissingRelationCount);
        }
        match self.status {
            ExactPlanarArrangementStatus::Required => {
                let readiness = self
                    .arrangement_readiness
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingArrangementReadiness)?;
                readiness
                    .validate()
                    .map_err(|_| ExactReportValidationError::InvalidArrangementReadiness)?;
                validate_arrangement_readiness_matches_blocker(readiness, &self.blocker)?;
                if !readiness.needs_planar_cells()
                    || self.blocker.coplanar_touching_pairs != 0
                    || readiness.graph_count
                        != self.blocker.coplanar_overlapping_pairs
                            + self.blocker.coplanar_touching_pairs
                {
                    return Err(ExactReportValidationError::ArrangementReadinessMismatch);
                }
            }
            ExactPlanarArrangementStatus::AlreadyMaterialized
            | ExactPlanarArrangementStatus::NoPositiveOverlap
            | ExactPlanarArrangementStatus::BoundaryPolicyRequired => {
                if let Some(readiness) = &self.arrangement_readiness {
                    readiness
                        .validate()
                        .map_err(|_| ExactReportValidationError::InvalidArrangementReadiness)?;
                    validate_arrangement_readiness_matches_blocker(readiness, &self.blocker)?;
                    if readiness.status == CoplanarArrangementReadinessStatus::NoCoplanarOverlap
                        && self.blocker.coplanar_overlapping_pairs
                            + self.blocker.coplanar_touching_pairs
                            != 0
                    {
                        return Err(ExactReportValidationError::ArrangementReadinessMismatch);
                    }
                }
            }
            ExactPlanarArrangementStatus::NotNamedOperation
            | ExactPlanarArrangementStatus::GraphUnknowns => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
            }
        }
        if self.is_required() {
            self.blocker
                .validate_for_kind(ExactBooleanBlockerKind::NeedsPlanarArrangement)?;
        } else if matches!(
            self.status,
            ExactPlanarArrangementStatus::BoundaryPolicyRequired
        ) {
            self.blocker
                .validate_for_kind(ExactBooleanBlockerKind::NeedsBoundaryPolicy)?;
        }
        Ok(())
    }

    /// Validate this planar-arrangement report against the source meshes.
    ///
    /// The retained arrangement-readiness summary is a compact view of exact
    /// coplanar graph state. This source replay recomputes that view for the
    /// same operation and rejects stale count/blocker summaries before a
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay = certify_planar_arrangement_report(left, right, self.operation)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained blocker report is fresh for `left` and `right`.
    ///
    /// The method first checks local report integrity and then recomputes the
    /// and source-drift are distinct facts a scheduler can react to.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match certify_planar_arrangement_report(left, right, self.operation) {
            Ok(replay) if self == &replay => ExactReportFreshness::Current,
            Ok(_) | Err(_) => ExactReportFreshness::SourceReplayMismatch,
        }
    }
}

/// Certification status for the remaining exact winding handoff.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactWindingReadinessStatus {
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
    /// planar-arrangement shortcut, so no volumetric winding handoff is needed.
    PlanarArrangementAlreadyMaterialized,
    /// Coplanar source-face cells are part of a closed-volumetric overlap and
    /// must be materialized before winding can consume the split cells.
    CoplanarVolumetricCellsRequired,
    /// Coplanar source-face cells were required, but the certified
    /// arrangement/cell-complex path has already materialized them, so no
    /// unresolved winding blocker remains at this handoff.
    CoplanarVolumetricCellsAlreadyMaterialized,
    /// Exact volumetric winding classifications are decided, but the retained
    /// split cells could not yet be assembled into certified output topology.
    VolumetricAssemblyRequired,
    /// A certified arrangement/cell-complex shortcut has already materialized
    /// this named Boolean, so no unresolved winding blocker remains at this
    /// handoff.
    ArrangementCellComplexAlreadyMaterialized,
    /// The named Boolean was already answered by regularized-solid semantics
    /// for one closed solid and one lower-dimensional open surface, so no
    /// winding handoff is needed.
    MixedDimensionalRegularizedSolidAlreadyMaterialized,
    /// The named Boolean was already answered by closed-convex exact
    /// materialization, so no winding handoff is needed.
    ConvexBooleanAlreadyMaterialized,
    /// The named Boolean was already answered by exact open-surface
    /// split-region arrangement, so no volumetric winding handoff is needed.
    OpenSurfaceArrangementAlreadyMaterialized,
    /// The named Boolean was already answered by exact surface identity or
    /// same-surface equality, so no winding handoff is needed.
    SurfaceEqualityAlreadyMaterialized,
    /// The named Boolean was already answered by certified closed-boundary
    /// touching regularized semantics, so no winding handoff is needed.
    ClosedBoundaryTouchingAlreadyMaterialized,
    /// The named Boolean was already answered by exact empty-operand
    /// semantics, so no winding handoff is needed.
    EmptyOperandAlreadyMaterialized,
    /// The named Boolean was already answered by certified disjoint mesh
    /// bounds, so no winding handoff is needed.
    BoundsDisjointAlreadyMaterialized,
    /// The named Boolean was already answered by certified open-surface graph
    /// disjointness, so no winding handoff is needed.
    OpenSurfaceDisjointAlreadyMaterialized,
    /// The named Boolean was already answered by an empty exact intersection
    /// graph and replayable closed-mesh winding reports proving separation.
    ClosedWindingSeparatedAlreadyMaterialized,
    /// The named Boolean was already answered by an empty exact intersection
    /// graph and replayable closed-mesh winding reports proving containment.
    ClosedWindingContainmentAlreadyMaterialized,
    /// The graph contains no retained face pairs requiring winding.
    NoNontrivialOverlap,
    /// Split regions and opposite-plane classifications were checked and are
    /// ready for the future exact winding/inside-outside policy.
    Ready,
}

/// Auditable report for the nontrivial overlap winding handoff.
///
/// This report is the certified boundary immediately before full named
/// union/intersection/difference winding semantics. It retains exact graph
/// counts and checked split-region plane classifications, but deliberately
/// topological policy remains explicit state instead of a hidden tolerance
/// decision.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactWindingReadinessReport {
    /// Requested named operation.
    pub operation: ExactBooleanOperation,
    /// Coarse readiness status.
    pub status: ExactWindingReadinessStatus,
    /// Whether graph extraction retained unknown events.
    pub graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub retained_face_pairs: usize,
    /// Total retained event records.
    pub retained_events: usize,
    /// Number of checked split regions prepared for winding.
    pub region_count: usize,
    /// Certified region-vs-opposite-plane classifications.
    pub region_classifications: Vec<FaceRegionPlaneClassification>,
    /// Relation counts for the blocker represented by this report.
    pub blocker: ExactBooleanBlocker,
    /// Checked coplanar-overlap readiness retained when winding is blocked by
    /// planar-cell extraction rather than by volumetric inside/outside policy.
    pub arrangement_readiness: Option<CoplanarArrangementReadinessReport>,
    /// Source-aware coplanar volumetric-cell evidence retained when readiness
    /// is blocked by, or has just consumed, coplanar source-face cells.
    ///
    /// The winding handoff must not reduce this state to raw coplanar pair
    /// counts: exact side evidence is what distinguishes boundary-only contact
    /// from a real volumetric-cell topology obligation.
    pub coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
}

impl ExactWindingReadinessReport {
    /// Return whether the report reached the winding-ready handoff.
    pub const fn is_ready(&self) -> bool {
        matches!(self.status, ExactWindingReadinessStatus::Ready)
    }

    /// Validate this winding-readiness report against the source meshes.
    ///
    /// Winding readiness retains exact split-region and opposite-plane facts
    /// without choosing the final inside/outside policy. This replay
    /// recomputes the whole public report for the same operation, making stale
    /// region facts and blocker summaries fail before downstream topology
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), ExactReportValidationError> {
        self.validate()?;
        let replay = certify_winding_readiness_report(left, right, self.operation)
            .map_err(|_| ExactReportValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactReportValidationError::SourceReplayMismatch)
        }
    }

    /// Classify whether this retained winding handoff is fresh for the source meshes.
    ///
    /// Local integrity is checked before source replay so copied reports can
    /// distinguish stale region classifications from source-geometry drift.
    /// summaries must replay before later winding policy consumes them.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> ExactReportFreshness {
        if let Err(error) = self.validate() {
            return error.into();
        }
        match certify_winding_readiness_report(left, right, self.operation) {
            Ok(replay) if self == &replay => ExactReportFreshness::Current,
            Ok(_) | Err(_) => ExactReportFreshness::SourceReplayMismatch,
        }
    }

    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.region_classifications
            .iter()
            .all(FaceRegionPlaneClassification::all_proof_producing)
    }

    /// Validate status, blocker, and checked-region artifact consistency.
    pub fn validate(&self) -> Result<(), ExactReportValidationError> {
        if matches!(self.status, ExactWindingReadinessStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactReportValidationError::GraphUnknownStatusMismatch);
        }
        validate_refinement_partition(
            matches!(self.status, ExactWindingReadinessStatus::GraphUnknowns),
            &self.blocker,
        )?;
        if self.coplanar_volumetric_evidence.is_some()
            && !matches!(
                self.status,
                ExactWindingReadinessStatus::Ready
                    | ExactWindingReadinessStatus::VolumetricAssemblyRequired
                    | ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized
                    | ExactWindingReadinessStatus::CoplanarVolumetricCellsAlreadyMaterialized
                    | ExactWindingReadinessStatus::CoplanarVolumetricCellsRequired
                    | ExactWindingReadinessStatus::ClosedBoundaryTouchingAlreadyMaterialized
            )
        {
            return Err(ExactReportValidationError::UnexpectedCoplanarVolumetricEvidence);
        }
        match self.status {
            ExactWindingReadinessStatus::GraphUnknowns => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::NeedsRefinement,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsRefinement)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::BoundaryPolicyRequired => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::NeedsBoundaryPolicy,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsBoundaryPolicy)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::PlanarArrangementRequired => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::NeedsPlanarArrangement,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsPlanarArrangement)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let readiness = self
                    .arrangement_readiness
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingArrangementReadiness)?;
                readiness
                    .validate()
                    .map_err(|_| ExactReportValidationError::InvalidArrangementReadiness)?;
                validate_arrangement_readiness_matches_blocker(readiness, &self.blocker)?;
                if !readiness.needs_planar_cells() || self.blocker.coplanar_touching_pairs != 0 {
                    return Err(ExactReportValidationError::ArrangementReadinessMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::NeedsPlanarArrangement,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsPlanarArrangement)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if let Some(readiness) = &self.arrangement_readiness {
                    readiness
                        .validate()
                        .map_err(|_| ExactReportValidationError::InvalidArrangementReadiness)?;
                    validate_arrangement_readiness_matches_blocker(readiness, &self.blocker)?;
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::CoplanarVolumetricCellsRequired => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let evidence = self
                    .coplanar_volumetric_evidence
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingCoplanarVolumetricEvidence)?;
                validate_coplanar_volumetric_evidence_matches_blocker(evidence, &self.blocker)?;
                if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                    return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::CoplanarVolumetricCellsAlreadyMaterialized => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let evidence = self
                    .coplanar_volumetric_evidence
                    .as_ref()
                    .ok_or(ExactReportValidationError::MissingCoplanarVolumetricEvidence)?;
                validate_coplanar_volumetric_evidence_matches_blocker(evidence, &self.blocker)?;
                if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                    return Err(ExactReportValidationError::CoplanarVolumetricEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::VolumetricAssemblyRequired => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                let expected = match self.blocker.kind {
                    ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells => {
                        ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells
                    }
                    _ => ExactBooleanBlockerKind::NeedsWinding,
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
                    (ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_matches_blocker(
                            evidence,
                            &self.blocker,
                        )?;
                        if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                            return Err(
                                ExactReportValidationError::CoplanarVolumetricEvidenceMismatch,
                            );
                        }
                    }
                    (ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells, None) => {
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
            ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                let expected = match self.blocker.kind {
                    ExactBooleanBlockerKind::NeedsBoundaryPolicy => {
                        ExactBooleanBlockerKind::NeedsBoundaryPolicy
                    }
                    ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells => {
                        ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells
                    }
                    _ => ExactBooleanBlockerKind::NeedsWinding,
                };
                blocker_kind(Some(&self.blocker), expected)?;
                self.blocker.validate_for_kind(expected)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                match (expected, self.coplanar_volumetric_evidence.as_ref()) {
                    (ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_matches_blocker(
                            evidence,
                            &self.blocker,
                        )?;
                        if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                            return Err(
                                ExactReportValidationError::CoplanarVolumetricEvidenceMismatch,
                            );
                        }
                    }
                    (ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells, None) => {
                        return Err(ExactReportValidationError::MissingCoplanarVolumetricEvidence);
                    }
                    (_, Some(_)) => {
                        return Err(
                            ExactReportValidationError::UnexpectedCoplanarVolumetricEvidence,
                        );
                    }
                    (_, None) => {}
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized => {
                if self.arrangement_readiness.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::NeedsWinding)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsWinding)?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::ConvexBooleanAlreadyMaterialized => {
                if self.arrangement_readiness.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::NeedsWinding)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsWinding)?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::OpenSurfaceArrangementAlreadyMaterialized => {
                if self.arrangement_readiness.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::NeedsWinding)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsWinding)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::SurfaceEqualityAlreadyMaterialized => {
                if self.arrangement_readiness.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::NeedsWinding)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsWinding)?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::ClosedBoundaryTouchingAlreadyMaterialized => {
                if self.arrangement_readiness.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::NeedsBoundaryPolicy,
                )?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsBoundaryPolicy)?;
                validate_blocker_count_bounds(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if let Some(evidence) = self.coplanar_volumetric_evidence.as_ref() {
                    validate_coplanar_boundary_only_evidence_shape(
                        evidence,
                        self.retained_face_pairs,
                        self.retained_events,
                    )?;
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::EmptyOperandAlreadyMaterialized
            | ExactWindingReadinessStatus::BoundsDisjointAlreadyMaterialized
            | ExactWindingReadinessStatus::OpenSurfaceDisjointAlreadyMaterialized
            | ExactWindingReadinessStatus::ClosedWindingSeparatedAlreadyMaterialized
            | ExactWindingReadinessStatus::ClosedWindingContainmentAlreadyMaterialized => {
                if self.arrangement_readiness.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::NeedsWinding)?;
                self.blocker
                    .validate_for_kind(ExactBooleanBlockerKind::NeedsWinding)?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingReadinessStatus::Ready => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactReportValidationError::StatusEvidenceMismatch);
                }
                let expected = match self.blocker.kind {
                    ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells => {
                        ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells
                    }
                    _ => ExactBooleanBlockerKind::NeedsWinding,
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
                    (ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_matches_blocker(
                            evidence,
                            &self.blocker,
                        )?;
                        if !evidence.obstacle.requires_coplanar_volumetric_cells() {
                            return Err(
                                ExactReportValidationError::CoplanarVolumetricEvidenceMismatch,
                            );
                        }
                    }
                    (ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells, None) => {
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
            ExactWindingReadinessStatus::NotNamedOperation
            | ExactWindingReadinessStatus::NoNontrivialOverlap => {
                if self.arrangement_readiness.is_some() {
                    return Err(ExactReportValidationError::UnexpectedArrangementReadiness);
                }
                match self.status {
                    ExactWindingReadinessStatus::NotNamedOperation
                        if !matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) =>
                    {
                        return Err(ExactReportValidationError::StatusEvidenceMismatch);
                    }
                    ExactWindingReadinessStatus::NoNontrivialOverlap
                        if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                            || self.retained_face_pairs != 0 =>
                    {
                        return Err(ExactReportValidationError::StatusEvidenceMismatch);
                    }
                    _ => {}
                }
                blocker_kind(Some(&self.blocker), ExactBooleanBlockerKind::NeedsWinding)?;
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

    #[test]
    fn freshness_classifies_retained_region_provenance_drift() {
        let stale_region_errors = [
            ExactReportValidationError::DuplicateRegionTriangulation,
            ExactReportValidationError::InvalidTriangulation,
            ExactReportValidationError::UntriangulatedAssemblyRegion,
            ExactReportValidationError::AssemblyVertexOutsideTriangulation,
            ExactReportValidationError::UnreferencedAssemblyVertex,
            ExactReportValidationError::InvalidAssembly,
            ExactReportValidationError::OutputMeshAssemblyMismatch,
            ExactReportValidationError::InvalidRegionClassification(
                FaceRegionPlaneValidationError::EmptyNodeSides,
            ),
            ExactReportValidationError::InvalidVolumetricClassification(
                ExactVolumetricRegionError::EmptyTriangulation,
            ),
            ExactReportValidationError::MissingVolumetricClassifications,
            ExactReportValidationError::UnexpectedVolumetricClassifications,
            ExactReportValidationError::OrphanedVolumetricClassification,
            ExactReportValidationError::UnclassifiedVolumetricTriangulation,
            ExactReportValidationError::VolumetricClassificationNotDecided,
            ExactReportValidationError::InvalidOutputMesh,
            ExactReportValidationError::ShortcutResultHasAssemblyArtifacts,
        ];
        for error in stale_region_errors {
            assert_eq!(
                ExactReportFreshness::from(error),
                ExactReportFreshness::StaleRegionFacts
            );
        }

        assert_eq!(
            ExactReportFreshness::from(ExactReportValidationError::OutputSourceReplayMismatch),
            ExactReportFreshness::SourceReplayMismatch
        );
        for error in [
            ExactReportValidationError::ShortcutResultHasUnknownGraph,
            ExactReportValidationError::SelectedRegionResultHasUnknownGraph,
            ExactReportValidationError::UnexpectedGraphEvents,
        ] {
            assert_eq!(
                ExactReportFreshness::from(error),
                ExactReportFreshness::StaleGraphUnknownStatus
            );
        }
        for error in [
            ExactReportValidationError::CertifiedReportHasBlocker,
            ExactReportValidationError::MissingBlocker,
        ] {
            assert_eq!(
                ExactReportFreshness::from(error),
                ExactReportFreshness::StaleBlockerEvidence
            );
        }
        for error in [
            ExactReportValidationError::InvalidPermutation,
            ExactReportValidationError::MismatchedTriangleSets,
        ] {
            assert_eq!(
                ExactReportFreshness::from(error),
                ExactReportFreshness::StaleStatusEvidence
            );
        }
        assert_eq!(
            ExactReportFreshness::from(
                ExactReportValidationError::SelectedRegionAssemblyViolatesSelection
            ),
            ExactReportFreshness::StaleStatusEvidence
        );
        assert_eq!(
            ExactReportFreshness::from(
                ExactReportValidationError::VolumetricMaterializedAssemblyViolatesOperation
            ),
            ExactReportFreshness::StaleStatusEvidence
        );
    }

    fn report_test_tetra(offset: [i64; 3]) -> ExactMesh {
        let [ox, oy, oz] = offset;
        ExactMesh::from_i64_triangles(
            &[ox, oy, oz, ox + 1, oy, oz, ox, oy + 1, oz, ox, oy, oz + 1],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
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
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
    }

    #[test]
    fn preflight_and_closure_freshness_classify_local_and_source_drift() {
        let left = report_test_tetra([0, 0, 0]);
        let right = report_test_tetra([3, 0, 0]);

        let preflight =
            preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union).unwrap();
        assert_eq!(
            preflight.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            preflight.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::CLOSED
            ),
            ExactReportFreshness::Current
        );

        let mut stale_preflight = preflight.clone();
        stale_preflight.retained_events = 1;
        assert_eq!(
            stale_preflight.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleStatusEvidence
        );

        let overlapping_right = report_test_tetra([0, 0, 0]);
        assert_eq!(
            preflight.freshness_against_sources(&left, &overlapping_right),
            ExactReportFreshness::SourceReplayMismatch
        );

        let closure =
            certify_volumetric_boundary_closure_report(&left, &right, ExactBooleanOperation::Union)
                .unwrap();
        assert_eq!(
            closure.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );

        let mut stale_closure = closure.clone();
        stale_closure.boundary_edges = 1;
        assert_eq!(
            stale_closure.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleStatusEvidence
        );
    }

    #[test]
    fn shortcut_and_blocker_reports_classify_freshness() {
        let left = report_test_tetra([0, 0, 0]);
        let right = report_test_tetra([3, 0, 0]);

        let refinement =
            certify_refinement_report(&left, &right, ExactBooleanOperation::Union).unwrap();
        assert_eq!(
            refinement.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );

        let mut stale_refinement = refinement.clone();
        stale_refinement.graph_had_unknowns = true;
        assert_eq!(
            stale_refinement.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleBlockerEvidence
        );

        let same_surface = certify_same_surface_report(&left, &left);
        assert_eq!(
            same_surface.freshness_against_sources(&left, &left),
            ExactReportFreshness::Current
        );
        assert_eq!(
            same_surface.freshness_against_sources(&left, &right),
            ExactReportFreshness::SourceReplayMismatch
        );

        let open_left = report_test_triangle(&[[0, 0, 0], [2, 0, 0], [0, 2, 0]]);
        let open_right = report_test_triangle(&[[5, 0, 0], [7, 0, 0], [5, 2, 0]]);
        let open_disjoint = certify_open_surface_disjoint_report(&open_left, &open_right).unwrap();
        assert_eq!(
            open_disjoint.freshness_against_sources(&open_left, &open_right),
            ExactReportFreshness::Current
        );

        let touching_right = report_test_triangle(&[[2, 0, 0], [0, 2, 0], [2, 2, 2]]);
        let boundary = certify_boundary_touching_report(&open_left, &touching_right).unwrap();
        assert_eq!(
            boundary.freshness_against_sources(&open_left, &touching_right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            boundary
                .blocker
                .freshness_against_sources(&open_left, &touching_right),
            ExactReportFreshness::Current
        );

        let mut stale_boundary = boundary.clone();
        stale_boundary.retained_events = 0;
        assert_eq!(
            stale_boundary.freshness_against_sources(&open_left, &touching_right),
            ExactReportFreshness::StaleBlockerEvidence
        );
    }

    #[test]
    fn boolean_result_freshness_classifies_local_source_and_operation_drift() {
        let left = report_test_tetra([0, 0, 0]);
        let right = report_test_tetra([3, 0, 0]);
        let result = boolean_exact_with_boundary_policy(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.operation_freshness_against_sources(
                &left,
                &right,
                ExactBooleanOperation::Union,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            ),
            ExactReportFreshness::Current
        );

        let mut stale_result = result.clone();
        stale_result.graph_had_unknowns = true;
        assert_eq!(
            stale_result.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleGraphUnknownStatus
        );

        assert_eq!(
            result.freshness_against_sources(&left, &left),
            ExactReportFreshness::SourceReplayMismatch
        );
        assert_eq!(
            result.operation_freshness_against_sources(
                &left,
                &right,
                ExactBooleanOperation::Intersection,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            ),
            ExactReportFreshness::OperationReplayMismatch
        );
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
    fn volumetric_boundary_blocked_report_rejects_stale_closure_evidence() {
        let mut report = valid_blocked_closure_report();
        report.validate().unwrap();

        report.coplanar_loop_groups = 1;
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
            ExactArrangementBlocker::UnsupportedCurvedPrimitive,
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
    fn planar_arrangement_required_rejects_intersection_relabel() {
        let mut report = ExactPlanarArrangementReport {
            operation: ExactBooleanOperation::Difference,
            status: ExactPlanarArrangementStatus::Required,
            graph_had_unknowns: false,
            retained_face_pairs: 1,
            retained_events: 1,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::NeedsPlanarArrangement,
                candidate_pairs: 0,
                coplanar_overlapping_pairs: 1,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            arrangement_readiness: Some(CoplanarArrangementReadinessReport {
                status: CoplanarArrangementReadinessStatus::NeedsPlanarCells,
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
        assert_eq!(
            report.validate(),
            Err(ExactReportValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn blocker_source_counts_include_unknown_segment_plane_events() {
        let graph = ExactIntersectionGraph {
            face_pairs: vec![crate::graph::FacePairEvents {
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
            }],
        };

        let blocker =
            blocker_source_counts(&graph).into_blocker(ExactBooleanBlockerKind::NeedsRefinement);
        assert_eq!(blocker.candidate_pairs, 1);
        assert_eq!(blocker.unknown_pairs, 1);
        assert_eq!(blocker.construction_failed_events, 0);
        assert!(
            blocker
                .validate_for_kind(ExactBooleanBlockerKind::NeedsRefinement)
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
                kind: ExactBooleanBlockerKind::NeedsRefinement,
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
