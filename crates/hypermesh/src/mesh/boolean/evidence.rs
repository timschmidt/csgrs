//! Auditable exact boolean evidence.
//!
//! These types are internal evidence objects produced by the exact boolean
//! staging layer. They carry graph counts, predicate certificates, and checked
//! kernel artifacts instead of collapsing exact topology decisions to `bool`.
//! Downstream mode layers should consume narrower borrowed kernel views.

use hyperlimit::{
    Aabb3Intersection, ApproximationPolicy, MeshSource, Point3, TriangleLocation,
    classify_aabb3_intersection, classify_point_triangle, compare_reals, compare_reals_report,
    project_point3, projected_polygon_area2_value,
};
use hyperreal::Real;
use std::cmp::Ordering;
use std::collections::BTreeSet;

use super::super::Mesh;
use super::super::arrangement3d::cell_complex::simplify::ExactSimplifiedCellComplex;
use super::super::arrangement3d::cell_complex::{
    ExactRegionOwnershipReport, ExactRegionOwnershipStatus, ExactSelectedCellComplex,
    ExactSelectedCellComplexCounts, arrangement_cell_complex_labeling_mode,
    validate_selected_gate_reports,
};
use super::super::arrangement3d::regularization::ExactArrangementBlocker;
use super::super::arrangement3d::regularization::ExactRegularizationMode;
use super::super::arrangement3d::{
    ExactArrangement3d, ExactTopologyAssemblyReport, ExactTopologyAssemblyStatus,
};
use super::super::facts::MeshFacts;
#[cfg(test)]
use super::super::graph::CoplanarArrangementEvidenceStatus;
use super::super::graph::MeshSide;
use super::super::graph::intersection::MeshFacePairRelation;
use super::super::graph::{
    CoplanarArrangementEvidence, ExactIntersectionGraph, IntersectionEvent,
    build_validated_intersection_graph,
};
use super::super::validation::MeshValidationMode;
use super::adjacent::{
    full_face_adjacent_certificate_from_graph,
    materialize_full_face_adjacent_union_from_certificate,
};
use super::affine_solid::{
    AffineOrthogonalSolidOperation, affine_orthogonal_solid_cell_selected_count,
    materialize_affine_orthogonal_solid_operation,
};
use super::contained_adjacent::{
    contained_face_adjacent_certificate_from_graph,
    materialize_contained_face_adjacent_union_from_certificate,
};
use super::convex::{
    intersect_closed_convex_solids, subtract_closed_convex_solids, union_closed_convex_solids,
};
use super::orthogonal_solid::{
    AxisAlignedOrthogonalSolidOperation, axis_aligned_orthogonal_solid_cell_selected_count,
    is_axis_aligned_box, materialize_axis_aligned_orthogonal_solid_cell_output,
};
use super::point3_exact_equal;
use super::region::{
    ExactBooleanAssemblyPlan, ExactOutputTriangle, ExactOutputTriangleOrientation,
    ExactRegionRetention, ExactRegionSelection, FaceRegionPlaneClassification,
    FaceRegionPlaneRelation, FaceRegionPlaneValidationError, FaceRegionTriangulation,
    boundary_node_point, validate_assembly_source_face_incidence,
};
#[cfg(test)]
use super::replay::exact_boolean_evaluation_for_replay_result_with_materialization;
#[cfg(test)]
use super::solid::ConvexSolidMeshClassification;
use super::solid::{ConvexSolidMeshRelation, classify_mesh_vertices_against_convex_solid_report};
use super::volumetric::{
    ExactVolumetricRegionClassification, ExactVolumetricRegionError, ExactVolumetricRegionRelation,
};
use super::volumetric_cells::{
    CoplanarVolumetricCellEvidenceReport, CoplanarVolumetricCellObstacle,
};
#[cfg(test)]
use super::winding::ClosedMeshWindingMeshReport;
use super::winding::{
    ClosedMeshWindingMeshRelation, classify_mesh_vertices_against_closed_mesh_winding_report,
};
#[cfg(test)]
use super::winding_evidence_report_for_request_from_graph_and_attempt;
use super::{
    ClosedRegularizedOperandKind, ExactBooleanOperation, ExactBooleanRequest,
    adjacent_union_completion_certification_from_graph, boolean_convex_meshes_optional,
    boolean_coplanar_mesh_overlay_optional, boolean_same_surface_meshes,
    boundary_touching_report_from_graph, closed_regularized_operand_kind,
    materialize_boolean_operation,
    materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph,
    materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph,
    materialize_open_surface_disjoint_meshes,
    materialize_volumetric_coplanar_boundary_closure_output_from_graph,
    open_surface_disjoint_report_from_graph,
    operation_evidence_for_exact_request_from_graph_with_retained_attempt,
    rematerialize_simplified_arrangement_cell_complex,
    replay_generic_arrangement_cell_complex_result,
    replay_selected_region_boolean_result_from_graph,
    volumetric_boundary_closure_report_from_graph, volumetric_retention_for_operation,
};
use hyperlimit::PredicateUse;

/// Validation failure for a retained exact evidence object.
///
/// Evidence validation checks the retained certificate object itself, not the
/// original geometry. It lets tests, fuzzing, and downstream mode code assert
/// that status, blocker kind, graph counts, and retained artifacts agree before
/// a result is trusted as certified evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum ExactEvidenceValidationError {
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
    /// A certified shortcut or boundary-mode result claimed unresolved graph
    /// events after materializing output topology.
    ShortcutResultHasUnknownGraph,
    /// A selected-region result claimed unresolved graph events after
    /// materializing output topology.
    SelectedRegionResultHasUnknownGraph,
    /// A selected-region result retained output triangles from a source side
    /// excluded by its declared selection mode.
    SelectedRegionAssemblyViolatesSelection,
    /// A selected-region result did not retain materialized evidence for a
    /// source region selected by its declared mode.
    SelectedRegionAssemblyMissingSelectedRegion,
    /// A volumetric materialized result retained output triangles that do not
    /// match the declared operation's per-cell volumetric retention mode.
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

/// Stage reached by an arrangement/cell-complex Boolean attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactArrangementBooleanStage {
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
    /// The triangulated mesh copied through the requested validation mode.
    Materialized,
}

/// Why an arrangement/cell-complex Boolean attempt declined to produce output.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum ExactArrangementBooleanDecline {
    /// Arrangement construction completed with blockers.
    ArrangementBlockers(Vec<ExactArrangementBlocker>),
    /// Cell labeling failed.
    Labeling(ExactArrangementBlocker),
    /// Exact topology assembly was not complete enough for cell output.
    TopologyAssembly(ExactTopologyAssemblyStatus),
    /// Region ownership was not resolved enough for named boolean selection.
    RegionOwnership(ExactRegionOwnershipStatus),
    /// Boolean cell selection failed.
    Selection(ExactArrangementBlocker),
    /// Exact simplification failed.
    Simplification(ExactArrangementBlocker),
    /// Exact triangulation failed.
    Triangulation(ExactArrangementBlocker),
    /// The triangulated mesh did not satisfy the requested validation mode.
    OutputValidation,
}

/// Why a retained arrangement/cell-complex attempt used a certified shortcut
/// or recovery output instead of the generic selected-cell triangulation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactArrangementBooleanShortcutReason {
    /// The shortcut was certified without constructing a generic arrangement.
    ShortcutSupportOnly,
    /// Arrangement construction had retained blockers.
    ArrangementConstructionBlocked,
    /// Topology assembly was not complete enough for generic output.
    TopologyAssemblyBlocked,
    /// Region ownership did not resolve the requested named operation.
    RegionOwnershipBlocked,
    /// Cell selection was blocked.
    SelectionBlocked,
    /// Exact simplification was blocked.
    SimplificationBlocked,
    /// Exact triangulation was blocked.
    TriangulationBlocked,
    /// The generic triangulated output did not satisfy the requested validation.
    OutputValidationBlocked,
    /// The generic path reached no more specific retained blocker.
    GenericMaterializationUnavailable,
}

pub(crate) const fn arrangement_attempt_stage_rank(stage: ExactArrangementBooleanStage) -> u8 {
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

/// Auditable result of trying the arrangement/cell-complex Boolean pipeline.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactArrangementBooleanAttempt {
    /// Operation attempted.
    pub(crate) operation: ExactBooleanOperation,
    /// Regularization mode used by the arrangement pipeline.
    pub(crate) mode: ExactRegularizationMode,
    /// Output validation mode used by shortcut recovery and final mesh copy.
    pub(crate) output_validation: MeshValidationMode,
    /// Furthest stage reached.
    pub(crate) stage: ExactArrangementBooleanStage,
    /// Reason no output was produced, when the attempt declined.
    pub(crate) decline: Option<ExactArrangementBooleanDecline>,
    /// Certified shortcut/recovery path that materialized output, when one did.
    ///
    /// A `None` value on a materialized attempt means the generic arrangement
    /// cell-complex path produced the output from retained topology and
    /// ownership evidence.
    pub(crate) materialized_shortcut: Option<ExactBooleanShortcutKind>,
    /// Reason a retained shortcut/recovery was used instead of the generic
    /// arrangement/cell-complex output.
    pub(crate) shortcut_reason: Option<ExactArrangementBooleanShortcutReason>,
    /// Arrangement blocker count observed after construction.
    pub(crate) arrangement_blockers: usize,
    /// Arrangement face-cell count, when construction succeeded.
    pub(crate) face_cells: usize,
    /// Connected shell/region count, when construction succeeded.
    pub(crate) regions: usize,
    /// Volume-region count, when closed shell topology produced a volume graph.
    pub(crate) volume_regions: usize,
    /// Volume adjacency count, when closed shell topology produced a volume graph.
    pub(crate) volume_adjacencies: usize,
    /// Retained lower-dimensional artifact count.
    pub(crate) lower_dimensional_artifacts: usize,
    /// Topology assembly status observed before consuming labeled cells.
    pub(crate) topology_assembly: Option<ExactTopologyAssemblyStatus>,
    /// Full topology assembly report consumed before labeled-cell output.
    pub(crate) topology_assembly_report: Option<ExactTopologyAssemblyReport>,
    /// Region ownership status observed before named cell selection.
    pub(crate) region_ownership: Option<ExactRegionOwnershipStatus>,
    /// Full region ownership report consumed before named cell selection.
    pub(crate) region_ownership_report: Option<ExactRegionOwnershipReport>,
    /// Selected face-cell count, when selection succeeded.
    pub(crate) selected_faces: usize,
    /// Selected faces whose output orientation is reversed.
    pub(crate) reversed_selected_faces: usize,
    /// Selected faces oriented by explicit volume adjacency evidence.
    pub(crate) volume_oriented_selected_faces: usize,
    /// Selected faces oriented by source-label operation rules.
    pub(crate) label_oriented_selected_faces: usize,
    /// Selected volume-region count, when selection succeeded.
    pub(crate) selected_volume_regions: usize,
    /// Retained selected cell complex consumed by simplification, when the
    /// generic arrangement path reached selection.
    pub(crate) selected_cell_complex: Option<ExactSelectedCellComplex>,
    /// Retained simplified cell complex consumed by triangulation, when the
    /// generic arrangement path reached simplification.
    pub(crate) simplified_cell_complex: Option<ExactSimplifiedCellComplex>,
    /// Output vertex count, when triangulation succeeded.
    pub(crate) output_vertices: usize,
    /// Output triangle count, when triangulation succeeded.
    pub(crate) output_triangles: usize,
    /// Retained output mesh facts, when a concrete triangulated mesh was built.
    pub(crate) output_facts: Option<MeshFacts>,
}

impl ExactArrangementBooleanAttempt {
    pub(crate) fn new(
        request: ExactBooleanRequest,
        mode: ExactRegularizationMode,
        stage: ExactArrangementBooleanStage,
    ) -> Self {
        Self {
            operation: request.operation,
            mode,
            output_validation: request.validation,
            stage,
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

    pub(crate) fn retain_arrangement_counts(&mut self, arrangement: &ExactArrangement3d) {
        self.arrangement_blockers = arrangement.blockers.len();
        self.face_cells = arrangement.face_cells.len();
        self.regions = arrangement
            .shells_or_regions
            .as_ref()
            .map_or(0, |regions| regions.len());
        self.volume_regions = arrangement
            .volume_regions
            .as_ref()
            .map_or(0, |regions| regions.len());
        self.volume_adjacencies = arrangement
            .volume_adjacencies
            .as_ref()
            .map_or(0, |adjacencies| adjacencies.len());
        self.lower_dimensional_artifacts = arrangement.lower_dimensional_artifacts.len();
    }

    pub(crate) fn retain_selected_counts(&mut self, counts: ExactSelectedCellComplexCounts) {
        self.selected_faces = counts.selected_faces;
        self.selected_volume_regions = counts.selected_volume_regions;
        self.reversed_selected_faces = counts.reversed_selected_faces;
        self.volume_oriented_selected_faces = counts.volume_oriented_selected_faces;
        self.label_oriented_selected_faces = counts.label_oriented_selected_faces;
    }

    pub(crate) fn retained_gate_reports(
        &self,
    ) -> Option<(&ExactTopologyAssemblyReport, &ExactRegionOwnershipReport)> {
        let topology = self.topology_assembly_report.as_ref()?;
        let ownership = self.region_ownership_report.as_ref()?;
        if topology.validate().is_ok()
            && matches!(topology.status, ExactTopologyAssemblyStatus::Complete)
            && ownership.validate().is_ok()
            && self.topology_assembly == Some(topology.status)
            && self.region_ownership == Some(ownership.status)
        {
            Some((topology, ownership))
        } else {
            None
        }
    }

    pub(crate) fn simplified_cell_complex_with_retained_gate_reports(
        &self,
    ) -> Option<&ExactSimplifiedCellComplex> {
        let simplified = self.simplified_cell_complex.as_ref()?;
        let (topology, ownership) = self.retained_gate_reports()?;
        if simplified.topology_assembly_report.as_ref() == Some(topology)
            && simplified.region_ownership_report.as_ref() == Some(ownership)
        {
            Some(simplified)
        } else {
            None
        }
    }

    /// Return whether this attempt materialized an arrangement cell-complex
    /// output, either through the generic path or through a certified
    /// arrangement shortcut/recovery path.
    pub(crate) fn materialized_arrangement_cell_complex_output(&self) -> bool {
        self.materialized_arrangement_cell_complex_shortcut_output()
            || self.materialized_generic_arrangement_cell_complex_output()
    }

    /// Return whether this attempt materialized through the certified
    /// arrangement cell-complex shortcut/recovery path.
    pub(crate) fn materialized_arrangement_cell_complex_shortcut_output(&self) -> bool {
        self.stage == ExactArrangementBooleanStage::Materialized
            && self.decline.is_none()
            && self.materialized_shortcut == Some(ExactBooleanShortcutKind::ArrangementCellComplex)
    }

    /// Return whether this attempt materialized through the generic retained
    /// arrangement/cell-complex path.
    pub(crate) fn materialized_generic_arrangement_cell_complex_output(&self) -> bool {
        if self.stage != ExactArrangementBooleanStage::Materialized || self.decline.is_some() {
            return false;
        }
        match self.materialized_shortcut {
            None => {
                self.retained_gate_reports().is_some() && self.resolves_requested_volume_ownership()
            }
            Some(_) => false,
        }
    }

    /// Return whether the retained region ownership evidence resolves this
    /// attempt's requested named operation.
    pub(crate) fn resolves_requested_volume_ownership(&self) -> bool {
        let (Some(status), Some(report)) =
            (self.region_ownership, self.region_ownership_report.as_ref())
        else {
            return false;
        };
        report.status == status
            && report.validate().is_ok()
            && report.resolves_requested_volume_ownership(self.operation)
    }

    /// Return whether another replay attempt certifies the same materialized
    /// arrangement/cell-complex output.
    pub(crate) fn materialized_output_matches_replay(&self, replay: &Self) -> bool {
        let same_source_output = self.operation == replay.operation
            && self.output_validation == replay.output_validation
            && self.mode == replay.mode
            && self.materialized_arrangement_cell_complex_output()
            && replay.materialized_arrangement_cell_complex_output()
            && self.output_vertices == replay.output_vertices
            && self.output_triangles == replay.output_triangles
            && self.output_facts == replay.output_facts;
        if !same_source_output {
            return false;
        }
        if self.materialized_shortcut == replay.materialized_shortcut
            && self.retained_gate_reports() == replay.retained_gate_reports()
        {
            return true;
        }
        self.materialized_generic_arrangement_cell_complex_output()
            && replay.materialized_arrangement_cell_complex_shortcut_output()
            && replay.retained_gate_reports().is_none()
    }

    /// Return the retained shortcut reason implied by the current attempt state.
    pub(crate) fn recovered_shortcut_reason(&self) -> ExactArrangementBooleanShortcutReason {
        match self.decline.as_ref() {
            Some(ExactArrangementBooleanDecline::ArrangementBlockers(_)) => {
                return ExactArrangementBooleanShortcutReason::ArrangementConstructionBlocked;
            }
            Some(ExactArrangementBooleanDecline::TopologyAssembly(_)) => {
                return ExactArrangementBooleanShortcutReason::TopologyAssemblyBlocked;
            }
            Some(ExactArrangementBooleanDecline::RegionOwnership(_)) => {
                return ExactArrangementBooleanShortcutReason::RegionOwnershipBlocked;
            }
            Some(ExactArrangementBooleanDecline::Labeling(_))
            | Some(ExactArrangementBooleanDecline::Selection(_)) => {
                return ExactArrangementBooleanShortcutReason::SelectionBlocked;
            }
            Some(ExactArrangementBooleanDecline::Simplification(_)) => {
                return ExactArrangementBooleanShortcutReason::SimplificationBlocked;
            }
            Some(ExactArrangementBooleanDecline::Triangulation(_)) => {
                return ExactArrangementBooleanShortcutReason::TriangulationBlocked;
            }
            Some(ExactArrangementBooleanDecline::OutputValidation) => {
                return ExactArrangementBooleanShortcutReason::OutputValidationBlocked;
            }
            None => {}
        }
        match self.stage {
            ExactArrangementBooleanStage::NotAttempted => {
                ExactArrangementBooleanShortcutReason::ShortcutSupportOnly
            }
            ExactArrangementBooleanStage::ArrangementBuilt if self.arrangement_blockers != 0 => {
                ExactArrangementBooleanShortcutReason::ArrangementConstructionBlocked
            }
            ExactArrangementBooleanStage::ArrangementBuilt => {
                ExactArrangementBooleanShortcutReason::TopologyAssemblyBlocked
            }
            ExactArrangementBooleanStage::Labeled => {
                if !self.resolves_requested_volume_ownership() {
                    ExactArrangementBooleanShortcutReason::RegionOwnershipBlocked
                } else {
                    ExactArrangementBooleanShortcutReason::SelectionBlocked
                }
            }
            ExactArrangementBooleanStage::Selected => {
                ExactArrangementBooleanShortcutReason::SimplificationBlocked
            }
            ExactArrangementBooleanStage::Simplified => {
                ExactArrangementBooleanShortcutReason::TriangulationBlocked
            }
            ExactArrangementBooleanStage::Triangulated => {
                ExactArrangementBooleanShortcutReason::OutputValidationBlocked
            }
            ExactArrangementBooleanStage::Materialized => {
                ExactArrangementBooleanShortcutReason::GenericMaterializationUnavailable
            }
        }
    }

    /// Record the output mesh certificate retained by this attempt.
    pub(crate) fn retain_output_mesh(&mut self, mesh: &Mesh) {
        self.output_vertices = mesh.vertices().len();
        self.output_triangles = mesh.facts().mesh.face_count;
        self.output_facts = Some(mesh.facts().mesh.clone());
    }

    pub(crate) fn validate_for_request_regularization_mode(
        &self,
        request: ExactBooleanRequest,
        mode: ExactRegularizationMode,
    ) -> Result<(), ExactEvidenceValidationError> {
        self.validate()?;
        if self.operation != request.operation
            || self.mode != mode
            || self.output_validation != request.validation
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        Ok(())
    }

    /// Validate this retained arrangement/cell-complex attempt as a coherent
    /// audit artifact.
    ///
    /// The attempt report is a public provenance object for a staged topology
    /// construction. Its stage, decline reason, shortcut materialization, and
    /// retained counts must describe one path through that state machine rather
    /// than an arbitrary mix of successful output and blockers.
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        let Some(oriented_selected_faces) = self
            .volume_oriented_selected_faces
            .checked_add(self.label_oriented_selected_faces)
        else {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        };
        if self.selected_faces > self.face_cells
            || self.selected_volume_regions > self.volume_regions
            || (self.volume_regions != 0 && self.regions == 0)
            || (self.volume_adjacencies != 0 && self.volume_regions < 2)
            || (self.selected_volume_regions != 0 && self.volume_regions == 0)
            || self.reversed_selected_faces > self.selected_faces
            || self.volume_oriented_selected_faces > self.selected_faces
            || self.label_oriented_selected_faces > self.selected_faces
            || oriented_selected_faces != self.selected_faces
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }

        if self.materialized_shortcut.is_some() != self.shortcut_reason.is_some() {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }

        match &self.decline {
            Some(decline) => {
                if self.materialized_shortcut.is_some()
                    || self.stage == ExactArrangementBooleanStage::Materialized
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                if let ExactArrangementBooleanDecline::ArrangementBlockers(blockers) = decline
                    && (blockers.is_empty() || blockers.len() != self.arrangement_blockers)
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                if !matches!(
                    (decline, self.stage),
                    (
                        ExactArrangementBooleanDecline::ArrangementBlockers(_)
                            | ExactArrangementBooleanDecline::Labeling(_)
                            | ExactArrangementBooleanDecline::TopologyAssembly(_),
                        ExactArrangementBooleanStage::ArrangementBuilt
                    ) | (
                        ExactArrangementBooleanDecline::RegionOwnership(_)
                            | ExactArrangementBooleanDecline::Selection(_),
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
                ) {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            None => {
                if !self.materialized_arrangement_cell_complex_output() {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
        }
        let pre_gate_output_validation = matches!(
            self.decline,
            Some(ExactArrangementBooleanDecline::OutputValidation)
        ) && self.stage
            == ExactArrangementBooleanStage::Triangulated
            && self.topology_assembly.is_none()
            && self.region_ownership.is_none();
        if let Some(selected) = &self.selected_cell_complex {
            let counts = selected.counts();
            let selected_matches_gate_reports = match self.retained_gate_reports() {
                Some((topology, ownership)) => {
                    selected.topology_assembly_report.as_ref() == Some(topology)
                        && selected.region_ownership_report.as_ref() == Some(ownership)
                }
                None => false,
            };
            if arrangement_attempt_stage_rank(self.stage)
                < arrangement_attempt_stage_rank(ExactArrangementBooleanStage::Selected)
                || selected.operation != self.operation
                || selected.validate().is_err()
                || !selected_matches_gate_reports
                || counts.selected_faces != self.selected_faces
                || counts.selected_volume_regions != self.selected_volume_regions
                || counts.reversed_selected_faces != self.reversed_selected_faces
                || counts.volume_oriented_selected_faces != self.volume_oriented_selected_faces
                || counts.label_oriented_selected_faces != self.label_oriented_selected_faces
            {
                return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
            }
        }
        if let Some(simplified) = &self.simplified_cell_complex
            && (arrangement_attempt_stage_rank(self.stage)
                < arrangement_attempt_stage_rank(ExactArrangementBooleanStage::Simplified)
                || self.selected_cell_complex.is_none()
                || simplified.operation != self.operation
                || simplified.validate().is_err()
                || self.simplified_cell_complex_with_retained_gate_reports() != Some(simplified)
                || simplified.selected_faces_before_simplification != self.selected_faces
                || simplified.oriented_selected_faces_before_simplification != self.selected_faces
                || simplified.reversed_selected_faces_before_simplification
                    != self.reversed_selected_faces
                || simplified.volume_oriented_selected_faces_before_simplification
                    != self.volume_oriented_selected_faces
                || simplified.label_oriented_selected_faces_before_simplification
                    != self.label_oriented_selected_faces)
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if self.stage == ExactArrangementBooleanStage::NotAttempted {
            if self.topology_assembly.is_some()
                || self.region_ownership.is_some()
                || self.selected_cell_complex.is_some()
                || self.simplified_cell_complex.is_some()
            {
                return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
            }
        } else if self.region_ownership.is_some() && self.topology_assembly.is_none() {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if !matches!(
            (&self.topology_assembly, &self.topology_assembly_report),
            (Some(status), Some(report)) if report.status == *status && report.validate().is_ok()
        ) && !matches!(
            (&self.topology_assembly, &self.topology_assembly_report),
            (None, None)
        ) {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if !matches!(
            (&self.region_ownership, &self.region_ownership_report),
            (Some(status), Some(report)) if report.status == *status && report.validate().is_ok()
        ) && !matches!(
            (&self.region_ownership, &self.region_ownership_report),
            (None, None)
        ) {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if self.region_ownership_report.is_some() && self.topology_assembly_report.is_none() {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        match self.decline.as_ref() {
            Some(ExactArrangementBooleanDecline::TopologyAssembly(status))
                if self.topology_assembly != Some(*status) || self.region_ownership.is_some() =>
            {
                return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
            }
            Some(ExactArrangementBooleanDecline::Labeling(_))
                if self.topology_assembly.is_none() || self.region_ownership.is_some() =>
            {
                return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
            }
            Some(ExactArrangementBooleanDecline::RegionOwnership(status))
                if self.region_ownership != Some(*status) =>
            {
                return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
            }
            Some(
                ExactArrangementBooleanDecline::Selection(_)
                | ExactArrangementBooleanDecline::Simplification(_)
                | ExactArrangementBooleanDecline::Triangulation(_),
            ) if self.region_ownership.is_none() => {
                return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
            }
            Some(ExactArrangementBooleanDecline::OutputValidation)
                if !pre_gate_output_validation && self.region_ownership.is_none() =>
            {
                return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
            }
            _ => {}
        }
        if self.decline.is_none()
            && !matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
            && self.region_ownership.is_some()
            && !self.materialized_arrangement_cell_complex_shortcut_output()
            && !self.resolves_requested_volume_ownership()
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if (self.stage == ExactArrangementBooleanStage::Selected
            || self.stage == ExactArrangementBooleanStage::Simplified
            || self.stage == ExactArrangementBooleanStage::Triangulated
            || self.selected_faces != 0
            || self.reversed_selected_faces != 0
            || self.volume_oriented_selected_faces != 0
            || self.label_oriented_selected_faces != 0
            || self.selected_volume_regions != 0)
            && self.region_ownership.is_none()
            && !pre_gate_output_validation
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if self.output_triangles != 0 && self.output_vertices == 0 {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if let Some(output_facts) = &self.output_facts {
            if output_facts.vertex_count != self.output_vertices
                || output_facts.face_count != self.output_triangles
            {
                return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
            }
            if arrangement_attempt_stage_rank(self.stage)
                < arrangement_attempt_stage_rank(ExactArrangementBooleanStage::Triangulated)
            {
                return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
            }
        }
        if self.decline.is_none()
            && self.operation == ExactBooleanOperation::Union
            && self.output_triangles == 0
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if self.decline.is_none()
            && self.stage == ExactArrangementBooleanStage::Materialized
            && self.output_facts.is_none()
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if self.stage == ExactArrangementBooleanStage::NotAttempted
            && (self.arrangement_blockers != 0
                || self.face_cells != 0
                || self.regions != 0
                || self.volume_regions != 0
                || self.volume_adjacencies != 0
                || self.lower_dimensional_artifacts != 0
                || self.selected_faces != 0
                || self.reversed_selected_faces != 0
                || self.volume_oriented_selected_faces != 0
                || self.label_oriented_selected_faces != 0
                || self.selected_volume_regions != 0
                || self.output_vertices != 0
                || self.output_triangles != 0
                || self.output_facts.is_some()
                || self.selected_cell_complex.is_some()
                || self.simplified_cell_complex.is_some())
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if arrangement_attempt_stage_rank(self.stage)
            < arrangement_attempt_stage_rank(ExactArrangementBooleanStage::Labeled)
            && (self.selected_faces != 0
                || self.reversed_selected_faces != 0
                || self.volume_oriented_selected_faces != 0
                || self.label_oriented_selected_faces != 0
                || self.selected_volume_regions != 0
                || self.selected_cell_complex.is_some())
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if arrangement_attempt_stage_rank(self.stage)
            < arrangement_attempt_stage_rank(ExactArrangementBooleanStage::Simplified)
            && self.simplified_cell_complex.is_some()
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if arrangement_attempt_stage_rank(self.stage)
            < arrangement_attempt_stage_rank(ExactArrangementBooleanStage::Triangulated)
            && (self.output_vertices != 0
                || self.output_triangles != 0
                || self.output_facts.is_some())
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        Ok(())
    }
}

fn validated_report_intersection_graph(
    left: &Mesh,
    right: &Mesh,
) -> Result<ExactIntersectionGraph, ExactEvidenceValidationError> {
    build_validated_intersection_graph(left, right)
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)
}

struct ExactBooleanSourceReplay<'a> {
    left: &'a Mesh,
    right: &'a Mesh,
    validated_graph: Option<Result<ExactIntersectionGraph, ExactEvidenceValidationError>>,
}

impl<'a> ExactBooleanSourceReplay<'a> {
    fn new(left: &'a Mesh, right: &'a Mesh) -> Self {
        Self {
            left,
            right,
            validated_graph: None,
        }
    }

    fn validated_graph(&mut self) -> Result<&ExactIntersectionGraph, ExactEvidenceValidationError> {
        if self.validated_graph.is_none() {
            self.validated_graph = Some(validated_report_intersection_graph(self.left, self.right));
        }
        match self
            .validated_graph
            .as_ref()
            .expect("validated graph cache set")
        {
            Ok(graph) => Ok(graph),
            Err(error) => Err(error.clone()),
        }
    }
}

fn no_region_facts(
    region_count: usize,
    classifications: &[FaceRegionPlaneClassification],
) -> Result<(), ExactEvidenceValidationError> {
    if region_count == 0 && classifications.is_empty() {
        Ok(())
    } else {
        Err(ExactEvidenceValidationError::UnexpectedRegionFacts)
    }
}

fn validate_blocker_count_bounds(
    blocker: &ExactBooleanBlocker,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactEvidenceValidationError> {
    let Some(classified_relation_pairs) = blocker
        .candidate_pairs
        .checked_add(blocker.coplanar_overlapping_pairs)
        .and_then(|count| count.checked_add(blocker.coplanar_touching_pairs))
    else {
        return Err(ExactEvidenceValidationError::InvalidBlockerCounts);
    };
    // `unknown_pairs` can overlap a classified relation when a candidate pair
    // carries an unknown event, but every retained graph pair must still be
    // covered by either a classified relation counter or unknown evidence.
    let Some(covered_relation_pairs) = classified_relation_pairs.checked_add(blocker.unknown_pairs)
    else {
        return Err(ExactEvidenceValidationError::InvalidBlockerCounts);
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
        Err(ExactEvidenceValidationError::InvalidBlockerCounts)
    } else {
        Ok(())
    }
}

fn validate_blocker_evidence(
    blocker: Option<&ExactBooleanBlocker>,
    expected: ExactBooleanBlockerKind,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<&ExactBooleanBlocker, ExactEvidenceValidationError> {
    let blocker = match blocker {
        Some(blocker) if blocker.kind == expected => blocker,
        Some(_) => return Err(ExactEvidenceValidationError::WrongBlockerKind),
        None => return Err(ExactEvidenceValidationError::MissingBlocker),
    };
    blocker.validate_for_kind(expected)?;
    validate_blocker_count_bounds(blocker, retained_face_pairs, retained_events)?;
    Ok(blocker)
}

fn validate_retained_graph_count_shape(
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactEvidenceValidationError> {
    if (retained_face_pairs == 0 && retained_events != 0)
        || (retained_face_pairs != 0 && retained_events == 0)
        || retained_events < retained_face_pairs
    {
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    } else {
        Ok(())
    }
}

fn validate_coplanar_arrangement_evidence_matches_blocker(
    evidence: &CoplanarArrangementEvidence,
    blocker: &ExactBooleanBlocker,
) -> Result<(), ExactEvidenceValidationError> {
    // The compact evidence report and blocker are two projections of the same
    // exact graph state; downstream planar-cell or winding checks must not
    // consume a summary with relabeled graph counts.
    if evidence.overlapping_graphs != blocker.coplanar_overlapping_pairs
        || evidence.touching_graphs != blocker.coplanar_touching_pairs
        || evidence.graph_count
            != blocker
                .coplanar_overlapping_pairs
                .checked_add(blocker.coplanar_touching_pairs)
                .ok_or(ExactEvidenceValidationError::CoplanarArrangementEvidenceMismatch)?
    {
        Err(ExactEvidenceValidationError::CoplanarArrangementEvidenceMismatch)
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

fn validate_adjacent_certified_boundary_blocker(
    blocker: &ExactBooleanBlocker,
    retained_face_pairs: usize,
    retained_events: usize,
) -> Result<(), ExactEvidenceValidationError> {
    if retained_face_pairs == 0 && retained_events == 0 && !blocker_has_any_evidence(blocker) {
        return (blocker.kind == ExactBooleanBlockerKind::BoundaryOnlyContact)
            .then_some(())
            .ok_or(ExactEvidenceValidationError::WrongBlockerKind);
    }
    blocker.validate_for_kind(ExactBooleanBlockerKind::BoundaryOnlyContact)
}

fn validate_refinement_partition(
    graph_unknown_status: bool,
    blocker: &ExactBooleanBlocker,
) -> Result<(), ExactEvidenceValidationError> {
    // Unknown predicate outcomes and failed exact constructions are both
    // boundary, planar-cell, and winding reports must not consume unresolved
    // construction state under a resolved status label.
    if graph_unknown_status {
        if blocker.unknown_pairs != 0 || blocker.construction_failed_events != 0 {
            Ok(())
        } else {
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        }
    } else if blocker.unknown_pairs != 0 || blocker.construction_failed_events != 0 {
        Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
    } else {
        Ok(())
    }
}

fn checked_region_facts(
    region_count: usize,
    classifications: &[FaceRegionPlaneClassification],
) -> Result<(), ExactEvidenceValidationError> {
    if region_count == 0 || classifications.is_empty() {
        return Err(ExactEvidenceValidationError::MissingRegionFacts);
    }
    let mut unique_regions = Vec::new();
    let mut unique_classifications = Vec::new();
    for classification in classifications {
        classification
            .validate()
            .map_err(ExactEvidenceValidationError::InvalidRegionClassification)?;
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
            return Err(ExactEvidenceValidationError::DuplicateRegionClassification);
        }
        unique_classifications.push(classification_key);
        // Winding-ready evidence must carry decided side facts, not an
        // "unknown" region/plane relation. Undecided predicates remain
        // explicit blockers instead of being mislabeled as classified regions.
        if !classification.all_proof_producing()
            || matches!(classification.relation, FaceRegionPlaneRelation::Unknown)
        {
            return Err(ExactEvidenceValidationError::RegionClassificationNotProofProducing);
        }
    }
    // `region_count` is a retained combinatorial fact, not a display counter.
    // It must match the unique region handles covered by plane classifications
    // so a later winding mode cannot silently consume stale or relabeled
    if unique_regions.len() != region_count {
        return Err(ExactEvidenceValidationError::RegionCountMismatch);
    }
    Ok(())
}

fn validate_coplanar_volumetric_evidence_matches_blocker(
    evidence: &CoplanarVolumetricCellEvidenceReport,
    blocker: &ExactBooleanBlocker,
    retained_face_pairs: usize,
    retained_events: usize,
    requirement: CoplanarVolumetricEvidenceRequirement,
) -> Result<(), ExactEvidenceValidationError> {
    validate_coplanar_volumetric_evidence(
        evidence,
        retained_face_pairs,
        retained_events,
        requirement,
    )?;
    if evidence.candidate_pairs != blocker.candidate_pairs
        || evidence.coplanar_touching_pairs != blocker.coplanar_touching_pairs
        || evidence.coplanar_overlapping_pairs != blocker.coplanar_overlapping_pairs
        || evidence.unknown_pairs != blocker.unknown_pairs
        || evidence.construction_failed_events != blocker.construction_failed_events
    {
        return Err(ExactEvidenceValidationError::CoplanarVolumetricEvidenceMismatch);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CoplanarVolumetricEvidenceRequirement {
    CountsOnly,
    NeedsCoplanarCells,
    BoundaryOnlyContact,
    MaterializedArrangement,
}

fn validate_coplanar_volumetric_evidence(
    evidence: &CoplanarVolumetricCellEvidenceReport,
    retained_face_pairs: usize,
    retained_events: usize,
    requirement: CoplanarVolumetricEvidenceRequirement,
) -> Result<(), ExactEvidenceValidationError> {
    evidence
        .validate()
        .map_err(|_| ExactEvidenceValidationError::InvalidCoplanarVolumetricEvidence)?;
    let Some(explicit_unknown_events) = evidence
        .unknown_events
        .checked_sub(evidence.unknown_segment_plane_events)
    else {
        return Err(ExactEvidenceValidationError::CoplanarVolumetricEvidenceMismatch);
    };
    let Some(retained_evidence_events) = evidence
        .segment_plane_events
        .checked_add(evidence.coplanar_edge_events)
        .and_then(|count| count.checked_add(evidence.coplanar_vertex_events))
        .and_then(|count| count.checked_add(explicit_unknown_events))
    else {
        return Err(ExactEvidenceValidationError::CoplanarVolumetricEvidenceMismatch);
    };
    if evidence.retained_face_pair_count != retained_face_pairs
        || retained_evidence_events != retained_events
    {
        return Err(ExactEvidenceValidationError::CoplanarVolumetricEvidenceMismatch);
    }
    let matches_requirement = match requirement {
        CoplanarVolumetricEvidenceRequirement::CountsOnly => true,
        CoplanarVolumetricEvidenceRequirement::NeedsCoplanarCells => matches!(
            evidence.obstacle,
            CoplanarVolumetricCellObstacle::NeedsCoplanarVolumetricCells
                | CoplanarVolumetricCellObstacle::MixedCoplanarAndCrossingCells
        ),
        CoplanarVolumetricEvidenceRequirement::BoundaryOnlyContact => {
            matches!(
                evidence.obstacle,
                CoplanarVolumetricCellObstacle::BoundaryOnlyContact
            ) && evidence.positive_area_coplanar_overlapping_pairs != 0
        }
        CoplanarVolumetricEvidenceRequirement::MaterializedArrangement => {
            matches!(
                evidence.obstacle,
                CoplanarVolumetricCellObstacle::NeedsCoplanarVolumetricCells
                    | CoplanarVolumetricCellObstacle::MixedCoplanarAndCrossingCells
            ) || (matches!(
                evidence.obstacle,
                CoplanarVolumetricCellObstacle::BoundaryOnlyContact
            ) && evidence.positive_area_coplanar_overlapping_pairs != 0)
        }
    };
    if !matches_requirement {
        return Err(ExactEvidenceValidationError::CoplanarVolumetricEvidenceMismatch);
    }
    Ok(())
}

/// Auditable result of an exact selected-region boolean pipeline.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactBooleanResult {
    /// Declared production path for this result.
    pub(crate) kind: ExactBooleanResultKind,
    /// Whether graph extraction contained unknown events before mode checks.
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
    /// Materialized exact output mesh validated under the requested mode.
    pub(crate) mesh: Mesh,
}

impl ExactBooleanResult {
    pub(crate) fn matches_retained_replay(
        &self,
        replay: &Self,
    ) -> Result<bool, ExactEvidenceValidationError> {
        Ok(self.kind == replay.kind
            && self.graph_had_unknowns == replay.graph_had_unknowns
            && self.region_classifications == replay.region_classifications
            && self.triangulations == replay.triangulations
            && self.assembly == replay.assembly
            && self.volumetric_classifications == replay.volumetric_classifications
            && ((self.topology_assembly_report == replay.topology_assembly_report
                && self.region_ownership_report == replay.region_ownership_report)
                || (self.kind.is_arrangement_cell_complex_shortcut()
                    && self.topology_assembly_report.is_none()
                    && self.region_ownership_report.is_none()))
            && retained_output_mesh_matches(&self.mesh, &replay.mesh)?)
    }
}

/// Declared production path for an exact boolean result.
///
/// Result kind is explicit so validation does not infer semantic intent from
/// empty vectors. That distinction matters for exact computing: selected-region
/// assembly, certified shortcuts, and boundary-mode projections are different
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
    /// then triangulating to an [`Mesh`].
    ArrangementCellComplex,
}

impl ExactBooleanShortcutKind {
    const fn required_named_operation(self) -> Option<ExactBooleanOperation> {
        match self {
            Self::ClosedBoundaryTouchingUnion | Self::ConvexUnion => {
                Some(ExactBooleanOperation::Union)
            }
            Self::ClosedBoundaryTouchingIntersection | Self::ConvexIntersection => {
                Some(ExactBooleanOperation::Intersection)
            }
            Self::ClosedBoundaryTouchingDifference | Self::ConvexDifference => {
                Some(ExactBooleanOperation::Difference)
            }
            Self::EmptyOperand
            | Self::BoundsDisjoint
            | Self::Identical
            | Self::SameSurface
            | Self::OpenSurfaceDisjoint
            | Self::ClosedWindingSeparated
            | Self::ClosedWindingContainment
            | Self::MixedDimensionalRegularizedSolid
            | Self::LowerDimensionalRegularizedSolid
            | Self::ConvexContainment
            | Self::ConvexSeparated
            | Self::ArrangementCellComplex => None,
        }
    }
}

impl ExactBooleanResultKind {
    fn certified_shortcut(self) -> Option<(ExactBooleanOperation, ExactBooleanShortcutKind)> {
        match self {
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut,
            } => Some((operation, shortcut)),
            _ => None,
        }
    }

    fn retains_region_artifacts(self) -> bool {
        matches!(
            self,
            ExactBooleanResultKind::SelectedRegions { .. }
                | ExactBooleanResultKind::OpenSurfaceArrangement { .. }
                | ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
        )
    }

    fn retains_volumetric_artifacts(self) -> bool {
        matches!(
            self,
            ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
        )
    }

    pub(crate) fn is_arrangement_cell_complex_shortcut(self) -> bool {
        matches!(
            self,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
                ..
            }
        )
    }

    pub(crate) fn arrangement_cell_complex_operation(self) -> Option<ExactBooleanOperation> {
        match self {
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

    fn request(self, validation: MeshValidationMode) -> ExactBooleanRequest {
        match self {
            ExactBooleanResultKind::SelectedRegions { selection } => ExactBooleanRequest {
                operation: ExactBooleanOperation::SelectedRegions(selection),
                validation: validation,
            },
            ExactBooleanResultKind::CertifiedShortcut { operation, .. }
            | ExactBooleanResultKind::OpenSurfaceArrangement { operation }
            | ExactBooleanResultKind::ArrangementCellComplexMaterialized { operation } => {
                ExactBooleanRequest {
                    operation: operation,
                    validation: validation,
                }
            }
        }
    }

    fn matches_request(self, request: ExactBooleanRequest) -> bool {
        self.request(request.validation).operation == request.operation
    }
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
    /// and triangulations because those are the checked boundary facts that
    /// justify the assembly; otherwise a caller could relabel an empty
    /// shortcut-like object as a selected-region boolean.
    /// Every retained triangulation must also have at least one matching
    /// retained region/plane classification for its source side and face, so
    /// the mesh boundary cannot contain triangulated topology disconnected from
    /// the exact side facts prepared for winding mode. Conversely, every
    /// retained region/plane classification must belong to a triangulated
    /// source region so stale or relabeled side facts cannot be interpreted as
    /// part of the output proof. Selected-region reports also require those
    /// side facts to be proof-producing and decided, rather than carrying an
    /// unknown relation beside a materialized output. Duplicate
    /// region/opposite-plane classifications are rejected for the same reason:
    /// retained side evidence is exact state, not a multiset that later
    /// winding code can count twice. The same rule applies to retained
    /// triangulations: each source region has one checked polygon-to-triangle
    /// boundary. Output assembly triangles must likewise point back to retained
    /// triangulated source regions,
    /// preventing post-hoc provenance relabeling after materialization, and
    /// their vertex sources must be members of the retained triangulation
    /// boundary for that source region; welded vertices may carry a different
    /// source witness, but their exact point must still replay to the retained
    /// boundary. The retained assembly must also avoid dead vertices so the
    /// topology boundary is the exact set consumed by mesh materialization. That
    /// rather than an opaque output mesh.
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        let retains_region_artifacts = self.kind.retains_region_artifacts();
        let retains_volumetric_artifacts = self.kind.retains_volumetric_artifacts();
        if !retains_region_artifacts
            && (!self.region_classifications.is_empty()
                || !self.triangulations.is_empty()
                || !self.assembly.vertices.is_empty()
                || !self.assembly.triangles.is_empty())
        {
            return Err(ExactEvidenceValidationError::ShortcutResultHasAssemblyArtifacts);
        }
        if retains_volumetric_artifacts && self.volumetric_classifications.is_empty() {
            return Err(ExactEvidenceValidationError::MissingVolumetricClassifications);
        }
        if !retains_volumetric_artifacts && !self.volumetric_classifications.is_empty() {
            return Err(ExactEvidenceValidationError::UnexpectedVolumetricClassifications);
        }
        if !retains_region_artifacts && self.graph_had_unknowns {
            return Err(ExactEvidenceValidationError::ShortcutResultHasUnknownGraph);
        }
        if let Some((operation, shortcut)) = self.kind.certified_shortcut() {
            if !shortcut_operation_matches(shortcut, operation) {
                return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
            }
            validate_shortcut_output_shape(shortcut, operation, &self.mesh)?;
        }
        if let ExactBooleanResultKind::OpenSurfaceArrangement { operation }
        | ExactBooleanResultKind::ArrangementCellComplexMaterialized { operation } = self.kind
            && matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        self.validate_arrangement_cell_complex_gate_reports()?;
        if retains_region_artifacts && self.graph_had_unknowns {
            return Err(ExactEvidenceValidationError::SelectedRegionResultHasUnknownGraph);
        }
        if retains_region_artifacts
            && (self.region_classifications.is_empty() || self.triangulations.is_empty())
        {
            return Err(ExactEvidenceValidationError::MissingRegionFacts);
        }

        self.validate_retained_region_and_volumetric_facts(
            retains_region_artifacts,
            retains_volumetric_artifacts,
        )?;
        if retains_region_artifacts
            && self.assembly.triangles.iter().any(|triangle| {
                !self.triangulations.iter().any(|triangulation| {
                    triangulation.side == triangle.source_side
                        && triangulation.face == triangle.source_face
                })
            })
        {
            return Err(ExactEvidenceValidationError::UntriangulatedAssemblyRegion);
        }
        if retains_region_artifacts {
            for triangle in &self.assembly.triangles {
                let Some(triangulation) = self.triangulations.iter().find(|triangulation| {
                    triangulation.side == triangle.source_side
                        && triangulation.face == triangle.source_face
                }) else {
                    return Err(ExactEvidenceValidationError::UntriangulatedAssemblyRegion);
                };
                for &vertex in &triangle.vertices {
                    let Some(assembly_vertex) = self.assembly.vertices.get(vertex) else {
                        return Err(ExactEvidenceValidationError::InvalidAssembly);
                    };
                    if !retains_volumetric_artifacts {
                        let mut inside_triangulation = false;
                        for source in &triangulation.boundary {
                            if source == &assembly_vertex.source {
                                inside_triangulation = true;
                                break;
                            }
                            match point3_exact_equal(
                                &assembly_vertex.point,
                                boundary_node_point(source),
                            ) {
                                Some(true) => {
                                    inside_triangulation = true;
                                    break;
                                }
                                Some(false) => {}
                                None => {
                                    return Err(
                                        ExactEvidenceValidationError::AssemblyVertexOutsideTriangulation,
                                    );
                                }
                            }
                        }
                        if !inside_triangulation {
                            return Err(
                                ExactEvidenceValidationError::AssemblyVertexOutsideTriangulation,
                            );
                        }
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
                return Err(ExactEvidenceValidationError::UnreferencedAssemblyVertex);
            }
        }
        self.assembly
            .validate()
            .map_err(|_| ExactEvidenceValidationError::InvalidAssembly)?;
        if retains_region_artifacts {
            let mut seen_triangle_vertex_sets = BTreeSet::new();
            for triangle in &self.assembly.triangles {
                let mut vertices = triangle.vertices;
                vertices.sort_unstable();
                if !seen_triangle_vertex_sets.insert(vertices) {
                    return Err(ExactEvidenceValidationError::DuplicateAssemblyTriangle);
                }
            }
        }
        self.mesh
            .validate_retained_state()
            .map_err(|_| ExactEvidenceValidationError::InvalidOutputMesh)?;
        let output_source = &self.mesh.provenance().source;
        let has_exact_boolean_label = output_source.label.starts_with("exact ")
            || output_source.label.starts_with("empty exact ");
        let has_arrangement_label = output_source.label.contains("arrangement")
            || output_source.label.contains("cell-complex")
            || output_source.label.contains("volumetric split-cell")
            || output_source.label.contains("orthogonal solid cell")
            || output_source.label.contains("full-face adjacent")
            || output_source.label.contains("contained-face adjacent");
        let label_matches_kind = if self.kind.is_arrangement_cell_complex_shortcut() {
            has_exact_boolean_label && has_arrangement_label
        } else {
            has_exact_boolean_label
        };
        if output_source.source != MeshSource::Exact
            || output_source.approximation != ApproximationPolicy::ExactOnly
            || !label_matches_kind
        {
            return Err(ExactEvidenceValidationError::InvalidOutputMeshProvenance);
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
            ExactBooleanResultKind::OpenSurfaceArrangement { operation } => {
                let Some(selection) = operation.open_surface_region_selection() else {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                };
                Some(selection)
            }
            _ => None,
        };
        let Some(selection) = selection else {
            return Ok(());
        };

        if self.assembly.triangles.iter().any(|triangle| {
            !matches!(
                (selection, triangle.source_side),
                (ExactRegionSelection::KeepAll, _)
                    | (ExactRegionSelection::KeepLeft, MeshSide::Left)
            )
        }) {
            return Err(ExactEvidenceValidationError::SelectedRegionAssemblyViolatesSelection);
        }
        validate_selected_region_assembly_covers_selection(
            selection,
            &self.triangulations,
            &self.assembly,
        )?;

        Ok(())
    }

    fn validate_retained_region_and_volumetric_facts(
        &self,
        retains_region_artifacts: bool,
        retains_volumetric_artifacts: bool,
    ) -> Result<(), ExactEvidenceValidationError> {
        let mut unique_classifications = BTreeSet::new();
        for classification in &self.region_classifications {
            classification
                .validate()
                .map_err(ExactEvidenceValidationError::InvalidRegionClassification)?;
            let classification_key = (
                classification.region_side,
                classification.region_face,
                classification.plane_side,
                classification.plane_face,
            );
            // The exact state cannot retain the same region/plane side fact
            // twice and still be a coherent winding boundary.
            if !unique_classifications.insert(classification_key) {
                return Err(ExactEvidenceValidationError::DuplicateRegionClassification);
            }
            if retains_region_artifacts
                && (!classification.all_proof_producing()
                    || matches!(classification.relation, FaceRegionPlaneRelation::Unknown))
            {
                return Err(ExactEvidenceValidationError::RegionClassificationNotProofProducing);
            }
        }
        let mut unique_triangulations = BTreeSet::new();
        for triangulation in &self.triangulations {
            triangulation
                .validate()
                .map_err(|_| ExactEvidenceValidationError::InvalidTriangulation)?;
            let triangulation_key = (triangulation.side, triangulation.face);
            // Each triangulation is the exact image of one retained
            // auditable object; duplicating it would make output assembly
            // provenance ambiguous even if the triangle soup still validates.
            if !unique_triangulations.insert(triangulation_key) {
                return Err(ExactEvidenceValidationError::DuplicateRegionTriangulation);
            }
        }
        let mut unique_volumetric_classifications = BTreeSet::new();
        for classification in &self.volumetric_classifications {
            classification
                .validate()
                .map_err(ExactEvidenceValidationError::InvalidVolumetricClassification)?;
            let classification_key = (
                classification.region_side,
                classification.region_face,
                classification.triangle,
            );
            if !unique_volumetric_classifications.insert(classification_key) {
                return Err(ExactEvidenceValidationError::DuplicateRegionClassification);
            }
            if retains_volumetric_artifacts
                && !matches!(
                    classification.relation,
                    ExactVolumetricRegionRelation::Inside
                        | ExactVolumetricRegionRelation::Outside
                        | ExactVolumetricRegionRelation::Boundary
                )
            {
                return Err(ExactEvidenceValidationError::VolumetricClassificationNotDecided);
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
            return Err(ExactEvidenceValidationError::UnclassifiedRegionTriangulation);
        }
        if retains_region_artifacts
            && self.region_classifications.iter().any(|classification| {
                !self.triangulations.iter().any(|triangulation| {
                    triangulation.side == classification.region_side
                        && triangulation.face == classification.region_face
                })
            })
        {
            return Err(ExactEvidenceValidationError::OrphanedRegionClassification);
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
            return Err(ExactEvidenceValidationError::UnclassifiedVolumetricTriangulation);
        }
        if retains_volumetric_artifacts
            && self
                .volumetric_classifications
                .iter()
                .any(|classification| {
                    !self.triangulations.iter().any(|triangulation| {
                        classification.region_side == triangulation.side
                            && classification.region_face == triangulation.face
                            && triangulation.triangles.chunks_exact(3).any(|triangle| {
                                classification.triangle == [triangle[0], triangle[1], triangle[2]]
                            })
                    })
                })
        {
            return Err(ExactEvidenceValidationError::OrphanedVolumetricClassification);
        }
        if retains_volumetric_artifacts {
            let mut classifications = self.volumetric_classifications.iter();
            for triangulation in &self.triangulations {
                for triangle in triangulation.triangles.chunks_exact(3) {
                    let Some(classification) = classifications.next() else {
                        return Err(
                            ExactEvidenceValidationError::VolumetricClassificationOrderMismatch,
                        );
                    };
                    if (
                        classification.region_side,
                        classification.region_face,
                        classification.triangle,
                    ) != (
                        triangulation.side,
                        triangulation.face,
                        [triangle[0], triangle[1], triangle[2]],
                    ) {
                        return Err(
                            ExactEvidenceValidationError::VolumetricClassificationOrderMismatch,
                        );
                    }
                    classification
                        .validate_representatives_against_triangulation(triangulation)
                        .map_err(ExactEvidenceValidationError::InvalidVolumetricClassification)?;
                }
            }
            if classifications.next().is_some() {
                return Err(ExactEvidenceValidationError::VolumetricClassificationOrderMismatch);
            }
        }
        Ok(())
    }

    fn validate_arrangement_cell_complex_gate_reports(
        &self,
    ) -> Result<(), ExactEvidenceValidationError> {
        if self.topology_assembly_report.is_none() && self.region_ownership_report.is_none() {
            return Ok(());
        }
        let operation = self
            .kind
            .arrangement_cell_complex_operation()
            .ok_or(ExactEvidenceValidationError::StatusEvidenceMismatch)?;
        let topology = self
            .topology_assembly_report
            .as_ref()
            .ok_or(ExactEvidenceValidationError::StatusEvidenceMismatch)?;
        let ownership = self
            .region_ownership_report
            .as_ref()
            .ok_or(ExactEvidenceValidationError::StatusEvidenceMismatch)?;
        validate_selected_gate_reports(Some(topology), Some(ownership), operation)
            .map_err(|_| ExactEvidenceValidationError::StatusEvidenceMismatch)?;
        if !ownership.matches_topology_gate_report(topology)
            || self.triangulations.len() > topology.arrangement_face_cells
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
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
        left: &Mesh,
        right: &Mesh,
    ) -> Result<(), ExactEvidenceValidationError> {
        self.validate()?;
        let mut arrangement_cell_complex_output_replayed = false;
        let mut source_replay = ExactBooleanSourceReplay::new(left, right);
        let validation = self.mesh.validation_mode();
        let result_request = self.kind.request(validation);
        if self.topology_assembly_report.is_some() || self.region_ownership_report.is_some() {
            let graph = source_replay.validated_graph()?;
            self.validate_arrangement_cell_complex_gate_reports_against_sources(
                graph, left, right,
            )?;
        }
        let retained_result_replay = match self.kind {
            ExactBooleanResultKind::SelectedRegions { selection } => {
                let graph = source_replay.validated_graph()?;
                Some(
                    replay_selected_region_boolean_result_from_graph(
                        graph, left, right, selection, validation,
                    )
                    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?,
                )
            }
            ExactBooleanResultKind::OpenSurfaceArrangement { operation } => {
                let graph = source_replay.validated_graph()?;
                Some(
                    super::open_surface_arrangement_result_from_graph(
                        graph, left, right, operation, validation,
                    )
                    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
                    .filter(|replay| {
                        matches!(
                            replay.kind,
                            ExactBooleanResultKind::OpenSurfaceArrangement {
                                operation: result_operation,
                            } if result_operation == operation
                        ) && replay.mesh.validation_mode() == validation
                    })
                    .ok_or(ExactEvidenceValidationError::SourceReplayMismatch)?,
                )
            }
            ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. } => {
                let graph = source_replay.validated_graph()?;
                let mut replay = replay_generic_arrangement_cell_complex_result(
                    graph,
                    left,
                    right,
                    result_request,
                )
                .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
                .ok_or(ExactEvidenceValidationError::SourceReplayMismatch)?;
                replay.kind = self.kind;
                Some(replay)
            }
            ExactBooleanResultKind::CertifiedShortcut { .. } => None,
        };
        if let Some(replay) = retained_result_replay {
            if !self.matches_retained_replay(&replay)? {
                return Err(ExactEvidenceValidationError::SourceReplayMismatch);
            }
            validate_assembly_source_face_incidence(&self.assembly, left, right)
                .map_err(|_| ExactEvidenceValidationError::OutputSourceReplayMismatch)?;
        }
        if matches!(
            self.kind,
            ExactBooleanResultKind::ArrangementCellComplexMaterialized { .. }
        ) {
            for classification in &self.volumetric_classifications {
                let Some(triangulation) = self.triangulations.iter().find(|triangulation| {
                    classification.region_side == triangulation.side
                        && classification.region_face == triangulation.face
                        && triangulation.triangles.chunks_exact(3).any(|triangle| {
                            classification.triangle == [triangle[0], triangle[1], triangle[2]]
                        })
                }) else {
                    return Err(ExactEvidenceValidationError::OrphanedVolumetricClassification);
                };
                let target = match classification.region_side {
                    MeshSide::Left => right,
                    MeshSide::Right => left,
                };
                classification
                    .validate_against_sources(triangulation, target)
                    .map_err(ExactEvidenceValidationError::InvalidVolumetricClassification)?;
            }
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                shortcut @ (ExactBooleanShortcutKind::EmptyOperand
                | ExactBooleanShortcutKind::BoundsDisjoint
                | ExactBooleanShortcutKind::Identical
                | ExactBooleanShortcutKind::SameSurface
                | ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid
                | ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid),
            ..
        } = self.kind
            && !certified_shortcut_output_matches_sources(
                shortcut,
                result_request,
                &self.mesh,
                &mut source_replay,
            )?
        {
            return Err(ExactEvidenceValidationError::SourceReplayMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut: ExactBooleanShortcutKind::OpenSurfaceDisjoint,
        } = self.kind
        {
            let graph = source_replay.validated_graph()?;
            let report = open_surface_disjoint_report_from_graph(graph, left, right);
            if report.status != ExactOpenSurfaceDisjointStatus::Certified {
                return Err(ExactEvidenceValidationError::SourceReplayMismatch);
            }
            report
                .validate_against_sources(left, right)
                .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
            let expected =
                materialize_open_surface_disjoint_meshes(left, right, operation, validation)
                    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
            if !self.matches_retained_replay(&expected)? {
                return Err(ExactEvidenceValidationError::SourceReplayMismatch);
            }
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            ..
        } = self.kind
            && let Some(matches_output) = arrangement_cell_complex_output_matches_sources(
                result_request,
                &self.mesh,
                &mut source_replay,
            )
            .unwrap_or(None)
        {
            if !matches_output {
                return Err(ExactEvidenceValidationError::SourceReplayMismatch);
            }
            arrangement_cell_complex_output_replayed = true;
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
        } = self.kind
            && !arrangement_cell_complex_output_replayed
            && let Some(replay) =
                boolean_coplanar_mesh_overlay_optional(left, right, operation, validation)
                    .ok()
                    .flatten()
            && self.matches_retained_replay(&replay)?
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
                shortcut,
                operation,
                &self.mesh,
                &mut source_replay,
            )?
        {
            return Err(ExactEvidenceValidationError::SourceReplayMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut:
                shortcut @ (ExactBooleanShortcutKind::ClosedWindingSeparated
                | ExactBooleanShortcutKind::ClosedWindingContainment),
        } = self.kind
            && {
                match certified_closed_winding_relation_from_sources(&mut source_replay)? {
                    Some(relation) => !relation_output_matches_sources(
                        shortcut,
                        ExactBooleanShortcutKind::ClosedWindingSeparated,
                        ExactBooleanShortcutKind::ClosedWindingContainment,
                        operation,
                        relation,
                        &self.mesh,
                        left,
                        right,
                    ),
                    None => true,
                }
            }
        {
            return Err(ExactEvidenceValidationError::SourceReplayMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            operation,
            shortcut:
                shortcut @ (ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion
                | ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection
                | ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference),
        } = self.kind
            && {
                let matches_sources = if let Some(true) =
                    arrangement_cell_complex_output_matches_sources(
                        result_request,
                        &self.mesh,
                        &mut source_replay,
                    )? {
                    false
                } else if !closed_boundary_touching_sources_match(shortcut, &mut source_replay)? {
                    false
                } else {
                    match (shortcut, operation) {
                        (
                            ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion,
                            ExactBooleanOperation::Union,
                        ) => concatenated_mesh_output_matches(&self.mesh, left, right, false),
                        (
                            ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection,
                            ExactBooleanOperation::Intersection,
                        ) => mesh_output_is_empty(&self.mesh),
                        (
                            ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
                            ExactBooleanOperation::Difference,
                        ) => mesh_output_matches(&self.mesh, left),
                        _ => false,
                    }
                };
                !matches_sources
            }
        {
            return Err(ExactEvidenceValidationError::SourceReplayMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut { shortcut, .. } = self.kind
            && shortcut != ExactBooleanShortcutKind::ArrangementCellComplex
            && !certified_shortcut_sources_match(shortcut, result_request, &mut source_replay)?
        {
            return Err(ExactEvidenceValidationError::SourceReplayMismatch);
        }
        if let ExactBooleanResultKind::CertifiedShortcut {
            shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            ..
        } = self.kind
            && !arrangement_cell_complex_output_replayed
        {
            return Err(ExactEvidenceValidationError::SourceReplayMismatch);
        }
        Ok(())
    }

    pub(crate) fn validate_arrangement_cell_complex_gate_reports_against_sources(
        &self,
        graph: &ExactIntersectionGraph,
        left: &Mesh,
        right: &Mesh,
    ) -> Result<(), ExactEvidenceValidationError> {
        if self.topology_assembly_report.is_none() && self.region_ownership_report.is_none() {
            return Ok(());
        }
        let arrangement =
            ExactArrangement3d::from_source_certified_intersection_graph_with_regularization_mode(
                graph.clone(),
                left,
                right,
                ExactRegularizationMode::REGULARIZED_SOLID,
            )
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
        self.validate_arrangement_cell_complex_gate_reports_against_arrangement(
            &arrangement,
            left,
            right,
            self.kind.arrangement_cell_complex_operation(),
        )
    }

    pub(crate) fn validate_arrangement_cell_complex_gate_reports_against_arrangement(
        &self,
        arrangement: &ExactArrangement3d,
        left: &Mesh,
        right: &Mesh,
        operation: Option<ExactBooleanOperation>,
    ) -> Result<(), ExactEvidenceValidationError> {
        if self.topology_assembly_report.is_none() && self.region_ownership_report.is_none() {
            return Ok(());
        }
        let replay_topology = arrangement.topology_assembly_report_with_regularization_mode(
            left,
            right,
            ExactRegularizationMode::REGULARIZED_SOLID,
        );
        if self.topology_assembly_report.as_ref() != Some(&replay_topology) {
            return Err(ExactEvidenceValidationError::SourceReplayMismatch);
        }
        let ownership_mode = arrangement_cell_complex_labeling_mode(
            arrangement,
            operation,
            ExactRegularizationMode::REGULARIZED_SOLID,
        );
        let replay_ownership = arrangement
            .region_ownership_report_with_regularization_mode(left, right, ownership_mode)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
        if self.region_ownership_report.as_ref() != Some(&replay_ownership) {
            return Err(ExactEvidenceValidationError::SourceReplayMismatch);
        }
        Ok(())
    }

    /// Validate this result against the operation and policies that produced it.
    ///
    /// [`Self::validate_against_sources`] audits retained source provenance,
    /// including arrangement-cell-complex gate reports when present. This
    /// stronger replay accepts a retained certified arrangement attempt only
    /// when its materialized mesh and gate reports match the result, otherwise
    /// it recomputes the named exact boolean entry point for the same
    /// operands, operation, and validation mode. That closes the shortcut
    /// replay gap: a certified output mesh cannot be relabeled as a different
    /// named operation or shortcut kind while still passing the source audit.
    pub(crate) fn validate_request_against_sources_with_retained_attempt(
        &self,
        left: &Mesh,
        right: &Mesh,
        request: ExactBooleanRequest,
        retained_arrangement_attempt: Option<&ExactArrangementBooleanAttempt>,
    ) -> Result<(), ExactEvidenceValidationError> {
        if !self.kind.matches_request(request) {
            return Err(ExactEvidenceValidationError::SourceReplayMismatch);
        }
        let request_validation_satisfied =
            self.mesh.validation_mode().satisfies(request.validation);
        let arrangement_result = self.kind.arrangement_cell_complex_operation().is_some();
        if arrangement_result
            && request_validation_satisfied
            && let Some(attempt) = retained_arrangement_attempt
            && attempt
                .validate_for_request_regularization_mode(
                    request,
                    ExactRegularizationMode::REGULARIZED_SOLID,
                )
                .is_ok()
            && attempt.materialized_arrangement_cell_complex_output()
        {
            self.validate()?;
            if attempt.materialized_generic_arrangement_cell_complex_output() {
                if !self.kind.is_arrangement_cell_complex_shortcut()
                    || self.kind.arrangement_cell_complex_operation() != Some(request.operation)
                    || self.topology_assembly_report.is_some()
                    || self.region_ownership_report.is_some()
                {
                    let simplified = attempt
                        .simplified_cell_complex_with_retained_gate_reports()
                        .ok_or(ExactEvidenceValidationError::SourceReplayMismatch)?;
                    let replay = rematerialize_simplified_arrangement_cell_complex(
                        request, simplified, false,
                    )
                    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
                    .ok_or(ExactEvidenceValidationError::SourceReplayMismatch)?;
                    if !retained_output_mesh_matches(&self.mesh, &replay.mesh)?
                        || self.topology_assembly_report != replay.topology_assembly_report
                        || self.region_ownership_report != replay.region_ownership_report
                    {
                        return Err(ExactEvidenceValidationError::SourceReplayMismatch);
                    }
                }
            } else if attempt.materialized_arrangement_cell_complex_shortcut_output() {
                if let Some((topology, ownership)) = attempt.retained_gate_reports() {
                    if self.topology_assembly_report.as_ref() != Some(topology)
                        || self.region_ownership_report.as_ref() != Some(ownership)
                    {
                        return Err(ExactEvidenceValidationError::SourceReplayMismatch);
                    }
                } else if self.topology_assembly_report.is_some()
                    || self.region_ownership_report.is_some()
                {
                    return Err(ExactEvidenceValidationError::SourceReplayMismatch);
                }
            } else {
                return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
            }
            let attempt_output_matches_mesh = if let Some(facts) = attempt.output_facts.as_ref() {
                self.mesh.vertices().len() == attempt.output_vertices
                    && self.mesh.facts().mesh.face_count == attempt.output_triangles
                    && &self.mesh.facts().mesh == facts
            } else {
                false
            };
            if attempt_output_matches_mesh {
                attempt.validate_against_sources_for_request(left, right, request)?;
                return Ok(());
            }
            return Err(ExactEvidenceValidationError::SourceReplayMismatch);
        }
        if request_validation_satisfied
            && matches!(
                self.kind,
                ExactBooleanResultKind::OpenSurfaceArrangement { .. }
                    | ExactBooleanResultKind::CertifiedShortcut {
                        shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
                        ..
                    }
            )
        {
            self.validate_against_sources(left, right)?;
            return Ok(());
        }
        self.validate()?;
        let replay = materialize_boolean_operation(
            left,
            right,
            request.operation,
            request.validation,
            None,
            None,
        )
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
        if self.matches_retained_replay(&replay)? {
            Ok(())
        } else {
            Err(ExactEvidenceValidationError::SourceReplayMismatch)
        }
    }
}

/// Replayable source-shape facts for exact boolean shortcuts that do not need
/// graph topology.
///
/// These facts deliberately mirror operation evidence shortcut semantics rather than the
/// lower-level bounds helper: an empty operand is certified as empty, not as a
/// bounds-disjoint non-empty pair even when it has no mesh bounds.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg(test)]
pub(crate) struct ExactTrivialBooleanFacts {
    /// The left source has no input triangles.
    pub(super) left_empty: bool,
    /// The right source has no input triangles.
    pub(super) right_empty: bool,
    /// Both sources are non-empty and their exact mesh AABBs are disjoint.
    pub(super) bounds_disjoint: bool,
}

/// Replayable source-shape facts for closed regularized-solid shortcut
/// supports.
///
/// These facts retain the exact mesh-topology predicates used to classify
/// whether an operand contributes closed volume. Empty operands are not
/// represented as lower-dimensional here because the public dispatcher gives
/// them distinct empty-operand provenance before regularized-solid shortcuts.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg(test)]
pub(crate) struct ExactRegularizedSolidBooleanFacts {
    /// The left source is a non-empty closed manifold solid.
    pub(super) left_closed_solid: bool,
    /// The right source is a non-empty closed manifold solid.
    pub(super) right_closed_solid: bool,
    /// The left source is a supported non-empty open manifold surface.
    pub(super) left_open_surface: bool,
    /// The right source is a supported non-empty open manifold surface.
    pub(super) right_open_surface: bool,
}

/// Replayable source facts for closed-convex boolean shortcuts.
#[derive(Clone, Debug, Eq, PartialEq)]
#[cfg(test)]
pub(crate) struct ExactConvexBooleanCapabilityFacts {
    /// Exact closed-convex union can be certified by the shortcut.
    pub(super) can_union: bool,
    /// Exact closed-convex intersection can be certified by the shortcut.
    pub(super) can_intersection: bool,
    /// Exact closed-convex difference can be certified by the shortcut.
    pub(super) can_difference: bool,
}

/// Replayable source facts for arrangement-cell-complex shortcut materializers
/// that cover cases the general arrangement attempt does not consume yet.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ExactArrangementCellComplexShortcutFacts {
    /// Both operands certify as exact retained axis-aligned boxes.
    pub(super) axis_aligned_box_pair: bool,
    /// Axis-aligned orthogonal cell decomposition supports union.
    pub(super) axis_aligned_union: bool,
    /// Axis-aligned orthogonal cell decomposition supports intersection.
    pub(super) axis_aligned_intersection: bool,
    /// Axis-aligned orthogonal cell decomposition supports difference.
    pub(super) axis_aligned_difference: bool,
    /// Affine orthogonal cell decomposition supports union.
    pub(super) affine_union: bool,
    /// Affine orthogonal cell decomposition supports intersection.
    pub(super) affine_intersection: bool,
    /// Affine orthogonal cell decomposition supports difference.
    pub(super) affine_difference: bool,
}

impl ExactArrangementCellComplexShortcutFacts {
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        let axis_aligned_shortcut_supported = self.axis_aligned_union
            || self.axis_aligned_intersection
            || self.axis_aligned_difference;
        let affine_shortcut_supported =
            self.affine_union || self.affine_intersection || self.affine_difference;
        if axis_aligned_shortcut_supported && affine_shortcut_supported {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        Ok(())
    }

    pub(crate) fn checked(self) -> Result<Self, ExactEvidenceValidationError> {
        self.validate()?;
        Ok(self)
    }

    pub(crate) fn validate_against_sources(
        &self,
        left: &Mesh,
        right: &Mesh,
    ) -> Result<(), ExactEvidenceValidationError> {
        self.validate()?;
        if self == &Self::from_sources(left, right) {
            Ok(())
        } else {
            Err(ExactEvidenceValidationError::SourceReplayMismatch)
        }
    }

    pub(crate) fn checked_from_sources(
        left: &Mesh,
        right: &Mesh,
    ) -> Result<Self, ExactEvidenceValidationError> {
        Self::from_sources(left, right).checked()
    }

    pub(crate) fn from_sources(left: &Mesh, right: &Mesh) -> Self {
        Self {
            axis_aligned_box_pair: is_axis_aligned_box(left) && is_axis_aligned_box(right),
            axis_aligned_union: axis_aligned_orthogonal_solid_cell_selected_count(
                left,
                right,
                AxisAlignedOrthogonalSolidOperation::Union,
            )
            .is_some(),
            axis_aligned_intersection: axis_aligned_orthogonal_solid_cell_selected_count(
                left,
                right,
                AxisAlignedOrthogonalSolidOperation::Intersection,
            )
            .is_some(),
            axis_aligned_difference: axis_aligned_orthogonal_solid_cell_selected_count(
                left,
                right,
                AxisAlignedOrthogonalSolidOperation::Difference,
            )
            .is_some(),
            affine_union: matches!(
                affine_orthogonal_solid_cell_selected_count(
                    left,
                    right,
                    AffineOrthogonalSolidOperation::Union,
                ),
                Some(_)
            ),
            affine_intersection: matches!(
                affine_orthogonal_solid_cell_selected_count(
                    left,
                    right,
                    AffineOrthogonalSolidOperation::Intersection,
                ),
                Some(_)
            ),
            affine_difference: matches!(
                affine_orthogonal_solid_cell_selected_count(
                    left,
                    right,
                    AffineOrthogonalSolidOperation::Difference,
                ),
                Some(_)
            ),
        }
    }

    pub(crate) fn materializes_operation(&self, operation: ExactBooleanOperation) -> bool {
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

fn certified_shortcut_sources_match(
    shortcut: ExactBooleanShortcutKind,
    request: ExactBooleanRequest,
    source_replay: &mut ExactBooleanSourceReplay<'_>,
) -> Result<bool, ExactEvidenceValidationError> {
    let operation = request.operation;
    if !shortcut_operation_matches(shortcut, operation) {
        return Ok(false);
    }
    let left = source_replay.left;
    let right = source_replay.right;
    match shortcut {
        ExactBooleanShortcutKind::EmptyOperand => {
            Ok(left.facts().mesh.face_count == 0 || right.facts().mesh.face_count == 0)
        }
        ExactBooleanShortcutKind::BoundsDisjoint => {
            Ok(meshes_are_certified_bounds_disjoint(left, right))
        }
        ExactBooleanShortcutKind::Identical => Ok(identical_mesh_report_from_sources(left, right)
            .status
            == ExactIdenticalMeshStatus::Certified),
        ExactBooleanShortcutKind::SameSurface => {
            let report = same_surface_report_from_sources(left, right);
            report.validate()?;
            Ok(report.status == ExactSameSurfaceStatus::Certified)
        }
        ExactBooleanShortcutKind::OpenSurfaceDisjoint => {
            let graph = source_replay.validated_graph()?;
            let report = open_surface_disjoint_report_from_graph(graph, left, right);
            report.validate()?;
            Ok(report.status == ExactOpenSurfaceDisjointStatus::Certified)
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
            closed_boundary_touching_sources_match(shortcut, source_replay)
        }
        ExactBooleanShortcutKind::ClosedWindingSeparated
        | ExactBooleanShortcutKind::ClosedWindingContainment => {
            let Some(relation) = certified_closed_winding_relation_from_sources(source_replay)?
            else {
                return Ok(false);
            };
            Ok(matches!(
                (shortcut, relation),
                (
                    ExactBooleanShortcutKind::ClosedWindingSeparated,
                    ReportMeshRelation::Separated
                ) | (
                    ExactBooleanShortcutKind::ClosedWindingContainment,
                    ReportMeshRelation::LeftInsideRight
                        | ReportMeshRelation::RightInsideLeft { .. }
                )
            ))
        }
        ExactBooleanShortcutKind::ConvexContainment
        | ExactBooleanShortcutKind::ConvexUnion
        | ExactBooleanShortcutKind::ConvexIntersection
        | ExactBooleanShortcutKind::ConvexDifference
        | ExactBooleanShortcutKind::ConvexSeparated => {
            convex_shortcut_sources_match(shortcut, operation, source_replay)
        }
        ExactBooleanShortcutKind::ArrangementCellComplex => {
            arrangement_cell_complex_sources_match(request, source_replay)
        }
    }
}

fn certified_shortcut_output_matches_sources(
    shortcut: ExactBooleanShortcutKind,
    request: ExactBooleanRequest,
    mesh: &Mesh,
    source_replay: &mut ExactBooleanSourceReplay<'_>,
) -> Result<bool, ExactEvidenceValidationError> {
    let operation = request.operation;
    let validation = request.validation;
    if !certified_shortcut_sources_match(shortcut, request, source_replay)? {
        return Ok(false);
    }
    let left = source_replay.left;
    let right = source_replay.right;
    Ok(match shortcut {
        ExactBooleanShortcutKind::EmptyOperand => {
            if left.facts().mesh.face_count != 0 && right.facts().mesh.face_count != 0 {
                false
            } else {
                match operation {
                    ExactBooleanOperation::Union
                        if validation == MeshValidationMode::CLOSED
                            && lower_dimensional_regularized_sources(left, right) =>
                    {
                        mesh_output_is_empty(mesh)
                    }
                    ExactBooleanOperation::Union => {
                        concatenated_mesh_output_matches(mesh, left, right, false)
                    }
                    ExactBooleanOperation::Intersection => mesh_output_is_empty(mesh),
                    ExactBooleanOperation::Difference if left.facts().mesh.face_count == 0 => {
                        mesh_output_is_empty(mesh)
                    }
                    ExactBooleanOperation::Difference
                        if validation == MeshValidationMode::CLOSED
                            && right.facts().mesh.face_count == 0
                            && closed_regularized_operand_kind(left)
                                == Some(ClosedRegularizedOperandKind::LowerDimensional) =>
                    {
                        mesh_output_is_empty(mesh)
                    }
                    ExactBooleanOperation::Difference => mesh_output_matches(mesh, left),
                    ExactBooleanOperation::SelectedRegions(_) => false,
                }
            }
        }
        ExactBooleanShortcutKind::BoundsDisjoint => {
            if left.facts().mesh.face_count == 0
                || right.facts().mesh.face_count == 0
                || (validation == MeshValidationMode::CLOSED
                    && (lower_dimensional_regularized_sources(left, right)
                        || mixed_dimensional_regularized_sources(left, right)))
            {
                false
            } else {
                match operation {
                    ExactBooleanOperation::Union => {
                        concatenated_mesh_output_matches(mesh, left, right, false)
                    }
                    ExactBooleanOperation::Intersection => mesh_output_is_empty(mesh),
                    ExactBooleanOperation::Difference => mesh_output_matches(mesh, left),
                    ExactBooleanOperation::SelectedRegions(_) => false,
                }
            }
        }
        ExactBooleanShortcutKind::Identical => {
            identical_output_matches_sources(operation, validation, mesh, left, right)
        }
        ExactBooleanShortcutKind::SameSurface => {
            identical_mesh_report_from_sources(left, right).status
                != ExactIdenticalMeshStatus::Certified
                && identical_output_matches_sources(operation, validation, mesh, left, right)
        }
        ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid => {
            if validation != MeshValidationMode::CLOSED
                && meshes_are_certified_bounds_disjoint(left, right)
            {
                false
            } else {
                let left_closed = closed_regularized_operand_kind(left)
                    == Some(ClosedRegularizedOperandKind::ClosedSolid);
                let right_closed = closed_regularized_operand_kind(right)
                    == Some(ClosedRegularizedOperandKind::ClosedSolid);
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
        }
        ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid => {
            if validation == MeshValidationMode::CLOSED
                && operation == ExactBooleanOperation::Intersection
                && lower_dimensional_regularized_sources(left, right)
            {
                let graph = source_replay.validated_graph()?;
                if !graph.has_unknowns()
                    && !graph.face_pairs.is_empty()
                    && mesh_output_is_empty(mesh)
                {
                    return Ok(false);
                }
            }
            if let Some(true) =
                arrangement_cell_complex_output_matches_sources(request, mesh, source_replay)?
            {
                return Ok(false);
            }
            validation == MeshValidationMode::CLOSED
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

fn identical_output_matches_sources(
    operation: ExactBooleanOperation,
    validation: MeshValidationMode,
    mesh: &Mesh,
    left: &Mesh,
    right: &Mesh,
) -> bool {
    if matches!(
        (
            closed_regularized_operand_kind(left),
            closed_regularized_operand_kind(right),
        ),
        (
            Some(ClosedRegularizedOperandKind::ClosedSolid),
            Some(ClosedRegularizedOperandKind::ClosedSolid)
        )
    ) || (validation == MeshValidationMode::CLOSED
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

fn retained_output_mesh_matches(
    left: &Mesh,
    right: &Mesh,
) -> Result<bool, ExactEvidenceValidationError> {
    Ok(mesh_output_matches_result(left, right)?
        && left.bounds() == right.bounds()
        && left.facts().mesh == right.facts().mesh
        && left.validation_mode() == right.validation_mode()
        && left.provenance() == right.provenance())
}

/// Replayable certification bundle for an exact boolean request.
///
/// These reports are intentionally redundant with the operation evidence summary. The
/// summary is the scheduling decision, while this bundle keeps the Yap-style
/// exact facts that explain which stage certified, blocked, or declined the
/// requested operation.
#[derive(Clone, Debug, PartialEq)]
#[cfg(test)]
pub(crate) struct ExactBooleanCertificationSet {
    /// Source-shape facts used by trivial shortcut supports.
    pub(super) trivial: ExactTrivialBooleanFacts,
    /// Source-shape facts used by closed regularized-solid shortcut supports.
    pub(super) regularized_solid: ExactRegularizedSolidBooleanFacts,
    /// Exact graph refinement status.
    pub(super) refinement: ExactRefinementReport,
    /// Boundary-contact mode status.
    pub(super) boundary_touching: ExactBoundaryTouchingReport,
    /// Open-surface disjointness shortcut status.
    pub(super) open_surface_disjoint: ExactOpenSurfaceDisjointReport,
    /// Adjacent closed-solid union completion shortcut status.
    pub(super) adjacent_union_completion: ExactAdjacentUnionCompletionReport,
    /// Identical-mesh shortcut status.
    pub(super) identical: ExactIdenticalMeshReport,
    /// Same-surface shortcut status.
    pub(super) same_surface: ExactSameSurfaceReport,
    /// Left vertices classified against the right closed mesh.
    pub(super) closed_winding_left_in_right: ClosedMeshWindingMeshReport,
    /// Right vertices classified against the left closed mesh.
    pub(super) closed_winding_right_in_left: ClosedMeshWindingMeshReport,
    /// Left vertices classified against the right convex solid.
    pub(super) convex_left_in_right: ConvexSolidMeshClassification,
    /// Right vertices classified against the left convex solid.
    pub(super) convex_right_in_left: ConvexSolidMeshClassification,
    /// Closed-convex shortcut capabilities.
    pub(super) convex_capabilities: ExactConvexBooleanCapabilityFacts,
    /// Arrangement-cell shortcut capabilities that cover cases not yet
    /// consumed by the full arrangement attempt report.
    pub(super) arrangement_cell_complex_shortcuts: ExactArrangementCellComplexShortcutFacts,
    /// Planar-arrangement evidence for coplanar surface output.
    pub(super) planar_arrangement: ExactPlanarArrangementReport,
    /// Winding/inside-outside evidence for named volumetric output.
    pub(super) winding_evidence: ExactWindingEvidenceReport,
    /// Volumetric boundary closure evidence, when meaningful for the request.
    pub(super) volumetric_boundary_closure: Option<ExactVolumetricBoundaryClosureReport>,
    /// Arrangement/cell-complex materialization attempt.
    pub(super) arrangement_attempt: Option<ExactArrangementBooleanAttempt>,
}

fn validate_shortcut_output_shape(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
    mesh: &Mesh,
) -> Result<(), ExactEvidenceValidationError> {
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
        return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
    }
    if requires_closed_solid_output
        && closed_regularized_operand_kind(mesh) != Some(ClosedRegularizedOperandKind::ClosedSolid)
    {
        return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
    }
    if requires_empty_or_closed_solid_output
        && mesh.facts().mesh.face_count != 0
        && closed_regularized_operand_kind(mesh) != Some(ClosedRegularizedOperandKind::ClosedSolid)
    {
        return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
    }
    if requires_lower_dimensional_output
        && closed_regularized_operand_kind(mesh)
            != Some(ClosedRegularizedOperandKind::LowerDimensional)
    {
        return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
    }
    Ok(())
}

fn convex_operation_output_matches_sources(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
    mesh: &Mesh,
    source_replay: &mut ExactBooleanSourceReplay<'_>,
) -> Result<bool, ExactEvidenceValidationError> {
    if !shortcut_operation_matches(shortcut, operation) {
        return Ok(false);
    }
    let left = source_replay.left;
    let right = source_replay.right;
    if matches!(
        shortcut,
        ExactBooleanShortcutKind::ConvexContainment | ExactBooleanShortcutKind::ConvexSeparated
    ) {
        let Some(relation) = certified_convex_relation_from_sources(operation, source_replay)?
        else {
            return Ok(false);
        };
        return Ok(relation_output_matches_sources(
            shortcut,
            ExactBooleanShortcutKind::ConvexSeparated,
            ExactBooleanShortcutKind::ConvexContainment,
            operation,
            relation,
            mesh,
            left,
            right,
        ));
    }
    let Some(replay) = (match shortcut {
        ExactBooleanShortcutKind::ConvexUnion => union_closed_convex_solids(left, right)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            .map(|result| result.mesh),
        ExactBooleanShortcutKind::ConvexIntersection => intersect_closed_convex_solids(left, right)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            .map(|result| result.mesh),
        ExactBooleanShortcutKind::ConvexDifference => subtract_closed_convex_solids(left, right)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            .map(|result| result.mesh),
        _ => unreachable!("only convex operation shortcuts are replayed here"),
    }) else {
        return Ok(false);
    };
    replay
        .validate_retained_state()
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
    Ok(mesh_output_matches(mesh, &replay))
}

fn relation_output_matches_sources(
    shortcut: ExactBooleanShortcutKind,
    separated_shortcut: ExactBooleanShortcutKind,
    containment_shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
    relation: ReportMeshRelation,
    mesh: &Mesh,
    left: &Mesh,
    right: &Mesh,
) -> bool {
    match relation {
        ReportMeshRelation::Separated => {
            if shortcut != separated_shortcut {
                return false;
            }
            match operation {
                ExactBooleanOperation::Union => {
                    concatenated_mesh_output_matches(mesh, left, right, false)
                }
                ExactBooleanOperation::Intersection => mesh_output_is_empty(mesh),
                ExactBooleanOperation::Difference => mesh_output_matches(mesh, left),
                ExactBooleanOperation::SelectedRegions(_) => false,
            }
        }
        ReportMeshRelation::LeftInsideRight => {
            if shortcut != containment_shortcut {
                return false;
            }
            match operation {
                ExactBooleanOperation::Union => mesh_output_matches(mesh, right),
                ExactBooleanOperation::Intersection => mesh_output_matches(mesh, left),
                ExactBooleanOperation::Difference => mesh_output_is_empty(mesh),
                ExactBooleanOperation::SelectedRegions(_) => false,
            }
        }
        ReportMeshRelation::RightInsideLeft {
            difference_keeps_both,
        } => {
            if shortcut != containment_shortcut {
                return false;
            }
            match operation {
                ExactBooleanOperation::Union => mesh_output_matches(mesh, left),
                ExactBooleanOperation::Intersection => mesh_output_matches(mesh, right),
                ExactBooleanOperation::Difference if difference_keeps_both => {
                    concatenated_mesh_output_matches(mesh, left, right, true)
                }
                ExactBooleanOperation::Difference | ExactBooleanOperation::SelectedRegions(_) => {
                    false
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ReportMeshRelation {
    Separated,
    LeftInsideRight,
    RightInsideLeft { difference_keeps_both: bool },
}

fn certified_convex_relation_from_sources(
    operation: ExactBooleanOperation,
    source_replay: &mut ExactBooleanSourceReplay<'_>,
) -> Result<Option<ReportMeshRelation>, ExactEvidenceValidationError> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return Ok(None);
    }
    let left = source_replay.left;
    let right = source_replay.right;
    let graph = source_replay.validated_graph()?;
    if graph.has_unknowns() {
        return Ok(None);
    }

    let left_in_right = classify_mesh_vertices_against_convex_solid_report(left, right);
    left_in_right
        .validate_against_sources(left, right)
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
    let right_in_left = classify_mesh_vertices_against_convex_solid_report(right, left);
    right_in_left
        .validate_against_sources(right, left)
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;

    if graph.face_pairs.is_empty() {
        return Ok(match (left_in_right.relation, right_in_left.relation) {
            (ConvexSolidMeshRelation::StrictlyInside, _) => {
                Some(ReportMeshRelation::LeftInsideRight)
            }
            (_, ConvexSolidMeshRelation::StrictlyInside) => {
                Some(ReportMeshRelation::RightInsideLeft {
                    difference_keeps_both: true,
                })
            }
            (ConvexSolidMeshRelation::Outside, ConvexSolidMeshRelation::Outside) => {
                Some(ReportMeshRelation::Separated)
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
            Some(ReportMeshRelation::LeftInsideRight)
        }
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
            if right_boundary_inside_left =>
        {
            Some(ReportMeshRelation::RightInsideLeft {
                difference_keeps_both: false,
            })
        }
        ExactBooleanOperation::Difference if left_boundary_inside_right => {
            Some(ReportMeshRelation::LeftInsideRight)
        }
        _ => None,
    })
}

fn mesh_output_matches(left: &Mesh, right: &Mesh) -> bool {
    mesh_output_matches_result(left, right).unwrap_or(false)
}

fn mesh_output_matches_result(
    left: &Mesh,
    right: &Mesh,
) -> Result<bool, ExactEvidenceValidationError> {
    if left.vertices().len() != right.vertices().len() || !retained_face_rows_equal(left, right) {
        return Ok(false);
    }
    for (left, right) in left.vertices().iter().zip(right.vertices()) {
        match point3_exact_equal(left, right) {
            Some(true) => {}
            Some(false) => return Ok(false),
            None => return Err(ExactEvidenceValidationError::SourceReplayMismatch),
        }
    }
    Ok(true)
}

fn mesh_output_is_empty(mesh: &Mesh) -> bool {
    mesh.vertices().is_empty() && mesh.facts().mesh.face_count == 0
}

fn concatenated_mesh_output_matches(
    mesh: &Mesh,
    left: &Mesh,
    right: &Mesh,
    reverse_right: bool,
) -> bool {
    concatenated_mesh_output_matches_result(mesh, left, right, reverse_right).unwrap_or(false)
}

fn concatenated_mesh_output_matches_result(
    mesh: &Mesh,
    left: &Mesh,
    right: &Mesh,
    reverse_right: bool,
) -> Result<bool, ExactEvidenceValidationError> {
    if mesh.vertices().len() != left.vertices().len() + right.vertices().len()
        || mesh.facts().mesh.face_count
            != left.facts().mesh.face_count + right.facts().mesh.face_count
        || mesh.facts().faces.len() < mesh.facts().mesh.face_count
        || left.facts().faces.len() < left.facts().mesh.face_count
        || right.facts().faces.len() < right.facts().mesh.face_count
    {
        return Ok(false);
    }
    for (candidate, expected) in mesh
        .vertices()
        .iter()
        .take(left.vertices().len())
        .zip(left.vertices())
    {
        match point3_exact_equal(candidate, expected) {
            Some(true) => {}
            Some(false) => return Ok(false),
            None => return Err(ExactEvidenceValidationError::SourceReplayMismatch),
        }
    }
    for (candidate, expected) in mesh
        .vertices()
        .iter()
        .skip(left.vertices().len())
        .zip(right.vertices())
    {
        match point3_exact_equal(candidate, expected) {
            Some(true) => {}
            Some(false) => return Ok(false),
            None => return Err(ExactEvidenceValidationError::SourceReplayMismatch),
        }
    }
    let left_face_count = left.facts().mesh.face_count;
    let mesh_faces = retained_face_rows(mesh).collect::<Vec<_>>();
    if mesh_faces[..left_face_count]
        .iter()
        .copied()
        .ne(retained_face_rows(left))
    {
        return Ok(false);
    }
    let right_offset = left.vertices().len();
    Ok(mesh_faces[left_face_count..]
        .iter()
        .copied()
        .zip(retained_face_rows(right))
        .all(|(candidate, expected)| {
            let [a, b, c] = expected;
            let expected = if reverse_right {
                [a + right_offset, c + right_offset, b + right_offset]
            } else {
                [a + right_offset, b + right_offset, c + right_offset]
            };
            candidate == expected
        }))
}

fn shortcut_operation_matches(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
) -> bool {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        return false;
    }
    match shortcut.required_named_operation() {
        Some(required) => required == operation,
        None => true,
    }
}

pub(crate) fn meshes_are_certified_bounds_disjoint(left: &Mesh, right: &Mesh) -> bool {
    if left.validate_retained_bounds_certificate().is_err()
        || right.validate_retained_bounds_certificate().is_err()
    {
        return false;
    }
    let (Some(left_bounds), Some(right_bounds)) =
        (left.bounds().mesh.as_ref(), right.bounds().mesh.as_ref())
    else {
        return left.facts().mesh.face_count == 0 || right.facts().mesh.face_count == 0;
    };
    classify_aabb3_intersection(
        &left_bounds.min,
        &left_bounds.max,
        &right_bounds.min,
        &right_bounds.max,
    )
    .value()
        == Some(Aabb3Intersection::Disjoint)
}

pub(crate) fn certified_convex_operation_shortcut_support(
    left: &Mesh,
    right: &Mesh,
    operation: ExactBooleanOperation,
) -> Option<ExactBooleanSupport> {
    let support = operation.convex_operation_support()?;
    if boolean_convex_meshes_optional(left, right, operation, MeshValidationMode::CLOSED)
        .ok()
        .flatten()
        .is_some()
    {
        Some(support)
    } else {
        None
    }
}

fn mixed_dimensional_regularized_sources(left: &Mesh, right: &Mesh) -> bool {
    matches!(
        (
            closed_regularized_operand_kind(left),
            closed_regularized_operand_kind(right),
        ),
        (
            Some(ClosedRegularizedOperandKind::ClosedSolid),
            Some(ClosedRegularizedOperandKind::LowerDimensional)
        ) | (
            Some(ClosedRegularizedOperandKind::LowerDimensional),
            Some(ClosedRegularizedOperandKind::ClosedSolid)
        )
    )
}

fn lower_dimensional_regularized_sources(left: &Mesh, right: &Mesh) -> bool {
    matches!(
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

fn closed_boundary_touching_sources_match(
    shortcut: ExactBooleanShortcutKind,
    source_replay: &mut ExactBooleanSourceReplay<'_>,
) -> Result<bool, ExactEvidenceValidationError> {
    let left = source_replay.left;
    let right = source_replay.right;
    if closed_regularized_operand_kind(left) != Some(ClosedRegularizedOperandKind::ClosedSolid)
        || closed_regularized_operand_kind(right) != Some(ClosedRegularizedOperandKind::ClosedSolid)
    {
        return Ok(false);
    }
    let graph = source_replay.validated_graph()?;
    let report = boundary_touching_report_from_graph(graph, left, right)
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
    report.validate()?;
    if report.status != ExactBoundaryTouchingStatus::Certified {
        if matches!(
            shortcut,
            ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection
                | ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference
        ) {
            let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right)
                .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
            evidence
                .validate()
                .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
            return Ok(matches!(
                evidence.obstacle,
                CoplanarVolumetricCellObstacle::BoundaryOnlyContact
            ) && evidence.positive_area_coplanar_overlapping_pairs != 0);
        }
        return Ok(false);
    }
    if shortcut == ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion
        && report.blocker.coplanar_overlapping_pairs != 0
    {
        let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
        evidence
            .validate()
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
        if evidence.positive_area_coplanar_overlapping_pairs != 0 {
            return Ok(false);
        }
    }
    Ok(true)
}

fn certified_closed_winding_relation_from_sources(
    source_replay: &mut ExactBooleanSourceReplay<'_>,
) -> Result<Option<ReportMeshRelation>, ExactEvidenceValidationError> {
    let left = source_replay.left;
    let right = source_replay.right;
    if closed_regularized_operand_kind(left) != Some(ClosedRegularizedOperandKind::ClosedSolid)
        || closed_regularized_operand_kind(right) != Some(ClosedRegularizedOperandKind::ClosedSolid)
    {
        return Ok(None);
    }
    let graph = source_replay.validated_graph()?;
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
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
    let right_in_left = classify_mesh_vertices_against_closed_mesh_winding_report(right, left);
    right_in_left
        .validate_against_sources(right, left)
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;

    Ok(match (left_in_right.relation, right_in_left.relation) {
        (ClosedMeshWindingMeshRelation::Outside, ClosedMeshWindingMeshRelation::Outside) => {
            Some(ReportMeshRelation::Separated)
        }
        (ClosedMeshWindingMeshRelation::StrictlyInside, _) => {
            Some(ReportMeshRelation::LeftInsideRight)
        }
        (_, ClosedMeshWindingMeshRelation::StrictlyInside) => {
            Some(ReportMeshRelation::RightInsideLeft {
                difference_keeps_both: true,
            })
        }
        _ => None,
    })
}

fn convex_shortcut_sources_match(
    shortcut: ExactBooleanShortcutKind,
    operation: ExactBooleanOperation,
    source_replay: &mut ExactBooleanSourceReplay<'_>,
) -> Result<bool, ExactEvidenceValidationError> {
    let left = source_replay.left;
    let right = source_replay.right;
    Ok(match shortcut {
        ExactBooleanShortcutKind::ConvexUnion => union_closed_convex_solids(left, right)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            .is_some(),
        ExactBooleanShortcutKind::ConvexIntersection => intersect_closed_convex_solids(left, right)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            .is_some(),
        ExactBooleanShortcutKind::ConvexDifference => subtract_closed_convex_solids(left, right)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            .is_some(),
        ExactBooleanShortcutKind::ConvexContainment | ExactBooleanShortcutKind::ConvexSeparated => {
            let Some(relation) = certified_convex_relation_from_sources(operation, source_replay)?
            else {
                return Ok(false);
            };
            matches!(
                (shortcut, relation),
                (
                    ExactBooleanShortcutKind::ConvexSeparated,
                    ReportMeshRelation::Separated
                ) | (
                    ExactBooleanShortcutKind::ConvexContainment,
                    ReportMeshRelation::LeftInsideRight
                        | ReportMeshRelation::RightInsideLeft { .. }
                )
            )
        }
        _ => unreachable!("only convex shortcuts are replayed here"),
    })
}

fn arrangement_cell_complex_sources_match(
    request: ExactBooleanRequest,
    source_replay: &mut ExactBooleanSourceReplay<'_>,
) -> Result<bool, ExactEvidenceValidationError> {
    let operation = request.operation;
    let validation = request.validation;
    let left = source_replay.left;
    let right = source_replay.right;
    if validation == MeshValidationMode::CLOSED
        && lower_dimensional_regularized_sources(left, right)
    {
        return Ok(true);
    }
    let graph = source_replay.validated_graph()?;
    if operation == ExactBooleanOperation::Union {
        let (report, _) = adjacent_union_completion_certification_from_graph(
            &graph, left, right, operation, None,
        )
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
        report.validate()?;
        if matches!(
            report.status,
            ExactAdjacentUnionCompletionStatus::CertifiedFullFace
                | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
        ) {
            return Ok(true);
        }
    }
    if graph.has_unknowns() {
        return Ok(false);
    }
    if operation == ExactBooleanOperation::Union {
        let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(&graph, left, right)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
        evidence
            .validate()
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
        if matches!(
            evidence.obstacle,
            CoplanarVolumetricCellObstacle::BoundaryOnlyContact
        ) && evidence.positive_area_coplanar_overlapping_pairs != 0
        {
            return Ok(true);
        }
    }
    let shortcut_facts =
        ExactArrangementCellComplexShortcutFacts::checked_from_sources(left, right)?;
    let operation_evidence = operation_evidence_for_exact_request_from_graph_with_retained_attempt(
        &graph,
        left,
        right,
        request,
        None,
        &shortcut_facts,
    )
    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
    operation_evidence.validate()?;
    Ok(operation_evidence.support == ExactBooleanSupport::CertifiedArrangementCellComplex)
}

fn arrangement_cell_complex_output_matches_sources(
    request: ExactBooleanRequest,
    mesh: &Mesh,
    source_replay: &mut ExactBooleanSourceReplay<'_>,
) -> Result<Option<bool>, ExactEvidenceValidationError> {
    let operation = request.operation;
    let validation = request.validation;
    let left = source_replay.left;
    let right = source_replay.right;
    let mut retained_mismatch = false;
    let solid_operation = operation.axis_aligned_orthogonal_solid_operation();
    if let Some(solid_operation) = solid_operation
        && let Some(replay) = materialize_axis_aligned_orthogonal_solid_cell_output(
            left,
            right,
            solid_operation,
            "exact arrangement orthogonal solid cell replay",
            validation,
        )
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
    {
        if mesh_output_matches(mesh, &replay) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    let validated_graph = source_replay.validated_graph()?;
    if let Some((replay, closure_report)) =
        materialize_volumetric_coplanar_boundary_closure_output_from_graph(
            &validated_graph,
            left,
            right,
            operation,
            validation,
        )
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
    {
        closure_report.validate()?;
        if mesh_output_matches(mesh, &replay) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if let Some(replay) =
        replay_generic_arrangement_cell_complex_result(validated_graph, left, right, request)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
    {
        if mesh_output_matches(mesh, &replay.mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if let Some(replay) = boolean_coplanar_mesh_overlay_optional(left, right, operation, validation)
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
    {
        if mesh_output_matches(mesh, &replay.mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if let Some((replay, evidence)) =
        materialize_closed_boundary_touching_regularized_boolean_with_evidence_from_graph(
            &validated_graph,
            left,
            right,
            operation,
            validation,
        )
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
    {
        evidence
            .validate()
            .map_err(|_| ExactEvidenceValidationError::StatusEvidenceMismatch)?;
        if mesh_output_matches(mesh, &replay.mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if let Some((replay, evidence)) =
        materialize_closed_no_volume_overlap_regularized_boolean_with_evidence_from_graph(
            &validated_graph,
            left,
            right,
            operation,
            validation,
        )
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
    {
        evidence
            .validate()
            .map_err(|_| ExactEvidenceValidationError::StatusEvidenceMismatch)?;
        if mesh_output_matches(mesh, &replay.mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if validation == MeshValidationMode::CLOSED
        && !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && lower_dimensional_regularized_sources(left, right)
    {
        if mesh_output_is_empty(mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    let Some(affine_operation) = operation.affine_orthogonal_solid_operation() else {
        return Ok(None);
    };
    if let Some(replay) =
        materialize_affine_orthogonal_solid_operation(left, right, affine_operation, validation)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
    {
        if mesh_output_matches(mesh, &replay.mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    let convex_replay = match operation {
        ExactBooleanOperation::Union => union_closed_convex_solids(left, right)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            .map(|replay| replay.mesh),
        ExactBooleanOperation::Intersection => intersect_closed_convex_solids(left, right)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            .map(|replay| replay.mesh),
        ExactBooleanOperation::Difference => subtract_closed_convex_solids(left, right)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            .map(|replay| replay.mesh),
        ExactBooleanOperation::SelectedRegions(_) => None,
    };
    if let Some(replay) = convex_replay {
        if mesh_output_matches(mesh, &replay) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        && left.facts().mesh.closed_manifold
        && right.facts().mesh.closed_manifold
        && same_surface_report_from_sources(left, right).status == ExactSameSurfaceStatus::Certified
    {
        let replay = boolean_same_surface_meshes(left, operation, validation)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
        if mesh_output_matches(mesh, &replay.mesh) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }

    if operation != ExactBooleanOperation::Union {
        return Ok(retained_mismatch.then_some(false));
    }

    let (adjacent_report, _) = adjacent_union_completion_certification_from_graph(
        &validated_graph,
        left,
        right,
        operation,
        None,
    )
    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
    adjacent_report.validate()?;
    match adjacent_report.status {
        ExactAdjacentUnionCompletionStatus::CertifiedFullFace => {
            let Some(certificate) =
                full_face_adjacent_certificate_from_graph(left, right, &validated_graph)
                    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            else {
                return Ok(retained_mismatch.then_some(false));
            };
            let Some(replay) = materialize_full_face_adjacent_union_from_certificate(
                left,
                right,
                &certificate,
                validation,
            )
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            else {
                return Ok(retained_mismatch.then_some(false));
            };
            if mesh_output_matches(mesh, &replay.mesh) {
                return Ok(Some(true));
            }
            retained_mismatch = true;
        }
        ExactAdjacentUnionCompletionStatus::CertifiedContainedFace => {
            let Some(certificate) =
                contained_face_adjacent_certificate_from_graph(left, right, &validated_graph)
                    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
            else {
                return Ok(retained_mismatch.then_some(false));
            };
            let Some(replay) = materialize_contained_face_adjacent_union_from_certificate(
                left,
                right,
                &certificate,
                validation,
            )
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
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
    if adjacent_report.status != ExactAdjacentUnionCompletionStatus::NoAdjacencyCertificate {
        return Ok(retained_mismatch.then_some(false));
    }
    if closed_regularized_operand_kind(left) != Some(ClosedRegularizedOperandKind::ClosedSolid)
        || closed_regularized_operand_kind(right) != Some(ClosedRegularizedOperandKind::ClosedSolid)
    {
        return Ok(retained_mismatch.then_some(false));
    }
    if validated_graph.has_unknowns() || validated_graph.face_pairs.is_empty() {
        return Ok(retained_mismatch.then_some(false));
    }
    let evidence = CoplanarVolumetricCellEvidenceReport::from_graph(validated_graph, left, right)
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
    evidence
        .validate()
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
    if matches!(
        evidence.obstacle,
        CoplanarVolumetricCellObstacle::BoundaryOnlyContact
    ) && evidence.positive_area_coplanar_overlapping_pairs != 0
    {
        if concatenated_mesh_output_matches(mesh, left, right, false) {
            return Ok(Some(true));
        }
        retained_mismatch = true;
    }
    Ok(retained_mismatch.then_some(false))
}

fn validate_volumetric_materialized_assembly_matches_operation(
    operation: ExactBooleanOperation,
    triangulations: &[FaceRegionTriangulation],
    classifications: &[ExactVolumetricRegionClassification],
    assembly: &ExactBooleanAssemblyPlan,
) -> Result<(), ExactEvidenceValidationError> {
    for triangulation in triangulations {
        for triangle in triangulation.triangles.chunks_exact(3) {
            let triangle = [triangle[0], triangle[1], triangle[2]];
            let expected = volumetric_retention_for_operation(
                operation,
                triangulation,
                triangle,
                classifications,
            );
            let mut retained_source_cells = 0usize;
            for output in &assembly.triangles {
                if output.source_side == triangulation.side
                    && output.source_face == triangulation.face
                    && output_triangle_matches_triangulated_cell(
                        output,
                        assembly,
                        triangulation,
                        triangle,
                    )?
                {
                    retained_source_cells += 1;
                }
            }
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
            let mut retained_duplicate_cells = 0usize;
            for output in &assembly.triangles {
                if (output.source_side != triangulation.side
                    || output.source_face != triangulation.face)
                    && output_triangle_matches_triangulated_cell(
                        output,
                        assembly,
                        triangulation,
                        triangle,
                    )?
                {
                    retained_duplicate_cells += 1;
                }
            }
            let expected_orientation = match expected {
                ExactRegionRetention::Keep => Some(ExactOutputTriangleOrientation::PreserveSource),
                ExactRegionRetention::KeepReversed => {
                    Some(ExactOutputTriangleOrientation::ReverseSource)
                }
                ExactRegionRetention::Drop => None,
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
                    ExactEvidenceValidationError::VolumetricMaterializedAssemblyViolatesOperation,
                );
            }
            let retained_source_subcells_cover_cell =
                if let Some(expected_orientation) = expected_orientation {
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
                } else {
                    false
                };
            match expected {
                ExactRegionRetention::Drop
                    if retained_source_cells != 0 || retained_source_subcells != 0 =>
                {
                    return Err(
                        ExactEvidenceValidationError::VolumetricMaterializedAssemblyViolatesOperation,
                    );
                }
                ExactRegionRetention::Keep | ExactRegionRetention::KeepReversed
                    if !retained_source_subcells_cover_cell && retained_duplicate_cells == 0 =>
                {
                    return Err(
                        ExactEvidenceValidationError::VolumetricMaterializedAssemblyViolatesOperation,
                    );
                }
                ExactRegionRetention::Keep
                | ExactRegionRetention::KeepReversed
                | ExactRegionRetention::Drop => {}
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
    let Some(cell_points) = triangulation_cell_triangle_points(triangulation, triangle) else {
        return false;
    };
    let cell_area = projected_polygon_area2_value(
        &[
            cell_points[0].clone(),
            cell_points[1].clone(),
            cell_points[2].clone(),
        ],
        triangulation.projection,
    );
    let Some(cell_area) = (match compare_reals(&cell_area, &Real::from(0)).value() {
        Some(Ordering::Less) => Some(-cell_area),
        Some(Ordering::Equal | Ordering::Greater) => Some(cell_area),
        None => None,
    }) else {
        return false;
    };
    let mut output_area = Real::from(0);
    let mut found = false;
    for output in outputs {
        let Some(points) = output_triangle_points(output, assembly) else {
            return false;
        };
        let area = projected_polygon_area2_value(
            &[points[0].clone(), points[1].clone(), points[2].clone()],
            triangulation.projection,
        );
        let Some(area) = (match compare_reals(&area, &Real::from(0)).value() {
            Some(Ordering::Less) => Some(-area),
            Some(Ordering::Equal | Ordering::Greater) => Some(area),
            None => None,
        }) else {
            return false;
        };
        output_area += &area;
        found = true;
    }
    found && compare_reals(&output_area, &cell_area).value() == Some(Ordering::Equal)
}

fn output_triangle_lies_in_triangulated_cell(
    output: &ExactOutputTriangle,
    assembly: &ExactBooleanAssemblyPlan,
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
) -> bool {
    let Some(cell_points) = triangulation_cell_triangle_points(triangulation, triangle) else {
        return false;
    };
    output.vertices.iter().all(|&vertex| {
        let Some(output_point) = assembly.vertices.get(vertex).map(|vertex| &vertex.point) else {
            return false;
        };
        let location = classify_point_triangle(
            &project_point3(cell_points[0], triangulation.projection),
            &project_point3(cell_points[1], triangulation.projection),
            &project_point3(cell_points[2], triangulation.projection),
            &project_point3(output_point, triangulation.projection),
        )
        .value();
        matches!(
            location,
            Some(TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex)
        )
    })
}

fn validate_selected_region_assembly_covers_selection(
    selection: ExactRegionSelection,
    triangulations: &[FaceRegionTriangulation],
    assembly: &ExactBooleanAssemblyPlan,
) -> Result<(), ExactEvidenceValidationError> {
    for triangulation in triangulations {
        if !matches!(
            (selection, triangulation.side),
            (ExactRegionSelection::KeepAll, _) | (ExactRegionSelection::KeepLeft, MeshSide::Left)
        ) || triangulation.triangles.is_empty()
        {
            continue;
        }

        // Duplicate exact cells may be canonicalized to one retained
        // topological copy after both sides have supplied the predicate
        // evidence proving coincidence. Every selected cell must still be
        // represented either by its own source label or by an exact duplicate
        // retained from the opposite side.
        let mut selected_cells_retained = true;
        for triangle in triangulation.triangles.chunks_exact(3) {
            let triangle = [triangle[0], triangle[1], triangle[2]];
            let mut retained = false;
            for output in &assembly.triangles {
                if output_triangle_matches_triangulated_cell(
                    output,
                    assembly,
                    triangulation,
                    triangle,
                )? {
                    retained = true;
                    break;
                }
            }
            if !retained {
                selected_cells_retained = false;
                break;
            }
        }
        if !selected_cells_retained {
            return Err(ExactEvidenceValidationError::SelectedRegionAssemblyMissingSelectedRegion);
        }
    }

    Ok(())
}

fn output_triangle_matches_triangulated_cell(
    output: &ExactOutputTriangle,
    assembly: &ExactBooleanAssemblyPlan,
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
) -> Result<bool, ExactEvidenceValidationError> {
    let Some(output_points) = output_triangle_points(output, assembly) else {
        return Ok(false);
    };
    let Some(cell_points) = triangulation_cell_triangle_points(triangulation, triangle) else {
        return Ok(false);
    };
    let mut matched = [false; 3];
    for output_point in output_points {
        let mut index = None;
        for (cell_index, cell_point) in cell_points.iter().enumerate() {
            if matched[cell_index] {
                continue;
            }
            match point3_exact_equal(output_point, cell_point) {
                Some(true) => {
                    index = Some(cell_index);
                    break;
                }
                Some(false) => {}
                None => return Err(ExactEvidenceValidationError::InvalidAssembly),
            }
        }
        let Some(index) = index else {
            return Ok(false);
        };
        matched[index] = true;
    }
    Ok(true)
}

fn triangulation_cell_triangle_points(
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
) -> Option<[&Point3; 3]> {
    Some([
        boundary_node_point(triangulation.boundary.get(triangle[0])?),
        boundary_node_point(triangulation.boundary.get(triangle[1])?),
        boundary_node_point(triangulation.boundary.get(triangle[2])?),
    ])
}

fn output_triangle_points<'a>(
    output: &ExactOutputTriangle,
    assembly: &'a ExactBooleanAssemblyPlan,
) -> Option<[&'a Point3; 3]> {
    Some([
        &assembly.vertices.get(output.vertices[0])?.point,
        &assembly.vertices.get(output.vertices[1])?.point,
        &assembly.vertices.get(output.vertices[2])?.point,
    ])
}

fn validate_output_mesh_matches_assembly(
    assembly: &ExactBooleanAssemblyPlan,
    mesh: &Mesh,
) -> Result<(), ExactEvidenceValidationError> {
    if assembly.vertices.len() != mesh.vertices().len()
        || assembly.triangles.len() != mesh.facts().mesh.face_count
        || mesh.facts().faces.len() < assembly.triangles.len()
    {
        return Err(ExactEvidenceValidationError::OutputMeshAssemblyMismatch);
    }
    // The materialized mesh is an edge artifact of the retained assembly, not
    // combinatorial chain as part of the exact object state, so the triangle
    // soup returned to callers must replay exactly from the audited assembly
    // plan for both selected-region and arrangement-materialized outputs.
    for (assembly_vertex, mesh_vertex) in assembly.vertices.iter().zip(mesh.vertices()) {
        match point3_exact_equal(&assembly_vertex.point, mesh_vertex) {
            Some(true) => {}
            Some(false) | None => {
                return Err(ExactEvidenceValidationError::OutputMeshAssemblyMismatch);
            }
        }
    }
    for (assembly_triangle, mesh_triangle) in
        assembly.triangles.iter().zip(retained_face_rows(mesh))
    {
        if assembly_triangle.vertices != mesh_triangle {
            return Err(ExactEvidenceValidationError::OutputMeshAssemblyMismatch);
        }
    }
    Ok(())
}

/// Certified support level for a requested exact boolean operation.
///
/// computing as an application-level contract: unresolved combinatorics must be
/// represented explicitly instead of being decided by approximate arithmetic.
/// These variants therefore distinguish executable certified shortcuts from
/// cases whose split regions are available but still need exact winding mode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactBooleanSupport {
    /// The request is an explicit selected-region assembly mode.
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
    /// The retained graph contains certified boundary contact events. This
    /// includes coplanar touching and the closed-solid case where positive-area
    /// coplanar overlaps plus adjacent contact-only candidates are proven
    /// boundary-only by exact winding evidence. A caller must choose a
    /// boundary/shared-feature mode before this can become named boolean
    /// output.
    RequiresBoundaryOnlyContact,
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
    /// refine, reject, or use a mode that explicitly accepts uncertainty.
    UnresolvedGraph,
}

impl ExactBooleanSupport {
    fn certified_operation_matches(self, operation: ExactBooleanOperation) -> bool {
        matches!(
            (self, operation),
            (
                Self::CertifiedClosedBoundaryTouchingUnion | Self::CertifiedConvexUnion,
                ExactBooleanOperation::Union,
            ) | (
                Self::CertifiedClosedBoundaryTouchingIntersection
                    | Self::CertifiedConvexIntersection,
                ExactBooleanOperation::Intersection,
            ) | (
                Self::CertifiedClosedBoundaryTouchingDifference | Self::CertifiedConvexDifference,
                ExactBooleanOperation::Difference,
            ) | (
                Self::CertifiedEmptyOperand
                    | Self::CertifiedBoundsDisjoint
                    | Self::CertifiedIdentical
                    | Self::CertifiedSameSurface
                    | Self::CertifiedOpenSurfaceDisjoint
                    | Self::CertifiedClosedWindingSeparated
                    | Self::CertifiedClosedWindingContainment
                    | Self::CertifiedMixedDimensionalRegularizedSolid
                    | Self::CertifiedLowerDimensionalRegularizedSolid
                    | Self::CertifiedConvexContainment
                    | Self::CertifiedConvexSeparated,
                ExactBooleanOperation::Union
                    | ExactBooleanOperation::Intersection
                    | ExactBooleanOperation::Difference,
            )
        )
    }

    fn open_surface_arrangement_operation(self) -> Option<ExactBooleanOperation> {
        match self {
            Self::CertifiedOpenSurfaceArrangementUnion => Some(ExactBooleanOperation::Union),
            Self::CertifiedOpenSurfaceArrangementIntersection => {
                Some(ExactBooleanOperation::Intersection)
            }
            Self::CertifiedOpenSurfaceArrangementDifference => {
                Some(ExactBooleanOperation::Difference)
            }
            _ => None,
        }
    }

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
        )
    }
}

/// Retained evidence for an exact boolean operation request.
///
/// The report gives internal callers a stable way to audit the current
/// implementation boundary. Shortcut variants are retained as materializable
/// exact results. For nontrivial named booleans, the report retains certified
/// split-region plane classifications without dispatching to the specialized
/// tolerance kernel.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactBooleanOperationEvidence {
    /// Requested operation.
    pub(super) operation: ExactBooleanOperation,
    /// Certified support level for the request.
    pub(super) support: ExactBooleanSupport,
    /// Whether retained graph events contain explicit unknowns.
    pub(super) graph_had_unknowns: bool,
    /// Retained face-pair records after exact broad/narrow scheduling.
    pub(super) retained_face_pairs: usize,
    /// Total retained event records across all retained face pairs.
    pub(super) retained_events: usize,
    /// Number of split-region boundaries produced for classification.
    pub(super) region_count: usize,
    /// Certified classifications of split regions against opposite face
    /// planes.
    pub(super) region_classifications: Vec<FaceRegionPlaneClassification>,
    /// Structured explanation for named operations that are certified enough
    /// to inspect but not yet executable by the selected mode.
    pub(super) blocker: Option<ExactBooleanBlocker>,
    /// Checked coplanar-overlap evidence retained when operation evidence stops
    /// at a planar arrangement boundary.
    ///
    /// This keeps positive-area coplanar graph evidence visible to structured
    /// replay instead of flattening it into a generic "unsupported" boolean.
    pub(super) coplanar_arrangement_evidence: Option<CoplanarArrangementEvidence>,
    /// Source-aware coplanar volumetric-cell evidence retained when the
    /// operation evidence crosses that exact boundary.
    ///
    /// This report separates boundary-only opposite-side shared faces from
    /// same-side or undecided positive-area coplanar overlap. Retaining it
    /// exact object evidence that authorized a blocker, a no-volume boundary
    /// shortcut, or an arrangement-materialized consumption of coplanar
    /// source-face cells.
    pub(super) coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
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
    pub(super) operation: ExactBooleanOperation,
    /// Certified closure status.
    pub(super) status: ExactVolumetricBoundaryClosureStatus,
    /// Number of output triangles in the retained boundary materialization.
    pub(super) output_triangles: usize,
    /// Number of boundary edges retained by the materialized output mesh.
    pub(super) boundary_edges: usize,
    /// Number of directed boundary loops, when loop extraction succeeded.
    pub(super) boundary_loops: usize,
    /// Number of boundary vertices whose outgoing directed boundary-edge count
    /// is not exactly one.
    pub(super) boundary_vertices_with_invalid_outgoing_degree: usize,
    /// Number of boundary vertices whose incoming directed boundary-edge count
    /// is not exactly one.
    pub(super) boundary_vertices_with_invalid_incoming_degree: usize,
    /// Number of undirected mesh edges used more than twice by output
    /// triangles, proving non-manifold topology before boundary-loop walking.
    pub(super) overused_boundary_edges: usize,
    /// Number of boundary loops proven not exactly coplanar.
    pub(super) noncoplanar_boundary_loops: usize,
    /// Number of repeated exact point pairs found inside directed boundary loops.
    pub(super) repeated_exact_boundary_points: usize,
    /// Number of exact point classes that appear at multiple topological
    /// vertices inside directed boundary loops.
    pub(super) self_contact_exact_points: usize,
    /// Number of topological boundary vertices participating in exact
    /// self-contact point classes.
    pub(super) self_contact_topological_vertices: usize,
    /// Number of split cycles around exact self-contact points with fewer than
    /// three distinct exact points.
    pub(super) self_contact_degenerate_cycles: usize,
    /// Number of split cycles around exact self-contact points with at least
    /// three distinct exact points.
    pub(super) self_contact_nondegenerate_cycles: usize,
    /// Number of coplanar loop groups produced by exact loop grouping.
    pub(super) coplanar_loop_groups: usize,
}

impl ExactVolumetricBoundaryClosureReport {
    /// Validate this report against the source meshes that produced it.
    pub(crate) fn validate_against_sources(
        &self,
        left: &Mesh,
        right: &Mesh,
    ) -> Result<(), ExactEvidenceValidationError> {
        self.validate()?;
        let replay = if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
            ExactVolumetricBoundaryClosureReport::no_materialized(self.operation)
        } else {
            let graph = validated_report_intersection_graph(left, right)?;
            volumetric_boundary_closure_report_from_graph(&graph, left, right, self.operation)
                .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
        };
        if self == &replay {
            Ok(())
        } else {
            Err(ExactEvidenceValidationError::SourceReplayMismatch)
        }
    }

    /// Validate status and retained closure counts.
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        let impossible_boundary_count_bounds = match self.output_triangles.checked_mul(3) {
            Some(max_triangle_edges) => {
                self.boundary_edges > max_triangle_edges
                    || self.boundary_loops != 0 && self.boundary_loops > self.boundary_edges / 3
                    || self.boundary_vertices_with_invalid_outgoing_degree > self.boundary_edges
                    || self.boundary_vertices_with_invalid_incoming_degree > self.boundary_edges
                    || self.noncoplanar_boundary_loops > self.boundary_loops
                    || self.coplanar_loop_groups > self.boundary_loops
                    || self.overused_boundary_edges > max_triangle_edges
                    || self.self_contact_topological_vertices > self.boundary_edges
                    || self.self_contact_exact_points > self.self_contact_topological_vertices / 2
                    || match self
                        .self_contact_topological_vertices
                        .checked_mul(self.self_contact_topological_vertices.saturating_sub(1))
                    {
                        Some(max_repeated_ordered_pairs) => {
                            self.repeated_exact_boundary_points > max_repeated_ordered_pairs / 2
                        }
                        None => true,
                    }
            }
            None => true,
        };
        let boundary_topology_failure_evidence =
            self.boundary_vertices_with_invalid_outgoing_degree != 0
                || self.boundary_vertices_with_invalid_incoming_degree != 0
                || self.overused_boundary_edges != 0;
        let valid_self_contact_evidence = match (
            2_usize.checked_mul(self.self_contact_exact_points),
            self.self_contact_degenerate_cycles
                .checked_add(self.self_contact_nondegenerate_cycles),
        ) {
            (Some(min_topological_vertices), Some(cycle_count)) => {
                self.repeated_exact_boundary_points != 0
                    && self.self_contact_exact_points != 0
                    && self.self_contact_topological_vertices >= min_topological_vertices
                    && self.repeated_exact_boundary_points
                        >= self.self_contact_topological_vertices - self.self_contact_exact_points
                    && cycle_count == self.self_contact_topological_vertices
            }
            _ => false,
        };

        if impossible_boundary_count_bounds {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        match &self.status {
            ExactVolumetricBoundaryClosureStatus::NoMaterializedBoundaryOutput => {
                if self.output_triangles != 0
                    || self.boundary_edges != 0
                    || self.boundary_loops != 0
                    || boundary_topology_failure_evidence
                    || self.noncoplanar_boundary_loops != 0
                    || self.repeated_exact_boundary_points != 0
                    || self.self_contact_exact_points != 0
                    || self.self_contact_topological_vertices != 0
                    || self.self_contact_degenerate_cycles != 0
                    || self.self_contact_nondegenerate_cycles != 0
                    || self.coplanar_loop_groups != 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactVolumetricBoundaryClosureStatus::AlreadyClosed => {
                if self.boundary_edges != 0
                    || self.boundary_loops != 0
                    || boundary_topology_failure_evidence
                    || self.noncoplanar_boundary_loops != 0
                    || self.repeated_exact_boundary_points != 0
                    || self.self_contact_exact_points != 0
                    || self.self_contact_topological_vertices != 0
                    || self.self_contact_degenerate_cycles != 0
                    || self.self_contact_nondegenerate_cycles != 0
                    || self.coplanar_loop_groups != 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable => {
                if self.output_triangles == 0
                    || self.boundary_edges == 0
                    || self.boundary_loops == 0
                    || boundary_topology_failure_evidence
                    || self.noncoplanar_boundary_loops != 0
                    || self.repeated_exact_boundary_points != 0
                    || self.self_contact_exact_points != 0
                    || self.self_contact_topological_vertices != 0
                    || self.self_contact_degenerate_cycles != 0
                    || self.self_contact_nondegenerate_cycles != 0
                    || self.coplanar_loop_groups == 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactVolumetricBoundaryClosureStatus::NonCoplanarBoundaryClosureRequired => {
                if self.output_triangles == 0
                    || self.boundary_edges == 0
                    || self.boundary_loops == 0
                    || boundary_topology_failure_evidence
                    || self.noncoplanar_boundary_loops == 0
                    || self.repeated_exact_boundary_points != 0
                    || self.self_contact_exact_points != 0
                    || self.self_contact_topological_vertices != 0
                    || self.self_contact_degenerate_cycles != 0
                    || self.self_contact_nondegenerate_cycles != 0
                    || self.coplanar_loop_groups != 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactVolumetricBoundaryClosureStatus::BoundaryLoopExactSelfContact => {
                if self.output_triangles == 0
                    || self.boundary_edges == 0
                    || self.boundary_loops == 0
                    || boundary_topology_failure_evidence
                    || self.noncoplanar_boundary_loops != 0
                    || !valid_self_contact_evidence
                    || self.coplanar_loop_groups != 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactVolumetricBoundaryClosureStatus::BoundaryTopologyNotLoop => {
                if self.output_triangles == 0
                    || self.boundary_edges == 0
                    || self.boundary_loops != 0
                    || !boundary_topology_failure_evidence
                    || self.noncoplanar_boundary_loops != 0
                    || self.repeated_exact_boundary_points != 0
                    || self.self_contact_exact_points != 0
                    || self.self_contact_topological_vertices != 0
                    || self.self_contact_degenerate_cycles != 0
                    || self.self_contact_nondegenerate_cycles != 0
                    || self.coplanar_loop_groups != 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactVolumetricBoundaryClosureStatus::BoundaryClosureBlocked(blocker) => {
                if self.output_triangles == 0
                    || self.boundary_edges == 0
                    || self.boundary_loops == 0
                    || boundary_topology_failure_evidence
                    || !((self.repeated_exact_boundary_points == 0
                        && self.self_contact_exact_points == 0
                        && self.self_contact_topological_vertices == 0
                        && self.self_contact_degenerate_cycles == 0
                        && self.self_contact_nondegenerate_cycles == 0)
                        || valid_self_contact_evidence)
                    || !matches!(
                        blocker,
                        ExactArrangementBlocker::UndecidableOrdering
                            | ExactArrangementBlocker::NonManifoldCellComplex
                    )
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
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
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
fn validate_winding_evidence_against_sources_for_request(
    report: &ExactWindingEvidenceReport,
    left: &Mesh,
    right: &Mesh,
    request: ExactBooleanRequest,
) -> Result<(), ExactEvidenceValidationError> {
    let mut source_replay = ExactBooleanSourceReplay::new(left, right);
    {
        let graph = source_replay.validated_graph()?;
        let retained_coplanar_volumetric_evidence_matches =
            if let Some(evidence) = report.coplanar_volumetric_evidence.as_ref() {
                let replay = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right)
                    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
                evidence == &replay
            } else {
                false
            };
        if report.operation == request.operation
            && report.status
                == ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized
            && !graph.has_unknowns()
            && !matches!(report.operation, ExactBooleanOperation::SelectedRegions(_))
            && report.retained_face_pairs == graph.face_pairs.len()
            && report.retained_events == graph.event_count()
            && report.region_count == 0
            && report.region_classifications.is_empty()
            && report.coplanar_arrangement_evidence.is_none()
            && retained_coplanar_volumetric_evidence_matches
            && volumetric_boundary_closure_report_from_graph(graph, left, right, report.operation)
                .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
                .status
                == ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
        {
            return Ok(());
        }
    }
    if axis_aligned_orthogonal_solid_winding_evidence_matches_sources(
        report,
        request,
        &mut source_replay,
    )? {
        return Ok(());
    }
    let graph = source_replay.validated_graph()?;
    let shortcut_facts =
        ExactArrangementCellComplexShortcutFacts::checked_from_sources(left, right)?;
    if let Ok(replay) = winding_evidence_report_for_request_from_graph_and_attempt(
        graph,
        left,
        right,
        request,
        None,
        &shortcut_facts,
    ) && report == &replay
    {
        return Ok(());
    }

    if let Ok(evaluation) =
        exact_boolean_evaluation_for_replay_result_with_materialization(left, right, request, true)
        && report == &evaluation.certifications.winding_evidence
    {
        return Ok(());
    }

    // Some retained witnesses, such as selected-region blockers and older
    // lower-dimensional shortcut reports, are still exact even when the
    // canonical evaluation cannot yet return them or supersedes them with an
    // arrangement/cell-complex materialization status.
    Err(ExactEvidenceValidationError::SourceReplayMismatch)
}

#[cfg(test)]
fn axis_aligned_orthogonal_solid_winding_evidence_matches_sources(
    report: &ExactWindingEvidenceReport,
    request: ExactBooleanRequest,
    source_replay: &mut ExactBooleanSourceReplay<'_>,
) -> Result<bool, ExactEvidenceValidationError> {
    if report.operation != request.operation
        || report.status != ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized
        || matches!(report.operation, ExactBooleanOperation::SelectedRegions(_))
        || report.region_count != 0
        || !report.region_classifications.is_empty()
        || report.coplanar_arrangement_evidence.is_some()
    {
        return Ok(false);
    }
    let left = source_replay.left;
    let right = source_replay.right;
    let graph = source_replay.validated_graph()?;
    if graph.has_unknowns() {
        return Ok(false);
    }
    let retained_coplanar_volumetric_evidence_matches =
        if let Some(evidence) = report.coplanar_volumetric_evidence.as_ref() {
            let replay = CoplanarVolumetricCellEvidenceReport::from_graph(graph, left, right)
                .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
            evidence == &replay
        } else {
            true
        };
    let retains_graph_evidence = report.retained_face_pairs == graph.face_pairs.len()
        && report.retained_events == graph.event_count()
        && retained_coplanar_volumetric_evidence_matches;
    let collapsed_winding_evidence = report.retained_face_pairs == 0
        && report.retained_events == 0
        && report.coplanar_volumetric_evidence.is_none()
        && report.blocker == ExactBooleanBlocker::default();
    if retains_graph_evidence || collapsed_winding_evidence {
        axis_aligned_orthogonal_solid_evidence_matches_sources(left, right, request)
    } else {
        Ok(false)
    }
}

#[cfg(test)]
fn axis_aligned_orthogonal_solid_evidence_matches_sources(
    left: &Mesh,
    right: &Mesh,
    request: ExactBooleanRequest,
) -> Result<bool, ExactEvidenceValidationError> {
    let Some(solid_operation) = request.operation.axis_aligned_orthogonal_solid_operation() else {
        return Ok(false);
    };
    materialize_axis_aligned_orthogonal_solid_cell_output(
        left,
        right,
        solid_operation,
        "exact arrangement orthogonal solid cell operation evidence replay",
        request.validation,
    )
    .map(|mesh| mesh.is_some())
    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)
}

impl ExactBooleanOperationEvidence {
    /// Validate this operation evidence report against source meshes and request.
    ///
    /// Boundary-only named booleans are intentionally blocked until a caller
    /// chooses how to project lower-dimensional contact. Request-native replay
    /// preserves that complete choice instead of splitting validation and
    /// boundary-contact handling away from the operation they certify.
    #[cfg(test)]
    pub(crate) fn validate_against_sources_for_request(
        &self,
        left: &Mesh,
        right: &Mesh,
        request: ExactBooleanRequest,
    ) -> Result<(), ExactEvidenceValidationError> {
        self.validate()?;
        let mut source_replay = ExactBooleanSourceReplay::new(left, right);
        let graph = source_replay.validated_graph()?;
        let retained_coplanar_volumetric_evidence_matches =
            if let Some(evidence) = self.coplanar_volumetric_evidence.as_ref() {
                let replay = CoplanarVolumetricCellEvidenceReport::from_graph(&graph, left, right)
                    .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
                evidence == &replay
            } else {
                false
            };
        if self.operation == request.operation
            && self.support == ExactBooleanSupport::CertifiedArrangementCellComplex
            && self.blocker.is_none()
            && self.retained_face_pairs == graph.face_pairs.len()
            && self.retained_events == graph.event_count()
            && self.region_count == 0
            && self.region_classifications.is_empty()
            && self.coplanar_arrangement_evidence.is_none()
            && retained_coplanar_volumetric_evidence_matches
            && volumetric_boundary_closure_report_from_graph(&graph, left, right, request.operation)
                .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
                .status
                == ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
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
            && axis_aligned_orthogonal_solid_evidence_matches_sources(left, right, request)?
        {
            return Ok(());
        }
        let shortcut_facts =
            ExactArrangementCellComplexShortcutFacts::checked_from_sources(left, right)?;
        if let Ok(replay) = operation_evidence_for_exact_request_from_graph_with_retained_attempt(
            &graph,
            left,
            right,
            request,
            None,
            &shortcut_facts,
        ) && self == &replay
        {
            return Ok(());
        }
        let replay = exact_boolean_evaluation_for_replay_result_with_materialization(
            left, right, request, true,
        )
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?
        .operation_evidence
        .clone();
        if self == &replay {
            Ok(())
        } else {
            Err(ExactEvidenceValidationError::SourceReplayMismatch)
        }
    }

    /// Validate support, blocker, and retained artifact consistency.
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        // OperationEvidence connects exact graph construction to later selection and
        // keeps contradictions visible as structured state rather than hiding
        // them behind a boolean success/failure bit.
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)?;
        let certified_support_matches_operation =
            self.support.certified_operation_matches(self.operation);
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
            return Err(ExactEvidenceValidationError::UnexpectedCoplanarVolumetricEvidence);
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
                    return Err(ExactEvidenceValidationError::CertifiedReportHasBlocker);
                }
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || !certified_support_matches_operation
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                if let Some(evidence) = self.coplanar_volumetric_evidence.as_ref() {
                    if !matches!(
                        self.support,
                        ExactBooleanSupport::CertifiedIdentical
                            | ExactBooleanSupport::CertifiedSameSurface
                    ) {
                        return Err(
                            ExactEvidenceValidationError::UnexpectedCoplanarVolumetricEvidence,
                        );
                    }
                    validate_coplanar_volumetric_evidence(
                        evidence,
                        self.retained_face_pairs,
                        self.retained_events,
                        CoplanarVolumetricEvidenceRequirement::CountsOnly,
                    )?;
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::CertifiedConvexUnion
            | ExactBooleanSupport::CertifiedConvexIntersection
            | ExactBooleanSupport::CertifiedConvexDifference => {
                if self.blocker.is_some() {
                    return Err(ExactEvidenceValidationError::CertifiedReportHasBlocker);
                }
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || !certified_support_matches_operation
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
            | ExactBooleanSupport::CertifiedConvexContainment
            | ExactBooleanSupport::CertifiedConvexSeparated => {
                if self.blocker.is_some() {
                    return Err(ExactEvidenceValidationError::CertifiedReportHasBlocker);
                }
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || !certified_support_matches_operation
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                if let Some(evidence) = self.coplanar_volumetric_evidence.as_ref() {
                    if !matches!(
                        self.support,
                        ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
                            | ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
                    ) {
                        return Err(
                            ExactEvidenceValidationError::UnexpectedCoplanarVolumetricEvidence,
                        );
                    }
                    validate_coplanar_volumetric_evidence(
                        evidence,
                        self.retained_face_pairs,
                        self.retained_events,
                        CoplanarVolumetricEvidenceRequirement::BoundaryOnlyContact,
                    )?;
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::CertifiedArrangementCellComplex => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.blocker.is_some()
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                if let Some(evidence) = self.coplanar_volumetric_evidence.as_ref() {
                    validate_coplanar_volumetric_evidence(
                        evidence,
                        self.retained_face_pairs,
                        self.retained_events,
                        CoplanarVolumetricEvidenceRequirement::MaterializedArrangement,
                    )?;
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
            | ExactBooleanSupport::CertifiedOpenSurfaceArrangementIntersection
            | ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference => {
                let expected_operation = self
                    .support
                    .open_surface_arrangement_operation()
                    .expect("matched open-surface arrangement support");
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.operation != expected_operation
                    || self.graph_had_unknowns
                    || self.blocker.is_some()
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresBoundaryOnlyContact => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                validate_blocker_evidence(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::BoundaryOnlyContact,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresPlanarArrangement => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                let blocker = validate_blocker_evidence(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::PlanarArrangement,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let evidence = self
                    .coplanar_arrangement_evidence
                    .as_ref()
                    .ok_or(ExactEvidenceValidationError::MissingCoplanarArrangementEvidence)?;
                evidence.validate().map_err(|_| {
                    ExactEvidenceValidationError::InvalidCoplanarArrangementEvidence
                })?;
                validate_coplanar_arrangement_evidence_matches_blocker(evidence, blocker)?;
                if !evidence.needs_planar_cells() || blocker.coplanar_touching_pairs != 0 {
                    return Err(ExactEvidenceValidationError::CoplanarArrangementEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresCoplanarVolumetricCells => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                let blocker = validate_blocker_evidence(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::CoplanarVolumetricCells,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                let evidence = self
                    .coplanar_volumetric_evidence
                    .as_ref()
                    .ok_or(ExactEvidenceValidationError::MissingCoplanarVolumetricEvidence)?;
                validate_coplanar_volumetric_evidence_matches_blocker(
                    evidence,
                    blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                    CoplanarVolumetricEvidenceRequirement::NeedsCoplanarCells,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::RequiresCertifiedWinding => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                let blocker = self
                    .blocker
                    .as_ref()
                    .ok_or(ExactEvidenceValidationError::MissingBlocker)?;
                let expected = match blocker.kind {
                    ExactBooleanBlockerKind::CoplanarVolumetricCells => {
                        ExactBooleanBlockerKind::CoplanarVolumetricCells
                    }
                    _ => ExactBooleanBlockerKind::Winding,
                };
                let blocker = validate_blocker_evidence(
                    Some(blocker),
                    expected,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                match (expected, self.coplanar_volumetric_evidence.as_ref()) {
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_matches_blocker(
                            evidence,
                            blocker,
                            self.retained_face_pairs,
                            self.retained_events,
                            CoplanarVolumetricEvidenceRequirement::NeedsCoplanarCells,
                        )?;
                    }
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, None) => {
                        return Err(
                            ExactEvidenceValidationError::MissingCoplanarVolumetricEvidence,
                        );
                    }
                    (_, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence(
                            evidence,
                            self.retained_face_pairs,
                            self.retained_events,
                            CoplanarVolumetricEvidenceRequirement::NeedsCoplanarCells,
                        )?;
                    }
                    (_, None) => {}
                }
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::UnresolvedGraph => {
                let blocker_has_refinement_evidence = if let Some(blocker) = self.blocker.as_ref() {
                    blocker.unknown_pairs != 0 || blocker.construction_failed_events != 0
                } else {
                    false
                };
                if !self.graph_had_unknowns && !blocker_has_refinement_evidence {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                validate_blocker_evidence(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::Refinement,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactBooleanSupport::SelectedRegionPolicy => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                if !matches!(self.operation, ExactBooleanOperation::SelectedRegions(_))
                    || self.graph_had_unknowns
                    || self.blocker.is_some()
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
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

/// Missing exact mode or refinement that blocks named boolean output.
///
/// unresolved application-layer topology as first-class state: a caller should
/// be able to distinguish "needs exact winding" from "needs a boundary output
/// mode" or "needs predicate refinement" without interpreting prose
/// diagnostics.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ExactBooleanBlocker {
    /// Missing mode or refinement class.
    pub(super) kind: ExactBooleanBlockerKind,
    /// Number of retained non-coplanar candidate face pairs.
    pub(super) candidate_pairs: usize,
    /// Number of retained coplanar positive-overlap face pairs.
    pub(super) coplanar_overlapping_pairs: usize,
    /// Number of retained coplanar touching face pairs.
    pub(super) coplanar_touching_pairs: usize,
    /// Number of retained unknown face pairs.
    pub(super) unknown_pairs: usize,
    /// Number of retained segment/plane events whose endpoint predicates
    /// certified a crossing but whose exact construction failed.
    pub(super) construction_failed_events: usize,
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
    /// This is the shared provenance-count boundary for operation evidence blockers and
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
    /// requirements, coplanar-only graphs route to planar cells or boundary-only contact,
    /// mixed coplanar/non-coplanar graphs need volumetric coplanar handling, and
    /// remaining resolved non-coplanar graph state needs winding.
    pub(crate) fn inferred_kind(&self) -> ExactBooleanBlockerKind {
        if self.unknown_pairs != 0 || self.construction_failed_events != 0 {
            ExactBooleanBlockerKind::Refinement
        } else if self.coplanar_overlapping_pairs != 0 || self.coplanar_touching_pairs != 0 {
            if self.candidate_pairs == 0 && self.coplanar_overlapping_pairs > 0 {
                ExactBooleanBlockerKind::PlanarArrangement
            } else if self.candidate_pairs == 0 {
                ExactBooleanBlockerKind::BoundaryOnlyContact
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
    ) -> Result<(), ExactEvidenceValidationError> {
        if self.kind != expected {
            return Err(ExactEvidenceValidationError::WrongBlockerKind);
        }
        let valid = match expected {
            ExactBooleanBlockerKind::Refinement => {
                self.unknown_pairs > 0 || self.construction_failed_events > 0
            }
            ExactBooleanBlockerKind::BoundaryOnlyContact => {
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
            Err(ExactEvidenceValidationError::InvalidBlockerCounts)
        }
    }
}

/// Exact boolean operation evidence blocker kind.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactBooleanBlockerKind {
    /// Predicate or equality refinement is required before mode can run.
    Refinement,
    /// A lower-dimensional shared-boundary output mode is required.
    BoundaryOnlyContact,
    /// A planar arrangement output model is required for coplanar surfaces.
    PlanarArrangement,
    /// Coplanar source-face cells must be materialized before closed
    /// volumetric winding can decide named output.
    CoplanarVolumetricCells,
    /// Full winding/inside-outside classification is required.
    Winding,
}

/// Certification status for exact refinement operation_evidence.
///
/// Refinement is the stage before application-level topology mode: exact
/// graph extraction retained an unknown predicate outcome or a construction
/// whose endpoint predicates certified an event but whose exact point/parameter
/// from winding or planar-arrangement mode, so it has a separate report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg(test)]
pub(crate) enum ExactRefinementStatus {
    /// The graph contains no retained unknowns or construction failures.
    NotRequired,
    /// The graph contains retained evidence that must be refined before mode.
    Required,
}

/// Auditable report for unresolved exact graph refinement.
///
/// This report is intentionally narrower than boolean operation_evidence. It answers
/// only whether exact graph construction is blocked by unknown predicates or
/// failed exact constructions, retaining the graph counts that justify the
/// answer. Later boundary, planar-arrangement, or winding reports should only
/// run as mode once this report is not required.
#[derive(Clone, Debug, Eq, PartialEq)]
#[cfg(test)]
pub(crate) struct ExactRefinementReport {
    /// Named operation whose graph was inspected.
    pub(super) operation: ExactBooleanOperation,
    /// Coarse refinement status.
    pub(super) status: ExactRefinementStatus,
    /// Whether graph extraction retained unknown predicate outcomes.
    pub(super) graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub(super) retained_face_pairs: usize,
    /// Total retained event records.
    pub(super) retained_events: usize,
    /// Refinement blocker counts, present only when refinement is required.
    pub(super) blocker: Option<ExactBooleanBlocker>,
}

#[cfg(test)]
impl ExactRefinementReport {
    /// Validate status, retained counts, and refinement blocker consistency.
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)
            .map_err(|_| ExactEvidenceValidationError::InvalidBlockerCounts)?;
        match self.status {
            ExactRefinementStatus::Required => {
                let blocker = validate_blocker_evidence(
                    self.blocker.as_ref(),
                    ExactBooleanBlockerKind::Refinement,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if self.graph_had_unknowns != (blocker.unknown_pairs > 0) {
                    return Err(ExactEvidenceValidationError::InvalidBlockerCounts);
                }
            }
            ExactRefinementStatus::NotRequired => {
                if self.blocker.is_some() {
                    return Err(ExactEvidenceValidationError::UnexpectedGraphEvents);
                }
                if self.graph_had_unknowns {
                    return Err(ExactEvidenceValidationError::InvalidBlockerCounts);
                }
            }
        }
        Ok(())
    }
}

/// Replayable exact identity certificate for the identical-mesh shortcut.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactIdenticalMeshReport {
    /// Coarse identity status.
    pub(super) status: ExactIdenticalMeshStatus,
    /// Number of left source vertices compared in original order.
    left_vertices: usize,
    /// Number of right source vertices compared in original order.
    right_vertices: usize,
    /// Number of left source triangles compared in original order.
    left_triangles: usize,
    /// Number of right source triangles compared in original order.
    right_triangles: usize,
    /// Exact coordinate comparison predicates used for original-order vertex
    /// identity.
    predicates: Vec<PredicateUse>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactIdenticalMeshStatus {
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
    #[cfg(test)]
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        if self.predicates.len() > self.left_vertices.saturating_mul(3) {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        match self.status {
            ExactIdenticalMeshStatus::VertexCountMismatch => {
                if self.left_vertices == self.right_vertices || !self.predicates.is_empty() {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactIdenticalMeshStatus::VertexCoordinateUndecided
            | ExactIdenticalMeshStatus::VertexCoordinateMismatch => {
                if self.left_vertices != self.right_vertices || self.predicates.is_empty() {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactIdenticalMeshStatus::TriangleSequenceMismatch => {
                if self.left_vertices != self.right_vertices
                    || self.predicates.len() != self.left_vertices.saturating_mul(3)
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactIdenticalMeshStatus::Certified => {
                if self.left_vertices != self.right_vertices
                    || self.left_triangles != self.right_triangles
                    || self.predicates.len() != self.left_vertices.saturating_mul(3)
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
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
    pub(super) status: ExactSameSurfaceStatus,
    /// Mapping from each left vertex index to the matched right vertex index.
    left_to_right: Vec<usize>,
    /// Mapping from each right vertex index to the matched left vertex index.
    right_to_left: Vec<usize>,
    /// Sorted left triangle vertex sets.
    left_triangles: Vec<[usize; 3]>,
    /// Sorted right triangle vertex sets remapped into left vertex indices.
    right_triangles: Vec<[usize; 3]>,
    /// Predicate certificates used by exact coordinate equality checks.
    predicates: Vec<PredicateUse>,
}

impl ExactSameSurfaceReport {
    /// Validate same-surface report invariants.
    ///
    /// Rejection statuses are still evidence states: count mismatches must not
    /// retain coordinate predicates, vertex-matching failures may keep only the
    /// partial left-to-right matches and predicate trail, and triangle-set
    /// mismatches must retain a valid full vertex permutation.
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        let predicates_all_proof_producing = self
            .predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing);
        match self.status {
            ExactSameSurfaceStatus::VertexCountMismatch
            | ExactSameSurfaceStatus::TriangleCountMismatch => {
                if !self.left_to_right.is_empty()
                    || !self.right_to_left.is_empty()
                    || !self.left_triangles.is_empty()
                    || !self.right_triangles.is_empty()
                    || !self.predicates.is_empty()
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
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
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                if matches!(
                    self.status,
                    ExactSameSurfaceStatus::VertexCoordinateMismatch
                ) && !predicates_all_proof_producing
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactSameSurfaceStatus::TriangleSetMismatch => {
                validate_full_permutation(&self.left_to_right, &self.right_to_left)?;
                if self.left_triangles.is_empty()
                    || self.right_triangles.is_empty()
                    || self.left_triangles == self.right_triangles
                {
                    return Err(ExactEvidenceValidationError::MismatchedTriangleSets);
                }
            }
            ExactSameSurfaceStatus::Certified => {
                validate_full_permutation(&self.left_to_right, &self.right_to_left)?;
                if self.left_triangles != self.right_triangles {
                    return Err(ExactEvidenceValidationError::MismatchedTriangleSets);
                }
                if !predicates_all_proof_producing {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
        }
        Ok(())
    }
}

/// Retained source-equality facts for trivial open-surface shortcuts.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactSurfaceEqualityReports {
    /// Original-row identical mesh evidence.
    pub(super) identical: ExactIdenticalMeshReport,
    /// Reindexed same-surface evidence, computed only when identity did not
    /// already certify the source pair.
    pub(super) same_surface: Option<ExactSameSurfaceReport>,
}

impl ExactSurfaceEqualityReports {
    pub(crate) fn from_sources(left: &Mesh, right: &Mesh) -> Self {
        let identical = identical_mesh_report_from_sources(left, right);
        let same_surface = if matches!(identical.status, ExactIdenticalMeshStatus::Certified) {
            None
        } else {
            Some(same_surface_report_from_sources(left, right))
        };
        Self {
            identical,
            same_surface,
        }
    }

    pub(crate) const fn identical_certified(&self) -> bool {
        matches!(self.identical.status, ExactIdenticalMeshStatus::Certified)
    }

    pub(crate) fn same_surface_certified(&self) -> bool {
        self.same_surface
            .as_ref()
            .is_some_and(|report| matches!(report.status, ExactSameSurfaceStatus::Certified))
    }

    pub(crate) fn any_certified(&self) -> bool {
        self.identical_certified() || self.same_surface_certified()
    }
}

/// Certify whether two meshes represent the same triangle surface.
///
/// The report preserves the exact coordinate-equality predicate certificates
/// used to find a vertex bijection and the sorted triangle sets compared after
/// remapping. This is the auditable form of the same-surface shortcut.
pub(crate) fn same_surface_report_from_sources(
    left: &Mesh,
    right: &Mesh,
) -> ExactSameSurfaceReport {
    if left.vertices().len() != right.vertices().len() {
        return ExactSameSurfaceReport {
            status: ExactSameSurfaceStatus::VertexCountMismatch,
            left_to_right: Vec::new(),
            right_to_left: Vec::new(),
            left_triangles: Vec::new(),
            right_triangles: Vec::new(),
            predicates: Vec::new(),
        };
    }
    if left.facts().mesh.face_count != right.facts().mesh.face_count {
        return ExactSameSurfaceReport {
            status: ExactSameSurfaceStatus::TriangleCountMismatch,
            left_to_right: Vec::new(),
            right_to_left: Vec::new(),
            left_triangles: Vec::new(),
            right_triangles: Vec::new(),
            predicates: Vec::new(),
        };
    }

    let (left_to_right, predicates, status) = certified_vertex_permutation_report(left, right);
    if status != ExactSameSurfaceStatus::Certified {
        return ExactSameSurfaceReport {
            status,
            left_to_right,
            right_to_left: Vec::new(),
            left_triangles: Vec::new(),
            right_triangles: Vec::new(),
            predicates,
        };
    }
    let mut right_to_left = vec![0; left_to_right.len()];
    for (left_index, &right_index) in left_to_right.iter().enumerate() {
        right_to_left[right_index] = left_index;
    }

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
pub(crate) fn identical_mesh_report_from_sources(
    left: &Mesh,
    right: &Mesh,
) -> ExactIdenticalMeshReport {
    let left_vertices = left.vertices().len();
    let right_vertices = right.vertices().len();
    let left_triangles = left.facts().mesh.face_count;
    let right_triangles = right.facts().mesh.face_count;
    let mut predicates = Vec::new();
    if left_vertices != right_vertices {
        return ExactIdenticalMeshReport {
            status: ExactIdenticalMeshStatus::VertexCountMismatch,
            left_vertices,
            right_vertices,
            left_triangles,
            right_triangles,
            predicates,
        };
    }

    for (left_vertex, right_vertex) in left.vertices().iter().zip(right.vertices()) {
        let x = compare_reals_report(&left_vertex.x, &right_vertex.x);
        let y = compare_reals_report(&left_vertex.y, &right_vertex.y);
        let z = compare_reals_report(&left_vertex.z, &right_vertex.z);
        predicates.push(PredicateUse::from_certificate(x.certificate));
        predicates.push(PredicateUse::from_certificate(y.certificate));
        predicates.push(PredicateUse::from_certificate(z.certificate));
        let Some(x_value) = x.outcome.value() else {
            return ExactIdenticalMeshReport {
                status: ExactIdenticalMeshStatus::VertexCoordinateUndecided,
                left_vertices,
                right_vertices,
                left_triangles,
                right_triangles,
                predicates,
            };
        };
        let Some(y_value) = y.outcome.value() else {
            return ExactIdenticalMeshReport {
                status: ExactIdenticalMeshStatus::VertexCoordinateUndecided,
                left_vertices,
                right_vertices,
                left_triangles,
                right_triangles,
                predicates,
            };
        };
        let Some(z_value) = z.outcome.value() else {
            return ExactIdenticalMeshReport {
                status: ExactIdenticalMeshStatus::VertexCoordinateUndecided,
                left_vertices,
                right_vertices,
                left_triangles,
                right_triangles,
                predicates,
            };
        };
        if x_value != Ordering::Equal || y_value != Ordering::Equal || z_value != Ordering::Equal {
            return ExactIdenticalMeshReport {
                status: ExactIdenticalMeshStatus::VertexCoordinateMismatch,
                left_vertices,
                right_vertices,
                left_triangles,
                right_triangles,
                predicates,
            };
        }
    }

    let status = if retained_face_rows_equal(left, right) {
        ExactIdenticalMeshStatus::Certified
    } else {
        ExactIdenticalMeshStatus::TriangleSequenceMismatch
    };
    ExactIdenticalMeshReport {
        status,
        left_vertices,
        right_vertices,
        left_triangles,
        right_triangles,
        predicates,
    }
}

fn certified_vertex_permutation_report(
    left: &Mesh,
    right: &Mesh,
) -> (Vec<usize>, Vec<PredicateUse>, ExactSameSurfaceStatus) {
    let mut left_to_right = Vec::with_capacity(left.vertices().len());
    let mut used_right = vec![false; right.vertices().len()];
    let mut predicates = Vec::new();

    for left_vertex in left.vertices() {
        let mut match_index = None;
        let mut saw_undecided = false;
        for (right_index, right_vertex) in right.vertices().iter().enumerate() {
            if used_right[right_index] {
                continue;
            }
            let x = compare_reals_report(&left_vertex.x, &right_vertex.x);
            let y = compare_reals_report(&left_vertex.y, &right_vertex.y);
            let z = compare_reals_report(&left_vertex.z, &right_vertex.z);
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

fn sorted_triangle_sets(mesh: &Mesh, right_to_left: Option<&[usize]>) -> Vec<[usize; 3]> {
    retained_face_rows(mesh)
        .map(|triangle| {
            let mut vertices = triangle.map(|vertex| match right_to_left {
                Some(mapping) => mapping[vertex],
                None => vertex,
            });
            vertices.sort_unstable();
            vertices
        })
        .collect()
}

fn retained_face_rows(mesh: &Mesh) -> impl Iterator<Item = [usize; 3]> + '_ {
    mesh.view().faces().map(|face| face.vertex_indices())
}

fn retained_face_rows_equal(left: &Mesh, right: &Mesh) -> bool {
    left.facts().mesh.face_count == right.facts().mesh.face_count
        && left.facts().faces.len() >= left.facts().mesh.face_count
        && right.facts().faces.len() >= right.facts().mesh.face_count
        && retained_face_rows(left).eq(retained_face_rows(right))
}

fn validate_full_permutation(
    left_to_right: &[usize],
    right_to_left: &[usize],
) -> Result<(), ExactEvidenceValidationError> {
    if left_to_right.len() != right_to_left.len() {
        return Err(ExactEvidenceValidationError::InvalidPermutation);
    }
    for (left, &right) in left_to_right.iter().enumerate() {
        if right >= right_to_left.len() || right_to_left[right] != left {
            return Err(ExactEvidenceValidationError::InvalidPermutation);
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
    pub(super) status: ExactOpenSurfaceDisjointStatus,
    /// Whether the left mesh satisfies the exact open-surface precondition.
    pub(super) left_open_surface: bool,
    /// Whether the right mesh satisfies the exact open-surface precondition.
    pub(super) right_open_surface: bool,
    /// Whether graph extraction retained unknown events.
    pub(super) graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub(super) retained_face_pairs: usize,
    /// Total retained event records.
    pub(super) retained_events: usize,
    /// Relation counts for retained face pairs.
    pub(super) blocker: ExactBooleanBlocker,
}

impl ExactOpenSurfaceDisjointReport {
    /// Validate this open-surface report against the source meshes.
    ///
    /// Open-surface disjointness is certified graph absence plus mesh-shape
    /// preconditions. This method recomputes both from `left` and `right`
    pub(crate) fn validate_against_sources(
        &self,
        left: &Mesh,
        right: &Mesh,
    ) -> Result<(), ExactEvidenceValidationError> {
        self.validate()?;
        let graph = validated_report_intersection_graph(left, right)?;
        let replay = open_surface_disjoint_report_from_graph(&graph, left, right);
        if self == &replay {
            Ok(())
        } else {
            Err(ExactEvidenceValidationError::SourceReplayMismatch)
        }
    }

    /// Validate status, graph counts, and blocker consistency.
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)?;
        if matches!(self.status, ExactOpenSurfaceDisjointStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactEvidenceValidationError::GraphUnknownStatusMismatch);
        }
        let expected_kind = match self.status {
            ExactOpenSurfaceDisjointStatus::GraphUnknowns => ExactBooleanBlockerKind::Refinement,
            ExactOpenSurfaceDisjointStatus::NotOpenSurface
            | ExactOpenSurfaceDisjointStatus::GraphHasFacePairs
            | ExactOpenSurfaceDisjointStatus::Certified => self.blocker.inferred_kind(),
        };
        validate_blocker_evidence(
            Some(&self.blocker),
            expected_kind,
            self.retained_face_pairs,
            self.retained_events,
        )?;
        validate_refinement_partition(
            matches!(self.status, ExactOpenSurfaceDisjointStatus::GraphUnknowns),
            &self.blocker,
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
                return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
            }
        } else if !self.left_open_surface || !self.right_open_surface {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        if self.status == ExactOpenSurfaceDisjointStatus::Certified
            && (self.retained_face_pairs != 0 || self.retained_events != 0)
        {
            return Err(ExactEvidenceValidationError::UnexpectedGraphEvents);
        }
        if self.status == ExactOpenSurfaceDisjointStatus::GraphHasFacePairs
            && self.retained_face_pairs == 0
        {
            return Err(ExactEvidenceValidationError::MissingRelationCount);
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
/// Boundary-only contacts require a caller-selected output mode because a
/// triangle mesh cannot encode the lower-dimensional intersection itself.
/// This report retains the exact graph counts that justify that mode gap,
/// computation sense.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct ExactBoundaryTouchingReport {
    /// Coarse boundary-touching certification status.
    pub(super) status: ExactBoundaryTouchingStatus,
    /// Whether graph extraction retained unknown events.
    pub(super) graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub(super) retained_face_pairs: usize,
    /// Total retained event records.
    pub(super) retained_events: usize,
    /// Relation counts for retained face pairs.
    pub(super) blocker: ExactBooleanBlocker,
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
    pub(super) operation: ExactBooleanOperation,
    /// Coarse certification status.
    pub(super) status: ExactAdjacentUnionCompletionStatus,
    /// Whether the left source mesh was a closed manifold.
    pub(super) left_closed: bool,
    /// Whether the right source mesh was a closed manifold.
    pub(super) right_closed: bool,
    /// Whether the stronger axis-aligned box path owns this pair.
    pub(super) axis_aligned_box_pair: bool,
    /// Whether another exact kernel should materialize this union first.
    pub(super) stronger_kernel_available: bool,
    /// Whether graph extraction retained unknown events.
    pub(super) graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub(super) retained_face_pairs: usize,
    /// Total retained event records.
    pub(super) retained_events: usize,
    /// Relation counts for retained face pairs.
    pub(super) blocker: ExactBooleanBlocker,
    /// Count of exact whole-face pairs consumed by full-face completion.
    pub(super) full_face_shared_faces: usize,
    /// Count of exact source-owned full patches consumed by full-face
    /// completion.
    pub(super) full_face_shared_patches: usize,
    /// Source side whose faces contain the opposite caps for contained-face
    /// completion.
    pub(super) contained_containing_side: Option<MeshSide>,
    /// Count of opposite-source faces removed by contained-face completion.
    pub(super) contained_faces: usize,
    /// Count of source faces replaced by holed remnants in contained-face
    /// completion.
    pub(super) containing_faces: usize,
}

impl ExactAdjacentUnionCompletionReport {
    /// Validate status, graph counts, and consumed topology counts.
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)?;
        if matches!(
            self.status,
            ExactAdjacentUnionCompletionStatus::GraphUnresolved
        ) && !self.graph_had_unknowns
            && self.blocker.construction_failed_events == 0
        {
            return Err(ExactEvidenceValidationError::GraphUnknownStatusMismatch);
        }
        if !matches!(
            self.status,
            ExactAdjacentUnionCompletionStatus::GraphUnresolved
        ) && (self.graph_had_unknowns || self.blocker.construction_failed_events != 0)
        {
            return Err(ExactEvidenceValidationError::GraphUnknownStatusMismatch);
        }
        let expected_kind = match self.status {
            ExactAdjacentUnionCompletionStatus::GraphUnresolved => {
                ExactBooleanBlockerKind::Refinement
            }
            ExactAdjacentUnionCompletionStatus::CertifiedFullFace
            | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace => {
                ExactBooleanBlockerKind::BoundaryOnlyContact
            }
            _ => self.blocker.inferred_kind(),
        };
        if self.blocker.kind != expected_kind {
            return Err(ExactEvidenceValidationError::WrongBlockerKind);
        }
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
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        };
        let Some(contained_counts) = self.contained_faces.checked_add(self.containing_faces) else {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        };
        if (self.retained_face_pairs != 0 && full_face_counts > self.retained_face_pairs)
            || self.contained_faces > self.retained_face_pairs
            || self.containing_faces > self.retained_face_pairs
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
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
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        match self.status {
            ExactAdjacentUnionCompletionStatus::NotUnion => {
                if matches!(self.operation, ExactBooleanOperation::Union) {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactAdjacentUnionCompletionStatus::NotClosedSolid => {
                if self.operation != ExactBooleanOperation::Union
                    || (self.left_closed && self.right_closed)
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactAdjacentUnionCompletionStatus::AxisAlignedBoxPair => {
                if self.operation != ExactBooleanOperation::Union
                    || !self.left_closed
                    || !self.right_closed
                    || !self.axis_aligned_box_pair
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactAdjacentUnionCompletionStatus::StrongerKernelAvailable => {
                if self.operation != ExactBooleanOperation::Union
                    || !self.left_closed
                    || !self.right_closed
                    || self.axis_aligned_box_pair
                    || !self.stronger_kernel_available
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactAdjacentUnionCompletionStatus::GraphUnresolved => {
                if self.operation != ExactBooleanOperation::Union
                    || !self.left_closed
                    || !self.right_closed
                    || self.axis_aligned_box_pair
                    || self.stronger_kernel_available
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactAdjacentUnionCompletionStatus::NoAdjacencyCertificate => {
                if self.operation != ExactBooleanOperation::Union
                    || !self.left_closed
                    || !self.right_closed
                    || self.axis_aligned_box_pair
                    || self.stronger_kernel_available
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
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
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
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
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                validate_adjacent_certified_boundary_blocker(
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
            }
        }
        if !matches!(
            self.status,
            ExactAdjacentUnionCompletionStatus::CertifiedFullFace
                | ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
        ) && (full_face_counts != 0
            || contained_counts != 0
            || self.contained_containing_side.is_some())
        {
            return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
        }
        Ok(())
    }

    /// Validate this report by replaying adjacency completion from source
    /// operands.
    pub(crate) fn validate_against_sources(
        &self,
        left: &Mesh,
        right: &Mesh,
    ) -> Result<(), ExactEvidenceValidationError> {
        self.validate()?;
        let graph = validated_report_intersection_graph(left, right)?;
        let (replay, _) = adjacent_union_completion_certification_from_graph(
            &graph,
            left,
            right,
            self.operation,
            None,
        )
        .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactEvidenceValidationError::SourceReplayMismatch)
        }
    }
}

impl ExactBoundaryTouchingReport {
    /// Validate status, retained relation counts, and blocker consistency.
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)?;
        if matches!(self.status, ExactBoundaryTouchingStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactEvidenceValidationError::GraphUnknownStatusMismatch);
        }
        let expected_kind = match self.status {
            ExactBoundaryTouchingStatus::GraphUnknowns => ExactBooleanBlockerKind::Refinement,
            ExactBoundaryTouchingStatus::Certified => ExactBooleanBlockerKind::BoundaryOnlyContact,
            ExactBoundaryTouchingStatus::NotBoundaryOnly => {
                let coplanar_pairs = self.blocker.coplanar_overlapping_pairs != 0
                    || self.blocker.coplanar_touching_pairs != 0;
                if self.blocker.unknown_pairs != 0 || self.blocker.construction_failed_events != 0 {
                    ExactBooleanBlockerKind::Refinement
                } else if self.blocker.candidate_pairs == 0 && !coplanar_pairs {
                    ExactBooleanBlockerKind::Winding
                } else if self.blocker.candidate_pairs == 0
                    && self.blocker.coplanar_overlapping_pairs == 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
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
        validate_blocker_evidence(
            Some(&self.blocker),
            expected_kind,
            self.retained_face_pairs,
            self.retained_events,
        )?;
        validate_refinement_partition(
            matches!(self.status, ExactBoundaryTouchingStatus::GraphUnknowns),
            &self.blocker,
        )?;
        if self.status == ExactBoundaryTouchingStatus::Certified
            && self.blocker.candidate_pairs == 0
            && self.blocker.coplanar_touching_pairs == 0
            && self.blocker.coplanar_overlapping_pairs == 0
        {
            return Err(ExactEvidenceValidationError::MissingRelationCount);
        }
        if self.status == ExactBoundaryTouchingStatus::Certified {
            self.blocker
                .validate_for_kind(ExactBooleanBlockerKind::BoundaryOnlyContact)?;
        }
        Ok(())
    }

    /// Validate this boundary-touching report against the source meshes.
    ///
    /// Boundary-only contact is a mode boundary over a resolved exact graph.
    /// Recomputing the report from the source meshes ensures the retained
    pub(crate) fn validate_against_sources(
        &self,
        left: &Mesh,
        right: &Mesh,
    ) -> Result<(), ExactEvidenceValidationError> {
        self.validate()?;
        let graph = validated_report_intersection_graph(left, right)?;
        let replay = boundary_touching_report_from_graph(&graph, left, right)
            .map_err(|_| ExactEvidenceValidationError::SourceReplayMismatch)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactEvidenceValidationError::SourceReplayMismatch)
        }
    }
}

/// Certification status for planar-arrangement blockers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactPlanarArrangementStatus {
    /// Selected-region assembly already carries its own explicit region mode.
    NotNamedOperation,
    /// Exact graph extraction retained unresolved events.
    GraphUnknowns,
    /// The requested named operation is already handled by a narrower certified
    /// coplanar surface output path.
    AlreadyMaterialized,
    /// The exact graph does not consist solely of positive-area coplanar
    /// overlaps requiring planar arrangement output.
    NoPositiveOverlap,
    /// Closed-solid coplanar contact was certified as a boundary-only contact
    /// blocker before planar-cell output should be considered.
    BoundaryOnlyContactRequired,
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
    pub(super) operation: ExactBooleanOperation,
    /// Coarse planar-arrangement certification status.
    pub(super) status: ExactPlanarArrangementStatus,
    /// Whether graph extraction retained unknown events.
    pub(super) graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub(super) retained_face_pairs: usize,
    /// Total retained event records.
    pub(super) retained_events: usize,
    /// Relation counts for retained face pairs.
    pub(super) blocker: ExactBooleanBlocker,
    /// Checked coplanar-overlap evidence summary retained from the graph
    /// layer.
    pub(super) coplanar_arrangement_evidence: Option<CoplanarArrangementEvidence>,
}

impl ExactPlanarArrangementReport {
    /// Validate status, retained relation counts, and blocker consistency.
    #[cfg(test)]
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)?;
        if matches!(self.status, ExactPlanarArrangementStatus::GraphUnknowns)
            != self.graph_had_unknowns
        {
            return Err(ExactEvidenceValidationError::GraphUnknownStatusMismatch);
        }
        // A graph-unknown arrangement report has not reached planar-cell
        // mode. It is still blocked on predicate/construction refinement, a
        let expected_kind = match self.status {
            ExactPlanarArrangementStatus::GraphUnknowns => ExactBooleanBlockerKind::Refinement,
            ExactPlanarArrangementStatus::BoundaryOnlyContactRequired => {
                ExactBooleanBlockerKind::BoundaryOnlyContact
            }
            ExactPlanarArrangementStatus::Required => ExactBooleanBlockerKind::PlanarArrangement,
            ExactPlanarArrangementStatus::NotNamedOperation
            | ExactPlanarArrangementStatus::AlreadyMaterialized
            | ExactPlanarArrangementStatus::NoPositiveOverlap => self.blocker.inferred_kind(),
        };
        validate_blocker_evidence(
            Some(&self.blocker),
            expected_kind,
            self.retained_face_pairs,
            self.retained_events,
        )?;
        validate_refinement_partition(
            matches!(self.status, ExactPlanarArrangementStatus::GraphUnknowns),
            &self.blocker,
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
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactPlanarArrangementStatus::GraphUnknowns => {}
            ExactPlanarArrangementStatus::AlreadyMaterialized
            | ExactPlanarArrangementStatus::NoPositiveOverlap
            | ExactPlanarArrangementStatus::BoundaryOnlyContactRequired => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                if matches!(self.status, ExactPlanarArrangementStatus::NoPositiveOverlap)
                    && self.blocker.candidate_pairs == 0
                    && self.blocker.coplanar_overlapping_pairs != 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
            ExactPlanarArrangementStatus::Required => {
                if matches!(self.operation, ExactBooleanOperation::SelectedRegions(_)) {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
            }
        }
        if matches!(self.status, ExactPlanarArrangementStatus::Required)
            && self.blocker.coplanar_overlapping_pairs == 0
        {
            return Err(ExactEvidenceValidationError::MissingRelationCount);
        }
        match self.status {
            ExactPlanarArrangementStatus::Required => {
                let evidence = self
                    .coplanar_arrangement_evidence
                    .as_ref()
                    .ok_or(ExactEvidenceValidationError::MissingCoplanarArrangementEvidence)?;
                evidence.validate().map_err(|_| {
                    ExactEvidenceValidationError::InvalidCoplanarArrangementEvidence
                })?;
                validate_coplanar_arrangement_evidence_matches_blocker(evidence, &self.blocker)?;
                if !evidence.needs_planar_cells()
                    || self.blocker.coplanar_touching_pairs != 0
                    || evidence.graph_count != self.blocker.coplanar_overlapping_pairs
                {
                    return Err(ExactEvidenceValidationError::CoplanarArrangementEvidenceMismatch);
                }
            }
            ExactPlanarArrangementStatus::AlreadyMaterialized
            | ExactPlanarArrangementStatus::NoPositiveOverlap
            | ExactPlanarArrangementStatus::BoundaryOnlyContactRequired => {
                let evidence = self
                    .coplanar_arrangement_evidence
                    .as_ref()
                    .ok_or(ExactEvidenceValidationError::MissingCoplanarArrangementEvidence)?;
                evidence.validate().map_err(|_| {
                    ExactEvidenceValidationError::InvalidCoplanarArrangementEvidence
                })?;
                validate_coplanar_arrangement_evidence_matches_blocker(evidence, &self.blocker)?;
                if evidence.status == CoplanarArrangementEvidenceStatus::NoCoplanarOverlap
                    && (self.blocker.coplanar_overlapping_pairs != 0
                        || self.blocker.coplanar_touching_pairs != 0)
                {
                    return Err(ExactEvidenceValidationError::CoplanarArrangementEvidenceMismatch);
                }
            }
            ExactPlanarArrangementStatus::NotNamedOperation
            | ExactPlanarArrangementStatus::GraphUnknowns => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
            }
        }
        if matches!(self.status, ExactPlanarArrangementStatus::Required) {
            self.blocker
                .validate_for_kind(ExactBooleanBlockerKind::PlanarArrangement)?;
        } else if matches!(
            self.status,
            ExactPlanarArrangementStatus::BoundaryOnlyContactRequired
        ) {
            self.blocker
                .validate_for_kind(ExactBooleanBlockerKind::BoundaryOnlyContact)?;
        }
        Ok(())
    }
}

/// Certification status for the remaining exact winding evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ExactWindingEvidenceStatus {
    /// Selected-region assembly already carries its own explicit region mode.
    NotNamedOperation,
    /// Exact graph extraction retained unresolved events.
    GraphUnknowns,
    /// Retained graph pairs are boundary-only contacts and need boundary
    /// output mode rather than winding.
    BoundaryOnlyContactRequired,
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

/// Auditable report for the nontrivial overlap winding evidence.
///
/// This report is the certified boundary immediately before full named
/// union/intersection/difference winding semantics. It retains exact graph
/// counts and checked split-region plane classifications, but deliberately
/// topological mode remains explicit state instead of a hidden tolerance
/// decision.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactWindingEvidenceReport {
    /// Requested named operation.
    pub(super) operation: ExactBooleanOperation,
    /// Coarse evidence status.
    pub(super) status: ExactWindingEvidenceStatus,
    /// Whether graph extraction retained unknown events.
    pub(super) graph_had_unknowns: bool,
    /// Retained face-pair records after exact scheduling.
    pub(super) retained_face_pairs: usize,
    /// Total retained event records.
    pub(super) retained_events: usize,
    /// Number of checked split regions prepared for winding.
    pub(super) region_count: usize,
    /// Certified region-vs-opposite-plane classifications.
    pub(super) region_classifications: Vec<FaceRegionPlaneClassification>,
    /// Relation counts for the blocker represented by this report.
    pub(super) blocker: ExactBooleanBlocker,
    /// Checked coplanar-overlap evidence retained when winding is blocked by
    /// planar-cell extraction rather than by volumetric inside/outside mode.
    pub(super) coplanar_arrangement_evidence: Option<CoplanarArrangementEvidence>,
    /// Source-aware coplanar volumetric-cell evidence retained when evidence
    /// is blocked by, or has just consumed, coplanar source-face cells.
    ///
    /// The winding evidence must not reduce this state to raw coplanar pair
    /// counts: exact side evidence is what distinguishes boundary-only contact
    /// from a real volumetric-cell topology obligation.
    pub(super) coplanar_volumetric_evidence: Option<CoplanarVolumetricCellEvidenceReport>,
}

impl ExactWindingEvidenceReport {
    /// Validate this winding-evidence report against the source meshes.
    ///
    /// Winding evidence retains exact split-region and opposite-plane facts.
    /// This replay recomputes the report for the same operation, making stale
    /// region facts and blocker summaries fail before downstream topology
    #[cfg(test)]
    pub(crate) fn validate_against_sources(
        &self,
        left: &Mesh,
        right: &Mesh,
    ) -> Result<(), ExactEvidenceValidationError> {
        self.validate()?;
        let request = ExactBooleanRequest {
            operation: self.operation,
            validation: MeshValidationMode::ALLOW_BOUNDARY,
        };
        validate_winding_evidence_against_sources_for_request(self, left, right, request)
    }

    /// Validate status, blocker, and checked-region artifact consistency.
    pub(crate) fn validate(&self) -> Result<(), ExactEvidenceValidationError> {
        validate_retained_graph_count_shape(self.retained_face_pairs, self.retained_events)?;
        let selected_regions_operation =
            matches!(self.operation, ExactBooleanOperation::SelectedRegions(_));
        if matches!(self.status, ExactWindingEvidenceStatus::GraphUnknowns)
            != self.graph_had_unknowns
            && !matches!(self.status, ExactWindingEvidenceStatus::NotNamedOperation)
        {
            return Err(ExactEvidenceValidationError::GraphUnknownStatusMismatch);
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
            && !matches!(
                self.status,
                ExactWindingEvidenceStatus::PlanarArrangementAlreadyMaterialized
                    | ExactWindingEvidenceStatus::CoplanarVolumetricCellsAlreadyMaterialized
                    | ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized
            )
        {
            return Err(ExactEvidenceValidationError::UnexpectedCoplanarVolumetricEvidence);
        }
        match self.status {
            ExactWindingEvidenceStatus::GraphUnknowns => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                validate_blocker_evidence(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::Refinement,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::BoundaryOnlyContactRequired => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                if selected_regions_operation {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                validate_blocker_evidence(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::BoundaryOnlyContact,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::PlanarArrangementRequired => {
                if selected_regions_operation {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                let blocker = validate_blocker_evidence(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::PlanarArrangement,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let evidence = self
                    .coplanar_arrangement_evidence
                    .as_ref()
                    .ok_or(ExactEvidenceValidationError::MissingCoplanarArrangementEvidence)?;
                evidence.validate().map_err(|_| {
                    ExactEvidenceValidationError::InvalidCoplanarArrangementEvidence
                })?;
                validate_coplanar_arrangement_evidence_matches_blocker(evidence, blocker)?;
                if !evidence.needs_planar_cells() || blocker.coplanar_touching_pairs != 0 {
                    return Err(ExactEvidenceValidationError::CoplanarArrangementEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::PlanarArrangementAlreadyMaterialized => {
                if selected_regions_operation {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                let blocker = validate_blocker_evidence(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::PlanarArrangement,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let evidence = self
                    .coplanar_arrangement_evidence
                    .as_ref()
                    .ok_or(ExactEvidenceValidationError::MissingCoplanarArrangementEvidence)?;
                evidence.validate().map_err(|_| {
                    ExactEvidenceValidationError::InvalidCoplanarArrangementEvidence
                })?;
                validate_coplanar_arrangement_evidence_matches_blocker(evidence, blocker)?;
                if !evidence.needs_planar_cells() {
                    return Err(ExactEvidenceValidationError::CoplanarArrangementEvidenceMismatch);
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::CoplanarVolumetricCellsRequired => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                if selected_regions_operation {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                validate_blocker_evidence(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::CoplanarVolumetricCells,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let evidence = self
                    .coplanar_volumetric_evidence
                    .as_ref()
                    .ok_or(ExactEvidenceValidationError::MissingCoplanarVolumetricEvidence)?;
                validate_coplanar_volumetric_evidence_matches_blocker(
                    evidence,
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                    CoplanarVolumetricEvidenceRequirement::NeedsCoplanarCells,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::CoplanarVolumetricCellsAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                if selected_regions_operation {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                validate_blocker_evidence(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::CoplanarVolumetricCells,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                let evidence = self
                    .coplanar_volumetric_evidence
                    .as_ref()
                    .ok_or(ExactEvidenceValidationError::MissingCoplanarVolumetricEvidence)?;
                validate_coplanar_volumetric_evidence_matches_blocker(
                    evidence,
                    &self.blocker,
                    self.retained_face_pairs,
                    self.retained_events,
                    CoplanarVolumetricEvidenceRequirement::NeedsCoplanarCells,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::VolumetricAssemblyRequired => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                if selected_regions_operation || self.retained_face_pairs == 0 {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                let expected = match self.blocker.kind {
                    ExactBooleanBlockerKind::CoplanarVolumetricCells => {
                        ExactBooleanBlockerKind::CoplanarVolumetricCells
                    }
                    _ => ExactBooleanBlockerKind::Winding,
                };
                validate_blocker_evidence(
                    Some(&self.blocker),
                    expected,
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
                            CoplanarVolumetricEvidenceRequirement::NeedsCoplanarCells,
                        )?;
                    }
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, None) => {
                        return Err(
                            ExactEvidenceValidationError::MissingCoplanarVolumetricEvidence,
                        );
                    }
                    (_, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence(
                            evidence,
                            self.retained_face_pairs,
                            self.retained_events,
                            CoplanarVolumetricEvidenceRequirement::NeedsCoplanarCells,
                        )?;
                    }
                    (_, None) => {}
                }
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::ArrangementCellComplexAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                if selected_regions_operation {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                let expected = match self.blocker.kind {
                    ExactBooleanBlockerKind::BoundaryOnlyContact => {
                        ExactBooleanBlockerKind::BoundaryOnlyContact
                    }
                    ExactBooleanBlockerKind::CoplanarVolumetricCells => {
                        ExactBooleanBlockerKind::CoplanarVolumetricCells
                    }
                    _ => ExactBooleanBlockerKind::Winding,
                };
                validate_blocker_evidence(
                    Some(&self.blocker),
                    expected,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                match (expected, self.coplanar_volumetric_evidence.as_ref()) {
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_matches_blocker(
                            evidence,
                            &self.blocker,
                            self.retained_face_pairs,
                            self.retained_events,
                            CoplanarVolumetricEvidenceRequirement::NeedsCoplanarCells,
                        )?;
                    }
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, None) => {
                        return Err(
                            ExactEvidenceValidationError::MissingCoplanarVolumetricEvidence,
                        );
                    }
                    (ExactBooleanBlockerKind::BoundaryOnlyContact, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence_matches_blocker(
                            evidence,
                            &self.blocker,
                            self.retained_face_pairs,
                            self.retained_events,
                            CoplanarVolumetricEvidenceRequirement::MaterializedArrangement,
                        )?;
                    }
                    (ExactBooleanBlockerKind::BoundaryOnlyContact, None)
                    | (ExactBooleanBlockerKind::Winding, None) => {}
                    (
                        ExactBooleanBlockerKind::Refinement
                        | ExactBooleanBlockerKind::PlanarArrangement,
                        None,
                    ) => {
                        return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                    }
                    (_, Some(_)) => {
                        return Err(
                            ExactEvidenceValidationError::UnexpectedCoplanarVolumetricEvidence,
                        );
                    }
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized
            | ExactWindingEvidenceStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || selected_regions_operation
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                validate_blocker_evidence(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::Winding,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::ConvexBooleanAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || selected_regions_operation
                    || self.graph_had_unknowns
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                validate_blocker_evidence(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::Winding,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::OpenSurfaceArrangementAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || selected_regions_operation
                    || self.graph_had_unknowns
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                validate_blocker_evidence(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::Winding,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::SurfaceEqualityAlreadyMaterialized => {
                let has_coplanar_evidence = self.coplanar_volumetric_evidence.is_some();
                if self.coplanar_arrangement_evidence.is_some()
                    || selected_regions_operation
                    || self.graph_had_unknowns
                    || (!has_coplanar_evidence
                        && (self.retained_face_pairs != 0 || self.retained_events != 0))
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                validate_blocker_evidence(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::Winding,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                if let Some(evidence) = self.coplanar_volumetric_evidence.as_ref() {
                    validate_coplanar_volumetric_evidence(
                        evidence,
                        self.retained_face_pairs,
                        self.retained_events,
                        CoplanarVolumetricEvidenceRequirement::CountsOnly,
                    )?;
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::ClosedBoundaryTouchingAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some()
                    || selected_regions_operation
                    || self.graph_had_unknowns
                    || self.retained_face_pairs == 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                validate_blocker_evidence(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::BoundaryOnlyContact,
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
                            CoplanarVolumetricEvidenceRequirement::BoundaryOnlyContact,
                        )?;
                    }
                    None if self.blocker.coplanar_overlapping_pairs != 0 => {
                        return Err(
                            ExactEvidenceValidationError::MissingCoplanarVolumetricEvidence,
                        );
                    }
                    None => {}
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::EmptyOperandAlreadyMaterialized
            | ExactWindingEvidenceStatus::BoundsDisjointAlreadyMaterialized
            | ExactWindingEvidenceStatus::OpenSurfaceDisjointAlreadyMaterialized
            | ExactWindingEvidenceStatus::ClosedWindingSeparatedAlreadyMaterialized
            | ExactWindingEvidenceStatus::ClosedWindingContainmentAlreadyMaterialized => {
                if self.coplanar_arrangement_evidence.is_some()
                    || self.coplanar_volumetric_evidence.is_some()
                    || selected_regions_operation
                    || self.graph_had_unknowns
                    || self.retained_face_pairs != 0
                    || self.retained_events != 0
                {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                validate_blocker_evidence(
                    Some(&self.blocker),
                    ExactBooleanBlockerKind::Winding,
                    self.retained_face_pairs,
                    self.retained_events,
                )?;
                no_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::Ready => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                if selected_regions_operation || self.retained_face_pairs == 0 {
                    return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                }
                let expected = match self.blocker.kind {
                    ExactBooleanBlockerKind::CoplanarVolumetricCells => {
                        ExactBooleanBlockerKind::CoplanarVolumetricCells
                    }
                    _ => ExactBooleanBlockerKind::Winding,
                };
                validate_blocker_evidence(
                    Some(&self.blocker),
                    expected,
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
                            CoplanarVolumetricEvidenceRequirement::NeedsCoplanarCells,
                        )?;
                    }
                    (ExactBooleanBlockerKind::CoplanarVolumetricCells, None) => {
                        return Err(
                            ExactEvidenceValidationError::MissingCoplanarVolumetricEvidence,
                        );
                    }
                    (_, Some(evidence)) => {
                        validate_coplanar_volumetric_evidence(
                            evidence,
                            self.retained_face_pairs,
                            self.retained_events,
                            CoplanarVolumetricEvidenceRequirement::NeedsCoplanarCells,
                        )?;
                    }
                    (_, None) => {}
                }
                checked_region_facts(self.region_count, &self.region_classifications)
            }
            ExactWindingEvidenceStatus::NotNamedOperation
            | ExactWindingEvidenceStatus::NoNontrivialOverlap => {
                if self.coplanar_arrangement_evidence.is_some() {
                    return Err(
                        ExactEvidenceValidationError::UnexpectedCoplanarArrangementEvidence,
                    );
                }
                match self.status {
                    ExactWindingEvidenceStatus::NotNamedOperation
                        if !selected_regions_operation =>
                    {
                        return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                    }
                    ExactWindingEvidenceStatus::NoNontrivialOverlap
                        if selected_regions_operation || self.retained_face_pairs != 0 =>
                    {
                        return Err(ExactEvidenceValidationError::StatusEvidenceMismatch);
                    }
                    _ => {}
                }
                if matches!(self.status, ExactWindingEvidenceStatus::NotNamedOperation) {
                    let expected = self.blocker.inferred_kind();
                    validate_blocker_evidence(
                        Some(&self.blocker),
                        expected,
                        self.retained_face_pairs,
                        self.retained_events,
                    )?;
                } else {
                    validate_blocker_evidence(
                        Some(&self.blocker),
                        ExactBooleanBlockerKind::Winding,
                        self.retained_face_pairs,
                        self.retained_events,
                    )?;
                }
                no_region_facts(self.region_count, &self.region_classifications)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::Triangle;
    use crate::mesh::boolean::region::{ExactOutputVertex, FaceRegionPlaneRelation};
    use crate::mesh::graph::FaceSplitBoundaryNode;
    use hyperlimit::SourceProvenance;

    #[test]
    fn selected_region_operation_evidence_accepts_empty_region_plan_with_boundary_face_pairs() {
        let mut operation_evidence = ExactBooleanOperationEvidence {
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

        operation_evidence.validate().unwrap();

        operation_evidence.region_count = 1;
        assert_eq!(
            operation_evidence.validate(),
            Err(ExactEvidenceValidationError::MissingRegionFacts)
        );
    }

    fn report_test_triangle(points: &[[i64; 3]; 3]) -> Mesh {
        Mesh::from_i64_triangles_with_validation_mode(
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
            MeshValidationMode::ALLOW_BOUNDARY,
        )
        .unwrap()
    }

    #[test]
    fn selected_region_result_rejects_duplicate_assembly_triangle() {
        let left = report_test_triangle(&[[0, 0, 0], [4, 0, 0], [0, 4, 0]]);
        let right = report_test_triangle(&[[1, -1, -1], [1, 3, 1], [1, 3, -1]]);
        let mut result = materialize_boolean_operation(
            &left,
            &right,
            ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll),
            MeshValidationMode::ALLOW_BOUNDARY,
            None,
            None,
        )
        .unwrap();
        result.validate().unwrap();
        assert!(!result.assembly.triangles.is_empty());

        let duplicate = result.assembly.triangles[0].clone();
        result.assembly.triangles.push(duplicate);

        assert_eq!(
            result.validate(),
            Err(ExactEvidenceValidationError::DuplicateAssemblyTriangle)
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
                    point: p0.clone(),
                    source: boundary[0].clone(),
                },
                ExactOutputVertex {
                    point: p1.clone(),
                    source: boundary[1].clone(),
                },
                ExactOutputVertex {
                    point: p2.clone(),
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
        let mesh = Mesh::new_with_validation_mode_and_version(
            vec![p0.clone(), p1.clone(), p2.clone()],
            vec![Triangle([0, 1, 2])],
            SourceProvenance::exact("exact boolean assembly plan"),
            MeshValidationMode::ALLOW_BOUNDARY,
            1,
        )
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
            Err(ExactEvidenceValidationError::SelectedRegionAssemblyMissingSelectedRegion)
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
        let mesh = Mesh::from_i64_triangles(&[0, 0, 0], &[]).unwrap();
        assert_eq!(mesh.facts().mesh.face_count, 0);
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
            mesh: Mesh::new(
                Vec::new(),
                Vec::new(),
                hyperlimit::SourceProvenance::exact("empty exact arrangement union shortcut"),
            )
            .unwrap(),
        };

        result.validate().unwrap();
    }

    #[test]
    fn arrangement_union_shortcut_request_replay_rejects_empty_output_for_nonempty_sources() {
        let left = report_test_triangle(&[[0, 0, 0], [2, 0, 0], [0, 2, 0]]);
        let right = report_test_triangle(&[[3, 0, 0], [5, 0, 0], [3, 2, 0]]);
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
            mesh: Mesh::new(
                Vec::new(),
                Vec::new(),
                hyperlimit::SourceProvenance::exact("empty exact arrangement union shortcut"),
            )
            .unwrap(),
        };

        result.validate().unwrap();
        assert_eq!(
            result.validate_against_sources(&left, &right),
            Err(ExactEvidenceValidationError::SourceReplayMismatch)
        );
        assert_eq!(
            result.validate_request_against_sources_with_retained_attempt(
                &left,
                &right,
                ExactBooleanRequest {
                    operation: ExactBooleanOperation::Union,
                    validation: MeshValidationMode::ALLOW_BOUNDARY,
                },
                None,
            ),
            Err(ExactEvidenceValidationError::SourceReplayMismatch)
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_self_contact_report_rejects_contradictory_status_evidence() {
        let mut report = valid_self_contact_closure_report();
        report.validate().unwrap();

        report.noncoplanar_boundary_loops = 1;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_self_contact_closure_report();
        report.coplanar_loop_groups = 1;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_self_contact_report_rejects_incoherent_contact_counts() {
        let mut report = valid_self_contact_closure_report();
        report.self_contact_topological_vertices = 1;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_self_contact_closure_report();
        report.repeated_exact_boundary_points = 0;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_self_contact_closure_report();
        report.self_contact_degenerate_cycles = 1;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_report_rejects_impossible_count_bounds() {
        let mut report = valid_noncoplanar_closure_report();
        report.status = ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable;
        report.noncoplanar_boundary_loops = 0;
        report.coplanar_loop_groups = 1;
        report.validate().unwrap();
        assert_eq!(
            report.status,
            ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable
        );

        let mut report = valid_noncoplanar_closure_report();
        report.boundary_loops = 2;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_noncoplanar_closure_report();
        report.noncoplanar_boundary_loops = 2;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_noncoplanar_closure_report();
        report.boundary_edges = 4;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_noncoplanar_closure_report();
        report.output_triangles = usize::MAX;
        report.boundary_edges = usize::MAX;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_topology_not_loop_closure_report();
        report.boundary_vertices_with_invalid_outgoing_degree = 3;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_self_contact_closure_report();
        report.repeated_exact_boundary_points = 2;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_self_contact_closure_report();
        report.repeated_exact_boundary_points = 3;
        report.self_contact_topological_vertices = 4;
        report.self_contact_degenerate_cycles = 4;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_self_contact_closure_report();
        report.output_triangles = usize::MAX;
        report.boundary_edges = usize::MAX;
        report.repeated_exact_boundary_points = usize::MAX;
        report.self_contact_topological_vertices = usize::MAX;
        report.self_contact_degenerate_cycles = usize::MAX;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_noncoplanar_closure_report();
        report.status = ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable;
        report.noncoplanar_boundary_loops = 0;
        report.coplanar_loop_groups = 2;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_blocked_report_rejects_stale_closure_evidence() {
        let mut report = valid_blocked_closure_report();
        report.validate().unwrap();

        report.coplanar_loop_groups = 1;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_blocked_closure_report();
        report.self_contact_exact_points = 1;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_topology_not_loop_closure_report();
        report.boundary_loops = 1;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_topology_not_loop_closure_report();
        report.noncoplanar_boundary_loops = 1;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_noncoplanar_report_rejects_stale_coplanar_grouping() {
        let mut report = valid_noncoplanar_closure_report();
        report.validate().unwrap();

        report.coplanar_loop_groups = 1;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );
    }

    #[test]
    fn volumetric_boundary_loop_statuses_reject_stale_topology_failure_evidence() {
        let mut report = valid_self_contact_closure_report();
        report.boundary_vertices_with_invalid_outgoing_degree = 1;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut report = valid_blocked_closure_report();
        report.overused_boundary_edges = 1;
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let mut operation_evidence = ExactBooleanOperationEvidence {
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
        operation_evidence.validate().unwrap();

        operation_evidence.operation = ExactBooleanOperation::Intersection;
        operation_evidence.validate().unwrap();

        operation_evidence.operation =
            ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll);
        assert_eq!(
            operation_evidence.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
            Err(ExactEvidenceValidationError::InvalidBlockerCounts)
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
            Err(ExactEvidenceValidationError::InvalidBlockerCounts)
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
                kind: ExactBooleanBlockerKind::BoundaryOnlyContact,
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
                kind: ExactBooleanBlockerKind::BoundaryOnlyContact,
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
        );

        let operation_evidence = ExactBooleanOperationEvidence {
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
            operation_evidence.validate(),
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
            Err(ExactEvidenceValidationError::StatusEvidenceMismatch)
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
            Err(ExactEvidenceValidationError::InvalidBlockerCounts)
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
            Err(ExactEvidenceValidationError::InvalidBlockerCounts)
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
        assert!(matches!(
            already_materialized.status,
            ExactPlanarArrangementStatus::AlreadyMaterialized
        ));
        assert!(!matches!(
            already_materialized.status,
            ExactPlanarArrangementStatus::Required
        ));
        already_materialized.coplanar_arrangement_evidence = None;
        assert_eq!(
            already_materialized.validate(),
            Err(ExactEvidenceValidationError::MissingCoplanarArrangementEvidence)
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
        assert!(!matches!(
            no_positive_overlap.status,
            ExactPlanarArrangementStatus::AlreadyMaterialized
        ));
        assert!(!matches!(
            no_positive_overlap.status,
            ExactPlanarArrangementStatus::Required
        ));
        no_positive_overlap.coplanar_arrangement_evidence = None;
        assert_eq!(
            no_positive_overlap.validate(),
            Err(ExactEvidenceValidationError::MissingCoplanarArrangementEvidence)
        );

        let mut boundary_only_contact = ExactPlanarArrangementReport {
            operation: ExactBooleanOperation::Difference,
            status: ExactPlanarArrangementStatus::BoundaryOnlyContactRequired,
            graph_had_unknowns: false,
            retained_face_pairs: 1,
            retained_events: 1,
            blocker: ExactBooleanBlocker {
                kind: ExactBooleanBlockerKind::BoundaryOnlyContact,
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
        boundary_only_contact.validate().unwrap();
        boundary_only_contact.coplanar_arrangement_evidence = None;
        assert_eq!(
            boundary_only_contact.validate(),
            Err(ExactEvidenceValidationError::MissingCoplanarArrangementEvidence)
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
            Err(ExactEvidenceValidationError::MissingCoplanarArrangementEvidence)
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
                kind: ExactBooleanBlockerKind::BoundaryOnlyContact,
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
            Err(ExactEvidenceValidationError::MissingCoplanarVolumetricEvidence)
        );

        let relabeled_evidence = CoplanarVolumetricCellEvidenceReport {
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 1,
            positive_area_coplanar_overlapping_pairs: 0,
            opposite_side_coplanar_overlapping_pairs: 0,
            obstacle: CoplanarVolumetricCellObstacle::BoundaryOnlyContact,
            ..evidence.clone()
        };
        relabeled_evidence.validate().unwrap();
        report.coplanar_volumetric_evidence = Some(relabeled_evidence);
        assert_eq!(
            report.validate(),
            Err(ExactEvidenceValidationError::CoplanarVolumetricEvidenceMismatch)
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
        let mut operation_evidence = ExactBooleanOperationEvidence {
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
        operation_evidence.validate().unwrap();

        operation_evidence.retained_events = 5;
        assert_eq!(
            operation_evidence.validate(),
            Err(ExactEvidenceValidationError::CoplanarVolumetricEvidenceMismatch)
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
            Err(ExactEvidenceValidationError::CoplanarVolumetricEvidenceMismatch)
        );

        let overflowing_evidence = CoplanarVolumetricCellEvidenceReport {
            segment_plane_events: usize::MAX,
            proper_crossing_events: usize::MAX,
            ..operation_evidence
                .coplanar_volumetric_evidence
                .as_ref()
                .expect("operation evidence retained coplanar volumetric evidence")
                .clone()
        };
        overflowing_evidence.validate().unwrap();
        evidence.retained_events = usize::MAX;
        evidence.coplanar_volumetric_evidence = Some(overflowing_evidence);
        assert_eq!(
            evidence.validate(),
            Err(ExactEvidenceValidationError::CoplanarVolumetricEvidenceMismatch)
        );
    }

    #[test]
    fn blocker_source_counts_include_unknown_segment_plane_events() {
        let graph =
            ExactIntersectionGraph::from_face_pairs(vec![crate::mesh::graph::FacePairEvents {
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
